#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MotorScope - J-Link RTT 电机实时可视化调试助手（FOC 版，分支 inmop_cur_loop_rtt）
=================================================================================

固件端（ws/foc.c）在 FOC ISR 内以 1kHz 向 RTT 通道 6 发送 MOTF 帧
（>2kHz 时自动切换为 48B 二进制帧），本程序用 pylink 读取、解析后通过
HTTP 推给浏览器，浏览器 Canvas 实时绘制：
  - 转子（极对数 10 的 20 块磁钢）按"转子实测电角度"转动
  - 控制角（I-F 合成角）指针 vs 转子角（编码器实测）双指针
  - id / iq 分解箭头 + is 电流合成矢量 + vd/vq 电压矢量
  - 同步状态、转速、电流/角度/偏差波形

用法
----
  仿真模式（无需硬件，演示 I-F 启动 -> 同步 -> 运行全过程）：
    python motor_scope.py --mode sim-foc

  J-Link 实机（需 pip install pylink-square + Segger J-Link 软件）：
    python motor_scope.py --mode jlink --device HC32F460 --speed-khz 4000

帧格式（文本）：
  MOTF,<mode>,<phase>,<rotor_mrad>,<theta_mrad>,<iq_ma>,<id_ma>,
       <vq_mv>,<vd_mv>,<spd_rpm>,<sync>,<diff_mrad>,<freq_cHz>,<ms>
"""
from __future__ import annotations

import argparse
import json
import math
import random
import re
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

WEB_DIR = Path(__file__).resolve().parent / "web"
HISTORY_FILE = Path(__file__).resolve().parent / "history.txt"
HISTORY_FILE_MAX_BYTES = 2 * 1024 * 1024      # history.txt 上限 2MB，达到即清空

PHASE_NAMES = {0: "idle", 1: "hold", 2: "ramp/sync", 3: "run", 4: "align"}
MODE_NAMES = {0: "停止", 1: "开环", 2: "电流环", 3: "对齐"}


# ======================================================================
# FOC 数据帧
# ======================================================================

@dataclass
class FocFrame:
    mode: int = 0          # FOC_MODE_*: 0 idle 1 openloop 2 curloop 3 align
    phase: int = 0         # 0 idle 1 hold 2 ramp/sync 3 run 4 align
    rotor_mrad: float = 0.0    # g_foc_if_rotor_rad * 1000（编码器实测电角度）
    theta_mrad: float = 0.0    # g_foc_theta_rad  * 1000（控制角 / I-F 合成角）
    iq_ma: float = 0.0
    id_ma: float = 0.0
    vq_mv: float = 0.0
    vd_mv: float = 0.0
    spd_rpm: float = 0.0
    sync: int = 0
    diff_mrad: float = 0.0     # 控制角 vs 转子角偏差
    freq_cHz: float = 0.0      # I-F 电频率
    ms: int = 0
    rotor_mech_mrad: float = 0.0  # 连续机械角（跨圈不折返，动画用）

    def to_list(self):
        return [self.mode, self.phase, int(round(self.rotor_mrad)),
                int(round(self.theta_mrad)), int(round(self.iq_ma)),
                int(round(self.id_ma)), int(round(self.vq_mv)),
                int(round(self.vd_mv)), int(round(self.spd_rpm)),
                self.sync, int(round(self.diff_mrad)),
                int(round(self.freq_cHz)), self.ms,
                int(round(self.rotor_mech_mrad))]


def _clean_rtt_line(line: str) -> str:
    """剥掉 ANSI 颜色码与 [MAIN] 之类的前缀（兼容 MAIN_E 打印）。"""
    line = re.sub(r"\x1b\[[0-9;]*m", "", line)
    line = re.sub(r"^\[[A-Za-z0-9_]+\]\s*", "", line)
    return line.strip()


def parse_text_line(line: str):
    """解析文本帧：MOTF,<13 个字段>（兼容 MAIN_E 的 [MAIN] 前缀/ANSI 颜色）。"""
    line = _clean_rtt_line(line)
    if not line.startswith("MOTF,"):
        return None
    p = line.split(",")
    if len(p) < 13:
        return None
    try:
        return FocFrame(
            mode=int(p[1]), phase=int(p[2]),
            rotor_mrad=float(p[3]), theta_mrad=float(p[4]),
            iq_ma=float(p[5]), id_ma=float(p[6]),
            vq_mv=float(p[7]), vd_mv=float(p[8]),
            spd_rpm=float(p[9]), sync=int(p[10]),
            diff_mrad=float(p[11]), freq_cHz=float(p[12]),
            ms=int(p[13]) if len(p) > 13 else 0,
            rotor_mech_mrad=float(p[14]) if len(p) > 14 else 0.0,
        )
    except (ValueError, IndexError):
        return None


_BIN_FMT = "<4sIiiiiiiiiiiBBBB"      # 52 字节小端二进制帧（含连续机械角）
_BIN_SIZE = struct.calcsize(_BIN_FMT)


def parse_binary(buf: bytes):
    """解析 48 字节二进制帧（MOTF magic）。"""
    if len(buf) < _BIN_SIZE or buf[:4] != b"MOTF":
        return None
    (magic, ms, rotor, theta, iq, id_, vq, vd, spd, diff, freq, mech,
     mode, phase, sync, rsv) = struct.unpack(_BIN_FMT, buf[:_BIN_SIZE])
    return FocFrame(mode=mode, phase=phase, rotor_mrad=float(rotor),
                    theta_mrad=float(theta), iq_ma=float(iq), id_ma=float(id_),
                    vq_mv=float(vq), vd_mv=float(vd), spd_rpm=float(spd),
                    sync=sync, diff_mrad=float(diff), freq_cHz=float(freq),
                    ms=ms, rotor_mech_mrad=float(mech))


class RttParser:
    """RTT 字节流 -> FocFrame。ch0 为文本（日志 + MOTF 行），>2kHz 时另有二进制帧。"""

    def __init__(self, log_motf: bool = True):
        self.buf = bytearray()
        self.log_motf = log_motf
        self._last_motf_log = 0.0
        self._prev_state = None

    def feed(self, data: bytes, sink, log_sink=None):
        self.buf.extend(data)
        # 文本模式优先：按行切分（ch0 混有 MAIN 日志与 MOTF 行）
        while True:
            idx = self.buf.find(b"\n")
            if idx < 0:
                break
            line = bytes(self.buf[:idx]).decode("utf-8", errors="replace")
            del self.buf[:idx + 1]
            line = line.strip()
            if not line:
                continue
            fr = parse_text_line(line)
            if fr is not None:
                sink(fr)
                if log_sink is not None and self.log_motf:
                    state = (fr.mode, fr.phase, fr.sync)
                    now = time.time()
                    if state != self._prev_state or (now - self._last_motf_log) >= 0.1:
                        self._last_motf_log = now
                        log_sink(line)   # 节流记录 MOTF 帧（~10 条/秒 + 状态变化）
                    self._prev_state = state
            elif log_sink is not None:
                log_sink(line)
        # 二进制模式（>2kHz 帧率）：按 magic 扫描
        if self.buf[:4] == b"MOTF" and len(self.buf) >= _BIN_SIZE:
            while True:
                idx = self.buf.find(b"MOTF")
                if idx < 0 or len(self.buf) - idx < _BIN_SIZE:
                    break
                fr = parse_binary(bytes(self.buf[idx:idx + _BIN_SIZE]))
                if fr is not None:
                    sink(fr)
                del self.buf[:idx + _BIN_SIZE]
        # 防内存膨胀
        if len(self.buf) > 8192:
            del self.buf[:-4096]


# ======================================================================
# 数据中心
# ======================================================================

class DataHub:
    def __init__(self, max_history=100, max_log=2000):
        self.lock = threading.Lock()
        self.seq = 0
        self.latest = None
        self.history = deque(maxlen=max_history)
        self.log_lines = deque(maxlen=max_log)
        self.log_seq = 0
        self.start_time = time.time()
        self._hf = None
        self._hist_bytes = 0
        self.frames_total = 0
        self.status = "starting"
        self.detail = ""

    def push(self, frame: FocFrame, raw: str = ""):
        with self.lock:
            self.seq += 1
            self.history.append((self.seq, time.time(), frame))
            self.latest = frame
            self.frames_total += 1
            self.status = "running"
            self._history_append(frame)
        if raw:
            self.log(raw)

    def _history_append(self, frame: FocFrame):
        """消费过的帧追加写入 history.txt；达到 2MB 上限即清空再写。
        用二进制追加模式，精确按字节计数（避免文本模式 \n->\r\n 的计数偏差）。"""
        try:
            ts = (time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                  + f".{int(time.time() * 1000) % 1000:03d}")
            data = (ts + " MOTF," + ",".join(str(x) for x in frame.to_list())
                    + "\r\n").encode("utf-8")
            if self._hf is None:
                self._hf = open(HISTORY_FILE, "ab")
                try:
                    self._hf.seek(0, 2)
                    self._hist_bytes = self._hf.tell()   # 同步已有文件大小
                except Exception:
                    self._hist_bytes = 0
            self._hist_bytes += len(data)
            if self._hist_bytes > HISTORY_FILE_MAX_BYTES:
                self._hf.truncate(0)                     # 达到上限：清空
                self._hist_bytes = len(data)             # 本行随后写入
            self._hf.write(data)
            self._hf.flush()
        except Exception:
            pass

    def clear_history(self):
        """清空 history.txt（页面退出时调用）。"""
        with self.lock:
            if self._hf is not None:
                try:
                    self._hf.close()
                except Exception:
                    pass
                self._hf = None
            self._hist_bytes = 0
        try:
            HISTORY_FILE.write_text("", encoding="utf-8")
        except Exception:
            pass

    def log(self, text: str):
        with self.lock:
            self.log_seq += 1
            self.log_lines.append((self.log_seq, text))

    def last_frame_age_ms(self):
        with self.lock:
            if not self.history:
                return -1.0
            return (time.time() - self.history[-1][1]) * 1000.0

    def set_status(self, status: str, detail: str = ""):
        with self.lock:
            self.status = status
            self.detail = detail

    def snapshot(self, last_seq: int = 0, log_since: int = 0):
        with self.lock:
            now = time.time()
            fps = self.frames_total / max(now - self.start_time, 1e-6)
            history = [
                [s, round(ts, 3), fr.to_list()]
                for s, ts, fr in self.history if s > last_seq
            ]
            latest = self.latest.to_list() if self.latest else None
            logs = [[s, txt] for s, txt in self.log_lines if s > log_since]
            last_age = -1.0
            if self.history:
                last_age = (now - self.history[-1][1]) * 1000.0
            return {
                "ok": True,
                "status": self.status,
                "detail": self.detail,
                "seq": self.seq,
                "fps": round(fps, 1),
                "uptime": round(now - self.start_time, 1),
                "latest": latest,
                "history": history,
                "logs": logs,
                "log_seq": self.log_seq,
                "last_age_ms": round(last_age, 1),
            }


# ======================================================================
# 仿真数据源：I-F 启动 -> 同步 -> 运行（mode 22 全流程）
# ======================================================================

def _wrap_rad(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class SimFoc:
    """生成与固件 MOTF 文本帧一致的仿真数据。极对数 10、母线 12V。"""

    def __init__(self, sample_hz: int = 200, stop_at: float = 0.0):
        self.sample_hz = sample_hz
        self.pp = 10
        self.t0 = time.time()
        self._last_theta = 0.0
        self.stop_at = stop_at      # >0 时：超过该秒数停止产生帧（模拟数据中断）

    def describe(self):
        return f"仿真FOC (I-F启动->同步->运行, 极对数{self.pp}, {self.sample_hz}Hz)"

    def next_frame(self) -> FocFrame:
        t = time.time() - self.t0
        self._mech_cum = getattr(self, "_mech_cum", 0.0)
        dt = 1.0 / self.sample_hz
        f = FocFrame(mode=2, sync=0)

        if t < 0.5:
            # idle
            f.phase = 0
        elif t < 2.5:
            # phase 1 hold: theta=0, iq ramp to 1200, rotor 锁在 0
            f.phase = 1
            f.theta_mrad = 0.0
            f.rotor_mrad = 30.0 * math.sin(2 * math.pi * 3 * t)   # 轻微抖动
            f.iq_ma = min(1200.0, 1200.0 * (t - 0.5) / 1.5)
            f.id_ma = 20.0 * math.sin(2 * math.pi * 2 * t)
            f.freq_cHz = 0.0
        elif t < 8.0:
            # phase 2 ramp/sync: 频率爬升, 转子逐渐跟上
            f.phase = 2
            freq = min(20.0, 20.0 * (t - 2.5) / 4.0)   # 0 -> 20Hz
            f.freq_cHz = freq * 100.0
            th = self._last_theta + 2 * math.pi * freq * dt
            th %= 2 * math.pi
            self._last_theta = th
            # 转子滞后于合成角：滞后量从 0 起呈铃形（先增大后收敛），
            # 避免 hold->ramp 切换时转子角突变（否则星星会跳一下）
            ramp_t = t - 2.5
            lag = 1.2 * math.sin(min(1.0, ramp_t / 5.5) * math.pi)
            lag = max(0.05, lag)
            rotor = th - lag
            rotor %= 2 * math.pi
            f.theta_mrad = th * 1000.0
            f.rotor_mrad = rotor * 1000.0
            f.diff_mrad = _wrap_rad(rotor - th) * 1000.0
            f.iq_ma = 1200.0 + 800.0 * (t - 2.5) / 5.5
            f.id_ma = 30.0 * math.sin(2 * math.pi * 4 * t)
            if t > 7.2:
                f.sync = 1
        else:
            # phase 3 run: 角度 = 转子实测角, 同步锁定
            f.phase = 3
            f.sync = 1
            freq = 20.0
            f.freq_cHz = freq * 100.0
            th = self._last_theta + 2 * math.pi * freq * dt
            th %= 2 * math.pi
            self._last_theta = th
            f.theta_mrad = th * 1000.0
            f.rotor_mrad = (th + 0.25) * 1000.0        # 负载角 ~0.25 rad
            f.diff_mrad = 250.0
            f.iq_ma = 2000.0 + 150.0 * math.sin(2 * math.pi * 0.8 * t)
            f.id_ma = 25.0 * math.sin(2 * math.pi * 20 * t)
        f.vq_mv = 500.0 + f.iq_ma * 0.15
        f.vd_mv = 40.0 + f.id_ma * 0.1
        f.spd_rpm = (f.freq_cHz / 100.0) * 60.0 / self.pp
        f.ms = int(t * 1000)
        self._last_theta = getattr(self, "_last_theta", 0.0)
        if f.phase == 2 or f.phase == 3:
            f.rotor_mrad = f.rotor_mrad % 6283.0
            f.theta_mrad = f.theta_mrad % 6283.0
        # 连续机械角（度）：由折返电角度解卷 / 极对数，跨圈累计（模拟 Z 不复位）
        elec_deg = f.rotor_mrad / 1000.0 * (180.0 / math.pi)
        d = elec_deg - (self._mech_cum * self.pp)
        d = ((d % 360.0) + 540.0) % 360.0 - 180.0
        self._mech_cum += d / self.pp
        f.rotor_mech_mrad = int(self._mech_cum * (math.pi / 180.0) * 1000.0)
        return f


def sim_loop(hub: DataHub, src: SimFoc, sample_hz: int):
    hub.set_status("running", src.describe())
    interval = 1.0 / sample_hz
    next_t = time.perf_counter()
    last_log = 0.0
    last_motf_log = 0.0
    prev_state = None
    while True:
        if src.stop_at > 0.0 and (time.time() - src.t0) > src.stop_at:
            time.sleep(0.2)         # 模拟"电机停止/数据中断"：不再产生帧
            continue
        f = src.next_frame()
        hub.push(f)
        now = time.time()
        if now - last_log >= 2.0:          # 每 2s 打一条心跳，方便预览日志搜索/复制
            last_log = now
            hub.log(f"SIM 心跳 t={now - src.t0:.1f}s (仿真模式)")
        # 与 jlink 解析器一致的 MOTF 节流日志（~10 条/秒 + 状态变化）
        state = (f.mode, f.phase, f.sync)
        if state != prev_state or (now - last_motf_log) >= 0.1:
            last_motf_log = now
            prev_state = state
            hub.log(
                f"MOTF,{f.mode},{f.phase},{int(f.rotor_mrad)},{int(f.theta_mrad)},"
                f"{int(f.iq_ma)},{int(f.id_ma)},{int(f.vq_mv)},{int(f.vd_mv)},"
                f"{int(f.spd_rpm)},{f.sync},{int(f.diff_mrad)},{int(f.freq_cHz)},{f.ms}")
        next_t += interval
        delay = next_t - time.perf_counter()
        while delay > 0:
            if delay > 0.002:
                time.sleep(delay - 0.0015)
            delay = next_t - time.perf_counter()


# ======================================================================
# J-Link RTT 数据源
# ======================================================================

class JLinkRttSource:
    def __init__(self, device: str, speed_khz: int = 4000, channel: int = 6,
                 rtt_addr: int = 0, ram_base: int = 0x1FFF8000,
                 ram_size: int = 0x2F000):
        try:
            import pylink  # type: ignore
        except Exception as exc:
            raise SystemExit(
                "未找到 pylink。请先安装：pip install pylink-square\n"
                "（同时需要已安装 Segger J-Link 软件，提供 JLinkARM.dll）\n"
                f"  原始错误：{exc}")
        self.pylink = pylink
        self.device = device
        self.speed_khz = speed_khz
        self.channel = channel
        self.rtt_addr = rtt_addr
        self.ram_base = ram_base
        self.ram_size = ram_size
        self.jl = None
        self._lock = threading.Lock()
        self.reconnect_evt = threading.Event()

    def describe(self):
        return f"J-Link RTT ch{self.channel} @ {self.device} ({self.speed_khz} kHz)"

    def set_channel(self, channel: int):
        with self._lock:
            self.channel = int(channel)
        self.reconnect_evt.set()

    def close(self):
        with self._lock:
            jl = self.jl
            self.jl = None
        if jl is not None:
            try:
                jl.close()
            except Exception:
                pass

    def _scan_rtt_cb(self, jl):
        """RAM 魔数扫描：找 "SEGGER RTT\0"。
        HC32F460 默认 RAM 0x1FFF8000 ~ 0x20026FFF (188K=0x2F000)，
        可用 --rtt-ram-base / --rtt-ram-size 覆盖。"""
        magic = b"SEGGER RTT\x00"
        chunk = 0x4000
        try:
            for off in range(0, self.ram_size, chunk):
                data = bytes(jl.memory_read(self.ram_base + off,
                                            min(chunk, self.ram_size - off)))
                idx = data.find(magic)
                if idx >= 0:
                    return self.ram_base + off + idx
        except Exception:
            pass
        return 0

    def open(self):
        jl = self.pylink.JLink()
        try:
            jl.open()
            # 关键：显式指定 SWD 接口。否则 J-Link DLL 可能"connect 不报错但目标访问
            # 全部失败（Target is not connected）"，导致找不到 RTT 控制块。
            try:
                jl.set_tif(self.pylink.enums.JLinkInterfaces.SWD)
            except Exception:
                pass
            print(f"[J-Link] SN={jl.serial_number} 固件={jl.firmware_version}")
            jl.connect(self.device, speed=self.speed_khz, verbose=True)
            try:
                if jl.halted():     # attach 可能停核；尽量恢复运行（部分 DLL 支持 exec Go）
                    jl.exec_command("Go")
            except Exception:
                pass
            addr = self.rtt_addr
            auto_ok = False
            if not addr:
                try:
                    jl.rtt_start()          # J-Link DLL 自动搜索
                    jl.rtt_get_num_up_buffers()
                    auto_ok = True
                except Exception:
                    addr = self._scan_rtt_cb(jl)
                    if not addr:
                        raise RuntimeError(
                            "未找到 RTT 控制块。请确认：1) 目标在运行且固件包含 SEGGER RTT；"
                            "2) 在 Keil 里看 _SEGGER_RTT 的地址，用 --rtt-addr 0x地址 指定")
                    print(f"[J-Link] 自动搜索失败，RAM 扫描定位到 RTT 控制块 @ 0x{addr:08X}")
                    try:
                        jl.rtt_stop()       # 复位 RTT 状态，否则后续指定地址会被忽略
                    except Exception:
                        pass
            if addr and not auto_ok:
                jl.rtt_start(block_address=addr)
            nbuf = None
            for _ in range(5):          # DLL 读取 CB 需要一点时间，重试几次
                try:
                    nbuf = jl.rtt_get_num_up_buffers()
                    break
                except Exception:
                    time.sleep(0.1)
            if nbuf is None:
                raise RuntimeError("RTT 控制块已定位但读取失败，请重试")
            if nbuf < (self.channel + 1):
                print(f"[J-Link] 警告：固件只有 {nbuf} 个上行通道（无 ch{self.channel}）。"
                      "请确认已烧录新固件（SEGGER_RTT_MAX_NUM_UP_BUFFERS=7）")
            print(f"[J-Link] RTT 控制块已定位（{nbuf} 个上行通道），读取通道 {self.channel} ...")
            self.jl = jl
        except Exception:
            try:
                jl.close()      # 连接失败务必释放，避免泄漏 J-Link 连接（多次重试会耗尽）
            except Exception:
                pass
            raise

    def next_chunk(self) -> bytes:
        return self.jl.rtt_read(self.channel, 2048)


def jlink_loop(hub: DataHub, src: JLinkRttSource):
    while True:
        try:
            src.open()
        except Exception as exc:
            msg = f"J-Link 连接失败，3 秒后自动重试: {exc}"
            hub.set_status("error", msg)
            hub.log("[MotorScope] " + msg)
            print("[MotorScope] " + msg)
            src.reconnect_evt.wait(timeout=3.0)
            src.reconnect_evt.clear()
            continue
        hub.set_status("running", src.describe())
        parser = RttParser()
        while True:
            if src.reconnect_evt.is_set():
                src.reconnect_evt.clear()
                src.close()
                break
            try:
                chunk = src.next_chunk()
            except Exception as exc:
                msg = f"RTT 读取失败（连接丢失），3 秒后自动重连: {exc}"
                hub.set_status("error", msg)
                hub.log("[MotorScope] " + msg)
                print("[MotorScope] " + msg)
                src.close()
                break
            if not chunk:
                time.sleep(0.003)
                continue
            parser.feed(chunk, hub.push, hub.log)
        time.sleep(0.2)


# ======================================================================
# HTTP 服务
# ======================================================================

class MotorHandler(BaseHTTPRequestHandler):
    hub: DataHub = None
    src = None

    def log_message(self, *args):
        pass

    def _send(self, code, ctype, body: bytes):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        self.do_GET()

    def do_GET(self):
        path = self.path.split("?")[0]
        if path.startswith("/data"):
            since = 0
            log_since = 0
            for key, val in (("since=", "since"), ("logsince=", "logsince")):
                if key in self.path:
                    try:
                        v = int(self.path.split(key)[1].split("&")[0])
                    except ValueError:
                        v = 0
                    if val == "since":
                        since = v
                    else:
                        log_since = v
            body = json.dumps(self.hub.snapshot(since, log_since)).encode("utf-8")
            self._send(200, "application/json; charset=utf-8", body)
            return
        if path.startswith("/health"):
            ch = getattr(self.src, "channel", None) if self.src is not None else None
            with self.hub.lock:
                now = time.time()
                age = -1.0
                if self.hub.history:
                    age = (now - self.hub.history[-1][1]) * 1000.0
                status = self.hub.status
                detail = self.hub.detail
                frames = self.hub.frames_total
                seq = self.hub.seq
                fps = self.hub.frames_total / max(now - self.hub.start_time, 1e-6)
                uptime = now - self.hub.start_time
            body = json.dumps({
                "ok": True,
                "status": status,
                "detail": detail,
                "channel": ch,
                "frames": frames,
                "seq": seq,
                "fps": round(fps, 1),
                "last_age_ms": round(age, 1),
                "uptime": round(uptime, 1),
            }).encode("utf-8")
            self._send(200, "application/json; charset=utf-8", body)
            return
        if path.startswith("/reconnect"):
            ch = None
            if "channel=" in self.path:
                try:
                    ch = int(self.path.split("channel=")[1].split("&")[0])
                except ValueError:
                    ch = None
            if (self.src is not None and hasattr(self.src, "set_channel")
                    and ch is not None):
                self.src.set_channel(ch)
                self.hub.set_status("connecting", f"重连中 ch{ch} ...")
            body = json.dumps({"ok": True, "channel": ch}).encode("utf-8")
            self._send(200, "application/json; charset=utf-8", body)
            return
        if path.startswith("/clearlogs"):
            with self.hub.lock:
                self.hub.log_lines.clear()
            body = json.dumps({"ok": True}).encode("utf-8")
            self._send(200, "application/json; charset=utf-8", body)
            return
        if path.startswith("/clear_history"):
            self.hub.clear_history()
            body = json.dumps({"ok": True}).encode("utf-8")
            self._send(200, "application/json; charset=utf-8", body)
            return
        rel = "index.html" if path in ("/", "/index.html") else path.lstrip("/")
        fpath = (WEB_DIR / rel).resolve()
        root = WEB_DIR.resolve()
        if not str(fpath).startswith(str(root)) or not fpath.is_file():
            self._send(404, "text/plain; charset=utf-8", b"not found")
            return
        ctype = {".html": "text/html; charset=utf-8",
                 ".js": "application/javascript; charset=utf-8",
                 ".css": "text/css; charset=utf-8",
                 ".png": "image/png"}.get(fpath.suffix.lower(),
                                          "application/octet-stream")
        self._send(200, ctype, fpath.read_bytes())


# ======================================================================
# 入口
# ======================================================================

def main():
    ap = argparse.ArgumentParser(
        description="MotorScope - J-Link RTT 电机实时可视化调试助手（FOC）")
    ap.add_argument("--mode", choices=["sim-foc", "jlink"], default="sim-foc")
    ap.add_argument("--device", default="HC32F460")
    ap.add_argument("--speed-khz", type=int, default=4000)
    ap.add_argument("--channel", type=int, default=0, help="RTT 上行通道号（默认 0）")
    ap.add_argument("--rtt-addr", type=lambda x: int(x, 0), default=0,
                    help="RTT 控制块地址（十六进制）；0 = 自动搜索")
    ap.add_argument("--rtt-ram-base", type=lambda x: int(x, 0), default=0x1FFF8000,
                    help="RAM 起始地址（用于扫描，HC32F460 默认 0x1FFF8000）")
    ap.add_argument("--rtt-ram-size", type=lambda x: int(x, 0), default=0x2F000,
                    help="RAM 大小（用于扫描，HC32F460 默认 0x2F000）")
    ap.add_argument("--rate", type=int, default=200, help="仿真采样率 Hz")
    ap.add_argument("--sim-stop-after", type=float, default=0.0,
                    help="仿真运行 N 秒后停止发帧（模拟数据中断，0=不停）")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()

    def _timer():
        if sys.platform == "win32":
            try:
                import ctypes
                ctypes.windll.winmm.timeBeginPeriod(1)
            except Exception:
                pass
    _timer()

    hub = DataHub()
    if args.mode == "jlink":
        src = JLinkRttSource(args.device, args.speed_khz, args.channel,
                             args.rtt_addr, args.rtt_ram_base, args.rtt_ram_size)
        thread = threading.Thread(target=jlink_loop, args=(hub, src), daemon=True)
    else:
        src = SimFoc(args.rate, args.sim_stop_after)
        thread = threading.Thread(target=sim_loop, args=(hub, src, args.rate),
                                  daemon=True)
    thread.start()

    MotorHandler.hub = hub
    MotorHandler.src = src
    try:
        httpd = ThreadingHTTPServer(("127.0.0.1", args.port), MotorHandler)
    except OSError as exc:
        print(f"[MotorScope] 端口 {args.port} 被占用：{exc}")
        return 1
    url = f"http://127.0.0.1:{args.port}/"
    print(f"[MotorScope] 数据源: {src.describe()}")
    if args.mode == "sim-foc":
        print("[MotorScope] ⚠️ 仿真模式：显示的是模拟数据，不是真实电机。"
              "实机请用 --mode jlink --device HC32F460 --speed-khz 1000")
    print(f"[MotorScope] 打开: {url}   (Ctrl+C 退出)")
    if not args.no_browser:
        import webbrowser
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[MotorScope] 已退出")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


