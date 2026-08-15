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

    def to_list(self):
        return [self.mode, self.phase, int(round(self.rotor_mrad)),
                int(round(self.theta_mrad)), int(round(self.iq_ma)),
                int(round(self.id_ma)), int(round(self.vq_mv)),
                int(round(self.vd_mv)), int(round(self.spd_rpm)),
                self.sync, int(round(self.diff_mrad)),
                int(round(self.freq_cHz)), self.ms]


def parse_text_line(line: str):
    """解析文本帧：MOTF,<13 个字段>"""
    line = line.strip()
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
        )
    except (ValueError, IndexError):
        return None


_BIN_FMT = "<4sIiiiiiiiiiBBBB"      # 48 字节小端二进制帧
_BIN_SIZE = struct.calcsize(_BIN_FMT)


def parse_binary(buf: bytes):
    """解析 48 字节二进制帧（MOTF magic）。"""
    if len(buf) < _BIN_SIZE or buf[:4] != b"MOTF":
        return None
    (magic, ms, rotor, theta, iq, id_, vq, vd, spd, diff, freq,
     mode, phase, sync, rsv) = struct.unpack(_BIN_FMT, buf[:_BIN_SIZE])
    return FocFrame(mode=mode, phase=phase, rotor_mrad=float(rotor),
                    theta_mrad=float(theta), iq_ma=float(iq), id_ma=float(id_),
                    vq_mv=float(vq), vd_mv=float(vd), spd_rpm=float(spd),
                    sync=sync, diff_mrad=float(diff), freq_cHz=float(freq),
                    ms=ms)


class RttParser:
    """RTT 字节流 -> FocFrame。自动识别文本/二进制两种帧。"""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes, sink):
        self.buf.extend(data)
        if len(self.buf) >= 5 and self.buf[:5] == b"MOTF,":
            # 文本模式：按行切分
            while True:
                idx = self.buf.find(b"\n")
                if idx < 0:
                    break
                line = bytes(self.buf[:idx]).decode("utf-8", errors="replace")
                del self.buf[:idx + 1]
                fr = parse_text_line(line)
                if fr is not None:
                    sink(fr)
        else:
            # 二进制模式：按 magic 扫描
            while True:
                idx = self.buf.find(b"MOTF")
                if idx < 0:
                    if len(self.buf) > 3:
                        del self.buf[:-3]
                    break
                if len(self.buf) - idx >= _BIN_SIZE:
                    fr = parse_binary(bytes(self.buf[idx:idx + _BIN_SIZE]))
                    if fr is not None:
                        sink(fr)
                    del self.buf[:idx + _BIN_SIZE]
                else:
                    break


# ======================================================================
# 数据中心
# ======================================================================

class DataHub:
    def __init__(self, max_history=500, max_log=300):
        self.lock = threading.Lock()
        self.seq = 0
        self.latest = None
        self.history = deque(maxlen=max_history)
        self.log_lines = deque(maxlen=max_log)
        self.start_time = time.time()
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
        if raw:
            self.log(raw)

    def log(self, text: str):
        with self.lock:
            self.log_lines.append(text)

    def set_status(self, status: str, detail: str = ""):
        with self.lock:
            self.status = status
            self.detail = detail

    def snapshot(self, last_seq: int = 0):
        with self.lock:
            now = time.time()
            fps = self.frames_total / max(now - self.start_time, 1e-6)
            history = [
                [s, round(ts, 3), fr.to_list()]
                for s, ts, fr in self.history if s > last_seq
            ]
            latest = self.latest.to_list() if self.latest else None
            return {
                "ok": True,
                "status": self.status,
                "detail": self.detail,
                "seq": self.seq,
                "fps": round(fps, 1),
                "uptime": round(now - self.start_time, 1),
                "latest": latest,
                "history": history,
                "logs": list(self.log_lines)[-20:],
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

    def __init__(self, sample_hz: int = 200):
        self.sample_hz = sample_hz
        self.pp = 10
        self.t0 = time.time()
        self._last_theta = 0.0

    def describe(self):
        return f"仿真FOC (I-F启动->同步->运行, 极对数{self.pp}, {self.sample_hz}Hz)"

    def next_frame(self) -> FocFrame:
        t = time.time() - self.t0
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
            # 转子滞后于合成角，滞后量随时间收敛
            lag = max(0.05, 1.2 * (1 - (t - 2.5) / 5.0))
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
        return f


def sim_loop(hub: DataHub, src: SimFoc, sample_hz: int):
    hub.set_status("running", src.describe())
    interval = 1.0 / sample_hz
    next_t = time.perf_counter()
    while True:
        hub.push(src.next_frame())
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
    def __init__(self, device: str, speed_khz: int = 4000, channel: int = 6):
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
        self.jl = None

    def describe(self):
        return f"J-Link RTT ch{self.channel} @ {self.device} ({self.speed_khz} kHz)"

    def open(self):
        jl = self.pylink.JLink()
        jl.open()
        print(f"[J-Link] SN={jl.serial_number} 固件={jl.firmware_version}")
        jl.connect(self.device, speed=self.speed_khz, verbose=True)
        try:
            jl.go()             # 确保目标处于运行态（attach 可能停核，避免打断正在跑的电机）
        except Exception:
            pass
        jl.rtt_start()          # 自动搜索 RTT 控制块
        print(f"[J-Link] RTT 控制块已定位，读取上行通道 {self.channel} ...")
        self.jl = jl

    def next_chunk(self) -> bytes:
        return self.jl.rtt_read(self.channel, 2048)


def jlink_loop(hub: DataHub, src: JLinkRttSource):
    try:
        src.open()
    except Exception as exc:
        hub.set_status("error", f"J-Link 连接失败: {exc}")
        print(f"[MotorScope] {exc}")
        return
    hub.set_status("running", src.describe())
    parser = RttParser()
    while True:
        try:
            chunk = src.next_chunk()
        except Exception as exc:
            hub.set_status("error", f"RTT 读取失败: {exc}")
            time.sleep(0.5)
            continue
        if not chunk:
            time.sleep(0.003)
            continue
        parser.feed(chunk, hub.push)


# ======================================================================
# HTTP 服务
# ======================================================================

class MotorHandler(BaseHTTPRequestHandler):
    hub: DataHub = None

    def log_message(self, *args):
        pass

    def _send(self, code, ctype, body: bytes):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path.startswith("/data"):
            since = 0
            if "since=" in self.path:
                try:
                    since = int(self.path.split("since=")[1].split("&")[0])
                except ValueError:
                    since = 0
            body = json.dumps(self.hub.snapshot(since)).encode("utf-8")
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
    ap.add_argument("--channel", type=int, default=6, help="RTT 上行通道号")
    ap.add_argument("--rate", type=int, default=200, help="仿真采样率 Hz")
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
        src = JLinkRttSource(args.device, args.speed_khz, args.channel)
        thread = threading.Thread(target=jlink_loop, args=(hub, src), daemon=True)
    else:
        src = SimFoc(args.rate)
        thread = threading.Thread(target=sim_loop, args=(hub, src, args.rate),
                                  daemon=True)
    thread.start()

    MotorHandler.hub = hub
    try:
        httpd = ThreadingHTTPServer(("127.0.0.1", args.port), MotorHandler)
    except OSError as exc:
        print(f"[MotorScope] 端口 {args.port} 被占用：{exc}")
        return 1
    url = f"http://127.0.0.1:{args.port}/"
    print(f"[MotorScope] 数据源: {src.describe()}")
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


