# -*- coding: utf-8 -*-
"""数据线程：J-Link RTT（复用 motor_scope 的 JLinkRttSource/RttParser/DataHub）
或 --mock 开发自测源。解析出的 FocFrame 经 Qt 信号发到主线程；
日志行（MOTF 节流 / 固件 MAIN_D 等）经 log_line 信号发到主线程日志面板。
jlink 模式用 motor_scope.DataHub 负责写 history_scope.txt / history_main.txt
（启动即清空，2MB 上限，与 web 版一致），避免 GUI 线程做文件 I/O。"""
import math
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "motor_scope"))
import motor_scope as ms  # noqa: E402  (复用解析/连接/历史文件逻辑)

from PySide6.QtCore import QThread, Signal  # noqa: E402

D2R = math.pi / 180.0


def _motf_line(fr):
    """生成与固件一致的 MOTF 文本行（21 字段，供 mock 日志面板/回看历史）。"""
    return ("MOTF,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d"
            % tuple(fr.to_list()))


class MockFrames:
    """开发自测源：生成类似 I-F 运行期的 MOTF 帧（非用户仿真功能，仅 --mock 用）。"""

    def __init__(self, rate=200):
        self.rate = rate
        self.i = 0
        self.pp = 10

    def next_frame(self):
        self.i += 1
        t = self.i / self.rate
        th_elec = (t * 20.0 * 360.0) % 360.0          # 20Hz 电频率（电角度，折叠）
        mech = t * 20.0 * 360.0 / self.pp              # 机械角度（°），连续不折叠
        iq = 1500.0 + 200.0 * math.sin(2 * math.pi * 0.8 * t)
        id = 25.0 * math.sin(2 * math.pi * 4 * t)
        return ms.FocFrame(
            mode=2, phase=3, sync=1,
            rotor_mrad=(th_elec + 0.25 * 180.0 / math.pi) * D2R * 1000.0,
            theta_mrad=th_elec * D2R * 1000.0,
            iq_ma=iq, id_ma=id,
            vq_mv=500.0 + iq * 0.15, vd_mv=40.0 + id * 0.1,
            spd_rpm=120.0, diff_mrad=250.0, freq_cHz=2000,
            ms=int(t * 1000), mech_mrad=mech * D2R * 1000.0,
            is_ma=math.hypot(id, iq),
            is_angle_mrad=math.atan2(iq, id) * 1000.0,
            v_mv=math.hypot(40.0 + id * 0.1, 500.0 + iq * 0.15),
            v_angle_mrad=math.atan2(500.0 + iq * 0.15, 40.0 + id * 0.1) * 1000.0,
            theta_mech_mrad=mech * D2R * 1000.0,
            cnt=int(mech * D2R * (4096.0 / (2 * math.pi))),
        )


class DataThread(QThread):
    frame_received = Signal(object)   # motor_scope.FocFrame
    log_line = Signal(str)            # 日志行（MOTF 节流 / 固件 MAIN_D 等）
    data_error = Signal(str)
    data_status = Signal(str)

    def __init__(self, mock=False, mock_rate=200, device="HC32F460",
                 speed_khz=1000, channel=0, rtt_addr=0,
                 ram_base=0x1FFF8000, ram_size=0x2F000, serial=None, parent=None):
        super().__init__(parent)
        self._mock = mock
        self._mock_rate = mock_rate
        self._cfg = dict(device=device, speed_khz=speed_khz, channel=channel,
                         rtt_addr=rtt_addr, ram_base=ram_base, ram_size=ram_size,
                         serial=serial)
        self._refresh_evt = threading.Event()
        self._writer = None           # jlink 模式：motor_scope.DataHub（写 history 文件）

    def set_serial(self, serial):
        """设置 J-Link 序列号（多 USB 口时选择，下次重连生效）。"""
        self._cfg["serial"] = serial

    def set_channel(self, channel):
        """设置 RTT 通道并触发重连（与 web 版"应用"一致）。"""
        self._cfg["channel"] = int(channel)
        self._refresh_evt.set()

    def set_device(self, device):
        """设置目标芯片型号（如 HC32F460 / Cortex-M4），下次重连生效。"""
        self._cfg["device"] = str(device).strip() or "HC32F460"

    def set_speed(self, speed_khz):
        """设置 SWD 接口速度（kHz），下次重连生效。"""
        self._cfg["speed_khz"] = int(speed_khz)

    def set_ram(self, ram_base, ram_size, rtt_addr):
        """设置 RTT 控制块扫描参数（RAM 基址/大小 + 显式 RTT 地址），下次重连生效。"""
        self._cfg["ram_base"] = int(ram_base)
        self._cfg["ram_size"] = int(ram_size)
        self._cfg["rtt_addr"] = int(rtt_addr)

    def refresh(self):
        """手动刷新：打断当前连接，按最新配置重连。"""
        self._refresh_evt.set()

    def clear_history(self):
        """清空 history_scope.txt / history_main.txt（窗口关闭时调用）。"""
        w = self._writer
        if w is not None:
            try:
                w.clear_history()
            except Exception:
                pass

    def run(self):
        if self._mock:
            self._run_mock()
        else:
            self._run_jlink()

    def _run_mock(self):
        self.data_status.emit("自测 mock 源（--mock）")
        gen = MockFrames(self._mock_rate)
        interval = 1.0 / self._mock_rate
        next_t = time.perf_counter()
        prev_state = None
        last_log = 0.0
        last_main = 0.0
        while not self.isInterruptionRequested():
            fr = gen.next_frame()
            self.frame_received.emit(fr)
            now = time.time()
            state = (fr.mode, fr.phase, fr.sync)
            if state != prev_state or (now - last_log) >= 0.1:
                last_log = now
                self.log_line.emit(_motf_line(fr))     # 节流 MOTF 日志（~10 条/秒）
                prev_state = state
            if (now - last_main) >= 2.0:               # 每 2s 一条固件日志
                last_main = now
                self.log_line.emit("MAIN_D: mock 固件日志 i=%d rate=%dHz"
                                   % (gen.i, self._mock_rate))
            next_t += interval
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)

    def _run_jlink(self):
        while not self.isInterruptionRequested():
            try:
                src = ms.JLinkRttSource(**self._cfg)
            except SystemExit as exc:
                self.data_error.emit(f"缺少 pylink / SEGGER J-Link 软件: {exc}")
                return
            try:
                src.open()
            except ms.JLinkOpenError as exc:
                self.data_error.emit(f"J-Link 连接失败（未找到/无法打开 J-Link），3 秒后重试: {exc}")
                src.close()
                time.sleep(3.0)
                continue
            except ms.ChipConnectError as exc:
                self.data_error.emit(f"芯片未连接（J-Link 已连接，目标无响应），3 秒后重试: {exc}")
                src.close()
                time.sleep(3.0)
                continue
            except ms.RttNoDataError as exc:
                self.data_error.emit(f"RTT 无数据（芯片已连接，未找到 RTT 控制块），3 秒后重试: {exc}")
                src.close()
                time.sleep(3.0)
                continue
            except Exception as exc:
                self.data_error.emit(f"连接异常，3 秒后重试: {exc}")
                src.close()
                time.sleep(3.0)
                continue
            self.data_status.emit(src.describe())
            # history 文件写入器：首次成功连接时创建即清空两文件（与 web 版一致）；
            # 之后重连复用同一实例，历史文件不因重连被清空。写操作在数据线程。
            if self._writer is None:
                self._writer = ms.DataHub(max_history=1, max_log=1)
            parser = ms.RttParser(log_motf=True)

            def frame_sink(fr):
                try:
                    self._writer.push(fr)          # history_scope.txt（全部帧）
                except Exception:
                    pass
                self.frame_received.emit(fr)

            def log_sink(line):
                try:
                    self._writer.log(line)         # 非 MOTF -> history_main.txt
                except Exception:
                    pass
                self.log_line.emit(line)

            while not self.isInterruptionRequested():
                if self._refresh_evt.is_set():
                    self._refresh_evt.clear()
                    break
                try:
                    chunk = src.next_chunk()
                except Exception as exc:
                    self.data_error.emit(f"RTT 读取失败，准备重连: {exc}")
                    break
                if not chunk:
                    time.sleep(0.003)
                    continue
                parser.feed(chunk, frame_sink, log_sink)
            src.close()
            time.sleep(0.2)