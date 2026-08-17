# -*- coding: utf-8 -*-
"""数据线程：J-Link RTT（复用 motor_scope 的 JLinkRttSource/RttParser）
或 --mock 开发自测源。解析出的 FocFrame 经 Qt 信号发到主线程。"""
import math
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "motor_scope"))
import motor_scope as ms  # noqa: E402  (复用解析/连接逻辑)

from PySide6.QtCore import QThread, Signal  # noqa: E402

D2R = math.pi / 180.0


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

    def set_serial(self, serial):
        """设置 J-Link 序列号（多 USB 口时选择，下次重连生效）。"""
        self._cfg["serial"] = serial

    def refresh(self):
        """手动刷新：打断当前连接，按最新配置重连。"""
        self._refresh_evt.set()

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
        while not self.isInterruptionRequested():
            self.frame_received.emit(gen.next_frame())
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
            except Exception as exc:
                self.data_error.emit(f"J-Link 连接失败，3 秒后重试: {exc}")
                src.close()
                time.sleep(3.0)
                continue
            self.data_status.emit(src.describe())
            parser = ms.RttParser(log_motf=False)
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
                parser.feed(chunk, self.frame_received.emit)
            src.close()
            time.sleep(0.2)
            # （原 finally 结构已并入：内层循环后统一 close）
