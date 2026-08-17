# -*- coding: utf-8 -*-
"""主窗口：电机动画 + 仪表 + 5 示波器 + 状态栏，QTimer 驱动刷新。"""
import time

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (QHBoxLayout, QLabel, QMainWindow, QSplitter,
                               QVBoxLayout, QWidget)

from widgets.gauge import Gauge
from widgets.motor_view import MotorView
from widgets.num_panel import NumPanel
from widgets.scope_view import ScopeView

# 与 motor_scope.py 的 FOC_MODE_* / FOC_PHASE_* 名称保持一致
MODE_NAMES = {0: "停止", 1: "开环", 2: "电流环", 3: "对齐"}
PHASE_NAMES = {0: "idle", 1: "hold", 2: "ramp/sync", 3: "run", 4: "align"}


class MainWindow(QMainWindow):
    REFRESH_MS = 30          # 刷新周期（约 33fps）
    STALE_MS = 400           # 数据中断判定（与 web 版一致）

    def __init__(self, hub, data_thread):
        super().__init__()
        self.hub = hub
        self.thread = data_thread
        self.setWindowTitle("MotorScope 原生版（PySide6）")
        central = QWidget()
        root = QHBoxLayout(central)
        self.motor = MotorView(hub)

        right = QVBoxLayout()
        self.gauge = Gauge(hub)
        self.nums = NumPanel(hub)
        right.addWidget(self.gauge)
        right.addWidget(self.nums)
        right.addStretch(1)

        split = QSplitter()
        split.addWidget(self.motor)
        right_box = QWidget()
        right_box.setLayout(right)
        split.addWidget(right_box)
        # MotorView 固定 560x560，stretch 对固定尺寸控件不生效（仅兜底）
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 2)

        scopes = QVBoxLayout()
        self.scopes = {
            "cur": ScopeView(hub, "cur"),
            "angle": ScopeView(hub, "angle"),
            "diff": ScopeView(hub, "diff"),
            "mode": ScopeView(hub, "mode"),
            "cnt": ScopeView(hub, "cnt"),
        }
        for s in self.scopes.values():
            scopes.addWidget(s)

        root.addWidget(split, 3)
        right_scopes = QWidget()
        right_scopes.setLayout(scopes)
        root.addWidget(right_scopes, 4)
        self.setCentralWidget(central)

        # 状态栏
        self.lbl_status = QLabel("连接中…")
        self.lbl_state = QLabel("")
        self.lbl_age = QLabel("")
        self._last_status_text = "连接中…"   # data_status 成功文案，中断恢复用
        self.statusBar().addWidget(self.lbl_status)
        self.statusBar().addWidget(self.lbl_state)
        self.statusBar().addWidget(self.lbl_age)

        # 数据线程信号
        self.thread.frame_received.connect(self._on_frame)
        self.thread.data_error.connect(self._on_error)
        self.thread.data_status.connect(self._on_status)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(self.REFRESH_MS)

    def _on_status(self, s):
        self._last_status_text = s
        self.lbl_status.setText(s)

    def _on_frame(self, fr):
        self.hub.push(fr, time.time())

    def _on_error(self, msg):
        self.lbl_status.setText("❌ " + msg)
        self.lbl_status.setStyleSheet("color:#ef4444;")

    def _refresh(self):
        self.motor.update()
        self.gauge.update()
        self.nums.refresh()
        for s in self.scopes.values():
            s.update()
        fr = self.hub.latest
        if fr:
            age = (time.time() - self.hub.last_frame_wall) * 1000.0
            if age > self.STALE_MS:
                self.lbl_status.setText("⚠️ 数据中断（%.0f ms 无新帧）" % age)
                self.lbl_status.setStyleSheet("color:#f59e0b;")
            else:
                # 数据恢复：清中断/错误样式，文案回到最近一次连接状态
                self.lbl_status.setStyleSheet("")
                self.lbl_status.setText(self._last_status_text)
            self.lbl_state.setText("mode%d %s | phase %s | %s"
                                   % (fr.mode, MODE_NAMES.get(fr.mode, "?"),
                                      PHASE_NAMES.get(fr.phase, "?"),
                                      "已同步" if fr.sync else "未同步"))
            self.lbl_age.setText("帧龄 %.0f ms | 总帧 %d" % (age, self.hub.frames_total))
        else:
            self.lbl_state.setText("等待数据…")