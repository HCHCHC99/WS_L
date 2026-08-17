# -*- coding: utf-8 -*-
"""主窗口：电机动画 + 仪表 + 5 示波器 + 状态栏，QTimer 驱动刷新。
工具栏：J-Link 序列号选择（多 USB 口）+ 手动刷新。
配色：米色浅色主题；状态栏区分 J-Link 设备层 / 目标芯片层连接失败。"""
import time

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (QComboBox, QHBoxLayout, QLabel, QMainWindow,
                               QPushButton, QSplitter, QVBoxLayout, QWidget)

from widgets.gauge import Gauge
from widgets.motor_view import MotorView
from widgets.num_panel import NumPanel
from widgets.scope_view import ScopeView

# 与 motor_scope.py 的 FOC_MODE_* / FOC_PHASE_* 名称保持一致
MODE_NAMES = {0: "停止", 1: "开环", 2: "电流环", 3: "对齐"}
PHASE_NAMES = {0: "idle", 1: "hold", 2: "ramp/sync", 3: "run", 4: "align"}

# 米色浅色主题
BG = "#F6F1E7"            # 窗口底色（米色）
PANEL = "#FDFBF7"         # 面板底（奶油白）
TEXT = "#3D3D38"          # 主文字（深暖灰）
TEXT2 = "#6E685C"         # 次文字
BORDER = "#D9CDB9"        # 边框
BTN = "#EFE7D6"           # 按钮底
BTN_HOVER = "#E6DCC7"     # 按钮悬停

# 状态栏错误配色（三种故障层分别区分）：
#   J-Link 设备层=琥珀；芯片未连接=红；RTT 无数据(芯片已连接)=紫；数据中断=琥珀
COLOR_ERR_JLINK = "#B45309"
COLOR_ERR_CHIP = "#DC2626"
COLOR_ERR_RTT = "#6D28D9"
COLOR_STALE = "#B45309"


class MainWindow(QMainWindow):
    REFRESH_MS = 30          # 刷新周期（约 33fps）
    STALE_MS = 400           # 数据中断判定（与 web 版一致）

    def __init__(self, hub, data_thread, serial_arg=None):
        super().__init__()
        self.hub = hub
        self.thread = data_thread
        self._serial_arg = serial_arg
        self._error_active = False     # 连接错误显示中（数据中断提示不覆盖它）
        self.setWindowTitle("MotorScope 原生版（PySide6）")

        central = QWidget()
        central.setStyleSheet(f"""
            background-color:{BG}; color:{TEXT};
            QComboBox{{background:{PANEL};color:{TEXT};border:1px solid {BORDER};padding:2px 8px;}}
            QComboBox QAbstractItemView{{background:{PANEL};color:{TEXT};selection-background-color:{BTN};}}
            QPushButton{{background:{BTN};color:{TEXT};border:1px solid {BORDER};padding:4px 12px;border-radius:4px;}}
            QPushButton:hover{{background:{BTN_HOVER};}}
            QLabel{{background:transparent;}}
        """)
        outer = QVBoxLayout(central)
        outer.setContentsMargins(6, 6, 6, 6)

        # ---- 工具栏：J-Link 序列号选择 + 刷新 ----
        bar = QHBoxLayout()
        bar.addWidget(QLabel("J-Link 序列号"))
        self.serial_combo = QComboBox()
        self._populate_serials()
        bar.addWidget(self.serial_combo)
        self.btn_refresh = QPushButton("刷新")
        self.btn_refresh.clicked.connect(self._on_refresh)
        bar.addWidget(self.btn_refresh)
        bar.addStretch(1)
        outer.addLayout(bar)

        # ---- 主体：左电机 + 右仪表 / 下方 5 示波器 ----
        root = QHBoxLayout()
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
        outer.addLayout(root, 1)
        self.setCentralWidget(central)

        # 状态栏（米色）
        self.statusBar().setStyleSheet(f"QStatusBar{{background:{BTN};color:{TEXT};}}")
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

    def _populate_serials(self):
        """枚举已连接 J-Link 序列号，填入下拉框（"自动"= 不指定）。
        优先保留当前选择，其次命令行 --serial。"""
        cur = self.serial_combo.currentData() if hasattr(self, "serial_combo") else None
        self.serial_combo.clear()
        self.serial_combo.addItem("自动（默认）", None)
        try:
            import pylink
            for sn in pylink.JLink().connected_emulators():
                self.serial_combo.addItem("SN %d" % sn, sn)
        except Exception:
            pass
        sel = cur if cur is not None else self._serial_arg
        if sel is not None:
            idx = self.serial_combo.findData(sel)
            if idx >= 0:
                self.serial_combo.setCurrentIndex(idx)

    def _on_refresh(self):
        """手动刷新：重新枚举 J-Link 列表，并按当前选择重连。"""
        self._populate_serials()      # 启动后才插上的 J-Link 也能被列出
        self.thread.set_serial(self.serial_combo.currentData())
        self.thread.refresh()
        self._error_active = False
        self.lbl_status.setText("正在刷新…")
        self.lbl_status.setStyleSheet("")

    def _on_status(self, s):
        self._error_active = False
        self._last_status_text = s
        self.lbl_status.setText(s)
        self.lbl_status.setStyleSheet("")

    def _on_frame(self, fr):
        self.hub.push(fr, time.time())

    def _error_color(self, msg):
        """按故障层配色：J-Link 设备层=琥珀；RTT 无数据(芯片已连接)=紫；其余(芯片未连接/读取丢失)=红。"""
        if msg.startswith("J-Link 连接失败") or msg.startswith("缺少 pylink"):
            return COLOR_ERR_JLINK
        if msg.startswith("RTT 无数据"):
            return COLOR_ERR_RTT
        return COLOR_ERR_CHIP

    def _on_error(self, msg):
        self._error_active = True
        color = self._error_color(msg)
        # 前缀本身就是区分（J-Link 连接失败 / 芯片连接失败），再叠加颜色区分
        self.lbl_status.setText("❌ " + msg)
        self.lbl_status.setStyleSheet(f"color:{color};font-weight:bold;")

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
                if not self._error_active:
                    # 连接失败期间不覆盖错误文案（保留 J-Link/芯片区分提示）
                    self.lbl_status.setText("⚠️ 数据中断（%.0f ms 无新帧）" % age)
                    self.lbl_status.setStyleSheet(f"color:{COLOR_STALE};")
            else:
                # 数据恢复：清中断/错误样式，文案回到最近一次连接状态
                self._error_active = False
                self.lbl_status.setStyleSheet("")
                self.lbl_status.setText(self._last_status_text)
            self.lbl_state.setText("mode%d %s | phase %s | %s"
                                   % (fr.mode, MODE_NAMES.get(fr.mode, "?"),
                                      PHASE_NAMES.get(fr.phase, "?"),
                                      "已同步" if fr.sync else "未同步"))
            self.lbl_age.setText("帧龄 %.0f ms | 总帧 %d" % (age, self.hub.frames_total))
        else:
            self.lbl_state.setText("等待数据…")