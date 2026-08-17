# -*- coding: utf-8 -*-
"""主窗口：电机动画 + 仪表 + 5 示波器 + 日志分面板 + 状态栏，QTimer 驱动刷新。
工具栏：
  - J-Link 序列号选择（多 USB 口）+ 刷新 + RTT 通道选择；
  - 暂停/跟随实时 + 窗口宽度对数滑条（0.1s~20s）+ 时间回看滑条。
导航模型与 web 版一致：暂停/回看时缓冲写入与裁剪冻结，电机/仪表/示波器跟随 view_frame。
配色：米色浅色主题；状态栏区分 J-Link 设备层 / 芯片未连接 / RTT 无数据。"""
import math
import time

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (QComboBox, QHBoxLayout, QLabel, QLineEdit,
                               QMainWindow, QPushButton, QSlider, QSpinBox,
                               QSplitter, QVBoxLayout, QWidget)

from widgets.gauge import Gauge
from widgets.log_panel import LogPanel
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

# 状态栏错误配色（三种故障层分别区分）
COLOR_ERR_JLINK = "#B45309"   # 琥珀：J-Link 设备层
COLOR_ERR_CHIP = "#DC2626"    # 红：芯片未连接
COLOR_ERR_RTT = "#6D28D9"     # 紫：RTT 无数据（芯片已连接）
COLOR_STALE = "#B45309"

# 窗口宽度对数滑条（与 web 版一致：0.1s ~ 20s）
WIN_SEC_MIN = 0.1
WIN_SEC_MAX = 20.0


def win_slider_to_sec(v):
    return WIN_SEC_MIN * (WIN_SEC_MAX / WIN_SEC_MIN) ** (v / 1000.0)


def win_sec_to_slider(s):
    c = math.log(WIN_SEC_MAX / WIN_SEC_MIN)
    return int(round(1000.0 * math.log(max(WIN_SEC_MIN, min(WIN_SEC_MAX, s))
                                       / WIN_SEC_MIN) / c))


BTN_STYLE = ("QPushButton{background:%s;color:%s;border:1px solid %s;"
             "padding:4px 12px;border-radius:4px;}"
             "QPushButton:hover{background:%s;}" % (BTN, TEXT, BORDER, BTN_HOVER))
BTN_ACTIVE = ("QPushButton{background:#8A7B61;color:#FDFBF7;border:1px solid #8A7B61;"
              "padding:4px 12px;border-radius:4px;}")


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
            QComboBox QLineEdit{{background:{PANEL};color:{TEXT};border:none;}}
            QLineEdit{{background:{PANEL};color:{TEXT};border:1px solid {BORDER};padding:2px 6px;}}
            QSpinBox{{background:{PANEL};color:{TEXT};border:1px solid {BORDER};padding:2px 6px;}}
            QLabel{{background:transparent;}}
            QSlider::groove:horizontal{{height:4px;background:{BORDER};border-radius:2px;}}
            QSlider::handle:horizontal{{width:14px;background:#8A7B61;border-radius:7px;margin:-5px 0;}}
        """)
        outer = QVBoxLayout(central)
        outer.setContentsMargins(6, 6, 6, 6)
        outer.setSpacing(4)

        # ---- 工具栏行 1：J-Link 序列号 / 芯片 / SWD 速度 / RTT 通道 + 应用、刷新 ----
        cfg = self.thread._cfg
        bar = QHBoxLayout()
        bar.addWidget(QLabel("J-Link 序列号"))
        self.serial_combo = QComboBox()
        self._populate_serials()
        bar.addWidget(self.serial_combo)
        bar.addSpacing(10)
        bar.addWidget(QLabel("芯片"))
        self.combo_chip = QComboBox()
        self.combo_chip.setEditable(True)      # 可直接键入 J-Link 设备表中任意型号
        self.combo_chip.addItems(["HC32F460", "Cortex-M4"])
        self.combo_chip.setCurrentText(cfg.get("device", "HC32F460"))
        self.combo_chip.setMaximumWidth(150)
        bar.addWidget(self.combo_chip)
        bar.addSpacing(10)
        bar.addWidget(QLabel("SWD 速度"))
        self.combo_speed = QComboBox()
        self.combo_speed.setEditable(True)     # 可直接键入任意 kHz
        for s in (100, 400, 1000, 4000):
            self.combo_speed.addItem("%d kHz" % s, s)
        _si = self.combo_speed.findText("%d kHz" % cfg.get("speed_khz", 1000))
        if _si >= 0:
            self.combo_speed.setCurrentIndex(_si)   # setCurrentText 不会更新索引，必须用 setCurrentIndex
        else:
            self.combo_speed.setEditText("%d kHz" % cfg.get("speed_khz", 1000))
        self.combo_speed.setMaximumWidth(110)
        bar.addWidget(self.combo_speed)
        bar.addSpacing(10)
        bar.addWidget(QLabel("RTT 通道"))
        self.spin_ch = QSpinBox()
        self.spin_ch.setRange(0, 15)
        self.spin_ch.setValue(cfg.get("channel", 0))
        bar.addWidget(self.spin_ch)
        self.btn_apply = QPushButton("应用")
        self.btn_apply.setStyleSheet(BTN_STYLE)
        self.btn_apply.clicked.connect(self._on_apply)
        bar.addWidget(self.btn_apply)
        self.btn_refresh = QPushButton("刷新")
        self.btn_refresh.setStyleSheet(BTN_STYLE)
        self.btn_refresh.clicked.connect(self._on_refresh)
        bar.addWidget(self.btn_refresh)
        bar.addStretch(1)
        outer.addLayout(bar)

        # ---- 工具栏行 2：RAM 扫描区域 / RTT 地址（hex，应用时生效）----
        bar_ram = QHBoxLayout()
        bar_ram.addWidget(QLabel("RAM 基址"))
        self.edit_ram_base = self._hex_edit(cfg.get("ram_base", 0x1FFF8000), 118)
        bar_ram.addWidget(self.edit_ram_base)
        bar_ram.addWidget(QLabel("RAM 大小"))
        self.edit_ram_size = self._hex_edit(cfg.get("ram_size", 0x2F000), 96)
        bar_ram.addWidget(self.edit_ram_size)
        bar_ram.addWidget(QLabel("RTT 地址 (0=自动)"))
        self.edit_rtt_addr = self._hex_edit(cfg.get("rtt_addr", 0), 118)
        bar_ram.addWidget(self.edit_rtt_addr)
        bar_ram.addWidget(QLabel("提示：RTT 自动搜索失败时，在 Keil 里看 _SEGGER_RTT 地址填入 RTT 地址；RAM 区域用于缩小扫描范围"))
        bar_ram.addStretch(1)
        outer.addLayout(bar_ram)

        # ---- 工具栏行 2：暂停/实时 + 窗口滑条 + 时间滑条 ----
        bar2 = QHBoxLayout()
        self.btn_pause = QPushButton("⏸ 暂停")
        self.btn_pause.setStyleSheet(BTN_STYLE)
        self.btn_pause.clicked.connect(self._on_pause)
        bar2.addWidget(self.btn_pause)
        self.btn_live = QPushButton("▶ 跟随实时")
        self.btn_live.setStyleSheet(BTN_ACTIVE)
        self.btn_live.clicked.connect(self._on_live)
        bar2.addWidget(self.btn_live)
        bar2.addSpacing(14)
        bar2.addWidget(QLabel("窗口"))
        self.win_slider = QSlider(Qt.Horizontal)
        self.win_slider.setRange(0, 1000)
        self.win_slider.setValue(win_sec_to_slider(1.0))
        self.win_slider.setMaximumWidth(220)
        self.win_slider.valueChanged.connect(self._on_win)
        bar2.addWidget(self.win_slider)
        self.lbl_win = QLabel("1.0 s")
        self.lbl_win.setStyleSheet("color:%s;" % TEXT2)
        bar2.addWidget(self.lbl_win)
        bar2.addSpacing(14)
        bar2.addWidget(QLabel("时间"))
        self.time_slider = QSlider(Qt.Horizontal)
        self.time_slider.setRange(0, 1000)
        self.time_slider.setValue(1000)
        self.time_slider.setMaximumWidth(300)
        self.time_slider.valueChanged.connect(self._on_time)
        bar2.addWidget(self.time_slider)
        self.lbl_time = QLabel("实时")
        self.lbl_time.setStyleSheet("color:%s;" % TEXT2)
        bar2.addWidget(self.lbl_time)
        bar2.addStretch(1)
        outer.addLayout(bar2)

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
        root_box = QWidget()
        root_box.setLayout(root)

        # ---- 日志分面板（MOTF / 固件日志）----
        logs_split = QSplitter(Qt.Horizontal)
        self.panel_motf = LogPanel(hub, "motf")
        self.panel_main = LogPanel(hub, "main")
        logs_split.addWidget(self.panel_motf)
        logs_split.addWidget(self.panel_main)
        logs_split.setStretchFactor(0, 1)
        logs_split.setStretchFactor(1, 1)

        main_split = QSplitter(Qt.Vertical)
        main_split.addWidget(root_box)
        main_split.addWidget(logs_split)
        main_split.setStretchFactor(0, 5)
        main_split.setStretchFactor(1, 2)
        outer.addWidget(main_split, 1)
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
        self.thread.log_line.connect(self.hub.log)
        self.thread.data_error.connect(self._on_error)
        self.thread.data_status.connect(self._on_status)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(self.REFRESH_MS)

    # ---------------- 工具栏 ----------------
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

    def _hex_edit(self, value, width=110):
        e = QLineEdit("0x%X" % value)
        e.setMaximumWidth(width)
        e.setStyleSheet("QLineEdit{background:%s;color:%s;border:1px solid %s;padding:2px 6px;font-family:Consolas,monospace;}" % (PANEL, TEXT, BORDER))
        return e

    def _parse_hex(self, text, default):
        try:
            return int(text.strip(), 0)
        except Exception:
            return default

    def _parse_speed(self, text):
        """解析速度文本：支持 '400 kHz' / '400k' / '400'。"""
        t = text.strip().lower().replace("khz", "").replace("k", "").strip()
        try:
            return int(t)
        except Exception:
            return 1000

    def _apply_settings(self, reload_serials=False):
        """读取工具栏全部连接参数，重新连接。"""
        if reload_serials:
            self._populate_serials()
        dev = self.combo_chip.currentText().strip() or "HC32F460"
        spd = self._parse_speed(self.combo_speed.currentText())
        ram_base = self._parse_hex(self.edit_ram_base.text(), 0x1FFF8000)
        ram_size = self._parse_hex(self.edit_ram_size.text(), 0x2F000)
        rtt_addr = self._parse_hex(self.edit_rtt_addr.text(), 0)
        self.thread.set_serial(self.serial_combo.currentData())
        self.thread.set_device(dev)
        self.thread.set_speed(spd)
        self.thread.set_channel(self.spin_ch.value())
        self.thread.set_ram(ram_base, ram_size, rtt_addr)
        self.thread.refresh()
        self._error_active = False
        self.lbl_status.setText("正在重连: %s @ %d kHz ..." % (dev, spd))
        self.lbl_status.setStyleSheet("")

    def _on_apply(self):
        """应用当前连接参数并重连。"""
        self._apply_settings(reload_serials=False)

    def _on_refresh(self):
        """刷新：重新枚举 J-Link 列表并按当前连接参数重连。"""
        self._apply_settings(reload_serials=True)

    def _on_pause(self):
        """暂停：停在当前最新时刻（缓冲写入/裁剪/最新帧全冻结）。"""
        self.hub.pause()

    def _on_live(self):
        """回到实时。"""
        self.hub.live()

    def _on_win(self, v):
        """窗口宽度：对数滑条；暂停/回看时保持视野中心（不跳回实时）。"""
        win = self.hub.window_bounds()
        center = (win[0] + win[1]) / 2 if win else 0
        self.hub.window_sec = win_slider_to_sec(v)
        if not self.hub.follow_live and win:
            self.hub.view_end = center + self.hub.window_sec / 2
        self.lbl_win.setText("%.1f s" % self.hub.window_sec)

    def _on_time(self, v):
        """时间回看滑条：拖动即进入回看（不再跟随实时）。"""
        win = self.hub.window_bounds()
        if not win:
            return
        self.hub.follow_live = False
        lo = win[0] + self.hub.window_sec
        hi = win[1]
        self.hub.view_end = lo + (v / 1000.0) * (hi - lo)

    # ---------------- 状态栏 ----------------
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
        self.lbl_status.setText("❌ " + msg)
        self.lbl_status.setStyleSheet(f"color:{color};font-weight:bold;")

    # ---------------- 主循环 ----------------
    def _update_nav(self):
        """同步滑条/标签/按钮高亮到当前导航状态（与 web updateSlider 一致）。"""
        win = self.hub.window_bounds()
        self.win_slider.blockSignals(True)
        self.win_slider.setValue(win_sec_to_slider(self.hub.window_sec))
        self.win_slider.blockSignals(False)
        self.lbl_win.setText("%.1f s" % self.hub.window_sec)
        if not win:
            self.time_slider.blockSignals(True)
            self.time_slider.setValue(1000)
            self.time_slider.blockSignals(False)
            self.lbl_time.setText("--")
            return
        minT, maxT = win
        lo = minT + self.hub.window_sec
        hi = maxT
        if self.hub.follow_live or hi <= lo + 1e-6:
            self.hub.view_end = maxT
            self.time_slider.blockSignals(True)
            self.time_slider.setValue(1000)
            self.time_slider.blockSignals(False)
            self.lbl_time.setText("实时")
            self.btn_live.setStyleSheet(BTN_ACTIVE)
            self.btn_pause.setStyleSheet(BTN_STYLE)
        else:
            frac = min(1.0, max(0.0, (self.hub.view_end - lo) / (hi - lo)))
            self.time_slider.blockSignals(True)
            self.time_slider.setValue(int(round(frac * 1000)))
            self.time_slider.blockSignals(False)
            self.lbl_time.setText("回看 %.2f s" % max(0.0, maxT - self.hub.view_end))
            self.btn_live.setStyleSheet(BTN_STYLE)
            self.btn_pause.setStyleSheet(BTN_ACTIVE)

    def _refresh(self):
        self.hub.refresh_view()
        self._update_nav()
        if self.hub.logs_dirty:
            for p in (self.panel_motf, self.panel_main):
                p.sync()
            self.hub.logs_dirty = False
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
                    self.lbl_status.setText("⚠️ 数据中断（%.0f ms 无新帧）" % age)
                    self.lbl_status.setStyleSheet(f"color:{COLOR_STALE};")
            else:
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

