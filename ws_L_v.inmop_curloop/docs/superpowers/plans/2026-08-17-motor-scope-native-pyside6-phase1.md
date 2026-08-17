# MotorScope 原生版（PySide6）Phase 1 核心 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 用 PySide6 原生重写 motor_scope 的 Phase 1 核心：J-Link RTT 数据链路 + 电机剖视图动画 + 5 个示波器 + 转速表/数值面板，做成双击即用的 exe（不依赖浏览器/Web 技术）。

**Architecture:** 进程内数据流（去掉 HTTP/浏览器）。`DataThread(QThread)` 复用 `tools/motor_scope/motor_scope.py` 的 `JLinkRttSource`/`RttParser`/`FocFrame` 解析，解析出的帧经 Qt 信号发到主线程写入 `Hub`（环形缓冲 + latest），`QTimer(~30ms)` 驱动各 QPainter 自绘控件重绘。提供 `--mock` 仅作开发自测界面渲染用（非用户仿真功能）。

**Tech Stack:** Python 3.10+, PySide6 (Qt6, LGPL), pylink（依赖 SEGGER J-Link 软件），PyInstaller（打包 onedir）。

**设计依据:** `docs/superpowers/specs/2026-08-17-motor-scope-native-pyside6-design.md`（已获用户确认）。MOTF 协议：文本 21 token / 二进制 76B，固件端零改动。

**测试约束:** 纯 Python 工程，可本机直接运行测试。GUI 冒烟用 `python main.py --mock` 人工查看；解析/缓冲用脚本断言验证。打包（PyInstaller）在最后任务给出命令，需用户环境执行。

---

## Task 1: `hub.py` —— 数据缓冲（纯 Python，可单测）

**Files:**
- Create: `D:\WS_L\ws_L_v.inmop_curloop\tools\motor_scope_native\hub.py`
- Test: 用 `py -` 内联断言验证

- [ ] **Step 1: 创建 hub.py**

```python
# -*- coding: utf-8 -*-
"""MotorScope 原生版：数据缓冲（环形 + 最新帧）。纯 Python，不依赖 Qt，便于单测。"""
from collections import deque
from math import pi

RAD2DEG = 180.0 / pi


class Hub:
    """按时间裁剪的环形缓冲：每帧把显示所需字段按"度/rad/mA/mV"换算后入队。
    与 web 版 scopeAppend 的换算一致（mrad->deg、mrad->rad、mA/mV 原值）。"""

    KEEP_SEC = 20.0          # 保留时长（s）
    MAXLEN = 30000           # 最大点数（与 web 版 SCOPE_CAP 一致）

    def __init__(self):
        self.t = deque(maxlen=self.MAXLEN)
        self.mode = deque(maxlen=self.MAXLEN)
        self.iq = deque(maxlen=self.MAXLEN)
        self.id = deque(maxlen=self.MAXLEN)
        self.vd = deque(maxlen=self.MAXLEN)
        self.vq = deque(maxlen=self.MAXLEN)
        self.spd = deque(maxlen=self.MAXLEN)
        self.freq = deque(maxlen=self.MAXLEN)
        self.diff_rad = deque(maxlen=self.MAXLEN)
        self.rotor_deg = deque(maxlen=self.MAXLEN)
        self.theta_deg = deque(maxlen=self.MAXLEN)
        self.mech_deg = deque(maxlen=self.MAXLEN)
        self.theta_mech_deg = deque(maxlen=self.MAXLEN)
        self.is_ma = deque(maxlen=self.MAXLEN)
        self.is_ang_deg = deque(maxlen=self.MAXLEN)
        self.v_mv = deque(maxlen=self.MAXLEN)
        self.v_ang_deg = deque(maxlen=self.MAXLEN)
        self.cnt = deque(maxlen=self.MAXLEN)
        self.latest = None
        self.frames_total = 0
        self.last_frame_wall = 0.0

    def _deques(self):
        return (self.t, self.mode, self.iq, self.id, self.vd, self.vq, self.spd,
                self.freq, self.diff_rad, self.rotor_deg, self.theta_deg,
                self.mech_deg, self.theta_mech_deg, self.is_ma, self.is_ang_deg,
                self.v_mv, self.v_ang_deg, self.cnt)

    def _popleft_all(self):
        for dq in self._deques():
            if dq:
                dq.popleft()

    def push(self, fr, wall_t):
        """fr: motor_scope.FocFrame；wall_t: 接收墙钟时间（秒）。"""
        while self.t and self.t[0] < wall_t - self.KEEP_SEC:
            self._popleft_all()
        self.t.append(wall_t)
        self.mode.append(fr.mode)
        self.iq.append(fr.iq_ma)
        self.id.append(fr.id_ma)
        self.vd.append(fr.vd_mv)
        self.vq.append(fr.vq_mv)
        self.spd.append(fr.spd_rpm)
        self.freq.append(fr.freq_cHz)
        self.diff_rad.append(fr.diff_mrad / 1000.0)
        self.rotor_deg.append(fr.rotor_mrad / 1000.0 * RAD2DEG)
        self.theta_deg.append(fr.theta_mrad / 1000.0 * RAD2DEG)
        self.mech_deg.append(fr.mech_mrad / 1000.0 * RAD2DEG)
        self.theta_mech_deg.append(fr.theta_mech_mrad / 1000.0 * RAD2DEG)
        self.is_ma.append(fr.is_ma)
        self.is_ang_deg.append(fr.is_angle_mrad / 1000.0 * RAD2DEG)
        self.v_mv.append(fr.v_mv)
        self.v_ang_deg.append(fr.v_angle_mrad / 1000.0 * RAD2DEG)
        self.cnt.append(fr.cnt)
        self.latest = fr
        self.frames_total += 1
        self.last_frame_wall = wall_t

    def last_t(self):
        return self.t[-1] if self.t else 0.0

    def first_t(self):
        return self.t[0] if self.t else 0.0
```

- [ ] **Step 2: 单测（内联断言）**

```python
# 在 tools\motor_scope_native 下运行
import sys
sys.path.insert(0, r"..\motor_scope")
import motor_scope as ms
from hub import Hub

h = Hub()
t0 = 1000.0
for i in range(1000):
    fr = ms.FocFrame(mode=2, phase=3, rotor_mrad=1000*i, theta_mrad=1000*i,
                     iq_ma=1500.0, id_ma=-25.0, vq_mv=500, vd_mv=40,
                     spd_rpm=120, sync=1, diff_mrad=250, freq_cHz=2000,
                     ms=i, mech_mrad=100*i, is_ma=1501, is_angle_mrad=3217,
                     v_mv=502, v_angle_mrad=-1234, theta_mech_mrad=100*i, cnt=4096*i)
    h.push(fr, t0 + i * 0.005)      # 200Hz
assert h.latest.iq_ma == 1500.0
assert h.rotor_deg[-1] == 1000.0    # 1000 mrad -> deg
assert h.frames_total == 1000
# 时间裁剪：推进 25s 后旧数据应被丢弃（只留最近 20s = 4000 点）
for i in range(1000, 6000):
    fr = ms.FocFrame(mode=2, phase=3, rotor_mrad=0, theta_mrad=0, iq_ma=0, id_ma=0,
                     vq_mv=0, vd_mv=0, spd_rpm=0, sync=1, diff_mrad=0, freq_cHz=0,
                     ms=i, mech_mrad=0, is_ma=0, is_angle_mrad=0, v_mv=0,
                     v_angle_mrad=0, theta_mech_mrad=0, cnt=0)
    h.push(fr, t0 + i * 0.005)
assert len(h.t) <= 4001, len(h.t)
assert h.t[0] >= (t0 + 5999 * 0.005) - 20.0 - 0.01
print("HUB TESTS PASSED, len(t) =", len(h.t))
```

- [ ] **Step 3: 运行测试**

Run: `py -`（上面的内联脚本，工作目录 `tools\motor_scope_native`）
Expected: 打印 `HUB TESTS PASSED, len(t) = ...`，无断言失败。

- [ ] **Step 4: 提交**

```bash
git add tools/motor_scope_native/hub.py
git commit -m "feat(motor_scope_native): Hub 环形数据缓冲（Phase1 核心）"
```


---

## Task 2: `data.py` —— 数据线程（J-Link 复用 + --mock 自测源）

**Files:**
- Create: `D:\WS_L\ws_L_v.inmop_curloop\tools\motor_scope_native\data.py`
- Test: 内联验证 MockFrames 生成帧字段有效

- [ ] **Step 1: 创建 data.py**

```python
# -*- coding: utf-8 -*-
"""数据线程：J-Link RTT（复用 motor_scope 的 JLinkRttSource/RttParser）
或 --mock 开发自测源。解析出的 FocFrame 经 Qt 信号发到主线程。"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "motor_scope"))
import motor_scope as ms  # noqa: E402  (复用解析/连接逻辑)

from PySide6.QtCore import QThread, Signal  # noqa: E402


class MockFrames:
    """开发自测源：生成类似 I-F 运行期的 MOTF 帧（非用户仿真功能，仅 --mock 用）。"""

    def __init__(self, rate=200):
        self.rate = rate
        self.i = 0
        self.pp = 10

    def next(self):
        self.i += 1
        t = self.i / self.rate
        th_elec = (t * 20.0 * 360.0) % 360.0          # 20Hz 电频率
        mech = th_elec / self.pp                       # 机械角度（°）
        iq = 1500.0 + 200.0 * math.sin(2 * math.pi * 0.8 * t)
        id = 25.0 * math.sin(2 * math.pi * 4 * t)
        d2r = math.pi / 180.0
        return ms.FocFrame(
            mode=2, phase=3, sync=1,
            rotor_mrad=th_elec * d2r * 1000.0,
            theta_mrad=th_elec * d2r * 1000.0,
            iq_ma=iq, id_ma=id,
            vq_mv=500.0 + iq * 0.15, vd_mv=40.0 + id * 0.1,
            spd_rpm=120.0, diff_mrad=250.0, freq_cHz=2000,
            ms=int(t * 1000), mech_mrad=mech * d2r * 1000.0,
            is_ma=math.hypot(id, iq),
            is_angle_mrad=math.atan2(iq, id) * 1000.0,
            v_mv=math.hypot(40.0 + id * 0.1, 500.0 + iq * 0.15),
            v_angle_mrad=math.atan2(500.0 + iq * 0.15, 40.0 + id * 0.1) * 1000.0,
            theta_mech_mrad=mech * d2r * 1000.0,
            cnt=int(mech * d2r * (4096.0 / (2 * math.pi))),
        )


class DataThread(QThread):
    frame_received = Signal(object)   # motor_scope.FocFrame
    data_error = Signal(str)
    data_status = Signal(str)

    def __init__(self, mock=False, mock_rate=200, device="HC32F460",
                 speed_khz=1000, channel=0, rtt_addr=0,
                 ram_base=0x1FFF8000, ram_size=0x2F000, parent=None):
        super().__init__(parent)
        self._mock = mock
        self._mock_rate = mock_rate
        self._cfg = dict(device=device, speed_khz=speed_khz, channel=channel,
                         rtt_addr=rtt_addr, ram_base=ram_base, ram_size=ram_size)

    def run(self):
        if self._mock:
            self._run_mock()
        else:
            self._run_jlink()

    def _run_mock(self):
        self.data_status.emit("自测 mock 源（--mock）")
        gen = MockFrames(self._mock_rate)
        interval = 1.0 / self._mock_rate
        while not self.isInterruptionRequested():
            self.frame_received.emit(gen.next())
            time.sleep(interval)

    def _run_jlink(self):
        try:
            src = ms.JLinkRttSource(**self._cfg)
        except SystemExit as exc:
            self.data_error.emit(f"缺少 pylink / SEGGER J-Link 软件: {exc}")
            return
        parser = ms.RttParser(log_motf=False)
        while not self.isInterruptionRequested():
            try:
                src.open()
            except Exception as exc:
                self.data_error.emit(f"J-Link 连接失败，3 秒后重试: {exc}")
                time.sleep(3.0)
                continue
            self.data_status.emit(src.describe())
            while not self.isInterruptionRequested():
                try:
                    chunk = src.next_chunk()
                except Exception as exc:
                    self.data_error.emit(f"RTT 读取失败，准备重连: {exc}")
                    src.close()
                    break
                if not chunk:
                    time.sleep(0.003)
                    continue
                parser.feed(chunk, self.frame_received.emit)
            time.sleep(0.2)
```

- [ ] **Step 2: 内联验证 MockFrames 与 Hub 对接**

```python
# 工作目录 tools\motor_scope_native
import sys
sys.path.insert(0, r"..\motor_scope")
from hub import Hub
from data import MockFrames

h = Hub()
gen = MockFrames(200)
for _ in range(500):
    h.push(gen.next(), 1000.0 + _ * 0.005)
assert h.latest is not None and h.frames_total == 500
assert h.mech_deg[-1] > 0 and h.cnt[-1] > 0
assert h.is_ma[-1] > 0
print("MOCK+HUB OK, mech_deg=%.1f cnt=%d is_ma=%.0f"
      % (h.mech_deg[-1], h.cnt[-1], h.is_ma[-1]))
```

- [ ] **Step 3: 运行验证**

Run: `py -`（上面的内联脚本）
Expected: 打印 `MOCK+HUB OK, ...`，无异常。

- [ ] **Step 4: 提交**

```bash
git add tools/motor_scope_native/data.py
git commit -m "feat(motor_scope_native): DataThread(J-Link/--mock) + MockFrames 自测源"
```

---

## Task 3: `widgets/motor_view.py` —— 电机剖视图（QPainter）

**Files:**
- Create: `D:\WS_L\ws_L_v.inmop_curloop\tools\motor_scope_native\widgets\__init__.py`（空文件）
- Create: `D:\WS_L\ws_L_v.inmop_curloop\tools\motor_scope_native\widgets\motor_view.py`

- [ ] **Step 1: 创建 widgets/__init__.py**（空文件，使 widgets 成为包）

- [ ] **Step 2: 创建 motor_view.py**

```python
# -*- coding: utf-8 -*-
"""电机剖视图：转子/磁钢/星标、θ 指针、id/iq/is/v 矢量（QPainter 自绘，实时最新帧）。"""
import math

from PySide6.QtCore import Qt, QPointF
from PySide6.QtGui import QColor, QPainter, QPen, QPainterPath
from PySide6.QtWidgets import QWidget

POLE_PAIRS = 10          # 与 motor_config.h FOC_POLE_PAIRS 一致
DEG = math.pi / 180.0

MW, MH = 560, 560
R_SY, R_ST, R_R, R_M, R_SH = 215, 168, 158, 118, 24


class MotorView(QWidget):
    def __init__(self, hub, parent=None):
        super().__init__(parent)
        self.hub = hub
        self.setFixedSize(MW, MH)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(0, 0, MW, MH, QColor("#0c1014"))
        if self.hub.latest is None:
            p.setPen(QColor("#5b6672"))
            p.drawText(self.rect(), Qt.AlignCenter, "等待数据…")
            return
        mech = self.hub.mech_deg[-1]
        ctrl = self.hub.theta_mech_deg[-1]
        rotor_elec = self.hub.rotor_deg[-1]
        ctrl_elec = self.hub.theta_deg[-1]
        fr = self.hub.latest
        cx, cy = MW / 2, MH / 2
        p.translate(cx, cy)

        # 定子 12 槽
        p.setBrush(QColor("#333b44"))
        p.setPen(QPen(QColor("#454e59"), 2))
        p.drawEllipse(QPointF(0, 0), R_SY, R_SY)
        p.setBrush(QColor("#0c1014"))
        p.drawEllipse(QPointF(0, 0), R_ST - 2, R_ST - 2)
        for k in range(12):
            p.save()
            p.rotate(k * 30.0)
            p.setBrush(QColor("#4c5560"))
            p.setPen(QPen(QColor("#5c6672"), 1.2))
            p.drawRect(-10, R_ST, 20, R_SY - R_ST)
            p.restore()

        # 机械角刻度
        for k in range(12):
            a = k * 30.0 * DEG
            p.setPen(QPen(QColor(255, 255, 255, 56), 1))
            p.drawLine(QPointF(math.cos(a) * (R_SY + 5), math.sin(a) * (R_SY + 5)),
                       QPointF(math.cos(a) * (R_SY + 14), math.sin(a) * (R_SY + 14)))
            p.setPen(QColor(255, 255, 255, 128))
            p.drawText(QPointF(math.cos(a) * (R_SY + 28) - 8, math.sin(a) * (R_SY + 28) + 4),
                       str(k * 30))

        # 转子体 + 20 磁钢
        p.setBrush(QColor("#22272e"))
        p.setPen(QPen(QColor("#39404a"), 2))
        p.drawEllipse(QPointF(0, 0), R_R, R_R)
        pole_deg = 180.0 / POLE_PAIRS
        for k in range(2 * POLE_PAIRS):
            a0 = (mech + k * pole_deg - pole_deg / 2) * DEG
            a1 = (mech + k * pole_deg + pole_deg / 2) * DEG
            path = QPainterPath()
            path.arcMoveTo(-R_R, -R_R, 2 * R_R, 2 * R_R, a0 / DEG)
            path.arcTo(-R_R, -R_R, 2 * R_R, 2 * R_R, a0 / DEG, (a1 - a0) / DEG)
            path.arcTo(-R_M, -R_M, 2 * R_M, 2 * R_M, a1 / DEG, -(a1 - a0) / DEG)
            path.closeSubpath()
            p.setBrush(QColor("#e5484d") if k % 2 == 0 else QColor("#3b82f6"))
            p.setPen(QPen(QColor(0, 0, 0, 90), 1))
            p.drawPath(path)

        # ★ 星标
        sx = math.cos(mech * DEG) * (R_R + R_M) / 2
        sy = math.sin(mech * DEG) * (R_R + R_M) / 2
        p.setPen(QPen(QColor("#241505"), 2.5))
        p.setBrush(QColor("#fde047"))
        p.drawText(QPointF(sx - 12, sy + 10), "★")

        # d/q 轴 + 控制角指针
        p.setPen(QPen(QColor(255, 255, 255, 76), 1, Qt.DashLine))
        p.drawLine(QPointF(0, 0), QPointF(math.cos(mech * DEG) * (R_R - 4), math.sin(mech * DEG) * (R_R - 4)))
        q_ang = mech + 90.0 / POLE_PAIRS
        p.setPen(QPen(QColor(255, 255, 255, 40), 1, Qt.DashLine))
        p.drawLine(QPointF(0, 0), QPointF(math.cos(q_ang * DEG) * (R_R - 4), math.sin(q_ang * DEG) * (R_R - 4)))
        p.setPen(QPen(QColor("#ffffff"), 2, Qt.DashLine))
        p.drawLine(QPointF(0, 0), QPointF(math.cos(ctrl * DEG) * (R_R - 10), math.sin(ctrl * DEG) * (R_R - 10)))

        # 电流/电压矢量
        max_a = max(500.0, abs(fr.iq_ma), abs(fr.id_ma), fr.is_ma) * 1.1
        lmax = R_R - 34.0
        self._vec(p, QColor("#22d3ee"), abs(fr.id_ma) / max_a * lmax, mech, 4)
        self._vec(p, QColor("#fb923c"), abs(fr.iq_ma) / max_a * lmax, q_ang, 4)
        is_ang = mech + (fr.is_angle_mrad / 1000.0) / POLE_PAIRS * 180.0 / math.pi
        self._vec(p, QColor("#facc15"), fr.is_ma / max_a * lmax, is_ang, 5)
        v_ang = mech + (fr.v_angle_mrad / 1000.0) / POLE_PAIRS * 180.0 / math.pi
        self._vec(p, QColor("#e879f9"), fr.v_mv / 1000.0 * 100.0, v_ang, 3, dashed=True)

        # 底部读数
        p.resetTransform()
        p.setPen(QColor("#9aa5b1"))
        p.drawText(8, MH - 8, "θe转子=%.0f° θm机械=%.0f° θe控制=%.0f° iq=%d id=%d n=%.0frpm cnt=%d"
                   % (rotor_elec % 360, mech % 360, ctrl_elec % 360,
                      int(fr.iq_ma), int(fr.id_ma), fr.spd_rpm, fr.cnt))

    def _vec(self, p, color, length, ang_deg, width, dashed=False):
        if length < 2:
            return
        a = ang_deg * DEG
        p.setPen(QPen(color, width, Qt.DashLine if dashed else Qt.SolidLine))
        p.drawLine(QPointF(0, 0), QPointF(math.cos(a) * length, math.sin(a) * length))
```

- [ ] **Step 3: 语法检查**

Run: `py -m py_compile widgets\motor_view.py widgets\__init__.py`
Expected: 退出码 0。

- [ ] **Step 4: 提交**

```bash
git add tools/motor_scope_native/widgets/
git commit -m "feat(motor_scope_native): MotorView 电机剖视图（QPainter）"
```


---

## Task 4: `widgets/scope_view.py` —— 通用示波器（5 个实例）

**Files:**
- Create: `D:\WS_L\ws_L_v.inmop_curloop\tools\motor_scope_native\widgets\scope_view.py`

- [ ] **Step 1: 创建 scope_view.py**

```python
# -*- coding: utf-8 -*-
"""通用示波器：从 Hub 环形缓冲取最近 window_sec 秒绘制波形。
支持 kind: cur / angle / diff / mode / cnt。QPainter 自绘 + 悬停读值。"""
import math

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QWidget

PI = math.pi


class ScopeView(QWidget):
    def __init__(self, hub, kind, parent=None):
        super().__init__(parent)
        self.hub = hub
        self.kind = kind
        self.window_sec = 1.0        # Phase 1 固定 1s 窗口（滑动条属 Phase 2）
        self.hover_x = None
        self.setMinimumSize(360, 110)
        self.setMouseTracking(True)

    # ---------------- Y 轴配置 ----------------
    def _config(self):
        ml, mr, mt, mb = 52, 10, 14, 22
        pw = max(10, self.width() - ml - mr)
        ph = max(10, self.height() - mt - mb)
        if self.kind == "cur":
            max_a = 1.0
            for v in self._window_values("iq") + self._window_values("id"):
                max_a = max(max_a, abs(v))
            max_a *= 1.15
            ymid = mt + ph / 2
            return dict(ml=ml, mr=mr, mt=mt, mb=mb, pw=pw, ph=ph,
                        ticks=[-max_a, -max_a / 2, 0, max_a / 2, max_a],
                        fmt=lambda v: "%.0f" % v, unit="mA",
                        ymap=lambda v: ymid - (v / max_a) * (ph / 2 - 14),
                        traces=[("iq", "#fb923c", False), ("id", "#22d3ee", False)])
        if self.kind == "angle":
            return dict(ml=ml, mr=mr, mt=mt, mb=mb, pw=pw, ph=ph,
                        ticks=[0, 90, 180, 270, 360],
                        fmt=lambda v: "%.0f" % v, unit="deg",
                        ymap=lambda v: mt + 10 + (360 - ((v % 360) + 360) % 360) / 360 * (ph - 20),
                        traces=[("rotor_deg", "#e5484d", False),
                                ("theta_deg", "#ffffff", True),
                                ("mech_deg", "#4ade80", False)])
        if self.kind == "mode":
            return dict(ml=ml, mr=mr, mt=mt, mb=mb, pw=pw, ph=ph,
                        ticks=[0, 1, 2, 3, 4],
                        fmt=lambda v: "%.0f" % v, unit="",
                        ymap=lambda v: mt + 10 + (4 - v) / 4 * (ph - 20),
                        traces=[("mode", "#f472b6", False)])
        if self.kind == "cnt":
            vals = self._window_values("cnt")
            mn = min(vals) if vals else 0.0
            mx = max(vals) if vals else 1.0
            if mx - mn < 1:
                mx = mn + 1
            return dict(ml=ml, mr=mr, mt=mt, mb=mb, pw=pw, ph=ph,
                        ticks=[mn, (mn + mx) / 2, mx],
                        fmt=lambda v: "%.0f" % v, unit="cnt",
                        ymap=lambda v: mt + 10 + (mx - v) / (mx - mn) * (ph - 20),
                        traces=[("cnt", "#f472b6", False)])
        # diff
        ymid = mt + ph / 2
        return dict(ml=ml, mr=mr, mt=mt, mb=mb, pw=pw, ph=ph,
                    ticks=[-PI, -PI / 2, 0, PI / 2, PI],
                    fmt=lambda v: "0" if v == 0 else "%.1fπ" % (v / PI), unit="rad",
                    ymap=lambda v: ymid - (v / PI) * (ph / 2 - 12),
                    traces=[("diff_rad", "#c084fc", False)])

    def _window_values(self, key):
        dq = getattr(self.hub, key)
        if not dq:
            return []
        t1 = self.hub.last_t()
        t0 = t1 - self.window_sec
        lo, hi = 0, len(dq) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if self.hub.t[mid] < t0:
                lo = mid + 1
            else:
                hi = mid
        return list(dq)[lo:]

    def paintEvent(self, event):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor("#10151a"))
        cfg = self._config()
        ml, mr, mt, mb = cfg["ml"], cfg["mr"], cfg["mt"], cfg["mb"]
        pw, ph = cfg["pw"], cfg["ph"]
        t1 = self.hub.last_t()
        t0 = t1 - self.window_sec

        def x(t):
            return ml + (t - t0) / max(1e-9, t1 - t0) * pw

        for v in cfg["ticks"]:
            y = cfg["ymap"](v)
            p.setPen(QPen(QColor(255, 255, 255, 16), 1))
            p.drawLine(ml, y, ml + pw, y)
            p.setPen(QColor("#7c8794"))
            p.drawText(2, y + 4, cfg["fmt"](v))
        p.setPen(QColor("#7c8794"))
        for k in range(5):
            tt = t0 + (t1 - t0) * k / 4
            p.drawText(int(x(tt)) - 12, self.height() - mb + 12,
                       "0" if abs(tt - t1) < 1e-6 else "-%.2fs" % (t1 - tt))
        p.drawText(self.width() - mr - 18, self.height() - mb + 12, "t/s")

        for key, color, dashed in cfg["traces"]:
            vals = self._window_values(key)
            if len(vals) < 2:
                continue
            p.setPen(QPen(QColor(color), 1.6, Qt.DashLine if dashed else Qt.SolidLine))
            n = len(vals)
            for j in range(n - 1):
                t_a = t0 + (t1 - t0) * j / (n - 1)
                t_b = t0 + (t1 - t0) * (j + 1) / (n - 1)
                p.drawLine(int(x(t_a)), int(cfg["ymap"](vals[j])),
                           int(x(t_b)), int(cfg["ymap"](vals[j + 1])))

        if self.hover_x is not None:
            p.setPen(QPen(QColor(255, 255, 255, 128), 1))
            p.drawLine(int(self.hover_x), mt, int(self.hover_x), mt + ph)
            th = t0 + (self.hover_x - ml) / max(1, pw) * (t1 - t0)
            rows = []
            for key, color, _ in cfg["traces"]:
                vals = self._window_values(key)
                if vals and t0 <= th <= t1:
                    idx = min(len(vals) - 1, int((th - t0) / max(1e-9, t1 - t0) * (len(vals) - 1)))
                    rows.append("%s %.0f" % (key, vals[idx]))
            p.setPen(QColor("#8fa0b0"))
            p.drawText(int(self.hover_x) + 8, mt + 12, " | ".join(rows))

    def mouseMoveEvent(self, e):
        self.hover_x = e.position().x()
        self.update()
        super().mouseMoveEvent(e)

    def leaveEvent(self, e):
        self.hover_x = None
        self.update()
        super().leaveEvent(e)
```

- [ ] **Step 2: 语法检查**

Run: `py -m py_compile widgets\scope_view.py`
Expected: 退出码 0。

- [ ] **Step 3: 提交**

```bash
git add tools/motor_scope_native/widgets/scope_view.py
git commit -m "feat(motor_scope_native): ScopeView 通用示波器（cur/angle/diff/mode/cnt）"
```

---

## Task 5: `widgets/gauge.py` + `widgets/num_panel.py` —— 仪表

**Files:**
- Create: `D:\WS_L\ws_L_v.inmop_curloop\tools\motor_scope_native\widgets\gauge.py`
- Create: `D:\WS_L\ws_L_v.inmop_curloop\tools\motor_scope_native\widgets\num_panel.py`

- [ ] **Step 1: 创建 gauge.py（半圆转速表）**

```python
# -*- coding: utf-8 -*-
"""半圆转速表（QPainter），量程自适应。"""
import math

from PySide6.QtCore import QPointF
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QWidget

PI = math.pi


class Gauge(QWidget):
    def __init__(self, hub, parent=None):
        super().__init__(parent)
        self.hub = hub
        self.rpm_max = 500.0
        self.setMinimumSize(240, 150)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#0d1217"))
        fr = self.hub.latest
        rpm = abs(fr.spd_rpm) if fr else 0.0
        if fr:
            self.rpm_max = max(self.rpm_max, rpm * 1.25)
        w, h = self.width(), self.height()
        cx, cy = w / 2, h - 14
        r = min(w / 2, h) - 22
        a0, a1 = PI, 2 * PI
        frac = min(1.0, rpm / self.rpm_max)
        p.setPen(QPen(QColor("#2a313a"), 13))
        p.drawArc(int(cx - r), int(cy - r), int(2 * r), int(2 * r),
                  int(-a1 * 16), int(-(a0 - a1) * 16))
        p.setPen(QColor("#5b6672"))
        for v in range(11):
            a = a0 + (a1 - a0) * v / 10
            p.drawLine(QPointF(cx + math.cos(a) * (r - 8), cy + math.sin(a) * (r - 8)),
                       QPointF(cx + math.cos(a) * (r + 8), cy + math.sin(a) * (r + 8)))
        na = a0 + (a1 - a0) * frac
        p.setPen(QPen(QColor("#22c55e"), 13))
        p.drawArc(int(cx - r), int(cy - r), int(2 * r), int(2 * r),
                  int(-na * 16), int(-(a0 - na) * 16))
        p.setPen(QColor("#ffffff"))
        p.drawText(int(cx - 40), int(cy - 36), "%.0f" % rpm)
        p.setPen(QColor("#8fa0b0"))
        p.drawText(int(cx - 20), int(cy - 16), "rpm")
```

- [ ] **Step 2: 创建 num_panel.py（数值网格）**

```python
# -*- coding: utf-8 -*-
"""数值面板：实时显示最新帧关键量（与 web 版数值面板一致，含 cnt）。"""
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QGridLayout, QLabel, QWidget

RAD2DEG = 180.0 / 3.14159265


class NumPanel(QWidget):
    ROWS = [
        ("转速", "rpm"), ("cnt", "cnt"),
        ("转子电角", "rotor"), ("控制角", "theta"),
        ("机械角", "mech"), ("iq", "iq_ma"),
        ("id", "id_ma"), ("is", "is_ma"),
        ("vq", "vq_mv"), ("vd", "vd_mv"),
        ("|v|", "v_mv"), ("频率", "freq"),
        ("偏差 diff", "diff"),
    ]

    def __init__(self, hub, parent=None):
        super().__init__(parent)
        self.hub = hub
        self._labels = {}
        grid = QGridLayout(self)
        grid.setContentsMargins(4, 4, 4, 4)
        for i, (name, key) in enumerate(self.ROWS):
            k = QLabel(name)
            v = QLabel("0")
            k.setStyleSheet("color:#7c8794;")
            v.setStyleSheet("color:#e8eef4;font-family:Consolas,monospace;")
            v.setAlignment(Qt.AlignRight)
            grid.addWidget(k, i, 0)
            grid.addWidget(v, i, 1)
            self._labels[key] = v

    def refresh(self):
        fr = self.hub.latest
        if not fr:
            return
        vals = {
            "rpm": "%.0f" % fr.spd_rpm,
            "cnt": str(fr.cnt),
            "rotor": "%.1f°" % ((fr.rotor_mrad / 1000.0 * RAD2DEG) % 360),
            "theta": "%.1f°" % ((fr.theta_mrad / 1000.0 * RAD2DEG) % 360),
            "mech": "%.1f°" % ((fr.mech_mrad / 1000.0 * RAD2DEG) % 360),
            "iq_ma": "%.0f mA" % fr.iq_ma,
            "id_ma": "%.0f mA" % fr.id_ma,
            "is_ma": "%.0f mA" % fr.is_ma,
            "vq_mv": "%.0f mV" % fr.vq_mv,
            "vd_mv": "%.0f mV" % fr.vd_mv,
            "v_mv": "%.0f mV" % fr.v_mv,
            "freq": "%.2f Hz" % (fr.freq_cHz / 100.0),
            "diff": "%.3f rad" % (fr.diff_mrad / 1000.0),
        }
        for key, text in vals.items():
            if key in self._labels:
                self._labels[key].setText(text)
```

- [ ] **Step 3: 语法检查**

Run: `py -m py_compile widgets\gauge.py widgets\num_panel.py`
Expected: 退出码 0。

- [ ] **Step 4: 提交**

```bash
git add tools/motor_scope_native/widgets/gauge.py tools/motor_scope_native/widgets/num_panel.py
git commit -m "feat(motor_scope_native): Gauge 转速表 + NumPanel 数值面板"
```


---

## Task 6: `app.py` + `main.py` —— 主窗口组装与入口

**Files:**
- Create: `D:\WS_L\ws_L_v.inmop_curloop\tools\motor_scope_native\app.py`
- Create: `D:\WS_L\ws_L_v.inmop_curloop\tools\motor_scope_native\main.py`

- [ ] **Step 1: 创建 app.py（主窗口：布局 + QTimer + 状态栏）**

```python
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
        self.lbl_fps = QLabel("")
        self.statusBar().addWidget(self.lbl_status)
        self.statusBar().addWidget(self.lbl_state)
        self.statusBar().addWidget(self.lbl_fps)

        # 数据线程信号
        self.thread.frame_received.connect(self._on_frame)
        self.thread.data_error.connect(self._on_error)
        self.thread.data_status.connect(lambda s: self.lbl_status.setText(s))

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(self.REFRESH_MS)

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
            names = {0: "停止", 1: "开环", 2: "电流环", 3: "对齐"}
            ph = {0: "idle", 1: "hold", 2: "ramp", 3: "run", 4: "align"}
            self.lbl_state.setText("mode%d %s | phase %s | %s"
                                   % (fr.mode, names.get(fr.mode, "?"),
                                      ph.get(fr.phase, "?"),
                                      "已同步" if fr.sync else "未同步"))
            age = (time.time() - self.hub.last_frame_wall) * 1000.0
            self.lbl_fps.setText("帧龄 %.0f ms | 总帧 %d" % (age, self.hub.frames_total))
            if age > self.STALE_MS:
                self.lbl_status.setText("⚠️ 数据中断（%.0f ms 无新帧）" % age)
        else:
            self.lbl_state.setText("等待数据…")
```

- [ ] **Step 2: 创建 main.py（入口）**

```python
# -*- coding: utf-8 -*-
"""MotorScope 原生版入口：python main.py [--mock] [--device ...]"""
import argparse
import sys

from PySide6.QtWidgets import QApplication

from app import MainWindow
from data import DataThread
from hub import Hub


def main():
    ap = argparse.ArgumentParser(description="MotorScope 原生版（PySide6）")
    ap.add_argument("--mock", action="store_true",
                    help="开发自测：用本地生成的 MOTF 帧喂界面（非仿真模式）")
    ap.add_argument("--mock-rate", type=int, default=200)
    ap.add_argument("--device", default="HC32F460")
    ap.add_argument("--speed-khz", type=int, default=1000)
    ap.add_argument("--channel", type=int, default=0)
    ap.add_argument("--rtt-addr", type=lambda x: int(x, 0), default=0)
    ap.add_argument("--rtt-ram-base", type=lambda x: int(x, 0), default=0x1FFF8000)
    ap.add_argument("--rtt-ram-size", type=lambda x: int(x, 0), default=0x2F000)
    args = ap.parse_args()

    app = QApplication(sys.argv)
    hub = Hub()
    thread = DataThread(mock=args.mock, mock_rate=args.mock_rate,
                        device=args.device, speed_khz=args.speed_khz,
                        channel=args.channel, rtt_addr=args.rtt_addr,
                        ram_base=args.rtt_ram_base, ram_size=args.rtt_ram_size)
    win = MainWindow(hub, thread)
    win.resize(1400, 900)
    win.show()
    thread.start()
    rc = app.exec()
    thread.requestInterruption()
    thread.wait(2000)
    sys.exit(rc)


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: 语法检查**

Run: `py -m py_compile app.py main.py`
Expected: 退出码 0。

- [ ] **Step 4: 提交**

```bash
git add tools/motor_scope_native/app.py tools/motor_scope_native/main.py
git commit -m "feat(motor_scope_native): MainWindow 组装 + main 入口（--mock 自测）"
```

---

## Task 7: 验证 + 打包

**Files:**
- Create: `D:\WS_L\ws_L_v.inmop_curloop\tools\motor_scope_native\requirements.txt`

- [ ] **Step 1: 依赖安装**

Run: `py -m pip install pyside6 pylink-square pyinstaller`
Expected: 安装成功（pylink 仅在实机模式需要；--mock 只需 PySide6）。

- [ ] **Step 2: --mock 冒烟**

Run（工作目录 `tools\motor_scope_native`）: `py main.py --mock`
Expected: 弹出原生窗口，左侧电机动画转动、5 个示波器出波形、转速表/数值面板刷新、状态栏"自测 mock 源"。人工确认无异常/无崩溃。

- [ ] **Step 3: 实机验证（用户执行）**

Run: `py main.py --device HC32F460 --speed-khz 1000 --channel 0`
Expected: 连上 J-Link 后界面实时显示电机数据（与 web 版一致），状态栏显示运行模式。

- [ ] **Step 4: 打包（PyInstaller onedir）**

Run（工作目录 `tools\motor_scope_native`）:
`py -m PyInstaller -D --name MotorScope --paths ..\motor_scope main.py`
Expected: 生成 `dist\MotorScope\MotorScope.exe`，双击运行；实机模式需本机已装 SEGGER J-Link 软件。

- [ ] **Step 5: 提交 requirements.txt**

先创建 `requirements.txt`：
```
pyside6>=6.5
pylink-square>=0.14
pyinstaller>=6.0
```
然后：
```bash
git add tools/motor_scope_native/requirements.txt
git commit -m "chore(motor_scope_native): requirements.txt（pyside6/pylink/pyinstaller）"
```

---

## Self-Review

- **Spec 覆盖**：设计文档 4(架构)/5(布局)/6(Phase1 范围)/7(打包)/8(错误处理)/9(验证) 全部落到任务：Task1 缓冲、Task2 数据线程、Task3-5 显示、Task6 组装、Task7 验证打包。
- **无占位符**：每个任务含完整代码与验证命令。
- **类型一致性**：`Hub.push(fr, wall_t)` 接收 `motor_scope.FocFrame`；`DataThread.frame_received` 发 FocFrame；控件从 `hub` 的 deque/latest 读取；字段名（mech_deg/rotor_deg/theta_deg/theta_mech_deg/is_ma/is_ang_deg/v_mv/v_ang_deg/cnt/diff_rad）在 hub.py 与各 widget 一致。
- **复用点**：`sys.path.insert` 指向 `tools/motor_scope`，import `motor_scope`（模块级仅常量/类定义，`main()` 由 `__main__` 保护，无副作用）。
- **测试约束**：无硬件时用 `--mock` 自测；解析正确性由现有 `_verify_rtt_fields.py` 覆盖（原生版复用同一解析函数）。
- **已知取舍**：Phase 1 示波器窗口固定 1s、无暂停/回看/日志/重连（Phase 2+）；电机动画用最新帧（未做帧间外推平滑）。
