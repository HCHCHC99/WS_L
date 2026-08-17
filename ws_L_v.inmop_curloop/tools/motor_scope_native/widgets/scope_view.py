# -*- coding: utf-8 -*-
"""通用示波器：从 Hub 环形缓冲取最近 window_sec 秒绘制波形。
支持 kind: cur / angle / diff / mode / cnt。QPainter 自绘 + 悬停读值。
米色浅色主题配色（曲线色为浅底加深版）。"""
import bisect
import math

from PySide6.QtCore import Qt, QRect
from PySide6.QtGui import QColor, QFont, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QWidget

PI = math.pi

# 米色主题
BG = "#FDFBF7"                # 示波器面板底（奶油白）
TEXT = "#6E685C"              # 刻度/标签
TEXT_MAIN = "#3D3D38"          # 标题（深暖灰）
TEXT_DIM = "#8A8578"          # 等待数据等
GRID = QColor(90, 82, 66, 36)   # 网格线（浅灰棕）
HOVER_LINE = QColor(90, 82, 66, 110)

# 曲线色（浅底加深版）
C_IQ = "#D97B1E"     # 橙
C_ID = "#0E7C9E"     # 青
C_ROTOR = "#C53030"  # 红
C_THETA = "#33332E"  # 近黑（原白）
C_MECH = "#2F855A"   # 绿
C_MODE = "#C0267B"   # 粉
C_CNT = "#C0267B"    # 粉
C_DIFF = "#805AD5"   # 紫


class ScopeView(QWidget):
    def __init__(self, hub, kind, parent=None):
        super().__init__(parent)
        self.hub = hub
        self.kind = kind
        self.window_sec = 1.0        # Phase 1 固定 1s 窗口（滑动条属 Phase 2）
        self.hover_x = None
        self.setMinimumSize(360, 110)
        self.setMouseTracking(True)

    # 窗口切片：每次 paint 只算一次 lo，返回真实时间戳与数值（不再均匀铺点）。
    # 导航：实时=最右端；暂停/回看=view_end；t0 按窗口宽度回退并钳制到数据左端。
    def _window(self):
        tq = self.hub.t
        if not tq:
            return None
        minT, maxT = self.hub.t[0], self.hub.t[-1]
        t1 = maxT if self.hub.follow_live else min(self.hub.view_end, maxT)
        t0 = t1 - self.hub.window_sec
        if t0 < minT:                      # 数据左端不足：右移窗口（与 web scopeWindow 一致）
            t0 = minT
            t1 = t0 + self.hub.window_sec
            if t1 > maxT:
                t1 = maxT
        lo, hi = 0, len(tq) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if tq[mid] < t0:
                lo = mid + 1
            else:
                hi = mid
        return lo, list(tq)[lo:], t0, t1

    def _values(self, key, lo):
        return list(getattr(self.hub, key))[lo:]

    # ---------------- Y 轴配置 ----------------
    def _config(self, lo):
        ml, mr, mt, mb = 62, 10, 30, 30
        pw = max(10, self.width() - ml - mr)
        ph = max(10, self.height() - mt - mb)
        if self.kind == "cur":
            max_a = 1.0
            for key in ("iq", "id"):
                for v in self._values(key, lo):
                    max_a = max(max_a, abs(v))
            max_a *= 1.15
            ymid = mt + ph / 2
            return dict(ml=ml, mr=mr, mt=mt, mb=mb, pw=pw, ph=ph,
                        ticks=[-max_a, -max_a / 2, 0, max_a / 2, max_a],
                        fmt=lambda v: "%.0f" % v, unit="mA",
                        ymap=lambda v: ymid - (v / max_a) * (ph / 2 - 14),
                        title="iq / id（mA）",
                        traces=[("iq", C_IQ, False), ("id", C_ID, False)])
        if self.kind == "angle":
            return dict(ml=ml, mr=mr, mt=mt, mb=mb, pw=pw, ph=ph,
                        ticks=[0, 90, 180, 270, 360],
                        fmt=lambda v: "%.0f" % v, unit="deg",
                        ymap=lambda v: mt + 10 + (360 - ((v % 360) + 360) % 360) / 360 * (ph - 20),
                        title="转子角 vs 控制角（°）",
                        traces=[("rotor_deg", C_ROTOR, False),
                                ("theta_deg", C_THETA, True),
                                ("mech_deg", C_MECH, False)])
        if self.kind == "mode":
            return dict(ml=ml, mr=mr, mt=mt, mb=mb, pw=pw, ph=ph,
                        ticks=[0, 1, 2, 3, 4],
                        fmt=lambda v: "%.0f" % v, unit="",
                        ymap=lambda v: mt + 10 + (4 - v) / 4 * (ph - 20),
                        title="模式 comm_mode（0停/1开环/2电流环/3对齐）",
                        traces=[("mode", C_MODE, False)])
        if self.kind == "cnt":
            vals = self._values("cnt", lo)
            mn = min(vals) if vals else 0.0
            mx = max(vals) if vals else 1.0
            if mx - mn < 1:
                mx = mn + 1
            return dict(ml=ml, mr=mr, mt=mt, mb=mb, pw=pw, ph=ph,
                        ticks=[mn, (mn + mx) / 2, mx],
                        fmt=lambda v: "%.0f" % v, unit="cnt",
                        ymap=lambda v: mt + 10 + (mx - v) / (mx - mn) * (ph - 20),
                        title="ABZ 编码器计数 cnt",
                        traces=[("cnt", C_CNT, False)])
        ymid = mt + ph / 2
        return dict(ml=ml, mr=mr, mt=mt, mb=mb, pw=pw, ph=ph,
                    ticks=[-PI, -PI / 2, 0, PI / 2, PI],
                    fmt=lambda v: "0" if v == 0 else "%.1fπ" % (v / PI), unit="rad",
                    ymap=lambda v: ymid - (v / PI) * (ph / 2 - 12),
                    title="角度偏差 diff（rad）",
                    traces=[("diff_rad", C_DIFF, False)])

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(BG))
        ml, mr, mt, mb = 62, 10, 30, 30
        pw = max(10, self.width() - ml - mr)
        ph = max(10, self.height() - mt - mb)
        w = self._window()
        if w is None:
            p.setPen(QColor(TEXT_DIM))
            p.drawText(self.rect(), Qt.AlignCenter, "等待数据…")
            return
        lo, ts, t0, t1 = w
        cfg = self._config(lo)

        # ---- 标题（顶部）----
        p.setFont(QFont("sans-serif", 10, QFont.Bold))
        p.setPen(QColor(TEXT_MAIN))
        p.drawText(4, mt - 12, cfg["title"])

        def x(t):
            return ml + (t - t0) / max(1e-9, t1 - t0) * pw

        # ---- Y 轴：网格 + 刻度值 + 单位（旋转 90°）----
        p.setFont(QFont("sans-serif", 9))
        for v in cfg["ticks"]:
            y = cfg["ymap"](v)
            p.setPen(QPen(GRID, 1))
            p.drawLine(ml, y, ml + pw, y)
            p.setPen(QColor(TEXT))
            p.drawText(QRect(20, y - 8, ml - 26, 16), Qt.AlignRight | Qt.AlignVCenter,
                       cfg["fmt"](v))
        if cfg["unit"]:
            p.save()
            p.translate(10, mt + ph / 2)
            p.rotate(-90)
            p.setPen(QColor(TEXT))
            p.drawText(-60, -8, 120, 16, Qt.AlignCenter, cfg["unit"])
            p.restore()

        # ---- X 轴：时间刻度 + t/s 标签（底部）----
        p.setPen(QColor(TEXT))
        for k in range(5):
            tt = t0 + (t1 - t0) * k / 4
            p.drawText(int(x(tt)) - 12, self.height() - mb + 10,
                       "0" if abs(tt - t1) < 1e-6 else "-%.2fs" % (t1 - tt))
        p.drawText(int(ml + pw / 2 - 12), self.height() - mb + 26, "t/s")

        # 波形：用真实时间戳定位（缺帧自然留白），按像素宽度抽稀
        for key, color, dashed in cfg["traces"]:
            vals = self._values(key, lo)
            if len(vals) < 2:
                continue
            path = QPainterPath()
            step = max(1, len(vals) // max(1, pw))
            for j in range(0, len(vals), step):
                px = int(x(ts[j]))
                py = int(cfg["ymap"](vals[j]))
                if j == 0:
                    path.moveTo(px, py)
                else:
                    path.lineTo(px, py)
            p.setPen(QPen(QColor(color), 1.6, Qt.DashLine if dashed else Qt.SolidLine))
            p.drawPath(path)

        if self.hover_x is not None:
            hx = min(max(self.hover_x, ml), ml + pw)     # clamp
            p.setPen(QPen(HOVER_LINE, 1))
            p.drawLine(int(hx), mt, int(hx), mt + ph)
            th = t0 + (hx - ml) / max(1, pw) * (t1 - t0)
            i = bisect.bisect_left(ts, th)
            if i >= len(ts):
                i = len(ts) - 1
            rows = []
            for key, color, _ in cfg["traces"]:
                vals = self._values(key, lo)
                if 0 <= i < len(vals):
                    rows.append("%s %.0f" % (key, vals[i]))
            p.setPen(QColor(TEXT))
            p.drawText(int(hx) + 8, mt + 12, " | ".join(rows))

    def mouseMoveEvent(self, e):
        self.hover_x = e.position().x()
        self.update()
        super().mouseMoveEvent(e)

    def leaveEvent(self, e):
        self.hover_x = None
        self.update()
        super().leaveEvent(e)