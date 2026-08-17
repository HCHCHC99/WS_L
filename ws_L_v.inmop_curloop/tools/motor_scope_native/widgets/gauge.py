# -*- coding: utf-8 -*-
"""半圆转速表（QPainter），量程自适应。"""
import math

from PySide6.QtCore import Qt, QPointF
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QWidget

PI = math.pi
R16 = 180.0 * 16.0 / PI        # rad -> Qt drawArc 单位（1/16 度）
LBW = 32                        # 刻度数值标签文本框宽


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
                  int(-a1 * R16), int(-(a0 - a1) * R16))
        p.setPen(QColor("#5b6672"))
        p.setFont(QFont("Consolas", 9))
        for v in range(11):
            a = a0 + (a1 - a0) * v / 10
            p.drawLine(QPointF(cx + math.cos(a) * (r - 8), cy + math.sin(a) * (r - 8)),
                       QPointF(cx + math.cos(a) * (r + 8), cy + math.sin(a) * (r + 8)))
            bx = int(cx + math.cos(a) * (r + 20)) - LBW // 2
            bx = max(0, min(bx, w - LBW))          # 两端标签钳制，避免裁切
            by = int(cy + math.sin(a) * (r + 20)) - 7
            p.drawText(bx, by, LBW, 14, Qt.AlignCenter,
                       "%d" % int(self.rpm_max * v / 10))
        na = a0 + (a1 - a0) * frac
        p.setPen(QPen(QColor("#22c55e"), 13))
        p.drawArc(int(cx - r), int(cy - r), int(2 * r), int(2 * r),
                  int(-na * R16), int(-(a0 - na) * R16))
        p.setPen(QColor("#ffffff"))
        p.drawText(int(cx - 40), int(cy - 36), "%.0f" % rpm)
        p.setPen(QColor("#8fa0b0"))
        p.drawText(int(cx - 20), int(cy - 16), "rpm")