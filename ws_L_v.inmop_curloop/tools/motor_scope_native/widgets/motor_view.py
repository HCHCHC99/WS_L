# -*- coding: utf-8 -*-
"""电机剖视图：转子/磁钢/星标、θ 指针、id/iq/is/v 矢量（QPainter 自绘，实时最新帧）。"""
import math

from PySide6.QtCore import Qt, QPointF
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPainterPath
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

        # ★ 星标（深色描边 + 黄色填充，同 web 版 strokeText+fillText）
        sx = math.cos(mech * DEG) * (R_R + R_M) / 2
        sy = math.sin(mech * DEG) * (R_R + R_M) / 2
        f_star = QFont("sans-serif", 24)
        f_star.setBold(True)
        p.setFont(f_star)
        p.setPen(QPen(QColor("#241505"), 2.5))
        p.drawText(QPointF(sx - 12, sy + 10), "★")
        p.setPen(QColor("#fde047"))
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
        max_a = self.hub.cur_max * 1.1
        lmax = R_R - 34.0
        self._vec(p, QColor("#22d3ee"), fr.id_ma / max_a * lmax, mech, 4)
        self._vec(p, QColor("#fb923c"), fr.iq_ma / max_a * lmax, q_ang, 4)
        is_len = fr.is_ma / max_a * lmax
        is_ang = mech + (fr.is_angle_mrad / 1000.0) / POLE_PAIRS * 180.0 / math.pi
        self._vec(p, QColor("#facc15"), is_len, is_ang, 5)
        v_len = min(fr.v_mv / 1000.0 * 100.0, lmax)
        v_ang = mech + (fr.v_angle_mrad / 1000.0) / POLE_PAIRS * 180.0 / math.pi
        self._vec(p, QColor("#e879f9"), v_len, v_ang, 3, dashed=True)

        # 标注：is 矢量顶端 / θ 控制角
        f_lab = QFont("sans-serif", 12)
        p.setFont(f_lab)
        p.setPen(QColor("#facc15"))
        p.drawText(QPointF(math.cos(is_ang * DEG) * (is_len + 18) - 8, math.sin(is_ang * DEG) * (is_len + 18) + 4), "is")
        p.setPen(QColor("#ffffff"))
        p.drawText(QPointF(math.cos(ctrl * DEG) * (R_R - 24) - 8, math.sin(ctrl * DEG) * (R_R - 24) + 4), "θ")

        # 中心轴
        p.setBrush(QColor("#0b0d10"))
        p.setPen(QPen(QColor("#2c333c"), 2))
        p.drawEllipse(QPointF(0, 0), R_SH, R_SH)

        # 底部读数
        p.resetTransform()
        p.setPen(QColor("#9aa5b1"))
        p.drawText(8, MH - 8, "θe转子=%.0f° θm机械=%.0f° θe控制=%.0f° iq=%d id=%d n=%.0frpm cnt=%d"
                   % (rotor_elec % 360, mech % 360, ctrl_elec % 360,
                      int(fr.iq_ma), int(fr.id_ma), fr.spd_rpm, fr.cnt))

    def _vec(self, p, color, length, ang_deg, width, dashed=False):
        if abs(length) < 2:
            return
        a = ang_deg * DEG
        p.setPen(QPen(color, width, Qt.DashLine if dashed else Qt.SolidLine))
        p.drawLine(QPointF(0, 0), QPointF(math.cos(a) * length, math.sin(a) * length))