# -*- coding: utf-8 -*-
"""电机剖视图：转子/磁钢/星标、θ 指针、id/iq/is/v 矢量（QPainter 自绘，实时最新帧）。
米色浅色主题配色。"""
import math

from PySide6.QtCore import Qt, QPointF
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPainterPath
from PySide6.QtWidgets import QWidget

from hub import MRAD2DEG

POLE_PAIRS = 10          # 与 motor_config.h FOC_POLE_PAIRS 一致
DEG = math.pi / 180.0

MW, MH = 560, 560
R_SY, R_ST, R_R, R_M, R_SH = 215, 168, 158, 118, 24

# 米色主题
BG = "#F6F1E7"
TEXT_MAIN = "#4A4A43"
TEXT_DIM = "#8A8578"
STATOR_OUT = "#E3DAC8"
STATOR_IN = "#FBF7EE"
TEETH = "#DCD2BD"
TEETH_PEN = "#C9BCA2"
ROTOR = "#E8E0CF"
ROTOR_PEN = "#B8AA8E"
MAG_RED = "#C53030"
MAG_BLUE = "#2B6CB0"
AXIS_DIM = QColor(90, 82, 66, 110)
AXIS_FAINT = QColor(90, 82, 66, 55)
STAR_OUTLINE = "#7C5A10"
STAR_FILL = "#F6C445"
CTRL_LINE = "#33332E"
HUB = "#BFAF93"
HUB_PEN = "#8A7B61"
VEC_ID = "#0E7C9E"
VEC_IQ = "#E07A1F"
VEC_IS = "#B8860B"
VEC_V = "#9D2FA8"


class MotorView(QWidget):
    def __init__(self, hub, parent=None):
        super().__init__(parent)
        self.hub = hub
        self.setFixedSize(MW, MH)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(0, 0, MW, MH, QColor(BG))
        fr = self.hub.view_frame or self.hub.latest   # 回看历史时用插值帧
        if fr is None:
            p.setPen(QColor(TEXT_DIM))
            p.drawText(self.rect(), Qt.AlignCenter, "等待数据…")
            return
        mech = fr.mech_mrad * MRAD2DEG
        ctrl = fr.theta_mech_mrad * MRAD2DEG
        rotor_elec = fr.rotor_mrad * MRAD2DEG
        ctrl_elec = fr.theta_mrad * MRAD2DEG
        cx, cy = MW / 2, MH / 2
        p.translate(cx, cy)

        # 定子 12 槽
        p.setBrush(QColor(STATOR_OUT))
        p.setPen(QPen(QColor(TEETH_PEN), 2))
        p.drawEllipse(QPointF(0, 0), R_SY, R_SY)
        p.setBrush(QColor(STATOR_IN))
        p.drawEllipse(QPointF(0, 0), R_ST - 2, R_ST - 2)
        for k in range(12):
            p.save()
            p.rotate(k * 30.0)
            p.setBrush(QColor(TEETH))
            p.setPen(QPen(QColor(TEETH_PEN), 1.2))
            p.drawRect(-10, R_ST, 20, R_SY - R_ST)
            p.restore()

        # 机械角刻度（浅底用深灰刻度线）
        for k in range(12):
            a = k * 30.0 * DEG
            p.setPen(QPen(AXIS_DIM, 1))
            p.drawLine(QPointF(math.cos(a) * (R_SY + 5), math.sin(a) * (R_SY + 5)),
                       QPointF(math.cos(a) * (R_SY + 14), math.sin(a) * (R_SY + 14)))
            p.setPen(QColor(TEXT_DIM))
            p.drawText(QPointF(math.cos(a) * (R_SY + 28) - 8, math.sin(a) * (R_SY + 28) + 4),
                       str(k * 30))

        # 转子体 + 20 磁钢
        p.setBrush(QColor(ROTOR))
        p.setPen(QPen(QColor(ROTOR_PEN), 2))
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
            p.setBrush(QColor(MAG_RED) if k % 2 == 0 else QColor(MAG_BLUE))
            p.setPen(QPen(QColor(0, 0, 0, 60), 1))
            p.drawPath(path)

        # ★ 星标（深色描边 + 金黄填充，同 web 版 strokeText+fillText）
        sx = math.cos(mech * DEG) * (R_R + R_M) / 2
        sy = math.sin(mech * DEG) * (R_R + R_M) / 2
        f_star = QFont("sans-serif", 24)
        f_star.setBold(True)
        p.setFont(f_star)
        p.setPen(QPen(QColor(STAR_OUTLINE), 2.5))
        p.drawText(QPointF(sx - 12, sy + 10), "★")
        p.setPen(QColor(STAR_FILL))
        p.drawText(QPointF(sx - 12, sy + 10), "★")

        # d/q 轴 + 控制角指针
        p.setPen(QPen(AXIS_DIM, 1, Qt.DashLine))
        p.drawLine(QPointF(0, 0), QPointF(math.cos(mech * DEG) * (R_R - 4), math.sin(mech * DEG) * (R_R - 4)))
        q_ang = mech + 90.0 / POLE_PAIRS
        p.setPen(QPen(AXIS_FAINT, 1, Qt.DashLine))
        p.drawLine(QPointF(0, 0), QPointF(math.cos(q_ang * DEG) * (R_R - 4), math.sin(q_ang * DEG) * (R_R - 4)))
        p.setPen(QPen(QColor(CTRL_LINE), 2, Qt.DashLine))
        p.drawLine(QPointF(0, 0), QPointF(math.cos(ctrl * DEG) * (R_R - 10), math.sin(ctrl * DEG) * (R_R - 10)))

        # 电流/电压矢量
        max_a = self.hub.cur_max * 1.1
        lmax = R_R - 34.0
        self._vec(p, QColor(VEC_ID), fr.id_ma / max_a * lmax, mech, 4)
        self._vec(p, QColor(VEC_IQ), fr.iq_ma / max_a * lmax, q_ang, 4)
        is_len = fr.is_ma / max_a * lmax
        is_ang = mech + (fr.is_angle_mrad / 1000.0) / POLE_PAIRS * 180.0 / math.pi
        self._vec(p, QColor(VEC_IS), is_len, is_ang, 5)
        v_len = min(fr.v_mv / 1000.0 * 100.0, lmax)
        v_ang = mech + (fr.v_angle_mrad / 1000.0) / POLE_PAIRS * 180.0 / math.pi
        self._vec(p, QColor(VEC_V), v_len, v_ang, 3, dashed=True)

        # 标注：is 矢量顶端 / θ 控制角
        f_lab = QFont("sans-serif", 12)
        p.setFont(f_lab)
        p.setPen(QColor(VEC_IS))
        p.drawText(QPointF(math.cos(is_ang * DEG) * (is_len + 18) - 8, math.sin(is_ang * DEG) * (is_len + 18) + 4), "is")
        p.setPen(QColor(CTRL_LINE))
        p.drawText(QPointF(math.cos(ctrl * DEG) * (R_R - 24) - 8, math.sin(ctrl * DEG) * (R_R - 24) + 4), "θ")

        # 中心轴
        p.setBrush(QColor(HUB))
        p.setPen(QPen(QColor(HUB_PEN), 2))
        p.drawEllipse(QPointF(0, 0), R_SH, R_SH)

        # 底部读数
        p.resetTransform()
        p.setPen(QColor(TEXT_MAIN))
        p.drawText(8, MH - 8, "θe转子=%.0f° θm机械=%.0f° θe控制=%.0f° iq=%d id=%d n=%.0frpm cnt=%d"
                   % (rotor_elec % 360, mech % 360, ctrl_elec % 360,
                      int(fr.iq_ma), int(fr.id_ma), fr.spd_rpm, fr.cnt))

    def _vec(self, p, color, length, ang_deg, width, dashed=False):
        if abs(length) < 2:
            return
        a = ang_deg * DEG
        p.setPen(QPen(color, width, Qt.DashLine if dashed else Qt.SolidLine))
        p.drawLine(QPointF(0, 0), QPointF(math.cos(a) * length, math.sin(a) * length))