# -*- coding: utf-8 -*-
"""数值面板：实时显示最新帧关键量（与 web 版数值面板一致，含 cnt）。米色浅色主题。"""
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QGridLayout, QLabel, QWidget

from hub import MRAD2DEG

# 米色主题
TEXT_LABEL = "#6E685C"
TEXT_VALUE = "#3D3D38"


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
        self._last = {}
        grid = QGridLayout(self)
        grid.setContentsMargins(4, 4, 4, 4)
        for i, (name, key) in enumerate(self.ROWS):
            k = QLabel(name)
            v = QLabel("0")
            k.setStyleSheet(f"color:{TEXT_LABEL};background:transparent;")
            v.setStyleSheet(f"color:{TEXT_VALUE};background:transparent;font-family:Consolas,monospace;")
            v.setAlignment(Qt.AlignRight)
            grid.addWidget(k, i, 0)
            grid.addWidget(v, i, 1)
            self._labels[key] = v

    def refresh(self):
        fr = self.hub.latest
        if not fr:
            return
        vals = {
            "rpm": "%.0f rpm" % fr.spd_rpm,
            "cnt": str(fr.cnt),
            "rotor": "%.1f°" % ((fr.rotor_mrad * MRAD2DEG) % 360),
            "theta": "%.1f°" % ((fr.theta_mrad * MRAD2DEG) % 360),
            "mech": "%.1f°" % ((fr.mech_mrad * MRAD2DEG) % 360),
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
            if text != self._last.get(key):
                self._labels[key].setText(text)
                self._last[key] = text