# -*- coding: utf-8 -*-
"""日志分面板：MOTF 数据帧 / 固件日志（MAIN_D 等）分开展示。
QPlainTextEdit 只读，支持鼠标框选 + Ctrl+C 复制；带搜索关键词、计数、自动滚动。
motf 面板额外有"复制（过滤后）"与"清空"按钮。米色浅色主题。"""
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (QApplication, QHBoxLayout, QLabel, QLineEdit,
                               QPlainTextEdit, QPushButton, QVBoxLayout,
                               QWidget)

TITLE = {"motf": "RTT 数据帧（MOTF）", "main": "固件日志（MAIN_D 等 · 本地记录）"}
BG = "#FFFFFF"
TEXT = "#3D3D38"
TEXT_DIM = "#8A8578"
BORDER = "#D9CDB9"
BTN = "#EFE7D6"
BTN_HOVER = "#E6DCC7"


class LogPanel(QWidget):
    def __init__(self, hub, mode="motf", parent=None):
        super().__init__(parent)
        self.hub = hub
        self.mode = mode
        self._filter = ""
        self.setMinimumHeight(130)

        v = QVBoxLayout(self)
        v.setContentsMargins(4, 4, 4, 4)
        v.setSpacing(3)

        # 标题 + 工具栏
        bar = QHBoxLayout()
        title = QLabel(TITLE[mode])
        title.setStyleSheet("font-weight:bold;color:%s;background:transparent;" % TEXT)
        bar.addWidget(title)
        bar.addStretch(1)
        self.search = QLineEdit()
        self.search.setPlaceholderText("搜索关键词（输入即过滤）")
        self.search.setMaximumWidth(220)
        self.search.textChanged.connect(self._on_search)
        self.search.setStyleSheet("background:%s;color:%s;border:1px solid %s;padding:2px 6px;"
                                  % (BG, TEXT, BORDER))
        bar.addWidget(self.search)
        self.lbl_count = QLabel("0 条")
        self.lbl_count.setStyleSheet("color:%s;background:transparent;" % TEXT_DIM)
        bar.addWidget(self.lbl_count)
        if mode == "motf":
            self.btn_copy = QPushButton("复制")
            self.btn_copy.clicked.connect(self._on_copy)
            bar.addWidget(self.btn_copy)
            self.btn_clear = QPushButton("清空")
            self.btn_clear.clicked.connect(self._on_clear)
            bar.addWidget(self.btn_clear)
        for b in (getattr(self, "btn_copy", None), getattr(self, "btn_clear", None)):
            if b is not None:
                b.setStyleSheet("QPushButton{background:%s;color:%s;border:1px solid %s;"
                                "padding:2px 10px;border-radius:3px;}"
                                "QPushButton:hover{background:%s;}"
                                % (BTN, TEXT, BORDER, BTN_HOVER))
        v.addLayout(bar)

        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.text.setFont(QFont("Consolas", 9))
        self.text.setStyleSheet("QPlainTextEdit{background:%s;color:%s;border:1px solid %s;}"
                                % (BG, TEXT, BORDER))
        v.addWidget(self.text, 1)

    # ---------------- 数据 ----------------
    def _base_lines(self):
        if self.mode == "motf":
            return [t for _, t in self.hub.logs if "MOTF," in t]
        return [t for _, t in self.hub.logs if "MOTF," not in t]

    def _filtered(self):
        f = self._filter.lower()
        base = self._base_lines()
        if not f:
            return list(base)
        return [l for l in base if f in l.lower()]

    def _on_search(self, s):
        self._filter = s
        self._render()

    def _on_copy(self):
        txt = "\n".join(self._filtered())
        QApplication.clipboard().setText(txt)
        old = self.btn_copy.text()
        self.btn_copy.setText("已复制")
        from PySide6.QtCore import QTimer
        QTimer.singleShot(1200, lambda: self.btn_copy.setText(old))

    def _on_clear(self):
        self.hub.clear_logs()
        self._render()

    def _render(self):
        lines = self._filtered()
        if self.mode == "motf":
            total = len(self._base_lines())
            self.lbl_count.setText("%d / %d 条" % (len(lines), total))
        else:
            self.lbl_count.setText("%d 条" % len(lines))
        if not lines:
            self.text.setPlainText("等待数据…")
            return
        sb = self.text.verticalScrollBar()
        at_bottom = sb.value() >= sb.maximum() - 8
        self.text.setPlainText("\n".join(lines))
        if at_bottom:
            sb.setValue(sb.maximum())

    def sync(self):
        """由主窗口定时器调用：hub 有新增日志时刷新（仅刷新时全量重绘，频率低开销可接受）。"""
        if self.hub.logs_dirty:
            self._render()