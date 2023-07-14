#!/usr/bin/env python3
import rospy
from PySide6.QtGui import QPainter, Qt
from PySide6.QtWidgets import QProgressBar

class Battery(QProgressBar):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setTextVisible(False)
        self.setOrientation(Qt.Horizontal)
        self.setRange(-1, 100)
        self.setValue(24)
        self.setStyleSheet('''
        QProgressBar {
            border: 4px solid white;
            background-color: #0070db;
            margin-top: 12px;
        }
        QProgressBar:horizontal {
            height: 60px;
            width: 120px;
            margin-right: 12px;
        }
        QProgressBar:vertical {
            height: 120px;
            width: 60px;
            margin-left: 12px;
        }
        QProgressBar::chunk {
            background-color: white;
            margin: 4px;
        }''')

    def paintEvent(self, event):
        super().paintEvent(event)
        qp = QPainter(self)
        qp.setPen(Qt.NoPen)
        qp.setBrush(Qt.white)
        w, h = self.width(), self.height()
        if self.orientation() == Qt.Horizontal:
            qp.drawRect(w, 12 + h / 4, -12, h / 2 - 12)
            dx, dy = 0, 12
        else:
            qp.drawRect(12 + w / 4, 0, w / 2 - 12, 12)
            dx, dy = 12, 0

        qp.setPen(self.palette().text().color())
        qp.drawText(self.rect().adjusted(dx, dy, 0, 0), Qt.AlignCenter, self.text())
        qp.setPen(Qt.NoPen)
