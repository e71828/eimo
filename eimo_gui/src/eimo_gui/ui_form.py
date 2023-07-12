# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'form.ui'
##
## Created by: Qt User Interface Compiler version 6.2.4
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QGridLayout, QGroupBox, QHBoxLayout,
    QLabel, QMainWindow, QMenuBar, QPushButton,
    QScrollArea, QSizePolicy, QSpacerItem, QStatusBar,
    QToolBox, QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1026, 800)
        MainWindow.setMinimumSize(QSize(1026, 800))
        MainWindow.setMaximumSize(QSize(1028, 844))
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.centralwidget.setMinimumSize(QSize(1026, 800))
        self.centralwidget.setMaximumSize(QSize(16777215, 800))
        self.horizontalLayout_2 = QHBoxLayout(self.centralwidget)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.groupBox_video = QGroupBox(self.centralwidget)
        self.groupBox_video.setObjectName(u"groupBox_video")
        self.groupBox_video.setMinimumSize(QSize(640, 720))
        self.groupBox_video.setMaximumSize(QSize(1280, 720))
        self.verticalLayout_2 = QVBoxLayout(self.groupBox_video)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.QScrollArea_1 = QScrollArea(self.groupBox_video)
        self.QScrollArea_1.setObjectName(u"QScrollArea_1")
        self.QScrollArea_1.setWidgetResizable(True)
        self.scrollAreaWidgetContents = QWidget()
        self.scrollAreaWidgetContents.setObjectName(u"scrollAreaWidgetContents")
        self.scrollAreaWidgetContents.setGeometry(QRect(0, 0, 616, 333))
        self.QScrollArea_1.setWidget(self.scrollAreaWidgetContents)

        self.verticalLayout_2.addWidget(self.QScrollArea_1)

        self.QScrollArea_2 = QScrollArea(self.groupBox_video)
        self.QScrollArea_2.setObjectName(u"QScrollArea_2")
        self.QScrollArea_2.setWidgetResizable(True)
        self.scrollAreaWidgetContents_2 = QWidget()
        self.scrollAreaWidgetContents_2.setObjectName(u"scrollAreaWidgetContents_2")
        self.scrollAreaWidgetContents_2.setGeometry(QRect(0, 0, 616, 333))
        self.QScrollArea_2.setWidget(self.scrollAreaWidgetContents_2)

        self.verticalLayout_2.addWidget(self.QScrollArea_2)


        self.horizontalLayout_2.addWidget(self.groupBox_video)

        self.groupBox_control = QGroupBox(self.centralwidget)
        self.groupBox_control.setObjectName(u"groupBox_control")
        self.groupBox_control.setMinimumSize(QSize(256, 720))
        self.groupBox_control.setMaximumSize(QSize(360, 720))
        self.groupBox_control.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.verticalLayout_3 = QVBoxLayout(self.groupBox_control)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalSpacer_2 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Fixed)

        self.verticalLayout_3.addItem(self.verticalSpacer_2)

        self.gridLayout = QGridLayout()
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setVerticalSpacing(20)
        self.change_depth = QLabel(self.groupBox_control)
        self.change_depth.setObjectName(u"change_depth")

        self.gridLayout.addWidget(self.change_depth, 1, 2, 1, 1)

        self.init_angle = QLabel(self.groupBox_control)
        self.init_angle.setObjectName(u"init_angle")

        self.gridLayout.addWidget(self.init_angle, 3, 1, 1, 1)

        self.label_feedback = QLabel(self.groupBox_control)
        self.label_feedback.setObjectName(u"label_feedback")

        self.gridLayout.addWidget(self.label_feedback, 2, 0, 1, 1)

        self.label_depth = QLabel(self.groupBox_control)
        self.label_depth.setObjectName(u"label_depth")

        self.gridLayout.addWidget(self.label_depth, 0, 2, 1, 1)

        self.value_depth = QLabel(self.groupBox_control)
        self.value_depth.setObjectName(u"value_depth")

        self.gridLayout.addWidget(self.value_depth, 2, 2, 1, 1)

        self.init_depth = QLabel(self.groupBox_control)
        self.init_depth.setObjectName(u"init_depth")

        self.gridLayout.addWidget(self.init_depth, 3, 2, 1, 1)

        self.label_control = QLabel(self.groupBox_control)
        self.label_control.setObjectName(u"label_control")

        self.gridLayout.addWidget(self.label_control, 1, 0, 1, 1)

        self.value_angle = QLabel(self.groupBox_control)
        self.value_angle.setObjectName(u"value_angle")

        self.gridLayout.addWidget(self.value_angle, 2, 1, 1, 1)

        self.lablel_header = QLabel(self.groupBox_control)
        self.lablel_header.setObjectName(u"lablel_header")

        self.gridLayout.addWidget(self.lablel_header, 0, 0, 1, 1)

        self.change_angle = QLabel(self.groupBox_control)
        self.change_angle.setObjectName(u"change_angle")

        self.gridLayout.addWidget(self.change_angle, 1, 1, 1, 1)

        self.label_init = QLabel(self.groupBox_control)
        self.label_init.setObjectName(u"label_init")

        self.gridLayout.addWidget(self.label_init, 3, 0, 1, 1)

        self.label_angle = QLabel(self.groupBox_control)
        self.label_angle.setObjectName(u"label_angle")

        self.gridLayout.addWidget(self.label_angle, 0, 1, 1, 1)


        self.verticalLayout_3.addLayout(self.gridLayout)

        self.verticalSpacer_3 = QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Fixed)

        self.verticalLayout_3.addItem(self.verticalSpacer_3)

        self.pB_Get_ROS = QPushButton(self.groupBox_control)
        self.pB_Get_ROS.setObjectName(u"pB_Get_ROS")
        self.pB_Get_ROS.setEnabled(True)

        self.verticalLayout_3.addWidget(self.pB_Get_ROS)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_3.addItem(self.verticalSpacer)

        self.toolBox = QToolBox(self.groupBox_control)
        self.toolBox.setObjectName(u"toolBox")
        self.toolBox.setMinimumSize(QSize(256, 360))
        self.toolBox.setMaximumSize(QSize(360, 360))
        self.video_control = QWidget()
        self.video_control.setObjectName(u"video_control")
        self.video_control.setGeometry(QRect(0, 0, 336, 298))
        self.verticalLayout = QVBoxLayout(self.video_control)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.gridLayout_2 = QGridLayout()
        self.gridLayout_2.setSpacing(30)
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.pB_rs_cam1 = QPushButton(self.video_control)
        self.pB_rs_cam1.setObjectName(u"pB_rs_cam1")

        self.gridLayout_2.addWidget(self.pB_rs_cam1, 0, 2, 1, 1)

        self.pB_start_cam = QPushButton(self.video_control)
        self.pB_start_cam.setObjectName(u"pB_start_cam")

        self.gridLayout_2.addWidget(self.pB_start_cam, 0, 0, 1, 1)

        self.pB_stop_cam = QPushButton(self.video_control)
        self.pB_stop_cam.setObjectName(u"pB_stop_cam")

        self.gridLayout_2.addWidget(self.pB_stop_cam, 1, 0, 1, 1)

        self.pB_rs_cam2 = QPushButton(self.video_control)
        self.pB_rs_cam2.setObjectName(u"pB_rs_cam2")

        self.gridLayout_2.addWidget(self.pB_rs_cam2, 1, 2, 1, 1)


        self.verticalLayout.addLayout(self.gridLayout_2)

        self.toolBox.addItem(self.video_control, u"Video Setting")
        self.video_capture = QWidget()
        self.video_capture.setObjectName(u"video_capture")
        self.video_capture.setGeometry(QRect(0, 0, 336, 298))
        self.verticalLayout_4 = QVBoxLayout(self.video_capture)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.gridLayout_3 = QGridLayout()
        self.gridLayout_3.setSpacing(30)
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.pB_Get_video1 = QPushButton(self.video_capture)
        self.pB_Get_video1.setObjectName(u"pB_Get_video1")

        self.gridLayout_3.addWidget(self.pB_Get_video1, 0, 1, 1, 1)

        self.pB_Get_video2 = QPushButton(self.video_capture)
        self.pB_Get_video2.setObjectName(u"pB_Get_video2")

        self.gridLayout_3.addWidget(self.pB_Get_video2, 1, 1, 1, 1)

        self.pB_Get_pic1 = QPushButton(self.video_capture)
        self.pB_Get_pic1.setObjectName(u"pB_Get_pic1")

        self.gridLayout_3.addWidget(self.pB_Get_pic1, 0, 0, 1, 1)

        self.pB_Get_pic2 = QPushButton(self.video_capture)
        self.pB_Get_pic2.setObjectName(u"pB_Get_pic2")

        self.gridLayout_3.addWidget(self.pB_Get_pic2, 1, 0, 1, 1)


        self.verticalLayout_4.addLayout(self.gridLayout_3)

        self.toolBox.addItem(self.video_capture, u"Vdieo Capture")

        self.verticalLayout_3.addWidget(self.toolBox)


        self.horizontalLayout_2.addWidget(self.groupBox_control)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1026, 22))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        self.toolBox.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.groupBox_video.setTitle(QCoreApplication.translate("MainWindow", u"Video", None))
        self.groupBox_control.setTitle(QCoreApplication.translate("MainWindow", u"Motion", None))
        self.change_depth.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.init_angle.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.label_feedback.setText(QCoreApplication.translate("MainWindow", u"Feedback", None))
        self.label_depth.setText(QCoreApplication.translate("MainWindow", u"depth/mm", None))
        self.value_depth.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.init_depth.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.label_control.setText(QCoreApplication.translate("MainWindow", u"Control", None))
        self.value_angle.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.lablel_header.setText(QCoreApplication.translate("MainWindow", u"PARAM", None))
        self.change_angle.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.label_init.setText(QCoreApplication.translate("MainWindow", u"Init", None))
        self.label_angle.setText(QCoreApplication.translate("MainWindow", u"Angle/Deg", None))
        self.pB_Get_ROS.setText(QCoreApplication.translate("MainWindow", u"Get Data From ROS", None))
        self.pB_rs_cam1.setText(QCoreApplication.translate("MainWindow", u"Restart Camera 1", None))
        self.pB_start_cam.setText(QCoreApplication.translate("MainWindow", u"Start all Camera", None))
        self.pB_stop_cam.setText(QCoreApplication.translate("MainWindow", u"Stop all Camera", None))
        self.pB_rs_cam2.setText(QCoreApplication.translate("MainWindow", u"Restart Camera 2", None))
        self.toolBox.setItemText(self.toolBox.indexOf(self.video_control), QCoreApplication.translate("MainWindow", u"Video Setting", None))
        self.pB_Get_video1.setText(QCoreApplication.translate("MainWindow", u"PushButton", None))
        self.pB_Get_video2.setText(QCoreApplication.translate("MainWindow", u"PushButton", None))
        self.pB_Get_pic1.setText(QCoreApplication.translate("MainWindow", u"Get Picure", None))
        self.pB_Get_pic2.setText(QCoreApplication.translate("MainWindow", u"Get Picure", None))
        self.toolBox.setItemText(self.toolBox.indexOf(self.video_capture), QCoreApplication.translate("MainWindow", u"Vdieo Capture", None))
    # retranslateUi

