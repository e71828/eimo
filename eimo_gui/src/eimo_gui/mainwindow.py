#!/bin/env python3
import sys
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, \
    QLabel, QGridLayout, QScrollArea, QSizePolicy
from PySide6.QtGui import QPixmap, QIcon, QImage, QPalette
from PySide6.QtCore import QThread, Signal, Slot, Qt, QEvent, QObject
from .ui_form import Ui_MainWindow
from .stream import CaptureIpCameraFramesWorker
from .robot import UpdateDataWorker, check_ROS_master

# Important:
# You need to run the following command to generate the ui_form.py file
#     pyside6-uic form.ui -o ui_form.py, or
#     pyside2-uic form.ui -o ui_form.py
class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.update_data_worker = None
        self.CaptureIpCameraFramesWorker_1 = None
        self.CaptureIpCameraFramesWorker_2 = None
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # rtsp://<Username>:<Password>@<IP Address>:<Port>/cam/realmonitor?channel=1&subtype=0
        self.url_1 = "rtsp://192.168.31.16:8556/unicast"
        self.url_2 = "rtsp://192.168.31.16:8555/unicast"

        # Dictionary to keep the state of a camera. The camera state will be: Normal or Maximized.
        self.list_of_cameras_state = {}

        # Create an instance of a QLabel class to show camera 1.
        self.camera_1 = QLabel()
        self.camera_1.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.camera_1.setScaledContents(True)
        self.camera_1.installEventFilter(self)
        self.camera_1.setObjectName("Camera_1")
        self.list_of_cameras_state["Camera_1"] = "Normal"

        # Create an instance of a QScrollArea class to scroll camera 1 image.
        self.ui.QScrollArea_1.setBackgroundRole(QPalette.Dark)
        self.ui.QScrollArea_1.setWidgetResizable(True)
        self.ui.QScrollArea_1.setWidget(self.camera_1)

        # Create an instance of a QLabel class to show camera 2.
        self.camera_2 = QLabel()
        self.camera_2.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.camera_2.setScaledContents(True)
        self.camera_2.installEventFilter(self)
        self.camera_2.setObjectName("Camera_2")
        self.list_of_cameras_state["Camera_2"] = "Normal"

        # Create an instance of a QScrollArea class to scroll camera 2 image.
        self.ui.QScrollArea_2.setBackgroundRole(QPalette.Dark)
        self.ui.QScrollArea_2.setWidgetResizable(True)
        self.ui.QScrollArea_2.setWidget(self.camera_2)

        # Start the thread CaptureIpCameraFramesWorker.
        self.ui.pB_start_cam.clicked.connect(self.start_video_later)
        # self.start_video_later()

        # Start the thread update_data_worker.
        self.ui.pB_Get_ROS.clicked.connect(self.start_data_worker_later)
        self.ui.pB_Get_ROS.setEnabled(False)
        if check_ROS_master():
            self.ui.pB_Get_ROS.setEnabled(True)


    def start_video_later(self):
        # Create an instance of CaptureIpCameraFramesWorker.
        self.CaptureIpCameraFramesWorker_1 = CaptureIpCameraFramesWorker(self.url_1)
        self.CaptureIpCameraFramesWorker_1.ImageUpdated.connect(lambda image: self.ShowCamera1(image))

        # Create an instance of CaptureIpCameraFramesWorker.
        self.CaptureIpCameraFramesWorker_2 = CaptureIpCameraFramesWorker(self.url_2)
        self.CaptureIpCameraFramesWorker_2.ImageUpdated.connect(lambda image: self.ShowCamera2(image))

        self.ui.pB_stop_cam.clicked.connect(self.stop_cam)
        self.ui.pB_rs_cam1.clicked.connect(self.restart_cam(1))
        self.ui.pB_rs_cam2.clicked.connect(self.restart_cam(2))
        self.ui.pB_start_cam.setEnabled(False)


    def start_data_worker_later(self):
        # Create an instance of DataProcessingWorker.
        self.update_data_worker = UpdateDataWorker()
        self.update_data_worker.signals.signal_init_depth.connect(lambda data: self.update_init_depth(data))
        self.update_data_worker.signals.signal_init_yaw.connect(lambda data: self.update_init_yaw(data))
        self.update_data_worker.signals.signal_yaw.connect(lambda data: self.update_yaw(data))
        self.update_data_worker.signals.signal_depth.connect(lambda data: self.update_depth(data))
        self.update_data_worker.signals.signal_setpoint_depth.connect(lambda data: self.update_setpoint_depth(data))
        self.update_data_worker.signals.signal_setpoint_yaw.connect(lambda data: self.update_setpoint_yaw(data))
        self.ui.pB_Get_ROS.setEnabled(False)

    @Slot()
    def update_init_depth(self, data: float) -> None:
        self.ui.init_depth.setText(str(data))
    @Slot()
    def update_init_yaw(self, data: float) -> None:
        self.ui.init_angle.setText(str(data))
    @Slot()
    def update_yaw(self, data: float) -> None:
        self.ui.value_angle.setText(str(data))
    @Slot()
    def update_depth(self, data: float) -> None:
        self.ui.value_depth.setText(str(data))
    @Slot()
    def update_setpoint_depth(self, data: float) -> None:
        self.ui.change_depth.setText(str(data))
    @Slot()
    def update_setpoint_yaw(self, data: float) -> None:
        self.ui.change_angle.setText(str(data))

    def ShowCamera1(self, frame: QImage) -> None:
        self.camera_1.setPixmap(QPixmap.fromImage(frame))

    @Slot()
    def ShowCamera2(self, frame: QImage) -> None:
        self.camera_2.setPixmap(QPixmap.fromImage(frame))

    # Override method for class MainWindow.
    def eventFilter(self, source: QObject, event: QEvent) -> bool:
        """
        Method to capture the events for objects with an event filter installed.
        :param source: The object for whom an event took place.
        :param event: The event that took place.
        :return: True if event is handled.
        """
        #
        if event.type() == QEvent.MouseButtonDblClick:
            if source.objectName() == 'Camera_1':
                #
                if self.list_of_cameras_state["Camera_1"] == "Normal":
                    self.ui.QScrollArea_2.hide()
                    self.list_of_cameras_state["Camera_1"] = "Maximized"
                    # self.showMaximized()
                else:
                    self.ui.QScrollArea_2.show()
                    self.list_of_cameras_state["Camera_1"] = "Normal"
                    # self.showNormal()
            elif source.objectName() == 'Camera_2':
                #
                if self.list_of_cameras_state["Camera_2"] == "Normal":
                    self.ui.QScrollArea_1.hide()
                    self.list_of_cameras_state["Camera_2"] = "Maximized"
                    # self.showMaximized()
                else:
                    self.ui.QScrollArea_1.show()
                    self.list_of_cameras_state["Camera_2"] = "Normal"
                    # self.showNormal()
            else:
                return super(MainWindow, self).eventFilter(source, event)
            return True
        else:
            return super(MainWindow, self).eventFilter(source, event)

    # Overwrite method closeEvent from class QMainWindow.
    def closeEvent(self, event) -> None:
        self.stop_cam()
        if self.CaptureIpCameraFramesWorker_1 is not None:
            self.CaptureIpCameraFramesWorker_2.quit()
            self.CaptureIpCameraFramesWorker_2.wait()
        if self.CaptureIpCameraFramesWorker_1 is not None:
            self.CaptureIpCameraFramesWorker_1.quit()
            self.CaptureIpCameraFramesWorker_1.wait()
        if self.update_data_worker is not None:
            self.update_data_worker.quit()
            self.update_data_worker.wait()
        # Accept the event
        event.accept()

    def stop_cam(self):
        # If thread getIpCameraFrameWorker_1 is running, then release it.
        if self.CaptureIpCameraFramesWorker_1.isRunning():
            self.CaptureIpCameraFramesWorker_1.capture.release()
        # If thread getIpCameraFrameWorker_2 is running, then release it.
        if self.CaptureIpCameraFramesWorker_2.isRunning():
            self.CaptureIpCameraFramesWorker_2.capture.release()
        self.ui.pB_start_cam.setEnabled(True)

    def restart_cam(self, cam):
        if cam == 2:
            if self.CaptureIpCameraFramesWorker_2.isRunning():
                self.CaptureIpCameraFramesWorker_2.capture.release()
            self.CaptureIpCameraFramesWorker_2.open()
        elif cam == 1:
            if self.CaptureIpCameraFramesWorker_1.isRunning():
                self.CaptureIpCameraFramesWorker_1.capture.release()
            self.CaptureIpCameraFramesWorker_1.open()



if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = MainWindow()
    widget.show()
    sys.exit(app.exec())
