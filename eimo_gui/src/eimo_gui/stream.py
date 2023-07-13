#!/bin/env python3
# This Python file uses the following encoding: utf-8
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import QThread, Signal


class CaptureIpCameraFramesWorker(QThread):
    # Signal emitted when a new image or a new frame is ready.
    ImageUpdated = Signal(QImage)

    def __init__(self, image_topic) -> None:
        # Instantiate CvBridge
        self.capture = None
        self.image_topic = image_topic
        super().__init__()
        self.frame_fmt = rospy.get_param(
            '~filename_format', 'frame_0_%04i.jpg')
        rospy.on_shutdown(self.release)


    def run(self) -> None:
        # Set up your subscriber and define its callback
        self.capture = rospy.Subscriber(self.image_topic, CompressedImage, self.image_callback)
        # Spin until ctrl + c
        rospy.spin()


    def image_callback(self, msg):
        # Convert the image to Qt format.
        # qt_rgb_image = QImage(msg.data, msg.width, msg.height, msg.step, QImage.Format_RGB888)
        qt_rgb_image = QImage.fromData(msg.data)
        # Emit this signal to notify that a new image or frame is available.
        self.ImageUpdated.emit(qt_rgb_image)

    def release(self):
        if self.capture is not None:
            self.capture.unregister()
        rospy.signal_shutdown('GUI shutdown')
