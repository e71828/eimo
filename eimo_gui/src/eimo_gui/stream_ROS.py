#!/bin/env python3
# This Python file uses the following encoding: utf-8

import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image

from PySide6.QtGui import QImage
from PySide6.QtCore import QThread, Signal, Qt


class CaptureIpCameraFramesWorker(QThread):
    # Signal emitted when a new image or a new frame is ready.
    ImageUpdated = Signal(QImage)

    def __init__(self, url) -> None:
        # Instantiate CvBridge
        self.bridge = CvBridge()

        super().__init__()
        rospy.init_node('image_listener')
        self.frame_fmt = rospy.get_param(
            '~filename_format', 'frame%04i_%i.jpg')
        # Define your image topic
        image_topic = "/cameras/left_hand_camera/image"
        # Set up your subscriber and define its callback
        self.capture = rospy.Subscriber(image_topic, Image, self.image_callback)
        # Spin until ctrl + c
        rospy.spin()


    def image_callback(self, msg):
        print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Convert image from BGR (cv2 default color format) to RGB (Qt default color format).
            cv_rgb_image = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
            # Convert the image to Qt format.
            bytes_per_line = msg.width * msg.channels
            qt_rgb_image = QImage(cv_rgb_image.data, msg.width, msg.height, bytes_per_line, QImage.Format_RGB888)
            # Scale the image.
            qt_rgb_image_scaled = qt_rgb_image.scaled(640, 360, Qt.KeepAspectRatio)  # 360p
            # Emit this signal to notify that a new image or frame is available.
            self.ImageUpdated.emit(qt_rgb_image_scaled)
        finally:
            pass
