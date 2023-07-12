#!/bin/env python3
# This Python file uses the following encoding: utf-8
import cv2

from PySide6.QtGui import QImage
from PySide6.QtCore import QThread, Signal, Qt

class CaptureIpCameraFramesWorker(QThread):
    # Signal emitted when a new image or a new frame is ready.
    ImageUpdated = Signal(QImage)

    def __init__(self, url) -> None:
        super(CaptureIpCameraFramesWorker, self).__init__()
        # Declare and initialize instance variables.
        self.shape = None
        self.channels = None
        self.width = None
        self.bytes_per_line = None
        self.height = None
        self.capture = None
        self.url = url
        self.fps = 0
        self.capture = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)

    def run(self) -> None:
        # Capture video from a network stream.
        # Get default video FPS.
        self.fps = self.capture.get(cv2.CAP_PROP_FPS)
        print(self.fps)
        # If video capturing has been initialized already.q
        # While the thread is active.
        while self.capture.isOpened():
            # Grabs, decodes and returns the next video frame.
            ret, frame = self.capture.read()
            # Get the frame height, width and channels.
            if self.shape is None:
                # Calculate the number of bytes per line.
                self.shape = frame.shape
                self.height, self.width, self.channels = frame.shape
                self.bytes_per_line = self.width * self.channels
            if ret:
                # Convert image from BGR (cv2 default color format) to RGB (Qt default color format).
                cv_rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                # Convert the image to Qt format.
                qt_rgb_image = QImage(cv_rgb_image.data, self.width, self.height, self.bytes_per_line, QImage.Format_RGB888)
                # Scale the image.
                qt_rgb_image_scaled = qt_rgb_image.scaled(640, 360, Qt.KeepAspectRatio)  # 360p
                # qt_rgb_image_scaled = qt_rgb_image.scaled(1280, 720, Qt.KeepAspectRatio)  # 720p
                # qt_rgb_image_scaled = qt_rgb_image.scaled(1920, 1080, Qt.KeepAspectRatio) # 1080p
                # Emit this signal to notify that a new image or frame is available.
                self.ImageUpdated.emit(qt_rgb_image_scaled)
            else:
                break

    def open(self):
        self.capture.open(self.url, cv2.CAP_FFMPEG)