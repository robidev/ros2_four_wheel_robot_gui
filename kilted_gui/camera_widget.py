# kilted_gui/camera_widget.py

from PyQt5.QtWidgets import QLabel
from PyQt5.QtGui import QImage, QPixmap
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraWidget(QLabel):
    def __init__(self, node):
        super().__init__()
        self.setText("Waiting for camera...")
        self.setFixedHeight(240)
        self.setFixedWidth(320)
        self.bridge = CvBridge()
        self.subscription = node.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.setPixmap(pixmap.scaled(self.width(), self.height()))
        except Exception as e:
            self.setText(f"Camera error: {e}")
