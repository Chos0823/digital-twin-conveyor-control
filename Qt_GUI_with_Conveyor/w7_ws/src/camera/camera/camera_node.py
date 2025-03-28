#!/usr/bin/env python3
import sys
import cv2
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtGui import QImage, QPixmap, QFont, QColor, QPalette
from PyQt5.QtCore import Qt, QTimer


# ROS2 카메라 퍼블리셔 노드 (CameraNode)
class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.publisher_ = self.create_publisher(Image, "camera/image_raw", 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # 기본 카메라 인덱스 (필요시 파라미터 변경)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
        else:
            self.get_logger().info("Camera opened successfully")
        # 30ms 타이머 (약 30fps)
        self.timer = self.create_timer(0.03, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publisher_.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"Error converting frame: {e}")
        else:
            self.get_logger().warn("Failed to read frame")

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        super().destroy_node()


# ROS2 이미지 서브스크라이버 노드 (ImageSubscriber)를 활용하여 GUI 업데이트
class ImageSubscriber(Node):
    def __init__(self, update_callback):
        super().__init__("image_subscriber")
        self.subscription = self.create_subscription(
            Image, "camera/image_raw", self.listener_callback, 10
        )
        self.bridge = CvBridge()
        self.update_callback = update_callback  # GUI 업데이트 함수

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(
                rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888
            )
            self.update_callback(qt_image)
        except Exception as e:
            self.get_logger().error(f"Error in listener callback: {e}")


# PyQt5 GUI 창 (CameraMonitor)
class CameraMonitor(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle("USB Camera Monitor")
        self.setGeometry(100, 100, 640, 480)
        self.set_palette()

        self.video_label = QLabel("카메라 피드가 표시됩니다.")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color: black;")

        layout = QVBoxLayout()
        layout.addWidget(self.video_label)
        self.setLayout(layout)

    def set_palette(self):
        palette = self.palette()
        palette.setColor(QPalette.Window, QColor("#f4f4f4"))
        self.setPalette(palette)

    def update_image(self, qt_image):
        pixmap = QPixmap.fromImage(qt_image)
        self.video_label.setPixmap(
            pixmap.scaled(
                self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
        )


def main(args=None):
    rclpy.init(args=args)
    # PyQt5 애플리케이션 생성 (메인 스레드)
    app = QApplication(sys.argv)
    monitor = CameraMonitor()
    monitor.show()

    # ROS2 카메라 퍼블리셔와 이미지 서브스크라이버 노드 생성
    camera_node = CameraNode()
    image_subscriber = ImageSubscriber(update_callback=monitor.update_image)

    # QTimer를 사용하여 ROS2 노드들을 주기적으로 spin_once() 호출
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(camera_node, timeout_sec=0.001))
    timer.timeout.connect(lambda: rclpy.spin_once(image_subscriber, timeout_sec=0.001))
    timer.start(10)  # 10ms 간격

    exit_code = app.exec_()

    camera_node.destroy_node()
    image_subscriber.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
