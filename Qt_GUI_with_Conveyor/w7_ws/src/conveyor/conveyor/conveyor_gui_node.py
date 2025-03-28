#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QLineEdit,
    QPushButton,
    QLabel,
    QComboBox,
    QGraphicsView,
    QGraphicsScene,
    QGraphicsPixmapItem
)
from PyQt5.QtGui import QFont, QColor, QPalette, QPixmap, QImage
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
import json
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

class ConveyorGUINode(Node):
    def __init__(self):
        super().__init__("conveyor_gui_node")
        self.cmd_publisher = self.create_publisher(String, "/conveyor/control", 10)
        self.gui_command_publisher = self.create_publisher(String, "/gui/command", 10)
        self.create_subscription(String, "conveyor_status", self.status_callback, 10)
        self.create_subscription(String, "gui/status", self.gui_status_callback, 10)
        self.status = "READY"
        self.get_logger().info("Conveyor GUI node started")

    def status_callback(self, msg: String):
        self.status = msg.data
        self.get_logger().info(f"Status updated: {self.status}")

    def gui_status_callback(self, msg: String):
        if msg.data == "컨베이어 작동 준비":
            self.get_logger().info("Received '컨베이어 작동 준비', commanding 1000mm move")
            command = {
                "control": "go",
                "distance.mm": 1000,
            }
            json_command = json.dumps(command)
            self.send_command(json_command)
        elif msg.data == "컨베이어 보정":
            self.get_logger().info("Received '컨베이어 보정', commanding 100mm move")
            command = {
                "control": "go",
                "distance.mm": 100,
            }
            json_command = json.dumps(command)
            self.send_command(json_command)

    def send_command(self, command: str):
        msg = String()
        msg.data = command
        self.cmd_publisher.publish(msg)
        self.get_logger().info(f"Command sent: {command}")

    def send_command_to_gui(self, command: str):  # 새로운 명령을 전송하는 메서드
        msg = String()
        msg.data = command
        self.gui_command_publisher.publish(msg)
        self.get_logger().info(f"GUI Command sent: {command}")


class ConveyorGUI(QWidget):
    def __init__(self, ros_node: ConveyorGUINode, image_thread):
        super().__init__()
        self.ros_node = ros_node
        self.image_thread = image_thread
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("🚀 Conveyor GUI Control")
        self.setGeometry(100, 100, 700, 1000)
        self.set_palette()

        layout = QVBoxLayout()

        self.label = QLabel("이동할 거리 (mm):")
        self.label.setFont(QFont("Arial", 12, QFont.Bold))
        self.label.setAlignment(Qt.AlignCenter)

        self.input_distance = QLineEdit()
        self.input_distance.setPlaceholderText("예: 100")
        self.input_distance.setFont(QFont("Arial", 11))

        self.btn_go = self.create_button("🚀 Go", "#2ecc71", self.go_command)
        self.btn_stop = self.create_button("🛑 Stop", "#e74c3c", self.stop_command)

        self.status_label = QLabel("Status: READY")
        self.status_label.setFont(QFont("Arial", 12, QFont.Bold))
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("color: #555;")

        # 블록 개수 선택 (Red, Blue)
        self.red_block_label = QLabel("Red Blocks (0-2):")
        self.red_block_combo = QComboBox()
        self.red_block_combo.addItems(["0", "1", "2"])

        self.blue_block_label = QLabel("Blue Blocks (0-2):")
        self.blue_block_combo = QComboBox()
        self.blue_block_combo.addItems(["0", "1", "2"])

        # 목표(goal) 선택
        self.goal_label = QLabel("Goal (1-3):")
        self.goal_combo = QComboBox()
        self.goal_combo.addItems(["1", "2", "3"])

        # 작업 시작 버튼
        self.btn_start_task = self.create_button("🚀 Start Task", "#3498db", self.start_task_command)

        # 이미지 뷰어 추가
        self.image_label = QLabel("Camera Feed:")
        self.image_label.setFont(QFont("Arial", 12, QFont.Bold))

        # PyQt5의 QGraphicsView로 이미지를 표시
        self.view = QGraphicsView(self)
        self.scene = QGraphicsScene(self)
        self.view.setScene(self.scene)
        self.pixmap_item = QGraphicsPixmapItem()
        self.scene.addItem(self.pixmap_item)

        # 레이아웃에 위젯 추가
        layout.addWidget(self.label)
        layout.addWidget(self.input_distance)
        layout.addWidget(self.btn_go)
        layout.addWidget(self.btn_stop)
        layout.addWidget(self.status_label)
        layout.addWidget(self.red_block_label)
        layout.addWidget(self.red_block_combo)
        layout.addWidget(self.blue_block_label)
        layout.addWidget(self.blue_block_combo)
        layout.addWidget(self.goal_label)
        layout.addWidget(self.goal_combo)
        layout.addWidget(self.btn_start_task)
        layout.addWidget(self.image_label)
        layout.addWidget(self.view)
        self.setLayout(layout)

        # 이미지 스레드 시작
        self.image_thread.start()

    def set_palette(self):
        palette = self.palette()
        palette.setColor(QPalette.Window, QColor("#f4f4f4"))
        self.setPalette(palette)

    def create_button(self, text, color, callback):
        button = QPushButton(text)
        button.setFont(QFont("Arial", 11))
        button.setStyleSheet(
            f"background-color: {color}; color: white; border-radius: 10px; padding: 8px;"
        )
        button.clicked.connect(callback)
        return button

    def go_command(self):
        distance_text = self.input_distance.text().strip()
        if not distance_text.isdigit():
            self.status_label.setText("⚠️ 숫자만 입력하세요!")
            return

        # JSON 형식으로 명령 전송
        command = {
            "control": "go",
            "distance.mm": int(distance_text),
        }
        json_command = json.dumps(command)
        self.ros_node.send_command(json_command)
        self.status_label.setText(f"Sent: {json_command}")

    def stop_command(self):
        # JSON 형식으로 stop 명령 전송
        command = {"control": "stop"}
        json_command = json.dumps(command)
        self.ros_node.send_command(json_command)
        self.status_label.setText("Sent: stop")

    def start_task_command(self):
        red_blocks = self.red_block_combo.currentText()
        blue_blocks = self.blue_block_combo.currentText()
        goal = self.goal_combo.currentText()

        # JSON 형식으로 작업 명령 전송 (gui/command로 전송)
        task_command = {
            "red": int(red_blocks),
            "blue": int(blue_blocks),
            "goal": int(goal),
        }
        json_command = json.dumps(task_command)
        self.ros_node.send_command_to_gui(json_command)  # 변경된 메서드 호출
        self.status_label.setText(f"Sent: {json_command}")

    def update_status_label(self):
        self.status_label.setText(f"Status: {self.ros_node.status}")

    def update_image(self, image_np):
        """이미지 업데이트를 위한 메소드"""
        if image_np is not None:
            # OpenCV 이미지 -> RGB로 변환
            image_rgb = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)

            # QImage로 변환
            height, width, channels = image_rgb.shape
            bytes_per_line = channels * width
            q_image = QImage(image_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)
            
            # QPixmap으로 변환하여 표시
            self.pixmap_item.setPixmap(QPixmap.fromImage(q_image))
            self.view.repaint()


class ImageThread(QThread):
    image_signal = pyqtSignal(np.ndarray)

    def __init__(self, ros_node: Node):
        super().__init__()
        self.ros_node = ros_node
        
        # QoS 설정: Reliable, Keep Last History로 설정하여 더 안정적이고 빠른 이미지 수신
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.subscription = self.ros_node.create_subscription(
            CompressedImage,
            'yolo/compressed',
            self.image_callback,
            qos_profile  # QoS 설정 추가
        )

    def image_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image_np is not None:
            self.image_signal.emit(image_np)
        else:
            self.ros_node.get_logger().error("이미지 디코딩에 실패했습니다.")

    def run(self):
        rclpy.spin(self.ros_node)


def main(args=None):
    rclpy.init(args=args)
    ros_node = ConveyorGUINode()

    # 이미지 스레드 준비
    image_thread = ImageThread(ros_node)
    app = QApplication(sys.argv)
    gui = ConveyorGUI(ros_node, image_thread)
    gui.show()

    # QTimer를 이용해 ROS2 spin_once() 및 GUI 상태 업데이트를 주기적으로 호출
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.001))  # 1ms 간격
    timer.timeout.connect(gui.update_status_label)
    timer.start(10)  # 10ms 간격

    # 이미지 수신 시 GUI 업데이트
    image_thread.image_signal.connect(gui.update_image)

    exit_code = app.exec_()
    ros_node.destroy_node()
    image_thread.quit()
    image_thread.wait()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
