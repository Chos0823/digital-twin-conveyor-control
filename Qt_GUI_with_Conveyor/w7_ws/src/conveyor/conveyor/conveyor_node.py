#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import threading
import json  # JSON 모듈 추가

# 시리얼 포트 및 스텝 설정
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
STEPS_PER_MM = 1085 / 100  # 1mm 당 스텝 수


class SerialHandler(threading.Thread):
    """
    ROS2 노드와 함께 시리얼 통신을 담당하는 별도 쓰레드
    """

    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.ser = None
        self.is_running = True

    def open_serial(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)  # 아두이노 초기화 대기
            self.node.get_logger().info("Serial port opened")
        except serial.SerialException as e:
            self.node.get_logger().error(f"Error opening serial port: {e}")

    def run(self):
        self.open_serial()
        while self.is_running:
            if self.ser and self.ser.in_waiting > 0:
                try:
                    status = self.ser.readline().decode().strip()
                    if status:
                        self.node.get_logger().info(f"Received: {status}")
                        msg = String()
                        msg.data = status
                        self.node.status_pub.publish(msg)
                except Exception as e:
                    self.node.get_logger().error(f"Serial read error: {e}")
            time.sleep(0.1)

    def stop(self):
        self.is_running = False
        if self.ser:
            self.ser.close()
            self.node.get_logger().info("Serial port closed")

    def send_data(self, data: str):
        if self.ser:
            self.ser.write((data + "\n").encode())
            self.node.get_logger().info(f"Sent: {data}")


class ConveyorNode(Node):
    """
    ROS2 노드: conveyor_node
    - "conveyor_command" 토픽을 구독하여 명령을 처리합니다.
    - 시리얼 통신을 통해 아두이노에 명령을 전송하고, 상태를 "conveyor_status"로 퍼블리시합니다.
    """

    def __init__(self):
        super().__init__("conveyor_node")
        self.status_pub = self.create_publisher(String, "conveyor_status", 10)
        self.create_subscription(String, "/conveyor/control", self.command_callback, 10)
        self.serial_handler = SerialHandler(self)
        self.serial_handler.start()
        self.get_logger().info("Conveyor node started")

    def command_callback(self, msg: String):
        try:
            # JSON 데이터 파싱
            command = json.loads(msg.data.strip())
            self.get_logger().info(f"Received command: {command}")

            if command["control"] == "go":
                distance_mm = command.get("distance.mm")
                if isinstance(distance_mm, int) and distance_mm > 0:
                    step_count = int(distance_mm * STEPS_PER_MM)
                    self.serial_handler.send_data(f"go,{step_count}")
                    self.get_logger().info(f"Sent go command: {step_count} steps")
                else:
                    self.get_logger().error("Invalid or missing 'distance.mm' value")
            elif command["control"] == "stop":
                self.serial_handler.send_data("1")
                self.get_logger().info("Sent stop command")
            else:
                self.get_logger().warn(f"Unknown command: {command['control']}")
        except json.JSONDecodeError:
            self.get_logger().error("Received invalid JSON format")

    def destroy_node(self):
        self.serial_handler.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
