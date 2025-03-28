import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class CompressedImageViewer(Node):
    def __init__(self):
        super().__init__('compressed_image_viewer')
        # "/camera/image/compressed" 토픽 이름은 필요에 따라 변경하세요.
        self.subscription = self.create_subscription(
            CompressedImage,
            'yolo/compressed',
            self.image_callback,
            10
        )
        # OpenCV 창 생성 (크기 조절 가능하도록 WINDOW_NORMAL)
        cv2.namedWindow("Compressed Image", cv2.WINDOW_NORMAL)

    def image_callback(self, msg: CompressedImage):
        # CompressedImage 메시지의 data를 numpy 배열로 변환
        np_arr = np.frombuffer(msg.data, np.uint8)
        # imdecode를 사용하여 JPEG 등 압축된 이미지를 디코딩
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image_np is not None:
            cv2.imshow("Compressed Image", image_np)
            # 짧은 시간 대기 (1ms)하여 창 업데이트
            cv2.waitKey(1)
        else:
            self.get_logger().error("이미지 디코딩에 실패했습니다.")

def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
