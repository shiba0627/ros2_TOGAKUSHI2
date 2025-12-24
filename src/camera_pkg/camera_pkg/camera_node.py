import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

CAM_ID = 4#接続したWEBカメラのID．起動のたびに変わる可能性あり

class CameraPublisher(Node):
    def __init__(self, video_device=0, topic_name='/camera/image', fps=30):
            super().__init__('camera_node')
            self.get_logger().info(f'CameraPublisher starting: device={video_device}, topic={topic_name}, fps={fps}')

            # Publisher （QoS depth は用途に応じて調整）
            self.pub = self.create_publisher(Image, topic_name, 10)

            # cv_bridge インスタンス
            self.bridge = CvBridge()

            # OpenCV VideoCapture（video_device は 0 や '/dev/video0' 等）
            self.cap = cv2.VideoCapture(video_device, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                # 別のバックエンドで試す
                self.get_logger().warn('Failed to open camera with V4L2 backend, trying default backend.')
                self.cap = cv2.VideoCapture(video_device)

            if not self.cap.isOpened():
                self.get_logger().error('カメラオープンできませんでした')
                raise RuntimeError('Cannot open video device')
            if self.cap.isOpened():
            # オートフォーカス有効
                self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
            width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            self.get_logger().info(f'Camera Resolution: {int(width)} x {int(height)}')

            # タイマー（ループ）
            period_sec = 1.0 / fps
            self.timer = self.create_timer(period_sec, self.timer_callback)

    def timer_callback(self):
            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.get_logger().warn('Frame read failed')
                return

            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')#sensor_msgs/Imageに変換
            except Exception as e:
                self.get_logger().error(f'cv2_to_imgmsg failed: {e}')
                return

            # ヘッダ情報（タイムスタンプ、フレームID）
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_frame'

            self.pub.publish(img_msg)
            # self.get_logger().debug('Published camera frame')
    def destroy_node(self):
            # ノード破棄時にカメラを解放
            try:
                if self.cap is not None and self.cap.isOpened():
                    self.cap.release()
            except Exception:
                pass
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraPublisher(video_device=CAM_ID, topic_name='/camera/image', fps=30)
    except RuntimeError as e:
        print(f'Failed to start CameraPublisher: {e}')
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
