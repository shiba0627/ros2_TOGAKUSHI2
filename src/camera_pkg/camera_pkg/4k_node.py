import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# --- 設定 ---
CAM_ID = 4        # カメラID
WIDTH = 720      # 指定したい幅
HEIGHT = 480      # 指定したい高さ
FPS = 30          # フレームレート
# -----------

class CameraPublisher(Node):
    def __init__(self, video_device=0, topic_name='/camera/image', width=640, height=480, fps=30):
        super().__init__('camera_node')
        self.get_logger().info(f'Starting: device={video_device}, target={width}x{height}, fps={fps}')

        # Publisher
        self.pub = self.create_publisher(Image, topic_name, 10)

        # cv_bridge
        self.bridge = CvBridge()

        # OpenCV VideoCapture
        self.cap = cv2.VideoCapture(video_device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().warn('V4L2 backend failed, trying default.')
            self.cap = cv2.VideoCapture(video_device)

        if not self.cap.isOpened():
            self.get_logger().error('カメラをオープンできませんでした')
            raise RuntimeError('Cannot open video device')

        if self.cap.isOpened():
            # --- 画像サイズ指定 ---
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            
            # オートフォーカス設定 (必要な場合)
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)

            # --- 実際に適用されたサイズを確認してログ表示 ---
            camera_fps = self.cap.get(cv2.CAP_PROP_FPS)
            actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            self.get_logger().info(f'Actual Camera Resolution: {int(actual_w)} x {int(actual_h)},{camera_fps}fps')

        # タイマー
        period_sec = 1.0 / fps
        self.timer = self.create_timer(period_sec, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Frame read failed')
            return

        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv2_to_imgmsg failed: {e}')
            return

        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_frame'

        self.pub.publish(img_msg)

    def destroy_node(self):
        try:
            if self.cap is not None and self.cap.isOpened():
                self.cap.release()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        # 引数でサイズを渡す
        node = CameraPublisher(
            video_device=CAM_ID, 
            topic_name='/camera/image', 
            width=WIDTH, 
            height=HEIGHT, 
            fps=FPS
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f'Error: {e}')
    finally:
        # node変数が定義されていない場合の対策
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()