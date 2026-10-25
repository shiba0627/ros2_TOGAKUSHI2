#/home/ubuntu/humble_ws/src/game_pkg/game_pkg/game_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import apriltag
import numpy as np
import playsound
import time
from enum import Enum
import pickle
import os
from datetime import datetime
#from time import perf_counter#処理時間計測用
from PIL import ImageDraw, ImageFont
from PIL import Image as PILImage

calib_dir = '/home/ubuntu/humble_ws/src/game_pkg/game_pkg/calib_files'
#save_dir = f'/home/ubuntu/Desktop/togakushi2_log/tag_logs'
save_dir = 'src/logger_pkg/logger_pkg/log_data'

class Mode(str, Enum):
    STOP = 'STOP'
    USER = 'USER'
    HELPER = 'HELPER'
    GAME = 'GAME'
    ERROR = 'ERROR'

class CameraSubscriberNode(Node):
    def __init__(self):
        super().__init__('game_node')
        self.bridge = CvBridge()
        self.detector = apriltag.Detector()#apriltag検出器

        # pillowフォント
        self.jp_font = ImageFont.truetype(
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            28
        )
        self.text_title = "きどうしました"
        self.text_subtitle = ""
        '''
        # ---- 処理時間計測用 ----
        self.profile_count = 0
        self.profile_sum = {
            'detect': 0.0,
            'pnp': 0.0,
            'callback': 0.0,
        }

        self.profile_max = {
            'detect': 0.0,
            'pnp': 0.0,
            'callback': 0.0,
        }

        self.profile_interval = 100  # 100フレームごとに表示
        # ---- 処理時間計測おわり ----
        '''

        # ---- AprilTagパラメータ設定 ----
        self.tag_size = 0.072  # [m] 実際のタグの一辺長
        self.half = self.tag_size / 2.0
        self.object_points = np.array([
            [-self.half,  self.half, 0],
            [ self.half,  self.half, 0],
            [ self.half, -self.half, 0],
            [-self.half, -self.half, 0],
        ], dtype=np.float32)
        self.camera_matrix = self.load_pickle(f'{calib_dir}/cameraMatrix.pkl')
        self.dist_coeffs = self.load_pickle(f'{calib_dir}/dist.pkl')
        self.tag_log = []#検出ログ

        #ゲームロジック
        self.play_flag = False
        self.last_detect_time = time.time()
        self.reset_delay = 3.0  # 秒数（検出なしでリセット）
        self.last_read_id = 0
        self.total_score = 0
        self.wait_goal = False#スタート未検出
        self.bonus_frag_0 = False#未加算
        self.bonus_frag_7 = False#未加算


        # ---- サブスクライブ ----
        self.subscription = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)
        self.get_logger().info('CameraSubscriberNode started: subscribing to /camera/image')
        #ロボット状態を購読
        self.state_sub = self.create_subscription(
            String, '/robot_state', self.state_update, 10)

        self.now_state = Mode.USER
        # ---- タイマーで定期的にリセット判定 ----
        self.create_timer(0.5, self.check_detection_timeout)
        
        #パブリッシャー
        self.img_pub = self.create_publisher(Image, '/camera/gameimage', 10)


    def state_update(self, msg: String):
        """/robot_stateトピックに基づいて内部の状態を更新する"""
        self.now_state = msg.data

    def image_callback(self, msg: Image):
        #callback_start = perf_counter()#処理時間計測
        border_color=(0,255,0)

        try:
            #pnp_total_time = 0.0#処理時間計測
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')#sensor_msgs/ImageをOpenCV形式に変換
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            #t0 = perf_counter()#処理時間計測
            results = self.detector.detect(gray)
            #detect_time = perf_counter() - t0# 処理時間計測

            if results:
                self.last_detect_time = time.time()#3秒待つ用

            for r in results:
                (ptA, ptB, ptC, ptD) = r.corners#角４点
                pts = np.array([ptA, ptB, ptC, ptD], dtype=int)
                #cv2.polylines(cv_image, [pts], True, (0, 255, 0), 2)
                h, w = cv_image.shape[:2]
                #cv2.rectangle(cv_image, (0, 0), (w, h), (0, 255, 0), thickness=20)

                tag_id = r.tag_id

                image_points = np.array(r.corners, dtype=np.float32)

                #t0 = perf_counter()#処理時間計測
                success, rvec, tvec = cv2.solvePnP(
                    self.object_points,
                    image_points,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                #pnp_total_time += perf_counter() - t0#処理時間計測
                if not success:
                    continue
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self.tag_log.append({
                    'time': stamp,
                    'tag_id': int(tag_id),
                    'tx': float(tvec[0, 0]),
                    'ty': float(tvec[1, 0]),
                    'tz': float(tvec[2, 0]),
                    'rx': float(rvec[0, 0]),
                    'ry': float(rvec[1, 0]),
                    'rz': float(rvec[2, 0]),
                })

                # ---- 音声再生 ----
                if self.now_state == Mode.GAME:
                    if not self.play_flag and tag_id == 1:#スタート
                        #playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/「よろしく頼む」.mp3', block=False)
                        playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/カウントダウン.mp3', block=False)
                        self.play_flag = True
                    elif not self.play_flag and tag_id == 2:#ゴール
                        #playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/決定ボタンを押す13.mp3', block=False)
                        #playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/「さあ、いくぞ！」.mp3', block=False)
                        #playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/クリア.mp3', block=False)
                        playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/ゴール2.mp3', block=False)
                        self.play_flag = True
                    elif not self.play_flag and tag_id == 3:#スタート
                        #playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/決定ボタンを押す17.mp3', block=False)
                        #playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/「これは強敵だな…！」.mp3', block=False)
                        playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/カウントダウン2.mp3', block=False)
                        self.play_flag = True
                    elif not self.play_flag and tag_id == 4:#ゴール
                        #playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/和太鼓でドン.mp3', block=False)
                        #playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/「やるじゃないか！」.mp3', block=False)
                        playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/ゴール.mp3', block=False)
                        self.play_flag = True
                    elif not self.play_flag and tag_id == 0:#ボーナス
                        #playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/ニワトリの鳴き声1.mp3', block=False)
                        #playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/「くらえ！」.mp3', block=False)
                        playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/ラッパのファンファーレ.mp3', block=False)
                        self.play_flag = True
                    elif not self.play_flag and tag_id == 7:#ボーナス
                        #playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/ニワトリの鳴き声1.mp3', block=False)
                        #playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/「くらえ！」.mp3', block=False)
                        playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/レベルアップ.mp3', block=False)
                        self.play_flag = True

                    if tag_id in (1, 3):#赤
                        border_color = (0, 0, 255)
                        self.text_title = "スタート！"
                        self.text_subtitle = "ゴールをめざそう"
                        self.wait_goal = True#スタート読み取り済み
                        self.bonus_frag_0 = False
                        self.bonus_frag_7 = False

                    elif tag_id in (2, 4):#青
                        border_color = (255, 0, 0)
                        self.text_title = "ゴール！"
                        self.text_subtitle = "つぎのスタートへ"
                        if self.wait_goal:
                            self.wait_goal = False#スタート未検出
                            self.total_score += 100
                        #self.bonus_frag_7 = False
                        #self.bonus_frag_0 = False

                    elif tag_id in (7, 0):#紫
                        border_color = (128, 0, 128)
                        self.text_title = "ボーナス！"
                        self.text_subtitle = "ボーナスタグを 発見"
                        if tag_id == 0 and not self.bonus_frag_0:#Falseなら
                            self.total_score += 50
                            self.bonus_frag_0 = True
                        if tag_id == 7 and not self.bonus_frag_7:#Falseなら
                            self.total_score += 50
                            self.bonus_frag_7 = True

                    else:
                        border_color = (0, 255, 0)

                #AprilTag検出時の矩形
                cv2.polylines(cv_image, [pts], True, border_color, 2)
                cv2.rectangle(cv_image, (0, 0), (w, h), border_color, thickness=20)
            
            #cv2.rectangle(img=cv_image, pt1=(15, 20), pt2=(180, 120), color=(0,0,0), thickness=-1)
            #cv2.putText(cv_image, f'{self.now_state}', (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
            #cv2.putText(cv_image, f'NEXT:{self.last_read_id+1}', (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
            #cv2.putText(cv_image, f'SCORE:{self.total_score}', (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

            # 日本語を描画
            self.effect_color = border_color
            cv_image = self.draw_japanese(cv_image)

            #加工済ゲーム画面を配信
            try:
                img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'cv2_to_imgmsg failed: {e}')
                return
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_frame_2'
            self.img_pub.publish(img_msg)

            '''
            # ---- 処理時間の集計 ----
            callback_time = perf_counter() - callback_start

            times = {
                'detect': detect_time,
                'pnp': pnp_total_time,
                'callback': callback_time,
            }
            
            self.profile_count += 1

            for key, value in times.items():
                self.profile_sum[key] += value
                self.profile_max[key] = max(
                    self.profile_max[key], value
                )

            if self.profile_count >= self.profile_interval:
                n = self.profile_count

                self.get_logger().info(  
                    f'[Performance] Last {n} frames | '
                    f'Detect avg/max: '
                    f'{self.profile_sum["detect"] / n * 1000:.2f}/'
                    f'{self.profile_max["detect"] * 1000:.2f} ms | '
                    f'PnP avg/max: '
                    f'{self.profile_sum["pnp"] / n * 1000:.2f}/'
                    f'{self.profile_max["pnp"] * 1000:.2f} ms | '
                    f'Callback avg/max: '
                    f'{self.profile_sum["callback"] / n * 1000:.2f}/'
                    f'{self.profile_max["callback"] * 1000:.2f} ms'
                )

                self.profile_count = 0

                for key in self.profile_sum:
                    self.profile_sum[key] = 0.0
                    self.profile_max[key] = 0.0
            '''
            #cv2.imshow('AprilTag Detection', cv_image)
            #cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Image callback error: {e}')

    def check_detection_timeout(self):
        """一定時間検出がなければフラグをリセット"""
        if self.play_flag and (time.time() - self.last_detect_time > self.reset_delay):
            self.play_flag = False
            self.get_logger().info('3秒経過, 音声再生フラグをリセット')
    
    def load_pickle(self, path):
        with open(path, 'rb') as f:
            return pickle.load(f)
    
    def save_logs(self):
        os.makedirs(save_dir, exist_ok=True)
        now_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        file_path = os.path.join(save_dir, f'tag_log_{now_str}.txt')
        with open(file_path, 'w') as f:
            f.write('# time[s] tag_id tx ty tz rx ry rz\n')
            for i in self.tag_log:
                f.write(
                    f"{i['time']:.6f} {i['tag_id']} "
                    f"{i['tx']:.4f} {i['ty']:.4f} {i['tz']:.4f} "
                    f"{i['rx']:.6f} {i['ry']:.6f} {i['rz']:.6f}\n"
                )

    def draw_japanese(self, cv_image, text=None, position=(20, 150)):
        if not getattr(self, "text_title", ""):
            return cv_image

        h, w = cv_image.shape[:2]

        # パネルのサイズと位置
        panel_x = 20
        panel_y = 20
        panel_w = min(w - 40, 220)
        panel_h = 125

        # 半透明の黒い背景
        overlay = cv_image.copy()
        cv2.rectangle(
            overlay,
            (panel_x, panel_y),
            (panel_x + panel_w, panel_y + panel_h),
            (20, 20, 20),
            thickness=-1
        )

        cv2.addWeighted(
            overlay, 0.72,
            cv_image, 0.28,
            0, cv_image
        )

        # アクセントカラーのライン
        cv2.rectangle(
            cv_image,
            (panel_x, panel_y),
            (panel_x + 6, panel_y + panel_h),
            self.effect_color or (255, 255, 255),
            thickness=-1
        )

        # 日本語テキストを描画
        pil_image = PILImage.fromarray(
            cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        )
        draw = ImageDraw.Draw(pil_image)

        # メインメッセージ
        draw.text(
            (panel_x + 22, panel_y + 15),
            self.text_title,
            font=self.jp_font,
            fill=(255, 255, 255),
            stroke_width=1,
            stroke_fill=(0, 0, 0)
        )

        # 補足メッセージ
        draw.text(
            (panel_x + 22, panel_y + 60),
            self.text_subtitle,
            font=ImageFont.truetype(
                "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
                20
            ),
            fill=(225, 225, 225)
        )

        draw.text(
                    (panel_x + 22, panel_y + 80),
                    f"{self.total_score}点",
                    font=ImageFont.truetype(
                        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
                        20
                    ),
                    fill=(225, 225, 225)
                )

        return cv2.cvtColor(
            np.array(pil_image),
            cv2.COLOR_RGB2BGR
        )
    
def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_logs()
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
