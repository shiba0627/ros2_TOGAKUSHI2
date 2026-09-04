#!/usr/bin/env python3
"""
pointcloud_saver_node.py

/rslidar_points (sensor_msgs/PointCloud2) を購読し、最新1フレームのみを
バッファ保持する。ROS2サービス (std_srvs/Trigger) が呼ばれた時点で、
その時のバッファ内容をNumPyバイナリ(.npz)としてディスクに保存する。

想定メッセージフォーマット:
    fields: x, y, z, intensity (すべて FLOAT32, offset 0/4/8/12)
    point_step: 16 (パディングなし、4 floatが連続)

保存先:
    /home/ubuntu/humble_ws/src/logger_pkg/logger_pkg/log_data/pointclauds/YYYYMMDD/pointcloud_HHMMSS_mmm.npz
    (YYYYMMDD, HHMMSS_mmm はサービス呼び出し=保存トリガー時刻)

npz内の配列:
    points        : (N, 4) float32  [x, y, z, intensity]
    stamp_sec     : スカラー int64  (フレームのheader.stamp.sec)
    stamp_nanosec : スカラー int64  (フレームのheader.stamp.nanosec)

    ※ stamp はフレーム単位の値であり、point-wiseのtimestampは元データに
      存在しないため、CSV版のように全行へ複製せずスカラーとして1つだけ
      保持する(容量・書き込み時間ともに削減できるため)。

    ※ points はfloat32のまま保存する。float64へ変換すると書き込みコスト・
      容量が倍になるだけで元データの精度は増えない。一方でstampは
      Unix epoch秒(約17億)でfloat32の仮数部(24bit ≈ 1677万)では
      正確に表現できないため、int64のまま別配列として保持している。

読み込み例:
    import numpy as np
    data = np.load("pointcloud_XXXXXX_XXX.npz")
    points = data["points"]              # (N, 4) float32
    stamp_sec = int(data["stamp_sec"])
    stamp_nanosec = int(data["stamp_nanosec"])

Flask GUI等からの呼び出し例:
    ros2 service call /save_pointcloud std_srvs/srv/Trigger {}
"""

import os
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger

# ---- 固定設定 -----------------------------------------------------------
POINTCLOUD_TOPIC = "/rslidar_points"
SAVE_SERVICE_NAME = "save_pointcloud"
START_AUTOSAVE_SERVICE = "start_pointcloud_recording"
STOP_AUTOSAVE_SERVICE = "stop_pointcloud_recording"
# BASE_SAVE_DIR = os.path.expanduser("~/togakushi2_data/pointclouds")
BASE_SAVE_DIR = os.path.expanduser("/home/ubuntu/humble_ws/src/logger_pkg/logger_pkg/log_data/pointclauds")
EXPECTED_POINT_STEP = 16  # x,y,z,intensity すべてfloat32、パディングなし
POINTS_PER_ROW = 4  # x, y, z, intensity


class PointCloudSaverNode(Node):
    def __init__(self):
        super().__init__("pointcloud_saver_node")

        # LiDARのようなセンサトピックはBEST_EFFORTで配信されることが多いため合わせる
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._latest_msg: PointCloud2 | None = None

        self._sub = self.create_subscription(
            PointCloud2, POINTCLOUD_TOPIC, self._pointcloud_callback, qos
        )

        self._save_srv = self.create_service(
            Trigger, SAVE_SERVICE_NAME, self._save_service_callback
        )

        os.makedirs(BASE_SAVE_DIR, exist_ok=True)

        self.get_logger().info(
            f"PointCloudSaverNode started. Subscribing to '{POINTCLOUD_TOPIC}', "
            f"save service: '/{SAVE_SERVICE_NAME}'"
        )

        # 自動保存モード
        self._auto_save = False
        self._timer = self.create_timer(1.0, self._timer_callback)  # 1秒周期
        # 自動保存開始サービス
        self._start_srv = self.create_service(
            Trigger, START_AUTOSAVE_SERVICE, self._start_callback
        )
        # 自動保存停止サービス
        self._stop_srv = self.create_service(
            Trigger, STOP_AUTOSAVE_SERVICE, self._stop_callback
        )

    def _stop_callback(self, request, response):
        # 自動保存停止
        self._auto_save = False
        response.success = True
        response.message = "Recording stopped"
        return response

    def _start_callback(self, request, response):
        # 自動保存開始
        self._auto_save = True
        response.success = True
        response.message = "Recording started"
        return response

    def _timer_callback(self):
        if not self._auto_save:  # Falseなら何もしない
            return
        if self._latest_msg is None:  # Noneなら何もしない
            return

        try:
            filepath = self._save_current_frame(self._latest_msg)
            # self.get_logger().info(f'auto saved:{filepath}')
        except Exception as e:
            self.get_logger().error(str(e))

    def _pointcloud_callback(self, msg: PointCloud2) -> None:
        # 最新フレームのみ保持(上書き)。過去フレームは保持しない。
        self._latest_msg = msg

    def _save_service_callback(self, request, response):
        # 保存サービスのレスポンス
        if self._latest_msg is None:
            response.success = False
            response.message = "No point cloud received yet."
            self.get_logger().warn(response.message)
            return response

        try:
            filepath = self._save_current_frame(self._latest_msg)
            response.success = True
            response.message = filepath
            self.get_logger().info(f"Saved point cloud to: {filepath}")
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"Failed to save point cloud: {exc}"
            self.get_logger().error(response.message)

        return response

    # ------------------------------------------------------------------
    def _save_current_frame(self, msg: PointCloud2) -> str:
        points = self._decode_pointcloud2(msg)  # (N, 4) float32, 変換なし

        trigger_time = datetime.now()
        date_dir = os.path.join(BASE_SAVE_DIR, trigger_time.strftime("%Y%m%d"))
        os.makedirs(date_dir, exist_ok=True)

        filename = trigger_time.strftime("pointcloud_%H%M%S_%f")[:-3] + ".npz"
        filepath = os.path.join(date_dir, filename)

        # スタンプはフレーム単位の値なのでスカラーとして1つだけ保持する。
        # int64のまま保存し、CSV版のような全行への複製はしない。
        stamp_sec = np.int64(msg.header.stamp.sec)
        stamp_nanosec = np.int64(msg.header.stamp.nanosec)

        np.savez(
            filepath,
            points=points,
            stamp_sec=stamp_sec,
            stamp_nanosec=stamp_nanosec,
        )

        return filepath

    # ------------------------------------------------------------------
    @staticmethod
    def _decode_pointcloud2(msg: PointCloud2) -> np.ndarray:
        """PointCloud2.data (bytes) を (N, 4) の x,y,z,intensity配列に変換する。

        x,y,z,intensityがすべてfloat32でoffset 0/4/8/12、point_step==16
        (パディングなし)の場合に限り、この単純なfrombuffer変換が正しく動作する。
        別センサ・別ドライバ設定でフィールド構成が変わった場合は要修正。
        """
        if msg.point_step != EXPECTED_POINT_STEP:
            raise ValueError(
                f"Unexpected point_step={msg.point_step} "
                f"(expected {EXPECTED_POINT_STEP}). "
                "PointCloud2 field layout may have changed; decoding logic "
                "needs to be updated."
            )

        raw = np.frombuffer(msg.data, dtype=np.float32)
        n_points = msg.width * msg.height
        points = raw.reshape(n_points, POINTS_PER_ROW)
        return points


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()