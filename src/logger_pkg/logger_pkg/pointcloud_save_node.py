#!/usr/bin/env python3
"""
pointcloud_saver_node.py

/rslidar_points (sensor_msgs/PointCloud2) を購読し、最新1フレームのみを
バッファ保持する。ROS2サービス (std_srvs/Trigger) が呼ばれた時点で、
その時のバッファ内容をCSVとしてディスクに保存する。

想定メッセージフォーマット:
    fields: x, y, z, intensity (すべて FLOAT32, offset 0/4/8/12)
    point_step: 16 (パディングなし、4 floatが連続)

保存先:
    ~/togakushi2_data/pointclouds/YYYYMMDD/pointcloud_HHMMSS_mmm.csv
    (YYYYMMDD, HHMMSS_mmm はサービス呼び出し=保存トリガー時刻)

CSV列:
    x, y, z, intensity, stamp_sec, stamp_nanosec
    (stamp_sec/nanosecは点群フレーム自体のheader.stamp。point-wiseの
     timestampは元データに存在しないため、フレーム単位の値を全行に付与する)

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
BASE_SAVE_DIR = os.path.expanduser("~/togakushi2_data/pointclouds")
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

    # ------------------------------------------------------------------
    def _pointcloud_callback(self, msg: PointCloud2) -> None:
        # 最新フレームのみ保持(上書き)。過去フレームは保持しない。
        self._latest_msg = msg

    # ------------------------------------------------------------------
    def _save_service_callback(self, request, response):
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
        points = self._decode_pointcloud2(msg)

        trigger_time = datetime.now()
        date_dir = os.path.join(BASE_SAVE_DIR, trigger_time.strftime("%Y%m%d"))
        os.makedirs(date_dir, exist_ok=True)

        filename = trigger_time.strftime("pointcloud_%H%M%S_%f")[:-3] + ".csv"
        filepath = os.path.join(date_dir, filename)

        stamp_sec = np.full((points.shape[0], 1), msg.header.stamp.sec, dtype=np.int64)
        stamp_nanosec = np.full(
            (points.shape[0], 1), msg.header.stamp.nanosec, dtype=np.int64
        )

        out = np.hstack([points.astype(np.float64), stamp_sec, stamp_nanosec])

        header = "x,y,z,intensity,stamp_sec,stamp_nanosec"
        np.savetxt(
            filepath,
            out,
            delimiter=",",
            header=header,
            comments="",
            fmt=["%.6f", "%.6f", "%.6f", "%.6f", "%d", "%d"],
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