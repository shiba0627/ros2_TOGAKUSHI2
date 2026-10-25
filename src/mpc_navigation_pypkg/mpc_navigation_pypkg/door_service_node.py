#!/usr/bin/env python3
"""出入口候補検出サービス (単一ファイル)。

  python3 door_service_node.py [--ros-args -p topic:=/rslidar_points -p reference_width:=0.8 ...]

/rslidar_points の最新1フレームを保持し、サービス /detect_doors (std_srvs/Trigger) が
呼ばれた時点でそのフレームから出入口候補トップNを計算して返す。
カスタムsrvを作らず済むよう、結果は response.message に JSON 文字列で入れる:
  {"frame_id": str, "num_clusters": int, "points_cm": [x0,y0,x1,y1,...] (cm整数),
   "candidates": [{"rank","p1","p2","center","width","width_error","goal_yaw"}, ...]}
座標は点群フレーム基準 (原点=LiDAR, +x=前方, +y=左)。
"""

import json
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2, PointField
from sklearn.cluster import DBSCAN
from std_srvs.srv import Trigger

_DTYPES = {
    PointField.INT8: np.int8, PointField.UINT8: np.uint8,
    PointField.INT16: np.int16, PointField.UINT16: np.uint16,
    PointField.INT32: np.int32, PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32, PointField.FLOAT64: np.float64,
}


# ----------------------------------------------------------------------
# PointCloud2 -> numpy
# ----------------------------------------------------------------------
def cloud_to_xyz(msg):
    """(x, y, z) の float32 配列3つ。x/y/zが無ければ None。"""
    names, formats, offsets = [], [], []
    for f in msg.fields:
        if f.name in ("x", "y", "z") and f.datatype in _DTYPES:
            names.append(f.name)
            formats.append(_DTYPES[f.datatype])
            offsets.append(f.offset)
    if not {"x", "y", "z"}.issubset(names):
        return None
    dtype = np.dtype({"names": names, "formats": formats,
                      "offsets": offsets, "itemsize": msg.point_step})
    if msg.is_bigendian:
        dtype = dtype.newbyteorder(">")
    n = msg.width * msg.height
    if n == 0:
        e = np.empty(0, dtype=np.float32)
        return e, e, e
    arr = np.frombuffer(msg.data, dtype=dtype, count=n)
    return tuple(arr[k].astype(np.float32) for k in ("x", "y", "z"))


# ----------------------------------------------------------------------
# 検出ロジック (ROS非依存)
# ----------------------------------------------------------------------
def filter_points(x, y, z, z_min, z_max, range_m, voxel=0.0):
    m = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    m &= (z >= z_min) & (z <= z_max)
    m &= (np.abs(x) < range_m) & (np.abs(y) < range_m)
    xy = np.column_stack((x[m], y[m])).astype(np.float64)
    if voxel > 0.0 and len(xy):
        kx = np.floor(xy[:, 0] / voxel).astype(np.int64)
        ky = np.floor(xy[:, 1] / voxel).astype(np.int64)
        _, idx = np.unique(kx * 2_000_003 + ky, return_index=True)
        xy = xy[np.sort(idx)]
    return xy


def goal_yaw_from_door(p1, p2):
    """ドアに垂直な進行方向(ロボット→ドアを抜ける向き) [rad]。元のcalculate_goalと同じ規則。"""
    cx, cy = (p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0
    nx, ny = -(p2[1] - p1[1]), p2[0] - p1[0]
    if cx * nx + cy * ny < 0:
        nx, ny = -nx, -ny
    return math.atan2(ny, nx)


def find_door_candidates(xy, eps, min_samples, min_cluster_points,
                         door_min, door_max, reference_width, top_n):
    """(候補dictのリスト(順位順), DBSCANクラスタ数)。"""
    if len(xy) == 0:
        return [], 0
    labels = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(xy)
    ids = [int(l) for l in np.unique(labels) if l != -1]
    n_clusters = len(ids)
    clusters = [xy[labels == l] for l in ids]
    clusters = [c for c in clusters if len(c) >= min_cluster_points]
    m = len(clusters)
    if m < 2:
        return [], n_clusters

    mins = np.array([c.min(axis=0) for c in clusters])
    maxs = np.array([c.max(axis=0) for c in clusters])
    trees = [None] * m
    raw = []
    for i in range(m):
        for j in range(i + 1, m):
            gap = np.maximum(0.0, np.maximum(mins[i] - maxs[j], mins[j] - maxs[i]))
            if math.hypot(gap[0], gap[1]) > door_max:   # bboxが遠ければ省略
                continue
            if trees[i] is None:
                trees[i] = cKDTree(clusters[i])
            d, idx = trees[i].query(clusters[j])
            k = int(np.argmin(d))
            dist = float(d[k])
            if not (door_min <= dist <= door_max):
                continue
            p1 = [float(v) for v in clusters[i][idx[k]]]
            p2 = [float(v) for v in clusters[j][k]]
            center = [(p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0]
            raw.append((abs(dist - reference_width), math.hypot(*center), p1, p2, center, dist))

    raw.sort(key=lambda r: (r[0], r[1]))   # 幅誤差が小さい順、同値なら近い順
    cands = [
        {"rank": rank, "p1": p1, "p2": p2, "center": center, "width": dist,
         "width_error": err, "goal_yaw": goal_yaw_from_door(p1, p2)}
        for rank, (err, _, p1, p2, center, dist) in enumerate(raw[:max(0, top_n)], start=1)
    ]
    return cands, n_clusters


# ----------------------------------------------------------------------
# ROS2 ノード
# ----------------------------------------------------------------------
class DoorServiceNode(Node):
    def __init__(self):
        super().__init__("door_service")
        for name, default in [
            ("topic", "/rslidar_points"),
            ("z_min", -1.4), ("z_max", 0.0),       # 高さフィルタ [m]
            ("range", 10.0),                       # 使う半径(正方形の半幅) [m]
            ("voxel", 0.0),                        # >0で簡易ダウンサンプル [m]
            ("eps", 0.3), ("min_samples", 4),      # DBSCAN
            ("min_cluster_points", 5),
            ("door_min", 0.55), ("door_max", 1.0), # ドア候補とみなすクラスタ間距離 [m]
            ("reference_width", 0.8),              # 順位付けの基準幅 [m]
            ("top_n", 3),
            ("max_cloud_age_sec", 2.0),            # これより古い点群は使わない (0で無効)
        ]:
            self.declare_parameter(name, default)

        self._lock = threading.Lock()
        self._latest = None   # (PointCloud2, 受信時刻)
        topic = self.get_parameter("topic").value
        self.create_subscription(PointCloud2, topic, self._on_cloud, qos_profile_sensor_data)
        self.create_service(Trigger, "detect_doors", self._on_detect)
        self.get_logger().info(f"subscribe: {topic} / service: detect_doors")

    def _p(self, name):
        return self.get_parameter(name).value

    def _on_cloud(self, msg):
        with self._lock:   # 変換はサービス呼び出し時に行う
            self._latest = (msg, time.monotonic())

    def _on_detect(self, request, response):
        try:
            ok, text = self._detect()
        except Exception as e:
            self.get_logger().error(f"detect failed: {e!r}")
            ok, text = False, json.dumps({"error": f"internal error: {e!r}"})
        response.success = ok
        response.message = text
        return response

    def _detect(self):
        t0 = time.monotonic()
        with self._lock:
            latest = self._latest
        if latest is None:
            return False, json.dumps({"error": "点群をまだ受信していません"})
        msg, t_rx = latest
        age = time.monotonic() - t_rx
        max_age = float(self._p("max_cloud_age_sec"))
        if max_age > 0.0 and age > max_age:
            return False, json.dumps({"error": f"点群が古すぎます ({age:.1f} s)"})
        xyz = cloud_to_xyz(msg)
        if xyz is None:
            return False, json.dumps({"error": "x/y/z フィールドが見つかりません"})

        xy = filter_points(*xyz, float(self._p("z_min")), float(self._p("z_max")),
                           float(self._p("range")), float(self._p("voxel")))
        cands, n_clusters = find_door_candidates(
            xy, float(self._p("eps")), int(self._p("min_samples")),
            int(self._p("min_cluster_points")), float(self._p("door_min")),
            float(self._p("door_max")), float(self._p("reference_width")),
            int(self._p("top_n")))

        ms = (time.monotonic() - t0) * 1000.0
        info = f"{len(cands)} 件 (点群 {len(xy)} 点, {n_clusters} クラスタ, {ms:.0f} ms)"
        self.get_logger().info(info)
        payload = {
            "frame_id": msg.header.frame_id,
            "num_clusters": n_clusters,
            "info": info,
            "points_cm": np.rint(xy * 100.0).astype(np.int32).ravel().tolist(),
            "candidates": cands,
        }
        return len(cands) > 0, json.dumps(payload, separators=(",", ":"))


def main(args=None):
    rclpy.init(args=args)
    node = DoorServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()