#!/usr/bin/env python3
"""経路生成サービス (単一ファイル。door_service_node.py と同じフォルダに置くこと)。

  python3 door_planner_node.py [--ros-args -p timeout_sec:=10.0 -p collision_margin:=0.05 ...]

サービス /plan_path (nav_msgs/GetPlan) を提供する。
  request.start / request.goal : 点群フレーム(原点=LiDAR, +x=前方, +y=左)上の姿勢 (x, y, yaw)
  request.tolerance            : ゴール位置許容誤差 [m] (0以下ならパラメータ position_tolerance)
  response.plan                : 経路 (nav_msgs/Path)。各ステップ(dt刻み)の姿勢。

/rslidar_points の最新1フレームを障害物として使い、元プランナーと同じ運動モデル
(前進 / その場左旋回 / その場右旋回, 速度一定) で A* 探索する。
経路が見つからない場合は、ゴールに最も近づいた経路を返す (return_partial:=false なら空のPath)。
開始姿勢で車体領域の内側にある点は、障害物として扱わず無視する。
"""

import heapq
import itertools
import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2

from door_service_node import cloud_to_xyz, filter_points


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def angle_diff(a, b):
    return normalize_angle(a - b)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ----------------------------------------------------------------------
# A* プランナー (ROS非依存)
# ----------------------------------------------------------------------
class TrajectoryPlanner:
    def __init__(self, xy, *, v=0.219, omega=0.267, dt=0.5, xy_res=0.05,
                 theta_res=math.radians(3.0), pos_tol=0.15, ang_tol=math.radians(10.0),
                 max_nodes=100000, substeps=12, margin=0.0, search_margin=2.0,
                 robot_x_min=0.0, robot_x_max=0.9, robot_half_width=0.275,
                 timeout=10.0, disable_collision=False):
        self.xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
        self.tree = cKDTree(self.xy) if len(self.xy) else None
        self.v, self.omega, self.dt = v, omega, dt
        self.xy_res, self.theta_res = xy_res, theta_res
        self.pos_tol, self.ang_tol = pos_tol, ang_tol
        self.max_nodes, self.substeps = max_nodes, substeps
        self.search_margin, self.timeout = search_margin, timeout
        self.disable_collision = disable_collision
        # 衝突判定用のロボット矩形 (マージン込み)
        self.bx0, self.bx1 = robot_x_min - margin, robot_x_max + margin
        self.by0, self.by1 = -robot_half_width - margin, robot_half_width + margin
        # 回転中心(LiDAR原点)から矩形の最遠点までの半径 + 1ステップの並進量
        far_x = max(abs(self.bx0), abs(self.bx1))
        self.r_query = math.hypot(far_x, max(abs(self.by0), abs(self.by1))) + v * dt
        # 前進と、LiDAR原点中心のその場旋回のみ (後進なし、並進と旋回の同時動作なし)
        self.actions = [(v, 0.0), (0.0, omega), (0.0, -omega)]
        self.goal = (0.0, 0.0, 0.0)

    # ---- 開始時に車体内にある点を除外 ----
    def drop_points_inside_body(self, x, y, th):
        """開始姿勢で車体領域(マージン込み)の内側にある点を障害物から外す。除外した点数を返す。
        車いす自身や取り付け物がLiDARに映り込んでいる場合に、始めから衝突扱いになるのを防ぐ。"""
        if self.tree is None:
            return 0
        d = self.xy - np.array([x, y])
        c, s = math.cos(th), math.sin(th)
        lx = c * d[:, 0] + s * d[:, 1]
        ly = -s * d[:, 0] + c * d[:, 1]
        inside = (lx >= self.bx0) & (lx <= self.bx1) & (ly >= self.by0) & (ly <= self.by1)
        n = int(inside.sum())
        if n:
            self.xy = self.xy[~inside]
            self.tree = cKDTree(self.xy) if len(self.xy) else None
        return n

    # ---- 衝突 ----
    def _hit(self, cx, cy, nx, ny, nth, radius):
        """姿勢列(nx,ny,nth)のいずれかで矩形が点群と重なるか。付近の点だけで判定する。"""
        if self.disable_collision or self.tree is None:
            return False
        idx = self.tree.query_ball_point([cx, cy], radius)
        if not idx:
            return False
        p = self.xy[idx]
        dx = p[:, 0][None, :] - nx[:, None]
        dy = p[:, 1][None, :] - ny[:, None]
        c, s = np.cos(nth)[:, None], np.sin(nth)[:, None]
        lx = c * dx + s * dy        # world -> robot
        ly = -s * dx + c * dy
        return bool(((lx >= self.bx0) & (lx <= self.bx1) & (ly >= self.by0) & (ly <= self.by1)).any())

    def pose_collides(self, x, y, th):
        return self._hit(x, y, np.array([x]), np.array([y]), np.array([th]), self.r_query)

    def transition_collides(self, x, y, th, v, om):
        t = np.linspace(self.dt / self.substeps, self.dt, self.substeps)
        nth = th + om * t
        if abs(om) < 1e-9:
            nx, ny = x + v * math.cos(th) * t, y + v * math.sin(th) * t
        else:
            r = v / om
            nx, ny = x + r * (np.sin(nth) - math.sin(th)), y - r * (np.cos(nth) - math.cos(th))
        return self._hit(x, y, nx, ny, nth, self.r_query)

    # ---- 運動モデル / 評価 ----
    def step(self, x, y, th, v, om):
        if abs(om) < 1e-9:
            return x + v * math.cos(th) * self.dt, y + v * math.sin(th) * self.dt, th
        nth = th + om * self.dt
        r = v / om
        return (x + r * (math.sin(nth) - math.sin(th)),
                y - r * (math.cos(nth) - math.cos(th)), normalize_angle(nth))

    def key(self, x, y, th):
        return (round(x / self.xy_res), round(y / self.xy_res),
                round(normalize_angle(th) / self.theta_res))

    def heuristic(self, x, y, th):
        gx, gy, gth = self.goal
        pos = math.hypot(x - gx, y - gy) / (self.v * self.dt)
        ang = abs(angle_diff(th, gth)) / (self.omega * self.dt)
        return pos + ang

    # ---- 探索 ----
    def search(self, start, goal):
        """戻り値: dict(path=(N,3), controls=[(v,omega)], reached, expanded, reason)"""
        self.goal = goal
        gx, gy, gth = goal
        sx, sy, sth = start
        t0 = time.monotonic()

        x_lo, x_hi = min(sx, gx) - self.search_margin, max(sx, gx) + self.search_margin
        y_lo, y_hi = min(sy, gy) - self.search_margin, max(sy, gy) + self.search_margin

        ignored = self.drop_points_inside_body(sx, sy, sth)
        if self.pose_collides(sx, sy, sth):   # 除外後は通常起きない (念のため)
            return dict(path=np.array([[sx, sy, sth]]), controls=[], reached=False,
                        expanded=0, ignored=ignored, reason="初期姿勢で衝突しています")

        k0 = self.key(sx, sy, sth)
        states = {k0: (sx, sy, sth)}
        parents = {k0: None}
        used = {k0: None}
        g_score = {k0: 0}
        closed = set()
        tie = itertools.count()
        heap = [(self.heuristic(sx, sy, sth), next(tie), k0)]
        closest_key, closest_score = k0, float("inf")
        expanded, reason = 0, "探索空間を使い切りました (経路なし)"

        while heap:
            if expanded >= self.max_nodes:
                reason = "最大探索ノード数に達しました"
                break
            if expanded % 256 == 0 and time.monotonic() - t0 > self.timeout:
                reason = "タイムアウトしました"
                break

            _, _, ck = heapq.heappop(heap)
            if ck in closed:
                continue
            closed.add(ck)
            x, y, th = states[ck]
            expanded += 1

            pe = math.hypot(x - gx, y - gy)
            ae = abs(angle_diff(th, gth))
            score = pe + 0.5 * ae
            if score < closest_score:
                closest_score, closest_key = score, ck

            if pe <= self.pos_tol and ae <= self.ang_tol:
                path, controls = self._reconstruct(ck, states, parents, used)
                return dict(path=path, controls=controls, reached=True,
                            expanded=expanded, ignored=ignored, reason="ゴールに到達")

            for v, om in self.actions:
                nx, ny, nth = self.step(x, y, th, v, om)
                if not (x_lo <= nx <= x_hi and y_lo <= ny <= y_hi):
                    continue
                if self.transition_collides(x, y, th, v, om):
                    continue
                nk = self.key(nx, ny, nth)
                g = g_score[ck] + 1
                if nk in g_score and g >= g_score[nk]:
                    continue
                g_score[nk] = g
                states[nk] = (nx, ny, nth)
                parents[nk] = ck
                used[nk] = (v, om)
                heapq.heappush(heap, (g + self.heuristic(nx, ny, nth), next(tie), nk))

        path, controls = self._reconstruct(closest_key, states, parents, used)
        return dict(path=path, controls=controls, reached=False, expanded=expanded,
                    ignored=ignored, reason=reason)

    @staticmethod
    def _reconstruct(key, states, parents, used):
        path, controls = [], []
        while key is not None:
            path.append(states[key])
            if used[key] is not None:
                controls.append(used[key])
            key = parents[key]
        path.reverse()
        controls.reverse()
        return np.array(path), controls


# ----------------------------------------------------------------------
# ROS2 ノード
# ----------------------------------------------------------------------
class DoorPlannerNode(Node):
    def __init__(self):
        super().__init__("door_planner")
        for name, default in [
            ("topic", "/rslidar_points"),
            ("z_min", -1.4), ("z_max", 0.0), ("range", 10.0),   # 点群フィルタ (検出ノードと揃える)
            ("max_cloud_age_sec", 2.0),
            ("forward_speed", 0.219), ("angular_speed", 0.267), ("dt", 0.5),
            ("xy_resolution", 0.05), ("theta_resolution_deg", 3.0),
            ("position_tolerance", 0.15), ("angle_tolerance_deg", 10.0),
            ("max_search_nodes", 100000), ("timeout_sec", 10.0),
            ("collision_substeps", 12), ("collision_margin", 0.0), ("search_margin", 2.0),
            ("robot_x_min", 0.0), ("robot_x_max", 0.9), ("robot_half_width", 0.275),
            ("disable_collision", False),
            ("return_partial", True),     # 未到達時にゴールへ最も近づいた経路を返す
        ]:
            self.declare_parameter(name, default)

        self._lock = threading.Lock()
        self._latest = None
        topic = self.get_parameter("topic").value
        self.create_subscription(PointCloud2, topic, self._on_cloud, qos_profile_sensor_data)
        self.create_service(GetPlan, "plan_path", self._on_plan)
        self.get_logger().info(f"subscribe: {topic} / service: plan_path")

    def _p(self, name):
        return self.get_parameter(name).value

    def _on_cloud(self, msg):
        with self._lock:
            self._latest = (msg, time.monotonic())

    def _on_plan(self, request, response):
        try:
            response.plan = self._plan(request)
        except Exception as e:
            self.get_logger().error(f"plan failed: {e!r}")
            response.plan = Path()
        return response

    def _plan(self, request):
        empty = Path()
        with self._lock:
            latest = self._latest
        if latest is None:
            self.get_logger().warn("点群をまだ受信していません")
            return empty
        msg, t_rx = latest
        max_age = float(self._p("max_cloud_age_sec"))
        if max_age > 0.0 and time.monotonic() - t_rx > max_age:
            self.get_logger().warn("点群が古すぎます")
            return empty
        xyz = cloud_to_xyz(msg)
        if xyz is None:
            self.get_logger().warn("x/y/z フィールドが見つかりません")
            return empty

        xy = filter_points(*xyz, float(self._p("z_min")), float(self._p("z_max")),
                           float(self._p("range")))
        start = (request.start.pose.position.x, request.start.pose.position.y,
                 yaw_from_quat(request.start.pose.orientation))
        goal = (request.goal.pose.position.x, request.goal.pose.position.y,
                yaw_from_quat(request.goal.pose.orientation))
        tol = float(request.tolerance)
        if tol <= 0.0:
            tol = float(self._p("position_tolerance"))

        planner = TrajectoryPlanner(
            xy,
            v=float(self._p("forward_speed")), omega=float(self._p("angular_speed")),
            dt=float(self._p("dt")), xy_res=float(self._p("xy_resolution")),
            theta_res=math.radians(float(self._p("theta_resolution_deg"))),
            pos_tol=tol, ang_tol=math.radians(float(self._p("angle_tolerance_deg"))),
            max_nodes=int(self._p("max_search_nodes")), substeps=int(self._p("collision_substeps")),
            margin=float(self._p("collision_margin")), search_margin=float(self._p("search_margin")),
            robot_x_min=float(self._p("robot_x_min")), robot_x_max=float(self._p("robot_x_max")),
            robot_half_width=float(self._p("robot_half_width")),
            timeout=float(self._p("timeout_sec")),
            disable_collision=bool(self._p("disable_collision")))

        t0 = time.monotonic()
        res = planner.search(start, goal)
        ms = (time.monotonic() - t0) * 1000.0
        path = res["path"]
        self.get_logger().info(
            f"{'到達' if res['reached'] else '未到達'}: {res['reason']} "
            f"(steps={len(path) - 1}, expanded={res['expanded']}, 車体内の点を{res['ignored']}点無視, {ms:.0f} ms, "
            f"goal=({goal[0]:.2f}, {goal[1]:.2f}, {math.degrees(goal[2]):.0f} deg))")
        if not res["reached"] and not bool(self._p("return_partial")):
            return empty

        out = Path()
        out.header.frame_id = msg.header.frame_id
        out.header.stamp = msg.header.stamp
        for x, y, th in path:
            ps = PoseStamped()
            ps.header = out.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
            out.poses.append(ps)
        return out


def main(args=None):
    rclpy.init(args=args)
    node = DoorPlannerNode()
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