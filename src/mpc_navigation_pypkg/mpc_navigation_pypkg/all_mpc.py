import math
from itertools import combinations

import numpy as np
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class GoalCalculator(Node):

    def __init__(self):
        super().__init__('goal_calculator')

        # ==========================================================
        # Subscriber
        # ==========================================================

        self.filtered_sub = self.create_subscription(
            PointCloud2,
            '/mpc/filtered_points',
            self.filtered_points_callback,
            10
        )

        # ==========================================================
        # DBSCAN parameters
        # ==========================================================

        self.eps = 0.3
        self.min_samples = 4

        # ドアとして許容するクラスタ間距離 [m]
        self.door_min_distance = 0.55
        self.door_max_distance = 1.0

        # ==========================================================
        # 車いす parameters
        # ==========================================================

        self.robot_width = 0.55
        self.robot_length = 1.0

        # LiDAR座標系での車いす原点
        #
        #        前方
        #         ↑
        #     ┌───────┐
        #     │       │
        #     │   ●   │ ← 原点
        #     │       │
        #     └───────┘
        #
        # 今回は原点から前方に1.0 m
        # 横方向±0.275 m
        #
        self.robot_x_min = 0.0
        self.robot_x_max = self.robot_length
        self.robot_y_min = -self.robot_width / 2.0
        self.robot_y_max = self.robot_width / 2.0

        # 衝突判定用マージン [m]
        self.collision_margin = 0

        # ==========================================================
        # 軌跡生成 parameters
        # ==========================================================

        # 車いす速度
        self.forward_speed = 0.219

        # 最大角速度
        self.max_angular_velocity = 0.267

        # シミュレーション時間
        self.simulation_time = 12.0

        # シミュレーション刻み
        self.dt = 0.2

        # ==========================================================
        # 軌道探索 parameters
        # ==========================================================

        # 状態の離散化分解能
        self.xy_resolution = 0.05
        self.theta_resolution = math.radians(5.0)

        # 探索する最大状態数
        # 動作時間の制約ではない
        self.max_search_nodes = 100000

        # 1ステップ内の衝突判定数
        self.collision_substeps = 5


        # ==========================================================
        # Goal parameters
        # ==========================================================

        self.position_tolerance = 0.15
        self.angle_tolerance = math.radians(10.0)

        # 評価関数重み
        self.position_weight = 1.0
        self.angle_weight = 0.5
        self.path_length_weight = 0.05

        # ==========================================================
        # Variables
        # ==========================================================

        self.xy = None
        self.labels = None
        self.cluster_num = 0
        self.xyID = None
        self.door_data = None

        self.center_x = None
        self.center_y = None
        self.goal_angle = None

        self.best_path = None

        self.get_logger().info(
            'goal_calculator started.'
        )
        self.max_simulation_time = 30.0

    # ==========================================================
    # PointCloud2 callback
    # ==========================================================

    def filtered_points_callback(self, msg):

        points = point_cloud2.read_points_numpy(
            msg,
            field_names=['x', 'y']
        )

        if points is None or len(points) == 0:
            self.get_logger().warn(
                'No points received.'
            )
            return

        # NaN除去
        points = points[
            np.isfinite(points[:, 0]) &
            np.isfinite(points[:, 1])
        ]

        if len(points) == 0:
            return

        self.xy = points

        # ==========================================================
        # DBSCAN
        # ==========================================================

        db = DBSCAN(
            eps=self.eps,
            min_samples=self.min_samples
        )

        db.fit(self.xy)

        self.labels = db.labels_

        valid_labels = self.labels[
            self.labels != -1
        ]

        if len(valid_labels) == 0:
            self.get_logger().warn(
                'No clusters detected.'
            )
            return

        self.cluster_num = int(
            valid_labels.max()
        ) + 1

        self.xyID = np.column_stack(
            (
                self.xy[:, 0],
                self.xy[:, 1],
                self.labels
            )
        )

        # ==========================================================
        # 出入口探索
        # ==========================================================

        self.split_data()

        if self.door_data is None:
            return

        # ==========================================================
        # ゴール計算
        # ==========================================================

        self.calculate_goal()

        # ==========================================================
        # 経路生成
        # ==========================================================

        self.generate_best_path()

        # ==========================================================
        # プロット
        # ==========================================================

        if self.best_path is not None:
            self.plot_result()

    # ==========================================================
    # 出入口検出
    # ==========================================================

    def split_data(self):

        elements = np.unique(
            self.labels[self.labels != -1]
        )

        best_door = None
        best_distance = float('inf')

        self.get_logger().info(
            f'Number of clusters: {len(elements)}'
        )

        # ==========================================================
        # 各クラスタの点数
        # ==========================================================

        for cluster_id in elements:

            cluster = self.xyID[
                self.xyID[:, 2] == cluster_id
            ]

            self.get_logger().info(
                f'Cluster {cluster_id}: '
                f'{len(cluster)} points'
            )

        # ==========================================================
        # クラスタ組み合わせ
        # ==========================================================

        for cluster1_id, cluster2_id in combinations(
            elements,
            2
        ):

            cluster1 = self.xyID[
                self.xyID[:, 2] == cluster1_id
            ]

            cluster2 = self.xyID[
                self.xyID[:, 2] == cluster2_id
            ]

            # 小さすぎるクラスタを除外
            if len(cluster1) < 5:
                continue

            if len(cluster2) < 5:
                continue

            # 最短距離
            shortest_xy, distance = self.cluster_range(
                cluster1,
                cluster2
            )

            self.get_logger().info(
                f'Cluster pair '
                f'{cluster1_id}-{cluster2_id}: '
                f'distance={distance:.3f} m'
            )

            # ======================================================
            # ドア候補
            # ======================================================

            if (
                self.door_min_distance
                <= distance
                <= self.door_max_distance
            ):

                if distance < best_distance:

                    best_distance = distance

                    best_door = np.array([
                        shortest_xy[0][0],
                        shortest_xy[0][1],
                        shortest_xy[1][0],
                        shortest_xy[1][1],
                        distance
                    ])

        # ==========================================================
        # 結果
        # ==========================================================

        self.door_data = best_door

        if self.door_data is not None:

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                'Door detected'
            )

            self.get_logger().info(
                f'P1 = '
                f'({self.door_data[0]:.3f}, '
                f'{self.door_data[1]:.3f})'
            )

            self.get_logger().info(
                f'P2 = '
                f'({self.door_data[2]:.3f}, '
                f'{self.door_data[3]:.3f})'
            )

            self.get_logger().info(
                f'distance = '
                f'{self.door_data[4]:.3f} m'
            )

            self.get_logger().info(
                '========================================'
            )

        else:

            self.get_logger().warn(
                'No door detected.'
            )

    # ==========================================================
    # 2クラスタ間の最短距離
    # ==========================================================

    def cluster_range(self, cls_1, cls_2):

        shortest_distance = float('inf')
        shortest_xy = None

        for p1 in cls_1:

            for p2 in cls_2:

                distance = self.range_calculate(
                    p1[0],
                    p2[0],
                    p1[1],
                    p2[1]
                )

                if distance < shortest_distance:

                    shortest_distance = distance

                    shortest_xy = np.array([
                        [p1[0], p1[1]],
                        [p2[0], p2[1]]
                    ])

        return shortest_xy, shortest_distance

    # ==========================================================
    # 2点間距離
    # ==========================================================

    def range_calculate(
        self,
        x1,
        x2,
        y1,
        y2
    ):

        return math.sqrt(
            (x1 - x2) ** 2 +
            (y1 - y2) ** 2
        )

    # ==========================================================
    # ゴール計算
    # ==========================================================

    def calculate_goal(self):

        x1 = self.door_data[0]
        y1 = self.door_data[1]

        x2 = self.door_data[2]
        y2 = self.door_data[3]

        # ==========================================================
        # ドア中心
        # ==========================================================

        self.center_x = (
            x1 + x2
        ) / 2.0

        self.center_y = (
            y1 + y2
        ) / 2.0

        # ==========================================================
        # ドア方向
        # ==========================================================

        door_angle = math.atan2(
            y2 - y1,
            x2 - x1
        )

        # ==========================================================
        # ドアに垂直な方向
        # ==========================================================

        self.goal_angle = (
            door_angle +
            math.pi / 2.0
        )

        # -π ～ π
        self.goal_angle = math.atan2(
            math.sin(self.goal_angle),
            math.cos(self.goal_angle)
        )

        self.get_logger().info(
            'Goal: '
            f'center=({self.center_x:.3f}, '
            f'{self.center_y:.3f}), '
            f'angle={math.degrees(self.goal_angle):.1f} deg'
        )

    # ==========================================================
    # 候補軌跡生成
    # ==========================================================

        # ==========================================================
    # ゴール到達軌道生成
    # ==========================================================

    def generate_best_path(self):

        # ==========================================================
        # 初期化
        # ==========================================================

        self.best_path = None

        self.get_logger().info(
            'Start trajectory search.'
        )

        # ==========================================================
        # パラメータ
        # ==========================================================

        v = self.forward_speed
        omega_max = self.max_angular_velocity
        dt = self.dt

        # ==========================================================
        # 使用可能な角速度
        #
        # 各ステップで自由に変更可能
        #
        # ==========================================================

        actions = [
            -omega_max,
            0.0,
            omega_max
        ]

        # ==========================================================
        # ゴール判定
        # ==========================================================

        position_tolerance = self.position_tolerance
        angle_tolerance = self.angle_tolerance

        # ==========================================================
        # 状態離散化
        # ==========================================================
        #
        # 探索状態を
        #
        # (x, y, theta)
        #
        # で管理する。
        #
        # 同じような状態を何度も探索しない。
        #
        # ==========================================================

        xy_resolution = self.xy_resolution
        theta_resolution = self.theta_resolution

        # ==========================================================
        # 状態をキーに変換
        # ==========================================================

        def make_state_key(
            x,
            y,
            theta
        ):

            theta = math.atan2(
                math.sin(theta),
                math.cos(theta)
            )

            return (
                round(
                    x / xy_resolution
                ),
                round(
                    y / xy_resolution
                ),
                round(
                    theta / theta_resolution
                )
            )

        # ==========================================================
        # 初期状態
        # ==========================================================

        initial_state = np.array([
            0.0,
            0.0,
            0.0
        ])

        initial_key = make_state_key(
            0.0,
            0.0,
            0.0
        )

        # ==========================================================
        # 探索キュー
        #
        # BFS型で探索する。
        #
        # 今回は「最短時間」を求めることが目的ではない。
        # まず「到達可能な軌道を1本見つける」ことを優先する。
        #
        # ==========================================================

        from collections import deque

        queue = deque()

        queue.append(
            initial_key
        )

        # ==========================================================
        # 状態
        # ==========================================================

        states = {
            initial_key: initial_state
        }

        # ==========================================================
        # 親状態
        # ==========================================================

        parents = {
            initial_key: None
        }

        # ==========================================================
        # 使用した角速度
        # ==========================================================

        actions_used = {
            initial_key: None
        }

        # ==========================================================
        # 探索済み状態
        # ==========================================================

        visited = {
            initial_key
        }

        # ==========================================================
        # 最もゴールに近かった状態
        # ==========================================================

        closest_key = initial_key

        closest_distance = math.sqrt(
            self.center_x ** 2 +
            self.center_y ** 2
        )

        # ==========================================================
        # 探索数
        # ==========================================================

        expanded_nodes = 0

        # ==========================================================
        # 探索開始
        # ==========================================================

        while queue:

            # ------------------------------------------------------
            # 探索上限
            # ------------------------------------------------------

            if expanded_nodes >= self.max_search_nodes:

                self.get_logger().warn(
                    'Maximum search nodes reached.'
                )

                break

            # ------------------------------------------------------
            # 現在状態
            # ------------------------------------------------------

            current_key = queue.popleft()

            current_state = states[
                current_key
            ]

            x = current_state[0]
            y = current_state[1]
            theta = current_state[2]

            expanded_nodes += 1

            # ======================================================
            # ゴール距離
            # ======================================================

            position_error = math.sqrt(
                (
                    x -
                    self.center_x
                ) ** 2
                +
                (
                    y -
                    self.center_y
                ) ** 2
            )

            # ======================================================
            # 最も近い状態を保存
            # ======================================================

            if position_error < closest_distance:

                closest_distance = position_error
                closest_key = current_key

            # ======================================================
            # ゴール姿勢誤差
            # ======================================================

            angle_error = abs(
                self.angle_difference(
                    theta,
                    self.goal_angle
                )
            )

            # ======================================================
            # ゴール到達判定
            # ======================================================

            if (
                position_error
                <= position_tolerance
                and
                angle_error
                <= angle_tolerance
            ):

                # ==================================================
                # 軌道復元
                # ==================================================

                self.best_path = self.reconstruct_path(
                    current_key,
                    states,
                    parents
                )

                # ==================================================
                # ログ
                # ==================================================

                self.get_logger().info(
                    '========================================'
                )

                self.get_logger().info(
                    'Goal reached!'
                )

                self.get_logger().info(
                    f'Expanded nodes = '
                    f'{expanded_nodes}'
                )

                self.get_logger().info(
                    f'Path steps = '
                    f'{len(self.best_path) - 1}'
                )

                self.get_logger().info(
                    f'Final position = '
                    f'({x:.3f}, {y:.3f})'
                )

                self.get_logger().info(
                    f'Goal position = '
                    f'({self.center_x:.3f}, '
                    f'{self.center_y:.3f})'
                )

                self.get_logger().info(
                    f'Position error = '
                    f'{position_error:.3f} m'
                )

                self.get_logger().info(
                    f'Final angle = '
                    f'{math.degrees(theta):.2f} deg'
                )

                self.get_logger().info(
                    f'Goal angle = '
                    f'{math.degrees(self.goal_angle):.2f} deg'
                )

                self.get_logger().info(
                    f'Angle error = '
                    f'{math.degrees(angle_error):.2f} deg'
                )

                self.get_logger().info(
                    '========================================'
                )

                return

            # ======================================================
            # 次の3状態を展開
            # ======================================================

            for omega in actions:

                # --------------------------------------------------
                # 新しい角度
                # --------------------------------------------------

                new_theta = (
                    theta +
                    omega * dt
                )

                new_theta = math.atan2(
                    math.sin(new_theta),
                    math.cos(new_theta)
                )

                # --------------------------------------------------
                # 新しい位置
                #
                # 現在のtheta方向へ前進
                # --------------------------------------------------

                new_x = (
                    x +
                    v *
                    math.cos(theta) *
                    dt
                )

                new_y = (
                    y +
                    v *
                    math.sin(theta) *
                    dt
                )

                # --------------------------------------------------
                # 新しい状態
                # --------------------------------------------------

                new_state = np.array([
                    new_x,
                    new_y,
                    new_theta
                ])

                # ==================================================
                # 衝突判定
                #
                # 0.2秒後だけではなく、
                # その途中も確認する。
                # ==================================================

                collision = (
                    self.check_transition_collision(
                        x,
                        y,
                        theta,
                        new_x,
                        new_y,
                        new_theta
                    )
                )

                if collision:

                    continue

                # ==================================================
                # 状態キー
                # ==================================================

                new_key = make_state_key(
                    new_x,
                    new_y,
                    new_theta
                )

                # ==================================================
                # 既に探索済みなら無視
                # ==================================================

                if new_key in visited:

                    continue

                # ==================================================
                # 状態を保存
                # ==================================================

                visited.add(
                    new_key
                )

                states[new_key] = new_state

                parents[new_key] = current_key

                actions_used[new_key] = omega

                # ==================================================
                # キューへ追加
                # ==================================================

                queue.append(
                    new_key
                )

            # ======================================================
            # 探索状況
            # ======================================================

            if expanded_nodes % 1000 == 0:

                self.get_logger().info(
                    f'Expanded={expanded_nodes}, '
                    f'queue={len(queue)}, '
                    f'closest='
                    f'{closest_distance:.3f} m'
                )

        # ==========================================================
        # ゴール未到達
        # ==========================================================

        self.get_logger().warn(
            '========================================'
        )

        self.get_logger().warn(
            'Goal was not reached.'
        )

        self.get_logger().warn(
            f'Expanded nodes = '
            f'{expanded_nodes}'
        )

        # ==========================================================
        # 最も近かった軌道を保存
        # ==========================================================

        if closest_key is not None:

            self.best_path = self.reconstruct_path(
                closest_key,
                states,
                parents
            )

            final_state = (
                self.best_path[-1]
            )

            final_position_error = math.sqrt(
                (
                    final_state[0] -
                    self.center_x
                ) ** 2
                +
                (
                    final_state[1] -
                    self.center_y
                ) ** 2
            )

            final_angle_error = abs(
                self.angle_difference(
                    final_state[2],
                    self.goal_angle
                )
            )

            self.get_logger().warn(
                f'Closest position = '
                f'({final_state[0]:.3f}, '
                f'{final_state[1]:.3f})'
            )

            self.get_logger().warn(
                f'Position error = '
                f'{final_position_error:.3f} m'
            )

            self.get_logger().warn(
                f'Angle error = '
                f'{math.degrees(final_angle_error):.2f} deg'
            )

        self.get_logger().warn(
            '========================================'
        )

        # ==========================================================
    # 軌道復元
    # ==========================================================

    def reconstruct_path(
        self,
        goal_key,
        states,
        parents
    ):

        path = []

        key = goal_key

        while key is not None:

            path.append(
                states[key].copy()
            )

            key = parents[key]

        path.reverse()

        return np.array(path)

        # ==========================================================
    # 移動区間の衝突判定
    # ==========================================================

    def check_transition_collision(
        self,
        x1,
        y1,
        theta1,
        x2,
        y2,
        theta2
    ):

        num_samples = self.collision_substeps

        # ==========================================================
        # 1ステップを細かく分割
        # ==========================================================

        theta_difference = self.angle_difference(
            theta2,
            theta1
        )

        for i in range(
            1,
            num_samples + 1
        ):

            ratio = (
                i /
                num_samples
            )

            x = (
                x1 +
                (x2 - x1) *
                ratio
            )

            y = (
                y1 +
                (y2 - y1) *
                ratio
            )

            theta = (
                theta1 +
                theta_difference *
                ratio
            )

            theta = math.atan2(
                math.sin(theta),
                math.cos(theta)
            )

            if self.check_collision(
                x,
                y,
                theta
            ):

                return True

        return False
    # ==========================================================
    # 車いすと点群の衝突判定
    # ==========================================================

    def check_collision(
        self,
        robot_x,
        robot_y,
        robot_theta
    ):

        if self.xy is None:
            return False

        # ==========================================================
        # 車いす座標系へ点群を変換
        # ==========================================================

        dx = self.xy[:, 0] - robot_x
        dy = self.xy[:, 1] - robot_y

        cos_theta = math.cos(
            robot_theta
        )

        sin_theta = math.sin(
            robot_theta
        )

        # ワールド座標 → 車いす座標
        local_x = (
            cos_theta * dx +
            sin_theta * dy
        )

        local_y = (
            -sin_theta * dx +
            cos_theta * dy
        )

        # ==========================================================
        # 車いす占有領域
        # ==========================================================

        x_min = (
            self.robot_x_min -
            self.collision_margin
        )

        x_max = (
            self.robot_x_max +
            self.collision_margin
        )

        y_min = (
            self.robot_y_min -
            self.collision_margin
        )

        y_max = (
            self.robot_y_max +
            self.collision_margin
        )

        # ==========================================================
        # 長方形内部に点群が存在するか
        # ==========================================================

        collision = (
            (local_x >= x_min) &
            (local_x <= x_max) &
            (local_y >= y_min) &
            (local_y <= y_max)
        )

        return bool(
            np.any(collision)
        )

    # ==========================================================
    # 角度差
    # ==========================================================

    def angle_difference(
        self,
        angle1,
        angle2
    ):

        return math.atan2(
            math.sin(angle1 - angle2),
            math.cos(angle1 - angle2)
        )

    # ==========================================================
    # プロット
    # ==========================================================

    def plot_result(self):

        fig, ax = plt.subplots(
            figsize=(10, 8)
        )

        # ==========================================================
        # 点群
        # ==========================================================

        ax.scatter(
            self.xy[:, 0],
            self.xy[:, 1],
            s=5,
            label='Filtered points'
        )

        # ==========================================================
        # ドア
        # ==========================================================

        x1 = self.door_data[0]
        y1 = self.door_data[1]

        x2 = self.door_data[2]
        y2 = self.door_data[3]

        ax.plot(
            [x1, x2],
            [y1, y2],
            linewidth=3,
            label='Door'
        )

        # ==========================================================
        # ゴール
        # ==========================================================

        ax.scatter(
            self.center_x,
            self.center_y,
            s=180,
            marker='*',
            label='Goal'
        )

        # ==========================================================
        # ゴール姿勢
        # ==========================================================

        arrow_length = 0.8

        ax.arrow(
            self.center_x,
            self.center_y,
            arrow_length *
            math.cos(self.goal_angle),
            arrow_length *
            math.sin(self.goal_angle),
            width=0.01,
            head_width=0.12,
            head_length=0.15,
            length_includes_head=True,
            label='Goal direction'
        )

        # ==========================================================
        # 車いす原点
        # ==========================================================

        ax.scatter(
            0,
            0,
            s=150,
            marker='x',
            linewidths=3,
            label='Start'
        )

        # ==========================================================
        # 最適経路
        # ==========================================================

        path = self.best_path

        ax.plot(
            path[:, 0],
            path[:, 1],
            linewidth=3,
            label='Best path'
        )

        # ==========================================================
        # 軌跡上の車いす姿勢
        # ==========================================================

        # 数点だけ表示
        indices = np.linspace(
            0,
            len(path) - 1,
            8,
            dtype=int
        )

        for index in indices:

            self.plot_robot(
                ax,
                path[index, 0],
                path[index, 1],
                path[index, 2]
            )

        # ==========================================================
        # ゴール到達車いす
        # ==========================================================

        self.plot_robot(
            ax,
            path[-1, 0],
            path[-1, 1],
            path[-1, 2]
        )

        # ==========================================================
        # 軸
        # ==========================================================

        ax.axhline(
            0,
            linewidth=1
        )

        ax.axvline(
            0,
            linewidth=1
        )

        ax.set_xlabel(
            'X [m]'
        )

        ax.set_ylabel(
            'Y [m]'
        )

        ax.set_title(
            'PointCloud, Wheelchair Collision '
            'and Planned Path'
        )

        ax.axis('equal')
        ax.grid(True)
        ax.legend()

        # ==========================================================
        # 保存
        # ==========================================================

        timestamp = (
            self.get_clock()
            .now()
            .nanoseconds
        )

        save_path = (
            '/home/ubuntu/humble_ws/'
            'src/mpc_navigation_pypkg/'
            'mpc_navigation_pypkg/output/'
            f'path_{timestamp}.png'
        )

        plt.savefig(
            save_path,
            dpi=150,
            bbox_inches='tight'
        )

        plt.close(fig)

        self.get_logger().info(
            f'Plot saved: {save_path}'
        )

    # ==========================================================
    # 車いす描画
    # ==========================================================

    def plot_robot(
        self,
        ax,
        x,
        y,
        theta
    ):

        # 車いす四隅
        corners_local = np.array([
            [self.robot_x_min, self.robot_y_min],
            [self.robot_x_max, self.robot_y_min],
            [self.robot_x_max, self.robot_y_max],
            [self.robot_x_min, self.robot_y_max]
        ])

        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        corners_world = []

        for px, py in corners_local:

            wx = (
                x +
                cos_theta * px -
                sin_theta * py
            )

            wy = (
                y +
                sin_theta * px +
                cos_theta * py
            )

            corners_world.append(
                [wx, wy]
            )

        corners_world = np.array(
            corners_world
        )

        # 閉じる
        corners_world = np.vstack([
            corners_world,
            corners_world[0]
        ])

        ax.plot(
            corners_world[:, 0],
            corners_world[:, 1],
            linewidth=1
        )

        # 車いす前方
        arrow_length = 0.3

        ax.arrow(
            x,
            y,
            arrow_length *
            math.cos(theta),
            arrow_length *
            math.sin(theta),
            width=0.005,
            head_width=0.05,
            head_length=0.08,
            length_includes_head=True
        )


# ==============================================================
# main
# ==============================================================

def main(args=None):

    rclpy.init(args=args)

    node = GoalCalculator()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()