# pointcloud2を購読、表示
# python3 pointcloud_2d_viewer.py 
# python3 door_service_node.py 
# python3 door_planner_node.py
# ros2 bag play rosbag2_2026_09_09-22_26_15/ --loop
# ros2 run rslidar_sdk rslidar_sdk_node
#!/usr/bin/env python3
#!/usr/bin/env python3
#!/usr/bin/env python3
#!/usr/bin/env python3
"""OpenCV ビューア兼サービスクライアント (単一ファイル)。

  python3 door_viewer_node.py [--ros-args -p range:=6.0 -p auto_interval_sec:=1.0]

画面下端のボタン(クリック)またはキーで操作:
  [Detect doors] / d : サービス /detect_doors を呼び、候補トップNを描画 (1位を自動選択)
  ドアをクリック / 1,2,3 : 経路を生成する候補ドアを選択 (選択中のドアは左上に表示)
  [Plan path]    / p : 選択中のドアまでの経路をサービス /plan_path で生成して描画
  [Auto]         / a : 検出の自動リクエストのON/OFF
  [+] [-]  / + -     : ズーム
  [Save path]    / w : 生成した経路の画像(PNG)と、経路手順のTEXTファイルを保存 (save_dir)
  [Save view]    / s : 点群ウィンドウの表示をそのままPNG保存
  文字情報(選択中のドア・候補一覧・状態)は別ウィンドウ "Door info" に表示
  q / ESC            : 終了
"""

import json
import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.srv import GetPlan
from std_srvs.srv import Trigger

RANK_COLORS = [(0, 0, 255), (0, 170, 255), (255, 200, 0)]  # BGR: 1位=赤, 2位=橙, 3位=水色
OTHER_COLOR = (180, 180, 180)
POINT_COLOR = (200, 200, 200)
GRID_COLOR = (60, 60, 60)

PATH_COLOR = (255, 0, 255)       # 経路(マゼンタ)
FOOT_COLOR = (0, 125, 0)        # 各ステップの車体矩形
BUTTON_DEFS = [("detect", "Detect doors", 150), ("plan", "Plan path", 120), ("save_path", "Save path", 110),
               ("auto", "Auto", 100),
               ("zoom_in", "+", 44), ("zoom_out", "-", 44), ("save", "Save view", 100)]
BUTTON_H, BUTTON_GAP = 36, 8


def rank_color(rank):
    return RANK_COLORS[rank - 1] if 1 <= rank <= len(RANK_COLORS) else OTHER_COLOR


def button_layout(size):
    y1 = size - 10
    y0 = y1 - BUTTON_H
    x, out = 10, []
    for action, label, w in BUTTON_DEFS:
        out.append((action, label, (x, y0, x + w, y1)))
        x += w + BUTTON_GAP
    return out


def hit_button(size, px, py):
    for action, _, (x0, y0, x1, y1) in button_layout(size):
        if x0 <= px <= x1 and y0 <= py <= y1:
            return action
    return None


def w2p(size, range_m, x, y):
    """ワールド(x前方, y左) -> 画像(col,row)。"""
    res = 2.0 * range_m / size
    c = size // 2
    return (int(round(c - y / res)), int(round(c - x / res)))


def pick_candidate(cands, size, range_m, px, py, tol=14):
    """クリック位置に最も近い(許容tol px以内の)候補のrank。無ければNone。"""
    best, best_d = None, tol
    q = np.array([px, py], dtype=np.float64)
    for c in cands:
        a = np.array(w2p(size, range_m, *c["p1"]), dtype=np.float64)
        b = np.array(w2p(size, range_m, *c["p2"]), dtype=np.float64)
        ab = b - a
        t = 0.0 if not ab.any() else float(np.clip(np.dot(q - a, ab) / np.dot(ab, ab), 0, 1))
        d = float(np.linalg.norm(q - (a + t * ab)))
        if d <= best_d:
            best, best_d = c["rank"], d
    return best


def render(points_xy, candidates, size, range_m, point_size,
           button_labels, button_states, selected_rank=None, path=None,
           footprint=(0.0, 0.9, 0.275), show_buttons=True):
    """点群の2D俯瞰図(前方+x=上, 左+y=左)にドア候補・経路・ボタンを描く。(文字情報は render_info 側)"""
    res = 2.0 * range_m / size
    cx = cy = size // 2

    def P(x, y):
        return w2p(size, range_m, x, y)

    img = np.zeros((size, size, 3), dtype=np.uint8)
    if points_xy is not None and len(points_xy):
        col = np.rint(cx - points_xy[:, 1] / res).astype(np.int32)
        row = np.rint(cy - points_xy[:, 0] / res).astype(np.int32)
        ok = (col >= 0) & (col < size) & (row >= 0) & (row < size)
        img[row[ok], col[ok]] = POINT_COLOR
        if point_size > 1:
            img = cv2.dilate(img, np.ones((point_size, point_size), np.uint8))

    step = 1.0 if range_m <= 6 else 2.0 if range_m <= 12 else 5.0
    r = step
    while r < range_m * 1.42:
        cv2.circle(img, (cx, cy), int(r / res), GRID_COLOR, 1, cv2.LINE_AA)
        cv2.putText(img, f"{r:g}m", (cx + 4, cy - int(r / res) - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (110, 110, 110), 1, cv2.LINE_AA)
        r += step
    cv2.line(img, (cx, 0), (cx, size), GRID_COLOR, 1)
    cv2.line(img, (0, cy), (size, cy), GRID_COLOR, 1)

    # --- 経路: 各ステップの車体矩形 + 軌跡 (候補より下の層) ---
    if path is not None and len(path):
        fx0, fx1, fhw = footprint
        local = np.array([[fx0, -fhw], [fx1, -fhw], [fx1, fhw], [fx0, fhw]])
        for i, (x, y, th) in enumerate(path):
            c_, s_ = math.cos(th), math.sin(th)
            poly = np.array([P(x + c_ * px - s_ * py, y + s_ * px + c_ * py) for px, py in local],
                            np.int32)
            last = i == len(path) - 1
            cv2.polylines(img, [poly], True, PATH_COLOR if last else FOOT_COLOR,
                          2 if last else 1, cv2.LINE_AA)
        if len(path) > 1:
            cv2.polylines(img, [np.array([P(x, y) for x, y, _ in path], np.int32)], False,
                          PATH_COLOR, 2, cv2.LINE_AA)

    tri = np.array([[cx, cy - 10], [cx - 6, cy + 6], [cx + 6, cy + 6]], np.int32)
    cv2.fillConvexPoly(img, tri, (0, 255, 0), cv2.LINE_AA)   # 自機

    for c in sorted(candidates, key=lambda c: -c["rank"]):   # 1位を最前面に
        color = rank_color(c["rank"])
        a, b, ctr = P(*c["p1"]), P(*c["p2"]), P(*c["center"])
        cv2.line(img, a, b, color, 2, cv2.LINE_AA)
        cv2.circle(img, a, 5, color, -1, cv2.LINE_AA)
        cv2.circle(img, b, 5, color, -1, cv2.LINE_AA)
        cv2.circle(img, ctr, 4, (255, 255, 255), -1, cv2.LINE_AA)
        tip = P(c["center"][0] + 0.5 * math.cos(c["goal_yaw"]),
                c["center"][1] + 0.5 * math.sin(c["goal_yaw"]))
        cv2.arrowedLine(img, ctr, tip, color, 2, cv2.LINE_AA, tipLength=0.3)
        #cv2.putText(img, f"#{c['rank']}", (ctr[0] + 8, ctr[1] - 8),
        #            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

    if not show_buttons:
        return img
    for action, default, (x0, y0_, x1, y1) in button_layout(size):
        label = button_labels.get(action, default)
        state = button_states.get(action, "normal")
        fill = {"normal": (70, 70, 70), "active": (60, 130, 60), "disabled": (40, 40, 40)}[state]
        fg = (120, 120, 120) if state == "disabled" else (240, 240, 240)
        cv2.rectangle(img, (x0, y0_), (x1, y1), fill, -1)
        cv2.rectangle(img, (x0, y0_), (x1, y1), (150, 150, 150), 1)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.putText(img, label, ((x0 + x1 - tw) // 2, (y0_ + y1 + th) // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, fg, 1, cv2.LINE_AA)
    return img


def render_info(selected, candidates, detect_info, plan_info, extra_info, width=620):
    """文字情報だけを表示する別ウィンドウ用の画像。"""
    lines = []   # (text, color, scale, thickness)
    white, gray = (230, 230, 230), (150, 150, 150)

    lines.append(("Selected door", gray, 0.45, 1))
    if selected is not None:
        lines.append((f"#{selected['rank']}  w={selected['width']:.2f} m  "
                      f"center=({selected['center'][0]:.2f}, {selected['center'][1]:.2f})  "
                      f"yaw={math.degrees(selected['goal_yaw']):+.0f} deg",
                      rank_color(selected["rank"]), 0.6, 2))
    else:
        lines.append(("(none)", gray, 0.6, 1))

    lines.append(("", white, 0.45, 1))
    lines.append(("Candidates", gray, 0.45, 1))
    if candidates:
        for c in sorted(candidates, key=lambda c: c["rank"]):
            lines.append((f"#{c['rank']}  w={c['width']:.2f} m  err={c['width_error']:.2f}  "
                          f"center=({c['center'][0]:.2f}, {c['center'][1]:.2f})  "
                          f"yaw={math.degrees(c['goal_yaw']):+.0f} deg",
                          rank_color(c["rank"]), 0.5, 1))
    else:
        lines.append(("(none)", gray, 0.5, 1))

    lines.append(("", white, 0.45, 1))
    lines.append(("Status", gray, 0.45, 1))
    for t in list(detect_info) + list(plan_info) + list(extra_info):
        lines.append((t, white, 0.45, 1))

    h = max(260, 20 + 22 * len(lines))
    img = np.zeros((h, width, 3), dtype=np.uint8)
    y = 24
    for text, color, scale, thick in lines:
        cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)
        y += 22
    return img


def describe_path(path, dt):
    """経路(各ステップ終了時の姿勢)から、各ステップの動作と、同じ動作の連続をまとめた手順を作る。"""
    rows = [dict(step=0, time=0.0, label="(開始)", v=None, omega=None,
                 x=path[0][0], y=path[0][1], th=path[0][2])]
    for i in range(1, len(path)):
        dx, dy = path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]
        dth = math.atan2(math.sin(path[i][2] - path[i - 1][2]),
                         math.cos(path[i][2] - path[i - 1][2]))
        dist = math.hypot(dx, dy)
        if dist > 1e-6:
            label = "前進"
        elif dth > 1e-6:
            label = "左旋回"
        elif dth < -1e-6:
            label = "右旋回"
        else:
            label = "静止"
        rows.append(dict(step=i, time=i * dt, label=label, v=dist / dt, omega=dth / dt,
                         dist=dist, dth=dth, x=path[i][0], y=path[i][1], th=path[i][2]))
    groups = []
    for r in rows[1:]:
        if groups and groups[-1]["label"] == r["label"]:
            g = groups[-1]
            g["n"] += 1
            g["dist"] += r["dist"]
            g["dth"] += r["dth"]
        else:
            groups.append(dict(label=r["label"], n=1, dist=r["dist"], dth=r["dth"]))
    return rows, groups


def format_path_text(res, dt, tolerances, stamp):
    """保存用のTEXT。res = _on_plan_response が作る結果dict。"""
    rows, groups = describe_path(res["path"], dt)
    cand, goal = res["cand"], res["goal"]
    steps = len(rows) - 1
    L = ["# 経路手順 (door path)",
         f"生成日時   : {stamp}",
         f"フレーム   : {res['frame_id']}  (原点=LiDAR, +x=前方, +y=左)",
         f"対象ドア   : #{cand['rank']}  幅 {cand['width']:.2f} m  "
         f"中心 ({cand['center'][0]:.2f}, {cand['center'][1]:.2f}) m",
         f"ゴール姿勢 : x={goal[0]:.3f} m, y={goal[1]:.3f} m, yaw={math.degrees(goal[2]):+.1f} deg",
         f"到達許容   : 位置 {tolerances[0]:.2f} m / 角度 {math.degrees(tolerances[1]):.0f} deg",
         f"結果       : {'到達' if res['reached'] else '未到達 (ゴールに最も近づいた経路)'}",
         f"最終誤差   : {res['pe']:.3f} m / {math.degrees(res['ae']):.1f} deg",
         f"ステップ数 : {steps}  (dt = {dt:g} s, 所要時間 {steps * dt:.1f} s)",
         "",
         "## 手順まとめ"]
    if not groups:
        L.append("(動作なし)")
    for k, g in enumerate(groups, start=1):
        if g["label"] == "前進":
            amount = f"{g['dist']:.2f} m"
        else:
            amount = f"{math.degrees(abs(g['dth'])):.1f} deg"
        L.append(f"{k:2d}. {g['label']} x{g['n']}  ({amount}, {g['n'] * dt:.1f} s)")
    L += ["", "## 詳細 (各ステップ終了時の姿勢)",
          "step  time[s]  動作      v[m/s]  omega[rad/s]   x[m]     y[m]    theta[deg]"]
    for r in rows:
        v = "    -  " if r["v"] is None else f"{r['v']:+7.3f}"
        om = "      -     " if r["omega"] is None else f"{r['omega']:+11.3f} "
        L.append(f"{r['step']:4d}  {r['time']:7.1f}  {r['label']:<6s}  {v}  {om}  "
                 f"{r['x']:+8.3f} {r['y']:+8.3f}   {math.degrees(r['th']):+8.2f}")
    return "\n".join(L) + "\n"


def parse_response(res):
    """Trigger応答 -> (points_xy[m], candidates, info行のリスト, frame_id)。"""
    try:
        d = json.loads(res.message)
    except json.JSONDecodeError:
        return None, [], [f"[NG] {res.message}"], ""
    if "error" in d:
        return None, [], [f"[NG] {d['error']}"], ""
    pts = np.asarray(d["points_cm"], dtype=np.float32)
    pts = (pts[: len(pts) // 2 * 2] / 100.0).reshape(-1, 2)
    status = "OK" if res.success else "NG (候補なし)"
    return pts, d["candidates"], [f"[{status}] {d['info']}", f"frame: {d['frame_id']}"], d["frame_id"]


class DoorViewer(Node):
    def __init__(self):
        super().__init__("door_viewer")
        self.declare_parameter("service_name", "detect_doors")
        self.declare_parameter("plan_service_name", "plan_path")
        self.declare_parameter("range", 6.0)          # 表示半径 [m]
        self.declare_parameter("image_size", 800)
        self.declare_parameter("point_size", 1)
        self.declare_parameter("auto_interval_sec", 0.0)   # >0で起動時から自動検出
        self.declare_parameter("position_tolerance", 0.15)  # 到達判定(プランナーと揃える) [m]
        self.declare_parameter("angle_tolerance_deg", 10.0)  # 同 [deg]
        self.declare_parameter("robot_x_min", 0.0)    # 車体矩形の描画用 (プランナーと揃える)
        self.declare_parameter("robot_x_max", 0.9)
        self.declare_parameter("robot_half_width", 0.275)
        self.declare_parameter("dt", 0.5)             # プランナーの dt (手順テキストの速度・時間計算用)
        self.declare_parameter("save_dir", "./output")       # 画像・テキストの保存先

        p = self.get_parameter
        self.range_m = float(p("range").value)
        self.size = int(p("image_size").value)
        self.point_size = max(1, int(p("point_size").value))
        self.auto_interval = float(p("auto_interval_sec").value)
        self.auto_on = self.auto_interval > 0.0
        self.pos_tol = float(p("position_tolerance").value)
        self.ang_tol = math.radians(float(p("angle_tolerance_deg").value))
        self.footprint = (float(p("robot_x_min").value), float(p("robot_x_max").value),
                          float(p("robot_half_width").value))
        self.dt = float(p("dt").value)
        self.save_dir = str(p("save_dir").value)
        self.client = self.create_client(Trigger, p("service_name").value)
        self.plan_client = self.create_client(GetPlan, p("plan_service_name").value)

        self._lock = threading.Lock()
        self._pending = self._plan_pending = False
        self._drawn_flags = (False, False, False)
        self._t_req = self._last_req = 0.0
        self._points, self._cands, self._frame_id = None, [], ""
        self._selected = None
        self._path = None
        self._plan_result = None
        self._save_msg = []
        self._info = ["click [Detect doors] (or press 'd')"]
        self._plan_info = []
        self._dirty = True
        self._last_img = None

    # ---- 検出 ----
    def request(self):
        if self._pending:
            return
        if not self.client.service_is_ready():
            self._set_info(["service not available (is door_service_node running?)"])
            return
        self._pending = True
        self._t_req = self._last_req = time.monotonic()
        self.client.call_async(Trigger.Request()).add_done_callback(self._on_response)

    def _on_response(self, future):
        ms = (time.monotonic() - self._t_req) * 1000.0
        try:
            pts, cands, info, frame_id = parse_response(future.result())
            info[-1] += f"   latency: {ms:.0f} ms"
            with self._lock:
                if pts is not None:
                    self._points, self._frame_id = pts, frame_id
                self._cands, self._info = cands, info
                self._selected = cands[0]["rank"] if cands else None   # 1位を既定選択
                self._path, self._plan_info = None, []                 # 古い経路は消す
                self._plan_result = None
                self._save_msg = []
                self._dirty = True
        except Exception as e:
            self._set_info([f"service call failed: {e!r}"])
        finally:
            self._pending = False

    def _set_info(self, lines):
        with self._lock:
            self._info, self._dirty = list(lines), True

    # ---- 経路生成 ----
    def _selected_candidate(self):
        with self._lock:
            for c in self._cands:
                if c["rank"] == self._selected:
                    return c
        return None

    def request_plan(self):
        if self._plan_pending:
            return
        cand = self._selected_candidate()
        if cand is None:
            self._set_plan_info(["select a door first (detect, then click a door or press 1-3)"])
            return
        if not self.plan_client.service_is_ready():
            self._set_plan_info(["service not available (is door_planner_node running?)"])
            return

        gx, gy = cand["center"]
        gyaw = cand["goal_yaw"]
        req = GetPlan.Request()
        req.start.header.frame_id = req.goal.header.frame_id = self._frame_id
        req.start.pose = Pose()
        req.start.pose.orientation.w = 1.0           # 現在位置 = 点群フレーム原点
        req.goal.pose.position.x, req.goal.pose.position.y = float(gx), float(gy)
        req.goal.pose.orientation.z, req.goal.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
        req.tolerance = float(self.pos_tol)

        self._plan_pending = True
        t0 = time.monotonic()
        rank = cand["rank"]
        self.plan_client.call_async(req).add_done_callback(
            lambda fut: self._on_plan_response(fut, (gx, gy, gyaw), rank, t0, cand))
        self._dirty = True

    def _on_plan_response(self, future, goal, rank, t0, cand=None):
        ms = (time.monotonic() - t0) * 1000.0
        result = None
        try:
            poses = future.result().plan.poses
            if not poses:
                path, lines = None, [f"[NG] #{rank}: 経路なし (plannerのログを確認)   {ms:.0f} ms"]
            else:
                path = np.array([[ps.pose.position.x, ps.pose.position.y,
                                  math.atan2(2 * ps.pose.orientation.w * ps.pose.orientation.z,
                                             1 - 2 * ps.pose.orientation.z ** 2)]
                                 for ps in poses])
                pe = math.hypot(path[-1, 0] - goal[0], path[-1, 1] - goal[1])
                ae = abs(math.atan2(math.sin(path[-1, 2] - goal[2]), math.cos(path[-1, 2] - goal[2])))
                reached = pe <= self.pos_tol and ae <= self.ang_tol
                result = dict(path=path, goal=goal, cand=cand, reached=reached, pe=pe, ae=ae,
                              frame_id=self._frame_id)
                lines = [f"[{'OK' if reached else 'NG 未到達'}] path to #{rank}: {len(path) - 1} steps, "
                         f"final error {pe:.2f} m / {math.degrees(ae):.1f} deg   {ms:.0f} ms"]
            with self._lock:
                self._path, self._plan_info, self._dirty = path, lines, True
                self._plan_result = result
                self._save_msg = []
        except Exception as e:
            self._set_plan_info([f"plan call failed: {e!r}"])
        finally:
            self._plan_pending = False

    def _set_plan_info(self, lines):
        with self._lock:
            self._plan_info, self._dirty = list(lines), True

    # ---- 経路の保存 (画像 + 手順TEXT) ----
    def save_path(self):
        with self._lock:
            res = self._plan_result
            points, cands = self._points, list(self._cands)
        if res is None:
            self._set_save_msg(["save path: 保存できる経路がありません (先に Plan path)"])
            return
        try:
            os.makedirs(self.save_dir, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            base = os.path.join(self.save_dir, f"door{res['cand']['rank']}_path_{stamp}")

            # 画像: ボタンなしで描き、記録として見やすいよう先頭に要約を入れる
            img = render(points, cands, self.size, self.range_m, self.point_size, {}, {},
                         selected_rank=res["cand"]["rank"], path=res["path"],
                         footprint=self.footprint, show_buttons=False)
            c = res["cand"]
            cap = [f"Door #{c['rank']}  w={c['width']:.2f} m  "
                   f"goal=({res['goal'][0]:.2f}, {res['goal'][1]:.2f}, "
                   f"{math.degrees(res['goal'][2]):+.0f} deg)",
                   f"{'REACHED' if res['reached'] else 'NOT REACHED'}  "
                   f"{len(res['path']) - 1} steps  final error "
                   f"{res['pe']:.2f} m / {math.degrees(res['ae']):.1f} deg"]
            for i, t in enumerate(cap):
                cv2.putText(img, t, (8, 20 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            rank_color(c["rank"]), 1, cv2.LINE_AA)
            if not cv2.imwrite(base + ".png", img):
                raise IOError(f"画像を書き込めません: {base}.png")

            text = format_path_text(res, self.dt, (self.pos_tol, self.ang_tol),
                                    time.strftime("%Y-%m-%d %H:%M:%S"))
            with open(base + ".txt", "w", encoding="utf-8") as f:
                f.write(text)
        except Exception as e:
            self._set_save_msg([f"save path failed: {e!r}"])
            return
        self.get_logger().info(f"saved {base}.png / {base}.txt")
        self._set_save_msg([f"saved: {base}.png", f"saved: {base}.txt"])

    def _set_save_msg(self, lines):
        with self._lock:
            self._save_msg, self._dirty = list(lines), True

    # ---- 操作 ----
    def _select(self, rank):
        with self._lock:
            if any(c["rank"] == rank for c in self._cands):
                self._selected, self._path, self._plan_info, self._dirty = rank, None, [], True
                self._plan_result = None
                self._save_msg = []

    def _do_action(self, action):
        if action == "detect":
            self.request()
        elif action == "plan":
            self.request_plan()
        elif action == "save_path":
            self.save_path()
        elif action.startswith("select:"):
            self._select(int(action.split(":")[1]))
        elif action == "auto":
            if self.auto_interval <= 0.0:
                self.auto_interval = 1.0
            self.auto_on = not self.auto_on
            self._dirty = True
        elif action == "zoom_in":
            self.range_m, self._dirty = max(1.0, self.range_m * 0.8), True
        elif action == "zoom_out":
            self.range_m, self._dirty = min(50.0, self.range_m * 1.25), True
        elif action == "save" and self._last_img is not None:
            fn = time.strftime("doors_%Y%m%d_%H%M%S.png")
            cv2.imwrite(fn, self._last_img)
            self.get_logger().info(f"saved {fn}")

    def _on_mouse(self, event, px, py, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        action = hit_button(self.size, px, py)
        if action:
            self._do_action(action)
            return
        with self._lock:
            cands = list(self._cands)
        rank = pick_candidate(cands, self.size, self.range_m, px, py)
        if rank is not None:
            self._select(rank)

    def run_gui(self):
        win = "Door candidates (top view)"
        cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(win, self._on_mouse)
        info_win = "Door info"
        cv2.namedWindow(info_win, cv2.WINDOW_AUTOSIZE)
        try:
            cv2.moveWindow(info_win, self.size + 30, 0)   # 点群ウィンドウの右隣へ
        except cv2.error:
            pass
        keys = {ord("d"): "detect", ord("p"): "plan", ord("w"): "save_path", ord("a"): "auto",
                ord("1"): "select:1", ord("2"): "select:2", ord("3"): "select:3",
                ord("+"): "zoom_in", ord("="): "zoom_in",
                ord("-"): "zoom_out", ord("_"): "zoom_out", ord("s"): "save"}

        while rclpy.ok():
            if self.auto_on and time.monotonic() - self._last_req >= max(self.auto_interval, 0.2):
                self.request()

            with self._lock:
                flags = (self._pending, self._plan_pending, self.auto_on)
                dirty = self._dirty or flags != self._drawn_flags
                self._dirty = False
                points, cands = self._points, list(self._cands)
                detect_info, plan_info = list(self._info), list(self._plan_info) + list(self._save_msg)
                can_save = self._plan_result is not None
                selected, path = self._selected, self._path

            if dirty or self._last_img is None:
                self._drawn_flags = flags
                can_plan = selected is not None and not self._plan_pending
                self._last_img = render(
                    points, cands, self.size, self.range_m, self.point_size,
                    {"detect": "Detecting..." if self._pending else "Detect doors",
                     "plan": "Planning..." if self._plan_pending else "Plan path",
                     "auto": f"Auto: {'ON' if self.auto_on else 'OFF'}"},
                    {"detect": "disabled" if self._pending else "normal",
                     "plan": "normal" if can_plan else "disabled",
                     "save_path": "normal" if can_save else "disabled",
                     "auto": "active" if self.auto_on else "normal"},
                    selected_rank=selected, path=path, footprint=self.footprint)
                cv2.imshow(win, self._last_img)
                sel_c = next((c for c in cands if c["rank"] == selected), None)
                cv2.imshow(info_win, render_info(
                    sel_c, cands, detect_info, plan_info,
                    [f"range: {self.range_m:g} m   points: {0 if points is None else len(points)}"]))

            key = cv2.waitKey(20) & 0xFF
            if key in (ord("q"), 27):
                break
            if key in keys:
                self._do_action(keys[key])
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = DoorViewer()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()  # GUIはメインスレッド
    try:
        node.run_gui()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()