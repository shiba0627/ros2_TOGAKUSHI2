#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time
import queue
import json

import psutil

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import Trigger
from togakushi2_interfaces.msg import Obstacle

from flask import Flask, Response, redirect, request as flask_request

# 点群保存サービス呼び出しのタイムアウト [s]
SAVE_POINTCLOUD_TIMEOUT = 3.0

# 点群配信レート上限 (Hz)。obstacle_node 側がこれより速ければ間引く。
SCAN_RATE_HZ = 20.0
SCAN_PERIOD  = 1.0 / SCAN_RATE_HZ

# obstacle_node 側の POINTS_MAX_RANGE と一致させる
POINTS_MAX_RANGE = 1.5

# ボクセル間引きセルサイズ [m]。
# 同じ (x,y) セルに落ちる点は1つに統合される。値を大きくすると点数↓・粒度↓。
VOXEL_SIZE = 0.1


# =========================
# ROS2 Node
# =========================
class WebGuiNode(Node):
    def __init__(self):
        super().__init__('web_ui_node')

        self.current_state = "UNKNOWN"
        self._near = {"front": False, "right": False, "back": False, "left": False}

        self.pub = self.create_publisher(String, '/state_transition', 10)
        self.gain_pub = self.create_publisher(Float32MultiArray, '/speed_gain', 10)
        self._angular_gain = 1.0
        self._linear_gain  = 1.0

        self.sub = self.create_subscription(String, '/robot_state', self.state_callback, 10)
        self.obstacle_sub = self.create_subscription(
            Obstacle, '/obstacle_info', self.obstacle_callback, 10)

        # obstacle_node が publish する直交座標点群 (JSON 文字列) をそのまま中継
        self._next_scan_send = 0.0
        self.points_sub = self.create_subscription(
            String, '/obstacle_points', self.points_callback, 10)

        # pointcloud_saver_node のサービス (すべて std_srvs/Trigger) クライアント
        self.save_pointcloud_client  = self.create_client(Trigger, 'save_pointcloud')
        self.start_autosave_client   = self.create_client(Trigger, 'start_pointcloud_recording')
        self.stop_autosave_client    = self.create_client(Trigger, 'stop_pointcloud_recording')

        # UI表示用の自動保存中フラグ。
        # 注意: これはこのFlaskプロセスがstart/stopを呼んだ結果を保持するだけの
        # ローカルな表示用フラグであり、pointcloud_saver_node側の実際の状態を
        # 問い合わせるサービスは無いため、ページを再読み込みすると(実際は
        # 記録中でも)未記録表示にリセットされる点に留意すること。
        self.autosave_active = False

    def _call_trigger(self, client, service_name, timeout=SAVE_POINTCLOUD_TIMEOUT):
        """std_srvs/Trigger サービスの共通呼び出し処理。(success, message) を返す。

        rclpy.spin(self) は別スレッド(ros_spin)で常時実行されているため、
        ここでは spin_until_future_complete を使わず、
        call_async + add_done_callback + threading.Event で結果を待つ。
        """
        if not client.wait_for_service(timeout_sec=0.5):
            return False, f"{service_name} service not available"

        done_event = threading.Event()
        result_holder = {}

        def _on_done(fut):
            try:
                result_holder['result'] = fut.result()
            except Exception as exc:  # noqa: BLE001
                result_holder['error'] = str(exc)
            done_event.set()

        future = client.call_async(Trigger.Request())
        future.add_done_callback(_on_done)

        if not done_event.wait(timeout=timeout):
            return False, f"Timed out waiting for {service_name} response"

        if 'error' in result_holder:
            return False, result_holder['error']

        res = result_holder['result']
        return res.success, res.message

    def call_save_pointcloud(self, timeout=SAVE_POINTCLOUD_TIMEOUT):
        """save_pointcloud サービスを呼び出す。(success: bool, message: str) を返す。"""
        return self._call_trigger(self.save_pointcloud_client, 'save_pointcloud', timeout)

    def call_start_autosave(self, timeout=SAVE_POINTCLOUD_TIMEOUT):
        """start_pointcloud_recording サービスを呼び出す。(success, message) を返す。"""
        success, message = self._call_trigger(
            self.start_autosave_client, 'start_pointcloud_recording', timeout)
        if success:
            self.autosave_active = True
        return success, message

    def call_stop_autosave(self, timeout=SAVE_POINTCLOUD_TIMEOUT):
        """stop_pointcloud_recording サービスを呼び出す。(success, message) を返す。"""
        success, message = self._call_trigger(
            self.stop_autosave_client, 'stop_pointcloud_recording', timeout)
        if success:
            self.autosave_active = False
        return success, message

    def state_callback(self, msg):
        self.current_state = msg.data
        push_state(msg.data)

    def obstacle_callback(self, msg):
        if len(msg.data) < 10:
            return
        self._near = {
            "front": bool(msg.data[1]), "right": bool(msg.data[3]),
            "back":  bool(msg.data[6]), "left":  bool(msg.data[9]),
        }
        push_obstacle(self._near)

    def points_callback(self, msg):
        """obstacle_node からの JSON {"points":[{"x":..,"y":..},...]} を
        ボクセル間引きしてコンパクト形式 {"x":[..],"y":[..]} で中継。"""
        now = time.monotonic()
        if now < self._next_scan_send:
            return
        self._next_scan_send = now + SCAN_PERIOD

        try:
            points = json.loads(msg.data).get("points", [])
        except Exception:
            return

        # ボクセル間引き: (x,y) を VOXEL_SIZE 角のセルに量子化し、重複セルを除去
        inv = 1.0 / VOXEL_SIZE
        seen = set()
        xs = []
        ys = []
        for p in points:
            px = p.get("x"); py = p.get("y")
            if px is None or py is None:
                continue
            key = (int(px * inv), int(py * inv))
            if key in seen:
                continue
            seen.add(key)
            xs.append(round(px, 2))
            ys.append(round(py, 2))

        # split-array + 空白なし JSON で転送量を最小化
        payload = json.dumps({"x": xs, "y": ys}, separators=(',', ':'))
        push_scan(payload)

    def send_cmd(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)
        self.get_logger().info(f'SEND: {text}')

    def publish_gain(self):
        msg = Float32MultiArray()
        msg.data = [self._angular_gain, self._linear_gain]
        self.gain_pub.publish(msg)
        self.get_logger().info(f'GAIN: angular={self._angular_gain} linear={self._linear_gain}')


# =========================
# Flask
# =========================
app = Flask(__name__)
ros_node = None

clients          = []
obstacle_clients = []
scan_clients     = []
stats_clients    = []


def _push(lst, payload):
    """共通 push。put 失敗(Full)時は最古を捨てて入れ直す。"""
    dead = []
    for q in lst:
        try:
            q.put_nowait(payload)
        except queue.Full:
            try: q.get_nowait()
            except queue.Empty: pass
            try: q.put_nowait(payload)
            except Exception: pass
        except Exception:
            dead.append(q)
    for q in dead:
        if q in lst: lst.remove(q)


def push_state(state):     _push(clients,          state)
def push_obstacle(near):   _push(obstacle_clients, json.dumps({"near": near}))
def push_scan(payload):    _push(scan_clients,     payload)   # 既に JSON 文字列
def push_stats(stats):     _push(stats_clients,    json.dumps(stats))


# =========================
# システム情報収集 (2秒ごと)
# =========================
def top_procs(n=5):
    procs = []
    for p in psutil.process_iter(['name', 'memory_info']):
        try:
            mb = p.info['memory_info'].rss / 1024 ** 2
            procs.append({"name": p.info['name'], "mb": round(mb, 1)})
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    procs.sort(key=lambda x: x['mb'], reverse=True)
    return procs[:n]


def collect_stats():
    psutil.cpu_percent(percpu=True)
    time.sleep(1)
    while True:
        bat   = psutil.sensors_battery()
        mem   = psutil.virtual_memory()
        disk  = psutil.disk_usage('/')
        cores = psutil.cpu_percent(percpu=True)

        stats = {
            "cpu":       round(sum(cores) / len(cores), 1),
            "cores":     [round(v, 1) for v in cores],
            "mem":       round(mem.percent, 1),
            "mem_used":  round(mem.used  / 1024 ** 3, 1),
            "mem_total": round(mem.total / 1024 ** 3, 1),
            "disk":      round(disk.percent, 1),
            "disk_used": round(disk.used  / 1024 ** 3, 1),
            "disk_total":round(disk.total / 1024 ** 3, 1),
            "bat_pct":   round(bat.percent, 1) if bat else None,
            "bat_charge":bat.power_plugged      if bat else None,
            "bat_secsleft": (
                int(bat.secsleft)
                if bat and bat.secsleft >= 0
                else None
            ),
            "procs":     top_procs(5),
        }
        push_stats(stats)
        time.sleep(2)


# =========================
# SSE エンドポイント
# =========================
@app.route("/events")
def events():
    q = queue.Queue()
    clients.append(q)
    def stream():
        yield f"data: {ros_node.current_state}\n\n"
        while True:
            yield f"data: {q.get()}\n\n"
    return Response(stream(), mimetype="text/event-stream")


@app.route("/obstacle_events")
def obstacle_events():
    q = queue.Queue()
    obstacle_clients.append(q)
    def stream():
        init = json.dumps({
            "near": {"front":False,"right":False,"back":False,"left":False}
        })
        yield f"data: {init}\n\n"
        while True:
            yield f"data: {q.get()}\n\n"
    return Response(stream(), mimetype="text/event-stream")


@app.route("/scan_events")
def scan_events():
    # 有界キュー: クライアント詰まり時は古いフレームを捨ててレイテンシ蓄積を防ぐ
    q = queue.Queue(maxsize=2)
    scan_clients.append(q)
    def stream():
        init = json.dumps({"x": [], "y": []})
        yield f"data: {init}\n\n"
        while True:
            yield f"data: {q.get()}\n\n"
    return Response(stream(), mimetype="text/event-stream")


@app.route("/stats_events")
def stats_events():
    q = queue.Queue()
    stats_clients.append(q)
    def stream():
        init = json.dumps({
            "cpu":0,"cores":[],"mem":0,"mem_used":0,"mem_total":0,
            "disk":0,"disk_used":0,"disk_total":0,
            "bat_pct":None,"bat_charge":None,"bat_secsleft":None,"procs":[]
        })
        yield f"data: {init}\n\n"
        while True:
            yield f"data: {q.get()}\n\n"
    return Response(stream(), mimetype="text/event-stream")


# =========================
# 状態遷移
# =========================
def move_to_target(target):
    order = ["STOP", "HELPER", "USER", "GAME"]
    if target not in order:
        return
    ros_node.send_cmd("RISE")
    time.sleep(0.25)
    if target == "STOP":
        return
    for _ in range(order.index(target)):
        ros_node.send_cmd("FALL")
        time.sleep(0.25)


# =========================
# HTML
# =========================
HTML = """<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>WHILL Control</title>
<style>
:root{
  --bg:#07090f; --sf:#0d1520; --bd:#1c2a3a;
  --ac:#38bdf8; --tx:#e2e8f0; --tx2:#94a3b8; --tx3:#475569;
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--tx);
     font-family:'Helvetica Neue',Arial,sans-serif;min-height:100dvh}
.wrap{max-width:440px;margin:0 auto;padding:14px}
h1{text-align:center;font-size:18px;font-weight:700;letter-spacing:4px;
   color:var(--ac);padding:12px 0 16px}
.card{background:var(--sf);border:1px solid var(--bd);
      border-radius:18px;padding:16px;margin-bottom:12px}

.clbl{font-size:10px;letter-spacing:2px;text-transform:uppercase;
      color:var(--tx);font-weight:500;text-align:center;margin-bottom:10px}
.slbl{font-size:10px;letter-spacing:2px;text-transform:uppercase;
      color:var(--tx);font-weight:500;margin-bottom:10px}

#state{font-size:36px;font-weight:700;text-align:center;
       letter-spacing:4px;color:var(--ac);transition:color .25s}

.rw{display:flex;flex-direction:column;align-items:center;gap:10px}
#rc{display:block;width:310px;height:310px}

.mrow{display:flex;justify-content:space-between;align-items:baseline;margin-bottom:3px}
.big{font-size:24px;font-weight:700;color:var(--tx);line-height:1}
.unit{font-size:12px;color:var(--tx2);margin-left:2px}
.sub{font-size:11px;color:var(--tx2);margin-bottom:7px}
.track{height:5px;background:#1a2535;border-radius:3px;overflow:hidden}
.fill{height:100%;border-radius:3px;transition:width .5s ease,background .5s ease}
.badge{font-size:11px;padding:2px 8px;border-radius:8px;font-weight:500}

.bat-row{display:flex;align-items:center;gap:14px}
.bat-icon{font-size:28px;flex-shrink:0}
.bat-body{flex:1}

.cores{display:grid;grid-template-columns:repeat(2,1fr);gap:6px;margin-top:10px}
.core-item{display:flex;flex-direction:column;gap:3px}
.core-top{display:flex;justify-content:space-between;font-size:11px}
.core-name{color:var(--tx);font-weight:500}
.core-pct{color:var(--tx2)}

.procs{display:flex;flex-direction:column;gap:5px;margin-top:10px}
.proc-row{display:flex;align-items:center;gap:8px}
.proc-rank{font-size:11px;color:var(--tx3);width:14px;text-align:right;flex-shrink:0}
.proc-name{font-size:12px;color:var(--tx);flex:1;
           white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.proc-mb{font-size:11px;color:var(--tx2);flex-shrink:0;min-width:64px;text-align:right}
.proc-bg{width:56px;height:4px;background:#1a2535;border-radius:2px;flex-shrink:0;overflow:hidden}
.proc-fg{height:100%;border-radius:2px;background:#38bdf8;transition:width .5s}

/* 速度調整 (ステッパー) — flex で完全センタリング */
.gain-rows{display:flex;flex-direction:column;gap:10px}
.gain-row{display:grid;grid-template-columns:36px 1fr 64px;align-items:center;gap:10px}
.gain-lbl{font-size:13px;color:var(--tx);font-weight:500}
.stepper{display:flex;align-items:stretch;background:#152030;
         border:1px solid var(--bd);border-radius:10px;overflow:hidden;height:42px}
.stepper button{
  flex:1;
  display:flex;
  align-items:center;
  justify-content:center;
  background:transparent;color:var(--tx);border:none;
  font-size:22px;font-weight:700;line-height:1;
  padding:0;cursor:pointer;
  font-family:'Helvetica Neue',Arial,sans-serif;
  transition:background .1s;
}
.stepper button:active{background:var(--bd)}
.stepper button:disabled{color:var(--tx3);cursor:not-allowed}
.stepper .val{flex:1;display:flex;align-items:center;justify-content:center;
              font-size:18px;font-weight:700;color:var(--ac);min-width:60px}
.gain-reset{font-size:10px;padding:0;height:42px;border:1px solid var(--bd);
            border-radius:8px;background:transparent;color:var(--tx2);
            cursor:pointer;white-space:nowrap}
.gain-reset:active{background:var(--bd)}

.bg{display:grid;grid-template-columns:repeat(2,1fr);gap:10px;margin-bottom:12px}
.bg form{margin:0}
button{width:100%;border:none;border-radius:13px;padding:17px 8px;
       font-size:17px;font-weight:700;letter-spacing:2px;color:#fff;
       cursor:pointer;transition:opacity .15s,transform .1s}
button:active{opacity:.8;transform:scale(.97)}
.s{background:#d97706}.h{background:#2563eb}
.u{background:#16a34a}.g{background:#7c3aed}

.save-pc{background:#0891b2}
.save-pc:disabled{background:#155e63;cursor:not-allowed;opacity:.6}
.pc-btn-row{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.save-pc.stop{background:#334155}
.save-pc.stop.active{background:#dc2626}
.save-pc.stop:disabled{background:#1e293b;cursor:not-allowed;opacity:.6}
.save-pc-status{font-size:11px;color:var(--tx2);margin-top:8px;
                 min-height:14px;word-break:break-all;text-align:center}
.save-pc-status.ok{color:#4ade80}
.save-pc-status.err{color:#f87171}
.pc-gap{height:10px}

footer{text-align:center;padding:12px 0 4px;color:var(--bd);
       font-size:10px;letter-spacing:2px}

@media(min-width:640px){
  .wrap{max-width:700px}
  #rc{width:480px;height:480px}
  .bg{grid-template-columns:repeat(4,1fr)}
  .bg button{font-size:15px;padding:14px 8px;letter-spacing:1px}
  .cores{grid-template-columns:repeat(4,1fr)}
}
</style>
</head>
<body>
<div class="wrap">
  <h1>WHILL CONTROL</h1>

  <!-- 現在の状態 -->
  <div class="card">
    <div class="clbl">現在の状態</div>
    <div id="state">UNKNOWN</div>
  </div>

  <!-- 状態変更ボタン -->
  <div class="bg">
    <form action="/set/STOP"   method="post"><button class="s">STOP</button></form>
    <form action="/set/HELPER" method="post"><button class="h">HELPER</button></form>
    <form action="/set/USER"   method="post"><button class="u">USER</button></form>
    <form action="/set/GAME"   method="post"><button class="g">GAME</button></form>
  </div>

  <!-- 速度調整 -->
  <div class="card">
    <div class="clbl">速度調整</div>
    <div class="gain-rows">
      <div class="gain-row">
        <span class="gain-lbl">旋回</span>
        <div class="stepper">
          <button id="dec-angular" onclick="stepGain('angular',-0.1)">−</button>
          <span class="val" id="vl-angular">1.0</span>
          <button id="inc-angular" onclick="stepGain('angular', 0.1)">+</button>
        </div>
        <button class="gain-reset" onclick="resetGain('angular')">リセット</button>
      </div>
      <div class="gain-row">
        <span class="gain-lbl">前後</span>
        <div class="stepper">
          <button id="dec-linear" onclick="stepGain('linear',-0.1)">−</button>
          <span class="val" id="vl-linear">1.0</span>
          <button id="inc-linear" onclick="stepGain('linear', 0.1)">+</button>
        </div>
        <button class="gain-reset" onclick="resetGain('linear')">リセット</button>
      </div>
    </div>
  </div>

  <!-- 点群保存 -->
  <div class="card">
    <div class="clbl">点群保存</div>
    <button id="save-pc-btn" class="save-pc" onclick="savePointcloud()">保存</button>
    <div class="save-pc-status" id="save-pc-status"></div>

    <div class="pc-gap"></div>

    <div class="pc-btn-row">
      <button id="autosave-start-btn" class="save-pc" onclick="startAutosave()">自動保存 開始</button>
      <button id="autosave-stop-btn" class="save-pc stop" onclick="stopAutosave()" disabled>自動保存 終了</button>
    </div>
    <div class="save-pc-status" id="autosave-status"></div>
  </div>

  <!-- 障害物レーダー + 点群 -->
  <div class="card">
    <div class="clbl">障害物レーダー</div>
    <div class="rw">
      <canvas id="rc" width="310" height="310"></canvas>
    </div>
  </div>

  

  <!-- バッテリー -->
  <div class="card">
    <div class="slbl">Battery</div>
    <div class="bat-row">
      <div class="bat-icon" id="bat-icon">🔋</div>
      <div class="bat-body">
        <div class="mrow">
          <div><span class="big" id="bat-val">--</span><span class="unit">%</span></div>
          <span class="badge" id="bat-badge"></span>
        </div>
        <div class="sub" id="bat-sub">取得中...</div>
        <div class="sub" id="bat-time" style="color:var(--tx);font-weight:500"></div>
        <div class="track"><div class="fill" id="bat-bar" style="width:0%"></div></div>
      </div>
    </div>
  </div>

  <!-- CPU -->
  <div class="card">
    <div class="mrow" style="margin-bottom:3px">
      <div class="slbl" style="margin-bottom:0">CPU</div>
      <div><span class="big" id="cpu-val">--</span><span class="unit">%</span></div>
    </div>
    <div class="sub" id="cpu-sub">取得中...</div>
    <div class="track"><div class="fill" id="cpu-bar" style="width:0%"></div></div>
    <div class="cores" id="cores"></div>
  </div>

  <!-- メモリ -->
  <div class="card">
    <div class="mrow" style="margin-bottom:3px">
      <div class="slbl" style="margin-bottom:0">Memory</div>
      <div><span class="big" id="mem-val">--</span><span class="unit">%</span></div>
    </div>
    <div class="sub" id="mem-sub">取得中...</div>
    <div class="track"><div class="fill" id="mem-bar" style="width:0%"></div></div>
    <div class="procs" id="procs"></div>
  </div>

  <footer>ROS2 HUMBLE &middot; FLASK &middot; SSE</footer>
</div>

<script>
// ─── 速度ゲイン (0.1刻みボタン) ─────────────────────────
const GAIN_MIN = 0.0, GAIN_MAX = 2.0;
let gains = {angular: 1.0, linear: 1.0};

function updateGainUI(axis) {
  const v = gains[axis];
  document.getElementById('vl-' + axis).textContent = v.toFixed(1);
  document.getElementById('dec-' + axis).disabled = v <= GAIN_MIN + 1e-6;
  document.getElementById('inc-' + axis).disabled = v >= GAIN_MAX - 1e-6;
}
function stepGain(axis, delta) {
  let v = gains[axis] + delta;
  v = Math.max(GAIN_MIN, Math.min(GAIN_MAX, v));
  v = Math.round(v * 10) / 10;
  if (v === gains[axis]) return;
  gains[axis] = v;
  updateGainUI(axis);
  sendGain();
}
function resetGain(axis) {
  if (gains[axis] === 1.0) return;
  gains[axis] = 1.0;
  updateGainUI(axis);
  sendGain();
}
function sendGain() {
  fetch('/set_gain', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({angular: gains.angular, linear: gains.linear})
  });
}
updateGainUI('angular');
updateGainUI('linear');

// ─── 点群保存(単発) ───────────────────────────────────
function savePointcloud() {
  const btn = document.getElementById('save-pc-btn');
  const statusEl = document.getElementById('save-pc-status');
  btn.disabled = true;
  statusEl.className = 'save-pc-status';
  statusEl.textContent = '保存中...';

  fetch('/save_pointcloud', { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      statusEl.className = 'save-pc-status ' + (d.success ? 'ok' : 'err');
      statusEl.textContent = d.success ? ('保存しました: ' + d.message) : ('失敗: ' + d.message);
    })
    .catch(err => {
      statusEl.className = 'save-pc-status err';
      statusEl.textContent = '通信エラー: ' + err;
    })
    .finally(() => {
      btn.disabled = false;
    });
}

// ─── 点群自動保存(開始・終了は独立したボタン) ───────────
// 注意: このフラグはページ内でのみ保持される表示用の状態。
// pointcloud_saver_node側の実際の記録状態を問い合わせるサービスは
// 無いため、ページ再読み込み後は「停止中」表示に戻る(実際に記録が
// 続いていても表示上はリセットされる)。
let autosaveActive = false;

function setAutosaveUI(active) {
  autosaveActive = active;
  const startBtn = document.getElementById('autosave-start-btn');
  const stopBtn  = document.getElementById('autosave-stop-btn');
  startBtn.disabled = active;
  stopBtn.disabled  = !active;
  stopBtn.classList.toggle('active', active);
}

function startAutosave() {
  const startBtn = document.getElementById('autosave-start-btn');
  const statusEl = document.getElementById('autosave-status');
  startBtn.disabled = true;
  statusEl.className = 'save-pc-status';
  statusEl.textContent = '開始中...';

  fetch('/start_pointcloud_recording', { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      statusEl.className = 'save-pc-status ' + (d.success ? 'ok' : 'err');
      statusEl.textContent = d.success ? '記録中です' : ('失敗: ' + d.message);
      setAutosaveUI(d.active);
    })
    .catch(err => {
      statusEl.className = 'save-pc-status err';
      statusEl.textContent = '通信エラー: ' + err;
      setAutosaveUI(autosaveActive);
    });
}

function stopAutosave() {
  const stopBtn = document.getElementById('autosave-stop-btn');
  const statusEl = document.getElementById('autosave-status');
  stopBtn.disabled = true;
  statusEl.className = 'save-pc-status';
  statusEl.textContent = '停止中...';

  fetch('/stop_pointcloud_recording', { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      statusEl.className = 'save-pc-status ' + (d.success ? 'ok' : 'err');
      statusEl.textContent = d.success ? '停止しました' : ('失敗: ' + d.message);
      setAutosaveUI(d.active);
    })
    .catch(err => {
      statusEl.className = 'save-pc-status err';
      statusEl.textContent = '通信エラー: ' + err;
      setAutosaveUI(autosaveActive);
    });
}

// ─── 状態 ─────────────────────────────────────────────
const stateEl = document.getElementById('state');
const SC = {STOP:'#f59e0b',HELPER:'#3b82f6',USER:'#22c55e',GAME:'#8b5cf6'};
new EventSource('/events').onmessage = e => {
  stateEl.textContent = e.data;
  stateEl.style.color = SC[e.data] || '#38bdf8';
};

// ─── レーダー + 点群描画 ───────────────────────────────
const cv = document.getElementById('rc');
const ctx = cv.getContext('2d');
const CX=155, CY=155, R0=20, R1=118, RL=138;
const D = Math.PI/180, GAP = 3;
const SECS = [
  {lbl:'前', nk:'front', a0:-135, a1:-45 },
  {lbl:'右', nk:'right', a0:-45,  a1: 45 },
  {lbl:'後', nk:'back',  a0: 45,  a1:135 },
  {lbl:'左', nk:'left',  a0:135,  a1:225 },
];
const NEAR = {front:false,right:false,back:false,left:false};

// obstacle_node の POINTS_MAX_RANGE と一致させる (m)
const POINTS_MAX_RANGE = 1.5;
const POINT_SCALE = (R1 - 2) / POINTS_MAX_RANGE;  // R1 が POINTS_MAX_RANGE に対応

let scanX = [], scanY = [];

function sec(rIn,rOut,a0d,a1d){
  ctx.beginPath();
  ctx.arc(CX,CY,rOut,(a0d+GAP)*D,(a1d-GAP)*D);
  ctx.arc(CX,CY,rIn, (a1d-GAP)*D,(a0d+GAP)*D,true);
  ctx.closePath();
}

function drawScan(){
  const n = scanX.length;
  if(!n) return;
  ctx.fillStyle = '#00ffcc';  // light cyan
  for(let i=0;i<n;i++){
    // ROS: x=前, y=左   ⇒   Canvas: dx = -y*scale, dy = -x*scale
    const dx = -scanY[i] * POINT_SCALE;
    const dy = -scanX[i] * POINT_SCALE;
    ctx.fillRect(CX + dx - 1.5, CY + dy - 1.5, 3, 3);
  }
}



function draw(){
  ctx.clearRect(0,0,310,310);
  // 外周点線リング (= POINTS_MAX_RANGE)
  ctx.beginPath();ctx.arc(CX,CY,R1,0,2*Math.PI);
  ctx.strokeStyle='#1a2535';ctx.lineWidth=0.8;
  ctx.setLineDash([4,4]);ctx.stroke();ctx.setLineDash([]);

  // セクター背景 (検知=赤、未検知=ダーク)
  SECS.forEach(s=>{
    const non = NEAR[s.nk];
    sec(R0,R1,s.a0,s.a1);
    ctx.fillStyle = non ? '#ef4444' : '#0d1520';
    ctx.fill();
    if(non){
      sec(R0,R1,s.a0,s.a1);
      ctx.strokeStyle='#f87171';ctx.lineWidth=1.5;ctx.stroke();
    }
  });

  //半径1mの円
  const R1M = R1 * (1.0 / POINTS_MAX_RANGE);
  
  ctx.beginPath();
  ctx.arc(CX, CY, R1M, 0, 2 * Math.PI);
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.5)'; // 白色、透明度50%
  ctx.lineWidth = 1;
  ctx.setLineDash([2, 4]); // 2ピクセル描画、4ピクセル空ける破線
  ctx.stroke();
  ctx.setLineDash([]); // 他の描画に影響しないよう破線をリセット
  // 点群(セクターの上に重ねる)
  drawScan();
  //追加
  // 四角形描画
const rectPts = [
  {x: 1.0, y: -0.3},
  {x: 1.0, y:  0.3},
  {x: 0.0, y:  0.3},
  {x: 0.0, y: -0.3},
];

ctx.beginPath();

rectPts.forEach((p, i) => {
  const px = CX - p.y * POINT_SCALE;
  const py = CY - p.x * POINT_SCALE;

  if (i === 0) {
    ctx.moveTo(px, py);
  } else {
    ctx.lineTo(px, py);
  }
});

ctx.closePath();
ctx.strokeStyle = '#38bdf8';
ctx.lineWidth = 2;
ctx.stroke();
  // ラベル(最前面)
  SECS.forEach(s=>{
    const non = NEAR[s.nk];
    const mid = ((s.a0+s.a1)/2)*D;
    ctx.fillStyle = non ? '#e2e8f0' : '#475569';
    ctx.font='600 14px "Helvetica Neue",Arial,sans-serif';
    ctx.textAlign='center';ctx.textBaseline='middle';
    ctx.fillText(s.lbl, CX+RL*Math.cos(mid), CY+RL*Math.sin(mid));
  });

  // 中央 WHILL アイコン
  ctx.beginPath();
  ctx.arc(CX, CY, R0 - 15, 0, 2 * Math.PI); // 少し余白を持たせる
  ctx.fillStyle = '#0d1520'; // セクターの背景と同じダークカラー
  ctx.fill();
  ctx.strokeStyle = '#38bdf8'; // アクセントカラーの青で縁取り
  ctx.lineWidth = 2; // 少し太めの縁取り
  ctx.stroke();
}
draw();

new EventSource('/obstacle_events').onmessage = e => {
  const d = JSON.parse(e.data);
  Object.assign(NEAR, d.near);
  draw();
};

new EventSource('/scan_events').onmessage = e => {
  const d = JSON.parse(e.data);
  scanX = d.x || [];
  scanY = d.y || [];
  draw();
};

// ─── システム情報 SSE ────────────────────────────────────
function barC(p){ return p>=85?'#ef4444':p>=60?'#f59e0b':'#22c55e'; }
function batC(p){ return p<=20?'#ef4444':p<=40?'#f59e0b':'#22c55e'; }

function setBar(id,pct,colorFn){
  const el = document.getElementById(id);
  el.style.width = pct + '%';
  el.style.background = colorFn(pct);
}
function updateCores(cores){
  const el = document.getElementById('cores');
  el.innerHTML = cores.map((v,i) =>
    `<div class="core-item">
       <div class="core-top">
         <span class="core-name">Core&nbsp;${i}</span>
         <span class="core-pct">${v}%</span>
       </div>
       <div class="track">
         <div class="fill" style="width:${v}%;background:${barC(v)}"></div>
       </div>
     </div>`
  ).join('');
}
function updateProcs(procs){
  const el = document.getElementById('procs');
  if(!procs.length){ el.innerHTML=''; return; }
  const maxMb = procs[0].mb || 1;
  el.innerHTML = procs.map((p,i) =>
    `<div class="proc-row">
       <div class="proc-rank">${i+1}</div>
       <div class="proc-name">${p.name}</div>
       <div class="proc-mb">${p.mb >= 1024
         ? (p.mb/1024).toFixed(1)+' GB'
         : p.mb.toFixed(0)+' MB'}</div>
       <div class="proc-bg">
         <div class="proc-fg" style="width:${Math.round(p.mb/maxMb*100)}%"></div>
       </div>
     </div>`
  ).join('');
}

new EventSource('/stats_events').onmessage = e => {
  const d = JSON.parse(e.data);
  document.getElementById('cpu-val').textContent = d.cpu;
  document.getElementById('cpu-sub').textContent =
    (d.cores.length ? d.cores.length : '--') + ' コア';
  setBar('cpu-bar', d.cpu, barC);
  updateCores(d.cores);

  document.getElementById('mem-val').textContent = d.mem;
  document.getElementById('mem-sub').textContent =
    d.mem_used + ' GB / ' + d.mem_total + ' GB';
  setBar('mem-bar', d.mem, barC);
  updateProcs(d.procs);

  if(d.bat_pct !== null){
    const p = d.bat_pct, c = d.bat_charge;
    document.getElementById('bat-val').textContent  = p;
    document.getElementById('bat-icon').textContent = c ? '⚡' : p<=20 ? '🪫' : '🔋';
    document.getElementById('bat-sub').textContent  = c ? '充電中' : 'バッテリー駆動';

    const tEl = document.getElementById('bat-time');
    if(d.bat_secsleft !== null){
      const h = Math.floor(d.bat_secsleft / 3600);
      const m = Math.floor((d.bat_secsleft % 3600) / 60);
      tEl.textContent = c
        ? `満充電まで約 ${h}時間${m}分`
        : `残り約 ${h}時間${m}分`;
    } else {
      tEl.textContent = '';
    }
    const badge = document.getElementById('bat-badge');
    if(c){
      badge.textContent = '充電中';
      badge.style.cssText = 'background:#14532d;color:#86efac';
    } else if(p <= 20){
      badge.textContent = '残量少';
      badge.style.cssText = 'background:#450a0a;color:#fca5a5';
    } else {
      badge.textContent = '';
      badge.style.cssText = '';
    }
    setBar('bat-bar', p, batC);
  } else {
    document.getElementById('bat-val').textContent  = 'N/A';
    document.getElementById('bat-icon').textContent = '🔌';
    document.getElementById('bat-sub').textContent  = 'バッテリーなし';
  }
};
</script>
</body>
</html>"""


@app.route("/")
def home():
    return HTML

@app.route("/set/<mode>", methods=["POST"])
def set_mode(mode):
    move_to_target(mode)
    return redirect("/")


@app.route("/set_gain", methods=["POST"])
def set_gain():
    data = flask_request.get_json(force=True)
    ros_node._angular_gain = max(0.0, min(2.0, float(data.get('angular', 1.0))))
    ros_node._linear_gain  = max(0.0, min(2.0, float(data.get('linear',  1.0))))
    ros_node.publish_gain()
    return {"angular": ros_node._angular_gain, "linear": ros_node._linear_gain}


@app.route("/save_pointcloud", methods=["POST"])
def save_pointcloud():
    success, message = ros_node.call_save_pointcloud()
    return {"success": success, "message": message}


@app.route("/start_pointcloud_recording", methods=["POST"])
def start_pointcloud_recording():
    success, message = ros_node.call_start_autosave()
    return {"success": success, "message": message, "active": ros_node.autosave_active}


@app.route("/stop_pointcloud_recording", methods=["POST"])
def stop_pointcloud_recording():
    success, message = ros_node.call_stop_autosave()
    return {"success": success, "message": message, "active": ros_node.autosave_active}


# =========================
# main
# =========================
def ros_spin():
    rclpy.spin(ros_node)

def main():
    global ros_node
    rclpy.init()
    ros_node = WebGuiNode()
    threading.Thread(target=ros_spin,      daemon=True).start()
    threading.Thread(target=collect_stats, daemon=True).start()
    app.run(host="0.0.0.0", port=5000, threaded=True)

if __name__ == "__main__":
    main()