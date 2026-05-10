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
from togakushi2_interfaces.msg import Obstacle

from flask import Flask, Response, redirect, request as flask_request

# =========================
# ROS2 Node
# =========================
class WebGuiNode(Node):
    def __init__(self):
        super().__init__('web_ui_node')

        self.current_state = "UNKNOWN"
        self._near = {"front": False, "right": False, "back": False, "left": False}
        self._far  = {
            "front_left": False, "front": False, "front_right": False,
            "right": False,
            "back_right": False, "back": False, "back_left": False,
            "left": False,
        }

        self.pub = self.create_publisher(String, '/state_transition', 10)
        self.gain_pub = self.create_publisher(Float32MultiArray, '/speed_gain', 10)
        self._angular_gain = 1.0
        self._linear_gain  = 1.0
        self.sub = self.create_subscription(String, '/robot_state', self.state_callback, 10)
        self.obstacle_sub = self.create_subscription(
            Obstacle, '/obstacle_info', self.obstacle_callback, 10)
        self.obstacle_far_sub = self.create_subscription(
            Obstacle, '/obstacle_info_far', self.obstacle_far_callback, 10)

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
        push_obstacle(self._near, self._far)

    def obstacle_far_callback(self, msg):
        if len(msg.data) < 8:
            return
        self._far = {
            "front_left":  bool(msg.data[0]), "front":      bool(msg.data[1]),
            "front_right": bool(msg.data[2]), "right":      bool(msg.data[3]),
            "back_right":  bool(msg.data[4]), "back":       bool(msg.data[5]),
            "back_left":   bool(msg.data[6]), "left":       bool(msg.data[7]),
        }
        push_obstacle(self._near, self._far)

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
stats_clients    = []


def _push(lst, payload):
    dead = []
    for q in lst:
        try:   q.put_nowait(payload)
        except: dead.append(q)
    for q in dead:
        if q in lst: lst.remove(q)

def push_state(state):        _push(clients,          state)
def push_obstacle(near, far): _push(obstacle_clients, json.dumps({"near": near, "far": far}))
def push_stats(stats):        _push(stats_clients,    json.dumps(stats))


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
    psutil.cpu_percent(percpu=True)   # 初回は基準値を作るだけ
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
                if bat and bat.secsleft >= 0   # -1=不明, -2=AC接続
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
            "near": {"front":False,"right":False,"back":False,"left":False},
            "far":  {"front_left":False,"front":False,"front_right":False,
                     "right":False,"back_right":False,"back":False,
                     "back_left":False,"left":False}
        })
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

/* ラベル（濃く） */
.clbl{font-size:10px;letter-spacing:2px;text-transform:uppercase;
      color:var(--tx);font-weight:500;text-align:center;margin-bottom:10px}
.slbl{font-size:10px;letter-spacing:2px;text-transform:uppercase;
      color:var(--tx);font-weight:500;margin-bottom:10px}

/* 状態 */
#state{font-size:36px;font-weight:700;text-align:center;
       letter-spacing:4px;color:var(--ac);transition:color .25s}

/* レーダー */
.rw{display:flex;flex-direction:column;align-items:center;gap:10px}
#rc{display:block;width:310px;height:310px}
.leg{display:flex;gap:16px;font-size:11px;color:var(--tx2)}
.li{display:flex;align-items:center;gap:5px}
.ld{width:9px;height:9px;border-radius:50%}
.nc{background:#ef4444}.fc{background:#f97316}

/* 共通メトリクス */
.mrow{display:flex;justify-content:space-between;align-items:baseline;margin-bottom:3px}
.big{font-size:24px;font-weight:700;color:var(--tx);line-height:1}
.unit{font-size:12px;color:var(--tx2);margin-left:2px}
.sub{font-size:11px;color:var(--tx2);margin-bottom:7px}
.track{height:5px;background:#1a2535;border-radius:3px;overflow:hidden}
.fill{height:100%;border-radius:3px;transition:width .5s ease,background .5s ease}
.badge{font-size:11px;padding:2px 8px;border-radius:8px;font-weight:500}

/* バッテリー */
.bat-row{display:flex;align-items:center;gap:14px}
.bat-icon{font-size:28px;flex-shrink:0}
.bat-body{flex:1}

/* コアグリッド (スマホ2列 / PC4列) */
.cores{display:grid;grid-template-columns:repeat(2,1fr);gap:6px;margin-top:10px}
.core-item{display:flex;flex-direction:column;gap:3px}
.core-top{display:flex;justify-content:space-between;font-size:11px}
.core-name{color:var(--tx);font-weight:500}
.core-pct{color:var(--tx2)}

/* プロセス一覧 */
.procs{display:flex;flex-direction:column;gap:5px;margin-top:10px}
.proc-row{display:flex;align-items:center;gap:8px}
.proc-rank{font-size:11px;color:var(--tx3);width:14px;text-align:right;flex-shrink:0}
.proc-name{font-size:12px;color:var(--tx);flex:1;
           white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.proc-mb{font-size:11px;color:var(--tx2);flex-shrink:0;min-width:64px;text-align:right}
.proc-bg{width:56px;height:4px;background:#1a2535;border-radius:2px;flex-shrink:0;overflow:hidden}
.proc-fg{height:100%;border-radius:2px;background:#38bdf8;transition:width .5s}

/* 速度調整 */
.gain-rows{display:flex;flex-direction:column;gap:10px}
.gain-row{display:grid;grid-template-columns:36px 1fr 38px 44px;align-items:center;gap:8px}
.gain-lbl{font-size:12px;color:var(--tx);font-weight:500}
.gain-slider{width:100%;accent-color:var(--ac);min-width:0}
.gain-val{font-size:15px;font-weight:700;color:var(--ac);text-align:right}
.gain-reset{font-size:10px;padding:3px 0;width:100%;border:1px solid var(--bd);
            border-radius:6px;background:transparent;color:var(--tx2);cursor:pointer;
            white-space:nowrap;text-align:center}
.gain-reset:active{background:var(--bd)}
.bg{display:grid;grid-template-columns:repeat(2,1fr);gap:10px;margin-bottom:12px}
.bg form{margin:0}
button{width:100%;border:none;border-radius:13px;padding:17px 8px;
       font-size:17px;font-weight:700;letter-spacing:2px;color:#fff;
       cursor:pointer;transition:opacity .15s,transform .1s}
button:active{opacity:.8;transform:scale(.97)}
.s{background:#d97706}.h{background:#2563eb}
.u{background:#16a34a}.g{background:#7c3aed}
footer{text-align:center;padding:12px 0 4px;color:var(--bd);
       font-size:10px;letter-spacing:2px}

/* PC (640px以上) */
@media(min-width:640px){
  .wrap{max-width:700px}
  #rc{width:480px;height:480px}
  .bg{grid-template-columns:repeat(4,1fr)}
  button{font-size:15px;padding:14px 8px;letter-spacing:1px}
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
        <input class="gain-slider" type="range" min="0" max="2" step="0.05"
               value="1" id="sl-angular" oninput="onGain()">
        <span class="gain-val" id="vl-angular">1.0</span>
        <button class="gain-reset" onclick="resetGain('angular')">リセット</button>
      </div>
      <div class="gain-row">
        <span class="gain-lbl">前後</span>
        <input class="gain-slider" type="range" min="0" max="2" step="0.05"
               value="1" id="sl-linear" oninput="onGain()">
        <span class="gain-val" id="vl-linear">1.0</span>
        <button class="gain-reset" onclick="resetGain('linear')">リセット</button>
      </div>
    </div>
  </div>

  <!-- 障害物レーダー -->
  <div class="card">
    <div class="clbl">障害物レーダー</div>
    <div class="rw">
      <canvas id="rc" width="310" height="310"></canvas>
      <div class="leg">
        <div class="li"><div class="ld nc"></div>近距離（内リング）</div>
        <div class="li"><div class="ld fc"></div>遠方（外リング）</div>
      </div>
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
// ─── 速度ゲイン ──────────────────────────────────────────
let gainTimer = null;
function onGain() {
  const a = parseFloat(document.getElementById('sl-angular').value);
  const l = parseFloat(document.getElementById('sl-linear').value);
  document.getElementById('vl-angular').textContent = a.toFixed(2);
  document.getElementById('vl-linear').textContent  = l.toFixed(2);
  // 連続操作時は最後の値だけ送る（100ms デバウンス）
  clearTimeout(gainTimer);
  gainTimer = setTimeout(() => sendGain(a, l), 100);
}
function resetGain(axis) {
  document.getElementById('sl-' + axis).value = 1;
  onGain();
}
function sendGain(angular, linear) {
  fetch('/set_gain', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({angular, linear})
  });
}

const stateEl = document.getElementById('state');
const SC = {STOP:'#f59e0b',HELPER:'#3b82f6',USER:'#22c55e',GAME:'#8b5cf6'};
new EventSource('/events').onmessage = e => {
  stateEl.textContent = e.data;
  stateEl.style.color = SC[e.data] || '#38bdf8';
};

// ─── レーダー ─────────────────────────────────────────────
const cv = document.getElementById('rc');
const ctx = cv.getContext('2d');
const CX=155, CY=155, R0=30, R1=72, R2=118, RL=134;
const D = Math.PI/180, GAP = 3;
const SECS = [
  {lbl:'前',  fk:'front',       nk:'front', a0:-112.5,a1:-67.5, hasN:true },
  {lbl:'右前', fk:'front_right', nk:null,    a0:-67.5, a1:-22.5, hasN:false},
  {lbl:'右',  fk:'right',       nk:'right', a0:-22.5, a1:22.5,  hasN:true },
  {lbl:'右後', fk:'back_right',  nk:null,    a0:22.5,  a1:67.5,  hasN:false},
  {lbl:'後',  fk:'back',        nk:'back',  a0:67.5,  a1:112.5, hasN:true },
  {lbl:'左後', fk:'back_left',   nk:null,    a0:112.5, a1:157.5, hasN:false},
  {lbl:'左',  fk:'left',        nk:'left',  a0:157.5, a1:202.5, hasN:true },
  {lbl:'左前', fk:'front_left',  nk:null,    a0:202.5, a1:247.5, hasN:false},
];
const NEAR = {front:false,right:false,back:false,left:false};
const FAR  = {front:false,front_right:false,right:false,back_right:false,
              back:false,back_left:false,left:false,front_left:false};

function sec(rIn,rOut,a0d,a1d){
  ctx.beginPath();
  ctx.arc(CX,CY,rOut,(a0d+GAP)*D,(a1d-GAP)*D);
  ctx.arc(CX,CY,rIn, (a1d-GAP)*D,(a0d+GAP)*D,true);
  ctx.closePath();
}
function draw(){
  ctx.clearRect(0,0,310,310);
  [R1,R2].forEach(r=>{
    ctx.beginPath();ctx.arc(CX,CY,r,0,2*Math.PI);
    ctx.strokeStyle='#1a2535';ctx.lineWidth=0.8;
    ctx.setLineDash([4,4]);ctx.stroke();ctx.setLineDash([]);
  });
  SECS.forEach(s=>{
    const fon=FAR[s.fk], non=s.hasN&&NEAR[s.nk];
    sec(R1,R2,s.a0,s.a1);
    ctx.fillStyle=fon?'#f97316':'#0f1a28';ctx.fill();
    if(fon){sec(R1,R2,s.a0,s.a1);ctx.strokeStyle='#fb923c';ctx.lineWidth=1.5;ctx.stroke();}
    if(s.hasN){
      sec(R0,R1,s.a0,s.a1);
      ctx.fillStyle=non?'#ef4444':'#0d1520';ctx.fill();
      if(non){sec(R0,R1,s.a0,s.a1);ctx.strokeStyle='#f87171';ctx.lineWidth=1.5;ctx.stroke();}
    }
    const mid=((s.a0+s.a1)/2)*D;
    ctx.fillStyle=(fon||non)?'#e2e8f0':'#475569';
    ctx.font=(s.hasN?'600 14px':'400 12px')+' "Helvetica Neue",Arial,sans-serif';
    ctx.textAlign='center';ctx.textBaseline='middle';
    ctx.fillText(s.lbl, CX+RL*Math.cos(mid), CY+RL*Math.sin(mid));
  });
  ctx.beginPath();ctx.arc(CX,CY,R0-3,0,2*Math.PI);
  ctx.fillStyle='#07090f';ctx.fill();
  ctx.strokeStyle='#334155';ctx.lineWidth=1.5;ctx.stroke();
  ctx.beginPath();ctx.moveTo(CX,CY-R0+9);ctx.lineTo(CX-4,CY-R0+17);ctx.lineTo(CX+4,CY-R0+17);
  ctx.closePath();ctx.fillStyle='#38bdf8';ctx.fill();
  ctx.fillStyle='#334155';ctx.font='700 8px sans-serif';
  ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText('WHILL',CX,CY+8);
}
draw();
new EventSource('/obstacle_events').onmessage = e => {
  const d = JSON.parse(e.data);
  Object.assign(NEAR,d.near); Object.assign(FAR,d.far); draw();
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

  // CPU
  document.getElementById('cpu-val').textContent = d.cpu;
  document.getElementById('cpu-sub').textContent =
    (d.cores.length ? d.cores.length : '--') + ' コア';
  setBar('cpu-bar', d.cpu, barC);
  updateCores(d.cores);

  // メモリ
  document.getElementById('mem-val').textContent = d.mem;
  document.getElementById('mem-sub').textContent =
    d.mem_used + ' GB / ' + d.mem_total + ' GB';
  setBar('mem-bar', d.mem, barC);
  updateProcs(d.procs);

  // バッテリー
  if(d.bat_pct !== null){
    const p = d.bat_pct, c = d.bat_charge;
    document.getElementById('bat-val').textContent  = p;
    document.getElementById('bat-icon').textContent = c ? '⚡' : p<=20 ? '🪫' : '🔋';
    document.getElementById('bat-sub').textContent  = c ? '充電中' : 'バッテリー駆動';

    // 残り稼働時間
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