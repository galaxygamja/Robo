from __future__ import annotations

import json
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import urlparse

from .simulation import SimulationEngine


_DASHBOARD = r'''<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Robo Control · 6대 관제 시뮬레이터</title>
  <style>
    :root{color-scheme:dark;--bg:#07111f;--panel:#0d1b2d;--line:#243b55;--text:#e8f1fb;--muted:#8ca4bd;--cyan:#3dd9eb;--green:#47e6a1;--red:#ff667d;--amber:#ffd166}
    *{box-sizing:border-box} body{margin:0;background:radial-gradient(circle at 15% 0,#142d47 0,var(--bg) 45%);color:var(--text);font:14px/1.45 system-ui,-apple-system,"Noto Sans KR",sans-serif}
    header{display:flex;justify-content:space-between;align-items:end;padding:22px 26px 14px;border-bottom:1px solid var(--line)}
    h1{font-size:23px;margin:0} h1 span{color:var(--cyan)} .sub{color:var(--muted);margin-top:4px}
    .badge{padding:6px 11px;border-radius:999px;background:#19304a;color:var(--cyan);font-weight:700;text-transform:uppercase}
    main{display:grid;grid-template-columns:minmax(480px,1fr) 340px;gap:16px;padding:16px;max-width:1450px;margin:auto}
    .panel{background:linear-gradient(145deg,rgba(18,38,61,.95),rgba(9,24,41,.96));border:1px solid var(--line);border-radius:14px;box-shadow:0 18px 55px #0005}
    .arena-panel{padding:14px}.toolbar{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:12px}
    button{border:1px solid #315475;background:#142a43;color:var(--text);border-radius:8px;padding:9px 13px;font-weight:700;cursor:pointer}button:hover{border-color:var(--cyan);color:var(--cyan)}button.danger{border-color:#70384a;color:#ff9aaa}
    canvas{display:block;width:100%;aspect-ratio:1.143/1.181;max-height:75vh;background:#081522;border:1px solid #34516d;border-radius:9px}
    aside{display:grid;gap:12px;align-content:start}.card{padding:14px}.card h2{font-size:13px;letter-spacing:.08em;text-transform:uppercase;color:var(--muted);margin:0 0 10px}
    .timer{font:700 40px/1 ui-monospace,monospace;color:var(--amber)}.metrics{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:12px}.metric{background:#081725;border-radius:8px;padding:9px}.metric b{display:block;font-size:18px;color:var(--cyan)}
    table{width:100%;border-collapse:collapse;font-size:12px}td,th{text-align:left;padding:6px;border-bottom:1px solid #21384f}.dot{width:8px;height:8px;border-radius:50%;display:inline-block;margin-right:5px;background:var(--green)}
    .warning{color:#ffc986;background:#302510;padding:9px;border-radius:8px;margin-top:7px}.event{color:#ff9aaa;border-left:2px solid var(--red);padding-left:8px;margin:7px 0}.empty{color:var(--muted)}
    footer{color:var(--muted);padding:0 18px 18px;text-align:center;font-size:12px}
    @media(max-width:900px){main{grid-template-columns:1fr}.arena-panel{padding:8px}header{padding:16px;align-items:start}.sub{max-width:70vw}}
  </style>
</head>
<body>
<header><div><h1><span>ROBO</span> CONTROL LAB</h1><div class="sub">하드웨어 독립형 6대 로봇 · 시간예약 A* · 120초 예선 시뮬레이션</div></div><div id="status" class="badge">연결 중</div></header>
<main>
  <section class="panel arena-panel">
    <div class="toolbar">
      <button onclick="act('start')">▶ 시작</button><button onclick="act('pause')">Ⅱ 일시정지</button><button onclick="act('reset')">↺ 초기화</button><button onclick="act('replan')">⌘ 재계획</button><button class="danger" onclick="act('stop')">■ 긴급정지</button>
    </div>
    <canvas id="arena" width="900" height="930"></canvas>
  </section>
  <aside>
    <section class="panel card"><h2>경기 시간</h2><div id="timer" class="timer">120.0</div><div class="metrics"><div class="metric"><span>완료 임무</span><b id="done">0 / 6</b></div><div class="metric"><span>계획 시간</span><b id="plan">0 ms</b></div><div class="metric"><span>재계획</span><b id="replans">0</b></div><div class="metric"><span>충돌</span><b id="collisions">0</b></div></div></section>
    <section class="panel card"><h2>로봇 상태</h2><table><thead><tr><th>ID</th><th>상태</th><th>임무</th><th>배터리</th></tr></thead><tbody id="robots"></tbody></table></section>
    <section class="panel card"><h2>안전 이벤트</h2><div id="events" class="empty">이벤트 없음</div></section>
    <section class="panel card"><h2>중요 안내</h2><div id="warnings"></div></section>
  </aside>
</main>
<footer>이 화면은 소프트웨어 검증용입니다. 시뮬레이션 성공은 실물 로봇·카메라·드론 검증을 대신하지 않습니다.</footer>
<script>
const canvas=document.querySelector('#arena'),ctx=canvas.getContext('2d');let state=null;
const colors=['#3dd9eb','#47e6a1','#ffd166','#ff7aa2','#9b8cff','#ff995c'];
function xy(p){return [p.x/state.field.width_m*canvas.width,p.y/state.field.height_m*canvas.height]}
function rect(r,fill,stroke){let [x,y]=xy(r),w=r.width/state.field.width_m*canvas.width,h=r.height/state.field.height_m*canvas.height;ctx.fillStyle=fill;ctx.fillRect(x,y,w,h);ctx.strokeStyle=stroke;ctx.strokeRect(x,y,w,h)}
function draw(){if(!state)return;ctx.clearRect(0,0,canvas.width,canvas.height);ctx.fillStyle='#071522';ctx.fillRect(0,0,canvas.width,canvas.height);
  ctx.strokeStyle='#16324b';ctx.lineWidth=1;for(let i=0;i<=20;i++){let x=i*canvas.width/20,y=i*canvas.height/20;ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,canvas.height);ctx.stroke();ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(canvas.width,y);ctx.stroke()}
  rect(state.field.start_zone,'#19453a88','#47e6a1');state.obstacles.forEach(o=>rect(o,'#5c3342','#ff8294'));
  state.tasks.forEach((t,i)=>{let [x,y]=xy(t.position);ctx.beginPath();ctx.arc(x,y,11,0,Math.PI*2);ctx.fillStyle=t.completed?'#47e6a1':colors[i%colors.length];ctx.fill();ctx.fillStyle='#06111d';ctx.font='bold 10px system-ui';ctx.textAlign='center';ctx.fillText(t.id,x,y+3)});
  state.robots.forEach((r,i)=>{if(r.path.length){ctx.strokeStyle=colors[i%colors.length]+'99';ctx.lineWidth=3;ctx.setLineDash([7,6]);ctx.beginPath();r.path.forEach((p,j)=>{let [x,y]=xy(p);j?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.stroke();ctx.setLineDash([])}let [x,y]=xy(r.position),radius=.063/state.field.width_m*canvas.width;ctx.save();ctx.translate(x,y);ctx.rotate(r.heading_rad);ctx.fillStyle=colors[i%colors.length];ctx.strokeStyle='#fff';ctx.lineWidth=2;ctx.beginPath();ctx.roundRect(-radius,-radius*.78,radius*2,radius*1.56,8);ctx.fill();ctx.stroke();ctx.beginPath();ctx.moveTo(radius*.25,0);ctx.lineTo(radius*.7,0);ctx.stroke();ctx.restore();ctx.fillStyle='#fff';ctx.font='bold 12px system-ui';ctx.textAlign='center';ctx.fillText(r.id,x,y+4)});
  ctx.strokeStyle='#7ba2c5';ctx.lineWidth=3;ctx.strokeRect(1.5,1.5,canvas.width-3,canvas.height-3)}
function update(s){state=s;document.querySelector('#status').textContent=s.status;document.querySelector('#timer').textContent=s.remaining_s.toFixed(1);document.querySelector('#done').textContent=`${s.metrics.completed_tasks} / ${s.metrics.total_tasks}`;document.querySelector('#plan').textContent=s.metrics.planning_ms.toFixed(1)+' ms';document.querySelector('#replans').textContent=s.metrics.replan_count;document.querySelector('#collisions').textContent=s.metrics.collision_count;
 document.querySelector('#robots').innerHTML=s.robots.map(r=>`<tr><td><i class="dot"></i>${r.id}</td><td>${r.status}</td><td>${r.task_id||'-'}</td><td>${r.battery_percent.toFixed(1)}%</td></tr>`).join('');document.querySelector('#warnings').innerHTML=s.warnings.map(w=>`<div class="warning">${w}</div>`).join('');document.querySelector('#events').innerHTML=s.events.length?s.events.slice().reverse().map(e=>`<div class="event">${e.timestamp_s.toFixed(2)}s · ${e.message}</div>`).join(''):'<span class="empty">이벤트 없음</span>';draw()}
async function poll(){try{let r=await fetch('/api/state',{cache:'no-store'});update(await r.json())}catch(e){document.querySelector('#status').textContent='연결 끊김'}}
async function act(action){await fetch('/api/mission/'+action,{method:'POST'});await poll()}
setInterval(poll,200);poll();
</script></body></html>'''


class ControlRequestHandler(BaseHTTPRequestHandler):
    engine: SimulationEngine

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _send(self, status: HTTPStatus, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        self._send(
            status,
            json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
            "application/json; charset=utf-8",
        )

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path == "/":
            self._send(HTTPStatus.OK, _DASHBOARD.encode("utf-8"), "text/html; charset=utf-8")
        elif path == "/api/state":
            self._json(HTTPStatus.OK, self.engine.snapshot())
        elif path == "/healthz":
            self._json(HTTPStatus.OK, {"ok": True, "status": self.engine.status.value})
        else:
            self._json(HTTPStatus.NOT_FOUND, {"error": "not_found"})

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        actions = {
            "/api/mission/start": self.engine.start,
            "/api/mission/pause": self.engine.pause,
            "/api/mission/reset": self.engine.reset,
            "/api/mission/replan": self.engine.replan,
            "/api/mission/stop": self.engine.emergency_stop,
        }
        action = actions.get(path)
        if action is None:
            self._json(HTTPStatus.NOT_FOUND, {"error": "unknown_action"})
            return
        action()
        self._json(HTTPStatus.OK, {"ok": True, "state": self.engine.snapshot()})


def make_server(engine: SimulationEngine, host: str, port: int) -> ThreadingHTTPServer:
    handler = type("BoundControlRequestHandler", (ControlRequestHandler,), {"engine": engine})
    return ThreadingHTTPServer((host, port), handler)


def serve(engine: SimulationEngine, host: str, port: int) -> None:
    engine.run_background()
    server = make_server(engine, host, port)
    try:
        server.serve_forever(poll_interval=0.25)
    finally:
        server.shutdown()
        server.server_close()
        engine.close()


def serve_in_thread(
    engine: SimulationEngine, host: str, port: int
) -> tuple[ThreadingHTTPServer, threading.Thread]:
    engine.run_background()
    server = make_server(engine, host, port)
    thread = threading.Thread(target=server.serve_forever, name="dashboard", daemon=True)
    thread.start()
    return server, thread

