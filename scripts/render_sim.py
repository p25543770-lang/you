"""Визуализация миссии: один самодостаточный HTML с канвасом (открывается без сервера).

Запуск: .venv/bin/python scripts/render_sim.py DATA.json OUT.html
DATA.json — результат запуска run_rect_mission (см. docs/sim/README.md).
"""
import json
import sys

TEMPLATE = r"""<!doctype html>
<html lang="ru"><head><meta charset="utf-8">
<title>Симуляция: робот 9:16 по лидару</title>
<style>
 body{font-family:system-ui,sans-serif;margin:16px;background:#f5f6f8;color:#222}
 #wrap{display:flex;gap:16px;flex-wrap:wrap}
 canvas{background:#fff;border:1px solid #ccc;border-radius:6px}
 .panel{min-width:260px}
 .row{margin:6px 0}
 button{padding:6px 12px;margin-right:6px}
 .big{font-size:20px;font-weight:600}
 .bad{color:#c0392b}.ok{color:#1e8449}
 table td{padding:2px 8px 2px 0}
 .legend span{display:inline-block;width:12px;height:12px;margin-right:4px;vertical-align:middle}
</style></head><body>
<h2>Симуляция: робот 0.60 × 0.75 м с хитбоксом, движется только по лидару</h2>
<div id="wrap">
 <div>
  <canvas id="c"></canvas>
  <div class="row">
   <button id="play">Пауза</button>
   <button id="restart">В начало</button>
   <label>Скорость
    <select id="speed"><option>0.5</option><option selected>1</option><option>4</option><option>10</option></select>
   </label>
   <label><input type="checkbox" id="showmap" checked> карта (итог)</label>
   <label><input type="checkbox" id="showlidar" checked> лидар</label>
   <label><input type="checkbox" id="showest" checked> оценка позы</label>
  </div>
  <div class="row"><input type="range" id="seek" min="0" max="1000" value="0" style="width:1000px"></div>
  <div class="legend row">
   <span style="background:#e67e22"></span>корпус (истина)
   <span style="background:#c0392b"></span>касание (хитбокс)
   <span style="background:#27ae60"></span>оценка по лидару
   <span style="background:#2e86c1"></span>план Hybrid A*
   <span style="background:#f4d03f"></span>лучи лидара
   <span style="background:#7f8c8d"></span>стеллажи
   <span style="background:#e74c3c"></span>касания за миссию (×)
  </div>
 </div>
 <div class="panel">
  <div class="big" id="status"></div>
  <table>
   <tr><td>Время</td><td id="t"></td></tr>
   <tr><td>Касаний до сих пор</td><td id="touch"></td></tr>
   <tr><td>Точек пройдено</td><td id="wpc"></td></tr>
   <tr><td>Ошибка оценки позы</td><td id="err"></td></tr>
   <tr><td>Пройдено пути</td><td id="odo"></td></tr>
   <tr><td>Перепланирований</td><td id="rep"></td></tr>
  </table>
  <div class="row" id="summary"></div>
 </div>
</div>
<script>
const DATA = __DATA__;
const W = DATA.arena[0], H = DATA.arena[1];
const cv = document.getElementById('c'), ctx = cv.getContext('2d');
cv.width = 1000; cv.height = Math.round(1000 * H / W);
const S = cv.width / W;
const BODY_W = DATA.body.w, BODY_L = DATA.body.l;
const frames = DATA.frames.filter(f => f.t !== undefined);
const tEnd = frames.length ? frames[frames.length - 1].t : 0;
const X = x => x * S, Y = y => cv.height - y * S;

function rectPoly(x, y, th, w, l) {
  const c = Math.cos(th), s = Math.sin(th), hw = w / 2, hl = l / 2;
  return [[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]].map(([lx, ly]) => [x + lx * c - ly * s, y + lx * s + ly * c]);
}
function drawPoly(pts, fill, stroke, dash) {
  ctx.beginPath();
  pts.forEach((p, i) => i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])));
  ctx.closePath();
  if (fill) { ctx.fillStyle = fill; ctx.fill(); }
  if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = 2; ctx.setLineDash(dash || []); ctx.stroke(); ctx.setLineDash([]); }
}
function frameAt(t) {
  let lo = 0, hi = frames.length - 1;
  while (lo < hi) { const m = (lo + hi + 1) >> 1; if (frames[m].t <= t) lo = m; else hi = m - 1; }
  return frames[lo];
}
function planAt(t) {
  // последний план, выданный до момента t
  let p = 0;
  for (const e of DATA.frames) { if (e.ev === 'plan' && e.t <= t) p = e.path; if (e.t !== undefined && e.t > t) break; }
  return p;
}
function draw(t) {
  const f = frameAt(t);
  ctx.clearRect(0, 0, cv.width, cv.height);
  // пол и склад
  ctx.fillStyle = '#fbfbfd'; ctx.fillRect(0, 0, cv.width, cv.height);
  if (document.getElementById('showmap').checked) {
    ctx.fillStyle = 'rgba(231,76,60,0.12)';
    for (const [i, j] of DATA.map_occ) ctx.fillRect(X(i * 0.1), Y((j + 1) * 0.1), S * 0.1, S * 0.1);
  }
  ctx.strokeStyle = '#333'; ctx.lineWidth = 4; ctx.strokeRect(0, 0, W * S, H * S);
  for (const [x0, y0, x1, y1] of DATA.shelves) {
    ctx.fillStyle = '#7f8c8d';
    ctx.fillRect(X(x0), Y(y1), (x1 - x0) * S, (y1 - y0) * S);
  }
  // точки маршрута
  DATA.waypoints.forEach((w, i) => {
    ctx.beginPath(); ctx.arc(X(w[0]), Y(w[1]), 9, 0, 2 * Math.PI);
    ctx.fillStyle = i < f.wp ? '#27ae60' : '#f39c12'; ctx.fill();
    ctx.fillStyle = '#fff'; ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(String(i + 1), X(w[0]), Y(w[1]));
  });
  // текущий план
  const pid = f.path || 0;
  if (pid && DATA.paths[pid]) {
    ctx.beginPath(); DATA.paths[pid].forEach((p, i) => i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])));
    ctx.strokeStyle = '#2e86c1'; ctx.lineWidth = 3; ctx.stroke();
  }
  // след истинной позы
  const idx = frames.indexOf(f);
  ctx.beginPath();
  for (let k = Math.max(0, idx - 400); k <= idx; k++) {
    const q = frames[k]; if (q.tx === undefined) continue;
    k === Math.max(0, idx - 400) ? ctx.moveTo(X(q.tx), Y(q.ty)) : ctx.lineTo(X(q.tx), Y(q.ty));
  }
  ctx.strokeStyle = 'rgba(230,126,34,0.5)'; ctx.lineWidth = 2; ctx.stroke();
  // лучи лидара
  if (document.getElementById('showlidar').checked && f.lidar) {
    for (let b = 0; b < f.lidar.length; b++) {
      const a = f.tth + 2 * Math.PI * b / f.lidar.length;
      const r = f.lidar[b];
      ctx.beginPath(); ctx.moveTo(X(f.tx), Y(f.ty));
      ctx.lineTo(X(f.tx + r * Math.cos(a)), Y(f.ty + r * Math.sin(a)));
      ctx.strokeStyle = 'rgba(241,196,15,0.7)'; ctx.lineWidth = 1; ctx.stroke();
    }
  }
  // оценка позы
  if (document.getElementById('showest').checked) {
    drawPoly(rectPoly(f.ex, f.ey, f.eth, BODY_W, BODY_L), null, '#27ae60', [6, 4]);
  }
  // истинный корпус и хитбокс
  const body = rectPoly(f.tx, f.ty, f.tth, BODY_W, BODY_L);
  const touching = f.ok === false;
  drawPoly(body, touching ? 'rgba(192,57,43,0.55)' : 'rgba(230,126,34,0.75)', touching ? '#c0392b' : '#a04000');
  // контакты хитбокса (углы)
  body.forEach(p => { ctx.beginPath(); ctx.arc(X(p[0]), Y(p[1]), 3, 0, 2 * Math.PI); ctx.fillStyle = '#222'; ctx.fill(); });
  // направление носа
  ctx.beginPath(); ctx.moveTo(X(f.tx), Y(f.ty));
  ctx.lineTo(X(f.tx + 0.5 * Math.cos(f.tth)), Y(f.ty + 0.5 * Math.sin(f.tth)));
  ctx.strokeStyle = '#000'; ctx.lineWidth = 3; ctx.stroke();
  // все касания за миссию
  for (const [x, y, th, tc] of DATA.collision_events) {
    if (tc > t) continue;
    ctx.strokeStyle = '#e74c3c'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(X(x) - 6, Y(y) - 6); ctx.lineTo(X(x) + 6, Y(y) + 6);
    ctx.moveTo(X(x) + 6, Y(y) - 6); ctx.lineTo(X(x) - 6, Y(y) + 6); ctx.stroke();
  }
  // панель
  const touchesNow = DATA.collision_events.filter(e => e[3] <= t).length;
  document.getElementById('t').textContent = f.t.toFixed(1) + ' с';
  document.getElementById('touch').innerHTML = `<span class="${touchesNow ? 'bad' : 'ok'}">${touchesNow}</span>`;
  document.getElementById('wpc').textContent = `${f.wp} из ${DATA.waypoints.length}`;
  document.getElementById('err').textContent = (Math.hypot(f.ex - f.tx, f.ey - f.ty) * 100).toFixed(0) + ' см';
  document.getElementById('odo').textContent = DATA.travelled_m.toFixed(1) + ' м (итог)';
  document.getElementById('rep').textContent = Object.keys(DATA.paths).length + ' (итог)';
  document.getElementById('status').innerHTML = touching
    ? '<span class="bad">КАСАНИЕ</span>' : '<span class="ok">едет</span>';
  document.getElementById('seek').value = Math.round(1000 * t / Math.max(tEnd, 1e-6));
}

let playing = true, tNow = 0, last = null;
const speedEl = document.getElementById('speed');
function loop(ts) {
  if (last === null) last = ts;
  const dtReal = (ts - last) / 1000; last = ts;
  if (playing) { tNow += dtReal * parseFloat(speedEl.value); if (tNow >= tEnd) { tNow = tEnd; playing = false; document.getElementById('play').textContent = 'Старт'; } }
  draw(tNow);
  requestAnimationFrame(loop);
}
document.getElementById('play').onclick = e => { playing = !playing; e.target.textContent = playing ? 'Пауза' : 'Старт'; };
document.getElementById('restart').onclick = () => { tNow = 0; playing = true; document.getElementById('play').textContent = 'Пауза'; };
document.getElementById('seek').oninput = e => { tNow = tEnd * e.target.value / 1000; draw(tNow); };
['showmap', 'showlidar', 'showest'].forEach(id => document.getElementById(id).onchange = () => draw(tNow));
document.getElementById('summary').innerHTML =
  `<b>Итог миссии ${DATA.seed}:</b> ${DATA.completed ? '<span class="ok">все точки пройдены</span>' : '<span class="bad">не все точки пройдены (' + DATA.reached + ' из ' + DATA.waypoints.length + ')</span>'}<br>` +
  `Касаний: <b>${DATA.touches}</b>. Время в симуляции: ${DATA.time_s} с. Ошибка оценки позы (среднее/макс): ${DATA.loc_err_mean} / ${DATA.loc_err_max} м.`;
requestAnimationFrame(loop);
</script></body></html>
"""


def main() -> None:
    src, out = sys.argv[1], sys.argv[2]
    with open(src, encoding="utf-8") as f:
        data = json.load(f)
    html = TEMPLATE.replace("__DATA__", json.dumps(data, ensure_ascii=False))
    with open(out, "w", encoding="utf-8") as f:
        f.write(html)
    print("written", out, len(html) // 1024, "KB")


if __name__ == "__main__":
    main()
