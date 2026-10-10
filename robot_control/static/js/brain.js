// Клиент симуляции ROS + ИИ: опрашивает /api/brain/state и рисует карту,
// нейроны, входы/выходы сети и граф топиков. Без внешних библиотек (CSP: script-src 'self').
(function () {
  "use strict";

  var canvas = document.getElementById("map");
  var ctx = canvas.getContext("2d");
  var POLL_MS = 250;
  var last = null;

  function $(id) { return document.getElementById(id); }
  function fmt(v, d) { return (typeof v === "number") ? v.toFixed(d) : "—"; }

  // ---- карта: мир (м) → холст; ось Y мира направлена вверх ----
  function toPx(x, y, arena, scale) {
    return [x * scale, canvas.height - y * scale];
  }

  function drawMap(s) {
    var W = s.arena[0], H = s.arena[1];
    var scale = canvas.width / W;
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // сетка 1 м
    ctx.strokeStyle = "#161b22"; ctx.lineWidth = 1;
    for (var gx = 0; gx <= W; gx++) { ctx.beginPath(); ctx.moveTo(gx * scale, 0); ctx.lineTo(gx * scale, canvas.height); ctx.stroke(); }
    for (var gy = 0; gy <= H; gy++) { ctx.beginPath(); ctx.moveTo(0, canvas.height - gy * scale); ctx.lineTo(canvas.width, canvas.height - gy * scale); ctx.stroke(); }

    // стены
    ctx.strokeStyle = "#8b96a5"; ctx.lineWidth = 3;
    ctx.strokeRect(1.5, 1.5, canvas.width - 3, canvas.height - 3);

    // стеллажи
    ctx.fillStyle = "#2a313c";
    s.shelves.forEach(function (r) {
      var p0 = toPx(r[0], r[3], H, scale), p1 = toPx(r[2], r[1], H, scale);
      ctx.fillRect(p0[0], p0[1], p1[0] - p0[0], p1[1] - p0[1]);
      ctx.strokeStyle = "#3b4454"; ctx.lineWidth = 1;
      ctx.strokeRect(p0[0], p0[1], p1[0] - p0[0], p1[1] - p0[1]);
    });

    // точки маршрута
    s.waypoints.forEach(function (w, i) {
      var p = toPx(w[0], w[1], H, scale);
      var cur = (i === s.goal.index);
      ctx.strokeStyle = cur ? "#e3b341" : "#5b6472"; ctx.lineWidth = 2;
      var r = cur ? 14 : 9;
      ctx.beginPath(); ctx.moveTo(p[0] - r, p[1] - r); ctx.lineTo(p[0] + r, p[1] + r);
      ctx.moveTo(p[0] + r, p[1] - r); ctx.lineTo(p[0] - r, p[1] + r); ctx.stroke();
      ctx.fillStyle = cur ? "#e3b341" : "#5b6472"; ctx.font = "12px system-ui";
      ctx.fillText(String(i + 1), p[0] + r + 2, p[1] - r);
    });

    // траектория
    if (s.trail.length > 1) {
      ctx.strokeStyle = "#58a6ff"; ctx.lineWidth = 2; ctx.beginPath();
      s.trail.forEach(function (pt, i) {
        var p = toPx(pt[0], pt[1], H, scale);
        if (i === 0) ctx.moveTo(p[0], p[1]); else ctx.lineTo(p[0], p[1]);
      });
      ctx.stroke();
    }

    // план планировщика (A*, после сглаживания) и опережающая точка
    if (s.path && s.path.length > 1) {
      ctx.strokeStyle = "#3fb950"; ctx.lineWidth = 3; ctx.setLineDash([10, 6]);
      ctx.beginPath();
      s.path.forEach(function (pt, i) {
        var p = toPx(pt[0], pt[1], H, scale);
        if (i === 0) ctx.moveTo(p[0], p[1]); else ctx.lineTo(p[0], p[1]);
      });
      ctx.stroke(); ctx.setLineDash([]);
    }

    // лидар: 16 лучей, i=0 — по курсу, далее против часовой
    var r = s.robot;
    var th = r.thetaDeg * Math.PI / 180;
    var rp = toPx(r.x, r.y, H, scale);
    s.lidar.forEach(function (d, i) {
      var a = th + 2 * Math.PI * i / s.lidar.length;
      var ex = r.x + d * Math.cos(a), ey = r.y + d * Math.sin(a);
      var p = toPx(ex, ey, H, scale);
      ctx.strokeStyle = d < 1.0 ? "rgba(248,81,73,0.8)" : "rgba(139,150,165,0.35)";
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(rp[0], rp[1]); ctx.lineTo(p[0], p[1]); ctx.stroke();
    });

    // робот: корпус-круг и маркер курса
    ctx.fillStyle = "rgba(63,185,80,0.85)"; ctx.strokeStyle = "#3fb950"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(rp[0], rp[1], 0.35 * scale, 0, 2 * Math.PI); ctx.fill();
    ctx.beginPath(); ctx.moveTo(rp[0], rp[1]);
    ctx.lineTo(rp[0] + Math.cos(th) * 0.5 * scale, rp[1] - Math.sin(th) * 0.5 * scale); ctx.stroke();

    if (s.localGoal) {
      var lp = toPx(s.localGoal.x, s.localGoal.y, H, scale);
      ctx.fillStyle = "#d2a8ff";
      ctx.beginPath(); ctx.arc(lp[0], lp[1], 6, 0, 2 * Math.PI); ctx.fill();
    }

    // цель (как круг радиуса 0.6 м)
    var gp = toPx(s.goal.x, s.goal.y, H, scale);
    ctx.strokeStyle = "rgba(227,179,65,0.6)"; ctx.setLineDash([6, 4]);
    ctx.beginPath(); ctx.arc(gp[0], gp[1], 0.6 * scale, 0, 2 * Math.PI); ctx.stroke();
    ctx.setLineDash([]);
  }

  // ---- нейроны: цвет по активности ----
  function colorFor(v) {
    var t = Math.max(-1, Math.min(1, v));
    if (t >= 0) { return "rgba(63,185,80," + (0.15 + 0.85 * t).toFixed(2) + ")"; }
    return "rgba(248,81,73," + (0.15 + 0.85 * -t).toFixed(2) + ")";
  }

  var built = 0;
  function buildNeurons(n) {
    if (built === n) return;
    built = n;
    var box = $("neurons"); box.innerHTML = "";
    for (var i = 0; i < n; i++) {
      var d = document.createElement("div");
      d.className = "neuron"; d.id = "nrn-" + i; d.title = "нейрон " + i;
      box.appendChild(d);
    }
    $("nn-title").textContent = "ИИ: нейроны скрытого слоя (" + n + ")";
  }

  function drawNeurons(s) {
    buildNeurons(s.neurons);
    var h = s.activity.hidden;
    for (var i = 0; i < h.length; i++) {
      var el = document.getElementById("nrn-" + i);
      if (!el) continue;
      el.style.background = colorFor(h[i]);
      el.textContent = h[i].toFixed(1);
    }
  }

  // ---- входы/выходы сети ----
  var INPUT_NAMES = [];
  for (var b = 0; b < 16; b++) INPUT_NAMES.push("луч " + b);
  INPUT_NAMES.push("sin курса", "cos курса", "дист. до цели", "скорость");

  function bar(name, value, lo, hi) {
    var pct = Math.max(0, Math.min(100, (value - lo) / (hi - lo) * 100));
    return '<div>' + name + '</div><div class="bar"><i style="left:' + Math.min(50, pct).toFixed(1) +
      '%;width:' + Math.abs(pct - 50).toFixed(1) + '%;' + (pct < 50 ? 'background:#f85149' : '') + '"></i></div><div class="num">' +
      value.toFixed(2) + '</div>';
  }

  function drawIO(s) {
    var x = s.activity.inputs || [];
    var rows = [];
    // показываем 4 входа, а не все 20 — иначе блок слишком длинный; лучи — агрегатами
    var left = 0, right = 0, front = 0, nL = 0, nR = 0, nF = 0;
    for (var i = 1; i <= 7; i++) { if (x[i] !== undefined) { left += x[i]; nL++; } }
    for (var j = 9; j <= 15; j++) { if (x[j] !== undefined) { right += x[j]; nR++; } }
    front = ((x[0] || 0) + (x[15] || 0) + (x[1] || 0)) / 3;
    rows.push(bar("слева", nL ? left / nL : 0, 0, 1));
    rows.push(bar("справа", nR ? right / nR : 0, 0, 1));
    rows.push(bar("впереди", front, 0, 1));
    rows.push(bar("sin цели", x[16] || 0, -1, 1));
    rows.push(bar("выход: скорость", s.activity.outputs[0] || 0, -1, 1));
    rows.push(bar("выход: руль", s.activity.outputs[1] || 0, -1, 1));
    $("io").innerHTML = rows.join("");
  }

  function drawTopics(s) {
    var rows = s.topics.map(function (t) {
      var cls = t.hz > 0 ? "ok" : "bad";
      return "<tr><td>" + t.name + "</td><td>" + t.type + '</td><td class="num ' + cls + '">' +
        fmt(t.hz, 1) + '</td><td class="num">' + t.count + "</td></tr>";
    });
    $("topics").innerHTML = rows.join("");
    $("nodes").textContent = "узлы: " + s.nodes.join(", ") + " · физика 50 Гц · ИИ 20 Гц";
  }

  function drawMetrics(s) {
    $("m-goals").textContent = s.mission.reached;
    $("m-cycles").textContent = s.mission.cycles;
    $("m-coll").textContent = s.robot.collisions;
    $("m-time").textContent = fmt(s.simTime, 1);
    $("m-speed").textContent = fmt(s.robot.v, 2);
    $("m-steer").textContent = fmt(s.robot.steerDeg, 1);
    $("m-odo").textContent = fmt(s.robot.odometer, 1);
    $("m-replans").textContent = s.planner.replans;
    $("m-planms").textContent = fmt(s.planner.planMsMean, 1);
    $("m-goal").textContent = "№" + (s.goal.index + 1) + " (" + fmt(s.goal.x, 1) + ", " + fmt(s.goal.y, 1) + ")";
    var v = s.validation;
    $("val").textContent = v
      ? "проверка обучения: полный маршрут без касаний на незнакомых раскладках — " +
        v.missionsFullyDone + " из " + v.layouts + "; среднее точек — " + v.avgGoalsReached +
        " из 3; касаний — " + v.avgCollisions + "."
      : "веса без проверки обучения";
  }

  function render(s) {
    drawMap(s);
    drawNeurons(s);
    drawIO(s);
    drawTopics(s);
    drawMetrics(s);
    $("sub").textContent = (s.running ? "работает" : "остановлена") + " · " +
      s.neurons + " нейронов · " + s.inputs + " входов · sim " + fmt(s.simTime, 1) + " с";
    $("sub").className = "sub " + (s.running ? "ok" : "bad");
  }

  function poll() {
    fetch("api/brain/state", { cache: "no-store", credentials: "same-origin" })
      .then(function (r) {
        if (r.status === 401 || r.status === 403) { location.href = "login"; return null; }
        if (!r.ok) throw new Error("HTTP " + r.status);
        return r.json();
      })
      .then(function (j) {
        if (!j) return;
        last = j.data;
        render(last);
      })
      .catch(function (e) {
        $("sub").textContent = "нет связи: " + e.message;
        $("sub").className = "sub bad";
      })
      .then(function () { setTimeout(poll, POLL_MS); });
  }

  $("reset").addEventListener("click", function () {
    fetch("api/brain/reset", { method: "POST", credentials: "same-origin" });
  });

  poll();
})();
