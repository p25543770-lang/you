/* Приборная оснастка пульта — дополнение к интерфейсу RUS SLAM.
 *
 * Задача: показать инженеру не только «что», но и «насколько этому можно
 * верить» — частоту опроса, возраст последней выборки, счётчики обмена и
 * тренды. Данные берутся из того же /api/state, что и остальные экраны
 * (GET /api/state), поэтому ничего дополнительно настраивать не нужно.
 *
 * Скрипт ничего не ломает: он заполняет только свои элементы с префиксом
 * `ins-`. Если элемента на странице нет — соответствующий блок пропускается.
 * Работает и на основном экране (main.html), и на инженерном пульте.
 */
(function () {
  "use strict";

  var $ = function (id) { return document.getElementById(id); };

  /* Период опроса: приборная линейка — вспомогательная, поэтому опрашиваем
     реже основного экрана (там 300 мс у main.js) и не конкурируем с ним за
     канал. Для счётчиков, частоты и трендов 2 Гц достаточно. */
  var POLL_MS = 500;
  var HISTORY = 150;          // точек в кольцевых буферах трендов

  var hist = { soc: [], spd: [], pwr: [] };
  var stamp = [];             // времена успешных выборок, мс
  var samples = 0;
  var badReads = 0;
  var lastOk = 0;
  var lastUptime = null;
  var restarts = 0;

  var LAMP = {
    ok: "#46bd7c",
    warn: "#dfa43a",
    err: "#e0574b",
    idle: "#5d7080"
  };

  function setLamp(id, state, text) {
    var el = $(id);
    if (!el) return;
    el.dataset.state = state;
    var dot = el.querySelector("i");
    if (dot) {
      dot.style.background = LAMP[state] || LAMP.idle;
      dot.style.boxShadow = state === "idle"
        ? "none"
        : "0 0 6px " + (LAMP[state] || LAMP.idle);
    }
    if (typeof text === "string") {
      el.lastChild.textContent = text;
    }
  }

  function setText(id, text) {
    var el = $(id);
    if (el) el.textContent = text;
  }

  function num(v, digits) {
    var n = Number(v);
    if (!isFinite(n)) return "—";
    return n.toFixed(digits === undefined ? 1 : digits);
  }

  /* Частота: по медиане интервалов между выборками — одиночный «провал»
     сети не должен показывать ложное падение частоты. */
  function rateHz() {
    if (stamp.length < 3) return null;
    var gaps = [];
    for (var i = 1; i < stamp.length; i++) {
      var d = stamp[i] - stamp[i - 1];
      if (d > 0) gaps.push(d);
    }
    if (!gaps.length) return null;
    gaps.sort(function (a, b) { return a - b; });
    var med = gaps[Math.floor(gaps.length / 2)];
    return med > 0 ? 1000 / med : null;
  }

  function push(list, value) {
    list.push(value);
    if (list.length > HISTORY) list.shift();
  }

  /* Тренд-дорожка: заполненная область + линия, автоподбор масштаба.
     Рисуется на canvas 2D без библиотек — робот работает без интернета. */
  function drawTrend(canvasId, list, min, max, color) {
    var cv = $(canvasId);
    if (!cv) return;
    var ctx = cv.getContext("2d");
    var w = cv.width;
    var h = cv.height;
    ctx.clearRect(0, 0, w, h);

    // Сетка: 4 деления по горизонтали, базовая линия.
    ctx.strokeStyle = "rgba(120,145,165,0.18)";
    ctx.lineWidth = 1;
    for (var gx = 1; gx < 4; gx++) {
      var x = Math.round(w * gx / 4) + 0.5;
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
      ctx.stroke();
    }
    for (var gy = 1; gy < 3; gy++) {
      var y = Math.round(h * gy / 3) + 0.5;
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }

    if (list.length < 2) return;

    var lo = min === null || min === undefined ? Math.min.apply(null, list) : min;
    var hi = max === null || max === undefined ? Math.max.apply(null, list) : max;
    if (hi === lo) { hi = lo + 1; }

    var px = function (i) { return (w - 1) * i / (list.length - 1); };
    var py = function (v) {
      var k = (v - lo) / (hi - lo);
      k = Math.max(0, Math.min(1, k));
      return h - 2 - k * (h - 4);
    };

    // Заливка под линией.
    ctx.beginPath();
    ctx.moveTo(px(0), h);
    for (var i = 0; i < list.length; i++) ctx.lineTo(px(i), py(list[i]));
    ctx.lineTo(px(list.length - 1), h);
    ctx.closePath();
    ctx.fillStyle = color.replace("1)", "0.14)");
    ctx.fill();

    // Сама линия.
    ctx.beginPath();
    ctx.moveTo(px(0), py(list[0]));
    for (var j = 1; j < list.length; j++) ctx.lineTo(px(j), py(list[j]));
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Последняя точка — «перо» самописца.
    ctx.beginPath();
    ctx.arc(px(list.length - 1), py(list[list.length - 1]), 2, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
  }

  function renderModules(state, bus) {
    var body = $("ins-mods");
    if (!body) return;
    var motors = state.motors || [];
    var online = {};
    (bus && bus.modules ? bus.modules : []).forEach(function (m) { online[m] = true; });

    var angles = [];
    body.innerHTML = motors.map(function (m) {
      var a = Number(m.angle) || 0;
      angles.push(a);
      var isOnline = m.online === undefined ? true : !!m.online;
      var mark = bus && !bus.simulated
        ? (isOnline || online[m.id] ? "да" : "нет")
        : "да";
      return "<tr>" +
        "<td>" + (m.id || "—") + "</td>" +
        "<td>" + num(a, 2) + "</td>" +
        "<td>" + num(m.rpm, 1) + "</td>" +
        "<td>" + num(m.temp, 1) + "</td>" +
        "<td>" + (m.homed ? "да" : "нет") + " · линия " + mark + "</td>" +
        "</tr>";
    }).join("");

    // Разброс углов: рассинхрон рулевых модулей виден сразу.
    var diff = $("ins-diff");
    if (diff) {
      if (angles.length > 1) {
        var spread = Math.max.apply(null, angles) - Math.min.apply(null, angles);
        diff.textContent = "Разброс углов между модулями: " + num(spread, 2) +
          "° (максимум по 4 модулям). АКБ: ячейка " +
          num(state.battery && state.battery.cellMin, 3) + "…" +
          num(state.battery && state.battery.cellMax, 3) + " В.";
      } else {
        diff.textContent = "Модули не отдают данные.";
      }
    }
  }

  function render(state) {
    var now = Date.now();
    samples++;
    stamp.push(now);
    if (stamp.length > 12) stamp.shift();
    lastOk = now;

    var bus = state.bus || null;
    var src = state.source || "—";
    var age = state.ts ? Math.max(0, now - state.ts * 1000) : null;
    var busAge = bus && bus.ageMs !== null && bus.ageMs !== undefined ? bus.ageMs : age;
    var hz = rateHz();

    /* Кампания: uptimeSec счётчика бэкенда; его уменьшение означает
       перезапуск процесса или переход на другой источник. */
    var up = state.uptimeSec;
    if (lastUptime !== null && typeof up === "number" && up < lastUptime) {
      restarts++;
      hist.soc.length = hist.spd.length = hist.pwr.length = 0;
    }
    if (typeof up === "number") lastUptime = up;

    /* --- счётчики --- */
    setText("ins-source", src);
    setText("ins-rate", hz === null ? "—" : num(hz, 1));
    setText("ins-age", age === null ? "—" : num(age, 0));
    setText("ins-frames", String(samples));
    setText("ins-uptime", history(up));
    setText("ins-bus-rate", hz === null ? "—" : num(hz, 1));
    setText("ins-bus-age", busAge === null || busAge === undefined ? "—" : num(busAge, 0));

    /* --- качество канала --- */
    var chip = $("ins-bus-chip");
    var quality;
    if (age === null || age > 3000) {
      quality = ["err", "НЕТ ОТВЕТА"];
    } else if (bus && bus.simulated) {
      quality = ["idle", "МОДЕЛЬ"];
    } else if (bus && bus.discarded > 0) {
      quality = ["warn", "ОШИБКИ ЛИНИИ"];
    } else {
      quality = ["ok", "НОРМА"];
    }
    if (chip) {
      chip.textContent = quality[1];
      chip.className = "csl-chip csl-chip-" +
        (quality[0] === "ok" ? "closed" : quality[0] === "warn" ? "locked" : "idle");
    }

    /* --- лампы --- */
    var linkOk = state.linkOk !== false && (age === null || age < 2000);
    setLamp("ins-lamp-link", linkOk ? "ok" : "err", "связь");

    var local = consoleState();
    /* Если на экране нет ни органа E-STOP, ни поля в состоянии, честнее
       показать «неизвестно», чем зелёную лампу «всё хорошо». */
    var estopKnown = local.estop !== null || typeof state.estop === "boolean";
    var estop = local.estop !== null ? local.estop : !!state.estop;
    setLamp("ins-lamp-estop", !estopKnown ? "idle" : estop ? "err" : "ok", "E-STOP");

    var stand = local.stand;
    setLamp("ins-lamp-stand", stand ? "warn" : "ok", "стенд");

    var cargo = state.cargo || {};
    setLamp("ins-lamp-cargo", cargo.closed ? "ok" : "warn",
      cargo.closed ? "отсек" : "отсек открыт");

    var batt = state.battery || {};
    var soc = Number(batt.soc);
    var battState = !isFinite(soc) ? "idle"
      : soc >= 30 ? "ok" : soc >= 15 ? "warn" : "err";
    setLamp("ins-lamp-batt", battState, "АКБ " + (isFinite(soc) ? num(soc, 0) + "%" : "—"));

    /* --- строки безопасности --- */
    setText("ins-safe-estop", estop ? "ВКЛ" : "ВЫКЛ");
    var zone = state.clearance !== undefined ? Number(state.clearance) : local.clearance;
    setText("ins-safe-zone", zone === null || zone === undefined || !isFinite(zone)
      ? "—" : num(zone, 2) + " м");
    var ready = (state.motors || []).every(function (m) { return m.homed; }) && linkOk;
    setText("ins-safe-ready", ready ? "ГОТОВ" : "НЕ ГОТОВ");
    setText("ins-safe-cargo", cargo.closed ? "ЗАКРЫТ" : "ОТКРЫТ");

    var vmaxOut = $("ins-vmax");
    var vmaxVal = local.vmax !== null ? local.vmax : Number(state.vmax);
    if (vmaxOut && isFinite(vmaxVal) && vmaxVal !== null) {
      vmaxOut.textContent = num(vmaxVal, 2) + " м/с";
    }

    /* Счётчик ошибок линии: отброшенные байты (реальная шина) плюс сбои
       чтения состояния на этом экране. */
    var crcErr = (bus && typeof bus.discarded === "number" ? bus.discarded : 0) + badReads;
    setText("crc", String(crcErr));

    /* --- журнал отметок карты --- */
    var mapN = $("ins-map-n");
    if (mapN) mapN.textContent = samples + " обновлений";

    /* --- тренды --- */
    if (isFinite(soc)) push(hist.soc, soc);
    if (typeof state.speedMps === "number") push(hist.spd, state.speedMps);
    if (typeof state.powerKw === "number") push(hist.pwr, state.powerKw);

    drawTrend("ins-trend-soc", hist.soc, 0, 100, "rgba(63,198,216,1)");
    drawTrend("ins-trend-spd", hist.spd, 0, null, "rgba(70,189,124,1)");
    drawTrend("ins-trend-pwr", hist.pwr, 0, null, "rgba(223,164,58,1)");

    renderModules(state, bus);
  }

  /* Часть величин живёт только в пульте и на шину не выходит: E-STOP,
     сервисный «стенд», клиренс и ограничение скорости задаются оператором
     на этом же экране. Их единственный достоверный источник — сами органы
     управления, поэтому читаем их из DOM, а не выдумываем значение. */
  function domFlag(id, on, off) {
    var el = $(id);
    if (!el) return null;
    var t = (el.textContent || "").trim();
    if (!t) return null;
    return t === on ? true : t === off ? false : null;
  }

  function domNumber(id) {
    var el = $(id);
    if (!el) return null;
    var m = String(el.textContent || "").replace(",", ".").match(/-?\d+(\.\d+)?/);
    return m ? Number(m[0]) : null;
  }

  function consoleState() {
    var estop = domFlag("estop-state", "ВКЛ", "ВЫКЛ");
    var vmaxEl = $("vmax");
    var clr = domNumber("clr-val");
    return {
      estop: estop,
      stand: document.body.classList.contains("service-stand") ||
        !!($("csl-stand") && $("csl-stand").checked),
      clearance: clr,
      vmax: vmaxEl ? Number(vmaxEl.value) : null
    };
  }

  function history(sec) {
    if (typeof sec !== "number" || !isFinite(sec)) return "—";
    var h = Math.floor(sec / 3600);
    var m = Math.floor((sec % 3600) / 60);
    var s = Math.floor(sec % 60);
    return (h ? h + ":" : "") +
      String(m).padStart(2, "0") + ":" + String(s).padStart(2, "0");
  }

  function poll() {
    fetch("api/state", { cache: "no-store" })
      .then(function (r) {
        if (!r.ok) throw new Error("HTTP " + r.status);
        return r.json();
      })
      .then(function (body) {
        if (body && body.ok && body.data) render(body.data);
        else badReads++;
      })
      .catch(function () { badReads++; });
  }

  /* При потере связи счётчики должны «застыть», а не показывать старое
     значение как свежее: помечаем линейку и ждём восстановления. */
  function watchdog() {
    if (!lastOk) return;
    var gap = Date.now() - lastOk;
    if (gap > POLL_MS * 4) {
      setLamp("ins-lamp-link", "err", "связь");
      var chip = $("ins-bus-chip");
      if (chip) {
        chip.textContent = "НЕТ ОТВЕТА";
        chip.className = "csl-chip csl-chip-idle";
      }
      var ageEl = $("ins-age");
      if (ageEl) ageEl.textContent = "—";
    }
  }

  /* Плашка «Поиск команд» в шапке: до этого она была просто надписью.
     Нажимаем ту же кнопку палитры, что и Ctrl+K, — обработчик один. */
  function wireSearch() {
    var box = $("ins-search");
    var trigger = $("csl-tb-palette");
    if (!box || !trigger) return;
    var open = function (e) {
      e.preventDefault();
      trigger.click();
    };
    box.addEventListener("click", open);
    box.addEventListener("keydown", function (e) {
      if (e.key === "Enter" || e.key === " ") open(e);
    });
  }

  function start() {
    wireSearch();
    poll();
    setInterval(poll, POLL_MS);
    setInterval(watchdog, 1000);
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", start);
  } else {
    start();
  }
})();
