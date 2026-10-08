/* ============================================================================
 * map.js — карта цеха на экране робота и на инженерном пульте.
 *
 * Карту строит борт: дальномер размечает клетки, борт отдаёт их строкой
 * («значение×количество») вместе с версией. Экран забирает клетки только когда
 * версия сменилась, а рисует каждый такт: робот, след, площадки, задание.
 *
 * Почему отдельный модуль: раньше карту рисовали сразу два места — демо-мир
 * vision.js на пульте и своя сетка в панели. Теперь источник один: то, что
 * действительно просканировал робот.
 *
 * Запуск на странице:
 *     RSMap.tick(state)             — киоск: состояние уже опрошено main.js;
 *     RSMap.start('map')            — пульт: модуль опрашивает борт сам;
 *     RSMap.active()                — true, когда борт отдал настоящую карту.
 * ========================================================================== */
'use strict';

(function () {
  const UNKNOWN = 0, FREE = 1, OCCUPIED = 2;
  const COL = {
    bg: '#0d1418', grid: '#152027', free: '#1d3b45', occ: '#d9a441',
    trail: '#3fc6d8', pad: '#46bd7c', goal: '#e0574b', robot: '#dde7ef',
    wheel: '#c4453a', text: '#93a6b6', text2: '#62727f',
  };

  const PULL_MS = 800;                 // чаще этого клетки не запрашиваем
  let canvas = null, ctx = null;
  let cells = null, cellsVersion = -1, cellsSize = [0, 0];
  let last = null, started = false, timer = null, busy = false, pulledAt = 0;

  function el(id) { return document.getElementById(id); }

  /** Разбирает строку клеток «значение×количество,…» — формат борта. */
  function decodeRle(text, expect) {
    const out = new Uint8Array(expect);
    let at = 0;
    String(text || '').split(',').forEach((chunk) => {
      const star = chunk.indexOf('*');
      if (star < 0) return;
      const value = parseInt(chunk.slice(0, star), 10);
      const count = parseInt(chunk.slice(star + 1), 10);
      if (!isFinite(value) || !isFinite(count)) return;
      for (let i = 0; i < count && at < expect; i++) out[at++] = value;
    });
    return out;
  }

  function attach(id) {
    const c = el(id);
    if (!c) return false;
    canvas = c;
    ctx = c.getContext && c.getContext('2d');
    return !!ctx;
  }

  /** Забирает клетки, когда борт сообщил новую версию карты. */
  async function pull(room) {
    if (!room || !room.ok || busy) return;
    if (cells && room.version === cellsVersion) return;
    // Пока цех разведуется, версия меняется каждый такт. Клетки весят больше
    // состояния, поэтому берём их не чаще PULL_MS: поза и след рисуются
    // каждым тактом, а сама карта догоняет за доли секунды.
    if (cells && Date.now() - pulledAt < PULL_MS) return;
    busy = true;
    pulledAt = Date.now();
    try {
      const res = await fetch('api/map', { cache: 'no-store' });
      const data = await res.json();
      if (data && data.ok && data.cells) {
        cellsSize = [data.w, data.h];
        cells = decodeRle(data.cells, data.w * data.h);
        cellsVersion = data.version;
      }
    } catch (e) {
      /* борт не ответил — показываем то, что уже есть */
    } finally {
      busy = false;
    }
  }

  function fit(state, w, h) {
    const world = (state && state.map && state.map.world) || { w: 4.8, h: 3.2 };
    const pad = 12;
    const padTop = 26;   // полоска под подпись цеха и размера клетки
    const scale = Math.min((w - pad * 2) / world.w, (h - padTop - pad) / world.h);
    const ox = (w - world.w * scale) / 2;
    // ось y смотрит вверх: на экране она вниз, поэтому переворачиваем
    const oy = padTop + (h - padTop - pad + world.h * scale) / 2;
    return { scale, ox, oy, padTop, world };
  }

  function toScreen(f, x, y) {
    return [f.ox + x * f.scale, f.oy - y * f.scale];
  }

  function roundRect(g, x, y, w, h, r) {
    g.beginPath();
    g.moveTo(x + r, y);
    g.arcTo(x + w, y, x + w, y + h, r);
    g.arcTo(x + w, y + h, x, y + h, r);
    g.arcTo(x, y + h, x, y, r);
    g.arcTo(x, y, x + w, y, r);
    g.closePath();
  }

/** Подпись полотна: что нарисовано и в каком масштабе (цифры — с бэкенда). */
  function drawCaption(g, f, state) {
    const room = (state && state.map) || {};
    g.fillStyle = COL.text2;
    g.font = '11px ui-monospace, monospace';
    g.fillText('ЦЕХ ' + f.world.w.toFixed(1).replace('.', ',') + ' × ' +
      f.world.h.toFixed(1).replace('.', ',') + ' М · КЛЕТКА ' +
      Number(room.res || 0.05).toFixed(2).replace('.', ',') + ' М · ИСТОЧНИК ' +
      String(room.source || '—').toUpperCase(),
      f.ox, f.padTop - 9);
  }

  function drawGrid(g, f) {
    g.strokeStyle = COL.grid;
    g.lineWidth = 1;
    for (let x = 0; x <= f.world.w + 1e-6; x += 0.5) {
      const [sx] = toScreen(f, x, 0);
      g.beginPath(); g.moveTo(sx, f.oy - f.world.h * f.scale); g.lineTo(sx, f.oy); g.stroke();
    }
    for (let y = 0; y <= f.world.h + 1e-6; y += 0.5) {
      const [, sy] = toScreen(f, 0, y);
      g.beginPath(); g.moveTo(f.ox, sy); g.lineTo(f.ox + f.world.w * f.scale, sy); g.stroke();
    }
    g.strokeStyle = '#2c3f4c';
    g.strokeRect(f.ox, f.oy - f.world.h * f.scale, f.world.w * f.scale, f.world.h * f.scale);
  }

  function drawCells(g, f, state, cell) {
    if (!cells) return;
    const w = cellsSize[0], h = cellsSize[1];
    const res = cell || (state.map && state.map.res) || 0.05;
    const cw = Math.max(1, res * f.scale), ch = cw;
    for (let y = 0; y < h; y++) {
      // в мире клетка y идёт снизу вверх, строка строки — сверху вниз
      const [sx0, sy0] = toScreen(f, 0, (y + 1) * res);
      for (let x = 0; x < w; x++) {
        const v = cells[y * w + x];
        if (v === UNKNOWN) continue;
        g.fillStyle = v === OCCUPIED ? COL.occ : COL.free;
        g.fillRect(sx0 + x * cw, sy0, cw + 0.5, ch + 0.5);
      }
    }
  }

  function drawPads(g, f, state) {
    (state.map.pads || []).forEach((p) => {
      const [x, y] = toScreen(f, p.x, p.y);
      g.fillStyle = COL.pad;
      g.globalAlpha = 0.85;
      g.beginPath(); g.arc(x, y, 5, 0, Math.PI * 2); g.fill();
      g.globalAlpha = 1;
      g.fillStyle = COL.text;
      g.font = '11px ui-monospace, monospace';
      g.fillText(p.label, x + 8, y + 4);
    });
  }

  function drawGoal(g, f, state) {
    const goal = state.map.goal || {};
    if (goal.x == null || goal.y == null) return;
    const [x, y] = toScreen(f, goal.x, goal.y);
    g.strokeStyle = COL.goal;
    g.lineWidth = 1.6;
    g.beginPath();
    g.moveTo(x - 8, y); g.lineTo(x + 8, y);
    g.moveTo(x, y - 8); g.lineTo(x, y + 8);
    g.stroke();
    g.beginPath(); g.arc(x, y, 7, 0, Math.PI * 2); g.stroke();
    g.fillStyle = COL.goal;
    g.font = '11px ui-monospace, monospace';
    g.fillText('задание', x + 10, y - 8);
  }

  function drawTrail(g, f, state) {
    const trail = state.map.trail || [];
    if (trail.length < 2) return;
    g.strokeStyle = COL.trail;
    g.globalAlpha = 0.5;
    g.lineWidth = 1.2;
    g.setLineDash([4, 3]);
    g.beginPath();
    trail.forEach((p, i) => {
      const [x, y] = toScreen(f, p[0], p[1]);
      if (i) g.lineTo(x, y); else g.moveTo(x, y);
    });
    g.stroke();
    g.setLineDash([]);
    g.globalAlpha = 1;
  }

  function drawRobot(g, f, state) {
    const pose = state.map.pose || {};
    const [x, y] = toScreen(f, pose.x || 0, pose.y || 0);
    const s = Math.max(10, 0.46 * f.scale);       // габарит машины, м → пиксели
    const yaw = pose.th || 0;
    g.save();
    g.translate(x, y);
    g.rotate(-yaw);                               // экран развёрнут: курс против часовой
    g.strokeStyle = COL.robot;
    g.fillStyle = 'rgba(63,198,216,0.14)';
    g.lineWidth = 1.6;
    roundRect(g, -s / 2, -s / 2, s, s, 3);
    g.fill();
    g.stroke();
    // нос машины и четыре колеса — как на панели двигателей
    g.beginPath();
    g.moveTo(s / 2 - 2, 0); g.lineTo(s / 2 + 7, 0);
    g.moveTo(s / 2 + 7, 0); g.lineTo(s / 2 + 2, -3);
    g.moveTo(s / 2 + 7, 0); g.lineTo(s / 2 + 2, 3);
    g.stroke();
    const angles = (state.ai && state.ai.angles) || {};
    const wheelAt = { FL: [-1, -1], FR: [-1, 1], RL: [1, -1], RR: [1, 1] };
    Object.keys(wheelAt).forEach((id) => {
      const [dx, dy] = wheelAt[id];
      const deg = Number(angles[id] || 0);
      g.save();
      g.translate(dx * (s / 2 + 2), dy * (s / 2 + 2));
      g.rotate((deg * Math.PI) / 180);
      g.fillStyle = COL.wheel;
      g.fillRect(-1.8, -3.4, 3.6, 6.8);
      g.restore();
    });
    g.restore();
  }

  /** Буфер канвы под её размер на странице: иначе карта растягивается. */
  function ensureSize() {
    const dw = Math.max(320, canvas.clientWidth | 0);
    const dh = Math.max(240, canvas.clientHeight | 0);
    if (canvas.width !== dw || canvas.height !== dh) {
      canvas.width = dw;
      canvas.height = dh;
    }
  }

  function draw(state) {
    if (!ctx) return false;
    const room = (state && state.map) || {};
    ensureSize();
    const w = canvas.width, h = canvas.height;
    const f = fit(state, w, h);
    ctx.fillStyle = COL.bg;
    ctx.fillRect(0, 0, w, h);
    drawGrid(ctx, f);
    if (room.ok) drawCaption(ctx, f, state);
    if (room.ok) {
      drawCells(ctx, f, state, room.res);
      drawTrail(ctx, f, state);
      drawPads(ctx, f, state);
      drawGoal(ctx, f, state);
    }
    drawRobot(ctx, f, state);
    return !!room.ok;
  }

  /** Состояние клетки по мировым координатам — для подсказок оператору. */
  function cellAt(x, y) {
    if (!cells) return null;
    const res = (last && last.map && last.map.res) || 0.05;
    const cx = Math.floor(x / res), cy = Math.floor(y / res);
    if (cx < 0 || cy < 0 || cx >= cellsSize[0] || cy >= cellsSize[1]) return null;
    const v = cells[cy * cellsSize[0] + cx];
    return {
      value: v,
      text: v === OCCUPIED ? 'препятствие' : v === FREE ? 'свободно' : 'не разведано',
    };
  }

  /** Мировые координаты точки канвы (по ней оператор читает обстановку). */
  function worldAt(clientX, clientY) {
    if (!canvas || !last || !canvas.getBoundingClientRect) return null;
    const rect = canvas.getBoundingClientRect();
    if (!rect.width || !rect.height) return null;
    const f = fit(last, canvas.width, canvas.height);
    const u = (clientX - rect.left) * (canvas.width / rect.width);
    const v = (clientY - rect.top) * (canvas.height / rect.height);
    const point = { x: (u - f.ox) / f.scale, y: (f.oy - v) / f.scale };
    point.cell = cellAt(point.x, point.y);
    return point;
  }

  function updateFoot(state) {
    const room = (state && state.map) || {};
    const set = (id, text) => { const e = el(id); if (e) e.textContent = text; };
    if (!room.ok) {
      set('sc-map-stats', 'карта недоступна');
      set('sc-map-pose', 'робот —');
      set('sc-map-goal', 'задание —');
      return;
    }
    const pose = room.pose || {};
    set('sc-map-stats', 'разведано ' + room.scanPct + '% · версия карты ' + room.version);
    set('sc-map-pose', 'робот ' + pose.x.toFixed(2) + ' / ' + pose.y.toFixed(2) + ' м · ' +
      Math.round(((pose.th || 0) * 180) / Math.PI) + '°');
    const goal = room.goal || {};
    set('sc-map-goal', 'задание: ' + (goal.label || '—'));
  }

  /** Киоск: состояние уже опрошено страницей — только рисуем. */
  function tick(state) {
    last = state || null;
    if (!canvas && !attach('sc-map')) return false;
    if (!last) return false;
    const room = last.map || {};
    if (room.ok) pull(room);
    updateFoot(last);
    return draw(last);
  }

  /** Пульт: модуль опрашивает борт сам (там своей карты нет). */
  function start(id) {
    if (!attach(id || 'map')) return false;
    if (started) return true;
    started = true;
    const step = async () => {
      try {
        const res = await fetch('api/state', { cache: 'no-store' });
        const body = await res.json();
        if (body && body.ok && body.data) tick(body.data);
      } catch (e) {
        /* борт молчит — рисуем последнее, что было */
      }
    };
    step();
    timer = setInterval(step, 400);
    return true;
  }

  function stop() {
    if (timer) clearInterval(timer);
    timer = null; started = false;
  }

  /** Пульт спрашивает: рисовать ли карту борта вместо демо-мира. */
  function active() {
    return !!(last && last.map && last.map.ok);
  }

  /** Киоск: карту рисует main.js по своему опросу. Пульт: своего опроса нет,
   *  модуль забирает состояние сам — иначе карта борта на пульт не попадёт. */
  function autostart() {
    if (el('sc-map')) { attach('sc-map'); return; }   // киоск: рисует main.js
    if (el('map')) start('map');                      // пульт: опрашиваем сами
  }

  if (typeof document !== 'undefined' && document.addEventListener) {
    if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', autostart);
    else autostart();
  }

  window.RSMap = {
    tick, start, stop, active, draw, decodeRle, attach, autostart, worldAt, cellAt,
    get version() { return cellsVersion; },
    get hasCells() { return !!cells; },
  };
})();
