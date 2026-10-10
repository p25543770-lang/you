/* ============================================================================
 * main.js — основной экран робота (киоск). Показывает состояние, управления нет.
 *
 * Данные: GET api/state раз в 300 мс (slam_gui/backend.py).
 *   • сервер не отвечает или последний ответ старше 2 с → значения не выдумываются:
 *     показываются последние полученные, помечаются «устарели», выводится тревога;
 *   • демо-данные — только при адресе с параметром ?demo=1 (показ без робота).
 *
 * Что показывает экран:
 *   меню слева: разделы (пункты ведут к разделам экрана, «Сервис» — к пульту),
 *   журнал событий за сеанс, связь с бортом и источник данных;
 *   шапка: маршрут, режим, часы, выход;
 *   полоса тревог: при проблемах — тревоги, без них — строка «тревог нет»;
 *   ключевые показатели АКБ и тяги;
 *   шасси: четыре модуля по углам (угол руля, обороты, температура, состояние).
 *
 * Журнал пишет только переходы: связь, связь с бортом, уровень АКБ, режим,
 * источник и состояние модулей (в норме ↔ проблема). Ход в норме не логируется.
 * ========================================================================== */
(function () {
  'use strict';

  /* ------------------------------------------------------------ настройки */
  const POLL_MS = 300;             // период опроса /api/state
  const DEMO_MS = 200;             // шаг отрисовки демо-данных
  const FETCH_TIMEOUT_MS = 1500;   // дольше — запрос неудачный
  const STALE_MS = 2000;           // ответ старше — данные устарели
  const EVENTS_KEEP = 40;          // событий в памяти
  const EVENTS_SHOW = 12;          // событий на экране
  const MOVE_RPM = 4;              // выше — модуль вращается
  const TEMP_WARN_C = 45;          // полоска температуры: жёлтая
  const TEMP_ALARM_C = 55;         // модуль в состоянии «перегрев»
  const TEMP_SCALE_C = 70;         // шкала полоски температуры
  const DEFAULT_LOW_V = 35.5;      // порог предупреждения АКБ, если сервер не прислал свой
  const DEFAULT_CRIT_V = 33.5;     // порог аварии АКБ, если сервер не прислал свой
  const MODULES = ['FL', 'FR', 'RL', 'RR'];
  const LEVEL_SEV = { 'НОРМА': 'ok', 'НИЗКИЙ': 'warn', 'КРИТИЧЕСКИЙ': 'err' };
  const LEVEL_WORD = { 'НОРМА': 'норма', 'НИЗКИЙ': 'низкий', 'КРИТИЧЕСКИЙ': 'критический' };
  const SOURCE_NAME = { sim: 'симуляция', serial: 'UART ×4 · 20 Гц', ros: 'ROS 2', demo: 'демо-данные' };

  const params = new URLSearchParams(window.location.search);
  const DEMO = params.get('demo') === '1';
  const SESSION_TOKEN = params.get('st');   // сессия без cookie (как в slam_auth.js)

  /* ------------------------------------------------------------ помощники */
  const $ = (id) => document.getElementById(id);
  const formatters = {};

  function fmt(value, digits) {
    if (typeof value !== 'number' || !isFinite(value)) return '—';
    if (!formatters[digits]) {
      formatters[digits] = new Intl.NumberFormat('ru-RU', {
        minimumFractionDigits: digits,
        maximumFractionDigits: digits,
      });
    }
    return formatters[digits].format(value);
  }

  function setText(node, text) {
    if (node && node.textContent !== text) node.textContent = text;
  }

  // Меняет только тон (ok / warn / err …); постоянные классы сохраняются.
  function setTone(node, base, tone) {
    if (!node) return;
    const cls = tone ? (base ? base + ' ' + tone : tone) : base;
    if (node.className !== cls) node.className = cls;
  }

  function make(tag, cls, text) {
    const node = document.createElement(tag);
    if (cls) node.className = cls;
    if (text !== undefined) node.textContent = text;
    return node;
  }

  const clockText = (date) => date.toLocaleTimeString('ru-RU', { hour12: false });
  const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
  const round1 = (x) => Math.round(x * 10) / 10;
  const round2 = (x) => Math.round(x * 100) / 100;

  /* --------------------------------------------------- состояние модуля */

  function hasTemp(m) {
    // SerialSource не передаёт температуру и отдаёт 0: это «нет данных», а не 0 °C.
    return typeof m.temp === 'number' && m.temp > 0;
  }

  // Текст о проблеме модуля; температуру добавляем только при перегреве.
  function problemText(m, s) {
    return m.id + ': ' + s.label + (s.key === 'hot' ? ', ' + fmt(m.temp, 0) + ' °C' : '');
  }

  function moduleState(m) {
    if (m.online === false) return { key: 'offline', label: 'нет связи', sev: 'err' };
    if (m.fault) return { key: 'fault', label: 'авария', sev: 'err' };
    if (m.homed === false) return { key: 'homing', label: 'хоминг', sev: 'warn' };
    if (Number(m.temp) > TEMP_ALARM_C) return { key: 'hot', label: 'перегрев', sev: 'err' };
    if (Math.abs(Number(m.rpm) || 0) > MOVE_RPM) return { key: 'move', label: 'движение', sev: 'move' };
    return { key: 'ready', label: 'готов', sev: 'ok' };
  }

  const isProblem = (s) => s.sev === 'warn' || s.sev === 'err';

  /* ------------------------------------------------------ демо (?demo=1) */

  function demoData(nowMs) {
    const t = nowMs / 1000;
    const speed = 0.55 + 0.45 * Math.sin(t / 9);                       // условная скорость
    const motors = MODULES.map((id, i) => {
      const rpm = speed * 260 + Math.sin(t * 2 + i) * 3;
      return {
        id,
        angle: Math.sin(t / (2.6 + i * 0.4)) * 34,
        rpm,
        temp: 34 + Math.abs(rpm) / 40 + Math.sin(t / 5 + i) * 1.4,
        homed: true,
        online: true,
        fault: false,
      };
    });
    const soc = 55 + 25 * Math.sin(t / 120);                           // медленный цикл заряда
    const amps = 6 + 12 * Math.abs(Math.sin(t / 9));
    const volts = 30 + (soc / 100) * 13.8 - amps * 0.075;
    const capacityWh = 714;
    const battery = {
      soc: round1(soc),
      volts: round2(volts),
      amps: round2(amps),
      watts: Math.round(volts * amps),
      remainingWh: Math.round(capacityWh * soc / 100),
      rangeKm: round1(capacityWh * soc / 100 / 32),
      state: amps > 0.2 ? 'разряд' : 'покой',
      level: volts >= DEFAULT_LOW_V ? 'НОРМА' : volts >= DEFAULT_CRIT_V ? 'НИЗКИЙ' : 'КРИТИЧЕСКИЙ',
      tempC: 28,
      thresholds: { lowV: DEFAULT_LOW_V, criticalV: DEFAULT_CRIT_V },
    };
    return {
      source: 'demo',
      mode: 'АВТОНОМНЫЙ РЕЖИМ',
      route: 'склад → зона выгрузки',
      motors,
      battery,
      powerKw: round2(battery.watts / 1000),
      speedMps: round2(speed),
      linkOk: true,
    };
  }

  /* ----------------------------------------------------- источник данных */

  const link = { data: null, at: 0, error: '' };   // последний удачный ответ

  async function poll() {
    const ctl = new AbortController();
    const timer = setTimeout(() => ctl.abort(), FETCH_TIMEOUT_MS);
    try {
      const res = await fetch('api/state', { cache: 'no-store', signal: ctl.signal });
      if (!res.ok) throw new Error('HTTP ' + res.status);
      const body = await res.json();
      if (!body || !body.ok || !body.data) throw new Error('пустой ответ сервера');
      link.data = body.data;
      link.at = Date.now();
      link.error = '';
    } catch (err) {
      link.error = err && err.name === 'AbortError' ? 'нет ответа от сервера' : String((err && err.message) || err);
    } finally {
      clearTimeout(timer);
    }
  }

  function currentView(nowMs) {
    if (DEMO) {
      return { demo: true, hasData: true, live: true, source: 'demo', data: demoData(nowMs) };
    }
    const hasData = link.data !== null;
    const live = hasData && nowMs - link.at < STALE_MS;
    return {
      demo: false,
      hasData,
      live,
      source: live ? String(link.data.source || '—') : null,
      data: link.data,
    };
  }

  /* ------------------------------------------------------------ тревоги */

  function buildAlerts(view) {
    if (!view.hasData) return [{ sev: 'warn', text: 'ожидание данных от робота' }];
    if (!view.live) {
      const sec = Math.round((Date.now() - link.at) / 1000);
      return [{ sev: 'err', text: 'нет связи с сервером · показаны последние данные, устарели на ' + sec + ' с' }];
    }
    const d = view.data;
    const b = d.battery || {};
    const th = b.thresholds || {};
    const low = typeof th.lowV === 'number' ? th.lowV : DEFAULT_LOW_V;
    const crit = typeof th.criticalV === 'number' ? th.criticalV : DEFAULT_CRIT_V;
    const out = [];

    if (d.linkOk === false) out.push({ sev: 'err', text: 'нет связи с бортом' });
    if (typeof b.volts === 'number') {
      if (b.volts < crit) out.push({ sev: 'err', text: 'АКБ ниже аварийного порога ' + fmt(crit, 1) + ' В' });
      else if (b.volts < low) out.push({ sev: 'warn', text: 'АКБ ниже порога предупреждения ' + fmt(low, 1) + ' В' });
    }
    (Array.isArray(d.motors) ? d.motors : []).forEach((m) => {
      const s = moduleState(m);
      if (isProblem(s)) out.push({ sev: s.sev, text: problemText(m, s) });
    });
    // сначала аварии, потом предупреждения
    return out.sort((x, y) => (x.sev === 'err' ? 0 : 1) - (y.sev === 'err' ? 0 : 1));
  }

  /* ------------------------------------------------------------ журнал */

  const journal = { items: [], prev: null, sig: null };   // sig: null → первая отрисовка всегда

  function note(sev, text) {
    journal.items.unshift({ sev, text, time: clockText(new Date()) });
    if (journal.items.length > EVENTS_KEEP) journal.items.length = EVENTS_KEEP;
  }

  function snapshotOf(view) {
    const d = view.data || {};
    const b = d.battery || {};
    const motors = {};
    (Array.isArray(d.motors) ? d.motors : []).forEach((m) => {
      motors[m.id] = { s: moduleState(m), m };
    });
    return {
      live: view.live,
      linkOk: d.linkOk !== false,
      level: b.level || null,
      volts: b.volts,
      mode: d.mode || null,
      source: view.source,
      motors,
    };
  }

  function trackEvents(view) {
    if (!view.hasData) return;                     // ещё ни одного ответа — писать не о чем
    const now = snapshotOf(view);
    const prev = journal.prev;
    journal.prev = now;

    if (!prev) {
      note('info', 'экран запущен · ' + (view.live ? 'данные: ' + (view.source || '—') : 'сервер недоступен'));
      return;
    }
    if (prev.live !== now.live) {
      if (now.live) note('ok', 'связь с сервером восстановлена');
      else note('err', 'нет связи с сервером · показаны последние данные');
    }
    if (now.live && prev.linkOk !== now.linkOk) {
      note(now.linkOk ? 'ok' : 'err', now.linkOk ? 'связь с бортом восстановлена' : 'связь с бортом пропала');
    }
    if (now.live && now.level && prev.level && now.level !== prev.level) {
      note(LEVEL_SEV[now.level] || 'info',
        'АКБ: ' + (LEVEL_WORD[now.level] || now.level.toLowerCase()) + ' · ' + fmt(now.volts, 1) + ' В');
    }
    if (now.live && now.mode && prev.mode && now.mode !== prev.mode) {
      note('info', 'режим: ' + now.mode.toLowerCase());
    }
    if (now.live && now.source && prev.source && now.source !== prev.source) {
      note('info', 'источник данных: ' + now.source);
    }
    MODULES.forEach((id) => {
      const p = prev.motors[id];
      const c = now.motors[id];
      if (!p || !c || p.s.key === c.s.key) return;
      if (isProblem(c.s)) note(c.s.sev, problemText(c.m, c.s));
      else if (isProblem(p.s)) note('ok', id + ': в норме');
    });
  }

  function renderEvents() {
    const list = $('ev-list');
    if (!list) return;
    const shown = journal.items.slice(0, EVENTS_SHOW);
    const sig = shown.map((e) => e.time + '|' + e.sev + '|' + e.text).join('\n');
    if (sig === journal.sig) return;               // список не менялся — DOM не трогаем
    journal.sig = sig;
    list.textContent = '';
    if (!shown.length) {
      list.appendChild(make('li', 'ev-empty', 'событий за сеанс пока нет'));
      return;
    }
    shown.forEach((e) => {
      const li = make('li', 'ev ev-' + e.sev);
      li.appendChild(make('i', 'ev-dot'));
      li.appendChild(make('time', '', e.time));
      li.appendChild(make('span', 'ev-text', e.text));
      list.appendChild(li);
    });
  }

  /* ------------------------------------------------------------ отрисовка */

  function renderHeader(view, d) {
    const mode = $('mode');
    if (mode) {
      setText(mode, view.hasData ? (d.mode || '—') : '—');
      setTone(mode, 'mode', d.mode && !/АВТОНОМ/i.test(d.mode) ? 'is-other' : '');
    }

    const chip = $('data-chip');
    let text = 'ожидание сервера';
    let tone = '';
    if (view.demo) { text = 'демо-данные'; tone = 'warn'; }
    else if (view.live) { text = 'сервер · ' + view.source; tone = 'ok'; }
    else if (view.hasData) { text = 'сервер недоступен'; tone = 'err'; }
    setText(chip, text);
    setTone(chip, 'chip', tone);
    if (chip && chip.title !== link.error) chip.title = link.error;

    // связь с бортом — карточка над оператором в меню слева: «Связь с бортом» и «есть · симуляция»
    let state = '—';
    let linkTone = '';
    let meta = 'нет данных';
    if (view.live) {
      const ok = d.linkOk !== false;
      state = ok ? 'есть' : 'нет';
      linkTone = ok ? 'ok' : 'err';
      meta = SOURCE_NAME[view.source] || 'источник: ' + (view.source || '—');
    } else if (view.hasData) {
      meta = 'сервер недоступен';
    }
    setText($('link-state'), state);
    setText($('link-meta'), meta);
    setTone($('link-card'), 'side-status', linkTone);
  }

  function renderAlerts(view) {
    const box = $('alerts');
    if (!box) return;
    const list = buildAlerts(view);
    // счётчик на пункте «Безопасность» в меню: скрыт, когда тревог нет
    const badge = $('nav-alerts');
    setText(badge, String(list.length));
    if (badge) badge.hidden = list.length === 0;

    if (!list.length) {                       // всё в норме: полоса остаётся спокойной
      setTone(box, 'alerts', 'ok');
      setText($('alerts-text'), 'тревог нет · связь с бортом, АКБ и модули в норме');
      setText($('alerts-more'), '');
      setTone(badge, 'nav-badge', '');
      return;
    }
    const top = list.some((a) => a.sev === 'err') ? 'err' : 'warn';
    setTone(box, 'alerts', top === 'warn' ? 'warn' : '');
    setText($('alerts-text'), list.slice(0, 3).map((a) => a.text).join(' · '));
    setText($('alerts-more'), list.length > 3 ? '+' + (list.length - 3) : '');
    setTone(badge, 'nav-badge', top === 'warn' ? 'warn' : '');
  }

  function renderKpis(d, b) {
    const th = b.thresholds || {};
    const low = typeof th.lowV === 'number' ? th.lowV : DEFAULT_LOW_V;
    const crit = typeof th.criticalV === 'number' ? th.criticalV : DEFAULT_CRIT_V;

    // заряд
    const soc = typeof b.soc === 'number' ? clamp(b.soc, 0, 100) : null;
    setText($('k-soc'), soc === null ? '—' : fmt(soc, 0));
    const bar = $('k-soc-bar');
    if (bar) {
      bar.style.width = (soc === null ? 0 : soc).toFixed(1) + '%';
      setTone(bar, '', soc === null ? '' : soc < 20 ? 'err' : soc < 40 ? 'warn' : '');
    }
    setText($('k-soc-sub'), b.state || b.level ? (b.state || '—') + ' · ' + (LEVEL_WORD[b.level] || '—') : '—');
    setTone($('k-soc-sub'), 'kpi-sub', LEVEL_SEV[b.level] || '');

    // напряжение
    const volts = typeof b.volts === 'number' ? b.volts : null;
    setText($('k-volts'), fmt(volts, 1));
    setTone($('k-volts'), 'kpi-num', volts === null ? '' : volts < crit ? 'err' : volts < low ? 'warn' : '');
    setText($('k-volts-sub'), 'порог ' + fmt(low, 1) + ' В');

    // ток
    setText($('k-amps'), fmt(b.amps, 1));
    setText($('k-amps-sub'), 'темп. АКБ ' + (typeof b.tempC === 'number' ? fmt(b.tempC, 0) + ' °C' : '—'));

    // запас хода
    setText($('k-range'), fmt(b.rangeKm, 1));
    setText($('k-range-sub'), typeof b.remainingWh === 'number' ? 'остаток ' + fmt(b.remainingWh, 0) + ' Вт·ч' : '—');

    // тяга
    setText($('k-power'), fmt(d.powerKw, 2));
    setText($('k-power-sub'), 'скорость ' + (typeof d.speedMps === 'number' ? fmt(d.speedMps, 2) + ' м/с' : '—'));
  }

  function renderChassis(view, d, motors) {
    let moving = 0;
    let online = 0;
    MODULES.forEach((id) => {
      const m = motors.find((x) => x.id === id);
      if (!m) return;
      if (m.online !== false) online += 1;
      if (Math.abs(Number(m.rpm) || 0) > MOVE_RPM) moving += 1;
    });
    setText($('chassis-meta'), view.hasData
      ? 'в движении ' + moving + ' из ' + MODULES.length + ' · на связи ' + online + ' из ' + MODULES.length
      : '—');
    setText($('speed'), typeof d.speedMps === 'number' ? fmt(d.speedMps, 2) : '—');

    MODULES.forEach((id) => {
      const m = motors.find((x) => x.id === id);
      const card = $('mod-' + id);
      const chip = $('mod-state-' + id);
      const wheel = $('wheel-' + id);
      const angleEl = $('angle-' + id);
      const rpmEl = $('rpm-' + id);
      const tempEl = $('temp-' + id);
      const bar = $('tbar-' + id);

      if (!m) {
        setText(chip, 'нет данных');
        setTone(chip, 'chip', 'idle');
        if (card) card.dataset.tone = 'idle';
        if (wheel) {
          wheel.dataset.tone = 'idle';
          wheel.style.transform = 'rotate(0deg)';
        }
        setText(angleEl, '—');
        setText(rpmEl, '—');
        setText(tempEl, '—');
        if (bar) {
          bar.style.width = '0%';
          setTone(bar, '', '');
        }
        return;
      }

      const s = moduleState(m);
      setText(chip, s.label);
      setTone(chip, 'chip', s.sev);
      if (card && card.dataset.tone !== s.sev) card.dataset.tone = s.sev;

      const hasAngle = typeof m.angle === 'number' && isFinite(m.angle);
      const angle = hasAngle ? clamp(m.angle, -90, 90) : 0;
      if (wheel) {
        if (wheel.dataset.tone !== s.sev) wheel.dataset.tone = s.sev;
        wheel.style.transform = 'rotate(' + angle.toFixed(1) + 'deg)';
      }
      setText(angleEl, hasAngle ? Math.round(angle) + '°' : '—');
      setText(rpmEl, typeof m.rpm === 'number' ? fmt(m.rpm, 0) : '—');

      const warm = hasTemp(m);
      setText(tempEl, warm ? fmt(m.temp, 0) : '—');
      if (bar) {
        bar.style.width = (warm ? clamp((m.temp / TEMP_SCALE_C) * 100, 0, 100) : 0).toFixed(1) + '%';
        setTone(bar, '', !warm ? '' : m.temp > TEMP_ALARM_C ? 'err' : m.temp >= TEMP_WARN_C ? 'warn' : '');
      }
    });
  }

  /* ------------------------------------------------------------ лидар */

  const LIDAR_RANGE_M = 10;        // радиус карты, м
  const LIDAR_RING_M = 2;          // шаг колец, м

  // Точки приходят как [{a: угол, рад (0 — вперёд, против часовой), r: дистанция, м}].
  // Без данных карта остаётся пустой: ничего не выдумываем.
  function lidarPoints(d) {
    if (!Array.isArray(d.lidar)) return [];
    return d.lidar.filter((p) => p && Number.isFinite(p.a) && Number.isFinite(p.r) && p.r >= 0);
  }

  function renderLidar(d) {
    const canvas = $('lidar-canvas');
    if (!canvas) return;
    const pts = lidarPoints(d);
    const has = pts.length > 0;
    const near = has ? Math.min(...pts.map((p) => p.r)) : null;

    setText($('lidar-meta'), has ? '360° · ' + pts.length + ' точек' : 'нет данных');
    setText($('lidar-count'), has ? String(pts.length) : '—');
    setText($('lidar-near'), near === null ? '—' : fmt(near, 2) + ' м');
    $('lidar-empty').hidden = has;
    drawLidar(canvas, pts);
  }

  function drawLidar(canvas, pts) {
    const ctx = canvas.getContext && canvas.getContext('2d');
    if (!ctx) return;
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (!w || !h) return;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const cx = w / 2;
    const cy = h / 2;
    const R = Math.max(10, Math.min(w, h) / 2 - 6);
    const k = R / LIDAR_RANGE_M;

    ctx.strokeStyle = '#2b4037';
    ctx.lineWidth = 1;
    for (let m = LIDAR_RING_M; m <= LIDAR_RANGE_M; m += LIDAR_RING_M) {
      ctx.beginPath();
      ctx.arc(cx, cy, m * k, 0, Math.PI * 2);
      ctx.stroke();
    }
    ctx.beginPath();                       // оси: вперёд и вбок
    ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R);
    ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy);
    ctx.stroke();

    ctx.fillStyle = '#9fe36b';
    pts.forEach((p) => {
      const r = Math.min(p.r, LIDAR_RANGE_M) * k;
      const x = cx - r * Math.sin(p.a);
      const y = cy - r * Math.cos(p.a);
      ctx.fillRect(x - 1.5, y - 1.5, 3, 3);
    });

    ctx.fillStyle = '#e6efe9';            // робот: треугольник носом вперёд
    ctx.beginPath();
    ctx.moveTo(cx, cy - 7);
    ctx.lineTo(cx + 5, cy + 5);
    ctx.lineTo(cx - 5, cy + 5);
    ctx.closePath();
    ctx.fill();
  }

  function renderRoute(d) {
    const parts = String(d.route || '').split('→').map((s) => s.trim()).filter(Boolean);
    setText($('route-from'), parts[0] || '—');
    setText($('route-to'), parts[1] || '—');
  }

  function render() {
    const view = currentView(Date.now());
    trackEvents(view);

    const d = view.data || {};
    const b = d.battery || {};
    const motors = Array.isArray(d.motors) ? d.motors : [];

    setText($('clock'), clockText(new Date()));
    renderHeader(view, d);
    renderAlerts(view);
    renderKpis(d, b);
    renderChassis(view, d, motors);
    renderLidar(d);
    renderRoute(d);
    renderEvents();
    document.body.classList.toggle('is-stale', !view.live);
  }

  /* ------------------------------------------------------------ запуск */

  function initLinks() {
    // ссылка на пульт сохраняет токен сессии, если он есть в адресе
    const consoleLink = $('console-link');
    if (consoleLink && SESSION_TOKEN) {
      consoleLink.href = '/console?st=' + encodeURIComponent(SESSION_TOKEN);
    }
  }

  async function loop() {
    try {
      if (!DEMO) await poll();
      render();
    } catch (err) {
      console.error('RUS SLAM main:', err);
    } finally {
      setTimeout(loop, DEMO ? DEMO_MS : POLL_MS);
    }
  }

  // Пункт меню подсвечивается по адресу: #chassis → «Ходовая», без адреса → «Пульт».
  function syncNav() {
    const hash = window.location.hash || '#';
    document.querySelectorAll('.nav a[href^="#"]').forEach((a) => {
      const on = a.getAttribute('href') === hash;
      a.classList.toggle('active', on);
      if (on) a.setAttribute('aria-current', 'true');
      else a.removeAttribute('aria-current');
    });
  }

  function boot() {
    initLinks();
    syncNav();
    window.addEventListener('hashchange', syncNav);
    loop();
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot);
  else boot();

  // для проверок в браузере и отладки
  window.RS_MAIN = { render, currentView, buildAlerts, moduleState, journal, link };
})();
