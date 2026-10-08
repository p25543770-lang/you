/* ============================================================================
 * main.js — основной экран робота: данные и взаимодействие.
 *
 * Источник данных:
 *   1) бэкенд `gui/backend.py` — GET /api/state каждые 300 мс (двигатели, АКБ,
 *      связь), POST /api/lock/open|close;
 *   2) если сервера нет (страница открыта как файл или обычным static-сервером)
 *      — локальная демонстрация, чтобы экран не оставался пустым.
 *
 * ПРАВИЛА ЭКРАНА (чтобы в интерфейсе не было дублей):
 *   • одно действие — один орган управления. Очистка ввода PIN — только клавиша
 *     «СБРОС» на клавиатуре; открытие/закрытие отсека — одна кнопка, её надпись
 *     меняется по состоянию. Второй кнопки сброса нет;
 *   • одно состояние показывается в одном месте: заряд — кольцо АКБ,
 *     связь с модулями — подвал панели «Двигатели», канал данных — чип в шапке.
 *
 * Строка состояния в подвале (маршрут, тяга, журнал) убрана — вместе с ней
 * ушёл и опрос /api/audit. Переход на инженерный пульт — столбик слева.
 * ========================================================================== */
(function () {
  'use strict';

  const $ = (id) => document.getElementById(id);
  const POLL_MS = 300;
  const PROMPT = 'Введите PIN-код и нажмите «Открыть»';

  /* ======================================================================
   * 1. Демонстрационный источник (резерв, когда API недоступен)
   * ==================================================================== */
  const DEMO = {
    motors: [
      { id: 'FL', title: 'передний левый', angle: 0, rpm: 0, temp: 36, homed: true },
      { id: 'FR', title: 'передний правый', angle: 0, rpm: 0, temp: 37, homed: true },
      { id: 'RL', title: 'задний левый', angle: 0, rpm: 0, temp: 35, homed: true },
      { id: 'RR', title: 'задний правый', angle: 0, rpm: 0, temp: 36, homed: true },
    ],
    battery: { soc: 78, volts: 41.2, amps: 12.4, rangeKm: 14.2, state: 'разряд', level: 'НОРМА' },
    cargo: { kg: 80, closed: true },
    mode: 'АВТОНОМНЫЙ РЕЖИМ',
    route: 'склад → зона выгрузки',
    powerKw: 2.4,
    linkOk: true,
    driveMode: 'прямо',        // какой манёвр показывает демонстрация
    localPin: '2580',          // только для демо-режима
    maxAttempts: 5,
    lockMs: 30000,
  };

  /* Демонстрационные режимы езды — та же логика, что в gui/backend.py.
     Раньше каждое колесо крутилось своей синусоидой, и по углам четырёх
     модулей нельзя было понять, куда едет робот. Теперь углы согласованы,
     как у настоящей 4WIS-машины: в повороте участвуют все четыре колеса
     (задние как передние), краб — чистый боком под 90°, разворот на месте
     (передние +90°, задние −90°) и задний ход. */
  const DRIVE_MODES = [
    { sec: 8, angles: { FL: 0, FR: 0, RL: 0, RR: 0 }, spin: 1, title: 'прямо' },
    { sec: 4, angles: { FL: 26, FR: 34, RL: 26, RR: 34 }, spin: 1, title: 'поворот вправо' },
    { sec: 4, angles: { FL: 90, FR: 90, RL: 90, RR: 90 }, spin: 1, title: 'краб боком' },
    { sec: 5, angles: { FL: 90, FR: 90, RL: -90, RR: -90 }, spin: 1, title: 'разворот на месте' },
    { sec: 4, angles: { FL: 0, FR: 0, RL: 0, RR: 0 }, spin: -1, title: 'назад' },
  ];
  const STEER_RATE = 70;                       // °/с — рулевой модуль не скачет

  function driveMode(t) {
    const cycle = DRIVE_MODES.reduce((sum, m) => sum + m.sec, 0);
    let x = t % cycle;
    for (const mode of DRIVE_MODES) {
      if (x < mode.sec) return mode;
      x -= mode.sec;
    }
    return DRIVE_MODES[0];
  }

  function demoStep(dt) {
    const b = DEMO.battery;
    const mode = driveMode(Date.now() / 1000);
    DEMO.driveMode = mode.title;
    const targetRpm = (b.amps / 12) * 260 * mode.spin;
    DEMO.motors.forEach((m, i) => {
      const want = mode.angles[m.id] || 0;
      const step = STEER_RATE * dt;
      const diff = want - m.angle;
      m.angle = Math.abs(diff) <= step ? want : m.angle + (diff < 0 ? -step : step);
      m.rpm += (targetRpm - m.rpm) * Math.min(1, dt * 2);
      m.temp = 34 + Math.abs(m.rpm) / 40 + Math.sin(Date.now() / 5000 + i) * 1.4;
    });
    b.amps += ((6 + 12 * Math.abs(Math.sin(Date.now() / 9000))) - b.amps) * Math.min(1, dt * 0.6);
    b.soc = Math.max(6, b.soc - b.amps * dt / 3600 * 100 / 18.6);
    b.volts = 30 + b.soc / 100 * 13.8 - b.amps * 0.075 / 12;
    b.rangeKm = b.soc / 100 * 18.5;
    b.state = 'разряд';
    b.level = b.volts < 33.5 ? 'КРИТИЧЕСКИЙ' : b.volts < 35.5 ? 'НИЗКИЙ' : 'НОРМА';
    DEMO.powerKw = b.volts * b.amps / 1000;
  }

  /* ======================================================================
   * 2. API-источник (gui/backend.py)
   * ==================================================================== */
  const api = {
    ok: false,          // есть ли ответ сервера
    data: null,
    at: 0,              // когда получен последний снимок
    source: '—',
    error: '',
    timer: null,
  };

  const hasFetch = typeof fetch === 'function';

  function apiUrl(path) { return path; }        // относительные URL — работает на любом порту

  /** Данные сервера свежие (ответ был меньше 2 с назад). */
  function apiFresh() { return api.ok && !!api.data && (Date.now() - api.at) < 2000; }

  async function apiPoll() {
    if (!hasFetch) return;
    try {
      const res = await fetch(apiUrl('api/state'), { cache: 'no-store' });
      if (!res.ok) throw new Error('HTTP ' + res.status);
      const body = await res.json();
      if (body && body.ok && body.data) {
        api.data = body.data;
        api.source = body.data.source || '—';
        api.at = Date.now();
        api.ok = true;
        api.error = '';
      }
    } catch (err) {
      api.ok = false;
      api.error = String(err && err.message ? err.message : err);
    }
  }

  async function apiLock(path, payload) {
    const res = await fetch(apiUrl(path), {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload || {}),
    });
    return res.json();
  }

  /** Данные для отрисовки: свежий снимок API либо локальная демонстрация. */
  function read() {
    if (apiFresh()) return api.data;
    return {
      motors: DEMO.motors,
      battery: DEMO.battery,
      cargo: DEMO.cargo,
      mode: DEMO.mode,
      driveMode: DEMO.driveMode,
      route: DEMO.route,
      powerKw: DEMO.powerKw,
      linkOk: DEMO.linkOk,
      ts: Date.now() / 1000,
    };
  }

  /* ======================================================================
   * 3. Состояние экрана
   * ==================================================================== */
  const lock = { pin: '', busy: false, demoOpen: false, fails: 0, lockUntil: 0 };

  /**
   * Единственный источник правды о замке: пока сервер отвечает — его состояние,
   * иначе — локальное демонстрационное. Экрану не нужна своя копия состояния.
   */
  function lockView() {
    if (apiFresh() && api.data.lock) {
      const l = api.data.lock;
      return {
        open: !!l.open,
        blocked: !!l.blocked,
        remainingMs: Number(l.remainingMs || 0),
        attemptsLeft: l.attemptsLeft,
        from: 'сервер',
      };
    }
    const left = Math.max(0, lock.lockUntil - Date.now());
    return {
      open: lock.demoOpen,
      blocked: left > 0,
      remainingMs: left,
      attemptsLeft: DEMO.maxAttempts - lock.fails,
      from: 'демо',
    };
  }

  /* ======================================================================
   * 4. Разметка
   * ==================================================================== */
  const KEYPAD = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '⌫', '0', 'СБРОС'];
  const KEY_LABEL = { '⌫': '⌫', 'СБРОС': 'СБРОС' };

  /* Панель двигателей — вид робота сверху: корпус в центре, колёсные модули
     по углам. Направление колеса показывают сразу двумя способами: само
     колесо повёрнуто на свой угол и рядом написано словами «влево/вправо/
     прямо». Знак минус в интерфейсе не показывается — по нему нельзя было
     понять, куда смотрит колесо. */
  function mountMotors() {
    const grid = $('sc-motor-grid');
    if (!grid) return;

    const card = (m) => `
      <article class="rnode" id="mc-${m.id}">
        <div class="rnode-top">
          <b class="rnode-id">${m.id}</b>
          <span class="rnode-name">${m.title}</span>
          <span class="sc-chip sc-chip-idle" id="mc-state-${m.id}">ожидание</span>
        </div>
        <div class="rnode-body">
          <div class="rstat"><span>об/мин</span><b id="mc-rpm-${m.id}">0</b></div>
          <div class="rstat"><span>температура</span><b id="mc-temp-${m.id}">36<small>°C</small></b></div>
        </div>
        <div class="rnode-foot">
          <span class="rnode-dir straight" id="mc-dir-${m.id}">прямо</span>
          <span class="rnode-deg" id="mc-angle-${m.id}">0°</span>
        </div>
        <div class="pbar" title="обороты модуля"><i id="mc-bar-${m.id}"></i></div>
      </article>`;

    /* Координаты четырёх рулевых модулей в системе схемы (viewBox 240×360):
       по углам восьмиугольной рамы, как на настоящей машине. */
    const MODULES = { FL: [48, 84], FR: [192, 84], RL: [48, 276], RR: [192, 276] };

    /* Колесо: чёрная шина с красным ободом (как на фото), внутри — стрелка
       качения. Поворачивается целиком как группа, вокруг ступицы модуля. */
    const wheel = (m) => {
      const [x, y] = MODULES[m.id];
      return `
        <g class="rwheel" id="mc-wheel-${m.id}" role="img" aria-label="${m.id}: колесо прямо">
          <rect class="rwheel-tyre" x="${x - 17}" y="${y - 34}" width="34" height="68" rx="9"></rect>
          <rect class="rwheel-rim" x="${x - 9}" y="${y - 25}" width="18" height="50" rx="6"></rect>
          <line class="rwheel-dir" x1="${x}" y1="${y - 22}" x2="${x}" y2="${y + 22}"></line>
          <polygon class="rwheel-arrow" points="${x},${y - 31} ${x - 5},${y - 21} ${x + 5},${y - 21}"></polygon>
        </g>`;
    };

    /* Ступица рулевого модуля — круглая, поверх колеса: ось поворота видна. */
    const collar = (id) => {
      const [x, y] = MODULES[id];
      return `
        <g class="rmod" aria-hidden="true">
          <circle class="rmod-collar" cx="${x}" cy="${y}" r="14"></circle>
          <circle class="rmod-axis" cx="${x}" cy="${y}" r="4"></circle>
        </g>`;
    };

    const side = (ids) => DEMO.motors.filter((m) => ids.indexOf(m.id) >= 0).map(card).join('');

    grid.innerHTML = `
      <div class="rmap-col rmap-left">${side(['FL', 'RL'])}</div>
      <div class="robot" id="sc-robot">
        <span class="robot-front">перед<i aria-hidden="true"></i></span>
        <svg class="robot-svg" viewBox="0 0 240 360" role="img"
             aria-label="Робот сверху: восьмиугольная рама, четыре поворотных колёсных модуля">
          <!-- рама: восьмиугольник со срезанными углами и поперечинами -->
          <polygon class="rframe" points="60,14 180,14 226,60 226,300 180,346 60,346 14,300 14,60"></polygon>
          <rect class="rframe-inner" x="52" y="108" width="136" height="144" rx="6"></rect>
          <line class="rbeam" x1="26" y1="108" x2="214" y2="108"></line>
          <line class="rbeam" x1="26" y1="252" x2="214" y2="252"></line>
          ${DEMO.motors.map(wheel).join('')}
          ${DEMO.motors.map((m) => collar(m.id)).join('')}
        </svg>
        <span class="robot-rear">корма</span>
      </div>
      <div class="rmap-col rmap-right">${side(['FR', 'RR'])}</div>`;
  }

  function mountKeypad() {
    const pad = $('sc-keypad');
    if (!pad) return;
    pad.innerHTML = KEYPAD.map((k) => {
      const mode = k === '⌫' || k === 'СБРОС';
      const label = KEY_LABEL[k] || k;
      const aria = k === '⌫' ? 'удалить последнюю цифру'
        : k === 'СБРОС' ? 'очистить ввод' : 'цифра ' + k;
      return `<button type="button" class="sc-key${mode ? ' sc-key-mode' : ''}" data-key="${k}"
                 aria-label="${aria}">${label}</button>`;
    }).join('');
    pad.querySelectorAll('[data-key]').forEach((b) => b.addEventListener('click', () => press(b.dataset.key)));
  }

  /* ======================================================================
   * 5. Замок грузового отсека
   * ==================================================================== */
  function press(k) {
    if (lock.busy) return;                       // идёт проверка PIN — ввод не принимаем
    if (lockView().blocked) return;              // блокировку считает сервер (или демо-режим)
    if (k === 'СБРОС' || k === 'C') {
      lock.pin = '';
      renderPin();
      setMsg(PROMPT);
      return;
    }
    if (k === '⌫') lock.pin = lock.pin.slice(0, -1);
    else if (lock.pin.length < 8) lock.pin += k;
    renderPin();
  }

  function renderPin() {
    const el = $('sc-pin-dots');
    if (el) el.textContent = lock.pin ? '•'.repeat(lock.pin.length).split('').join(' ') : '— — — —';
  }

  function setMsg(text, cls) {
    const el = $('sc-pin-msg');
    if (!el) return;
    el.textContent = text;
    el.className = 'sc-pin-msg' + (cls ? ' ' + cls : '');
  }

  function shake() {
    const box = $('sc-lock-state');
    if (!box || !box.parentElement) return;
    box.parentElement.classList.add('sc-shake');
    setTimeout(() => box.parentElement.classList.remove('sc-shake'), 400);
  }

  /** Главное действие: открыть отсек по PIN либо закрыть уже открытый. */
  function toggleCargo() {
    if (lockView().open) closeCargo(); else tryOpen();
  }

  async function tryOpen() {
    if (lock.busy) return;
    const lv = lockView();
    if (lv.blocked) {
      setMsg('Ввод заблокирован: ещё ' + Math.ceil(lv.remainingMs / 1000) + ' с', 'err');
      return;
    }
    if (lock.pin.length < 4) { setMsg('PIN — не менее 4 цифр', 'warn'); return; }
    lock.busy = true;
    try {
      if (api.ok) {
        const res = await apiLock('api/lock/open', { pin: lock.pin });
        lock.pin = '';
        renderPin();
        if (res.ok) {
          setMsg('Доступ разрешён', 'ok');
          await apiAudit();
        } else if (res.reason === 'blocked') {
          setMsg('Ввод заблокирован: ещё ' + Math.ceil((res.remainingMs || 0) / 1000) + ' с', 'err');
          shake();
        } else {
          const left = res.attemptsLeft;
          setMsg(left !== undefined ? 'Неверный PIN. Осталось попыток: ' + left : 'Неверный PIN', 'err');
          shake();
        }
      } else {
        // демо-режим: проверка на месте
        const pin = lock.pin;
        lock.pin = '';
        renderPin();
        if (pin === DEMO.localPin) {
          lock.demoOpen = true;
          lock.fails = 0;
          setMsg('Доступ разрешён (демо-режим)', 'ok');
        } else {
          lock.fails += 1;
          if (lock.fails >= DEMO.maxAttempts) {
            lock.fails = 0;
            lock.lockUntil = Date.now() + DEMO.lockMs;
            setMsg('Пять неудачных попыток. Блокировка на 30 с', 'err');
          } else {
            setMsg('Неверный PIN. Осталось попыток: ' + (DEMO.maxAttempts - lock.fails), 'err');
          }
          shake();
        }
      }
    } catch (err) {
      setMsg('Нет связи с сервером: ' + (err && err.message ? err.message : err), 'err');
    } finally {
      lock.busy = false;
      render();
    }
  }

  async function closeCargo() {
    if (!lockView().open) return;
    lock.busy = true;
    try {
      if (api.ok) await apiLock('api/lock/close', {});
    } catch (err) { /* закрываем локально в любом случае */ }
    lock.demoOpen = false;
    lock.pin = '';
    renderPin();
    setMsg('Отсек закрыт', 'ok');
    lock.busy = false;
    render();
  }

  /* ======================================================================
   * 6. Отрисовка
   * ==================================================================== */
  function motorState(m) {
    if (m.online === false) return ['нет связи', 'sc-chip-err'];
    if (m.fault) return ['авария', 'sc-chip-err'];
    if (m.homed === false) return ['хоминг', 'sc-chip-warn'];
    if (m.temp > 55) return ['перегрев', 'sc-chip-err'];
    if (Math.abs(m.rpm) > 4) return ['движение', 'sc-chip-move'];
    return ['готов', 'sc-chip-ok'];
  }

  function fmt(v, digits) {
    return Number(v || 0).toFixed(digits === undefined ? 1 : digits).replace('.', ',');
  }

  function render() {
    const d = read();
    const fresh = apiFresh();

    /* двигатели: карточки + сводка */
    let moving = 0;
    let online = 0;
    const motors = d.motors || [];
    motors.forEach((m) => {
      const a = Number(m.angle || 0);
      const word = Math.abs(a) < 3 ? 'прямо' : (a > 0 ? 'вправо' : 'влево');
      const wheel = $('mc-wheel-' + m.id);
      if (wheel) {
        wheel.style.transform = 'rotate(' + a.toFixed(1) + 'deg)';
        wheel.setAttribute('aria-label', m.id + ': колесо ' + word
          + (word === 'прямо' ? '' : ' ' + Math.abs(Math.round(a)) + ' градусов'));
      }
      const dir = $('mc-dir-' + m.id);
      if (dir) {
        dir.textContent = word;
        dir.className = 'rnode-dir ' + (word === 'прямо' ? 'straight' : word === 'влево' ? 'left' : 'right');
      }
      const ang = $('mc-angle-' + m.id);
      if (ang) ang.textContent = (word === 'прямо' ? 0 : Math.abs(Math.round(a))) + '°';
      const rpm = $('mc-rpm-' + m.id);
      if (rpm) rpm.textContent = String(Math.round(m.rpm || 0));
      const temp = $('mc-temp-' + m.id);
      if (temp) temp.innerHTML = Math.round(m.temp || 0) + '<small>°C</small>';
      const bar = $('mc-bar-' + m.id);
      if (bar) {
        bar.style.width = Math.min(100, Math.abs(m.rpm || 0) / 3) + '%';
        bar.classList.toggle('neg', (m.rpm || 0) < 0);
      }
      const st = $('mc-state-' + m.id);
      if (st) {
        const s = motorState(m);
        st.textContent = s[0];
        st.className = 'sc-chip ' + s[1];
      }
      if (Math.abs(m.rpm || 0) > 4) moving += 1;
      if (m.online !== false) online += 1;
    });
    const sum = $('sc-motors-sum');
    if (sum) {
      // манёвр сообщает только демонстрация: реальные модули его не знают
      const manoeuvre = d.driveMode ? 'манёвр: ' + d.driveMode + ' · ' : '';
      sum.textContent = manoeuvre + 'в движении: ' + moving + ' из ' + motors.length
        + ' · модули на связи: ' + online + ' из ' + motors.length;
    }

    /* карта цеха: клетки и поза робота — в модуле map.js */
    if (window.RSMap && RSMap.tick) RSMap.tick(d);

    /* ИИ: сеть, команда, обучение, ROS 2 — в модуле ai_panel.js */
    if (window.RSAiPanel && RSAiPanel.render) RSAiPanel.render(d);

    /* шапка */
    const mode = $('sc-mode');
    if (mode) mode.textContent = d.mode || '—';
    const dot = $('sc-link-dot');
    if (dot) dot.classList.toggle('off', !d.linkOk);
    const txt = $('sc-link-text');
    if (txt) txt.textContent = d.linkOk ? 'связь с бортом: есть' : 'связь с бортом: нет';

    /* режим данных: сервер или демо */
    const chip = $('sc-data');
    if (chip) {
      if (fresh) {
        chip.textContent = 'данные: сервер (' + api.source + ')';
        chip.className = 'sc-chip sc-chip-ok';
      } else {
        chip.textContent = hasFetch ? 'данные: демо · сервер недоступен' : 'данные: демо';
        chip.className = 'sc-chip sc-chip-warn';
      }
    }

  }

  function clock() {
    const el = $('sc-clock');
    if (el) el.textContent = new Date().toLocaleTimeString('ru-RU', { hour12: false });
  }

  /* ======================================================================
   * 7. Навигация: основной экран ↔ инженерный пульт
   * ==================================================================== */
  /* Переход на пульт: в средах без cookie адрес несёт сессию (RS_AUTH),
     поэтому пароль спрашивается один раз — при входе. */
  function goConsole() {
    const url = 'index.html';
    if (window.RS_AUTH && window.RS_AUTH.go) window.RS_AUTH.go(url);
    else window.location.href = url;
  }

  function initNav() {
    const link = $('sc-console-link');
    if (link) link.addEventListener('click', (e) => { e.preventDefault(); goConsole(); });
  }

  /* ======================================================================
   * 8. Запуск
   * ==================================================================== */
  function boot() {
    mountMotors();
    mountKeypad();
    renderPin();
    setMsg(PROMPT);
    initNav();
    clock();
    setInterval(clock, 1000);

    /* Физическая клавиатура (дубликатов органов управления не создаёт).
       Привязываем её только тогда, когда на экране есть клавиатура набора:
       иначе цифры и Enter открывали бы отсек «вслепую» — без индикации и
       без единого органа управления на виду. */
    const keypad = document.getElementById('sc-keypad');
    if (keypad) {
      document.addEventListener('keydown', (e) => {
        if (/^[0-9]$/.test(e.key)) press(e.key);
        else if (e.key === 'Backspace') press('⌫');
        else if (e.key === 'Escape') press('СБРОС');
        else if (e.key === 'Enter') toggleCargo();
      });
    }

    // Опрос бэкенда
    if (hasFetch) {
      apiPoll().then(render);
      api.timer = setInterval(apiPoll, POLL_MS);
    }

    // Отрисовка; локальная анимация — только когда данных сервера нет
    let last = performance.now();
    setInterval(() => {
      const now = performance.now();
      const dt = Math.min(0.5, (now - last) / 1000);
      last = now;
      if (!apiFresh()) demoStep(dt);
      render();
    }, 200);

    window.addEventListener('beforeunload', () => {
      if (api.timer) clearInterval(api.timer);
    });
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot);
  else boot();

  // Для тестов и отладки в консоли браузера
  window.RS_MAIN = { api, lock, read, press, tryOpen, closeCargo, toggleCargo, lockView, render };
})();
