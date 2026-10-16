/* Бортовой экран: телеметрия и функции без PIN; PIN нужен только для кейса. */
(function () {
  'use strict';

  const LIDAR_RANGE_M = 10;
  const LIDAR_RING_M = 2;
  const POLL_MS = 250;

  const $ = (id) => document.getElementById(id);
  const setText = (el, txt) => { if (el) el.textContent = txt; };
  const fmt = (v, d) => {
    if (v === null || v === undefined || !Number.isFinite(+v)) return '—';
    return (+v).toFixed(d == null ? 1 : d);
  };

  function setSteeringAuthorized(on) {
    if (!on) stopDrive();
    steeringAuthorized = !!on;
    renderSteeringControls(lastData || {});
  }

  let lastData = null;
  let live = false;
  // На киоске PIN нужен только для открытия грузового отсека. Серверная
  // авторизация руления/тяги остаётся отдельной настройкой веб-пульта.
  let steeringAuthorized = true;
  let stateFetchPending = false;
  let driveTimer = null;
  let driveDirection = 0;
  let driveQueued = null;
  let driveSending = false;
  let driveFlush = Promise.resolve();
  const pendingSteer = Object.create(null);
  const pendingZero = Object.create(null);
  const pendingCalib = Object.create(null);
  const steerResult = Object.create(null);

  async function fetchState() {
    // Не запускаем параллельные 250-мс опросы: медленный ответ не должен
    // перезаписать более свежий скан лидара или телеметрию.
    if (stateFetchPending) return;
    stateFetchPending = true;
    try {
      const r = await fetch('api/kiosk_state', { cache: 'no-store' });
      if (!r.ok) throw new Error('http ' + r.status);
      const j = await r.json();
      if (j && j.ok && j.data) {
        lastData = j.data;
        live = true;
      } else {
        live = false;
      }
    } catch (e) {
      live = false;
    } finally {
      stateFetchPending = false;
    }
  }

  function clockText(d) {
    const pad = (n) => String(n).padStart(2, '0');
    return pad(d.getHours()) + ':' + pad(d.getMinutes());
  }

  function batteryPct(b) {
    if (b && b.soc != null && Number.isFinite(+b.soc)) return Math.max(0, Math.min(100, +b.soc));
    return null;
  }

  function renderBattery(d) {
    // A configured JBD BMS is authoritative; never fall back to simulated SOC
    // when its USB link is lost. Range is not provided by the JBD registers.
    const bms = window._kioskBms;
    const useBms = Boolean(bms && bms.configured);
    const measured = useBms && bms.connected && bms.data ? bms.data : {};
    const b = useBms ? {
      soc: measured.soc, volts: measured.volts, amps: measured.amps,
      tempC: measured.temperatures && measured.temperatures.length ? Math.max(...measured.temperatures) : null,
      rangeKm: null, state: bms.connected ? 'JBD BMS · ' + bms.reason : 'JBD BMS · нет свежих данных'
    } : (d.battery || {});
    const pct = batteryPct(b);
    const socEl = $('k-soc');
    const bar = $('k-soc-bar');
    setText(socEl, pct == null ? '—' : Math.round(pct));
    if (bar) bar.style.width = pct == null ? '0%' : pct + '%';

    let stateText = b.state || '—';
    document.body.classList.remove('soc-low', 'soc-crit');
    if (pct != null) {
      if (pct < 10) { stateText = 'КРИТИЧЕСКИЙ РАЗРЯД'; document.body.classList.add('soc-crit'); }
      else if (pct < 20) { stateText = 'Низкий заряд'; document.body.classList.add('soc-low'); }
    }
    setText($('k-soc-state'), stateText);

    // Подсказка на главном экране: панель АКБ — вход на страницу 4 (все банки).
    const more = $('battery-more');
    if (more) {
      let hint = 'Подробно: все банки и защиты →';
      if (bms && bms.configured === false) hint = 'BMS не подключена · настроить →';
      else if (useBms && !bms.connected) hint = 'BMS не отвечает · проверить →';
      else if (useBms && measured.cellCount) hint = 'Подробно: ' + measured.cellCount + ' банок и защиты →';
      more.textContent = hint;
      more.dataset.tone = bms && (bms.configured === false || (useBms && !bms.connected)) ? 'warn' : '';
    }

    setText($('k-volts'), fmt(b.volts, 1));
    setText($('k-amps'), fmt(b.amps, 1));
    setText($('k-power'), fmt(useBms ? (measured.watts == null ? null : measured.watts / 1000) : d.powerKw, 2));
    setText($('k-range'), fmt(b.rangeKm, 1));
    setText($('k-btemp'), fmt(b.tempC, 0));
    setText($('speed'), fmt(d.speedMps, 1));
  }

  function modTone(m) {
    if (!m) return 'idle';
    if (m.fault) return 'err';
    if (m.online === false) return 'idle';
    if (m.temp != null && +m.temp > 70) return 'warn';
    if (m.online) {
      if (m.moving) return 'move';
      return 'ok';
    }
    return 'idle';
  }

  function modStateText(m) {
    if (!m) return '—';
    if (m.fault) return 'ошибка';
    if (m.online === false) return 'нет связи';
    if (m.moving) return 'в движении';
    return 'онлайн';
  }

  function renderChassis(d) {
    const motors = Array.isArray(d.motors) ? d.motors : [];
    const link = !!d.linkOk;
    const moving = motors.filter((m) => m && m.moving).length;
    const online = motors.filter((m) => m && m.online).length;
    const parts = [];
    if (d.mode) parts.push(d.mode);
    parts.push('модулей: ' + online + '/4');
    if (moving) parts.push('рулевых модулей в движении ' + moving);
    parts.push(link ? 'связь: ок' : 'нет связи');
    setText($('chassis-meta'), parts.join(' · '));

    ['FL', 'FR', 'RL', 'RR'].forEach((id) => {
      const m = motors.find((x) => x.id === id) || {};
      const tone = modTone(m);
      const card = $('mod-' + id);
      const wheel = $('wheel-' + id);
      if (card) card.setAttribute('data-tone', tone);
      if (wheel) {
        wheel.setAttribute('data-tone', tone);
        wheel.classList.toggle('is-driving', !!m.moving);
      }
      const chip = $('mod-state-' + id);
      if (chip) {
        chip.textContent = modStateText(m);
        chip.className = 'chip ' + tone;
      }
      const hasAngle = m.angle != null && Number.isFinite(+m.angle);
      const angle = hasAngle ? Math.max(-90, Math.min(90, +m.angle)) : 0;
      setText($('angle-' + id), hasAngle ? Math.round(angle) + '°' : '—');
      if (wheel) wheel.style.transform = 'rotate(' + angle.toFixed(1) + 'deg)';
      setText($('rpm-' + id), m.rpm == null ? '—' : fmt(m.rpm, 0));
      setText($('temp-' + id), m.temp == null ? '—' : fmt(m.temp, 0));
    });
  }

  function renderMode(d) {
    const pill = $('mode-pill');
    if (!pill) return;
    const link = !!d.linkOk;
    if (!link) {
      pill.textContent = 'НЕТ СВЯЗИ';
      pill.className = 'mode-pill err';
    } else if (d.mode) {
      const t = String(d.mode);
      pill.textContent = t;
      if (/ошиб|неисправ|авар/i.test(t)) pill.className = 'mode-pill err';
      else if (/режим|автоном|готов/i.test(t)) pill.className = 'mode-pill';
      else pill.className = 'mode-pill idle';
    } else {
      pill.textContent = 'ОЖИДАНИЕ';
      pill.className = 'mode-pill idle';
    }
  }

  function lidarPoints(d) {
    if (!Array.isArray(d.lidar)) return [];
    return d.lidar.filter((p) => p && Number.isFinite(p.a) && Number.isFinite(p.r) && p.r >= 0);
  }

  // Сервер помечает выдуманный скан (RC_SLAM_SOURCE=sim, ?demo=1, авто-скан
  // стенда): такие точки показываются как демо, а не как реальный лидар.
  function lidarIsDemo(d) {
    if (d && d.lidarDemo === true) return true;
    const st = d && d.lidarStatus;
    return !!(st && st.demo === true);
  }

  // Откуда реально читается скан (GET /api/lidar): драйвер ROS 2 с топиком или
  // USB-порт приложения. Подпись под картой — единственное место, где оператор
  // видит, кто сейчас держит лидар, поэтому ROS 2 назван явно.
  function lidarSourceText(d) {
    const st = d && d.lidarStatus;
    if (!st) return '';
    const ros = st.ros;
    if (ros && ros.connected) {
      const parts = ['ROS 2', String(ros.topic || '/scan')];
      const supervisor = ros.supervisor;
      if (supervisor && supervisor.driver) {
        parts.push(String(supervisor.driver_package || supervisor.driver));
      }
      if (ros.hz) parts.push(ros.hz + ' Гц');
      return parts.join(' · ');
    }
    const det = st.detected;
    if (!det || !det.port) return '';
    const parts = [String(det.port).split('/').pop()];
    if (det.protocol) parts.push(String(det.protocol));
    if (det.baud) parts.push(det.baud + ' бод');
    if (st.source === 'peer') parts.push('скан от пульта');
    return parts.join(' · ');
  }

  function drawLidar(canvas, pts) {
    if (!canvas) return;
    const ctx = canvas.getContext && canvas.getContext('2d');
    if (!ctx) return;
    const dpr = window.devicePixelRatio || 1;
    // clientWidth/clientHeight бывают 0, если раскладка ещё не посчиталась.
    // Размер атрибута canvas хранится в физических пикселях, поэтому при
    // fallback делим его на DPR, иначе на Retina-кiosk карта раздувается вдвое.
    const rect = canvas.getBoundingClientRect ? canvas.getBoundingClientRect() : null;
    const parent = canvas.parentElement;
    const w = canvas.clientWidth || (rect && rect.width) || (parent && parent.clientWidth) || (canvas.width / dpr) || 280;
    const h = canvas.clientHeight || (rect && rect.height) || (parent && parent.clientHeight) || (canvas.height / dpr) || 280;
    if (!w || !h) return;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const cx = w / 2;
    const cy = h / 2;
    const R = Math.max(20, Math.min(w, h) / 2 - 20);
    const k = R / LIDAR_RANGE_M;

    // сетка колец
    ctx.strokeStyle = '#2a2a2c';
    ctx.lineWidth = 1.5;
    for (let m = LIDAR_RING_M; m <= LIDAR_RANGE_M; m += LIDAR_RING_M) {
      ctx.beginPath();
      ctx.arc(cx, cy, m * k, 0, Math.PI * 2);
      ctx.stroke();
    }
    // подписи метров
    ctx.fillStyle = '#7a7a7e';
    ctx.font = '12px ' + getComputedStyle(document.body).fontFamily;
    for (let m = LIDAR_RING_M; m <= LIDAR_RANGE_M; m += LIDAR_RING_M) {
      ctx.fillText(m + ' м', cx + 4, cy - m * k - 4);
    }
    // оси
    ctx.strokeStyle = '#333335';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R);
    ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy);
    ctx.stroke();
    // вперёд (ось Y-) пометим
    ctx.fillStyle = '#ffffff';
    ctx.beginPath();
    ctx.moveTo(cx, cy - R + 2);
    ctx.lineTo(cx - 5, cy - R + 12);
    ctx.lineTo(cx + 5, cy - R + 12);
    ctx.closePath();
    ctx.fill();

    // Дальность карты — 10 м. Дальние точки не прижимаем к внешнему кольцу:
    // прежний clamp создавал искусственную сплошную окружность вокруг карты.
    const visiblePts = pts.filter((point) => point.r <= LIDAR_RANGE_M);
    if (visiblePts.length) {
      const near = Math.min.apply(null, visiblePts.map((p) => p.r));
      visiblePts.forEach((p) => {
        const r = p.r * k;
        const x = cx - r * Math.sin(p.a);
        const y = cy - r * Math.cos(p.a);
        // Ближайшее препятствие подсвечиваем отдельным тоном.
        if (p.r === near && near < 1.0) ctx.fillStyle = '#e5e5e5';
        else if (p.r === near && near < 2.0) ctx.fillStyle = '#c2c2c2';
        else ctx.fillStyle = '#ffffff';
        ctx.fillRect(x - 2, y - 2, 4, 4);
      });
    }
  }

  function demoLidar() {
    const pts = [];
    const t = Date.now() / 1000;
    const speed = 0.7;
    for (let i = 0; i < 180; i++) {
      const a = (i / 180) * Math.PI * 2 - Math.PI;
      const cosA = Math.cos(a);
      const sinA = Math.sin(a);
      const rx = Math.abs(cosA) > 1e-4 ? Math.abs(3.5 / cosA) : 12;
      const ry = Math.abs(sinA) > 1e-4 ? Math.abs(2.5 / sinA) : 12;
      let r = Math.min(rx, ry);
      if (Math.abs(a - 0.6) < 0.08) r = 1.8 + Math.sin(t) * 0.1;
      if (Math.abs(a + 0.9) < 0.07) r = 2.4;
      r += Math.sin(t * speed + a * 3) * 0.1;
      if (r < 0.4) r = 0.4;
      if (r > 9.8) r = 9.8;
      pts.push({ a, r });
    }
    return pts;
  }

  // Подробная диагностика берётся только из статуса сервера: экран не угадывает
  // причину отказа. Без свежих реальных точек телеметрия закрывается блокером.
  // «Продолжить без лидара»: оператор сам закрыл плашку. Телеметрия
  // открывается, скан лидара при этом не выдумывается (карта пустая, «скана
  // нет»), а внизу экрана остаётся предупреждение — по нему плашка открывается
  // снова. Как только лидар заработал, флаг сбрасывается: следующая потеря
  // скана снова закроет экран.
  let lidarDismissed = false;

  function renderLidarBlocker(d) {
    const blocker = $('lidar-blocker');
    if (!blocker) return;
    const st = d && d.lidarStatus && typeof d.lidarStatus === 'object' ? d.lidarStatus : null;
    const pts = d ? lidarPoints(d) : [];
    const demo = d ? lidarIsDemo(d) : false;
    const working = live && d.linkOk !== false && st && st.connected === true && pts.length > 0 && !demo;
    if (working) lidarDismissed = false;
    const warnBar = $('lidar-warn-bar');
    if (warnBar) {
      warnBar.hidden = !!working || !lidarDismissed;
      if (!working && lidarDismissed) {
        setText($('lidar-warn-text'), st && st.reason ? String(st.reason) : (live ? 'скана нет' : 'нет связи с сервером'));
      }
    }
    blocker.hidden = !!working || lidarDismissed;
    if (working) return;

    const title = $('lidar-block-title');
    const reasonEl = $('lidar-block-reason');
    const detailsEl = $('lidar-block-details');
    const hintEl = $('lidar-block-hint');
    let reason;
    if (!live) {
      reason = 'Нет связи с сервером киоска — получить фактическую диагностику лидара не удалось.';
    } else if (!st) {
      reason = 'Сервер не передал статус лидара.';
    } else if (demo || st.demo) {
      reason = st.reason || 'Получены только демонстрационные данные; реальный лидар не подтверждён.';
    } else if (st.reason) {
      reason = String(st.reason);
    } else if (st.connected !== true) {
      reason = 'Сервер сообщает, что лидар не подключён.';
    } else {
      reason = 'Статус подключения есть, но свежих точек скана нет.';
    }
    setText(title, working ? 'Лидар работает' : (!live ? 'Нет связи с сервером' : 'Лидар не готов'));
    setText(reasonEl, reason);

    const details = [];
    if (st) {
      if (st.last_error) details.push('Ошибка драйвера: ' + String(st.last_error));
      const usb = st.usb_diagnostics;
      if (usb && usb.summary) details.push(String(usb.summary));
      if (usb && Array.isArray(usb.ports)) {
        usb.ports.forEach((port) => {
          const flags = [];
          if (port.accessible === false) flags.push('нет доступа');
          if (port.excluded) flags.push('исключён настройками');
          details.push(String(port.port || port.device || 'USB-порт') + (flags.length ? ' — ' + flags.join(', ') : ''));
        });
      }
    }
    if (detailsEl) {
      detailsEl.hidden = details.length === 0;
      setText(detailsEl, details.join(' · '));
    }
    const hint = st && st.hint ? String(st.hint) : '';
    if (hintEl) {
      hintEl.hidden = !hint;
      setText(hintEl, hint);
    }
    renderLidarPorts(st);
  }

  // ---------------------------------------------------------------------
  // Блок «Порты на роботе»: где физически искать лидар.
  //
  // Всё берётся из статуса сервера (lidarStatus.ports): список serial-портов с
  // ролью каждого (кандидат / мастер руля / модуль шасси / занят / не отвечает),
  // пути by-id и by-path (по by-path виден разъём или хаб), и режим «найти
  // лидар» — сервер следит, какой порт появился или исчез при переподключении
  // кабеля. Экран ничего не угадывает: нет данных — так и пишет.
  // ---------------------------------------------------------------------
  let findRequestPending = false;
  let findLastHighlight = null;

  function shortPath(path) {
    const text = String(path || '');
    const idx = text.lastIndexOf('/');
    return idx >= 0 ? text.slice(idx + 1) : text;
  }

  function addPath(li, label, paths) {
    if (!Array.isArray(paths) || !paths.length) return;
    paths.forEach((path) => {
      const span = document.createElement('span');
      span.className = 'lidar-port-path';
      const b = document.createElement('b');
      b.textContent = label + ': ';
      span.appendChild(b);
      span.appendChild(document.createTextNode(String(path)));
      li.appendChild(span);
    });
  }

  function findEventText(ev) {
    if (!ev || typeof ev !== 'object') return '';
    const kind = ev.kind === 'appeared' ? 'появился' : (ev.kind === 'disappeared' ? 'исчез' : String(ev.kind || ''));
    const where = Array.isArray(ev.by_path) && ev.by_path.length ? ' · ' + shortPath(ev.by_path[0]) : '';
    return shortPath(ev.device) + ' ' + kind + where;
  }

  function renderLidarPorts(st) {
    const box = $('lidar-ports');
    if (!box) return;
    const list = $('lidar-ports-list');
    const emptyEl = $('lidar-ports-empty');
    const usbEl = $('lidar-ports-usb');
    const findStatus = $('lidar-find-status');
    const findBtn = $('lidar-find-btn');
    const placeEl = $('lidar-place-note');
    const overview = st && st.ports && typeof st.ports === 'object' && !Array.isArray(st.ports) ? st.ports : null;

    if (!overview) {
      box.hidden = false;
      if (list) list.innerHTML = '';
      if (emptyEl) {
        emptyEl.hidden = false;
        setText(emptyEl, st
          ? 'Сервер не передал список портов (старая версия сервера или статус без поля ports). Проверить вручную: ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-path/'
          : 'Список портов недоступен: нет статуса лидара от сервера.');
      }
      if (usbEl) usbEl.hidden = true;
      if (findStatus) findStatus.hidden = true;
      if (findBtn) findBtn.disabled = true;
      if (placeEl) placeEl.hidden = true;
      return;
    }
    box.hidden = false;
    if (findBtn) findBtn.disabled = false;

    const find = overview.find && typeof overview.find === 'object' ? overview.find : null;
    const highlight = find && find.highlight ? String(find.highlight) : null;
    const events = find && Array.isArray(find.events) ? find.events : [];
    const eventByDevice = {};
    events.forEach((ev) => { if (ev && ev.device) eventByDevice[String(ev.device)] = ev; });

    const ports = Array.isArray(overview.ports) ? overview.ports : [];
    if (list) {
      list.innerHTML = '';
      ports.forEach((port) => {
        const li = document.createElement('li');
        const role = String(port.role || 'candidate');
        li.className = 'lidar-port role-' + role + (highlight && String(port.device) === highlight ? ' is-highlight' : '');
        const line = document.createElement('div');
        line.className = 'lidar-port-line';
        const name = document.createElement('span');
        name.className = 'lidar-port-name';
        name.textContent = String(port.device || port.port || 'порт');
        line.appendChild(name);
        const roleEl = document.createElement('span');
        roleEl.className = 'lidar-port-role';
        roleEl.textContent = String(port.role_label || role);
        line.appendChild(roleEl);
        const ev = eventByDevice[String(port.device)];
        if (ev) {
          const change = document.createElement('span');
          change.className = 'lidar-port-change';
          change.textContent = ev.kind === 'appeared' ? '← только что появился' : '← только что исчез';
          line.appendChild(change);
        }
        li.appendChild(line);
        const whyParts = [];
        if (port.why) whyParts.push(String(port.why));
        if (port.accessible === false && role !== 'no_access' && role !== 'missing') whyParts.push('нет доступа (группа dialout)');
        if (port.note && role === 'candidate') whyParts.push('последняя попытка: ' + String(port.note));
        if (whyParts.length) {
          const why = document.createElement('div');
          why.className = 'lidar-port-why';
          why.textContent = whyParts.join(' · ');
          li.appendChild(why);
        }
        addPath(li, 'by-path (разъём/хаб)', port.by_path);
        addPath(li, 'by-id', port.by_id);
        if ((!port.by_path || !port.by_path.length) && role !== 'missing') {
          const none = document.createElement('span');
          none.className = 'lidar-port-path';
          none.textContent = 'by-path: нет (система не дала физический USB-путь)';
          li.appendChild(none);
        }
        list.appendChild(li);
      });
      // Порт, который исчез при отключении кабеля: в списке его уже нет, но
      // именно он и есть ответ на вопрос «где лидар» — показываем отдельно.
      events.slice().reverse().forEach((ev) => {
        if (!ev || ev.kind !== 'disappeared') return;
        if (ports.some((port) => String(port.device) === String(ev.device))) return;
        const li = document.createElement('li');
        li.className = 'lidar-port role-missing' + (highlight === String(ev.device) ? ' is-highlight' : '');
        const line = document.createElement('div');
        line.className = 'lidar-port-line';
        const name = document.createElement('span');
        name.className = 'lidar-port-name';
        name.textContent = String(ev.device);
        line.appendChild(name);
        const change = document.createElement('span');
        change.className = 'lidar-port-change';
        change.textContent = '← исчез при отключении кабеля';
        line.appendChild(change);
        li.appendChild(line);
        addPath(li, 'by-path (разъём/хаб)', ev.by_path);
        addPath(li, 'by-id', ev.by_id);
        list.appendChild(li);
      });
    }

    if (emptyEl) {
      if (!ports.length) {
        emptyEl.hidden = false;
        const devices = Array.isArray(overview.usb_devices) ? overview.usb_devices : [];
        const checks = [
          'Serial-портов в системе нет (/dev/ttyUSB*, /dev/ttyACM*, /dev/serial/by-*). Что проверить:',
          '1. Кабель лидара и USB-переходника вставлен до конца с обеих сторон; кабель с передачей данных, не только зарядный.',
          '2. Питание лидара: мотор крутится? Если нет — питание или разъём питания.',
          '3. USB-переходник (CP2102/CH340/FTDI): переткнуть в другой разъём, проверить светодиод.',
          '4. В терминале: lsusb и ls -l /dev/serial/by-path/ — ниже то, что видит ядро по USB.',
        ];
        const tty = overview.tty && typeof overview.tty === 'object' ? overview.tty : null;
        if (tty && Array.isArray(tty.names)) {
          const usbLike = Array.isArray(tty.usb_like) ? tty.usb_like : [];
          checks.push('ls /dev/tty*: ' + tty.count + ' записей' + (usbLike.length
            ? ', из них последовательные порты: ' + usbLike.join(', ')
            : ', USB/UART-портов (ttyUSB*, ttyACM*, ttyAMA*) среди них нет — устройств не обнаружено'));
        } else {
          checks.push('ls /dev/tty*: сервер не передал результат.');
        }
        const lsusb = overview.lsusb && typeof overview.lsusb === 'object' ? overview.lsusb : null;
        if (lsusb && lsusb.available && Array.isArray(lsusb.lines) && lsusb.lines.length) {
          checks.push('lsusb: ' + lsusb.lines.join(' | '));
        } else if (lsusb && lsusb.available) {
          checks.push('lsusb: устройств не обнаружено' + (lsusb.error ? ' (' + lsusb.error + ')' : ''));
        } else if (lsusb) {
          checks.push('lsusb: ' + (lsusb.error || 'недоступен'));
        }
        if (devices.length) {
          checks.push('USB-устройства по sysfs (без serial-драйвера): ' + devices.map((dev) => [dev.id, dev.manufacturer, dev.product].filter(Boolean).join(' ')).join('; ') + ' — если среди них переходник лидара, не загружен драйвер tty (dmesg | tail).');
        } else {
          checks.push('USB-устройств (кроме хабов) по sysfs не обнаружено: до USB лидар не доходит — кабель, питание или переходник.');
        }
        setText(emptyEl, checks.join('\n'));
      } else {
        emptyEl.hidden = true;
      }
    }
    if (usbEl) usbEl.hidden = true;

    if (findStatus) {
      findStatus.hidden = false;
      if (find && find.active) {
        const left = Number.isFinite(+find.expires_in_s) ? Math.round(+find.expires_in_s) : null;
        let text = 'Режим «найти лидар» включён' + (left != null ? ' (ещё ' + left + ' с)' : '') +
          '. Отключи и снова подключи кабель лидара — порт, который исчезнет и появится, подсветится в списке. ' +
          'Это единственный способ по программе определить, в каком разъёме лидар.';
        if (events.length) text += ' Последнее: ' + findEventText(events[events.length - 1]) + '.';
        else text += ' Изменений портов пока нет.';
        setText(findStatus, text);
      } else if (find && events.length) {
        setText(findStatus, 'Режим «найти лидар» завершён' + (find.stopped_reason ? ' (' + find.stopped_reason + ')' : '') +
          '. Последнее изменение: ' + findEventText(events[events.length - 1]) + '. Подсвеченный порт — тот, что менялся при переподключении.');
      } else if (find && find.stopped_reason) {
        setText(findStatus, 'Режим «найти лидар» завершён (' + find.stopped_reason + '): порты не менялись — кабель, который переподключали, система не заметила.');
      } else {
        setText(findStatus, 'Нажми «Найти лидар», затем отключи и подключи кабель лидара: сервер покажет, какой порт при этом исчез и появился.');
      }
    }
    if (findBtn) {
      const active = !!(find && find.active);
      findBtn.classList.toggle('is-active', active);
      findBtn.textContent = active ? 'Остановить поиск' : 'Найти лидар';
      findBtn.dataset.action = active ? 'stop' : 'start';
    }
    if (placeEl) {
      const note = overview.place_note ? String(overview.place_note) : '';
      placeEl.hidden = !note;
      setText(placeEl, note ? 'Подсказка (RC_LIDAR_PLACE_NOTE): ' + note : '');
    }
    if (highlight && highlight !== findLastHighlight) {
      findLastHighlight = highlight;
      try { if (navigator.vibrate) navigator.vibrate(60); } catch (e) { /* не везде есть */ }
    }
  }

  async function toggleLidarFind() {
    if (findRequestPending) return;
    const btn = $('lidar-find-btn');
    const action = btn && btn.dataset && btn.dataset.action === 'stop' ? 'stop' : 'start';
    findRequestPending = true;
    if (btn) btn.disabled = true;
    try {
      const r = await fetch('api/lidar/find', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ action }),
      });
      const j = await r.json().catch(() => null);
      if (!r.ok || !j || !j.ok) throw new Error((j && j.error) || ('http ' + r.status));
      if (j.find && lastData && lastData.lidarStatus && lastData.lidarStatus.ports) {
        lastData.lidarStatus.ports.find = j.find;
      }
      fetchState();
    } catch (e) {
      const statusEl = $('lidar-find-status');
      if (statusEl) {
        statusEl.hidden = false;
        setText(statusEl, 'Режим «найти лидар» не включился: ' + (e && e.message ? e.message : e) + '. Сервер не поддерживает api/lidar/find или недоступен.');
      }
    } finally {
      findRequestPending = false;
      if (btn) btn.disabled = false;
    }
  }

  function initLidarFind() {
    const btn = $('lidar-find-btn');
    if (!btn) return;
    btn.addEventListener('click', (ev) => {
      if (ev && ev.preventDefault) ev.preventDefault();
      toggleLidarFind();
    });
  }

  function initLidarContinue() {
    const btn = $('lidar-continue-btn');
    if (btn) {
      btn.addEventListener('click', (ev) => {
        if (ev && ev.preventDefault) ev.preventDefault();
        lidarDismissed = true;
        renderLidarBlocker(lastData || {});
      });
    }
    const bar = $('lidar-warn-bar');
    if (bar) {
      bar.addEventListener('click', (ev) => {
        if (ev && ev.preventDefault) ev.preventDefault();
        lidarDismissed = false;
        renderLidarBlocker(lastData || {});
      });
    }
  }

  // На основной карте остаётся короткая подпись; при отсутствии лидара
  // блокер показывает оператору серверную причину и не выдаёт демо за реальный скан.
  function renderLidar(d) {
    const canvas = $('lidar-canvas');
    let pts = lidarPoints(d);
    let isDemo = lidarIsDemo(d);
    const demoReq = new URLSearchParams(window.location.search).get('demo') === '1' || localStorage.getItem('kiosk_demo') === '1';
    if (!pts.length && (demoReq || (typeof autoScanOn !== 'undefined' && autoScanOn))) {
      pts = demoLidar();
      isDemo = true;
    }
    const has = pts.length > 0;
    const near = has ? Math.min.apply(null, pts.map((p) => p.r)) : null;
    const visibleCount = pts.filter((point) => point.r <= LIDAR_RANGE_M).length;

    const source = has && !isDemo ? lidarSourceText(d) : '';
    setText($('lidar-meta'), has
      ? (isDemo
        ? 'демо · ' + pts.length + ' точек (не лидар)'
        : 'в реальном времени · ' + pts.length + ' точек · на карте: ' + visibleCount + '/' + pts.length + (source ? ' · ' + source : ''))
      : 'нет скана');
    setText($('lidar-count'), has ? String(pts.length) : '—');
    setText($('lidar-near'), near == null ? '—' : fmt(near, 2) + ' м');
    const empty = $('lidar-empty');
    if (empty) {
      empty.hidden = has && !isDemo;
      if (!(has && !isDemo)) {
        setText(empty, 'скана лидара нет');
      }
      empty.title = '';
    }
    drawLidar(canvas, pts);
  }

  // Экран рисуется по блокам, и сбой одного не гасит остальные. Раньше любое
  // исключение в render() обрывало и loop() (он без try/catch), и бортовой
  // экран замирал целиком: ни часов, ни карты лидара — при живом сервере.
  const failedSteps = {};

  function step(name, draw) {
    try {
      draw();
    } catch (err) {
      if (!failedSteps[name]) {
        failedSteps[name] = true;
        console.warn('Бортовой экран: блок ' + name + ' не нарисован:', err && err.message ? err.message : err);
      }
    }
  }

  function render() {
    const d = lastData || {};
    step('clock', () => setText($('clock'), clockText(new Date())));
    step('link', () => {
      const dataChip = $('data-chip');
      if (!dataChip) return;
      if (!live) {
        dataChip.textContent = 'НЕТ СВЯЗИ С СЕРВЕРОМ';
        dataChip.className = 'chip err';
      } else if (d.linkOk === false) {
        dataChip.textContent = 'нет данных от робота';
        dataChip.className = 'chip warn';
      } else {
        dataChip.textContent = 'в реальном времени';
        dataChip.className = 'chip ok';
      }
    });
    step('stale', () => document.body.classList.toggle('is-stale', !live));

    step('mode', () => renderMode(d));
    step('battery', () => renderBattery(d));
    step('chassis', () => renderChassis(d));
    step('lidar', () => renderLidar(d));
    step('lidar-blocker', () => renderLidarBlocker(d));
    step('steering', () => renderSteeringControls(d));
  }

  function loop() {
    try {
      render();
    } catch (err) {
      console.warn('Бортовой экран: сбой отрисовки:', err && err.message ? err.message : err);
    } finally {
      requestAnimationFrame(() => setTimeout(loop, 250));
    }
  }

  function boot() {
    fetchState();
    setInterval(fetchState, POLL_MS);
    loop();
    initKioskControls();
    initLidarFind();
    initLidarContinue();
    initSwipePage();
    initSteeringControls();
  }

  /* =======================================================================
     КИОСК: функции доступны без PIN; PIN-клавиатура вызывается только при
     открытии грузового отсека. Руление и тяга сохраняют свои лимиты и watchdog.
     ======================================================================= */
  function initKioskControls() {
    const overlay = $('ctrl-overlay');
    const backdrop = $('ctrl-backdrop');
    const sheet = overlay ? overlay.querySelector('.ctrl-sheet') : null;
    const openBtn = $('k-ctrl-btn');
    const closeBtn = $('ctrl-close');
    const pinView = $('pin-view');
    const ctrlView = $('ctrl-view');
    const pinDots = $('pin-dots');
    const pinDisplay = $('pin-display');
    const pinMsg = $('pin-msg');
    const pinPad = $('pin-pad');
    const pinSubmit = $('pin-submit');
    const pinCancel = $('pin-cancel');
    const ctrlBack = $('ctrl-back');
    const ctrlLogout = $('ctrl-logout');
    const lockChip = $('ctrl-lock-chip');
    const lockDetail = $('ctrl-lock-detail');
    const attemptsEl = $('ctrl-attempts');
    const subtitle = $('ctrl-subtitle');
    const titleEl = document.getElementById('ctrl-title');
    const compartmentBtn = $('btn-compartment');
    const btnSteeringControls = $('btn-steering-controls');
    const compartmentTitle = $('compartment-title');
    const compartmentHint = $('compartment-hint');
    const autoscanBtn = $('btn-autoscan');
    const autoscanHint = $('autoscan-hint');
    const autoscanState = $('autoscan-state');
    const scanBeam = $('lidar-scan-beam');
    const scanLabel = $('lidar-scan-label');
    const lidarSection = document.querySelector('.lidar');
    const btnThemes = $('btn-themes');
    const themesView = $('themes-view');
    const themesGrid = $('themes-grid');
    const themesBack = $('themes-back');
    const themesToStats = $('themes-to-stats');
    const btnResetMap = $('btn-reset-map');
    const resetView = $('reset-view');
    const resetBack = $('reset-back');
    const resetCancel = $('reset-cancel');
    const resetConfirm = $('reset-confirm');
    const resetConfirmCheck = $('reset-confirm-check');
    const resetMsg = $('reset-msg');

    if (!overlay || !openBtn || !pinDots) return;

    let pinBuf = '';
    let lockState = null;   // {open, blocked, remainingMs, attemptsLeft}
    let lockTimer = null;
    let busy = false;
    let dragging = false;
    let lastHoveredBtn = null;
    // авто-сканирование лидара (по умолчанию выкл)
    let autoScanOn = false;
    try { autoScanOn = localStorage.getItem('kiosk_autoscan') === '1'; } catch(e) {}

    // ---------- helpers ----------
    function setHidden(el, h) { if (el) el.hidden = h; }
    function setMsg(text, kind) {
      if (!pinMsg) return;
      pinMsg.textContent = text || '';
      pinMsg.className = 'pin-msg' + (kind ? ' ' + kind : '');
    }
    function vibrate(pattern) {
      try { if (navigator.vibrate) navigator.vibrate(pattern); } catch (e) {}
    }
    function updatePinDisplay() {
      const dots = pinDots.querySelectorAll('span');
      dots.forEach((sp, i) => sp.classList.toggle('filled', i < pinBuf.length));
      if (pinDisplay) {
        if (!pinBuf.length) pinDisplay.textContent = '••••';
        else {
          const masked = '•'.repeat(pinBuf.length);
          const empty = '·'.repeat(Math.max(0, 4 - pinBuf.length));
          // показываем 4 точки минимум, до 8
          pinDisplay.textContent = masked + (pinBuf.length < 4 ? empty.slice(pinBuf.length) : '');
          // если >4, показываем все
          if (pinBuf.length > 4) pinDisplay.textContent = masked;
        }
      }
      if (pinSubmit) pinSubmit.disabled = busy || pinBuf.length < 4;
    }
    function shakeDots() {
      pinDots.classList.remove('shake');
      void pinDots.offsetWidth;
      pinDots.classList.add('shake');
      setTimeout(() => pinDots.classList.remove('shake'), 420);
    }
    function setBusy(b) {
      busy = b;
      if (pinSubmit) pinSubmit.disabled = b || pinBuf.length < 4;
      pinPad.querySelectorAll('.pad-btn').forEach(btn => {
        if (lockState && lockState.blocked) btn.disabled = true;
        else btn.disabled = b;
      });
      if (compartmentBtn) compartmentBtn.disabled = b || !!(lockState && lockState.blocked && !lockState.open);
    }

    function showPinView() {
      pinBuf = '';
      setHidden(pinView, false);
      setHidden(ctrlView, true);
      setHidden(themesView, true);
      setHidden(resetView, true);
      if (subtitle) subtitle.textContent = 'PIN запрашивается только для открытия кейса';
      if (titleEl) titleEl.textContent = 'Открыть отсек';
      const hint = $('pin-hint');
      if (hint) hint.textContent = 'Введите PIN-код · 4–8 цифр';
      if (pinSubmit) pinSubmit.textContent = 'Открыть отсек';
      if (pinCancel) pinCancel.textContent = 'Назад к функциям';
      setMsg('', '');
      updatePinDisplay();
    }
    function showCtrlView() {
      setHidden(pinView, true);
      setHidden(ctrlView, false);
      setHidden(themesView, true);
      setHidden(resetView, true);
      if (subtitle) subtitle.textContent = 'функции доступны без PIN · PIN нужен только для открытия кейса';
      if (titleEl) titleEl.textContent = 'Функции';
    }

    function openOverlay() {
      pinBuf = '';
      showCtrlView();
      setMsg('', '');
      overlay.hidden = false;
      overlay.setAttribute('aria-hidden', 'false');
      document.body.style.overflow = 'hidden';
      if (sheet) sheet.focus();
      refreshLockStatus();
      vibrate(10);
    }

    function closeOverlay() {
      void stopDrive();
      overlay.hidden = true;
      overlay.setAttribute('aria-hidden', 'true');
      document.body.style.overflow = '';
      pinBuf = '';
      if (lockTimer) { clearInterval(lockTimer); lockTimer = null; }
      setHidden(pinView, true);
      setHidden(resetView, true);
      setHidden(themesView, true);
      setMsg('', '');
      if (openBtn) openBtn.focus();
    }

    function dismissOverlayForSteering() {
      void stopDrive();
      overlay.hidden = true;
      overlay.setAttribute('aria-hidden', 'true');
      document.body.style.overflow = '';
      pinBuf = '';
      if (lockTimer) { clearInterval(lockTimer); lockTimer = null; }
      setHidden(pinView, true);
      setHidden(ctrlView, true);
      setHidden(themesView, true);
      setHidden(resetView, true);
      if (window._showSteeringPage) window._showSteeringPage();
    }

    // Старое имя оставлено для совместимости с обработчиками: авторизация для
    // управления больше не требуется, поэтому сразу открываем экран руления.
    window._openSteeringAuth = () => {
      if (window._showSteeringPage) window._showSteeringPage();
      else openOverlay();
    };

    // ---------- lock API ----------
    async function fetchLock() {
      try {
        const r = await fetch('api/kiosk_lock', { cache: 'no-store' });
        const j = await r.json();
        if (j && j.ok && j.lock) {
          setSteeringAuthorized(!!j.steeringAuthorized);
          return j.lock;
        }
      } catch (e) {}
      return null;
    }
    async function refreshLockStatus() {
      const lock = await fetchLock();
      if (lock) {
        lockState = lock;
        renderLock(lock);
        if (lock.blocked) startBlockTimer(lock.remainingMs);
        else stopBlockTimer();
      }
    }
    function renderLock(lock) {
      lockState = lock;
      const open = !!lock.open;
      const blocked = !!lock.blocked;
      if (lockChip) {
        lockChip.textContent = open ? 'ОТСЕК ОТКРЫТ' : blocked ? 'БЛОКИРОВКА' : 'отсек закрыт';
        lockChip.className = 'ctrl-chip ' + (open ? 'open' : blocked ? 'blocked' : 'closed');
      }
      if (lockDetail) {
        if (open) lockDetail.textContent = blocked
          ? 'закрыть можно без PIN'
          : 'закрой после разгрузки';
        else if (blocked) lockDetail.textContent = 'пауза ' + Math.ceil(lock.remainingMs / 1000) + ' с';
        else lockDetail.textContent = 'готов к открытию';
      }
      if (attemptsEl) {
        if (blocked) attemptsEl.textContent = '';
        else if (lock.attemptsLeft != null && lock.attemptsLeft < 5) attemptsEl.textContent = 'попыток: ' + lock.attemptsLeft;
        else attemptsEl.textContent = '';
      }
      if (compartmentBtn && compartmentTitle && compartmentHint) {
        if (open) {
          compartmentBtn.disabled = false;
          compartmentTitle.textContent = 'Закрыть отсек';
          compartmentHint.textContent = blocked ? 'закрыть можно без PIN' : 'нажми чтобы запереть';
          compartmentBtn.classList.add('opened');
        } else if (blocked) {
          compartmentBtn.disabled = true;
          compartmentTitle.textContent = 'Заблокировано';
          compartmentHint.textContent = 'ещё ' + Math.ceil(lock.remainingMs / 1000) + ' с';
          compartmentBtn.classList.remove('opened');
        } else {
          compartmentBtn.disabled = false;
          compartmentTitle.textContent = 'Открыть отсек';
          compartmentHint.textContent = 'грузовой отсек закрыт';
          compartmentBtn.classList.remove('opened');
        }
      }
      setBusy(false);
      // если блокировка — дизаблим пад
      if (blocked) {
        pinPad.querySelectorAll('.pad-btn').forEach(b => b.disabled = true);
        if (pinSubmit) pinSubmit.disabled = true;
        setMsg('Слишком много попыток. Подожди ' + Math.ceil(lock.remainingMs/1000) + ' с', 'err');
      } else {
        pinPad.querySelectorAll('.pad-btn').forEach(b => b.disabled = busy);
        updatePinDisplay();
      }
    }
    let blockInterval = null;
    function startBlockTimer(ms) {
      stopBlockTimer();
      let rem = ms;
      blockInterval = setInterval(() => {
        rem -= 250;
        if (rem <= 0) {
          stopBlockTimer();
          refreshLockStatus();
          setMsg('Блокировка снята — вводи PIN заново', 'ok');
          pinPad.querySelectorAll('.pad-btn').forEach(b => b.disabled = false);
          updatePinDisplay();
        } else {
          const remaining = Math.ceil(rem / 1000);
          if (lockState && lockState.open) {
            if (lockChip) lockChip.textContent = 'ОТСЕК ОТКРЫТ';
            if (lockDetail) lockDetail.textContent = 'закрыть можно без PIN';
            if (compartmentTitle) compartmentTitle.textContent = 'Закрыть отсек';
            if (compartmentHint) compartmentHint.textContent = 'закрыть можно без PIN';
            if (compartmentBtn) compartmentBtn.disabled = false;
            setMsg('PIN временно заблокирован · кейс можно закрыть без PIN', 'warn');
          } else {
            if (lockChip) lockChip.textContent = 'БЛОКИРОВКА';
            if (lockDetail) lockDetail.textContent = 'пауза ' + remaining + ' с';
            if (compartmentTitle) compartmentTitle.textContent = 'Заблокировано';
            if (compartmentHint) compartmentHint.textContent = 'ещё ' + remaining + ' с';
            setMsg('Заблокировано. Ещё ' + remaining + ' с', 'err');
          }
        }
      }, 250);
    }
    function stopBlockTimer() { if (blockInterval) { clearInterval(blockInterval); blockInterval = null; } }

    // ---------- авто-сканирование ----------
    function updateAutoscanUI(on) {
      autoScanOn = !!on;
      try { localStorage.setItem('kiosk_autoscan', autoScanOn ? '1' : '0'); } catch(e) {}
      if (autoscanBtn) {
        autoscanBtn.setAttribute('aria-pressed', autoScanOn ? 'true' : 'false');
        autoscanBtn.classList.toggle('is-on', autoScanOn);
      }
      if (autoscanHint) autoscanHint.textContent = autoScanOn ? 'включено — лидар сканирует' : 'выкл — нажми чтобы включить';
      if (autoscanState) autoscanState.textContent = autoScanOn ? '●' : '○';
      if (scanBeam) scanBeam.hidden = !autoScanOn;
      if (scanLabel) scanLabel.hidden = !autoScanOn;
      if (lidarSection) lidarSection.classList.toggle('is-scanning', autoScanOn);
    }
    function toggleAutoscan() {
      updateAutoscanUI(!autoScanOn);
      vibrate(autoScanOn ? [20,30,20] : 12);
      setMsg(autoScanOn ? 'Авто-сканирование включено' : 'Авто-сканирование выключено', autoScanOn ? 'ok' : 'warn');
      setTimeout(() => { if (pinMsg && pinMsg.textContent.includes('Авто-')) setMsg('', ''); }, 1600);
    }

    // ========================= ТЕМЫ: 20 видов =========================
    const THEMES = [
      {id:'bw',       name:'Чёрно-белый',   hint:'как на скрине',      accent:'#ffffff', bg:'#070709', panel:'#141416'},
      {id:'rus',      name:'RUS SLAM',      hint:'оригинал лайм',      accent:'#9fe36b', bg:'#070d0a', panel:'#12201a'},
      {id:'lavender', name:'Lavender',      hint:'WEB TACTICS',        accent:'#c9b3ff', bg:'#0a0a14', panel:'#181826'},
      {id:'ocean',    name:'Ocean',         hint:'голубой',            accent:'#4fc3f7', bg:'#061018', panel:'#122636'},
      {id:'sunset',   name:'Sunset',        hint:'оранжевый',          accent:'#ff8c42', bg:'#120a06', panel:'#241a0f'},
      {id:'crimson',  name:'Crimson',       hint:'малиновый',          accent:'#ff4d6d', bg:'#120607', panel:'#241214'},
      {id:'nord',     name:'Nord',          hint:'скандинавский',      accent:'#88c0d0', bg:'#0b0f14', panel:'#15202e'},
      {id:'dracula',  name:'Dracula',       hint:'фиолетовый',         accent:'#bd93f9', bg:'#0e0e14', panel:'#1e1e2e'},
      {id:'light',    name:'Светлая',       hint:'инверсия',           accent:'#0a0a0a', bg:'#f4f4f6', panel:'#ffffff'},
      {id:'cyber',    name:'Cyber',         hint:'неон циан',          accent:'#00ffcc', bg:'#050507', panel:'#10181e'},
      {id:'forest',   name:'Forest',        hint:'лес',                accent:'#7ddc52', bg:'#0a120a', panel:'#142814'},
      {id:'amber',    name:'Amber',         hint:'янтарь',             accent:'#ffb84d', bg:'#120e06', panel:'#241d0f'},
      {id:'teal',     name:'Teal',          hint:'бирюза',             accent:'#2ec4b6', bg:'#061214', panel:'#0f2a26'},
      {id:'violet',   name:'Violet',        hint:'фиолет',             accent:'#a78bfa', bg:'#0f0a14', panel:'#22182e'},
      {id:'midnight', name:'Midnight',      hint:'ночной синий',       accent:'#60a5fa', bg:'#060a14', panel:'#0f1a2e'},
      {id:'sand',     name:'Sand',          hint:'песочный',           accent:'#e6c07a', bg:'#12120a', panel:'#242414'},
      {id:'graphite', name:'Graphite',      hint:'графит',             accent:'#d0d0d4', bg:'#0e0e0e', panel:'#1a1a1a'},
      {id:'mint',     name:'Mint',          hint:'мята',               accent:'#6bd39a', bg:'#0a1412', panel:'#142722'},
      {id:'peach',    name:'Peach',         hint:'персик',             accent:'#ffb088', bg:'#14100a', panel:'#2a2018'},
      {id:'ice',      name:'Ice',           hint:'лёд',                accent:'#7dd3e0', bg:'#0a1418', panel:'#14272e'},
    ];
    let currentTheme = 'bw';
    try { currentTheme = localStorage.getItem('kiosk_theme') || currentTheme; } catch(e) {}
    function applyTheme(id) {
      const t = THEMES.find(x => x.id === id);
      if (!t) return;
      currentTheme = id;
      document.body.setAttribute('data-theme', id);
      try { localStorage.setItem('kiosk_theme', id); } catch(e) {}
      // подсветка активной карточки в сетке
      if (themesGrid) {
        themesGrid.querySelectorAll('.theme-card').forEach(c => {
          c.classList.toggle('is-active', c.getAttribute('data-theme') === id);
        });
      }
    }
    function renderThemes() {
      if (!themesGrid) return;
      themesGrid.innerHTML = '';
      THEMES.forEach(th => {
        const btn = document.createElement('button');
        btn.type = 'button';
        btn.className = 'theme-card' + (th.id === currentTheme ? ' is-active' : '');
        btn.setAttribute('data-theme', th.id);
        btn.setAttribute('role', 'listitem');
        btn.setAttribute('aria-label', th.name);
        btn.innerHTML = '<div class="theme-preview" style="--preview-bg:' + th.bg + ';--preview-panel:' + th.panel + ';--preview-accent:' + th.accent + '"><span class="theme-dot"></span></div><b>' + th.name + '</b><small>' + th.hint + '</small>';
        btn.addEventListener('click', () => {
          applyTheme(th.id);
          vibrate(14);
          setMsg('Тема: ' + th.name, 'ok');
          setTimeout(() => { if (pinMsg && pinMsg.textContent.includes('Тема:')) setMsg('', ''); }, 1200);
        });
        themesGrid.appendChild(btn);
      });
    }
    function showThemes() {
      setHidden(pinView, true);
      setHidden(ctrlView, true);
      setHidden(themesView, false);
      if (subtitle) subtitle.textContent = 'выбери тему — 20 видов';
      if (titleEl) titleEl.textContent = 'Темы';
      renderThemes();
      if (themesView) themesView.scrollTop = 0;
    }
    function hideThemesToCtrl() {
      setHidden(themesView, true);
      setHidden(ctrlView, false);
      setHidden(pinView, true);
      if (subtitle) subtitle.textContent = 'доступ открыт · выбери действие';
      if (titleEl) titleEl.textContent = 'Функции';
    }
    function showReset() {
      setHidden(pinView, true);
      setHidden(ctrlView, true);
      setHidden(themesView, true);
      setHidden(resetView, false);
      if (subtitle) subtitle.textContent = 'сброс карты — подтверждение';
      if (titleEl) titleEl.textContent = 'Сброс карты';
      if (resetConfirmCheck) resetConfirmCheck.checked = false;
      if (resetConfirm) resetConfirm.disabled = true;
      if (resetMsg) { resetMsg.textContent = ''; resetMsg.className = 'reset-msg'; }
      if (resetView) resetView.scrollTop = 0;
      vibrate(10);
    }
    function hideResetToCtrl() {
      setHidden(resetView, true);
      setHidden(ctrlView, false);
      setHidden(pinView, true);
      setHidden(themesView, true);
      if (subtitle) subtitle.textContent = 'доступ открыт · выбери действие';
      if (titleEl) titleEl.textContent = 'Функции';
    }
    async function doResetMap() {
      if (!resetConfirmCheck || !resetConfirmCheck.checked) {
        if (resetMsg) { resetMsg.textContent = 'Поставь галочку подтверждения'; resetMsg.className = 'reset-msg err'; }
        vibrate([30,30,30]);
        return;
      }
      if (resetConfirm) { resetConfirm.disabled = true; resetConfirm.textContent = 'Сбрасываю…'; }
      try {
        try { await fetch('api/map/reset', { method: 'POST' }); } catch(e) {}
        try { await fetch('api/kiosk_state', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ action: 'reset_map' }) }); } catch(e) {}
        try { localStorage.removeItem('lidar_map'); localStorage.removeItem('slam_grid'); localStorage.removeItem('kiosk_lidar'); } catch(e) {}
        try {
          if (window.grid && window.grid.fill) { window.grid.fill(0); if (window.explored !== undefined) window.explored = 0; if (window.distDirty !== undefined) window.distDirty = true; }
          if (window._lidarGrid && window._lidarGrid.fill) window._lidarGrid.fill(0);
        } catch(e) {}
        const lidarCanvas = document.getElementById('lidar-canvas');
        if (lidarCanvas) {
          const ctx = lidarCanvas.getContext('2d');
          if (ctx) {
            const dpr = window.devicePixelRatio || 1;
            const w = lidarCanvas.clientWidth || 300, h = lidarCanvas.clientHeight || 300;
            if (lidarCanvas.width !== Math.round(w*dpr) || lidarCanvas.height !== Math.round(h*dpr)) {
              lidarCanvas.width = Math.round(w*dpr); lidarCanvas.height = Math.round(h*dpr);
            }
            ctx.setTransform(dpr,0,0,dpr,0,0);
            ctx.clearRect(0,0,w,h);
            ctx.fillStyle = '#101a16';
            ctx.fillRect(0,0,w,h);
            ctx.fillStyle = '#6f877d';
            ctx.font = '12px Inter, sans-serif';
            ctx.textAlign = 'center';
            ctx.fillText('карта сброшена', w/2, h/2);
            ctx.strokeStyle = '#2b4037';
            ctx.lineWidth = 1;
            ctx.beginPath(); ctx.arc(w/2, h/2, Math.min(w,h)/2 - 10, 0, Math.PI*2); ctx.stroke();
          }
        }
        const countEl = document.getElementById('lidar-count');
        const nearEl = document.getElementById('lidar-near');
        const metaEl = document.getElementById('lidar-meta');
        if (countEl) countEl.textContent = '—';
        if (nearEl) nearEl.textContent = '—';
        if (metaEl) metaEl.textContent = 'карта сброшена';
        const emptyEl = document.getElementById('lidar-empty');
        if (emptyEl) { emptyEl.textContent = 'карта сброшена — ждём новый скан'; emptyEl.hidden = false; }
        if (resetMsg) { resetMsg.textContent = 'Карта лидара сброшена'; resetMsg.className = 'reset-msg ok'; }
        vibrate([20,40,20]);
        setTimeout(() => {
          hideResetToCtrl();
          setMsg('Карта лидара сброшена', 'ok');
          setTimeout(() => { if (pinMsg && pinMsg.textContent.includes('Карта')) setMsg('', ''); }, 1800);
        }, 900);
      } catch (e) {
        if (resetMsg) { resetMsg.textContent = 'Ошибка сброса'; resetMsg.className = 'reset-msg err'; }
        vibrate([40,80,40]);
      } finally {
        if (resetConfirm) { resetConfirm.disabled = false; resetConfirm.textContent = 'Да, сбросить'; }
      }
    }
    // применить сохранённую тему сразу (и на статистике без PIN)
    applyTheme(currentTheme);

    // ---------- pad handling (сюда водить пальцами) ----------
    function pushDigit(d) {
      if (lockState && lockState.blocked) return;
      if (pinBuf.length >= 8) { shakeDots(); vibrate([30,40,30]); return; }
      pinBuf += d;
      updatePinDisplay();
      setMsg('', '');
      vibrate(12);
      // haptic + visual
      const btn = pinPad.querySelector('[data-k="' + d + '"]');
      if (btn) { btn.classList.add('is-pressed'); setTimeout(() => btn.classList.remove('is-pressed'), 120); }
    }
    function backspace() {
      if (!pinBuf.length) return;
      pinBuf = pinBuf.slice(0, -1);
      updatePinDisplay();
      vibrate(10);
    }
    function clearPin() {
      pinBuf = '';
      updatePinDisplay();
      setMsg('', '');
      vibrate(10);
    }
    function handleKey(k) {
      if (k >= '0' && k <= '9') pushDigit(k);
      else if (k === 'C' || k === 'c' || k === 'С' || k === 'с') clearPin();
      else if (k === '⌫' || k === 'Backspace') backspace();
    }

    async function submitPin() {
      if (busy) return;
      if (pinBuf.length < 4) { setMsg('PIN — 4…8 цифр', 'warn'); shakeDots(); vibrate([20,30,20]); return; }
      if (lockState && lockState.blocked) { setMsg('Подожди — ввод заблокирован', 'err'); return; }
      setBusy(true);
      setMsg('Проверяю PIN и открываю отсек…', 'warn');
      try {
        const response = await fetch('api/kiosk_lock/open', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ pin: pinBuf })
        });
        const result = await response.json();
        if (result && result.lock) renderLock(result.lock);
        else await refreshLockStatus();
        if (response.ok && result && result.ok) {
          pinBuf = '';
          updatePinDisplay();
          setMsg('', '');
          if (lockDetail) lockDetail.textContent = 'отсек открыт — забери груз';
          vibrate([20, 40, 20]);
          showCtrlView();
          return;
        }

        const reason = result && result.reason;
        if (reason === 'blocked' || (result && result.lock && result.lock.blocked)) {
          shakeDots(); vibrate([40,80,40,80,80]);
          setMsg('Заблокировано на ' + Math.ceil(((result && result.remainingMs) ||
            (lockState && lockState.remainingMs) || 30000) / 1000) + ' с', 'err');
        } else if (reason === 'wrong') {
          shakeDots(); vibrate([30,30,30]);
          setMsg('Неверный PIN. Осталось попыток: ' + (result.attemptsLeft != null ? result.attemptsLeft : '?'), 'err');
        } else if (reason === 'format') {
          shakeDots(); setMsg('PIN — только цифры, 4…8', 'warn');
        } else {
          shakeDots(); setMsg((result && result.error) || 'Не удалось открыть отсек', 'err');
        }
        setBusy(false);
        updatePinDisplay();
      } catch (error) {
        setMsg('Нет связи с роботом', 'err');
        setBusy(false);
      }
    }

    async function toggleCompartment() {
      if (busy) return;
      const open = !!(lockState && lockState.open);
      if (!open && lockState && lockState.blocked) {
        setMsg('Отсек заблокирован', 'err');
        return;
      }
      if (!open) {
        // PIN спрашиваем именно при открытии кейса, а не при входе в функции.
        showPinView();
        return;
      }
      setBusy(true);
      setMsg('Запираю…', 'warn');
      try {
        const response = await fetch('api/kiosk_lock/close', {
          method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}'
        });
        const result = await response.json();
        if (!response.ok || !result || !result.ok) throw new Error((result && result.error) || 'Не удалось закрыть отсек.');
        if (result.lock) renderLock(result.lock);
        else await refreshLockStatus();
        vibrate(20);
        if (lockDetail) lockDetail.textContent = 'отсек заперт';
        setMsg('Отсек закрыт', 'ok');
      } catch (error) {
        setMsg(error.message || 'Нет связи', 'err');
      } finally {
        setBusy(false);
      }
    }

    function mockToast(name) {
      vibrate(12);
      // показываем внизу оверлея быстро
      setMsg(name + ' — демо (в железе тут реальная команда)', 'warn');
      setTimeout(() => { if (pinMsg && pinMsg.textContent.includes('демо')) setMsg('', ''); }, 1800);
    }

    // ---------- events ----------
    openBtn.addEventListener('click', openOverlay);
    if (backdrop) backdrop.addEventListener('click', closeOverlay);
    if (closeBtn) closeBtn.addEventListener('click', closeOverlay);
    if (pinCancel) pinCancel.addEventListener('click', () => {
      pinBuf = '';
      setMsg('', '');
      showCtrlView();
    });
    if (ctrlBack) ctrlBack.addEventListener('click', closeOverlay);
    if (ctrlLogout) ctrlLogout.addEventListener('click', () => {
      // запереть если открыто и выйти
      if (lockState && lockState.open) {
        fetch('api/kiosk_lock/close', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' })
          .finally(closeOverlay);
      } else closeOverlay();
    });

    // === пад: тап + вести пальцем — сюда водить пальцами ===
    // pointerdown сразу вводит цифру (тап), а drag вводит следующие
    let pointerHandled = false;
    function handlePadAction(k) {
      if (k === 'C') clearPin();
      else if (k === '⌫') backspace();
      else if (/^[0-9]$/.test(k)) pushDigit(k);
    }
    // клик-фолбек для старых браузеров (если pointer не сработал)
    pinPad.addEventListener('click', (e) => {
      if (pointerHandled) { pointerHandled = false; return; }
      const btn = e.target.closest('.pad-btn');
      if (!btn || btn.disabled) return;
      const k = btn.getAttribute('data-k');
      handlePadAction(k);
    });

    pinPad.addEventListener('pointerdown', (e) => {
      const btn = e.target.closest('.pad-btn');
      if (!btn || btn.disabled) return;
      dragging = true;
      lastHoveredBtn = btn;
      pointerHandled = true;
      // сразу вводим цифру под пальцем — тап без движения уже работает
      handlePadAction(btn.getAttribute('data-k'));
      // визуал
      btn.classList.add('is-pressed');
      setTimeout(() => btn.classList.remove('is-pressed'), 140);
      try { pinPad.setPointerCapture(e.pointerId); } catch (ex) {}
      // флаг сбросится в click-хендлере или таймаутом, чтобы следующий тап не игнорил
      setTimeout(() => { pointerHandled = false; }, 600);
      e.preventDefault();
    });
    pinPad.addEventListener('pointermove', (e) => {
      if (!dragging) return;
      const el = document.elementFromPoint(e.clientX, e.clientY);
      const btn = el ? el.closest('.pad-btn') : null;
      if (btn && btn !== lastHoveredBtn && !btn.disabled) {
        lastHoveredBtn = btn;
        const k = btn.getAttribute('data-k');
        // чтобы не наспамить удержанием на одной кнопке — только при смене кнопки
        if (/^[0-9]$/.test(k)) {
          pushDigit(k);
          btn.classList.add('is-pressed');
          setTimeout(() => btn.classList.remove('is-pressed'), 120);
        } else if (k === '⌫') backspace();
        else if (k === 'C') clearPin();
      }
    });
    function stopDrag(e) {
      if (dragging) {
        dragging = false;
        lastHoveredBtn = null;
        try { pinPad.releasePointerCapture(e.pointerId); } catch(e2){}
      }
    }
    pinPad.addEventListener('pointerup', stopDrag);
    pinPad.addEventListener('pointercancel', stopDrag);
    pinPad.addEventListener('pointerleave', (e) => {
      // не сбрасываем сразу — вдруг вернётся
    });

    if (pinSubmit) pinSubmit.addEventListener('click', submitPin);
    if (btnSteeringControls) btnSteeringControls.addEventListener('click', dismissOverlayForSteering);
    if (compartmentBtn) compartmentBtn.addEventListener('click', toggleCompartment);
    if (autoscanBtn) autoscanBtn.addEventListener('click', toggleAutoscan);
    if (btnThemes) btnThemes.addEventListener('click', showThemes);
    if (themesBack) themesBack.addEventListener('click', hideThemesToCtrl);
    if (themesToStats) themesToStats.addEventListener('click', closeOverlay);
    if (btnResetMap) btnResetMap.addEventListener('click', showReset);
    if (resetBack) resetBack.addEventListener('click', hideResetToCtrl);
    if (resetCancel) resetCancel.addEventListener('click', hideResetToCtrl);
    if (resetConfirmCheck) resetConfirmCheck.addEventListener('change', () => {
      if (resetConfirm) resetConfirm.disabled = !resetConfirmCheck.checked;
    });
    if (resetConfirm) resetConfirm.addEventListener('click', doResetMap);
    // mock cards (остальные — демо)
    overlay.querySelectorAll('.ctrl-card[data-mock]').forEach(card => {
      card.addEventListener('click', () => mockToast(card.getAttribute('data-mock')));
    });

    // keyboard
    document.addEventListener('keydown', (e) => {
      if (overlay.hidden) {
        // быстрый вызов по цифрам? не, только когда открыто — PIN клавиатура
        if (e.key === 'Enter' && document.activeElement === openBtn) openOverlay();
        return;
      }
      // когда оверлей открыт
      if (e.key === 'Escape') {
        e.preventDefault();
        if (resetView && !resetView.hidden) { hideResetToCtrl(); return; }
        if (themesView && !themesView.hidden) { hideThemesToCtrl(); return; }
        closeOverlay(); return;
      }
      if (resetView && !resetView.hidden) {
        if (e.key === 'Enter' && resetConfirm && !resetConfirm.disabled) { e.preventDefault(); doResetMap(); }
        return;
      }
      if (themesView && !themesView.hidden) {
        // в выборе темы — Esc уже обработан, остальные клавиши не трогаем
        return;
      }
      // если в pinView
      if (!pinView.hidden) {
        if (e.key >= '0' && e.key <= '9') { e.preventDefault(); pushDigit(e.key); }
        else if (e.key === 'Backspace') { e.preventDefault(); backspace(); }
        else if (e.key.toLowerCase() === 'c' || e.key === 'Delete') { e.preventDefault(); clearPin(); }
        else if (e.key === 'Enter') { e.preventDefault(); submitPin(); }
      } else {
        // в ctrlView — Enter на отсеке
        if (e.key === 'Enter') {
          e.preventDefault();
          toggleCompartment();
        }
      }
    });

    // init state
    updatePinDisplay();
    refreshLockStatus();
    updateAutoscanUI(autoScanOn);
    // Обновляем состояние замка кейса, даже когда панель функций закрыта.
    setInterval(refreshLockStatus, 1500);
  }

  /* ============ СВАЙП-СТРАНИЦА: поворот модулей + камеры/логи ============ */
  function initSwipePage() {
    const page = $('swipe-page');
    const closeBtn = $('swipe-close');
    const cameraView = $('swipe-cameras-view');
    const motorView = $('swipe-motors-view');
    const batteryView = $('swipe-battery-view');
    const swipeTitle = $('swipe-title-text');
    const swipeTitleHint = $('swipe-title-hint');
    const kiosk = document.querySelector('.kiosk');
    const hintBar = $('swipe-hint-bar');
    const logList = $('log-list');
    const logCount = $('log-count');
    const logStatus = $('log-status');
    const logClear = $('log-clear');
    const logCopy = $('log-copy');
    const logAuto = $('log-autoscroll');
    const cam1Canvas = $('cam1-canvas');
    const cam2Canvas = $('cam2-canvas');
    const cam1Chip = $('cam1-chip');
    const cam2Chip = $('cam2-chip');
    const cam1Empty = $('cam1-empty');
    const cam2Empty = $('cam2-empty');
    if (!page || !kiosk) return;

    let isOpen = false;
    let activeSwipeView = 'cameras';
    let activeSwipeDirection = 'left';
    let camLoopsStarted = false;
    let anim1 = null, anim2 = null;

    const sheet = page.querySelector('.swipe-sheet');
    const backdrop = $('swipe-backdrop');

    function selectSwipeView(name) {
      const showMotors = name === 'motors';
      const showBattery = name === 'battery';
      if (activeSwipeView === 'motors' && !showMotors) void stopDrive();
      activeSwipeView = showMotors ? 'motors' : (showBattery ? 'battery' : 'cameras');
      activeSwipeDirection = showMotors ? 'right' : 'left';
      page.dataset.openDirection = activeSwipeDirection;
      if (cameraView) cameraView.hidden = showMotors || showBattery;
      if (motorView) motorView.hidden = !showMotors;
      if (batteryView) batteryView.hidden = !showBattery;
      if (swipeTitle) swipeTitle.textContent = showMotors ? 'Руление и тяга' : (showBattery ? '4 · Аккумулятор' : 'Камеры и логи');
      if (swipeTitleHint) swipeTitleHint.textContent = showBattery
        ? 'Все банки · свайп вправо — назад к статистике'
        : (showMotors ? 'свайп влево — назад к статистике' : 'свайп вправо — назад к статистике');
      if (!showBattery) setText($('swipe-live'), '● LIVE');
      if (isOpen && activeSwipeView === 'cameras') { startCamLoops(); fetchLogs(); }
    }

    function openSwipe(view) {
      if (isOpen) {
        if (view) selectSwipeView(view);
        return;
      }
      const overlay = $('ctrl-overlay');
      if (overlay && !overlay.hidden) return;
      isOpen = true;
      selectSwipeView(view || 'motors');
      page.hidden = false;
      if (sheet) sheet.style.transform = '';
      if (backdrop) backdrop.style.opacity = '';
      void page.offsetWidth;
      page.classList.add('is-open');
      document.body.classList.add('swipe-open');
      page.setAttribute('aria-hidden', 'false');
      if (hintBar) hintBar.style.opacity = '0';
    }
    function closeSwipe() {
      if (!isOpen) return;
      isOpen = false;
      if (activeSwipeView === 'motors') void stopDrive();
      page.classList.remove('is-open');
      document.body.classList.remove('swipe-open');
      page.setAttribute('aria-hidden', 'true');
      if (hintBar) hintBar.style.opacity = '';
      if (sheet) sheet.style.transform = '';
      if (backdrop) backdrop.style.opacity = '';
      setTimeout(() => { if (!isOpen) page.hidden = true; }, 480);
    }
    window._openSwipe = (view) => openSwipe(view || 'motors');
    window._closeSwipe = closeSwipe;
    window._showSteeringPage = () => {
      if (!isOpen) openSwipe('motors');
      else selectSwipeView('motors');
    };
    selectSwipeView('cameras');
    ['battery-open', 'swipe-battery-open'].forEach((id) => {
      const button = $(id);
      if (button) button.addEventListener('click', () => openSwipe('battery'));
    });
    // Вся панель АКБ на главном экране открывает страницу 4 — её не нужно искать.
    const batteryPanel = $('battery-panel');
    if (batteryPanel) {
      // Свайп, начатый на панели, — это жест страниц, а не нажатие.
      let panelDown = null;
      batteryPanel.addEventListener('pointerdown', (event) => {
        panelDown = { x: event.clientX, y: event.clientY };
      });
      batteryPanel.addEventListener('click', (event) => {
        const moved = panelDown && Number.isFinite(event.clientX) &&
          (Math.abs(event.clientX - panelDown.x) > 12 || Math.abs(event.clientY - panelDown.y) > 12);
        panelDown = null;
        if (!moved) openSwipe('battery');
      });
      batteryPanel.addEventListener('keydown', (event) => {
        if (event.key === 'Enter' || event.key === ' ') {
          event.preventDefault();
          openSwipe('battery');
        }
      });
    }

    if (closeBtn) closeBtn.addEventListener('click', closeSwipe);
    if (backdrop) backdrop.addEventListener('click', closeSwipe);
    // клик по листу вне? — закрывать только по бэкдропу

    // запрет выделения мышью — чтобы тяга не красила текст, кроме логов
    kiosk.addEventListener('selectstart', (e) => e.preventDefault());
    kiosk.addEventListener('dragstart', (e) => e.preventDefault());
    page.addEventListener('selectstart', (e) => {
      if (e.target.closest && e.target.closest('.log-list')) return;
      e.preventDefault();
    });
    page.addEventListener('dragstart', (e) => e.preventDefault());

    // На главном экране направления разделены: вправо — моторы, влево — камеры/логи.
    // Возврат с каждой панели — обратным свайпом.
    let tStartX = 0, tStartY = 0, tTracking = false, dragging = false, dragType = null;
    let tDragView = null;

    function directionForView(view) {
      return view === 'motors' ? 'right' : 'left';
    }
    function signedSwipeDistance(dx, direction) {
      return direction === 'right' ? dx : -dx;
    }
    function closeDirection() {
      return activeSwipeDirection === 'right' ? 'left' : 'right';
    }
    function isClosingSwipe(dx) {
      return signedSwipeDistance(dx, closeDirection()) > 0;
    }
    function swipeProgress(dx, vw, type, direction) {
      const distance = vw * 0.52;
      const signed = signedSwipeDistance(dx, type === 'open' ? direction : closeDirection());
      return Math.max(0, Math.min(1, type === 'open' ? signed / distance : 1 - signed / distance));
    }
    function setPageProgress(px, direction) {
      const progress = Math.max(0, Math.min(1, px));
      const offset = (direction === 'right' ? -1 : 1) * (1 - progress) * 105;
      if (sheet) sheet.style.transform = 'translateX(' + offset + '%)';
      if (backdrop) backdrop.style.opacity = String(progress);
      if (kiosk) {
        kiosk.style.transform = 'scale(' + (1 - progress * 0.015) + ')';
        kiosk.style.filter = 'brightness(' + (1 - progress * 0.08) + ')';
      }
    }
    function beginDrag(direction) {
      page.dataset.openDirection = direction;
      page.hidden = false;
      page.classList.add('is-open');
      if (sheet) sheet.style.transition = 'none';
      if (backdrop) backdrop.style.transition = 'none';
      if (kiosk) kiosk.style.transition = 'none';
      void page.offsetWidth;
      dragging = true;
    }
    function endDragCleanup() {
      if (sheet) { sheet.style.transition = ''; sheet.style.transform = ''; }
      if (backdrop) { backdrop.style.transition = ''; backdrop.style.opacity = ''; }
      if (kiosk) { kiosk.style.transition = ''; kiosk.style.transform = ''; kiosk.style.filter = ''; }
      dragging = false;
      dragType = null;
      tDragView = null;
      if (hintBar) hintBar.style.opacity = '';
    }
    function dragCompletes(dx, vw, type, direction) {
      const travelled = signedSwipeDistance(dx, type === 'open' ? direction : closeDirection());
      return travelled > vw * 0.18 || (Math.abs(dx) > 70 && travelled > 0);
    }
    function finishSwipe(type, view, shouldRemainOpen) {
      if (shouldRemainOpen) {
        if (type === 'open') openSwipe(view);
        else {
          page.hidden = false;
          page.classList.add('is-open');
          document.body.classList.add('swipe-open');
          page.setAttribute('aria-hidden', 'false');
        }
      } else if (type === 'close') {
        closeSwipe();
      } else {
        page.classList.remove('is-open');
        document.body.classList.remove('swipe-open');
        page.setAttribute('aria-hidden', 'true');
        setTimeout(() => { if (!isOpen) page.hidden = true; }, 480);
      }
    }

    function tStart(x, y) {
      tStartX = x; tStartY = y; tTracking = true; dragging = false; dragType = null; tDragView = null;
    }
    function tMove(x, y) {
      if (!tTracking) return;
      const dx = x - tStartX;
      const dy = y - tStartY;
      if (!dragging) {
        if (Math.abs(dx) < 10) return;
        if (Math.abs(dx) < Math.abs(dy) * 1.1) { tTracking = false; return; }
        const overlay = $('ctrl-overlay');
        if (!isOpen && overlay && !overlay.hidden) { tTracking = false; return; }
        if (isOpen && !isClosingSwipe(dx)) { tTracking = false; return; }
        dragType = isOpen ? 'close' : 'open';
        tDragView = dragType === 'open' ? (dx > 0 ? 'motors' : 'cameras') : activeSwipeView;
        if (dragType === 'open') selectSwipeView(tDragView);
        const direction = dragType === 'open' ? directionForView(tDragView) : activeSwipeDirection;
        beginDrag(direction);
        if (!isOpen) page.setAttribute('aria-hidden', 'false');
        document.body.classList.add('swipe-open');
      }
      const vw = window.innerWidth || 400;
      const direction = dragType === 'open' ? directionForView(tDragView) : activeSwipeDirection;
      const progress = swipeProgress(dx, vw, dragType, direction);
      setPageProgress(progress, direction);
      if (hintBar) hintBar.style.opacity = String(dragType === 'open' ? 1 - progress : progress);
    }
    function tEnd(x, y) {
      if (!tTracking && !dragging) return;
      const wasDragging = dragging;
      const dx = x - tStartX;
      const dy = y - tStartY;
      const vw = window.innerWidth || 400;
      tTracking = false;
      if (!wasDragging) {
        if (Math.abs(dx) < 60 || Math.abs(dx) < Math.abs(dy) * 1.1) return;
        if (!isOpen) {
          if (dx > 65) openSwipe('motors');
          else if (dx < -65) openSwipe('cameras');
        } else if (isClosingSwipe(dx) && Math.abs(dx) > 65) closeSwipe();
        return;
      }
      const type = dragType;
      const view = tDragView;
      const direction = type === 'open' ? directionForView(view) : activeSwipeDirection;
      const completed = dragCompletes(dx, vw, type, direction);
      endDragCleanup();
      finishSwipe(type, view, type === 'open' ? completed : !completed);
    }

    // Touch gestures on the base screen and on the open panel.
    kiosk.addEventListener('touchstart', (event) => {
      if (event.touches.length !== 1) return;
      tStart(event.touches[0].clientX, event.touches[0].clientY);
    }, {passive: true});
    kiosk.addEventListener('touchmove', (event) => {
      if (event.touches.length !== 1) return;
      tMove(event.touches[0].clientX, event.touches[0].clientY);
    }, {passive: true});
    kiosk.addEventListener('touchend', (event) => {
      const touch = event.changedTouches[0];
      if (touch) tEnd(touch.clientX, touch.clientY);
    }, {passive: true});
    page.addEventListener('touchstart', (event) => {
      if (event.touches.length !== 1) return;
      if (event.target.closest && event.target.closest('input, button, select, textarea')) return;
      tStart(event.touches[0].clientX, event.touches[0].clientY);
    }, {passive: true});
    page.addEventListener('touchmove', (event) => {
      if (event.touches.length !== 1) return;
      tMove(event.touches[0].clientX, event.touches[0].clientY);
    }, {passive: true});
    page.addEventListener('touchend', (event) => {
      const touch = event.changedTouches[0];
      if (touch) tEnd(touch.clientX, touch.clientY);
    }, {passive: true});

    // Mouse drag mirrors touch behavior for desktop previews.
    let mTracking = false, mStartX = 0, mStartY = 0, mDragging = false, mType = null;
    let mView = null, mOrigin = null;
    function mStart(x, y, origin) {
      mStartX = x; mStartY = y; mTracking = true; mDragging = false; mType = null; mView = null; mOrigin = origin;
    }
    kiosk.addEventListener('mousedown', (event) => {
      if (event.button !== 0) return;
      const overlay = $('ctrl-overlay');
      if (overlay && !overlay.hidden) return;
      mStart(event.clientX, event.clientY, 'kiosk');
    });
    page.addEventListener('mousedown', (event) => {
      if (event.button !== 0) return;
      if (event.target.closest && event.target.closest('input, button, select, textarea')) return;
      mStart(event.clientX, event.clientY, 'page');
    });
    window.addEventListener('mousemove', (event) => {
      if (!mTracking) return;
      const dx = event.clientX - mStartX;
      const dy = event.clientY - mStartY;
      if (!mDragging) {
        if (Math.abs(dx) < 12) return;
        if (Math.abs(dx) < Math.abs(dy) * 1.1) { mTracking = false; return; }
        const overlay = $('ctrl-overlay');
        if (!isOpen && overlay && !overlay.hidden) { mTracking = false; return; }
        if (isOpen && !isClosingSwipe(dx)) { mTracking = false; return; }
        mType = isOpen ? 'close' : 'open';
        mView = mType === 'open' ? (dx > 0 ? 'motors' : 'cameras') : activeSwipeView;
        if (mType === 'open') selectSwipeView(mView);
        const direction = mType === 'open' ? directionForView(mView) : activeSwipeDirection;
        page.dataset.openDirection = direction;
        page.hidden = false;
        page.classList.add('is-open');
        if (sheet) sheet.style.transition = 'none';
        if (backdrop) backdrop.style.transition = 'none';
        if (kiosk) kiosk.style.transition = 'none';
        void page.offsetWidth;
        mDragging = true;
        dragging = true;
        dragType = mType;
        document.body.classList.add('swipe-open');
        if (!isOpen) page.setAttribute('aria-hidden', 'false');
      }
      const vw = window.innerWidth || 400;
      const direction = mType === 'open' ? directionForView(mView) : activeSwipeDirection;
      setPageProgress(swipeProgress(dx, vw, mType, direction), direction);
      if (hintBar) hintBar.style.opacity = String(mType === 'open'
        ? 1 - swipeProgress(dx, vw, mType, direction)
        : swipeProgress(dx, vw, mType, direction));
      event.preventDefault();
    });
    window.addEventListener('mouseup', (event) => {
      if (!mTracking && !mDragging) return;
      const wasDrag = mDragging;
      const dx = event.clientX - mStartX;
      const dy = event.clientY - mStartY;
      const vw = window.innerWidth || 400;
      const type = mType;
      const view = mView;
      mTracking = false; mDragging = false; dragging = false; dragType = null;
      if (sheet) { sheet.style.transition = ''; sheet.style.transform = ''; }
      if (backdrop) { backdrop.style.transition = ''; backdrop.style.opacity = ''; }
      if (kiosk) { kiosk.style.transition = ''; kiosk.style.transform = ''; kiosk.style.filter = ''; }
      if (hintBar) hintBar.style.opacity = '';
      if (!wasDrag) {
        if (Math.abs(dx) < 80 || Math.abs(dx) < Math.abs(dy) * 1.1) { mOrigin = null; return; }
        if (mOrigin === 'kiosk' && !isOpen) openSwipe(dx > 0 ? 'motors' : 'cameras');
        else if (mOrigin === 'page' && isOpen && isClosingSwipe(dx)) closeSwipe();
        mOrigin = null;
        return;
      }
      const direction = type === 'open' ? directionForView(view) : activeSwipeDirection;
      const completed = dragCompletes(dx, vw, type, direction);
      finishSwipe(type, view, type === 'open' ? completed : !completed);
      mOrigin = null; mView = null; mType = null;
    });

    // Esc закрывает
    document.addEventListener('keydown', (e) => {
      if (e.key === 'Escape' && isOpen) { e.preventDefault(); closeSwipe(); }
    });

    // также клик по оверлею вне? не нужно

    // --- КАМЕРЫ: поддержка USB-вебок, бэкенд-стримов и демо ---
    let cam1Video = null, cam2Video = null;
    let cam1Ready = false, cam2Ready = false;
    let backendImg1 = null, backendImg2 = null;
    let startingWebcams = false;

    async function checkBackendCams() {
      try {
        const r = await fetch('api/cameras');
        if (r.ok) {
          const j = await r.json();
          if (j && Array.isArray(j.cameras)) {
            const active = j.cameras.filter((c) => c.active || c.exists);
            if (active.length > 0) {
              if (!backendImg1) {
                const img1 = new Image();
                img1.src = 'api/camera/0/stream';
                backendImg1 = img1;
              }
              if (active.length > 1 && !backendImg2) {
                const img2 = new Image();
                img2.src = 'api/camera/1/stream';
                backendImg2 = img2;
              }
            }
          }
        }
      } catch (e) {}
    }

    async function startBrowserCams() {
      if (startingWebcams || (cam1Ready && cam2Ready)) return;
      if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) return;
      startingWebcams = true;
      try {
        let devs = [];
        try {
          const all = await navigator.mediaDevices.enumerateDevices();
          devs = all.filter((d) => d.kind === 'videoinput');
        } catch (e) {}

        if (!cam1Ready) {
          try {
            const c1 = {
              video: devs.length > 0
                ? { deviceId: { ideal: devs[0].deviceId }, width: { ideal: 640 }, height: { ideal: 360 } }
                : { facingMode: 'user', width: { ideal: 640 }, height: { ideal: 360 } }
            };
            const s1 = await navigator.mediaDevices.getUserMedia(c1);
            if (!cam1Video) {
              cam1Video = document.createElement('video');
              cam1Video.autoplay = true;
              cam1Video.muted = true;
              cam1Video.playsInline = true;
              cam1Video.style.display = 'none';
              document.body.appendChild(cam1Video);
            }
            cam1Video.srcObject = s1;
            await cam1Video.play().catch(() => {});
            cam1Ready = true;
          } catch (e) {
            console.warn('Передняя вебка не запустилась:', e);
          }
        }

        if (devs.length > 1 && !cam2Ready) {
          try {
            const c2 = {
              video: { deviceId: { exact: devs[1].deviceId }, width: { ideal: 640 }, height: { ideal: 360 } }
            };
            const s2 = await navigator.mediaDevices.getUserMedia(c2);
            if (!cam2Video) {
              cam2Video = document.createElement('video');
              cam2Video.autoplay = true;
              cam2Video.muted = true;
              cam2Video.playsInline = true;
              cam2Video.style.display = 'none';
              document.body.appendChild(cam2Video);
            }
            cam2Video.srcObject = s2;
            await cam2Video.play().catch(() => {});
            cam2Ready = true;
          } catch (e) {
            console.warn('Задняя вебка не запустилась:', e);
          }
        }
      } catch (e) {
        console.warn('Ошибка доступа к вебкам:', e);
      } finally {
        startingWebcams = false;
      }
    }

    function startCamLoops() {
      if (camLoopsStarted) return;
      camLoopsStarted = true;

      checkBackendCams();
      startBrowserCams();

      const c1View = $('cam1-view');
      const c2View = $('cam2-view');
      if (c1View) c1View.addEventListener('click', () => { startBrowserCams(); checkBackendCams(); });
      if (c2View) c2View.addEventListener('click', () => { startBrowserCams(); checkBackendCams(); });

      const ctx1 = cam1Canvas ? cam1Canvas.getContext('2d') : null;
      const ctx2 = cam2Canvas ? cam2Canvas.getContext('2d') : null;

      function resize(c, ctx) {
        const dpr = window.devicePixelRatio || 1;
        const rect = c.getBoundingClientRect();
        const w = Math.max(320, Math.round(rect.width * dpr));
        const h = Math.max(180, Math.round(rect.height * dpr));
        if (c.width !== w || c.height !== h) { c.width = w; c.height = h; }
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        return {w: rect.width, h: rect.height};
      }

      let t0 = performance.now();
      function drawCam1(now) {
        if (!ctx1 || !cam1Canvas) return;
        const {w, h} = resize(cam1Canvas, ctx1);

        let src1 = null;
        if (cam1Ready && cam1Video && cam1Video.videoWidth > 0) src1 = cam1Video;
        else if (backendImg1 && backendImg1.complete && backendImg1.naturalWidth > 0) src1 = backendImg1;

        if (src1) {
          try {
            ctx1.drawImage(src1, 0, 0, w, h);
          } catch (e) {
            ctx1.fillStyle = '#0f0f0f'; ctx1.fillRect(0, 0, w, h);
          }
          if (cam1Empty) cam1Empty.hidden = true;
          if (cam1Chip) { cam1Chip.textContent = '● LIVE · передняя'; cam1Chip.className = 'cam-chip ok'; }
          const vw = src1.videoWidth || src1.naturalWidth || 640;
          const vh = src1.videoHeight || src1.naturalHeight || 360;
          if (cam1Meta) cam1Meta.textContent = `${vw}×${vh} · 30 fps`;

          // перекрестие
          ctx1.strokeStyle = 'rgba(255,255,255,.3)';
          ctx1.lineWidth = 1;
          ctx1.beginPath();
          ctx1.moveTo(w / 2 - 14, h / 2); ctx1.lineTo(w / 2 + 14, h / 2);
          ctx1.moveTo(w / 2, h / 2 - 14); ctx1.lineTo(w / 2, h / 2 + 14);
          ctx1.stroke();
          ctx1.fillStyle = 'rgba(255,255,255,.85)';
          ctx1.font = '11px ui-monospace, monospace';
          ctx1.fillText('CAM 1 · ДЕМО · ' + new Date().toLocaleTimeString('ru-RU'), 10, 18);
          anim1 = requestAnimationFrame(drawCam1);
          return;
        }

        const isDemo = new URLSearchParams(window.location.search).get('demo') === '1' || localStorage.getItem('kiosk_demo') === '1';
        if (isDemo) {
          if (cam1Empty) cam1Empty.hidden = true;
          if (cam1Chip) { cam1Chip.textContent = '○ ДЕМО · не камера'; cam1Chip.className = 'cam-chip warn'; }
          const t = (now - t0) / 1000;
          ctx1.fillStyle = '#0f0f0f';
          ctx1.fillRect(0, 0, w, h);
          const grad = ctx1.createLinearGradient(0, 0, 0, h);
          grad.addColorStop(0, '#1a1a1c');
          grad.addColorStop(0.55, '#141416');
          grad.addColorStop(1, '#0e0e0e');
          ctx1.fillStyle = grad;
          ctx1.fillRect(0, 0, w, h);
          ctx1.strokeStyle = 'rgba(255,255,255,.22)';
          ctx1.setLineDash([18, 18]);
          ctx1.lineWidth = 3;
          ctx1.beginPath();
          ctx1.moveTo(w / 2, h * 0.35); ctx1.lineTo(w / 2, h);
          ctx1.stroke();
          ctx1.setLineDash([]);
          ctx1.strokeStyle = '#2a2a2c';
          ctx1.lineWidth = 2;
          ctx1.beginPath();
          ctx1.moveTo(w * 0.18, h * 0.35); ctx1.lineTo(0, h);
          ctx1.moveTo(w * 0.82, h * 0.35); ctx1.lineTo(w, h);
          ctx1.stroke();
          const x = w / 2 + Math.sin(t * 0.7) * w * 0.18;
          const y = h * 0.52 + Math.sin(t * 0.9) * 6;
          ctx1.fillStyle = '#ffffff';
          ctx1.globalAlpha = 0.95;
          ctx1.fillRect(x - 36, y - 18, 72, 36);
          ctx1.globalAlpha = 1;
          ctx1.fillStyle = 'rgba(255,255,255,.08)';
          ctx1.fillRect(0, h - 48, w, 48);
          ctx1.strokeStyle = 'rgba(255,255,255,.55)';
          ctx1.lineWidth = 1;
          ctx1.beginPath();
          ctx1.moveTo(w / 2 - 14, h / 2); ctx1.lineTo(w / 2 + 14, h / 2);
          ctx1.moveTo(w / 2, h / 2 - 14); ctx1.lineTo(w / 2, h / 2 + 14);
          ctx1.stroke();
          ctx1.fillStyle = 'rgba(255,255,255,.75)';
          ctx1.font = '11px ui-monospace, monospace';
          ctx1.fillText('CAM 1 · 30fps · ' + new Date().toLocaleTimeString('ru-RU'), 10, 18);
          anim1 = requestAnimationFrame(drawCam1);
          return;
        }

        // Режим ожидания вебки
        ctx1.fillStyle = '#0b1411';
        ctx1.fillRect(0, 0, w, h);
        ctx1.fillStyle = '#6f877d';
        ctx1.font = '12px Inter, sans-serif';
        ctx1.textAlign = 'center';
        ctx1.fillText('кликните для включения вебки', w / 2, h / 2);
        ctx1.strokeStyle = 'rgba(255,255,255,.06)';
        ctx1.lineWidth = 1;
        ctx1.beginPath();
        ctx1.moveTo(w * 0.5, h * 0.22); ctx1.lineTo(w * 0.5, h * 0.78);
        ctx1.moveTo(w * 0.22, h * 0.5); ctx1.lineTo(w * 0.78, h * 0.5);
        ctx1.stroke();
        if (cam1Empty) cam1Empty.hidden = false;
        if (cam1Chip) { cam1Chip.textContent = 'нет сигнала'; cam1Chip.className = 'cam-chip'; }
        anim1 = requestAnimationFrame(drawCam1);
      }

      function drawCam2(now) {
        if (!ctx2 || !cam2Canvas) return;
        const {w, h} = resize(cam2Canvas, ctx2);

        let src2 = null;
        if (cam2Ready && cam2Video && cam2Video.videoWidth > 0) src2 = cam2Video;
        else if (backendImg2 && backendImg2.complete && backendImg2.naturalWidth > 0) src2 = backendImg2;

        if (src2) {
          try {
            ctx2.drawImage(src2, 0, 0, w, h);
          } catch (e) {
            ctx2.fillStyle = '#0f0f0f'; ctx2.fillRect(0, 0, w, h);
          }
          if (cam2Empty) cam2Empty.hidden = true;
          if (cam2Chip) { cam2Chip.textContent = '● LIVE · кормовая'; cam2Chip.className = 'cam-chip ok'; }
          const vw = src2.videoWidth || src2.naturalWidth || 640;
          const vh = src2.videoHeight || src2.naturalHeight || 360;
          if (cam2Meta) cam2Meta.textContent = `${vw}×${vh} · 30 fps`;

          ctx2.strokeStyle = 'rgba(255,255,255,.3)';
          ctx2.lineWidth = 1;
          ctx2.beginPath();
          ctx2.moveTo(w / 2 - 14, h / 2); ctx2.lineTo(w / 2 + 14, h / 2);
          ctx2.moveTo(w / 2, h / 2 - 14); ctx2.lineTo(w / 2, h / 2 + 14);
          ctx2.stroke();
          ctx2.fillStyle = 'rgba(255,255,255,.85)';
          ctx2.font = '11px ui-monospace, monospace';
          ctx2.fillText('CAM 2 · ДЕМО · ' + new Date().toLocaleTimeString('ru-RU'), 10, 18);
          anim2 = requestAnimationFrame(drawCam2);
          return;
        }

        const isDemo = new URLSearchParams(window.location.search).get('demo') === '1' || localStorage.getItem('kiosk_demo') === '1';
        if (isDemo) {
          if (cam2Empty) cam2Empty.hidden = true;
          if (cam2Chip) { cam2Chip.textContent = '○ ДЕМО · не камера'; cam2Chip.className = 'cam-chip warn'; }
          const t = (now - t0) / 1000;
          ctx2.fillStyle = '#0a0a0a';
          ctx2.fillRect(0, 0, w, h);
          const grad = ctx2.createLinearGradient(0, 0, 0, h);
          grad.addColorStop(0, '#141416');
          grad.addColorStop(1, '#0a0a0a');
          ctx2.fillStyle = grad;
          ctx2.fillRect(0, 0, w, h);
          ctx2.strokeStyle = '#222226';
          ctx2.lineWidth = 1;
          for (let i = 0; i < w; i += 40) { ctx2.beginPath(); ctx2.moveTo(i + (t * 18) % 40, 0); ctx2.lineTo(i + (t * 18) % 40, h); ctx2.stroke(); }
          for (let j = 0; j < h; j += 36) { ctx2.beginPath(); ctx2.moveTo(0, j); ctx2.lineTo(w, j); ctx2.stroke(); }
          const sx = w * 0.5 + Math.cos(t * 0.5) * w * 0.12;
          ctx2.fillStyle = '#ffffff';
          ctx2.fillRect(sx - 42, h * 0.35, 84, h * 0.45);
          ctx2.fillStyle = '#1a1a1c';
          ctx2.fillRect(sx - 30, h * 0.45, 60, 8);
          ctx2.fillRect(sx - 30, h * 0.62, 60, 8);
          const vg = ctx2.createRadialGradient(w / 2, h / 2, w * 0.35, w / 2, h / 2, w * 0.85);
          vg.addColorStop(0, 'transparent');
          vg.addColorStop(1, 'rgba(0,0,0,.55)');
          ctx2.fillStyle = vg;
          ctx2.fillRect(0, 0, w, h);
          ctx2.fillStyle = 'rgba(255,255,255,.75)';
          ctx2.font = '11px ui-monospace, monospace';
          ctx2.fillText('CAM 2 · 30fps · ' + new Date().toLocaleTimeString('ru-RU'), 10, 18);
          anim2 = requestAnimationFrame(drawCam2);
          return;
        }

        // Режим ожидания
        ctx2.fillStyle = '#0b1411';
        ctx2.fillRect(0, 0, w, h);
        ctx2.fillStyle = '#6f877d';
        ctx2.font = '12px Inter, sans-serif';
        ctx2.textAlign = 'center';
        ctx2.fillText('кликните для включения вебки', w / 2, h / 2);
        ctx2.strokeStyle = 'rgba(255,255,255,.06)';
        ctx2.lineWidth = 1;
        ctx2.beginPath();
        ctx2.moveTo(w * 0.5, h * 0.22); ctx2.lineTo(w * 0.5, h * 0.78);
        ctx2.moveTo(w * 0.22, h * 0.5); ctx2.lineTo(w * 0.78, h * 0.5);
        ctx2.stroke();
        if (cam2Empty) cam2Empty.hidden = false;
        if (cam2Chip) { cam2Chip.textContent = 'нет сигнала'; cam2Chip.className = 'cam-chip'; }
        anim2 = requestAnimationFrame(drawCam2);
      }
      if (ctx1) requestAnimationFrame(drawCam1);
      if (ctx2) requestAnimationFrame(drawCam2);
      window.addEventListener('resize', () => {
        if (cam1Canvas) { const r = cam1Canvas.getBoundingClientRect(); cam1Canvas.width = Math.round(r.width*(window.devicePixelRatio||1)); }
        if (cam2Canvas) { const r = cam2Canvas.getBoundingClientRect(); cam2Canvas.width = Math.round(r.width*(window.devicePixelRatio||1)); }
      });
    }

    // --- ЛОГИ: настоящий журнал робота (/api/kiosk_logs) ---
    // Панель показывает события этого процесса: подключение и потеря лидара,
    // ошибки портов, проверки PIN, команды руления и тяги. Выдуманных строк
    // здесь нет: если событий не было, панель так и пишет. Демо-поток для
    // проверки экрана включается только явно — «?demo=1» в адресе или
    // localStorage.kiosk_demo = '1' — и подписывается как демо.
    const KNOWN_LEVELS = ['info', 'ok', 'warn', 'err', 'debug'];
    const LOG_STATUS_BASE = 'журнал робота · /api/kiosk_logs';
    const demoLogsOn = new URLSearchParams(window.location.search).get('demo') === '1'
      || localStorage.getItem('kiosk_demo') === '1';
    let logs = [];
    let logIndex = new Map();

    function clockLabel(entry) {
      const label = entry && entry.time;
      if (typeof label === 'string' && label) return label;
      const ts = Number(entry && entry.ts);
      const d = Number.isFinite(ts) ? new Date(ts * 1000) : new Date();
      return String(d.getHours()).padStart(2,'0') + ':' + String(d.getMinutes()).padStart(2,'0')
        + ':' + String(d.getSeconds()).padStart(2,'0');
    }

    function setLogStatus(text) {
      if (logStatus) logStatus.textContent = text;
    }

    // Строка, рождённая самим экраном (очистка списка, копирование). Это тоже
    // настоящее действие оператора, поэтому подписываем его «экран:».
    function pushLocalLog(level, msg) {
      const now = new Date();
      const ts = String(now.getHours()).padStart(2,'0') + ':' + String(now.getMinutes()).padStart(2,'0')
        + ':' + String(now.getSeconds()).padStart(2,'0');
      logs.push({key: null, ts, level, msg, repeat: 1, local: true});
      if (logs.length > 220) logs.shift();
      renderLogs();
    }

    // Запись журнала с сервера. Повторяющиеся события (лидар, удержание тяги)
    // сервер сворачивает в счётчик «×N» — обновляем ту же строку, а не плодим.
    function addJournalEntry(entry) {
      if (!entry || typeof entry !== 'object') return;
      const seq = Number(entry.seq);
      const msg = String(entry.msg || '');
      if (!Number.isFinite(seq) || !msg) return;
      const key = seq + '|' + String(entry.ts || '') + '|' + msg;
      const known = logIndex.get(key);
      if (known) {
        known.repeat = Number(entry.repeat) > 0 ? Number(entry.repeat) : known.repeat;
        known.ts = clockLabel(entry);
        return;
      }
      const level = KNOWN_LEVELS.indexOf(String(entry.level)) >= 0 ? String(entry.level) : 'info';
      const row = {key, ts: clockLabel(entry), level, msg,
                   repeat: Number(entry.repeat) > 0 ? Number(entry.repeat) : 1,
                   source: String(entry.source || ''), local: false};
      logIndex.set(key, row);
      logs.push(row);
      if (logs.length > 220) {
        const dropped = logs.shift();
        if (dropped && dropped.key) logIndex.delete(dropped.key);
      }
    }

    function renderLogs() {
      if (!logList) return;
      const frag = document.createDocumentFragment();
      logList.innerHTML = '';
      const slice = logs.slice(-160);
      slice.forEach((l) => {
        const row = document.createElement('div');
        row.className = 'log-line';
        row.setAttribute('data-level', l.level);
        if (l.local) row.setAttribute('data-local', '1');
        const ts = document.createElement('span'); ts.className='log-ts'; ts.textContent = l.ts;
        const lvl = document.createElement('span'); lvl.className='log-lvl '+l.level; lvl.textContent = l.level;
        const msg = document.createElement('span'); msg.className='log-msg';
        msg.textContent = l.repeat > 1 ? l.msg + '  ×' + l.repeat : l.msg;
        row.appendChild(ts); row.appendChild(lvl); row.appendChild(msg);
        if (l.source) row.title = 'источник: ' + l.source;
        frag.appendChild(row);
      });
      logList.appendChild(frag);
      if (logCount) logCount.textContent = logs.length + '';
      if (logAuto && logAuto.checked) logList.scrollTop = logList.scrollHeight;
    }

    async function fetchLogs() {
      let data = null;
      let problem = '';
      try {
        const r = await fetch('api/kiosk_logs?limit=120', {cache:'no-store'});
        if (r.status === 401 || r.status === 403) {
          setLogStatus('журнал робота · нужен вход оператора (8090 — без входа)');
          return;
        }
        if (!r.ok) {
          setLogStatus('журнал робота · экран отвечает ' + r.status);
          return;
        }
        // На пульте (8080) журнал закрыт входом: вместо JSON приезжает
        // страница входа — так и говорим, а не «нет связи».
        const j = await r.json().catch(() => null);
        if (!j) {
          setLogStatus('журнал робота · ответ не JSON — на пульте нужен вход оператора');
          return;
        }
        data = (j.ok && j.data) ? j.data : null;
      } catch (e) {
        problem = 'нет связи с /api/kiosk_logs';
      }
      if (problem) {
        setLogStatus('журнал робота · ' + problem);
        return;
      }
      if (!data || !Array.isArray(data.entries)) {
        setLogStatus('журнал робота · ответ без записей');
        return;
      }
      data.entries.forEach(addJournalEntry);
      renderLogs();
      describeJournal(data);
    }

    // Русские окончания: «1 запись · 2 записи · 5 записей».
    function ruCount(n, forms) {
      const abs = Math.abs(Number(n) || 0) % 100;
      const last = abs % 10;
      if (abs > 10 && abs < 20) return forms[2];
      if (last === 1) return forms[0];
      if (last >= 2 && last <= 4) return forms[1];
      return forms[2];
    }

    // Подпись под журналом: сколько настоящих записей, сколько ошибок, откуда
    // данные и не потерялись ли старые строки из-за размера буфера.
    function describeJournal(data) {
      const levels = data.levels || {};
      const errors = Number(levels.err) || 0;
      const count = Number(data.count) || 0;
      if (!count && !logs.length) {
        setLogStatus(LOG_STATUS_BASE + ' · событий ещё не было');
        return;
      }
      const parts = ['журнал робота',
                     count + ' ' + ruCount(count, ['запись', 'записи', 'записей'])];
      if (errors) parts.push(errors + ' ' + ruCount(errors, ['ошибка', 'ошибки', 'ошибок']));
      if (Number(data.dropped)) parts.push('вытеснено ' + data.dropped);
      if (data.source) parts.push('источник: ' + data.source);
      if (demoLogsOn) parts.push('демо-строки включены');
      setLogStatus(parts.join(' · '));
    }

    // Демо-поток: только по явному запросу и всегда с пометкой «демо».
    const demoMsgs = [
      ['info','SLAM: скан лидара 360° — 412 точек'],
      ['info','мотор FL: 12.4° · 42 об/мин'],
      ['info','АКБ: 77.8% · 39.2В · 1.1А · запас 17.3 км'],
      ['ok','связь с модулями: ок (4/4)'],
      ['warn','лидар: ближайшая точка 0.84 м — замедление'],
      ['info','камера 1: кадр захвачен 1920×1080'],
    ];
    if (demoLogsOn) {
      pushLocalLog('warn','демо-режим: строки ниже выдуманы для проверки экрана');
      setInterval(() => {
        const pick = demoMsgs[Math.floor(Math.random()*demoMsgs.length)];
        pushLocalLog(pick[0], 'демо: ' + pick[1]);
      }, 1400);
    }
    setInterval(fetchLogs, 2500);
    fetchLogs();

    if (logClear) logClear.addEventListener('click', () => {
      logs = []; logIndex = new Map(); renderLogs();
      pushLocalLog('info','экран: список очищен, журнал на роботе не тронут');
    });
    if (logCopy) logCopy.addEventListener('click', async () => {
      const txt = logs.map(l => l.ts+' ['+l.level.toUpperCase()+'] '+l.msg).join('\n');
      try { await navigator.clipboard.writeText(txt); pushLocalLog('ok','экран: список скопирован в буфер'); }
      catch(e) { pushLocalLog('warn','экран: не удалось скопировать — выдели вручную'); }
    });

    // также свайп индикатор — при первом посещении подмигнуть
    setTimeout(() => {
      if (!isOpen && hintBar) {
        try { hintBar.animate([{transform:'translateX(-50%) scale(1)'},{transform:'translateX(-50%) scale(1.04)'},{transform:'translateX(-50%) scale(1)'}], {duration: 900, iterations: 1}); } catch(e) {}
      }
    }, 1800);
  }

  /* ============ Руление и безопасная тяга BLDC ============ */
  function renderSteeringControls(data) {
    const panel = $('swipe-motors-view');
    if (!panel) return;
    const d = data || {};
    const control = d.steeringControl || {};
    const available = control.available === true;
    const source = String(control.source || d.source || 'unknown');
    const motors = Array.isArray(d.motors) ? d.motors : [];
    const drive = d.drive || {};
    const tractionActive = !!drive.active || Number(drive.commandedPercent || 0) !== 0;
    const sourceStatus = $('steer-source-status');
    const authStatus = $('steer-auth-status');

    if (sourceStatus) {
      if (!available && String(control.source || '') === 'master-peer') {
        sourceStatus.textContent = 'Руление и тяга — только на бортовом экране (порт 8090). Здесь показана телеметрия.';
      } else if (!available) {
        sourceStatus.textContent = 'Ручной поворот доступен только при RC_SLAM_SOURCE=master и подключённом UART мастера.';
      } else if (d.linkOk === false) {
        sourceStatus.textContent = 'Мастер выбран, но свежей связи с рулевыми модулями пока нет.';
      } else if (control.calibration) {
        sourceStatus.textContent = 'Мастер подключён · ноль руля — «Калибровка» по оптодатчику D18; «0° вручную» — если датчика нет.';
      } else {
        sourceStatus.textContent = 'Мастер подключён · перед поворотом задайте ручной программный ноль каждому модулю.';
      }
    }
    if (authStatus) authStatus.textContent = steeringAuthorized ? 'доступ без PIN' : 'управление недоступно';

    const now = Date.now();
    ['FL', 'FR', 'RL', 'RR'].forEach((id) => {
      const card = panel.querySelector('[data-steer-card="' + id + '"]');
      const slider = $('steer-slider-' + id);
      const currentEl = $('steer-current-' + id);
      const targetEl = $('steer-target-' + id);
      const stateEl = $('steer-state-' + id);
      const sendButton = panel.querySelector('[data-steer-send="' + id + '"]');
      const originButton = panel.querySelector('[data-steer-origin="' + id + '"]');
      const calibButton = panel.querySelector('[data-steer-calib="' + id + '"]');
      if (!card || !slider) return;

      const motor = motors.find((m) => m && m.id === id) || null;
      const current = motor && motor.angle != null && Number.isFinite(+motor.angle) ? +motor.angle : null;
      const target = motor && motor.targetAngle != null && Number.isFinite(+motor.targetAngle) ? +motor.targetAngle : null;
      const online = !!(motor && motor.online === true && current != null);
      const homed = !!(motor && motor.homed === true);
      const moving = !!(motor && motor.moving);
      const calibrating = !!(motor && motor.calibrating);
      const busy = moving || calibrating;
      let pending = pendingSteer[id] || null;
      let zeroPending = pendingZero[id] || null;
      let calibPending = pendingCalib[id] || null;
      const homeSource = motor && motor.homeSource ? String(motor.homeSource) : (homed ? 'manual' : 'none');
      const calibError = motor && Number(motor.calibError) > 0 ? Number(motor.calibError) : 0;
      const calibErrorText = (motor && motor.calibErrorText) || (calibError ? 'код ' + calibError : '');

      if (pending && current != null && !busy && Math.abs(current - pending.angle) <= 1) {
        delete pendingSteer[id];
        pending = null;
      } else if (pending && !busy && now - pending.sentAt > 15000) {
        delete pendingSteer[id];
        pending = null;
      }
      if (zeroPending && homed) {
        delete pendingZero[id];
        zeroPending = null;
      } else if (zeroPending && now - zeroPending.sentAt > 15000) {
        delete pendingZero[id];
        zeroPending = null;
      }
      // Калибровка: ждём, что модуль начал её (calibrating), и показываем итог.
      if (calibPending && calibrating) {
        calibPending.started = true;
      } else if (calibPending && calibPending.started && !calibrating) {
        delete pendingCalib[id];
        calibPending = null;
        steerResult[id] = homed && homeSource === 'opto'
          ? { text: 'ноль найден по датчику', error: false, until: now + 8000 }
          : { text: 'ошибка калибровки: ' + (calibErrorText || 'ноль не найден'), error: true, until: now + 15000 };
      } else if (calibPending && !calibPending.started && now - calibPending.sentAt > 8000) {
        delete pendingCalib[id];
        calibPending = null;
        steerResult[id] = { text: 'калибровка не началась · проверьте прошивку v2', error: true, until: now + 8000 };
      }

      let value = pending ? pending.angle : (moving && target != null ? target : (current == null ? 0 : current));
      value = Math.max(-85, Math.min(85, Math.round(value)));
      const dirty = card.dataset.steerDirty === 'true';
      if (!dirty) slider.value = String(value);
      if (currentEl) currentEl.textContent = current == null ? '—' : Math.round(current) + '°';
      // Ползунок ограничен ±85°, а при калибровке цель — до ±200°: показываем настоящую.
      if (targetEl) targetEl.textContent = (!dirty && busy && target != null ? Math.round(target) : Math.round(+slider.value || 0)) + '°';

      const ready = available && steeringAuthorized && online && homed && !busy && !pending && !zeroPending && !calibPending && !tractionActive;
      const canSetZero = available && steeringAuthorized && online && !busy && !pending && !zeroPending && !calibPending && !tractionActive;
      const canCalibrate = canSetZero && !!control.calibration;
      slider.disabled = !ready;
      card.querySelectorAll('[data-steer-adjust="' + id + '"], [data-steer-zero="' + id + '"]').forEach((button) => {
        button.disabled = !ready;
      });
      if (originButton) originButton.disabled = !canSetZero;
      if (calibButton) {
        calibButton.disabled = !canCalibrate;
        calibButton.hidden = !control.calibration && available;
      }
      const homeEl = $('steer-home-' + id);
      if (homeEl) {
        homeEl.dataset.home = !online ? 'none' : (calibError && !homed ? 'error' : homeSource);
        homeEl.textContent = !online ? 'ноль: —'
          : homeSource === 'opto' && homed ? 'ноль: по датчику'
          : homeSource === 'manual' && homed ? 'ноль: вручную'
          : calibError ? 'ноль: нет · ' + calibErrorText
          : 'ноль: не задан';
        homeEl.title = homeEl.textContent;
      }
      const optoEl = $('steer-opto-' + id);
      if (optoEl) {
        const opto = online && motor && typeof motor.opto === 'boolean' ? motor.opto : null;
        optoEl.dataset.opto = opto === null ? 'unknown' : (opto ? 'on' : 'off');
        optoEl.textContent = opto === null ? 'датчик: —' : (opto ? 'датчик: закрыт' : 'датчик: свободен');
      }
      if (sendButton) sendButton.disabled = !ready || !dirty;
      card.dataset.ready = String(ready);

      let label;
      if (!available) label = 'нет управления';
      else if (!steeringAuthorized) label = 'управление недоступно';
      else if (!online) label = 'нет связи';
      else if (calibrating) label = 'калибровка по датчику…';
      else if (calibPending) label = 'калибровка в очереди';
      else if (moving) label = 'поворачивается';
      else if (tractionActive) label = 'тяга включена · сначала стоп';
      else if (zeroPending) label = 'ноль в очереди';
      else if (pending) label = 'команда отправлена';
      else if (!homed && calibError) label = 'ошибка калибровки';
      else if (!homed) label = control.calibration ? 'нужна калибровка' : 'нужен ручной ноль';
      else label = homeSource === 'opto' ? 'готов · ноль по датчику' : 'готов · ноль вручную';

      const message = steerResult[id];
      if (message && message.until > now) {
        label = message.text;
        card.dataset.error = String(!!message.error);
      } else {
        delete steerResult[id];
        card.dataset.error = 'false';
      }
      if (stateEl) stateEl.textContent = label;
    });

    const calibAll = $('steer-calib-all');
    if (calibAll) {
      const online = motors.filter((m) => m && m.online === true);
      const anyBusy = online.some((m) => m.moving || m.calibrating || m.cycling);
      const anyPending = Object.keys(pendingCalib).length > 0;
      calibAll.disabled = !(available && control.calibration && steeringAuthorized && online.length > 0 &&
        !anyBusy && !anyPending && !tractionActive);
      calibAll.textContent = online.some((m) => m.calibrating) ? 'Калибровка идёт…' : 'Калибровать все колёса';
    }
    renderMasterMessages(d.masterMessages);
    renderDriveControls(d);
  }

  function renderMasterMessages(list) {
    const box = $('steer-messages');
    if (!box) return;
    const items = Array.isArray(list) ? list.slice(-4) : [];
    box.hidden = items.length === 0;
    const html = items.map((item) => {
      const text = String((item && item.text) || '');
      const warn = /ОШИБКА|нет связи|не доставлена|нет команды|отклонена|заблокирован/.test(text);
      const time = item && item.ts ? clockText(new Date(item.ts * 1000)) : '';
      return '<li' + (warn ? ' data-warn="true"' : '') + '><time>' + escapeHtml(time) + '</time> ' + escapeHtml(text) + '</li>';
    }).join('');
    if (box.dataset.html !== html) {
      box.innerHTML = html;
      box.dataset.html = html;
    }
  }

  function escapeHtml(value) {
    return String(value == null ? '' : value).replace(/[&<>"']/g, (c) => ({
      '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
    }[c]));
  }

  const DRIVE_REASON_TEXT = {
    serial_unavailable: 'Нет связи с USB-мастером.',
    module_offline: 'Нет свежей телеметрии всех четырёх модулей.',
    drive_firmware_required: 'Обновите прошивку всех четырёх плат для тяги и watchdog.',
    not_homed: 'Сначала откалибруйте колёса по датчику (или задайте 0° вручную).',
    module_busy: 'Дождитесь остановки рулевых модулей.',
    steering_misaligned: 'Колёса должны смотреть в одном направлении (±2,5°).',
    control_not_supported: 'Тяга доступна только в режиме master.',
    command_queue_full: 'Очередь команд занята.',
    pin_required: 'PIN истёк; плата остановится по watchdog.'
  };

  function renderDriveControls(data) {
    const status = $('drive-status');
    const speed = $('drive-speed');
    if (!status || !speed) return;
    const d = data || {};
    const drive = d.drive || {};
    const supported = !!(d.steeringControl && d.steeringControl.available);
    const max = Math.max(1, Math.min(20, Number(drive.maxPercent) || 20));
    speed.max = String(max);
    if (Number(speed.value) > max) speed.value = String(max);
    setText($('drive-speed-value'), String(speed.value));
    const enabled = supported && steeringAuthorized && !!drive.available;
    speed.disabled = !enabled;
    const back = $('drive-back');
    const forward = $('drive-forward');
    const stop = $('drive-stop');
    if (back) back.disabled = !enabled || !drive.canDrive;
    if (forward) forward.disabled = !enabled || !drive.canDrive;
    if (stop) stop.disabled = !enabled;
    const reason = drive.reason && DRIVE_REASON_TEXT[drive.reason];
    if (!supported && String((d.steeringControl || {}).source || '') === 'master-peer') status.textContent = 'Тяга — только на бортовом экране (порт 8090).';
    else if (!supported) status.textContent = 'Тяга доступна при RC_SLAM_SOURCE=master.';
    else if (!steeringAuthorized) status.textContent = 'Управление временно недоступно.';
    else if (reason) status.textContent = reason;
    else if (drive.active) status.textContent = 'Движение включено · отпустите кнопку для торможения.';
    else status.textContent = 'Готово · модули на связи, колёса выровнены.';
    status.dataset.ready = String(!!drive.canDrive && enabled);
  }

  function startDrive(direction) {
    if (!steeringAuthorized) {
      if (window._openSteeringAuth) window._openSteeringAuth();
      return;
    }
    const drive = (lastData && lastData.drive) || {};
    if (!drive.canDrive) {
      renderDriveControls(lastData || {});
      return;
    }
    driveDirection = direction;
    if (driveTimer) clearInterval(driveTimer);
    const heartbeat = () => {
      if (driveDirection) void queueDrive(driveDirection * Number(($('drive-speed') || {}).value || 8));
    };
    heartbeat();
    driveTimer = setInterval(heartbeat, 200);
  }

  function releaseDrive() {
    const wasDriving = driveDirection !== 0 || driveTimer !== null;
    driveDirection = 0;
    if (driveTimer) clearInterval(driveTimer);
    driveTimer = null;
    if (wasDriving && steeringAuthorized) void queueDrive(0);
  }

  async function stopDrive() {
    driveDirection = 0;
    if (driveTimer) clearInterval(driveTimer);
    driveTimer = null;
    if (!steeringAuthorized) {
      setText($('drive-status'), 'PIN истёк — аппаратный watchdog остановит моторы.');
      return;
    }
    await queueDrive(0);
    setText($('drive-status'), 'Стоп отправлен · PWM 0 · тормоз включён.');
  }

  function queueDrive(percent) {
    driveQueued = percent;
    if (driveSending) return driveFlush;
    driveSending = true;
    driveFlush = (async () => {
      while (driveQueued !== null) {
        const target = driveQueued;
        driveQueued = null;
        const controller = new AbortController();
        const timeout = setTimeout(() => controller.abort(), 1200);
        try {
          const response = await fetch('api/kiosk_drive', {
            method: 'POST', cache: 'no-store', signal: controller.signal,
            headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'drive' },
            body: JSON.stringify({ percent: target })
          });
          const body = await response.json().catch(() => ({}));
          if (!response.ok || !body.ok) {
            const reason = body.reason || (response.status === 401 ? 'pin_required' : 'command_failed');
            setText($('drive-status'), DRIVE_REASON_TEXT[reason] || 'Команда тяги отклонена.');
            if (response.status === 401 || reason === 'pin_required') steeringAuthorized = false;
            if (target !== 0) {
              driveDirection = 0;
              if (driveTimer) clearInterval(driveTimer);
              driveTimer = null;
              driveQueued = 0;
            }
          } else if (target === 0) {
            setText($('drive-status'), 'PWM 0 · тормоз включён.');
          } else {
            setText($('drive-status'), 'Тяга ' + body.percent + '% · удерживайте кнопку.');
          }
        } catch (error) {
          setText($('drive-status'), 'Связь потеряна · watchdog остановит моторы не позднее 650 мс.');
          if (target !== 0) {
            driveDirection = 0;
            if (driveTimer) clearInterval(driveTimer);
            driveTimer = null;
            driveQueued = 0;
          }
        } finally {
          clearTimeout(timeout);
        }
      }
    })().finally(() => { driveSending = false; });
    return driveFlush;
  }

  function initSteeringControls() {
    const panel = $('swipe-motors-view');
    if (!panel) return;
    const ids = ['FL', 'FR', 'RL', 'RR'];
    const driveSpeed = $('drive-speed');
    if (driveSpeed) driveSpeed.addEventListener('input', () => {
      setText($('drive-speed-value'), String(driveSpeed.value));
      if (driveDirection) void queueDrive(driveDirection * Number(driveSpeed.value));
    });
    const bindHoldButton = (id, direction) => {
      const button = $(id);
      if (!button) return;
      button.addEventListener('pointerdown', (event) => {
        event.preventDefault();
        startDrive(direction);
        try { button.setPointerCapture(event.pointerId); } catch (error) {}
      });
      button.addEventListener('click', (event) => event.preventDefault());
      button.addEventListener('contextmenu', (event) => event.preventDefault());
    };
    bindHoldButton('drive-back', -1);
    bindHoldButton('drive-forward', 1);
    const driveStopButton = $('drive-stop');
    if (driveStopButton) driveStopButton.addEventListener('click', () => { void stopDrive(); });
    window.addEventListener('pointerup', releaseDrive);
    window.addEventListener('pointercancel', releaseDrive);
    window.addEventListener('blur', releaseDrive);
    document.addEventListener('visibilitychange', () => { if (document.hidden) releaseDrive(); });

    // Не даём горизонтальному движению по слайдеру закрыть swipe-панель.
    ['pointerdown', 'pointermove', 'pointerup', 'mousedown', 'mouseup'].forEach((name) => {
      panel.addEventListener(name, (event) => {
        if (event.target.closest && event.target.closest('input, button, select, textarea')) event.stopPropagation();
      });
    });
    ['touchstart', 'touchmove', 'touchend'].forEach((name) => {
      panel.addEventListener(name, (event) => {
        if (event.target.closest && event.target.closest('input, button, select, textarea')) event.stopPropagation();
      }, {passive: true});
    });

    function markDirty(id, value) {
      const card = panel.querySelector('[data-steer-card="' + id + '"]');
      const slider = $('steer-slider-' + id);
      const output = $('steer-target-' + id);
      if (!card || !slider) return;
      slider.value = String(Math.max(-85, Math.min(85, Math.round(value))));
      card.dataset.steerDirty = 'true';
      delete steerResult[id];
      if (output) output.textContent = Math.round(+slider.value) + '°';
      renderSteeringControls(lastData || {});
    }

    function errorText(reason) {
      const messages = {
        pin_required: 'PIN истёк · разблокируйте',
        control_not_supported: 'управление только на бортовом экране',
        module_offline: 'нет свежей связи',
        serial_unavailable: 'порт мастера недоступен',
        not_homed: 'сначала калибровка или 0° вручную',
        module_busy: 'дождитесь остановки модуля / выключите тягу',
        angle_limit: 'угол за пределами ±85°',
        command_queue_full: 'очередь команд занята',
        invalid_module: 'неизвестный модуль',
        invalid_angle: 'неверный угол',
      };
      return messages[reason] || 'команда не принята';
    }

    async function sendSteer(id) {
      const card = panel.querySelector('[data-steer-card="' + id + '"]');
      const slider = $('steer-slider-' + id);
      if (!card || !slider) return;
      if (!steeringAuthorized) {
        if (window._openSteeringAuth) window._openSteeringAuth();
        return;
      }
      const angle = Math.max(-85, Math.min(85, Number(slider.value)));
      const sendButton = panel.querySelector('[data-steer-send="' + id + '"]');
      if (sendButton) sendButton.disabled = true;
      try {
        const response = await fetch('api/kiosk_steer', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'steering' },
          body: JSON.stringify({ module: id, angle })
        });
        const result = await response.json();
        if (!response.ok || !result || !result.ok) {
          if (response.status === 401 || (result && result.reason === 'pin_required')) {
            setSteeringAuthorized(false);
          }
          steerResult[id] = { text: errorText(result && result.reason), error: true, until: Date.now() + 6000 };
          renderSteeringControls(lastData || {});
          if (response.status === 401 && window._openSteeringAuth) window._openSteeringAuth();
          return;
        }
        pendingSteer[id] = { angle: Number(result.angle), sentAt: Date.now() };
        card.dataset.steerDirty = 'false';
        steerResult[id] = { text: 'команда передана · без тяги', error: false, until: Date.now() + 6000 };
        renderSteeringControls(lastData || {});
      } catch (error) {
        steerResult[id] = { text: 'нет связи с сервером', error: true, until: Date.now() + 6000 };
        renderSteeringControls(lastData || {});
      }
    }

    async function setSoftwareZero(id) {
      if (!steeringAuthorized) {
        if (window._openSteeringAuth) window._openSteeringAuth();
        return;
      }
      const confirmed = window.confirm(
        'Запасной вариант без датчика: сначала физически выставьте колесо ' + id + ' прямо. Команда не двигает мотор — она назначит текущую позицию за 0°. Ноль потеряется после перезапуска Arduino. Продолжить?'
      );
      if (!confirmed) return;
      const button = panel.querySelector('[data-steer-origin="' + id + '"]');
      if (button) button.disabled = true;
      try {
        const response = await fetch('api/kiosk_zero', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'steering' },
          body: JSON.stringify({ module: id })
        });
        const result = await response.json();
        if (!response.ok || !result || !result.ok) {
          if (response.status === 401 || (result && result.reason === 'pin_required')) setSteeringAuthorized(false);
          steerResult[id] = { text: errorText(result && result.reason), error: true, until: Date.now() + 6000 };
          if (response.status === 401 && window._openSteeringAuth) window._openSteeringAuth();
          return;
        }
        pendingZero[id] = { sentAt: Date.now() };
        steerResult[id] = { text: 'программный ноль отправлен', error: false, until: Date.now() + 6000 };
      } catch (error) {
        steerResult[id] = { text: 'нет связи с сервером', error: true, until: Date.now() + 6000 };
      } finally {
        renderSteeringControls(lastData || {});
      }
    }

    async function startCalibration(id) {
      if (!steeringAuthorized) {
        if (window._openSteeringAuth) window._openSteeringAuth();
        return;
      }
      const all = id === 'ALL';
      const confirmed = window.confirm(
        (all ? 'Все четыре колеса' : 'Колесо ' + id) +
        ' повернётся: вправо до 100°, затем влево до 200°, пока не найдёт оптодатчик D18, и встанет в ноль. ' +
        'Колёса не должны упираться, рядом не должно быть людей. Тяга во время калибровки запрещена. Начать?'
      );
      if (!confirmed) return;
      const targets = all ? ids : [id];
      const button = all ? $('steer-calib-all') : panel.querySelector('[data-steer-calib="' + id + '"]');
      if (button) button.disabled = true;
      try {
        const response = await fetch('api/kiosk_calibrate', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'steering' },
          body: JSON.stringify({ module: id })
        });
        const result = await response.json().catch(() => ({}));
        if (!response.ok || !result || !result.ok) {
          if (response.status === 401 || (result && result.reason === 'pin_required')) setSteeringAuthorized(false);
          targets.forEach((target) => {
            steerResult[target] = { text: errorText(result && result.reason), error: true, until: Date.now() + 6000 };
          });
          if (response.status === 401 && window._openSteeringAuth) window._openSteeringAuth();
          return;
        }
        const offline = Array.isArray(result.offline) ? result.offline : [];
        targets.filter((target) => !offline.includes(target)).forEach((target) => {
          pendingCalib[target] = { sentAt: Date.now(), started: false };
          delete steerResult[target];
        });
        offline.forEach((target) => {
          steerResult[target] = { text: 'нет связи · не калибруется', error: true, until: Date.now() + 6000 };
        });
      } catch (error) {
        targets.forEach((target) => {
          steerResult[target] = { text: 'нет связи с сервером', error: true, until: Date.now() + 6000 };
        });
      } finally {
        renderSteeringControls(lastData || {});
      }
    }

    const calibAllButton = $('steer-calib-all');
    if (calibAllButton) calibAllButton.addEventListener('click', () => { void startCalibration('ALL'); });

    ids.forEach((id) => {
      const slider = $('steer-slider-' + id);
      const card = panel.querySelector('[data-steer-card="' + id + '"]');
      const calib = panel.querySelector('[data-steer-calib="' + id + '"]');
      if (calib) calib.addEventListener('click', () => { void startCalibration(id); });
      // Ползунок двигает цель на экране; команда уходит, когда палец отпущен
      // (change), — одна команда на жест, а не поток при каждом шаге.
      if (slider) {
        slider.addEventListener('input', () => markDirty(id, slider.value));
        slider.addEventListener('change', () => { void sendSteer(id); });
      }
      panel.querySelectorAll('[data-steer-adjust="' + id + '"]').forEach((button) => {
        button.addEventListener('click', () => {
          markDirty(id, Number(slider.value) + Number(button.dataset.step));
          void sendSteer(id);
        });
      });
      const zero = panel.querySelector('[data-steer-zero="' + id + '"]');
      if (zero) zero.addEventListener('click', () => { markDirty(id, 0); void sendSteer(id); });
      const origin = panel.querySelector('[data-steer-origin="' + id + '"]');
      if (origin) origin.addEventListener('click', () => setSoftwareZero(id));
      const send = panel.querySelector('[data-steer-send="' + id + '"]');
      if (send) send.addEventListener('click', () => sendSteer(id));
      if (card) card.dataset.steerDirty = 'false';
    });

    renderSteeringControls(lastData || {});
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot);
  else boot();
})();
