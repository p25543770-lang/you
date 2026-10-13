/* Бортовой экран робота (киоск): АКБ + шасси + лидар. Без управления + PIN-оверлей для отсека. */
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

  let lastData = null;
  let live = false;
  // флаг: когда водишь пальцем по корпусу для оборота — глушим тягу (моторы не едут)
  let fingerSteerActive = false;
  let fingerSteerAngle = 0;

  async function fetchState() {
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
    }
  }

  function clockText(d) {
    const pad = (n) => String(n).padStart(2, '0');
    return pad(d.getHours()) + ':' + pad(d.getMinutes());
  }

  function batteryPct(b) {
    if (b && Number.isFinite(+b.soc)) return Math.max(0, Math.min(100, +b.soc));
    return null;
  }

  function renderBattery(d) {
    const b = d.battery || {};
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

    setText($('k-volts'), fmt(b.volts, 1));
    setText($('k-amps'), fmt(b.amps, 1));
    setText($('k-power'), fmt(d.powerKw, 2));
    setText($('k-range'), fmt(b.rangeKm, 1));
    setText($('k-btemp'), fmt(b.tempC, 0));
    // когда крутишь пальцем — только руление, тяга ноль
    if (fingerSteerActive) setText($('speed'), '0.0');
    else setText($('speed'), fmt(d.speedMps, 1));
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
    let moving = motors.filter((m) => m && m.moving).length;
    const online = motors.filter((m) => m && m.online).length;
    const parts = [];
    if (fingerSteerActive) {
      moving = 0;
      parts.push('РУЛЕНИЕ');
      parts.push('модулей: ' + online + '/4');
      parts.push('тяга: 0 · поворот ' + Math.round(fingerSteerAngle) + '°');
    } else {
      if (d.mode) parts.push(d.mode);
      parts.push('модулей: ' + online + '/4');
      if (moving) parts.push('в движении ' + moving);
    }
    parts.push(link ? 'связь: ок' : 'нет связи');
    setText($('chassis-meta'), parts.join(' · '));

    ['FL', 'FR', 'RL', 'RR'].forEach((id) => {
      const mRaw = motors.find((x) => x.id === id) || {};
      // если крутишь пальцем — только руление, без тяги
      let m = mRaw;
      let angleOverride = null;
      let rpmOverride = null;
      let movingOverride = null;
      if (fingerSteerActive) {
        const isFront = id[0] === 'F';
        const ang = isFront ? fingerSteerAngle : -fingerSteerAngle * 0.72;
        angleOverride = ang;
        rpmOverride = 0;
        movingOverride = false;
      }
      const eff = angleOverride != null ? {...mRaw, angle: angleOverride, rpm: rpmOverride, moving: movingOverride} : mRaw;
      const tone = fingerSteerActive ? (Math.abs(angleOverride||0) > 2 ? 'move' : 'idle') : modTone(mRaw);
      const card = $('mod-' + id);
      const wheel = $('wheel-' + id);
      if (card) card.setAttribute('data-tone', tone);
      if (wheel) {
        wheel.setAttribute('data-tone', tone);
        if (eff.moving) wheel.classList.add('is-driving');
        else wheel.classList.remove('is-driving');
      }
      const chip = $('mod-state-' + id);
      if (chip) {
        if (fingerSteerActive) {
          chip.textContent = Math.abs(angleOverride||0) > 2 ? 'поворот' : 'онлайн';
          chip.className = 'chip ' + (Math.abs(angleOverride||0) > 2 ? 'move' : 'idle');
        } else {
          chip.textContent = modStateText(mRaw);
          chip.className = 'chip ' + tone;
        }
      }
      const hasAngle = eff.angle != null && Number.isFinite(+eff.angle);
      const angle = hasAngle ? Math.max(-90, Math.min(90, +eff.angle)) : 0;
      setText($('angle-' + id), hasAngle ? Math.round(angle) + '°' : '—');
      if (wheel) wheel.style.transform = 'rotate(' + angle.toFixed(1) + 'deg)';
      if (fingerSteerActive) setText($('rpm-' + id), '0');
      else setText($('rpm-' + id), mRaw.rpm == null ? '—' : fmt(mRaw.rpm, 0));
      setText($('temp-' + id), mRaw.temp == null ? '—' : fmt(mRaw.temp, 0));
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

  function drawLidar(canvas, pts) {
    if (!canvas) return;
    const ctx = canvas.getContext && canvas.getContext('2d');
    if (!ctx) return;
    const dpr = window.devicePixelRatio || 1;
    // clientWidth/clientHeight бывают 0, если браузер не поддерживает
    // CSS aspect-ratio (старые встроенные веб-вьюхи на борту) или раскладка
    // ещё не посчиталась — тогда берём размер ближайшего предка/канвы, чтобы
    // карта лидара не оставалась пустой навсегда.
    const w = canvas.clientWidth || (canvas.parentElement && canvas.parentElement.clientWidth) || canvas.width || 280;
    const h = canvas.clientHeight || (canvas.parentElement && canvas.parentElement.clientHeight) || canvas.height || 280;
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

    // точки скана
    if (pts.length) {
      const near = Math.min.apply(null, pts.map((p) => p.r));
      pts.forEach((p) => {
        const r = Math.min(p.r, LIDAR_RANGE_M) * k;
        const x = cx - r * Math.sin(p.a);
        const y = cy - r * Math.cos(p.a);
        // ближайшее препятствие подсвечиваем красным
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

  function renderLidar(d) {
    const canvas = $('lidar-canvas');
    let pts = lidarPoints(d);
    let isDemo = false;
    const demoReq = new URLSearchParams(window.location.search).get('demo') === '1' || localStorage.getItem('kiosk_demo') === '1';
    if (!pts.length && (demoReq || (typeof autoScanOn !== 'undefined' && autoScanOn))) {
      pts = demoLidar();
      isDemo = true;
    }
    const has = pts.length > 0;
    const near = has ? Math.min.apply(null, pts.map((p) => p.r)) : null;

    setText($('lidar-meta'), has ? (isDemo ? 'демо · ' + pts.length + ' точек' : 'в реальном времени · ' + pts.length + ' точек') : 'нет скана');
    setText($('lidar-count'), has ? String(pts.length) : '—');
    setText($('lidar-near'), near == null ? '—' : fmt(near, 2) + ' м');
    $('lidar-empty').hidden = has;
    drawLidar(canvas, pts);
  }

  function render() {
    setText($('clock'), clockText(new Date()));
    const d = lastData || {};
    const dataChip = $('data-chip');
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
    document.body.classList.toggle('is-stale', !live);

    renderMode(d);
    renderBattery(d);
    renderChassis(d);
    renderLidar(d);
  }

  function loop() {
    render();
    requestAnimationFrame(() => setTimeout(loop, 250));
  }

  function boot() {
    fetchState();
    setInterval(fetchState, POLL_MS);
    loop();
    initKioskControls();
    initSwipePage();
    initFingerSteer();
  }

  /* =======================================================================
     УПРАВЛЕНИЕ ПО PIN: кнопка → клавиатура (води пальцем) → функции
     — статистика остаётся без пароля, PIN нужен только для этого оверлея
     — закрыть → снова статистика, по новой нужен PIN (как просил)
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
    let pinVerified = null; // храним верный PIN в памяти до выхода к статистике
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
      compartmentBtn.disabled = b;
    }

    function showPinView() {
      setHidden(pinView, false);
      setHidden(ctrlView, true);
      setHidden(themesView, true);
      setHidden(resetView, true);
      if (subtitle) subtitle.textContent = 'введи PIN-код — откроются функции';
      if (titleEl) titleEl.textContent = 'Управление роботом';
      setMsg('', '');
      updatePinDisplay();
    }
    function showCtrlView() {
      setHidden(pinView, true);
      setHidden(ctrlView, false);
      setHidden(themesView, true);
      setHidden(resetView, true);
      if (subtitle) subtitle.textContent = 'доступ открыт · выбери действие';
      if (titleEl) titleEl.textContent = 'Функции';
      refreshLockStatus();
    }

    function openOverlay() {
      pinBuf = '';
      // каждый раз заново — как просил: вернулся к статистике → по новой PIN
      pinVerified = null;
      showPinView();
      updatePinDisplay();
      setMsg('', '');
      overlay.hidden = false;
      overlay.setAttribute('aria-hidden', 'false');
      document.body.style.overflow = 'hidden';
      // фокус на шит для ESC
      if (sheet) sheet.focus();
      refreshLockStatus();
      vibrate(10);
    }
    function closeOverlay() {
      overlay.hidden = true;
      overlay.setAttribute('aria-hidden', 'true');
      document.body.style.overflow = '';
      pinBuf = '';
      // сбрасываем авторизацию при возврате к статистике (и так по новой)
      pinVerified = null;
      if (lockTimer) { clearInterval(lockTimer); lockTimer = null; }
      setHidden(resetView, true);
      setHidden(themesView, true);
      // халтура: если отсек открыт — он остаётся открыт, просто прячем панель
      setMsg('', '');
      if (openBtn) openBtn.focus();
    }

    // ---------- lock API ----------
    async function fetchLock() {
      try {
        const r = await fetch('api/kiosk_lock', { cache: 'no-store' });
        const j = await r.json();
        if (j && j.ok && j.lock) return j.lock;
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
        lockChip.textContent = blocked ? 'БЛОКИРОВКА' : (open ? 'ОТСЕК ОТКРЫТ' : 'отсек закрыт');
        lockChip.className = 'ctrl-chip ' + (blocked ? 'blocked' : open ? 'open' : 'closed');
      }
      if (lockDetail) {
        if (blocked) lockDetail.textContent = 'пауза ' + Math.ceil(lock.remainingMs / 1000) + ' с';
        else lockDetail.textContent = open ? 'закрой после разгрузки' : 'готов к открытию';
      }
      if (attemptsEl) {
        if (blocked) attemptsEl.textContent = '';
        else if (lock.attemptsLeft != null && lock.attemptsLeft < 5) attemptsEl.textContent = 'попыток: ' + lock.attemptsLeft;
        else attemptsEl.textContent = '';
      }
      if (compartmentBtn && compartmentTitle && compartmentHint) {
        if (blocked) {
          compartmentBtn.disabled = true;
          compartmentTitle.textContent = 'Заблокировано';
          compartmentHint.textContent = 'ещё ' + Math.ceil(lock.remainingMs / 1000) + ' с';
          compartmentBtn.classList.remove('opened');
        } else if (open) {
          compartmentBtn.disabled = false;
          compartmentTitle.textContent = 'Закрыть отсек';
          compartmentHint.textContent = 'нажми чтобы запереть';
          compartmentBtn.classList.add('opened');
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
          if (lockChip) lockChip.textContent = 'БЛОКИРОВКА';
          if (lockDetail) lockDetail.textContent = 'пауза ' + Math.ceil(rem/1000) + ' с';
          if (compartmentTitle) compartmentTitle.textContent = 'Заблокировано';
          if (compartmentHint) compartmentHint.textContent = 'ещё ' + Math.ceil(rem/1000) + ' с';
          setMsg('Заблокировано. Ещё ' + Math.ceil(rem/1000) + ' с', 'err');
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
      setMsg('Проверяю…', 'warn');
      try {
        const r = await fetch('api/kiosk_lock/verify', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ pin: pinBuf })
        });
        const j = await r.json();
        if (j && j.ok) {
          pinVerified = pinBuf; // запоминаем до возврата к статистике
          if (j.lock) renderLock(j.lock);
          setMsg('Доступ разрешён', 'ok');
          vibrate([20, 40, 20]);
          // маленькая пауза чтобы увидеть успех
          setTimeout(() => {
            showCtrlView();
            setBusy(false);
          }, 420);
        } else {
          // ошибка
          if (j && j.lock) renderLock(j.lock);
          else await refreshLockStatus();
          const reason = j ? j.reason : 'unknown';
          if (reason === 'blocked' || (j && j.lock && j.lock.blocked)) {
            shakeDots(); vibrate([40,80,40,80,80]);
            setMsg('Заблокировано на ' + Math.ceil((j.remainingMs|| lockState.remainingMs|| 30000)/1000) + ' с', 'err');
            // startBlockTimer handled by renderLock
          } else if (reason === 'wrong') {
            shakeDots(); vibrate([30,30,30]);
            setMsg('Неверный PIN. Осталось попыток: ' + (j.attemptsLeft != null ? j.attemptsLeft : '?'), 'err');
            // не очищаем полностью — даём исправить
          } else if (reason === 'format') {
            shakeDots(); setMsg('PIN — только цифры, 4…8', 'warn');
          } else {
            shakeDots(); setMsg('Ошибка проверки', 'err');
          }
          setBusy(false);
          updatePinDisplay();
        }
      } catch (e) {
        setMsg('Нет связи с роботом', 'err');
        setBusy(false);
      }
    }

    async function toggleCompartment() {
      if (busy) return;
      if (lockState && lockState.blocked) { setMsg('Отсек заблокирован', 'err'); return; }
      const open = !!(lockState && lockState.open);
      setBusy(true);
      if (pinMsg) setMsg(open ? 'Запираю…' : 'Открываю…', 'warn');
      // если закрываем — PIN не нужен; если открываем — нужен pinVerified
      try {
        let r, j;
        if (open) {
          r = await fetch('api/kiosk_lock/close', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' });
          j = await r.json();
        } else {
          if (!pinVerified) { setMsg('Введи PIN заново', 'err'); showPinView(); setBusy(false); return; }
          r = await fetch('api/kiosk_lock/open', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ pin: pinVerified })
          });
          j = await r.json();
          if (!j.ok && j.reason === 'blocked') {
            if (j.lock) renderLock(j.lock);
            setMsg('Заблокировано', 'err');
            setBusy(false);
            return;
          }
          if (!j.ok) {
            // возможно PIN сброшен пока был в панели — просим ввести заново
            setMsg('PIN устарел — введи заново', 'err');
            showPinView();
            pinBuf = '';
            updatePinDisplay();
            setBusy(false);
            return;
          }
        }
        if (j && j.lock) renderLock(j.lock);
        else await refreshLockStatus();
        vibrate(open ? 20 : [20,40,20]);
        // toast inside status
        if (lockDetail) lockDetail.textContent = open ? 'отсек заперт' : 'отсек открыт — забери груз';
        setMsg(open ? 'Отсек закрыт' : 'Отсек открыт', 'ok');
      } catch (e) {
        setMsg('Нет связи', 'err');
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
    if (pinCancel) pinCancel.addEventListener('click', closeOverlay);
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
    // периодически обновляем статус отсека, даже когда оверлей закрыт (чтобы чип был актуален)
    setInterval(() => {
      if (!overlay.hidden) refreshLockStatus();
    }, 1500);
  }

  /* ============ СВАЙП-СТРАНИЦА: 2 камеры + логи (свайп вправо) ============ */
  function initSwipePage() {
    const page = $('swipe-page');
    const openBtn = $('swipe-open');
    const closeBtn = $('swipe-close');
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
    let camLoopsStarted = false;
    let anim1 = null, anim2 = null;

    const sheet = page.querySelector('.swipe-sheet');
    const backdrop = $('swipe-backdrop');

    function openSwipe() {
      if (isOpen) return;
      const overlay = $('ctrl-overlay');
      if (overlay && !overlay.hidden) return;
      isOpen = true;
      page.hidden = false;
      if (sheet) sheet.style.transform = '';
      if (backdrop) backdrop.style.opacity = '';
      void page.offsetWidth;
      page.classList.add('is-open');
      document.body.classList.add('swipe-open');
      page.setAttribute('aria-hidden', 'false');
      if (hintBar) hintBar.style.opacity = '0';
      startCamLoops();
      fetchLogs();
    }
    function closeSwipe() {
      if (!isOpen) return;
      isOpen = false;
      page.classList.remove('is-open');
      document.body.classList.remove('swipe-open');
      page.setAttribute('aria-hidden', 'true');
      if (hintBar) hintBar.style.opacity = '';
      if (sheet) sheet.style.transform = '';
      if (backdrop) backdrop.style.opacity = '';
      setTimeout(() => { if (!isOpen) page.hidden = true; }, 480);
    }
    window._openSwipe = openSwipe;
    window._closeSwipe = closeSwipe;

    if (openBtn) openBtn.addEventListener('click', openSwipe);
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

    // --- свайп пальцем: ведёшь ВЛЕВО — туда (камеры/логи), ВПРАВО — обратно (новая анимация) ---
    let tStartX = 0, tStartY = 0, tTracking = false, dragging = false, dragType = null;

    function setPageProgress(px) {
      const p = Math.max(0, Math.min(1, px));
      if (sheet) sheet.style.transform = 'translateX(' + ((1 - p) * 105) + '%)';
      if (backdrop) backdrop.style.opacity = String(p);
      // киоск лёгкий параллакс
      const kioskEl = kiosk;
      if (kioskEl) {
        const s = 1 - p * 0.015;
        const br = 1 - p * 0.08;
        kioskEl.style.transform = 'scale(' + s + ')';
        kioskEl.style.filter = 'brightness(' + br + ')';
      }
    }
    function beginDrag() {
      page.hidden = false;
      if (sheet) sheet.style.transition = 'none';
      if (backdrop) backdrop.style.transition = 'none';
      if (kiosk) { kiosk.style.transition = 'none'; }
      void page.offsetWidth;
      dragging = true;
    }
    function endDragCleanup() {
      if (sheet) sheet.style.transition = '';
      if (backdrop) backdrop.style.transition = '';
      if (kiosk) { kiosk.style.transition = ''; kiosk.style.transform = ''; kiosk.style.filter = ''; }
      dragging = false;
      dragType = null;
    }

    function tStart(x, y) { tStartX = x; tStartY = y; tTracking = true; dragging = false; dragType = null; }
    function tMove(x, y) {
      if (!tTracking) return;
      const dx = x - tStartX;
      const dy = y - tStartY;
      if (!dragging) {
        if (Math.abs(dx) < 10) return;
        if (Math.abs(dx) < Math.abs(dy) * 1.1) { tTracking = false; return; }
        // с закрытого — только ВЛЕВО (dx < 0), с открытого — только ВПРАВО (dx > 0)
        if (!isOpen && dx > 0) { tTracking = false; return; }
        if (isOpen && dx < 0) { tTracking = false; return; }
        const ov = $('ctrl-overlay');
        if (!isOpen && ov && !ov.hidden) { tTracking = false; return; }
        dragType = !isOpen ? 'open' : 'close';
        beginDrag();
        if (!isOpen) page.setAttribute('aria-hidden', 'false');
        document.body.classList.add('swipe-open');
      }
      if (!dragging) return;
      const vw = window.innerWidth || 400;
      let progress;
      if (dragType === 'open') {
        // swipe left: dx 0..-vw  → progress 0..1
        progress = Math.max(0, Math.min(1, -dx / (vw * 0.52)));
        setPageProgress(progress);
        if (hintBar) hintBar.style.opacity = String(1 - progress);
      } else {
        // close: dx 0..+vw → progress 1..0
        progress = Math.max(0, Math.min(1, 1 - dx / (vw * 0.52)));
        setPageProgress(progress);
        if (hintBar) hintBar.style.opacity = String(progress);
      }
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
        if (!isOpen && dx < -65) openSwipe();
        else if (isOpen && dx > 65) closeSwipe();
        return;
      }
      const absDx = Math.abs(dx);
      const threshold = vw * 0.18;
      let shouldOpen;
      if (dragType === 'open') shouldOpen = -dx > threshold || absDx > 70 && dx < 0;
      else shouldOpen = !(dx > threshold || absDx > 70);
      // сбросить инлайн
      if (sheet) sheet.style.transform = '';
      if (backdrop) backdrop.style.opacity = '';
      if (kiosk) { kiosk.style.transform = ''; kiosk.style.filter = ''; kiosk.style.transition = ''; }
      if (sheet) sheet.style.transition = '';
      if (backdrop) backdrop.style.transition = '';
      dragging = false;
      dragType = null;
      if (hintBar) hintBar.style.opacity = '';
      if (shouldOpen) {
        if (!isOpen) openSwipe();
        else { page.hidden = false; page.classList.add('is-open'); document.body.classList.add('swipe-open'); page.setAttribute('aria-hidden','false'); }
      } else {
        if (isOpen) closeSwipe();
        else {
          page.classList.remove('is-open');
          document.body.classList.remove('swipe-open');
          page.setAttribute('aria-hidden','true');
          setTimeout(() => { if (!isOpen) page.hidden = true; }, 480);
        }
      }
    }

    // touch
    kiosk.addEventListener('touchstart', (e) => {
      if (e.touches.length !== 1) return;
      tStart(e.touches[0].clientX, e.touches[0].clientY);
    }, {passive:true});
    kiosk.addEventListener('touchmove', (e) => {
      if (e.touches.length !== 1) return;
      tMove(e.touches[0].clientX, e.touches[0].clientY);
    }, {passive:true});
    kiosk.addEventListener('touchend', (e) => {
      const t = e.changedTouches[0];
      if (t) tEnd(t.clientX, t.clientY);
    }, {passive:true});
    page.addEventListener('touchstart', (e) => {
      if (e.touches.length !== 1) return;
      // не перехватывать если тянут скролл внутри логов вертикально — дадим шанс
      tStart(e.touches[0].clientX, e.touches[0].clientY);
    }, {passive:true});
    page.addEventListener('touchmove', (e) => {
      if (e.touches.length !== 1) return;
      // если палец внутри log-list и скролл вертикальный — не мешаем
      const target = e.target;
      if (target && target.closest && target.closest('.log-list')) {
        // если ещё не dragging — посмотрим на dx/dy, но не форсируем prevent
      }
      tMove(e.touches[0].clientX, e.touches[0].clientY);
    }, {passive:true});
    page.addEventListener('touchend', (e) => {
      const t = e.changedTouches[0];
      if (t) tEnd(t.clientX, t.clientY);
    }, {passive:true});

    // мышь drag для десктопа — тоже с тягой
    let mTracking = false, mStartX = 0, mStartY = 0, mDragging = false, mType = null;
    function mStart(x,y,origin){
      mStartX=x; mStartY=y; mTracking=true; mDragging=false; mType=null;
      // origin: 'kiosk' или 'page'
      mOrigin = origin;
    }
    let mOrigin = null;
    kiosk.addEventListener('mousedown', (e) => {
      if (e.button !== 0) return;
      const ov = $('ctrl-overlay');
      if (ov && !ov.hidden) return;
      mStart(e.clientX, e.clientY, 'kiosk');
    });
    page.addEventListener('mousedown', (e) => {
      if (e.button !== 0) return;
      mStart(e.clientX, e.clientY, 'page');
    });
    window.addEventListener('mousemove', (e) => {
      if (!mTracking) return;
      const dx = e.clientX - mStartX;
      const dy = e.clientY - mStartY;
      if (!mDragging) {
        if (Math.abs(dx) < 12) return;
        if (Math.abs(dx) < Math.abs(dy)*1.1) { mTracking=false; return; }
        if (!isOpen && dx > 0) { mTracking=false; return; }
        if (isOpen && dx < 0) { mTracking=false; return; }
        const ov = $('ctrl-overlay');
        if (!isOpen && ov && !ov.hidden) { mTracking=false; return; }
        mType = !isOpen ? 'open' : 'close';
        page.hidden = false;
        if (sheet) sheet.style.transition = 'none';
        if (backdrop) backdrop.style.transition = 'none';
        if (kiosk) kiosk.style.transition = 'none';
        void page.offsetWidth;
        mDragging = true;
        dragging = true;
        dragType = mType;
        document.body.classList.add('swipe-open');
        if (!isOpen) page.setAttribute('aria-hidden','false');
      }
      if (!mDragging) return;
      const vw = window.innerWidth || 400;
      let p;
      if (mType === 'open') p = Math.max(0, Math.min(1, -dx/(vw*0.52)));
      else p = Math.max(0, Math.min(1, 1 - dx/(vw*0.52)));
      setPageProgress(p);
      if (hintBar) hintBar.style.opacity = String(mType === 'open' ? 1 - p : p);
      e.preventDefault();
    });
    window.addEventListener('mouseup', (e) => {
      if (!mTracking && !mDragging) return;
      const wasDrag = mDragging;
      const dx = e.clientX - mStartX;
      const dy = e.clientY - mStartY;
      const vw = window.innerWidth || 400;
      const curDragging = mDragging;
      const curType = mType;
      mTracking = false; mDragging = false;
      dragging = false; dragType = null;
      if (sheet) sheet.style.transition = '';
      if (backdrop) backdrop.style.transition = '';
      if (kiosk) { kiosk.style.transition = ''; kiosk.style.transform = ''; kiosk.style.filter = ''; }
      if (sheet) sheet.style.transform = '';
      if (backdrop) backdrop.style.opacity = '';
      if (hintBar) hintBar.style.opacity = '';
      if (!wasDrag) {
        if (Math.abs(dx) < 80 || Math.abs(dx) < Math.abs(dy)*1.1) { mOrigin=null; return; }
        if (mOrigin === 'kiosk' && !isOpen && dx < -80) openSwipe();
        else if (mOrigin === 'page' && isOpen && dx > 80) closeSwipe();
        mOrigin=null;
        return;
      }
      const thr = vw*0.18;
      let shouldOpen;
      if (curType === 'open') shouldOpen = -dx > thr;
      else shouldOpen = !(dx > thr);
      if (shouldOpen) {
        if (!isOpen) openSwipe();
        else { page.hidden=false; page.classList.add('is-open'); document.body.classList.add('swipe-open'); page.setAttribute('aria-hidden','false'); }
      } else {
        if (isOpen) closeSwipe();
        else { page.classList.remove('is-open'); document.body.classList.remove('swipe-open'); page.setAttribute('aria-hidden','true'); setTimeout(()=>{ if(!isOpen) page.hidden=true; },480); }
      }
      mOrigin=null;
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
          ctx1.fillText('CAM 1 · LIVE · ' + new Date().toLocaleTimeString('ru-RU'), 10, 18);
          anim1 = requestAnimationFrame(drawCam1);
          return;
        }

        const isDemo = new URLSearchParams(window.location.search).get('demo') === '1' || localStorage.getItem('kiosk_demo') === '1';
        if (isDemo) {
          if (cam1Empty) cam1Empty.hidden = true;
          if (cam1Chip) { cam1Chip.textContent = '● LIVE · демо'; cam1Chip.className = 'cam-chip ok'; }
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
          ctx2.fillText('CAM 2 · LIVE · ' + new Date().toLocaleTimeString('ru-RU'), 10, 18);
          anim2 = requestAnimationFrame(drawCam2);
          return;
        }

        const isDemo = new URLSearchParams(window.location.search).get('demo') === '1' || localStorage.getItem('kiosk_demo') === '1';
        if (isDemo) {
          if (cam2Empty) cam2Empty.hidden = true;
          if (cam2Chip) { cam2Chip.textContent = '● LIVE · демо'; cam2Chip.className = 'cam-chip ok'; }
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

    // --- ЛОГИ: /api/audit + локальные демо-события ---
    let logs = [];
    let logSeq = 0;

    function pushLog(level, msg) {
      const ts = new Date();
      const hh = String(ts.getHours()).padStart(2,'0');
      const mm = String(ts.getMinutes()).padStart(2,'0');
      const ss = String(ts.getSeconds()).padStart(2,'0');
      logs.push({ts: hh+':'+mm+':'+ss, level, msg});
      if (logs.length > 220) logs.shift();
      renderLogs();
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
        const ts = document.createElement('span'); ts.className='log-ts'; ts.textContent = l.ts;
        const lvl = document.createElement('span'); lvl.className='log-lvl '+l.level; lvl.textContent = l.level;
        const msg = document.createElement('span'); msg.className='log-msg'; msg.textContent = l.msg;
        row.appendChild(ts); row.appendChild(lvl); row.appendChild(msg);
        frag.appendChild(row);
      });
      logList.appendChild(frag);
      if (logCount) logCount.textContent = logs.length + '';
      if (logAuto && logAuto.checked) logList.scrollTop = logList.scrollHeight;
    }

    async function fetchLogs() {
      try {
        const r = await fetch('api/audit?limit=40', {cache:'no-store'});
        const j = await r.json();
        if (j && j.ok && Array.isArray(j.audit) && j.audit.length) {
          // преобразуем аудит в логи
          const mapped = j.audit.slice().reverse().map(a => {
            const d = new Date(a.ts*1000);
            const ts = String(d.getHours()).padStart(2,'0')+':'+String(d.getMinutes()).padStart(2,'0')+':'+String(d.getSeconds()).padStart(2,'0');
            let lvl = 'info';
            if (a.action === 'unlock' && a.ok) lvl = 'ok';
            else if (!a.ok) lvl = 'err';
            else if (a.action === 'pin_change') lvl = 'warn';
            return {ts, level:lvl, msg: '['+a.action+'] '+(a.detail|| (a.ok?'ok':'fail'))};
          });
          // если новые — добавляем
          mapped.forEach(m => {
            const key = m.ts+m.msg;
            if (!logs.some(x=> x.ts===m.ts && x.msg===m.msg)) logs.push(m);
          });
          if (logs.length>220) logs.splice(0, logs.length-220);
          if (logStatus) logStatus.textContent = 'аудит · ' + j.audit.length + ' записей · ' + (j.integrity && j.integrity.ok ? 'целостность ok' : 'проверь цепь');
          renderLogs();
          return;
        }
        if (logStatus) logStatus.textContent = 'журнал аудита · пусто — демо';
      } catch(e) {
        if (logStatus) logStatus.textContent = 'нет связи с /api/audit — демо';
      }
      // если аудит пуст — держим демо-логи
      if (!logs.length) {
        pushLog('info','система готова · ожидание данных');
      }
    }

    // демо-поток логов каждые 1.2-2.5 сек
    const demoMsgs = [
      ['info','SLAM: скан лидара 360° — 412 точек'],
      ['info','мотор FL: 12.4° · 42 об/мин'],
      ['info','мотор FR: -8.1° · 41 об/мин'],
      ['info','АКБ: 77.8% · 39.2В · 1.1А · запас 17.3 км'],
      ['info','маршрут: склад → зона выгрузки · 0.6 м/с'],
      ['ok','связь с модулями: ок (4/4)'],
      ['warn','лидар: ближайшая точка 0.84 м — замедление'],
      ['info','навигация: коррекция курса -3.2°'],
      ['info','камера 1: кадр захвачен 1920×1080'],
      ['info','камера 2: кадр захвачен 1920×1080'],
      ['info','телеметрия: отправка /api/state'],
    ];
    setInterval(() => {
      const pick = demoMsgs[Math.floor(Math.random()*demoMsgs.length)];
      // немного рандома времени
      if (Math.random() < 0.55) pushLog(pick[0], pick[1]);
    }, 1400);
    setInterval(fetchLogs, 2500);
    // первичная загрузка
    fetchLogs();
    // начальные логи
    pushLog('ok','бортовой экран RUS SLAM запущен');
    pushLog('info','свайп влево → камеры и логи · вправо — назад');
    pushLog('info','PIN 7232 · /api/kiosk_lock');
    setTimeout(() => pushLog('info','лидар: инициализация · дальность 10 м'), 600);

    if (logClear) logClear.addEventListener('click', () => { logs = []; renderLogs(); pushLog('info','логи очищены'); });
    if (logCopy) logCopy.addEventListener('click', async () => {
      const txt = logs.map(l => l.ts+' ['+l.level.toUpperCase()+'] '+l.msg).join('\n');
      try { await navigator.clipboard.writeText(txt); pushLog('ok','логи скопированы в буфер'); }
      catch(e) { pushLog('warn','не удалось скопировать — выдели вручную'); }
    });

    // также свайп индикатор — при первом посещении подмигнуть
    setTimeout(() => {
      if (!isOpen && hintBar) {
        try { hintBar.animate([{transform:'translateX(-50%) scale(1)'},{transform:'translateX(-50%) scale(1.04)'},{transform:'translateX(-50%) scale(1)'}], {duration: 900, iterations: 1}); } catch(e) {}
      }
    }, 1800);
  }

  /* ============ ПОВОРОТ ПАЛЬЦЕМ: води по корпусу — оборот ============ */
  function initFingerSteer() {
    const bodyEl = $('body-drag') || document.querySelector('.body');
    if (!bodyEl) return;
    const wheels = {
      FL: $('wheel-FL'), FR: $('wheel-FR'),
      RL: $('wheel-RL'), RR: $('wheel-RR')
    };
    const angleEls = {
      FL: $('angle-FL'), FR: $('angle-FR'),
      RL: $('angle-RL'), RR: $('angle-RR')
    };
    let isDragging = false;
    let startX = 0;
    let startAngle = 0;
    let curAngle = 0;
    let animFrame = null;
    let returnAnim = null;
    const LIMIT = 32; // градусов лимит
    const SENS = 0.38; // deg per px

    function applyAngle(a) {
      curAngle = Math.max(-LIMIT, Math.min(LIMIT, a));
      fingerSteerAngle = curAngle;
      if (isDragging) fingerSteerActive = true;
      // передние — в сторону движения, задние — противоположно для разворота на месте
      const front = curAngle;
      const rear = -curAngle * 0.72;
      const map = {FL: front, FR: front, RL: rear, RR: rear};
      ['FL','FR','RL','RR'].forEach(id => {
        const w = wheels[id];
        const el = angleEls[id];
        const ang = map[id];
        if (w) w.style.transform = 'rotate(' + ang.toFixed(1) + 'deg)';
        if (el) el.textContent = Math.round(ang) + '°';
        if (w) w.setAttribute('data-tone', Math.abs(ang) > 2 ? 'move' : 'idle');
        const card = document.getElementById('mod-' + id);
        if (card) card.setAttribute('data-tone', Math.abs(ang) > 2 ? 'move' : 'idle');
      });
      // тяга — ноль пока крутишь (по ТЗ)
      const speedEl = $('speed');
      if (speedEl) speedEl.textContent = '0.0';
      // сразу обновить чипы/мэта без ожидания следующего polling
      const meta = $('chassis-meta');
      if (meta) {
        const link = lastData ? !!lastData.linkOk : true;
        const online = lastData && Array.isArray(lastData.motors) ? lastData.motors.filter(m=>m&&m.online).length : 4;
        meta.textContent = 'РУЛЕНИЕ · модулей: ' + online + '/4 · тяга: 0 · поворот ' + Math.round(curAngle) + '° · ' + (link?'связь: ок':'нет связи');
      }
      ['FL','FR','RL','RR'].forEach(id => {
        const isFront = id[0]==='F';
        const ang = isFront ? curAngle : -curAngle*0.72;
        const chip = $('mod-state-' + id);
        if (chip) {
          chip.textContent = Math.abs(ang) > 2 ? 'поворот' : 'онлайн';
          chip.className = 'chip ' + (Math.abs(ang) > 2 ? 'move' : 'idle');
        }
        const rpmEl = $('rpm-' + id);
        if (rpmEl) rpmEl.textContent = '0';
      });
    }

    function stopReturnAnim() {
      if (returnAnim) { cancelAnimationFrame(returnAnim); returnAnim = null; }
    }

    function animateBack() {
      stopReturnAnim();
      const from = curAngle;
      const start = performance.now();
      const dur = 520;
      // держим флаг активным пока крутим обратно
      fingerSteerActive = true;
      function step(now) {
        const t = Math.min(1, (now - start)/dur);
        const ease = 1 - Math.pow(1 - t, 3);
        const a = from * (1 - ease);
        curAngle = a;
        fingerSteerAngle = a;
        // визуально
        const front = a;
        const rear = -a * 0.72;
        const map = {FL: front, FR: front, RL: rear, RR: rear};
        ['FL','FR','RL','RR'].forEach(id => {
          const w = wheels[id];
          const el = angleEls[id];
          const ang = map[id];
          if (w) w.style.transform = 'rotate(' + ang.toFixed(1) + 'deg)';
          if (el) el.textContent = Math.round(ang) + '°';
          if (w) w.setAttribute('data-tone', Math.abs(ang) > 1 ? 'move' : 'idle');
          const card = document.getElementById('mod-' + id);
          if (card) card.setAttribute('data-tone', Math.abs(ang) > 1 ? 'move' : 'idle');
        });
        const speedEl = $('speed');
        if (speedEl) speedEl.textContent = '0.0';
        if (t < 1) {
          returnAnim = requestAnimationFrame(step);
        } else {
          curAngle = 0;
          fingerSteerAngle = 0;
          fingerSteerActive = false;
          bodyEl.classList.remove('is-dragging');
          // вернуть реальный рендер
          if (lastData) { renderChassis(lastData); renderBattery(lastData); }
          else { fingerSteerActive = false; }
        }
      }
      returnAnim = requestAnimationFrame(step);
    }

    function onDown(x) {
      stopReturnAnim();
      isDragging = true;
      fingerSteerActive = true;
      startX = x;
      startAngle = curAngle;
      bodyEl.classList.add('is-dragging');
      bodyEl.setPointerCapture && bodyEl.setPointerCapture(1);
      if (navigator.vibrate) try { navigator.vibrate(12); } catch(e) {}
    }
    function onMove(x) {
      if (!isDragging) return;
      const dx = x - startX;
      applyAngle(startAngle + dx * SENS);
    }
    function onUp() {
      if (!isDragging) return;
      isDragging = false;
      bodyEl.classList.remove('is-dragging');
      if (navigator.vibrate) try { navigator.vibrate([14, 20, 14]); } catch(e) {}
      animateBack();
    }

    // pointer events — работают и мышью и пальцем
    bodyEl.addEventListener('pointerdown', (e) => {
      if (e.button !== 0 && e.pointerType === 'mouse') return;
      // не дать свайпу страницы перехватить: стоп пропагейшн
      e.stopPropagation();
      onDown(e.clientX);
      try { bodyEl.setPointerCapture(e.pointerId); } catch(ex){}
    });
    bodyEl.addEventListener('pointermove', (e) => {
      if (!isDragging) return;
      e.stopPropagation();
      onMove(e.clientX);
      e.preventDefault();
    }, {passive:false});
    bodyEl.addEventListener('pointerup', (e) => {
      e.stopPropagation();
      onUp();
      try { bodyEl.releasePointerCapture(e.pointerId); } catch(ex){}
    });
    bodyEl.addEventListener('pointercancel', (e) => {
      e.stopPropagation();
      onUp();
    });
    bodyEl.addEventListener('pointerleave', (e) => {
      // не отпускаем если палец вышел — ждём up
    });

    // touch fallback для старых браузеров без pointer
    bodyEl.addEventListener('touchstart', (e) => {
      if (e.touches.length !== 1) return;
      e.stopPropagation();
      onDown(e.touches[0].clientX);
    }, {passive:true});
    bodyEl.addEventListener('touchmove', (e) => {
      if (!isDragging || e.touches.length !== 1) return;
      e.stopPropagation();
      onMove(e.touches[0].clientX);
      e.preventDefault();
    }, {passive:false});
    bodyEl.addEventListener('touchend', (e) => {
      e.stopPropagation();
      onUp();
    }, {passive:true});

    // также можно крутить колёсиком мыши для десктопа (опционально)
    bodyEl.addEventListener('wheel', (e) => {
      e.preventDefault();
      applyAngle(curAngle + Math.sign(e.deltaX || e.deltaY) * 2.2);
      clearTimeout(bodyEl._wheelTimer);
      bodyEl._wheelTimer = setTimeout(animateBack, 700);
    }, {passive:false});

    // подсказка: первый раз пульсация
    setTimeout(() => {
      bodyEl.animate && bodyEl.animate([{transform:'scale(1)'},{transform:'scale(1.01)'},{transform:'scale(1)'}], {duration: 700, iterations: 1});
    }, 2200);
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot);
  else boot();
})();
