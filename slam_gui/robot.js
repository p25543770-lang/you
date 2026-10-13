/* Бортовой экран: телеметрия + PIN-защищённый адресный поворот рулевых модулей. */
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
    steeringAuthorized = !!on;
    renderSteeringControls(lastData || {});
  }

  let lastData = null;
  let live = false;
  let steeringAuthorized = false;
  let steeringUnlockPending = false;
  const pendingSteer = Object.create(null);
  const steerResult = Object.create(null);

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

  function renderLidar(d) {
    const canvas = $('lidar-canvas');
    const pts = lidarPoints(d);
    const has = pts.length > 0;
    const near = has ? Math.min.apply(null, pts.map((p) => p.r)) : null;

    setText($('lidar-meta'), has ? 'в реальном времени · ' + pts.length + ' точек' : 'нет скана');
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
    renderSteeringControls(d);
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
    initSteeringControls();
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
    function revokeSteeringSession() {
      setSteeringAuthorized(false);
      Object.keys(pendingSteer).forEach((id) => delete pendingSteer[id]);
      Object.keys(steerResult).forEach((id) => delete steerResult[id]);
      fetch('api/kiosk_lock/session/end', {
        method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}', keepalive: true
      }).catch(() => {});
    }

    function closeOverlay() {
      overlay.hidden = true;
      overlay.setAttribute('aria-hidden', 'true');
      document.body.style.overflow = '';
      pinBuf = '';
      pinVerified = null;
      steeringUnlockPending = false;
      revokeSteeringSession();
      if (lockTimer) { clearInterval(lockTimer); lockTimer = null; }
      setHidden(resetView, true);
      setHidden(themesView, true);
      setMsg('', '');
      if (openBtn) openBtn.focus();
    }

    function dismissOverlayForSteering() {
      overlay.hidden = true;
      overlay.setAttribute('aria-hidden', 'true');
      document.body.style.overflow = '';
      pinBuf = '';
      pinVerified = null;
      steeringUnlockPending = false;
      if (lockTimer) { clearInterval(lockTimer); lockTimer = null; }
      setHidden(pinView, true);
      setHidden(ctrlView, true);
      setHidden(themesView, true);
      setHidden(resetView, true);
      if (window._showSteeringPage) window._showSteeringPage();
    }

    window._openSteeringAuth = () => {
      steeringUnlockPending = true;
      openOverlay();
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
          pinVerified = pinBuf;
          setSteeringAuthorized(!!j.steeringAuthorized);
          if (j.lock) renderLock(j.lock);
          setMsg('Доступ разрешён', 'ok');
          vibrate([20, 40, 20]);
          setTimeout(() => {
            if (steeringUnlockPending) dismissOverlayForSteering();
            else showCtrlView();
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
    // Проверяем блокировку PIN и срок сессии руления, даже когда оверлей закрыт.
    setInterval(refreshLockStatus, 1500);
  }

  /* ============ СВАЙП-СТРАНИЦА: поворот модулей + камеры/логи ============ */
  function initSwipePage() {
    const page = $('swipe-page');
    const closeBtn = $('swipe-close');
    const cameraView = $('swipe-cameras-view');
    const motorView = $('swipe-motors-view');
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
      activeSwipeView = showMotors ? 'motors' : 'cameras';
      activeSwipeDirection = showMotors ? 'right' : 'left';
      page.dataset.openDirection = activeSwipeDirection;
      if (cameraView) cameraView.hidden = showMotors;
      if (motorView) motorView.hidden = !showMotors;
      if (swipeTitle) swipeTitle.textContent = showMotors ? 'Поворот модулей' : 'Камеры и логи';
      if (swipeTitleHint) swipeTitleHint.textContent = showMotors
        ? 'свайп влево — назад · только рулевые углы, без тяги'
        : 'свайп вправо — назад к статистике';
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
    window._openSwipe = (view) => openSwipe(view || 'motors');
    window._closeSwipe = closeSwipe;
    window._showSteeringPage = () => {
      if (!isOpen) openSwipe('motors');
      else selectSwipeView('motors');
    };
    selectSwipeView('cameras');

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

    // --- КАМЕРЫ: без симуляции — показываем «нет сигнала» до реального потока ---
    function startCamLoops() {
      if (camLoopsStarted) return;
      camLoopsStarted = true;
      if (cam1Empty) cam1Empty.hidden = false;
      if (cam2Empty) cam2Empty.hidden = false;
      if (cam1Chip) { cam1Chip.textContent = 'нет сигнала'; cam1Chip.className = 'cam-chip'; }
      if (cam2Chip) { cam2Chip.textContent = 'нет сигнала'; cam2Chip.className = 'cam-chip'; }
      return;

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
        const t = (now - t0) / 1000;
        // фон — асфальт
        ctx1.fillStyle = '#0f0f0f';
        ctx1.fillRect(0,0,w,h);
        // перспектива дороги
        const grad = ctx1.createLinearGradient(0,0,0,h);
        grad.addColorStop(0,'#1a1a1c');
        grad.addColorStop(0.55,'#141416');
        grad.addColorStop(1,'#0e0e0e');
        ctx1.fillStyle = grad;
        ctx1.fillRect(0,0,w,h);
        // разметка
        ctx1.strokeStyle = 'rgba(255,255,255,.22)';
        ctx1.setLineDash([18,18]);
        ctx1.lineWidth = 3;
        ctx1.beginPath();
        ctx1.moveTo(w/2, h*0.35);
        ctx1.lineTo(w/2, h);
        ctx1.stroke();
        ctx1.setLineDash([]);
        // бордюры
        ctx1.strokeStyle = '#2a2a2c';
        ctx1.lineWidth = 2;
        ctx1.beginPath();
        ctx1.moveTo(w*0.18, h*0.35); ctx1.lineTo(0, h);
        ctx1.moveTo(w*0.82, h*0.35); ctx1.lineTo(w, h);
        ctx1.stroke();
        // движущийся прямоугольник — будто ящик/робот впереди
        const x = w/2 + Math.sin(t*0.7)*w*0.18;
        const y = h*0.52 + Math.sin(t*0.9)*6;
        ctx1.fillStyle = '#ffffff';
        ctx1.globalAlpha = 0.95;
        ctx1.fillRect(x-36, y-18, 72, 36);
        ctx1.globalAlpha = 1;
        ctx1.fillStyle = 'rgba(255,255,255,.08)';
        ctx1.fillRect(0, h-48, w, 48);
        // перекрестие
        ctx1.strokeStyle = 'rgba(255,255,255,.55)';
        ctx1.lineWidth = 1;
        ctx1.beginPath();
        ctx1.moveTo(w/2-14, h/2); ctx1.lineTo(w/2+14, h/2);
        ctx1.moveTo(w/2, h/2-14); ctx1.lineTo(w/2, h/2+14);
        ctx1.stroke();
        ctx1.fillStyle = 'rgba(255,255,255,.75)';
        ctx1.font = '11px ui-monospace, monospace';
        ctx1.fillText('CAM 1 · 30fps · ' + new Date().toLocaleTimeString('ru-RU'), 10, 18);
        anim1 = requestAnimationFrame(drawCam1);
      }
      function drawCam2(now) {
        if (!ctx2 || !cam2Canvas) return;
        const {w, h} = resize(cam2Canvas, ctx2);
        const t = (now - t0) / 1000;
        ctx2.fillStyle = '#0a0a0a';
        ctx2.fillRect(0,0,w,h);
        const grad = ctx2.createLinearGradient(0,0,0,h);
        grad.addColorStop(0,'#141416');
        grad.addColorStop(1,'#0a0a0a');
        ctx2.fillStyle = grad;
        ctx2.fillRect(0,0,w,h);
        // сетка пола
        ctx2.strokeStyle = '#222226';
        ctx2.lineWidth = 1;
        for (let i=0;i<w;i+=40) { ctx2.beginPath(); ctx2.moveTo(i + (t*18)%40, 0); ctx2.lineTo(i + (t*18)%40, h); ctx2.stroke(); }
        for (let j=0;j<h;j+=36) { ctx2.beginPath(); ctx2.moveTo(0,j); ctx2.lineTo(w,j); ctx2.stroke(); }
        // объект сзади — стена/стеллаж
        const sx = w*0.5 + Math.cos(t*0.5)*w*0.12;
        ctx2.fillStyle = '#ffffff';
        ctx2.fillRect(sx-42, h*0.35, 84, h*0.45);
        ctx2.fillStyle = '#1a1a1c';
        ctx2.fillRect(sx-30, h*0.45, 60, 8);
        ctx2.fillRect(sx-30, h*0.62, 60, 8);
        // виньетка
        const vg = ctx2.createRadialGradient(w/2,h/2, w*0.35, w/2,h/2, w*0.85);
        vg.addColorStop(0,'transparent');
        vg.addColorStop(1,'rgba(0,0,0,.55)');
        ctx2.fillStyle = vg;
        ctx2.fillRect(0,0,w,h);
        ctx2.fillStyle = 'rgba(255,255,255,.75)';
        ctx2.font = '11px ui-monospace, monospace';
        ctx2.fillText('CAM 2 · 30fps · ' + new Date().toLocaleTimeString('ru-RU'), 10, 18);
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
    pushLog('info','свайп вправо → поворот модулей · влево → камеры и логи');
    pushLog('info','управление рулевыми модулями защищено PIN-кодом');
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

  /* ============ Адресный поворот рулевых модулей (без тяги) ============ */
  function renderSteeringControls(data) {
    const panel = $('swipe-motors-view');
    if (!panel) return;
    const d = data || {};
    const control = d.steeringControl || {};
    const available = control.available === true;
    const source = String(control.source || d.source || 'unknown');
    const motors = Array.isArray(d.motors) ? d.motors : [];
    const sourceStatus = $('steer-source-status');
    const authStatus = $('steer-auth-status');
    const unlockButton = $('steer-unlock');
    const lockButton = $('steer-lock');

    if (sourceStatus) {
      if (!available) {
        sourceStatus.textContent = 'Ручной поворот доступен только при RC_SLAM_SOURCE=master и подключённом UART мастера.';
      } else if (d.linkOk === false) {
        sourceStatus.textContent = 'Мастер выбран, но свежей связи с рулевыми модулями пока нет.';
      } else {
        sourceStatus.textContent = 'Мастер подключён · изменение угла одного модуля за команду.';
      }
    }
    if (authStatus) authStatus.textContent = steeringAuthorized ? 'PIN подтверждён' : 'управление закрыто';
    if (unlockButton) unlockButton.hidden = steeringAuthorized;
    if (lockButton) lockButton.hidden = !steeringAuthorized;

    const now = Date.now();
    ['FL', 'FR', 'RL', 'RR'].forEach((id) => {
      const card = panel.querySelector('[data-steer-card="' + id + '"]');
      const slider = $('steer-slider-' + id);
      const currentEl = $('steer-current-' + id);
      const targetEl = $('steer-target-' + id);
      const stateEl = $('steer-state-' + id);
      const sendButton = panel.querySelector('[data-steer-send="' + id + '"]');
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

      if (pending && current != null && !busy && Math.abs(current - pending.angle) <= 1) {
        delete pendingSteer[id];
        pending = null;
      } else if (pending && !busy && now - pending.sentAt > 15000) {
        delete pendingSteer[id];
        pending = null;
      }

      let value = pending ? pending.angle : (moving && target != null ? target : (current == null ? 0 : current));
      value = Math.max(-32, Math.min(32, Math.round(value)));
      const dirty = card.dataset.steerDirty === 'true';
      if (!dirty) slider.value = String(value);
      if (currentEl) currentEl.textContent = current == null ? '—' : Math.round(current) + '°';
      if (targetEl) targetEl.textContent = Math.round(+slider.value || 0) + '°';

      const ready = available && steeringAuthorized && online && homed && !busy && !pending;
      slider.disabled = !ready;
      card.querySelectorAll('[data-steer-adjust="' + id + '"], [data-steer-zero="' + id + '"]').forEach((button) => {
        button.disabled = !ready;
      });
      if (sendButton) sendButton.disabled = !ready || !dirty;
      card.dataset.ready = String(ready);

      let label;
      if (!available) label = 'нет управления';
      else if (!steeringAuthorized) label = 'нужен PIN';
      else if (!online) label = 'нет связи';
      else if (calibrating) label = 'калибровка';
      else if (!homed) label = 'нужна калибровка';
      else if (moving) label = 'поворачивается';
      else if (pending) label = 'команда отправлена';
      else label = 'готов';

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
  }

  function initSteeringControls() {
    const panel = $('swipe-motors-view');
    if (!panel) return;
    const ids = ['FL', 'FR', 'RL', 'RR'];
    const unlockButton = $('steer-unlock');
    const lockButton = $('steer-lock');

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
      slider.value = String(Math.max(-32, Math.min(32, Math.round(value))));
      card.dataset.steerDirty = 'true';
      delete steerResult[id];
      if (output) output.textContent = Math.round(+slider.value) + '°';
      renderSteeringControls(lastData || {});
    }

    function errorText(reason) {
      const messages = {
        pin_required: 'PIN истёк · разблокируйте',
        control_not_supported: 'источник не поддерживает управление',
        module_offline: 'нет свежей связи',
        serial_unavailable: 'порт мастера недоступен',
        not_homed: 'сначала нужна калибровка',
        module_busy: 'дождитесь остановки модуля',
        angle_limit: 'угол за пределами ±32°',
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
      const angle = Math.max(-32, Math.min(32, Number(slider.value)));
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

    ids.forEach((id) => {
      const slider = $('steer-slider-' + id);
      const card = panel.querySelector('[data-steer-card="' + id + '"]');
      if (slider) slider.addEventListener('input', () => markDirty(id, slider.value));
      panel.querySelectorAll('[data-steer-adjust="' + id + '"]').forEach((button) => {
        button.addEventListener('click', () => markDirty(id, Number(slider.value) + Number(button.dataset.step)));
      });
      const zero = panel.querySelector('[data-steer-zero="' + id + '"]');
      if (zero) zero.addEventListener('click', () => markDirty(id, 0));
      const send = panel.querySelector('[data-steer-send="' + id + '"]');
      if (send) send.addEventListener('click', () => sendSteer(id));
      if (card) card.dataset.steerDirty = 'false';
    });

    if (unlockButton) unlockButton.addEventListener('click', () => {
      if (window._openSteeringAuth) window._openSteeringAuth();
    });
    if (lockButton) lockButton.addEventListener('click', async () => {
      try {
        await fetch('api/kiosk_lock/session/end', {
          method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}'
        });
      } catch (error) {}
      setSteeringAuthorized(false);
      ids.forEach((id) => { delete pendingSteer[id]; delete steerResult[id]; });
      renderSteeringControls(lastData || {});
    });
    renderSteeringControls(lastData || {});
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot);
  else boot();
})();
