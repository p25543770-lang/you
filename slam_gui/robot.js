/* Бортовой экран робота (киоск): АКБ + шасси + лидар. Без управления. */
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
    if (moving) parts.push('в движении ' + moving);
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
        if (m.moving) wheel.classList.add('is-driving');
        else wheel.classList.remove('is-driving');
      }
      const chip = $('mod-state-' + id);
      if (chip) { chip.textContent = modStateText(m); chip.className = 'chip ' + tone; }
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
    ctx.strokeStyle = '#2b4037';
    ctx.lineWidth = 1.5;
    for (let m = LIDAR_RING_M; m <= LIDAR_RANGE_M; m += LIDAR_RING_M) {
      ctx.beginPath();
      ctx.arc(cx, cy, m * k, 0, Math.PI * 2);
      ctx.stroke();
    }
    // подписи метров
    ctx.fillStyle = '#5a7266';
    ctx.font = '12px ' + getComputedStyle(document.body).fontFamily;
    for (let m = LIDAR_RING_M; m <= LIDAR_RANGE_M; m += LIDAR_RING_M) {
      ctx.fillText(m + ' м', cx + 4, cy - m * k - 4);
    }
    // оси
    ctx.strokeStyle = '#3b5549';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R);
    ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy);
    ctx.stroke();
    // вперёд (ось Y-) пометим
    ctx.fillStyle = '#9fe36b';
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
        if (p.r === near && near < 1.0) ctx.fillStyle = '#ff6b6b';
        else if (p.r === near && near < 2.0) ctx.fillStyle = '#ffb84d';
        else ctx.fillStyle = '#9fe36b';
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
  }

  function loop() {
    render();
    requestAnimationFrame(() => setTimeout(loop, 250));
  }

  function boot() {
    fetchState();
    setInterval(fetchState, POLL_MS);
    loop();
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot);
  else boot();
})();
