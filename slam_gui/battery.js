/* Страница 4 бортового экрана — аккумулятор JBD BMS: всё, что отдаёт плата
   (заряд, ток с направлением, энергия, время до разряда/заряда, ключи, защиты,
   счётчики срабатываний, температуры, паспорт, пороги защит), карта всех банок
   и подключение BMS прямо с экрана. Из BMS только читаем, никаких записей. */
(function () {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const page = $('swipe-battery-view');
  if (!page) return;

  const LFP_MIN_MV = 2500;          // шкала банки LiFePO4, как в ploter_a1
  const LFP_MAX_MV = 3650;
  const SPREAD_WARN_MV = 30;        // разброс выше — внимание
  const CELL_HIGHLIGHT_MV = 20;     // отклонение от среднего — подсветка банки
  const TEMP_WARN_C = 45;           // PACK.tempWarnC / tempMaxC в backend.py
  const TEMP_MAX_C = 60;
  const SETUP_POLL_MS = 4000;

  let snapshot = null;
  let receivedAt = 0;
  let transportOk = false;
  let setupOpen = false;
  let setupData = null;
  let setupHtmlKey = '';
  let selectedPort = '';
  let setupBusy = false;
  let setupTimer = null;
  let setupAutoOpened = false;

  const text = (id, value) => { const el = $(id); if (el) el.textContent = value; };
  const fmt = (value, decimals) => value == null || !Number.isFinite(Number(value)) ? '—' : Number(value).toFixed(decimals);
  const signed = (value, decimals) => {
    if (value == null || !Number.isFinite(Number(value))) return '—';
    const text = Math.abs(Number(value)).toFixed(decimals);
    if (Number(text) === 0) return text;           // −0.2 мВ → «0», без «−0»
    return (Number(value) > 0 ? '+' : '−') + text;
  };

  function make(tag, className, content) {
    const node = document.createElement(tag);
    if (className) node.className = className;
    if (content != null) node.textContent = content;
    return node;
  }

  function hours(value) {
    if (value == null || !Number.isFinite(Number(value))) return '—';
    const total = Math.round(Number(value) * 60);
    if (total >= 60 * 100) return '> 99 ч';
    return Math.floor(total / 60) + ' ч ' + String(total % 60).padStart(2, '0') + ' мин';
  }

  function fact(list, label, value, tone) {
    const row = make('div', tone ? 'tone-' + tone : '');
    row.append(make('dt', '', label), make('dd', '', value));
    list.append(row);
  }

  // ------------------------------------------------------------- телеметрия
  function render() {
    window._kioskBms = transportOk ? snapshot : (snapshot ? Object.assign({}, snapshot, { connected: false }) : null);
    const data = snapshot && snapshot.data;
    const fresh = transportOk && Date.now() - receivedAt < 5000 && snapshot && snapshot.connected;
    const available = Boolean(fresh && data);
    const configured = Boolean(snapshot && snapshot.configured);
    page.classList.toggle('bms-stale', !available);
    page.classList.toggle('bms-unconfigured', transportOk && !configured);
    const reason = !transportOk ? 'Нет связи с сервером BMS' : snapshot.reason;
    text('bms-status', available || (transportOk && !configured) ? reason : reason + ' · показания неактуальны');
    text('bms-updated', data
      ? 'Измерено: ' + new Date(data.updatedAt * 1000).toLocaleString('ru-RU') + (available ? '' : ' · последние известные данные')
      : (transportOk && !configured
        ? 'BMS ещё не подключена: нажмите «Подключение BMS», выберите порт и «Сохранить и подключить».'
        : 'Нет измерений: проверьте кабель USB-UART и питание BMS.'));
    const report = $('bms-report');
    report.setAttribute('aria-disabled', available ? 'false' : 'true');
    if (available) report.setAttribute('href', 'api/battery/report.csv');
    else report.removeAttribute('href');
    const live = $('swipe-live');
    if (!page.hidden && live) live.textContent = available ? (snapshot.complete ? '● BMS' : 'НЕ ВСЕ БАНКИ') : 'НЕТ ДАННЫХ BMS';
    const setupButton = $('bms-setup-open');
    if (setupButton) setupButton.classList.toggle('attention', transportOk && (!configured || !available));
    if (transportOk && !configured && !page.hidden && !setupAutoOpened && snapshot.setupAvailable) {
      setupAutoOpened = true;
      openSetup(true);
    }

    renderHero(data);
    renderSummary(data);
    renderProtection(data);
    renderTemperatures(data);
    renderPassport(data);
    renderThresholds(data);
    renderCells(data);
  }

  function renderHero(data) {
    const soc = data && Number.isFinite(Number(data.soc)) ? Math.max(0, Math.min(100, Number(data.soc))) : null;
    text('bms-soc', soc == null ? '—' : String(Math.round(soc)));
    const bar = $('bms-soc-bar');
    if (bar && bar.style) bar.style.width = soc == null ? '0%' : soc + '%';
    text('bms-capacity', data
      ? 'остаток ' + fmt(data.remainingAh, 2) + ' из ' + fmt(data.fullAh, 2) + ' Ач'
      : 'остаток — из — Ач');
  }

  function renderSummary(data) {
    const summary = $('bms-summary');
    summary.replaceChildren();
    const d = data || {};
    const direction = d.directionText ? ' · ' + d.directionText : '';
    const timeLabel = d.direction === 'charge' ? 'До полного заряда' : 'До разряда';
    const timeValue = d.direction === 'charge' ? hours(d.hoursToFull)
      : (d.direction === 'discharge' ? hours(d.hoursToEmpty) : (data ? 'ток почти 0' : '—'));
    const metrics = [
      ['Напряжение', fmt(d.volts, 2) + ' В', data && Number.isFinite(Number(d.sumVolts)) ? 'сумма банок ' + fmt(d.sumVolts, 2) + ' В' : ''],
      ['Ток (+ заряд / − разряд)', signed(d.amps, 2) + ' А', data ? direction.replace(' · ', '') : ''],
      ['Мощность', signed(d.watts, 0) + ' Вт', ''],
      ['Энергия', data ? '≈ ' + fmt(d.remainingWh, 0) + ' / ' + fmt(d.fullWh, 0) + ' Вт·ч' : '— Вт·ч', 'по ёмкости BMS и текущему напряжению'],
      [timeLabel, timeValue, data && d.direction !== 'idle' ? 'при нынешнем токе' : ''],
      ['Циклы', fmt(d.cycles, 0), '']
    ];
    metrics.forEach(([label, value, note]) => {
      const item = make('div');
      item.append(make('span', '', label), make('b', '', value));
      if (note) item.append(make('small', '', note));
      summary.append(item);
    });
  }

  function renderProtection(data) {
    const fets = $('bms-fets');
    if (fets) {
      fets.replaceChildren();
      if (data) {
        fets.append(make('span', data.chargeOn ? 'pill on' : 'pill off', 'Ключ заряда: ' + (data.chargeOn ? 'вкл' : 'выкл')));
        fets.append(make('span', data.dischargeOn ? 'pill on' : 'pill off', 'Ключ разряда: ' + (data.dischargeOn ? 'вкл' : 'выкл')));
        const balancing = Array.isArray(data.balancingCells) ? data.balancingCells : [];
        fets.append(make('span', balancing.length ? 'pill info' : 'pill', balancing.length
          ? 'Балансировка: банки ' + balancing.join(', ') : 'Балансировка: нет'));
      } else {
        fets.append(make('span', 'pill', 'Ключи: нет данных'));
      }
    }
    text('bms-faults', data ? (data.faults.length ? 'Защиты: ' + data.faults.join(' · ') : 'Активных защит нет') : 'Защиты: нет данных');
    $('bms-faults').classList.toggle('bms-warning', Boolean(data && data.faults.length));
    $('bms-faults').classList.toggle('bms-ok', Boolean(data && !data.faults.length));

    const counters = $('bms-counters');
    if (!counters) return;
    counters.replaceChildren();
    if (!data) return;
    const list = Array.isArray(data.errorCounters) ? data.errorCounters : null;
    if (!list) {
      counters.append(make('p', 'panel-meta', data.errorCountersState === 'unavailable'
        ? 'Счётчики срабатываний: плата их не отдаёт (регистр 0xAA).'
        : 'Счётчики срабатываний: читаю…'));
      return;
    }
    const hits = list.filter((item) => Number(item.count) > 0);
    counters.append(make('p', 'panel-meta', hits.length
      ? 'Счётчики срабатываний за жизнь платы:' : 'Счётчики срабатываний: защиты ни разу не срабатывали.'));
    hits.forEach((item) => counters.append(make('span', 'pill warn', item.label + ': ' + item.count)));
  }

  function renderTemperatures(data) {
    const box = $('bms-temps');
    if (!box) return;
    box.replaceChildren();
    if (!data) {
      box.append(make('span', 'pill', 'Нет данных'));
      return;
    }
    if (!data.temperatures.length) {
      box.append(make('span', 'pill', 'Датчиков температуры нет'));
      return;
    }
    data.temperatures.forEach((value, index) => {
      const tone = value >= TEMP_MAX_C ? 'crit' : (value >= TEMP_WARN_C ? 'warn' : 'on');
      box.append(make('span', 'pill ' + tone, 'Датчик ' + (index + 1) + ': ' + fmt(value, 1) + ' °C'));
    });
  }

  function renderPassport(data) {
    const list = $('bms-passport');
    if (!list) return;
    list.replaceChildren();
    const d = data || {};
    const name = d.deviceName || (data ? (d.deviceNameState === 'unavailable' ? 'плата не отдаёт имя' : 'читаю…') : '—');
    fact(list, 'Устройство', name);
    fact(list, 'Дата выпуска', d.manufactureDateText || (data ? 'не указана' : '—'));
    fact(list, 'Прошивка BMS', d.firmwareText || '—');
    fact(list, 'Банок', data ? d.receivedCount + ' получено / ' + d.cellCount + ' заявлено' : '—', data && !d.complete ? 'warn' : '');
    fact(list, 'Датчиков температуры', data ? String(d.ntcCount) : '—');
    fact(list, 'Циклов заряда', fmt(d.cycles, 0));
    const design = Array.isArray(d.thresholds) ? d.thresholds.find((item) => item.key === 'design_cap') : null;
    fact(list, 'Ёмкость: полная / номинальная', data
      ? fmt(d.fullAh, 2) + ' / ' + (design && design.value != null ? fmt(design.value, 2) : '—') + ' Ач' : '—');
    fact(list, 'Порт', (snapshot && snapshot.port) || 'не выбран');
    fact(list, 'На связи с', snapshot && snapshot.connectedSince
      ? new Date(snapshot.connectedSince * 1000).toLocaleTimeString('ru-RU') : '—');
  }

  const THRESHOLD_PAIRS = [
    ['covp', 'covp_rel'], ['cuvp', 'cuvp_rel'], ['povp', 'povp_rel'], ['puvp', 'puvp_rel'],
    ['chgoc', null], ['dsgoc', null], ['chgot', 'chgot_rel'], ['chgut', 'chgut_rel'],
    ['dsgot', 'dsgot_rel'], ['dsgut', 'dsgut_rel'], ['bal_start', null], ['bal_window', null],
    ['design_cap', null], ['cell_cnt', null]
  ];

  function thresholdText(item) {
    if (!item || item.value == null) return 'нет данных';
    const decimals = item.unit === 'В' ? (Math.abs(item.value) >= 10 ? 2 : 3)
      : (item.unit === 'шт' || item.unit === 'мВ' ? 0 : (item.unit === '°C' ? 1 : 2));
    return fmt(item.value, decimals) + ' ' + item.unit;
  }

  function renderThresholds(data) {
    const list = $('bms-thresholds');
    if (!list) return;
    list.replaceChildren();
    const state = data ? data.thresholdsState : null;
    const items = data && Array.isArray(data.thresholds) ? data.thresholds : [];
    const byKey = Object.create(null);
    items.forEach((item) => { byKey[item.key] = item; });
    text('bms-thresholds-state', !data ? 'Нет данных от BMS.'
      : state === 'reading' ? 'Читаю пороги из памяти BMS (по два регистра за опрос, телеметрия не тормозит)…'
        : state === 'unavailable' ? 'Плата не отдаёт пороги обычным чтением: часть плат JBD показывает их только в заводском режиме, а вход в него — запись. Приложение в BMS не пишет, поэтому пороги здесь не показываются.'
          : state === 'partial' ? 'Часть порогов плата не отдала — они помечены «нет данных».'
            : 'Значения прочитаны из BMS и только показываются; изменить их отсюда нельзя.');
    THRESHOLD_PAIRS.forEach(([key, release]) => {
      const item = byKey[key];
      if (!item) return;
      let value = thresholdText(item);
      if (release && byKey[release] && byKey[release].value != null) value += ' · возврат ' + thresholdText(byKey[release]);
      fact(list, item.label, value, item.value == null ? 'muted' : '');
    });
  }

  function renderCells(data) {
    const spread = data && Number(data.deltaMv) > SPREAD_WARN_MV;
    text('bms-spread', data
      ? 'Мин: ' + fmt(data.minMv / 1000, 3) + ' В (банка ' + data.minCell + ') · макс: ' + fmt(data.maxMv / 1000, 3) +
        ' В (банка ' + data.maxCell + ') · разброс: ' + data.deltaMv + ' мВ' + (spread ? ' — больше ' + SPREAD_WARN_MV + ' мВ' : '') +
        ' · среднее: ' + fmt(data.averageMv / 1000, 3) + ' В · сумма: ' + fmt(data.sumVolts, 2) + ' В'
      : 'Разброс: —');
    $('bms-spread').classList.toggle('bms-warning', Boolean(spread));
    text('bms-count', data ? 'Получено банок: ' + data.receivedCount + ' / ' + data.cellCount + (data.complete ? ' · полный отчёт' : ' · ВНИМАНИЕ: число банок не совпадает, статистика только по полученным') : 'Банки: нет данных');

    const tiles = $('bms-cell-tiles');
    if (tiles) {
      tiles.replaceChildren();
      (data ? data.cells : []).forEach((bank) => {
        const tile = make('div', 'bms-tile');
        const tags = [];
        if (bank.mv == null) tags.push('missing');
        else {
          if (bank.mv === data.minMv) tags.push('min');
          if (bank.mv === data.maxMv) tags.push('max');
          if (bank.deviationMv != null && bank.deviationMv <= -CELL_HIGHLIGHT_MV) tags.push('low');
          if (bank.deviationMv != null && bank.deviationMv >= CELL_HIGHLIGHT_MV) tags.push('high');
        }
        if (bank.balancing) tags.push('balancing');
        tile.className = 'bms-tile' + tags.map((tag) => ' is-' + tag).join('');
        const fill = make('i', 'bms-tile-fill');
        const level = bank.mv == null ? 0 : Math.max(0, Math.min(100, (bank.mv - LFP_MIN_MV) / (LFP_MAX_MV - LFP_MIN_MV) * 100));
        if (fill.style) fill.style.height = level.toFixed(0) + '%';
        tile.append(fill, make('span', 'bms-tile-no', String(bank.number)),
          make('b', '', bank.mv == null ? '—' : fmt(bank.mv / 1000, 3)),
          make('small', '', bank.mv == null ? 'нет данных' : signed(bank.deviationMv, 0) + ' мВ' + (bank.balancing ? ' · баланс' : '')));
        tiles.append(tile);
      });
    }

    const body = $('bms-cells');
    body.replaceChildren();
    if (!data || !data.cells.length) {
      const row = document.createElement('tr'); const cell = document.createElement('td');
      cell.colSpan = 5; cell.textContent = 'Нет данных от BMS'; row.append(cell); body.append(row);
      return;
    }
    data.cells.forEach((bank) => {
      const row = document.createElement('tr');
      const tags = [];
      if (bank.mv == null) tags.push('НЕТ ДАННЫХ');
      else {
        if (bank.mv === data.minMv) tags.push('МИН');
        if (bank.mv === data.maxMv) tags.push('МАКС');
      }
      if (bank.number > data.cellCount) tags.push('СВЕРХ ЗАЯВЛЕННОГО ЧИСЛА');
      if (tags.length) row.className = 'bms-extreme';
      [String(bank.number).padStart(2, '0'), bank.mv == null ? '—' : fmt(bank.mv / 1000, 3) + ' В · ' + bank.mv + ' мВ',
        fmt(bank.deviationMv, 1) + ' мВ', bank.balancing ? 'Активна' : 'Нет', tags.join(' · ') || '—'
      ].forEach((value) => { const cell = document.createElement('td'); cell.textContent = value; row.append(cell); });
      body.append(row);
    });
  }

  async function poll() {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), 2500);
    try {
      const response = await fetch('api/battery', { cache: 'no-store', signal: controller.signal });
      if (!response.ok) throw new Error('http ' + response.status);
      const result = await response.json();
      if (!result.ok || !result.battery) throw new Error('invalid battery response');
      snapshot = result.battery;
      receivedAt = Date.now();
      transportOk = true;
    } catch (error) {
      transportOk = false;
    } finally {
      clearTimeout(timer);
      render();
      setTimeout(poll, 1000);
    }
  }

  // ------------------------------------------------- подключение BMS (порт)
  const ROLE_TEXT = {
    bms: 'BMS', master: 'мастер руля', lidar: 'лидар', chassis: 'шасси',
    busy: 'занят', arduino: 'Arduino?', free: 'свободен'
  };

  function setupMessage(message, tone) {
    const el = $('bms-setup-msg');
    if (!el) return;
    el.textContent = message || '';
    if (el.dataset) el.dataset.tone = tone || '';
  }

  function renderSetup() {
    const list = $('bms-port-list');
    if (!list || !setupData) return;
    text('bms-setup-current', setupData.port
      ? 'сейчас: ' + setupData.port + (setupData.connected ? ' · на связи' : ' · нет ответа')
      : 'порт не выбран');
    const ports = Array.isArray(setupData.ports) ? setupData.ports : [];
    if (!selectedPort) {
      const preferred = ports.find((item) => item.role === 'bms') || ports.find((item) => item.role === 'free');
      if (preferred) selectedPort = preferred.port;
    }
    const key = JSON.stringify([ports, selectedPort, setupBusy]);
    if (key === setupHtmlKey) return;
    setupHtmlKey = key;
    list.replaceChildren();
    if (!ports.length) {
      list.append(make('li', 'empty', 'USB/serial-порты не найдены: подключите переходник BMS к USB и подождите пару секунд.'));
      return;
    }
    ports.forEach((item) => {
      const locked = ['master', 'lidar', 'chassis'].indexOf(item.role) >= 0;
      const row = make('li', 'role-' + item.role + (item.port === selectedPort ? ' selected' : ''));
      const label = make('label');
      const radio = document.createElement('input');
      radio.type = 'radio';
      radio.name = 'bms-port';
      radio.value = item.port;
      radio.checked = item.port === selectedPort;
      radio.disabled = locked || setupBusy;
      radio.addEventListener('change', () => {
        selectedPort = item.port;
        setupHtmlKey = '';
        renderSetup();
      });
      const name = make('span', 'port-name', item.port);
      const meta = make('small', '', [item.description, item.note].filter(Boolean).join(' · '));
      label.append(radio, name, make('span', 'port-role', ROLE_TEXT[item.role] || item.role), meta);
      row.append(label);
      list.append(row);
    });
  }

  function setBusy(busy) {
    setupBusy = busy;
    ['bms-probe', 'bms-scan', 'bms-save', 'bms-disable'].forEach((id) => {
      const button = $(id);
      if (button) button.disabled = busy || (setupData && setupData.available === false);
    });
    setupHtmlKey = '';
    renderSetup();
  }

  async function loadSetup() {
    try {
      const response = await fetch('api/battery/setup', { cache: 'no-store' });
      const result = await response.json();
      setupData = result;
      if (result.available === false) {
        setupMessage(result.reason || 'Подключение BMS настраивается на бортовом экране.', 'warn');
        setBusy(false);
      }
      renderSetup();
    } catch (error) {
      setupMessage('Сервер не отвечает — список портов не получен.', 'error');
    }
  }

  async function post(url, body) {
    const response = await fetch(url, {
      method: 'POST', cache: 'no-store', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {})
    });
    const result = await response.json().catch(() => ({}));
    if (!response.ok) throw new Error(result.error || ('Ошибка HTTP ' + response.status));
    return { response, result };
  }

  async function probeSelected() {
    if (!selectedPort) return setupMessage('Выберите порт в списке.', 'warn');
    setBusy(true);
    setupMessage('Проверяю ' + selectedPort + '…', '');
    try {
      const { result } = await post('api/battery/probe', { port: selectedPort });
      if (result.found) setupMessage('Найдена BMS: ' + (result.summary || selectedPort) + '. Нажмите «Сохранить и подключить».', 'ok');
      else setupMessage(selectedPort + ': ' + (result.error || 'JBD BMS не ответила') + '.', 'error');
    } catch (error) {
      setupMessage(error.message, 'error');
    } finally {
      setBusy(false);
    }
  }

  async function scanPorts() {
    setBusy(true);
    setupMessage('Ищу BMS на свободных портах (мастер, лидар и Arduino пропускаются)…', '');
    try {
      const { result } = await post('api/battery/scan', {});
      const found = Array.isArray(result.found) ? result.found : [];
      if (found.length) {
        selectedPort = found[0];
        setupMessage('BMS найдена: ' + found.join(', ') + '. Проверьте выбор и нажмите «Сохранить и подключить».', 'ok');
      } else {
        const tried = Array.isArray(result.results) ? result.results.length : 0;
        setupMessage('BMS не найдена: проверено портов ' + tried + '. Проверьте кабель TX/RX/GND и питание BMS.', 'error');
      }
    } catch (error) {
      setupMessage(error.message, 'error');
    } finally {
      setBusy(false);
      void loadSetup();
    }
  }

  async function savePort(port) {
    setBusy(true);
    setupMessage(port ? 'Сохраняю ' + port + ' и подключаюсь…' : 'Отключаю BMS…', '');
    try {
      const { response, result } = await post('api/battery/port', { port });
      if (!response.ok || !result.ok) throw new Error(result.error || 'Не удалось сохранить порт.');
      setupMessage(port ? 'Сохранено в .env (RC_BMS_PORT). Подключение к BMS — данные появятся через пару секунд.'
        : 'BMS отключена, RC_BMS_PORT очищен.', 'ok');
    } catch (error) {
      setupMessage(error.message, 'error');
    } finally {
      setBusy(false);
      void loadSetup();
    }
  }

  function openSetup(open) {
    setupOpen = open;
    const block = $('bms-setup');
    if (block) block.hidden = !open;
    const button = $('bms-setup-open');
    if (button) button.setAttribute('aria-expanded', open ? 'true' : 'false');
    if (setupTimer) clearInterval(setupTimer);
    setupTimer = null;
    if (!open) return;
    void loadSetup();
    setupTimer = setInterval(() => { if (!page.hidden && setupOpen && !setupBusy) void loadSetup(); }, SETUP_POLL_MS);
  }

  function bind(id, handler) {
    const el = $(id);
    if (el && el.addEventListener) el.addEventListener('click', handler);
  }
  bind('bms-setup-open', () => openSetup(!setupOpen));
  bind('bms-probe', () => { void probeSelected(); });
  bind('bms-scan', () => { void scanPorts(); });
  bind('bms-save', () => {
    if (!selectedPort) return setupMessage('Выберите порт в списке.', 'warn');
    void savePort(selectedPort);
  });
  bind('bms-disable', () => {
    if (window.confirm && !window.confirm('Отключить чтение BMS и очистить RC_BMS_PORT в .env?')) return;
    void savePort('');
  });

  // Switching views must update the LIVE marker even between polls.
  new MutationObserver(render).observe(page, { attributes: true, attributeFilter: ['hidden'] });
  window.RCBattery = { openSetup: () => openSetup(true), refresh: render };
  poll();
})();
