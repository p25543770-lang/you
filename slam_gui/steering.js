/* PIN-защищённая адресная панель поворота для веб-интерфейса оператора. */
(function () {
  'use strict';

  if (window.RCSteeringPanel) return;
  const IDS = ['FL', 'FR', 'RL', 'RR'];
  let authorized = false;
  let snapshot = null;
  let pollTimer = null;
  let driveTimer = null;
  let driveDirection = 0;
  let driveQueued = null;
  let driveSending = false;
  let driveFlush = Promise.resolve();
  const pending = Object.create(null);
  const feedback = Object.create(null);
  const $ = (id) => document.getElementById(id);

  function addUi() {
    if ($('rc-steering-launch')) return;
    const launch = document.createElement('button');
    launch.id = 'rc-steering-launch';
    launch.className = 'rc-steering-launch';
    launch.type = 'button';
    launch.textContent = 'Рулевое';
    launch.title = 'Поворот рулевых модулей';

    const overlay = document.createElement('div');
    overlay.id = 'rc-steering-overlay';
    overlay.className = 'rc-steering-overlay';
    overlay.hidden = true;
    overlay.innerHTML = `
      <section class="rc-steering-dialog" role="dialog" aria-modal="true" aria-labelledby="rc-steering-title">
        <header class="rc-steering-head">
          <div><h2 id="rc-steering-title">Управление приводами</h2>
            <p>Рулевые шаговые моторы + удерживаемое низкоскоростное движение BLDC</p></div>
          <button type="button" class="rc-steering-close" aria-label="Закрыть">×</button>
        </header>
        <div class="rc-steering-safety" role="note"><b>Ноль руля — по оптодатчику D18:</b> «Калибровка» поворачивает колесо (до 100° вправо и 200° влево) и ставит его в середину флажка. «Задать 0°» — запасной ручной вариант без датчика. Ноль теряется после перезапуска Arduino.</div>
        <div class="rc-steering-toolbar">
          <span id="rc-steering-auth">Нужен PIN</span>
          <button type="button" class="rc-steering-btn" id="rc-steering-unlock">Разблокировать</button>
          <button type="button" class="rc-steering-btn" id="rc-steering-lock" hidden>Заблокировать</button>
          <span id="rc-steering-source" role="status">Проверка связи…</span>
        </div>
        <section class="rc-drive-panel" aria-label="Тяговые моторы">
          <div class="rc-drive-top"><div><b>Тяга BLDC</b><small>Зажмите направление; отпустите для торможения</small></div>
            <span id="rc-drive-status" role="status">Нужен ноль всех колёс (калибровка)</span></div>
          <label class="rc-drive-speed">Ограничение скорости <b><span id="rc-drive-speed-value">8</span>%</b>
            <input id="rc-drive-speed" type="range" min="1" max="20" value="8" disabled></label>
          <div class="rc-drive-actions">
            <button type="button" data-drive-direction="-1" disabled>Зажать: назад</button>
            <button type="button" data-drive-direction="1" disabled>Зажать: вперёд</button>
            <button type="button" id="rc-drive-stop" class="stop" disabled>СТОП</button>
          </div>
          <small class="rc-drive-warning">Программный стоп и watchdog (650 мс), не аварийный выключатель. Перед запуском вывесьте колёса и установите аппаратную аварийную кнопку в силовую цепь.</small>
        </section>
        <div class="rc-steering-grid">${IDS.map((id) => `
          <article class="rc-steering-card" data-rc-steering="${id}">
            <header><b>${id}</b><span data-status>нет связи</span></header>
            <div class="rc-steering-values"><span>Сейчас <b data-current>—</b></span><span>Цель <b data-target>0°</b></span></div>
            <input data-range type="range" min="-85" max="85" step="1" value="0" aria-label="${id}, целевой угол" disabled>
            <div class="rc-steering-actions">
              <button type="button" data-calib disabled>Калибровка</button>
              <button type="button" data-origin disabled>Задать 0°</button>
              <button type="button" data-step="-5" disabled>−5°</button>
              <button type="button" data-center disabled>0°</button>
              <button type="button" data-step="5" disabled>+5°</button>
              <button type="button" class="primary" data-send disabled>Повернуть</button>
            </div>
            <small data-message aria-live="polite"></small>
          </article>`).join('')}
        </div>
      </section>`;
    const header = document.querySelector('.top-status, .head-actions');
    if (header) header.append(launch);
    else document.body.append(launch);
    document.body.append(overlay);

    launch.addEventListener('click', () => {
      overlay.hidden = false;
      void refresh();
      if (pollTimer) clearInterval(pollTimer);
      pollTimer = setInterval(() => void refresh(), 500);
    });
    overlay.querySelector('.rc-steering-close').addEventListener('click', close);
    overlay.addEventListener('click', (event) => { if (event.target === overlay) close(); });
    overlay.querySelector('#rc-steering-unlock').addEventListener('click', unlock);
    overlay.querySelector('#rc-steering-lock').addEventListener('click', lock);
    IDS.forEach((id) => bindCard(id));
    bindDriveControls();
    window.addEventListener('pointerup', releaseDrive);
    window.addEventListener('pointercancel', releaseDrive);
    window.addEventListener('blur', releaseDrive);
    document.addEventListener('visibilitychange', () => { if (document.hidden) releaseDrive(); });
  }

  function close() {
    const overlay = $('rc-steering-overlay');
    if (overlay) overlay.hidden = true;
    if (pollTimer) clearInterval(pollTimer);
    pollTimer = null;
    releaseDrive();
    void lock();
  }

  async function json(url, init) {
    const response = await fetch(url, { cache: 'no-store', ...init });
    const body = await response.json().catch(() => ({}));
    return { response, body };
  }

  async function refresh() {
    const overlay = $('rc-steering-overlay');
    if (!overlay || overlay.hidden) return;
    try {
      const [{ body: state }, { body: auth }] = await Promise.all([
        json('api/kiosk_state'), json('api/kiosk_lock')
      ]);
      if (!state.ok || !state.data) throw new Error('Нет телеметрии');
      snapshot = state.data;
      authorized = !!auth.steeringAuthorized;
      $('rc-steering-auth').textContent = authorized ? 'PIN подтверждён' : 'Нужен PIN';
      $('rc-steering-unlock').hidden = authorized;
      $('rc-steering-lock').hidden = !authorized;
      const control = snapshot.steeringControl || {};
      const source = $('rc-steering-source');
      source.textContent = !control.available
        ? 'Управление доступно при RC_SLAM_SOURCE=master и подключённом USB мастера.'
        : snapshot.linkOk === false
          ? 'Мастер выбран, но свежей телеметрии пока нет.'
          : control.calibration
            ? 'Мастер на связи · откалибруйте колёса по датчику перед рулением и тягой.'
            : 'Мастер на связи · задайте ручной ноль перед рулением и тягой.';
      IDS.forEach((id) => renderCard(id));
      renderDrive();
    } catch (error) {
      $('rc-steering-source').textContent = error && error.message ? error.message : 'Нет связи с сервером.';
      IDS.forEach((id) => renderCard(id));
      renderDrive();
    }
  }

  function renderCard(id) {
    const card = document.querySelector('[data-rc-steering="' + id + '"]');
    if (!card) return;
    const motor = snapshot && Array.isArray(snapshot.motors)
      ? snapshot.motors.find((item) => item && item.id === id) : null;
    const control = (snapshot && snapshot.steeringControl) || {};
    const online = !!(motor && motor.online === true && Number.isFinite(+motor.angle));
    const zeroed = !!(motor && motor.homed === true);
    const calibrating = !!(motor && motor.calibrating);
    const moving = !!(motor && (motor.moving || motor.calibrating));
    const homeSource = motor && motor.homeSource ? String(motor.homeSource) : (zeroed ? 'manual' : 'none');
    const drive = (snapshot && snapshot.drive) || {};
    const tractionActive = !!drive.active || Number(drive.commandedPercent || 0) !== 0;
    const current = online ? Number(motor.angle) : null;
    const target = online && Number.isFinite(+motor.targetAngle) ? Number(motor.targetAngle) : current;
    const entry = pending[id];
    if (entry && entry.calib && calibrating) entry.started = true;
    if (entry && ((entry.zero && zeroed) || (entry.calib && entry.started && !calibrating) ||
        (!entry.zero && !entry.calib && current != null &&
        Math.abs(current - Number(entry.angle)) <= 1 && !moving))) delete pending[id];
    const activePending = pending[id] && Date.now() - pending[id].at < 15000;
    if (pending[id] && !activePending) delete pending[id];
    const ready = !!control.available && authorized && online && zeroed && !moving && !activePending && !tractionActive;
    const canZero = !!control.available && authorized && online && !moving && !activePending && !tractionActive;
    const range = card.querySelector('[data-range]');
    if (!card.dataset.dirty && current != null) range.value = String(Math.round(target));
    card.querySelector('[data-current]').textContent = current == null ? '—' : current.toFixed(1) + '°';
    card.querySelector('[data-target]').textContent = Math.round(+range.value || 0) + '°';
    range.disabled = !ready;
    card.querySelectorAll('[data-step], [data-center]').forEach((button) => { button.disabled = !ready; });
    card.querySelector('[data-send]').disabled = !ready || !card.dataset.dirty;
    card.querySelector('[data-origin]').disabled = !canZero;
    const calibButton = card.querySelector('[data-calib]');
    calibButton.disabled = !canZero || !control.calibration;
    calibButton.hidden = !!control.available && !control.calibration;
    const status = card.querySelector('[data-status]');
    status.textContent = !control.available ? 'нет master' : !authorized ? 'нужен PIN'
      : !online ? 'нет связи' : calibrating ? 'калибровка по датчику…' : moving ? 'движется'
        : tractionActive ? 'тяга включена · стоп' : activePending ? 'команда в очереди'
          : !zeroed ? (motor && motor.calibErrorText ? 'ошибка калибровки: ' + motor.calibErrorText
            : control.calibration ? 'нужна калибровка' : 'нужен ручной ноль')
            : homeSource === 'opto' ? 'готов · ноль по датчику' : 'готов · ноль вручную';
    const msg = feedback[id];
    card.querySelector('[data-message]').textContent = msg && msg.until > Date.now() ? msg.text : '';
    card.classList.toggle('error', !!(msg && msg.error && msg.until > Date.now()));
  }

  const DRIVE_REASON_TEXT = {
    serial_unavailable: 'Нет связи с USB-мастером.',
    module_offline: 'Нет свежей телеметрии всех четырёх модулей.',
    drive_firmware_required: 'Обновите прошивку всех четырёх плат для тяги и watchdog.',
    not_homed: 'Сначала откалибруйте колёса по датчику (или задайте 0° вручную).',
    module_busy: 'Дождитесь остановки рулевых модулей.',
    steering_misaligned: 'Для тяги колёса должны смотреть в одном направлении (±2,5°).',
    drive_limit: 'Скорость превышает безопасный предел 20%.',
    control_not_supported: 'Тяга доступна только в режиме master.',
    command_queue_full: 'Очередь команд занята.',
    pin_required: 'PIN истёк — разблокируйте управление снова.'
  };

  function bindDriveControls() {
    const speed = $('rc-drive-speed');
    if (speed) speed.addEventListener('input', () => {
      $('rc-drive-speed-value').textContent = String(speed.value);
      if (driveDirection) void queueDrive(driveDirection * Number(speed.value));
    });
    document.querySelectorAll('[data-drive-direction]').forEach((button) => {
      button.addEventListener('pointerdown', (event) => {
        event.preventDefault();
        const direction = Number(button.dataset.driveDirection) < 0 ? -1 : 1;
        startDrive(direction);
        try { button.setPointerCapture(event.pointerId); } catch (error) {}
      });
      button.addEventListener('click', (event) => event.preventDefault());
      button.addEventListener('contextmenu', (event) => event.preventDefault());
    });
    const stop = $('rc-drive-stop');
    if (stop) stop.addEventListener('click', () => { void stopDrive(); });
  }

  function renderDrive() {
    const statusEl = $('rc-drive-status');
    const slider = $('rc-drive-speed');
    if (!statusEl || !slider) return;
    const drive = (snapshot && snapshot.drive) || {};
    const max = Math.max(1, Math.min(20, Number(drive.maxPercent) || 20));
    slider.max = String(max);
    if (Number(slider.value) > max) slider.value = String(max);
    $('rc-drive-speed-value').textContent = String(slider.value);
    const supported = !!(snapshot && snapshot.steeringControl && snapshot.steeringControl.available);
    const authorizedForDrive = authorized && supported && !!drive.available;
    slider.disabled = !authorizedForDrive;
    document.querySelectorAll('[data-drive-direction]').forEach((button) => {
      button.disabled = !authorizedForDrive || !drive.canDrive;
    });
    $('rc-drive-stop').disabled = !authorizedForDrive;
    const reason = drive.reason && DRIVE_REASON_TEXT[drive.reason];
    if (!supported) statusEl.textContent = 'Нет master · установите RC_SLAM_SOURCE=master.';
    else if (!authorized) statusEl.textContent = 'Нужен PIN для движения.';
    else if (reason) statusEl.textContent = reason;
    else if (drive.active) statusEl.textContent = 'Движение включено · отпустите кнопку для торможения.';
    else statusEl.textContent = 'Готово · все модули на связи и колёса выровнены.';
    statusEl.dataset.ready = String(!!drive.canDrive && authorizedForDrive);
  }

  function startDrive(direction) {
    if (!authorized) {
      $('rc-drive-status').textContent = 'Сначала разблокируйте PIN.';
      void unlock();
      return;
    }
    const drive = (snapshot && snapshot.drive) || {};
    if (!drive.canDrive) {
      renderDrive();
      return;
    }
    driveDirection = direction;
    if (driveTimer) clearInterval(driveTimer);
    const sendHeartbeat = () => {
      if (!driveDirection) return;
      void queueDrive(driveDirection * Number($('rc-drive-speed').value || 8));
    };
    sendHeartbeat();
    driveTimer = setInterval(sendHeartbeat, 200);
  }

  function releaseDrive() {
    const wasDriving = driveDirection !== 0 || driveTimer !== null;
    driveDirection = 0;
    if (driveTimer) clearInterval(driveTimer);
    driveTimer = null;
    if (wasDriving && authorized) void queueDrive(0);
  }

  async function stopDrive() {
    driveDirection = 0;
    if (driveTimer) clearInterval(driveTimer);
    driveTimer = null;
    if (!authorized) {
      $('rc-drive-status').textContent = 'PIN истёк — плата остановится по watchdog; разблокируйте для команды СТОП.';
      return;
    }
    await queueDrive(0);
    $('rc-drive-status').textContent = 'Стоп отправлен · PWM 0 · тормоз включён.';
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
        const timer = setTimeout(() => controller.abort(), 1200);
        try {
          const response = await fetch('api/drive', {
            method: 'POST', cache: 'no-store', signal: controller.signal,
            headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'drive' },
            body: JSON.stringify({ percent: target })
          });
          const body = await response.json().catch(() => ({}));
          if (!response.ok || !body.ok) {
            const reason = body.reason || (response.status === 401 ? 'pin_required' : 'command_failed');
            $('rc-drive-status').textContent = DRIVE_REASON_TEXT[reason] || 'Команда тяги отклонена.';
            if (response.status === 401 || reason === 'pin_required') authorized = false;
            if (target !== 0) {
              driveDirection = 0;
              if (driveTimer) clearInterval(driveTimer);
              driveTimer = null;
              driveQueued = 0;
            }
          } else if (target === 0) {
            $('rc-drive-status').textContent = 'PWM 0 · тормоз включён.';
          } else {
            $('rc-drive-status').textContent = 'Тяга ' + body.percent + '% · удерживайте кнопку.';
          }
        } catch (error) {
          $('rc-drive-status').textContent = 'Связь потеряна · аппаратный watchdog остановит моторы не позднее 650 мс.';
          if (target !== 0) {
            driveDirection = 0;
            if (driveTimer) clearInterval(driveTimer);
            driveTimer = null;
            driveQueued = 0;
          }
        } finally {
          clearTimeout(timer);
        }
      }
    })().finally(() => { driveSending = false; });
    return driveFlush;
  }

  function bindCard(id) {
    const card = document.querySelector('[data-rc-steering="' + id + '"]');
    const range = card.querySelector('[data-range]');
    range.addEventListener('input', () => {
      card.dataset.dirty = 'true';
      card.querySelector('[data-target]').textContent = Math.round(+range.value) + '°';
      renderCard(id);
    });
    card.querySelectorAll('[data-step]').forEach((button) => button.addEventListener('click', () => {
      range.value = String(Math.max(-85, Math.min(85, Number(range.value) + Number(button.dataset.step))));
      card.dataset.dirty = 'true';
      card.querySelector('[data-target]').textContent = Math.round(+range.value) + '°';
      renderCard(id);
    }));
    card.querySelector('[data-center]').addEventListener('click', () => {
      range.value = '0'; card.dataset.dirty = 'true';
      card.querySelector('[data-target]').textContent = '0°'; renderCard(id);
    });
    card.querySelector('[data-send]').addEventListener('click', () => void sendAngle(id));
    card.querySelector('[data-origin]').addEventListener('click', () => void setZero(id));
    card.querySelector('[data-calib]').addEventListener('click', () => void calibrate(id));
  }

  async function unlock() {
    const pin = window.prompt('Введите PIN для управления рулевыми и тяговыми моторами');
    if (pin === null) return;
    try {
      const { response, body } = await json('api/kiosk_lock/verify', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ pin })
      });
      authorized = response.ok && !!body.steeringAuthorized;
      $('rc-steering-auth').textContent = authorized ? 'PIN подтверждён' : 'Неверный PIN';
      await refresh();
    } catch (error) {
      $('rc-steering-auth').textContent = 'Сервер не отвечает';
    }
  }

  async function lock() {
    driveDirection = 0;
    if (driveTimer) clearInterval(driveTimer);
    driveTimer = null;
    if (authorized) await queueDrive(0);
    authorized = false;
    try { await json('api/kiosk_lock/session/end', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' }); }
    catch (error) { /* closing the panel must still lock local controls */ }
    IDS.forEach((id) => { delete pending[id]; renderCard(id); });
    const auth = $('rc-steering-auth');
    if (auth) auth.textContent = 'Нужен PIN';
    const unlockButton = $('rc-steering-unlock');
    const lockButton = $('rc-steering-lock');
    if (unlockButton) unlockButton.hidden = false;
    if (lockButton) lockButton.hidden = true;
  }

  async function sendAngle(id) {
    if (!authorized) return unlock();
    const card = document.querySelector('[data-rc-steering="' + id + '"]');
    const angle = Math.max(-85, Math.min(85, Number(card.querySelector('[data-range]').value)));
    try {
      const { response, body } = await json('api/kiosk_steer', {
        method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'steering' },
        body: JSON.stringify({ module: id, angle })
      });
      if (!response.ok || !body.ok) {
        if (response.status === 401) authorized = false;
        feedback[id] = { text: reasonText(body.reason), error: true, until: Date.now() + 6000 };
      } else {
        pending[id] = { at: Date.now(), angle: body.angle };
        card.dataset.dirty = '';
        feedback[id] = { text: 'команда отправлена', error: false, until: Date.now() + 6000 };
      }
    } catch (error) {
      feedback[id] = { text: 'сервер не отвечает', error: true, until: Date.now() + 6000 };
    }
    await refresh();
  }

  async function setZero(id) {
    if (!authorized) return unlock();
    if (!window.confirm('Запасной вариант без датчика. Физически выставьте колесо ' + id + ' прямо. Команда не двигает мотор; она назначает текущую позицию за программный 0°. После перезапуска Arduino ноль нужно задать заново. Продолжить?')) return;
    try {
      const { response, body } = await json('api/kiosk_zero', {
        method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'steering' },
        body: JSON.stringify({ module: id })
      });
      if (!response.ok || !body.ok) {
        if (response.status === 401) authorized = false;
        feedback[id] = { text: reasonText(body.reason), error: true, until: Date.now() + 6000 };
      } else {
        pending[id] = { at: Date.now(), zero: true };
        feedback[id] = { text: 'программный ноль отправлен', error: false, until: Date.now() + 6000 };
      }
    } catch (error) {
      feedback[id] = { text: 'сервер не отвечает', error: true, until: Date.now() + 6000 };
    }
    await refresh();
  }

  async function calibrate(id) {
    if (!authorized) return unlock();
    if (!window.confirm('Колесо ' + id + ' повернётся вправо до 100°, затем влево до 200°, пока не найдёт оптодатчик D18, и встанет в ноль. Колесо не должно упираться, рядом не должно быть людей. Начать?')) return;
    try {
      const { response, body } = await json('api/kiosk_calibrate', {
        method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'steering' },
        body: JSON.stringify({ module: id })
      });
      if (!response.ok || !body.ok) {
        if (response.status === 401) authorized = false;
        feedback[id] = { text: reasonText(body.reason), error: true, until: Date.now() + 6000 };
      } else {
        pending[id] = { at: Date.now(), calib: true, started: false };
        feedback[id] = { text: 'калибровка запущена', error: false, until: Date.now() + 6000 };
      }
    } catch (error) {
      feedback[id] = { text: 'сервер не отвечает', error: true, until: Date.now() + 6000 };
    }
    await refresh();
  }

  function reasonText(reason) {
    return ({
      pin_required: 'PIN истёк · разблокируйте', module_offline: 'модуль не отвечает',
      serial_unavailable: 'USB-порт мастера недоступен', not_homed: 'сначала калибровка или 0° вручную',
      module_busy: 'дождитесь остановки модуля', angle_limit: 'предел ±85°',
      command_queue_full: 'очередь команд занята', control_not_supported: 'источник не поддерживает поворот'
    })[reason] || 'команда не принята';
  }

  function start() {
    addUi();
    // Нужный модуль включается только на страницах оператора/веб-панели.
    document.documentElement.classList.add('rc-steering-ready');
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', start);
  else start();
  window.RCSteeringPanel = { open: () => { const button = $('rc-steering-launch'); if (button) button.click(); } };
})();
