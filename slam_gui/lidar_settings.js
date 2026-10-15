/* Настройки лидара: веб-пульт требует вход оператора, бортовой экран — без PIN.
   Здесь меняются только параметры лидара, не руление и не тяга робота. */
(function () {
  'use strict';
  if (window.RCLidarSettings) return;

  const $ = (id) => document.getElementById(id);
  const isKiosk = document.body.classList.contains('kiosk-body');
  let csrfToken = '';
  let opened = false;
  let statusTimer = null;

  async function getJson(url, init) {
    const response = await fetch(url, { cache: 'no-store', ...(init || {}) });
    const body = await response.json().catch(() => null);
    return { response, body };
  }

  function esc(value) {
    return String(value == null ? '' : value).replace(/[&<>"']/g, (character) => ({
      '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
    }[character]));
  }

  function mount() {
    if ($('rc-lidar-launch')) return;

    const button = document.createElement('button');
    button.id = 'rc-lidar-launch';
    button.className = 'rc-lidar-launch';
    button.type = 'button';
    button.textContent = isKiosk ? 'Настройки' : 'Лидар';
    button.title = isKiosk ? 'Настройки бортового экрана и лидара' : 'Настройки лидара 360°';
    button.setAttribute('aria-haspopup', 'dialog');
    button.setAttribute('aria-controls', 'rc-lidar-overlay');
    button.setAttribute('aria-expanded', 'false');

    const overlay = document.createElement('div');
    overlay.id = 'rc-lidar-overlay';
    overlay.className = 'rc-lidar-overlay';
    overlay.hidden = true;
    overlay.innerHTML = `
      <section class="rc-lidar-dialog" role="dialog" aria-modal="true" aria-labelledby="rc-lidar-title">
        <header class="rc-lidar-head">
          <div>
            <h2 id="rc-lidar-title">${isKiosk ? 'Настройки бортового экрана' : 'Настройки лидара 360°'}</h2>
            <p>${isKiosk
              ? 'Настройки доступны без PIN. Выберите порт и протокол лидара; изменения применятся сразу, без перезапуска.'
              : 'Выберите порт, протокол и режим мотора. Изменения сохраняются в .env и применяются сразу.'}</p>
          </div>
          <button type="button" class="rc-lidar-close" aria-label="Закрыть">×</button>
        </header>
        <div class="rc-lidar-status" id="rc-lidar-status" role="status" aria-live="polite">Загружаю статус…</div>
        <form id="rc-lidar-form" class="rc-lidar-form">
          <label>Откуда берётся скан
            <select id="rc-lidar-source">
              <option value="auto">auto — ROS 2, если он есть, иначе USB-порт</option>
              <option value="ros">ros — ROS 2 (топик /scan, драйвер пакетом)</option>
              <option value="serial">serial — USB-порт читает само приложение</option>
            </select>
          </label>
          <label>Порт (пусто — автопоиск)
            <input id="rc-lidar-port" list="rc-lidar-ports" placeholder="/dev/serial/by-id/… или /dev/ttyUSB0" autocomplete="off" />
            <datalist id="rc-lidar-ports"></datalist>
          </label>
          <label>Протокол
            <select id="rc-lidar-protocol">
              <option value="auto">auto — подобрать самому (в ROS 2 — драйвер сам)</option>
              <option value="rplidar">rplidar — RPLIDAR A1/A2/A3/S1/C1</option>
              <option value="ld06">ld06 — LD06/LD19</option>
              <option value="lds01">lds01 — LDS-01/LDS-02 (TurtleBot3)</option>
            </select>
          </label>
          <label>Скорость, бод
            <select id="rc-lidar-baud">
              <option value="">авто — подобрать по модели</option>
              <option value="115200">115200 (A1/A2M8)</option>
              <option value="230400">230400 (LD06/LD19, LDS-01)</option>
              <option value="256000">256000 (A2M7/A2M12/A3/S1)</option>
              <option value="460800">460800 (C1)</option>
            </select>
          </label>
          <label>Мотор RPLIDAR (линия переходника)
            <select id="rc-lidar-motor">
              <option value="auto">auto — перебрать варианты</option>
              <option value="none">none — линии не трогать</option>
              <option value="dtr-low">dtr-low</option><option value="dtr-high">dtr-high</option>
              <option value="rts-low">rts-low</option><option value="rts-high">rts-high</option>
            </select>
          </label>
          <label class="rc-lidar-check"><input type="checkbox" id="rc-lidar-autoprobe" /> Перебирать все USB-порты (автопоиск)</label>
          <p class="rc-lidar-msg" id="rc-lidar-msg" role="status" aria-live="polite"></p>
          <footer class="rc-lidar-actions">
            <button type="button" class="rc-lidar-btn" id="rc-lidar-rescan">Пересканировать</button>
            <button type="submit" class="rc-lidar-btn primary" id="rc-lidar-save">Сохранить и применить</button>
            <button type="button" class="rc-lidar-btn" id="rc-lidar-dismiss">Закрыть</button>
          </footer>
        </form>
      </section>`;

    const header = document.querySelector('.top-status, .head-actions, .top-right');
    if (header) header.append(button);
    else {
      button.classList.add('rc-lidar-floating');
      document.body.append(button);
    }
    document.body.append(overlay);

    button.addEventListener('click', open);
    overlay.querySelector('.rc-lidar-close').addEventListener('click', close);
    $('rc-lidar-dismiss').addEventListener('click', close);
    overlay.addEventListener('click', (event) => { if (event.target === overlay) close(); });
    window.addEventListener('keydown', (event) => { if (event.key === 'Escape' && opened) close(); });
    $('rc-lidar-form').addEventListener('submit', (event) => { event.preventDefault(); void save(); });
    $('rc-lidar-rescan').addEventListener('click', () => void rescan());
  }

  function open() {
    opened = true;
    const overlay = $('rc-lidar-overlay');
    if (overlay) overlay.hidden = false;
    const button = $('rc-lidar-launch');
    if (button) button.setAttribute('aria-expanded', 'true');
    void load();
  }

  function close() {
    opened = false;
    const overlay = $('rc-lidar-overlay');
    if (overlay) overlay.hidden = true;
    const button = $('rc-lidar-launch');
    if (button) button.setAttribute('aria-expanded', 'false');
    if (statusTimer) {
      clearInterval(statusTimer);
      statusTimer = null;
    }
  }

  function msg(text, bad) {
    const element = $('rc-lidar-msg');
    if (!element) return;
    element.textContent = text || '';
    element.dataset.bad = bad ? '1' : '';
  }

  // Строка про ROS 2: драйвер, порт, частота и состояние супервизора. Оператор
  // должен видеть, что лидар отдаёт именно пакет ROS 2 (а не подбор портов), и
  // что делать, если скана нет.
  function rosLine(status) {
    const ros = status && status.ros;
    if (!ros) return '';
    const parts = [];
    if (ros.supervisor && ros.supervisor.driver) {
      const driver = ros.supervisor.driver_title || ros.supervisor.driver;
      const port = ros.supervisor.port ? ' · ' + String(ros.supervisor.port).split('/').pop() : '';
      parts.push('драйвер ROS 2: ' + String(driver) + port);
    }
    if (ros.connected) {
      parts.push(String(ros.points || 0) + ' точек · ' + String(ros.hz || 0) + ' Гц');
    } else if (ros.publishers) {
      parts.push('топик ' + String(ros.topic) + ' публикуют: ' + String(ros.publishers));
    }
    if (ros.distro) parts.push('ROS 2 ' + String(ros.distro));
    const text = parts.length
      ? `ROS 2: ${parts.join(' · ')}`
      : (ros.available === false ? 'ROS 2: не подключён к процессу' : 'ROS 2: состояние неизвестно');
    return esc(text);
  }

  function renderStatus(status) {
    const element = $('rc-lidar-status');
    if (!element || !status) return;
    const detected = status.detected;
    const found = detected && detected.port
      ? [String(detected.port).split('/').pop(), detected.protocol, detected.baud ? detected.baud + ' бод' : '']
        .filter(Boolean).join(' · ')
      : 'устройство не найдено';
    const usb = status.usb_diagnostics && status.usb_diagnostics.summary
      ? status.usb_diagnostics.summary : '';
    const source = status.source === 'peer' ? ' · скан от второго экрана' : '';
    const reason = [status.reason, status.hint].filter(Boolean).join(' — ');
    const ros = rosLine(status);
    element.innerHTML = `<b>${status.connected ? 'Лидар подключён' : 'Скана пока нет'}</b>` +
      `${source} · точек: ${esc(status.points_count || 0)}<br>` +
      `<span>устройство: ${esc(found)}${reason ? ' · ' + esc(reason) : ''}` +
      `${ros ? '<br>' + ros : ''}${usb ? '<br>' + esc(usb) : ''}</span>`;
    const launch = $('rc-lidar-launch');
    if (launch) launch.dataset.state = status.connected ? 'ok' : 'bad';
  }

  function populateSettings(settings) {
    const value = settings || {};
    $('rc-lidar-source').value = value.source || 'auto';
    $('rc-lidar-port').value = value.port || '';
    $('rc-lidar-protocol').value = value.protocol || 'auto';
    $('rc-lidar-baud').value = value.baud || '';
    $('rc-lidar-motor').value = value.motor || 'auto';
    $('rc-lidar-autoprobe').checked = value.autoprobe !== false;
  }

  async function refreshStatus() {
    if (!opened) return;
    try {
      const { response, body } = await getJson('api/lidar');
      if (response.ok && body && body.ok !== false) renderStatus(body);
    } catch (error) {
      // Оставляем последнюю диагностику на экране: краткий обрыв сети не должен
      // затирать причину, по которой лидар не нашёлся.
    }
  }

  function startStatusPolling() {
    if (statusTimer) clearInterval(statusTimer);
    statusTimer = setInterval(() => { void refreshStatus(); }, 1500);
  }

  async function load() {
    msg('');
    try {
      const { response, body } = await getJson('api/lidar/settings');
      if (!response.ok || !body || !body.ok) {
        msg((body && body.error) || 'Настройки недоступны: проверьте соединение с бортовым экраном.', true);
        if (body && body.status) renderStatus(body.status);
        return;
      }
      csrfToken = body.csrfToken || csrfToken;
      populateSettings(body.settings);
      const ports = (body.options && body.options.ports) || [];
      $('rc-lidar-ports').innerHTML = ports.map((port) => `<option value="${esc(port)}">`).join('');
      renderStatus(body.status);
      startStatusPolling();
    } catch (error) {
      msg('Сервер не отвечает — настройки лидара не загрузились.', true);
    }
  }

  function payload() {
    return {
      source: $('rc-lidar-source').value,
      port: $('rc-lidar-port').value.trim(),
      protocol: $('rc-lidar-protocol').value,
      baud: $('rc-lidar-baud').value,
      motor: $('rc-lidar-motor').value,
      autoprobe: $('rc-lidar-autoprobe').checked
    };
  }

  async function post(url, body) {
    const headers = { 'Content-Type': 'application/json' };
    if (csrfToken) headers['X-CSRF-Token'] = csrfToken;
    return getJson(url, { method: 'POST', headers, body: JSON.stringify(body || {}) });
  }

  async function save() {
    msg('Сохраняю настройки и запускаю поиск лидара…');
    const saveButton = $('rc-lidar-save');
    if (saveButton) saveButton.disabled = true;
    try {
      const { response, body } = await post('api/lidar/settings', payload());
      if (!response.ok || !body || !body.ok) {
        msg((body && body.error) || 'Не удалось сохранить настройки.', true);
        return;
      }
      populateSettings(body.settings);
      renderStatus(body.status);
      msg('Сохранено и применено. Статус обновляется автоматически.');
    } catch (error) {
      msg('Сервер не отвечает при сохранении.', true);
    } finally {
      if (saveButton) saveButton.disabled = false;
    }
  }

  async function rescan() {
    msg('Перезапускаю поиск лидара…');
    const rescanButton = $('rc-lidar-rescan');
    if (rescanButton) rescanButton.disabled = true;
    try {
      const { response, body } = await post('api/lidar/rescan', {});
      if (!response.ok || !body || !body.ok) {
        msg((body && body.error) || 'Не удалось пересканировать.', true);
        return;
      }
      renderStatus(body.status);
      // При источнике ros «пересканировать» значит перезапустить драйвер через
      // супервизор ROS 2 — сервер отвечает, что именно произошло.
      msg(body.ros_rescan
        ? 'ROS 2: ' + body.ros_rescan
        : 'Поиск запущен заново с текущими настройками.');
    } catch (error) {
      msg('Сервер не отвечает при пересканировании.', true);
    } finally {
      if (rescanButton) rescanButton.disabled = false;
    }
  }

  function start() {
    mount();
    window.RCLidarSettings = { open };
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', start);
  else start();
})();
