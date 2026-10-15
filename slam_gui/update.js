/* Кнопка обновляет одним fast-forward весь репозиторий: Flask-сервер,
   веб-пульт и статические файлы бортового экрана. */
(function () {
  'use strict';

  if (window.RCUpdater) return;
  const $ = (id) => document.getElementById(id);
  const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
  let csrfToken = '';
  let currentCheck = null;
  let opened = false;
  let following = false;

  async function getJson(url, init, timeout = 30000) {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), timeout);
    try {
      const response = await fetch(url, { cache: 'no-store', ...init, signal: controller.signal });
      const body = await response.json().catch(() => null);
      return { response, body };
    } finally {
      clearTimeout(timer);
    }
  }

  function mount() {
    if ($('rc-update-launch')) return;
    const button = document.createElement('button');
    button.id = 'rc-update-launch';
    button.className = 'rc-update-launch';
    button.type = 'button';
    button.textContent = 'Обновить с GitHub';
    button.title = 'Скачать с GitHub, переустановить, собрать и перезапустить сервер и экран';

    const overlay = document.createElement('div');
    overlay.id = 'rc-update-overlay';
    overlay.className = 'rc-update-overlay';
    overlay.hidden = true;
    overlay.innerHTML = `
      <section class="rc-update-dialog" role="dialog" aria-modal="true" aria-labelledby="rc-update-title">
        <header class="rc-update-head"><div><h2 id="rc-update-title">Обновление с GitHub</h2>
          <p>Скачиваю коммиты с GitHub → ставлю код fast-forward → переустанавливаю зависимости (ensure_deps.sh, офлайн из vendor/) → собираю и проверяю код → перезапускаю веб-пульт и бортовой экран. Прошивки Arduino кнопка не прошивает.</p></div>
          <button type="button" class="rc-update-close" aria-label="Закрыть">×</button></header>
        <div class="rc-update-facts">
          <div><span>Репозиторий</span><b id="rc-update-repo">—</b></div>
          <div><span>Ветка</span><b id="rc-update-branch">—</b></div>
          <div><span>Установлено</span><b id="rc-update-from">—</b></div>
          <div><span>Доступно</span><b id="rc-update-to">—</b></div>
        </div>
        <div class="rc-update-progress-wrap" aria-live="polite">
          <div class="rc-update-progress-label"><b id="rc-update-phase">Нажмите «Проверить».</b><span id="rc-update-percent">0%</span></div>
          <div class="rc-update-track"><i id="rc-update-progress"></i></div>
          <p id="rc-update-message" role="status"></p>
        </div>
        <pre id="rc-update-log" class="rc-update-log" hidden></pre>
        <footer class="rc-update-actions">
          <button type="button" class="rc-update-btn" id="rc-update-check">Проверить</button>
          <button type="button" class="rc-update-btn primary" id="rc-update-run" disabled>Установить обновление</button>
          <button type="button" class="rc-update-btn" id="rc-update-dismiss">Закрыть</button>
        </footer>
      </section>`;
    const header = document.querySelector('.top-status, .head-actions, .top-right');
    if (header) header.append(button);
    else { button.classList.add('rc-update-floating'); document.body.append(button); }
    document.body.append(overlay);

    button.addEventListener('click', () => {
      opened = true;
      overlay.hidden = false;
      void check(false);
    });
    overlay.querySelector('.rc-update-close').addEventListener('click', close);
    $('rc-update-dismiss').addEventListener('click', close);
    overlay.addEventListener('click', (event) => { if (event.target === overlay) close(); });
    $('rc-update-check').addEventListener('click', () => void check(true));
    $('rc-update-run').addEventListener('click', () => void startUpdate());
    window.addEventListener('keydown', (event) => { if (event.key === 'Escape' && opened) close(); });

    // Неблокирующая проверка: подсветить кнопку при появлении новой версии.
    setTimeout(() => void check(false), 5000);
  }

  function close() {
    opened = false;
    const overlay = $('rc-update-overlay');
    if (overlay) overlay.hidden = true;
  }

  function setButton(label, state) {
    const button = $('rc-update-launch');
    if (!button) return;
    button.textContent = label;
    button.dataset.state = state || '';
  }

  function showCheck(data) {
    currentCheck = data;
    csrfToken = data.csrfToken || csrfToken;
    $('rc-update-repo').textContent = data.repo || '—';
    $('rc-update-branch').textContent = data.branch || '—';
    $('rc-update-from').textContent = data.local || '—';
    $('rc-update-to').textContent = data.latest || '—';
    $('rc-update-percent').textContent = '0%';
    $('rc-update-progress').style.width = '0%';
    if (data.busy) {
      $('rc-update-phase').textContent = 'Обновление уже выполняется…';
      $('rc-update-message').textContent = 'Подключаюсь к его прогрессу.';
      $('rc-update-run').disabled = true;
      setButton('Обновление…', 'busy');
      void followUpdate();
      return;
    }
    if (data.dirty) {
      $('rc-update-phase').textContent = 'Есть локальные изменения';
      $('rc-update-message').textContent = 'Автообновление отключено, чтобы не перезаписать файлы. Сохраните изменения в Git и проверьте снова.';
      $('rc-update-run').disabled = true;
      setButton('Проверить', '');
    } else if (data.updateAvailable) {
      $('rc-update-phase').textContent = 'Доступна новая версия';
      $('rc-update-message').textContent = 'После установки сервер и отдельный киоск-процесс перезапустятся; страница обновится автоматически.';
      $('rc-update-run').disabled = !data.canUpdate || following;
      setButton('Есть обновление', 'available');
    } else {
      $('rc-update-phase').textContent = 'Установлена последняя версия';
      $('rc-update-message').textContent = data.latestMsg || 'Обновлений для этой ветки нет.';
      $('rc-update-run').disabled = true;
      setButton('Актуально', 'current');
    }
  }

  async function check(forceOpen) {
    if (forceOpen) {
      opened = true;
      $('rc-update-overlay').hidden = false;
    }
    if (opened) {
      $('rc-update-phase').textContent = 'Проверяю Git-ветку…';
      $('rc-update-message').textContent = 'Сверяю установленную версию с выбранной веткой remote.';
      $('rc-update-check').disabled = true;
      $('rc-update-run').disabled = true;
    }
    try {
      const { response, body } = await getJson('api/update/check', undefined, 40000);
      if (!response.ok || !body || !body.ok) {
        const message = body && body.error ? body.error : 'Не удалось проверить обновления.';
        if (opened) {
          $('rc-update-phase').textContent = 'Проверка не выполнена';
          $('rc-update-message').textContent = message;
          setButton('Повторить', 'error');
        } else {
          // Фоновая проверка не должна прятать саму кнопку: оператор ищет
          // именно «Обновить с GitHub», а не «Повторить» после тихого сбоя
          // сети/ветки. Ошибка видна красным состоянием, текст — по делу.
          setButton('Обновить с GitHub', 'error');
        }
        return;
      }
      showCheck(body);
    } catch (error) {
      if (opened) {
        $('rc-update-phase').textContent = 'Сервер обновлений недоступен';
        $('rc-update-message').textContent = 'Проверьте сеть и конфигурацию Git remote.';
        setButton('Повторить', 'error');
      } else {
        setButton('Обновить с GitHub', 'error');
      }
    } finally {
      if (opened) $('rc-update-check').disabled = false;
    }
  }

  async function startUpdate() {
    if (!currentCheck || !currentCheck.canUpdate || !csrfToken || following) return;
    if (!window.confirm('Установить новую версию? Обновятся сервер и интерфейс. Во время перезапуска связь пропадёт на несколько секунд.')) return;
    $('rc-update-run').disabled = true;
    $('rc-update-check').disabled = true;
    $('rc-update-message').textContent = 'Запускаю обновление…';
    try {
      const { response, body } = await getJson('api/update/run', {
        method: 'POST', headers: { 'X-CSRF-Token': csrfToken, 'Content-Type': 'application/json' }, body: '{}'
      }, 15000);
      if (!response.ok || !body || !body.ok) {
        $('rc-update-phase').textContent = 'Не удалось запустить';
        $('rc-update-message').textContent = (body && (body.error || body.phase)) || 'Повторите проверку.';
        $('rc-update-check').disabled = false;
        return;
      }
      await followUpdate();
    } catch (error) {
      $('rc-update-phase').textContent = 'Нет ответа сервера';
      $('rc-update-message').textContent = 'Проверяю статус обновления…';
      await followUpdate();
    }
  }

  async function followUpdate() {
    if (following) return;
    following = true;
    let rev = -1;
    let failures = 0;
    try {
      while (opened) {
        try {
          const { response, body } = await getJson('api/update/status?since=' + encodeURIComponent(rev), undefined, 30000);
          if (!response.ok || !body) throw new Error('status');
          failures = 0;
          rev = Number(body.rev) || 0;
          renderStatus(body);
          if (body.status === 'done') {
            if (Number(body.changes) > 0 && body.restart) {
              await waitForRestart(body.toSha || '');
            }
            return;
          }
          if (body.status === 'error') return;
          if (body.status === 'idle') {
            await sleep(500);
            continue;
          }
        } catch (error) {
          failures++;
          if (failures > 80) {
            $('rc-update-phase').textContent = 'Сервер долго не отвечает';
            $('rc-update-message').textContent = 'Проверьте запуск сервера и обновите страницу вручную.';
            return;
          }
          await sleep(750);
        }
      }
    } finally {
      following = false;
      $('rc-update-check').disabled = false;
    }
  }

  function renderStatus(state) {
    $('rc-update-phase').textContent = state.phase || 'Обновляю…';
    const pct = Math.max(0, Math.min(100, Number(state.pct) || 0));
    $('rc-update-percent').textContent = pct + '%';
    $('rc-update-progress').style.width = pct + '%';
    if (state.error) $('rc-update-message').textContent = state.error;
    const log = $('rc-update-log');
    const lines = Array.isArray(state.output) ? state.output : [];
    if (lines.length) {
      log.hidden = false;
      log.textContent = lines.slice(-14).join('\n');
      log.scrollTop = log.scrollHeight;
    }
    if (state.status === 'busy') setButton('Обновление…', 'busy');
    if (state.status === 'error') setButton('Повторить', 'error');
    if (state.status === 'done') {
      $('rc-update-run').disabled = true;
      if (Number(state.changes) === 0) setButton('Актуально', 'current');
    }
  }

  async function waitForRestart(expectedSha) {
    $('rc-update-phase').textContent = 'Перезапускаю сервер и экран робота…';
    $('rc-update-message').textContent = 'Жду ответ уже запущенного процесса с новой версией кода.';
    const until = Date.now() + 90000;
    while (opened && Date.now() < until) {
      try {
        const { body } = await getJson('api/update/version', undefined, 3500);
        const buildSha = String(body && body.buildSha || '');
        if (body && body.ok && buildSha && (!expectedSha || buildSha === expectedSha)) {
          window.location.reload();
          return;
        }
      } catch (error) { /* процесс перезапускается */ }
      await sleep(900);
    }
    $('rc-update-phase').textContent = 'Обновление установлено';
    $('rc-update-message').textContent = 'Сервер обновлён, но процесс с новой версией кода не ответил автоматически. Проверьте службы и перезагрузите страницу.';
  }

  function start() {
    mount();
    window.RCUpdater = { check: () => check(true) };
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', start);
  else start();
})();
