/* ============================================================================
 * ai_panel.js — панель «Искусственный интеллект» на экране робота.
 *
 * Что показывает (всё — настоящие числа с борта, GET /api/state, блок «ai»):
 *   • сеть: 32 нейрона (24 скрытых + 8 выходов) живой сеткой активности;
 *   • команду: куда встали колёса, какая тяга, слово манёвра;
 *   • намерение сети и манёвр учителя — видно, где сеть ещё догоняет;
 *   • обучение: сколько тактов и какая ошибка (на ходу), плюс ошибка урока;
 *   • ROS 2: топик /cmd_vel, сколько команд опубликовано, включена ли выдача;
 *   • препятствия с дальномера и задание, к которому едет робот;
 *   • журнал: последние команды с временем.
 *
 * Панель строит свою разметку сама: main.html держит только контейнер
 * #sc-ai-panel, поэтому экран не разрастается пустыми блоками.
 * ========================================================================== */
'use strict';

(function () {
  //: Подписи входов сети — короткие: столбец киоска узкий, а видеть надо все 10.
  const INPUT_LABELS = {
    dx: 'вперёд', dy: 'вбок', dth: 'курс', crab: 'краб',
    obstL: 'лев', obstC: 'центр', obstR: 'прав',
    soc: 'АКБ', cargo: 'груз', speed: 'скор',
  };
  const NEURONS = 32, HIDDEN = 24;

  let host = null, refs = null;

  function el(id) { return document.getElementById(id); }

  function cell(html) {
    const div = document.createElement('div');
    div.innerHTML = html;
    return div.firstElementChild || div;
  }

  /** Строит разметку панели один раз. */
  function mount() {
    if (!host) return null;
    if (refs) return refs;
    host.innerHTML = [
      '<div class="sc-ai-net">',
      '  <div class="sc-ai-neurons" id="sc-ai-neurons" role="img"',
      '       aria-label="Активность 32 нейронов сети"></div>',
      '  <div class="sc-ai-net-legend">',
      '    <span><i class="sc-ai-leg-in"></i>входы 10</span>',
      '    <span><i class="sc-ai-leg-hid"></i>скрытых ' + HIDDEN + '</span>',
      '    <span><i class="sc-ai-leg-out"></i>выходов 8</span>',
      '  </div>',
      '</div>',
      '<div class="sc-ai-cmd">',
      '  <b id="sc-ai-command">—</b>',
      '  <span id="sc-ai-intent">намерение: —</span>',
      '</div>',
      '<div class="sc-ai-inputs" id="sc-ai-inputs"></div>',
      '<div class="sc-ai-rows">',
      '  <div><span>колёса</span><b id="sc-ai-angles">—</b></div>',
      '  <div><span>тяга</span><b id="sc-ai-throttle">—</b></div>',
      '  <div><span>обучение</span><b id="sc-ai-train">—</b></div>',
      '  <div><span>отклик</span><b id="sc-ai-tick">—</b></div>',
      '  <div><span>ROS 2</span><b id="sc-ai-ros">—</b></div>',
      '  <div><span>препятствия</span><b id="sc-ai-obst">—</b></div>',
      '  <div><span>задание</span><b id="sc-ai-goal">—</b></div>',
      '</div>',
      '<div class="sc-ai-log" id="sc-ai-log"></div>',
    ].join('');
    const grid = el('sc-ai-neurons');
    for (let i = 0; i < NEURONS; i++) {
      const cls = 'sc-ai-n' + (i < HIDDEN ? '' : ' sc-ai-n-out');
      const node = cell('<i class="' + cls + '" data-i="' + i + '"></i>');
      node.title = i < HIDDEN ? ('скрытый нейрон ' + (i + 1)) : ('выход ' + (i - HIDDEN + 1));
      grid.appendChild(node);
    }
    const inputs = el('sc-ai-inputs');
    Object.keys(INPUT_LABELS).forEach((name) => {
      const row = cell('<span class="sc-ai-in" data-in="' + name + '"><i></i><small>' +
        INPUT_LABELS[name] + '</small></span>');
      inputs.appendChild(row);
    });
    refs = {
      neurons: grid.querySelectorAll('i'),
      command: el('sc-ai-command'),
      intent: el('sc-ai-intent'),
      inputs: inputs.querySelectorAll('.sc-ai-in'),
      angles: el('sc-ai-angles'),
      throttle: el('sc-ai-throttle'),
      train: el('sc-ai-train'),
      tick: el('sc-ai-tick'),
      ros: el('sc-ai-ros'),
      obst: el('sc-ai-obst'),
      goal: el('sc-ai-goal'),
      log: el('sc-ai-log'),
    };
    return refs;
  }

  function setText(node, text) { if (node) node.textContent = text; }

  function fmt(v, digits) {
    return Number(v || 0).toFixed(digits === undefined ? 2 : digits).replace('.', ',');
  }

  function wheelText(angles) {
    const order = ['FL', 'FR', 'RL', 'RR'];
    return order.map((id) => id + ' ' + Math.round(Number((angles || {})[id] || 0)) + '°').join(' · ');
  }

  function neuronActivity(values) {
    if (!refs) return;
    const list = refs.neurons || [];
    for (let i = 0; i < list.length; i++) {
      const value = Number(values && values[i] || 0);
      const node = list[i];
      node.style.opacity = (0.16 + 0.84 * Math.min(1, Math.abs(value))).toFixed(2);
      node.dataset.sign = value >= 0 ? 'plus' : 'minus';
    }
  }

  function inputBars(inputs) {
    if (!refs || !refs.inputs) return;
    for (let i = 0; i < refs.inputs.length; i++) {
      const row = refs.inputs[i];
      const name = row.dataset.in;
      const value = Number((inputs || {})[name] || 0);
      const bar = row.querySelector('i');
      if (!bar) continue;
      bar.style.width = (Math.min(1, Math.abs(value)) * 100).toFixed(1) + '%';
      bar.dataset.sign = value >= 0 ? 'plus' : 'minus';
      row.title = INPUT_LABELS[name] + ': ' + fmt(value);
    }
  }

  function obstacleText(inputs) {
    inputs = inputs || {};
    return 'слева ' + fmt(inputs.obstL) + ' · центр ' + fmt(inputs.obstC) +
      ' · справа ' + fmt(inputs.obstR);
  }

  function rosText(ros, available) {
    if (!ros) return 'нет данных';
    const parts = [];
    if (!available) return ros.reason || 'сеть ведёт только стенд';
    parts.push(ros.topic + ' · публикаций ' + (ros.published || 0));
    parts.push(ros.enabled ? 'выдача включена' : 'выдача выключена (RC_AI_ROS=1)');
    if (!ros.available) parts.push(ros.reason || 'rclpy не найден');
    return parts.join(' · ');
  }

  /** Отрисовка по состоянию борта. Возвращает true, если блок ИИ был в данных. */
  function render(state) {
    host = el('sc-ai-panel');
    if (!host) return false;
    if (!mount()) return false;
    const ai = (state && state.ai) || {};
    if (!ai || ai.available === false) {
      // нет блока ИИ: либо реальные модули (сеть ведёт только стенд), либо
      // страница работает в демо-режиме без сервера — панель не выдумывает цифры
      setText(refs.command, 'ИИ недоступен');
      setText(refs.intent, ai.reason || 'борт не отдал блок ИИ — включите источник «стенд»');
      neuronActivity([]);
      inputBars({});
      setText(refs.angles, '—');
      setText(refs.throttle, '—');
      setText(refs.train, '—');
      setText(refs.tick, '—');
      setText(refs.ros, ai.reason || '—');
      setText(refs.obst, '—');
      setText(refs.goal, '—');
      return false;
    }

    setText(refs.command, ai.command || '—');
    setText(refs.intent, 'намерение: ' + (ai.intent || '—') + ' · учитель: ' + (ai.teacher || '—'));
    neuronActivity(ai.activations || []);
    inputBars(ai.inputs || {});
    setText(refs.angles, wheelText(ai.angles));
    setText(refs.throttle, fmt(ai.throttle) + ' · ' + fmt(ai.speed, 1) + ' м/с');
    if (refs.tick) {
      const budget = Number(ai.budgetMs || 5);
      const spent = Number(ai.tickMs || 0);
      refs.tick.textContent = fmt(spent, 2) + ' мс · бюджет ' + fmt(budget, 0) + ' мс' +
        (ai.tickAvgMs != null ? ' · сред. ' + fmt(ai.tickAvgMs, 2) : '');
      refs.tick.dataset.over = spent > budget ? 'yes' : 'no';
      refs.tick.title = 'отклик такта: сеть, обзор дальномера, карта, ROS 2 — уложились в ' +
        fmt(budget, 0) + ' мс';
    }
    setText(refs.train, 'тактов ' + (ai.steps || 0) + ' · ошибка ' + fmt(ai.loss, 4) +
      (ai.pretrainLoss != null ? ' · урок ' + fmt(ai.pretrainLoss, 4) : ''));
    setText(refs.ros, rosText(ai.ros, ai.available !== false));
    setText(refs.obst, obstacleText(ai.inputs));
    const goal = ai.goal || {};
    setText(refs.goal, (goal.label || '—') + (goal.dist != null ? ' · ' + fmt(goal.dist, 1) + ' м' : '') +
      (goal.dth != null ? ' · курс ' + Math.round(goal.dth) + '°' : ''));
    if (refs.log) {
      refs.log.innerHTML = (ai.log || []).slice(-4).reverse()
        .map((row) => '<span><b>' + (row.time || '') + '</b> ' + (row.label || '') + '</span>')
        .join('') || '<span>журнал пока пуст</span>';
    }
    return true;
  }

  window.RSAiPanel = { render, mount, wheelText, rosText };
})();
