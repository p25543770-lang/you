/* ============================================================================
 * js_ai.js — панель ИИ и карта: разметка, отрисовка, честность данных.
 *
 * Проверяем на заглушке DOM и заглушке canvas:
 *   • панель строит сеть из 32 клеток: 24 скрытых + 8 выходов помечены;
 *   • полоски входов показывают все 10 входов сети;
 *   • команда, колёса, обучение, ROS 2, препятствия и журнал попадают в текст;
 *   • строка про ROS не выдаёт стенд за робота с rclpy;
 *   • карта разбирает клетки борта («значение×количество») и рисует их,
 *     а также робота, площадки и задание;
 *   • без блока ИИ (реальные модули) панель честно пишет «ИИ недоступен»
 *     и ничего не выбрасывает.
 *
 * Запуск:  node tests/js_ai.js
 * ========================================================================== */
'use strict';

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const AI_JS = path.join(__dirname, '..', 'slam_gui', 'ai_panel.js');
const MAP_JS = path.join(__dirname, '..', 'slam_gui', 'map.js');
const failures = [];

function check(label, ok, detail) {
  console.log((ok ? 'OK   ' : 'FAIL ') + label + (detail ? ' → ' + detail : ''));
  if (!ok) failures.push(label + (detail ? ': ' + detail : ''));
}

/* --- маленький DOM: разбирает разметку панели на элементы ---------------- */
const registry = new Map();

function makeEl(tag, attrs) {
  const el = {
    tagName: String(tag || 'div').toUpperCase(),
    id: (attrs && attrs.id) || '',
    className: (attrs && attrs.class) || '',
    dataset: {},
    style: {},
    children: [],
    _html: '',
    textContent: '',
    title: '',
    classList: {
      _set: new Set(),
      add(c) { this._set.add(c); },
      remove(c) { this._set.delete(c); },
      toggle(c, on) { if (on === undefined ? !this._set.has(c) : on) this._set.add(c); else this._set.delete(c); },
      contains(c) { return this._set.has(c); },
    },
    setAttribute(k, v) { this[k] = v; },
    getAttribute() { return null; },
    removeAttribute() {},
    addEventListener() {}, removeEventListener() {},
    appendChild(c) { this.children.push(c); return c; },
    removeChild() {}, remove() {},
    getContext() { return null; },
    get firstElementChild() { return this.children[0] || null; },
    get lastElementChild() { return this.children[this.children.length - 1] || null; },
    set innerHTML(v) {
      this._html = String(v);
      this.children = [];
      const re = /<([a-zA-Z]+)([^>]*?)(\/?)>/g;
      let m;
      while ((m = re.exec(this._html))) {
        const a = m[2] || '';
        // вложенное содержимое: <span …><i></i></span> — дети внутри детей
        const close = this._html.indexOf('</' + m[1] + '>', re.lastIndex);
        const inner = close > 0 ? this._html.slice(re.lastIndex, close) : '';
        if (close > 0) re.lastIndex = close;
        const id = (a.match(/id="([^"]+)"/) || [])[1];
        const cls = (a.match(/class="([^"]+)"/) || [])[1];
        const child = makeEl(m[1], { id: id, class: cls });
        (a.match(/data-([a-z-]+)="([^"]*)"/g) || []).forEach((raw) => {
          const [, key, val] = raw.match(/data-([a-z-]+)="([^"]*)"/) || [];
          if (key) child.dataset[key.replace(/-(\w)/g, (_, c) => c.toUpperCase())] = val;
        });
        if (inner.indexOf('<') >= 0) child.innerHTML = inner;
        this.children.push(child);
        if (id && !registry.has(id)) registry.set(id, child);
      }
    },
    get innerHTML() { return this._html; },
    querySelector(sel) { return this.querySelectorAll(sel)[0] || null; },
    querySelectorAll(sel) {
      const byClass = (c) => this.children.filter((k) => String(k.className).split(/\s+/).indexOf(c) >= 0);
      if (sel.charAt(0) === '.') return byClass(sel.slice(1));
      if (sel === 'i') return this.children.filter((k) => k.tagName === 'I');
      return this.children.filter((k) => k.id === sel);
    },
  };
  return el;
}

function makeCanvas() {
  const el = makeEl('canvas');
  el.width = 660;
  el.height = 420;
  el.clientWidth = 660;
  el.clientHeight = 420;
  el.getBoundingClientRect = () => ({ left: 0, top: 0, width: 660, height: 420 });
  el.calls = [];
  el.getContext = () => new Proxy({}, {
    get: (t, p) => {
      if (p === 'canvas') return el;
      if (p === 'measureText') return () => ({ width: 10 });
      return (...args) => { el.calls.push([p, args]); };
    },
    set: () => true,
  });
  return el;
}

const state = {
  map: {
    ok: true, w: 8, h: 4, res: 0.05, version: 7, scanPct: 42,
    world: { w: 0.4, h: 0.2 },
    pose: { x: 0.2, y: 0.1, th: 0.5 },
    trail: [[0.1, 0.1], [0.2, 0.1]],
    pads: [{ x: 0.05, y: 0.05, label: 'А' }],
    goal: { x: 0.3, y: 0.15, label: 'площадка Б' },
  },
  ai: {
    available: true, neurons: 32, layers: [10, 24, 8],
    command: 'вперёд · влево · 0,6 м/с', mode: 'вперёд', steer: 'влево',
    intent: 'вперёд', teacher: 'краб',
    angles: { FL: -30, FR: -30, RL: -30, RR: -30 },
    throttle: 0.62, speed: 0.6,
    loss: 0.0085, lossAvg: 0.01, pretrainLoss: 0.19, steps: 12345,
    activations: Array.from({ length: 32 }, (_, i) => (i % 3 - 1) * 0.4),
    inputs: {
      dx: 0.4, dy: -0.3, dth: 0.2, crab: 0, obstL: 0.2, obstC: 0.9,
      obstR: 0.35, soc: 0.78, cargo: 1, speed: 0.3,
    },
    goal: { label: 'площадка Б', dist: 1.8, dth: 12.4 },
    log: [
      { time: '12:00:01', label: 'вперёд · влево · 0,6 м/с', mode: 'вперёд' },
      { time: '12:00:09', label: 'разворот · влево · 0,3 м/с', mode: 'разворот' },
    ],
    ros: {
      available: false, enabled: false, topic: '/cmd_vel', published: 0,
      reason: 'rclpy не найден — ИИ ведёт стенд', error: '',
    },
  },
};

function load(file, extraGlobals) {
  const src = fs.readFileSync(file, 'utf8');
  const sandbox = Object.assign({
    window: {}, console, setInterval: () => 1, clearInterval: () => {},
    fetch: () => Promise.reject(new Error('нет сети в заглушке')),
    Promise, Math, Number, String, Array, Object, JSON, parseInt, isFinite,
  }, extraGlobals);
  sandbox.window = sandbox;
  sandbox.globalThis = sandbox;
  vm.createContext(sandbox);
  vm.runInContext(src, sandbox, { filename: file });
  return sandbox;
}

/* --- 1. Панель ИИ -------------------------------------------------------- */
const panelHost = makeEl('section', { id: 'sc-ai-panel' });
registry.set('sc-ai-panel', panelHost);
const dom = {
  getElementById: (id) => registry.get(id) || null,
  createElement: (tag) => makeEl(tag),
  addEventListener: () => {},
  readyState: 'complete',
};

const aiCtx = load(AI_JS, { document: dom });
const rendered = aiCtx.RSAiPanel.render(state);
check('панель ИИ отрисовала состояние борта', rendered === true);

const neurons = registry.get('sc-ai-neurons').children;
check('в сети 32 нейрона', neurons.length === 32, 'получено ' + neurons.length);
check('8 выходных нейронов помечены', neurons.filter((n) => n.className.indexOf('sc-ai-n-out') >= 0).length === 8);
check('24 скрытых нейрона', neurons.filter((n) => n.className.indexOf('sc-ai-n-out') < 0).length === 24);
check('активность разложена по знаку',
  neurons.some((n) => n.dataset.sign === 'plus') && neurons.some((n) => n.dataset.sign === 'minus'));

const inputs = registry.get('sc-ai-inputs').children;
check('все 10 входов сети показаны', inputs.length === 10, 'получено ' + inputs.length);
check('полоска входа шире при большем значении',
  Number(registry.get('sc-ai-inputs').querySelectorAll('.sc-ai-in')[5].querySelector('i').style.width
    .replace('%', '')) > 50);

check('команда с борта попала на панель', registry.get('sc-ai-command').textContent === state.ai.command);
check('намерение и учитель показаны',
  /намерение: вперёд/.test(registry.get('sc-ai-intent').textContent) &&
  /учитель: краб/.test(registry.get('sc-ai-intent').textContent),
  registry.get('sc-ai-intent').textContent);
check('углы всех четырёх колёс видны',
  registry.get('sc-ai-angles').textContent === 'FL -30° · FR -30° · RL -30° · RR -30°',
  registry.get('sc-ai-angles').textContent);
check('обучение: такты и ошибка',
  /тактов 12345/.test(registry.get('sc-ai-train').textContent) &&
  /ошибка 0,0085/.test(registry.get('sc-ai-train').textContent),
  registry.get('sc-ai-train').textContent);

const ros = registry.get('sc-ai-ros').textContent;
check('ROS 2: показан топик и честная причина', /\/cmd_vel/.test(ros) && /rclpy/.test(ros), ros);
check('препятствия с дальномера в цифрах',
  /слева 0,20/.test(registry.get('sc-ai-obst').textContent) &&
  /центр 0,90/.test(registry.get('sc-ai-obst').textContent),
  registry.get('sc-ai-obst').textContent);
check('задание с расстоянием до цели',
  /площадка Б/.test(registry.get('sc-ai-goal').textContent) &&
  /1,8 м/.test(registry.get('sc-ai-goal').textContent),
  registry.get('sc-ai-goal').textContent);
check('журнал команд заполнен',
  /12:00:09/.test(registry.get('sc-ai-log').innerHTML), registry.get('sc-ai-log').innerHTML);

/* --- 2. Панель без блока ИИ: реальные модули ----------------------------- */
const offState = { ai: { available: false, reason: 'реальные модули: ИИ ведёт только стенд' } };
check('без блока ИИ панель говорит об этом',
  aiCtx.RSAiPanel.render(offState) === false &&
  registry.get('sc-ai-command').textContent === 'ИИ недоступен' &&
  /реальные модули/.test(registry.get('sc-ai-intent').textContent));

/* --- 3. Карта: разбор клеток и отрисовка --------------------------------- */
// подвал карты держит разметка киоска (main.html), а не модуль
['sc-map-stats', 'sc-map-pose', 'sc-map-goal'].forEach((id) => registry.set(id, makeEl('span', { id: id })));
const mapCanvas = makeCanvas();
registry.set('sc-map', mapCanvas);

// борт отдаёт клетки: 8×4 по 0,05 м, всё свободно
const cellsAnswer = { ok: true, w: 8, h: 4, res: 0.05, version: 7, cells: '1*32' };
const mapCtx = load(MAP_JS, {
  document: dom,
  fetch: () => Promise.resolve({ json: () => Promise.resolve(cellsAnswer) }),
});

const decoded = mapCtx.RSMap.decodeRle('0*20,1*8,2*4', 32);
check('клетки борта разбираются верно',
  decoded.length === 32 && decoded[0] === 0 && decoded[20] === 1 && decoded[31] === 2);

mapCtx.RSMap.tick(state);
check('карта нарисована по состоянию борта', mapCanvas.calls.length > 0);
const ops = mapCanvas.calls.map((c) => c[0]);
check('на карте есть заливка свободного и препятствий', ops.filter((o) => o === 'fillRect').length >= 2);
check('робот нарисован',
  ops.indexOf('stroke') >= 0 && mapCanvas.calls.some((c) => c[0] === 'rotate'));
check('площадка подписана',
  mapCanvas.calls.some((c) => c[0] === 'fillText' && c[1][0] === 'А'));
check('карта борта считается настоящей', mapCtx.RSMap.active() === true);
check('подвал карты: разведано и задание',
  /разведано 42%/.test(registry.get('sc-map-stats').textContent) &&
  /площадка Б/.test(registry.get('sc-map-goal').textContent),
  registry.get('sc-map-stats').textContent + ' / ' + registry.get('sc-map-goal').textContent);
check('карта без данных борта не выдаётся за настоящую',
  mapCtx.RSMap.tick({ map: { ok: false } }) === false);

/* --- 4. Клик по карте борта читает клетку -------------------------------- */
(async function readCell() {
  mapCtx.RSMap.tick(state);                   // вернуть карту борта после проверки отказа
  await new Promise((resolve) => setTimeout(resolve, 0));   // клетки приходят запросом
  const free = mapCtx.RSMap.worldAt(340, 200);
  check('клик по карте даёт мировые координаты',
    free && free.x > 0 && free.y > 0 && free.x < 0.4 && free.y < 0.2,
    free ? free.x.toFixed(3) + ', ' + free.y.toFixed(3) : 'нет точки');
  check('клетка под кликом читается: состояние известно',
    free && free.cell && ['свободно', 'препятствие', 'не разведано'].indexOf(free.cell.text) >= 0,
    free && free.cell ? free.cell.text : 'нет клетки');
  const outside = mapCtx.RSMap.worldAt(0, 0);
  check('за пределами карты клетки нет', outside && !outside.cell);

  console.log(failures.length ? '\nошибок: ' + failures.length
    : '\nпанель ИИ и карта: разметка и данные сходятся');
  process.exit(failures.length ? 1 : 0);
}());
