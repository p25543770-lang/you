/* Проверка отрисовки карты лидара в браузере — без браузера.
 *
 * Экраны робота (slam_gui/main.js и slam_gui/robot.js) написаны под браузер и
 * ничего не экспортируют, поэтому здесь поднимается минимальный DOM, скрипт
 * выполняется как в странице, а результат читается из тех же элементов, которые
 * видит оператор: подпись под картой (#lidar-meta), счётчик точек
 * (#lidar-count), плашка «скана лидара нет» (#lidar-empty) и число нарисованных
 * точек на канве.
 *
 * Запуск (из теста tests/test_gui_lidar_render.py или вручную):
 *   node tests/js/lidar_screen.js <main|robot> <файл-payload.json>
 * На stdout — JSON: { meta, count, near, emptyHidden, emptyText, dots, title }.
 */
'use strict';

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const ROOT = path.resolve(__dirname, '..', '..');
const screen = process.argv[2] || 'robot';
const payloadFile = process.argv[3];
if (!payloadFile) {
  console.error('нужен файл с ответом API');
  process.exit(2);
}
const payload = JSON.parse(fs.readFileSync(payloadFile, 'utf8'));

/* ------------------------------------------------------------------ DOM */
class ClassList {
  constructor() { this.set = new Set(); }
  add(...names) { names.forEach((n) => this.set.add(n)); }
  remove(...names) { names.forEach((n) => this.set.delete(n)); }
  contains(name) { return this.set.has(name); }
  toggle(name, force) {
    const on = force === undefined ? !this.set.has(name) : !!force;
    if (on) this.set.add(name); else this.set.delete(name);
    return on;
  }
}

function makeCtx(el) {
  // Контекст канвы: всё, что рисует точки, запоминаем (по fillRect видно,
  // сколько точек скана реально попало на карту).
  const target = {
    canvas: el,
    fillRect: (...args) => { el.dots.push(args); },
    clearRect: () => { el.dots = []; },
    measureText: () => ({ width: 10 }),
  };
  return new Proxy(target, {
    get(obj, prop) {
      if (prop in obj) return obj[prop];
      return () => {};
    },
    set(obj, prop, value) { obj[prop] = value; return true; },
  });
}

class El {
  constructor(id, tag) {
    this.id = id || '';
    this.tagName = (tag || 'div').toUpperCase();
    this.textContent = '';
    this.innerHTML = '';
    this.hidden = false;
    this.title = '';
    this.value = '0';
    this.checked = false;
    this.disabled = false;
    this.style = { setProperty() {}, removeProperty() {}, cssText: '' };
    this.dataset = {};
    this.classList = new ClassList();
    this.children = [];
    this.parentElement = null;
    this.clientWidth = 320;
    this.clientHeight = 320;
    this.width = 320;
    this.height = 320;
    this.dots = [];
    this.readyState = 'complete';
    this._innerHTML = '';
    this.events = {};
  }
  /* Панель логов чистит список через `innerHTML = ''` — в браузере это
   * снимает прежние строки, поэтому и здесь они не должны накапливаться. */
  /* `el.className = 'a b'` в браузере заменяет classList — здесь так же,
   * иначе роль и подсветка порта в блоке «Порты на роботе» не читались бы. */
  get className() { return Array.from(this.classList.set).join(' '); }
  set className(value) {
    this.classList.set = new Set(String(value || '').split(/\s+/).filter(Boolean));
  }
  get innerHTML() { return this._innerHTML; }
  set innerHTML(value) {
    this._innerHTML = value;
    if (value === '') this.children = [];
  }
  addEventListener(name, listener) { (this.events[name] ||= []).push(listener); }
  removeEventListener() {}
  appendChild(child) { this.children.push(child); child.parentElement = this; return child; }
  querySelector() { return null; }
  querySelectorAll() { return []; }
  getContext() { if (!this._ctx) this._ctx = makeCtx(this); return this._ctx; }
  getBoundingClientRect() { return { width: 320, height: 320, top: 0, left: 0 }; }
  setAttribute() {}
  getAttribute() { return null; }
  focus() {}
  click() { (this.events.click || []).forEach((listener) => listener({ target: this, preventDefault() {} })); }
  closest() { return null; }
  play() { return Promise.resolve(); }
}

const elements = new Map();
function byId(id) {
  if (!elements.has(id)) {
    elements.set(id, new El(id, /canvas/.test(id) ? 'canvas' : 'div'));
  }
  return elements.get(id);
}

/* ------------------------------------------------------- раскладка окна
 * Экраны читают геометрию только через DOM (clientWidth/clientHeight,
 * getBoundingClientRect), поэтому «настоящую» раскладку здесь заменяют
 * прямоугольники: шапка сверху, ниже колонки, справа — панель лидара.
 * Числа повторяют slam_gui/robot.css: .kiosk padding 22/30, .lidar padding
 * 18/24/22, шапка панели ~46 px, строка показателей ~96 px, .lidar-body gap 14.
 *
 * Переменные окружения (без них геометрия прежняя — 320×320):
 *   WINDOW=1430x1300   — окно киоска;
 *   MAP_AREA=560x1100  — размер области карты лидара, как его посчитала
 *                        страница (проверка «контейнер 560×1100 / 1100×560»);
 *   RESIZE_WINDOW=WxH  — во время прогона окно меняют (поворот экрана);
 *   RESIZE_MAP=WxH     — во время прогона меняют размер области карты.
 */
function parseSize(value) {
  const match = /^(\d+)\s*[xх×]\s*(\d+)$/.exec(String(value || '').trim());
  return match ? { w: Number(match[1]), h: Number(match[2]) } : null;
}

const PANEL_PAD = { x: 24, top: 18, bottom: 22 };
const PANEL_HEAD = 46;      // заголовок панели: h2 + отступ
const STATS_H = 96;         // строка трёх плиток показателей
const BODY_GAP = 14;        // .lidar-body gap
const HEAD_H = 96;          // шапка экрана
const KIOSK_PAD = { x: 30, y: 22 };

function layoutBoxes(win, area) {
  /* Панель лидара — в правой колонке (окно шире 1200): слева шасси требует
     своё, панели остаётся остаток. Узкое окно — одна колонка на всю ширину. */
  const colsW = Math.max(0, win.w - 2 * KIOSK_PAD.x);
  const wide = win.w > 1200;
  let panelW = wide ? Math.max(140, Math.round(colsW * 0.2) - 40) : colsW;
  if (area) panelW = area.w + 2 * PANEL_PAD.x;
  const panelX = wide ? win.w - KIOSK_PAD.x - panelW : KIOSK_PAD.x;
  const panelTop = KIOSK_PAD.y + HEAD_H + 18;
  const slotW = area ? area.w : Math.max(0, panelW - 2 * PANEL_PAD.x);
  const slotTop = panelTop + PANEL_PAD.top + PANEL_HEAD;
  const slotH = area
    ? area.h
    : Math.max(0, win.h - KIOSK_PAD.y - panelTop - PANEL_PAD.bottom - STATS_H - BODY_GAP);
  const panelH = PANEL_PAD.top + PANEL_HEAD + slotH + BODY_GAP + STATS_H + PANEL_PAD.bottom;
  return {
    win,
    panel: { x: panelX, y: panelTop, w: panelW, h: panelH },
    slot: { x: panelX + PANEL_PAD.x, y: slotTop, w: slotW, h: slotH },
  };
}

const layoutRequest = {
  window: parseSize(process.env.WINDOW),
  area: parseSize(process.env.MAP_AREA),
};
const resizeRequest = {
  window: parseSize(process.env.RESIZE_WINDOW),
  area: parseSize(process.env.RESIZE_MAP),
};
const layoutEnabled = Boolean(layoutRequest.window || layoutRequest.area ||
                               resizeRequest.window || resizeRequest.area);
let layoutState = layoutEnabled
  ? layoutBoxes(layoutRequest.window || { w: 1430, h: 1300 }, layoutRequest.area)
  : null;

function rectOf(box) {
  return {
    x: box.x, y: box.y, width: box.w, height: box.h,
    top: box.y, left: box.x, right: box.x + box.w, bottom: box.y + box.h,
  };
}

/* Внутри области карты лежит .lidar-map — квадрат: либо сторона, посчитанная
   экраном (robot.js → sizeLidarMap ставит width/height в px), либо то, что
   даёт CSS (aspect-ratio по меньшей стороне области, по центру области). */
function mapBox() {
  const slot = layoutState.slot;
  const style = byId('lidar-map').style || {};
  const cssSide = Math.min(slot.w, slot.h);
  const w = parseFloat(style.width) || cssSide;
  const h = parseFloat(style.height) || cssSide;
  return { x: slot.x + (slot.w - w) / 2, y: slot.y + (slot.h - h) / 2, w, h };
}

function applyLayout() {
  if (!layoutEnabled) return;
  const boxes = {
    'kiosk-root': { x: 0, y: 0, w: layoutState.win.w, h: layoutState.win.h },
    'lidar-map-slot': layoutState.slot,
    'lidar-map': mapBox(),
  };
  Object.keys(boxes).forEach((id) => {
    const el = byId(id);
    const box = boxes[id];
    el.clientWidth = Math.floor(box.w);
    el.clientHeight = Math.floor(box.h);
    el.getBoundingClientRect = () => rectOf(box);
  });
  // Канва занимает всю карту (robot.css: #lidar-canvas width/height 100%), а у
  // карты рамка 1 px — поэтому размеры канвы считаем от карты каждый раз
  // заново: экран может только что задать ей сторону.
  const canvas = byId('lidar-canvas');
  const canvasBox = () => {
    const map = mapBox();
    return { x: map.x + 1, y: map.y + 1, w: Math.max(0, map.w - 2), h: Math.max(0, map.h - 2) };
  };
  Object.defineProperty(canvas, 'clientWidth', { get: () => Math.floor(canvasBox().w), configurable: true });
  Object.defineProperty(canvas, 'clientHeight', { get: () => Math.floor(canvasBox().h), configurable: true });
  canvas.getBoundingClientRect = () => rectOf(canvasBox());
}

function layoutSnapshot() {
  if (!layoutEnabled) return null;
  const slot = layoutState.slot;
  const map = mapBox();
  const canvas = byId('lidar-canvas');
  const style = byId('lidar-map').style || {};
  const dpr = global.devicePixelRatio || 1;
  return {
    window: layoutState.win,
    area: { w: slot.w, h: slot.h },
    side: Math.min(slot.w, slot.h),
    slot: { x: slot.x, y: slot.y, w: slot.w, h: slot.h },
    map: { w: map.w, h: map.h, x: map.x, y: map.y },
    canvas: {
      clientW: canvas.clientWidth, clientH: canvas.clientHeight,
      attrW: canvas.width, attrH: canvas.height,
      roundedW: Math.round(canvas.clientWidth * dpr), dpr,
    },
    mapStyle: { width: style.width, height: style.height, paddingBottom: style.paddingBottom },
    dots: canvas.dots.length,
  };
}

applyLayout();

const body = new El('body');
const documentElement = new El('html');
global.document = {
  readyState: 'complete',
  body,
  documentElement,
  title: '',
  getElementById: byId,
  createElement: (tag) => new El('', tag),
  createDocumentFragment: () => new El('', 'fragment'),
  createTextNode: (text) => ({ textContent: text }),
  /* Свайп-панель камер и логов объявлена на `.kiosk`; без него весь блок
   * (в том числе панель логов) не выполняется — отдаём ему элемент. */
  querySelector: (selector) => (selector === '.kiosk' ? byId('kiosk-root') : null),
  querySelectorAll: () => [],
  addEventListener: () => {},
  removeEventListener: () => {},
  hidden: false,
};

const rafQueue = [];
const timeouts = [];
/* Ошибки отрисовки экраны логируют в console — собираем их, чтобы тест мог
 * требовать «отрисовка прошла без сбоев»: именно так ловится падение одного
 * блока, из-за которого карта лидара перестаёт обновляться. */
const renderErrors = [];
const otherWarnings = [];
const RENDER_MARKERS = ['не нарисован', 'сбой отрисовки'];
const originalWarn = console.warn.bind(console);
const originalError = console.error.bind(console);
console.warn = (...args) => {
  const line = args.map((a) => String(a)).join(' ');
  if (RENDER_MARKERS.some((m) => line.includes(m))) renderErrors.push(line);
  else otherWarnings.push(line);
};
console.error = (...args) => {
  renderErrors.push(args.map((a) => String(a)).join(' '));
};
global.window = global;
global.self = global;
global._kioskBms = payload.bms || null;
// Обработчики window в браузере — события; здесь их запоминаем, чтобы
// проверка раскладки могла в нужный момент выстрелить `resize` (поворот
// экрана) и убедиться, что канва лидара перерисовалась по новому размеру.
const windowHandlers = new Map();
global.addEventListener = (name, listener) => {
  if (typeof listener !== 'function') return;
  if (!windowHandlers.has(name)) windowHandlers.set(name, []);
  windowHandlers.get(name).push(listener);
};
global.removeEventListener = (name, listener) => {
  const list = windowHandlers.get(name) || [];
  const index = list.indexOf(listener);
  if (index >= 0) list.splice(index, 1);
};
function fireWindowEvent(name) {
  (windowHandlers.get(name) || []).slice().forEach((listener) => {
    try { listener({ type: name }); } catch (err) { /* экран не должен падать */ }
  });
}
global.dispatchEvent = () => true;
/* HARNESS_DEMO=1 — экран с явно включённым демо-режимом (`?demo=1`). */
global.location = {
  search: process.env.HARNESS_DEMO === '1' ? '?demo=1' : '',
  hash: '',
  href: 'http://127.0.0.1:8090/kiosk',
  protocol: 'http:',
};
global.localStorage = {
  store: {},
  getItem(key) { return Object.prototype.hasOwnProperty.call(this.store, key) ? this.store[key] : null; },
  setItem(key, value) { this.store[key] = String(value); },
  removeItem(key) { delete this.store[key]; },
};
// В Node 22 global.navigator — только для чтения, поэтому определяем свой.
Object.defineProperty(global, 'navigator', {
  value: { userAgent: 'node-harness', vibrate: () => true, language: 'ru' },
  configurable: true,
  writable: true,
});
global.getComputedStyle = () => ({ fontFamily: 'sans-serif', getPropertyValue: () => '' });
global.requestAnimationFrame = (cb) => { rafQueue.push(cb); return rafQueue.length; };
global.cancelAnimationFrame = () => {};
global.matchMedia = () => ({ matches: false, addEventListener() {}, removeEventListener() {} });
global.URLSearchParams = URLSearchParams;
global.AbortController = class {
  constructor() { this.signal = { aborted: false }; }
  abort() { this.signal.aborted = true; }
};
/* Ответ на журнал событий (/api/kiosk_logs). Если заготовка его не несёт —
 * отдаём пустой настоящий журнал: экран не должен ничего выдумывать сам. */
const emptyJournal = {
  ok: true,
  data: { entries: [], count: 0, shown: 0, dropped: 0, limit: 400,
          levels: { info: 0, ok: 0, warn: 0, err: 0, debug: 0 },
          uptimeSec: 0, source: null, demo: false, journal: 'robot' },
};
const fetchCalls = [];
global.fetch = async (url, options) => {
  const text = String(url);
  fetchCalls.push({ url: text, method: (options && options.method) || 'GET', body: options && options.body ? String(options.body) : null });
  if (/lidar\/find/.test(text)) {
    return { ok: true, status: 200, json: async () => ({ ok: true, find: { active: true, events: [], expires_in_s: 180 } }) };
  }
  if (/camera/.test(text)) {
    return { ok: true, status: 200, json: async () => ({ ok: true, cameras: [] }) };
  }
  if (/kiosk_logs/.test(text)) {
    return { ok: true, status: 200, json: async () => payload.logs || emptyJournal };
  }
  return { ok: true, status: 200, json: async () => payload };
};

const originalSetTimeout = global.setTimeout;
global.setTimeout = (cb, ms) => { timeouts.push(cb); return timeouts.length; };
global.setInterval = () => 0;
global.clearInterval = () => {};

/* ------------------------------------------------------------- запуск */
const file = path.join(ROOT, 'slam_gui', screen === 'main' ? 'main.js' : 'robot.js');
vm.runInThisContext(fs.readFileSync(file, 'utf8'), { filename: file });

const sleep = (ms) => new Promise((resolve) => originalSetTimeout(resolve, ms));

/* Кадры отрисовки: и requestAnimationFrame, и setTimeout под нашим контролем. */
async function flushFrames(rounds) {
  for (let i = 0; i < rounds; i += 1) {
    await sleep(5);
    while (rafQueue.length) {
      const cb = rafQueue.shift();
      try { cb(Date.now()); } catch (err) { /* экран не должен падать */ }
    }
    while (timeouts.length) {
      const cb = timeouts.shift();
      try { cb(); } catch (err) { /* dito */ }
    }
  }
}

(async function run() {
  await flushFrames(6);

  // Смена раскладки: окно повернули или контейнер стал другим (560×1100 ↔
  // 1100×560). Экран узнаёт об этом событием resize, а не только кадром
  // телеметрии, поэтому проверяем ровно это: размеры до и после.
  let layoutBefore = layoutSnapshot();
  let layoutAfter = null;
  if (resizeRequest.area || resizeRequest.window) {
    layoutState = layoutBoxes(resizeRequest.window || layoutRequest.window || { w: 1430, h: 1300 },
                             resizeRequest.area || layoutRequest.area);
    applyLayout();
    fireWindowEvent('orientationchange');
    fireWindowEvent('resize');
    await flushFrames(4);
    layoutAfter = layoutSnapshot();
  }

  const navigation = [];
  if (screen === 'robot' && payload.batteryNavigation) {
    byId('ctrl-overlay').hidden = true;
    function state() {
      return { battery: !byId('swipe-battery-view').hidden,
               cameras: !byId('swipe-cameras-view').hidden,
               motors: !byId('swipe-motors-view').hidden,
               open: byId('swipe-page').classList.contains('is-open'),
               title: byId('swipe-title-text').textContent };
    }
    byId('battery-open').click(); navigation.push(state());
    global._openSwipe('motors'); navigation.push(state());
    byId('swipe-battery-open').click(); navigation.push(state());
    byId('swipe-close').click(); navigation.push(state());
    global._openSwipe('cameras'); navigation.push(state());
  }

  const meta = byId('lidar-meta');
  const count = byId('lidar-count');
  const near = byId('lidar-near');
  const empty = byId('lidar-empty');
  const canvas = byId('lidar-canvas');
  // Панель «Логи»: строки, которые видит оператор (время · уровень · текст).
  // Скрипт собирает строки во фрагменте и лишь потом вставляет его в список —
  // в браузере фрагмент не остаётся узлом, поэтому разворачиваем его сами.
  const logRows = [];
  (byId('log-list').children || []).forEach((child) => {
    if (child.tagName === 'FRAGMENT') logRows.push(...(child.children || []));
    else logRows.push(child);
  });
  const logLines = logRows
    .map((row) => (row.children || []).map((cell) => cell.textContent))
    .filter((parts) => parts.length)
    .map((parts) => ({ time: parts[0], level: parts[1], msg: parts.slice(2).join(' ') }));
  /* Блок «Порты на роботе» (бортовой экран): по строке на порт — текст всех
   * вложенных элементов и классы (роль, подсветка режима «найти лидар»). */
  function textOf(el) {
    if (!el) return '';
    const own = el.textContent ? [String(el.textContent)] : [];
    return own.concat((el.children || []).map(textOf)).filter(Boolean).join(' ');
  }
  const lidarPorts = (byId('lidar-ports-list').children || []).map((li) => ({
    className: Array.from(li.classList.set).join(' '),
    text: textOf(li),
  }));
  /* «Продолжить без лидара»: клик, кадр отрисовки, затем — состояние плашки
   * и предупреждения; второй клик по предупреждению возвращает плашку. */
  const continueFlow = [];
  if (screen === 'robot' && process.env.HARNESS_CLICK_CONTINUE === '1') {
    const snap = () => ({ blocked: !byId('lidar-blocker').hidden, warn: !byId('lidar-warn-bar').hidden,
                          warnText: byId('lidar-warn-text').textContent, emptyText: byId('lidar-empty').textContent,
                          dots: byId('lidar-canvas').dots.length });
    byId('lidar-continue-btn').click(); continueFlow.push(snap());
    byId('lidar-warn-bar').click(); continueFlow.push(snap());
  }
  if (screen === 'robot' && process.env.HARNESS_CLICK_FIND === '1') {
    byId('lidar-find-btn').click();
    await sleep(10);
  }
  console.log(JSON.stringify({
    screen,
    // Геометрия панели лидара — только когда её задали проверки раскладки.
    layout: layoutEnabled ? { window: layoutState.win, before: layoutBefore, after: layoutAfter } : null,
    navigation,
    lidarPorts,
    continueFlow,
    lidarFindStatus: byId('lidar-find-status').textContent,
    lidarFindButton: byId('lidar-find-btn').textContent,
    lidarPortsEmpty: byId('lidar-ports-empty').hidden ? '' : byId('lidar-ports-empty').textContent,
    lidarPlaceNote: byId('lidar-place-note').hidden ? '' : byId('lidar-place-note').textContent,
    fetchCalls,
    soc: byId('k-soc').textContent,
    volts: byId('k-volts').textContent,
    batteryState: byId('k-soc-state').textContent,
    meta: meta.textContent,
    count: count.textContent,
    near: near.textContent,
    emptyHidden: empty.hidden,
    emptyText: empty.textContent,
    emptyTitle: empty.title,
    lidarBlocked: !byId('lidar-blocker').hidden,
    lidarBlockTitle: byId('lidar-block-title').textContent,
    lidarBlockReason: byId('lidar-block-reason').textContent,
    lidarBlockDetails: byId('lidar-block-details').textContent,
    lidarBlockHint: byId('lidar-block-hint').textContent,
    dots: canvas.dots.length,
    logLines,
    logStatus: byId('log-status').textContent,
    renderErrors,
    otherWarnings,
  }, null, 2));
  console.warn = originalWarn;
  console.error = originalError;
})().catch((err) => {
  originalError(err && err.stack ? err.stack : String(err));
  process.exit(1);
});
