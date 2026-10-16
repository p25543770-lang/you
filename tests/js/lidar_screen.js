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
// main.js вешает обработчики на window: в браузере это событие, здесь — заглушка.
global.addEventListener = () => {};
global.removeEventListener = () => {};
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

(async function run() {
  // Даём fetch из boot() завершиться, затем исполняем отложенные кадры
  // отрисовки: и requestAnimationFrame, и setTimeout здесь под нашим контролем.
  for (let i = 0; i < 6; i += 1) {
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
