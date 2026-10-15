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
  }
  addEventListener() {}
  removeEventListener() {}
  appendChild(child) { this.children.push(child); child.parentElement = this; return child; }
  querySelector() { return null; }
  querySelectorAll() { return []; }
  getContext() { if (!this._ctx) this._ctx = makeCtx(this); return this._ctx; }
  getBoundingClientRect() { return { width: 320, height: 320, top: 0, left: 0 }; }
  setAttribute() {}
  getAttribute() { return null; }
  focus() {}
  click() {}
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
  createTextNode: (text) => ({ textContent: text }),
  querySelector: () => null,
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
// main.js вешает обработчики на window: в браузере это событие, здесь — заглушка.
global.addEventListener = () => {};
global.removeEventListener = () => {};
global.dispatchEvent = () => true;
global.location = { search: '', hash: '', href: 'http://127.0.0.1:8090/kiosk', protocol: 'http:' };
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
global.fetch = async (url) => {
  const text = String(url);
  if (/camera/.test(text)) {
    return { ok: true, status: 200, json: async () => ({ ok: true, cameras: [] }) };
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

  const meta = byId('lidar-meta');
  const count = byId('lidar-count');
  const near = byId('lidar-near');
  const empty = byId('lidar-empty');
  const canvas = byId('lidar-canvas');
  console.log(JSON.stringify({
    screen,
    meta: meta.textContent,
    count: count.textContent,
    near: near.textContent,
    emptyHidden: empty.hidden,
    emptyText: empty.textContent,
    emptyTitle: empty.title,
    dots: canvas.dots.length,
    renderErrors,
    otherWarnings,
  }, null, 2));
  console.warn = originalWarn;
  console.error = originalError;
})().catch((err) => {
  originalError(err && err.stack ? err.stack : String(err));
  process.exit(1);
});
