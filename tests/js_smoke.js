/* ============================================================================
 * js_smoke.js — прогон скриптов страницы в пустом DOM.
 *
 * Зачем: на экране киоска часть блоков удалена (грузовой отсек, его клавиатура
 * и строка состояния в подвале), а код страницы остался. Все обращения к DOM
 * обязаны быть защищёнными: если скрипт дёрнет удалённый элемент без проверки
 * или вызовет удалённую функцию, экран в браузере останется пустым.
 *
 * Заглушка отдаёт null на любой поиск элемента — то же самое, что «блока на
 * странице нет». Если скрипт переживает это без исключений, значит он честно
 * деградирует, а не падает.
 *
 * Запуск:  node tests/js_smoke.js slam_gui/main.js slam_gui/instrument.js
 * Код возврата: 0 — все файлы прошли, 1 — есть исключения.
 * ========================================================================== */
'use strict';

const fs = require('fs');
const vm = require('vm');

const noop = () => {};
const anyFn = new Proxy(function () {}, {
  get: () => anyFn,
  apply: () => anyFn,
  set: () => true,
});

function makeCtx() {
  return new Proxy({}, {
    get: (t, p) => (p in t ? t[p] : anyFn),
    set: (t, p, v) => { t[p] = v; return true; },
  });
}

function makeEl(tag) {
  return {
    tagName: String(tag).toUpperCase(),
    style: new Proxy({}, { get: (t, p) => (p in t ? t[p] : ''), set: (t, p, v) => { t[p] = v; return true; } }),
    dataset: {},
    classList: { add: noop, remove: noop, toggle: noop, contains: () => false },
    children: [], childNodes: [],
    innerHTML: '', textContent: '', value: '',
    width: 0, height: 0,
    appendChild: (c) => c, removeChild: noop, remove: noop,
    addEventListener: noop, removeEventListener: noop,
    setAttribute: noop, removeAttribute: noop, getAttribute: () => null,
    querySelector: () => null, querySelectorAll: () => [],
    getContext: () => makeCtx(),
    focus: noop, blur: noop, click: noop, scrollIntoView: noop,
    getBoundingClientRect: () => ({ top: 0, left: 0, width: 320, height: 120, right: 320, bottom: 120 }),
  };
}

const documentStub = {
  readyState: 'complete',
  title: 'smoke',
  hidden: false,
  visibilityState: 'visible',
  body: makeEl('body'),
  documentElement: makeEl('html'),
  getElementById: () => null,
  querySelector: () => null,
  querySelectorAll: () => [],
  createElement: makeEl,
  createElementNS: (_ns, tag) => makeEl(tag),
  addEventListener: noop,
  removeEventListener: noop,
};

const errors = [];
const sandbox = {
  document: documentStub,
  console,
  setInterval: () => 0, clearInterval: noop, setTimeout: () => 0, clearTimeout: noop,
  requestAnimationFrame: () => 0, cancelAnimationFrame: noop,
  performance: { now: () => Date.now() },
  fetch: () => Promise.reject(new Error('сети нет (заглушка)')),
  localStorage: { getItem: () => null, setItem: noop, removeItem: noop },
  navigator: { userAgent: 'smoke' },
  location: { href: 'http://robot/', hash: '', search: '', pathname: '/' },
  matchMedia: () => ({ matches: false, addEventListener: noop }),
  getComputedStyle: () => ({ getPropertyValue: () => '' }),
  devicePixelRatio: 1,
  addEventListener: noop, removeEventListener: noop,
  innerWidth: 1280, innerHeight: 800,
  btoa: (s) => Buffer.from(String(s), 'binary').toString('base64'),
  atob: (s) => Buffer.from(String(s), 'base64').toString('binary'),
  URL, URLSearchParams, TextEncoder, TextDecoder,
  Math, Date, JSON, Promise, Object, Array, Error, TypeError,
  String, Number, Boolean, RegExp, Map, Set, isNaN, parseInt, parseFloat,
};
sandbox.window = sandbox;
sandbox.self = sandbox;
sandbox.globalThis = sandbox;

const files = process.argv.slice(2);
if (!files.length) {
  console.log('укажите файлы: node tests/js_smoke.js slam_gui/main.js …');
  process.exit(2);
}

for (const f of files) {
  const code = fs.readFileSync(f, 'utf8');
  try {
    vm.runInNewContext(code, sandbox, { filename: f });
    console.log('OK   ' + f);
  } catch (e) {
    console.log('FAIL ' + f + ' → ' + e.message);
    errors.push(f + ': ' + e.message);
  }
}

// дать шанс упасть отложенным промисам
setTimeout(() => {
  if (errors.length) {
    console.log('исключения:\n  ' + errors.join('\n  '));
    process.exit(1);
  }
  console.log('все скрипты пережили пустой DOM');
}, 50);

process.on('unhandledRejection', (e) => {
  errors.push('необработанный reject: ' + (e && e.message));
});
