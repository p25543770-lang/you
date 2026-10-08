/* ============================================================================
 * js_motors.js — панель двигателей: схема робота и направление колёс.
 *
 * Проверяем на заглушке DOM, что разметка панели и её отрисовка сходятся:
 *   • в разметке есть корпус робота и четыре колеса (FL/FR/RL/RR);
 *   • колесо поворачивается на свой угол, как его отдал борт;
 *   • рядом направление написано словами («влево», «прямо», «вправо»);
 *   • в подвал панели попадает текущий манёвр.
 *
 * Раньше панель показывала четыре стрелочных прибора с одним и тем же
 * рисунком, а знак минус перед углом ничего не говорил о направлении.
 *
 * Запуск:  node tests/js_motors.js
 * ========================================================================== */
'use strict';

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const MAIN_JS = path.join(__dirname, '..', 'slam_gui', 'main.js');
const failures = [];

function check(label, ok, detail) {
  console.log((ok ? 'OK   ' : 'FAIL ') + label + (detail ? ' → ' + detail : ''));
  if (!ok) failures.push(label + (detail ? ': ' + detail : ''));
}

function makeEl(id) {
  const el = {
    id: id || '',
    style: {},
    dataset: {},
    _html: '',
    textContent: '',
    value: '',
    classList: {
      _set: new Set(),
      add(c) { this._set.add(c); },
      remove(c) { this._set.delete(c); },
      toggle(c, on) { if (on === undefined ? !this._set.has(c) : on) this._set.add(c); else this._set.delete(c); },
      contains(c) { return this._set.has(c); },
    },
    set innerHTML(v) {
      this._html = String(v);
      // как настоящий DOM: id из разметки становятся элементами страницы
      const ids = String(v).match(/id="([^"]+)"/g) || [];
      ids.forEach((raw) => {
        const child = raw.slice(4, -1);
        if (!registry.has(child)) registry.set(child, makeEl(child));
      });
    },
    get innerHTML() { return this._html; },
    addEventListener() {}, removeEventListener() {},
    setAttribute(k, v) { this['attr_' + k] = v; },
    removeAttribute() {}, getAttribute() { return null; },
    appendChild(c) { return c; }, remove() {},
    querySelector() { return null; }, querySelectorAll() { return []; },
    getContext() { return null; },
    getBoundingClientRect() { return { width: 320, height: 120, top: 0, left: 0 }; },
  };
  return el;
}

const registry = new Map();
['sc-motor-grid', 'sc-motors-sum', 'sc-clock', 'sc-mode', 'sc-data', 'sc-link-dot',
 'sc-link-text', 'sc-ring-fill', 'sc-soc', 'sc-soc-label', 'sc-volts', 'sc-amps',
 'sc-range', 'sc-batt-state', 'sc-batt-hint', 'sc-console-link'].forEach((id) => {
  registry.set(id, makeEl(id));
});
registry.get('sc-motor-grid').id = 'sc-motor-grid';

const state = {
  ok: true,
  data: {
    motors: [
      { id: 'FL', title: 'передний левый', angle: 26, rpm: 210, temp: 35, homed: true },
      { id: 'FR', title: 'передний правый', angle: 34, rpm: 213, temp: 40, homed: true },
      { id: 'RL', title: 'задний левый', angle: 0, rpm: 215, temp: 42, homed: true },
      { id: 'RR', title: 'задний правый', angle: -90, rpm: -216, temp: 40, homed: true },
    ],
    battery: { soc: 76, volts: 41.2, amps: 12.4, rangeKm: 14.2, state: 'разряд', level: 'НОРМА' },
    mode: 'АВТОНОМНЫЙ РЕЖИМ',
    driveMode: 'поворот вправо',
    route: 'склад → зона выгрузки',
    powerKw: 2.4,
    linkOk: true,
  },
};

const sandbox = {
  console,
  document: {
    readyState: 'complete',
    title: 'motors',
    body: makeEl('body'),
    documentElement: makeEl('html'),
    getElementById: (id) => (registry.has(id) ? registry.get(id) : null),
    querySelector: () => null,
    querySelectorAll: () => [],
    createElement: () => makeEl(''),
    createElementNS: () => makeEl(''),
    addEventListener() {}, removeEventListener() {},
  },
  setTimeout: (fn) => { if (typeof fn === 'function') fn(); return 0; },
  clearTimeout() {},
  setInterval: () => 0,
  clearInterval() {},
  requestAnimationFrame: () => 0,
  performance: { now: () => Date.now() },
  fetch: () => Promise.resolve({
    ok: true,
    json: async () => ({ ok: true, source: 'sim', data: state.data }),
  }),
  localStorage: { getItem: () => null, setItem() {} },
  navigator: { userAgent: 'test' },
  location: { href: 'https://host/', search: '', origin: 'https://host' },
  matchMedia: () => ({ matches: false, addEventListener() {} }),
  getComputedStyle: () => ({ getPropertyValue: () => '' }),
  devicePixelRatio: 1,
  addEventListener() {}, removeEventListener() {},
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

vm.runInNewContext(fs.readFileSync(MAIN_JS, 'utf8'), sandbox, { filename: 'main.js' });

const gridHtml = registry.get('sc-motor-grid').innerHTML;
const el = (id) => registry.get(id);

// 1. Разметка схемы собрана целиком
check('в панели есть корпус робота', /id="sc-robot"/.test(gridHtml));
check('в панели четыре колеса', ['FL', 'FR', 'RL', 'RR']
  .every((id) => gridHtml.indexOf('id="mc-wheel-' + id + '"') >= 0));
check('у робота помечен перед', /robot-front[\s\S]*?перед/.test(gridHtml));

// 2. Отрисовка поворачивает колёса и пишет направление словами
setTimeout(() => {
  sandbox.RS_MAIN.render();

  check('колесо FL повёрнуто на свой угол',
    String(el('mc-wheel-FL').style.transform) === 'rotate(26.0deg)', el('mc-wheel-FL').style.transform);
  check('колесо RR повёрнуто влево',
    String(el('mc-wheel-RR').style.transform) === 'rotate(-90.0deg)', el('mc-wheel-RR').style.transform);
  check('FL подписан «вправо»', el('mc-dir-FL').textContent === 'вправо', el('mc-dir-FL').textContent);
  check('RL подписан «прямо»', el('mc-dir-RL').textContent === 'прямо', el('mc-dir-RL').textContent);
  check('RR подписан «влево»', el('mc-dir-RR').textContent === 'влево', el('mc-dir-RR').textContent);
  check('угол показан без минуса', el('mc-angle-RR').textContent === '90°', el('mc-angle-RR').textContent);
  const wheelLabel = String(el('mc-wheel-RR')['attr_aria-label'] || '');
  check('подпись колеса говорит о направлении',
    /влево 90 градусов/.test(wheelLabel), wheelLabel);

  // 3. Показания модулей и манёвр
  check('об/мин попали в свою карточку', el('mc-rpm-FR').textContent === '213', el('mc-rpm-FR').textContent);
  check('температура не обрезана единицей',
    el('mc-temp-FR').innerHTML === '40<small>°C</small>', el('mc-temp-FR').innerHTML);
  check('в подвале виден манёвр',
    el('sc-motors-sum').textContent.indexOf('манёвр: поворот вправо') === 0, el('sc-motors-sum').textContent);

  if (failures.length) {
    console.log('расхождения:\n  ' + failures.join('\n  '));
    process.exit(1);
  }
  console.log('панель двигателей: схема робота, направление и показания сходятся');
}, 20);
