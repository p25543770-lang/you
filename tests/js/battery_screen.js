'use strict';
const assert = require('assert');
const fs = require('fs');
const vm = require('vm');

class El {
  constructor(tag) {
    this.tag = tag || 'div';
    this.children = []; this.textContent = ''; this.hidden = false; this.attrs = {};
    this.className = ''; this.style = {}; this.dataset = {}; this.listeners = {};
    this.classes = new Set();
    this.classList = {
      toggle: (name, on) => on ? this.classes.add(name) : this.classes.delete(name),
      contains: (name) => this.classes.has(name)
    };
  }
  append(...items) { this.children.push(...items); }
  replaceChildren(...items) { this.children = items; }
  setAttribute(key, value) { this.attrs[key] = value; }
  removeAttribute(key) { delete this.attrs[key]; }
  addEventListener(name, fn) { (this.listeners[name] = this.listeners[name] || []).push(fn); }
  fire(name) { (this.listeners[name] || []).forEach((fn) => fn({ type: name })); }
}
const elements = new Map();
const el = (id) => { if (!elements.has(id)) elements.set(id, new El()); return elements.get(id); };
const texts = (node) => [node.textContent, ...node.children.map(texts)].join(' ');
const find = (node, predicate) => {
  if (predicate(node)) return node;
  for (const child of node.children) { const hit = find(child, predicate); if (hit) return hit; }
  return null;
};
const page = el('swipe-battery-view');
el('bms-setup').hidden = true;      // как в robot.html: блок подключения скрыт
let observer;
let nextPoll;
let fail = false;
const posts = [];
let sample = {
  updatedAt: Date.now() / 1000, soc: 50, volts: 51.2, amps: -12.5, watts: -640,
  direction: 'discharge', directionText: 'разряд', remainingAh: 25, fullAh: 50,
  remainingWh: 1280, fullWh: 2560, hoursToEmpty: 2, hoursToFull: null,
  cycles: 123, temperatures: [25, 47], ntcCount: 2, firmwareText: '0x21',
  manufactureDate: '2023-03-14', manufactureDateText: '14.03.2023',
  deviceName: 'SP04S034', deviceNameState: 'ok',
  chargeOn: true, dischargeOn: false, faults: [], balancingCells: [32],
  minMv: 3200, maxMv: 3231, minCell: 1, maxCell: 32, deltaMv: 31, averageMv: 3215.5, sumVolts: 102.9,
  cellCount: 32, receivedCount: 32, complete: true,
  errorCountersState: 'ok',
  errorCounters: [{ key: 'sc', label: 'Короткое замыкание', count: 0 }, { key: 'dsgoc', label: 'Перегрузка разряда', count: 3 }],
  thresholdsState: 'ok',
  thresholds: [
    { key: 'covp', label: 'Перенапряжение банки', value: 3.65, unit: 'В' },
    { key: 'covp_rel', label: 'Возврат после перенапряжения банки', value: 3.4, unit: 'В' },
    { key: 'dsgoc', label: 'Перегрузка по разряду', value: 60, unit: 'А' },
    { key: 'design_cap', label: 'Номинальная ёмкость', value: 18.6, unit: 'Ач' }
  ],
  cells: Array.from({ length: 32 }, (_, i) => ({ number: i + 1, mv: 3200 + i, deviationMv: i - 15.5, balancing: i === 31 }))
};
let status = { connected: true, complete: true, configured: true, reason: 'Все банки прочитаны', data: sample,
  port: '/dev/ttyUSB0', connectedSince: Date.now() / 1000 - 60, setupAvailable: true };
const setup = {
  ok: true, available: true, port: null, configured: false, connected: false, requiresPin: false,
  ports: [
    { port: '/dev/ttyACM0', role: 'master', note: 'мастер руля (RC_SLAM_PORT) — не трогаем' },
    { port: '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0', role: 'free', note: 'свободен', description: 'QinHeng CH340' }
  ]
};
const context = {
  document: { getElementById: el, createElement: (tag) => new El(tag) },
  MutationObserver: class { constructor(fn) { observer = fn; } observe() {} },
  Date, AbortController, JSON, Object,
  setTimeout: (fn, ms) => { if (ms === 1000) nextPoll = fn; return 1; }, clearTimeout() {},
  setInterval: () => 2, clearInterval() {},
  confirm: () => true,
  fetch: async (url, init) => {
    if (url === 'api/battery') {
      if (fail) throw new Error('offline');
      return { ok: true, status: 200, json: async () => ({ ok: true, battery: status }) };
    }
    if (url === 'api/battery/setup') return { ok: true, status: 200, json: async () => setup };
    const body = JSON.parse(init.body);
    posts.push([url, body]);
    if (url === 'api/battery/probe') {
      return { ok: true, status: 200, json: async () => ({ ok: true, found: true, port: body.port, summary: 'JBD BMS: 41.90 В, 77 %, банок 12' }) };
    }
    if (url === 'api/battery/port') return { ok: true, status: 200, json: async () => ({ ok: true, port: body.port, saved: true }) };
    throw new Error('unexpected ' + url);
  }
};
const flush = () => new Promise((resolve) => setImmediate(resolve));
(async () => {
  context.window = context;
  vm.runInNewContext(fs.readFileSync('slam_gui/battery.js', 'utf8'), context);
  await flush();
  // Все банки — таблица точных значений и плитки.
  assert.equal(el('bms-cells').children.length, 32);
  const last = el('bms-cells').children[31].children;
  assert.equal(last[0].textContent, '32');
  assert.equal(last[1].textContent, '3.231 В · 3231 мВ');
  assert.equal(last[3].textContent, 'Активна');
  assert.equal(last[4].textContent, 'МАКС');
  assert.equal(el('bms-cell-tiles').children.length, 32);
  assert.match(el('bms-cell-tiles').children[31].className, /is-max/);
  assert.match(el('bms-cell-tiles').children[31].className, /is-balancing/);
  assert.match(el('bms-cell-tiles').children[0].className, /is-min/);
  assert.match(el('bms-spread').textContent, /банка 1\).*банка 32\).*разброс: 31 мВ — больше 30 мВ.*сумма: 102\.90 В/);
  // Главное: заряд, ток с направлением, энергия, время до разряда.
  assert.equal(el('bms-soc').textContent, '50');
  assert.equal(el('bms-soc-bar').style.width, '50%');
  assert.equal(el('bms-capacity').textContent, 'остаток 25.00 из 50.00 Ач');
  const summary = texts(el('bms-summary'));
  assert.equal(el('bms-summary').children.length, 6);
  assert.match(summary, /−12\.50 А/);
  assert.match(summary, /разряд/);
  assert.match(summary, /≈ 1280 \/ 2560 Вт·ч/);
  assert.match(summary, /До разряда 2 ч 00 мин/);
  // Ключи, защиты, счётчики, температуры, паспорт, пороги.
  assert.match(texts(el('bms-fets')), /Ключ разряда: выкл/);
  assert.match(texts(el('bms-fets')), /Балансировка: банки 32/);
  assert.equal(el('bms-faults').textContent, 'Активных защит нет');
  assert.match(texts(el('bms-counters')), /Перегрузка разряда: 3/);
  assert(!/Короткое замыкание/.test(texts(el('bms-counters'))), 'нулевые счётчики не показываются');
  assert.match(texts(el('bms-temps')), /Датчик 2: 47\.0 °C/);
  assert.equal(el('bms-temps').children[1].className, 'pill warn');
  const passport = texts(el('bms-passport'));
  assert.match(passport, /SP04S034/);
  assert.match(passport, /14\.03\.2023/);
  assert.match(passport, /0x21/);
  assert.match(passport, /50\.00 \/ 18\.60 Ач/);
  assert.match(passport, /\/dev\/ttyUSB0/);
  const thresholds = texts(el('bms-thresholds'));
  assert.match(thresholds, /Перенапряжение банки 3\.650 В · возврат 3\.400 В/);
  assert.match(thresholds, /Перегрузка по разряду 60\.00 А/);
  assert.equal(el('bms-report').attrs['aria-disabled'], 'false');
  assert.equal(el('swipe-live').textContent, '● BMS');
  // Потеря связи: данные остаются, но явно устаревшие, CSV недоступен.
  fail = true;
  await nextPoll();
  assert.equal(el('bms-cells').children.length, 32);
  assert(page.classes.has('bms-stale'));
  assert.equal(el('bms-report').attrs['aria-disabled'], 'true');
  assert.equal(el('bms-report').attrs.href, undefined);
  assert.match(el('bms-status').textContent, /Нет связи/);
  assert.equal(el('swipe-live').textContent, 'НЕТ ДАННЫХ BMS');
  fail = false;
  sample.cells[31].mv = null;
  sample.receivedCount = 31; sample.complete = false;
  status.complete = false; status.reason = 'Неполный отчёт';
  await nextPoll();
  assert.equal(el('bms-cells').children[31].children[4].textContent, 'НЕТ ДАННЫХ');
  assert.match(el('bms-cell-tiles').children[31].className, /is-missing/);
  assert.equal(el('swipe-live').textContent, 'НЕ ВСЕ БАНКИ');
  page.hidden = true;
  el('swipe-live').textContent = '● LIVE';
  observer();
  assert.equal(el('swipe-live').textContent, '● LIVE'); // do not overwrite camera status
  // BMS не настроена: каркас страницы с прочерками и подсказка, как подключить.
  status = { connected: false, configured: false, complete: false, reason: 'BMS не настроена', data: null, setupAvailable: true };
  await nextPoll();
  assert.equal(el('bms-cells').children.length, 1);
  assert.equal(el('bms-cells').children[0].children[0].textContent, 'Нет данных от BMS');
  assert.equal(el('bms-summary').children.length, 6);
  assert.match(texts(el('bms-summary')), /— В/);
  assert.match(el('bms-updated').textContent, /Подключение BMS/);
  assert(el('bms-setup-open').classes.has('attention'));
  assert.equal(el('bms-setup').hidden, true, 'на скрытой странице блок не раскрывается');
  // Страницу открыли — блок подключения раскрывается сам и показывает порты.
  page.hidden = false;
  observer();
  await flush(); await flush();
  assert.equal(el('bms-setup').hidden, false);
  const ports = el('bms-port-list').children;
  assert.equal(ports.length, 2);
  const masterRadio = find(ports[0], (node) => node.tag === 'input');
  const freeRadio = find(ports[1], (node) => node.tag === 'input');
  assert.equal(masterRadio.disabled, true, 'порт мастера выбрать нельзя');
  assert.equal(freeRadio.checked, true, 'свободный порт выбран по умолчанию');
  el('bms-probe').fire('click');
  await flush(); await flush();
  assert.deepEqual(posts[0], ['api/battery/probe', { port: '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0' }]);
  assert.match(el('bms-setup-msg').textContent, /Найдена BMS: JBD BMS: 41\.90 В/);
  el('bms-save').fire('click');
  await flush(); await flush();
  assert.deepEqual(posts[1], ['api/battery/port', { port: '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0' }]);
  assert.match(el('bms-setup-msg').textContent, /Сохранено в \.env/);
  console.log('Battery page: full JBD info, all banks, stale state, CSV and on-screen BMS setup OK');
})().catch((error) => { console.error(error); process.exitCode = 1; });
