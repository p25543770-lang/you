/* ============================================================================
 * js_nav.js — переходы между экранами сохраняют сессию.
 *
 * Когда браузер не сохраняет cookie (встроенный предпросмотр в iframe),
 * сессия едет в адресе как параметр st. Ссылки и location.href его не несут,
 * поэтому переходы «киоск → пульт» и «пульт → киоск» должны добавлять токен
 * сами (RS_AUTH.withToken из slam_auth.js) — иначе пароль спросят повторно.
 *
 * Запуск:  node tests/js_nav.js
 * Код возврата: 0 — проверки прошли, 1 — есть расхождения.
 * ========================================================================== */
'use strict';

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const AUTH_JS = path.join(__dirname, '..', 'slam_gui', 'slam_auth.js');
const failures = [];

function browser(search) {
  const sandbox = {
    console,
    setTimeout: () => 0, clearTimeout: () => {},
    URL, URLSearchParams, Promise, Math, Date, JSON,
    location: { href: 'https://host/', search, origin: 'https://host' },
    navigator: { userAgent: 'test' },
    document: {
      readyState: 'complete', title: 'test', hidden: false,
      body: { classList: { add() {}, remove() {}, toggle() {}, contains: () => false } },
      documentElement: {},
      getElementById: () => null, querySelector: () => null, querySelectorAll: () => [],
      createElement: () => ({
        style: {}, classList: { add() {}, remove() {} },
        appendChild: () => {}, addEventListener: () => {},
      }),
      addEventListener: () => {}, removeEventListener: () => {},
    },
    fetch: () => Promise.reject(new Error('сети нет (заглушка)')),
    XMLHttpRequest: function () {},
    localStorage: { getItem: () => null, setItem: () => {} },
    getComputedStyle: () => ({ getPropertyValue: () => '' }),
    matchMedia: () => ({ matches: false, addEventListener: () => {} }),
    performance: { now: () => Date.now() },
    requestAnimationFrame: () => 0, setInterval: () => 0, clearInterval: () => {},
    addEventListener: () => {}, removeEventListener: () => {},
    innerWidth: 1280, innerHeight: 800, devicePixelRatio: 1,
    btoa: (s) => Buffer.from(String(s), 'binary').toString('base64'),
    atob: (s) => Buffer.from(String(s), 'base64').toString('binary'),
    TextEncoder, TextDecoder,
    isNaN, parseInt, parseFloat, String, Number, Boolean, Object, Array, RegExp, Error, TypeError,
  };
  sandbox.window = sandbox;
  sandbox.self = sandbox;
  sandbox.globalThis = sandbox;
  sandbox.XMLHttpRequest.prototype = { open() {} };
  vm.runInNewContext(fs.readFileSync(AUTH_JS, 'utf8'), sandbox, { filename: 'slam_auth.js' });
  return sandbox;
}

function check(label, condition, detail) {
  console.log((condition ? 'OK   ' : 'FAIL ') + label + (detail ? ' → ' + detail : ''));
  if (!condition) failures.push(label + (detail ? ': ' + detail : ''));
}

// 1. Токен в адресе — переход его сохраняет
const withToken = browser('?st=SEED123');
const carried = withToken.RS_AUTH && withToken.RS_AUTH.withToken('index.html');
check('переход несёт токен сессии', !!(carried && carried.includes('st=SEED123')), String(carried));

// 2. Токена нет — адрес не меняется (обычный браузер с cookie)
const withoutToken = browser('');
check('без токена переход не меняется', !withoutToken.RS_AUTH);

// 3. Внешние адреса токен не получают (сессия не утекает на чужой хост)
const external = withToken.RS_AUTH.withToken('https://example.com/panel');
check('чужой хост токен не получает', !external.includes('st='), external);

// 4. Переходы в коде страниц пользуются этим механизмом
const mainJs = fs.readFileSync(path.join(__dirname, '..', 'slam_gui', 'main.js'), 'utf8');
const appJs = fs.readFileSync(path.join(__dirname, '..', 'slam_gui', 'app.js'), 'utf8');
check('киоск → пульт сохраняет сессию', /RS_AUTH[\s\S]{0,80}go\(/.test(mainJs));
check('пульт → киоск сохраняет сессию', /RS_AUTH[\s\S]{0,80}go\(/.test(appJs));

if (failures.length) {
  console.log('расхождения:\n  ' + failures.join('\n  '));
  process.exit(1);
}
console.log('переходы между экранами сохраняют сессию');
