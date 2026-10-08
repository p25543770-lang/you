/* Добавлено пультом оператора (не входит в исходный rus_slam).
 *
 * В средах, где браузер не сохраняет cookie (встроенный предпросмотр в iframe
 * с opaque-origin), сессия передаётся подписанным токеном st в адресе страницы.
 * Подзапросы (fetch/XHR) сами его не несут, поэтому добавляем его здесь — из
 * адреса текущей страницы.
 *
 * В обычном браузере (робот, телефон, ноутбук) cookie работают, токена st в
 * адресе нет — и скрипт не меняет ничего.
 */
(function () {
  "use strict";

  var token = new URLSearchParams(window.location.search).get("st");
  if (!token) {
    return;
  }

  function withToken(url) {
    try {
      var target = new URL(url, window.location.href);
      if (target.origin === window.location.origin && !target.searchParams.has("st")) {
        target.searchParams.set("st", token);
      }
      return target.toString();
    } catch (err) {
      return url;
    }
  }

  var nativeFetch = window.fetch;
  if (typeof nativeFetch === "function") {
    window.fetch = function (input, init) {
      if (typeof input === "string") {
        return nativeFetch.call(this, withToken(input), init);
      }
      if (input && typeof input.url === "string") {
        return nativeFetch.call(this, new Request(withToken(input.url), input), init);
      }
      return nativeFetch.call(this, input, init);
    };
  }

  var nativeOpen = XMLHttpRequest.prototype.open;
  XMLHttpRequest.prototype.open = function (method, url) {
    var args = Array.prototype.slice.call(arguments);
    if (typeof url === "string") {
      args[1] = withToken(url);
    }
    return nativeOpen.apply(this, args);
  };
})();
