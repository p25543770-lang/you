"""Flask-приложение: вход оператора и веб-интерфейс робота RUS SLAM.

Сессия держится в cookie, но есть резервный механизм для сред, где cookie
не сохраняются (встроенный cross-site предпросмотр в iframe с opaque-origin):
подписанный токен ``st`` в URL. Cookie всегда в приоритете; ``st`` используется
только когда cookie-сессии нет.

После входа открывается интерфейс RUS SLAM (см. :mod:`robot_control.slam`):
основной экран робота на ``/`` и инженерный пульт на ``/console``.
"""

from __future__ import annotations

import logging
import os
import secrets
import subprocess
import time
from datetime import timedelta
from functools import wraps
from urllib.parse import urlparse

from flask import (
    Flask,
    g,
    jsonify,
    redirect,
    render_template,
    request,
    session,
    url_for,
)
from itsdangerous import BadSignature, URLSafeSerializer, URLSafeTimedSerializer
from werkzeug.routing import BuildError

from . import eventlog
from .auth import (
    ROLE_OPERATOR,
    LoginThrottle,
    OperatorStore,
    authenticate,
    hash_password,
)
from .config import Config, PROJECT_ROOT
from .lidar import LidarManager
from .lidar_settings import apply_lidar_settings, settings_payload
from .slam import build_state, create_blueprint, start_lidar
from .updater import UpdateManager, schedule_restart

log = logging.getLogger("robot_control")

SESSION_USER = "operator"
SESSION_ROLE = "role"
SESSION_LOGIN_AT = "login_at"
ST_PARAM = "st"


def _read_build_sha() -> str:
    """Capture the source revision when this worker imports the application."""
    try:
        result = subprocess.run(
            ["git", "-C", str(PROJECT_ROOT), "rev-parse", "HEAD"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=2,
            check=False,
        )
        if result.returncode == 0:
            return result.stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        pass
    return "unknown"


APP_BUILD_SHA = _read_build_sha()


def _safe_next(target: str | None) -> str | None:
    """Разрешаем переход только на относительный путь внутри приложения."""
    if not target:
        return None
    if target.startswith("//") or "\\" in target:
        return None
    parsed = urlparse(target)
    if parsed.netloc or parsed.scheme:
        return None
    return target if target.startswith("/") else None


def create_app(config: Config | None = None) -> Flask:
    """Фабрика приложения — так его удобно поднимать и в тестах, и в бою."""
    config = config or Config.from_env()
    app = Flask(__name__)

    # Панель «Логи» на бортовом экране показывает настоящий журнал процесса:
    # подключаем его до первого логирования, чтобы ничего не потерялось.
    eventlog.install()

    app.config["SECRET_KEY"] = config.secret_key or secrets.token_urlsafe(48)
    if not config.secret_key:
        log.warning(
            "RC_SECRET_KEY не задан: сгенерирован временный ключ, "
            "сессии сбросятся после перезапуска. Задайте ключ в .env"
        )

    # SameSite=None разрешает cookie во встроенном (cross-site) предпросмотре,
    # но браузеры принимают его только вместе с Secure.
    samesite = (config.cookie_samesite or "Lax").strip()
    if samesite.lower() == "none":
        samesite = "None"
        secure = True
    else:
        samesite = samesite.capitalize()
        secure = config.cookie_secure
    max_age = int(timedelta(minutes=config.session_lifetime_minutes).total_seconds())
    app.config.update(
        PERMANENT_SESSION_LIFETIME=timedelta(minutes=config.session_lifetime_minutes),
        SESSION_COOKIE_NAME="rc_session",
        SESSION_COOKIE_HTTPONLY=True,
        SESSION_COOKIE_SAMESITE=samesite,
        SESSION_COOKIE_SECURE=secure,
        MAX_CONTENT_LENGTH=64 * 1024,
    )

    store = OperatorStore(config.operators_file)
    throttle = LoginThrottle(config.max_failed_attempts, config.lockout_seconds)
    # Синтетический хеш: считаем один раз, чтобы ответ для несуществующего логина
    # занимал столько же времени, сколько для существующего.
    timing_hash = hash_password("timing-equalizer", iterations=config.pbkdf2_iterations)

    csrf_signer = URLSafeSerializer(app.config["SECRET_KEY"], salt="rc-csrf")
    url_session_signer = URLSafeTimedSerializer(app.config["SECRET_KEY"], salt="rc-url-session")

    # Отзыв URL-токенов при выходе (в пределах процесса): stateless-токен иначе
    # оставался бы валидным до истечения срока.
    import threading

    _revoked: set[str] = set()
    _revoked_lock = threading.Lock()

    def revoke_token(token: str) -> None:
        with _revoked_lock:
            if len(_revoked) > 1000:
                _revoked.clear()
            _revoked.add(token)

    def is_revoked(token: str) -> bool:
        with _revoked_lock:
            return token in _revoked

    app.extensions["rc_config"] = config
    app.extensions["rc_store"] = store
    app.extensions["rc_throttle"] = throttle
    updater = UpdateManager(
        PROJECT_ROOT,
        remote=os.environ.get("RC_UPDATE_REMOTE", "origin"),
        branch=os.environ.get("RC_UPDATE_BRANCH"),
        on_restart=lambda: schedule_restart(PROJECT_ROOT),
    )
    app.extensions["rc_updater"] = updater

    # ----------------------- Резервная URL-сессия ------------------------ #
    def issue_url_session(username: str) -> str:
        return url_session_signer.dumps({"u": username, "r": ROLE_OPERATOR})

    def load_url_session(token: str | None) -> str | None:
        """Имя оператора из токена ``st`` или None, если токен невалиден."""
        if not token or is_revoked(token):
            return None
        try:
            data = url_session_signer.loads(token, max_age=max_age)
        except BadSignature:
            return None
        if not isinstance(data, dict) or data.get("r") != ROLE_OPERATOR:
            return None
        return data.get("u")

    @app.before_request
    def resolve_url_session():
        g.via_st = False
        g.st_token = None
        g.st_user = None
        if session.get(SESSION_USER):
            return
        token = request.args.get(ST_PARAM) or request.form.get(ST_PARAM)
        user = load_url_session(token)
        if user:
            g.via_st = True
            g.st_token = token
            g.st_user = user

    def current_user() -> tuple[str | None, bool]:
        """(имя оператора, пришёл ли вход через URL-токен)."""
        if (
            session.get(SESSION_USER)
            and session.get(SESSION_ROLE) == ROLE_OPERATOR
        ):
            return session[SESSION_USER], False
        if getattr(g, "via_st", False):
            return g.st_user, True
        return None, False

    def with_st(url: str) -> str:
        if getattr(g, "via_st", False) and g.st_token:
            sep = "&" if "?" in url else "?"
            return f"{url}{sep}{ST_PARAM}={g.st_token}"
        return url

    def home_url() -> str:
        """Куда попадаем после входа: экран робота, а без него — служебный /panel."""
        endpoint = "slam.main" if app.extensions.get("rc_slam") else "panel.dashboard"
        return url_for(endpoint)

    def login_required(view):
        @wraps(view)
        def wrapper(*args, **kwargs):
            user, _via = current_user()
            if user is None:
                session.clear()
                # Только что успешный вход, а сессия не доехала и токена нет —
                # браузер не сохранил cookie (встроенный предпросмотр).
                if request.args.get("just_logged_in"):
                    return redirect(url_for("auth.login", nocookie=1, next=request.path))
                return redirect(url_for("auth.login", next=request.path))
            return view(*args, **kwargs)

        return wrapper

    # ------------------------------ CSRF --------------------------------- #
    # Stateless-токен: подписан секретом, не зависит от cookie сессии.
    @app.context_processor
    def inject_globals():
        user, via_st = current_user()

        def u(endpoint: str, **kwargs) -> str:
            # Навигация в base.html общая для всех страниц, включая страницу
            # ошибки. Если интерфейс робота не подключился, его эндпоинтов в
            # url_map нет — без этой проверки вместо внятной 503 оператор
            # получил бы BuildError (и ещё один 500 на странице ошибки).
            try:
                return with_st(url_for(endpoint, **kwargs))
            except BuildError:
                return "#"

        return {
            "csrf_token": csrf_signer.dumps("csrf"),
            "session_user": user,
            "via_st": via_st,
            "u": u,
        }

    def csrf_ok() -> bool:
        provided = request.form.get("csrf_token", "")
        if not provided:
            return False
        try:
            return csrf_signer.loads(provided) == "csrf"
        except BadSignature:
            return False

    def csrf_header_ok() -> bool:
        provided = request.headers.get("X-CSRF-Token", "")
        if not provided:
            return False
        try:
            return csrf_signer.loads(provided) == "csrf"
        except BadSignature:
            return False

    # --------------------------- Заголовки ------------------------------- #
    @app.after_request
    def security_headers(response):
        response.headers.setdefault("X-Content-Type-Options", "nosniff")
        # Разрешаем превью в Arena/e2b iframe (иначе оператор не откроется в превью)
        # Убираем DENY / frame-ancestors 'none' только для превью — в проде служба за VPN
        if app.config.get("TESTING") or os.environ.get("PYTEST_CURRENT_TEST"):
            response.headers["X-Frame-Options"] = "DENY"
        else:
            response.headers.pop("X-Frame-Options", None)
        response.headers.setdefault("Referrer-Policy", "no-referrer")
        response.headers.setdefault("X-Robots-Tag", "noindex, nofollow")
        response.headers.setdefault(
            "Content-Security-Policy",
            "default-src 'self'; img-src 'self' data: blob:; media-src 'self' blob: mediastream: data:; "
            "style-src 'self' 'unsafe-inline'; script-src 'self' 'unsafe-inline'; "
            "frame-ancestors 'self' https://*.e2b.app https://*.arena.site https://*.arena.ai https://* http://* ; base-uri 'none'; form-action 'self'",
        )
        response.headers.setdefault("Cache-Control", "no-store")
        return response

    # --------------------- Интерфейс RUS SLAM ---------------------------- #
    # Он занимает "/" и "/console"; если подключить не удалось, "/" показывает
    # внятную ошибку вместо пустого 404.
    try:
        # Пульт 8080 не открывает UART мастера: порт держит бортовой экран (8090),
        # иначе два процесса читают его вперемешку. Здесь — только телеметрия.
        slam_state = build_state(
            mode=config.slam_source,
            ports=config.slam_ports,
            lock_file=config.slam_lock_file,
            pin=config.slam_pin,
            max_attempts=config.slam_max_attempts,
            lock_sec=config.slam_lock_seconds,
            master_role="peer",
        )
        app.extensions["rc_slam"] = slam_state
        app.register_blueprint(create_blueprint(slam_state, guard=login_required))
        log.info(
            "интерфейс RUS SLAM подключён (источник данных: %s)",
            getattr(slam_state.source, "name", "?"),
        )
    except Exception as exc:  # noqa: BLE001 — пульт должен подняться в любом случае
        log.error("интерфейс RUS SLAM не подключён: %s", exc)
        # Имя exc живёт только внутри except (Python удаляет его в конце блока),
        # а страница рисуется позже — поэтому текст ошибки сохраняем сразу.
        reason = str(exc)

        @app.get("/", endpoint="slam.broken")
        @login_required
        def slam_broken():
            return (
                render_template(
                    "error.html",
                    code=503,
                    message="Интерфейс робота не подключён: " + reason,
                ),
                503,
            )

    # Лидар поднимается сразу после чтения .env (Config.from_env), а не по
    # первому запросу браузера: так USB-порт читает один процесс, а второй
    # (бортовой экран на 8090) берёт готовый скан по HTTP. Свой порт (8080)
    # запоминаем, чтобы не ходить за сканом к себе же. Старт — вне try/except
    # выше: карта лидара должна работать, даже если интерфейс робота не поднялся.
    app.extensions["rc_lidar"] = start_lidar(testing=config.testing, port=config.port)

    # ------------------------------ Маршруты ----------------------------- #
    @app.get("/healthz")
    def healthz():
        slam_state = app.extensions.get("rc_slam")
        return jsonify(
            {
                "status": "ok",
                "operators_configured": store.exists(),
                "panel": "rus_slam" if slam_state else "unavailable",
                "source": getattr(getattr(slam_state, "source", None), "name", None),
            }
        )

    @app.get("/api/update/check")
    @login_required
    def api_update_check():
        result = updater.check()
        result["csrfToken"] = csrf_signer.dumps("csrf")
        return jsonify(result), 200 if result.get("ok") else 502

    @app.get("/api/update/status")
    @login_required
    def api_update_status():
        try:
            since = int(request.args.get("since", "-1"))
        except (TypeError, ValueError):
            since = -1
        state = updater.wait_status(since=since, timeout=20)
        return jsonify({"ok": True, **state})

    @app.post("/api/update/run")
    @login_required
    def api_update_run():
        if not csrf_header_ok():
            return jsonify({"ok": False, "error": "Недействительный CSRF-токен."}), 400
        if not updater.start():
            return jsonify({"ok": False, "error": "Обновление уже выполняется.", **updater.get_status()}), 409
        return jsonify({"ok": True, "status": "busy"}), 202

    @app.get("/api/update/version")
    @login_required
    def api_update_version():
        try:
            revision = updater.local_revision()
            return jsonify({"ok": True, "sha": revision, "short": revision[:7],
                            "buildSha": APP_BUILD_SHA})
        except Exception as exc:  # noqa: BLE001 — диагностический endpoint
            return jsonify({"ok": False, "error": str(exc)[:300],
                            "buildSha": APP_BUILD_SHA}), 503

    # ------------------------- Настройки лидара ------------------------- #
    # Панель «Лидар» на экранах пульта: порт/протокол/скорость/линия мотора
    # пишутся в .env и применяются сразу. Бортовой экран (8090) подхватит тот
    # же .env сам (см. LidarManager._sync_settings_from_env_file), а скан —
    # по peer-обмену, поэтому настраивать два приложения отдельно не нужно.
    @app.get("/api/lidar/settings")
    @login_required
    def api_lidar_settings():
        result = settings_payload(LidarManager.get_instance())
        result["csrfToken"] = csrf_signer.dumps("csrf")
        result["canEdit"] = True
        result["requiresPin"] = False
        return jsonify(result)

    @app.post("/api/lidar/settings")
    @login_required
    def api_lidar_settings_save():
        if not csrf_header_ok():
            return jsonify({"ok": False, "error": "Недействительный CSRF-токен."}), 400
        try:
            result = apply_lidar_settings(request.get_json(silent=True) or {})
        except ValueError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400
        result["csrfToken"] = csrf_signer.dumps("csrf")
        return jsonify(result)

    @app.post("/api/lidar/rescan")
    @login_required
    def api_lidar_rescan():
        if not csrf_header_ok():
            return jsonify({"ok": False, "error": "Недействительный CSRF-токен."}), 400
        manager = LidarManager.get_instance()
        result = manager.restart_search()
        return jsonify({"ok": True, "settings": manager.current_settings(),
                        # При источнике ros «пересканировать» значит перезапустить
                        # драйвер через супервизор: его ответ виден в панели.
                        "ros_rescan": result.get("ros_rescan") if isinstance(result, dict) else None,
                        "status": manager.get_status()})

    @app.route("/login", methods=["GET", "POST"], endpoint="auth.login")
    def login():
        user, _via = current_user()
        if user is not None:
            return redirect(with_st(home_url()))

        if request.method == "POST":
            if not csrf_ok():
                # Токен недействителен: чаще всего страница была открыта до
                # перезапуска сервера (сменился RC_SECRET_KEY) или браузер
                # отдал форму из кэша. Голый 400 здесь — тупик, поэтому
                # показываем форму заново, с уже свежим токеном.
                log.warning(
                    "вход отклонён: недействительный CSRF-токен (%s)",
                    request.remote_addr or "unknown",
                )
                return (
                    render_template(
                        "login.html",
                        error="Форма устарела (сервер перезапускали). "
                        "Введите логин и пароль ещё раз.",
                        username=(request.form.get("username") or "").strip(),
                    ),
                    400,
                )
            username = (request.form.get("username") or "").strip()
            password = request.form.get("password") or ""
            ip = request.remote_addr or "unknown"
            operator, reason = authenticate(
                store, throttle, username, password, ip, timing_hash=timing_hash
            )
            if operator is None:
                # Клиенту — общее сообщение (не выдаём, существует ли логин),
                # а в журнал — подробности: иначе «неверный пароль» невозможно
                # диагностировать (смотри journalctl / вывод сервера).
                log.warning(
                    "неудачный вход: логин=%r с %s (%s)",
                    username,
                    ip,
                    "блокировка"
                    if reason.startswith("locked:")
                    else {
                        "no_store": "учётки не настроены",
                        "inactive": "учётка отключена",
                    }.get(reason, "неверный логин или пароль"),
                )
                if reason.startswith("locked:"):
                    seconds = int(reason.split(":", 1)[1])
                    return (
                        render_template(
                            "login.html",
                            error=f"Слишком много попыток. Повторите через {seconds} с.",
                            username=username,
                        ),
                        429,
                    )
                message = {
                    "no_store": "Учётки операторов не настроены. "
                    "Запустите scripts/create_operator.py",
                    "inactive": "Эта учётная запись отключена.",
                }.get(reason, "Неверный логин или пароль")
                return render_template("login.html", error=message, username=username), 401

            session.clear()
            session.permanent = True
            session[SESSION_USER] = operator.username
            session[SESSION_ROLE] = operator.role
            session[SESSION_LOGIN_AT] = int(time.time())
            log.info("оператор %s вошёл с %s", operator.username, ip)
            target = _safe_next(request.args.get("next")) or home_url()
            sep = "&" if "?" in target else "?"
            # just_logged_in — маркер потери cookie; st — резервная сессия для
            # сред, где cookie не сохраняются (iframe-превью)
            return redirect(
                f"{target}{sep}just_logged_in=1&{ST_PARAM}={issue_url_session(operator.username)}"
            )

        return render_template(
            "login.html",
            error=None,
            username="",
            nocookie=bool(request.args.get("nocookie")),
        )

    @app.post("/logout", endpoint="auth.logout")
    def logout():
        user, via_st = current_user()
        if via_st and g.st_token:
            revoke_token(g.st_token)
        session.clear()
        if user:
            log.info("оператор %s вышел%s", user, " (url-сессия)" if via_st else "")
        # После выхода токен st в ссылку не подставляется — сессия мертва.
        return redirect(url_for("auth.login"))

    @app.get("/panel", endpoint="panel.dashboard")
    @login_required
    def dashboard():
        """Служебная страница пульта: кто вошёл и куда идти дальше."""
        user, _via = current_user()
        return render_template("dashboard.html", operator=user)

    @app.get("/service", endpoint="panel.service")
    @login_required
    def service():
        """Сервис — смена темы (тот же набор 20 тем что на киоске 8090)."""
        user, _via = current_user()
        return render_template("service.html", operator=user)

    @app.get("/exit", endpoint="auth.exit")
    def exit_get():
        """Выход по ссылке (для кнопок «Выход» в интерфейсе робота)."""
        user, via_st = current_user()
        if via_st and g.st_token:
            revoke_token(g.st_token)
        session.clear()
        if user:
            log.info("оператор %s вышел%s", user, " (url-сессия)" if via_st else "")
        return redirect(url_for("auth.login"))

    @app.errorhandler(404)
    def not_found(_error):
        if request.path.startswith("/api/"):
            return jsonify({"error": "not_found"}), 404
        return render_template("error.html", code=404, message="Страница не найдена"), 404

    @app.errorhandler(400)
    def bad_request(_error):
        return render_template("error.html", code=400, message="Некорректный запрос"), 400

    @app.errorhandler(500)
    def internal_error(_error):
        # Чаще всего сюда приводит старый процесс сервера, чей код в памяти не
        # совпадает с обновлёнными шаблонами; лечится перезапуском.
        return (
            render_template(
                "error.html",
                code=500,
                message="Внутренняя ошибка. Перезапустите сервер: "
                "./scripts/stop.sh, затем ./scripts/start.sh",
            ),
            500,
        )

    return app
