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
import secrets
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

from .auth import (
    ROLE_OPERATOR,
    LoginThrottle,
    OperatorStore,
    authenticate,
    hash_password,
)
from .config import Config
from .brain.web import create_brain_blueprint
from .slam import build_state, create_blueprint

log = logging.getLogger("robot_control")

SESSION_USER = "operator"
SESSION_ROLE = "role"
SESSION_LOGIN_AT = "login_at"
ST_PARAM = "st"


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
            return with_st(url_for(endpoint, **kwargs))

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

    # --------------------------- Заголовки ------------------------------- #
    @app.after_request
    def security_headers(response):
        response.headers.setdefault("X-Content-Type-Options", "nosniff")
        response.headers.setdefault("X-Frame-Options", "DENY")
        response.headers.setdefault("Referrer-Policy", "no-referrer")
        response.headers.setdefault("X-Robots-Tag", "noindex, nofollow")
        # 'unsafe-inline' нужен только для стилей: в интерфейсе робота есть
        # атрибуты style="…" (инженерный пульт). Скрипты — по-прежнему только
        # свои файлы, инлайн-JS остаётся запрещён.
        response.headers.setdefault(
            "Content-Security-Policy",
            "default-src 'self'; img-src 'self' data:; "
            "style-src 'self' 'unsafe-inline'; script-src 'self'; "
            "frame-ancestors 'none'; base-uri 'none'; form-action 'self'",
        )
        response.headers.setdefault("Cache-Control", "no-store")
        return response

    # --------------------- Интерфейс RUS SLAM ---------------------------- #
    # Он занимает "/" и "/console"; если подключить не удалось, "/" показывает
    # внятную ошибку вместо пустого 404.
    try:
        slam_state = build_state(
            mode=config.slam_source,
            ports=config.slam_ports,
            lock_file=config.slam_lock_file,
            pin=config.slam_pin,
            max_attempts=config.slam_max_attempts,
            lock_sec=config.slam_lock_seconds,
        )
        app.extensions["rc_slam"] = slam_state
        app.register_blueprint(create_blueprint(slam_state, guard=login_required))
        log.info(
            "интерфейс RUS SLAM подключён (источник данных: %s)",
            getattr(slam_state.source, "name", "?"),
        )
    except Exception as exc:  # noqa: BLE001 — пульт должен подняться в любом случае
        log.error("интерфейс RUS SLAM не подключён: %s", exc)

        @app.get("/", endpoint="slam.broken")
        @login_required
        def slam_broken():
            return (
                render_template(
                    "error.html",
                    code=503,
                    message="Интерфейс робота не подключён: " + str(exc),
                ),
                503,
            )

    # --------------- Симуляция ROS + ИИ (страница /brain) ----------------- #
    # Симуляция поднимается лениво — при первом открытии страницы.
    brain_holder: dict = {}
    app.extensions["rc_brain"] = brain_holder
    app.register_blueprint(create_brain_blueprint(brain_holder, guard=login_required))

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
