"""Безопасное самообновление Git-клона пульта оператора.

Обновление fast-forward-only: грязное дерево и расходящиеся ветки не затираются.
Пользовательские данные (.env, data/, var/) в репозитории игнорируются и не трогаются.
"""

from __future__ import annotations

import json
import os
import re
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
from typing import Callable
from urllib.parse import urlparse

try:
    import fcntl
except ImportError:  # pragma: no cover - production target is Linux/Ubuntu
    fcntl = None


class UpdateError(RuntimeError):
    """Update is not safe or could not be completed."""


class UpdateManager:
    def __init__(
        self,
        root: Path | str,
        *,
        remote: str = "origin",
        branch: str | None = None,
        on_restart: Callable[[], None] | None = None,
        dependency_installer: Callable[[Path], tuple[int, str]] | None = None,
        build_runner: Callable[[Path], tuple[int, str]] | None = None,
        ros_builder: Callable[[Path], tuple[int, str]] | None = None,
    ) -> None:
        self.root = Path(root).resolve()
        self.remote = remote.strip() or "origin"
        self.branch_override = (branch or "").strip()
        self.on_restart = on_restart
        self.dependency_installer = dependency_installer
        self.build_runner = build_runner
        self.ros_builder = ros_builder
        self.state_path = self.root / "var" / "update-state.json"
        self.lock_path = self.root / "var" / "update.lock"
        self._state_lock = threading.RLock()
        self._thread_lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._fallback_lock = threading.Lock()

    @staticmethod
    def _short(sha: str) -> str:
        return sha[:7] if sha else ""

    def _command(self, args: list[str], *, timeout: float = 20) -> subprocess.CompletedProcess[str]:
        env = os.environ.copy()
        env["GIT_TERMINAL_PROMPT"] = "0"
        return subprocess.run(
            ["git", "-C", str(self.root), *args],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
            env=env,
        )

    def _require_clone(self) -> None:
        result = self._command(["rev-parse", "--is-inside-work-tree"])
        if result.returncode != 0 or result.stdout.strip() != "true":
            raise UpdateError("Автообновление доступно только в Git-клоне проекта.")

    def _branch(self) -> str:
        branch = self.branch_override
        if not branch:
            result = self._command(["branch", "--show-current"])
            branch = result.stdout.strip() if result.returncode == 0 else ""
        if not branch or branch == "HEAD":
            raise UpdateError("Не удалось определить ветку. Задайте RC_UPDATE_BRANCH в .env.")
        if not re.fullmatch(r"[A-Za-z0-9._/-]{1,120}", branch) or branch.startswith("-") or ".." in branch:
            raise UpdateError("Некорректное имя ветки автообновления.")
        return branch

    def _remote_url(self) -> str:
        if not re.fullmatch(r"[A-Za-z0-9._-]{1,64}", self.remote):
            raise UpdateError("Некорректное имя Git remote в RC_UPDATE_REMOTE.")
        result = self._command(["remote", "get-url", self.remote])
        if result.returncode != 0:
            raise UpdateError(f"Git remote «{self.remote}» не настроен.")
        return result.stdout.strip()

    @staticmethod
    def _repo_label(remote_url: str) -> str:
        # Отображаем только публичную часть GitHub URL; не раскрываем логин,
        # токены и другие credentials, если они есть в URL remote.
        match = re.search(r"github\.com[:/]([^/\s:]+/[^/\s]+?)(?:\.git)?$", remote_url)
        if match:
            return match.group(1)
        parsed = urlparse(remote_url)
        return Path(parsed.path or remote_url).name.removesuffix(".git") or "Git remote"

    def _latest_sha(self, branch: str) -> str:
        result = self._command(["ls-remote", "--heads", self.remote, f"refs/heads/{branch}"], timeout=35)
        if result.returncode != 0:
            message = (result.stderr or "GitHub не ответил").strip().splitlines()
            raise UpdateError("Не удалось проверить обновления: " + (message[-1][:240] if message else "Git remote недоступен."))
        for line in result.stdout.splitlines():
            fields = line.split()
            if len(fields) == 2 and fields[1] == f"refs/heads/{branch}" and re.fullmatch(r"[0-9a-f]{40,64}", fields[0]):
                return fields[0]
        raise UpdateError(f"Ветка «{branch}» отсутствует в remote «{self.remote}».")

    def local_revision(self) -> str:
        self._require_clone()
        result = self._command(["rev-parse", "HEAD"])
        if result.returncode != 0:
            raise UpdateError("Не удалось определить установленную версию.")
        return result.stdout.strip()

    def check(self) -> dict:
        try:
            self._require_clone()
            branch = self._branch()
            remote_url = self._remote_url()
            local = self.local_revision()
            latest = self._latest_sha(branch)
            status = self._command(["status", "--porcelain", "--untracked-files=all"])
            if status.returncode != 0:
                raise UpdateError("Не удалось проверить рабочую копию Git.")
            dirty = bool(status.stdout.strip())
            state = self.get_status()
            return {
                "ok": True,
                "repo": self._repo_label(remote_url),
                "branch": branch,
                "remote": self.remote,
                "local": self._short(local),
                "localSha": local,
                "latest": self._short(latest),
                "latestSha": latest,
                "updateAvailable": local != latest,
                "dirty": dirty,
                "canUpdate": local != latest and not dirty and state.get("status") != "busy",
                "busy": state.get("status") == "busy",
                "tooling": {"git": True},
                "mode": "clone",
            }
        except (OSError, subprocess.TimeoutExpired, UpdateError) as exc:
            return {"ok": False, "error": str(exc)[:500]}

    def _default_state(self) -> dict:
        return {
            "status": "idle",
            "phase": "",
            "stage": "",
            "pct": 0,
            "from": "",
            "to": "",
            "fromSha": "",
            "toSha": "",
            "branch": "",
            "repo": "",
            "changes": 0,
            "restart": False,
            "output": [],
            "error": "",
            "started": None,
            "done": None,
            "rev": 0,
        }

    def _read_state_file(self) -> dict:
        try:
            state = json.loads(self.state_path.read_text(encoding="utf-8"))
            if isinstance(state, dict):
                base = self._default_state()
                base.update(state)
                if not isinstance(base.get("output"), list):
                    base["output"] = []
                return base
        except (OSError, ValueError, TypeError):
            pass
        return self._default_state()

    def _lock_is_held(self) -> bool:
        if fcntl is None:
            return self._thread_lock.locked()
        try:
            self.lock_path.parent.mkdir(parents=True, exist_ok=True)
            with self.lock_path.open("a+") as lock_file:
                try:
                    fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    return True
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
                return False
        except OSError:
            return False

    def _write_state(self, patch: dict, *, log: str | None = None) -> dict:
        with self._state_lock:
            state = self._read_state_file()
            state.update(patch)
            state["rev"] = int(state.get("rev", 0)) + 1
            state["updated"] = time.time()
            if log:
                lines = list(state.get("output") or [])
                lines.append(str(log)[:300])
                state["output"] = lines[-80:]
            self.state_path.parent.mkdir(parents=True, exist_ok=True)
            fd, tmp_name = tempfile.mkstemp(prefix="update-state-", suffix=".tmp", dir=self.state_path.parent)
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as handle:
                    json.dump(state, handle, ensure_ascii=False)
                    handle.flush()
                    os.fsync(handle.fileno())
                os.replace(tmp_name, self.state_path)
            finally:
                try:
                    os.unlink(tmp_name)
                except FileNotFoundError:
                    pass
            return state

    def get_status(self) -> dict:
        state = self._read_state_file()
        if state.get("status") == "busy" and not self._lock_is_held():
            state = self._write_state({
                "status": "error",
                "phase": "Обновление прервалось до завершения.",
                "error": "Процесс обновления остановился. Проверьте соединение и попробуйте ещё раз.",
                "restart": False,
                "done": time.time(),
            }, log="Обнаружено прерванное обновление.")
        return state

    def wait_status(self, since: int = -1, timeout: float = 20) -> dict:
        until = time.monotonic() + max(0.0, min(float(timeout), 25.0))
        while True:
            state = self.get_status()
            if int(state.get("rev", 0)) > since or state.get("status") in {"done", "error", "cancelled"}:
                return state
            if time.monotonic() >= until:
                return state
            time.sleep(0.2)

    def start(self) -> bool:
        if not self._thread_lock.acquire(blocking=False):
            return False
        lock_file = None
        try:
            self.lock_path.parent.mkdir(parents=True, exist_ok=True)
            lock_file = self.lock_path.open("a+")
            if fcntl is not None:
                try:
                    fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    lock_file.close()
                    self._thread_lock.release()
                    return False
            else:  # pragma: no cover - Windows fallback
                if not self._fallback_lock.acquire(blocking=False):
                    lock_file.close()
                    self._thread_lock.release()
                    return False
            thread = threading.Thread(target=self._run_update, args=(lock_file,), daemon=True, name="robot-control-update")
            self._thread = thread
            thread.start()
            return True
        except Exception:
            if lock_file is not None and not lock_file.closed:
                lock_file.close()
            self._thread_lock.release()
            raise

    def _run_process(self, args: list[str], *, timeout: float = 20) -> subprocess.CompletedProcess[str]:
        return subprocess.run(args, cwd=self.root, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, timeout=timeout, check=False)

    def _set(self, *, phase: str, stage: str, pct: int, **patch) -> dict:
        return self._write_state({"phase": phase, "stage": stage, "pct": max(0, min(100, int(pct))), **patch})

    def _install_dependencies(self) -> tuple[int, str]:
        if self.dependency_installer:
            return self.dependency_installer(self.root)
        script = self.root / "scripts" / "ensure_deps.sh"
        if not script.is_file():
            return 0, "Скрипт установки зависимостей не найден — шаг пропущен."
        result = subprocess.run(["bash", str(script)], cwd=self.root, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, timeout=900, check=False)
        return result.returncode, result.stdout[-6000:]

    def _build_check(self) -> tuple[int, str]:
        """«Сборка» Python-проекта: компиляция всех модулей интерпретатором .venv.

        Отдельного компилятора у проекта нет, поэтому сборка — это
        ``compileall`` по коду приложений и проверка, что главный пакет
        импортируется. Падение здесь означает, что новая версия не запустится:
        обновление откатывается, а работающая старая продолжает обслуживать.
        """
        if self.build_runner:
            return self.build_runner(self.root)
        venv_python = self.root / ".venv" / "bin" / "python"
        python = str(venv_python) if venv_python.is_file() else sys.executable
        targets = [name for name in ("robot_control", "slam_gui/backend.py",
                                     "run.py", "run_kiosk.py")
                   if (self.root / name).exists()]
        if not targets:
            return 0, "Проверять нечего — исходников не найдено."
        result = subprocess.run(
            [python, "-m", "compileall", "-q", *targets],
            cwd=self.root, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, timeout=300, check=False)
        output = result.stdout[-6000:]
        if result.returncode != 0:
            return result.returncode, output
        check = subprocess.run(
            [python, "-c", "import robot_control.app, robot_control.kiosk_app; print('import ok')"],
            cwd=self.root, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, timeout=120, check=False)
        return check.returncode, (output + check.stdout[-2000:]).strip()

    def _build_ros_workspace(self) -> tuple[int, str]:
        """ROS 2: докомпилировать ros2_ws новой версии (лидар через /scan).

        Без sudo и без установки пакетов — из службы/панели пароль спросить
        негде: только ``colcon build`` изменившихся исходников (и докачка
        драйверов из исходников). Сам ROS 2 ставит ярлык «Запустить робот».
        Ошибка здесь обновление не откатывает: приложения работают и без ROS 2
        (лидар по USB), а причина остаётся в журнале обновления.
        """
        if self.ros_builder:
            return self.ros_builder(self.root)
        script = self.root / "scripts" / "ros2_setup.sh"
        if not script.is_file():
            return 0, "Скрипт ROS 2 не найден — шаг пропущен."
        if (os.environ.get("RC_ROS_AUTO") or "1").strip() == "0" or \
                (os.environ.get("RC_LIDAR_SOURCE") or "").strip().lower() == "serial":
            return 0, "ROS 2 для лидара выключен (RC_ROS_AUTO=0 или RC_LIDAR_SOURCE=serial) — шаг пропущен."
        try:
            from . import ros_env  # noqa: PLC0415 — нужен только здесь
            if ros_env.prefix() is None:
                return 0, ("ROS 2 ещё не установлен — его поставит и скомпилирует ярлык "
                           "«Запустить робот» (или ./scripts/ros2_setup.sh).")
        except Exception:  # noqa: BLE001 — проверка не должна ломать обновление
            pass
        try:
            result = subprocess.run(
                ["bash", str(script), "--no-install", "--no-env", "--no-wait", "--quiet"],
                cwd=self.root, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, timeout=1800, check=False)
        except subprocess.TimeoutExpired:
            return 124, "сборка ros2_ws не уложилась в 30 минут — продолжится при следующем запуске робота"
        return result.returncode, result.stdout[-4000:]

    def _run_update(self, lock_file) -> None:
        old_sha = ""
        merged = False
        branch = ""
        repo = ""
        try:
            self._require_clone()
            branch = self._branch()
            remote_url = self._remote_url()
            repo = self._repo_label(remote_url)
            old_sha = self.local_revision()
            dirty = self._command(["status", "--porcelain", "--untracked-files=all"])
            if dirty.returncode != 0:
                raise UpdateError("Не удалось проверить рабочую копию.")
            if dirty.stdout.strip():
                raise UpdateError("Обновление остановлено: в рабочей копии есть изменения. Сначала сохраните их.")

            latest = self._latest_sha(branch)
            self._write_state({
                **self._default_state(),
                "status": "busy", "phase": "Проверяю версию…", "stage": "check", "pct": 8,
                "from": self._short(old_sha), "fromSha": old_sha, "to": self._short(latest),
                "toSha": latest, "branch": branch, "repo": repo,
                "started": time.time(), "restart": False,
            }, log=f"Источник: {repo} · ветка {branch}.")
            if latest == old_sha:
                self._write_state({
                    "status": "done", "phase": "Уже установлена последняя версия.",
                    "stage": "done", "pct": 100, "to": self._short(old_sha), "toSha": old_sha,
                    "changes": 0, "restart": False, "done": time.time(), "error": "",
                }, log="Изменений нет.")
                return

            self._set(phase="Скачиваю обновление…", stage="download", pct=18)
            git_env = os.environ.copy()
            git_env["GIT_TERMINAL_PROMPT"] = "0"
            fetch = subprocess.Popen(
                ["git", "-C", str(self.root), "fetch", "--progress", "--no-tags", "--no-recurse-submodules", self.remote, branch],
                cwd=self.root, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL,
                env=git_env,
            )
            started = time.monotonic()
            while fetch.poll() is None:
                elapsed = time.monotonic() - started
                self._set(phase="Скачиваю обновление…", stage="download", pct=min(48, 18 + int(elapsed / 2)))
                if elapsed > 600:
                    fetch.kill()
                    fetch.wait(timeout=5)
                    raise UpdateError("Скачивание обновления превысило 10 минут.")
                time.sleep(0.5)
            if fetch.returncode != 0:
                raise UpdateError("Не удалось скачать коммиты с Git remote.")

            fetched = self._command(["rev-parse", "FETCH_HEAD"])
            if fetched.returncode != 0:
                raise UpdateError("Git не вернул скачанную версию.")
            new_sha = fetched.stdout.strip()
            count_result = self._command(["rev-list", "--count", f"{old_sha}..{new_sha}"])
            if count_result.returncode != 0:
                raise UpdateError("Не удалось сравнить версии.")
            changes = int(count_result.stdout.strip() or "0")
            if changes <= 0:
                self._write_state({
                    "status": "done", "phase": "Уже установлена последняя версия.",
                    "stage": "done", "pct": 100, "to": self._short(old_sha), "toSha": old_sha,
                    "changes": 0, "restart": False, "done": time.time(), "error": "",
                }, log="Изменений нет.")
                return

            ancestor = self._command(["merge-base", "--is-ancestor", old_sha, new_sha])
            if ancestor.returncode != 0:
                raise UpdateError("Обновление не применено: ветки разошлись; автоматически перезаписывать файлы небезопасно.")
            self._set(phase=f"Применяю {changes} коммит(ов)…", stage="install", pct=55)
            merge = self._command(["merge", "--ff-only", "--no-edit", new_sha], timeout=60)
            if merge.returncode != 0:
                raise UpdateError("Git не смог выполнить безопасное fast-forward-обновление.")
            merged = True

            # Полный цикл, как при ручной переустановке: зависимости ставим
            # КАЖДЫЙ раз (ensure_deps.sh сам пропускает уже установленное и
            # работает офлайн из vendor/), а не только когда поменялся
            # requirements.txt, — иначе «обновил, а на роботе старый flask»
            # оставалось возможным при ручных правках окружения.
            self._set(phase="Переустанавливаю зависимости (ensure_deps.sh)…",
                      stage="dependencies", pct=68)
            code, output = self._install_dependencies()
            if output.strip():
                for line in output.splitlines()[-15:]:
                    self._write_state({}, log=line)
            if code != 0:
                rollback = self._command(["reset", "--hard", old_sha], timeout=60)
                merged = rollback.returncode != 0
                detail = output.strip().splitlines()[-1] if output.strip() else "pip завершился с ошибкой"
                if rollback.returncode != 0:
                    raise UpdateError("Не удалось установить зависимости и откатить исходный код; требуется ручная проверка. " + detail[:180])
                raise UpdateError("Не удалось установить зависимости; исходный код откатан. " + detail[:200])

            # Сборка-проверка новой версии до перезапуска: битый код не должен
            # заменить работающий — при ошибке компиляции откатываемся.
            self._set(phase="Собираю и проверяю новую версию…", stage="build", pct=84)
            code, output = self._build_check()
            if output.strip():
                for line in output.splitlines()[-15:]:
                    self._write_state({}, log=line)
            if code != 0:
                rollback = self._command(["reset", "--hard", old_sha], timeout=60)
                merged = rollback.returncode != 0
                detail = output.strip().splitlines()[-1] if output.strip() else "compileall завершился с ошибкой"
                if rollback.returncode != 0:
                    raise UpdateError("Новая версия не собралась и откатить код не удалось; требуется ручная проверка. " + detail[:180])
                raise UpdateError("Новая версия не собралась (проверка кода); исходный код откатан. " + detail[:200])
            # ROS 2: пакет zapret_lidar и драйверы из исходников компилируются
            # сами, если их исходники изменились (секунды, если менять нечего).
            self._set(phase="Компилирую ROS 2 (ros2_ws)…", stage="build", pct=88)
            ros_code, ros_output = self._build_ros_workspace()
            for line in ros_output.strip().splitlines()[-10:]:
                self._write_state({}, log=line)
            if ros_code not in (0, 75):
                self._write_state({}, log="ROS 2: ros2_ws не собран (код %s) — приложение работает, "
                                          "лидар по USB; ярлык «Запустить робот» докачает и "
                                          "соберёт недостающее сам." % ros_code)
            self._set(phase="Сборка прошла — перезапускаю пульт и экран…",
                      stage="build", pct=92)

            current = self.local_revision()
            self._write_state({
                "status": "done", "phase": "Обновление установлено. Перезапускаю приложение и экран…",
                "stage": "done", "pct": 100, "to": self._short(current), "toSha": current,
                "changes": changes, "restart": True, "done": time.time(), "error": "",
            }, log=f"Установлено коммитов: {changes}.")
            if self.on_restart:
                try:
                    self.on_restart()
                except Exception as exc:  # restart should not roll back an installed update
                    self._write_state({"phase": "Обновление установлено, но перезапуск не запустился.",
                                       "restart": False, "error": str(exc)[:300]},
                                      log="Перезапустите службы вручную.")
        except (OSError, subprocess.TimeoutExpired, ValueError, UpdateError) as exc:
            if merged and old_sha:
                self._command(["reset", "--hard", old_sha], timeout=60)
            self._write_state({
                "status": "error", "phase": "Обновление не выполнено.",
                "stage": "error", "pct": 0, "error": str(exc)[:500],
                "restart": False, "done": time.time(),
            }, log=str(exc))
        finally:
            try:
                if fcntl is not None:
                    fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
                else:  # pragma: no cover - Windows fallback
                    self._fallback_lock.release()
            except OSError:
                pass
            try:
                lock_file.close()
            except OSError:
                pass
            try:
                self._thread_lock.release()
            except RuntimeError:
                pass


def _read_cmdline(pid: int) -> list[str]:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
        return [item.decode("utf-8", "replace") for item in raw.split(b"\0") if item]
    except OSError:
        return []


def _process_running(pid: int) -> bool:
    try:
        stat = Path(f"/proc/{pid}/stat").read_text(encoding="ascii")
        return stat.split()[2] != "Z"
    except (OSError, IndexError):
        return False


def _restart_kiosk(root: Path) -> None:
    """Перезапустить только киоск-процессы, запущенные из этого проекта."""
    proc = Path("/proc")
    if not proc.is_dir():
        return
    target = (root / "run_kiosk.py").resolve()
    matches: list[tuple[int, list[str], Path, bool]] = []
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid in {os.getpid(), os.getppid()}:
            continue
        argv = _read_cmdline(pid)
        if not argv:
            continue
        script = next((arg for arg in argv[1:] if Path(arg).name == "run_kiosk.py"), None)
        if not script:
            continue
        try:
            cwd = Path(os.readlink(entry / "cwd")).resolve()
            script_path = Path(script)
            if not script_path.is_absolute():
                script_path = cwd / script_path
            if script_path.resolve() != target:
                continue
            cgroup = (entry / "cgroup").read_text(encoding="utf-8", errors="replace")
            managed = "robot-kiosk.service" in cgroup
            matches.append((pid, argv, cwd, managed))
        except (OSError, RuntimeError):
            continue

    for pid, argv, cwd, managed in matches:
        try:
            os.kill(pid, signal.SIGHUP)
        except OSError:
            continue
        if managed:
            # robot-kiosk.service has Restart=always; systemd restarts old builds
            # which don't yet handle SIGHUP themselves.
            continue
        time.sleep(1.0)
        if _process_running(pid):
            # Newer run_kiosk.py catches SIGHUP and re-execs itself in-place.
            continue
        # First update from an older manual launch: it has no SIGHUP handler.
        # Re-use its original executable/arguments so --no-open and port flags remain.
        try:
            subprocess.Popen(argv, cwd=cwd, stdin=subprocess.DEVNULL,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                             start_new_session=True, close_fds=True)
        except OSError:
            continue


def _restart_lidar(root: Path) -> None:
    """Перезапуск супервизора лидара ROS 2 с новым кодом (если его поднял ярлык).

    Цикл ``ros2_lidar.sh --keep-alive`` по SIGUSR1 останавливает супервизор
    (тот гасит драйвер) и сразу запускает заново — уже из обновлённого ros2_ws.
    """
    pid_path = root / "var" / "lidar.pid"
    try:
        pid = int(pid_path.read_text(encoding="utf-8").strip())
    except (OSError, ValueError):
        return
    if pid <= 0 or not _process_running(pid):
        return
    if not any("ros2_lidar.sh" in part for part in _read_cmdline(pid)):
        return
    try:
        os.kill(pid, signal.SIGUSR1)
    except OSError:
        pass


def schedule_restart(root: Path | str) -> None:
    """Gracefully reload Gunicorn and re-exec the separate kiosk process."""
    if os.environ.get("PYTEST_CURRENT_TEST"):
        return
    project_root = Path(root).resolve()

    def restart() -> None:
        time.sleep(1.4)  # let POST /run and the terminal update state reach the browser
        _restart_kiosk(project_root)
        _restart_lidar(project_root)
        parent = os.getppid()
        parent_cmd = " ".join(_read_cmdline(parent)).lower()
        try:
            if "gunicorn" in parent_cmd:
                os.kill(parent, signal.SIGHUP)  # Gunicorn replaces workers with the new code
                return
            # Development/manual Flask server: replace this process with run.py.
            entry = project_root / "run.py"
            if entry.is_file():
                args = [sys.executable, str(entry)]
                if "--dev" in sys.argv:
                    args.append("--dev")
                os.execv(sys.executable, args)
        except OSError:
            return

    threading.Thread(target=restart, daemon=True, name="robot-control-restart").start()
