"""ROS 2 ставится и компилируется сам: scripts/ros2_setup.sh, ros2_lidar.sh, ros_env.

Раньше ROS 2 без флагов не ставился вовсе (``ros2_setup.sh`` только печатал
подсказку), драйверы — только с ``--drivers``, ldlidar — только с ``--ldlidar``,
а сборка шла ``--packages-select zapret_lidar`` и падала на ``source
setup.bash`` под ``set -u``. Здесь весь путь прогоняется на фальшивой системе:
``apt-get``, ``sudo``, ``dpkg``, ``curl``, ``git``, ``colcon``, ``rosdep`` и
``vcs`` подменены скриптами, которые записывают вызовы и изображают установку
(``/opt/ros/<дистрибутив>`` создаётся во временном каталоге). Настоящие apt и
sudo не вызываются никогда.
"""

from __future__ import annotations

import json
import os
import shutil
import signal
import subprocess
import sys
import textwrap
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
WS = ROOT / "ros2_ws"
PYVER = "%d.%d" % sys.version_info[:2]

pytestmark = pytest.mark.skipif(shutil.which("bash") is None or shutil.which("flock") is None,
                                reason="нужны bash и flock")

# Скрипт setup.bash в стиле colcon: читает $COLCON_TRACE, не проверяя, задан ли
# он, — ровно то, на чём падал source под set -u.
COLCON_LIKE_SETUP = textwrap.dedent('''\
    # generated from colcon_bash/shell/template/prefix_chain.bash.em (тест)
    if [ -n "$COLCON_TRACE" ]; then echo "# trace"; fi
    _p="$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)"
    export PYTHONPATH="$_p/lib/python%(pyver)s/site-packages${PYTHONPATH:+:$PYTHONPATH}"
    export AMENT_PREFIX_PATH="$_p${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$_p/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export ROS_DISTRO="$(basename "$_p")"
    unset _p
''') % {"pyver": PYVER}

FAKES: dict[str, str] = {
    "sudo": r'''#!/usr/bin/env bash
echo "sudo $*" >> "$FAKE_DIR/calls.log"
case "${1:-}" in
  -n|-v) [[ "${FAKE_SUDO:-ok}" == ok ]] && exit 0 || exit 1 ;;
esac
while [[ "${1:-}" == -* ]]; do
  case "$1" in -u) shift 2 ;; *) shift ;; esac
done
exec "$@"
''',
    "apt-get": r'''#!/usr/bin/env bash
echo "apt-get $*" >> "$FAKE_DIR/calls.log"
sub=""; pkgs=(); skip=0
for a in "$@"; do
  if [[ $skip -eq 1 ]]; then skip=0; continue; fi
  case "$a" in
    -o) skip=1 ;;
    -*) ;;
    *) if [[ -z "$sub" ]]; then sub="$a"; else pkgs+=("$a"); fi ;;
  esac
done
make_prefix() {
  local d="$1" p="$RC_ROS_OPT_DIR/$1"
  mkdir -p "$p/share/ament_index/resource_index/packages" "$p/lib/python$FAKE_PYVER/site-packages/rclpy" "$p/bin"
  cp "$FAKE_DIR/colcon_setup.bash" "$p/setup.bash"
  for pkg in rclpy sensor_msgs std_msgs std_srvs; do touch "$p/share/ament_index/resource_index/packages/$pkg"; done
  echo "ok = True" > "$p/lib/python$FAKE_PYVER/site-packages/rclpy/__init__.py"
}
index_add() {
  local p
  for p in "$RC_ROS_OPT_DIR"/*/; do touch "$p/share/ament_index/resource_index/packages/$1"; break; done
}
case "$sub" in
  update) exit "${FAKE_APT_UPDATE_RC:-0}" ;;
  install)
    for p in "${pkgs[@]}"; do
      for f in ${FAKE_APT_FAIL:-}; do [[ "$p" == "$f" ]] && exit 100; done
    done
    for p in "${pkgs[@]}"; do
      echo "$p" >> "$FAKE_DIR/installed.txt"
      case "$p" in
        ros-*-ros-base) d="${p#ros-}"; make_prefix "${d%-ros-base}" ;;
        ros-*-hls-lfcd-lds-driver) index_add hls_lfcd_lds_driver ;;
        ros-*-rplidar-ros) index_add rplidar_ros ;;
        ros-*-turtlebot3-node) index_add turtlebot3_node ;;
      esac
    done ;;
esac
exit 0
''',
    "apt-cache": r'''#!/usr/bin/env bash
pkg="${2:-}"
avail=1
case "$pkg" in
  ros-*) [[ -f "$FAKE_DIR/repo" || "${FAKE_REPO:-0}" == 1 ]] || avail=0 ;;
esac
for m in ${FAKE_APT_MISSING:-}; do [[ "$pkg" == "$m" ]] && avail=0; done
echo "$pkg:"
echo "  Installed: (none)"
if [[ $avail -eq 1 ]]; then echo "  Candidate: 1.0-1"; else echo "  Candidate: (none)"; fi
''',
    "dpkg-query": r'''#!/usr/bin/env bash
pkg="${@: -1}"
grep -qxF "$pkg" "$FAKE_DIR/installed.txt" 2>/dev/null && printf 'install ok installed' && exit 0
exit 1
''',
    "dpkg": r'''#!/usr/bin/env bash
echo "dpkg $*" >> "$FAKE_DIR/calls.log"
case "${1:-}" in
  --print-architecture) echo "${FAKE_ARCH:-amd64}" ;;
  -i) echo ros2-apt-source >> "$FAKE_DIR/installed.txt"; touch "$FAKE_DIR/repo" ;;
esac
exit 0
''',
    "curl": r'''#!/usr/bin/env bash
echo "curl $*" >> "$FAKE_DIR/calls.log"
out=""; url=""; prev=""
for a in "$@"; do
  [[ "$prev" == "-o" ]] && out="$a"
  case "$a" in http*) url="$a" ;; esac
  prev="$a"
done
[[ "${FAKE_CURL_FAIL:-0}" == 1 ]] && exit 22
case "$url" in
  *api.github.com*) echo '{"tag_name": "1.3.0"}' ;;
  *ros2.repos) [[ -n "$out" ]] && printf 'repositories: {}\n' > "$out" ;;
  *) if [[ -n "$out" ]]; then echo fake > "$out"; else echo fake; fi ;;
esac
''',
    "git": r'''#!/usr/bin/env bash
echo "git $*" >> "$FAKE_DIR/calls.log"
[[ "${1:-}" == clone ]] || exit 0
target="${@: -1}"; url="${@: -2:1}"
for b in ${FAKE_GIT_MISSING_BRANCHES:-}; do
  [[ " $* " == *" --branch $b "* ]] && exit 128
done
[[ "${FAKE_GIT_FAIL:-0}" == 1 ]] && exit 128
name="$(basename "$url" .git)"
pkgxml() { mkdir -p "$1"; printf '<package format="3">\n  <name>%s</name>\n</package>\n' "$2" > "$1/package.xml"; }
case "$name" in
  turtlebot3)
    for p in turtlebot3_node turtlebot3_bringup turtlebot3_navigation2 turtlebot3_teleop; do pkgxml "$target/$p" "$p"; done ;;
  ldlidar_stl_ros2)
    pkgxml "$target" "$name"
    mkdir -p "$target/ldlidar_driver/src/logger"
    printf '#include "log_module.h"\n\n#include <time.h>\n' > "$target/ldlidar_driver/src/logger/log_module.cpp" ;;
  ros2) mkdir -p "$target"; printf 'repositories: {}\n' > "$target/ros2.repos" ;;
  *) pkgxml "$target" "$name" ;;
esac
''',
    "colcon": r'''#!/usr/bin/env bash
echo "colcon $* MAKEFLAGS=${MAKEFLAGS:-}" >> "$FAKE_DIR/calls.log"
case "${1:-}" in
  list) echo "src/ros2/rclpy/rclpy"; exit 0 ;;
  build) ;;
  *) exit 0 ;;
esac
base=""; prev=""
for a in "$@"; do [[ "$prev" == "--install-base" ]] && base="$a"; prev="$a"; done
if [[ -n "$base" ]]; then
  # Сборка самого ROS 2 из исходников.
  [[ "${FAKE_CORE_SLEEP:-0}" != 0 ]] && sleep "$FAKE_CORE_SLEEP"
  mkdir -p "$base/share/ament_index/resource_index/packages" "$base/lib/python$FAKE_PYVER/site-packages/rclpy"
  cp "$FAKE_DIR/colcon_setup.bash" "$base/setup.bash"
  for pkg in rclpy sensor_msgs std_msgs std_srvs; do touch "$base/share/ament_index/resource_index/packages/$pkg"; done
  echo "ok = True" > "$base/lib/python$FAKE_PYVER/site-packages/rclpy/__init__.py"
  exit "${FAKE_CORE_RC:-0}"
fi
mkdir -p install
rc=0
while read -r xml; do
  dir="$(dirname "$xml")"
  [[ -e "$dir/COLCON_IGNORE" ]] && continue
  name="$(sed -n 's:.*<name>\(.*\)</name>.*:\1:p' "$xml" | head -n 1)"
  if [[ " ${FAKE_COLCON_FAIL:-} " == *" $name "* ]]; then rc=1; continue; fi
  mkdir -p "install/$name/share/ament_index/resource_index/packages"
  touch "install/$name/share/ament_index/resource_index/packages/$name"
  echo "$name" >> "$FAKE_DIR/built.txt"
done < <(find src -name .git -prune -o -name package.xml -print)
cp "$FAKE_DIR/colcon_setup.bash" install/setup.bash
exit $rc
''',
    "rosdep": '#!/usr/bin/env bash\necho "rosdep $*" >> "$FAKE_DIR/calls.log"\nexit 0\n',
    "vcs": r'''#!/usr/bin/env bash
echo "vcs $*" >> "$FAKE_DIR/calls.log"
if [[ "${1:-}" == import ]]; then
  dest="${@: -1}"
  mkdir -p "$dest/ros2/rclpy/rclpy"
  printf '<package format="3"><name>rclpy</name></package>\n' > "$dest/ros2/rclpy/rclpy/package.xml"
fi
''',
    "add-apt-repository": '#!/usr/bin/env bash\necho "add-apt-repository $*" >> "$FAKE_DIR/calls.log"\n',
    "systemctl": '#!/usr/bin/env bash\necho "systemctl $*" >> "$FAKE_DIR/calls.log"\nexit 3\n',
    "cmake": '#!/usr/bin/env bash\nexit 0\n',
    "g++": '#!/usr/bin/env bash\nexit 0\n',
    "make": '#!/usr/bin/env bash\nexit 0\n',
}

OS_RELEASES = {
    "jammy": 'PRETTY_NAME="Ubuntu 22.04.5 LTS"\nID=ubuntu\nVERSION_CODENAME=jammy\nUBUNTU_CODENAME=jammy\n',
    "noble": 'PRETTY_NAME="Ubuntu 24.04.2 LTS"\nID=ubuntu\nVERSION_CODENAME=noble\nUBUNTU_CODENAME=noble\n',
    "mint": ('PRETTY_NAME="Linux Mint 22"\nID=linuxmint\nID_LIKE="ubuntu debian"\n'
             'VERSION_CODENAME=wilma\nUBUNTU_CODENAME=noble\n'),
    "bookworm": 'PRETTY_NAME="Debian GNU/Linux 12 (bookworm)"\nID=debian\nVERSION_CODENAME=bookworm\n',
    "gentoo": 'PRETTY_NAME="Gentoo Linux"\nID=gentoo\n',
}


class FakeSystem:
    """Временный проект + фальшивые системные команды для ros2_setup.sh."""

    def __init__(self, tmp_path: Path, os_name: str = "jammy") -> None:
        self.tmp = tmp_path
        self.project = tmp_path / "robot"
        (self.project / "scripts").mkdir(parents=True)
        for name in ("ros2_setup.sh", "ros2_lidar.sh"):
            shutil.copy2(SCRIPTS / name, self.project / "scripts" / name)
        shutil.copytree(WS / "src" / "zapret_lidar", self.project / "ros2_ws" / "src" / "zapret_lidar",
                        ignore=shutil.ignore_patterns("__pycache__"))
        (self.project / ".env").write_text("RC_SECRET_KEY=x\nRC_LIDAR_SOURCE=auto\n", encoding="utf-8")
        self.fake = tmp_path / "fake"
        self.bin = self.fake / "bin"
        self.bin.mkdir(parents=True)
        for name, body in FAKES.items():
            path = self.bin / name
            path.write_text(body, encoding="utf-8")
            path.chmod(0o755)
        (self.fake / "colcon_setup.bash").write_text(COLCON_LIKE_SETUP, encoding="utf-8")
        self.opt = tmp_path / "opt_ros"
        self.opt.mkdir()
        self.apt_dir = tmp_path / "etc_apt"
        (self.apt_dir / "sources.list.d").mkdir(parents=True)
        (self.apt_dir / "sources.list").write_text(
            "deb http://archive.ubuntu.com/ubuntu jammy main restricted\n", encoding="utf-8")
        self.os_release = tmp_path / "os-release"
        self.set_os(os_name)
        self.env = {
            key: value for key, value in os.environ.items()
            if not key.startswith(("RC_", "ROS_", "AMENT_", "COLCON_"))
        }
        self.env.update({
            "PATH": "%s:%s" % (self.bin, os.environ.get("PATH", "/usr/bin:/bin")),
            "FAKE_DIR": str(self.fake),
            "FAKE_PYVER": PYVER,
            "RC_ROS_OS_RELEASE": str(self.os_release),
            "RC_ROS_OPT_DIR": str(self.opt),
            "RC_ROS_APT_DIR": str(self.apt_dir),
            "RC_ROS_ROSDEP_LIST": str(tmp_path / "rosdep" / "20-default.list"),
            "TMPDIR": str(tmp_path),
            "LD_LIBRARY_PATH": "",
            "PYTHONPATH": "",
        })

    def set_os(self, name: str) -> None:
        self.os_release.write_text(OS_RELEASES[name], encoding="utf-8")

    def run(self, *args: str, script: str = "ros2_setup.sh", timeout: float = 120,
            **env: str) -> subprocess.CompletedProcess[str]:
        full_env = dict(self.env, **env)
        return subprocess.run(
            ["bash", str(self.project / "scripts" / script), *args],
            cwd=str(self.project), env=full_env, capture_output=True, text=True,
            timeout=timeout, stdin=subprocess.DEVNULL, start_new_session=True)

    def calls(self) -> list[str]:
        path = self.fake / "calls.log"
        return path.read_text(encoding="utf-8").splitlines() if path.is_file() else []

    def reset_calls(self) -> None:
        for name in ("calls.log", "built.txt"):
            (self.fake / name).unlink(missing_ok=True)

    def built(self) -> list[str]:
        path = self.fake / "built.txt"
        return path.read_text(encoding="utf-8").split() if path.is_file() else []

    def state(self) -> dict:
        path = self.project / "var" / "ros2_setup.json"
        return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else {}

    def env_file(self) -> str:
        return (self.project / ".env").read_text(encoding="utf-8")


def installs(calls: list[str]) -> list[str]:
    """Пакеты из всех вызовов apt-get install."""
    out: list[str] = []
    for line in calls:
        if line.startswith("apt-get ") and " install " in line:
            out += [word for word in line.split(" install ", 1)[1].split()
                    if not word.startswith("-")]
    return out


@pytest.fixture
def system(tmp_path):
    return FakeSystem(tmp_path)


# --------------------------------------------------------------------------- #
# Чистая Ubuntu: всё ставится и компилируется само, без флагов
# --------------------------------------------------------------------------- #


def test_fresh_ubuntu_gets_ros_drivers_and_a_built_workspace_without_flags(system):
    result = system.run()
    assert result.returncode == 0, result.stdout + result.stderr
    calls = system.calls()
    packages = installs(calls)

    # 1. Официальный способ подключить репозиторий ROS 2: universe + ros2-apt-source.
    assert any(line.startswith("add-apt-repository") and "universe" in line for line in calls)
    assert any("ros2-apt-source_1.3.0.jammy_all.deb" in line for line in calls if line.startswith("curl"))
    assert any(line.startswith("dpkg -i") for line in calls)
    # 2. Сам ROS 2 и инструменты сборки; удалять пакеты apt не разрешено.
    assert "ros-humble-ros-base" in packages
    assert "ros-dev-tools" in packages
    assert all("--no-remove" in line for line in calls if line.startswith("apt-get") and " install " in line)
    assert any("DPkg::Lock::Timeout" in line for line in calls if line.startswith("apt-get"))
    # 3. Драйверы из apt, ldlidar — из исходников (его в apt нет).
    for name in ("ros-humble-hls-lfcd-lds-driver", "ros-humble-rplidar-ros", "ros-humble-turtlebot3-node"):
        assert name in packages, name
    assert any(line.startswith("git clone") and "ldlidar_stl_ros2" in line for line in calls)
    # Известная ошибка компиляции ldlidar на Ubuntu 24.04+ правится сама.
    log_module = (system.project / "ros2_ws/src/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp")
    assert "#include <pthread.h>" in log_module.read_text(encoding="utf-8")
    # 4. rosdep для исходников и colcon по ВСЕМУ ros2_ws (не только zapret_lidar).
    assert any(line.startswith("rosdep install") and "--from-paths" in line for line in calls)
    builds = [line for line in calls if line.startswith("colcon build")]
    assert len(builds) == 1
    assert "--packages-select" not in builds[0]
    assert "--continue-on-error" in builds[0]
    assert "MAKEFLAGS=-j" in builds[0]
    assert set(system.built()) >= {"zapret_lidar", "ldlidar_stl_ros2"}
    # 5. Всё проверено — лидар переключается на ROS 2.
    assert "RC_LIDAR_SOURCE=ros" in system.env_file()
    assert "RC_LIDAR_ROS_TOPIC=/scan" in system.env_file()
    state = system.state()
    assert state["state"] == "ok", state
    assert state["distro"] == "humble"
    assert (system.project / "var" / "ros2_setup.log").is_file()


def test_second_run_is_quick_and_installs_nothing(system):
    assert system.run().returncode == 0
    system.reset_calls()
    started = time.monotonic()
    result = system.run("--quiet")
    assert result.returncode == 0, result.stdout + result.stderr
    calls = system.calls()
    assert installs(calls) == []
    assert not [line for line in calls if line.startswith(("colcon build", "git clone", "rosdep install"))]
    assert not [line for line in calls if line.startswith("sudo ") and "-n" not in line.split()[1:2]]
    assert time.monotonic() - started < 30


def test_changed_sources_are_recompiled_and_rebuild_forces_it(system):
    assert system.run().returncode == 0
    system.reset_calls()
    (system.project / "ros2_ws/src/zapret_lidar/zapret_lidar/new_module.py").write_text("X = 1\n")
    assert system.run("--quiet").returncode == 0
    assert [line for line in system.calls() if line.startswith("colcon build")]

    system.reset_calls()
    assert system.run("--quiet").returncode == 0
    assert not [line for line in system.calls() if line.startswith("colcon build")]
    assert system.run("--rebuild", "--quiet").returncode == 0
    assert [line for line in system.calls() if line.startswith("colcon build")]


def test_editing_our_package_recompiles_without_rosdep_or_sudo(system):
    assert system.run().returncode == 0
    system.reset_calls()
    (system.project / "ros2_ws/src/zapret_lidar/zapret_lidar/drivers.py").write_text(
        (WS / "src/zapret_lidar/zapret_lidar/drivers.py").read_text(encoding="utf-8") + "\n# правка\n",
        encoding="utf-8")
    assert system.run("--quiet", FAKE_SUDO="deny").returncode == 0
    calls = system.calls()
    assert [line for line in calls if line.startswith("colcon build")]
    # rosdep (и пароль sudo) нужен только при новых/изменённых драйверах.
    assert not [line for line in calls if line.startswith("rosdep")]
    assert not [line for line in calls if line.startswith("sudo -v")]


def test_driver_set_from_env_file_is_honoured(system):
    with (system.project / ".env").open("a", encoding="utf-8") as handle:
        handle.write("RC_ROS_DRIVERS=sllidar_ros2\n")
    result = system.run()
    assert result.returncode == 0, result.stdout + result.stderr
    calls = system.calls()
    assert any(line.startswith("git clone") and "sllidar_ros2" in line for line in calls)
    assert not any(line.startswith("git clone") and "ldlidar_stl_ros2" in line for line in calls)
    assert "sllidar_ros2" in system.built()


def test_linux_mint_uses_the_ubuntu_codename(system):
    system.set_os("mint")
    result = system.run()
    assert result.returncode == 0, result.stdout + result.stderr
    calls = system.calls()
    assert "ros-jazzy-ros-base" in installs(calls)
    assert any(".noble_all.deb" in line for line in calls if line.startswith("curl"))
    assert (system.opt / "jazzy" / "setup.bash").is_file()


def test_noble_suites_are_completed_for_ros_dev_tools(system):
    system.set_os("noble")
    sources = system.apt_dir / "sources.list.d" / "ubuntu.sources"
    sources.write_text("Types: deb\nURIs: http://archive.ubuntu.com/ubuntu\nSuites: noble\n"
                       "Components: main restricted universe multiverse\n", encoding="utf-8")
    assert system.run().returncode == 0
    assert "Suites: noble noble-updates noble-backports" in sources.read_text(encoding="utf-8")
    # universe уже включён — add-apt-repository не нужен.
    assert not [line for line in system.calls() if line.startswith("add-apt-repository")]


def test_legacy_ros_list_is_replaced_by_ros2_apt_source(system):
    legacy = system.apt_dir / "sources.list.d" / "ros2.list"
    legacy.write_text("deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] "
                      "http://packages.ros.org/ros2/ubuntu jammy main\n", encoding="utf-8")
    assert system.run().returncode == 0
    assert not legacy.exists(), "старый ros2.list конфликтует с ros2-apt-source (Signed-By)"


def test_driver_missing_from_apt_is_cloned_and_compiled(system):
    result = system.run(FAKE_APT_MISSING="ros-humble-hls-lfcd-lds-driver",
                        FAKE_GIT_MISSING_BRANCHES="humble")
    assert result.returncode == 0, result.stdout + result.stderr
    calls = system.calls()
    assert "ros-humble-hls-lfcd-lds-driver" not in installs(calls)
    clones = [line for line in calls if line.startswith("git clone") and "hls_lfcd_lds_driver" in line]
    # Ветка дистрибутива отсутствует — берётся следующая (rolling-devel).
    assert any("--branch humble" in line for line in clones)
    assert any("--branch rolling-devel" in line for line in clones)
    assert "hls_lfcd_lds_driver" in system.built()


def test_apt_failure_for_a_driver_falls_back_to_sources(system):
    result = system.run(FAKE_APT_FAIL="ros-humble-rplidar-ros")
    assert result.returncode == 0, result.stdout + result.stderr
    assert any(line.startswith("git clone") and "rplidar_ros" in line and "--branch ros2" in line
               for line in system.calls())
    assert "rplidar_ros" in system.built()


def test_unavailable_optional_driver_does_not_ask_for_sudo_every_start(system):
    assert system.run(FAKE_APT_MISSING="ros-humble-turtlebot3-node").returncode == 0
    # turtlebot3_node из исходников не собирается без явного выбора модели opencr.
    assert not [line for line in system.calls() if line.startswith("git clone") and "turtlebot3" in line]
    system.reset_calls()
    assert system.run("--quiet", FAKE_APT_MISSING="ros-humble-turtlebot3-node").returncode == 0
    assert not [line for line in system.calls() if line.startswith(("sudo -v", "apt-get"))]


def test_opencr_model_builds_turtlebot3_node_from_sources_trimmed(system):
    with (system.project / ".env").open("a", encoding="utf-8") as handle:
        handle.write("RC_LIDAR_PROTOCOL=opencr\n")
    result = system.run(FAKE_APT_MISSING="ros-humble-turtlebot3-node")
    assert result.returncode == 0, result.stdout + result.stderr
    repo = system.project / "ros2_ws" / "src" / "turtlebot3"
    assert (repo / "turtlebot3_node" / "package.xml").is_file()
    # Навигация/телеуправление потянули бы Nav2 — они исключены из сборки.
    assert (repo / "turtlebot3_navigation2" / "COLCON_IGNORE").is_file()
    assert not (repo / "turtlebot3_node" / "COLCON_IGNORE").exists()
    assert "turtlebot3_node" in system.built()
    assert "turtlebot3_navigation2" not in system.built()


def test_operator_choice_serial_is_never_overwritten(system):
    (system.project / ".env").write_text("RC_LIDAR_SOURCE=serial\n", encoding="utf-8")
    assert system.run().returncode == 0
    assert "RC_LIDAR_SOURCE=serial" in system.env_file()
    assert "RC_LIDAR_SOURCE=ros" not in system.env_file()


def test_partial_driver_build_failure_is_reported_but_not_fatal(system):
    result = system.run(FAKE_COLCON_FAIL="ldlidar_stl_ros2")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "ldlidar_stl_ros2" in result.stdout
    assert system.state()["state"] == "warn"
    # Сломанный драйвер не компилируется заново на каждом старте.
    system.reset_calls()
    assert system.run("--quiet", FAKE_COLCON_FAIL="ldlidar_stl_ros2").returncode == 0
    assert not [line for line in system.calls() if line.startswith("colcon build")]


def test_failed_workspace_build_exits_with_error(system):
    result = system.run(FAKE_COLCON_FAIL="zapret_lidar")
    assert result.returncode == 1
    assert system.state()["state"] == "failed"
    assert "RC_LIDAR_SOURCE=auto" in system.env_file()


# --------------------------------------------------------------------------- #
# Нет пароля/терминала, --no-install, неподдерживаемые системы
# --------------------------------------------------------------------------- #


def test_without_password_and_terminal_nothing_is_installed_and_code_is_3(system):
    result = system.run(FAKE_SUDO="deny")
    assert result.returncode == 3, result.stdout + result.stderr
    assert installs(system.calls()) == []
    state = system.state()
    assert state["state"] == "need_root"
    assert "пароль" in state["message"]
    assert "Запустить робот" in result.stdout


@pytest.mark.skipif(shutil.which("colcon") is not None, reason="в системе есть настоящий colcon")
def test_missing_colcon_without_password_is_reported_as_need_root(system):
    make_colcon_prefix(system)
    (system.bin / "colcon").unlink()
    # Запасной путь «colcon через pip» без сети тоже не сработает.
    result = system.run(FAKE_SUDO="deny", PIP_NO_INDEX="1", PIP_FIND_LINKS="", PIP_DISABLE_PIP_VERSION_CHECK="1")
    assert result.returncode == 3, result.stdout + result.stderr
    assert "пароль администратора" in result.stdout
    assert system.state()["state"] == "need_root"


def test_no_install_without_ros_leaves_state_untouched(system):
    result = system.run("--no-install", "--no-env")
    assert result.returncode == 1
    assert [line for line in system.calls() if line != "dpkg --print-architecture"] == []
    assert not (system.project / "var" / "ros2_setup.json").exists()


def test_no_install_builds_but_never_calls_apt_or_sudo(system):
    assert system.run().returncode == 0
    (system.project / "ros2_ws/src/zapret_lidar/zapret_lidar/changed.py").write_text("Y = 2\n")
    system.reset_calls()
    result = system.run("--no-install", "--no-env", "--quiet", FAKE_SUDO="deny")
    assert result.returncode == 0, result.stdout + result.stderr
    calls = system.calls()
    assert [line for line in calls if line.startswith("colcon build")]
    assert not [line for line in calls if line.startswith(("apt-get", "sudo", "rosdep install"))]


def test_running_setup_is_not_waited_for_with_no_wait(system):
    (system.project / "var").mkdir()
    lock = system.project / "var" / "ros2_setup.lock"
    lock.write_text("")
    holder = subprocess.Popen(["flock", str(lock), "sleep", "20"])
    try:
        time.sleep(0.3)
        result = system.run("--no-wait")
        assert result.returncode == 75
        assert "уже идёт" in result.stdout
    finally:
        holder.kill()
        holder.wait()


def test_unknown_os_gets_a_clear_explanation(system):
    system.set_os("gentoo")
    result = system.run()
    assert result.returncode == 1
    assert "готовыми пакетами не выпускается" in result.stdout + result.stderr
    assert installs(system.calls()) == []


def test_debian_compiles_ros_from_sources_in_background(system):
    system.set_os("bookworm")
    check = json.loads(system.run("--check", "--json").stdout)
    assert check["method"] == "source"
    assert check["distro"] == "jazzy"

    result = system.run("--background-build")
    assert result.returncode == 4, result.stdout + result.stderr
    calls = system.calls()
    assert any(line.startswith("vcs import") and "--shallow" in line for line in calls)
    assert any(line.startswith("rosdep install") for line in calls)
    # Исходники драйверов скачаны сразу, пока есть сеть и терминал.
    assert (system.project / "ros2_ws/src/ldlidar_stl_ros2/package.xml").is_file()

    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        state = system.state()
        if state.get("state") in ("ok", "warn", "failed") and state.get("phase") == "done":
            break
        time.sleep(0.3)
    state = system.state()
    assert state.get("state") in ("ok", "warn"), state
    target = system.opt / "jazzy"
    assert (target / "setup.bash").is_file()
    assert not (target / ".zapret_build_incomplete").exists()
    core = [line for line in system.calls() if line.startswith("colcon build") and "--merge-install" in line]
    assert core and "--packages-up-to" in core[0] and "rclpy" in core[0]
    # Проприетарный Connext и запасной Cyclone DDS не компилируются.
    assert "--packages-skip" in core[0] and "rmw_connextdds" in core[0] and "cyclonedds" in core[0]
    assert "ldlidar_stl_ros2" in system.built()
    assert (target / ".zapret_source_build").is_file()

    # Дальше этот ROS 2 — «свой»: ни apt-пакетов ros-jazzy-*, ни rosdep, ни пересборки ядра.
    system.reset_calls()
    result = system.run("--quiet")
    assert result.returncode == 0, result.stdout + result.stderr
    calls = system.calls()
    assert not [pkg for pkg in installs(calls) if pkg.startswith("ros-jazzy-")]
    assert not [line for line in calls if line.startswith(("rosdep install", "vcs import"))]
    assert not [line for line in calls if "--merge-install" in line]


def test_incomplete_source_build_is_not_mistaken_for_ros(system):
    prefix = system.opt / "jazzy"
    prefix.mkdir()
    (prefix / "setup.bash").write_text("# colcon пишет его и после неудачи\n")
    (prefix / ".zapret_build_incomplete").write_text("")
    system.set_os("bookworm")
    payload = json.loads(system.run("--check", "--json").stdout)
    assert payload["prefix"] == ""


# --------------------------------------------------------------------------- #
# ros2_lidar.sh: source под set -u, автосборка, --keep-alive
# --------------------------------------------------------------------------- #


def make_colcon_prefix(system: FakeSystem, distro: str = "humble") -> Path:
    prefix = system.opt / distro
    (prefix / "share" / "ament_index" / "resource_index" / "packages").mkdir(parents=True)
    (prefix / "lib" / ("python" + PYVER) / "site-packages").mkdir(parents=True)
    (prefix / "setup.bash").write_text(COLCON_LIKE_SETUP, encoding="utf-8")
    return prefix


def test_lidar_script_survives_colcon_setup_scripts_under_set_u(system):
    make_colcon_prefix(system)
    result = system.run("--status", "--no-setup", script="ros2_lidar.sh")
    output = result.stdout + result.stderr
    assert "unbound variable" not in output, output
    assert "COLCON_TRACE" not in output
    # Дошли до проверки топиков: команды ros2 в тесте нет — так и говорится.
    assert "ros2 не найдена" in output


def test_lidar_script_builds_a_missing_workspace_by_itself(system):
    make_colcon_prefix(system)
    result = system.run("--print", script="ros2_lidar.sh")
    assert result.returncode == 0
    assert not [line for line in system.calls() if line.startswith("colcon build")], \
        "--print ничего не собирает"

    fake_python = system.bin / "python3"
    fake_python.write_text(textwrap.dedent('''\
        #!/usr/bin/env bash
        if [[ " $* " == *" zapret_lidar.supervisor "* ]]; then
          echo "supervisor PYTHONPATH=$PYTHONPATH" >> "$FAKE_DIR/calls.log"; exit 0
        fi
        exec %s "$@"
    ''') % sys.executable, encoding="utf-8")
    fake_python.chmod(0o755)
    result = system.run(script="ros2_lidar.sh")
    assert result.returncode == 0, result.stdout + result.stderr
    calls = system.calls()
    assert [line for line in calls if line.startswith("colcon build")]
    started = [line for line in calls if line.startswith("supervisor ")]
    assert started and "ros2_ws/src/zapret_lidar" in started[0]


def test_keep_alive_restarts_on_sigusr1_and_stops_on_term(system):
    make_colcon_prefix(system)
    fake_python = system.bin / "python3"
    fake_python.write_text(textwrap.dedent('''\
        #!/usr/bin/env bash
        if [[ " $* " == *" zapret_lidar.supervisor "* ]]; then
          echo start >> "$FAKE_DIR/supervisor.log"
          trap 'echo stop >> "$FAKE_DIR/supervisor.log"; exit 0' TERM
          while :; do sleep 0.1; done
        fi
        exec %s "$@"
    ''') % sys.executable, encoding="utf-8")
    fake_python.chmod(0o755)
    proc = subprocess.Popen(
        ["bash", str(system.project / "scripts" / "ros2_lidar.sh"), "--keep-alive", "--no-setup"],
        cwd=str(system.project), env=system.env, stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL, start_new_session=True)
    events = system.fake / "supervisor.log"
    pidfile = system.project / "var" / "lidar.pid"

    def wait_for(predicate, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.1)
        return False

    try:
        assert wait_for(lambda: events.is_file() and events.read_text().count("start") == 1)
        assert pidfile.read_text().strip() == str(proc.pid)
        os.kill(proc.pid, signal.SIGUSR1)       # «обновление: перезапусти с новым кодом»
        assert wait_for(lambda: events.read_text().count("start") == 2)
        assert "stop" in events.read_text()
        os.kill(proc.pid, signal.SIGTERM)
        proc.wait(timeout=15)
        assert events.read_text().count("stop") == 2
        assert not pidfile.exists()
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()


# --------------------------------------------------------------------------- #
# Настоящий colcon (если он есть): пакет zapret_lidar реально собирается
# --------------------------------------------------------------------------- #


@pytest.mark.skipif(shutil.which("colcon") is None, reason="нужен colcon (pip install colcon-common-extensions)")
def test_real_colcon_builds_zapret_lidar_through_the_setup_script(tmp_path):
    system = FakeSystem(tmp_path)
    (system.bin / "colcon").unlink()        # настоящий colcon из PATH
    make_colcon_prefix(system)
    result = system.run("--no-install", "--no-env", timeout=600)
    assert result.returncode == 0, result.stdout + result.stderr
    install = system.project / "ros2_ws" / "install"
    assert (install / "zapret_lidar" / "share" / "ament_index" / "resource_index"
            / "packages" / "zapret_lidar").is_file()
    assert (install / "zapret_lidar" / "lib" / "zapret_lidar" / "supervisor_node").is_file()
    check = subprocess.run(
        ["bash", "-c", 'set -u; set +u; . install/setup.bash; set -u; '
                       'python3 -c "import zapret_lidar.drivers as d; print(len(d.DRIVERS))"'],
        cwd=str(system.project / "ros2_ws"), capture_output=True, text=True, timeout=60,
        env=system.env)
    assert check.returncode == 0, check.stderr
    assert int(check.stdout.strip()) >= 5


# --------------------------------------------------------------------------- #
# Приложения: окружение ROS 2 с самого старта (robot_control/ros_env.py)
# --------------------------------------------------------------------------- #

from robot_control import ros_env  # noqa: E402


@pytest.fixture
def clean_ros_env(monkeypatch):
    for key in ("RC_ROS_PREFIX", "ROS_DISTRO", "AMENT_PREFIX_PATH", "RC_ROS_REEXEC",
                ros_env.ENV_READY_FLAG, "RC_ROS_SETUP_STATE", "COLCON_TRACE"):
        monkeypatch.delenv(key, raising=False)
    ros_env.reset_cache()
    yield monkeypatch
    ros_env.reset_cache()


def colcon_prefix(tmp_path: Path, distro: str = "jazzy") -> Path:
    prefix = tmp_path / distro
    (prefix / "share" / "ament_index" / "resource_index" / "packages").mkdir(parents=True)
    (prefix / "lib" / ("python" + PYVER) / "site-packages").mkdir(parents=True)
    (prefix / "setup.bash").write_text(COLCON_LIKE_SETUP, encoding="utf-8")
    return prefix


def test_process_env_is_what_source_setup_bash_gives(tmp_path, clean_ros_env):
    prefix = colcon_prefix(tmp_path)
    clean_ros_env.setenv("RC_ROS_PREFIX", str(prefix))
    clean_ros_env.setenv("LD_LIBRARY_PATH", "/usr/local/lib")
    env = ros_env.process_env()
    assert env is not None
    # set +u внутри: $COLCON_TRACE не задан, а source всё равно прошёл.
    assert env["LD_LIBRARY_PATH"].split(":")[0] == str(prefix / "lib")
    assert "/usr/local/lib" in env["LD_LIBRARY_PATH"]
    assert str(prefix / "lib" / ("python" + PYVER) / "site-packages") in env["PYTHONPATH"]
    assert env["ROS_DISTRO"] == "jazzy"
    assert env[ros_env.ENV_READY_FLAG] == str(prefix)
    assert "SHLVL" not in env and "_" not in env


def test_process_env_falls_back_to_computed_paths_without_setup_bash(tmp_path, clean_ros_env):
    prefix = colcon_prefix(tmp_path)
    (prefix / "setup.bash").unlink()
    (prefix / "lib" / "aarch64-linux-gnu").mkdir()
    clean_ros_env.setenv("RC_ROS_PREFIX", str(prefix))
    env = ros_env.process_env()
    assert env is not None
    assert str(prefix / "lib") in env["LD_LIBRARY_PATH"].split(":")
    # Не только x86_64: робот на ARM (Raspberry Pi, Jetson) тоже находит библиотеки.
    assert str(prefix / "lib" / "aarch64-linux-gnu") in env["LD_LIBRARY_PATH"].split(":")


def test_reexec_happens_once_and_can_be_disabled(tmp_path, clean_ros_env):
    prefix = colcon_prefix(tmp_path)
    clean_ros_env.setenv("RC_ROS_PREFIX", str(prefix))
    calls: list[tuple[str, list[str], dict]] = []
    clean_ros_env.setattr(ros_env.os, "execve",
                          lambda path, args, env: calls.append((path, list(args), dict(env))))
    assert ros_env.reexec_needed() is True
    ros_env.reexec_with_ros_env(["run_kiosk.py", "--no-open"])
    assert len(calls) == 1
    path, args, env = calls[0]
    assert path == sys.executable and args == [sys.executable, "run_kiosk.py", "--no-open"]
    assert env[ros_env.ENV_READY_FLAG] == str(prefix)
    assert env["LD_LIBRARY_PATH"].startswith(str(prefix / "lib"))

    # Уже перезапущены с этим префиксом — второго раза нет (никаких циклов).
    clean_ros_env.setenv(ros_env.ENV_READY_FLAG, str(prefix))
    assert ros_env.reexec_with_ros_env(["x"]) is False
    clean_ros_env.delenv(ros_env.ENV_READY_FLAG)
    clean_ros_env.setenv("RC_ROS_REEXEC", "0")
    assert ros_env.reexec_with_ros_env(["x"]) is False
    assert len(calls) == 1


def test_no_reexec_without_ros_and_incomplete_build_is_not_ros(tmp_path, clean_ros_env):
    clean_ros_env.setattr(ros_env, "DEFAULT_PREFIX", tmp_path / "opt")
    clean_ros_env.setattr(ros_env.os, "execve", lambda *a: pytest.fail("перезапуска быть не должно"))
    assert ros_env.reexec_with_ros_env(["x"]) is False
    prefix = colcon_prefix(tmp_path / "opt")
    (prefix / ros_env.INCOMPLETE_MARK).write_text("")
    clean_ros_env.setenv("RC_ROS_PREFIX", str(prefix))
    assert ros_env.prefix() is None
    assert ros_env.reexec_with_ros_env(["x"]) is False


def write_state(tmp_path: Path, **data) -> Path:
    path = tmp_path / "ros2_setup.json"
    path.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
    return path


def test_hint_tells_the_operator_what_the_auto_setup_is_doing(tmp_path, clean_ros_env):
    clean_ros_env.setattr(ros_env, "DEFAULT_PREFIX", tmp_path / "opt")
    clean_ros_env.setenv("RC_ROS_SETUP_STATE", str(write_state(
        tmp_path, state="running", phase="ros-base", message="скачиваю и ставлю ROS 2 jazzy",
        pid=os.getpid())))
    reason = ros_env.missing_reason()
    assert "ROS 2 не установлен" in reason and ros_env.SETUP_COMMAND in reason
    assert "сейчас идёт автоустановка" in reason and "jazzy" in reason
    assert ros_env.describe()["setup"]["state"] == "running"

    clean_ros_env.setenv("RC_ROS_SETUP_STATE", str(write_state(
        tmp_path, state="need_root", message="нужен пароль", pid=1)))
    assert "пароль администратора" in ros_env.missing_reason()
    assert "Запустить робот" in ros_env.missing_reason()

    dead = subprocess.Popen([sys.executable, "-c", "pass"])
    dead.wait()
    clean_ros_env.setenv("RC_ROS_SETUP_STATE", str(write_state(
        tmp_path, state="running", message="компилирую", pid=dead.pid)))
    assert ros_env.setup_state()["state"] == "interrupted"
    assert "прервали" in ros_env.missing_reason()


def test_ros_installed_after_start_asks_for_a_restart(tmp_path, clean_ros_env):
    prefix = colcon_prefix(tmp_path)
    clean_ros_env.setenv("RC_ROS_PREFIX", str(prefix))
    clean_ros_env.setattr(ros_env, "_rclpy_error", "ImportError: librcl.so: cannot open shared object file")
    reason = ros_env.missing_reason(prefix)
    assert "без его окружения" in reason and "перезапусти" in reason
    clean_ros_env.setenv(ros_env.ENV_READY_FLAG, str(prefix))
    assert "без его окружения" not in ros_env.missing_reason(prefix)


def test_sllidar_is_a_source_build_not_an_apt_package():
    assert "sllidar_ros2" not in ros_env.APT_PACKAGES
    assert "sllidar_ros2" in ros_env.GIT_SOURCES
    assert "lyrical" in ros_env.ROS_DISTROS


# --------------------------------------------------------------------------- #
# Мост в приложении: сначала rclpy.init(), потом узел
# --------------------------------------------------------------------------- #


def test_bridge_initialises_rclpy_before_creating_the_node(monkeypatch):
    from robot_control.ros_lidar import RosLidar

    events: list[str] = []
    rclpy = type(sys)("rclpy")
    state = {"init": False}
    rclpy.ok = lambda: state["init"]

    def init(args=None):
        events.append("init")
        state["init"] = True
    rclpy.init = init
    rclpy.spin = lambda node: events.append("spin")

    class Node:
        def __init__(self, name):
            if not state["init"]:
                raise RuntimeError("rcl node's context is invalid")
            events.append("node:" + name)

        def create_subscription(self, *a, **k):
            return object()

        def create_timer(self, *a, **k):
            return object()

        def create_client(self, *a, **k):
            return object()

    node_mod = type(sys)("rclpy.node")
    node_mod.Node = Node
    qos = type(sys)("rclpy.qos")
    for name in ("DurabilityPolicy", "HistoryPolicy", "ReliabilityPolicy"):
        setattr(qos, name, type(name, (), {"VOLATILE": 0, "KEEP_LAST": 0, "BEST_EFFORT": 0}))
    qos.QoSProfile = lambda **kwargs: kwargs
    sensor = type(sys)("sensor_msgs.msg")
    sensor.LaserScan = object
    std = type(sys)("std_msgs.msg")
    std.String = object
    for name, module in (("rclpy", rclpy), ("rclpy.node", node_mod), ("rclpy.qos", qos),
                         ("sensor_msgs", type(sys)("sensor_msgs")), ("sensor_msgs.msg", sensor),
                         ("std_msgs", type(sys)("std_msgs")), ("std_msgs.msg", std)):
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(ros_env, "import_rclpy", lambda: rclpy)

    bridge = RosLidar()
    bridge._spin()
    assert bridge.error == "", bridge.error
    assert events[:3] == ["init", "node:zapret_lidar_bridge", "spin"]


# --------------------------------------------------------------------------- #
# Обновление с GitHub: ros2_ws докомпилируется, а сбой ROS 2 не откатывает код
# --------------------------------------------------------------------------- #


def test_update_compiles_ros_workspace_and_survives_its_failure(tmp_path):
    from test_updater import git, make_remote, wait_for_terminal
    from robot_control.updater import UpdateManager

    _remote, seed, clone = make_remote(tmp_path)
    (seed / "README.md").write_text("version two\n", encoding="utf-8")
    git(seed, "add", "README.md")
    git(seed, "commit", "-m", "update")
    git(seed, "push", "origin", "main")

    calls: list[str] = []
    manager = UpdateManager(
        clone, branch="main",
        dependency_installer=lambda root: (calls.append("deps") or 0, "deps ok"),
        build_runner=lambda root: (calls.append("build") or 0, "build ok"),
        ros_builder=lambda root: (calls.append("ros") or 1, "colcon: ldlidar_stl_ros2 failed"),
        on_restart=lambda: calls.append("restart"),
    )
    assert manager.start() is True
    state = wait_for_terminal(manager)
    assert state["status"] == "done", state
    assert calls == ["deps", "build", "ros", "restart"]
    assert git(clone, "rev-parse", "HEAD") == git(seed, "rev-parse", "HEAD")
    log_text = json.dumps(manager.get_status(), ensure_ascii=False)
    assert "ros2_ws не собран" in log_text


def test_update_restarts_the_lidar_supervisor_loop(tmp_path):
    from robot_control import updater

    proc = subprocess.Popen(["bash", "-c", "exec -a ros2_lidar.sh sleep 30"])
    try:
        (tmp_path / "var").mkdir()
        (tmp_path / "var" / "lidar.pid").write_text(str(proc.pid))
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and "ros2_lidar.sh" not in " ".join(updater._read_cmdline(proc.pid)):
            time.sleep(0.05)
        updater._restart_lidar(tmp_path)
        assert proc.wait(timeout=5) == -signal.SIGUSR1
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()


def test_foreign_pid_in_lidar_pidfile_is_left_alone(tmp_path):
    from robot_control import updater

    proc = subprocess.Popen(["sleep", "30"])
    try:
        (tmp_path / "var").mkdir()
        (tmp_path / "var" / "lidar.pid").write_text(str(proc.pid))
        updater._restart_lidar(tmp_path)
        time.sleep(0.2)
        assert proc.poll() is None
    finally:
        proc.kill()
        proc.wait()


# --------------------------------------------------------------------------- #
# Запуск ярлыком и установщик знают про автоустановку
# --------------------------------------------------------------------------- #


def test_start_robot_sets_up_ros_and_starts_the_lidar_by_itself():
    text = (SCRIPTS / "start_robot.sh").read_text(encoding="utf-8")
    assert "ros2_setup.sh --background-build --no-wait" in text
    assert "ros2_lidar.sh --keep-alive" in text
    assert "RC_ROS_AUTO" in text
    stop = (SCRIPTS / "stop.sh").read_text(encoding="utf-8")
    assert "lidar.pid" in stop


def test_installer_installs_ros_by_default():
    text = (SCRIPTS / "install_ubuntu.sh").read_text(encoding="utf-8")
    assert 'ROS2="auto"' in text
    assert "bash scripts/ros2_setup.sh" in text
    # Старое поведение «ROS 2 только если уже стоит» убрано.
    assert 'compgen -G "/opt/ros/*"' not in text


def test_ros_scripts_are_executable_in_git():
    """Без бита исполнения ./scripts/ros2_setup.sh — «Permission denied», а
    служба robot-lidar — 203/EXEC: именно так ROS 2 «не устанавливался»."""
    if shutil.which("git") is None or not (ROOT / ".git").exists():
        pytest.skip("нужен git-клон")
    result = subprocess.run(["git", "-C", str(ROOT), "ls-files", "-s", "--", "scripts/*.sh"],
                            capture_output=True, text=True, timeout=30)
    modes = {line.split()[3]: line.split()[0] for line in result.stdout.splitlines()}
    assert modes, result.stderr
    for name in ("scripts/ros2_setup.sh", "scripts/ros2_lidar.sh", "scripts/start_robot.sh"):
        assert modes.get(name) == "100755", "%s: режим %s" % (name, modes.get(name))
