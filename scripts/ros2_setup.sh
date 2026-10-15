#!/usr/bin/env bash
#
# ROS 2 для лидара робота Zapret: ставится, докачивается и компилируется сам.
#
#   ./scripts/ros2_setup.sh              всё сразу: ROS 2 + драйверы + сборка ros2_ws
#   ./scripts/ros2_setup.sh --check      только показать, что найдено (код 0/1)
#   ./scripts/ros2_setup.sh --check --json   то же, машинный вывод
#   ./scripts/ros2_setup.sh --status     идёт ли установка сейчас и чем закончилась
#   ./scripts/ros2_setup.sh --no-install только собрать ros2_ws, без apt и sudo
#   ./scripts/ros2_setup.sh --rebuild    пересобрать ros2_ws с нуля
#   ./scripts/ros2_setup.sh --service    ещё и службу robot-lidar (systemd)
#   ./scripts/ros2_setup.sh --no-env     не трогать .env (RC_LIDAR_SOURCE=ros)
#   ./scripts/ros2_setup.sh --distro jazzy   выбрать дистрибутив ROS 2 вручную
#   Прежние флаги --install, --drivers и --ldlidar по-прежнему принимаются: всё
#   это теперь делается и без них.
#
# Каждый шаг выполняется, только если он нужен (повторный запуск — секунды):
#   1. ROS 2: /opt/ros/<дистрибутив>; нет — подключается официальный репозиторий
#      (пакет ros2-apt-source) и ставятся ros-<дистрибутив>-ros-base и ros-dev-tools.
#      Где готовых пакетов нет (Debian, Raspberry Pi OS, не-LTS Ubuntu), ROS 2
#      скачивается исходниками с GitHub и компилируется сам (--background-build —
#      в фоне, пока робот работает с лидаром по USB).
#   2. Драйверы лидаров — из apt; чего в apt нет (ldlidar_stl_ros2 и др.),
#      клонируется в ros2_ws/src, а известные ошибки их сборки правятся сами.
#   3. rosdep ставит зависимости исходников, colcon компилирует весь ros2_ws;
#      повторно — только если исходники изменились.
#   4. В .env: RC_LIDAR_SOURCE=ros, когда rclpy и сборка проверены (выбранный
#      оператором serial не трогается).
# Пароль sudo спрашивается один раз и только если что-то ставится. Без терминала
# (служба, автозапуск) установка с паролем невозможна — код 3 и подсказка.
# Коды: 0 — готово, 1 — ошибка, 3 — нужен пароль администратора, 4 — ROS 2
# компилируется в фоне, 75 — установка уже идёт. Журнал: var/ros2_setup.log.
#
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1
ROOT="$PWD"
SELF="$ROOT/scripts/ros2_setup.sh"
ORIG_ARGS="$*"

# Настройки RC_ROS_* можно держать в .env, как и остальные настройки робота
# (переменная окружения важнее файла — как в Config.from_env).
for _key in RC_ROS_PREFIX RC_ROS_DRIVERS RC_ROS_SOURCE_BUILD RC_ROS_BUILD_JOBS RC_ROS_APT_MIRROR \
            RC_ROS_SOURCE_DIR RC_ROS_CORE_PACKAGES RC_ROS_CORE_SKIP RC_LIDAR_PROTOCOL; do
  if [[ -z "${!_key:-}" && -f "$ROOT/.env" ]]; then
    _value="$(grep -E "^${_key}=" "$ROOT/.env" 2>/dev/null | tail -n 1 | cut -d= -f2- || true)"
    _value="${_value%\"}"; _value="${_value#\"}"
    [[ -n "$_value" ]] && export "$_key=$_value"
  fi
done
unset _key _value

DISTRO_ARG=""
MODE="setup"            # setup | check | status
DO_SYSTEM=1             # можно ставить системные пакеты (apt, sudo)
DO_BUILD=1
DO_REBUILD=0
DO_SERVICE=0
WRITE_ENV=1
AS_JSON=0
QUIET=0
WAIT_LOCK=1
BACKGROUND_BUILD=0
# Сборка самого ROS 2 из исходников там, где нет готовых пакетов:
# auto — когда больше нечем, 1 — принудительно, 0 — никогда.
SOURCE_BUILD="${RC_ROS_SOURCE_BUILD:-auto}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --distro)            DISTRO_ARG="${2:-}"; shift 2 ;;
    --distro=*)          DISTRO_ARG="${1#*=}"; shift ;;
    # Прежние флаги: установка и драйверы теперь делаются по умолчанию.
    --install|--drivers|--ldlidar) DO_SYSTEM=1; shift ;;
    --no-install|--build-only)     DO_SYSTEM=0; shift ;;
    --no-build)          DO_BUILD=0; shift ;;
    --rebuild)           DO_REBUILD=1; shift ;;
    --service)           DO_SERVICE=1; shift ;;
    --no-env)            WRITE_ENV=0; shift ;;
    --check)             MODE="check"; shift ;;
    --json)              AS_JSON=1; shift ;;
    --status)            MODE="status"; shift ;;
    --quiet|-q)          QUIET=1; shift ;;
    --no-wait)           WAIT_LOCK=0; shift ;;
    --background-build)  BACKGROUND_BUILD=1; shift ;;
    --from-source)       SOURCE_BUILD=1; shift ;;
    --no-source-build)   SOURCE_BUILD=0; shift ;;
    -h|--help)           awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"; exit 0 ;;
    *) printf '[ros2] неизвестный аргумент: %s (см. --help)\n' "$1" >&2; exit 1 ;;
  esac
done

# --- пути (RC_ROS_* для нестандартных систем и тестов) ---------------------- #
OS_RELEASE="${RC_ROS_OS_RELEASE:-/etc/os-release}"
OPT_ROS="${RC_ROS_OPT_DIR:-/opt/ros}"
APT_DIR="${RC_ROS_APT_DIR:-/etc/apt}"
ROSDEP_LIST="${RC_ROS_ROSDEP_LIST:-/etc/ros/rosdep/sources.list.d/20-default.list}"
WS="$ROOT/ros2_ws"
VAR_DIR="$ROOT/var"
STATE_FILE="$VAR_DIR/ros2_setup.json"
LOG_FILE="$VAR_DIR/ros2_setup.log"
LOCK_FILE="$VAR_DIR/ros2_setup.lock"
STAMP_FILE="$WS/install/.zapret_build_stamp"
ROSDEP_STAMP="$VAR_DIR/ros2_rosdep.stamp"
UNAVAILABLE_FILE="$VAR_DIR/ros2_unavailable.txt"
APT_STAMP="$VAR_DIR/ros2_apt_update.stamp"
TOOLS_VENV="$VAR_DIR/ros-tools"
# Метка «сборка ROS 2 из исходников не закончена»: такой префикс не считается
# установленным ROS 2 (colcon пишет setup.bash даже после неудачной сборки).
INCOMPLETE_MARK=".zapret_build_incomplete"
# Метка «этот ROS 2 скомпилирован из исходников»: пакетов ros-<дистрибутив>-* для
# такой системы нет, поэтому драйверы — сразу из исходников, а rosdep не нужен.
SOURCE_MARK=".zapret_source_build"
KNOWN_DISTROS="lyrical kilted jazzy iron humble rolling galactic foxy"
# Версия ros2-apt-source, если GitHub API недоступен (лимит запросов и т.п.).
ROS_APT_SOURCE_FALLBACK="1.3.0"
ROS_APT_URL="${RC_ROS_APT_MIRROR:-http://packages.ros.org/ros2/ubuntu}"
ROSDEP_SKIP_KEYS="fastcdr rti-connext-dds-6.0.1 rti-connext-dds-7.3.0 urdfdom_headers"
# Что собирать из исходников ROS 2, если готовых пакетов нет: ровно то, что
# нужно пакету zapret_lidar и драйверам лидаров (rclpy, rclcpp, сообщения,
# ros2 launch/run/topic), без RViz, демо и симуляторов.
CORE_PACKAGES="${RC_ROS_CORE_PACKAGES:-ros_environment ament_cmake_auto ament_cmake_ros ament_cmake_python rclcpp rclcpp_components rclpy sensor_msgs std_msgs std_srvs launch_ros ros2launch ros2run ros2topic ros2node ros2pkg ros2param ros2service}"
# Не собираем: RTI Connext (нужен проприетарный SDK) и Cyclone DDS с iceoryx —
# запасные middleware; по умолчанию в ROS 2 работает Fast DDS. В package.xml
# rmw_implementation они лишь дублируют необязательную group_depend.
CORE_SKIP="${RC_ROS_CORE_SKIP:-rmw_connextdds rmw_connextdds_common rmw_connextddsmicro rti_connext_dds_cmake_module rmw_cyclonedds_cpp cyclonedds iceoryx_binding_c iceoryx_hoofs iceoryx_posh}"
read -r -a CORE_PKGS <<< "$CORE_PACKAGES"
read -r -a CORE_SKIP_PKGS <<< "$CORE_SKIP"
export GIT_TERMINAL_PROMPT=0
# colcon/vcs/rosdep из pip — запасной путь, если apt их не дал (в конце PATH:
# версии из apt важнее).
[[ -d "$TOOLS_VENV/bin" ]] && PATH="$PATH:$TOOLS_VENV/bin"

if [[ -t 1 ]]; then
  C_OK=$'\033[1;32m'; C_WARN=$'\033[1;33m'; C_ERR=$'\033[1;31m'; C_OFF=$'\033[0m'
else
  C_OK=""; C_WARN=""; C_ERR=""; C_OFF=""
fi
log()  { printf '%s[ros2]%s %s\n' "$C_OK" "$C_OFF" "$*"; }
info() { [[ $QUIET -eq 1 ]] || log "$@"; }
warn() { printf '%s[ros2]%s %s\n' "$C_WARN" "$C_OFF" "$*"; }
err()  { printf '%s[ros2]%s %s\n' "$C_ERR" "$C_OFF" "$*" >&2; }
have() { command -v "$1" >/dev/null 2>&1; }

# --- система ------------------------------------------------------------------ #
OS_ID="" OS_NAME="" OS_VERSION_CODENAME="" OS_UBUNTU_CODENAME=""
read_os() {
  local key value
  [[ -r "$OS_RELEASE" ]] || return 0
  while IFS='=' read -r key value || [[ -n "$key" ]]; do
    value="${value%\"}"; value="${value#\"}"; value="${value%\'}"; value="${value#\'}"
    case "$key" in
      ID)               OS_ID="$value" ;;
      PRETTY_NAME)      OS_NAME="$value" ;;
      VERSION_CODENAME) OS_VERSION_CODENAME="$value" ;;
      UBUNTU_CODENAME)  OS_UBUNTU_CODENAME="$value" ;;
    esac
  done < "$OS_RELEASE"
}
read_os
# Linux Mint, Pop!_OS, Zorin и т.п. называют себя иначе, но пакеты у них —
# от Ubuntu: решает UBUNTU_CODENAME (так велит и официальная инструкция ROS 2).
OS_CODENAME="${OS_UBUNTU_CODENAME:-$OS_VERSION_CODENAME}"
ARCH="$(dpkg --print-architecture 2>/dev/null || uname -m)"

# Готовые пакеты ROS 2 (packages.ros.org) — только для этих версий Ubuntu (REP 2000).
apt_distros_for() {
  case "$1" in
    jammy)    echo "humble" ;;
    noble)    echo "jazzy kilted" ;;
    resolute) echo "lyrical rolling" ;;
    focal)    echo "foxy" ;;   # Ubuntu 20.04: Foxy без поддержки, но пакеты ещё есть
  esac
}
apt_distro_for() { local d; d="$(apt_distros_for "$1")"; printf '%s' "${d%% *}"; }
# Системы без готовых пакетов: какой ROS 2 компилировать из исходников.
source_distro_for() {
  case "$1" in
    bullseye)                    echo humble ;;
    bookworm|oracular)           echo jazzy ;;
    trixie|forky|plucky|questing) echo lyrical ;;
    jammy)                       echo humble ;;   # 32-битный ARM: пакетов нет и для LTS
    noble)                       echo jazzy ;;
    resolute)                    echo lyrical ;;
  esac
}
# rosdep не знает свежие не-LTS Ubuntu: берём правила ближайшей LTS.
rosdep_os_for() {
  case "$1" in
    oracular)        echo "ubuntu:noble" ;;
    plucky|questing) echo "ubuntu:resolute" ;;
  esac
}
binary_arch_ok() { case "$ARCH" in amd64|arm64|x86_64|aarch64) return 0 ;; esac; return 1; }
binaries_for() {
  local distro="$1"
  [[ -n "$distro" ]] && binary_arch_ok && have apt-get || return 1
  [[ " $(apt_distros_for "$OS_CODENAME") " == *" $distro "* ]]
}

# --- префикс ROS 2 -------------------------------------------------------------- #
prefix_ok() {
  [[ -n "$1" && -d "$1" ]] || return 1
  [[ -f "$1/setup.bash" || -f "$1/local_setup.bash" ]] || return 1
  [[ ! -e "$1/$INCOMPLETE_MARK" ]]
}

find_prefix() {
  local name candidate
  if [[ -n "${RC_ROS_PREFIX:-}" ]] && prefix_ok "$RC_ROS_PREFIX"; then
    echo "$RC_ROS_PREFIX"; return 0
  fi
  for name in "$DISTRO_ARG" "${ROS_DISTRO:-}" "$(apt_distro_for "$OS_CODENAME")" \
              "$(source_distro_for "$OS_CODENAME")" $KNOWN_DISTROS; do
    [[ -n "$name" ]] || continue
    if prefix_ok "$OPT_ROS/$name"; then echo "$OPT_ROS/$name"; return 0; fi
  done
  for candidate in "$OPT_ROS"/*; do
    if prefix_ok "$candidate"; then echo "$candidate"; return 0; fi
  done
  return 1
}

prefix_distro() {
  local base="$1" name=""
  name="$(sed -n 's/^set;ROS_DISTRO;//p' \
    "$base/share/ros_environment/environment/1.ros_distro.dsv" 2>/dev/null | head -n 1)"
  [[ -n "$name" ]] || name="$(basename "$base")"
  printf '%s' "$name"
}

PREFIX="$(find_prefix || true)"
DISTRO=""
METHOD=""
resolve_distro() {
  if [[ -n "$DISTRO_ARG" ]]; then DISTRO="$DISTRO_ARG"
  elif [[ -n "$PREFIX" ]]; then DISTRO="$(prefix_distro "$PREFIX")"
  else DISTRO="$(apt_distro_for "$OS_CODENAME")"
       [[ -n "$DISTRO" ]] || DISTRO="$(source_distro_for "$OS_CODENAME")"
       [[ -n "$DISTRO" ]] || DISTRO="${ROS_DISTRO:-humble}"
  fi
}
choose_method() {
  if [[ -n "$PREFIX" ]]; then METHOD="present"
  elif [[ "$SOURCE_BUILD" != "1" ]] && binaries_for "$DISTRO"; then METHOD="apt"
  elif [[ "$SOURCE_BUILD" != "0" ]] && have apt-get \
       && { [[ -n "$(source_distro_for "$OS_CODENAME")" ]] || [[ "$SOURCE_BUILD" == "1" ]]; }; then
    METHOD="source"
  else METHOD="none"
  fi
}
resolve_distro
choose_method

# --- python и rclpy --------------------------------------------------------------- #
python_bin() {
  if [[ -x "$ROOT/.venv/bin/python" ]]; then echo "$ROOT/.venv/bin/python"; else echo "python3"; fi
}

prefix_pythonpath() {
  local base="$1" version dir
  version="$("$(python_bin)" -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null || echo 3.10)"
  for dir in "$base/lib/python$version/site-packages" \
             "$base/lib/python3/dist-packages" \
             "$base/lib/python3/site-packages"; do
    [[ -d "$dir" ]] && printf '%s:' "$dir"
  done
}

# rclpy — это C-расширение: ему нужны не только PYTHONPATH, но и LD_LIBRARY_PATH
# с библиотеками ROS 2. Поэтому проверяем так же, как работает робот: source
# setup.bash и import rclpy интерпретатором приложения (.venv).
rclpy_ok() {
  local base="$1" py
  [[ -n "$base" ]] || return 1
  py="$(python_bin)"
  if [[ -f "$base/setup.bash" ]]; then
    # shellcheck disable=SC1091
    ( set +u; . "$base/setup.bash" >/dev/null 2>&1; "$py" -c 'import rclpy' ) >/dev/null 2>&1 && return 0
  fi
  PYTHONPATH="$(prefix_pythonpath "$base")${PYTHONPATH:-}" \
  LD_LIBRARY_PATH="$base/lib:${LD_LIBRARY_PATH:-}" "$py" -c 'import rclpy' >/dev/null 2>&1
}

ament_packages() {
  local dir
  for dir in "${PREFIX:-/nonexistent}"/share/ament_index/resource_index/packages \
             "$WS"/install/share/ament_index/resource_index/packages \
             "$WS"/install/*/share/ament_index/resource_index/packages; do
    [[ -d "$dir" ]] || continue
    find "$dir" -maxdepth 1 -type f -printf '%f\n' 2>/dev/null
  done | sort -u
}

pkg_installed() {
  local name="$1" base
  for base in "${PREFIX:-}" "$WS/install/$name" "$WS/install"; do
    [[ -n "$base" && -f "$base/share/ament_index/resource_index/packages/$name" ]] && return 0
  done
  return 1
}

ws_ok() {
  [[ -f "$WS/install/setup.bash" ]] || return 1
  [[ -f "$WS/install/zapret_lidar/share/ament_index/resource_index/packages/zapret_lidar" \
     || -f "$WS/install/share/ament_index/resource_index/packages/zapret_lidar" ]]
}

# Пакеты в ros2_ws/src (без COLCON_IGNORE): имя из package.xml.
ws_source_packages() {
  local dir
  find "$WS/src" -name .git -prune -o -name package.xml -printf '%h\n' 2>/dev/null \
    | while read -r dir; do
        [[ -e "$dir/COLCON_IGNORE" ]] && continue
        sed -n 's:.*<name>[[:space:]]*\([^<[:space:]]*\)[[:space:]]*</name>.*:\1:p' "$dir/package.xml" | head -n 1
      done | sort -u
}

# Отпечаток зависимостей исходников (package.xml всех пакетов, кроме нашего):
# rosdep (а с ним sudo) нужен, только когда появился или изменился драйвер.
deps_fingerprint() {
  {
    printf 'distro=%s\n' "$DISTRO"
    find "$WS/src" -name .git -prune -o -name package.xml -print 2>/dev/null | LC_ALL=C sort \
      | while read -r xml; do
          [[ "$xml" == "$WS/src/zapret_lidar/package.xml" ]] && continue
          printf '%s\n' "$xml"; cat "$xml" 2>/dev/null
        done
  } | sha256sum | cut -c1-32
}

# Отпечаток исходников ros2_ws: сборка повторяется, только когда он меняется.
src_fingerprint() {
  {
    printf 'prefix=%s distro=%s\n' "${PREFIX:-}" "$DISTRO"
    find "$WS/src" \( -name .git -o -name __pycache__ -o -name '*.egg-info' \) -prune \
      -o -type f ! -name '*.pyc' -printf '%P %s %T@\n' 2>/dev/null | LC_ALL=C sort
  } | sha256sum | cut -c1-32
}

read_setting() {  # значение настройки: окружение, затем .env
  local key="$1" value
  value="$(printenv "$key" 2>/dev/null || true)"
  if [[ -z "$value" && -f "$ROOT/.env" ]]; then
    value="$(grep -E "^${key}=" "$ROOT/.env" 2>/dev/null | tail -n 1 | cut -d= -f2- || true)"
    value="${value%\"}"; value="${value#\"}"
  fi
  printf '%s' "$value"
}

# --- режим проверки (ничего не ставит и не пишет) ------------------------------- #
json_str() {
  local s="$1"
  s="${s//\\/\\\\}"; s="${s//\"/\\\"}"
  s="${s//$'\n'/ }"; s="${s//$'\r'/ }"; s="${s//$'\t'/ }"
  printf '"%s"' "$s"
}

main_check() {
  local packages rclpy="нет" built="нет" os_label
  packages="$(ament_packages || true)"
  [[ -n "$PREFIX" ]] && rclpy_ok "$PREFIX" && rclpy="да"
  ws_ok && built="да"
  os_label="${OS_NAME:-${OS_ID:-?}} (${OS_CODENAME:-?}, $ARCH)"
  if [[ $AS_JSON -eq 1 ]]; then
    printf '{"distro":%s,"prefix":%s,"rclpy":%s,"packages":[%s],"workspace":%s,"built":%s,"method":%s,"os":%s}\n' \
      "$(json_str "$DISTRO")" "$(json_str "${PREFIX:-}")" \
      "$([[ "$rclpy" == "да" ]] && echo true || echo false)" \
      "$(printf '%s\n' "$packages" | sed '/^$/d;s/.*/"&"/' | paste -sd, -)" \
      "$(json_str "$([[ -d "$WS/install" ]] && echo "$WS/install" || echo "")")" \
      "$([[ "$built" == "да" ]] && echo true || echo false)" \
      "$(json_str "$METHOD")" "$(json_str "$os_label")"
  else
    echo "ROS 2:      ${PREFIX:-не найден}"
    echo "дистрибутив: $DISTRO"
    echo "система:    $os_label"
    echo "rclpy:      $rclpy"
    echo "пакеты ROS 2 (драйверы и zapret_lidar):"
    printf '%s\n' "$packages" | sed '/^$/d;s/^/  • /'
    if [[ "$built" == "да" ]]; then
      echo "рабочее пространство: $WS/install (собрано)"
    else
      echo "рабочее пространство: не собрано"
    fi
    if [[ -z "$PREFIX" ]]; then
      case "$METHOD" in
        apt)    echo "поставить и собрать всё само: ./scripts/ros2_setup.sh (ROS 2 $DISTRO из apt)" ;;
        source) echo "поставить и собрать всё само: ./scripts/ros2_setup.sh (готовых пакетов нет — ROS 2 $DISTRO скомпилируется из исходников)" ;;
        *)      echo "для этой системы ROS 2 готовыми пакетами не выпускается (нужна Ubuntu 22.04/24.04)" ;;
      esac
    elif [[ "$built" != "да" || "$rclpy" != "да" ]]; then
      echo "докачать и собрать недостающее: ./scripts/ros2_setup.sh"
    fi
  fi
  [[ -n "$PREFIX" && "$rclpy" == "да" ]] && exit 0
  exit 1
}

main_status() {
  local busy="нет"
  if [[ -f "$LOCK_FILE" ]] && have flock; then
    flock -n "$LOCK_FILE" true 2>/dev/null || busy="да"
  fi
  echo "установка ROS 2 идёт сейчас: $busy"
  if [[ -f "$STATE_FILE" ]]; then
    if have python3; then
      python3 - "$STATE_FILE" <<'PY' 2>/dev/null || cat "$STATE_FILE"
import json, sys, time
data = json.load(open(sys.argv[1], encoding="utf-8"))
names = {"ok": "готово", "warn": "готово с замечаниями", "running": "идёт",
         "background": "компилируется в фоне", "failed": "ошибка",
         "need_root": "нужен пароль администратора"}
print("последний результат:  %s — %s" % (names.get(data.get("state"), data.get("state")),
                                         data.get("message") or data.get("phase") or ""))
if data.get("distro"):
    print("дистрибутив ROS 2:    %s (%s)" % (data.get("distro"), data.get("method") or "?"))
if data.get("updated"):
    print("обновлено:            %s" % time.strftime("%Y-%m-%d %H:%M:%S",
                                                     time.localtime(int(data["updated"]))))
PY
    else
      cat "$STATE_FILE"
    fi
  else
    echo "установка ROS 2 ещё не запускалась"
  fi
  if [[ -f "$LOG_FILE" ]]; then
    echo
    echo "конец журнала ($LOG_FILE):"
    tail -n 15 "$LOG_FILE"
  fi
  exit 0
}

[[ "$MODE" == "check" ]] && main_check
[[ "$MODE" == "status" ]] && main_status

# ============================================================================ #
# Установка и сборка
# ============================================================================ #

set_state() {  # set_state <состояние> <этап> [сообщение]
  local state="$1" phase="$2" message="${3:-}"
  mkdir -p "$VAR_DIR" 2>/dev/null || return 0
  {
    printf '{"state":%s,"phase":%s,"message":%s,' \
      "$(json_str "$state")" "$(json_str "$phase")" "$(json_str "$message")"
    printf '"distro":%s,"prefix":%s,"method":%s,"os":%s,' \
      "$(json_str "$DISTRO")" "$(json_str "${PREFIX:-}")" "$(json_str "$METHOD")" \
      "$(json_str "${OS_NAME:-$OS_ID $OS_CODENAME}")"
    printf '"pid":%s,"updated":%s,"log":%s}\n' "$$" "$(date +%s)" "$(json_str "$LOG_FILE")"
  } > "$STATE_FILE.tmp.$$" 2>/dev/null && mv -f "$STATE_FILE.tmp.$$" "$STATE_FILE" 2>/dev/null
  rm -f "$STATE_FILE.tmp.$$" 2>/dev/null
}

TEE_PID=""
start_log() {
  mkdir -p "$VAR_DIR" 2>/dev/null || return 0
  if [[ -f "$LOG_FILE" ]] && (( $(stat -c %s "$LOG_FILE" 2>/dev/null || echo 0) > 2000000 )); then
    mv -f "$LOG_FILE" "$LOG_FILE.1" 2>/dev/null || true
  fi
  printf '\n===== %s · ros2_setup.sh %s\n' "$(date '+%F %T')" "$ORIG_ARGS" >> "$LOG_FILE" 2>/dev/null || return 0
  # Фоновая сборка уже пишет вывод прямо в журнал — второй tee не нужен.
  [[ "${RC_ROS_SETUP_LOGGED:-0}" == "1" ]] && return 0
  exec > >(tee -a "$LOG_FILE") 2>&1
  TEE_PID=$!
}

acquire_lock() {
  have flock || return 0
  mkdir -p "$VAR_DIR" 2>/dev/null || return 0
  [[ -e "$LOCK_FILE" ]] || : > "$LOCK_FILE" 2>/dev/null || return 0
  [[ -r "$LOCK_FILE" ]] || return 0
  exec 8<"$LOCK_FILE"
  if ! flock -n 8; then
    if [[ $WAIT_LOCK -eq 0 ]]; then
      log "установка ROS 2 уже идёт в другом процессе (журнал: $LOG_FILE)"
      exit 75
    fi
    log "установка ROS 2 уже идёт в другом окне — жду, пока она закончится (журнал: $LOG_FILE)…"
    flock 8
  fi
}
release_lock() { exec 8<&- 2>/dev/null || true; }

# --- права администратора ----------------------------------------------------- #
ROOT_MODE=""            # root | sudo | ask | none
KEEPALIVE_PID=""
NEED_ROOT_FOR=()
has_tty() { [[ -t 0 ]] || { : </dev/tty; } 2>/dev/null; }
detect_root() {
  if [[ $EUID -eq 0 ]]; then ROOT_MODE="root"
  elif ! have sudo; then ROOT_MODE="none"
  elif sudo -n true 2>/dev/null; then ROOT_MODE="sudo"
  elif has_tty; then ROOT_MODE="ask"
  else ROOT_MODE="none"
  fi
}
start_keepalive() {
  [[ -n "$KEEPALIVE_PID" ]] && return 0
  # Пароль sudo живёт ~15 минут, а установка ROS 2 бывает дольше: продлеваем,
  # пока работает этот скрипт, чтобы пароль спросили один раз.
  ( exec 8<&- 2>/dev/null; while kill -0 "$$" 2>/dev/null; do sudo -n -v 2>/dev/null || exit 0; sleep 50; done ) \
    >/dev/null 2>&1 &
  KEEPALIVE_PID=$!
}
need_root() {  # need_root <зачем> — 0, если права есть (спросит пароль один раз)
  local why="$1"
  [[ -n "$ROOT_MODE" ]] || detect_root
  case "$ROOT_MODE" in
    root) return 0 ;;
    sudo) start_keepalive; return 0 ;;
    ask)
      log "нужны права администратора ($why) — sudo спросит пароль один раз"
      if sudo -v; then ROOT_MODE="sudo"; start_keepalive; return 0; fi
      ROOT_MODE="none" ;;
  esac
  [[ " ${NEED_ROOT_FOR[*]-} " == *" $why "* ]] || NEED_ROOT_FOR+=("$why")
  return 1
}
as_root() {
  case "$ROOT_MODE" in
    root) "$@" ;;
    sudo) sudo "$@" ;;
    *) err "нет прав администратора для: $*"; return 97 ;;
  esac
}

# Сборка и клоны — от владельца проекта, даже если скрипт запустили через sudo:
# иначе build/ и install/ станут root-овыми и следующий запуск их не перезапишет.
OWNER_UID="$(stat -c %u "$ROOT" 2>/dev/null || id -u)"
as_owner() {
  if [[ $EUID -eq 0 && "$OWNER_UID" != "0" ]]; then
    local user home
    user="$(id -un "$OWNER_UID" 2>/dev/null || true)"
    home="$(getent passwd "$OWNER_UID" 2>/dev/null | cut -d: -f6)"
    if [[ -n "$user" ]] && have runuser; then
      runuser -u "$user" -- env HOME="${home:-/tmp}" "$@"
    else
      sudo -u "#$OWNER_UID" -H "$@"
    fi
  else
    "$@"
  fi
}

# вызывается из trap
# shellcheck disable=SC2329
on_exit() {
  local status=$?
  [[ -n "$KEEPALIVE_PID" ]] && kill "$KEEPALIVE_PID" 2>/dev/null
  if [[ $EUID -eq 0 && "$OWNER_UID" != "0" ]]; then
    chown "$OWNER_UID" "$VAR_DIR" "$VAR_DIR"/ros2_* "$LOG_FILE"* "$STATE_FILE" "$LOCK_FILE" 2>/dev/null
  fi
  if [[ -n "$TEE_PID" ]]; then
    exec 1>&- 2>&-
    wait "$TEE_PID" 2>/dev/null
  fi
  return "$status"
}

# --- apt ------------------------------------------------------------------------ #
APT_UPDATED=0
apt_get() {
  as_root env DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=a apt-get -y -q \
    -o DPkg::Lock::Timeout=600 -o Dpkg::Options::=--force-confdef \
    -o Dpkg::Options::=--force-confold "$@"
}
apt_update() {
  [[ $APT_UPDATED -eq 1 && "${1:-}" != "force" ]] && return 0
  log "apt: обновляю списки пакетов…"
  # Чужой сломанный репозиторий не повод останавливаться: нужный нам может
  # обновиться нормально — дальше это проверит apt_has.
  apt_get update || warn "apt update завершился с ошибками (часть репозиториев недоступна) — продолжаю"
  APT_UPDATED=1
  date +%s > "$APT_STAMP" 2>/dev/null || true
}
apt_lists_fresh() {  # списки обновлялись этим скриптом за последние сутки
  local last
  last="$(cat "$APT_STAMP" 2>/dev/null || echo 0)"
  [[ "$last" =~ ^[0-9]+$ ]] && (( $(date +%s) - last < 86400 ))
}
apt_has() {
  local candidate
  have apt-cache || return 1
  candidate="$(apt-cache policy "$1" 2>/dev/null | awk '/Candidate:/ {print $2; exit}')"
  [[ -n "$candidate" && "$candidate" != "(none)" ]]
}
deb_installed() {
  dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q "install ok installed"
}
apt_install() {  # ставит недостающее; удалять пакеты apt не разрешено (--no-remove)
  local pkg missing=()
  for pkg in "$@"; do deb_installed "$pkg" || missing+=("$pkg"); done
  [[ ${#missing[@]} -eq 0 ]] && return 0
  log "apt: ставлю ${missing[*]}"
  apt_get install --no-remove "${missing[@]}" && return 0
  # Типичные причины: прерванный dpkg, устаревшие пакеты системы (ROS 2 собран
  # под свежую Ubuntu). Чиним штатно и повторяем — без удаления пакетов.
  warn "apt не поставил пакеты с первого раза — доделываю dpkg, обновляю систему (apt upgrade) и повторяю"
  as_root dpkg --configure -a || true
  apt_update force
  apt_get upgrade || true
  apt_get install --no-remove "${missing[@]}"
}
apt_install_available() {  # то же, но пакеты, которых в apt нет, пропускаются
  local pkg present=()
  for pkg in "$@"; do
    if deb_installed "$pkg" || apt_has "$pkg"; then present+=("$pkg")
    else info "в apt нет $pkg — пропускаю"
    fi
  done
  [[ ${#present[@]} -eq 0 ]] && return 0
  apt_install "${present[@]}"
}

# --- репозиторий ROS 2 ------------------------------------------------------------ #
ensure_universe() {
  [[ "$OS_ID" == "ubuntu" ]] || return 0
  grep -rqsE '^[^#]*\buniverse\b' "$APT_DIR/sources.list" "$APT_DIR/sources.list.d" && return 0
  log "включаю репозиторий Ubuntu universe (часть зависимостей ROS 2 там)…"
  apt_install software-properties-common || true
  if have add-apt-repository; then
    as_root add-apt-repository -y universe || warn "не удалось включить universe"
    APT_UPDATED=0
  fi
}

# Ubuntu 24.04: без noble-updates в ubuntu.sources ros-dev-tools не ставится
# (конфликт зависимостей) — официальная инструкция ROS 2 велит добавить.
fix_noble_suites() {
  local file="$APT_DIR/sources.list.d/ubuntu.sources"
  [[ "$OS_CODENAME" == "noble" && -f "$file" ]] || return 0
  grep -qE '^Suites:[[:space:]]*noble[[:space:]]*$' "$file" || return 0
  log "ubuntu.sources: добавляю noble-updates и noble-backports (нужны для ros-dev-tools)"
  as_root sed -i -E 's/^Suites:[[:space:]]*noble[[:space:]]*$/Suites: noble noble-updates noble-backports/' "$file" \
    && APT_UPDATED=0
}

# Старое описание репозитория (ros2.list с ключом ros-archive-keyring) конфликтует
# с пакетом ros2-apt-source («Conflicting values set for option Signed-By») —
# удаляем, как велит официальная инструкция по переходу.
drop_legacy_ros_lists() {
  local file
  for file in "$APT_DIR"/sources.list.d/*.list; do
    [[ -f "$file" ]] || continue
    grep -qE '^[^#]*packages\.ros\.org/ros2/ubuntu' "$file" || continue
    log "убираю старое описание репозитория ROS 2: $file (его заменит ros2-apt-source)"
    as_root rm -f "$file"
  done
}

ros_apt_source_version() {
  local version
  version="$(curl -fsSL --max-time 20 \
      https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest 2>/dev/null \
    | sed -n 's/.*"tag_name"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p' | head -n 1)"
  printf '%s' "${version:-$ROS_APT_SOURCE_FALLBACK}"
}

install_ros_key_repo() {
  local key="/usr/share/keyrings/ros-archive-keyring.gpg" list="$APT_DIR/sources.list.d/ros2.list" arch
  arch="$(dpkg --print-architecture 2>/dev/null || echo amd64)"
  log "подключаю репозиторий ROS 2 ключом: $ROS_APT_URL"
  as_root curl -fsSL --retry 3 --max-time 60 \
    https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$key" || return 1
  echo "deb [arch=$arch signed-by=$key] $ROS_APT_URL $OS_CODENAME main" \
    | as_root tee "$list" >/dev/null || return 1
  APT_UPDATED=0
}

# Официальный способ (docs.ros.org): пакет ros2-apt-source с GitHub — он
# подключает репозиторий и сам обновляет ключ, когда тот истекает.
setup_ros_repo() {
  local version url deb
  deb_installed ros2-apt-source && [[ -z "${RC_ROS_APT_MIRROR:-}" ]] && return 0
  have curl || apt_install curl ca-certificates || return 1
  if [[ -n "${RC_ROS_APT_MIRROR:-}" ]]; then
    install_ros_key_repo
    return
  fi
  version="$(ros_apt_source_version)"
  url="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.${OS_CODENAME}_all.deb"
  deb="$(mktemp "${TMPDIR:-/tmp}/ros2-apt-source.XXXXXX")" || return 1
  log "подключаю репозиторий ROS 2: ros2-apt-source $version для $OS_CODENAME"
  if curl -fsSL --retry 3 --max-time 120 -o "$deb" "$url"; then
    drop_legacy_ros_lists
    if as_root dpkg -i "$deb"; then
      rm -f "$deb"; APT_UPDATED=0; return 0
    fi
  fi
  rm -f "$deb"
  warn "ros2-apt-source не скачался или не поставился — подключаю репозиторий по ключу (старый способ)"
  install_ros_key_repo
}

# --- 1. ROS 2 из apt ----------------------------------------------------------------- #
install_ros_apt() {
  if [[ $DO_SYSTEM -eq 0 ]]; then
    warn "ROS 2 не установлен, а установка пакетов выключена (--no-install)"
    return 1
  fi
  need_root "установка ROS 2 $DISTRO" || return 3
  log "ставлю ROS 2 $DISTRO для ${OS_NAME:-$OS_CODENAME} — всё скачается само (обычно 5–15 минут)…"
  set_state running ros-repo "подключаю репозиторий ROS 2"
  apt_update
  if ! apt_has "ros-$DISTRO-ros-base"; then
    apt_install_available ca-certificates curl gnupg || true
    ensure_universe
    fix_noble_suites
    setup_ros_repo || warn "репозиторий ROS 2 подключить не удалось"
    apt_update force
  fi
  if ! apt_has "ros-$DISTRO-ros-base"; then
    err "в apt нет пакета ros-$DISTRO-ros-base: репозиторий ROS 2 ($ROS_APT_URL) недоступен."
    err "проверь интернет и запусти ещё раз: ./scripts/ros2_setup.sh (журнал: $LOG_FILE)"
    return 1
  fi
  set_state running ros-base "скачиваю и ставлю ROS 2 $DISTRO (ros-base)"
  if ! apt_install "ros-$DISTRO-ros-base"; then
    err "ROS 2 $DISTRO не поставился — подробности выше и в $LOG_FILE"
    return 1
  fi
  log "ROS 2 $DISTRO поставлен"
  # Инструменты сборки (colcon, rosdep, vcstool) — как в официальной инструкции;
  # отдельной транзакцией: их сбой не отменяет уже поставленный ROS 2.
  set_state running tools "ставлю инструменты сборки (ros-dev-tools)"
  apt_install_available ros-dev-tools build-essential cmake git python3-serial \
    || warn "инструменты сборки поставились не все — попробую ещё раз при сборке"
}

# --- 1б. ROS 2 из исходников (нет готовых пакетов) ------------------------------------- #
pip_tools() {
  [[ -x "$TOOLS_VENV/bin/colcon" ]] && { PATH="$PATH:$TOOLS_VENV/bin"; return 0; }
  have python3 || return 1
  log "colcon из apt недоступен — ставлю colcon, vcstool и rosdep через pip ($TOOLS_VENV)"
  as_owner python3 -m venv --system-site-packages "$TOOLS_VENV" || return 1
  as_owner "$TOOLS_VENV/bin/pip" install -q -U colcon-common-extensions vcstool rosdep || return 1
  PATH="$PATH:$TOOLS_VENV/bin"
}

rosdep_ready() {  # rosdep init (root) и update — без них rosdep install не работает
  have rosdep || return 1
  if [[ ! -f "$ROSDEP_LIST" ]] && [[ "$ROOT_MODE" == "root" || "$ROOT_MODE" == "sudo" ]]; then
    as_root rosdep init >/dev/null 2>&1 || warn "rosdep init не сработал"
  fi
  rosdep update --rosdistro "$DISTRO" >/dev/null 2>&1 \
    || { warn "rosdep update не сработал (нужен доступ к raw.githubusercontent.com)"; return 1; }
}

core_fetch_sources() {
  local src="$1"
  [[ -f "$src/.zapret_sources_ok" ]] && return 0
  set_state running core-sources "скачиваю исходники ROS 2 $DISTRO с GitHub"
  as_owner mkdir -p "$src/src" || return 1
  if [[ ! -s "$src/ros2.repos" ]]; then
    if ! as_owner curl -fsSL --retry 3 --max-time 60 -o "$src/ros2.repos" \
         "https://raw.githubusercontent.com/ros2/ros2/$DISTRO/ros2.repos"; then
      rm -rf "$src/.ros2-meta"
      if ! as_owner git clone -q --depth 1 -b "$DISTRO" https://github.com/ros2/ros2.git "$src/.ros2-meta" \
         || ! as_owner cp "$src/.ros2-meta/ros2.repos" "$src/ros2.repos"; then
        err "не скачался список исходников ROS 2 (ros2.repos)"
        return 1
      fi
    fi
  fi
  log "скачиваю исходники ROS 2 $DISTRO (около сотни репозиториев с GitHub)…"
  if ! (cd "$src" && as_owner vcs import --shallow --retry 3 --input ros2.repos src); then
    err "не все исходники ROS 2 скачались — запусти ещё раз, скачанное докачается"
    return 1
  fi
  as_owner touch "$src/.zapret_sources_ok"
}

core_rosdep() {
  local src="$1" os_override paths=()
  rosdep_ready || return 1
  set_state running core-deps "ставлю зависимости ROS 2 (rosdep)"
  mapfile -t paths < <(cd "$src" && colcon list --base-paths src --packages-up-to "${CORE_PKGS[@]}" \
                         --packages-skip "${CORE_SKIP_PKGS[@]}" --paths-only 2>/dev/null | sed "s|^|$src/|")
  [[ ${#paths[@]} -gt 0 ]] || { warn "colcon не нашёл пакеты ROS 2 в $src/src"; return 1; }
  os_override="$(rosdep_os_for "$OS_CODENAME")"
  ROS_DISTRO="$DISTRO" ROS_VERSION=2 ROS_PYTHON_VERSION=3 \
    rosdep install --from-paths "${paths[@]}" --ignore-src -y -r --rosdistro "$DISTRO" \
      --skip-keys "$ROSDEP_SKIP_KEYS" ${os_override:+--os "$os_override"}
}

core_build() {
  local src="$1" target="$2" jobs workers rc
  read -r jobs workers <<< "$(build_parallelism)"
  set_state running core-build "компилирую ROS 2 $DISTRO из исходников (надолго)"
  log "компилирую ROS 2 $DISTRO из исходников в $target: от 30 минут до нескольких часов…"
  as_owner touch "$target/$INCOMPLETE_MARK" || return 1
  (
    cd "$src" || exit 1
    as_owner env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
      MAKEFLAGS="-j$jobs" ROS_DISTRO="$DISTRO" ROS_PYTHON_VERSION=3 \
      colcon build --merge-install --install-base "$target" --parallel-workers "$workers" \
        --packages-up-to "${CORE_PKGS[@]}" --packages-skip "${CORE_SKIP_PKGS[@]}" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  )
  rc=$?
  if [[ $rc -eq 0 && -f "$target/setup.bash" ]]; then
    rm -f "$target/$INCOMPLETE_MARK"
    as_owner touch "$target/$SOURCE_MARK" 2>/dev/null || true
    log "ROS 2 $DISTRO скомпилирован: $target"
    return 0
  fi
  err "сборка ROS 2 из исходников не удалась (код $rc) — подробности в $LOG_FILE;"
  err "повторный запуск продолжит с того же места: ./scripts/ros2_setup.sh"
  return 1
}

core_build_background() {
  local args=(--no-install --from-source --distro "$DISTRO")
  [[ $WRITE_ENV -eq 0 ]] && args+=(--no-env)
  log "ROS 2 $DISTRO компилируется в фоне (от 30 минут до нескольких часов); робот пока"
  log "работает, лидар читается по USB. Ход сборки: ./scripts/ros2_setup.sh --status"
  set_state background core-build "ROS 2 $DISTRO компилируется из исходников в фоне — лидар пока по USB"
  release_lock
  if have setsid; then
    RC_ROS_SETUP_LOGGED=1 setsid nohup bash "$SELF" "${args[@]}" </dev/null >>"$LOG_FILE" 2>&1 &
  else
    RC_ROS_SETUP_LOGGED=1 nohup bash "$SELF" "${args[@]}" </dev/null >>"$LOG_FILE" 2>&1 &
  fi
  disown 2>/dev/null || true
}

install_ros_source() {
  local src="${RC_ROS_SOURCE_DIR:-$VAR_DIR/ros2-src-$DISTRO}" target="$OPT_ROS/$DISTRO"
  log "для ${OS_NAME:-этой системы} ($ARCH) готовых пакетов ROS 2 нет — ROS 2 $DISTRO скачается"
  log "исходниками с GitHub и скомпилируется сам"
  if [[ $DO_SYSTEM -eq 1 ]]; then
    need_root "сборка ROS 2 $DISTRO из исходников: компилятор и зависимости" || return 3
    set_state running core-tools "ставлю инструменты для сборки ROS 2 из исходников"
    apt_update
    apt_install_available build-essential cmake git curl ca-certificates python3-dev \
      python3-pip python3-venv python3-setuptools python3-serial libboost-system-dev || true
    # Debian/Raspberry Pi OS: репозиторий ROS 2 есть (там ros-dev-tools), пакетов ROS — нет.
    if ! have colcon || ! have vcs || ! have rosdep; then
      setup_ros_repo >/dev/null 2>&1 && apt_update force && apt_install_available ros-dev-tools
    fi
    { have colcon && have vcs && have rosdep; } || pip_tools \
      || { err "нет colcon/vcstool/rosdep — собрать ROS 2 нечем"; return 1; }
    if ! as_root mkdir -p "$target" || ! as_root chown "$OWNER_UID" "$target"; then
      err "не удалось создать $target"
      return 1
    fi
  fi
  [[ -d "$target" && -w "$target" ]] || {
    err "нет каталога $target с правом записи: запусти ./scripts/ros2_setup.sh в терминале"
    return 1
  }
  { have colcon && have vcs; } || { err "нет colcon/vcstool — запусти ./scripts/ros2_setup.sh в терминале"; return 1; }
  core_fetch_sources "$src" || return 1
  if [[ $DO_SYSTEM -eq 1 ]]; then
    core_rosdep "$src" || warn "rosdep поставил не всё — сборка покажет, чего не хватает"
  fi
  if [[ $BACKGROUND_BUILD -eq 1 ]]; then
    ensure_drivers   # исходники драйверов скачиваем сейчас, пока есть терминал и сеть
    core_build_background
    return 4
  fi
  core_build "$src" "$target"
}

# Исходники ROS 2 скачаны и каталог установки готов — компилировать можно и
# без прав администратора (так работает фоновая сборка).
source_build_prepared() {
  [[ "$METHOD" == "source" && -d "$OPT_ROS/$DISTRO" && -w "$OPT_ROS/$DISTRO" \
     && -f "${RC_ROS_SOURCE_DIR:-$VAR_DIR/ros2-src-$DISTRO}/.zapret_sources_ok" ]]
}

explain_unsupported() {
  err "ROS 2 для «${OS_NAME:-этой системы}» ($ARCH) готовыми пакетами не выпускается,"
  err "а какой дистрибутив компилировать из исходников — неизвестно."
  cat <<EOF
Варианты:
  • Ubuntu 22.04 (ROS 2 Humble) или 24.04 (ROS 2 Jazzy) — там всё ставится само;
  • указать дистрибутив вручную и скомпилировать: ./scripts/ros2_setup.sh --distro jazzy --from-source
Пока ROS 2 нет, лидар читается напрямую по USB (RC_LIDAR_SOURCE=auto).
EOF
}

install_ros() {
  case "$METHOD" in
    apt)    install_ros_apt ;;
    source) install_ros_source ;;
    *)      explain_unsupported; return 1 ;;
  esac
}

# --- 2. инструменты сборки --------------------------------------------------------- #
ensure_tools() {
  local missing=()
  have colcon || missing+=(colcon)
  have git || missing+=(git)
  { have g++ && have cmake && have make; } || missing+=(компилятор)
  [[ ${#missing[@]} -eq 0 ]] && return 0
  if [[ $DO_SYSTEM -eq 1 ]] && need_root "инструменты сборки: ${missing[*]}"; then
    set_state running tools "ставлю инструменты сборки: ${missing[*]}"
    apt_update
    local pkgs=(build-essential cmake git python3-serial)
    if apt_has ros-dev-tools; then pkgs+=(ros-dev-tools)
    else pkgs+=(python3-colcon-common-extensions python3-rosdep python3-vcstool)
    fi
    apt_install_available "${pkgs[@]}" || warn "часть инструментов сборки не поставилась"
  fi
  have colcon || pip_tools || true
  have colcon || { warn "нет colcon — ros2_ws собрать нечем (поставится при запуске с паролем администратора)"; return 1; }
}

# --- 3. драйверы лидаров ------------------------------------------------------------- #
# ROS 2 без готовых пакетов: сейчас компилируется из исходников или уже собран так.
no_binaries() { [[ "$METHOD" == "source" ]] || [[ -n "${PREFIX:-}" && -f "$PREFIX/$SOURCE_MARK" ]]; }

model_driver() {  # драйвер для модели из RC_LIDAR_PROTOCOL (выбрана — нужна обязательно)
  local model
  model="$(read_setting RC_LIDAR_PROTOCOL | tr '[:upper:]' '[:lower:]')"
  case "$model" in
    sllidar*)                                 echo sllidar_ros2 ;;
    opencr|burger|waffle*|turtlebot3_node)    echo turtlebot3_node ;;
    ld06|ld-06|ld19|ld-19|stl27l|stl-27l|d500|ldlidar*) echo ldlidar_stl_ros2 ;;
    rplidar*)                                 echo rplidar_ros ;;
    lds*|hls*|lfcd*|turtlebot*)               echo hls_lfcd_lds_driver ;;
  esac
}
wanted_drivers() {
  local model
  model="$(model_driver)"
  # список пакетов через пробел — без кавычек намеренно
  # shellcheck disable=SC2086
  printf '%s\n' ${RC_ROS_DRIVERS:-hls_lfcd_lds_driver rplidar_ros ldlidar_stl_ros2 turtlebot3_node} \
    ${model:+"$model"} | awk 'NF && !seen[$0]++'
}
explicit_driver() { [[ " ${RC_ROS_DRIVERS:-} $(model_driver) " == *" $1 "* ]]; }
apt_name_for() {
  case "$1" in
    hls_lfcd_lds_driver|rplidar_ros|turtlebot3_node) printf 'ros-%s-%s' "$DISTRO" "${1//_/-}" ;;
  esac
}
# URL и ветки по очереди (первая подходящая); ветки — как в rosdistro.
source_spec_for() {
  case "$1" in
    hls_lfcd_lds_driver) echo "https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git $DISTRO rolling-devel main" ;;
    rplidar_ros)         echo "https://github.com/Slamtec/rplidar_ros.git ros2" ;;
    sllidar_ros2)        echo "https://github.com/Slamtec/sllidar_ros2.git main" ;;
    ldlidar_stl_ros2)    echo "https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git master" ;;
    turtlebot3_node)     echo "https://github.com/ROBOTIS-GIT/turtlebot3.git $DISTRO main" ;;
  esac
}
source_dir_for() { case "$1" in turtlebot3_node) echo turtlebot3 ;; *) echo "$1" ;; esac; }
# turtlebot3_node тянет за собой dynamixel_sdk, turtlebot3_msgs и др.: из
# исходников — только если модель «opencr» выбрана явно.
source_allowed() { [[ "$1" != "turtlebot3_node" ]] || explicit_driver turtlebot3_node; }

remember_unavailable() {
  mkdir -p "$VAR_DIR" 2>/dev/null && printf '%s %s\n' "$DISTRO" "$1" >> "$UNAVAILABLE_FILE"
}
known_unavailable() {
  [[ $DO_REBUILD -eq 0 ]] && ! explicit_driver "$1" \
    && grep -qxF "$DISTRO $1" "$UNAVAILABLE_FILE" 2>/dev/null
}

clone_driver() {
  local pkg="$1" spec url branches branch dir target
  spec="$(source_spec_for "$pkg")"
  [[ -n "$spec" ]] || return 1
  read -r url branches <<< "$spec"
  dir="$(source_dir_for "$pkg")"
  target="$WS/src/$dir"
  [[ -d "$target" ]] && return 0
  have git || { warn "нет git — $pkg из исходников не скачать"; return 1; }
  log "скачиваю исходники драйвера $pkg: $url"
  set_state running sources "скачиваю исходники драйвера $pkg"
  rm -rf "$target.tmp"
  for branch in $branches ""; do
    if [[ -n "$branch" ]]; then
      as_owner timeout 600 git clone -q --depth 1 --branch "$branch" "$url" "$target.tmp" >/dev/null 2>&1 && break
    else
      as_owner timeout 600 git clone -q --depth 1 "$url" "$target.tmp" && break
    fi
    rm -rf "$target.tmp"
  done
  [[ -d "$target.tmp" ]] || { warn "не удалось скачать $pkg (нет сети?) — попробую при следующем запуске"; return 1; }
  as_owner mv "$target.tmp" "$target" || return 1
  [[ "$pkg" == "turtlebot3_node" ]] && trim_turtlebot3 "$target"
  return 0
}

# Из репозитория turtlebot3 нужны только узел платы и её параметры: навигация,
# картография и телеуправление потянули бы Nav2 и Cartographer.
trim_turtlebot3() {
  local dir
  for dir in "$1"/*/; do
    case "$(basename "$dir")" in
      turtlebot3_node|turtlebot3_bringup|turtlebot3_description) ;;
      *) [[ -f "$dir/package.xml" ]] && as_owner touch "$dir/COLCON_IGNORE" ;;
    esac
  done
}

# Известные ошибки сборки драйверов, которые авторы не исправили: правим сами,
# иначе «скомпилировалось само» превращается в ошибку компилятора.
patch_sources() {
  local file="$WS/src/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp"
  # Ubuntu 24.04+ (GCC 13): pthread_mutex_* не объявлены — нет #include <pthread.h>
  # (ldrobotSensorTeam/ldlidar_stl_ros2#23, #29).
  if [[ -f "$file" ]] && ! grep -q '^#include <pthread.h>' "$file"; then
    log "ldlidar_stl_ros2: добавляю #include <pthread.h> (иначе не компилируется на Ubuntu 24.04+)"
    as_owner sed -i 's|^#include "log_module.h"|#include "log_module.h"\n#include <pthread.h>|' "$file"
  fi
}

ensure_drivers() {
  local pkg apt_name missing=() apt_list=() src_list=() failed=()
  while read -r pkg; do
    [[ -n "$pkg" ]] || continue
    pkg_installed "$pkg" && continue
    [[ -d "$WS/src/$(source_dir_for "$pkg")" ]] && continue   # исходники есть — соберутся
    known_unavailable "$pkg" && continue
    missing+=("$pkg")
  done < <(wanted_drivers)
  if [[ ${#missing[@]} -eq 0 ]]; then
    patch_sources
    return 0
  fi
  info "драйверы лидаров, которых не хватает: ${missing[*]}"

  # Свежие списки apt нужны, чтобы честно решить «пакет или исходники».
  if [[ $DO_SYSTEM -eq 1 ]] && ! no_binaries && ! apt_lists_fresh; then
    for pkg in "${missing[@]}"; do
      if [[ -n "$(apt_name_for "$pkg")" ]]; then
        need_root "пакеты драйверов лидаров" && apt_update
        break
      fi
    done
  fi
  for pkg in "${missing[@]}"; do
    apt_name="$(apt_name_for "$pkg")"
    if [[ -n "$apt_name" ]] && ! no_binaries && apt_has "$apt_name"; then
      if [[ $DO_SYSTEM -eq 1 ]]; then apt_list+=("$pkg")
      else info "драйвер $pkg поставится из apt при запуске с правами администратора"
      fi
    elif source_allowed "$pkg"; then
      src_list+=("$pkg")
    else
      info "драйвер $pkg: готового пакета для ROS 2 $DISTRO нет — пропускаю"
      info "  (собрать из исходников: RC_ROS_DRIVERS=\"$pkg\" ./scripts/ros2_setup.sh)"
      # Запоминаем, только если списки apt честные (обновлены) или пакетов ROS 2
      # нет вовсе: иначе устаревший кэш навсегда «спрятал» бы пакет.
      if no_binaries || { [[ $DO_SYSTEM -eq 1 ]] && apt_lists_fresh; }; then
        remember_unavailable "$pkg"
      fi
    fi
  done

  if [[ ${#apt_list[@]} -gt 0 ]]; then
    if need_root "пакеты драйверов лидаров"; then
      set_state running drivers "ставлю драйверы лидаров из apt: ${apt_list[*]}"
      local names=()
      for pkg in "${apt_list[@]}"; do names+=("$(apt_name_for "$pkg")"); done
      if ! apt_install "${names[@]}"; then
        # Пакетом не вышло — по одному, а что не ставится, соберём из исходников.
        for pkg in "${apt_list[@]}"; do
          apt_install "$(apt_name_for "$pkg")" || failed+=("$pkg")
        done
      fi
    else
      failed=("${apt_list[@]}")
    fi
    for pkg in "${failed[@]}"; do
      source_allowed "$pkg" && src_list+=("$pkg")
    done
  fi
  for pkg in "${src_list[@]}"; do
    clone_driver "$pkg" || true
  done
  patch_sources
}

# --- 4. зависимости исходников (rosdep) ------------------------------------------------ #
ws_rosdep() {
  local extra fingerprint os_override
  [[ $DO_SYSTEM -eq 1 ]] || return 0
  extra="$(ws_source_packages | grep -vx zapret_lidar || true)"
  [[ -n "$extra" ]] || return 0
  fingerprint="$(deps_fingerprint)"
  [[ "$(cat "$ROSDEP_STAMP" 2>/dev/null)" == "$fingerprint" ]] && return 0
  if no_binaries; then
    # ROS 2 собран из исходников: rosdep не знает его пакетов как apt — ставим
    # только системные библиотеки драйверов.
    need_root "библиотеки драйверов" && apt_install_available libboost-system-dev
    printf '%s\n' "$fingerprint" > "$ROSDEP_STAMP" 2>/dev/null
    return 0
  fi
  need_root "зависимости драйверов из исходников (rosdep)" || return 0
  set_state running rosdep "ставлю зависимости исходников (rosdep): ${extra//$'\n'/ }"
  log "rosdep: зависимости исходников ros2_ws (${extra//$'\n'/ })…"
  os_override="$(rosdep_os_for "$OS_CODENAME")"
  if rosdep_ready && ROS_DISTRO="$DISTRO" ROS_VERSION=2 ROS_PYTHON_VERSION=3 \
       rosdep install --from-paths "$WS/src" --ignore-src -y -r --rosdistro "$DISTRO" \
         --skip-keys "$ROSDEP_SKIP_KEYS" ${os_override:+--os "$os_override"}; then
    printf '%s\n' "$fingerprint" > "$ROSDEP_STAMP" 2>/dev/null
  else
    warn "rosdep поставил не всё — ставлю известные зависимости драйверов напрямую"
    apt_install_available libboost-system-dev || true
  fi
}

# --- 5. сборка ros2_ws --------------------------------------------------------------- #
build_parallelism() {  # «потоки_компилятора параллельных_пакетов» с учётом памяти
  local cpus mem_kb budget workers jobs
  cpus="$(nproc 2>/dev/null || echo 2)"
  mem_kb="$(awk '/MemAvailable/ {print $2; exit}' /proc/meminfo 2>/dev/null)"
  [[ "$mem_kb" =~ ^[0-9]+$ ]] || mem_kb=4000000
  # ~1.5 ГБ на поток компилятора C++: иначе на машинах с 2–4 ГБ сборка
  # rplidar_ros «вешает» систему (Slamtec/rplidar_ros#154).
  budget=$(( mem_kb / 1500000 ))
  (( budget < 1 )) && budget=1
  (( budget > cpus )) && budget=$cpus
  if [[ "${RC_ROS_BUILD_JOBS:-}" =~ ^[0-9]+$ ]] && (( RC_ROS_BUILD_JOBS > 0 )); then
    budget=$RC_ROS_BUILD_JOBS
  fi
  workers=1
  (( budget >= 4 )) && workers=2
  jobs=$(( budget / workers ))
  (( jobs < 1 )) && jobs=1
  printf '%s %s\n' "$jobs" "$workers"
}

BUILD_FAILED=()
build_workspace() {
  local fingerprint jobs workers rc pkg packages
  [[ -n "$PREFIX" ]] || return 1
  have colcon || { err "нет colcon — ros2_ws компилировать нечем"; return 1; }
  fingerprint="$(src_fingerprint)"
  if [[ $DO_REBUILD -eq 0 ]] && ws_ok && [[ "$(cat "$STAMP_FILE" 2>/dev/null)" == "$fingerprint" ]]; then
    info "ros2_ws собран и с тех пор не менялся — компилировать нечего"
    return 0
  fi
  if [[ $DO_REBUILD -eq 1 ]]; then
    log "пересобираю ros2_ws с нуля…"
    rm -rf "$WS/build" "$WS/install" "$WS/log"
    rm -f "$UNAVAILABLE_FILE"
  fi
  packages="$(ws_source_packages | paste -sd' ' -)"
  read -r jobs workers <<< "$(build_parallelism)"
  set_state running build "компилирую ros2_ws: $packages"
  log "компилирую ros2_ws (colcon build; потоков компилятора: $jobs, пакетов разом: $workers): $packages"
  # set +u: setup.bash от ROS 2 и colcon читают неопределённые переменные
  # ($COLCON_TRACE и др.) — под set -u source обрывал весь скрипт.
  # $1/$2 раскрывает вложенный bash
  # shellcheck disable=SC2016
  as_owner bash -c 'set +u; . "$1" >/dev/null || true; cd "$2" || exit 1; shift 2; exec "$@"' \
    ros2_build "$PREFIX/setup.bash" "$WS" \
    env MAKEFLAGS="-j$jobs" colcon build --symlink-install --continue-on-error \
      --parallel-workers "$workers" \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  rc=$?
  BUILD_FAILED=()
  for pkg in $packages; do
    pkg_installed "$pkg" || BUILD_FAILED+=("$pkg")
  done
  if ! ws_ok; then
    err "ros2_ws не собрался (colcon, код $rc): пакет zapret_lidar не установлен — см. $LOG_FILE"
    return 1
  fi
  # Отпечаток пишем и при частичной неудаче (драйвер, который не компилируется,
  # не должен компилироваться заново при каждом старте робота); повторить —
  # --rebuild или любая правка исходников.
  printf '%s\n' "$fingerprint" > "$STAMP_FILE" 2>/dev/null
  if [[ ${#BUILD_FAILED[@]} -gt 0 ]]; then
    warn "собрано, кроме: ${BUILD_FAILED[*]} (ошибки компиляции — в $LOG_FILE; повторить: ./scripts/ros2_setup.sh --rebuild)"
    return 2
  fi
  log "ros2_ws собран: $WS/install"
  return 0
}

# --- 6. .env и служба ----------------------------------------------------------------- #
env_file_value() { grep -E "^$1=" .env 2>/dev/null | tail -n 1 | cut -d= -f2- || true; }
touch_env() {
  local key="$1" value="$2" current
  current="$(env_file_value "$key")"
  if ! grep -qE "^${key}=" .env 2>/dev/null; then
    printf '%s=%s\n' "$key" "$value" >> .env
    log "в .env: ${key}=${value}"
  elif [[ "$current" != "$value" ]]; then
    sed -i "s|^${key}=.*|${key}=${value}|" .env
    log "в .env: ${key} было «${current}» → стало «${value}»"
  fi
}
any_driver_installed() {
  local pkg
  for pkg in hls_lfcd_lds_driver rplidar_ros sllidar_ros2 ldlidar_stl_ros2 turtlebot3_node; do
    pkg_installed "$pkg" && return 0
  done
  return 1
}
write_env() {
  [[ -f .env ]] || { info "нет .env — RC_LIDAR_SOURCE не меняю"; return 0; }
  if [[ "$RCLPY" != "да" ]]; then
    warn "rclpy не импортируется — RC_LIDAR_SOURCE в .env не меняю (auto: ROS 2, иначе USB)"
    return 0
  fi
  if ! any_driver_installed; then
    warn "в ROS 2 нет ни одного драйвера лидара — RC_LIDAR_SOURCE в .env не меняю"
    return 0
  fi
  if ! ws_ok; then
    warn "ros2_ws не собран — RC_LIDAR_SOURCE в .env не меняю (auto: ROS 2, иначе USB)"
    return 0
  fi
  local current
  current="$(env_file_value RC_LIDAR_SOURCE)"
  case "${current,,}" in
    ""|auto|ros|ros2)
      local topic
      topic="$(env_file_value RC_LIDAR_ROS_TOPIC)"
      touch_env RC_LIDAR_SOURCE ros
      touch_env RC_LIDAR_ROS_TOPIC "${topic:-/scan}"
      ;;
    *)
      info "в .env RC_LIDAR_SOURCE=$current — это выбор оператора, не трогаю"
      ;;
  esac
}

install_service() {
  have systemctl || { warn "systemd нет — службу не ставлю"; return 0; }
  [[ -f scripts/robot-lidar.service ]] || { warn "нет scripts/robot-lidar.service"; return 0; }
  need_root "служба robot-lidar" || { warn "службу robot-lidar без прав администратора не поставить"; return 0; }
  local user
  user="${SUDO_USER:-$(id -un "$OWNER_UID" 2>/dev/null || id -un)}"
  log "ставлю службу robot-lidar (лидар поднимается до приложений)…"
  # systemd запускает скрипт напрямую: без бита исполнения — 203/EXEC.
  chmod +x scripts/ros2_lidar.sh scripts/ros2_setup.sh 2>/dev/null || true
  as_root cp scripts/robot-lidar.service /etc/systemd/system/ || { warn "служба не скопировалась"; return 0; }
  as_root sed -i "s|/opt/robot-control|$ROOT|g; s|User=robotctl|User=$user|g; s|Group=robotctl|Group=$user|g" \
    /etc/systemd/system/robot-lidar.service
  as_root systemctl daemon-reload
  log "включить сейчас: sudo systemctl enable --now robot-lidar"
}

# --- итог -------------------------------------------------------------------------------- #
finish() {  # finish <код> <состояние> <сообщение>
  local code="$1" state="$2" message="$3"
  set_state "$state" "done" "$message"
  exit "$code"
}

main_setup() {
  local rc build_rc=0 drivers
  # Без права ставить пакеты и без ROS 2 делать нечего (кроме фоновой сборки
  # ROS 2 из исходников): выходим, не трогая журнал и состояние установки —
  # иначе, например, обновление из панели затирало бы «нужен пароль» на «ошибку».
  if [[ -z "$PREFIX" && $DO_SYSTEM -eq 0 ]] && ! source_build_prepared; then
    info "ROS 2 не установлен — с --no-install ставить нечего (поставит ./scripts/ros2_setup.sh)"
    exit 1
  fi
  start_log
  acquire_lock
  trap on_exit EXIT
  # Пока ждали замок, другой процесс мог всё поставить — смотрим заново.
  PREFIX="$(find_prefix || true)"
  resolve_distro
  choose_method
  set_state running start "проверяю ROS 2"
  info "система: ${OS_NAME:-неизвестна} (${OS_CODENAME:-?}, $ARCH)"

  if [[ -z "$PREFIX" ]]; then
    install_ros
    rc=$?
    [[ $rc -eq 4 ]] && exit 4
    PREFIX="$(find_prefix || true)"
    if [[ -z "$PREFIX" ]]; then
      if [[ $rc -eq 3 || ${#NEED_ROOT_FOR[@]} -gt 0 ]]; then
        warn "ROS 2 не поставлен: нужен пароль администратора, а спросить его негде (нет терминала)."
        warn "Запусти ярлык «Запустить робот» или ./scripts/ros2_setup.sh в терминале — пароль спросят один раз."
        finish 3 need_root "ROS 2 не установлен: нужен пароль администратора — запусти ярлык «Запустить робот» или ./scripts/ros2_setup.sh в терминале"
      fi
      finish 1 failed "ROS 2 не установлен — подробности в $LOG_FILE"
    fi
    DISTRO="$(prefix_distro "$PREFIX")"
    METHOD="present"
  fi
  info "ROS 2: $PREFIX (дистрибутив $DISTRO)"

  ensure_tools
  ensure_drivers
  ws_rosdep
  if [[ $DO_BUILD -eq 1 ]]; then
    build_workspace
    build_rc=$?
  fi

  RCLPY="нет"
  rclpy_ok "$PREFIX" && RCLPY="да"
  [[ "$RCLPY" == "да" ]] || warn "rclpy не импортируется интерпретатором $(python_bin) — версия Python .venv должна совпадать с ROS 2 $DISTRO"
  [[ $WRITE_ENV -eq 1 ]] && write_env
  [[ $DO_SERVICE -eq 1 ]] && install_service

  drivers="$(for p in hls_lfcd_lds_driver rplidar_ros sllidar_ros2 ldlidar_stl_ros2 turtlebot3_node; do
               pkg_installed "$p" && printf '%s ' "$p"; done)"
  if [[ $build_rc -eq 1 || ( $DO_BUILD -eq 1 && ! -f "$WS/install/setup.bash" ) ]]; then
    if [[ ${#NEED_ROOT_FOR[@]} -gt 0 ]]; then
      warn "ros2_ws не собран: для «${NEED_ROOT_FOR[*]}» нужен пароль администратора"
      finish 3 need_root "ros2_ws не собран: нужен пароль администратора (${NEED_ROOT_FOR[*]}) — запусти ярлык «Запустить робот»"
    fi
    finish 1 failed "ROS 2 $DISTRO есть, но ros2_ws не собрался — подробности в $LOG_FILE"
  fi
  if [[ $QUIET -eq 0 || $build_rc -ne 0 || ${#NEED_ROOT_FOR[@]} -gt 0 ]]; then
    echo
    log "ROS 2 $DISTRO: $PREFIX · rclpy: $RCLPY · ros2_ws: $(ws_ok && echo собран || echo не собран)"
    log "драйверы лидаров: ${drivers:-нет ни одного}"
    [[ $QUIET -eq 1 ]] || {
      echo "  лидар запускается сам вместе с роботом (ярлык «Запустить робот»)"
      echo "  вручную: ./scripts/ros2_lidar.sh · статус: ./scripts/ros2_lidar.sh --status"
    }
  fi
  if [[ ${#NEED_ROOT_FOR[@]} -gt 0 ]]; then
    warn "не сделано без пароля администратора: ${NEED_ROOT_FOR[*]} — запусти ярлык «Запустить робот» в терминале"
    finish 0 warn "часть шагов ждёт пароля администратора: ${NEED_ROOT_FOR[*]}"
  fi
  if [[ $build_rc -eq 2 ]]; then
    finish 0 warn "ROS 2 $DISTRO готов; не скомпилировались: ${BUILD_FAILED[*]}"
  fi
  [[ -n "$drivers" ]] || finish 0 warn "ROS 2 $DISTRO готов, но ни одного драйвера лидара не установлено"
  finish 0 ok "ROS 2 $DISTRO готов: $drivers"
}

main_setup
