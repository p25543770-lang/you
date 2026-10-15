#!/usr/bin/env bash
#
# Запуск лидара робота Zapret через ROS 2.
#
#   ./scripts/ros2_lidar.sh                    запустить (модель и порт — из .env)
#   ./scripts/ros2_lidar.sh --model lds01 --port /dev/ttyUSB0
#   ./scripts/ros2_lidar.sh --status           что видно: топик /scan и его частота
#   ./scripts/ros2_lidar.sh --print            показать команду, ничего не запуская
#   ./scripts/ros2_lidar.sh --keep-alive       перезапускать супервизор, если он упал
#   ./scripts/ros2_lidar.sh --build            пересобрать ros2_ws перед запуском
#
# Что запускается: супервизор лидара (пакет zapret_lidar) — он сам подбирает
# ROS 2-драйвер под подключённый лидар (rplidar_ros, hls_lfcd_lds_driver,
# ldlidar_stl_ros2, turtlebot3_node), следит за свежестью /scan и публикует
# состояние в /zapret/lidar/status. Веб-пульт и бортовой экран читают /scan.
#
# Готовить ничего не нужно: нет ROS 2 — скрипт сам вызовет ./scripts/ros2_setup.sh
# (докачает и скомпилирует), изменились исходники ros2_ws — пересоберёт.
# Ярлык «Запустить робот» запускает лидар сам; автозапуск при загрузке —
# ./scripts/ros2_setup.sh --service.
#
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1
ROOT="$PWD"

MODEL=""
PORT=""
BAUD=""
PREFIX_ARG=""
DO_STATUS=0
DO_PRINT=0
DO_BUILD=0
KEEP_ALIVE=0
NO_SETUP=0
ENGINE=""

log()  { printf '\033[1;32m[lidar]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[lidar]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[lidar]\033[0m %s\n' "$*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --model)      MODEL="${2:-}"; shift 2 ;;
    --port)       PORT="${2:-}"; shift 2 ;;
    --baud)       BAUD="${2:-}"; shift 2 ;;
    --prefix)     PREFIX_ARG="${2:-}"; shift 2 ;;
    --status)     DO_STATUS=1; shift ;;
    --print)      DO_PRINT=1; shift ;;
    --build)      DO_BUILD=1; shift ;;
    --keep-alive) KEEP_ALIVE=1; shift ;;
    --no-setup)   NO_SETUP=1; shift ;;          # не ставить и не собирать ничего самому
    --engine)     ENGINE="${2:-}"; shift 2 ;;   # venv | python | ros2
    -h|--help)    awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"; exit 0 ;;
    *) die "неизвестный аргумент: $1 (см. --help)" ;;
  esac
done

# Значение настройки: сначала переменная окружения (её же видит приложение и
# служба systemd через EnvironmentFile), потом файл .env — как в Config.from_env.
read_env() {
  local key="$1" fallback="${2:-}" value
  value="$(printenv "$key" 2>/dev/null || true)"
  if [[ -z "$value" ]]; then
    value="$(grep -E "^${key}=" .env 2>/dev/null | tail -n 1 | cut -d= -f2- || true)"
  fi
  printf '%s' "${value:-$fallback}"
}

# --- 1. что запускать: модель, порт, скорость -------------------------------- #
[[ -n "$MODEL" ]] || MODEL="$(read_env RC_LIDAR_PROTOCOL auto)"
[[ -n "$PORT"  ]] || PORT="$(read_env RC_LIDAR_PORT '')"
[[ -n "$BAUD"  ]] || BAUD="$(read_env RC_LIDAR_BAUD '')"
SCAN_TOPIC="$(read_env RC_LIDAR_ROS_TOPIC /scan)"
STATUS_TOPIC="$(read_env RC_LIDAR_ROS_STATUS_TOPIC /zapret/lidar/status)"

# --- 2. префикс ROS 2 и собранное рабочее пространство ----------------------- #
OPT_ROS="${RC_ROS_OPT_DIR:-/opt/ros}"
[[ -n "${RC_ROS_PREFIX:-}" ]] || RC_ROS_PREFIX="$(read_env RC_ROS_PREFIX '')"
prefix_ok() {
  [[ -n "$1" && -d "$1" ]] || return 1
  [[ -f "$1/setup.bash" || -f "$1/local_setup.bash" ]] || return 1
  [[ ! -e "$1/.zapret_build_incomplete" ]]   # ROS 2 из исходников ещё компилируется
}
find_prefix() {
  local name candidate
  for candidate in "$PREFIX_ARG" "${RC_ROS_PREFIX:-}"; do
    prefix_ok "$candidate" && { echo "$candidate"; return 0; }
  done
  for name in "${ROS_DISTRO:-}" lyrical kilted jazzy iron humble rolling galactic foxy; do
    [[ -n "$name" ]] || continue
    prefix_ok "$OPT_ROS/$name" && { echo "$OPT_ROS/$name"; return 0; }
  done
  for candidate in "$OPT_ROS"/*; do
    prefix_ok "$candidate" && { echo "$candidate"; return 0; }
  done
  return 1
}

PREFIX="$(find_prefix || true)"
# Собранное рабочее пространство: обычно внутри проекта, но путь можно задать
# (RC_ROS_WS_INSTALL) — например, если пакет собран в другом месте.
INSTALL="${RC_ROS_WS_INSTALL:-$ROOT/ros2_ws/install}"
SRC_PKG="$ROOT/ros2_ws/src/zapret_lidar"

have() { command -v "$1" >/dev/null 2>&1; }
interactive() { [[ -t 1 ]]; }

# --- 3. самоустановка: ROS 2 нет — поставить, исходники изменились — собрать ---- #
# Только для настоящего запуска: --print и --status ничего не меняют в системе.
if [[ $DO_PRINT -eq 0 && $DO_STATUS -eq 0 && $NO_SETUP -eq 0 \
      && "$(read_env RC_ROS_AUTO 1)" != "0" && -f scripts/ros2_setup.sh ]]; then
  if [[ -z "$PREFIX" ]]; then
    log "ROS 2 не найден — ставлю и собираю сам (./scripts/ros2_setup.sh)…"
    bash scripts/ros2_setup.sh --no-wait
    PREFIX="$(find_prefix || true)"
  elif [[ -z "${RC_ROS_WS_INSTALL:-}" ]]; then
    # Секунды, если менять нечего; без sudo — только сборка ros2_ws.
    setup_args=(--no-install --no-env --no-wait --quiet)
    [[ $DO_BUILD -eq 1 ]] && setup_args+=(--rebuild)
    bash scripts/ros2_setup.sh "${setup_args[@]}" \
      || warn "ros2_ws собрать не удалось — супервизор запустится прямо из исходников"
  fi
fi

WS_SETUP=""
[[ -f "$INSTALL/setup.bash" ]] && WS_SETUP="$INSTALL/setup.bash"

python_bin() {
  if [[ -x "$ROOT/.venv/bin/python" ]]; then echo "$ROOT/.venv/bin/python"; else echo "python3"; fi
}

python_site() {
  local base="$1" version dir
  version="$("$(python_bin)" -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null || echo 3.10)"
  for dir in "$base/lib/python$version/site-packages" "$base/share/zapret_lidar"; do
    [[ -d "$dir" ]] && printf '%s:' "$dir"
  done
}

# Супервизор запускаем из .venv проекта: там pyserial, которым супервизор быстро
# проверяет порты (автоподбор модели лидара), а rclpy приходит из ROS 2 через
# source setup.bash. Без .venv — системным python3 (rclpy у ROS 2 под него), и
# лишь в крайнем случае обычным ``ros2 launch`` (ему нужен собранный ros2_ws).
# Пакет zapret_lidar — чистый Python: если ros2_ws не собран, он берётся прямо
# из ros2_ws/src.
choose_engine() {
  if [[ -n "$ENGINE" ]]; then echo "$ENGINE"; return 0; fi
  if [[ -x .venv/bin/python ]]; then echo venv; return 0; fi
  if have python3; then echo python; return 0; fi
  if have ros2 && [[ -n "$WS_SETUP" ]]; then echo ros2; return 0; fi
  echo ""
}

ENGINE="$(choose_engine)"

ros_args=()
if [[ -n "$MODEL" && "$MODEL" != "auto" ]]; then ros_args+=(-p "model:=$MODEL"); else ros_args+=(-p "model:=auto"); fi
[[ -n "$PORT" ]] && ros_args+=(-p "port:=$PORT")
[[ -n "$BAUD" ]] && ros_args+=(-p "baud:=$BAUD")
ros_args+=(-p "scan_topic:=$SCAN_TOPIC" -p "status_topic:=$STATUS_TOPIC")

CMD=()
case "$ENGINE" in
  venv)   CMD=("$ROOT/.venv/bin/python" -m zapret_lidar.supervisor --ros-args "${ros_args[@]}") ;;
  python) CMD=(python3 -m zapret_lidar.supervisor --ros-args "${ros_args[@]}") ;;
  ros2)   CMD=(ros2 launch zapret_lidar lidar.launch.py "model:=$MODEL")
          [[ -n "$PORT" ]] && CMD+=("port:=$PORT")
          [[ -n "$BAUD" ]] && CMD+=("baud:=$BAUD")
          CMD+=("scan_topic:=$SCAN_TOPIC" "status_topic:=$STATUS_TOPIC") ;;
esac

if [[ $DO_PRINT -eq 1 ]]; then
  echo "ROS 2:      ${PREFIX:-не найден}"
  echo "рабочее пр-во: ${WS_SETUP:-не собрано (zapret_lidar берётся из ros2_ws/src)}"
  echo "движок:     ${ENGINE:-нет}"
  if [[ ${#CMD[@]} -gt 0 ]]; then
    echo "команда:    ${CMD[*]}"
  else
    echo "команда:    (нечего запускать: нет python3 и команды ros2)"
  fi
  exit 0
fi

if [[ -z "$PREFIX" ]]; then
  warn "ROS 2 не найден и сам не поставился (подробности: ./scripts/ros2_setup.sh --status)"
  # Служба systemd перезапускает скрипт через 5 с — без паузы это был бы
  # бесконечный поток попыток установки.
  interactive || sleep "${RC_ROS_RETRY_S:-60}"
  die "поставить вручную: ./scripts/ros2_setup.sh (в терминале — спросит пароль администратора)"
fi

# --- 4. окружение ROS 2 ----------------------------------------------------------- #
# set +u: setup.bash от ROS 2 и colcon читают неопределённые переменные
# ($COLCON_TRACE, $AMENT_TRACE_SETUP_FILES…): под set -u source обрывал скрипт
# молча, и служба robot-lidar падала, не успев ничего сказать.
set +u
# shellcheck disable=SC1090,SC1091
. "$PREFIX/setup.bash" >/dev/null || warn "не удалось выполнить source $PREFIX/setup.bash"
if [[ -n "$WS_SETUP" ]]; then
  # shellcheck disable=SC1090
  . "$WS_SETUP" >/dev/null || warn "не удалось выполнить source $WS_SETUP"
fi
set -u

# --- 5. режим проверки -------------------------------------------------------- #
if [[ $DO_STATUS -eq 1 ]]; then
  echo "ROS 2:       $PREFIX"
  echo "топик скана: $SCAN_TOPIC"
  if [[ -f var/lidar.pid ]] && kill -0 "$(cat var/lidar.pid 2>/dev/null)" 2>/dev/null; then
    echo "супервизор:  запущен вместе с роботом (pid $(cat var/lidar.pid), журнал var/lidar.log)"
  fi
  if ! have ros2; then
    warn "команда ros2 не найдена — проверка частоты недоступна"
    exit 1
  fi
  echo
  ros2 topic list 2>/dev/null | grep -E "scan|zapret" || warn "топиков /scan и /zapret/lidar/status пока нет"
  echo
  log "состояние супервизора:"
  timeout 5 ros2 topic echo --once "$STATUS_TOPIC" 2>/dev/null \
    || warn "супервизор не публикует состояние — он вообще запущен? (./scripts/ros2_lidar.sh)"
  echo
  log "частота скана (5 с):"
  timeout 6 ros2 topic hz --window 10 "$SCAN_TOPIC" 2>/dev/null \
    || warn "в $SCAN_TOPIC точек нет: лидар не отдаёт скан (см. состояние супервизора выше)"
  exit 0
fi

# --- 6. запуск --------------------------------------------------------------- #
[[ ${#CMD[@]} -gt 0 ]] || die "нечем запускать: нет python3 и команды ros2 → ./scripts/ros2_setup.sh"

if [[ "$ENGINE" == "venv" || "$ENGINE" == "python" ]]; then
  # PYTHONPATH: собранный пакет и python-пакеты ROS 2 (rclpy, sensor_msgs) уже
  # добавил source; дублируем их на случай нестандартного префикса, а исходники
  # zapret_lidar — последними (если ros2_ws не собран).
  EXTRA="$(python_site "$INSTALL/zapret_lidar")$(python_site "$PREFIX")"
  export PYTHONPATH="${EXTRA}${PYTHONPATH:+$PYTHONPATH:}$ROOT/ros2_ws/src/zapret_lidar"
  [[ -d "$SRC_PKG" ]] || warn "нет $SRC_PKG — супервизор найдётся, только если ros2_ws собран"
fi

log "запускаю супервизор лидара (модель: $MODEL, порт: ${PORT:-автопоиск}, движок: $ENGINE)"
if [[ $KEEP_ALIVE -eq 0 ]]; then
  exec "${CMD[@]}"
fi

# --keep-alive: супервизор упал — поднимаем снова (пауза растёт до минуты, если
# он падает сразу). pid цикла — в var/lidar.pid: по нему ./scripts/stop.sh и
# ярлык «Запустить робот» понимают, что лидар уже запущен, а обновление с GitHub
# шлёт SIGUSR1 — «перезапусти супервизор с новым кодом».
mkdir -p var
echo $$ > var/lidar.pid
CHILD=""
RESTART_NOW=0
stop_child() {
  if [[ -n "$CHILD" ]] && kill -0 "$CHILD" 2>/dev/null; then
    kill -TERM "$CHILD" 2>/dev/null
    for _ in $(seq 1 50); do kill -0 "$CHILD" 2>/dev/null || break; sleep 0.1; done
    kill -KILL "$CHILD" 2>/dev/null
  fi
  CHILD=""
}
cleanup() {
  stop_child
  [[ "$(cat var/lidar.pid 2>/dev/null)" == "$$" ]] && rm -f var/lidar.pid
}
trap 'cleanup; exit 0' TERM INT HUP
trap 'RESTART_NOW=1; stop_child' USR1
trap cleanup EXIT
delay=5
while :; do
  started=$SECONDS
  "${CMD[@]}" &
  CHILD=$!
  wait "$CHILD"
  status=$?
  # wait прерывается сигналом раньше, чем супервизор вышел: дожидаемся его.
  [[ -n "$CHILD" ]] && wait "$CHILD" 2>/dev/null
  CHILD=""
  if [[ $RESTART_NOW -eq 1 ]]; then
    RESTART_NOW=0
    delay=5
    log "перезапускаю супервизор лидара (обновление)"
    continue
  fi
  if (( SECONDS - started > 60 )); then delay=5; else delay=$(( delay < 60 ? delay * 2 : 60 )); fi
  warn "супервизор лидара завершился (код $status) — перезапуск через ${delay} с"
  sleep "$delay" &
  wait $! 2>/dev/null
done
