#!/usr/bin/env bash
#
# Установка и запуск пульта оператора на Ubuntu одной командой:
#
#   git clone https://github.com/p25543770-lang/you.git robot-control
#   cd robot-control
#   ./scripts/install_ubuntu.sh
#
# Скрипт сам ставит зависимости, ROS 2 с драйверами лидаров (и компилирует
# ros2_ws), создаёт .env и оператора, создаёт один ярлык «Запустить робот» и
# запускает веб-пульт + бортовой экран.
# В следующий раз — один двойной щелчок по этому ярлыку.
# Автозапуск при загрузке по умолчанию выключен.
#
# Флаги:
#   --no-run         установить, но не запускать приложение
#   --autostart      включить systemd для обоих серверов и запуск ярлыка после входа;
#                    точка доступа Wi-Fi — см. --ap-autostart
#   --systemd        то же, что --autostart (устаревшее имя флага)
#   --ap             поднять на этой машине точку доступа Wi-Fi (робот сам раздаёт сеть)
#   --ap-autostart   поднимать точку доступа и при каждой загрузке
#   --ros2           (по умолчанию) ROS 2 для лидара ставится сам: ROS 2, драйверы
#                    лидаров, rosdep и сборка ros2_ws; где готовых пакетов нет —
#                    ROS 2 компилируется из исходников в фоне. С --autostart ещё и
#                    служба robot-lidar
#   --no-ros2        не ставить ROS 2 (лидар читается напрямую по USB); запоминается
#                    в .env (RC_ROS_AUTO=0), чтобы ярлык тоже не ставил ROS 2
#
set -euo pipefail
cd "$(dirname "$0")/.."

RUN=1
AUTOSTART=0
AP=0
AP_AUTOSTART=0
ROS2="auto"                   # auto = ставить, если в .env не выключено (RC_ROS_AUTO=0)
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-run)       RUN=0; shift ;;
    --autostart)    AUTOSTART=1; shift ;;
    --systemd)      AUTOSTART=1; shift ;;                 # старое имя флага
    --ap)           AP=1; shift ;;
    --ap-autostart) AP=1; AP_AUTOSTART=1; shift ;;
    --ros2)         ROS2=1; shift ;;
    --no-ros2)      ROS2=0; shift ;;
    -h|--help)      awk 'NR==1{next} /^#/{print; next} {exit}' "$0"; exit 0 ;;
    *) echo "неизвестный аргумент: $1" >&2; exit 1 ;;
  esac
done

log() { printf '\033[1;32m[install]\033[0m %s\n' "$*"; }
SUDO=""
if [[ $EUID -ne 0 ]] && command -v sudo >/dev/null 2>&1; then SUDO=sudo; fi

# --- 0. версия Python ------------------------------------------------------- #
if ! python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3, 9) else 1)' 2>/dev/null; then
  echo "Нужен Python 3.9 или новее, а у вас $(python3 --version 2>&1)." >&2
  echo "На Ubuntu 22.04+ подходящий python3 уже стоит; на 20.04 обновите Python." >&2
  exit 1
fi
log "python3: $(python3 --version 2>&1) (нужен 3.9+)"

# --- 1-2. python и зависимости (работает офлайн через vendor/) -------------- #
./scripts/ensure_deps.sh

# USB Serial мастера доступен пользователю через группу dialout.
LOGIN_USER="${SUDO_USER:-${USER:-$(id -un)}}"
if getent group dialout >/dev/null 2>&1 && ! id -nG "$LOGIN_USER" 2>/dev/null | tr ' ' '\n' | grep -qx dialout; then
  if $SUDO usermod -aG dialout "$LOGIN_USER"; then
    log "$LOGIN_USER добавлен в dialout; выйдите из сеанса Ubuntu и войдите снова, чтобы группа вступила в силу"
  else
    log "не удалось добавить $LOGIN_USER в dialout; выполните: sudo usermod -aG dialout $LOGIN_USER"
  fi
fi

# --- 3. .env с секретным ключом ---------------------------------------------- #
if [[ ! -f .env ]]; then
  log "создаю .env с новым RC_SECRET_KEY…"
  {
    echo "RC_SECRET_KEY=$(python3 -c 'import secrets;print(secrets.token_urlsafe(48))')"
    echo "RC_BIND_HOST=0.0.0.0"
    echo "RC_PORT=8080"
    echo "RC_SLAM_SOURCE=master"
    echo "RC_SLAM_PORT=/dev/ttyACM0"
    echo "RC_UPDATE_REMOTE=origin"
    echo "RC_UPDATE_BRANCH=main"
  } > .env
  chmod 600 .env
else
  log ".env уже есть — не трогаю"
fi

# --- 4. учётка оператора ------------------------------------------------------ #
if [[ ! -f data/operators.json ]]; then
  if [[ -t 0 ]]; then
    log "создаю учётку оператора (логин по умолчанию admin)…"
    .venv/bin/python scripts/create_operator.py \
      --username "${RC_OPERATOR:-admin}" --min-length "${RC_MIN_PASSWORD_LENGTH:-4}"
  else
    log "терминала для ввода нет — создаю учётку автоматически…"
    ./scripts/create_default_operator.sh "${RC_OPERATOR:-admin}"
  fi
else
  log "учётки уже созданы — пропускаю"
fi

# --- 4b. проверка: что реально записалось ------------------------------------- #
# Fail fast: если учётка почему-то не создалась, сервер потом будет отвергать
# любой пароль как «неверный» — лучше упасть здесь с понятной причиной.
log "проверяю записанные учётки…"
.venv/bin/python scripts/create_operator.py --list

# --- 4c. лидар через ROS 2 ----------------------------------------------------- #
# Зачем: приложение само подбирать протокол лидара по USB больше не нужно —
# драйвер ROS 2 (hls_lfcd_lds_driver, rplidar_ros, ldlidar_stl_ros2,
# turtlebot3_node) публикует sensor_msgs/LaserScan в /scan, а пульт и экран
# читают топик. ROS 2 и драйверы ставятся сами (apt; чего в apt нет — клонируется
# и компилируется), ros2_ws собирается colcon-ом; приложения из .venv видят
# rclpy через окружение ROS 2 (robot_control/ros_env.py).
set_env_flag() {
  local key="$1" value="$2"
  [[ -f .env ]] || return 0
  if grep -qE "^${key}=" .env; then
    sed -i "s|^${key}=.*|${key}=${value}|" .env
  else
    printf '%s=%s\n' "$key" "$value" >> .env
  fi
}
if [[ "$ROS2" == "0" ]]; then
  set_env_flag RC_ROS_AUTO 0
  log "ROS 2 не ставлю (--no-ros2, запомнено в .env: RC_ROS_AUTO=0) — лидар читается по USB"
  log "  включить позже: ./scripts/install_ubuntu.sh --ros2 (или ./scripts/ros2_setup.sh)"
elif [[ "$ROS2" == "auto" ]] && grep -qE '^RC_ROS_AUTO=0' .env 2>/dev/null; then
  log "ROS 2 выключен в .env (RC_ROS_AUTO=0) — не ставлю; включить: ./scripts/install_ubuntu.sh --ros2"
else
  [[ "$ROS2" == "1" ]] && grep -qE '^RC_ROS_AUTO=' .env 2>/dev/null && set_env_flag RC_ROS_AUTO 1
  log "ROS 2 для лидара: ставлю и компилирую всё само (ROS 2, драйверы лидаров, ros2_ws)…"
  ROS2_ARGS=(--background-build)
  [[ $AUTOSTART -eq 1 ]] && ROS2_ARGS+=(--service)
  ROS2_STATUS=0
  bash scripts/ros2_setup.sh "${ROS2_ARGS[@]}" || ROS2_STATUS=$?
  case "$ROS2_STATUS" in
    0) if [[ $AUTOSTART -eq 1 ]]; then
         log "ROS 2 готов; служба robot-lidar поставлена (лидар поднимается до приложений)"
       else
         log "ROS 2 готов; лидар запускается сам вместе с роботом (ярлык «Запустить робот»)"
       fi ;;
    4) log "ROS 2 компилируется из исходников в фоне — лидар пока читается по USB"
       log "  ход сборки: ./scripts/ros2_setup.sh --status" ;;
    *) log "ROS 2 настроен не до конца (код $ROS2_STATUS) — лидар пока на USB (RC_LIDAR_SOURCE=auto)"
       log "  ярлык «Запустить робот» докачает и соберёт недостающее сам; журнал: var/ros2_setup.log" ;;
  esac
fi

# --- 5. порт в фаерволе -------------------------------------------------------- #
if command -v ufw >/dev/null 2>&1 && ufw status 2>/dev/null | grep -q "Status: active"; then
  if command -v sudo >/dev/null 2>&1 && [[ $EUID -ne 0 ]]; then SUDO=sudo; else SUDO=; fi
  $SUDO ufw allow 8080/tcp >/dev/null
  log "ufw: порт 8080 открыт"
fi

# --- 6. автозапуск (только по флагу) ----------------------------------------- #
# По умолчанию не ставим ничего: робот после включения не поднимает приложение
# само. Хочешь автозапуск — --autostart (или позже ./scripts/autostart.sh on).
if [[ $AUTOSTART -eq 1 ]]; then
  log "ставлю systemd-юнит и включаю автозапуск…"
  if command -v sudo >/dev/null 2>&1 && [[ $EUID -ne 0 ]]; then SUDO=sudo; else SUDO=; fi
  for unit in robot-control robot-kiosk; do
    $SUDO cp "scripts/$unit.service" /etc/systemd/system/
    $SUDO sed -i "s|/opt/robot-control|$PWD|g; s|User=robotctl|User=$LOGIN_USER|g; s|Group=robotctl|Group=$LOGIN_USER|g" \
      "/etc/systemd/system/$unit.service"
    if [[ "$unit" == "robot-control" ]]; then
      $SUDO sed -i "s|^ExecStart=.*|ExecStart=$PWD/.venv/bin/python $PWD/run.py|" \
        "/etc/systemd/system/$unit.service"
    else
      $SUDO sed -i "s|^ExecStart=.*|ExecStart=$PWD/.venv/bin/python $PWD/run_kiosk.py --no-open|" \
        "/etc/systemd/system/$unit.service"
    fi
  done
  $SUDO systemctl daemon-reload
  $SUDO systemctl enable --now robot-control robot-kiosk
  log "службы robot-control и robot-kiosk запущены (автозапуск включён)"
  # Лидар ROS 2 при автозапуске службами: ярлыка нет — супервизор поднимает
  # служба robot-lidar (её поставил ros2_setup.sh --service выше).
  if [[ -f /etc/systemd/system/robot-lidar.service ]]; then
    $SUDO systemctl enable --now robot-lidar || log "служба robot-lidar не запустилась — см. journalctl -u robot-lidar"
    log "служба robot-lidar включена: лидар ROS 2 поднимается при загрузке"
  fi
else
  log "автозапуск не ставлю: пульт и экран запускаются вручную"
  log "  включить: ./scripts/install_ubuntu.sh --autostart (или ./scripts/autostart.sh on)"
fi

# --- 6b. точка доступа Wi-Fi (опция) ---------------------------------------- #
if [[ $AP -eq 1 ]]; then
  if [[ $AP_AUTOSTART -eq 1 ]]; then
    log "поднимаю точку доступа Wi-Fi (и буду поднимать при загрузке)…"
    ./scripts/setup_ap.sh --autostart || log "точку доступа поднять не удалось — см. текст выше"
  else
    log "поднимаю точку доступа Wi-Fi (без автоподъёма при загрузке)…"
    ./scripts/setup_ap.sh || log "точку доступа поднять не удалось — см. текст выше"
  fi
fi

# --- 7. один ярлык для обоих экранов ----------------------------------------- #
if [[ $AUTOSTART -eq 1 ]]; then
  ./scripts/make_shortcut.sh --autostart || log "ярлык создать не удалось — не критично"
else
  ./scripts/make_shortcut.sh || log "ярлык создать не удалось — не критично"
fi

IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo
log "готово"
echo
echo "  Локально : http://127.0.0.1:8080/"
[[ -n "$IP" ]] && echo "  В сети   : http://${IP}:8080/   (с телефона/ноутбука в той же сети)"
echo
echo "  Подключение: логин/пароль оператора, которые вы задали на шаге 4."
if [[ $AP -eq 1 ]]; then
  echo "  Через Wi-Fi робота: подключитесь к сети из data/wifi.txt"
  echo "    и откройте http://10.42.0.1:8080/"
else
  echo "  Точка доступа Wi-Fi (если машина робота раздаёт сеть):"
  echo "    sudo ./scripts/setup_ap.sh            # поднять сейчас"
  echo "    sudo ./scripts/setup_ap.sh --autostart  # + поднимать при загрузке"
fi
echo
if [[ $AUTOSTART -eq 1 ]]; then
  echo "  Автозапуск серверов включён; выключить: ./scripts/autostart.sh off"
else
  echo "  Запуск обоих экранов: двойной щелчок по ярлыку «Запустить робот»."
  echo "  Из терминала: ./scripts/start_robot.sh"
  echo "  Остановить:   ./scripts/stop.sh"
fi

if [[ $RUN -eq 1 ]]; then
  echo
  log "запускаю веб-пульт и экран робота одним комплектом (Ctrl+C для остановки)…"
  RC_SKIP_DEPS=1 exec ./scripts/start_robot.sh
fi
