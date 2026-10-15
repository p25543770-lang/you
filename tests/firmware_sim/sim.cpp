// Хост-симуляция прошивок руля: мастер (FL) + три слейва (FR/RL/RR) на общей
// шине I2C, у каждого — модель TB6600 и оптодатчика нуля на D18.
//
// Сборка и запуск (это делает tests/test_firmware.py):
//   g++ -std=c++17 -I tests/firmware_sim/stubs tests/firmware_sim/sim.cpp -o sim
//   ./sim            — все сценарии (каждый в своём процессе, как после сброса)
//   ./sim moves      — один сценарий
//   ./sim tlm_dump   — вывести USB мастера после калибровки (для разбора сервером)
#include "arduino_sim.h"

#include <chrono>
#include <fcntl.h>
#include <functional>
#include <sys/wait.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

sim::Board board_fl, board_fr, board_rl, board_rr;

namespace master {
#define SIM_BOARD board_fl
#include "board_prelude.h"
#include "../../firmware/motor_master/motor_master.ino"
#undef SIM_BOARD
}  // namespace master
#undef MODULE_ID
#undef TX_BUFFER_SIZE

namespace node_fr {
#define SIM_BOARD board_fr
#define NODE_ADDRESS 2
#define MODULE_ID "FR"
#include "board_prelude.h"
#include "../../firmware/motor_node/motor_node.ino"
#undef SIM_BOARD
}  // namespace node_fr
#undef NODE_ADDRESS
#undef MODULE_ID
#undef TX_BUFFER_SIZE

namespace node_rl {
#define SIM_BOARD board_rl
#define NODE_ADDRESS 3
#define MODULE_ID "RL"
#include "board_prelude.h"
#include "../../firmware/motor_node/motor_node.ino"
#undef SIM_BOARD
}  // namespace node_rl
#undef NODE_ADDRESS
#undef MODULE_ID
#undef TX_BUFFER_SIZE

namespace node_rr {
#define SIM_BOARD board_rr
#define NODE_ADDRESS 4
#define MODULE_ID "RR"
#include "board_prelude.h"
#include "../../firmware/motor_node/motor_node.ino"
#undef SIM_BOARD
}  // namespace node_rr
#undef NODE_ADDRESS
#undef MODULE_ID
#undef TX_BUFFER_SIZE

// ------------------------------------------------------------------------
static int failures = 0;

#define CHECK(cond, ...)                                                   \
  do {                                                                     \
    if (!(cond)) {                                                         \
      ++failures;                                                          \
      std::fprintf(stderr, "  FAIL %s:%d: %s — ", __FILE__, __LINE__, #cond); \
      std::fprintf(stderr, __VA_ARGS__);                                   \
      std::fprintf(stderr, "\n");                                          \
    }                                                                      \
  } while (0)

static void wire_boards() {
  board_fl.name = "FL";
  board_fl.setup_fn = master::setup;
  board_fl.loop_fn = master::loop;
  board_fl.isr_fn = master::TIMER1_COMPA_vect_handler;
  board_fr.name = "FR";
  board_fr.setup_fn = node_fr::setup;
  board_fr.loop_fn = node_fr::loop;
  board_fr.isr_fn = node_fr::TIMER1_COMPA_vect_handler;
  board_rl.name = "RL";
  board_rl.setup_fn = node_rl::setup;
  board_rl.loop_fn = node_rl::loop;
  board_rl.isr_fn = node_rl::TIMER1_COMPA_vect_handler;
  board_rr.name = "RR";
  board_rr.setup_fn = node_rr::setup;
  board_rr.loop_fn = node_rr::loop;
  board_rr.isr_fn = node_rr::TIMER1_COMPA_vect_handler;
  for (sim::Board* board : {&board_fl, &board_fr, &board_rl, &board_rr}) board->wire.board = board;
}

static void boot(std::vector<sim::Board*> list) {
  sim::boards = list;
  // слейвы раньше мастера: шина готова к первому опросу
  for (auto it = list.rbegin(); it != list.rend(); ++it) {
    (*it)->reset_stats();
    (*it)->setup_fn();
  }
  sim::run_for_ms(5);
}

static bool wait_until(const std::function<bool()>& predicate, double timeout_ms) {
  const int64_t end = sim::now_us + (int64_t)(timeout_ms * 1000.0);
  while (sim::now_us < end) {
    if (predicate()) return true;
    sim::run_for_ms(1);
  }
  return predicate();
}

static bool has(const sim::Board& board, const char* text) {
  return board.serial.output.find(text) != std::string::npos;
}

static long count(const sim::Board& board, const char* text) {
  long n = 0;
  size_t pos = 0;
  while ((pos = board.serial.output.find(text, pos)) != std::string::npos) {
    ++n;
    pos += std::strlen(text);
  }
  return n;
}

static void check_signals(const sim::Board& board) {
  CHECK(board.dir_setup_violations == 0, "%s: DIR сменился позже чем за 5 мкс до PUL (%ld)",
        board.name, board.dir_setup_violations);
  CHECK(board.steps_while_disabled == 0, "%s: шаги при ENA != HIGH (%ld)", board.name,
        board.steps_while_disabled);
  CHECK(board.min_high_us >= 50, "%s: импульс PUL короче 50 мкс (%lld)", board.name,
        (long long)board.min_high_us);
}

// ---------------------------------------------------------------- один модуль
#define FR node_fr

static bool fr_idle() { return FR::calib_state == FR::CAL_IDLE && !FR::motorBusy(); }

static void calibrate_fr(double timeout_ms = 30000) {
  board_fr.serial.send("k");
  sim::run_for_ms(2);
  CHECK(FR::calib_state != FR::CAL_IDLE || FR::calib_error != FR::CAL_OK || FR::zero_set,
        "калибровка не началась");
  CHECK(wait_until(fr_idle, timeout_ms), "калибровка не закончилась за %.0f мс", timeout_ms);
}

static void scenario_boot() {
  boot({&board_fr});
  CHECK(board_fr.level[8] == HIGH, "ENA должен быть HIGH сразу после включения");
  CHECK(board_fr.level[6] == LOW, "PUL в покое LOW");
  CHECK(board_fr.mode[18] == INPUT_PULLUP, "D18 — INPUT_PULLUP");
  CHECK(!FR::zero_set, "ноль после включения не задан");
  CHECK(has(board_fr, "--- SLAVE FR, I2C 2"), "приветствие слейва: %s", board_fr.serial.output.c_str());
  CHECK(board_fr.level[5] == HIGH && board_fr.analog[3] == 0, "BLDC: тормоз включён, PWM 0");
}

static void scenario_calib_left() {
  board_fr.phys = -4000;                 // колесо на −30°, флажок в [−200, 200]
  boot({&board_fr});
  const int64_t started = sim::now_us;
  calibrate_fr();
  const double seconds = (sim::now_us - started) / 1e6;
  CHECK(FR::zero_set && FR::home_source == FR::HOME_OPTO && FR::calib_error == FR::CAL_OK,
        "ноль по датчику: zero=%d home=%d err=%d", FR::zero_set, FR::home_source, FR::calib_error);
  CHECK(std::labs(board_fr.phys) <= 1, "колесо в середине флажка, phys=%ld", board_fr.phys);
  CHECK(FR::stepNow() == 0, "счётчик в нуле: %ld", (long)FR::stepNow());
  CHECK(has(board_fr, "Калибровка ЗАВЕРШЕНА!"), "нет сообщения о завершении");
  CHECK(has(board_fr, "ширина флажка 3.00°"), "ширина флажка 400 шагов = 3.00°: %s",
        board_fr.serial.output.c_str());
  CHECK(seconds < 20.0, "калибровка шла %.1f с", seconds);
  CHECK(board_fr.min_period_us == 100, "рабочий шаг 100 мкс (10 кГц), а min период %lld",
        (long long)board_fr.min_period_us);
  CHECK(!has(board_fr, "ищу влево"), "датчик слева от колеса находится поиском вправо");
  check_signals(board_fr);
}

static void scenario_calib_right() {
  board_fr.phys = 3000;                  // флажок левее: вправо 100° пусто, потом влево
  boot({&board_fr});
  calibrate_fr();
  CHECK(has(board_fr, "справа датчика нет, ищу влево на 200°"), "переход к поиску влево");
  CHECK(has(board_fr, "датчик найден слева"), "датчик найден поиском влево");
  CHECK(FR::zero_set && std::labs(board_fr.phys) <= 1, "ноль найден: phys=%ld", board_fr.phys);
  CHECK(board_fr.phys_max >= 3000 + 13333 - 1, "сначала прошёл вправо 100°: max=%ld", board_fr.phys_max);
  check_signals(board_fr);
}

static void scenario_calib_inside() {
  board_fr.phys = 50;                    // старт внутри флажка
  boot({&board_fr});
  calibrate_fr();
  CHECK(has(board_fr, "датчик уже закрыт"), "старт с закрытым датчиком");
  CHECK(FR::zero_set && std::labs(board_fr.phys) <= 1, "ноль найден: phys=%ld", board_fr.phys);
  check_signals(board_fr);
}

static void scenario_calib_far_left() {
  board_fr.phys = -13000;                // −97.5°: ещё в зоне поиска вправо
  boot({&board_fr});
  calibrate_fr();
  CHECK(FR::zero_set && std::labs(board_fr.phys) <= 1, "ноль найден: phys=%ld", board_fr.phys);
}

static void scenario_calib_out_of_range() {
  board_fr.phys = -14000;                // −105°: за зоной поиска ±100°
  boot({&board_fr});
  calibrate_fr();
  CHECK(!FR::zero_set && FR::calib_error == FR::CAL_ERR_NOT_FOUND, "ошибка «не найден»: err=%d",
        FR::calib_error);
  CHECK(has(board_fr, "ОШИБКА калибровки: датчик не сработал"), "текст ошибки");
  CHECK(board_fr.phys == -14000 + 13333 - 26667, "вправо 100° и влево 200°: phys=%ld", board_fr.phys);
  CHECK(!FR::motorBusy(), "мотор остановлен");
}

static void scenario_calib_no_sensor() {
  board_fr.opto_present = false;
  boot({&board_fr});
  calibrate_fr();
  CHECK(FR::calib_error == FR::CAL_ERR_NOT_FOUND && !FR::zero_set, "без датчика — ошибка");
  CHECK(board_fr.phys_max == 13333 && board_fr.phys_min == -13334,
        "ход поиска +100°/−200°: min=%ld max=%ld", board_fr.phys_min, board_fr.phys_max);
  sim::run_for_ms(200);
  CHECK(board_fr.phys == -13334 && !FR::motorBusy(), "после ошибки колесо стоит");
}

static void scenario_calib_stuck() {
  board_fr.opto_stuck = true;
  boot({&board_fr});
  calibrate_fr();
  CHECK(FR::calib_error == FR::CAL_ERR_STUCK && !FR::zero_set, "залипший датчик: err=%d",
        FR::calib_error);
  CHECK(std::labs(board_fr.phys - 4000) <= 1, "выход из флажка ограничен 30°: phys=%ld", board_fr.phys);
}

static void scenario_calib_abort() {
  board_fr.phys = -8000;
  boot({&board_fr});
  board_fr.serial.send("k");
  sim::run_for_ms(30);
  board_fr.serial.send("s");
  sim::run_for_ms(5);
  const long stopped_at = board_fr.phys;
  CHECK(FR::calib_state == FR::CAL_IDLE && FR::calib_error == FR::CAL_ERR_ABORTED,
        "s отменяет калибровку");
  CHECK(has(board_fr, "прервана командой s") && has(board_fr, "ПАУЗА."), "сообщения об отмене");
  sim::run_for_ms(300);
  CHECK(board_fr.phys == stopped_at, "после s колесо не едет: %ld → %ld", stopped_at, board_fr.phys);
  board_fr.serial.send("g");
  sim::run_for_ms(50);
  CHECK(board_fr.phys == stopped_at, "g не продолжает отменённую калибровку");
  board_fr.serial.send("k");
  sim::run_for_ms(2);
  CHECK(FR::calib_state != FR::CAL_IDLE, "после отмены калибровку можно запустить снова");
  CHECK(wait_until(fr_idle, 30000) && FR::zero_set, "повторная калибровка прошла");
}

static void scenario_calib_asymmetric() {
  board_fr.flag_lo = 100;                // флажок [0.75°, 6.75°], середина 500 шагов
  board_fr.flag_hi = 900;
  board_fr.hysteresis = 30;
  board_fr.phys = 6000;
  boot({&board_fr});
  calibrate_fr();
  CHECK(FR::zero_set && std::labs(board_fr.phys - 500) <= 1,
        "ноль — середина флажка (гистерезис не сдвигает): phys=%ld", board_fr.phys);
}

static void scenario_moves() {
  board_fr.phys = -2500;
  boot({&board_fr});
  calibrate_fr();
  board_fr.reset_stats();

  board_fr.serial.send("w 12.5");
  CHECK(wait_until([] { return !FR::motorBusy() && FR::stepNow() == 1667; }, 1000),
        "w 12.5 → 1667 шагов, сейчас %ld", (long)FR::stepNow());
  CHECK(board_fr.phys == 1667, "колесо на 12.5°: phys=%ld", board_fr.phys);

  board_fr.serial.send("w 40");
  sim::run_for_ms(50);
  CHECK(board_fr.phys == 1667 && has(board_fr, "угол вне ±32°"), "w 40 отклонена");

  const int64_t started = sim::now_us;
  board_fr.serial.send("d 10 f");
  sim::run_for_ms(2);
  CHECK(wait_until([] { return !FR::motorBusy(); }, 1000), "d 10 f закончилась");
  const double ms = (sim::now_us - started) / 1000.0;
  CHECK(board_fr.phys == 3000, "d 10 f → +1333 шага: phys=%ld", board_fr.phys);
  CHECK(ms > 130.0 && ms < 140.0, "1333 шага по 100 мкс ≈ 133 мс, а заняло %.1f мс", ms);
  CHECK(has(board_fr, "ВПЕРЕД (F) на 10.00°") && has(board_fr, "Цель достигнута. Угол: 22.50°"),
        "сообщения как в проверенном скетче: %s", board_fr.serial.output.c_str());

  board_fr.serial.send("d 5 b");
  CHECK(wait_until([] { return !FR::motorBusy() && FR::stepNow() == 2333; }, 1000),
        "d 5 b → 2333, сейчас %ld", (long)FR::stepNow());
  board_fr.serial.send("d 10");
  board_fr.serial.send("d -5 f");
  board_fr.serial.send("d 5 x");
  sim::run_for_ms(50);
  CHECK(has(board_fr, "Шаблон: d [градусы] [f/b]"), "d без направления");
  CHECK(has(board_fr, "Ошибка: Угол > 0!"), "отрицательный угол, как в скетче");
  CHECK(has(board_fr, "Ошибка направления! f или b"), "неверное направление");
  CHECK(board_fr.phys == 2333, "ошибочные команды не двигают колесо");

  board_fr.serial.send("h");
  CHECK(wait_until([] { return !FR::motorBusy() && FR::stepNow() == 0; }, 1000), "h → 0");
  CHECK(board_fr.phys == 0, "колесо прямо: phys=%ld", board_fr.phys);
  board_fr.serial.send("p");
  sim::run_for_ms(5);
  CHECK(has(board_fr, "Текущий: 0.00° | Целевой: 0.00° [СТОИТ] · ноль по датчику"), "p: %s",
        board_fr.serial.output.c_str());
  check_signals(board_fr);
}

static void scenario_swing() {
  boot({&board_fr});
  board_fr.serial.send("z");
  sim::run_for_ms(2);
  board_fr.reset_stats();
  board_fr.serial.send("c 20");
  sim::run_for_ms(1500);
  CHECK(board_fr.phys_min == 0 && board_fr.phys_max == 2667, "качание 0…20°: %ld…%ld",
        board_fr.phys_min, board_fr.phys_max);
  CHECK(FR::cycle_active, "цикл идёт");
  board_fr.serial.send("c c");
  sim::run_for_ms(5);
  const long at = board_fr.phys;
  sim::run_for_ms(300);
  CHECK(!FR::cycle_active && board_fr.phys == at, "c c останавливает качание");
  CHECK(has(board_fr, "ЦИКЛ качания на 20.00°") && has(board_fr, "ЦИКЛ ВЫКЛ. Угол:"), "сообщения цикла");
  board_fr.serial.send("c c");
  sim::run_for_ms(5);
  CHECK(has(board_fr, "Цикл и так выключен."), "повторное c c");
}

static void scenario_pause_resume() {
  boot({&board_fr});
  board_fr.serial.send("z");
  board_fr.serial.send("d 90 f");        // 12000 шагов ≈ 1,2 с
  sim::run_for_ms(300);
  board_fr.serial.send("s");
  sim::run_for_ms(5);
  const long paused = board_fr.phys;
  CHECK(paused > 2000 && paused < 4000, "стоп на ходу: phys=%ld", paused);
  CHECK(FR::stepNow() == paused, "счётчик совпадает с колесом после s: %ld vs %ld",
        (long)FR::stepNow(), paused);
  board_fr.serial.send("g");
  CHECK(wait_until([] { return !FR::motorBusy() && FR::stepNow() == 12000; }, 2000),
        "g довёл до цели, сейчас %ld", (long)FR::stepNow());
  CHECK(board_fr.phys == 12000 && has(board_fr, "ПРОДОЛЖЕНИЕ..."), "phys=%ld", board_fr.phys);
  board_fr.serial.send("g");
  sim::run_for_ms(5);
  CHECK(has(board_fr, "Уже в цели."), "g без прерванного хода");
}

static void scenario_zero_manual() {
  board_fr.phys = 700;
  boot({&board_fr});
  board_fr.serial.send("w 5");
  sim::run_for_ms(20);
  CHECK(has(board_fr, "w: ноль не задан"), "w без нуля отклонена");
  CHECK(board_fr.phys == 700, "без нуля абсолютная команда не двигает колесо");
  board_fr.serial.send("z");
  sim::run_for_ms(2);
  CHECK(FR::zero_set && FR::home_source == FR::HOME_MANUAL && FR::stepNow() == 0, "z — ручной ноль");
  CHECK((FR::moduleStatus() & FR::STATUS_HOME_OPTO) == 0, "ручной ноль не помечен как датчик");
  board_fr.serial.send("w -5");
  CHECK(wait_until([] { return !FR::motorBusy() && FR::stepNow() == -667; }, 1000), "w -5 после z");
  CHECK(board_fr.phys == 700 - 667, "phys=%ld", board_fr.phys);
}

static void scenario_drive() {
  boot({&board_fr});
  board_fr.serial.send("t 8");
  sim::run_for_ms(5);
  CHECK(board_fr.analog[3] == 0 && board_fr.level[5] == HIGH, "без нуля тяга запрещена");
  board_fr.serial.send("z");
  board_fr.serial.send("t 8");
  sim::run_for_ms(5);
  CHECK(board_fr.analog[3] == 20 && board_fr.level[5] == LOW, "t 8 → PWM 20/255, тормоз снят");
  board_fr.serial.send("d 5 f");
  sim::run_for_ms(20);
  CHECK(board_fr.phys == 0 && has(board_fr, "руль заблокирован: включена тяга"), "при тяге руль не крутится");
  for (int i = 0; i < 5; ++i) {
    board_fr.serial.send("t 8");
    sim::run_for_ms(200);
  }
  CHECK(FR::drive_active && board_fr.analog[3] == 20, "heartbeat каждые 200 мс держит тягу");
  sim::run_for_ms(700);
  CHECK(!FR::drive_active && board_fr.analog[3] == 0 && board_fr.level[5] == HIGH,
        "watchdog 650 мс: PWM 0, тормоз");
  CHECK(has(board_fr, "нет команды дольше 650 мс"), "сообщение watchdog");
  board_fr.serial.send("t 8");
  board_fr.serial.send("t -8");
  sim::run_for_ms(5);
  CHECK(board_fr.analog[3] == 0 && board_fr.level[5] == HIGH, "реверс без остановки — тормоз");
  board_fr.serial.send("t 25");
  sim::run_for_ms(5);
  CHECK(board_fr.analog[3] == 0, "t 25 — за пределом, тормоз");
  board_fr.serial.send("t 8");
  sim::run_for_ms(5);
  board_fr.serial.send("s");
  sim::run_for_ms(5);
  CHECK(board_fr.analog[3] == 0 && board_fr.level[5] == HIGH, "s выключает и тягу");
}

static void scenario_serial_garbage() {
  boot({&board_fl, &board_fr, &board_rl, &board_rr});
  for (const char* chunk : {"@TLM mod=RR deg=0.00 tgt=0.00 mov", "\x07\xff#"}) {
    for (const char* p = chunk; *p; ++p) board_fl.serial.input.push_back((uint8_t)*p);
    sim::run_for_ms(1200);               // обрывок без \n висит дольше секунды
  }
  board_fl.serial.send("k");
  sim::run_for_ms(5);
  CHECK(master::calib_state != master::CAL_IDLE, "после мусора в порту команда k всё равно выполнена");
  board_fl.serial.send("@TLM mod=FL deg=0.00 tgt=0.00 moving=0 cal=0");
  board_fl.serial.send("# FL: эхо сообщения");
  sim::run_for_ms(5);
  CHECK(!has(board_fl, "Неизв. команда: @TLM") && !has(board_fl, "Неизв. команда: #"),
        "эхо телеметрии и сообщений не считается командой");
  board_fl.serial.send("d 1");            // частичный ввод человека с паузами < 1 с
  sim::run_for_ms(5);
  CHECK(has(board_fl, "Шаблон: d [градусы] [f/b]"), "обычные ошибки по-прежнему видны");
}

static void scenario_frame_limits() {
  boot({&board_fr});
  FR::current_step = -123456;
  FR::target_step = 123456;
  FR::rpm_x10 = 4000000000UL;
  FR::refreshFrame();
  const uint8_t index = FR::frame_front;
  const std::string frame(FR::frame[index], FR::frame_length[index]);
  CHECK(frame.size() <= 32, "кадр I2C не длиннее 32 байт: %zu «%s»", frame.size(), frame.c_str());
  CHECK(frame.rfind("-99999,99999,", 0) == 0, "шаги ограничены ±99999: %s", frame.c_str());
  CHECK(frame.find(",99999,0\n") != std::string::npos, "об/мин ограничены: %s", frame.c_str());
}

// ---------------------------------------------------------------- вся система
#define M master
static std::vector<sim::Board*> all_boards() { return {&board_fl, &board_fr, &board_rl, &board_rr}; }

static bool all_idle() {
  return M::calib_state == M::CAL_IDLE && !M::motorBusy() &&
         node_fr::calib_state == node_fr::CAL_IDLE && !node_fr::motorBusy() &&
         node_rl::calib_state == node_rl::CAL_IDLE && !node_rl::motorBusy() &&
         node_rr::calib_state == node_rr::CAL_IDLE && !node_rr::motorBusy();
}

static bool all_zero() { return M::zero_set && node_fr::zero_set && node_rl::zero_set && node_rr::zero_set; }

static void calibrate_all() {
  board_fl.phys = -3000;
  board_fr.phys = 2500;                  // справа от флажка: поиск влево
  board_rl.phys = 40;                    // внутри флажка
  board_rr.phys = -9000;
  board_rr.flag_lo = 300;                // у RR флажок смещён: ноль будет на 500 шагах
  board_rr.flag_hi = 700;
  boot(all_boards());
  sim::run_for_ms(600);
  board_fl.serial.send("k");
  sim::run_for_ms(5);
  CHECK(wait_until(all_idle, 30000), "калибровка всех колёс закончилась");
  sim::run_for_ms(600);                  // мастер опросит слейвов
}

static void scenario_system_calibrate_all() {
  calibrate_all();
  CHECK(all_zero(), "ноль задан у всех: FL=%d FR=%d RL=%d RR=%d", M::zero_set, node_fr::zero_set,
        node_rl::zero_set, node_rr::zero_set);
  CHECK(std::labs(board_fl.phys) <= 1 && std::labs(board_fr.phys) <= 1 &&
        std::labs(board_rl.phys) <= 1 && std::labs(board_rr.phys - 500) <= 1,
        "колёса на нулях датчиков: %ld %ld %ld %ld", board_fl.phys, board_fr.phys, board_rl.phys,
        board_rr.phys);
  const std::string& out = board_fl.serial.output;
  for (const char* module : {"FL", "FR", "RL", "RR"}) {
    const std::string tag = std::string("@TLM mod=") + module + " deg=0.00 tgt=0.00 moving=0 cal=1 cycle=0 calib=0";
    CHECK(out.find(tag) != std::string::npos, "телеметрия %s после калибровки", module);
  }
  CHECK(has(board_fl, "home=opto err=0") && has(board_fl, "fw=2"), "поля home/err/fw в @TLM");
  CHECK(has(board_fl, "# FR: Калибровка ЗАВЕРШЕНА!") && has(board_fl, "# RR: Калибровка ЗАВЕРШЕНА!"),
        "мастер сообщает о калибровке слейвов");
  CHECK(has(board_fl, "# FL: Калибровка ЗАВЕРШЕНА!"), "и о своей");
  for (sim::Board* board : all_boards()) check_signals(*board);
}

static void scenario_system_server_commands() {
  calibrate_all();
  board_fl.serial.send("w FR 12.5");
  sim::run_for_ms(400);
  CHECK(board_fr.phys == 1667 && std::labs(board_fl.phys) <= 1 && std::labs(board_rl.phys) <= 1,
        "w FR 12.5 крутит только FR: FR=%ld FL=%ld RL=%ld", board_fr.phys, board_fl.phys, board_rl.phys);
  sim::run_for_ms(300);
  CHECK(has(board_fl, "@TLM mod=FR deg=12.50 tgt=12.50 moving=0 cal=1"), "телеметрия FR на 12.5°");
  board_fl.serial.send("w FL -7");
  sim::run_for_ms(300);
  CHECK(M::stepNow() == -933, "w FL -7 — локальный модуль: %ld", (long)M::stepNow());
  board_fl.serial.send("w RL zero");
  sim::run_for_ms(50);
  CHECK(node_rl::home_source == node_rl::HOME_MANUAL, "w RL zero — ручной ноль RL");
  board_fl.serial.send("w RR calib");
  sim::run_for_ms(5);
  CHECK(node_rr::calib_state != node_rr::CAL_IDLE, "w RR calib запускает калибровку RR");
  CHECK(wait_until([] { return node_rr::calib_state == node_rr::CAL_IDLE; }, 30000) &&
        node_rr::home_source == node_rr::HOME_OPTO, "RR откалиброван заново");
  board_fl.serial.send("w XX 5");
  board_fl.serial.send("w FL 40");
  sim::run_for_ms(20);
  CHECK(has(board_fl, "неизвестный модуль XX"), "неизвестный модуль");
  CHECK(has(board_fl, "# FL: w: угол вне ±32°"), "предел угла у сервера");
}

static void send_drive_heartbeat(int percent, int times) {
  char command[16];
  std::snprintf(command, sizeof(command), "t %d", percent);
  for (int i = 0; i < times; ++i) {
    board_fl.serial.send(command);
    sim::run_for_ms(200);
  }
}

static bool all_pwm(int value) {
  return board_fl.analog[3] == value && board_fr.analog[3] == value && board_rl.analog[3] == value &&
         board_rr.analog[3] == value;
}

static void scenario_system_drive() {
  calibrate_all();
  board_fl.serial.send("w RR 3.75");    // RR выравниваем по остальным (его ноль смещён флажком)
  sim::run_for_ms(300);
  board_fl.serial.send("w RR 0");
  sim::run_for_ms(300);
  send_drive_heartbeat(8, 3);
  CHECK(all_pwm(20), "t 8 — все четыре BLDC: %d %d %d %d", board_fl.analog[3], board_fr.analog[3],
        board_rl.analog[3], board_rr.analog[3]);
  board_fl.serial.send("b");
  sim::run_for_ms(10);
  CHECK(all_pwm(0) && board_fl.level[5] == HIGH && board_rr.level[5] == HIGH, "b — тормоз у всех");
  board_fl.serial.send("w FR 10");
  sim::run_for_ms(600);
  send_drive_heartbeat(8, 2);
  CHECK(all_pwm(0), "колёса не выровнены — тяга запрещена");
  board_fl.serial.send("w FR 0");
  sim::run_for_ms(600);
  send_drive_heartbeat(8, 1);
  CHECK(all_pwm(20), "выровняли — тяга снова разрешена");
  sim::run_for_ms(800);
  CHECK(all_pwm(0), "без heartbeat все останавливаются по watchdog");
}

static void scenario_system_slave_offline() {
  calibrate_all();
  board_rr.wire.connected = false;
  sim::run_for_ms(1500);
  CHECK(has(board_fl, "# RR: нет связи по I2C (адрес 4)"), "мастер сообщает о потере RR");
  const size_t mark = board_fl.serial.output.size();
  sim::run_for_ms(600);
  const std::string tail = board_fl.serial.output.substr(mark);
  CHECK(tail.find("@TLM mod=RR") == std::string::npos, "строк RR больше нет: сервер видит «нет связи»");
  CHECK(tail.find("@TLM mod=FR") != std::string::npos, "остальные модули на связи");
  send_drive_heartbeat(8, 2);
  CHECK(all_pwm(0), "без RR тяга запрещена");
  board_fl.serial.send("d 10 f");
  sim::run_for_ms(20);
  CHECK(has(board_fl, "# RR: команда «d 10 f» не доставлена по I2C"), "недоставленная команда видна");
  board_rr.wire.connected = true;
  sim::run_for_ms(600);
  CHECK(count(board_fl, "# RR: на связи по I2C") >= 2, "RR вернулся на связь");
}

static void scenario_system_broadcast() {
  calibrate_all();
  board_fl.serial.send("d 10 f");
  sim::run_for_ms(5);
  CHECK(wait_until(all_idle, 2000), "d 10 f у всех закончилась");
  CHECK(M::stepNow() == 1333 && node_fr::stepNow() == 1333 && node_rl::stepNow() == 1333 &&
        node_rr::stepNow() == 1333, "одна команда — все четыре колеса +10°");
  board_fl.serial.send("d 90 b");
  sim::run_for_ms(200);
  board_fl.serial.send("s");
  sim::run_for_ms(5);
  const long fl = board_fl.phys;
  const long rr = board_rr.phys;
  sim::run_for_ms(300);
  CHECK(board_fl.phys == fl && board_rr.phys == rr, "s останавливает все колёса");
  board_fl.serial.send("h");
  sim::run_for_ms(5);
  CHECK(wait_until(all_idle, 3000), "h у всех закончилась");
  CHECK(M::stepNow() == 0 && node_fr::stepNow() == 0 && node_rl::stepNow() == 0 && node_rr::stepNow() == 0,
        "h — все в нуле");
  sim::run_for_ms(300);                  // мастер опросит слейвов
  board_fl.serial.send("p");
  sim::run_for_ms(5);
  CHECK(has(board_fl, "# RL: Текущий: 0.00° | Целевой: 0.00° [СТОИТ]"), "p показывает слейвов");
}

static void scenario_tlm_dump() {
  calibrate_all();
  board_fl.serial.send("w FR 12.5");
  sim::run_for_ms(700);
  std::fwrite(board_fl.serial.output.data(), 1, board_fl.serial.output.size(), stdout);
}

// ./sim pty /tmp/master-link — мастер и три слейва в реальном времени за
// псевдотерминалом: сервер робота (RC_SLAM_PORT=/tmp/master-link) видит настоящую
// логику прошивки — для ручной проверки экрана без железа.
static int run_pty(const char* link_path) {
  const int fd = posix_openpt(O_RDWR | O_NOCTTY);
  if (fd < 0 || grantpt(fd) != 0 || unlockpt(fd) != 0) {
    std::perror("posix_openpt");
    return 2;
  }
  // Сторона робота сразу в raw-режиме: без эха телеметрии обратно в «мастер».
  const int slave = open(ptsname(fd), O_RDWR | O_NOCTTY);
  if (slave >= 0) {
    termios mode;
    if (tcgetattr(slave, &mode) == 0) {
      cfmakeraw(&mode);
      tcsetattr(slave, TCSANOW, &mode);
    }
    close(slave);
  }
  unlink(link_path);
  if (symlink(ptsname(fd), link_path) != 0) {
    std::perror("symlink");
    return 2;
  }
  fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
  wire_boards();
  board_fl.phys = -3000;
  board_fr.phys = 2500;
  board_rl.phys = 40;
  board_rr.phys = -9000;
  boot(all_boards());
  std::printf("мастер на %s → %s\n", link_path, ptsname(fd));
  std::fflush(stdout);
  size_t sent = 0;
  const auto wall_start = std::chrono::steady_clock::now();
  const int64_t sim_start = sim::now_us;
  for (;;) {
    char buffer[256];
    const ssize_t got = read(fd, buffer, sizeof(buffer));
    for (ssize_t i = 0; i < got; ++i) board_fl.serial.input.push_back((uint8_t)buffer[i]);
    sim::run_for_ms(10);
    const std::string& out = board_fl.serial.output;
    if (out.size() > sent) {
      const ssize_t written = write(fd, out.data() + sent, out.size() - sent);
      if (written > 0) sent += (size_t)written;
    }
    std::this_thread::sleep_until(wall_start + std::chrono::microseconds(sim::now_us - sim_start));
  }
}

struct Scenario {
  const char* name;
  void (*fn)();
};

static const Scenario SCENARIOS[] = {
    {"boot", scenario_boot},
    {"calib_left", scenario_calib_left},
    {"calib_right", scenario_calib_right},
    {"calib_inside", scenario_calib_inside},
    {"calib_far_left", scenario_calib_far_left},
    {"calib_out_of_range", scenario_calib_out_of_range},
    {"calib_no_sensor", scenario_calib_no_sensor},
    {"calib_stuck", scenario_calib_stuck},
    {"calib_abort", scenario_calib_abort},
    {"calib_asymmetric", scenario_calib_asymmetric},
    {"moves", scenario_moves},
    {"swing", scenario_swing},
    {"pause_resume", scenario_pause_resume},
    {"zero_manual", scenario_zero_manual},
    {"drive", scenario_drive},
    {"frame_limits", scenario_frame_limits},
    {"serial_garbage", scenario_serial_garbage},
    {"system_calibrate_all", scenario_system_calibrate_all},
    {"system_server_commands", scenario_system_server_commands},
    {"system_drive", scenario_system_drive},
    {"system_slave_offline", scenario_system_slave_offline},
    {"system_broadcast", scenario_system_broadcast},
};

static int run_one(const Scenario& scenario) {
  wire_boards();
  scenario.fn();
  if (failures) {
    std::fprintf(stderr, "%s: %d ошибок\n", scenario.name, failures);
    for (sim::Board* board : sim::boards) {
      std::fprintf(stderr, "--- вывод %s ---\n%s\n", board->name, board->serial.output.c_str());
    }
  }
  return failures ? 1 : 0;
}

int main(int argc, char** argv) {
  if (argc > 2 && std::strcmp(argv[1], "pty") == 0) return run_pty(argv[2]);
  if (argc > 1 && std::strcmp(argv[1], "tlm_dump") == 0) {
    wire_boards();
    scenario_tlm_dump();
    return failures ? 1 : 0;
  }
  int failed = 0;
  int ran = 0;
  for (const Scenario& scenario : SCENARIOS) {
    if (argc > 1 && std::strcmp(argv[1], scenario.name) != 0) continue;
    ++ran;
    std::fflush(stdout);
    const pid_t pid = fork();
    if (pid == 0) _exit(run_one(scenario));
    int status = 0;
    waitpid(pid, &status, 0);
    const bool ok = WIFEXITED(status) && WEXITSTATUS(status) == 0;
    std::printf("%-26s %s\n", scenario.name, ok ? "ok" : "FAIL");
    if (!ok) ++failed;
  }
  if (!ran) {
    std::fprintf(stderr, "нет сценария %s\n", argc > 1 ? argv[1] : "");
    return 2;
  }
  std::printf("%d/%d сценариев прошли\n", ran - failed, ran);
  return failed ? 1 : 0;
}
