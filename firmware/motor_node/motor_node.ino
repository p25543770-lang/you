#include <Wire.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ============================================================
// Узел мотора (слейв). Одна плата = один модуль: FR, RL или RR.
//
// Мастер (motor_master.ino) рассылает сюда ту же строку команды, что получил
// в Serial: "c 30", "d 45 f", "k", "h", "s", "g", "z", "c c".
// Мастер опрашивает узел запросом чтения и получает строку телеметрии:
//   "шаг,цель,движение,калибровка,идёт_калибровка,оптодатчик,цикл\n"
// Например: "-1234,0,1,1,0,0,0\n" (133.33 шага = 1°).
//
// Шаги формирует таймер (прерывание 20 кГц), как и на мастере, поэтому
// движение не зависит от обмена по I2C.
// ============================================================

// --- Адрес на шине I2C: 2 = FR, 3 = RL, 4 = RR (как в мастере) ---
const int MY_ADDRESS = 2;

// --- Распиновка ---
const int PUL = 6;
const int DIR = 7;
const int ENA = 8;
const int OPTO_PIN = 18;

// --- Настройки скорости и калибровки (как у мастера) ---
const unsigned long STEP_PERIOD_US = 100;
const unsigned long CALIB_STEP_PERIOD_US = 2000;
const float STEPS_PER_DEGREE = 133.333333;
const int CALIB_SEARCH_DEGREES = 100;
const int CALIB_SLOW_SEARCH_DEGREES = 20;
const bool OPTO_ACTIVE_LOW = true;

// --- Состояние мотора (часть переменных меняет таймер шагов) ---
volatile long current_step = 0;
volatile long target_step = 0;
volatile bool is_moving = false;
volatile unsigned long step_period_current = STEP_PERIOD_US;
volatile bool pulse_state = false;
volatile unsigned int tick_count = 0;
volatile bool move_done = false;

bool is_cycling = false;
long cycle_start_step = 0;
long cycle_offset_steps = 0;
bool cycle_direction_forward = true;
bool calibrated = false;            // true после поиска нуля по оптодатчику

enum CalibState {
  CALIB_IDLE,
  CALIB_FAST_RIGHT,
  CALIB_FAST_LEFT,
  CALIB_SLOW_RIGHT,
  CALIB_SLOW_LEFT,
  CALIB_MOVE_TO_ZERO
};

volatile CalibState calib_state = CALIB_IDLE;
long calib_right_edge = 0;
long calib_left_edge = 0;
long calib_zero_point = 0;
long calib_start_step = 0;

// --- Команда с шины: обработчик прерывания только копирует байты ---
volatile bool rx_ready = false;
char rx_buf[48];

// --- Телеметрия для мастера: две копии строки, переключаются без блокировки ---
char tlm_buf[2][32];
volatile uint8_t tlm_len[2] = {0, 0};
volatile uint8_t tlm_front = 0;      // копия, которую отдаём по запросу
uint8_t tlm_back = 1;                // копия, которую готовит основной цикл

void processCommand(String cmd);
void handleCalibration();
void onMoveFinished();
void refreshTelemetry();

// --- Таймер шагов: прерывание 20 кГц (каждые 50 мкс) ---
void setupStepTimer() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11);   // CTC, делитель 8: 2 МГц (плата на 16 МГц)
  OCR1A = 99;                            // 100 тактов = 50 мкс
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

ISR(TIMER1_COMPA_vect) {
  if (!is_moving) {
    tick_count = 0;
    return;
  }
  if (current_step == target_step) {
    is_moving = false;
    move_done = true;
    tick_count = 0;
    return;
  }
  unsigned long half = step_period_current / 100;   // полупериод в тиках по 50 мкс
  if (half < 1) half = 1;
  if (++tick_count < half) return;
  tick_count = 0;
  pulse_state = !pulse_state;
  digitalWrite(PUL, pulse_state);
  if (!pulse_state) {
    if (target_step > current_step) current_step++;
    else current_step--;
    if (current_step == target_step) {
      is_moving = false;
      move_done = true;
    }
  }
}

void onCommandReceived(int count) {
  uint8_t n = 0;
  while (Wire.available() && n < sizeof(rx_buf) - 1) {
    rx_buf[n++] = (char)Wire.read();
  }
  while (Wire.available()) Wire.read();   // лишнее — выкидываем
  rx_buf[n] = '\0';
  rx_ready = true;
}

// Ответ мастеру: готовая строка, ничего не считаем внутри прерывания I2C
void onTelemetryRequested() {
  uint8_t idx = tlm_front;
  Wire.write((const uint8_t*)tlm_buf[idx], tlm_len[idx]);
}

void setup() {
  Wire.begin(MY_ADDRESS);
  Wire.onReceive(onCommandReceived);
  Wire.onRequest(onTelemetryRequested);

  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENA, OUTPUT);
  digitalWrite(ENA, HIGH);
  pinMode(OPTO_PIN, INPUT_PULLUP);

  setupStepTimer();
  refreshTelemetry();
}

void loop() {
  if (rx_ready) {
    noInterrupts();
    String cmd = String(rx_buf);
    rx_ready = false;
    interrupts();
    cmd.trim();
    cmd.toLowerCase();
    if (cmd.length() > 0) processCommand(cmd);
  }

  if (move_done) {
    noInterrupts();
    move_done = false;
    interrupts();
    onMoveFinished();
  }

  if (calib_state != CALIB_IDLE) {
    handleCalibration();
  }

  static unsigned long last_refresh_ms = 0;
  unsigned long now_ms = millis();
  if (now_ms - last_refresh_ms >= 20) {
    last_refresh_ms = now_ms;
    refreshTelemetry();
  }
}

void onMoveFinished() {
  if (is_cycling) {
    if (cycle_direction_forward) {
      digitalWrite(DIR, HIGH);
      noInterrupts();
      target_step = cycle_start_step;
      is_moving = true;
      interrupts();
      cycle_direction_forward = false;
    } else {
      digitalWrite(DIR, LOW);
      noInterrupts();
      target_step = cycle_start_step + cycle_offset_steps;
      is_moving = true;
      interrupts();
      cycle_direction_forward = true;
    }
  }
}

// Строка телеметрии в свободную копию, потом переключаем копию
void refreshTelemetry() {
  noInterrupts();
  long c = current_step;
  bool m = is_moving;
  long t = target_step;
  interrupts();
  bool opto;
  if (OPTO_ACTIVE_LOW) {
    opto = (digitalRead(OPTO_PIN) == LOW);
  } else {
    opto = (digitalRead(OPTO_PIN) == HIGH);
  }
  int n = snprintf(tlm_buf[tlm_back], sizeof(tlm_buf[0]), "%ld,%ld,%d,%d,%d,%d,%d\n",
                   c, t,
                   m ? 1 : 0,
                   calibrated ? 1 : 0,
                   calib_state != CALIB_IDLE ? 1 : 0,
                   opto ? 1 : 0,
                   is_cycling ? 1 : 0);
  if (n <= 0 || n >= (int)sizeof(tlm_buf[0])) return;
  tlm_len[tlm_back] = (uint8_t)n;
  tlm_front = tlm_back;
  tlm_back = tlm_back ^ 1;
}

void handleCalibration() {
  bool opto_triggered;
  if (OPTO_ACTIVE_LOW) {
    opto_triggered = (digitalRead(OPTO_PIN) == LOW);
  } else {
    opto_triggered = (digitalRead(OPTO_PIN) == HIGH);
  }

  long local_current;
  noInterrupts();
  local_current = current_step;
  interrupts();

  switch (calib_state) {
    case CALIB_FAST_RIGHT:
      if (opto_triggered) {
        noInterrupts();
        calib_right_edge = current_step;
        is_moving = false;
        interrupts();

        step_period_current = CALIB_STEP_PERIOD_US;
        digitalWrite(DIR, HIGH);
        noInterrupts();
        target_step = current_step - (long)(CALIB_SLOW_SEARCH_DEGREES * STEPS_PER_DEGREE);
        is_moving = true;
        interrupts();
        calib_state = CALIB_SLOW_RIGHT;
      } else {
        long steps_done = local_current - calib_start_step;
        if (steps_done > (long)(CALIB_SEARCH_DEGREES * STEPS_PER_DEGREE)) {
          noInterrupts();
          is_moving = false;
          interrupts();

          digitalWrite(DIR, HIGH);
          noInterrupts();
          calib_start_step = current_step;
          target_step = current_step - (long)(2 * CALIB_SEARCH_DEGREES * STEPS_PER_DEGREE);
          is_moving = true;
          interrupts();
          calib_state = CALIB_FAST_LEFT;
        }
      }
      break;

    case CALIB_FAST_LEFT:
      if (opto_triggered) {
        noInterrupts();
        calib_left_edge = current_step;
        is_moving = false;
        interrupts();

        step_period_current = CALIB_STEP_PERIOD_US;
        digitalWrite(DIR, LOW);
        noInterrupts();
        target_step = current_step + (long)(CALIB_SLOW_SEARCH_DEGREES * STEPS_PER_DEGREE);
        is_moving = true;
        interrupts();
        calib_state = CALIB_SLOW_LEFT;
      }
      break;

    case CALIB_SLOW_RIGHT:
      if (opto_triggered) {
        noInterrupts();
        calib_right_edge = current_step;
        is_moving = false;
        interrupts();

        digitalWrite(DIR, HIGH);
        noInterrupts();
        target_step = current_step - (long)(2 * CALIB_SLOW_SEARCH_DEGREES * STEPS_PER_DEGREE);
        is_moving = true;
        interrupts();
        calib_state = CALIB_SLOW_LEFT;
      }
      break;

    case CALIB_SLOW_LEFT:
      if (opto_triggered) {
        noInterrupts();
        calib_left_edge = current_step;
        is_moving = false;
        interrupts();

        calib_zero_point = (calib_right_edge + calib_left_edge) / 2;
        step_period_current = STEP_PERIOD_US;
        if (calib_zero_point > current_step) {
          digitalWrite(DIR, LOW);
        } else {
          digitalWrite(DIR, HIGH);
        }
        noInterrupts();
        target_step = calib_zero_point;
        is_moving = true;
        interrupts();
        calib_state = CALIB_MOVE_TO_ZERO;
      }
      break;

    case CALIB_MOVE_TO_ZERO:
      if (local_current == target_step && !is_moving) {
        noInterrupts();
        current_step = 0;
        target_step = 0;
        is_moving = false;
        interrupts();

        calibrated = true;
        calib_state = CALIB_IDLE;
      }
      break;

    default:
      break;
  }
}

void processCommand(String cmd) {
  if (cmd == "k") {
    if (calib_state != CALIB_IDLE) return;
    is_cycling = false;
    noInterrupts();
    is_moving = false;
    interrupts();

    step_period_current = STEP_PERIOD_US;
    digitalWrite(DIR, LOW);

    noInterrupts();
    calib_start_step = current_step;
    target_step = current_step + (long)(CALIB_SEARCH_DEGREES * STEPS_PER_DEGREE);
    is_moving = true;
    interrupts();

    calib_state = CALIB_FAST_RIGHT;
  }
  else if (cmd.startsWith("c ")) {
    String param = cmd.substring(2);
    param.trim();
    if (param == "c") {
      if (is_cycling) {
        is_cycling = false;
        noInterrupts();
        is_moving = false;
        interrupts();
      }
    } else {
      float degrees = param.toFloat();
      if (degrees <= 0) return;
      noInterrupts();
      cycle_start_step = current_step;
      interrupts();
      cycle_offset_steps = round(degrees * STEPS_PER_DEGREE);
      is_cycling = true;
      cycle_direction_forward = true;
      digitalWrite(DIR, LOW);
      noInterrupts();
      target_step = cycle_start_step + cycle_offset_steps;
      is_moving = true;
      interrupts();
    }
  }
  else if (cmd.startsWith("d ")) {
    is_cycling = false;
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    if (secondSpace == -1) return;
    String degStr = cmd.substring(firstSpace + 1, secondSpace);
    String dirStr = cmd.substring(secondSpace + 1);
    degStr.trim();
    dirStr.trim();
    if (dirStr.length() > 1) dirStr = dirStr.substring(0, 1);
    float degrees = degStr.toFloat();
    if (degrees <= 0) return;
    long steps = round(degrees * STEPS_PER_DEGREE);
    if (dirStr == "f") {
      digitalWrite(DIR, LOW);
      noInterrupts();
      target_step = current_step + steps;
      is_moving = true;
      interrupts();
    }
    else if (dirStr == "b") {
      digitalWrite(DIR, HIGH);
      noInterrupts();
      target_step = current_step - steps;
      is_moving = true;
      interrupts();
    }
  }
  else if (cmd == "h") {
    is_cycling = false;
    noInterrupts();
    long current = current_step;
    interrupts();
    if (current == 0) return;
    if (0 > current) {
      digitalWrite(DIR, LOW);
    } else {
      digitalWrite(DIR, HIGH);
    }
    noInterrupts();
    target_step = 0;
    is_moving = true;
    interrupts();
  }
  else if (cmd == "s") {
    is_cycling = false;
    noInterrupts();
    is_moving = false;
    interrupts();
  }
  else if (cmd == "g") {
    noInterrupts();
    if (current_step != target_step) {
      is_moving = true;
    }
    interrupts();
  }
  else if (cmd == "z") {
    is_cycling = false;
    noInterrupts();
    current_step = 0;
    target_step = 0;
    is_moving = false;
    interrupts();
    calibrated = false;   // ноль задан вручную, а не по оптодатчику
  }
  // "p" на узле ничего не печатает: состояние мастер получает запросом телеметрии
}
