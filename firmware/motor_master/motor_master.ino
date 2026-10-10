#include <Wire.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const int SLAVE_ADDRESSES[] = {2, 3, 4};
const char* SLAVE_MODULE_IDS[] = {"FR", "RL", "RR"};   // модули узлов по адресам выше
const int NUM_SLAVES = 3;

// --- Распиновка ---
const int PUL = 6;
const int DIR = 7;
const int ENA = 8;
const int OPTO_PIN = 18;

// --- Настройки скорости ---
const unsigned long STEP_PERIOD_US = 100;
const unsigned long CALIB_STEP_PERIOD_US = 2000;
const float STEPS_PER_DEGREE = 133.333333;

// --- Настройки калибровки ---
const int CALIB_SEARCH_DEGREES = 100;
const int CALIB_SLOW_SEARCH_DEGREES = 20;
const bool OPTO_ACTIVE_LOW = true;

// --- Телеметрия для веб-сервера (строки "@TLM ..." каждые TLM_PERIOD_MS) ---
const char* MODULE_ID = "FL";          // какой модуль это мастер: FL / FR / RL / RR
const unsigned long TLM_PERIOD_MS = 250;
bool calibrated = false;               // true после поиска нуля по оптодатчику
unsigned long last_tlm_ms = 0;

// --- Состояние мотора (часть переменных меняет таймер шагов) ---
volatile long current_step = 0;
volatile long target_step = 0;
volatile bool is_moving = false;
volatile unsigned long step_period_current = STEP_PERIOD_US;
volatile bool pulse_state = false;
volatile unsigned int tick_count = 0;
volatile bool move_done = false;       // цель достигнута, основной цикл обработает

// --- Циклический режим ---
bool is_cycling = false;
long cycle_start_step = 0;
long cycle_offset_steps = 0;
bool cycle_direction_forward = true;

String inputString = "";

// --- Машина состояний калибровки ---
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

// --- Очередь вывода: строки телеметрии уходят в порт по мере освобождения буфера UART ---
char tx_buf[512];
int tx_len = 0;
int tx_pos = 0;

void processCommand(String cmd);
void handleCalibration();
void onMoveFinished();
void sendTelemetry();
void pollSlaves();

// --- Таймер шагов: прерывание 20 кГц (каждые 50 мкс), не зависит от печати и I2C ---
void setupStepTimer() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11);   // режим CTC, делитель 8: 2 МГц (плата на 16 МГц)
  OCR1A = 99;                            // 100 тактов = 50 мкс
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

ISR(TIMER1_COMPA_vect) {
  if (!is_moving) {
    tick_count = 0;
    return;
  }
  if (current_step == target_step) {     // защита: цель уже достигнута
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
  if (!pulse_state) {                    // спад импульса = один шаг
    if (target_step > current_step) current_step++;
    else current_step--;
    if (current_step == target_step) {
      is_moving = false;
      move_done = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  inputString.reserve(64);

  Wire.begin();
  Wire.setClock(400000);

  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENA, OUTPUT);
  digitalWrite(ENA, HIGH);

  pinMode(OPTO_PIN, INPUT_PULLUP);

  setupStepTimer();

  Serial.println(F("--- MASTER: Система с калибровкой ---"));
  Serial.println(F("Команды: c [градусы], c c, d [градусы] [f/b], h, s, g, p, z, k"));
}

void loop() {
  if (move_done) {
    noInterrupts();
    move_done = false;
    interrupts();
    onMoveFinished();
  }

  if (calib_state != CALIB_IDLE) {
    handleCalibration();
  }

  unsigned long now_ms = millis();
  if (now_ms - last_tlm_ms >= TLM_PERIOD_MS && tx_pos >= tx_len) {
    last_tlm_ms = now_ms;
    sendTelemetry();
  }
  while (tx_pos < tx_len && Serial.availableForWrite() > 0) {
    Serial.write(tx_buf[tx_pos++]);
  }

  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      inputString.trim();
      if (inputString.length() > 0) {
        for (int i = 0; i < NUM_SLAVES; i++) {
          Wire.beginTransmission(SLAVE_ADDRESSES[i]);
          Wire.write(inputString.c_str());
          Wire.endTransmission();
        }
        processCommand(inputString);
      }
      inputString = "";
    } else {
      if (inputString.length() < 60) inputString += inChar;
    }
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
  } else if (calib_state == CALIB_IDLE) {
    float current_angle = (float)current_step / STEPS_PER_DEGREE;
    Serial.print(F("Цель достигнута. Угол: "));
    Serial.print(current_angle, 2);
    Serial.println(F("°"));
  }
}

// --- Телеметрия: строки формируются в tx_buf, отправка идёт по мере свободного места в UART ---

// Градусы в сотых без float при печати: "-9.25"
void fmtCenti(long v, char* out) {
  long a = v < 0 ? -v : v;
  snprintf(out, 12, "%s%ld.%02ld", v < 0 ? "-" : "", a / 100, a % 100);
}

// Строка для сервера: "@TLM mod=FL deg=-9.25 tgt=0.00 moving=1 cal=1 cycle=0 calib=0 opto=0 t=12345"
void appendTlm(const char* mod, long c, long t, bool m, bool cal, bool cycle, bool calib, bool opto) {
  char dbuf[12];
  char tbuf[12];
  char line[120];
  fmtCenti((long)round((float)c / STEPS_PER_DEGREE * 100.0), dbuf);
  fmtCenti((long)round((float)t / STEPS_PER_DEGREE * 100.0), tbuf);
  int n = snprintf(line, sizeof(line),
                   "@TLM mod=%s deg=%s tgt=%s moving=%d cal=%d cycle=%d calib=%d opto=%d t=%lu\r\n",
                   mod, dbuf, tbuf, m ? 1 : 0, cal ? 1 : 0, cycle ? 1 : 0, calib ? 1 : 0, opto ? 1 : 0,
                   (unsigned long)millis());
  if (n > 0 && tx_len + n < (int)sizeof(tx_buf)) {
    memcpy(tx_buf + tx_len, line, n);
    tx_len += n;
  }
}

void sendTelemetry() {
  noInterrupts();
  long c = current_step;
  long t = target_step;
  bool m = is_moving;
  interrupts();
  bool opto;
  if (OPTO_ACTIVE_LOW) {
    opto = (digitalRead(OPTO_PIN) == LOW);
  } else {
    opto = (digitalRead(OPTO_PIN) == HIGH);
  }
  tx_len = 0;
  tx_pos = 0;
  appendTlm(MODULE_ID, c, t, m, calibrated, is_cycling, calib_state != CALIB_IDLE, opto);
  pollSlaves();
}

// Опрос узлов FR, RL, RR: ответ "шаг,цель,движение,кал,калиб,опто,цикл\n"
bool parseFields(const char* s, long* v, int count) {
  for (int i = 0; i < count; i++) {
    char* end;
    v[i] = strtol(s, &end, 10);
    if (end == s) return false;
    s = end;
    if (i < count - 1) {
      if (*s != ',') return false;
      s++;
    }
  }
  return true;
}

void pollSlaves() {
  for (int i = 0; i < NUM_SLAVES; i++) {
    int got = Wire.requestFrom(SLAVE_ADDRESSES[i], 32);
    if (got <= 0) continue;                 // узел не ответил — строку не добавляем
    char buf[40];
    int len = 0;
    while (Wire.available()) {
      char ch = (char)Wire.read();
      if (ch == '\n') break;
      if (len < 39) buf[len++] = ch;
    }
    while (Wire.available()) Wire.read();
    buf[len] = '\0';
    long v[7];
    if (!parseFields(buf, v, 7)) continue;  // битая строка — пропускаем
    appendTlm(SLAVE_MODULE_IDS[i], v[0], v[1], v[2] == 1, v[3] == 1, v[6] == 1, v[4] == 1, v[5] == 1);
  }
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

        Serial.print(F("Калибровка: Быстрый поиск вправо - найдено на шаге "));
        Serial.println(calib_right_edge);

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
          Serial.println(F("Калибровка: Не найдено вправо, едем влево на 200°..."));
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

        Serial.print(F("Калибровка: Быстрый поиск влево - найдено на шаге "));
        Serial.println(calib_left_edge);

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

        Serial.print(F("Калибровка: Медленный подход справа - точная точка "));
        Serial.println(calib_right_edge);

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

        Serial.print(F("Калибровка: Медленный подход слева - точная точка "));
        Serial.println(calib_left_edge);

        calib_zero_point = (calib_right_edge + calib_left_edge) / 2;
        Serial.print(F("Калибровка: Точка 0 = "));
        Serial.println(calib_zero_point);

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
        Serial.println(F("Калибровка ЗАВЕРШЕНА! Точка 0 установлена."));
      }
      break;

    default:
      break;
  }
}

void processCommand(String cmd) {
  cmd.toLowerCase();

  if (cmd == "k") {
    if (calib_state != CALIB_IDLE) {
      Serial.println(F("Калибровка уже запущена!"));
      return;
    }

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
    Serial.println(F("Калибровка: Быстрый поиск вправо на 100°..."));
  }
  else if (cmd.startsWith("c ")) {
    String param = cmd.substring(2);
    param.trim();
    if (param == "c") {
      if (is_cycling) {
        is_cycling = false;
        noInterrupts();
        is_moving = false;
        long copy_step = current_step;
        interrupts();
        float current_angle = (float)copy_step / STEPS_PER_DEGREE;
        Serial.print(F("ЦИКЛ ВЫКЛ. Угол: "));
        Serial.print(current_angle, 2);
        Serial.println(F("°"));
      } else {
        Serial.println(F("Цикл и так выключен."));
      }
    } else {
      float degrees = param.toFloat();
      if (degrees <= 0) {
        Serial.println(F("Ошибка: Угол > 0!"));
        return;
      }
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
      Serial.print(F("ЦИКЛ качания на "));
      Serial.print(degrees, 2);
      Serial.println(F("°"));
    }
  }
  else if (cmd.startsWith("d ")) {
    is_cycling = false;
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    if (secondSpace == -1) {
      Serial.println(F("Шаблон: d [градусы] [f/b]"));
      return;
    }
    String degStr = cmd.substring(firstSpace + 1, secondSpace);
    String dirStr = cmd.substring(secondSpace + 1);
    degStr.trim();
    dirStr.trim();
    if (dirStr.length() > 1) dirStr = dirStr.substring(0, 1);
    float degrees = degStr.toFloat();
    if (degrees <= 0) {
      Serial.println(F("Ошибка: Угол > 0!"));
      return;
    }
    long steps = round(degrees * STEPS_PER_DEGREE);
    if (dirStr == "f") {
      digitalWrite(DIR, LOW);
      noInterrupts();
      target_step = current_step + steps;
      is_moving = true;
      interrupts();
      Serial.print(F("ВПЕРЕД (F) на "));
      Serial.print(degrees, 2);
      Serial.println(F("°"));
    }
    else if (dirStr == "b") {
      digitalWrite(DIR, HIGH);
      noInterrupts();
      target_step = current_step - steps;
      is_moving = true;
      interrupts();
      Serial.print(F("НАЗАД (B) на "));
      Serial.print(degrees, 2);
      Serial.println(F("°"));
    }
    else {
      Serial.println(F("Ошибка направления! f или b"));
    }
  }
  else if (cmd == "h") {
    is_cycling = false;
    noInterrupts();
    long current = current_step;
    interrupts();
    if (current == 0) {
      Serial.println(F("Уже в 0°."));
      return;
    }
    if (0 > current) {
      digitalWrite(DIR, LOW);
      Serial.println(F("Домой: ВПЕРЕД..."));
    } else {
      digitalWrite(DIR, HIGH);
      Serial.println(F("Домой: НАЗАД..."));
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
    long copy_step = current_step;
    interrupts();
    float current_angle = (float)copy_step / STEPS_PER_DEGREE;
    Serial.print(F("ПАУЗА. Угол: "));
    Serial.print(current_angle, 2);
    Serial.println(F("°"));
  }
  else if (cmd == "g") {
    noInterrupts();
    if (current_step != target_step) {
      is_moving = true;
      Serial.println(F("ПРОДОЛЖЕНИЕ..."));
    } else {
      Serial.println(F("Уже в цели."));
    }
    interrupts();
  }
  else if (cmd == "p") {
    noInterrupts();
    long c = current_step;
    long t = target_step;
    bool m = is_moving;
    interrupts();
    Serial.print(F("Текущий: "));
    Serial.print((float)c / STEPS_PER_DEGREE, 2);
    if (is_cycling) {
      Serial.println(F("° [ЦИКЛ]"));
    } else {
      Serial.print(F("° | Целевой: "));
      Serial.print((float)t / STEPS_PER_DEGREE, 2);
      Serial.println(m ? F("° [ДВИЖ]") : F("° [СТОИТ]"));
    }
  }
  else if (cmd == "z") {
    is_cycling = false;
    noInterrupts();
    current_step = 0;
    target_step = 0;
    is_moving = false;
    interrupts();
    calibrated = false;   // ноль задан вручную, а не по оптодатчику
    Serial.println(F("ОБНУЛЕНА."));
  }
  else {
    Serial.println(F("Неизв. команда"));
  }
}
