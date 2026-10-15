// Хост-симуляция Arduino Mega для прошивок firmware/motor_master и motor_node.
//
// Каждая плата — свой экземпляр Board: выводы, USB Serial, конец шины I2C,
// модель шагового привода (TB6600 шагает по фронту PUL, если ENA = HIGH,
// направление — по DIR) и оптодатчика нуля на D18 (флажок с гистерезисом).
// Скетч компилируется внутри своего namespace (см. sim.cpp), а функции
// digitalWrite/Serial/Wire там подменяются привязанными к своей плате.
// Время общее: тик Timer1 — 50 мкс, прерывания всех плат вызываются по тику,
// loop() каждой платы — между тиками.
#pragma once

#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <strings.h>
#include <vector>
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define RISING 3
#define PROGMEM
#define PSTR(s) (s)
#define F(s) (s)
#define snprintf_P snprintf
#define vsnprintf_P vsnprintf
#define strncpy_P strncpy
#define strcpy_P strcpy
#define strcmp_P strcmp
#define strcasecmp_P strcasecmp
#define memcpy_P memcpy
#define ISR(vector) void vector##_handler()

typedef uint8_t byte;
typedef bool boolean;

// Регистры Timer1: прошивка их только настраивает, ход задаёт симуляция.
inline uint8_t TCCR1A = 0, TCCR1B = 0, TIMSK1 = 0;
inline uint16_t OCR1A = 0, TCNT1 = 0;
constexpr int WGM12 = 3, CS11 = 1, OCIE1A = 1;

namespace sim {

inline int64_t now_us = 0;
inline int64_t next_tick_us = 50;
void advance(int64_t us);

struct Board;

class SimSerial {
 public:
  std::deque<uint8_t> input;
  std::string output;
  void begin(unsigned long) {}
  int available() { return (int)input.size(); }
  int read() {
    if (input.empty()) return -1;
    const int ch = input.front();
    input.pop_front();
    return ch;
  }
  int availableForWrite() { return 63; }
  size_t write(uint8_t ch) {
    output.push_back((char)ch);
    return 1;
  }
  void send(const std::string& line) {
    for (char ch : line) input.push_back((uint8_t)ch);
    input.push_back('\n');
  }
};

class SimWire;
inline SimWire* bus[128] = {nullptr};

class SimWire {
 public:
  Board* board = nullptr;
  int address = -1;
  bool connected = true;                 // false — плата «отключена» от шины
  void (*on_receive)(int) = nullptr;
  void (*on_request)() = nullptr;
  std::vector<uint8_t> tx;               // мастер: передача; слейв: ответ на запрос
  std::deque<uint8_t> rx;                // принятые байты
  int tx_address = -1;
  long transmissions = 0;

  void begin() { address = -1; }
  void begin(uint8_t addr) {
    address = addr;
    bus[addr] = this;
  }
  void setClock(uint32_t) {}
  void onReceive(void (*fn)(int)) { on_receive = fn; }
  void onRequest(void (*fn)()) { on_request = fn; }
  void beginTransmission(uint8_t addr) {
    tx_address = addr;
    tx.clear();
  }
  size_t write(uint8_t value) {
    if (tx.size() >= 32) return 0;       // буфер Wire на AVR — 32 байта
    tx.push_back(value);
    return 1;
  }
  size_t write(const uint8_t* data, size_t length) {
    size_t written = 0;
    for (size_t i = 0; i < length; ++i) written += write(data[i]);
    return written;
  }
  uint8_t endTransmission() {
    ++transmissions;
    SimWire* slave = tx_address >= 0 && tx_address < 128 ? bus[tx_address] : nullptr;
    if (!slave || !slave->connected || !slave->on_receive) return 2;  // NACK адреса
    slave->rx.clear();
    for (uint8_t value : tx) slave->rx.push_back(value);
    slave->on_receive((int)tx.size());
    return 0;
  }
  uint8_t requestFrom(uint8_t addr, uint8_t quantity) {
    rx.clear();
    SimWire* slave = addr < 128 ? bus[addr] : nullptr;
    if (!slave || !slave->connected || !slave->on_request) return 0;
    slave->tx.clear();
    slave->on_request();
    for (uint8_t i = 0; i < quantity; ++i) {
      rx.push_back(i < slave->tx.size() ? slave->tx[i] : 0xFF);  // дальше — линия в 1
    }
    return quantity;
  }
  int available() { return (int)rx.size(); }
  int read() {
    if (rx.empty()) return -1;
    const int value = rx.front();
    rx.pop_front();
    return value;
  }
};

struct Board {
  const char* name = "";
  uint8_t level[80] = {0};
  uint8_t mode[80] = {0};
  int analog[80] = {0};
  SimSerial serial;
  SimWire wire;
  void (*setup_fn)() = nullptr;
  void (*loop_fn)() = nullptr;
  void (*isr_fn)() = nullptr;
  void (*tach_fn)() = nullptr;

  // Модель руля: phys — где колесо на самом деле, в шагах от «прямо».
  long phys = 0;
  long phys_min = 0, phys_max = 0;
  bool opto_present = true;
  bool opto_stuck = false;
  long flag_lo = -200, flag_hi = 200;    // шаги, где датчик закрыт
  long hysteresis = 20;
  bool opto_state = false;
  // Проверки сигналов TB6600.
  int64_t last_dir_change_us = -1000000;
  int64_t pul_rise_us = 0;
  int64_t last_rise_us = -1;
  int64_t min_high_us = 1000000;
  int64_t min_period_us = 1000000;
  long dir_setup_violations = 0;
  long steps = 0;
  long steps_while_disabled = 0;

  void reset_stats() {
    phys_min = phys_max = phys;
    min_high_us = min_period_us = 1000000;
    last_rise_us = -1;
    steps = 0;
  }

  bool opto_active() {
    if (opto_stuck) return true;
    if (!opto_present) return false;
    const bool inside = phys >= flag_lo && phys <= flag_hi;
    const bool held = phys >= flag_lo - hysteresis && phys <= flag_hi + hysteresis;
    opto_state = opto_state ? held : inside;
    return opto_state;
  }

  void pin_mode(uint8_t pin, uint8_t value) {
    mode[pin] = value;
    if (value == INPUT_PULLUP) level[pin] = HIGH;
  }

  void digital_write(uint8_t pin, uint8_t value) {
    value = value ? HIGH : LOW;
    const uint8_t before = level[pin];
    level[pin] = value;
    if (pin == 7 && before != value) last_dir_change_us = now_us;
    if (pin != 6 || before == value) return;
    if (value == HIGH) {
      // TB6600 делает шаг по фронту PUL.
      if (now_us - last_dir_change_us < 5) ++dir_setup_violations;
      if (last_rise_us >= 0 && now_us - last_rise_us < min_period_us) min_period_us = now_us - last_rise_us;
      last_rise_us = now_us;
      pul_rise_us = now_us;
      if (level[8] != HIGH) {
        ++steps_while_disabled;
        return;
      }
      phys += level[7] == LOW ? 1 : -1;
      ++steps;
      if (phys < phys_min) phys_min = phys;
      if (phys > phys_max) phys_max = phys;
    } else if (now_us - pul_rise_us < min_high_us) {
      min_high_us = now_us - pul_rise_us;
    }
  }

  int digital_read(uint8_t pin) {
    if (pin == 18) return opto_active() ? LOW : HIGH;
    return level[pin];
  }
};

inline std::vector<Board*> boards;

inline void tick_all() {
  for (Board* board : boards) {
    if (board->isr_fn) board->isr_fn();
  }
}

inline void advance(int64_t us) {
  const int64_t target = now_us + us;
  while (next_tick_us <= target) {
    now_us = next_tick_us;
    next_tick_us += 50;
    tick_all();
  }
  now_us = target;
}

// loop() каждой платы зовётся раз за тик, но в разной фазе: на чётных тиках
// сразу после прерывания, на нечётных — за 1 мкс до следующего. Так ловятся
// гонки вроде «DIR сменили, а фронт PUL пришёл раньше чем через 5 мкс».
inline int64_t loop_round = 0;

inline void run_for_ms(double ms) {
  const int64_t end = now_us + (int64_t)(ms * 1000.0);
  while (now_us < end) {
    now_us = next_tick_us;
    next_tick_us += 50;
    tick_all();
    if (++loop_round & 1) now_us = next_tick_us - 1;
    for (Board* board : boards) {
      if (board->loop_fn) board->loop_fn();
    }
  }
}

}  // namespace sim

inline unsigned long millis() { return (unsigned long)(sim::now_us / 1000); }
inline unsigned long micros() { return (unsigned long)sim::now_us; }
inline void delayMicroseconds(unsigned int us) { sim::advance(us); }
inline void delay(unsigned long ms) { sim::advance((int64_t)ms * 1000); }
inline void noInterrupts() {}
inline void interrupts() {}
