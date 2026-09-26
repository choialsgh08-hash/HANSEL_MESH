#pragma once
#include <stdint.h>
#include <algorithm>
#include <cmath>
#include <sstream>

static unsigned long clockMs = 0;
static int duties[32] = {};
static constexpr int A0=14, A1=15, A2=16, A3=17;
static constexpr int HIGH=1, LOW=0, OUTPUT=1, INPUT_PULLUP=2, RISING=3;
inline unsigned long millis() { return clockMs; }
inline void analogWrite(int pin, int duty) { duties[pin] = duty; }
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return HIGH; }
inline void pinMode(int, int) {}
inline void noInterrupts() {}
inline void interrupts() {}
inline int digitalPinToInterrupt(int p) { return p-2; }
inline void attachInterrupt(int, void (*)(), int) {}
template <class T, class L, class H> T constrain(T v, L lo, H hi) {
  return std::max(T(lo), std::min(T(hi), v));
}
struct SerialMock {
  std::ostringstream output;
  void begin(int) {}
  int available() { return 0; }
  char read() { return 0; }
  template<class T> void print(T v) { output << v; }
  template<class T> void print(T v, int) { output << v; }
  template<class T> void println(T v) { output << v << '\n'; }
  template<class T> void println(T v, int) { output << v << '\n'; }
} Serial;
struct Servo {
  bool enabled = false;
  int pulse = 1500, attaches = 0;
  void attach(int, int, int) { enabled = true; ++attaches; }
  void detach() { enabled = false; }
  bool attached() const { return enabled; }
  void writeMicroseconds(int v) { pulse = v; }
};
