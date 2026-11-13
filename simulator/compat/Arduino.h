#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#define ARDUINO 100
#define INPUT 0
#define FALLING 2
#define BIN 2
#define B10000 16
#define B1000 8
#define B1 1
#define F(value) value
using boolean = bool;
using std::abs;
using std::isnan;
using std::max;
using std::min;
template<class T> T constrain(T x, T low, T high) { return std::clamp(x, low, high); }

namespace sim {
inline uint64_t clock_us = 0;
inline void (*interrupt_handler)() = nullptr;
inline double pid_last_ms = -1, pid_dt_ms = 0;
inline unsigned pid_updates = 0, imu_reads = 0, fifo_resets = 0;
}
// The AVR clock is 32 bits even on 64-bit desktop hosts.
inline unsigned long millis() { return uint32_t(sim::clock_us / 1000); }
inline unsigned long micros() { return uint32_t(sim::clock_us); }
inline void delay(unsigned long ms) { sim::clock_us += ms * 1000; }
inline void pinMode(int, int) {}
inline void attachInterrupt(int, void (*fn)(), int) { sim::interrupt_handler = fn; }
inline uint8_t TWBR = 0;
struct SerialStub {
    void begin(int) {}
    explicit operator bool() const { return true; }
    template<class... T> void print(T...) {}
    template<class... T> void println(T...) {}
};
inline SerialStub Serial;
