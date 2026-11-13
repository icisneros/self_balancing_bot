#pragma once
#include "Arduino.h"
#ifdef SIM_COMPILING_PID
#include "../vendor/PID_v1.h"
#else
// Rename, then wrap the unchanged upstream class solely to observe Compute().
#define PID NativePID
#include "../vendor/PID_v1.h"
#undef PID
class PID : public NativePID {
public:
    using NativePID::NativePID;
    bool Compute() {
        bool computed = NativePID::Compute();
        if (computed) {
            double now = millis();
            sim::pid_dt_ms = sim::pid_last_ms < 0 ? 0 : now - sim::pid_last_ms;
            sim::pid_last_ms = now;
            ++sim::pid_updates;
        }
        return computed;
    }
};
#endif
