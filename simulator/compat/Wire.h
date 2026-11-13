#pragma once
#include "Arduino.h"
struct WireStub {
    void begin() {}
    void setClock(long) {}
    void beginTransmission(int) {}
    int endTransmission() { return 0; }
};
inline WireStub Wire;
