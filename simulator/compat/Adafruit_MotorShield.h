#pragma once
#include "Arduino.h"
#define FORWARD 1
#define BACKWARD 2
#define RELEASE 4
class Adafruit_DCMotor {
    uint8_t speed_ = 0, direction_ = RELEASE;
public:
    void setSpeed(uint8_t speed) { speed_ = speed; }
    void run(uint8_t direction) { direction_ = direction; }
    double command() const {
        return direction_ == FORWARD ? speed_ : direction_ == BACKWARD ? -speed_ : 0;
    }
};
class Adafruit_MotorShield {
    Adafruit_DCMotor motors_[4];
public:
    void begin() {}
    Adafruit_DCMotor* getMotor(int n) { return &motors_[n - 1]; }
};
