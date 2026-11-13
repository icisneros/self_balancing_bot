#pragma once
#include "Arduino.h"
#include <deque>
struct Quaternion { float pitch = 0; };
struct VectorFloat { float x = 0, y = 0, z = 1; };
class MPU6050 {
    std::deque<float> packets_;
    bool corrupt_ = false;
public:
    void initialize() {}
    uint8_t dmpInitialize() { return 0; }
    void setXAccelOffset(int) {}
    void setYAccelOffset(int) {}
    void setZAccelOffset(int) {}
    void setXGyroOffset(int) {}
    void setYGyroOffset(int) {}
    void setZGyroOffset(int) {}
    void setDMPEnabled(bool) {}
    uint8_t getIntStatus() { return packets_.size() >= 24 ? 0x12 : 0x02; }
    uint16_t dmpGetFIFOPacketSize() { return 42; }
    uint16_t getFIFOCount() { return packets_.size() * 42 + (corrupt_ ? 1 : 0); }
    void resetFIFO() { packets_.clear(); corrupt_ = false; ++sim::fifo_resets; }
    void getFIFOBytes(uint8_t* buffer, uint16_t) {
        float pitch = packets_.empty() ? 0 : packets_.front();
        if (!packets_.empty()) packets_.pop_front();
        std::memcpy(buffer, &pitch, sizeof(pitch));
        ++sim::imu_reads;
    }
    void dmpGetQuaternion(Quaternion* q, const uint8_t* buffer) {
        std::memcpy(&q->pitch, buffer, sizeof(q->pitch));
    }
    void dmpGetGravity(VectorFloat*, Quaternion*) {}
    void dmpGetYawPitchRoll(float* ypr, Quaternion* q, VectorFloat*) {
        ypr[0] = ypr[2] = 0; ypr[1] = q->pitch * M_PI / 180;
    }
    void inject(float pitch, bool corrupt = false) {
        if (packets_.size() < 24) packets_.push_back(pitch);
        corrupt_ = corrupt;
        if (sim::interrupt_handler) sim::interrupt_handler();
    }
};
