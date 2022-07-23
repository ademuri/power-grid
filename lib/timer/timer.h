#pragma once

#include <Arduino.h>

class Timer {
 public:
  Timer(uint32_t duration) : duration_(duration) {}

  void Reset() {
    started_ = true;
    expires_ = millis() + duration_;
  }
  void Stop() {
    started_ = false;
  }
  bool Expired() { return started_ && millis() > expires_; }
  bool Active() { return started_ && millis() <= expires_; }

 protected:
  const uint32_t duration_;
  uint32_t expires_ = 0;
  bool started_ = false;
};