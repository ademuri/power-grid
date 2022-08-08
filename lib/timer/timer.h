#pragma once

#include <Arduino.h>

class Timer {
 public:
  Timer(uint32_t duration) : duration_(duration) {}

  void Reset() {
    started_ = true;
    expires_ = millis() + duration_;
  }
  void Stop() { started_ = false; }
  bool Expired() { return started_ && millis() > expires_; }
  bool Active() { return started_ && millis() <= expires_; }
  bool Running() { return started_; }

 protected:
  const uint32_t duration_;
  uint32_t expires_ = 0;
  bool started_ = false;
};

class CountUpTimer {
 public:
  CountUpTimer() {}

  void Reset() {
    started_ = true;
    started_millis_ = millis();
  }

  void Stop() {
    started_ = false;
  }

  bool IsRunning() {
    return started_;
  }

  uint32_t Get() {
    if (started_) {
      return millis() - started_millis_;
    } else {
      return 0;
    }
  }

 private:
  uint32_t started_millis_ = 0;
  bool started_ = false;
};
