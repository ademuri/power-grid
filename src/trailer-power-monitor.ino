#include <ESPmDNS.h>
#include <WiFi.h>
#include <dashboard.h>
#include <InterpolationLib.h>
#include <state-manager.h>

#include "constants.h"
#include "timer.h"

// Tuning constants
// If the current draw from the vehicle is greater than this, disconnect it.
static constexpr float kMaxLoadAmps = 12.0;
// After disconnecting the vehicle, how long to wait before re-connecting.
static constexpr uint32_t kVehicleOffDelay = 20 * 1000;
// If the vehicle voltage drops below this, disconnect it.
static constexpr float kVehicleLowThreshold =  10.0;

// If the battery voltage drops below this, disconnect the battery.
static constexpr float kBatteryLowThreshold = 11.4;
// How long the battery voltage must be low before disconnection.
static constexpr uint32_t kBatteryLowDuration = 20 * 1000;
// After disconnecting the battery, how long to wait before reconnecting.
// This is long so that any built-up pressure in the compressor has time to discharge before we try to start it again.
static constexpr uint32_t kBatteryLowDelay = 10 * 60 * 1000;

// Pin definitions
static constexpr int kChargeEn = 13;
static constexpr int kRelayEn = 22;
static constexpr int kBuzzer = 25;
static constexpr int kTempInternal = 26;
static constexpr int kTempExternal = 27;
static constexpr int kVehicleSense = 32;
static constexpr int kBatterySense = 33;
static constexpr int kLoadSense = 34;

static constexpr int kLed0 = 12;
static constexpr int kSw1 = 21;
static constexpr int kLed1 = 4;
static constexpr int kSw2 = 19;
static constexpr int kLed2 = 5;

static constexpr uint16_t kDacScale = 4096;

// Note: the voltage dividers for reading vehicle and battery voltages are a bit off, possibly because of leakage current through the zener diode?
static constexpr size_t kNumVoltages = 7;
double kTestVoltages[kNumVoltages] = {10, 11, 12, 13, 14, 15, 16};
double kVehicleVoltages[kNumVoltages] = {9.15, 10.12, 11.05, 11.97, 12.73, 13.52, 14.27};
double kBatteryVoltages[kNumVoltages] = {9.45, 10.41, 11.43, 12.30, 13.09, 13.84, 14.55};

AsyncWebServer *server;
Dashboard *dashboard;

float vehicle_volts = 0;
float battery_volts = 0;
float load_amps = 0;

bool vehicle_connected = false;
bool battery_connected = false;

Timer vehicle_off_timer{kVehicleOffDelay};
Timer battery_low_timer{kBatteryLowDuration};
Timer battery_off_timer{kBatteryLowDelay};

float GetVehicleVolts() {
  uint16_t raw = analogRead(kVehicleSense);
  float calculated = raw * (1 + 6.2) * 3.3 / 1 / kDacScale;
  return Interpolation::Linear(kVehicleVoltages, kTestVoltages, kNumVoltages, calculated, /*clamp=*/false);
}

float GetBatteryVolts() {
  uint16_t raw = analogRead(kBatterySense);
  float calculated = raw * (1 + 6.2) * 3.3 / 1 / kDacScale;
  return Interpolation::Linear(kBatteryVoltages, kTestVoltages, kNumVoltages, calculated, /*clamp=*/false);
}

float GetLoadAmps() {
  // Load resistor is 0.01 Ohms, and amplifier is 50x
  uint32_t raw = analogRead(kLoadSense);
  return raw * 330.0 / 50 / kDacScale;
}

void setup() {
  Serial.begin(115200);

  pinMode(kChargeEn, OUTPUT);
  digitalWrite(kChargeEn, HIGH);
  pinMode(kRelayEn, OUTPUT);
  pinMode(kBuzzer, OUTPUT);
  pinMode(kVehicleSense, INPUT);
  pinMode(kBatterySense, INPUT);
  pinMode(kLoadSense, INPUT);

  pinMode(kLed0, OUTPUT);
  pinMode(kSw1, INPUT_PULLUP);
  pinMode(kLed1, OUTPUT);
  pinMode(kSw2, INPUT_PULLUP);
  pinMode(kLed2, OUTPUT);

  Serial.print("Connecting to wifi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED);
  delay(500);
  Serial.println(" done.");

  if (MDNS.begin("trailer-power-monitor")) {
    // Add service to MDNS-SD
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS responder started");
  } else {
    Serial.println("Error setting up MDNS responder!");
  }

  server = new AsyncWebServer(80);
  dashboard = new Dashboard(server);
  dashboard->Add<uint32_t>("Uptime", millis, 5000);
  dashboard->Add<bool>("Vehicle Connected", vehicle_connected, 1000);
  dashboard->Add<bool>("Battery Connected", battery_connected, 1000);
  dashboard->Add<float>("Vehicle V", vehicle_volts, 1000);
  dashboard->Add<float>("Battery V", battery_volts, 1000);
  dashboard->Add<float>("Load A", []() { return load_amps; }, 1000);
  server->begin();

  Serial.println(WiFi.localIP());
}

void loop() {
  vehicle_volts = GetVehicleVolts();
  battery_volts = GetBatteryVolts();
  load_amps = GetLoadAmps();

  const bool battery_low = battery_volts < kBatteryLowThreshold;
  const bool over_current = load_amps > kMaxLoadAmps;
  
  if (battery_low) {
    if (!battery_low_timer.Active() && !battery_low_timer.Expired()) {
      battery_low_timer.Reset();
    }
    if (battery_low_timer.Expired() && battery_connected) {
      digitalWrite(kRelayEn, LOW);
      battery_connected = false;
      battery_off_timer.Reset();
    }
  } else {
    // !battery_low
    battery_low_timer.Stop();
    if (!battery_off_timer.Active() || battery_off_timer.Expired()) {
      // Try connecting the battery
      digitalWrite(kRelayEn, HIGH);
      battery_connected = true;
    }
  }

  if (vehicle_connected) {
    if (over_current) {
      digitalWrite(kChargeEn, LOW);
      vehicle_connected = false;
      vehicle_off_timer.Reset();
    }
  } else {
    // !vehicle_connected
    if (!vehicle_off_timer.Active() || vehicle_off_timer.Expired()) {
      digitalWrite(kChargeEn, HIGH);
      vehicle_connected = true;
    }
  }

  digitalWrite(kLed1, battery_connected);
  digitalWrite(kLed2, vehicle_connected);
}
