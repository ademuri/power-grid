#include <ESPmDNS.h>
#include <WiFi.h>
#include <dashboard.h>
#include <InterpolationLib.h>

#include "constants.h"

const int kChargeEn = 13;
const int kRelayEn = 22;
const int kBuzzer = 25;
const int kTempInternal = 26;
const int kTempExternal = 27;
const int kVehicleSense = 32;
const int kBatterySense = 33;
const int kLoadSense = 34;

const int kLed0 = 12;
const int kSw1 = 21;
const int kLed1 = 4;
const int kSw2 = 19;
const int kLed2 = 5;

const uint16_t kDacScale = 4096;

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

float GetVehicleVolts() {
  uint16_t raw = analogRead(kVehicleSense);
  float calculated = raw * (1 + 6.2) * 3.3 / 1 / kDacScale;
  return Interpolation::Linear(kVehicleVoltages, kTestVoltages, kNumVoltages, calculated, /*clamp=*/true);
}

float GetBatteryVolts() {
  uint16_t raw = analogRead(kBatterySense);
  float calculated = raw * (1 + 6.2) * 3.3 / 1 / kDacScale;
  return Interpolation::Linear(kBatteryVoltages, kTestVoltages, kNumVoltages, calculated, /*clamp=*/true);
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
  dashboard->Add<float>("Vehicle V", []() { return vehicle_volts; }, 100);
  dashboard->Add<float>("Battery V", []() { return battery_volts; } , 100);
  dashboard->Add<float>("Load A", []() { return load_amps; }, 100);
  server->begin();

  Serial.println(WiFi.localIP());
}

void loop() {
  vehicle_volts = GetVehicleVolts();
  battery_volts = GetBatteryVolts();
  load_amps = GetLoadAmps();
}
