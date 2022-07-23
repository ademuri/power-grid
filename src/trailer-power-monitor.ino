#include <ESPmDNS.h>
#include <WiFi.h>
#include <dashboard.h>

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

AsyncWebServer *server;
Dashboard *dashboard;

float GetVehicleVolts() {
  uint16_t raw = analogRead(kVehicleSense);
  return raw * (36 + 160) * 3.3 / 36 / kDacScale;
}

float GetBatteryVolts() {
  uint16_t raw = analogRead(kVehicleSense);
  return raw * (36 + 160) * 3.3 / 36 / kDacScale;
}

uint32_t GetLoadMilliamps() {
  // Load resistor is 0.01 Ohms, and amplifier is 50x
  uint32_t raw = analogRead(kLoadSense);
  return raw * 330 / 50 / kDacScale;
}

void setup() {
  Serial.begin(115200);

  pinMode(kChargeEn, OUTPUT);
  pinMode(kRelayEn, OUTPUT);
  pinMode(kBuzzer, OUTPUT);
  pinMode(kVehicleSense, INPUT);
  pinMode(kBatterySense, INPUT);
  pinMode(kLoadSense, INPUT);

  pinMode(kLed0, OUTPUT);
  pinMode(kSw1, INPUT);
  pinMode(kLed1, OUTPUT);
  pinMode(kSw2, INPUT);
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
  dashboard->Add<float>("Vehicle mV", GetVehicleVolts, 1000);
  dashboard->Add<float>("Battery mV", GetBatteryVolts, 1000);
  dashboard->Add<uint32_t>("Load mA", GetLoadMilliamps, 1000);
  server->begin();
}

void loop() {}