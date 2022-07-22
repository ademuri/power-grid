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

AsyncWebServer *server;
Dashboard *dashboard;

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
  server->begin();
}

void loop() {}