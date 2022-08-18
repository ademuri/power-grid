#include <ArduinoOTA.h>
#include <DS18B20.h>
#include <ESPmDNS.h>
#include <InterpolationLib.h>
#include <WiFi.h>
#include <dashboard.h>
#include <state-manager.h>

#include <string>

#include "constants.h"
#include "timer.h"

// Tuning constants
// If the current draw from the vehicle is greater than this, disconnect the
// vehicle.
static constexpr float kMaxLoadAmps = 12.0;
// After disconnecting the vehicle, how long to wait before re-connecting.
static constexpr uint32_t kVehicleOffDelay = 20 * 1000;
// If the vehicle voltage drops below this, disconnect it.
static constexpr float kVehicleLowThreshold = 10.0;
// If the vehicle is lower than the battery for this duration, disconnect the
// vehicle.
static constexpr uint32_t kVehicleLowerThanBatteryDuration = 30 * 1000;
// Periodically, disconnect the vehicle and check its voltage.
static constexpr uint32_t kVehicleCheckPeriod = 10 * 1000;
// When checking the vehicle while disconnected, wait this long after
// disconnecting before measuring the voltage.
static constexpr uint32_t kVehicleCheckDelay = 10;
// When the vehicle is not connected, the voltage must be this high to connect
// it. This is to prevent draining the vehicle's battery when the engine isn't
// running.
static constexpr float kVehicleDisconnectedCutoff = 13.0;

// If the battery voltage drops below this, disconnect the battery.
static constexpr float kBatteryLowThreshold = 11.4;
// How long the battery voltage must be low before disconnection.
static constexpr uint32_t kBatteryLowDuration = 20 * 1000;
// After disconnecting the battery, how long to wait before reconnecting.
// This is long so that any built-up pressure in the compressor has time to
// discharge before we try to start it again.
static constexpr uint32_t kBatteryLowDelay = 10 * 60 * 1000;
// Try to reset the inverter after this period
static constexpr uint32_t kInverterTryResetPeriod = 30 * 60 * 1000;
// Always reset the inverter after this period
static constexpr uint32_t kInverterHardResetPeriod = 60 * 60 * 1000;
// If the current from the vehicle is below this, reset the inverter
static constexpr float kInverterResetCurrent = 4.0;
// How long to wait after seeing a battery min, before trying to reset inverter.
// This attemps to avoid turning off the inverter while the load is running.
static constexpr uint32_t kBatteryMinResetDelay = 15 * 60 * 1000;

// Pin definitions
static constexpr int kChargeEn = 13;
static constexpr int kRelayEn = 22;
static constexpr int kBuzzer = 25;
static constexpr int kTempInternal = 26;
static constexpr int kTempExternal = 27;
static constexpr int kVehicleSense = 32;
static constexpr int kBatterySense = 33;
static constexpr int kLoadSense = 34;
static constexpr int kInsideTempPin = 26;
static constexpr int kOutsideTempPin = 27;

static constexpr int kLed0 = 12;
static constexpr int kSw1 = 21;
static constexpr int kLed1 = 4;
static constexpr int kSw2 = 19;
static constexpr int kLed2 = 5;

static constexpr int kBuzzerPwmChannel = 0;

static constexpr uint16_t kDacScale = 4096;

// Note: the voltage dividers for reading vehicle and battery voltages are a bit
// off, possibly because of leakage current through the zener diode?
static constexpr size_t kNumVoltages = 7;
double kTestVoltages[kNumVoltages] = {10, 11, 12, 13, 14, 15, 16};
double kVehicleVoltages[kNumVoltages] = {9.15,  10.12, 11.05, 11.97,
                                         12.73, 13.52, 14.27};
double kBatteryVoltages[kNumVoltages] = {9.45,  10.41, 11.43, 12.30,
                                         13.09, 13.84, 14.55};

AsyncWebServer *server;
Dashboard *dashboard;

float vehicle_volts_disconnected = 0;
float vehicle_volts = 0;
float battery_volts = 0;
float battery_volts_no_vehicle = 0;
float load_amps = 0;

// Battery low voltage tracking
float battery_volts_min = 100.0;
uint32_t battery_min_time = 0;
float battery_min_time_volts = 0;
CountUpTimer battery_min_timer;
CountUpTimer time_since_battery_min_timer;
static constexpr float kBatteryMinThreshold = 0.3;

bool vehicle_connected = false;
bool battery_connected = false;

Timer vehicle_off_timer{kVehicleOffDelay};
Timer vehicle_lower_than_battery_timer{kVehicleLowerThanBatteryDuration};
Timer battery_low_timer{kBatteryLowDuration};
Timer battery_off_timer{kBatteryLowDelay};
Timer inverter_try_reset_timer{kInverterTryResetPeriod};
Timer inverter_hard_reset_timer{kInverterHardResetPeriod};
Timer check_vehicle_period{kVehicleCheckPeriod};

DS18B20 board_therm = DS18B20(kInsideTempPin);
DS18B20 external_therm = DS18B20(kOutsideTempPin);

char *FormatFloat(const float f, size_t decimal_places) {
  static constexpr size_t size = 20;
  static char buffer[size];
  static char format[size];
  if (snprintf(format, size, "%%.%df", decimal_places) <= 0) {
    buffer[0] = '\0';
    return buffer;
  }
  if (snprintf(buffer, size, format, f) <= 0) {
    buffer[0] = '\0';
  }
  return buffer;
}

char *FormatTemp(const float t) {
  static constexpr size_t size = 10;
  static char buffer[size];
  snprintf(buffer, size, "%.0f °F", t);
  return buffer;
}

float GetVehicleVolts() {
  uint16_t raw = analogRead(kVehicleSense);
  float calculated = raw * (1 + 6.2) * 3.3 / 1 / kDacScale;
  return Interpolation::Linear(kVehicleVoltages, kTestVoltages, kNumVoltages,
                               calculated, /*clamp=*/false);
}

float GetBatteryVolts() {
  uint16_t raw = analogRead(kBatterySense);
  float calculated = raw * (1 + 6.2) * 3.3 / 1 / kDacScale;
  return Interpolation::Linear(kBatteryVoltages, kTestVoltages, kNumVoltages,
                               calculated, /*clamp=*/false);
}

float GetLoadAmps() {
  // Load resistor is 0.01 Ohms, and amplifier is 50x
  uint32_t raw = analogRead(kLoadSense);
  return raw * 330.0 / 50 / kDacScale;
}

float board_temp = 0;
float external_temp = 0;

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

  // Serial.print("Setting up AP... ");
  // WiFi.softAP("TrailerPowerMonitor");
  // Serial.println("Done.");
  // Serial.print("IP address: ");
  // Serial.println(WiFi.softAPIP());

  Serial.print("Connecting to wifi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
    ;
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
  dashboard->Add<uint32_t>("Uptime", millis, 10000);
  dashboard->Add<bool>("Vehicle Connected", vehicle_connected, 5100);
  dashboard->Add<bool>("Battery Connected", battery_connected, 5200);
  dashboard->Add<char *>(
      "Vehicle V", []() { return FormatFloat(vehicle_volts, 2); }, 1020);
  dashboard->Add<char *>(
      "Vehicle V (floating)",
      []() { return FormatFloat(vehicle_volts_disconnected, 2); }, 5300);
  dashboard->Add<char *>(
      "Battery V", []() { return FormatFloat(battery_volts, 2); }, 1040);
  dashboard->Add<char *>(
      "Battery V (no vehicle)",
      []() { return FormatFloat(battery_volts_no_vehicle, 2); }, 5400);
  // dashboard->Add<char *>(
  //     "Battery V, min", []() { return FormatFloat(battery_min_time_volts, 2); },
  //     10100);
  // dashboard->Add<uint32_t>("Battery min time", battery_min_time, 10200);
  // dashboard->Add<uint32_t>(
  //     "Minutes since battery min",
  //     []() { return time_since_battery_min_timer.Get() / (60 * 1000); }, 9900);
  dashboard->Add<char *>(
      "Load A", []() { return FormatFloat(load_amps, 2); }, 900);
  dashboard->Add<char *>(
      "External temp", []() { return FormatTemp(external_temp); },
      12100);
  dashboard->Add<char *>(
      "Board temp", []() { return FormatTemp(board_temp); },
      12500);
  server->begin();

  Serial.println(WiFi.localIP());

  ledcAttachPin(kBuzzer, kBuzzerPwmChannel);
  ledcWriteTone(kBuzzerPwmChannel, 2000);
  delay(500);
  ledcWriteTone(kBuzzerPwmChannel, 0);

  inverter_try_reset_timer.Reset();
  inverter_hard_reset_timer.Reset();

  if (external_therm.getNumberOfDevices() < 1) {
    Serial.println("No outside temp sensor found");
  }
  if (board_therm.getNumberOfDevices() < 1) {
    Serial.println("No inside temp sensor found");
  }

  external_therm.selectNext();
  board_therm.selectNext();

  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else  // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS
        // using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
      });

  ArduinoOTA.begin();
}

void loop() {
  ArduinoOTA.handle();

  vehicle_volts = GetVehicleVolts();
  battery_volts = GetBatteryVolts();
  load_amps = GetLoadAmps();
  if (millis() > 10000) {
    float prev_min = battery_volts_min;
    battery_volts_min = std::min(battery_volts_min, battery_volts);
    if (prev_min != battery_volts_min &&
        std::abs(battery_volts_min - battery_min_time_volts) >
            kBatteryMinThreshold / 2) {
      battery_min_time = 0;
      battery_min_time_volts = battery_volts_min;
    }
  }
  if (std::abs(battery_volts - battery_volts_min < kBatteryMinThreshold) ||
      battery_volts < (battery_volts_min - kBatteryMinThreshold)) {
    if (battery_min_timer.IsRunning()) {
      battery_min_time = std::max(battery_min_timer.Get(), battery_min_time);
    } else {
      battery_min_timer.Reset();
      time_since_battery_min_timer.Reset();
    }
  } else {
    if (battery_min_timer.IsRunning()) {
      battery_min_time = std::max(battery_min_timer.Get(), battery_min_time);
      battery_min_timer.Stop();
    }
  }

  const bool battery_low = battery_volts < kBatteryLowThreshold;
  const bool over_current = load_amps > kMaxLoadAmps;
  const bool vehicle_low = vehicle_volts < kVehicleLowThreshold;

  if (vehicle_volts >= battery_volts) {
    vehicle_lower_than_battery_timer.Stop();
  } else if (!vehicle_lower_than_battery_timer.Running()) {
    vehicle_lower_than_battery_timer.Reset();
  }

  if (battery_low) {
    if (!battery_low_timer.Running()) {
      battery_low_timer.Reset();
    }
    if (battery_low_timer.Expired() && battery_connected) {
      digitalWrite(kRelayEn, HIGH);
      battery_connected = false;
      battery_off_timer.Reset();
    }
  } else {
    // !battery_low
    battery_low_timer.Stop();
    if (!battery_off_timer.Active() || battery_off_timer.Expired()) {
      // Try connecting the battery
      digitalWrite(kRelayEn, LOW);
      battery_connected = true;
    } else {
      // Inverter continues to be on
      if (inverter_hard_reset_timer.Expired() ||
          (inverter_try_reset_timer.Expired() &&
           load_amps < kInverterResetCurrent &&
           time_since_battery_min_timer.Get() > kBatteryMinResetDelay)) {
        digitalWrite(kRelayEn, HIGH);
        delay(100);
        digitalWrite(kRelayEn, LOW);
        inverter_try_reset_timer.Reset();
        inverter_hard_reset_timer.Reset();
      }
    }
  }

  if (vehicle_connected) {
    if (over_current || vehicle_low ||
        vehicle_lower_than_battery_timer.Expired()) {
      digitalWrite(kChargeEn, LOW);
      vehicle_connected = false;
      vehicle_off_timer.Reset();
      check_vehicle_period.Stop();
    } else if (check_vehicle_period.Expired()) {
      digitalWrite(kChargeEn, LOW);
      delay(kVehicleCheckDelay);
      vehicle_volts_disconnected = GetVehicleVolts();
      battery_volts_no_vehicle = GetBatteryVolts();
      digitalWrite(kChargeEn, HIGH);
      check_vehicle_period.Reset();

      if (vehicle_volts_disconnected < kVehicleDisconnectedCutoff) {
        digitalWrite(kChargeEn, LOW);
        vehicle_connected = false;
        vehicle_off_timer.Reset();
        check_vehicle_period.Stop();
      }
    }
  } else {
    // !vehicle_connected
    if ((!vehicle_off_timer.Active() || vehicle_off_timer.Expired()) &&
        !vehicle_low && !vehicle_lower_than_battery_timer.Running() &&
        vehicle_volts > kVehicleDisconnectedCutoff) {
      digitalWrite(kChargeEn, HIGH);
      vehicle_connected = true;
      check_vehicle_period.Reset();
    }
  }

  digitalWrite(kLed0, (millis() / 100) % 50 == 0);
  digitalWrite(kLed1, battery_connected);
  digitalWrite(kLed2, vehicle_connected);

  if (std::abs(external_therm.getTempC() + .0625) < 0.01) {
    external_therm.selectNext();
  }
  if (std::abs(board_therm.getTempC() + .0625) < 0.01) {
    board_therm.selectNext();
  }
  external_temp = external_therm.getTempF();
  board_temp = board_therm.getTempF();
}
