// NMEA GPS emulator for ArduPilot flight controllers.
//
// Wiring (ESP32 -> SpeedyBee F405):
//   GND     -> GND
//   GPIO17  -> RX6 (UART6 = SERIAL6 in ArduPilot)
//
// Modes (switch with the BOOT button or the USB serial console):
//   0 - silence       -> Mission Planner shows "No GPS"
//   1 - no fix        -> "No Fix"
//   3 - 3D fix        -> "3D Fix"

#include <Arduino.h>

#include "nmea.h"

namespace {

constexpr int kGpsTxPin = 17;
constexpr int kGpsRxPin = 16;  // not wired, the FC's UBX probes are ignored
constexpr uint32_t kGpsBaud = 115200;
constexpr uint32_t kUpdatePeriodMs = 200;  // 5 Hz
constexpr int kLedPin = 2;
constexpr int kButtonPin = 0;  // BOOT button, active low
constexpr uint32_t kDebounceMs = 50;

// Simulated position: Kyiv, Maidan Nezalezhnosti
constexpr double kLatitude = 50.450100;
constexpr double kLongitude = 30.523400;
constexpr float kAltitudeM = 179.0f;
constexpr uint8_t kSatellitesWithFix = 12;
constexpr uint8_t kSatellitesNoFix = 2;
constexpr float kHdop = 0.8f;
constexpr time_t kStartEpoch = 1790942400;  // 2026-10-02 12:00:00 UTC

enum class Mode : uint8_t { NoGps, NoFix, Fix3D };

Mode mode = Mode::Fix3D;
bool echoNmea = false;
uint32_t lastUpdateMs = 0;

const char* modeName(Mode m) {
  switch (m) {
    case Mode::NoGps: return "NO GPS (output silenced)";
    case Mode::NoFix: return "NO FIX";
    case Mode::Fix3D: return "3D FIX";
  }
  return "?";
}

void setMode(Mode m) {
  mode = m;
  Serial.printf("[gps-sim] mode: %s\n", modeName(mode));
}

void printHelp() {
  Serial.println();
  Serial.println("[gps-sim] commands:");
  Serial.println("  0 - No GPS (stop NMEA output)");
  Serial.println("  1 - No Fix");
  Serial.println("  3 - 3D Fix");
  Serial.println("  v - toggle NMEA echo to this console");
  Serial.println("  h - help");
  Serial.println("  BOOT button cycles modes");
  Serial.printf("[gps-sim] NMEA out: GPIO%d @ %lu baud, %lu Hz\n", kGpsTxPin,
                static_cast<unsigned long>(kGpsBaud),
                static_cast<unsigned long>(1000 / kUpdatePeriodMs));
  Serial.printf("[gps-sim] mode: %s\n", modeName(mode));
}

void handleConsole() {
  while (Serial.available() > 0) {
    switch (Serial.read()) {
      case '0': setMode(Mode::NoGps); break;
      case '1': setMode(Mode::NoFix); break;
      case '3': setMode(Mode::Fix3D); break;
      case 'v':
        echoNmea = !echoNmea;
        Serial.printf("[gps-sim] NMEA echo: %s\n", echoNmea ? "on" : "off");
        break;
      case 'h': printHelp(); break;
      default: break;
    }
  }
}

void handleButton() {
  static bool lastStable = HIGH;
  static bool lastRead = HIGH;
  static uint32_t changedAt = 0;

  const bool reading = digitalRead(kButtonPin);
  if (reading != lastRead) {
    lastRead = reading;
    changedAt = millis();
  }
  if (millis() - changedAt < kDebounceMs || reading == lastStable) return;

  lastStable = reading;
  if (lastStable == LOW) {
    setMode(mode == Mode::NoGps ? Mode::NoFix : mode == Mode::NoFix ? Mode::Fix3D : Mode::NoGps);
  }
}

// LED: off = No GPS, slow blink = No Fix, solid = 3D Fix
void updateLed() {
  switch (mode) {
    case Mode::NoGps: digitalWrite(kLedPin, LOW); break;
    case Mode::NoFix: digitalWrite(kLedPin, (millis() / 500) % 2); break;
    case Mode::Fix3D: digitalWrite(kLedPin, HIGH); break;
  }
}

GpsState currentState() {
  const uint32_t now = millis();
  GpsState s{};
  s.hasFix = mode == Mode::Fix3D;
  s.latitude = kLatitude;
  s.longitude = kLongitude;
  s.altitudeM = kAltitudeM;
  s.satellites = s.hasFix ? kSatellitesWithFix : kSatellitesNoFix;
  s.hdop = kHdop;
  s.speedKnots = 0.0f;
  s.courseDeg = 0.0f;
  s.utcEpoch = kStartEpoch + now / 1000;
  s.utcMillis = now % 1000;
  return s;
}

void sendSentence(const char* sentence, size_t len) {
  if (len == 0) return;
  Serial2.write(reinterpret_cast<const uint8_t*>(sentence), len);
  if (echoNmea) Serial.write(reinterpret_cast<const uint8_t*>(sentence), len);
}

// GGA and RMC go out back to back: ArduPilot only reports a position
// when both arrive within 150 ms of each other.
void sendNmeaBurst() {
  const GpsState s = currentState();
  char buf[128];
  sendSentence(buf, nmeaBuildGGA(s, buf, sizeof(buf)));
  sendSentence(buf, nmeaBuildRMC(s, buf, sizeof(buf)));
  sendSentence(buf, nmeaBuildVTG(s, buf, sizeof(buf)));
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Serial2.begin(kGpsBaud, SERIAL_8N1, kGpsRxPin, kGpsTxPin);
  pinMode(kLedPin, OUTPUT);
  pinMode(kButtonPin, INPUT_PULLUP);
  delay(200);
  printHelp();
}

void loop() {
  handleConsole();
  handleButton();
  updateLed();

  const uint32_t now = millis();
  if (now - lastUpdateMs >= kUpdatePeriodMs) {
    lastUpdateMs = now;
    if (mode != Mode::NoGps) sendNmeaBurst();
  }
}
