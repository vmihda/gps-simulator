#pragma once

#include <stddef.h>
#include <stdint.h>
#include <time.h>

// Snapshot of the simulated receiver state used to build NMEA sentences
struct GpsState {
  bool hasFix;
  double latitude;   // degrees, + north
  double longitude;  // degrees, + east
  float altitudeM;   // above mean sea level
  uint8_t satellites;
  float hdop;
  float speedKnots;
  float courseDeg;
  time_t utcEpoch;   // whole seconds, UTC
  uint16_t utcMillis;
};

// XOR of all characters between '$' and '*' (body must contain neither)
uint8_t nmeaChecksum(const char* body);

// Writes "ddmm.mmmmm,N" (latitude) or "dddmm.mmmmm,E" (longitude)
void nmeaFormatCoord(double degrees, bool isLongitude, char* out, size_t cap);

// Each builder writes a full sentence including "$", "*XX" and "\r\n".
// Returns the sentence length, or 0 if the buffer is too small.
size_t nmeaBuildGGA(const GpsState& s, char* out, size_t cap);
size_t nmeaBuildRMC(const GpsState& s, char* out, size_t cap);
size_t nmeaBuildVTG(const GpsState& s, char* out, size_t cap);
