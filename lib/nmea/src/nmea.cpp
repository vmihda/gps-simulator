#include "nmea.h"

#include <math.h>
#include <stdio.h>

namespace {

constexpr long long kMinuteScale = 100000;  // 5 decimal places of minutes
constexpr float kKnotsToKmh = 1.852f;

// Appends "*XX\r\n" to a sentence body that starts with '$'
size_t finishSentence(char* out, size_t cap, int bodyLen) {
  if (bodyLen <= 0 || static_cast<size_t>(bodyLen) + 5 >= cap) return 0;
  const uint8_t ck = nmeaChecksum(out + 1);
  const int total = bodyLen + snprintf(out + bodyLen, cap - bodyLen, "*%02X\r\n", ck);
  return static_cast<size_t>(total);
}

void formatTime(const GpsState& s, char* out, size_t cap) {
  struct tm t;
  gmtime_r(&s.utcEpoch, &t);
  snprintf(out, cap, "%02d%02d%02d.%02d", t.tm_hour, t.tm_min, t.tm_sec, s.utcMillis / 10);
}

void formatDate(const GpsState& s, char* out, size_t cap) {
  struct tm t;
  gmtime_r(&s.utcEpoch, &t);
  snprintf(out, cap, "%02d%02d%02d", t.tm_mday, t.tm_mon + 1, t.tm_year % 100);
}

}  // namespace

uint8_t nmeaChecksum(const char* body) {
  uint8_t ck = 0;
  for (const char* p = body; *p != '\0' && *p != '*'; ++p) ck ^= static_cast<uint8_t>(*p);
  return ck;
}

void nmeaFormatCoord(double degrees, bool isLongitude, char* out, size_t cap) {
  const char hemisphere = isLongitude ? (degrees < 0 ? 'W' : 'E') : (degrees < 0 ? 'S' : 'N');
  // Work in integer units of 1e-5 minutes so minutes can never print as 60.00000
  const long long total = llround(fabs(degrees) * 60.0 * kMinuteScale);
  const long long deg = total / (60 * kMinuteScale);
  const long long rem = total % (60 * kMinuteScale);
  snprintf(out, cap, isLongitude ? "%03lld%02lld.%05lld,%c" : "%02lld%02lld.%05lld,%c", deg,
           rem / kMinuteScale, rem % kMinuteScale, hemisphere);
}

size_t nmeaBuildGGA(const GpsState& s, char* out, size_t cap) {
  char time[16];
  formatTime(s, time, sizeof(time));
  int n;
  if (s.hasFix) {
    char lat[16], lon[16];
    nmeaFormatCoord(s.latitude, false, lat, sizeof(lat));
    nmeaFormatCoord(s.longitude, true, lon, sizeof(lon));
    n = snprintf(out, cap, "$GPGGA,%s,%s,%s,1,%02u,%.1f,%.1f,M,0.0,M,,", time, lat, lon,
                 s.satellites, s.hdop, s.altitudeM);
  } else {
    n = snprintf(out, cap, "$GPGGA,%s,,,,,0,%02u,99.99,,M,,M,,", time, s.satellites);
  }
  return finishSentence(out, cap, n);
}

size_t nmeaBuildRMC(const GpsState& s, char* out, size_t cap) {
  char time[16], date[8];
  formatTime(s, time, sizeof(time));
  formatDate(s, date, sizeof(date));
  int n;
  if (s.hasFix) {
    char lat[16], lon[16];
    nmeaFormatCoord(s.latitude, false, lat, sizeof(lat));
    nmeaFormatCoord(s.longitude, true, lon, sizeof(lon));
    n = snprintf(out, cap, "$GPRMC,%s,A,%s,%s,%.2f,%.2f,%s,,,A", time, lat, lon, s.speedKnots,
                 s.courseDeg, date);
  } else {
    n = snprintf(out, cap, "$GPRMC,%s,V,,,,,,,%s,,,N", time, date);
  }
  return finishSentence(out, cap, n);
}

size_t nmeaBuildVTG(const GpsState& s, char* out, size_t cap) {
  int n;
  if (s.hasFix) {
    n = snprintf(out, cap, "$GPVTG,%.2f,T,,M,%.2f,N,%.2f,K,A", s.courseDeg, s.speedKnots,
                 s.speedKnots * kKnotsToKmh);
  } else {
    n = snprintf(out, cap, "$GPVTG,,T,,M,,N,,K,N");
  }
  return finishSentence(out, cap, n);
}
