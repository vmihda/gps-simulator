#include <unity.h>
#include <stdio.h>
#include <string.h>

#include "nmea.h"

static GpsState makeFix() {
  GpsState s{};
  s.hasFix = true;
  s.latitude = 48.1173;   // 48°07.038' N
  s.longitude = 11.5166667;  // 11°31.000' E
  s.altitudeM = 545.4f;
  s.satellites = 8;
  s.hdop = 0.9f;
  s.speedKnots = 0.0f;
  s.courseDeg = 0.0f;
  s.utcEpoch = 1790942400;  // 2026-10-02 12:00:00 UTC
  s.utcMillis = 500;
  return s;
}

// Recomputes the checksum of a full sentence and compares it with the "*XX" suffix
static bool checksumMatches(const char* sentence) {
  const char* star = strchr(sentence, '*');
  if (sentence[0] != '$' || star == nullptr) return false;
  uint8_t ck = 0;
  for (const char* p = sentence + 1; p < star; ++p) ck ^= static_cast<uint8_t>(*p);
  char expected[3];
  snprintf(expected, sizeof(expected), "%02X", ck);
  return strncmp(star + 1, expected, 2) == 0 && strcmp(star + 3, "\r\n") == 0;
}

void test_checksum_matches_reference_sentence() {
  // Classic reference sentence: $GPGGA,123519,...*47
  TEST_ASSERT_EQUAL_HEX8(0x47, nmeaChecksum("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"));
}

void test_latitude_is_formatted_as_ddmm() {
  char buf[16];
  nmeaFormatCoord(48.1173, false, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING("4807.03800,N", buf);
}

void test_longitude_is_formatted_as_dddmm_with_hemisphere() {
  char buf[16];
  nmeaFormatCoord(-11.5166667, true, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING("01131.00000,W", buf);
}

void test_minutes_never_round_up_to_sixty() {
  char buf[16];
  nmeaFormatCoord(50.9999999999, false, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING("5100.00000,N", buf);
}

void test_gga_with_fix_reports_quality_1() {
  char buf[128];
  GpsState s = makeFix();
  size_t n = nmeaBuildGGA(s, buf, sizeof(buf));
  TEST_ASSERT_EQUAL(strlen(buf), n);
  TEST_ASSERT_TRUE(checksumMatches(buf));
  TEST_ASSERT_NOT_NULL(strstr(buf, "$GPGGA,120000.50,4807.03800,N,01131.00000,E,1,08,0.9,545.4,M,"));
}

void test_gga_without_fix_reports_quality_0() {
  char buf[128];
  GpsState s = makeFix();
  s.hasFix = false;
  s.satellites = 2;
  nmeaBuildGGA(s, buf, sizeof(buf));
  TEST_ASSERT_TRUE(checksumMatches(buf));
  TEST_ASSERT_NOT_NULL(strstr(buf, "$GPGGA,120000.50,,,,,0,02,"));
}

void test_rmc_with_fix_is_active_and_has_date() {
  char buf[128];
  GpsState s = makeFix();
  nmeaBuildRMC(s, buf, sizeof(buf));
  TEST_ASSERT_TRUE(checksumMatches(buf));
  TEST_ASSERT_NOT_NULL(strstr(buf, "$GPRMC,120000.50,A,4807.03800,N,01131.00000,E,0.00,0.00,021026,,,A*"));
}

void test_rmc_without_fix_is_void() {
  char buf[128];
  GpsState s = makeFix();
  s.hasFix = false;
  nmeaBuildRMC(s, buf, sizeof(buf));
  TEST_ASSERT_TRUE(checksumMatches(buf));
  TEST_ASSERT_NOT_NULL(strstr(buf, "$GPRMC,120000.50,V,,,,,,,021026,,,N*"));
}

void test_vtg_reports_speed_in_knots_and_kmh() {
  char buf[128];
  GpsState s = makeFix();
  s.speedKnots = 10.0f;
  s.courseDeg = 90.0f;
  nmeaBuildVTG(s, buf, sizeof(buf));
  TEST_ASSERT_TRUE(checksumMatches(buf));
  TEST_ASSERT_NOT_NULL(strstr(buf, "$GPVTG,90.00,T,,M,10.00,N,18.52,K,A*"));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_checksum_matches_reference_sentence);
  RUN_TEST(test_latitude_is_formatted_as_ddmm);
  RUN_TEST(test_longitude_is_formatted_as_dddmm_with_hemisphere);
  RUN_TEST(test_minutes_never_round_up_to_sixty);
  RUN_TEST(test_gga_with_fix_reports_quality_1);
  RUN_TEST(test_gga_without_fix_reports_quality_0);
  RUN_TEST(test_rmc_with_fix_is_active_and_has_date);
  RUN_TEST(test_rmc_without_fix_is_void);
  RUN_TEST(test_vtg_reports_speed_in_knots_and_kmh);
  return UNITY_END();
}
