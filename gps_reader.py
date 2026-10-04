"""Reads NMEA from the ESP32 GPS emulator over USB serial and logs the fix."""

import logging
import os
import time

import serial

PORT = os.environ.get("GPS_PORT", "/dev/ttyUSB0")
BAUD = 115200
BOOT_WAIT_S = 2.0    # ESP32 may reboot when the port is opened
NMEA_WAIT_S = 1.5
STALL_S = 5.0
RECONNECT_S = 3.0
LOG_PERIOD_S = 1.0   # NMEA arrives at 5 Hz, log once per second

log = logging.getLogger("gps-reader")


def checksum_ok(line):
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, tail = line[1:].partition("*")
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    try:
        return calc == int(tail[:2], 16)
    except ValueError:
        return False


def to_degrees(value, hemisphere):
    # NMEA "ddmm.mmmm" / "dddmm.mmmm" -> signed decimal degrees
    dot = value.index(".")
    degrees = int(value[:dot - 2]) + float(value[dot - 2:]) / 60
    return -degrees if hemisphere in ("S", "W") else degrees


def read_line(port):
    return port.readline().decode("ascii", errors="replace").strip()


def wait_for_nmea(port, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if checksum_ok(read_line(port)):
            return True
    return False


def ensure_echo(port):
    # NMEA reaches USB only while echo is on; 'v' toggles it, so send it only when silent
    if wait_for_nmea(port, NMEA_WAIT_S):
        return True
    port.write(b"v")
    return wait_for_nmea(port, NMEA_WAIT_S)


def configure(port):
    time.sleep(BOOT_WAIT_S)
    port.reset_input_buffer()
    port.write(b"3")  # 3D fix mode
    if not ensure_echo(port):
        raise RuntimeError("ESP32 does not send NMEA over USB")
    log.info("GPS configured: 3D fix mode, NMEA echo on")


def log_gga(f):
    # $GPGGA,time,lat,N,lon,E,quality,sats,hdop,alt,M,...
    if len(f) < 10:
        return
    if f[6] == "0" or not f[2]:
        log.info("NO FIX sats=%s", f[7])
        return
    log.info("FIX lat=%.6f lon=%.6f alt=%sm sats=%s hdop=%s utc=%s",
             to_degrees(f[2], f[3]), to_degrees(f[4], f[5]), f[9], f[7], f[8], f[1])


def stream(port):
    mode = "3D FIX"
    last_nmea = time.monotonic()
    last_log = 0.0
    while True:
        line = read_line(port)
        now = time.monotonic()
        if line.startswith("[gps-sim]"):
            log.info("esp32: %s", line)
            if "mode:" in line:
                mode = line.split("mode:", 1)[1].strip()
            continue
        if not checksum_ok(line):
            # Silence outside "NO GPS" mode means the ESP32 rebooted and lost the echo flag
            if now - last_nmea > STALL_S and not mode.startswith("NO GPS"):
                log.warning("no NMEA for %.0f s, re-enabling echo", STALL_S)
                if not ensure_echo(port):
                    raise RuntimeError("NMEA stream lost")
                last_nmea = time.monotonic()
            continue
        last_nmea = now
        fields = line.split("*", 1)[0].split(",")
        if fields[0].endswith("GGA") and now - last_log >= LOG_PERIOD_S:
            last_log = now
            log_gga(fields)


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    while True:
        try:
            with serial.Serial(PORT, BAUD, timeout=1) as port:
                log.info("opened %s @ %d", PORT, BAUD)
                configure(port)
                stream(port)
        except (serial.SerialException, OSError, RuntimeError) as exc:
            log.warning("%s; retry in %.0f s", exc, RECONNECT_S)
            time.sleep(RECONNECT_S)


if __name__ == "__main__":
    main()