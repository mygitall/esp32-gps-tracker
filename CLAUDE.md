# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

ESP32 GPS tracker with ATGM336H GPS + Air780EX 4G LTE Cat.1 module. Reports real-time location, battery voltage, and signal quality to a cloud server. Web map (Leaflet.js) for real-time tracking.

Two repos:
- **Firmware**: `esp32-gps-tracker` (this repo) — Arduino .ino sketches
- **Server**: `esp32` repo at `/Users/zhangweiwei/Desktop/AI开发/esp32/` — PHP + HTML

## Build & flash commands

```bash
# Compile
arduino-cli compile --fqbn esp32:esp32:esp32 full_firmware_4g/full_firmware_4g.ino

# USB flash
arduino-cli upload --fqbn esp32:esp32:esp32 -p /dev/cu.usbserial-0001 full_firmware_4g/full_firmware_4g.ino

# OTA flash (WiFi required, device at esp32-gps.local)
python3 ~/Library/Arduino15/packages/esp32/hardware/esp32/3.3.8/tools/espota.py \
  -i 192.168.31.141 -p 3232 -a "12345678" -f <binary>.bin

# FTP upload server files
python3 -c "
import ftplib
ftp = ftplib.FTP(); ftp.connect('gao367888125.zfkwp02.guaixing.cn', 21, timeout=15)
ftp.login('gao367888125', 'c6f57i8j')
# ... ftp.storbinary('STOR /esp32/mmq/<file>', ...)
"
```

## Pin mapping

| ESP32 GPIO | Connected to |
|-----------|-------------|
| GPIO4 (AIR_RX) | Air780EX TX |
| GPIO5 (AIR_TX) | Air780EX RX |
| GPIO16 (GPS_RX) | ATGM336H TX |
| GPIO17 (GPS_TX) | ATGM336H RX |
| GPIO27 (AIR_PWR) | S8050 base → PWRKEY control |
| GPIO2 | Status LED |

- Air780EX baud: 115200
- ATGM336H baud: 9600

## Architecture

**Data flow**: GPS → ESP32 → Air780EX 4G → HTTP GET/POST → receiver.php → MySQL → api.php → map.html

**Firmware** (`full_firmware_4g/full_firmware_4g.ino`, current v7.1):
- GPS: TinyGPS++ parses NMEA from Serial2
- 4G: Raw AT commands over Serial1 to Air780EX for HTTP/MQTT over TCP
- HTTP batch: GPS points buffered, flushed every 10s as GET `?batch=lat,lng,...|lat,lng,...`
- Coordinates use `double` (64-bit); `float` loses sub-1m precision
- MQTT disabled as of v7.0 (broker.emqx.io unreachable from China Unicom)
- PDP kept alive by checking CIFSR before touching CSTT/CIICR
- PWRKEY auto-on via S8050 transistor; AT probe before toggle avoids shutting off already-on module

**Server** (`/Users/zhangweiwei/Desktop/AI开发/esp32/`):
- `receiver.php`: data ingest. Supports GET single + GET batch + POST JSON batch
- `api.php`: query API. `?latest=1` for latest point, `?from=...&to=...` for range
- `subscribe_daemon.php`: MQTT subscriber for backup data path (cron mode)
- `config.php`: DB & MQTT config
- `map.html`: Leaflet + Gaode (Amap) tiles. HTTP polling every 3s + MQTT WebSocket fallback

## Key gotchas

- GPS TX wire (ATGM336H → GPIO16) is prone to coming loose — first thing to check when GPS shows `wait...`
- Air780EX `AT+CBC` returns voltage in mV directly (not comma-separated)
- CSTT/CIICR returns `+CME ERROR: 3` when PDP already active — not a real error, skip them
- `delay(50)` at end of loop() is needed; removing it floods Serial1 and crashes AT communication
- Battery percentage uses 20mV resolution lipoPct() with 1%/s smooth ramp
- HTTP URL uses `rssi` field for battery percentage (legacy naming)
- ESP32 DevKit needs 5V input; single 18650 (3.7-4.2V) needs boost converter
- `esp_task_wdt` API changed in ESP32 Arduino 3.x — uses `esp_task_wdt_config_t` struct
