/*
 * Configuration template.
 *
 *   1. Copy this file to "config.h" (same folder).
 *   2. Fill in your WiFi credentials and home coordinates.
 *   3. config.h is git-ignored so your secrets never get committed.
 */
#pragma once

// ---- WiFi ----------------------------------------------------------------
#define WIFI_SSID  "YOUR_WIFI_NAME"
#define WIFI_PASS  "YOUR_WIFI_PASSWORD"

// ---- Home location -------------------------------------------------------
// Default: centre of Pinerolo (TO), Italy.
#define HOME_LAT   44.8848
#define HOME_LON   7.3306

// Half-size of the search box in degrees (~1.0 deg latitude is ~111 km).
// Smaller = less data to parse on the ESP8266, larger = wider sky coverage
// (but more RAM used; on a quiet sky raise it, if it reboots lower it).
#define SEARCH_RADIUS_DEG  1.0

// How often to poll OpenSky, in milliseconds. Anonymous access has a small
// daily budget (~400 calls): 60 s already exceeds it over 24 h, so for
// continuous use set OPENSKY_USER/PASS below (a free account = far more calls).
#define UPDATE_INTERVAL_MS  60000

// Include aircraft that OpenSky reports as on-ground (parked/taxiing).
// Set to false if you live near an airport and do not want parked planes
// to win "nearest aircraft" or appear as radar contacts.
#define INCLUDE_ON_GROUND  false

// Compass heading (deg, 0=N 90=E 180=S 270=W) that the wall / device faces.
// Used by the radar screen to mark the wall direction.
#define WALL_HEADING_DEG  194

// POSIX timezone string for the NTP clock (default: Europe/Rome). See
// https://github.com/nayarsystems/posix_tz_db for other zones.
#define TIMEZONE  "CET-1CEST,M3.5.0,M10.5.0/3"

// How often to refresh weather from Open-Meteo (ms). 10 min is plenty.
#define WEATHER_INTERVAL_MS  600000

// ---- OpenSky OAuth2 client (optional) ------------------------------------
// Leave both empty for anonymous access (small ~400 calls/day budget). For a
// much larger quota, create a free account at https://opensky-network.org/,
// then Account -> API clients -> create a client, and paste the resulting
// clientId / clientSecret here. The firmware fetches an OAuth2 token and sends
// it as a Bearer header.
#define OPENSKY_CLIENT_ID      ""
#define OPENSKY_CLIENT_SECRET  ""

// ---- OLED display ----------------------------------------------------------
// Two SSD1306 128x64 module types are supported; pick yours:
//   false = 7-pin 4-wire SPI module (GND VCC SCK SDA RES DC CS)
//   true  = 4-pin I2C module        (GND VCC SCL SDA)
#define OLED_USE_I2C  false

// I2C pins (only used when OLED_USE_I2C is true). Defaults are the ESP8266
// standard I2C pins:
#define PIN_OLED_SCL  5    // D1
#define PIN_OLED_SDA  4    // D2

// SPI control pins (only used when OLED_USE_I2C is false). SCK -> GPIO14 (D5)
// and SDA/MOSI -> GPIO13 (D7) are fixed by HW SPI; these three are
// configurable:
#define PIN_OLED_RST  16   // D0
#define PIN_OLED_DC    4   // D2
#define PIN_OLED_CS    5   // D1
