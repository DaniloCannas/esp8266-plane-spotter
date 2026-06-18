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

// Compass heading (deg, 0=N 90=E 180=S 270=W) that the wall / device faces.
// Used by the 3D SKY VIEW screen to place the aircraft relative to the wall.
#define WALL_HEADING_DEG  194

// ---- OpenSky account (optional) ------------------------------------------
// Leave both empty for anonymous access (lower rate limit). A free account
// at https://opensky-network.org/ gives you many more daily requests.
#define OPENSKY_USER  ""
#define OPENSKY_PASS  ""

// ---- OLED wiring (hardware SPI) ------------------------------------------
// SCK -> GPIO14 (D5) and SDA/MOSI -> GPIO13 (D7) are fixed by HW SPI.
// These three are configurable:
#define PIN_OLED_RST  16   // D0
#define PIN_OLED_DC    4   // D2
#define PIN_OLED_CS    5   // D1
