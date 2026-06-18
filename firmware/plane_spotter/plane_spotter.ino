/*
 * ESP8266 Plane Spotter
 * --------------------------------------------------------------------------
 * Shows the aircraft currently flying closest to your home on a 0.96" SSD1306
 * SPI OLED, plus a bunch of nerdy statistics. Live ADS-B data is pulled from
 * the free OpenSky Network REST API.
 *
 * Board   : any ESP8266 (NodeMCU v2/v3, Wemos D1 mini, ...)
 * Display : 0.96" OLED 4-wire SPI, SSD1306 128x64 (7 pins:
 *           GND, VCC, SCK, SDA, RES, DC, CS)
 *
 * Wiring (default, see README for the full table):
 *   OLED      ESP8266 (NodeMCU label / GPIO)
 *   GND  -->  GND
 *   VCC  -->  3V3
 *   SCK  -->  D5  / GPIO14   (HW SPI SCLK, fixed)
 *   SDA  -->  D7  / GPIO13   (HW SPI MOSI, fixed)
 *   RES  -->  D0  / GPIO16
 *   DC   -->  D2  / GPIO4
 *   CS   -->  D1  / GPIO5
 *
 * Libraries (install from the Arduino Library Manager):
 *   - U8g2        by olikraus
 *   - ArduinoJson by Benoit Blanchon (v6.x)
 *
 * Copy config.example.h to config.h and fill in your details before flashing.
 */

#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <U8g2lib.h>
#include <math.h>

#include "config.h"

// ---------------------------------------------------------------------------
// Display: SSD1306 128x64, 4-wire hardware SPI.
// HW SPI uses the fixed ESP8266 pins SCLK=GPIO14 (D5) and MOSI=GPIO13 (D7);
// only CS / DC / RESET are configurable here.
// ---------------------------------------------------------------------------
#define OLED_CS   PIN_OLED_CS
#define OLED_DC   PIN_OLED_DC
#define OLED_RST  PIN_OLED_RST

U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, OLED_CS, OLED_DC, OLED_RST);

// ---------------------------------------------------------------------------
// Data model
// ---------------------------------------------------------------------------
struct Aircraft {
  char   icao24[8];
  char   callsign[10];
  char   country[24];
  double lat;
  double lon;
  float  altitudeM;   // geometric/barometric altitude in metres
  float  velocityMs;  // ground speed in m/s
  float  trackDeg;    // true track over ground (0 = north)
  float  vrateMs;     // vertical rate in m/s (+climb / -descent)
  bool   onGround;
  double distanceKm;  // great-circle distance from home
  double bearingDeg;  // bearing from home to aircraft
  bool   valid;
};

Aircraft nearest;

// Runtime statistics (the nerdy bit)
struct Stats {
  uint32_t requestsOk   = 0;
  uint32_t requestsFail = 0;
  uint16_t inView       = 0;   // aircraft inside the search box on last poll
  uint16_t maxInView    = 0;   // session record
  double   closestEver  = 1e9; // closest distance seen this session (km)
  uint32_t lastUpdateMs = 0;
} stats;

uint8_t  screen          = 0;          // which screen is showing
uint32_t lastScreenSwap  = 0;
uint32_t lastPoll        = 0;
bool     firstFetchDone  = false;

const uint8_t  NUM_SCREENS    = 4;
const uint32_t SCREEN_SWAP_MS = 7000;

// ---------------------------------------------------------------------------
// Geo helpers
// ---------------------------------------------------------------------------
static double deg2rad(double d) { return d * (PI / 180.0); }
static double rad2deg(double r) { return r * (180.0 / PI); }

// Great-circle distance (Haversine) in kilometres.
double haversineKm(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371.0;
  double dLat = deg2rad(lat2 - lat1);
  double dLon = deg2rad(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
             sin(dLon / 2) * sin(dLon / 2);
  return R * 2 * atan2(sqrt(a), sqrt(1 - a));
}

// Initial bearing from point 1 to point 2, degrees 0..360.
double bearingDeg(double lat1, double lon1, double lat2, double lon2) {
  double y = sin(deg2rad(lon2 - lon1)) * cos(deg2rad(lat2));
  double x = cos(deg2rad(lat1)) * sin(deg2rad(lat2)) -
             sin(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(lon2 - lon1));
  double b = rad2deg(atan2(y, x));
  return fmod(b + 360.0, 360.0);
}

const char* compass(double bearing) {
  static const char* dirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  return dirs[(int)((bearing + 22.5) / 45.0) % 8];
}

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint8_t dots = 0;
  while (WiFi.status() != WL_CONNECTED) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(0, 12, "Plane Spotter");
    u8g2.drawStr(0, 30, "Connecting WiFi");
    u8g2.setCursor(0, 46);
    u8g2.print(WIFI_SSID);
    u8g2.setCursor(0, 62);
    for (uint8_t i = 0; i < (dots % 16) + 1; i++) u8g2.print('.');
    u8g2.sendBuffer();
    delay(400);
    dots++;
  }
  Serial.printf("\n[wifi] connected SSID=%s IP=%s RSSI=%d\n",
                WIFI_SSID, WiFi.localIP().toString().c_str(), WiFi.RSSI());
}

// ---------------------------------------------------------------------------
// OpenSky fetch
// ---------------------------------------------------------------------------
String buildUrl() {
  double lamin = HOME_LAT - SEARCH_RADIUS_DEG;
  double lamax = HOME_LAT + SEARCH_RADIUS_DEG;
  double lomin = HOME_LON - SEARCH_RADIUS_DEG;
  double lomax = HOME_LON + SEARCH_RADIUS_DEG;

  String url = "https://opensky-network.org/api/states/all?";
  url += "lamin=" + String(lamin, 4);
  url += "&lomin=" + String(lomin, 4);
  url += "&lamax=" + String(lamax, 4);
  url += "&lomax=" + String(lomax, 4);
  return url;
}

// Pulls aircraft states, keeps the nearest one. Returns true on success.
bool fetchAircraft() {
  WiFiClientSecure client;
  client.setInsecure();                 // skip cert validation (read-only data)
  // Do NOT shrink the RX buffer below the default 16 KB: OpenSky does not
  // negotiate a smaller TLS fragment (no MFLN), so a small RX buffer makes the
  // TLS handshake fail and every fetch silently returns "no aircraft".
  client.setBufferSizes(16384, 512);

  HTTPClient https;
  https.setReuse(false);
  String url = buildUrl();
  Serial.printf("[fetch] heap=%u GET %s\n", ESP.getFreeHeap(), url.c_str());
  if (!https.begin(client, url)) {
    Serial.println("[fetch] https.begin() failed");
    stats.requestsFail++;
    return false;
  }

  if (strlen(OPENSKY_USER) > 0) {
    https.setAuthorization(OPENSKY_USER, OPENSKY_PASS);
  }

  int code = https.GET();
  Serial.printf("[fetch] HTTP %d\n", code);
  if (code != HTTP_CODE_OK) {
    https.end();
    stats.requestsFail++;
    return false;
  }

  // Filter: keep only the fields we actually use from every state vector.
  // OpenSky state indices: 0 icao24, 1 callsign, 2 origin_country,
  // 5 longitude, 6 latitude, 7 baro_altitude, 8 on_ground, 9 velocity,
  // 10 true_track, 11 vertical_rate, 13 geo_altitude.
  JsonDocument filter;
  JsonArray el = filter["states"].to<JsonArray>().add<JsonArray>();
  for (int i = 0; i <= 13; i++) el[i] = false;
  el[0] = el[1] = el[2] = el[5] = el[6] = true;
  el[7] = el[8] = el[9] = el[10] = el[11] = el[13] = true;

  // OpenSky replies with Transfer-Encoding: chunked. getStream() would hand the
  // raw chunked bytes (hex length markers) to the parser and yield nothing, so
  // we use getString(), which de-chunks the body before we parse it.
  String payload = https.getString();
  https.end();
  Serial.printf("[fetch] payload=%u bytes\n", payload.length());

  JsonDocument doc;
  DeserializationError err = deserializeJson(
      doc, payload, DeserializationOption::Filter(filter));

  if (err) {
    Serial.printf("[fetch] JSON error: %s\n", err.c_str());
    stats.requestsFail++;
    return false;
  }

  JsonArray states = doc["states"].as<JsonArray>();
  Aircraft best;
  best.valid      = false;
  best.distanceKm = 1e9;
  uint16_t count  = 0;

  for (JsonArray s : states) {
    if (s.isNull() || s[5].isNull() || s[6].isNull()) continue;
    double lon = s[5].as<double>();
    double lat = s[6].as<double>();
    double d   = haversineKm(HOME_LAT, HOME_LON, lat, lon);
    count++;

    if (d < best.distanceKm) {
      best.distanceKm = d;
      best.lat        = lat;
      best.lon        = lon;
      best.bearingDeg = bearingDeg(HOME_LAT, HOME_LON, lat, lon);
      best.onGround   = s[8] | false;

      // geo altitude (13) preferred, fall back to barometric (7)
      best.altitudeM  = s[13].isNull() ? (s[7] | 0.0f) : s[13].as<float>();
      best.velocityMs = s[9]  | 0.0f;
      best.trackDeg   = s[10] | 0.0f;
      best.vrateMs    = s[11] | 0.0f;

      const char* cs = s[1] | "";
      strncpy(best.callsign, cs, sizeof(best.callsign) - 1);
      best.callsign[sizeof(best.callsign) - 1] = '\0';
      // trim trailing spaces OpenSky pads callsigns with
      for (int i = strlen(best.callsign) - 1; i >= 0 && best.callsign[i] == ' '; i--)
        best.callsign[i] = '\0';
      if (best.callsign[0] == '\0') strcpy(best.callsign, "(no id)");

      const char* ic = s[0] | "";
      strncpy(best.icao24, ic, sizeof(best.icao24) - 1);
      best.icao24[sizeof(best.icao24) - 1] = '\0';

      const char* co = s[2] | "?";
      strncpy(best.country, co, sizeof(best.country) - 1);
      best.country[sizeof(best.country) - 1] = '\0';

      best.valid = true;
    }
  }

  stats.inView       = count;
  if (count > stats.maxInView) stats.maxInView = count;
  stats.requestsOk++;
  stats.lastUpdateMs = millis();

  nearest = best;
  if (nearest.valid && nearest.distanceKm < stats.closestEver)
    stats.closestEver = nearest.distanceKm;

  Serial.printf("[fetch] inView=%u nearest=%s dist=%.1fkm valid=%d heap=%u\n",
                count, nearest.valid ? nearest.callsign : "-",
                nearest.valid ? nearest.distanceKm : 0.0, nearest.valid,
                ESP.getFreeHeap());
  return true;
}

// ---------------------------------------------------------------------------
// Drawing helpers
// ---------------------------------------------------------------------------
void drawHeader(const char* title) {
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(0, 6, title);
  u8g2.drawHLine(0, 8, 128);
}

// Arrow pointing toward `angle` (0 = up/north), centred at (cx,cy).
void drawArrow(int cx, int cy, int r, double angleDeg) {
  double a = deg2rad(angleDeg);
  // tip
  int tx = cx + (int)(sin(a) * r);
  int ty = cy - (int)(cos(a) * r);
  // tail
  int bx = cx - (int)(sin(a) * r);
  int by = cy + (int)(cos(a) * r);
  u8g2.drawLine(bx, by, tx, ty);
  // arrow head
  double left  = a + deg2rad(150);
  double right = a - deg2rad(150);
  u8g2.drawLine(tx, ty, tx + (int)(sin(left)  * (r / 2)), ty - (int)(cos(left)  * (r / 2)));
  u8g2.drawLine(tx, ty, tx + (int)(sin(right) * (r / 2)), ty - (int)(cos(right) * (r / 2)));
}

// Small top-view plane silhouette centred at (cx,cy).
void drawPlaneTop(int cx, int cy) {
  u8g2.drawVLine(cx, cy - 3, 8);      // fuselage
  u8g2.drawHLine(cx - 4, cy - 1, 9);  // main wings
  u8g2.drawHLine(cx - 2, cy + 3, 5);  // tailplane
}

// WiFi signal bars (0..4), bottom-aligned at baseline y, growing right.
void drawSignalBars(int x, int y, int rssi) {
  int bars = 0;
  if (rssi >= -55)      bars = 4;
  else if (rssi >= -65) bars = 3;
  else if (rssi >= -75) bars = 2;
  else if (rssi >= -85) bars = 1;
  for (int i = 0; i < 4; i++) {
    int h = 2 + i * 2;
    if (i < bars) u8g2.drawBox(x + i * 3, y - h, 2, h);
    else          u8g2.drawFrame(x + i * 3, y - h, 2, h);
  }
}

// Page indicator dots in the top-right corner.
void drawPageDots(int current) {
  int x0 = 128 - NUM_SCREENS * 5 - 1;
  for (int i = 0; i < NUM_SCREENS; i++) {
    int cx = x0 + i * 5;
    if (i == current) u8g2.drawBox(cx, 1, 3, 3);
    else              u8g2.drawFrame(cx, 1, 3, 3);
  }
}

void screenNearest() {
  drawHeader("NEAREST AIRCRAFT");

  if (!nearest.valid) {
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(0, 30, "No aircraft in");
    u8g2.drawStr(0, 44, "range right now.");
    return;
  }

  u8g2.setFont(u8g2_font_7x14B_tr);
  u8g2.drawStr(0, 24, nearest.callsign);

  u8g2.setFont(u8g2_font_6x12_tr);
  char line[24];
  snprintf(line, sizeof(line), "%.1f km %s", nearest.distanceKm, compass(nearest.bearingDeg));
  u8g2.drawStr(0, 40, line);

  if (nearest.onGround) {
    u8g2.drawStr(0, 54, "on ground");
  } else {
    snprintf(line, sizeof(line), "%.0f m / FL%03.0f",
             nearest.altitudeM, nearest.altitudeM * 3.28084 / 100.0);
    u8g2.drawStr(0, 54, line);
  }

  // heading arrow + speed on the right
  drawArrow(110, 34, 11, nearest.trackDeg);
  snprintf(line, sizeof(line), "%.0f", nearest.velocityMs * 3.6); // km/h
  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.drawStr(98, 56, line);
  u8g2.drawStr(98, 63, "km/h");

  // altitude gauge on the far-right column (0..FL400)
  const int gT = 14, gB = 50;
  u8g2.drawFrame(125, gT, 3, gB - gT);
  if (!nearest.onGround) {
    float fl = nearest.altitudeM * 3.28084f / 100.0f;
    float fr = fl / 400.0f;
    if (fr > 1) fr = 1;
    if (fr < 0) fr = 0;
    int fh = (int)((gB - gT - 2) * fr);
    u8g2.drawBox(126, gB - 1 - fh, 1, fh);
  }
}

void screenDetails() {
  drawHeader("FLIGHT DETAILS");
  u8g2.setFont(u8g2_font_5x7_tr);

  if (!nearest.valid) {
    u8g2.drawStr(0, 30, "Waiting for data...");
    return;
  }

  char line[32];
  snprintf(line, sizeof(line), "ICAO24 : %s", nearest.icao24);
  u8g2.drawStr(0, 20, line);
  snprintf(line, sizeof(line), "From   : %s", nearest.country);
  u8g2.drawStr(0, 30, line);
  snprintf(line, sizeof(line), "Track  : %.0f deg %s", nearest.trackDeg, compass(nearest.trackDeg));
  u8g2.drawStr(0, 40, line);
  snprintf(line, sizeof(line), "V/rate : %+.1f m/s", nearest.vrateMs);
  u8g2.drawStr(0, 50, line);
  snprintf(line, sizeof(line), "Pos %.3f,%.3f", nearest.lat, nearest.lon);
  u8g2.drawStr(0, 60, line);
}

// Pseudo-3D view of where the aircraft sits relative to the wall the device is
// mounted on. The wall faces WALL_HEADING_DEG; we draw a perspective floor and
// place the plane by relative azimuth (left/right), distance (depth) and
// elevation (height, with a stem + ground shadow for depth).
void screen3DWall() {
  drawHeader("3D SKY VIEW");

  // perspective floor trapezoid
  const int nearY = 60, farY = 27;
  const int nLx = 6,  nRx = 122;   // near (front, wide) edge
  const int fLx = 47, fRx = 81;    // far (horizon, narrow) edge
  u8g2.drawLine(nLx, nearY, fLx, farY);
  u8g2.drawLine(nRx, nearY, fRx, farY);
  u8g2.drawLine(fLx, farY, fRx, farY);
  for (int k = 1; k <= 3; k++) {            // depth lines
    float d = k / 4.0f;
    int lx = nLx + (int)((fLx - nLx) * d);
    int rx = nRx + (int)((fRx - nRx) * d);
    int yy = nearY + (int)((farY - nearY) * d);
    u8g2.drawLine(lx, yy, rx, yy);
  }
  for (int k = 1; k <= 4; k++) {            // azimuth lines
    float u = k / 5.0f;
    int nx = nLx + (int)((nRx - nLx) * u);
    int fx = fLx + (int)((fRx - fLx) * u);
    u8g2.drawLine(nx, nearY, fx, farY);
  }

  // the wall + the device on it + heading label
  u8g2.drawBox(nLx, nearY + 1, nRx - nLx, 2);
  int devx = (nLx + nRx) / 2;
  u8g2.drawBox(devx - 1, nearY - 1, 3, 3);
  u8g2.setFont(u8g2_font_4x6_tr);
  char wl[12];
  snprintf(wl, sizeof(wl), "WALL %d %s", (int)WALL_HEADING_DEG, compass((double)WALL_HEADING_DEG));
  u8g2.drawStr(2, 63, wl);

  if (!nearest.valid) {
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(36, 18, "waiting...");
    return;
  }

  // relative azimuth: 0 = straight out from wall, + = right, - = left
  double relAz = nearest.bearingDeg - (double)WALL_HEADING_DEG;
  while (relAz > 180)   relAz -= 360;
  while (relAz <= -180) relAz += 360;
  bool behind = fabs(relAz) > 90.0;

  double azC = relAz;
  if (azC > 90)  azC = 90;
  if (azC < -90) azC = -90;
  float u = (float)((azC + 90.0) / 180.0);          // 0 left .. 1 right
  float dd = (float)(nearest.distanceKm / 120.0);   // 0 near .. 1 far
  if (dd > 1) dd = 1;
  if (dd < 0) dd = 0;

  int lx = nLx + (int)((fLx - nLx) * dd);
  int rx = nRx + (int)((fRx - nRx) * dd);
  int gy = nearY + (int)((farY - nearY) * dd);
  int gx = lx + (int)((rx - lx) * u);

  double horizM = nearest.distanceKm * 1000.0;
  double elev = rad2deg(atan2((double)nearest.altitudeM, horizM > 1 ? horizM : 1));
  int lift = (int)((elev / 60.0) * 30.0 * (1.0 - 0.4 * dd));
  if (lift > 32) lift = 32;
  if (lift < 2)  lift = 2;
  int py = gy - lift;
  if (py < farY - 3) py = farY - 3;

  u8g2.drawEllipse(gx, gy, 3, 1);   // ground shadow
  u8g2.drawLine(gx, gy, gx, py);    // height stem
  drawPlaneTop(gx, py);

  // info line in the sky region
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(2, 18, nearest.callsign);
  char info[16];
  snprintf(info, sizeof(info), "%s%d EL%d", relAz >= 0 ? "R" : "L", (int)fabs(relAz), (int)elev);
  u8g2.drawStr(126 - u8g2.getStrWidth(info), 18, info);
  if (behind) {
    u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.drawStr(40, 25, "(behind wall)");
  }
}

void screenStats() {
  drawHeader("STATISTICS");
  u8g2.setFont(u8g2_font_5x7_tr);

  char line[32];
  uint32_t up = millis() / 1000;
  snprintf(line, sizeof(line), "Uptime : %02lu:%02lu:%02lu",
           (up / 3600), (up % 3600) / 60, up % 60);
  u8g2.drawStr(0, 20, line);

  snprintf(line, sizeof(line), "In view: %u (max %u)", stats.inView, stats.maxInView);
  u8g2.drawStr(0, 30, line);

  if (stats.closestEver < 1e8)
    snprintf(line, sizeof(line), "Closest: %.1f km", stats.closestEver);
  else
    snprintf(line, sizeof(line), "Closest: --");
  u8g2.drawStr(0, 40, line);

  snprintf(line, sizeof(line), "Req ok/err: %lu/%lu", stats.requestsOk, stats.requestsFail);
  u8g2.drawStr(0, 50, line);

  drawSignalBars(0, 60, WiFi.RSSI());
  snprintf(line, sizeof(line), "%ddBm  heap %dk",
           WiFi.RSSI(), ESP.getFreeHeap() / 1024);
  u8g2.drawStr(16, 60, line);
}

void render() {
  u8g2.clearBuffer();
  switch (screen) {
    case 0: screenNearest(); break;
    case 1: screenDetails(); break;
    case 2: screen3DWall();  break;
    case 3: screenStats();   break;
  }
  drawPageDots(screen);
  u8g2.sendBuffer();
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  u8g2.begin();
  u8g2.setContrast(180);

  // splash
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_7x14B_tr);
  u8g2.drawStr(8, 28, "PLANE SPOTTER");
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(20, 46, "ESP8266 + OLED");
  u8g2.sendBuffer();
  delay(1500);

  connectWiFi();
  nearest.valid = false;
}

void loop() {
  uint32_t now = millis();

  if (!firstFetchDone || now - lastPoll >= UPDATE_INTERVAL_MS) {
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    fetchAircraft();
    lastPoll = now;
    firstFetchDone = true;
  }

  if (now - lastScreenSwap >= SCREEN_SWAP_MS) {
    screen = (screen + 1) % NUM_SCREENS;
    lastScreenSwap = now;
  }

  render();
  delay(50);
}
