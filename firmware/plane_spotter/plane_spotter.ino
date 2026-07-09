/*
 * ESP8266 Plane Spotter
 * --------------------------------------------------------------------------
 * Shows the aircraft currently flying closest to your home on a 0.96" SSD1306
 * OLED, plus a bunch of nerdy statistics. Live ADS-B data is pulled from
 * the free OpenSky Network REST API.
 *
 * Board   : any ESP8266 (NodeMCU v2/v3, Wemos D1 mini, ...)
 * Display : 0.96" OLED, SSD1306 128x64, either
 *           - 4-wire SPI (7 pins: GND, VCC, SCK, SDA, RES, DC, CS), or
 *           - I2C (4 pins: GND, VCC, SCL, SDA)
 *           Pick with OLED_USE_I2C in config.h.
 *
 * Wiring (defaults, see README for the full tables):
 *   SPI OLED  ESP8266                    I2C OLED  ESP8266
 *   GND  -->  GND                        GND  -->  GND
 *   VCC  -->  3V3                        VCC  -->  3V3
 *   SCK  -->  D5 / GPIO14 (HW, fixed)    SCL  -->  D1 / GPIO5
 *   SDA  -->  D7 / GPIO13 (HW, fixed)    SDA  -->  D2 / GPIO4
 *   RES  -->  D0 / GPIO16
 *   DC   -->  D2 / GPIO4
 *   CS   -->  D1 / GPIO5
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
#include <time.h>
#include <sys/time.h>

#include "config.h"

#ifndef INCLUDE_ON_GROUND
#define INCLUDE_ON_GROUND true
#endif

// ---------------------------------------------------------------------------
// Display: SSD1306 128x64, 4-wire hardware SPI or hardware I2C, selected by
// OLED_USE_I2C in config.h.
// SPI: HW SPI uses the fixed ESP8266 pins SCLK=GPIO14 (D5) and MOSI=GPIO13
//   (D7); only CS / DC / RESET are configurable.
// I2C: SCL/SDA pins come from config.h (defaults: SCL=GPIO5/D1, SDA=GPIO4/D2,
//   the ESP8266 standard I2C pins). Most modules answer at address 0x3C,
//   U8g2's default; if yours is 0x3D, add u8g2.setI2CAddress(0x3D * 2)
//   before begin().
// ---------------------------------------------------------------------------
#ifndef OLED_USE_I2C
#define OLED_USE_I2C false   // config.h files from before this switch are SPI
#endif

#if OLED_USE_I2C
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE,
                                         /* clock=*/ PIN_OLED_SCL,
                                         /* data=*/  PIN_OLED_SDA);
#else
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, PIN_OLED_CS, PIN_OLED_DC,
                                            PIN_OLED_RST);
#endif

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
  int    category;    // OpenSky emitter category (state index 17, 0 = unknown)
  double distanceKm;  // great-circle distance from home
  double bearingDeg;  // bearing from home to aircraft
  bool   valid;
};

Aircraft nearest;

// Per-aircraft state for the radar, including enough to dead-reckon (estimate)
// the position between data refreshes so the blips creep in real time.
struct Blip { double lat; double lon; float track; float speedMs; };
const uint8_t MAX_BLIPS = 20;
Blip     blips[MAX_BLIPS];
uint8_t  blipCount  = 0;
uint32_t lastDataMs = 0;   // millis() of the last successful aircraft fetch

// OpenSky OAuth2 bearer token (when client credentials are configured).
String   accessToken;
uint32_t tokenExpiryMs = 0;

// Current weather (from Open-Meteo).
struct Weather {
  float tempC;
  float windKmh;
  int   humidity;
  int   code;      // WMO weather code
  bool  valid = false;
} weather;
uint32_t lastWeatherPoll = 0;

// Short hourly forecast (a few hours ahead).
struct Fcast { int hour; float tempC; int code; };
const uint8_t FC_N = 3;
Fcast   fcast[FC_N];
uint8_t fcCount = 0;

// Route / airline / ETA for the current nearest aircraft (from hexdb.io).
struct RouteInfo {
  char   callsign[10];   // which callsign this data is for
  char   airline[18];
  char   dep[6];
  char   arr[6];
  bool   haveRoute;
  bool   haveArrPos;
  double arrLat, arrLon;
} routeInfo;

// Runtime statistics (the nerdy bit)
struct Stats {
  uint32_t requestsOk   = 0;
  uint32_t requestsFail = 0;
  uint16_t inView       = 0;   // aircraft inside the search box on last poll
  uint16_t maxInView    = 0;   // session record
  double   closestEver  = 1e9; // closest distance seen this session (km)
  uint32_t lastUpdateMs = 0;
} stats;

uint8_t  screen           = 0;         // which screen is showing
uint32_t lastScreenSwap   = 0;
uint32_t lastPoll         = 0;
bool     firstFetchDone   = false;
bool     firstWeatherDone = false;

const uint8_t  NUM_SCREENS    = 5;
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
  url += "&extended=1";   // include the aircraft category (state index 17)
  return url;
}

// Whether OAuth2 client credentials are configured.
bool oauthConfigured() { return strlen(OPENSKY_CLIENT_ID) > 0; }

// Request a fresh OAuth2 access token (client_credentials grant). Returns true
// on success and stores it in `accessToken` with an expiry a minute early.
bool fetchToken() {
  WiFiClientSecure client;
  client.setInsecure();
  client.setBufferSizes(16384, 512);

  HTTPClient https;
  https.setReuse(false);
  if (!https.begin(client,
        "https://auth.opensky-network.org/auth/realms/opensky-network/protocol/openid-connect/token")) {
    Serial.println("[auth] begin() failed");
    return false;
  }
  https.addHeader("Content-Type", "application/x-www-form-urlencoded");
  String body = "grant_type=client_credentials&client_id=";
  body += OPENSKY_CLIENT_ID;
  body += "&client_secret=";
  body += OPENSKY_CLIENT_SECRET;

  int code = https.POST(body);
  Serial.printf("[auth] token HTTP %d\n", code);
  if (code != HTTP_CODE_OK) { https.end(); return false; }

  String payload = https.getString();
  https.end();

  JsonDocument filter;
  filter["access_token"] = true;
  filter["expires_in"]   = true;
  JsonDocument doc;
  if (deserializeJson(doc, payload, DeserializationOption::Filter(filter))) {
    Serial.println("[auth] token JSON parse failed");
    return false;
  }
  const char* tok = doc["access_token"] | "";
  if (!tok[0]) return false;

  accessToken = tok;
  int exp = doc["expires_in"] | 1800;
  tokenExpiryMs = millis() + (uint32_t)(exp > 120 ? exp - 60 : exp) * 1000UL;
  Serial.printf("[auth] token ok (len=%u, expires in %ds)\n", accessToken.length(), exp);
  return true;
}

// Pulls aircraft states, keeps the nearest one. Returns true on success.
bool fetchAircraft() {
  // Refresh the OAuth2 token first, before the data client exists, so we never
  // hold two 16 KB TLS buffers at once.
  if (oauthConfigured() &&
      (accessToken.length() == 0 || (int32_t)(millis() - tokenExpiryMs) >= 0)) {
    fetchToken();
  }

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

  if (oauthConfigured() && accessToken.length() > 0) {
    https.addHeader("Authorization", "Bearer " + accessToken);
  }

  int code = https.GET();
  Serial.printf("[fetch] HTTP %d\n", code);
  if (code != HTTP_CODE_OK) {
    if (code == HTTP_CODE_UNAUTHORIZED) accessToken = "";  // force token refresh
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
  for (int i = 0; i <= 17; i++) el[i] = false;
  el[0] = el[1] = el[2] = el[5] = el[6] = true;
  el[7] = el[8] = el[9] = el[10] = el[11] = el[13] = el[17] = true;

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
  blipCount = 0;

  for (JsonArray s : states) {
    if (s.isNull() || s[5].isNull() || s[6].isNull()) continue;
    bool onGround = s[8] | false;
    if (onGround && !INCLUDE_ON_GROUND) continue;

    double lon = s[5].as<double>();
    double lat = s[6].as<double>();
    double d   = haversineKm(HOME_LAT, HOME_LON, lat, lon);
    double brg = bearingDeg(HOME_LAT, HOME_LON, lat, lon);
    count++;

    if (blipCount < MAX_BLIPS) {
      blips[blipCount].lat     = lat;
      blips[blipCount].lon     = lon;
      blips[blipCount].track   = s[10] | 0.0f;
      blips[blipCount].speedMs = onGround ? 0.0f : (s[9] | 0.0f);
      blipCount++;
    }

    if (d < best.distanceKm) {
      best.distanceKm = d;
      best.lat        = lat;
      best.lon        = lon;
      best.bearingDeg = brg;
      best.onGround   = onGround;
      best.category   = s[17] | 0;

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
  lastDataMs         = millis();

  nearest = best;
  if (nearest.valid && nearest.distanceKm < stats.closestEver)
    stats.closestEver = nearest.distanceKm;

  Serial.printf("[fetch] inView=%u blips=%u nearest=%s cat=%d dist=%.1fkm valid=%d heap=%u\n",
                count, blipCount, nearest.valid ? nearest.callsign : "-",
                nearest.valid ? nearest.category : -1,
                nearest.valid ? nearest.distanceKm : 0.0, nearest.valid,
                ESP.getFreeHeap());
  return true;
}

// ---------------------------------------------------------------------------
// Weather (Open-Meteo, no API key required)
// ---------------------------------------------------------------------------
bool fetchWeather() {
  WiFiClientSecure client;
  client.setInsecure();
  client.setBufferSizes(16384, 512);

  HTTPClient https;
  https.setReuse(false);
  String url = "https://api.open-meteo.com/v1/forecast?latitude=";
  url += String(HOME_LAT, 4);
  url += "&longitude=" + String(HOME_LON, 4);
  url += "&current=temperature_2m,relative_humidity_2m,weather_code,wind_speed_10m";
  url += "&hourly=temperature_2m,weather_code&forecast_hours=8&timezone=auto";
  if (!https.begin(client, url)) return false;

  int code = https.GET();
  Serial.printf("[wx] HTTP %d\n", code);
  if (code != HTTP_CODE_OK) { https.end(); return false; }
  String payload = https.getString();
  https.end();

  JsonDocument filter;
  filter["current"] = true;
  filter["hourly"]  = true;
  JsonDocument doc;
  if (deserializeJson(doc, payload, DeserializationOption::Filter(filter))) return false;
  JsonObject c = doc["current"];
  if (c.isNull()) return false;

  weather.tempC    = c["temperature_2m"]       | 0.0f;
  weather.humidity = c["relative_humidity_2m"] | 0;
  weather.code     = c["weather_code"]         | 0;
  weather.windKmh  = c["wind_speed_10m"]       | 0.0f;
  weather.valid    = true;

  // short forecast: a few hours ahead (index 0 of forecast_hours == now)
  JsonArray ht = doc["hourly"]["temperature_2m"].as<JsonArray>();
  JsonArray hc = doc["hourly"]["weather_code"].as<JsonArray>();
  int nowH = -1;
  if (timeReady()) {
    time_t t = time(nullptr);
    struct tm lt;
    localtime_r(&t, &lt);
    nowH = lt.tm_hour;
  }
  const int offs[FC_N] = {2, 4, 6};
  fcCount = 0;
  for (uint8_t k = 0; k < FC_N; k++) {
    int idx = offs[k];
    if ((int)ht.size() > idx) {
      fcast[fcCount].tempC = ht[idx] | 0.0f;
      fcast[fcCount].code  = hc[idx] | 0;
      fcast[fcCount].hour  = (nowH < 0) ? -1 : ((nowH + idx) % 24);
      fcCount++;
    }
  }

  Serial.printf("[wx] %.1fC hum=%d%% wind=%.0f code=%d fc=%u\n",
                weather.tempC, weather.humidity, weather.windKmh, weather.code, fcCount);
  return true;
}

// Short label and icon-kind (0 sun,1 part,2 cloud,3 fog,4 rain,5 snow,6 storm)
// for a WMO weather code.
const char* wxText(int code) {
  if (code == 0)                  return "CLEAR";
  if (code <= 2)                  return "PARTLY";
  if (code == 3)                  return "OVERCAST";
  if (code == 45 || code == 48)   return "FOG";
  if (code >= 51 && code <= 57)   return "DRIZZLE";
  if (code >= 61 && code <= 67)   return "RAIN";
  if (code >= 71 && code <= 77)   return "SNOW";
  if (code >= 80 && code <= 82)   return "SHOWERS";
  if (code >= 85 && code <= 86)   return "SNOW";
  if (code >= 95)                 return "STORM";
  return "WX";
}
int wxKind(int code) {
  if (code == 0)                                       return 0;
  if (code <= 2)                                       return 1;
  if (code == 3)                                       return 2;
  if (code == 45 || code == 48)                        return 3;
  if ((code >= 51 && code <= 67) || (code >= 80 && code <= 82)) return 4;
  if ((code >= 71 && code <= 77) || (code >= 85 && code <= 86)) return 5;
  if (code >= 95)                                      return 6;
  return 2;
}

// ---------------------------------------------------------------------------
// Route / airline / ETA (hexdb.io, free, no key)
// ---------------------------------------------------------------------------
// Generic HTTPS GET into a String. Returns true on HTTP 200.
bool httpGetString(const String& url, String& out) {
  WiFiClientSecure client;
  client.setInsecure();
  client.setBufferSizes(16384, 512);
  HTTPClient https;
  https.setReuse(false);
  if (!https.begin(client, url)) return false;
  int code = https.GET();
  if (code != HTTP_CODE_OK) { https.end(); return false; }
  out = https.getString();
  https.end();
  return true;
}

// Offline ICAO airline-designator table (first 3 letters of the callsign).
struct Airline { const char* code; const char* name; };
const Airline AIRLINES[] = {
  {"RYR","Ryanair"},   {"EJU","easyJet EU"},{"EZY","easyJet"},   {"WZZ","Wizz Air"},
  {"VOE","Volotea"},   {"VLG","Vueling"},   {"ITY","ITA Airways"},{"AZA","Alitalia"},
  {"AFR","Air France"},{"DLH","Lufthansa"}, {"BAW","British AW"},{"KLM","KLM"},
  {"IBE","Iberia"},    {"SWR","SWISS"},     {"AUA","Austrian"},  {"TAP","TAP Air"},
  {"SAS","SAS"},       {"FIN","Finnair"},   {"LOT","LOT Polish"},{"THY","Turkish"},
  {"UAE","Emirates"},  {"QTR","Qatar"},     {"ETD","Etihad"},    {"ELY","El Al"},
  {"AEE","Aegean"},    {"TRA","Transavia"}, {"NAX","Norwegian"}, {"EWG","Eurowings"},
  {"BEL","Brussels"},  {"TVF","Transavia"}, {"ENT","Enter Air"}, {"DAL","Delta"},
  {"UAL","United"},    {"AAL","American"},  {"ACA","Air Canada"},{"UPS","UPS"},
  {"FDX","FedEx"},     {"BCS","DHL Air"},   {"MSR","EgyptAir"},  {"RAM","Royal Air Maroc"},
  {"AZU","Azul"},      {"QFA","Qantas"},    {"SIA","Singapore"}, {"NJE","NetJets"},
  {"EXS","Jet2"},      {"WUK","Wizz UK"},   {"NSZ","Norse"},     {"MMZ","euroAtlantic"},
};

const char* airlineName(const char* callsign) {
  static char fb[4];
  if (!callsign || strlen(callsign) < 3) return "GA / Private";
  char p[4] = { (char)toupper(callsign[0]), (char)toupper(callsign[1]),
                (char)toupper(callsign[2]), 0 };
  for (auto& a : AIRLINES) if (strcmp(a.code, p) == 0) return a.name;
  strcpy(fb, p);
  return fb;   // unknown -> show the 3-letter operator code
}

// Look up departure/arrival airports (and arrival coords for ETA) for a
// callsign. Always fills the airline; route/ETA are best-effort.
void fetchRoute(const char* callsign) {
  strncpy(routeInfo.airline, airlineName(callsign), sizeof(routeInfo.airline) - 1);
  routeInfo.airline[sizeof(routeInfo.airline) - 1] = '\0';
  routeInfo.haveRoute = false;
  routeInfo.haveArrPos = false;
  routeInfo.dep[0] = routeInfo.arr[0] = '\0';
  strncpy(routeInfo.callsign, callsign, sizeof(routeInfo.callsign) - 1);
  routeInfo.callsign[sizeof(routeInfo.callsign) - 1] = '\0';

  String payload;
  if (httpGetString(String("https://hexdb.io/api/v1/route/icao/") + callsign, payload)) {
    JsonDocument d;
    if (!deserializeJson(d, payload)) {
      const char* r = d["route"] | "";
      const char* dash = strchr(r, '-');
      if (r[0] && dash) {
        size_t dl = dash - r;
        if (dl < sizeof(routeInfo.dep)) {
          strncpy(routeInfo.dep, r, dl);
          routeInfo.dep[dl] = '\0';
          strncpy(routeInfo.arr, dash + 1, sizeof(routeInfo.arr) - 1);
          routeInfo.arr[sizeof(routeInfo.arr) - 1] = '\0';
          routeInfo.haveRoute = true;
        }
      }
    }
  }

  if (routeInfo.haveRoute && routeInfo.arr[0]) {
    String ap;
    if (httpGetString(String("https://hexdb.io/api/v1/airport/icao/") + routeInfo.arr, ap)) {
      JsonDocument d;
      if (!deserializeJson(d, ap) && !d["latitude"].isNull()) {
        routeInfo.arrLat = d["latitude"]  | 0.0;
        routeInfo.arrLon = d["longitude"] | 0.0;
        routeInfo.haveArrPos = true;
      }
    }
  }

  Serial.printf("[route] %s %s %s>%s eta=%s\n", callsign, routeInfo.airline,
                routeInfo.haveRoute ? routeInfo.dep : "?",
                routeInfo.haveRoute ? routeInfo.arr : "?",
                routeInfo.haveArrPos ? "yes" : "no");
}

// ---------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------
bool timeReady() { return time(nullptr) > 1700000000; }

void fmtClock(char* buf, size_t n, bool withSecs) {
  if (!timeReady()) { strncpy(buf, withSecs ? "--:--:--" : "--:--", n); return; }
  time_t t = time(nullptr);
  struct tm lt;
  localtime_r(&t, &lt);
  strftime(buf, n, withSecs ? "%H:%M:%S" : "%H:%M", &lt);
}

// Forward great-circle position: move (lat,lon) by distM metres along trackDeg.
void projectLatLon(double lat, double lon, float trackDeg, double distM,
                   double& outLat, double& outLon) {
  double dr = distM / 6371000.0;
  double b  = deg2rad(trackDeg);
  double la = deg2rad(lat), lo = deg2rad(lon);
  double nla = asin(sin(la) * cos(dr) + cos(la) * sin(dr) * cos(b));
  double nlo = lo + atan2(sin(b) * sin(dr) * cos(la), cos(dr) - sin(la) * sin(nla));
  outLat = rad2deg(nla);
  outLon = rad2deg(nlo);
}

// ---------------------------------------------------------------------------
// Drawing helpers
// ---------------------------------------------------------------------------
// Tactical header: title + screen index on the left, NTP clock on the right.
// The header lives in the top band; on two-color OLEDs (yellow top 16 rows)
// the body content below is laid out to start at row 16+, in the second color.
void drawHeader(const char* title) {
  u8g2.setFont(u8g2_font_5x7_tr);
  char left[20];
  snprintf(left, sizeof(left), "%s %d/%d", title, screen + 1, NUM_SCREENS);
  u8g2.drawStr(2, 6, left);

  char t[10];
  fmtClock(t, sizeof(t), true);
  u8g2.drawStr(127 - u8g2.getStrWidth(t), 6, t);

  u8g2.drawHLine(0, 8, 128);
  u8g2.drawVLine(0, 0, 3);     // HUD corner ticks
  u8g2.drawVLine(127, 0, 3);
}

// Filled cloud silhouette, left edge near (cx-7), vertically around cy.
void drawCloud(int cx, int cy) {
  u8g2.drawDisc(cx - 5, cy, 4);
  u8g2.drawDisc(cx + 1, cy - 3, 5);
  u8g2.drawDisc(cx + 6, cy, 4);
  u8g2.drawBox(cx - 5, cy, 12, 5);
}

void drawWeatherIcon(int cx, int cy, int kind) {
  switch (kind) {
    case 0: { // sun
      u8g2.drawDisc(cx, cy, 5);
      for (int a = 0; a < 360; a += 45) {
        double r = deg2rad(a);
        u8g2.drawLine(cx + (int)(cos(r) * 7), cy + (int)(sin(r) * 7),
                      cx + (int)(cos(r) * 9), cy + (int)(sin(r) * 9));
      }
      break;
    }
    case 1: // partly cloudy
      u8g2.drawDisc(cx - 3, cy - 4, 4);
      drawCloud(cx + 2, cy + 2);
      break;
    case 3: // fog
      drawCloud(cx, cy - 2);
      for (int i = 0; i < 3; i++) u8g2.drawHLine(cx - 7, cy + 5 + i * 2, 15);
      break;
    case 4: // rain
      drawCloud(cx, cy - 2);
      for (int i = -4; i <= 6; i += 5) u8g2.drawLine(cx + i, cy + 4, cx + i - 2, cy + 8);
      break;
    case 5: // snow
      drawCloud(cx, cy - 2);
      for (int i = -4; i <= 6; i += 5) {
        u8g2.drawPixel(cx + i, cy + 6);
        u8g2.drawHLine(cx + i - 1, cy + 6, 3);
        u8g2.drawVLine(cx + i, cy + 5, 3);
      }
      break;
    case 6: // storm
      drawCloud(cx, cy - 2);
      u8g2.drawLine(cx, cy + 4, cx - 3, cy + 7);
      u8g2.drawLine(cx - 3, cy + 7, cx + 1, cy + 7);
      u8g2.drawLine(cx + 1, cy + 7, cx - 2, cy + 10);
      break;
    default: // overcast / generic cloud
      drawCloud(cx, cy);
      break;
  }
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

// Short label for an OpenSky emitter category.
const char* typeName(int cat) {
  switch (cat) {
    case 2:  return "Light";
    case 3:  return "Small";
    case 4:  return "Airliner";
    case 5:  return "Heavy";
    case 6:  return "Heavy";
    case 7:  return "Jet";
    case 8:  return "Heli";
    case 9:  return "Glider";
    case 10: return "Balloon";
    case 14: return "Drone";
    default: return "Aircraft";
  }
}

// Icon (~16x12) for an aircraft type, centred at (cx,cy).
void drawTypeIcon(int cx, int cy, int cat) {
  switch (cat) {
    case 8:  // helicopter
      u8g2.drawDisc(cx - 1, cy, 2);
      u8g2.drawHLine(cx - 7, cy - 3, 15);          // main rotor
      u8g2.drawLine(cx + 1, cy, cx + 7, cy + 1);   // tail boom
      u8g2.drawVLine(cx + 7, cy - 2, 5);           // tail rotor
      break;
    case 9:  // glider (long slim wings)
      u8g2.drawHLine(cx - 8, cy, 17);
      u8g2.drawVLine(cx, cy - 2, 7);
      u8g2.drawHLine(cx - 2, cy + 5, 5);
      break;
    case 10: // balloon / lighter-than-air
      u8g2.drawCircle(cx, cy - 2, 4);
      u8g2.drawLine(cx - 3, cy + 1, cx - 1, cy + 5);
      u8g2.drawLine(cx + 3, cy + 1, cx + 1, cy + 5);
      u8g2.drawFrame(cx - 1, cy + 5, 3, 2);
      break;
    case 14: // drone (quadcopter)
      u8g2.drawBox(cx - 1, cy - 1, 3, 3);
      u8g2.drawLine(cx - 5, cy - 4, cx + 5, cy + 4);
      u8g2.drawLine(cx + 5, cy - 4, cx - 5, cy + 4);
      u8g2.drawCircle(cx - 5, cy - 4, 2);
      u8g2.drawCircle(cx + 5, cy - 4, 2);
      u8g2.drawCircle(cx - 5, cy + 4, 2);
      u8g2.drawCircle(cx + 5, cy + 4, 2);
      break;
    case 2:
    case 3:  // light / small plane (straight wings)
      u8g2.drawVLine(cx, cy - 4, 10);
      u8g2.drawHLine(cx - 5, cy - 1, 11);
      u8g2.drawHLine(cx - 2, cy + 4, 5);
      break;
    default: // airliner / generic (swept wings, top view)
      u8g2.drawVLine(cx, cy - 5, 12);
      u8g2.drawTriangle(cx, cy - 1, cx - 7, cy + 3, cx - 1, cy + 1);
      u8g2.drawTriangle(cx, cy - 1, cx + 7, cy + 3, cx + 1, cy + 1);
      u8g2.drawTriangle(cx, cy + 4, cx - 3, cy + 6, cx - 1, cy + 5);
      u8g2.drawTriangle(cx, cy + 4, cx + 3, cy + 6, cx + 1, cy + 5);
      break;
  }
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

// OpenSky often leaves the emitter category at 0 (unknown). When that happens
// we make a rough guess from altitude + ground speed so the icon still varies.
// Real category data always wins. Estimated types are flagged with '~' on screen.
int effectiveCategory(const Aircraft& a) {
  if (a.category > 0) return a.category;     // real data
  if (a.onGround)     return 0;
  float kmh = a.velocityMs * 3.6f;
  float m   = a.altitudeM;
  if (kmh < 120 && m < 2200) return 8;       // slow & low -> guess helicopter
  if (kmh < 300 && m < 5000) return 3;       // medium      -> guess small plane
  return 4;                                  // fast / high -> airliner
}
bool isEstimatedType(const Aircraft& a) { return a.category == 0 && !a.onGround; }

void screenNearest() {
  drawHeader("TARGET");

  if (!nearest.valid) {
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(0, 34, "NO TARGET");
    u8g2.drawStr(0, 48, "in range.");
    return;
  }

  u8g2.setFont(u8g2_font_7x14B_tr);
  u8g2.drawStr(0, 28, nearest.callsign);

  // aircraft-type icon, between the callsign and the heading arrow
  drawTypeIcon(82, 22, effectiveCategory(nearest));

  u8g2.setFont(u8g2_font_6x12_tr);
  char line[24];
  snprintf(line, sizeof(line), "%.1f km %s", nearest.distanceKm, compass(nearest.bearingDeg));
  u8g2.drawStr(0, 44, line);

  if (nearest.onGround) {
    u8g2.drawStr(0, 58, "on ground");
  } else {
    snprintf(line, sizeof(line), "%.0f m / FL%03.0f",
             nearest.altitudeM, nearest.altitudeM * 3.28084 / 100.0);
    u8g2.drawStr(0, 58, line);
  }

  // heading arrow + speed on the right
  drawArrow(110, 38, 11, nearest.trackDeg);
  snprintf(line, sizeof(line), "%.0f", nearest.velocityMs * 3.6); // km/h
  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.drawStr(98, 56, line);
  u8g2.drawStr(98, 63, "km/h");

  // altitude gauge on the far-right column (0..FL400)
  const int gT = 18, gB = 54;
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
  drawHeader("INTEL");

  if (!nearest.valid) {
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(0, 34, "NO INTEL");
    return;
  }

  bool haveRoute = (strcmp(routeInfo.callsign, nearest.callsign) == 0);
  char line[32];

  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 24, nearest.callsign);

  u8g2.setFont(u8g2_font_5x7_tr);
  snprintf(line, sizeof(line), "LINE %s", haveRoute ? routeInfo.airline : airlineName(nearest.callsign));
  u8g2.drawStr(0, 35, line);

  if (haveRoute && routeInfo.haveRoute)
    snprintf(line, sizeof(line), "RTE  %s > %s", routeInfo.dep, routeInfo.arr);
  else
    snprintf(line, sizeof(line), "RTE  unknown");
  u8g2.drawStr(0, 45, line);

  if (haveRoute && routeInfo.haveArrPos && nearest.velocityMs > 20) {
    double dk  = haversineKm(nearest.lat, nearest.lon, routeInfo.arrLat, routeInfo.arrLon);
    int    min = (int)(dk / (nearest.velocityMs * 3.6) * 60.0);
    snprintf(line, sizeof(line), "ETA  %dh%02dm  %.0fkm", min / 60, min % 60, dk);
  } else {
    snprintf(line, sizeof(line), "ETA  --");
  }
  u8g2.drawStr(0, 54, line);

  snprintf(line, sizeof(line), "ID %s HDG %03.0f", nearest.icao24, nearest.trackDeg);
  u8g2.drawStr(0, 63, line);
}

// North-up radar (PPI). Home at the centre, range rings (outer = 120 km), a
// rotating sweep, and a blip per aircraft. Blips are dead-reckoned from their
// last track+speed so they creep in real time between data refreshes, and use
// radar persistence: bright just after the sweep passes, then a faint dot.
void screenRadar() {
  drawHeader("RADAR");

  const int cx = 31, cy = 39, R = 23;
  const float MAX_KM = 120.0f;
  float elapsed = (millis() - lastDataMs) / 1000.0f;   // s since last fetch

  // rings + axes
  u8g2.drawCircle(cx, cy, R);
  u8g2.drawCircle(cx, cy, (R * 2) / 3);
  u8g2.drawCircle(cx, cy, R / 3);
  u8g2.drawHLine(cx - R, cy, 2 * R + 1);
  u8g2.drawVLine(cx, cy - R, 2 * R + 1);
  u8g2.drawDisc(cx, cy, 1);

  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.drawStr(cx - 1, cy - R - 1, "N");

  // wall-facing tick just outside the ring
  {
    double a = deg2rad((double)WALL_HEADING_DEG);
    u8g2.drawDisc(cx + (int)(sin(a) * (R + 2)), cy - (int)(cos(a) * (R + 2)), 1);
  }

  // rotating sweep (~4 s/turn, clockwise)
  float sweepDeg = fmodf(millis() / 12.0f, 360.0f);
  double sw = deg2rad(sweepDeg);
  u8g2.drawLine(cx, cy, cx + (int)(sin(sw) * R), cy - (int)(cos(sw) * R));

  // blips, dead-reckoned + persistence
  int   nearIdx = -1;
  float nearD   = 1e9;
  for (uint8_t i = 0; i < blipCount; i++) {
    double la = blips[i].lat, lo = blips[i].lon;
    if (blips[i].speedMs > 0 && elapsed > 0)
      projectLatLon(blips[i].lat, blips[i].lon, blips[i].track,
                    blips[i].speedMs * elapsed, la, lo);
    double dist = haversineKm(HOME_LAT, HOME_LON, la, lo);
    if (dist < nearD) { nearD = dist; nearIdx = i; }

    float fr = (float)(dist / MAX_KM);
    if (fr > 1) continue;
    int rr  = (int)(fr * R);
    double brg = bearingDeg(HOME_LAT, HOME_LON, la, lo);
    int bx = cx + (int)(sin(deg2rad(brg)) * rr);
    int by = cy - (int)(cos(deg2rad(brg)) * rr);

    float behind = fmodf(sweepDeg - (float)brg + 360.0f, 360.0f);
    if (behind < 50) u8g2.drawDisc(bx, by, 1);   // freshly swept
    else             u8g2.drawPixel(bx, by);     // fading
  }

  // highlight the closest live contact
  if (nearIdx >= 0) {
    double la = blips[nearIdx].lat, lo = blips[nearIdx].lon;
    if (blips[nearIdx].speedMs > 0 && elapsed > 0)
      projectLatLon(blips[nearIdx].lat, blips[nearIdx].lon, blips[nearIdx].track,
                    blips[nearIdx].speedMs * elapsed, la, lo);
    double dist = haversineKm(HOME_LAT, HOME_LON, la, lo);
    double brg  = bearingDeg(HOME_LAT, HOME_LON, la, lo);
    float fr = (float)(dist / MAX_KM);
    if (fr <= 1) {
      int rr = (int)(fr * R);
      int bx = cx + (int)(sin(deg2rad(brg)) * rr);
      int by = cy - (int)(cos(deg2rad(brg)) * rr);
      u8g2.drawCircle(bx, by, 3);
      u8g2.drawDisc(bx, by, 1);
    }
  }

  // side info panel
  const int px = 62;
  if (!nearest.valid) {
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(px, 36, "NO CONTACT");
    return;
  }

  int ec = effectiveCategory(nearest);
  drawTypeIcon(px + 7, 23, ec);
  u8g2.setFont(u8g2_font_5x7_tr);
  char tname[12];
  snprintf(tname, sizeof(tname), "%s%s", isEstimatedType(nearest) ? "~" : "", typeName(ec));
  u8g2.drawStr(px + 18, 24, tname);
  u8g2.drawStr(px, 36, nearest.callsign);

  char l[20];
  snprintf(l, sizeof(l), "RNG %.0fkm", nearest.distanceKm);
  u8g2.drawStr(px, 47, l);
  snprintf(l, sizeof(l), "BRG %03.0f %s", nearest.bearingDeg, compass(nearest.bearingDeg));
  u8g2.drawStr(px, 57, l);

  u8g2.setFont(u8g2_font_4x6_tr);
  snprintf(l, sizeof(l), "%u CONTACTS", blipCount);
  u8g2.drawStr(px, 63, l);
}

void screenWeather() {
  drawHeader("WX");

  if (!weather.valid) {
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(18, 38, "no wx data");
    return;
  }

  drawWeatherIcon(18, 30, wxKind(weather.code));

  // big temperature, with a real degree glyph aligned to the baseline
  char t[8];
  snprintf(t, sizeof(t), "%.0f", weather.tempC);
  u8g2.setFont(u8g2_font_logisoso16_tn);
  u8g2.drawStr(40, 34, t);
  int w = u8g2.getStrWidth(t);
  u8g2.setFont(u8g2_font_9x15_tf);
  u8g2.drawStr(40 + w + 2, 34, "\xB0" "C");

  // condition + current humidity/wind
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(40, 45, wxText(weather.code));
  u8g2.setFont(u8g2_font_4x6_tr);
  char l[26];
  snprintf(l, sizeof(l), "HUM %d%%  WIND %.0fkm/h", weather.humidity, weather.windKmh);
  u8g2.drawStr(2, 53, l);

  // minimal next-hours forecast strip
  u8g2.drawHLine(0, 55, 128);
  int fx = 2;
  for (uint8_t k = 0; k < fcCount; k++) {
    char fb[12];
    if (fcast[k].hour >= 0) snprintf(fb, sizeof(fb), "%02dh %.0fc", fcast[k].hour, fcast[k].tempC);
    else                    snprintf(fb, sizeof(fb), "+%dh %.0fc", (k + 1) * 2, fcast[k].tempC);
    u8g2.drawStr(fx, 63, fb);
    fx += 44;
  }
}

void screenSystem() {
  drawHeader("SYSTEM");

  // big clock HH:MM:SS
  char clk[12];
  fmtClock(clk, sizeof(clk), true);
  u8g2.setFont(u8g2_font_logisoso16_tn);
  int w = u8g2.getStrWidth(clk);
  int x0 = 4;
  u8g2.drawStr(x0, 34, clk);

  // fast-updating milliseconds
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  char msb[6];
  snprintf(msb, sizeof(msb), ".%03d", (int)(tv.tv_usec / 1000));
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(x0 + w + 2, 34, msb);

  // date
  u8g2.setFont(u8g2_font_5x7_tr);
  char dl[24];
  if (timeReady()) {
    time_t tt = time(nullptr);
    struct tm lt;
    localtime_r(&tt, &lt);
    strftime(dl, sizeof(dl), "%a %Y-%m-%d", &lt);
  } else {
    strcpy(dl, "SYNCING NTP...");
  }
  u8g2.drawStr(2, 46, dl);

  // uptime + target count
  uint32_t up = millis() / 1000;
  char l[30];
  snprintf(l, sizeof(l), "UP %02lu:%02lu:%02lu   TGT %u",
           up / 3600, (up % 3600) / 60, up % 60, stats.inView);
  u8g2.drawStr(2, 55, l);

  // link status: signal bars + details
  drawSignalBars(2, 63, WiFi.RSSI());
  u8g2.setFont(u8g2_font_4x6_tr);
  snprintf(l, sizeof(l), "%ddBm RAM%dk REQ%lu/%lu",
           WiFi.RSSI(), ESP.getFreeHeap() / 1024, stats.requestsOk, stats.requestsFail);
  u8g2.drawStr(18, 63, l);
}

void render() {
  u8g2.clearBuffer();
  switch (screen) {
    case 0: screenNearest(); break;
    case 1: screenDetails(); break;
    case 2: screenRadar();   break;
    case 3: screenWeather(); break;
    case 4: screenSystem();  break;
  }
  u8g2.sendBuffer();
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
#if OLED_USE_I2C
  u8g2.setBusClock(400000);   // 400 kHz I2C: a full frame in ~25 ms, needed
                              // for the ~30 fps radar sweep (100 kHz manages
                              // only ~10 fps). SSD1306 modules handle it fine.
#endif
  u8g2.begin();
  u8g2.setContrast(180);

  // splash
  u8g2.clearBuffer();
  u8g2.drawFrame(0, 0, 128, 64);
  u8g2.setFont(u8g2_font_7x14B_tr);
  u8g2.drawStr(8, 28, "PLANE SPOTTER");
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(14, 46, "TACTICAL ADS-B v2");
  u8g2.sendBuffer();
  delay(1500);

  connectWiFi();

  // NTP time (timezone from config). Non-blocking; screens show --:-- until set.
  configTime(TIMEZONE, "pool.ntp.org", "time.google.com");

  nearest.valid = false;
}

void loop() {
  uint32_t now = millis();

  if (!firstFetchDone || now - lastPoll >= UPDATE_INTERVAL_MS) {
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    fetchAircraft();
    if (nearest.valid && strcmp(routeInfo.callsign, nearest.callsign) != 0)
      fetchRoute(nearest.callsign);
    lastPoll = now;
    firstFetchDone = true;
  }

  if (!firstWeatherDone || now - lastWeatherPoll >= WEATHER_INTERVAL_MS) {
    if (WiFi.status() == WL_CONNECTED) fetchWeather();
    lastWeatherPoll = now;
    firstWeatherDone = true;
  }

  if (now - lastScreenSwap >= SCREEN_SWAP_MS) {
    screen = (screen + 1) % NUM_SCREENS;
    lastScreenSwap = now;
  }

  render();
  delay(33);   // ~30 fps: smooth radar sweep + fast-ticking clock
}
