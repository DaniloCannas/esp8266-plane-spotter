# ✈️ ESP8266 Plane Spotter

A tiny desktop gadget that shows the **aircraft currently flying closest to your
home** on a 0.96" OLED, plus a rotation of nerdy live statistics. Real ADS-B
data comes from the free [OpenSky Network](https://opensky-network.org/) REST
API over WiFi.

Default home location is the **centre of Pinerolo (TO), Italy** — change it in
`config.h` to point at your own roof.

---

## What it shows

The display cycles through four screens every 7 seconds (page dots in the
top-right corner show which one you're on):

1. **Nearest aircraft** — callsign, a type icon, distance + compass direction
   from home, altitude (metres and flight level), an arrow pointing in the
   aircraft's direction of travel, ground speed, and a vertical altitude gauge.
2. **Flight details** — ICAO24 hex address, country of registration, true
   track, vertical rate (climb/descent), and live lat/lon.
3. **Radar** — a North-up PPI radar with your home at the centre, range rings
   (outer ring = 120 km), a rotating sweep, and a blip for every aircraft in
   view. The nearest is highlighted and detailed in the side panel (type icon,
   callsign, distance, flight level, contact count). A small tick outside the
   ring marks the direction the wall faces (`WALL_HEADING_DEG`).
4. **Statistics** — uptime, aircraft in view (current & session max), closest
   approach this session, OpenSky request ok/error counters, WiFi signal bars +
   RSSI and free heap.

The **type icon** comes from the OpenSky emitter category (requested with
`extended=1`): airliner, light/small plane, jet, helicopter, glider, balloon,
drone, or a generic plane when the category is unknown.

Data refreshes every 60 seconds (configurable — see the rate-limit note below).

---

## Hardware

| Part | Notes |
|------|-------|
| ESP8266 board | NodeMCU v2/v3, Wemos D1 mini, or similar |
| 0.96" OLED, **SSD1306**, **4-wire SPI** | 7-pin module: `GND VCC SCK SDA RES DC CS` |

### Wiring

Hardware SPI is used, so `SCK` and `SDA` are fixed; the other three pins are
configurable in `config.h`.

| OLED pin | ESP8266 (NodeMCU label / GPIO) | Role |
|----------|-------------------------------|------|
| GND | GND | Ground |
| VCC | 3V3 | Power |
| SCK | **D5 / GPIO14** | HW SPI clock (fixed) |
| SDA | **D7 / GPIO13** | HW SPI data / MOSI (fixed) |
| RES | D0 / GPIO16 | Reset |
| DC  | D2 / GPIO4  | Data/Command |
| CS  | D1 / GPIO5  | Chip select |

> The default pins avoid the ESP8266 boot-strapping pins (GPIO0/2/15), so the
> board flashes and boots reliably.

```
        ESP8266 (NodeMCU)                OLED SSD1306 SPI
      ┌───────────────────┐            ┌──────────────────┐
      │ 3V3 ──────────────┼────────────┤ VCC              │
      │ GND ──────────────┼────────────┤ GND              │
      │ D5/GPIO14 ────────┼────────────┤ SCK              │
      │ D7/GPIO13 ────────┼────────────┤ SDA (MOSI)       │
      │ D0/GPIO16 ────────┼────────────┤ RES              │
      │ D2/GPIO4  ────────┼────────────┤ DC               │
      │ D1/GPIO5  ────────┼────────────┤ CS               │
      └───────────────────┘            └──────────────────┘
```

---

## Build & flash

### Option A — Arduino IDE

1. Install the **ESP8266 board package** (Boards Manager → "esp8266").
2. Install libraries via **Library Manager**:
   - `U8g2` by olikraus
   - `ArduinoJson` by Benoit Blanchon (v6.x)
3. Copy `firmware/plane_spotter/config.example.h` → `config.h` and fill in your
   WiFi credentials (and your coordinates, if not Pinerolo).
4. Open `firmware/plane_spotter/plane_spotter.ino`, select your board, and
   upload.

### Option B — PlatformIO

```bash
cd firmware
# copy and edit the config first:
cp plane_spotter/config.example.h plane_spotter/config.h
pio run -t upload
pio device monitor
```

`platformio.ini` already pins the board, libraries, and 115200 monitor speed.

---

## Configuration

All settings live in `config.h` (git-ignored so your WiFi password stays
private). Key options:

| Define | Meaning |
|--------|---------|
| `WIFI_SSID` / `WIFI_PASS` | Your network |
| `HOME_LAT` / `HOME_LON` | Your home coordinates (default: Pinerolo) |
| `SEARCH_RADIUS_DEG` | Half-size of the sky box to query (~1.0° ≈ 111 km) |
| `UPDATE_INTERVAL_MS` | Poll period for OpenSky |
| `WALL_HEADING_DEG` | Compass heading the wall/device faces (used by the radar wall tick) |
| `OPENSKY_CLIENT_ID` / `OPENSKY_CLIENT_SECRET` | Optional OpenSky OAuth2 client for a much bigger budget |

---

## Notes on the OpenSky API

- Anonymous access has a **small daily request budget (~400 calls)**, shared
  per public IP. Polling every 60 s is ~1440 calls/day, so anonymously you
  *will* hit `HTTP 429 Too many requests` and the display freezes on the last
  data (the firmware keeps showing the last known aircraft when rate-limited).
- **Recommended: use an OpenSky account.** Create one, go to *Account → API
  clients*, create a client, and put the `clientId`/`clientSecret` into
  `OPENSKY_CLIENT_ID`/`OPENSKY_CLIENT_SECRET`. The firmware then does the OAuth2
  `client_credentials` flow (token from `auth.opensky-network.org`, sent as a
  `Bearer` header, auto-refreshed) and gets a far larger quota — 60 s polling
  works comfortably. Anonymous (no client) still works if you raise
  `UPDATE_INTERVAL_MS` to ~300000 (5 min).
- **Aircraft-type icons:** OpenSky's emitter category (state index 17, via
  `extended=1`) is frequently `0` (unknown) in practice, so when it is missing
  the firmware estimates the type from altitude + speed and flags it with a
  leading `~` (e.g. `~Small`). Real category data, when present, is used as-is.
- Coverage is community ADS-B, so very low / very local traffic may not always
  appear. A larger `SEARCH_RADIUS_DEG` finds more planes but uses more RAM to
  parse (the ESP8266 only has ~40 KB free heap, so don't go wild).
- TLS certificate validation is skipped (`setInsecure()`) to keep memory low;
  fine for read-only public data.

---

## License

MIT — see [LICENSE](LICENSE).
