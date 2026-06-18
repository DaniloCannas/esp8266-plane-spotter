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

1. **Nearest aircraft** — callsign, distance + compass direction from home,
   altitude (metres and flight level), a little arrow pointing in the aircraft's
   direction of travel, ground speed, and a vertical altitude gauge.
2. **Flight details** — ICAO24 hex address, country of registration, true
   track, vertical rate (climb/descent), and live lat/lon.
3. **3D sky view** — a pseudo-3D perspective of where the aircraft is relative
   to the **wall the device is mounted on**. Set `WALL_HEADING_DEG` to the
   compass heading the wall faces; the plane is then placed by relative azimuth
   (left/right of the wall), distance (depth into the scene) and elevation
   (height above the floor, drawn with a stem and ground shadow). `Rxx`/`Lxx` is
   the angle right/left of the wall, `ELxx` the elevation angle; planes behind
   the wall are flagged `(behind wall)`.
4. **Statistics** — uptime, aircraft in view (current & session max), closest
   approach this session, OpenSky request ok/error counters, WiFi signal bars +
   RSSI and free heap.

Data refreshes every 30 seconds (configurable).

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
| `WALL_HEADING_DEG` | Compass heading the wall/device faces (used by the 3D view) |
| `OPENSKY_USER` / `OPENSKY_PASS` | Optional free account for higher rate limits |

---

## Notes on the OpenSky API

- Anonymous access works but has a **limited daily request budget**. Creating a
  free account and setting `OPENSKY_USER`/`OPENSKY_PASS` raises the limit.
- Coverage is community ADS-B, so very low / very local traffic may not always
  appear. A larger `SEARCH_RADIUS_DEG` finds more planes but uses more RAM to
  parse (the ESP8266 only has ~40 KB free heap, so don't go wild).
- TLS certificate validation is skipped (`setInsecure()`) to keep memory low;
  fine for read-only public data.

---

## License

MIT — see [LICENSE](LICENSE).
