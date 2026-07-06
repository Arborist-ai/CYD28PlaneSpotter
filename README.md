# CYD28 Plane Spotter ✈️

A real-time aircraft tracker for the **ESP32 Cheap Yellow Display (CYD)** that renders a live radar PPI (Plan Position Indicator), shows nearby aircraft details, and provides weather & system status — all on a 2.8" 240×320 TFT touchscreen.

## Features

- **Radar PPI** — rotating sweep line with aircraft blips, each showing callsign, flight level (FL), and speed in knots. Configurable range with on-screen +/- buttons.
- **Target Intel** — detailed view of the nearest aircraft: callsign, route (origin → destination), country, distance/bearing, altitude, speed, and heading arrow.
- **Top 5 In Range** — scrollable list of the 5 closest flights with route and altitude info.
- **Weather & System** — live temperature, humidity, and wind from Open-Meteo, plus system stats (uptime, WiFi RSSI, free RAM, API request counters).
- **Web Configuration** — built-in web server to change home location, search radius, and radar range from any browser on the same network.
- **Dual-core architecture** — HTTP/API calls run on Core 0 (FreeRTOS task), UI rendering and touch handling run on Core 1, keeping the display smooth.

## Hardware

- **Board**: ESP32-2432S028 (CYD28) — 2.8" TFT with ST7789 driver
- **Touch**: XPT2046 resistive touch controller
- **Display**: 240×320 pixels, rotation 1 (landscape)

## Data Sources

| Source | Purpose |
|--------|---------|
| [OpenSky Network](https://opensky-network.org/) | Real-time ADS-B aircraft states |
| [hexdb.io](https://hexdb.io/) | ICAO hex → route lookup (departure/arrival airports) |
| [Open-Meteo](https://open-meteo.com/) | Weather (temperature, humidity, wind speed) |

## Project Structure

```
CYD28/
├── platformio.ini          # PlatformIO build configuration
├── config.env.example      # Configuration template (copy to config.env)
├── .gitignore
├── README.md
└── src/
    ├── main.cpp            # Main application code
    └── config.env          # Your private config (gitignored)
```

## Setup

1. **Clone & open** in VS Code with the PlatformIO extension.
2. **Copy the config template**:
   ```
   cp config.env.example src/config.env
   ```
3. **Edit `src/config.env`** with your WiFi credentials and OpenSky API keys:
   ```c
   #define WIFI_SSID "YourWiFi"
   #define WIFI_PASS "YourPassword"
   #define OPENSKY_CLIENT_ID "your-client-id"      // or leave blank for anonymous
   #define OPENSKY_CLIENT_SECRET "your-secret"     // or leave blank for anonymous
   ```
4. **Adjust defaults** for your home location if desired:
   ```c
   #define DEFAULT_HOME_LAT      47.5774   // your latitude
   #define DEFAULT_HOME_LON      8.5212    // your longitude
   ```
5. **Build & upload**:
   ```
   pio run --target upload
   ```
6. Once connected, find the device IP in the serial monitor and open it in a browser to change settings live.

## Usage

- **Tap the screen** to cycle through the 4 screens.
- **On the Radar screen**, tap the **+/- buttons** at the bottom-left to adjust range (10–200 km).
- **Open the device IP** in a browser to configure location and range.

## Dependencies

- [TFT_eSPI](https://github.com/Bodmer/TFT_eSPI) — TFT display driver
- [ArduinoJson](https://arduinojson.org/) — JSON parsing
- [XPT2046_Touchscreen](https://github.com/PaulStoffregen/XPT2046_Touchscreen) — Touch controller

All managed automatically via PlatformIO `lib_deps`.

## License

MIT
