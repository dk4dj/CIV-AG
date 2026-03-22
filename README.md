# WT32-ETH01 — CI-V → Antenna Genius Bridge

An Arduino sketch for the **WT32-ETH01** (ESP32 + LAN8720) that bridges an **Icom CI-V bus** to a **4O3A Antenna Genius** antenna switch over Ethernet. When the transceiver changes frequency, the correct antenna port is selected automatically — no manual intervention required.

---

## Features

- **Automatic AG discovery** via UDP broadcast (port 9007) — no IP configuration needed
- **Full AG configuration readout** on connect: band table and antenna list are read directly from the device
- **CI-V frequency tracking** on UART2 — transceiver address is learned automatically from the first received frame; compatible with all Icom radios
- **Automatic band → antenna mapping** based on the AG's own band/frequency configuration and antenna TX masks
- **Conflict detection**: GPIO4 goes HIGH immediately when the target antenna is already in use by another radio port
- **Built-in web interface** served on port 80 with live updates via Server-Sent Events (SSE) — no page refresh needed
- **Non-blocking architecture**: web server remains responsive during AG discovery, connect and configuration readout
- **DHCP** — the WT32-ETH01 obtains its IP address automatically

---

## Hardware

| Signal | GPIO | Notes |
|---|---|---|
| ETH RMII CLK | GPIO 0 | Input from LAN8720 (`ETH_CLOCK_GPIO0_IN`) |
| ETH Power/Reset | GPIO 16 | LAN8720 power pin (`ETH_POWER_PIN`) |
| ETH MDC | GPIO 23 | |
| ETH MDIO | GPIO 18 | |
| CI-V RX (UART2) | GPIO 5 | 1 kΩ pull-up to 3.3 V recommended |
| CI-V TX (UART2) | GPIO 17 | Optional, reserved for future polling |
| Conflict output | GPIO 4 | HIGH = antenna conflict detected |

> **Board variant note:** `ETH_POWER_PIN` (GPIO16) behaviour differs between WT32-ETH01 hardware revisions. On v1.4 GPIO16 is routed to the pin header only and should be set to `-1`. On other revisions it controls the LAN8720 power/reset and must be set to `16`. If the board fails to start with the error `emac_esp32_init: reset timeout`, try toggling this setting first.

---

## How It Works

```
Icom TRX                WT32-ETH01                   Antenna Genius
  CI-V ──── UART2 ────► frequency decode ───TCP────► port set (band + antenna)
                              │
                         web browser ◄─── SSE ────── live status + debug log
```

1. On boot the sketch waits for a DHCP lease, then listens for the AG UDP broadcast on port 9007.
2. Once the AG is found, a TCP connection is established and the full band/antenna configuration is read from the device.
3. Every CI-V frequency frame on UART2 is decoded, matched against the AG band table, and the best antenna (by TX mask) is selected.
4. A `port set` command is sent to the AG only when the band actually changes — not on every frequency update.
5. If the target antenna is already assigned to the other radio port, GPIO4 is driven HIGH instantly and a warning appears in the web interface.
6. The web server runs non-blocking throughout all phases — it is reachable from the moment Ethernet is up.

---

## Web Interface

Open `http://<device-ip>/` in any browser. The page updates automatically without reloading, using Server-Sent Events.

| Section | Content |
|---|---|
| Transceiver | CI-V address (auto-detected), current frequency, active band |
| Antenna Genius | Device name, IP address, connection status |
| Port assignment | Active antenna on Port A and Port B |
| Band config | Full band table from AG — active band highlighted in green |
| Antenna config | Full antenna list with TX masks in hex |
| Debug log | Live log output, last 300 lines, auto-scroll |

A red conflict banner appears at the top of the page whenever GPIO4 is HIGH.

Additional HTTP endpoints:

| Endpoint | Description |
|---|---|
| `GET /` | Main HTML dashboard |
| `GET /api` | JSON snapshot of all live data and full AG configuration |
| `GET /events` | SSE stream — `status` events (JSON) and `log` events (text lines) |

---

## Setup

### Arduino IDE

| Setting | Value |
|---|---|
| Board | ESP32 Dev Module (or WT32-ETH01 if installed) |
| Flash Mode | QIO |
| Flash Size | 4 MB |
| ESP32 core | Espressif ESP32 Arduino ≥ 2.x |

No additional libraries are required — `ETH.h`, `WiFiUdp.h`, `WiFiClient.h` and `WebServer.h` are all part of the ESP32 Arduino core.

### Configuration constants

All hardware and timing parameters are `#define` constants at the top of the sketch:

```cpp
// CI-V
#define CIV_BAUD        19200  // Match your transceiver's CI-V baud rate (9600 or 19200)
#define CIV_RX_PIN      5      // UART2 RX — GPIO5
#define CIV_TX_PIN      17     // UART2 TX — GPIO17

// Ethernet
#define ETH_POWER_PIN   16     // LAN8720 power/reset pin — set to -1 if not connected

// Antenna Genius
#define AG_RADIO_PORT   1      // 1 = Radio Port A, 2 = Radio Port B
#define AG_KEEPALIVE_MS 5000   // Keepalive ping interval in ms

// Conflict output
#define CONFLICT_PIN    4      // GPIO driven HIGH on antenna conflict

// Web server
#define WEB_PORT        80
#define LOG_BUF_LINES   80     // Number of log lines kept in memory

// Port status polling
#define PORT_POLL_MS    2000   // Active port get polling interval in ms
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `emac_esp32_init: reset timeout` | Wrong `ETH_POWER_PIN` or CLK mode | Set `ETH_POWER_PIN` to `16` or `-1` depending on board revision; ensure `ETH_CLK_MODE = ETH_CLOCK_GPIO0_IN` |
| Web interface not reachable | `handleClient()` blocked by AG discovery | Fixed in current version — web server runs non-blocking from boot |
| No CI-V frames received | Wrong RX pin or baud rate | Check `CIV_RX_PIN` (GPIO5) and `CIV_BAUD` match your hardware and transceiver |
| AG not found | No UDP broadcast received | Confirm AG and WT32 are on the same subnet; check that no firewall blocks UDP port 9007 |
| Conflict GPIO stays HIGH | Other port still holds the antenna | Manually switch the other radio port away from the shared antenna |

---

## Antenna Genius API

This project implements the [4O3A Antenna Genius API](https://github.com/4o3a/genius-api-docs/wiki/Antenna-Genius-API):

- **Discovery:** UDP broadcast on port 9007
- **Control:** TCP ASCII protocol on port 9007
- **Commands used:** `band list`, `antenna list`, `port get`, `port set`, `ping`
- **Status messages** (`S0|port ...`) are parsed in real time to keep port state in sync without polling

---

## License

This project is licensed under the **GNU General Public License v3.0** — see the [LICENSE](LICENSE) file for details.

---

*This project was developed with the assistance of [Claude.ai](https://claude.ai) by Anthropic.*
