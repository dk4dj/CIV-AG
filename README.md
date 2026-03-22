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
- **DHCP** — the WT32-ETH01 obtains its IP address automatically

---

## Hardware

| Signal | GPIO | Notes |
|---|---|---|
| ETH RMII CLK | GPIO 0 | Input from LAN8720 |
| ETH MDC | GPIO 23 | |
| ETH MDIO | GPIO 18 | |
| CI-V RX (UART2) | GPIO 16 | 1 kΩ pull-up to 3.3 V recommended |
| CI-V TX (UART2) | GPIO 17 | Optional, for future polling |
| Conflict output | GPIO 4 | HIGH = antenna conflict detected |

> **Note:** On WT32-ETH01 v1.4, GPIO16 is routed to the pin header and is **not** connected to the LAN8720 power pin. Set `ETH_POWER_PIN` to `-1` in the sketch if your board variant does not use it.

---

## How It Works

```
Icom TRX                WT32-ETH01               Antenna Genius
  CI-V ──── UART2 ────► frequency decode ──TCP──► port set (band + antenna)
                              │
                         web browser ◄── SSE ── live status + debug log
```

1. On boot the sketch waits for a DHCP lease and then listens for the AG UDP broadcast.
2. Once the AG is found, a TCP connection to port 9007 is established and the full band/antenna configuration is read.
3. Every CI-V frequency frame is decoded, matched to a band, and the best antenna (by TX mask) is selected.
4. A `port set` command is sent to the AG only when the band actually changes.
5. If the target antenna is already assigned to the other radio port, GPIO4 is driven HIGH instantly.

---

## Web Interface

Open `http://<device-ip>/` in any browser. The page updates automatically without reloading.

| Section | Content |
|---|---|
| Transceiver | CI-V address, current frequency, active band |
| Antenna Genius | Device name, IP, connection status |
| Port assignment | Active antenna on Port A and Port B |
| Band config | Full band table from AG (active band highlighted) |
| Antenna config | Full antenna list with TX masks |
| Debug log | Live serial log, last 300 lines |

A red banner appears at the top of the page whenever an antenna conflict is active.

Additional endpoints:

- `GET /api` — JSON snapshot of all live data and configuration
- `GET /events` — SSE stream (`status` and `log` events)

---

## Setup

### Arduino IDE

| Setting | Value |
|---|---|
| Board | ESP32 Dev Module (or WT32-ETH01 if installed) |
| Flash Mode | QIO |
| Flash Size | 4 MB |
| esp32 core | Espressif ESP32 Arduino ≥ 2.x |

No additional libraries are required — `ETH.h`, `WiFiUdp.h`, `WiFiClient.h` and `WebServer.h` are all included in the ESP32 Arduino core.

### Configuration constants

All hardware and timing parameters are `#define` constants at the top of the sketch:

```cpp
#define CIV_BAUD       19200   // Match your transceiver's CI-V baud rate
#define CIV_RX_PIN     16      // UART2 RX pin
#define AG_RADIO_PORT  1       // 1 = Radio Port A, 2 = Radio Port B
#define CONFLICT_PIN   4       // GPIO driven HIGH on antenna conflict
#define PORT_POLL_MS   2000    // Active port status polling interval (ms)
```

---

## Antenna Genius API

This project implements the [4O3A Antenna Genius API](https://github.com/4o3a/genius-api-docs/wiki/Antenna-Genius-API):

- **Discovery:** UDP broadcast on port 9007
- **Control:** TCP ASCII protocol on port 9007
- Commands used: `band list`, `antenna list`, `port get`, `port set`, `ping`
- Status messages (`S0|port ...`) are parsed to keep port state in sync in real time

---

## License

MIT
