# WT32-ETH01 — CI-V → Antenna Genius Bridge

An Arduino sketch for the **WT32-ETH01** (ESP32 + LAN8720) that bridges an **Icom CI-V bus** to a **4O3A Antenna Genius** antenna switch over Ethernet. When the transceiver changes frequency, the correct antenna port is selected automatically. An optional **EA DOGL128W-6** display with rotary encoder allows manual antenna selection directly on the device.

---

## Features

- **Automatic AG discovery** via UDP broadcast (port 9007) — no IP configuration needed
- **Full AG configuration readout** on connect: band table and antenna list read directly from the device
- **CI-V frequency tracking** on UART2 — transceiver address learned automatically; compatible with all Icom radios
- **Automatic band → antenna mapping** based on the AG's own band/frequency configuration and antenna TX masks
- **Active CI-V polling** when no broadcast is received for 3 seconds (Cmd 0x03)
- **Conflict detection**: GPIO4 goes HIGH immediately when the target antenna is already in use by another radio port
- **Built-in web interface** on port 80 with live updates via Server-Sent Events — no page refresh needed
- **Optional DOGL128W-6 display** (128×64 px, ST7565) with rotary encoder for manual antenna selection
- **Non-blocking architecture**: web server remains responsive throughout all phases
- **DHCP** — IP address obtained automatically

---

## Hardware

### GPIO Assignment

| Signal | GPIO | Notes |
|---|---|---|
| ETH RMII CLK | 0 | Input from LAN8720 (`ETH_CLOCK_GPIO0_IN`) |
| ETH Power/Reset | 16 | LAN8720 power pin |
| ETH MDC | 23 | |
| ETH MDIO | 18 | |
| ETH RMII (fixed) | 19, 21, 22, 25, 26, 27 | Cannot be reassigned |
| CI-V RX (UART2) | 5 | 1 kΩ pull-up to 3.3 V recommended |
| CI-V TX (UART2) | 17 | Reserved for active polling |
| Conflict output | 4 | HIGH = antenna conflict detected |
| Display MOSI (SI) | 15 | Software-SPI |
| Display SCK | 14 | Software-SPI |
| Display CS | 12 | Active low |
| Display A0 | 2 | Strapping pin — usable after boot |
| Display RST | 33 | Active low |
| Encoder A | 39 | Input-only, ISR on CHANGE, ext. 10 kΩ pull-up required |
| Encoder B | 36 | Input-only, polled, ext. 10 kΩ pull-up required |
| Encoder button | 35 | Input-only, polled, ext. 10 kΩ pull-up required |

> **Note:** GPIO35, GPIO36, and GPIO39 are input-only with no internal pull-up resistors. External 10 kΩ pull-up resistors to 3.3 V are **mandatory** for the encoder inputs.

> **Board variant note:** `ETH_POWER_PIN` (GPIO16) behaviour differs between WT32-ETH01 revisions. Set to `-1` if your board does not use GPIO16 for the LAN8720 reset. If the board fails to start with `emac_esp32_init: reset timeout`, toggle this setting first.

---

## How It Works

```
Icom TRX                WT32-ETH01                   Antenna Genius
  CI-V ──── UART2 ────► frequency decode ───TCP────► port set (band + antenna)
                              │
                         web browser ◄─── SSE ────── live status + debug log
                              │
                       DOGL128 display ◄── rotary encoder (manual ant. select)
```

1. On boot, DHCP lease obtained; AG found via UDP broadcast on port 9007.
2. TCP connection established; `sub port all` subscribes to asynchronous port status updates.
3. `band list` and `antenna list` read from the AG — no manual mapping needed.
4. CI-V frames decoded on UART2; transceiver address learned from first frame.
5. `port set` sent to AG only on band change — not on every frequency update.
6. If no CI-V broadcast received for 3 s, frequency polled actively (Cmd 0x03).
7. Conflict detected → GPIO4 HIGH + red banner in web interface + display warning.

---

## Web Interface

Open `http://<device-ip>/` in any browser. Updates automatically via SSE.

| Section | Content |
|---|---|
| Transceiver | CI-V address, frequency, active band |
| Antenna Genius | Name, IP, serial number, firmware, radio ports, antennas, stack mode, uptime, status |
| Port assignment | Active antenna and band on Port A and B, TX indicator |
| Band config | Full band table from AG, active band highlighted |
| Antenna config | Full antenna list with TX masks |
| Debug log | Live log, last 300 lines, auto-scroll |

Additional endpoints:

| Endpoint | Description |
|---|---|
| `GET /` | Main HTML dashboard |
| `GET /api` | JSON snapshot of all live data and AG configuration |
| `GET /events` | SSE stream — `status` (JSON) and `log` (text) events |

---

## Display & Encoder (optional)

Set `#define DISPLAY_ENABLED 1` to activate. Set to `0` for zero overhead — no pins reserved, no library required.

### Display Screens

**Status screen** (default): frequency, band, active antenna, AG connection status, conflict indicator.

**Antenna selection menu**: scrollable list of antennas valid for the current band (filtered by TX mask). Active antenna marked `[akt]`. Opens by rotating or pressing the encoder.

**Confirmation dialog**: shown before switching — press to confirm, rotate to cancel.

The menu closes automatically after 10 seconds of inactivity (`MENU_TIMEOUT_MS`).

### Encoder Operation

| Action | Effect |
|---|---|
| Rotate (status screen) | Open antenna selection menu |
| Rotate (menu) | Navigate up/down |
| Rotate (confirm) | Cancel, return to menu |
| Press (status screen) | Open antenna selection menu |
| Press (menu) | Open confirmation dialog |
| Press (confirm) | Switch antenna, return to status |

---

## Setup

### Arduino IDE

| Setting | Value |
|---|---|
| Board | ESP32 Dev Module (or WT32-ETH01 if installed) |
| Flash Mode | QIO |
| Flash Size | 4 MB |
| ESP32 core | Espressif ESP32 Arduino ≥ 2.x |

### Libraries

| Library | Source | Required when |
|---|---|---|
| ETH, WiFiUdp, WiFiClient, WebServer | ESP32 Arduino core | always |
| U8g2 by olikraus | Arduino Library Manager | `DISPLAY_ENABLED 1` |

### Configuration constants

```cpp
// Feature toggle
#define DISPLAY_ENABLED  1     // 0 = disable display and encoder entirely

// CI-V
#define CIV_BAUD        19200  // 9600 or 19200 — match your transceiver
#define CIV_RX_PIN      5      // UART2 RX
#define CIV_TX_PIN      17     // UART2 TX
#define CIV_POLL_MS     500    // Active polling interval when no broadcast
#define CIV_BROADCAST_TIMEOUT_MS 3000  // Switch to polling after this many ms

// Ethernet
#define ETH_POWER_PIN   16     // Set to -1 if not connected to LAN8720 reset

// Antenna Genius
#define AG_RADIO_PORT   1      // 1 = Port A, 2 = Port B
#define AG_KEEPALIVE_MS 5000   // Ping interval
#define AG_RECONNECT_MS 15000  // Reconnect attempt interval
#define AG_CFG_TIMEOUT_MS 8000 // Per-step config timeout

// Conflict output
#define CONFLICT_PIN    4

// Display
#define DISP_CONTRAST   22     // 0-63, typical 20-30 for DOGL128 at 3.3 V
// U8G2_R2 = 180° rotation — change to R0/R1/R3 to match your mounting

// Encoder
#define ENC_DEBOUNCE_MS  5
#define MENU_TIMEOUT_MS  10000

// Web server
#define WEB_PORT        80
#define LOG_BUF_LINES   150
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `emac_esp32_init: reset timeout` | Wrong `ETH_POWER_PIN` or CLK mode | Toggle `ETH_POWER_PIN` between `16` and `-1`; ensure `ETH_CLK_MODE = ETH_CLOCK_GPIO0_IN` |
| Web interface not reachable | Blocked during config | Fixed — web server runs non-blocking from boot |
| No CI-V frames | Wrong RX pin or baud rate | Check `CIV_RX_PIN` (GPIO5) and `CIV_BAUD` |
| AG not found | No UDP broadcast | Confirm AG and WT32 on same subnet; UDP port 9007 not blocked |
| Band/antenna list empty | Config timeout | Increase `AG_CFG_TIMEOUT_MS`; check serial log for timeout messages |
| Encoder phantom steps | Missing pull-up resistors | Add 10 kΩ pull-ups to 3.3 V on GPIO35, GPIO36, GPIO39 |
| Display blank | Wrong contrast or rotation | Adjust `DISP_CONTRAST`; try U8G2_R0 through R3 |
| Conflict GPIO stays HIGH | Other port holds the antenna | Switch other radio port manually |

---

## Antenna Genius API

This project implements the [4O3A Antenna Genius API](https://github.com/4o3a/genius-api-docs/wiki/Antenna-Genius-API):

- **Discovery:** UDP broadcast on port 9007 — all fields parsed (ip, port, v, serial, name, ports, antennas, mode, uptime)
- **Control:** TCP ASCII protocol on port 9007
- **Subscription:** `sub port all` — asynchronous port status updates
- **Commands used:** `band list`, `antenna list`, `port get`, `port set`, `ping`
- **Key protocol notes:** command terminator is `\r\n`; response blocks end with `R<seq>|0|` (empty message field); `agCfgSeq` must be set to `agSeq - 1` after each `sendAGCommand()` call

---

## License

This project is licensed under the **GNU General Public License v3.0** — see the [LICENSE](LICENSE) file for details.

---

*This project was developed with the assistance of [Claude.ai](https://claude.ai) by Anthropic.*
