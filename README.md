# WT32-ETH01 — CI-V → Antenna Genius Bridge

Arduino sketch for the **WT32-ETH01** (ESP32 + LAN8720). Reads Icom CI-V frequency data from UART2 and automatically switches a **4O3A Antenna Genius** antenna selector to the correct antenna for the active band. A **FlexRadio protocol emulation** signals PTT state simultaneously to the AG and to SmartSDR so that both systems detect conflicts before transmitting into a shared antenna. An optional **EA DOGL128W-6** display with rotary encoder allows manual antenna selection directly on the device.

---

## Features

- **Automatic AG discovery** — listens for UDP broadcast on port 9007; extracts IP, port, serial, firmware, ports, antennas, mode, uptime
- **Full AG configuration readout** — band table and antenna list read from device on every connect
- **CI-V frequency tracking** on UART2 — transceiver address learned automatically from first received frame; compatible with all Icom radios
- **Automatic band → antenna mapping** based on AG TX masks; active polling (Cmd 0x03) when no broadcast received for 3 s
- **Three-layer TX protection** — all activated and released together via `setConflict()`:
  - GPIO4 HIGH → hardware PTT inhibit line to transceiver
  - CI-V Cmd 0x16/0x66 → TX Inhibit on newer Icom transceivers (older models ignore it silently)
  - FlexRadio `state=PTT_REQUESTED` → sent to AG **and** SmartSDR simultaneously
- **CI-V timeout safety** — after 5 s without CI-V frames the port is released and all three inhibit mechanisms activated
- **FlexRadio VITA-49 multi-client emulation** — UDP discovery broadcast every 2 s + TCP server on port 4992 supporting up to 4 simultaneous clients (AG + SmartSDR + others); each client receives the interlock status independently
- **Splash screen** at boot: original artwork scaled to 128×64 px, stored in PROGMEM
- **Display inversion on error** — ST7565 hardware invert (Cmd 0xA7/0xA6) activated on any conflict or CI-V timeout; restored automatically when condition clears
- **Built-in web interface** on port 80 with live SSE updates
- **Log export** — `GET /log` downloads all 300 log entries as a timestamped `.log` file
- **Non-blocking architecture** throughout — web server, FlexRadio emulation and CI-V always responsive
- **DHCP** — no network configuration required
- All optional features individually switchable via `#define` with zero overhead when disabled

---

## Hardware

### GPIO Assignment

| Signal | GPIO | Notes |
|---|---|---|
| ETH CLK input | 0 | `ETH_CLOCK_GPIO0_IN`, from LAN8720 crystal |
| ETH Power/Reset | 16 | LAN8720 nRST — try `-1` if boot shows reset timeout |
| ETH MDC | 23 | |
| ETH MDIO | 18 | |
| ETH RMII (fixed) | 19, 21, 22, 25, 26, 27 | Cannot be reassigned |
| CI-V RX (UART2) | 5 | 1 kΩ pull-up to 3.3 V recommended |
| CI-V TX (UART2) | 17 | Active polling + TX Inhibit frames |
| TX inhibit / conflict output | 4 | HIGH = any conflict or CI-V timeout active |
| Display SI (MOSI) | 15 | Software-SPI |
| Display SCK | 14 | Software-SPI |
| Display CS | 12 | Active low |
| Display A0 | 2 | Strapping pin — safe to use after boot |
| Display RST | 33 | Active low |
| Encoder A | 39 | Input-only, ISR on CHANGE, ext. 10 kΩ pull-up **required** |
| Encoder B | 36 | Input-only, polled (no ISR), ext. 10 kΩ pull-up **required** |
| Encoder button | 35 | Input-only, polled, ext. 10 kΩ pull-up **required** |

> **GPIO 35/36/39** are input-only with no internal pull-up resistors. External 10 kΩ pull-up resistors to 3.3 V are mandatory. GPIO 36 and 39 share 270 pF internal capacitance, so Encoder B is polled (no ISR) to avoid crosstalk.

---

## Architecture

```
Icom TRX                  WT32-ETH01                     Antenna Genius
  CI-V ─── UART2 ───────► band detect ──── TCP ─────────► port set
                                │                              │
                          CI-V timeout                  source=FLEX tx=1
                          → release port                inhibit=1 on Port B
                          → setConflict(true)                  │
                          → GPIO4 HIGH                   SmartSDR (Port B)
                          → Cmd 16/66                         ▲
                          → PTT_REQUESTED                      │
                                │                    ┌─────────┴──────────┐
                          VITA-49 UDP ───────────────►   flex discovery    │
                          TCP :4992   ◄──────────────── connect            │
                          (up to 4    ────────────────► S0|interlock       │
                           clients)   ◄──────────────── keepalive/ping     │
                                │     ────────────────► PTT updates        │
                                │                    └────────────────────-┘
                          web browser ◄── SSE ────── live status + log
                                │
                          DOGL128 ◄─── encoder ────── manual antenna select
                          (inverted on error)
```

**All three inhibit mechanisms fire together** via `setConflict(true, reason)`:
1. CI-V timeout (TRX silent for 5 s)
2. Target antenna already in use by another port

**All three release together** via `setConflict(false, reason)`:
1. New CI-V frequency frame received (TRX back online)
2. Conflict resolved (antenna freed)

---

## Web Interface

Open `http://<device-ip>/` in any browser. Updates automatically via SSE (Server-Sent Events).

| Section | Content |
|---|---|
| Funkgerät (CI-V) | Address, frequency, active band, TRX status (active / inhibited) |
| Antenna Genius | Name, IP, TCP port, serial, firmware, radio ports, antennas, stack mode, uptime, status |
| Port-Belegung | Port A+B antenna and band, TX indicator, conflict status, FlexRadio client count, PTT interlock state |
| Band-Konfiguration | Full band table from AG, active band highlighted |
| Antennen-Konfiguration | Full antenna list with TX masks |
| Debug-Log | Live log with timestamps (mm:ss.mmm), last 300 entries, auto-scroll, download button |

Additional endpoints: `GET /api` (full JSON snapshot), `GET /events` (SSE stream), `GET /log` (log file download).

### Log Download

The **"↧ Log herunterladen"** button in the log section downloads all current log entries as a plain-text `.log` file. The file includes a header with device IP, uptime, AG name and firmware, followed by all entries in chronological order with `\r\n` line endings. The filename contains the current uptime for uniqueness (e.g. `wt32-bridge-145s.log`).

All log entries carry a `mm:ss.mmm` timestamp relative to device boot, making it easy to correlate events across the AG, CI-V, FlexRadio and web subsystems.

---

## Display & Encoder

Set `#define DISPLAY_ENABLED 1` to activate. `0` = zero overhead, no pins reserved.

**Boot splash screen**: original artwork scaled to 100×64 px (aspect ratio preserved), centered on 128×64, stored as 1 bpp XBM in PROGMEM.

**Display inversion**: when any conflict or CI-V timeout is active, the ST7565 hardware invert command (0xA7) is sent — the entire display inverts instantly without redrawing. Restored to normal (0xA6) when the condition clears.

**Status screen** (default): frequency (large), band, active antenna, AG status, conflict/inhibit indicator.

**Antenna selection menu**: scrollable list filtered by TX mask for the current band.

| Action | Effect |
|---|---|
| Rotate (status screen) | Open antenna menu |
| Rotate (menu) | Navigate up/down |
| Press (menu) | Open confirmation dialog |
| Press (confirm) | Switch antenna, return to status |
| Timeout (10 s) | Return to status screen |

---

## FlexRadio Emulation

Set `#define FLEX_EMULATION_ENABLED 1` to activate.

**Multi-client TCP server** on port 4992 — supports up to `FLEX_MAX_CLIENTS` (4) simultaneous connections. Both the Antenna Genius and SmartSDR connect independently and each receives the interlock status. Log entries show the client index: `[FLEX:0]`, `[FLEX:1]`, etc.

**Discovery broadcast** (UDP 255.255.255.255:4992, every 2 s): VITA-49 `ExtDataWithStream` packet with OUI `0x1C2D` (FlexRadio) and `PacketClassCode=0xFFFF`.

**Connect sequence** (per client):
```
← V3.3.15\n              (version prologue)
← H00000001\n            (client handle)
→ C2|keepalive enable    (client sends this)
← R2|00000000|\n         (OK)
← S00000000|interlock state=PTT_REQUESTED/READY ...\n   (immediate status push)
→ C3|ping                (periodic keepalive)
← R3|00000000|\n
```

The interlock status is pushed immediately after `keepalive enable` without waiting for a `sub tx` subscription, because the real FlexRadio sends it proactively.

**PTT state mapping:**

| CI-V state | Interlock state |
|---|---|
| Active (frames received within timeout) | `state=PTT_REQUESTED source=SW tx_allowed=1` |
| Timeout / address unknown | `state=READY source= tx_allowed=1` |

---

## TX Inhibit Mechanisms

All three are controlled exclusively via `setConflict(bool active, const char *reason)`:

| Layer | How | Effect |
|---|---|---|
| GPIO4 HIGH | `digitalWrite(CONFLICT_PIN, HIGH)` | Hardware PTT line to TRX |
| CI-V Cmd 0x16/0x66 | `FE FE <addr> E0 16 66 01/00 FD` over UART2 | TX Inhibit on Icom TRX |
| FlexRadio interlock | `S00000000\|interlock state=PTT_REQUESTED` to all TCP clients | SmartSDR inhibited by AG |

Older Icom transceivers that do not support Cmd 0x16/0x66 respond with `FA FA` (FAIL) or ignore the frame — no negative effect.

---

## Setup

### Arduino IDE

| Setting | Value |
|---|---|
| Board | ESP32 Dev Module |
| Flash Mode | QIO |
| Flash Size | 4 MB |
| ESP32 core | Espressif ESP32 Arduino ≥ 2.x |

### Libraries

| Library | Required when |
|---|---|
| ETH, WiFiUdp, WiFiClient, WiFiServer, WebServer | always (ESP32 Arduino core) |
| U8g2 by olikraus | `DISPLAY_ENABLED 1` |

### Key Configuration Constants

```cpp
#define DISPLAY_ENABLED          1      // 0 = no display/encoder
#define FLEX_EMULATION_ENABLED   1      // 0 = no FlexRadio emulation
#define FLEX_MAX_CLIENTS         4      // simultaneous TCP clients (AG + SmartSDR + ...)

#define CIV_BAUD                 19200
#define CIV_TIMEOUT_MS           5000   // release port + inhibit after silence
#define CIV_BROADCAST_TIMEOUT_MS 3000   // switch to active polling after this

#define AG_RADIO_PORT            1      // 1 = Port A, 2 = Port B
#define ETH_POWER_PIN            16     // -1 on some WT32-ETH01 revisions
#define CONFLICT_PIN             4      // HIGH = any inhibit active
#define LOG_BUF_LINES            300    // log ring buffer size

#define FLEX_MODEL               "FLEX-6300"
#define FLEX_VERSION             "3.3.15"
#define DISP_CONTRAST            22     // 0–63 for ST7565
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `emac_esp32_init: reset timeout` | Wrong `ETH_POWER_PIN` | Try `16` or `-1` |
| AG not found | UDP broadcast blocked | Check subnet; look for `[DISC]` in log |
| SmartSDR still transmits on conflict | SmartSDR not connected to our TCP server | Check for `[FLEX:x] Client verbunden` in log; verify SmartSDR shows radio in flex list |
| AG sets `inhibit=1` but SmartSDR ignores | SmartSDR uses its own internal interlock, not AG inhibit | Ensure SmartSDR connects directly to our port 4992 and receives `PTT_REQUESTED` |
| No `sub tx` from client | Normal — status is sent proactively after `keepalive enable` | Expected behaviour |
| TX not inhibited on timeout | CI-V address not yet learned | Inhibit only sent after `civAddr != 0x00` |
| Display blank / inverted at boot | Wrong contrast or rotation | Adjust `DISP_CONTRAST`; try `U8G2_R0` through `R3` |
| Encoder phantom steps | Missing pull-up resistors | 10 kΩ to 3.3 V on GPIO35, 36, 39 |
| Log download button no effect | Browser cached old page | Hard-refresh (Ctrl+Shift+R); check for `[WEB] Log-Export` in log |

---

## License

GNU General Public License v3.0

*Developed with the assistance of [Claude.ai](https://claude.ai) by Anthropic.*
