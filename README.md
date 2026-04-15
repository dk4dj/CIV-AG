# WT32-ETH01 — CI-V → Antenna Genius Bridge

Arduino sketch for the **WT32-ETH01** (ESP32 + LAN8720). Reads Icom CI-V frequency data from UART2 and automatically switches a **4O3A Antenna Genius** antenna selector to the correct antenna for the active band. A **two-layer FlexRadio implementation** signals PTT state both to the AG and directly to SmartSDR, providing reliable TX inhibit for both radios. An optional **EA DOGL128W-6** display with rotary encoder allows manual antenna selection and FLEX radio configuration on the device.

---

## Features

- **Automatic AG discovery** — listens for UDP broadcast on port 9007; extracts IP, port, serial, firmware, ports, antennas, mode, uptime
- **Full AG configuration readout** — band table and antenna list read on every connect
- **CI-V frequency tracking** on UART2 — transceiver address learned automatically; compatible with all Icom radios
- **Automatic band → antenna mapping** based on AG TX masks; active polling (Cmd 0x03) when no broadcast received for 3 s
- **Four-layer TX protection** — all activated and released together via `setConflict()`:
  - GPIO4 HIGH → hardware PTT inhibit line to transceiver
  - CI-V Cmd 0x16/0x66 → TX Inhibit on newer Icom transceivers (older ignore silently)
  - FlexRadio server `state=PTT_REQUESTED` → AG sets `inhibit=1` on Port B
  - SmartSDR Ethernet Interlock `not_ready` → SmartSDR shows TX inhibit directly
- **CI-V timeout safety** — after 5 s without CI-V frames the port is released and all four inhibit mechanisms activated
- **FlexRadio two-layer implementation:**
  - **Server** (port 4992, up to 4 clients): emulates a FlexRadio for the AG; sends `PTT_REQUESTED` proactively after `keepalive enable`
  - **Client**: connects to the real FLEX radio, registers ANT-type interlock; `not_ready`/`ready` commands control SmartSDR TX directly
- **Automatic FLEX radio selection** — priority order: AG Port 2 `source=FLEX` nickname match → NVS default IP → single radio found → user selection
- **Persistent default radio** — stored in NVS (survives reboot), configurable via web UI or long encoder press
- **Splash screen** at boot: original artwork scaled to 128×64 px, stored in PROGMEM
- **Display inversion on error** — ST7565 hardware invert activated on any conflict or CI-V timeout
- **Built-in web interface** on port 80 with live SSE updates, FLEX radio selection panel
- **Log export** — `GET /log` downloads all 300 timestamped log entries
- **Non-blocking architecture** throughout
- **DHCP** — no network configuration required

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
                          → setConflict(true)           inhibit=1 Port B
                          → GPIO4 HIGH                        │
                          → Cmd 16/66                   SmartSDR (Port B)
                          → PTT_REQUESTED (server)            ▲
                          → not_ready (client)                │
                                │                    ┌────────┴────────────┐
                          VITA-49 UDP ───────────────►   flex discovery     │
                          TCP :4992                  ◄─── connect (AG)      │
                          (server, 4 clients) ───────►   PTT_REQUESTED      │
                                │                    └─────────────────────-┘
                          VITA-49 UDP ◄──────────────    SmartSDR discovery
                          TCP :4992  ─────────────────► connect to FLEX
                          (client)   ◄─────────────────  H<handle>
                                     ────────────────►   interlock create
                                     ◄────────────────   R|0|<id>
                                     ────────────────►   not_ready / ready
                                │
                          web browser ◄── SSE ────── live status + log
                          /flex-select               FLEX radio selection
                                │
                          DOGL128 ◄─── encoder ───── antenna + FLEX select
```

---

## Web Interface

Open `http://<device-ip>/` in any browser.

| Section | Content |
|---|---|
| Funkgerät (CI-V) | Address, frequency, active band, TRX status |
| Antenna Genius | Name, IP, firmware, ports, antennas, uptime, status |
| Port-Belegung | Port A+B antenna/band/TX, conflict, FlexRadio server status, SmartSDR interlock status |
| FLEX-Radio-Auswahl | Appears automatically when multiple FLEX radios found; select and set default |
| Band-/Antennen-Konfiguration | Full tables from AG |
| Debug-Log | Live log with `mm:ss.mmm` timestamps, 300 entries, download button |

**Endpoints:** `GET /` dashboard, `GET /api` JSON, `GET /events` SSE, `GET /log` log download, `POST /flex-select?ip=` select radio, `POST /flex-default?ip=` set persistent default.

---

## FlexRadio Implementation

### Server (AG interlock)

UDP discovery broadcast every 2 s + TCP server on port 4992, up to 4 simultaneous clients. On `keepalive enable` from client, immediately sends current interlock state without waiting for `sub tx`. Log entries: `[FLEX:0]`, `[FLEX:1]`, etc.

### Client (SmartSDR direct interlock)

Connects as TCP client to the real FLEX radio. Protocol:
```
→ C1|client program CIV-Bridge
→ C2|interlock create type=ANT name=CIV-Bridge serial=DK4DJ-1
← R2|0|1
[on conflict:]  → C3|interlock not_ready 1
[resolved:]     → C4|interlock ready 1
```
Log entries: `[FRINT]`

### Automatic radio selection

1. AG Port 2 reports `source=FLEX nickname=<name>` → match against discovered radios
2. Default IP from NVS (`Preferences`, key `flex/defaultIP`)
3. Exactly one radio found → auto-select
4. Multiple radios, no match → **user selection required** (yellow web banner + display menu)

Setting a default: web button "Standard" or encoder long press (≥ 2 s) in the FLEX select menu.

---

## TX Inhibit Mechanisms

All controlled via `setConflict(bool active, const char *reason)`:

| Layer | How | Effect |
|---|---|---|
| GPIO4 HIGH | `digitalWrite` | Hardware PTT line |
| CI-V Cmd 0x16/0x66 | `FE FE <addr> E0 16 66 01 FD` | Icom TX Inhibit |
| FlexRadio server | `S00000000\|interlock state=PTT_REQUESTED` | AG `inhibit=1` on Port B |
| FlexRadio client | `C<n>\|interlock not_ready <id>` | SmartSDR TX blocked directly |

---

## Display & Encoder

**Status screen**: frequency, band, antenna, AG status, conflict indicator.

**Antenna selection menu**: TX-mask-filtered list, scrollbar, confirm dialog.

**FLEX radio selection menu** (appears automatically when multiple radios found):
- Rotate to scroll, short press to select
- Long press (≥ 2 s) to select **and** save as persistent default
- No timeout — stays until selection is made

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
| ETH, WiFiUdp, WiFiClient, WiFiServer, WebServer, Preferences | always (ESP32 Arduino core) |
| U8g2 by olikraus | `DISPLAY_ENABLED 1` |

### Key Configuration Constants

```cpp
#define DISPLAY_ENABLED          1
#define FLEX_EMULATION_ENABLED   1
#define FLEX_MAX_CLIENTS         4      // simultaneous AG/server TCP clients
#define FLEX_MAX_RADIOS          8      // max discovered FLEX radios
#define FLEX_INTERLOCK_NAME      "CIV-Bridge"
#define FLEX_INTERLOCK_SERIAL    "DK4DJ-1"
#define CIV_BAUD                 19200
#define CIV_TIMEOUT_MS           5000
#define AG_RADIO_PORT            1      // 1=Port A, 2=Port B
#define ETH_POWER_PIN            16     // -1 on some board revisions
#define CONFLICT_PIN             4
#define LOG_BUF_LINES            300
#define DISP_CONTRAST            22
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `emac_esp32_init: reset timeout` | Wrong `ETH_POWER_PIN` | Try `16` or `-1` |
| AG not found | UDP broadcast blocked | Check `[DISC]` in log |
| SmartSDR still transmits | FLEX radio not selected | Check for `[FRINT] FLEX-Radio entdeckt` in log; use web selection panel |
| No `[FRINT]` entries | SmartSDR not running or on different subnet | Verify SmartSDR sends VITA-49 broadcasts to port 4992 |
| Interlock ID −1 | `interlock create` failed | Check `[FRINT] <<` responses in log |
| FLEX radio selection card not appearing | Only one radio on network | Auto-selected; check `[FRINT] Auto-Auswahl` |
| Default not saved | NVS write failed | Check heap; `Preferences` needs ~8 KB |
| TX not inhibited on timeout | CI-V address not yet learned | Inhibit sent only after `civAddr != 0x00` |
| Display wrong orientation | Board mounting | Change `U8G2_R2` to `R0`/`R1`/`R3` |

---

## License

GNU General Public License v3.0

*Developed with the assistance of [Claude.ai](https://claude.ai) by Anthropic.*
