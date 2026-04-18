# WT32-ETH01 — CI-V → Antenna Genius Bridge

Arduino sketch for the **WT32-ETH01** (ESP32 + LAN8720). Reads Icom CI-V frequency data from UART2 and automatically switches a **4O3A Antenna Genius** antenna selector to the correct antenna for the active band. A **two-layer FlexRadio implementation** signals PTT state both to the AG and directly to SmartSDR, providing reliable TX inhibit for both radios. An optional **EA DOGL128W-6** display with rotary encoder allows manual antenna selection and FLEX radio configuration on the device.

---

## Features

- **Automatic AG discovery** — listens for UDP broadcast on port 9007; extracts IP, port, serial, firmware, ports, antennas, mode, uptime
- **Full AG configuration readout** — band table and antenna list read on every connect
- **CI-V frequency tracking** on UART2 — transceiver address learned automatically; compatible with all Icom radios
- **Active polling** after `CIV_BROADCAST_TIMEOUT_MS` (1 s) without broadcast via Cmd 0x03
- **Automatic band → antenna mapping** based on AG TX masks
- **Four-layer TX protection** — all activated/released together via `setConflict()`:
  - GPIO4 HIGH → hardware PTT inhibit line to transceiver
  - CI-V Cmd 0x16/0x66 → TX Inhibit on newer Icom transceivers (older ignore silently)
  - FlexRadio server `state=PTT_REQUESTED` → AG sets `inhibit=1` on Port B
  - SmartSDR Ethernet Interlock `not_ready` → SmartSDR shows TX inhibit directly
- **Conflict reason display** — web UI and log show which transceiver (CI-V address + frequency + antenna) was inhibited
- **Correct conflict re-evaluation** after band change on either port — `wantedAntId` tracks the requested antenna independently of AG status (AG sets `txant=0` when `inhibit=1`)
- **CI-V timeout safety** — after `CIV_TIMEOUT_MS` (5 s) without CI-V frames the port is released and all four inhibit mechanisms activated
- **FlexRadio two-layer implementation:**
  - **Server** (port 4992, up to `FLEX_MAX_CLIENTS` = 4 clients): emulates a FlexRadio for the AG; sends `PTT_REQUESTED` proactively after `keepalive enable` (no `sub tx` required)
  - **Client**: connects to the real FLEX radio, registers ANT-type interlock; `not_ready`/`ready` commands control SmartSDR TX directly
- **Separate UDP sockets**: `flexDiscUdp` (send-only, no `begin()`) and `flexListenUdp` (receive-only, bound to port 4992) — avoids ESP32 send/receive conflict on the same socket
- **Automatic FLEX radio selection** — priority: AG Port 2 `source=FLEX` nickname match → NVS default IP → single radio found → user selection
- **Persistent default radio** — stored in NVS (survives reboot), configurable via web UI or long encoder press
- **Shared JSON builder** `buildStatusJson()` — identical status for SSE push and `/api` polling; band names resolved by `bands[i].id` lookup (AG Band-ID is 1-based, array is 0-based)
- **Splash screen** at boot: original artwork scaled to 128×64 px, stored in PROGMEM
- **Display inversion on error** — ST7565 hardware invert activated on any conflict or CI-V timeout
- **Built-in web interface** on port 80 with live SSE updates, FLEX radio selection panel, conflict detail display
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
Icom TRX (CI-V)              WT32-ETH01                    Antenna Genius
  CI-V ── UART2 ──────────► band detect ──── TCP ─────────► port set
                                  │                              │
                          ┌───────┴────────┐             source=FLEX tx=1
                          │  wantedAntId   │             inhibit=1 Port B
                          │  (survives     │                     │
                          │   inhibit=0)   │            SmartSDR (Port B)
                          └───────┬────────┘                     ▲
                          CI-V timeout                           │
                          → setConflict(true)          ┌─────────┴───────────┐
                          → GPIO4 HIGH                 │  flexListenUdp:4992  │
                          → Cmd 16/66                  │  VITA-49 Discovery  │
                          → PTT_REQUESTED (server)     │  → flexRadioList[]  │
                          → not_ready (client)         │  → Auto-Select       │
                          → conflictReason set         └─────────────────────┘
                                  │
                          VITA-49 UDP ◄─────────────────── SmartSDR broadcasts
                          flexDiscUdp (send only)
                          TCP :4992 Server ────────────── AG connects here
                          (up to 4 clients)
                          TCP :4992 Client ────────────── connect to FLEX
                                           ─────────────► interlock create
                                           ◄─────────────  R|0|<id>
                                           ─────────────► not_ready / ready
                                  │
                          web browser ◄── SSE ─── buildStatusJson()
                          /api (30 s poll) ◄──── buildStatusJson()
                          /flex-select, /flex-default
                                  │
                          DOGL128 ◄─── encoder ──── antenna + FLEX select
```

---

## Web Interface

Open `http://<device-ip>/` in any browser.

| Section | Content |
|---|---|
| Funkgerät (CI-V) | Address, frequency, active band, TRX status |
| Antenna Genius | Name, IP, firmware, ports, antennas, uptime, status |
| Port-Belegung | Port A+B antenna/band/TX, **conflict badge with inhibited TRX details**, FlexRadio server + client status |
| FLEX-Radio-Auswahl | Appears automatically when multiple FLEX radios found |
| Band-/Antennen-Konfiguration | Full tables from AG |
| Debug-Log | Live log with `mm:ss.mmm` timestamps, 300 entries, download button |

**Conflict display:** when active, the badge and red banner show: `⚠ KONFLIKT – CI-V 0x94 (24.899 MHz) gesperrt – Ant Beam_OB11-3`

**Endpoints:** `GET /` dashboard, `GET /api` JSON, `GET /events` SSE, `GET /log` log download, `POST /flex-select?ip=` select radio, `POST /flex-default?ip=` set persistent default.

---

## FlexRadio Implementation

### Server (AG interlock)

UDP discovery broadcast every 2 s + TCP server on port 4992. On `keepalive enable` from client, immediately sends current interlock state — no `sub tx` required. Log entries: `[FLEX:0]`, `[FLEX:1]`, etc.

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

### UDP socket separation (critical)

Two separate sockets prevent the ESP32 send/receive conflict:
- `flexDiscUdp` — **send only**, no `begin()`, used by `flexSendDiscovery()`
- `flexListenUdp` — **receive only**, `begin(4992)`, used by `flexRadioLoop()`

### Automatic radio selection

1. AG Port 2 reports `source=FLEX nickname=X` → match `agPort2FlexNickname` against discovered list
2. Default IP from NVS (`Preferences`, key `flex/defaultIP`)
3. Exactly one radio found → auto-select
4. Multiple → `flexSelectPending = true` → yellow web banner + display menu

Setting a default: web button "Standard" or encoder long press (≥ 2 s) in the FLEX select menu.

---

## Conflict Logic

### wantedAntId

The AG sets `txant=0` in its status message when `inhibit=1` is active. Without a separate tracking variable, `portTxAnt[AG_RADIO_PORT-1]` would become 0, causing:
- Web UI to show `---` for the antenna name
- `checkAndSignalConflict()` to use `wantedAntId=0` → conflict never detected

`wantedAntId` is set in `setAGPort()` and cleared in `releaseAGPort()`. It is never overwritten by AG status updates.

### Re-evaluation trigger

`parsePortLine()` calls `checkAndSignalConflict(0)` (which uses `wantedAntId`) whenever the **other** port changes state — immediately resolving or detecting conflicts without waiting for the next CI-V frame.

---

## TX Inhibit Mechanisms

All controlled via `setConflict(bool active, const char *reason)`:

| Layer | How | Effect |
|---|---|---|
| GPIO4 HIGH | `digitalWrite` | Hardware PTT line |
| CI-V Cmd 0x16/0x66 | `FE FE <addr> E0 16 66 01 FD` | Icom TX Inhibit |
| FlexRadio server | `S00000000\|interlock state=PTT_REQUESTED` | AG `inhibit=1` on Port B |
| FlexRadio client | `C<n>\|interlock not_ready <id>` | SmartSDR TX blocked directly |

`conflictReason` is set with CI-V address, frequency and antenna name for display in the web UI.

---

## Display & Encoder

**Status screen**: frequency, band, antenna, AG status, conflict indicator.

**Antenna selection menu**: TX-mask-filtered list, scrollbar, confirm dialog.

**FLEX radio selection menu** (appears automatically when multiple radios found):
- Rotate to scroll, short press to select
- Long press (≥ 2 s) to select **and** save as persistent default (NVS)
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
#define CIV_BROADCAST_TIMEOUT_MS 1000   // poll after 1 s without broadcast
#define CIV_TIMEOUT_MS           5000   // release port after 5 s without CI-V
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
| SmartSDR still transmits | FLEX radio not selected | Check `[FRINT] FLEX-Radio entdeckt` in log |
| No `[FRINT]` entries | `flexListenUdp` not receiving | Verify port 4992 not blocked by firewall |
| Conflict not cleared after band change | `wantedAntId` not set | Ensure `setAGPort()` was called before conflict |
| Port A/B shows `---` after conflict | AG reports `txant=0` during inhibit | Handled by `wantedAntId` fallback |
| Interlock ID −1 | `interlock create` failed | Check `[FRINT] <<` responses in log |
| FLEX radio selection card not appearing | Only one radio on network | Auto-selected; check `[FRINT] Auto-Auswahl` |
| Default not saved | NVS write failed | Check heap; `Preferences` needs ~8 KB |

---

## License

GNU General Public License v3.0

*Developed with the assistance of [Claude.ai](https://claude.ai) by Anthropic.*
