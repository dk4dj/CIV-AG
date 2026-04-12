# WT32-ETH01 — CI-V → Antenna Genius Bridge

Arduino sketch for the **WT32-ETH01** (ESP32 + LAN8720). Reads Icom CI-V frequency data from UART2 and automatically switches a **4O3A Antenna Genius** antenna selector to the correct antenna for the active band. A **FlexRadio protocol emulation** signals PTT state to the AG so that SmartSDR on the second radio port detects conflicts before transmitting into a busy antenna. An optional **EA DOGL128W-6** display with rotary encoder allows manual antenna selection directly on the device.

---

## Features

- **Automatic AG discovery** — listens for UDP broadcast on port 9007, extracts all discovery fields (IP, port, serial, firmware, ports, antennas, mode, uptime)
- **Full AG configuration readout** — band table and antenna list read directly from the device on every connect
- **CI-V frequency tracking** on UART2 — transceiver address learned automatically from first received frame; compatible with all Icom radios
- **Automatic band → antenna mapping** based on AG TX masks; active polling (Cmd 0x03) when no broadcast received for 3 s
- **Three-layer TX protection** — all activated together via `setConflict()`:
  - GPIO4 HIGH → hardware PTT inhibit line to transceiver
  - CI-V Cmd 0x16/0x66 → TX Inhibit on newer Icom transceivers (ignored silently by older models)
  - FlexRadio `state=PTT_REQUESTED` → AG signals conflict to SmartSDR on Port B
- **CI-V timeout safety** — after 5 s without CI-V frames the port is released (`auto=1 band=0 ant=0`) and all three inhibit mechanisms activated
- **FlexRadio VITA-49 emulation** — UDP discovery broadcast every 2 s + TCP server on port 4992; AG connects back and receives interlock status updates
- **Built-in web interface** on port 80 with live SSE updates — shows all AG discovery fields, port status, FlexRadio connection and PTT state, CI-V release status
- **Optional DOGL128W-6 display** (128×64, ST7565) with rotary encoder
- **Non-blocking architecture** throughout — web server and FlexRadio emulation always responsive
- **DHCP** — no network configuration required
- All optional features individually switchable via `#define` with zero overhead when disabled

---

## Hardware

### GPIO Assignment

| Signal | GPIO | Notes |
|---|---|---|
| ETH CLK input | 0 | `ETH_CLOCK_GPIO0_IN`, from LAN8720 crystal |
| ETH Power/Reset | 16 | LAN8720 nRST — set to `-1` on some board revisions |
| ETH MDC | 23 | |
| ETH MDIO | 18 | |
| ETH RMII (fixed) | 19, 21, 22, 25, 26, 27 | Cannot be reassigned |
| CI-V RX (UART2) | 5 | 1 kΩ pull-up to 3.3 V recommended |
| CI-V TX (UART2) | 17 | Active polling + TX Inhibit frames |
| TX inhibit / conflict output | 4 | HIGH = conflict or CI-V timeout active |
| Display SI (MOSI) | 15 | Software-SPI |
| Display SCK | 14 | Software-SPI |
| Display CS | 12 | Active low |
| Display A0 | 2 | Strapping pin — safe to use after boot |
| Display RST | 33 | Active low |
| Encoder A | 39 | Input-only, ISR on CHANGE, ext. 10 kΩ pull-up **required** |
| Encoder B | 36 | Input-only, polled (no ISR), ext. 10 kΩ pull-up **required** |
| Encoder button | 35 | Input-only, polled, ext. 10 kΩ pull-up **required** |

> **GPIO 35/36/39** are input-only with no internal pull-up resistors. External 10 kΩ pull-up resistors to 3.3 V are mandatory. GPIO 36 and 39 share 270 pF internal capacitance (CAPP/CAPN), so Encoder B is read by polling to avoid ISR crosstalk.

> **Board revision note:** `ETH_POWER_PIN` (GPIO16) behaviour differs between WT32-ETH01 revisions. If boot produces `emac_esp32_init: reset timeout`, try setting `ETH_POWER_PIN` to `-1`.

---

## Architecture

```
Icom TRX                  WT32-ETH01                     Antenna Genius
  CI-V ─── UART2 ───────► band detect ──── TCP ─────────► port set
                                │                              │
                          CI-V timeout                    source=FLEX
                          → release port                  tx=1 (active)
                          → GPIO4 HIGH                    tx=0 (released)
                          → Cmd 16/66                          │
                                │                         conflict signal
                          VITA-49 UDP ──────────────────► flex discovery
                          TCP server  ◄────────────────── sub tx all
                          → interlock state                    │
                                │                         SmartSDR (Port B)
                          web browser ◄── SSE ────────── live status + log
                                │
                          DOGL128 display ◄── encoder (manual antenna select)
```

**All three inhibit mechanisms fire together** via `setConflict(true, reason)`:
1. CI-V timeout (TRX off for 5 s)
2. Target antenna already in use by Port B (SmartSDR)

**All three release together** via `setConflict(false, reason)`:
1. New CI-V frame received (TRX back online)
2. Conflict resolved (antenna freed by Port B)

---

## Web Interface

Open `http://<device-ip>/` in any browser. Updates automatically via SSE.

| Section | Content |
|---|---|
| Funkgerät (CI-V) | Address, frequency, active band, TRX status (active / inhibited) |
| Antenna Genius | Name, IP, TCP port, serial, firmware, radio ports, antennas, stack mode, uptime, status |
| Port-Belegung | Port A+B antenna and band, TX indicator, conflict status, FlexRadio connection, PTT interlock state |
| Band-Konfiguration | Full band table from AG, active band highlighted |
| Antennen-Konfiguration | Full antenna list with TX masks |
| Debug Log | Live log, last 150 lines, auto-scroll |

Additional endpoints: `GET /api` (full JSON snapshot), `GET /events` (SSE stream with `status` and `log` events).

---

## Display & Encoder

Set `#define DISPLAY_ENABLED 1` to activate. `0` = zero overhead, no pins reserved.

**Status screen** (default): frequency (large), band, active antenna, AG status, conflict/inhibit indicator.

**Antenna selection menu**: scrollable list filtered by TX mask for the current band. Active antenna marked `[akt]`. Scroll bar shown when list exceeds 4 entries.

**Confirmation dialog**: press = switch antenna, rotate = cancel.

Menu closes automatically after `MENU_TIMEOUT_MS` (10 s) of inactivity.

| Action | Effect |
|---|---|
| Rotate (status screen) | Open antenna menu |
| Rotate (menu) | Navigate up/down |
| Rotate (confirm) | Cancel, return to menu |
| Press (status screen) | Open antenna menu |
| Press (menu) | Open confirmation dialog |
| Press (confirm) | Switch antenna, return to status |

---

## FlexRadio Emulation

Set `#define FLEX_EMULATION_ENABLED 1` to activate. `0` = zero overhead.

**Discovery broadcast** (UDP 255.255.255.255:4992, every 2 s): VITA-49 `ExtDataWithStream` packet with OUI `0x1C2D` (FlexRadio) and `PacketClassCode=0xFFFF`. Payload fields: `model`, `ip`, `port`, `serial`, `status=Available`, `version`, `nickname`.

**TCP server** (port 4992, one client = the AG): on connect sends version prologue `V3.3.15` and handle `H00000001`. Responds to `sub tx all` with current interlock state. All other commands answered with `R<seq>|00000000|`.

**PTT state mapping:**

| CI-V state | Interlock state sent to AG |
|---|---|
| Active (frames received, no timeout) | `state=PTT_REQUESTED source=SW` |
| Timeout / address unknown | `state=READY source=` |

Status updates are sent only when the state actually changes (`flexUpdatePttStatus()`).

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

### Configuration constants

```cpp
// Feature toggles
#define DISPLAY_ENABLED          1      // 0 = disable display + encoder entirely
#define FLEX_EMULATION_ENABLED   1      // 0 = disable FlexRadio emulation entirely

// CI-V
#define CIV_BAUD                 19200  // match transceiver setting
#define CIV_POLL_MS              500    // active poll interval when no broadcast
#define CIV_BROADCAST_TIMEOUT_MS 3000   // switch to polling after this ms
#define CIV_TIMEOUT_MS           5000   // release port + inhibit after this ms

// Antenna Genius
#define AG_RADIO_PORT            1      // 1 = Port A, 2 = Port B
#define AG_KEEPALIVE_MS          5000   // ping interval
#define AG_RECONNECT_MS          15000  // reconnect attempt interval
#define AG_CFG_TIMEOUT_MS        8000   // per-step config timeout
#define PORT_POLL_MS             3000   // port status polling interval

// Hardware
#define ETH_POWER_PIN            16     // -1 on some WT32-ETH01 revisions
#define CONFLICT_PIN             4      // HIGH = any inhibit active

// Display
#define DISP_CONTRAST            22     // 0–63, typical 20–30 at 3.3 V
// U8G2_R2 = 180° rotation — change to R0/R1/R3 to match mounting

// Encoder
#define ENC_DEBOUNCE_MS          5
#define MENU_TIMEOUT_MS          10000

// FlexRadio identity (shown in AG 'flex list')
#define FLEX_MODEL               "FLEX-6300"
#define FLEX_SERIAL              "CIV-Bridge-1"
#define FLEX_NICKNAME            "CIV-Bridge"
#define FLEX_VERSION             "3.3.15"
```

---

## TX Inhibit Mechanisms

The sketch uses three independent inhibit layers that are **always activated and released together** via `setConflict()`:

| Mechanism | Condition | How |
|---|---|---|
| GPIO4 HIGH | Any conflict or CI-V timeout | Hardware PTT inhibit line |
| CI-V Cmd 0x16/0x66 | Same | Sent to TRX address over UART2 |
| FlexRadio `PTT_REQUESTED` | CI-V active / timeout | Sent to AG via TCP |

Older Icom transceivers that do not support Cmd 0x16/0x66 will respond with `FA FA` (FAIL) or ignore the frame silently — no negative effect.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `emac_esp32_init: reset timeout` | Wrong `ETH_POWER_PIN` | Try `16` or `-1` |
| AG not found | UDP broadcast not reaching device | Check subnet; look for `[DISC]` in serial log |
| Config timeouts | AG slow / buffer issue | Increase `AG_CFG_TIMEOUT_MS` |
| AG ignores commands | Wrong line terminator or buffering | Verify `\r\n` + `flush()` in `sendAGCommand()` |
| All AG responses discarded | `agCfgSeq` set before `sendAGCommand()` | Must be `agSeq - 1` **after** the call |
| SmartSDR no conflict warning | AG not in FlexRadio mode | Check `[FLEX]` log; verify AG shows radio in `flex list` |
| TX not inhibited on timeout | TRX address not yet learned | Inhibit only sent after CI-V address is known |
| Encoder phantom steps | Missing pull-up resistors | 10 kΩ to 3.3 V on GPIO35, GPIO36, GPIO39 |
| Display blank / inverted | Wrong contrast or rotation | Adjust `DISP_CONTRAST`; try `U8G2_R0` through `R3` |

---

## License

GNU General Public License v3.0

*Developed with the assistance of [Claude.ai](https://claude.ai) by Anthropic.*
