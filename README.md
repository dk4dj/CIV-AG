# WT32-ETH01 — CI-V → Antenna Genius Bridge

Arduino sketch for the **WT32-ETH01** (ESP32 + LAN8720). Reads Icom CI-V frequency and TX status, automatically switches a **4O3A Antenna Genius** antenna selector, and provides reliable TX inhibit for both the Icom transceiver and SmartSDR/FlexRadio via a two-layer interlock implementation. An optional **EA DOGL128W-6** display with rotary encoder provides on-device status, antenna selection, and system information.

---

## Features

- **Automatic AG discovery** — UDP broadcast on port 9007; extracts IP, port, serial, firmware, ports, antennas, mode, uptime
- **Full AG configuration readout** — band table and antenna list on every connect
- **CI-V frequency tracking** on UART2 — transceiver address learned automatically; active polling (Cmd 0x03) after `CIV_BROADCAST_TIMEOUT_MS` (1 s)
- **CI-V TX status** (Cmd 0x1C/0x00) polled every 200 ms — drives TX/RX indicator on display and web UI
- **Automatic band → antenna mapping** based on AG TX masks
- **Four-layer TX protection** — all controlled together via `setConflict()`:
  - GPIO4 HIGH → hardware PTT inhibit line
  - CI-V Cmd 0x16/0x66 → TX Inhibit on newer Icom transceivers
  - FlexRadio server `state=PTT_REQUESTED` → AG sets `inhibit=1` on Port B
  - SmartSDR Ethernet Interlock `not_ready` → SmartSDR TX blocked directly
- **Conflict reason display** — web UI and log show which transceiver was inhibited with address, frequency and antenna name
- **Correct conflict detection** — `wantedAntId` tracks the last requested antenna independently of AG status (AG sets `txant=0` when `inhibit=1`); re-evaluated on every Port B update
- **Source=AUTO guard** — ignores AG startup state for own port until first CI-V frame received
- **CI-V timeout** (`CIV_TIMEOUT_MS` = 5 s) — releases port and activates all inhibit mechanisms
- **FlexRadio two-layer implementation:**
  - **Server** (port 4992, up to `FLEX_MAX_CLIENTS` = 4): emulates a FlexRadio for the AG; sends `PTT_REQUESTED` proactively after `keepalive enable`
  - **Client**: connects to real FLEX radio; `interlock create type=ANT model=... valid_antennas=`; `not_ready`/`ready` with hex interlock ID
- **Separate UDP sockets** — `flexDiscUdp` (send-only) and `flexListenUdp` (receive-only on port 4992) to avoid ESP32 conflict
- **Automatic FLEX radio selection** — priority: AG Port 2 nickname → NVS default IP → single radio → user selection
- **Persistent default radio** — NVS (`Preferences`), survives reboot
- **Shared JSON builder** `buildStatusJson()` — identical data for SSE push and `/api` polling
- **Band-ID lookup** by `bands[i].id` (AG Band-IDs are 1-based; array is 0-based)
- **Splash screen** at boot (128×64 px, PROGMEM)
- **Display inversion** on any conflict or CI-V timeout (ST7565 hardware invert)
- **Built-in web interface** on port 80 with live SSE, FLEX selection panel, conflict detail
- **Log export** — `GET /log`, 300 timestamped entries
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
| Display A0 | 2 | Strapping pin — safe after boot |
| Display RST | 33 | Active low |
| Encoder A | 39 | Input-only, ISR CHANGE, ext. 10 kΩ pull-up **required** |
| Encoder B | 36 | Input-only, ISR CHANGE, ext. 10 kΩ pull-up **required** |
| Encoder button | 35 | Input-only, polled, ext. 10 kΩ pull-up **required** |

> GPIO 35/36/39 are input-only with no internal pull-ups. GPIO 36/39 lie in register `GPIO.in1.val` (not `GPIO.in`). The ISR reads both pins as `(GPIO.in1.val >> (PIN - 32)) & 1`.

---

## Display & Encoder

### Menu states

| State | How to enter | How to leave |
|---|---|---|
| `MENU_STATUS` | Default / timeout | Short press → `MENU_ANT_SELECT`; long press ≥1s → `MENU_INFO` |
| `MENU_INFO` | Long press ≥1s from Status | Any press → `MENU_STATUS` |
| `MENU_ANT_SELECT` | Short press from Status or rotate | Press → `MENU_CONFIRM`; timeout |
| `MENU_CONFIRM` | Press from Ant Select | Press (confirm) or timeout |
| `MENU_FLEX_SELECT` | Automatic when multiple FLEX radios found | Short press = select; long press ≥2s = select + set NVS default |

### Status screen layout
```
21.465 MHz              RX     ← TX/RX indicator (inverted block when TX)
Band: 15m
Ant:  Beam_OB11-3
AG: OK  Port A           ← bottom status bar
```

### Info screen (long press)
```
WT32: 192.168.1.202
AG:   192.168.1.221
FW:4.1.8 S/N:C7-F6-42
FLEX: 192.168.1.100
      (Flexi)
────────────────────
CI-V: 0x94   19200 Bd
[Taste]=zurück
```

### Encoder: full 4-state Gray-code decoding
Both channels (GPIO 39/36) trigger `CHANGE` interrupts. The ISR uses a 4-bit state machine — only valid Gray-code transitions count; glitches and crosstalk are filtered automatically. Adjust `ENC_PULSES_PER_DETENT` (default 4) if steps are missed or doubled.

---

## Web Interface

Open `http://<device-ip>/` in any browser.

| Section | Content |
|---|---|
| Funkgerät (CI-V) | Address, frequency, active band, TX/RX status |
| Antenna Genius | Name, IP, firmware, uptime, status |
| Port-Belegung | Port A/B antenna/band, **Funkgerät TX** (from Cmd 0x1C), conflict badge with inhibited TRX details, FlexRadio status |
| FLEX-Radio-Auswahl | Yellow warning card when multiple radios found |
| Band-/Antennen-Konfiguration | Full AG tables |
| Debug-Log | Live SSE log, 300 entries, download |

**Endpoints:** `GET /`, `GET /api`, `GET /events`, `GET /log`, `POST /flex-select?ip=`, `POST /flex-default?ip=`

---

## CI-V Protocol Details

| Command | Direction | Purpose |
|---|---|---|
| `FE FE <trx> E0 03 FD` | WT32 → TRX | Poll frequency |
| `FE FE <trx> E0 00 FD` | TRX → WT32 | Frequency broadcast |
| `FE FE E0 <trx> 00 ... FD` | TRX → WT32 | Frequency response |
| `FE FE <trx> E0 1C 00 FD` | WT32 → TRX | Poll TX status (every 200 ms) |
| `FE FE E0 <trx> 1C 00 00 FD` | TRX → WT32 | RX status |
| `FE FE E0 <trx> 1C 00 01 FD` | TRX → WT32 | TX status |
| `FE FE <trx> E0 16 66 01 FD` | WT32 → TRX | TX Inhibit ON |
| `FE FE <trx> E0 16 66 00 FD` | WT32 → TRX | TX Inhibit OFF |

---

## FlexRadio Interlock

### Server (AG interlock)
- VITA-49 discovery broadcast every 2 s + TCP server port 4992
- Sends `S00000000|interlock state=PTT_REQUESTED` immediately after `keepalive enable`
- Log prefix: `[FLEX:N]`

### Client (SmartSDR direct interlock)
```
→ C1|client program CIV-Bridge
→ C2|interlock create type=ANT model=CIV-Bridge serial=DK4DJ-1 valid_antennas=
← R2|0|000000F4          ← hex interlock ID
→ C3|interlock not_ready 000000F4   ← conflict
→ C4|interlock ready 000000F4       ← resolved
```
- Log prefix: `[FRINT]`
- FLEX radio IP from `ip=` field in VITA-49 payload (more reliable than UDP sender IP)
- Auto-select priority: AG Port 2 nickname → NVS default → single radio → user

---

## TX Inhibit Mechanisms

All controlled via `setConflict(bool active, const char *reason)`:

| Layer | Mechanism | Effect |
|---|---|---|
| GPIO4 HIGH | `digitalWrite` | Hardware PTT line to TRX |
| CI-V 0x16/0x66 | Serial2 frame | Icom TX Inhibit (newer models) |
| FlexRadio server | `PTT_REQUESTED` status | AG `inhibit=1` on Port B |
| FlexRadio client | `interlock not_ready` | SmartSDR TX blocked directly |

`conflictReason` is built with CI-V address, frequency and antenna name. Set before `wantedAntId = 0` so the last active antenna appears in the conflict message.

---

## Antenna Conflict Detection

### wantedAntId

The AG sets `txant=0` in status messages when `inhibit=1`. Without a separate tracking variable `portTxAnt[AG_RADIO_PORT-1]` would become 0, causing false "no conflict" decisions.

`wantedAntId` is set in `setAGPort()`, cleared in `releaseAGPort()`, and never overwritten by AG status updates. `checkAndSignalConflict()` uses it as fallback when called with argument 0.

### Antenna menu filtering (`menuBuildAntList`)

The selection menu excludes:
- `activeAnt` — already active on own port
- `blockedAnt = portTxAnt[otherIdx]` — currently used by SmartSDR

This prevents selecting an antenna that is already occupied on either port.

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

| Library | When required |
|---|---|
| ETH, WiFiUdp, WiFiClient, WiFiServer, WebServer, Preferences | Always (ESP32 core) |
| U8g2 by olikraus | `DISPLAY_ENABLED 1` |

### Key Configuration Constants

```cpp
#define DISPLAY_ENABLED          1
#define FLEX_EMULATION_ENABLED   1
#define AG_RADIO_PORT            1      // 1=Port A, 2=Port B
#define ETH_POWER_PIN            16     // try -1 on some board revisions
#define CONFLICT_PIN             4
#define CIV_BAUD                 19200
#define CIV_BROADCAST_TIMEOUT_MS 1000   // active poll after 1 s without broadcast
#define CIV_TIMEOUT_MS           5000   // release port after 5 s without CI-V
#define FLEX_INTERLOCK_NAME      "CIV-Bridge"
#define FLEX_INTERLOCK_SERIAL    "DK4DJ-1"
#define FLEX_MAX_CLIENTS         4
#define FLEX_MAX_RADIOS          8
#define ENC_PULSES_PER_DETENT    4      // 4 for most encoders; try 2 or 1 if steps skip
#define DISP_CONTRAST            45
#define DEBUG_CONFLICT           0      // set to 1 for conflict diagnostic logging
#define LOG_BUF_LINES            300
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `TG1WDT_SYS_RESET` at boot | Wrong `ETH_POWER_PIN` | Try `ETH_POWER_PIN -1` |
| AG not found | UDP blocked | Check `[DISC]` in log |
| SmartSDR still transmits | FLEX radio not connected | Check `[FRINT]` entries in log |
| No `[FRINT]` entries | `flexListenUdp` not receiving | Verify port 4992 not blocked |
| Interlock ID −1 | `interlock create` rejected | Check `[FRINT] <<` responses |
| Conflict after band change | AG `txant=0` during inhibit | Handled by `wantedAntId` |
| Port A/B shows `---` | Port released after timeout | Normal — clears on next CI-V |
| Encoder skips steps | Wrong `ENC_PULSES_PER_DETENT` | Try values 1, 2, 4 |
| TX indicator wrong | TRX does not respond to Cmd 0x1C | Check CI-V Transceive setting |
| Conflict at startup | AG reports old session state | Handled by source=AUTO guard |

---

## License

GNU General Public License v3.0

*Developed with the assistance of [Claude.ai](https://claude.ai) by Anthropic.*
