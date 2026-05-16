# CLAUDE.md — Development Context

**Project:** WT32-ETH01 CI-V → Antenna Genius Bridge  
**File:** `WT32_CIV_AntennaGenius.ino` (~3400 lines, single-file Arduino sketch)  
**Hardware:** WT32-ETH01 (ESP32 + LAN8720 Ethernet)

Read this before modifying any protocol-related code. Sections marked **CRITICAL** have caused real bugs during development.

---

## Architecture

### Loop structure (strictly non-blocking)

```
loop()
 ├── webServer.handleClient()
 ├── handleSSEClient()
 ├── flexLoop()                    ← VITA-49 broadcast + AG-server + FLEX-client
 ├── [return if !ethConnected]
 ├── discoverAntennaGenius()
 ├── connectToAG()
 ├── handleAGReceive()             ← line reader + config state machine
 ├── menuHandleEncoder()           ← #if DISPLAY_ENABLED
 ├── [return if !agConfigDone]
 ├── handleCIV()                   ← UART2 reader + timeout + TX-status poll
 ├── sendKeepalive()
 ├── pollPortStatus()
 └── displayLoop()                 ← #if DISPLAY_ENABLED
```

### AG config state machine (`agCfgState`)
```
AGCFG_IDLE → AGCFG_WAIT_SUB_PORT → AGCFG_WAIT_BAND_LIST
           → AGCFG_WAIT_ANTENNA_LIST → AGCFG_DONE
```
Config starts 200 ms after prologue (`agConfigStartAt` non-blocking timer).

### Display menu states
```
MENU_STATUS ←── timeout / any press from MENU_INFO ─────────────────┐
  │ short press → MENU_ANT_SELECT                                     │
  │ long press ≥1s → MENU_INFO                                        │
  ▼                                                                    │
MENU_INFO (Info-Seite: IP, CI-V, AG, FLEX)                           │
MENU_ANT_SELECT ── press ──► MENU_CONFIRM ──► switch ────────────────┘
MENU_FLEX_SELECT  ← flexSelectPending=true (no timeout)
  │ short press → select radio
  │ long press ≥2s → select + save NVS default
```

---

## Antenna Genius Protocol — CRITICAL

### Command format
```
C<seq>|<command>\r\n
```
Use `agClient.write()` + `agClient.flush()`. **Never `print()`.**

### sendAGCfgCmd() — CRITICAL
Always use `sendAGCfgCmd()` for config commands — it sets `agCfgSeq = agSeq - 1` and `agCfgTimeout` atomically **after** the send. `agSeq` is incremented inside `sendAGCommand()`, so `agCfgSeq` must be set after the call, never before.

### Config sequence
1. `sub port all` — `all` mandatory; `sub antenna` does not exist
2. `band list` → `antenna list` — each ends with `R<seq>|0|` (empty message)
3. AG pushes `S0|port 1 ...` asynchronously → `parsePortLine()`

---

## CI-V Protocol

### Timing
- `CIV_BROADCAST_TIMEOUT_MS = 1000` ms → active freq poll (Cmd 0x03)
- TX status poll (Cmd 0x1C/0x00) every 200 ms once `civAddr != 0`
- `CIV_TIMEOUT_MS = 5000` ms → `releaseAGPort()` + `setConflict(true)`

### TX status polling — Cmd 0x1C/0x00
```
→ FE FE <civAddr> E0 1C 00 FD
← FE FE E0 <civAddr> 1C 00 <status> FD   (status: 0x00=RX, 0x01=TX)
```
Sets `civTransmitting`, `portTx[AG_RADIO_PORT-1]`, triggers `dispNeedsRedraw` and `ssePushStatus()`.

### TX Inhibit — Cmd 0x16/0x66
```
FE FE <civAddr> E0 16 66 01/00 FD
```
Only sent when `civAddr != 0x00`. **Never call `sendCIVTxInhibit()` directly — use `setConflict()`.**

### GPIO register for pins 32–39 — CRITICAL
GPIO 36 and 39 lie in `GPIO.in1.val`, **not** `GPIO.in`. The encoder ISR reads:
```cpp
uint8_t a = (GPIO.in1.val >> (ENC_A_PIN - 32)) & 1;
uint8_t b = (GPIO.in1.val >> (ENC_B_PIN - 32)) & 1;
```
Using `GPIO.in` returns 0 for these pins — the encoder appears non-functional.

---

## setConflict() — Single Point of Control — CRITICAL

```
setConflict(bool active, const char *reason)
 ├── early exit if conflictActive == active
 ├── conflictActive = active
 ├── digitalWrite(CONFLICT_PIN, ...)
 ├── if active: build conflictReason (civAddr + civFreqStr + antennas[wantedAntId-1].name)
 │   NOTE: called BEFORE wantedAntId = 0 so the antenna name is still available
 ├── webLog(...)
 ├── sendCIVTxInhibit(active)
 ├── flexSendInterlockStatus(active)   → AG clients  [FLEX_EMULATION_ENABLED]
 ├── flexRadioSendInterlock(active)    → FLEX radio  [FLEX_EMULATION_ENABLED]
 ├── displaySetInvert(active)          [DISPLAY_ENABLED]
 ├── dispNeedsRedraw = true            [DISPLAY_ENABLED]
 └── ssePushStatus()
```

---

## Conflict Detection — CRITICAL

### wantedAntId

The AG sets `txant=0` when `inhibit=1`. Without a tracking variable, `portTxAnt[AG_RADIO_PORT-1]` becomes 0, causing false "no conflict" decisions and `---` in the web UI antenna display.

- Set in `setAGPort()` for own port only
- Cleared **after** `setConflict(true)` in `releaseAGPort()` so `conflictReason` can show the antenna name
- Never overwritten by `parsePortLine()` or AG status updates

### checkAndSignalConflict(wantedAnt)

Conflict condition:
```cpp
bool conflict = (wantedAnt != 0) && (otherTxAnt != 0) && (otherTxAnt == wantedAnt);
```
`otherTxAnt == 0` → other port has no antenna configured → no conflict possible.

Pass the explicit value whenever available (`antId` from `handleCIV`/`menuSelectAntenna`). Pass `wantedAntId` from `parsePortLine()` re-check.

### parsePortLine() re-check — CRITICAL
Every time **the other port** changes, `checkAndSignalConflict(wantedAntId)` is called. Guard: only when `wantedAntId > 0` (own port is active). This resolves conflicts immediately when SmartSDR changes band without waiting for the next CI-V frame.

### source=AUTO guard
When AG reports `source=AUTO` for own port and `wantedAntId == 0`, the `txant` value is replaced with `effectiveTxant = 0`. Prevents the last session's state from being treated as current after reboot or CIV-timeout.

### Band-ID vs. array index — CRITICAL
```cpp
// portBand[] = AG Band-ID (1-based, from "band=N" in AG status)
// bands[] = 0-based array with .id field matching AG Band-ID
// WRONG:  bands[portBand[0]].name   ← off-by-one
// CORRECT:
for (uint8_t i = 0; i < MAX_BANDS; i++)
    if (bands[i].valid && bands[i].id == portBand[0]) { ... }

// lastBandId from freqToBandId() IS a 0-based array index:
bands[lastBandId].name   ← correct
```

---

## Antenna Menu Filtering

`menuBuildAntList()` excludes two categories:
```cpp
uint8_t activeAnt  = currentAntId();           // own active antenna
uint8_t blockedAnt = portTxAnt[otherIdx];      // SmartSDR's active antenna
#define ANT_BLOCKED(aid) ((aid)==activeAnt || ((aid)==blockedAnt && blockedAnt!=0))
```
This prevents selecting an antenna already in use on either port — including antennas used by SmartSDR on the other port even for different bands.

---

## FlexRadio Server (AG Interlock)

### Connect sequence
```
← V3.3.15 + H00000001     (prologue)
→ keepalive enable
← R|00000000|
← S00000000|interlock state=PTT_REQUESTED/READY ...   ← proactive push
```
Status is sent **immediately after `keepalive enable`** — the AG never sends `sub tx`. `flexSendInterlockStatus()` sets `flexTxActive` to keep it in sync with the actually sent status.

---

## FlexRadio Client (SmartSDR Direct Interlock)

### UDP sockets — CRITICAL
```cpp
static WiFiUDP flexDiscUdp;    // SEND ONLY — no begin(), ephemeral port
static WiFiUDP flexListenUdp;  // RECEIVE ONLY — begin(4992) in flexSetup()
```
Do not call `flexDiscUdp.begin()` — it breaks simultaneous send/receive on ESP32.

### Protocol
```
→ C1|client program CIV-Bridge
→ C2|interlock create type=ANT model=CIV-Bridge serial=DK4DJ-1 valid_antennas=
← R2|0|000000F4             ← hex interlock ID → strtol(..., 16)
→ C3|interlock not_ready 000000F4
→ C4|interlock ready 000000F4
```
- Interlock ID is **hexadecimal** — use `strtol(p2+1, nullptr, 16)`, not `atoi()`
- Error code is hexadecimal — use `strtoul(hexCode, nullptr, 16) != 0`
- Line terminator: `\r\n` (not just `\n`)
- ANT interlock starts in `ready` state — force send by setting `flexRadioInhibit = true` before calling `flexRadioSendInterlock(conflictActive)` after registration
- `flexRadioSend()` is `static void` (file-scope helper, no forward declaration)

### FLEX radio selection priority
1. AG Port 2 `source=FLEX nickname=X` → match `agPort2FlexNickname` against list
2. `flexDefaultIP` from NVS (namespace `"flex"`, key `"defaultIP"`)
3. `flexRadioCount == 1` → auto-select
4. Multiple radios → `flexSelectPending = true` → web card + `MENU_FLEX_SELECT`

### agPort2FlexNickname — CRITICAL placement
Declared **globally** (not inside `#if FLEX_EMULATION_ENABLED`) because `parsePortLine()` uses it unconditionally. Do not move it inside the guard.

### First connection timing
`flexRadioLastTry = FLEX_RADIO_RECONNECT_MS` at init → first connect attempt is immediate when IP becomes known.

---

## buildStatusJson() — Single JSON Source

Both `ssePushStatus()` and `handleApiJson()` call `buildStatusJson()`. `handleApiJson()` removes the trailing `}` and appends `bands[]` and `antennas[]`.

**antA/antB fallback:**
```cpp
uint8_t ia = portTxAnt[0];
if (ia == 0 && AG_RADIO_PORT == 1) ia = wantedAntId;  // inhibit=1 fallback
```

**civTx field:** `"civTx": true/false` from `civTransmitting` (Cmd 0x1C/0x00), separate from `txA`/`txB` which come from AG status.

---

## Logging

Format: `mm:ss.mmm [TAG] message`. Ring buffer 300 entries. `GET /log` for download.

| Prefix | Source |
|---|---|
| `[ETH]` | Ethernet events |
| `[DISC]` | AG UDP discovery |
| `[AG]` | AG TCP commands/responses |
| `[CIV]` | CI-V frames (freq + TX status) |
| `[CONF]` | setConflict() calls |
| `[FLEX:N]` | FlexRadio server client N |
| `[FLEX]` | FlexRadio server broadcast |
| `[FRINT]` | FlexRadio client (SmartSDR interlock) |
| `[DISP]` | Display/encoder |
| `[WEB]` | Web server |

Enable `#define DEBUG_CONFLICT 1` for per-check conflict diagnostic logging.

---

## Key Variables Reference

| Variable | Type | Description |
|---|---|---|
| `wantedAntId` | `uint8_t` | Last antenna requested via `setAGPort()`. Fallback in `checkAndSignalConflict(0)` and `buildStatusJson()`. Cleared **after** `setConflict()` in `releaseAGPort()`. |
| `civTransmitting` | `bool` | True when TRX is transmitting (Cmd 0x1C/0x00). Drives `portTx[]`, display TX/RX, web `civTx` field. |
| `conflictActive` | `bool` | Current conflict state. Toggled only by `setConflict()`. |
| `conflictReason` | `char[64]` | Built by `setConflict(true,...)`, cleared by `setConflict(false,...)`. Contains CI-V addr, freq, antenna name. |
| `portTxAnt[2]` | `uint8_t[]` | txant from AG status. **May be 0 during inhibit=1**. Use `wantedAntId` as fallback. |
| `portBand[2]` | `uint8_t[]` | AG Band-ID (1-based). Search `bands[i].id == portBand[x]` for name. |
| `lastBandId` | `int8_t` | 0-based index into `bands[]` from `freqToBandId()`. Use `bands[lastBandId]` directly. |
| `agPort2FlexNickname` | `char[32]` | FLEX radio nickname from AG Port 2 `source=FLEX`. **Global scope** (not in FLEX guard). |
| `flexListenUdp` | `WiFiUDP` | Receive-only, `begin(4992)` in `flexSetup()`. |
| `flexDiscUdp` | `WiFiUDP` | Send-only, **no `begin()`**. |
| `flexRadioLastTry` | `uint32_t` | Init to `FLEX_RADIO_RECONNECT_MS` for immediate first connect. |
| `effectiveTxant` | local `uint8_t` | In `parsePortLine()`: 0 when AG reports source=AUTO for own port and `wantedAntId==0`. |

---

## Bug History

| Symptom | Root cause | Fix |
|---|---|---|
| ETH reset timeout | `ETH_POWER_PIN` wrong | `16` or `-1` |
| AG ignores commands | `print()` + `\r` only | `write()` + `flush()` + `\r\n` |
| All AG responses discarded | `agCfgSeq = agSeq` before send | `sendAGCfgCmd()` |
| No `[FRINT]` entries | `flexDiscUdp.begin(4992)` blocked send | Separate `flexListenUdp` |
| Interlock ID = -1 | `atoi()` on hex ID | `strtol(..., 16)` |
| Conflict not resolved | `parsePortLine` only checked if `conflictActive` | Re-check on every other-port update |
| False conflict at startup | AG returns last session state (source=AUTO) | `effectiveTxant = 0` guard |
| Conflict persists after release | `portTxAnt` not reset in `releaseAGPort()` | Explicit reset to 0 |
| antA/antB shows `---` | `portTxAnt=0` during inhibit | `wantedAntId` fallback |
| bandA/bandB wrong name | `bands[portBand[0]]` off-by-one | Search by `bands[i].id == portBand[x]` |
| Encoder not working | `GPIO.in` used for pins 32–39 | `GPIO.in1.val >> (PIN-32)` |
| Encoder skips/doubles | Wrong pulses-per-detent | `ENC_PULSES_PER_DETENT` define |
| SmartSDR antenna selectable in menu | `menuBuildAntList()` didn't filter other port | `blockedAnt = portTxAnt[otherIdx]` |
| Display not updating | `dispNeedsRedraw` not set on CI-V events | Set on `civFreqMHz` and `civTransmitting` change |
| TX status always RX | Cmd 0x1C not sent | `requestCIVTxStatus()` every 200 ms |
