# CLAUDE.md — Development Context

**Project:** WT32-ETH01 CI-V → Antenna Genius Bridge  
**File:** `WT32_CIV_AntennaGenius.ino` (~3160 lines, single-file Arduino sketch)  
**Hardware:** WT32-ETH01 (ESP32 + LAN8720 Ethernet)

Read this before modifying any protocol-related code. Every section marked **CRITICAL** has caused real bugs during development.

---

## Architecture

### Loop structure (strictly non-blocking)

```
loop()
 ├── webServer.handleClient()      ← always first, every iteration
 ├── handleSSEClient()
 ├── flexLoop()                    ← Discovery broadcast + AG-server + FLEX-client
 ├── [return if !ethConnected]
 ├── discoverAntennaGenius()
 ├── connectToAG()
 ├── handleAGReceive()             ← line reader + config state machine
 ├── menuHandleEncoder()           ← #if DISPLAY_ENABLED
 ├── [return if !agConfigDone]
 ├── handleCIV()                   ← UART2 reader + timeout check
 ├── sendKeepalive()
 ├── pollPortStatus()
 └── displayLoop()                 ← #if DISPLAY_ENABLED
```

`flexLoop()` calls `flexHandleTcpClient()` (server for AG) + `flexRadioLoop()` (client to FLEX radio).

### State machines

**AG config state** (`agCfgState`):
```
AGCFG_IDLE → AGCFG_WAIT_SUB_PORT → AGCFG_WAIT_BAND_LIST → AGCFG_WAIT_ANTENNA_LIST → AGCFG_DONE
```
Config starts 200 ms after prologue (`agConfigStartAt` non-blocking timer).

**Display menu state** (`menuState`):
```
MENU_STATUS ←── timeout ──────────────────────────────────┐
     │ rotate/press                                         │
     ▼                                                      │
MENU_ANT_SELECT ── press ──► MENU_CONFIRM ──► switch ──────┘

MENU_FLEX_SELECT  ← flexSelectPending=true (no timeout)
     │ press short → flexRadioSelectByIdx()
     │ press long ≥2s → select + save NVS default
```

---

## Antenna Genius Protocol — CRITICAL

### Command format
```
C<seq>|<command>\r\n
```
Use `agClient.write()` + `agClient.flush()`. **Never `print()`.**

### sendAGCfgCmd() — CRITICAL
```cpp
// WRONG — always use sendAGCfgCmd() instead:
sendAGCommand("band list");
agCfgSeq = agSeq - 1;
agCfgTimeout = millis() + AG_CFG_TIMEOUT_MS;

// CORRECT:
sendAGCfgCmd("band list");
```
`sendAGCfgCmd()` sets `agCfgSeq = agSeq - 1` and `agCfgTimeout` atomically after the send.
**agCfgSeq must be set AFTER sendAGCommand() because agSeq is incremented inside it.**

### Config sequence
1. `sub port all` — `all` mandatory; `sub antenna` does not exist
2. `band list` → `antenna list` — each block ends with `R<seq>|0|` (empty message)
3. AG pushes `S0|port 1 ...` / `S0|port 2 ...` asynchronously → `parsePortLine()`

---

## CI-V Protocol

### Timing
- `CIV_BROADCAST_TIMEOUT_MS = 1000` ms without broadcast → active polling (Cmd 0x03)
- `CIV_TIMEOUT_MS = 5000` ms without any CI-V → `releaseAGPort()` + `setConflict(true)`

### TX Inhibit — Cmd 0x16/0x66
```
FE FE <civAddr> E0 16 66 01/00 FD   (01=ON, 00=OFF)
```
Only sent when `civAddr != 0x00`. **Never call `sendCIVTxInhibit()` directly — always use `setConflict()`.**

---

## setConflict() — Single Point of Control — CRITICAL

```cpp
void setConflict(bool active, const char *reason)
 ├── early exit if conflictActive == active
 ├── conflictActive = active
 ├── digitalWrite(CONFLICT_PIN, ...)
 ├── if active: build conflictReason string (civAddr + civFreqStr + antName)
 ├── webLog(...)
 ├── sendCIVTxInhibit(active)
 ├── flexSendInterlockStatus(active)   // server → AG clients  [FLEX_EMULATION_ENABLED]
 ├── flexRadioSendInterlock(active)    // client → FLEX radio  [FLEX_EMULATION_ENABLED]
 ├── displaySetInvert(active)          // [DISPLAY_ENABLED]
 ├── dispNeedsRedraw = true            // [DISPLAY_ENABLED]
 └── ssePushStatus()
```

`conflictReason` is built from `civAddr`, `civFreqStr`, and `antennas[wantedAntId-1].name`. Displayed in the web UI badge and banner.

---

## Conflict Detection — CRITICAL

### wantedAntId

The AG sets `txant=0` in status messages whenever `inhibit=1`. This overwrites `portTxAnt[AG_RADIO_PORT-1]` to 0, which would cause:
1. `checkAndSignalConflict()` to see "no antenna wanted" → no conflict detected
2. Web UI to show `---` for antenna name

**Solution:** `wantedAntId` tracks the last requested antenna independently:
- Set in `setAGPort()` for `AG_RADIO_PORT` only
- Cleared in `releaseAGPort()` to 0
- Never touched by `parsePortLine()` or AG status updates

### checkAndSignalConflict(wantedAnt)

```cpp
// Pass 0 to use wantedAntId as fallback:
checkAndSignalConflict(0);      // from parsePortLine() re-check
checkAndSignalConflict(antId);  // from handleCIV() / menuSelectAntenna()
```

### parsePortLine() re-check — CRITICAL

Every time **the other port** changes state, `parsePortLine()` calls `checkAndSignalConflict(0)`. This ensures conflicts are detected/resolved immediately when SmartSDR changes band — without waiting for the next CI-V frame from the Icom radio.

```cpp
if (portId == otherPort) {
    checkAndSignalConflict(0);   // uses wantedAntId
}
```

No `conflictActive` guard — runs on every port update of the other port.

### Band-ID vs. Array index — CRITICAL

```cpp
// portBand[] contains AG Band-ID (1-based, from "band=N" in AG status)
// bands[] is 0-based array with .id field
// WRONG: bands[portBand[0]].name   ← off-by-one
// CORRECT:
for (uint8_t i = 0; i < MAX_BANDS; i++)
    if (bands[i].valid && bands[i].id == portBand[0]) { bandA = bands[i].name; break; }

// lastBandId from freqToBandId() IS a 0-based array index (iterates from i=1):
bands[lastBandId].name   ← correct
```

---

## buildStatusJson() — Single JSON Source

Both `ssePushStatus()` and `handleApiJson()` call `buildStatusJson()`. This ensures the 30-second `loadConfig()` polling and live SSE pushes always return identical field sets.

`handleApiJson()` calls `buildStatusJson()`, removes the trailing `}`, and appends `bands[]` and `antennas[]`.

**antA/antB fallback:**
```cpp
uint8_t ia = portTxAnt[0];
if (ia == 0 && (AG_RADIO_PORT == 1)) ia = wantedAntId;  // inhibit=1 fallback
```

---

## FlexRadio Server (AG Interlock)

### Connect sequence
```
← V3.3.15\n + H00000001\n     (prologue)
→ C2|keepalive enable
← R2|00000000|\n
← S00000000|interlock state=PTT_REQUESTED/READY ...\n   ← proactive push
```
**Status is sent after `keepalive enable`, NOT after `sub tx all`** — the AG never sends `sub tx`.

### ssePushStatus() timing

`ssePushStatus()` is called from `setConflict()` and from `flexRadioSendInterlock()`. At the time `setConflict(false)` fires, `portTxAnt[AG_RADIO_PORT-1]` may still be 0 (AG hasn't confirmed yet). `wantedAntId` is used as fallback in `buildStatusJson()`.

---

## FlexRadio Client (SmartSDR Direct Interlock)

### UDP sockets — CRITICAL

Two separate sockets prevent the ESP32 UDP send/receive conflict:
```cpp
static WiFiUDP flexDiscUdp;    // send only — no begin(), used by flexSendDiscovery()
static WiFiUDP flexListenUdp;  // receive only — begin(4992), used by flexRadioLoop()
```
**Do NOT call `flexDiscUdp.begin()`** — it breaks simultaneous send.
**`flexListenUdp.begin(4992)` is called in `flexSetup()`.**

### Protocol
```
→ C1|client program CIV-Bridge
→ C2|interlock create type=ANT name=CIV-Bridge serial=DK4DJ-1
← R2|0|<interlock_id>
→ C3|interlock not_ready <id>   ← conflict
→ C4|interlock ready <id>       ← resolved
```
`flexRadioSend()` is `static void` (file-scope only). Log prefix: `[FRINT]`.

### ssePushStatus() after interlock changes

`flexRadioSendInterlock()` calls `ssePushStatus()` after each send. `flexRadioLoop()` calls `ssePushStatus()` after successful connect. This ensures the web UI updates immediately.

### Radio discovery and selection

VITA-49 packets received on `flexListenUdp`. OUI check: `buf[8]==0x00 && buf[9]==0x1C && buf[10]==0x2D`. IP extracted from `ip=` field in ASCII payload (more reliable than UDP sender IP with NAT).

**Auto-select priority:**
1. AG Port 2 `source=FLEX nickname=X` → `agPort2FlexNickname` match
2. `flexDefaultIP` from NVS
3. `flexRadioCount == 1` → auto
4. Multiple → `flexSelectPending = true` → web card + `MENU_FLEX_SELECT`

**`agPort2FlexNickname` is declared globally** (not inside `#if FLEX_EMULATION_ENABLED`) because `parsePortLine()` uses it unconditionally. Do not move it inside the guard.

### First connection timing

`flexRadioLastTry` is initialized to `FLEX_RADIO_RECONNECT_MS` so that `millis() - flexRadioLastTry` is already large at startup → first connection attempt is immediate after IP is known.

---

## Logging

All entries: `mm:ss.mmm [TAG] message`. Ring buffer 300 entries. Export via `GET /log`.

| Prefix | Source |
|---|---|
| `[ETH]` | Ethernet events |
| `[DISC]` | AG UDP discovery |
| `[AG]` | AG TCP commands/responses |
| `[CIV]` | CI-V frames |
| `[CONF]` | setConflict() calls (includes conflictReason) |
| `[FLEX:N]` | FlexRadio server client N |
| `[FLEX]` | FlexRadio server broadcast |
| `[FRINT]` | FlexRadio client (SmartSDR interlock) |
| `[DISP]` | Display/encoder events |
| `[WEB]` | Web server events |

---

## Web Endpoints

| Endpoint | Method | Description |
|---|---|---|
| `/` | GET | HTML dashboard with SSE |
| `/api` | GET | Full JSON snapshot (buildStatusJson + bands + antennas) |
| `/events` | GET | SSE stream |
| `/log` | GET | Log file download |
| `/flex-select` | POST | Select FLEX radio (`ip=x.x.x.x`) |
| `/flex-default` | POST | Set + persist default FLEX radio |

---

## Key Variables Reference

| Variable | Type | Description |
|---|---|---|
| `wantedAntId` | `uint8_t` | Last antenna requested via `setAGPort()`. Never 0 while port is active. Falls back in `checkAndSignalConflict(0)` and `buildStatusJson()`. |
| `conflictActive` | `bool` | Current conflict state. Toggled only by `setConflict()`. |
| `conflictReason` | `char[64]` | Human-readable reason string built by `setConflict(true,...)`. Cleared by `setConflict(false,...)`. Contains CI-V addr, freq, antenna name. |
| `portTxAnt[2]` | `uint8_t[]` | txant from AG status. **May be 0 during inhibit=1**. Use `wantedAntId` as fallback. |
| `portBand[2]` | `uint8_t[]` | AG Band-ID (1-based). Search `bands[i].id == portBand[x]` for name. |
| `lastBandId` | `int8_t` | 0-based index into `bands[]` from `freqToBandId()`. Use `bands[lastBandId]` directly. |
| `agPort2FlexNickname` | `char[32]` | FLEX radio nickname from AG Port 2 `source=FLEX`. Global scope (not in guard). |
| `flexListenUdp` | `WiFiUDP` | Receive-only UDP socket, `begin(4992)`. Separate from `flexDiscUdp`. |
| `flexDiscUdp` | `WiFiUDP` | Send-only UDP socket. No `begin()` call. |
| `flexRadioLastTry` | `uint32_t` | Init to `FLEX_RADIO_RECONNECT_MS` for immediate first connect. |

---

## Bug History

| Symptom | Root cause | Fix |
|---|---|---|
| ETH reset timeout | `ETH_POWER_PIN` wrong | `16` or `-1` |
| AG ignores all commands | `print()` + `\r` only | `write()` + `flush()` + `\r\n` |
| All AG responses discarded | `agCfgSeq = agSeq` before send | `sendAGCfgCmd()` sets after |
| `delay(200)` blocked loop | Blocking delay | `agConfigStartAt` timer |
| No `[FRINT]` entries | `flexDiscUdp.begin(4992)` blocked send | Separate `flexListenUdp` |
| FLEX radio never discovered | `flexListenUdp` not bound | `flexSetup()` calls `begin(4992)` |
| SmartSDR not inhibited | AG `inhibit=1` ignored by SmartSDR | Direct TCP client interlock |
| Conflict not detected after sequence | `portTxAnt=0` during `inhibit=1` | `wantedAntId` fallback |
| Conflict not cleared after other port changes band | `parsePortLine` only checked if `conflictActive` | Re-check on every other-port update |
| Port A/B shows `---` in web UI during conflict | `portTxAnt=0` fed to `buildStatusJson` | `wantedAntId` fallback in antA/antB |
| bandA/bandB wrong band name | `bands[portBand[0]]` (array index ≠ band ID) | Search by `bands[i].id == portBand[x]` |
| FLEX/interlock status not updated in web UI | `ssePushStatus()` not called after connect/interlock change | Added after `flexRadioConnected=true` and in `flexRadioSendInterlock()` |
| `handleApiJson` missing bandA/bandB | Fields not in old duplicate JSON block | `buildStatusJson()` shared function |
| 20 s delay before first FLEX connect | `flexRadioLastTry=0`, condition `millis()-0 < 20000` | Init to `FLEX_RADIO_RECONNECT_MS` |
| `agPort2FlexNickname` not declared | Inside `FLEX_EMULATION_ENABLED` guard | Moved to global scope |
| `flexRadioListAge` write-only | Never read | Removed |
| `flexSendLine()` (broadcast) dead code | No callers after refactor | Removed |
