# CLAUDE.md — Development Context

**Project:** WT32-ETH01 CI-V → Antenna Genius Bridge  
**File:** `WT32_CIV_AntennaGenius.ino` (~3080 lines, single-file Arduino sketch)  
**Hardware:** WT32-ETH01 (ESP32 + LAN8720 Ethernet)

Read this before modifying any protocol-related code. Every section marked **CRITICAL** has caused real bugs during development.

---

## Architecture

### Loop structure (strictly non-blocking)

```
loop()
 ├── webServer.handleClient()      ← always first, every iteration
 ├── handleSSEClient()
 ├── flexLoop()                    ← Discovery broadcast + multi-client TCP server + FLEX client
 ├── [return if !ethConnected]
 ├── discoverAntennaGenius()       ← until AG found
 ├── connectToAG()                 ← until connected
 ├── handleAGReceive()             ← line reader + config state machine + agConfigStartAt timer
 ├── menuHandleEncoder()           ← optional, runs even during config phase
 ├── [return if !agConfigDone]
 ├── handleCIV()                   ← UART2 reader + timeout check
 ├── sendKeepalive()
 ├── pollPortStatus()
 └── displayLoop()                 ← optional
```

`flexLoop()` calls both `flexHandleTcpClient()` (server for AG) and `flexRadioLoop()` (client to FLEX radio).

### State machines

**AG config state** (`agCfgState`):
```
AGCFG_IDLE → AGCFG_WAIT_SUB_PORT → AGCFG_WAIT_BAND_LIST → AGCFG_WAIT_ANTENNA_LIST → AGCFG_DONE
```
Config starts 200 ms after prologue (`agConfigStartAt` non-blocking timer).

**Display menu state** (`menuState`):
```
MENU_STATUS ←── timeout/cancel ──────────────────────────┐
     │ rotate/press                                        │
     ▼                                                     │
MENU_ANT_SELECT ── press ──► MENU_CONFIRM ──► switch ─────┘

MENU_FLEX_SELECT  ← flexSelectPending=true (no timeout)
     │ press (short) → flexRadioSelectByIdx()
     │ press (long ≥2s) → select + save NVS default
```

---

## Antenna Genius Protocol — CRITICAL

### Command format
```
C<seq>|<command>\r\n
```
Use `agClient.write()` + `agClient.flush()`. **Never `print()`.**

### Sequence number — CRITICAL
```cpp
sendAGCommand("some command");
agCfgSeq = agSeq - 1;   // ← AFTER the call, not before
```
`sendAGCommand()` increments `agSeq` **after** writing. Setting `agCfgSeq` before causes all responses to be silently discarded.

### Config sequence
1. `sub port all` — `all` mandatory; `sub antenna` does not exist
2. AG sends `S0|port 1 ...` / `S0|port 2 ...` asynchronously — no `port get` needed at startup
3. `band list` then `antenna list` — each ends with empty-message `R<seq>|0|`

### TCP port
`connectToAG()` uses `agTcpPort` from discovery with fallback to `AG_PORT` (9007).

---

## CI-V Protocol

### Frame format
```
FE FE <dst> <src> <cmd> [subcommand] [data] FD
```
Controller address: `0xE0`. TRX address: learned from first received frame.

### TX Inhibit — Cmd 0x16/0x66
```
FE FE <civAddr> E0 16 66 01/00 FD   (01=ON, 00=OFF)
```
Only sent when `civAddr != 0x00`. Older transceivers respond `FA FA` or ignore — no negative effect.

**Never call `sendCIVTxInhibit()` directly — always use `setConflict()`.**

### CI-V timeout
After `CIV_TIMEOUT_MS` (5 s) with `civAddr != 0` and `lastCivRx > 0`:
- `civPortReleased = true` → `releaseAGPort()` → `setConflict(true, "TRX Timeout")`

On next CI-V frame: `civPortReleased = false`, `setConflict(false, "TRX wieder aktiv")`.

---

## setConflict() — Single Point of Control — CRITICAL

**All GPIO4, CI-V Inhibit, FlexRadio server/client, and display inversion go through `setConflict()`.**

```cpp
void setConflict(bool active, const char *reason)
 ├── early exit if conflictActive == active
 ├── conflictActive = active
 ├── digitalWrite(CONFLICT_PIN, ...)
 ├── webLog(...)
 ├── sendCIVTxInhibit(active)
 ├── flexSendInterlockStatus(active)   // server → AG clients
 ├── flexRadioSendInterlock(active)    // client → FLEX radio
 ├── displaySetInvert(active)          // #if DISPLAY_ENABLED
 ├── dispNeedsRedraw = true            // #if DISPLAY_ENABLED
 └── ssePushStatus()
```

`checkAndSignalConflict()` guards with `else if (conflictActive)` to avoid spurious `setConflict(false)` on first band change.

---

## FlexRadio Server (AG Interlock)

### Connect sequence
```
← V3.3.15\n + H00000001\n     (prologue)
→ C2|keepalive enable
← R2|00000000|\n
← S00000000|interlock state=PTT_REQUESTED/READY ...\n    ← immediate push
→ C3|ping  ← R3|00000000|\n  (periodic)
```

**Status push is proactive** — sent immediately after `keepalive enable`, not after `sub tx all` (which the AG never sends). `flexSendLineTo(idx, ...)` for per-client replies, `flexSendInterlockStatus()` broadcasts to all clients.

### Multi-client
Up to `FLEX_MAX_CLIENTS` (4) simultaneous connections. Each has own `FlexClient` struct with line buffer and `txActive` state. Log prefix: `[FLEX:0]`, `[FLEX:1]`, etc.

---

## FlexRadio Client (SmartSDR Direct Interlock)

### Why
The AG sets `inhibit=1` on Port B when it receives `PTT_REQUESTED`, but SmartSDR ignores the AG `inhibit` flag. SmartSDR only reacts to its own internal interlock received directly via TCP from a connected "radio". The WT32 must connect as a client to the real FLEX radio and register an ANT-type interlock there.

### Protocol
```
→ C1|client program CIV-Bridge
→ C2|interlock create type=ANT name=CIV-Bridge serial=DK4DJ-1
← R2|0|<interlock_id>      ← ID stored in flexRadioInterlockId
→ C3|interlock not_ready <id>   ← TX blocked in SmartSDR
→ C4|interlock ready <id>       ← TX allowed
```
`flexRadioSend()` is a private static helper (not in forward declarations). Log prefix: `[FRINT]`.

### Radio discovery and selection

VITA-49 discovery packets from SmartSDR are received on `flexDiscUdp` (same socket used for our own broadcasts). OUI check: `buf[8]==0x00 && buf[9]==0x1C && buf[10]==0x2D`, sender IP ≠ own IP.

All discovered radios stored in `flexRadioList[FLEX_MAX_RADIOS]`. **Auto-select priority:**

1. AG Port 2 `source=FLEX nickname=X` → match `agPort2FlexNickname` against list
2. `flexDefaultIP` from NVS (`Preferences`, namespace `"flex"`, key `"defaultIP"`)
3. `flexRadioCount == 1` → auto-select
4. Multiple → `flexSelectPending = true` → web card + `MENU_FLEX_SELECT`

`flexRadioSelectByIdx()` sets `flexRadioIPKnown = true`, resets connection, logs the selection. `flexRadioSelectByIP()` searches list by IP string.

**Persistent default:** saved via `flexPrefs.putString("defaultIP", ip)`. Loaded in `flexSetup()`. Set via `POST /flex-default?ip=` or encoder long press (≥ 2 s).

### `agPort2FlexNickname` — CRITICAL placement
This variable is declared **globally** (not inside `#if FLEX_EMULATION_ENABLED`) because `parsePortLine()` uses it unconditionally. It is populated in `parsePortLine()` only when `FLEX_EMULATION_ENABLED` is active. Do not move it inside the guard.

---

## Logging

All entries: `mm:ss.mmm [TAG] message`. Ring buffer 300 entries. Export via `GET /log`.

| Prefix | Source |
|---|---|
| `[ETH]` | Ethernet events |
| `[DISC]` | AG UDP discovery |
| `[AG]` | AG TCP commands/responses |
| `[CIV]` | CI-V frames |
| `[CONF]` | setConflict() calls |
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
| `/api` | GET | Full JSON snapshot |
| `/events` | GET | SSE stream |
| `/log` | GET | Log file download |
| `/flex-select` | POST | Select FLEX radio (`ip=x.x.x.x`) |
| `/flex-default` | POST | Set + persist default FLEX radio |

---

## GPIO Reference

| GPIO | Function |
|---|---|
| 0 | ETH CLK input (RMII) |
| 2 | Display A0 (strapping pin, safe after boot) |
| 4 | Conflict/TX-inhibit output (HIGH = active) |
| 5 | CI-V RX (UART2) |
| 12 | Display CS |
| 14 | Display SCK |
| 15 | Display MOSI (SI) |
| 16 | ETH Power/Reset |
| 17 | CI-V TX (UART2) |
| 18 | ETH MDIO |
| 19, 21, 22, 25, 26, 27 | ETH RMII (fixed) |
| 23 | ETH MDC |
| 33 | Display RST |
| 35 | Encoder button (input-only) |
| 36 | Encoder B (input-only, polled) |
| 39 | Encoder A (input-only, ISR) |

---

## Known Issues & Limitations

**PTT approximation:** `PTT_REQUESTED` is sent whenever CI-V frames are received — not only when PTT key is pressed.

**Single SSE client:** Only one browser tab receives live updates.

**AG P2P mode:** If Port shows `source=P2P`, manual `port set source=MANUAL` may be overridden immediately.

---

## Bug History

| Symptom | Root cause | Fix |
|---|---|---|
| ETH reset timeout | `ETH_POWER_PIN` wrong | `16` or `-1` |
| Web UI blocked during startup | `handleClient()` inside `agConfigDone` | Moved to top of `loop()` |
| AG ignores all commands | `print()` + `\r` only | `write()` + `flush()` + `\r\n` |
| All AG responses discarded | `agCfgSeq = agSeq` before send | `agCfgSeq = agSeq - 1` after |
| Config hangs | `sub antenna` + missing `all` | Fixed command |
| `delay(200)` blocked loop | Blocking delay in `agProcessLine()` | `agConfigStartAt` timer |
| Discovery UDP re-created | Local `WiFiUDP` in `flexSendDiscovery()` | Static `flexDiscUdp` |
| Spurious TX Inhibit OFF | `setConflict(false)` always called | `else if (conflictActive)` guard |
| SmartSDR not inhibited — AG `inhibit=1` ignored | SmartSDR uses own interlock, not AG flag | Direct TCP client interlock |
| SmartSDR never connected | SmartSDR doesn't auto-connect to "radios" | WT32 connects as client to FLEX radio |
| Only one FLEX client (AG) | Single-client server | Multi-client `FlexClient[FLEX_MAX_CLIENTS]` |
| Status pushed after `sub tx` that never arrived | AG sends `keepalive`, not `sub tx` | Push after `keepalive enable` |
| Log download button no effect | `client.stop()` race | Full `String` + single `webServer.send()` |
| `agPort2FlexNickname` not declared | Inside `FLEX_EMULATION_ENABLED` guard | Moved to global scope |
| `flexRadioListAge` write-only | Never read | Removed |
