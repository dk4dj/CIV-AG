# CLAUDE.md — Development Context

**Project:** WT32-ETH01 CI-V → Antenna Genius Bridge  
**File:** `WT32_CIV_AntennaGenius.ino` (~2500 lines, single-file Arduino sketch)  
**Hardware:** WT32-ETH01 (ESP32 + LAN8720 Ethernet)

Read this before modifying any protocol-related code. Every section marked **CRITICAL** has caused real bugs during development.

---

## Architecture

### Loop structure (strictly non-blocking)

```
loop()
 ├── webServer.handleClient()      ← always first, every iteration
 ├── handleSSEClient()
 ├── flexLoop()                    ← Discovery broadcast + multi-client TCP server
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

**Rule:** `webServer.handleClient()` and `flexLoop()` run unconditionally every loop iteration. Moving them inside `if(agConfigDone)` breaks the web UI and FlexRadio emulation during startup.

### State machines

**AG config state** (`agCfgState`):
```
AGCFG_IDLE → AGCFG_WAIT_SUB_PORT → AGCFG_WAIT_BAND_LIST → AGCFG_WAIT_ANTENNA_LIST → AGCFG_DONE
```
Config starts 200 ms after prologue received (`agConfigStartAt` non-blocking timer).

**Display menu state** (`menuState`):
```
MENU_STATUS ←── timeout/cancel ──┐
     │ rotate/press               │
     ▼                            │
MENU_ANT_SELECT ── press ──► MENU_CONFIRM ──► switch + return
```

---

## Antenna Genius Protocol — CRITICAL

### Command format
```
C<seq>|<command>\r\n
```
Use `agClient.write((const uint8_t*)buf, strlen(buf))` + `agClient.flush()`.  
**Never use `agClient.print()`** — may buffer and not send immediately.  
Terminator is `\r\n` (both CR+LF). The spec says `\r` only but firmware requires both.

### Response format
```
R<seq>|<hex_code>|<message>\n
```
- `hex_code == "0"` or `"00000000"` = success
- Multi-line block: one `R<seq>|0|<data>` line per entry
- **Block terminator: `R<seq>|0|` with empty message field**

### Sequence number — CRITICAL
`sendAGCommand()` increments `agSeq` **after** writing. Therefore:
```cpp
// CORRECT:
sendAGCommand("some command");
agCfgSeq = agSeq - 1;   // ← AFTER the call

// WRONG — all responses silently discarded:
agCfgSeq = agSeq;
sendAGCommand("some command");
```

### Config sequence
1. `sub port all` — `all` parameter mandatory; `sub antenna` does not exist
2. AG sends `S0|port 1 ...` and `S0|port 2 ...` asynchronously — port status known before any `port get`
3. `band list` — multi-line, ends with empty-message terminator
4. `antenna list` — same pattern

### TCP port
`connectToAG()` uses `agTcpPort` from the discovery packet with fallback to `AG_PORT` (9007).

---

## CI-V Protocol

### Frame format
```
FE FE <dst> <src> <cmd> [subcommand] [data] FD
```
Controller address: `0xE0`. Transceiver address: learned from first received frame.

### TX Inhibit command — Cmd 0x16/0x66
```
FE FE <civAddr> E0 16 66 <00/01> FD
  01 = TX Inhibit ON
  00 = TX Inhibit OFF
```
Only sent when `civAddr != 0x00`. Older transceivers respond with `FA FA` or ignore — no negative effect. **Never call `sendCIVTxInhibit()` directly — always go through `setConflict()`.**

### CI-V timeout flow
After `CIV_TIMEOUT_MS` (5 s) with `civAddr != 0` and `lastCivRx > 0`:
- `civPortReleased = true`
- `releaseAGPort()` → `port set N auto=1 source=AUTO band=0 rxant=0 txant=0`
- `setConflict(true, "TRX Timeout")`

On next CI-V frame: `civPortReleased = false`, `setConflict(false, "TRX wieder aktiv")`.

---

## setConflict() — Single Point of Control — CRITICAL

**Every change to GPIO4, CI-V TX Inhibit, and display inversion goes through `setConflict()`.**  
The only exception is the LOW init in `setup()`.

```cpp
void setConflict(bool active, const char *reason);
```

```
setConflict(active)
 ├── if (conflictActive == active) return   ← early exit, no spurious log entries
 ├── conflictActive = active
 ├── digitalWrite(CONFLICT_PIN, ...)
 ├── webLog(...)
 ├── sendCIVTxInhibit(active)
 ├── displaySetInvert(active)               ← #if DISPLAY_ENABLED only
 ├── dispNeedsRedraw = true                 ← #if DISPLAY_ENABLED only
 └── ssePushStatus()
```

`checkAndSignalConflict()` additionally guards with `else if (conflictActive)` to avoid calling `setConflict(false)` when there was no active conflict — prevents spurious TX Inhibit OFF and log entries on first band change.

**Trigger points:**

| Trigger | Call | Where |
|---|---|---|
| CI-V timeout | `setConflict(true, "TRX Timeout")` | `releaseAGPort()` |
| Antenna conflict | `setConflict(true, "Antenne von anderem Port belegt")` | `checkAndSignalConflict()` |
| TRX resumes | `setConflict(false, "TRX wieder aktiv")` | `processCIVFrame()` |
| Conflict resolved | `setConflict(false, "Konflikt aufgeloest")` | `checkAndSignalConflict()` |

---

## FlexRadio Emulation — Multi-Client

### Why multi-client
The AG connects to port 4992 to receive interlock status for its own conflict handling.
SmartSDR **also** connects to port 4992 directly — it uses the FlexRadio internal interlock, not the AG `inhibit` flag. Both must receive `PTT_REQUESTED` independently.

**The AG sets `inhibit=1` on its port but SmartSDR ignores that.** SmartSDR only reacts to `state=PTT_REQUESTED` received directly from the TCP server it believes is a FlexRadio.

### Client management
```cpp
#define FLEX_MAX_CLIENTS 4

struct FlexClient {
  WiFiClient  tcp;
  bool        connected  = false;
  char        lineBuf[128];
  uint32_t    lineBufLen = 0;
  bool        txActive   = false;  // last PTT state sent to this client
};
static FlexClient flexClients[FLEX_MAX_CLIENTS];
```

Each client has its own line buffer and `txActive` state. Log entries include client index: `[FLEX:0]`, `[FLEX:1]`, etc.

### Connect sequence (per client)
```
← V3.3.15\n              (version prologue)
← H00000001\n            (fixed handle)
→ C2|keepalive enable    (client sends this — AG and SmartSDR both do)
← R2|00000000|\n
← S00000000|interlock state=PTT_REQUESTED/READY source=SW reason= tx_allowed=1\n
→ C3|ping
← R3|00000000|\n
```

### Status push timing — CRITICAL
The real FlexRadio sends `S0|interlock state=...` **proactively** after every state change, without waiting for `sub tx all`. The AG (and SmartSDR) send `keepalive enable` right after connecting, and the status is sent immediately in the `keepalive` handler. **Do not wait for `sub tx` — it may never come.**

```cpp
// keepalive handler in flexProcessLine():
if (strncmp(cmd, "keepalive", 9) == 0) {
    flexSendLineTo(idx, reply);          // R<seq>|00000000|
    // push current status immediately to THIS client only:
    flexSendLineTo(idx, status_line);
    flexClients[idx].txActive = txNow;
}
```

`flexSendInterlockStatus()` broadcasts to **all** connected clients.  
`flexSendLineTo(idx, line)` sends to a **specific** client (used for per-client replies and initial status).

### PTT state mapping
| Condition | State |
|---|---|
| `civAddr != 0` AND `lastCivRx > 0` AND `!civPortReleased` | `PTT_REQUESTED` |
| Otherwise | `READY` |

`flexUpdatePttStatus()` runs every loop, compares against `flexTxActive`, sends only on change.

---

## Display

### Splash screen
1024-byte PROGMEM array `splashBitmap[]` — original artwork scaled to 100×64 (aspect ratio preserved), centered on 128×64 canvas, 1 bpp XBM LSB-first. Drawn via `u8g2.drawXBMP(0, 0, SPLASH_W, SPLASH_H, splashBitmap)` in `displaySetup()`.

### Error inversion
```cpp
void displaySetInvert(bool invert) {
  u8g2.sendF("c", invert ? 0xA7 : 0xA6);  // ST7565 hardware command
}
```
Called from `setConflict()` under `#if DISPLAY_ENABLED`. Instant — no redraw needed. `dispNeedsRedraw = true` is also set so the status screen refreshes with updated content after the inversion.

---

## Logging

All log entries have a `mm:ss.mmm` timestamp prepended (relative to device boot):
```
02:34.567 [FLEX:0] Client verbunden von 192.168.1.221
02:34.570 [FLEX:0] >> V3.3.15
02:34.572 [FLEX:0] << C2|keepalive enable
02:34.575 [FLEX:0] Interlock nach keepalive: READY
```

Ring buffer: `LOG_BUF_LINES = 300` entries. Exported via `GET /log` as `text/plain` with `Content-Disposition: attachment`. The entire log is built into a `String` and sent with a single `webServer.send()` call — do not use `webServer.client()` + `client.stop()` as this races with TCP flush on the ESP32.

---

## Web Endpoints

| Endpoint | Description |
|---|---|
| `GET /` | HTML dashboard with SSE auto-update |
| `GET /api` | Full JSON snapshot |
| `GET /events` | SSE stream (`status` and `log` events) |
| `GET /log` | Log file download (text/plain, attachment) |

JSON fields: `civAddr`, `civFreq`, `activeBand`, `agName`, `agSerial`, `agFw`, `agIP`, `agPort`, `agPorts`, `agAntennas`, `agMode`, `agUptime`, `agStatus`, `portA`, `portB`, `bandA`, `bandB`, `txA`, `txB`, `conflict`, `civReleased`, `flexConn`, `flexPtt`, `bands[]`, `antennas[]`

---

## GPIO Reference

### Occupied
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
| 35 | Encoder button (input-only, ext. pull-up required) |
| 36 | Encoder B (input-only, polled, ext. pull-up required) |
| 39 | Encoder A (input-only, ISR, ext. pull-up required) |

### Free
GPIO 1 (TX0), 3 (RX0), 32, 34

---

## Known Issues & Limitations

**Single SSE client:** Only one browser tab receives live updates.

**PTT approximation:** `PTT_REQUESTED` is sent whenever the TRX is powered on and sending CI-V frames — not only when the PTT key is pressed (PTT key state is not observable via standard CI-V frequency broadcasts). This is conservative: Port A is considered busy for the entire time the Icom TRX is active.

**SmartSDR interlock dependency:** SmartSDR must connect to our TCP server on port 4992 and receive `PTT_REQUESTED` directly. It does **not** react to the `inhibit=1` flag set by the AG on the port. The AG sets `inhibit=1` correctly but SmartSDR ignores it.

---

## Bug History

| Symptom | Root cause | Fix |
|---|---|---|
| ETH reset timeout | `ETH_POWER_PIN` wrong value | `16` or `-1` depending on board revision |
| Web UI blocked during startup | `handleClient()` inside `agConfigDone` guard | Moved to top of `loop()`, unconditional |
| AG ignores all commands | `print()` buffering + `\r` only terminator | `write()` + `flush()` + `\r\n` |
| All AG responses discarded | `agCfgSeq = agSeq` before `sendAGCommand()` | `agCfgSeq = agSeq - 1` after call |
| Config hangs at `sub port all` | `sub antenna` does not exist; `all` param missing | Removed; `sub port all` with `all` |
| Config hangs at `port get` | TCP buffer exhausted after large list responses | Removed `port get` from init |
| CI-V parser missed frames | `return` instead of `continue` in byte loop | Fixed |
| Timeout fires immediately | Timeout check before data read | Moved after `!gotData` check |
| `delay(200)` after prologue blocked loop | Blocking delay in `agProcessLine()` | Replaced with `agConfigStartAt` timer |
| Discovery UDP socket re-created each call | Local `WiFiUDP` in `flexSendDiscovery()` | Made `flexDiscUdp` static global |
| Spurious TX Inhibit OFF on first band change | `checkAndSignalConflict()` called `setConflict(false)` unconditionally | Added `else if (conflictActive)` guard |
| `connectToAG()` used hardcoded port | Discovery-supplied port ignored | Now uses `agTcpPort` with fallback |
| SmartSDR not inhibited despite AG `inhibit=1` | Single-client server — SmartSDR was rejected | Rebuilt TCP server for up to 4 clients |
| SmartSDR never received `PTT_REQUESTED` | Waited for `sub tx all` that never came | Status pushed immediately after `keepalive enable` |
| Log download button had no effect | `client.stop()` raced with TCP flush | Entire log built as `String`, sent with single `webServer.send()` |
| Compilation error: `extern` vs `static` linkage | `flexProcessLine` declared `extern`, implemented `static` | Removed `static` from implementation |
| Compilation error: `operator+` on string literals | Python replacement generated invalid lambda expression in JSON builder | Replaced with local block variable `_fc` |
