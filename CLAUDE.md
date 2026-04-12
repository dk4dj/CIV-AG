# CLAUDE.md — Development Context

**Project:** WT32-ETH01 CI-V → Antenna Genius Bridge  
**File:** `WT32_CIV_AntennaGenius.ino` (~2280 lines, single-file Arduino sketch)  
**Hardware:** WT32-ETH01 (ESP32 + LAN8720 Ethernet)

Read this before modifying any protocol-related code. Every section marked **CRITICAL** has caused real bugs during development.

---

## Architecture

### Loop structure (strictly non-blocking)

```
loop()
 ├── webServer.handleClient()      ← always first, every iteration
 ├── handleSSEClient()
 ├── flexLoop()                    ← Discovery broadcast + TCP server
 ├── [return if !ethConnected]
 ├── discoverAntennaGenius()       ← until AG found
 ├── connectToAG()                 ← until connected
 ├── handleAGReceive()             ← non-blocking line reader + config state machine
 ├── menuHandleEncoder()           ← optional, even during config phase
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
Config starts 200 ms after prologue received (`agConfigStartAt` timer, non-blocking).

**Display menu state** (`menuState`):
```
MENU_STATUS ←─── timeout/rotate/confirm ───┐
     │ rotate/press                          │
     ▼                                       │
MENU_ANT_SELECT ── press ──► MENU_CONFIRM ──┘
```

---

## Antenna Genius Protocol — CRITICAL

### Command format
```
C<seq>|<command>\r\n
```
Use `agClient.write((const uint8_t*)buf, strlen(buf))` + `agClient.flush()`.  
**Never use `agClient.print()`** — it buffers and may not send immediately.  
Terminator is `\r\n` (both CR+LF). The spec says `\r` only, but firmware requires both.

### Response format
```
R<seq>|<hex_code>|<message>\n
```
- `hex_code == "0"` or `"00000000"` = success
- Multi-line responses: one `R<seq>|0|<data>` line per entry
- **Block terminator: `R<seq>|0|` with empty message field** — nothing after the second `|`

### Sequence number — CRITICAL BUG
`sendAGCommand()` increments `agSeq` **after** writing. Therefore:
```cpp
// CORRECT:
sendAGCommand("some command");
agCfgSeq = agSeq - 1;   // ← AFTER the call

// WRONG — causes all responses to be silently discarded:
agCfgSeq = agSeq;
sendAGCommand("some command");
```
This mistake was responsible for weeks of debugging. The response arrives with the correct sequence number but `agCfgSeq` points to the next number, causing the seq-check to fail silently.

### Config sequence
1. `sub port all` — `all` parameter mandatory; `sub antenna` does not exist in the API
2. After success: AG sends `S0|port 1 ...` and `S0|port 2 ...` asynchronously — port status already known before any `port get`
3. `band list` — multi-line, ends with empty-message terminator
4. `antenna list` — same pattern
5. No `port get` needed at startup — TCP buffer exhaustion issues were caused by `port get` calls after large list responses

### Port switching
```
port set 1 auto=0 source=MANUAL band=5 rxant=1 txant=1
```
`tx` and `inhibit` are read-only — cannot be set via `port set`.

### AG connects to us (discovery reply port)
`connectToAG()` uses `agTcpPort` from the discovery packet, with fallback to `AG_PORT` (9007) if `agTcpPort == 0`.

---

## CI-V Protocol

### Frame format
```
FE FE <dst> <src> <cmd> [subcommand] [data] FD
```
Controller address: `0xE0`. Transceiver address: learned from first received frame (`civAddr`).

### Frequency encoding
5 BCD bytes, little-endian, 1 Hz resolution. See `decodeCIVFreq()`.

### Active polling
When no broadcast received for `CIV_BROADCAST_TIMEOUT_MS` (3 s), sends `FE FE <addr> E0 03 FD` every `CIV_POLL_MS` (500 ms).

### TX Inhibit command (Cmd 0x16, Subcmd 0x66)
```
FE FE <civAddr> E0 16 66 <00/01> FD
  00 = TX Inhibit OFF
  01 = TX Inhibit ON
```
Only sent when `civAddr != 0x00` (address must be learned first). Older transceivers respond with `FA FA` (FAIL) or ignore silently — no negative effect.

### CI-V timeout
After `CIV_TIMEOUT_MS` (5 s) with no frames, and `civAddr != 0` and `lastCivRx > 0`:
- `civPortReleased = true` (prevents repeated triggering)
- `releaseAGPort()` — sends `port set N auto=1 source=AUTO band=0 rxant=0 txant=0`
- `setConflict(true, "TRX Timeout")`

On next CI-V frame: `civPortReleased = false`, `setConflict(false, "TRX wieder aktiv")`.

---

## setConflict() — Single Point of Control

**All GPIO4, CI-V Inhibit, and SSE updates go through `setConflict()`.**  
Never call `digitalWrite(CONFLICT_PIN, ...)` directly (except the LOW init in `setup()`).

```cpp
void setConflict(bool active, const char *reason);
```

Has internal early-exit when state doesn't change. `checkAndSignalConflict()` additionally guards against calling `setConflict(false)` when there was no active conflict (avoids spurious log entries on first band change).

**Trigger points:**
| Trigger | Call |
|---|---|
| CI-V timeout | `setConflict(true, "TRX Timeout")` in `releaseAGPort()` |
| Antenna conflict | `setConflict(true, "Antenne von anderem Port belegt")` in `checkAndSignalConflict()` |
| TRX resumes | `setConflict(false, "TRX wieder aktiv")` in `processCIVFrame()` |
| Conflict resolved | `setConflict(false, "Konflikt aufgeloest")` in `checkAndSignalConflict()` |

---

## FlexRadio Emulation Protocol

### Why
The AG uses the FlexRadio PTT mechanism for conflict detection between radio ports. By emulating a minimal FlexRadio, the WT32 can signal Port A activity to the AG, which then blocks SmartSDR on Port B from switching to the same antenna.

### Discovery broadcast (UDP → 255.255.255.255:4992)
Sent every `FLEX_DISCOVERY_MS` (2 s) via static `flexDiscUdp` object (never re-created per call).

VITA-49 packet structure (big-endian 32-bit words):
```
Word 0: Header — pkt_type=3 (ExtDataWithStream) | c=1 | packet_count(4b) | packet_size(16b)
Word 1: Stream ID = 0x00000000
Word 2: OUI = 0x001C2D00  (FlexRadio OUI = 0x1C2D, upper byte = 0)
Word 3: InformationClassCode=0x0000 | PacketClassCode=0xFFFF
Word 4+: ASCII payload, space-padded to 32-bit boundary
```

Required payload fields: `model=` `ip=` `port=` `serial=` `status=Available` `version=` `nickname=`

### TCP server exchange (port 4992)
```
AG connects
← V3.3.15\n                              (version prologue)
← H00000001\n                            (client handle, hex)
→ C1|sub tx all\n                        (AG subscribes)
← S00000000|interlock state=READY source= reason= tx_allowed=1\n
← R1|00000000|\n                         (OK reply to sub)

[on PTT change:]
← S00000000|interlock state=PTT_REQUESTED source=SW reason= tx_allowed=1\n
← S00000000|interlock state=READY source= reason= tx_allowed=1\n
```

All other commands (e.g. `client program`, `client gui`) answered with `R<seq>|00000000|\n`.

### PTT state mapping
| Condition | State sent |
|---|---|
| `civAddr != 0` AND `lastCivRx > 0` AND `!civPortReleased` | `PTT_REQUESTED` |
| Otherwise | `READY` |

`flexUpdatePttStatus()` is called every loop and only sends an update when state changes.

---

## GPIO Reference (WT32-ETH01)

### Occupied
| GPIO | Function |
|---|---|
| 0 | ETH CLK input (RMII, from LAN8720) |
| 5 | CI-V RX (UART2) |
| 16 | ETH Power/Reset (LAN8720) — `-1` on some boards |
| 17 | CI-V TX (UART2) |
| 18 | ETH MDIO |
| 19, 21, 22, 25, 26, 27 | ETH RMII (fixed, cannot be reassigned) |
| 23 | ETH MDC |
| 4 | Conflict/TX-inhibit output (HIGH = active) |
| 2 | Display A0 (strapping pin, safe after boot) |
| 12 | Display CS |
| 14 | Display SCK |
| 15 | Display MOSI (SI) |
| 33 | Display RST |
| 35 | Encoder button (input-only, ext. pull-up required) |
| 36 | Encoder B (input-only, polled, ext. pull-up required) |
| 39 | Encoder A (input-only, ISR, ext. pull-up required) |

### Free
GPIO 1 (TX0), 3 (RX0), 32, 34

### Input-only (GPIO 34, 35, 36, 39)
No internal pull-up or pull-down. External resistors mandatory for digital use.  
GPIO 36/39 share 270 pF internal capacitance (CAPP/CAPN pads) → Encoder B on GPIO36 is read by polling, not ISR, to avoid crosstalk.

---

## Known Issues & Limitations

**Single SSE client:** Only one browser tab receives live updates. A second `/events` connection disconnects the first.

**PTT resolution:** `PTT_REQUESTED` is sent whenever the TRX is powered on and CI-V frames are received — not only when actually transmitting (PTT is not observable via CI-V alone). This is a conservative approximation; the AG considers Port A busy for the entire time the Icom TRX is active.

**AG P2P mode:** If the AG port shows `source=P2P`, `port set source=MANUAL` commands may be overridden by the P2P source. The port switch appears to work but the AG reverts immediately.

**No TX mask changes:** An earlier approach manipulated antenna TX masks to block SmartSDR — this was deliberately removed. The correct long-term solution if per-antenna conflict detection is needed is the FlexRadio emulation + AG-native conflict handling now implemented.

---

## Bug History

| Symptom | Root cause | Fix |
|---|---|---|
| ETH reset timeout | `ETH_POWER_PIN` wrong value | `16` or `-1` depending on board revision |
| Web UI blocked during startup | `handleClient()` inside `agConfigDone` guard | Moved to top of `loop()`, unconditional |
| AG ignores all commands | `print()` buffering + `\r` only terminator | `write()` + `flush()` + `\r\n` |
| All AG responses discarded | `agCfgSeq = agSeq` before `sendAGCommand()` | `agCfgSeq = agSeq - 1` after call |
| Config hangs after `sub port all` | `sub antenna` does not exist; `all` param missing | Removed `sub antenna`; `sub port all` with `all` param |
| Config hangs at `port get` | TCP buffer exhausted after large list responses | Removed `port get` from init; port status from `S0|port` events |
| CI-V parser missed frames | `return` instead of `continue` in byte loop | Fixed with `continue` |
| Timeout fires immediately | Timeout check before data read | Moved after `!gotData` check |
| `delay(200)` after prologue blocked loop | Blocking delay in `agProcessLine()` | Replaced with `agConfigStartAt` non-blocking timer |
| Discovery UDP socket re-created each call | Local `WiFiUDP` object in `flexSendDiscovery()` | Made `flexDiscUdp` a static global |
| Spurious TX Inhibit sent on first band change | `checkAndSignalConflict()` called `setConflict(false)` unconditionally | Added `else if (conflictActive)` guard |
| `connectToAG()` used hardcoded port 9007 | Discovery-supplied port ignored | Now uses `agTcpPort` with fallback to `AG_PORT` |
