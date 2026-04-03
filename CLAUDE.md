# CLAUDE.md — Development Context for WT32-ETH01 CI-V → Antenna Genius Bridge

This file documents the project architecture, design decisions, known pitfalls, and protocol details accumulated during development. It is intended to give an AI assistant (or a returning developer) full context without needing to re-read the entire codebase.

---

## Project Overview

Single-file Arduino sketch (`WT32_CIV_AntennaGenius.ino`) for the WT32-ETH01 (ESP32 + LAN8720 Ethernet). It reads Icom CI-V frequency data from a serial bus and automatically switches the 4O3A Antenna Genius antenna selector to the correct antenna for the current band.

---

## Architecture

### State Machine Overview

```
boot
 └─ ETH DHCP
     └─ discoverAntennaGenius()   ← UDP broadcast listener
         └─ connectToAG()         ← TCP connect + prologue
             └─ agStartConfig()   ← sub port all → band list → antenna list
                 └─ AGCFG_DONE
                     ├─ handleAGReceive()   ← non-blocking line reader
                     ├─ handleCIV()         ← UART2 frame decoder + active poll
                     ├─ sendKeepalive()     ← ping every 5 s
                     ├─ pollPortStatus()    ← port get 1/2 every 3 s
                     └─ displayLoop()       ← optional DOGL128 + encoder
```

### Key Design Principles

- **Fully non-blocking**: no `delay()` or blocking loops after boot. Everything runs in `loop()` using `millis()`-based timers.
- **Web server always responsive**: `webServer.handleClient()` is called at the very top of `loop()`, before any state checks, so the web UI works even during AG discovery or reconnect phases.
- **Single TCP connection to AG**: the AG only tolerates a limited number of concurrent connections. Reconnect interval is 15 s to avoid exhausting the AG's connection pool.

---

## Antenna Genius Protocol — Critical Notes

These are hard-won lessons from debugging. Read carefully before modifying AG communication code.

### Command Format

```
C<seq>|<command>\r\n
```

- Terminator is `\r\n` (both CR and LF). The spec says only `\r` but the actual firmware requires `\r\n`.
- Use `agClient.write((const uint8_t*)buf, strlen(buf))` + `agClient.flush()` — NOT `agClient.print()`. `print()` may buffer and delay sending.

### Response Format

```
R<seq>|<hex_code>|<message>
```

- `hex_code` = `0` means success. Any other value is an error.
- Multi-line responses (e.g. `band list`) send one `R<seq>|0|<data>` line per entry.
- **The block ends with `R<seq>|0|` — an empty message field (nothing after the second `|`).**
- There is NO blank line between entries and NO blank line at the end. The empty-message line IS the terminator.

### Sequence Number Bug — CRITICAL

`sendAGCommand()` increments `agSeq` **after** sending. Therefore, after calling `sendAGCommand("foo")`, the command was sent with sequence number `agSeq - 1`. Always set:

```cpp
sendAGCommand("some command");
agCfgSeq = agSeq - 1;   // NOT agSeq!
```

Setting `agCfgSeq = agSeq` before `sendAGCommand()` is wrong — the stored sequence will not match the AG's response, causing all responses to be silently discarded.

### Subscriptions

- `sub port all` — subscribes to port status changes for all radio ports. The parameter `all` is **mandatory**. `sub port` without a parameter returns an error.
- `sub antenna` — **does not exist**. The available subscription objects are: `port`, `antenna` (only as a status reload notification), `group`, `output`, `relay`.
- After a successful `sub port all`, the AG immediately sends `S0|port 1 ...` and `S0|port 2 ...` with the current port state. These arrive before the `R1|0|` acknowledgement.

### Status Messages

```
S0|port 1 auto=1 source=P2P band=3 rxant=1 txant=1 inband=0 tx=0 inhibit=0
S0|antenna reload
```

- `S0|antenna reload` means antenna names/masks changed → re-run `antenna list`.
- `tx=1` is set by the AG itself when the port is transmitting. Do NOT set it via `port set`.

### Config Sequence

The correct startup sequence after TCP connect + prologue is:

1. `sub port all` → wait for `R1|0|` (empty message)
2. `band list` → wait for multiple `R2|0|band N ...` lines, terminated by `R2|0|` (empty)
3. `antenna list` → same pattern
4. Port status is already known from the `S0|port` messages sent after step 1 — no need for `port get` during init. `port get` calls during config cause timeout issues due to TCP buffer exhaustion after large `band list`/`antenna list` responses.

### Port Switching

```
C7|port set 1 auto=0 source=MANUAL band=5 rxant=1 txant=1\r\n
```

- `source=MANUAL` overrides automatic band detection.
- If the AG is in `P2P` mode (`source=P2P` in status), `port set` commands may be overridden by the P2P source.

---

## CI-V Protocol Notes

### Frame Format

```
FE FE <dst> <src> <cmd> [data...] FD
```

- `FE FE` = preamble (two bytes)
- `FD` = end-of-message
- Frequency broadcast (auto): `cmd = 0x00`, addressed to `0x00` (broadcast) or specific controller
- Frequency poll response: `cmd = 0x03`
- Our controller address: `0xE0`

### Frequency Encoding

5 BCD bytes, little-endian, 1 Hz resolution:

```cpp
uint64_t decodeCIVFreq(const uint8_t *data) {
  uint64_t freq = 0, mult = 1;
  for (int i = 0; i < 5; i++) {
    freq += (data[i] & 0x0F) * mult;        mult *= 10;
    freq += ((data[i] >> 4) & 0x0F) * mult; mult *= 10;
  }
  return freq; // Hz
}
```

### Active Polling

If no CI-V broadcast received for `CIV_BROADCAST_TIMEOUT_MS` (3000 ms), the sketch sends an active frequency request every `CIV_POLL_MS` (500 ms):

```
FE FE <trx_addr> E0 03 FD
```

The transceiver address is learned automatically from the first received frame.

### Parser Bug History

The original `handleCIV()` used `return` instead of `continue` when a non-preamble byte was received at position 0. This caused the parser to exit the `while(Serial2.available())` loop on any inter-frame garbage byte, missing subsequent frames. Fixed with `continue`.

---

## Web Interface Architecture

### SSE (Server-Sent Events)

- Single client at a time (`sseClient`). New connection replaces old one.
- Two event types: `log` (single log line) and `status` (JSON blob with all live fields).
- On new SSE connection, the full log ring buffer is replayed.
- Keepalive comment (`: ping\n\n`) sent every 15 s to keep the connection alive.

### JSON Fields (`/api` and `status` SSE event)

```json
{
  "civAddr": "0x94",
  "civFreq": "14.225000 MHz",
  "activeBand": "20m",
  "agName": "Antenna_Genius",
  "agSerial": "C7-F6-42",
  "agFw": "4.1.8",
  "agIP": "192.168.1.221",
  "agPort": 9007,
  "agPorts": 2,
  "agAntennas": 8,
  "agMode": "master",
  "agUptime": 3034,
  "agStatus": "OK",
  "portA": "Beam_OB11-3",
  "portB": "---",
  "bandA": "20m",
  "bandB": "---",
  "txA": false,
  "txB": false,
  "conflict": false,
  "bands": [...],
  "antennas": [...]
}
```

### Non-Blocking Web Server

`webServer.handleClient()` must be called in **every** loop iteration, including during AG config, discovery, and reconnect phases. Placing it inside `if (agConfigDone)` blocks the web UI for the entire startup phase.

---

## Display & Encoder Architecture

### Conditional Compilation

```cpp
#define DISPLAY_ENABLED 1   // set to 0 to disable entirely
```

All display/encoder code is wrapped in `#if DISPLAY_ENABLED` / `#endif`. When disabled: no `#include <U8g2lib.h>`, no pin reservation, no RAM usage.

### U8g2 Configuration

```cpp
U8G2_ST7565_EA_DOGM128_1_4W_SW_SPI u8g2(
  U8G2_R2,           // 180° rotation — change to R0/R1/R3 if needed
  DISP_SCK_PIN,      // 14
  DISP_MOSI_PIN,     // 15
  DISP_CS_PIN,       // 12
  DISP_A0_PIN,       // 2
  DISP_RST_PIN       // 33
);
```

- `_1_` suffix = single-page rendering (128 bytes RAM buffer, not 1 KB full framebuffer)
- Software SPI used because hardware SPI pins (GPIO19=MISO, GPIO23=MOSI) are occupied by Ethernet

### Encoder ISR Safety

The Gray-code ISR runs on GPIO39 (Encoder A). Encoder B (GPIO36) is read by polling in `menuHandleEncoder()` to avoid the known crosstalk issue between GPIO36 and GPIO39 (they share 270 pF internal capacitance via CAPP/CAPN pads).

The ISR accumulates delta values in `volatile int8_t encDelta`. The loop reads and resets this with `noInterrupts()`/`interrupts()`. Steps are divided by 2 (change `/2` to `/4` for encoders with 4 pulses per detent).

### Menu State Machine

```
MENU_STATUS ──rotate/press──► MENU_ANT_SELECT ──press──► MENU_CONFIRM
     ▲                              │  ▲                       │
     │                       rotate │  │ rotate/timeout        │ press
     └──────────── timeout ─────────┘  └───────────────────────┘
```

`menuBuildAntList()` filters antennas by the current band's TX mask. If no antenna has the band bit set, all antennas are shown as fallback.

---

## GPIO Reference (WT32-ETH01)

### Occupied Pins

| GPIO | Function | Modifiable? |
|---|---|---|
| 0 | ETH CLK input (RMII) | No |
| 16 | ETH Power/Reset (LAN8720) | Set to -1 on some boards |
| 18 | ETH MDIO | No |
| 19 | ETH RMII TXD0 | No |
| 21 | ETH RMII TX_EN | No |
| 22 | ETH RMII TXD1 | No |
| 23 | ETH MDC | No |
| 25 | ETH RMII RXD0 | No |
| 26 | ETH RMII RXD1 | No |
| 27 | ETH RMII CRS | No |
| 5 | CI-V RX (UART2) | Yes — configurable |
| 17 | CI-V TX (UART2) | Yes — configurable |
| 4 | Conflict output | Yes — configurable |
| 2 | Display A0 | Yes — strapping pin, safe after boot |
| 12 | Display CS | Yes |
| 14 | Display SCK | Yes |
| 15 | Display MOSI | Yes |
| 33 | Display RST | Yes |
| 35 | Encoder button | Yes — input-only, no internal pull-up |
| 36 | Encoder B | Yes — input-only, no internal pull-up |
| 39 | Encoder A | Yes — input-only, no internal pull-up |

### Free Pins

GPIO 1 (TX0), 3 (RX0), 32 are available. GPIO 34 is also free (input-only). GPIO 1/3 are UART0 (used for serial monitor/flashing).

---

## Known Issues & Limitations

### AG Source Mode Conflict

If the AG radio port is in `source=P2P` mode (receiving band data from another source, e.g. directly from a radio via the DB15 connector), `port set source=MANUAL` commands may be overridden by the P2P source. In this case the port switching will appear to work (command is sent, AG acknowledges) but the AG immediately reverts. Solution: ensure the AG port is configured for LAN/manual control in the AG web interface.

### Single SSE Client

The current implementation supports only one simultaneous SSE client. A second browser tab connecting to `/events` will disconnect the first. For multi-client support, a proper async web server (e.g. ESPAsyncWebServer) would be needed, but this increases memory pressure significantly.

### TCP Buffer After Large Responses

`band list` and `antenna list` responses can be several hundred bytes. After receiving these, the TCP receive buffer may be partially full, causing subsequent `port get` responses to be delayed or dropped. This is why `port get` calls were removed from the init sequence — port status is obtained from the asynchronous `S0|port` messages sent after `sub port all`.

### ADC / GPIO36+39 Interrupt Warning

Espressif documents that interrupts on GPIO36 and GPIO39 should not be used when ADC or Wi-Fi sleep mode is active. This project does not use ADC or Wi-Fi sleep, so the ISR on GPIO39 (Encoder A) is safe.

### Display Contrast

The `DISP_CONTRAST` value (default 22) is board-specific. The DOGL128W-6 datasheet specifies the internal LCD supply voltage via the contrast register. Values between 18 and 28 are typical for 3.3 V operation. Too high causes solid black; too low causes invisible text.

---

## Development History Summary

| Issue | Root Cause | Fix |
|---|---|---|
| ETH reset timeout at boot | `ETH_POWER_PIN = -1` instead of `16` | Set `ETH_POWER_PIN 16` |
| Web UI unreachable during startup | `handleClient()` behind `agConfigDone` guard | Moved to top of `loop()` |
| AG commands ignored silently | `agClient.print()` buffering + `\r` only terminator | Switched to `write()` + `flush()` + `\r\n` |
| All AG responses discarded | `agCfgSeq = agSeq` set before `sendAGCommand()` | Changed to `agCfgSeq = agSeq - 1` after call |
| Config hangs after `sub port all` | `sub antenna` does not exist in API | Removed; `sub port all` requires `all` parameter |
| Config hangs at `port get` | TCP buffer exhausted after large list responses | Removed `port get` from init; port status from `S0|port` events |
| CI-V parser missed frames | `return` instead of `continue` in byte loop | Fixed to `continue` |
| Timeout fires immediately | Timeout check ran before reading available data | Moved timeout check after data read, only when `!gotData` |
