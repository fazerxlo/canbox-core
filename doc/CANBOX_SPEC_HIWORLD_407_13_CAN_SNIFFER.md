# Hiworld (0x5AA5) CAN Delta Sniffer & Scenario Replay Specification
## Master Catalog: Peugeot 407 On-Demand Reconstructable CAN Delta Sniffer, UART Pacing, HU Action Isolation & Tooling

**Document File:** `CANBOX_SPEC_HIWORLD_407_13_CAN_SNIFFER.md`  
**Document Version:** 1.0.0  
**Target Platform:** Automotive Android Infotainment SoC $\longleftrightarrow$ Pure C99 CAN Box Firmware (`canbox-core`)  
**Vehicle Network:** Peugeot 407 & PSA Platform (CAN 2004 / PSA 15 Comfort & Body CAN @ 125 kbps)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` Framing, Additive Checksum)  
**Command Identifiers:** `0xD0` (`HIWORLD_CMD_SNIFFER_CONTROL`), `0xD1` (`HIWORLD_CMD_SNIFFER_DATA`)  

---

# Table of Contents
1. [Overview & Architectural Context](#1-overview--architectural-context)
2. [Hiworld 5AA5 Serial Protocol Specification](#2-hiworld-5aa5-serial-protocol-specification)
   - 2.1 Downlink Control Frame: `0xD0` (`HIWORLD_CMD_SNIFFER_CONTROL`)
   - 2.2 Uplink Data Batch Frame: `0xD1` (`HIWORLD_CMD_SNIFFER_DATA`)
   - 2.3 Subtype Identifiers (`0x01`, `0x02`, `0xFF`)
   - 2.4 Variable-Length Record Binary Packing (`7 + DLC` Bytes)
3. [Safety & Isolation Guards During Sniffer Mode](#3-safety--isolation-guards-during-sniffer-mode)
   - 3.1 Head Unit Action Command Suppression
   - 3.2 Hardware Illumination & Lighting GPIO Lock
   - 3.3 Switched ACC Power Supply Lock (Always ON)
   - 3.4 Inactivity Sleep Watchdog Bypass
4. [NUC131 Hardware Constraints & UART Pacing](#4-nuc131-hardware-constraints--uart-pacing)
5. [Offline Reconstruction & Reverse-Engineering Tooling](#5-offline-reconstruction--reverse-engineering-tooling)
   - 5.1 Chronological Bit-Diff Report (`<file>_diff.txt`)
   - 5.2 Deterministic Full-Rate Log Reconstruction (`<file>_reconstructed.log`)
6. [Desktop Test Harness & Verification Vectors](#6-desktop-test-harness--verification-vectors)

---

## 1. Overview & Architectural Context

When diagnosing PSA CAN bus communication or playing back raw `candump` / `dump_*.log` scenarios on test benches or in-vehicle, streaming raw 125 kbps CAN bus traffic directly over a 38,400 bps UART bridge would immediately overflow the MCU's internal 128-byte UART transmit buffer and drop frames.

The **30-second Reconstructable CAN Delta Sniffer Mode** provides an on-demand protocol running inside `canbox-core` that:
1. Tracks up to **128 unique CAN IDs** in a compact static state table without dynamic heap allocation.
2. Emits compact binary records **only on initial discovery, payload transitions, DLC changes, or injected `CAN_TX` frames**.
3. Accumulates repeat counters and estimates frame periods for periodic unchanged frames without transmitting them during steady state.
4. Flushes an End-of-Capture summary batch upon session expiration so the entire 30-second full-rate CAN stream can be deterministically reconstructed offline.
5. Completely suppresses Head Unit action commands and holds ACC ON and lighting GPIO stable so that log playback does not trigger nuisance UI popups (reverse camera, climate overlay, volume toggles) or cut power.

---

## 2. Hiworld 5AA5 Serial Protocol Specification

### 2.1 Downlink Control Frame: `0xD0` (`HIWORLD_CMD_SNIFFER_CONTROL`)

Sent by the Android Head Unit to activate or terminate sniffer mode.

```
[0x5A] [0xA5] [LEN=0x02] [0xD0] [MODE] [DURATION_S] [CS]
```

- **Sync Header:** `0x5A 0xA5`
- **Length:** `0x02` (Payload length in bytes)
- **Command ID:** `0xD0`
- **Payload Byte 0 (`MODE`):**
  - `0x00`: Stop sniffer immediately and flush final summary batch.
  - `0x01`: Start Delta + Period sniffer session.
- **Payload Byte 1 (`DURATION_S`):**
  - Capture duration in seconds (`1..30`, hard maximum 30 seconds). Any value `0` or `> 30` is clamped to 30. Sniffer automatically stops transmitting CAN records after at most 30 seconds regardless of whether the Head Unit sends a stop command.
- **Checksum (`CS`):**
  $\text{CS} = ((\text{LEN} + \text{CMD} + \sum \text{Payload}) - 1) \pmod{256}$

#### Downlink Acknowledgment
Per standard Hiworld protocol rules, the MCU automatically acknowledges `0xD0`:
```
5A A5 01 FF D0 CF
```

---

### 2.2 Uplink Data Batch Frame: `0xD1` (`HIWORLD_CMD_SNIFFER_DATA`)

Streamed periodically from `canbox-core` to the Android Head Unit.

```
[0x5A] [0xA5] [LEN] [0xD1] [SUBTYPE] [RECORDS...] [CS]
```

- **Pacing Rule:** Flushed at most **ONE `0xD1` frame per 10 ms tick**.
- **Maximum Payload:** $\le 52$ bytes ($1$ byte subtype $+$ up to $51$ bytes of packed records).
- **Maximum Wire Size:** $\le 57$ bytes ($5$ bytes header/checksum $+$ up to $52$ bytes payload).

---

### 2.3 Subtype Identifiers

| Subtype | Name | Purpose |
| :---: | :--- | :--- |
| `0x01` | **Live Event Batch** | Initial discovery of a CAN ID, payload byte/bit transition, or injected `CAN_TX` frame. |
| `0x02` | **End-of-Capture Summary** | Flushed at end of session dripping last seen payloads, final repeat counts, and periods. |
| `0xFF` | **Sniffer Finished Marker** | Signals completion of summary flush: `5A A5 04 D1 FF <TOTAL_IDS> <DROPPED_EVENTS_LE16> <CS>` |

---

### 2.4 Variable-Length Record Binary Packing (`7 + DLC` Bytes)

Inside `SUBTYPE 0x01` and `SUBTYPE 0x02`, records are serialized sequentially using variable-length DLC packing:

```
+---------------+-----------------+--------------------+---------------+-------------------+
| dt_ms (2B LE) | id_flags_dlc    | prev_repeat_count  | period_10ms   | data[0..dlc-1]    |
| (0..60000 ms) | (2B LE)         | (2B LE)            | (1B)          | (0..8 bytes)      |
+---------------+-----------------+--------------------+---------------+-------------------+
```

1. **`dt_ms` (`uint16_t` LE, 2 bytes):**
   Milliseconds elapsed since `can_sniffer_start()` ($0 \dots 60000$).
2. **`id_flags_dlc` (`uint16_t` LE, 2 bytes):**
   - Bits `10..0`: 11-bit standard CAN ID (`0x000..0x7FF`).
   - Bits `14..11`: CAN `dlc` ($0 \dots 8$).
   - Bit `15` (`0x8000`): Direction flag:
     - `0`: Received from vehicle bus (`CAN_RX`).
     - `1`: Injected by `canbox-core` firmware (`CAN_TX`).
3. **`prev_repeat_count` (`uint16_t` LE, 2 bytes):**
   How many times the *previous* identical payload of this CAN ID repeated before this emission ($0$ on first discovery or `CAN_TX`).
4. **`period_10ms` (`uint8_t`, 1 byte):**
   Measured average reception interval in 10 ms units ($0$ = aperiodic / first frame, $5$ = 50 ms, $10$ = 100 ms, $50$ = 500 ms, $100$ = 1000 ms, $255$ = $\ge 2.55\text{ s}$).
5. **`data` ($0 \dots 8$ bytes):**
   Payload bytes for the frame.

---

## 3. Safety & Isolation Guards During Sniffer Mode

When replaying vehicle logs on physical test benches or during active capture:

### 3.1 Head Unit Action Command Suppression
During active sniffing (`can_sniffer_is_active() == true`):
- **Steering Wheel & Stalk Keys (`0x11`, `0x21`):** Completely suppressed.
- **Door / Boot / Bonnet Status (`0x12`):** Completely suppressed (no overlay popups).
- **Climate / Dual Zone HVAC (`0x31`):** Completely suppressed (no climate bar popups).
- **Parking Radar AAS (`0x41`):** Completely suppressed (no distance beeps or sensor popups).
- **Reverse Gear (`0x11` bit / `GPIO_PIN_REVERSE_OUT`):** Completely suppressed (no camera switches).
- **Telemetry & Powertrain (`0x1A`):** Suppressed.
- **Alert Popups & Summary (`0x42`, `0xEA`):** Suppressed.
- **Radio & CD Changer Media (`0x84`, `0x85`, `0x86`, `0x97`):** Suppressed.

### 3.2 Hardware Illumination & Lighting GPIO Lock
The physical `GPIO_PIN_ILL_OUT` trigger wire to the Head Unit is locked in its state and never toggled during sniffing, preventing screen backlights from flickering when log data containing lighting transitions is fed into the CAN controller.

### 3.3 Switched ACC Power Supply Lock (Always ON)
The switched accessory power supply pin (`GPIO_PIN_HEADUNIT_POWER`) is locked to **ON** (`true`) as soon as sniffer mode starts and throughout the session. If log scenarios contain ignition OFF frames, the Head Unit does not shut down.

### 3.4 Inactivity Sleep Watchdog Bypass
The 3.0-second CAN silence inactivity sleep watchdog inside `can_router_periodic_100ms()` is bypassed while sniffing. The MCU remains awake and link keep-alive heartbeats (`0xFF`) continue to be emitted.

---

## 4. NUC131 Hardware Constraints & UART Pacing

The Nuvoton NUC131 microcontroller operates with a 128-byte hardware FIFO for UART TX at 38,400 baud (bandwidth $\approx 3.84\text{ bytes/ms}$).

- A continuous burst of CAN frames could saturate the buffer in $<30\text{ ms}$.
- By bounding each `0xD1` frame payload to $\le 52$ bytes (wire size $\le 57$ bytes) and flushing at most **once every 10 ms**, UART link utilization remains $\le 55\%$, completely preventing buffer overflow and blocking.

---

## 5. Offline Reconstruction & Reverse-Engineering Tooling

The Python 3 tool `tools/reconstruct_sniffer_dump.py` accepts `.csniff` files or raw UART stream captures and produces two output files:

```bash
./tools/reconstruct_sniffer_dump.py capture.csniff
```

### 5.1 Chronological Bit-Diff Report (`<file>_diff.txt`)
Displays an executive table of all seen CAN IDs, their measured cycle times, and a timeline highlighting exact changed bytes and flipped bits:

```text
================================================================================
 OpenCanbox Reconstructed Sniffer Report & Delta Analysis
================================================================================
Total Unique CAN IDs : 34 (reported by MCU: 34)
Total Records Extracted : 128
Dropped Events (Overflow) : 0

CAN IDs Summary Table:
--------------------------------------------------------------------------------
 CAN ID | Type | DLC | Cycle Period    | Transitions | Total Frames | Activity
--------------------------------------------------------------------------------
 0x1E3  | RX   |  7  |  100 ms (10.0 Hz) |           1 |          300 | [+0.1s .. +30.0s]
 0x15B  | TX   |  8  |  500 ms ( 2.0 Hz) |           0 |           60 | [+0.5s .. +30.0s]
--------------------------------------------------------------------------------

Chronological Transitions & Bit Flipped Timeline:
================================================================================
[+ 0.100s] CAN 0x1E3 (DLC 7) [DISCOVERED]: 1D 30 0B 0B 00 00 02
[+ 0.500s] CAN_TX 0x15B (DLC 8): 01 03 B1 40 00 D0 00 10
[+ 0.700s] CAN 0x1E3 (DLC 7) [CHANGED after 5 repeats / ~100ms period]:
         Old: 1D 30 0B 0B 00 00 02
         New: 1D 30 0C 0B 00 00 02
         Diff: Byte[2]: 0x0B -> 0x0C [bit0: 1->0 bit1: 1->0 bit2: 0->1]
```

### 5.2 Deterministic Full-Rate Log Reconstruction (`<file>_reconstructed.log`)
Expands the repeat counters at the measured period intervals, generating a 100% full-rate standard `dump_*.log` that can be directly fed into `canbox-core` replay harnesses and simulator tests:

```text
     0.100 [CAN] ID:1E3 DLC:7 DATA:1D300B0B000002
     0.200 [CAN] ID:1E3 DLC:7 DATA:1D300B0B000002
     0.300 [CAN] ID:1E3 DLC:7 DATA:1D300B0B000002
     0.400 [CAN] ID:1E3 DLC:7 DATA:1D300B0B000002
     0.500 [CAN] ID:1E3 DLC:7 DATA:1D300B0B000002
     0.500 [CAN_TX] ID:15B DLC:8 DATA:0103B14000D00010
     0.600 [CAN] ID:1E3 DLC:7 DATA:1D300B0B000002
     0.700 [CAN] ID:1E3 DLC:7 DATA:1D300C0B000002
```

---

## 6. Desktop Test Harness & Verification Vectors

All verification vectors are covered under `test/test_protocol_parser/test_can_sniffer.c`:

| Test Function | Verification Coverage |
| :--- | :--- |
| `test_can_sniffer_0xd0_start_stop_and_ack` | Validates downlink `0xD0` parsing and standard Hiworld `0xFF` ACK frame emission. |
| `test_can_sniffer_dedup_and_delta_trigger` | Verifies identical periodic frame deduplication and immediate emission on 1-bit change with accurate repeat counts. |
| `test_can_sniffer_tx_logging_and_filter_bypass` | Confirms unmapped CAN IDs bypass router filters into sniffer, and `CAN_TX` frames carry bit 15 flag. |
| `test_can_sniffer_pacing_and_summary_flush` | Enforces $\le 57$-byte wire packet pacing, summary batching, and `0xFF` completion packet. |
| `test_can_sniffer_suppresses_hu_actions_locks_acc_and_lights` | Validates suppression of UI commands, ACC lock ON, and lighting GPIO hold during log playback. |

