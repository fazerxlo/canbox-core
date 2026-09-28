# OpenCanbox Differential Frame Analyzer (`diff_canbox_frames.py`)

A high-performance diagnostic tool for analyzing OpenCanbox E2E dump logs (`dump_*.log`). It automatically correlates CAN bus actions with serial output, and calculates the exact byte-by-byte differences between the original OEM Canbox (`[ORIG_CANBOX]`) and the OpenCanbox emulator (`[APP_UART_TX]`).

---

## 1. Why This Tool?

Raw E2E logs generated during bench tests typically contain **5,000 to 6,000+ lines** of high-frequency periodic traffic:
* CAN bus background heartbeats repeating every 10–50ms.
* Periodic serial keep-alives (e.g. Hiworld `0xFF` heartbeat every 100ms).

Manually inspecting or passing raw logs to an AI agent burns tens of thousands of tokens and wastes valuable development time. 

`diff_canbox_frames.py` solves this by:
1. **Deduplicating & Compressing Traffic:** Compresses 6,000-line logs down to ~20–70 key events (**98%+ token and noise reduction**).
2. **Reassembling Fragmented Serial Streams:** Handles fragmented UART reads from physical serial ports (e.g., `5a` split from `a50c31...`) and multi-packet burst writes.
3. **Correlating CAN Actions to Responses:** Directly links state changes on the vehicle CAN bus to the serial packets emitted by both the real OEM canbox and OpenCanbox.
4. **Pinpointing Discrepancies:** Highlights the exact differing byte indices, payload values, bitwise masks, and checksum validity.

---

## 2. Requirements

* **Python 3.8+**
* **Standard Library Only:** Zero external dependencies (no `pip install` required).

---

## 3. CLI Usage & Syntax

Make sure the script is executable:
```bash
chmod +x tools/diff_canbox_frames.py
```

Run the analyzer:
```bash
./tools/diff_canbox_frames.py <dump_file.log> [OPTIONS]
```

### Options Reference

| Argument | Description | Example |
| :--- | :--- | :--- |
| `log` / `-l`, `--log` | Path to the dump log file. | `dump_2026-09-24_18-13-11.log` |
| `-c`, `--can-ids` | Filter interesting CAN IDs in hex (space- or comma-separated). If omitted, tracks any CAN ID that changed state. | `-c 1D0 1E3` or `-c 1d0,12d` |
| `-p`, `--prefix` | Filter interesting serial frame prefixes or command opcodes (hex). | `-p 31` or `-p 5aa50c31` or `-p 11 12` |
| `-w`, `--window` | Maximum time correlation window in seconds between CAN actions and serial packets (default: `1.2s`). | `-w 0.5` |
| `-s`, `--summary` | Output only the aggregate command difference summary table. | `-s` |
| `--export-compressed` | Save the deduplicated/compressed event timeline to a file. | `--export-compressed compact.log` |
| `--no-dedup` | Disable deduplication of repeated consecutive CAN and serial frames. | `--no-dedup` |
| `--keep-heartbeat` | Include periodic heartbeat frames (`0xFF` / `5aa501ff...`). | `--keep-heartbeat` |
| `--no-color` | Disable ANSI color highlighting (useful for piping into files). | `--no-color` |

---

## 4. Practical Recipes

### HVAC / Climate Control Debugging
Isolate climate CAN messages (`0x1D0`, `0x1E3`, `0x12D`) and compare Hiworld HVAC status frames (`CMD 0x31`):
```bash
./tools/diff_canbox_frames.py dump_2026-09-24_18-13-11.log -c 1D0 1E3 12D -p 31
```

### Doors & Windows Status
Check door contact CAN frames against Hiworld door packets (`CMD 0x12`):
```bash
./tools/diff_canbox_frames.py dump_*.log -c 36 E1 F6 -p 12
```

### Steering Wheel & Stalk Keys
Inspect button presses and wheel scroll packets (`CMD 0x21` / `0x22`):
```bash
./tools/diff_canbox_frames.py dump_*.log -c 217 21F -p 21 22
```

### Reverse & Parking Radar
Inspect parking sensor CAN signals against radar status frames (`CMD 0x32` / `0x33`):
```bash
./tools/diff_canbox_frames.py dump_*.log -c 128 161 -p 32 33
```

### Quick Diagnostic Summary
Scan an entire 6,000-line capture across all commands without raw frame logs:
```bash
./tools/diff_canbox_frames.py dump_*.log -s
```

### Export a Compressed Log
Generate an ultra-compact version of the log containing only relevant transitions for archival or sharing:
```bash
./tools/diff_canbox_frames.py dump_*.log -c 1D0 1E3 -p 31 --export-compressed compressed.log
```

---

## 5. Output Format Explanation

Each relevant event is presented in a self-contained block:

```text
================================================================================
EVENT #2 | Time: 1790266362.075 - 1790266362.298 (dt: +0.223s) | CMD: 0x31 (HVAC_STATUS)
--------------------------------------------------------------------------------
CAN ACTION:
  1790266362.021 | ID: 0x1E3  (DLC: 8) DATA: 11 30 12 0b 00 [20] 03 00 | Delta: Byte 5: 0x60 -> 0x20
  1790266362.297 | ID: 0x1D0  (DLC: 8) DATA: 28 00 03 [02] 00 12 0b 00 | Delta: Byte 3: 0x06 -> 0x02

CANBOX APP DIFFERENCE:
  ORIG (1790266362.075): 5a a5 0c 31 45 08 00 03 30 04 32 2a 00 00 00 [78] [94]
  APP  (1790266362.298): 5a a5 0c 31 45 08 00 03 30 04 32 2a 00 00 00 [00] [1c]

  DIFFERENCE BREAKDOWN:
    * Frame Byte 15   : ORIG=0x78 (120)  !=  APP=0x00 (  0)  [XOR: 0x78]
    * Checksum Byte   : ORIG=0x94 (148)  !=  APP=0x1C ( 28)  [XOR: 0x88]
================================================================================
```

### Key Elements:
* **`CAN ACTION`:** Shows the exact CAN ID and data bytes that triggered the event. Changed bytes are bracketed `[ ]` and described under `Delta: Byte X: Old -> New`.
* **`CANBOX APP DIFFERENCE`:** Displays the full serial frame from OEM hardware (`ORIG`) and OpenCanbox (`APP`), with differing bytes highlighted.
* **`DIFFERENCE BREAKDOWN`:** Shows the index of the mismatch, decimal and hex values, and bitwise difference (`XOR`).
* **Discrepancy Indicators:**
  * `[NO APP RESPONSE DETECTED]`: Real OEM canbox emitted a packet, but OpenCanbox was silent (unhandled trigger or missing parser logic).
  * `[NO OEM CANBOX FRAME]`: OpenCanbox emitted an extra or spurious packet that the OEM hardware never sends.
  * `[INVALID CS]`: Protocol checksum calculation mismatch.

---

## 6. End-of-Run Summary

At the end of analysis, a breakdown table summarizes overall parity:

```text
================================================================================
                               ANALYSIS SUMMARY                                 
================================================================================
  Total Event Pairs      : 99
  Matched & Identical    : 0
  Matched with Diff      : 20
  Missing in App (Silent): 30
  Extra/Spurious in App  : 49

  Command Breakdown:
    - CMD: 0x11 (CAR_BASE_INFO)     : 2 diffs, 18 extra in app | Differing Byte Indices: [2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]
    - CMD: 0x12 (DOORS_WINDOWS)     : 2 missing in app
    - CMD: 0x31 (HVAC_STATUS)       : 16 diffs, 3 extra in app | Differing Byte Indices: [8, 15, 16]
    - CMD: 0x71 (FEATURE_CONFIG_1)  : 1 diffs, 1 missing in app, 9 extra in app | Differing Byte Indices: [4, 5, 6]
================================================================================
```

