---
name: extract-alert-log
description: >-
  Use this skill when the user asks to print, extract, decode, or analyze Alert Log request data, CAN alert journals (0x120), real-time warning popups (0x1A1), or serial warning frames (0x42/0xEA) from E2E dump logs.
---

# Extract OpenCanbox Vehicle Alert Logs & Request Data

Use this skill to extract, decode, and analyze vehicle alert requests and diagnostic logs from E2E dump log files (`dump_*.log`).

Raw dump logs can contain thousands of repetitive CAN heartbeats and UART keep-alives (`0xFF`). **Never read the entire raw dump log into your context.** Instead, use the automated extraction tool `tools/extract_alert_log.py` to get an instant, fully decoded markdown report.

---

## 1. Quick Usage

Run the tool directly on any target log file:

```bash
python3 tools/extract_alert_log.py <dump_file.log>
```

### Options:
- `--summary-only`: Print only high-level summary tables.
- `--csv <path>`: Override path to `doc/RT4_CAN_ALERTS_0x1A1.csv` (auto-detected by default).
- `--c-profile <path>`: Override path to `src/profiles/peugeot_407.c` (auto-detected by default).

---

## 2. Report Sections & Diagnostic Interpretation

The tool partitions the log analysis into 5 core diagnostic sections:

### 2.1 Section 1: Alert Log Requests & Cockpit Triggers
Pinpoints downlink and in-cabin requests that trigger diagnostic cycles:
- **Cockpit CHECK Button Press (`CAN ID 0x221`, Byte 0 Bit 5 `0x20`):**
  Triggered when the driver presses the cockpit CHECK button on the dashboard or wiper stalk.
- **Head Unit Downlink Query (`APP_UART_RX Cmd 0x2F`):**
  Emitted by the Android infotainment app (`DignosticFrgment`) requesting the active fault journal.
- **Vehicle Model Handshake (`APP_UART_RX Cmd 0x24`):**
  Identifies protocol initialization (e.g. `22 00` -> Peugeot 407).
- **EMF Display Alert Target (`CAN ID 0x167`, Bytes 4..5):**
  Target alert ID addressed by the multi-function display.
- **BSI Diagnostic Request (`CAN ID 0x39B`):**
  Diagnostic session commands routed to the BSI.

### 2.2 Section 2: Persistent Alert Log / Diagnostic Journal (`CAN ID 0x120`)
Reassembles the 21-byte bitfield (168 bits) transmitted by the BSI over 3 multiplexed blocks (`0x7C`, `0xBC`, `0xFC`):
- **Block 1 (`0x7C` / `01b`):** Bytes 0..6 (Bits 0..55)
- **Block 2 (`0xBC` / `10b`):** Bytes 7..13 (Bits 56..111)
- **Block 3 (`0xFC` / `11b`):** Bytes 14..20 (Bits 112..167)

For every set bit, the report displays:
- Bit index, block number, byte and bit position
- Internal `Alarm_BitToIndex_Tab` index
- Canonical 15-bit PSA CAN Alarm ID (`Alarm_IndexToPointer_Tab`)
- Severity level (🛑 STOP, ⚠️ SERVICE, ℹ️ INFO)
- **Sent in 0x42 (Std):** Standard Hiworld wire code dispatched to Head Unit (marked with ✅ if observed in UART stream)
- **Sent in 0xEA (Ext):** Extended PSA CAN wire code dispatched to Head Unit (marked with ✅ if observed in UART stream)
- Official PSA / RT4 alert description
- Vehicle subsystem / category

### 2.3 Section 3: Real-Time Warning Popups (`CAN ID 0x1A1`)
Decodes immediate warning frames (`MSG_BSI_CDE_PTR_MESSAGE`):
- **Display Modal Request:** Byte 2 Bit 7 (`1` = center popup modal, `0` = background update).
- **Acoustic Chime:** Byte 2 Bits [3:0] (Single chime, Double chime, Warning gong, Rapid beep).
- **Affected Doors Bitmask:** Byte 3 decoded into Front Left, Front Right, Rear Left, Rear Right, Boot, Bonnet, Rear Screen.
- **Parameter Detail:** Byte 4 (wheel sensor index, bulb index).

### 2.4 Section 4: Head Unit Uplink Verification (`APP_UART_TX` vs `ORIG_CANBOX`)
Inspects serial transmissions to the Android Head Unit:
- **Cmd `0x42` (Standard Hiworld Warning Info):** Single alert (`Len = 0x02`) or multi-alert summary (`Len = 0x18`).
- **Cmd `0xEA` (Custom Extended Warning Info):** Single alert (`Len = 0x05`) or extended summary (`Len = 0x31`).
- Compares OpenCanbox output (`APP_UART_TX`) against OEM CAN box output (`ORIG_CANBOX`) when available.

### 2.5 Section 5: Diagnostic Cross-Check & Consistency Analysis
Cross-references the active faults in the CAN journal (`0x120`) against the summary packets emitted over UART:
- **Parity Check:** Verifies that all active faults in the CAN journal appear in the serial summary list.
- **Phantom Alert Detection:** Flags any spurious alert IDs present in the serial summary that are NOT active in the vehicle's CAN journal.
- **Missing Alert Detection:** Flags any active vehicle faults omitted from the serial summary.

---

## 3. Workflow for Investigating Alert Issues

1. **Run extraction tool on the dump log:**
   ```bash
   python3 tools/extract_alert_log.py dump_2026-10-05_10-23-19.log
   ```
2. **Review Section 1** to confirm whether a downlink query (`0x2F`) or cockpit trigger (`0x221`) initiated the diagnostic sequence.
3. **Inspect Section 2** for the true list of persistent vehicle faults currently held in the BSI's memory.
4. **Inspect Section 3** to verify whether real-time popups (`0x1A1`) occurred during the capture.
5. **Check Section 5** for diagnostic parity:
   - If **Phantom Alerts** are detected, verify `Alarm_BitToIndex_Tab` in `src/profiles/peugeot_407.c` against `doc/RT4_CAN_ALERTS_0x1A1.csv`.
   - If **Missing Alerts** are detected, check rate limiting, max alert capacity (`CANBOX_MAX_ACTIVE_ALERTS`), or byte offset packing.
