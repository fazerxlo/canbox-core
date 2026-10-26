---
name: extract-alert-log
description: >-
  Use this skill when the user asks to print, extract, decode, or analyze Alert Log request data, CAN alert journals (0x120), real-time warning popups (0x1A1), or serial warning frames (0x42/0xEA) from E2E dump logs.
---

# Extract OpenCanbox Vehicle Alert Logs

**Never read raw `dump_*.log` files into context.** Run the alert extractor:

```bash
python3 tools/extract_alert_log.py <dump_file.log> [--summary-only]
```

## Report Sections
1. **Triggers:** Cockpit CHECK button (`0x221` B0 bit 5 `0x20`), HU query (`Cmd 0x2F`), handshake (`Cmd 0x24`), EMF target (`0x167`), BSI diag (`0x39B`).
2. **Persistent Journal (`0x120`):** Reassembles 21-byte (168-bit) bitfield across multiplexed blocks `0x7C` (B0..6), `0xBC` (B7..13), `0xFC` (B14..20), mapping bits via `Alarm_BitToIndex_Tab` $\rightarrow$ `Alarm_IndexToPointer_Tab` (`src/profiles/psa_407_alerts.c`) to `0x42` (Std) and `0xEA` (Ext) wire codes.
3. **Real-Time Popups (`0x1A1`):** Modal bit (B2 bit 7), chime (B2[3:0]), door mask (B3), parameter (B4).
4. **Uplink Verification:** Compares `APP_UART_TX` vs `ORIG_CANBOX` for `0x42` (`Len 0x02`/`0x18`) and `0xEA` (`Len 0x05`/`0x31`).
5. **Parity Check:** Flags **Phantom Alerts** (in UART but not `0x120`) and **Missing Alerts** (in `0x120` but missing from UART).
