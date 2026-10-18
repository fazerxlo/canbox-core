---
name: fix-canbox-from-log
description: >-
  Use this skill when the user provides an E2E dump log (`dump_*.log`) or asks to fix, resolve, debug, or align an application bug or protocol issue based on vehicle or bench logs.
---

# Fix OpenCanbox from E2E Dump Logs

Resolve translation bugs, state oscillation, and protocol mismatches from `dump_*.log` files.

## Fast-Path Rules
1. **User-Rule Short-Circuit:** If the user states the exact fix rule along with a log, extract only the `[CAN_RX]` test vectors needed for regression tests and implement immediately.
2. **`[APP_UART_TX]` Caveat:** `[APP_UART_TX]` in hardware logs came from the MCU binary flashed at capture time, not necessarily current local C code.
3. **Never Read Raw Logs:** Always use triage/diff scripts below.

## Workflow
1. **Automated Triage:**
   ```bash
   ./tools/triage_canbox_log.py <dump_file.log>
   ```
2. **Focused Diff (if needed):**
   ```bash
   ./tools/diff_canbox_frames.py <dump_file.log> --preset <hvac|radar|radio|swc|doors|trip|tpms|alerts|bsi_config>
   # Or explicit: ./tools/diff_canbox_frames.py <dump_file.log> -c <CAN_IDS> -p <CMD_HEX>
   ```
3. **Diagnose Layer:**
   - **Oscillation:** Two CAN frames (`0x1D0` vs `0x1E3`, or `0x165` vs `0x225`/`0x3A5`) overwriting the same field. Gate sub-frames so authoritative frames (`0x165`, `0x1E3`) own mode transitions.
   - **L2 (`src/profiles/psa_407_*.c`, `profile_psa.c`):** Bitmasks, DLC bounds, sentinel guards.
   - **L3 (`src/core/can_router.c`):** `dirty.*` / `memcmp` delta logic and `source_mode` gating.
   - **L5 (`src/protocols/proto_hiworld_adapter.c`, `hiworld_psa_builders.c`):** Wire offsets & checksums.
4. **Verify & Test:**
   Add log CAN vectors to `test/test_protocol_parser/test_407_*.c` or `test/test_integration/test_integration_hiworld.c`, iterate with `./tools/test_fast.sh`, and run `./tools/verify_all.sh` once at completion.
