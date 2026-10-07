1. [x] Air intake glitch (RESOLVED)
- Root cause: Frame 0x1E3 bit 4 was mistakenly decoded as `ac_on` instead of Auto Air Intake (AQS), causing `psa_hvac_process_can_0x1e3` to overwrite `aqs_auto = false` whenever blower was in manual speed mode.
- Fixed in `src/profiles/peugeot_407.c` (aligned 0x1D0 and 0x1E3 decoding) and `src/protocols/proto_hiworld_adapter.c`.
- Verified with Unit Test Vector 9 in `test_peugeot_407.c`.

2. [x] Full paths in documentation (RESOLVED)
- Added strict constraint in `AGENTS.md` ("Path Privacy in Documentation") prohibiting absolute local paths in documentation.
- Sanitized existing documentation files to remove local filesystem paths.

3. [x] Air intake glitch on front defrost (RESOLVED)
- Root cause: Frame 0x1D0 Byte 4 sends `0x20` (physical forced fresh air flaps) during front demist (`data[0] == 0x19`), causing `psa_hvac_process_can_0x1d0` to overwrite `aqs_auto = false`, while 0x1E3 maintained `aqs_auto = true` (Byte 0 Bit 4 = `0x10`). This caused continuous cycling between auto and manual fresh air intake.
- Fixed in `src/profiles/peugeot_407.c`: updated `psa_hvac_process_can_0x1d0` to decode `front_max_defrost = (data[0] == 0x19)` and preserve `aqs_auto` from 0x1E3 during front defrost, matching real OEM canbox ground truth from `dump_2026-10-06_14-44-27.log`.
- Verified with Unit Test Vector 10 in `test_peugeot_407.c`.

4. [x] Too many serial frames / HU missing frames (RESOLVED)
- Root causes identified and resolved:
  1) Heartbeat `0xFF` was emitted every 100 ms (10 Hz), contributing 61% of all serial frames and 38% of total byte volume.
     -> Reduced to 1 Hz (1000 ms / 10 ticks) in `src/protocols/proto_hiworld_adapter.c`.
  2) Android HU sent `0x24` (`5A A5 02 24 22 00 47`) every 3 seconds because OpenCanbox never sent the expected ACK frame (`5A A5 01 FF 24 23`). When `0x24` was received, OpenCanbox re-blasted 6 configuration frames (`0xF0`, `0x71`, `0x72`, `0x76`, `0x79`, `0xC1` = 70 bytes) in an immediate 0ms burst.
     -> Aligned with real OEM canbox ground truth: Added immediate ACK response (`5A A5 01 FF 24 23`) upon receiving `0x24` in `src/protocols/hiworld_connection.c`.
     -> Suppressed redundant 70-byte re-transmission if the car model is already initialized and unchanged. Verified in unit and integration test suites.

5. [x] Serial messages split across multiple lines in log (RESOLVED)
- Root cause: UART is an asynchronous byte stream without line delimiters. At 38400 baud, bytes arrive every 260 µs. Non-blocking `ser.read()` in `canbox_e2e_logger.py` logged each raw OS read chunk on a separate line.
- Fixed in `tools/canbox_e2e_logger.py`: Added `SerialFrameReassembler` which parses frames by preamble (`0x5A 0xA5` for Hiworld, `0x2E` for Raise, `0xD5`/`0xFD` for Bagoo) and payload length. Each complete frame is now cleanly logged on its own single line for `APP_UART_TX`, `APP_UART_RX`, and `ORIG_CANBOX`.