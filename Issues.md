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