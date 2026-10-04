1. [x] Air intake glitch (RESOLVED)
- Root cause: Frame 0x1E3 bit 4 was mistakenly decoded as `ac_on` instead of Auto Air Intake (AQS), causing `psa_hvac_process_can_0x1e3` to overwrite `aqs_auto = false` whenever blower was in manual speed mode.
- Fixed in `src/profiles/peugeot_407.c` (aligned 0x1D0 and 0x1E3 decoding) and `src/protocols/proto_hiworld_adapter.c`.
- Verified with Unit Test Vector 9 in `test_peugeot_407.c`.

2. [x] Full paths in documentation (RESOLVED)
- Added strict constraint in `AGENTS.md` ("Path Privacy in Documentation") prohibiting absolute local paths in documentation.
- Sanitized existing documentation files to remove local filesystem paths.

