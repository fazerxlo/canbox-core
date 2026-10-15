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

6. [x] When flashing buzzer make sound and CAN bus is shortened (RESOLVED)
- Root causes:
  1) `hal_system_reboot()` in `hal_system_nuc131.c` called `SYS_ResetChip()` directly without pre-reset GPIO isolation. Transceiver standby `PC3` remained LOW (active), discrete outputs (`PA8/9/12/13`) were active, and `SYS->GPD_MFP` CAN0 pin multiplexing remained mapped across CPU reset.
  2) In `hiworld_iap_nuc131.c`:
     - `SYS_GPD_MFP` was never cleared in `iap_init_hardware()`, keeping `PD6`/`PD7` connected to the unclocked CAN0 peripheral instead of GPIO push-pull recessive `1`.
     - `GPIOA` registers were completely missing in `iap_init_hardware()`, allowing discrete outputs to float.
     - On password mismatch/timeout, if `force_iap` was set, the bootloader prematurely jumped back to the application (`jump_to_app()`), which pulled `PC3` LOW and caused a rapid reboot loop where the buzzer chirped on every reset cycle.
     - `jump_to_app()` pulled `PC3` LOW before jumping to app, rather than letting the application's CAN driver safely release standby once configured.
  3) In deep sleep, `PB0` (UART0_RXD) was not armed as an EXTI wake-up source, ignoring flasher triggers during sleep.
  4) `tools/flash_hiworld_serial.py` lacked serial port auto-detection by active telemetry, and did not flush stale bytes before transmitting the unlock password.
- Fixes applied:
  - Enforced pre-reset isolation in `hal_system_reboot()`: `PA9/12/13` off, `PA8` (ACC) held active HIGH (+12V), `PB3/12` driven HIGH (buzzer silence), `PC3` driven HIGH (transceiver standby), `SYS->GPD_MFP` cleared, `PD7` driven HIGH push-pull (recessive), and UART0 TX FIFO completely drained before `SYS_ResetChip()`.
  - Added `SYS_GPD_MFP` clearing and `GPIOA` configuration (`GPIOA_PMD = 0x75F5FF0F`, `GPIOA_DOUT = 0x00004DFF`, keeping `PA8` ACC active HIGH) in `iap_init_hardware()` and `startup_bootloader.S`.
  - Retained bootloader retry loop when `force_iap` is true (`if (app_valid && !force_iap) jump_to_app()`), eliminating the reboot loop.
  - Removed premature `PC3` low-drive in `jump_to_app()`, keeping CAN transceiver in standby until the application initializes CAN mode.
  - Added EXTI falling-edge wake-up on `PB0` (UART0_RXD) in `hal_system_enter_deep_sleep()`.
  - Enhanced `tools/flash_hiworld_serial.py` to auto-detect active `0x5A 0xA5` telemetry ports, and flush buffers before password transmission.
- Verified on live NUC131 hardware: 100% wireless serial firmware update completed all 192 records (26,112 bytes) in 9.20s with zero acoustic buzzer whine, zero CAN bus disruption, and immediate application boot.

7. [x] When RD4 radio is off the RACIO is continously activating switching (RESOLVED)
- Root cause:
  - From `dump_2026-10-15_18-37-15.log`, when the factory RD4 radio is turned off (`0x165` byte 0 bit 7 = 0), background CAN telemetry from the RD4 tuner (`0x225`) and CD drive (`0x3A5`) continues to be broadcast periodically.
  - Previously, `psa_rd4_process_can_0x165()` only checked if `data[0] == 0x00` to detect standby/off, missing valid standby states like `0x48 c8 00 00` where bit 7 (`0x80`) is clear.
  - In addition, decoders in `src/profiles/profile_psa.c` (`0x225`, `0x265`, `0x2A5`, `0x0A4`, `0x125`, `0x325`, `0x365`, `0x3A5`, `0x3A6`) continued to decode background traffic into `state->media`, which marked `radio.updated = true` and `cdc.updated = true`.
  - In `src/core/can_router.c`, receiving `0x225` or `0x3A5` caused the router to dispatch `0x84 0x01` (Tuner ON FM1) / `0x85` (presets), followed by `0x84 0xFF` (Radio OFF). This continuous flip-flopping every ~500 ms caused the Android Head Unit to repeatedly activate/switch to the Radio app.
- Fixes applied:
  - In `src/profiles/peugeot_407.c` (`psa_rd4_process_can_0x165`): Verified bit 7 (`data[0] & 0x80`) as authoritative RD4 audio power flag. If bit 7 is clear, sets `source_mode = 0xFF`, `power_status = 0x00`, and `radio->updated = true`.
  - In `src/profiles/profile_psa.c`: Added guard `if (state->radio_present && !state->radio_on) return;` across tuner and CD decoder hooks (`0x225`, `0x265`, `0x2A5`, `0x0A4`, `0x125`, `0x325`, `0x365`, `0x3A5`, `0x3A6`), so background bus traffic from an idle RD4 tuner/CD drive does not alter active canonical state or wake the radio when it is confirmed off.
  - In `src/core/can_router.c`: Added `is_radio_off = (s_current_state.radio_present && !s_current_state.radio_on);` checks across `radio.updated`, `preset_updated`, `radio_text_updated`, `cdc.updated`, and the 60s media slow resync. When radio is confirmed off, OpenCanbox completely suppresses `0x84`, `0x85`, `0x86`, and `0x97` serial frames to the Head Unit, while silently keeping `s_last_sent_state.media` synchronized.
- Verified:
  - Integration test `test_integration_hiworld_rd4_radio_off_suppresses_cd_tuner_frames` transcribing the exact vector sequence from `dump_2026-10-15_18-37-15.log` (`0x165` off `48 c8 00 00`, background `0x225`, `0x3A5`, `0x2A5`, 60s resync, and subsequent turn ON).
  - All 110 unit tests in `native_test_runner` and 39 integration tests in `integration_test` passed with zero errors.