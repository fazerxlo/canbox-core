# AI Agent Workflow Guidelines: OpenCanbox Core

Embedded C99 automotive firmware & CAN bus bridge for **Peugeot 407 (PSA CAN2004 / AEE2004 Comfort Bus @ 125 kbps)** to **Hiworld Android Head Unit (`0x5A 0xA5`, 38,400 baud 8N1)**.

---

## 1. Fast-Path Token Efficiency Rules

1. **Silent Builds (`-s`):** Always pass `-s` to PlatformIO builds: `~/.platformio/penv/bin/pio run -s -e stm32_cbox`.
2. **Surgical Line-Range Reads & Hard 120-Line Cap:** **NEVER** call `read_file` without `StartLine`/`EndLine` (or with a span >120 lines) on `.c`, `.h`, or `CANBOX_SPEC_*.md` files. Run `./tools/code_map.py <pattern>` (or `grep -n`) first to get exact function `[StartLine-EndLine]` spans, then batch independent surgical `read_file`, `edit_file`, and command calls in parallel in a single turn.
3. **Capped Greps, Python One-Liners & Output Paging:** Always cap search and script outputs (`| head -n 40`, `-m 20`, or `-c`) so `run_command` never hits the 10 KB truncation ceiling. In `python3 -c` one-liners, never use escaped quotes inside f-string expressions (`f"{d.get(\"k\")}"` fails on Python <3.12). Never run unconstrained greps over `.log` or build files.
4. **Silent Daemons & No Status Polling:** Launch background processes (e.g., OpenOCD, loggers) quietly (`-d0` or redirecting stdout/stderr to a log file). Never poll `manage_task status` on high-throughput daemons or long-running verification commands (`verify_all.sh`); stop calling tools and await reactive system wakeup.
5. **Targeted Log Slicing:** When analyzing multi-megabyte captures, never view or cat raw files. Use `tools/triage_canbox_log.py`, `candump` timestamp ranges, or focused one-line Python parsers bounded to $\pm 1\text{ s}$ around the event.
6. **Scripted Mass Edits & `edit_file` Over `write_file`:** Prefer `edit_file` on changed functions over rewriting entire >200-line files with `write_file`. Use Python/shell one-liners via `run_command` when refactoring across 3+ files or pruning >30% of a file.
7. **Test Fast, Gate Once:** Iterate with `./tools/test_fast.sh` (or `./tools/test_fast.sh -e integration_test`). **NEVER execute `.pio/build/native_test/program` directly** (it is an infinite-loop SocketCAN daemon that hangs; unit tests live in `native_test_runner`). Run `./tools/verify_all.sh` **strictly once** at task completion. Never re-run C builds after editing `*.md` docs.
8. **Single-Call Log Triage:** Run `./tools/triage_canbox_log.py <dump.log>` once (it includes CAN state transitions, `[APP_UART_RX]` downlinks, `[APP_UART_TX]` uplinks, and causal timelines). Avoid writing ad-hoc Python log parsers unless inspecting an unlisted CAN ID. Note: `[APP_UART_TX]` in logs reflects the binary flashed at capture time, not necessarily current local source.
9. **ESP-IDF Registration:** Always register new `src/**/*.c` files in `src/CMakeLists.txt` in the same turn.
10. **Spec Implementation Fast Path:** Treat user specs (`doc/CANBOX_SPEC_*.md`) as pre-verified ground truth. Do not run exploratory greps or subagent audits on documented bitfields/formulas. Run `./tools/code_map.py <CAN_ID|Cmd>` + surgical reads in parallel (Turn 1), batch all code edits (Turn 2), and batch test + doc updates (Turn 3).
11. **Pre-Diagnosed Bug Fast Path:** When the user provides target files, functions, log evidence, or root causes (or when resuming from a compacted summary with established RCA), execute strictly via the 3-turn pipeline: Turn 1 (parallel surgical reads of target function + test via `./tools/code_map.py`), Turn 2 (batch edits across source, builders, tests, and specs in a single turn), Turn 3 (run `./tools/test_fast.sh` with `WaitMsBeforeAsync=10000`, then `./tools/verify_all.sh` once with zero status polling). Never run broad exploratory greps, archive scans, or external decompiled codebase searches (`QF_Canbus_system/`) on diagnosed bugs.
12. **External Firmware & Disassembly Audits (`research` Subagent):** When cross-checking external decompiled codebases or disassembly (`hiworld-canbus-PSA/`, `payload.asm`, `payload.bin`, `QF_Canbus_system/`) exceeding ~300 lines, delegate the binary/ASM audit to `invoke_subagent(TypeName="research")` so raw disassembly stays out of the main context window and avoids triggering `/compact`. Never dump multi-thousand-line annotated `.asm` files to `/tmp/*.txt` and page through them with `read_file`/`sed`.
13. **Physical Workbench Verification Fast Path (`can0` & UART):**
    - **Port & Bus Topology:** `can0` is PSA Comfort CAN @ **125 kbps** on `/dev/ttyUSB3` (held by `slcand`; **NEVER** probe or open `/dev/ttyUSB3` directly). Canbox UART is on `/dev/ttyUSB0` @ fixed **38,400 baud, 8N1** (Raise `0x2E` & Hiworld `0x5A`; **NEVER** toggle or drop baud to 19,200).
    - **Sleeping Bus Keepalive & Interlocks:** Standalone workbench ECUs require 40 ms continuous keepalive (`0x036`, `0x0F6`, `0x128`, `0x15B`, `0x168`). Climate telemetry strictly requires **`0x1E3`** (`MSG_ETAT_CLIMAV_EMF`) alongside `0x1D0`. High-current heating loads (Rear Defrost / Heated Mirrors) require engine running simulation on **`0x0B6`** (900 RPM: `1C 20 00 00 00 00 00 D0`) and `0x12D` (`80 32 32 00 64 00 00 80`), otherwise the BSI interlock inhibits heating.
    - **Rear Defrost Ground Truth:** Fascia button demand is `0x1D0` Byte 0 Bit 6 (`0x40`: `0x29` $\longleftrightarrow$ `0x69`); Climate ECU indicator is `0x1E3` Byte 1 Bit 3 (`0x08`: `0x00` $\longleftrightarrow$ `0x08`).
    - **Standard Workbench Toolchain:** Never write ad-hoc Python test scripts when these proven tools exist:
      - `python3 tools/probe_canbox_uart.py --port /dev/ttyUSB0` (Baud & protocol lock in <2s)
      - `python3 tools/workbench_live_sniffer.py` (Real-time delta CAN + UART sniffer)
      - `python3 tools/test_clima_canbox.py` (Climate telemetry sweep)
      - `python3 tools/workbench_defrost_verifier.py --port /dev/ttyUSB0 -v` (Automated 3-cycle verifier)

---

## 2. Architecture, Subsystem & Spec Map

| Layer | Files | Responsibility |
| :--- | :--- | :--- |
| **L1: HAL** | `include/hal/*.h`, `src/hal/` | Hardware abstraction (`hal_can`, `hal_uart`, `hal_gpio`, `hal_system`). Zero MCU headers outside `src/hal/`. |
| **L2: Profile** | `src/profiles/profile_psa.c`<br>`src/profiles/psa_407_*.c`<br>`src/profiles/psa_source_arbiter.c` | `s_psa_rules[]` routing table (`profile_psa.c`), CAN2004 decoders (`psa_407_rd4_audio.c`, `psa_407_hvac_body.c`, `psa_407_telemetry.c`, `psa_407_alerts.c`, `psa_407_car_config.c`), Source Arbiter (`psa_source_arbiter.c`). |
| **L3: Router** | `include/core/can_router.h`<br>`src/core/can_router.c` | $O(1)$ 256-byte CAN ID bitmask filter, `vehicle_state_t` cache, `dirty.*` + `memcmp` delta dispatch, 60s periodic full resync. |
| **L4/L5: Proto** | `src/protocols/proto_hiworld*.c`<br>`src/protocols/hiworld_*.c` | Hiworld framing (`0x5A 0xA5 <len> <cmd> <payload...> <cs>`, `cs = ((len + cmd + sum) - 1) & 0xFF`), builders (`hiworld_psa_builders.c`), handshake (`hiworld_connection.c`). |

### Subsystem Lookup Table (CAN IDs $\leftrightarrow$ UART Cmds $\leftrightarrow$ Files $\leftrightarrow$ Specs)

| Subsystem | CAN IDs (RX / TX) | Hiworld UART Cmds | Source & Unit Test Files | Spec File (`doc/`) |
| :--- | :--- | :--- | :--- | :--- |
| **Keys / Stalk / Console** | `0x21F`, `0x221`, `0x3E5` | `0x11`, `0x21` | `psa_407_hvac_body.c`, `test_407_swc_hvac_radar.c` | `CANBOX_SPEC_HIWORLD_407_01_STEERING_STALK_KEYS.md` |
| **HVAC / Climate** | `0x1D0`, `0x208`, `0x0F6` | `0x31` | `psa_407_hvac_body.c`, `test_407_swc_hvac_radar.c` | `CANBOX_SPEC_HIWORLD_407_02_DUAL_ZONE_HVAC.md` |
| **TPMS** | `0x268` | `0x66`, `0x65` | `psa_407_telemetry.c`, `test_407_tpms_extended.c` | `CANBOX_SPEC_HIWORLD_407_03_TPMS_TIRE_PRESSURE.md` |
| **Parking Radar (AAS)** | `0x0E1` | `0x41`, `0x43` | `psa_407_hvac_body.c`, `test_407_swc_hvac_radar.c` | `CANBOX_SPEC_HIWORLD_407_04_PARKING_RADAR_AAS.md` |
| **Steering Angle (SAS)** | `0x305` | `0x11` (Bytes 8–9) | `psa_407_hvac_body.c`, `test_407_swc_hvac_radar.c` | `CANBOX_SPEC_HIWORLD_407_05_STEERING_ANGLE_SAS.md` |
| **Trip Computer** | `0x2A1`, `0x261`, `0x221`, `0x167` (TX) | `0x13`–`0x15`, `0x1B` | `psa_407_telemetry.c`, `psa_407_car_config.c`, `test_407_trip_doors.c` | `CANBOX_SPEC_HIWORLD_407_06_TRIP_COMPUTER_TELEMETRY.md` |
| **Doors / Body / Speed** | `0x168`, `0x220`, `0x128`, `0x0B6` | `0x11`, `0x12`, `0x79` | `psa_407_telemetry.c`, `test_407_trip_doors.c` | `CANBOX_SPEC_HIWORLD_407_07_DOORS_BODY_STATUS.md` |
| **JBL Amplifier / DSP** | `0x1E5`, `0x1A5` | `0x56`, `0x86`, `0xAD` | `psa_407_rd4_audio.c`, `test_407_rd4_media.c` | `CANBOX_SPEC_HIWORLD_407_08_JBL_AMPLIFIER_DSP.md` |
| **RD4 Audio & Arbiter** | `0x165`, `0x225`, `0x2A5`, `0x3A5`, `0x0A4` | `0x84`, `0x85`, `0x97`, `0x60` | `psa_407_rd4_audio.c`, `psa_source_arbiter.c`, `test_407_rd4_media.c`, `test_407_source_arbiter.c` | `CANBOX_SPEC_HIWORLD_407_09_RD4_MFD_MEDIA_TEXT.md` |
| **Clock / MFD Emulation** | `0x3F6` (TX), `0x0DF` (TX), `0x167` (TX), `0x525` | `0xCB`, `0xA6`, `0x9A`, `0xCA` | `psa_407_car_config.c`, `test_407_car_config.c` | `CANBOX_SPEC_HIWORLD_407_10_BSI_SETTINGS_CLOCK_SYNC.md` |
| **Car Config (BSI)** | `0x361`, `0x260`, `0x15B` (TX) | `0x71`, `0x72`, `0x76`, `0x79`, `0x7B`, `0x7D` | `psa_407_car_config.c`, `test_407_car_config.c` | `CANBOX_SPEC_HIWORLD_407_11_CAR_CONFIGURATION_OPTIONS.md` |
| **Alerts & Diagnostics** | `0x120`, `0x1A1` | `0x42`, `0xEA`, `0xFA` | `psa_407_alerts.c`, `test_407_alerts.c` | `CANBOX_SPEC_HIWORLD_407_12_ALERTS_DIAGNOSTICS.md` |
| **Handshake / Link** | — | `0x24`, `0x30`, `0xFE`, `0xFF`, `0xF0` | `hiworld_connection.c`, `proto_hiworld.c`, `test_hiworld_parser.c` | `CANBOX_SPEC_HIWORLD_GENERIC_CONNECTION_PHASE.md` |
| **OEM Firmware Diffs** | All 407 IDs | All 407 Cmds | `profile_psa.c`, `hiworld_psa_builders.c` | `CANBOX_SPEC_HIWORLD_CUSTOMIZATIONS_AND_DIFFERENCES.md` |

### Key State-Machine & Timing Rules
* **RD4 Audio (`0x165` Authority):** `0x165` owns `present`, `sleep`, `power_status`, and `source_mode` (`<0x20` Tuner, `0x20` CD, `0x31` CDC, `0x40..0x41` AUX, `0xFF` OFF). Sub-frames (`0x225`, `0x2A5`, `0x0A4`, `0x3A5`, etc.) only update cached fields without turning radio ON or overwriting `source_mode >= 0x20`.
* **Serial Dispatch Classes (38,400 baud protection):**
  - **Immediate Event (0 ms cooldown, on change):** Keys (`0x11`, `0x21`), Doors (`0x12`), Radar (`0x41`), Alerts (`0x42`, `0xEA`), Audio/DSP (`0x56`, `0x84`–`0x86`, `0x97`).
  - **Throttled Fast Telemetry ($\le 10\text{ Hz}$):** SAS steering angle (`0x11` in reverse), speed/RPM.
  - **State-Cached Subsystem ($\le 2\text{ Hz}$ on change):** HVAC (`0x31`), Trip (`0x13`–`0x15`), TPMS (`0x66`).
  - **Heartbeat & Resync:** Link ping `0xFF` at $1\text{ Hz}$ (1000 ms); full state resync on `0x24`/`0x30` handshake and every $60\text{ s}$.

---

## 3. Strict Engineering Boundaries

* **Pure C99 & Zero Heap:** `-std=c99 -Wall -Wextra -Werror`. `malloc`/`calloc`/`free` and blocking loops are banned.
* **Lock-Free SPSC:** Power-of-two `ring_buffer_t` only; no mutexes/RTOS primitives in core.
* **Safe Unpacking & Bounds:** Always check `frame->dlc` before indexing `data[]`. Use `read_be16()`/`read_le16()` (PSA CAN is Big-Endian; watch Java signed-byte casts `(uint8_t)` and sentinels `0xFF`/`0xFFFF`).
* **Strict CAN2004 Isolation:** Only implement proven CAN2004 signals (e.g., radar is `0x0E1`; `0x260` is BSI profiles, NOT radar). Zero unverified placeholders.
* **Path Privacy:** Never write absolute paths (`/home/...` or `file:///...`) in repo `*.md` files.
* **Doc & Tracker Updates:** Update `doc/CANBOX_SPEC_HIWORLD_407_*.md` on spec changes. Update `doc/TODO_PROGRESS.md` **only** when a feature status (`[ ]`/`[-]`/`[x]`) or command changes.
* **Lifecycle:** Ingest spec $\rightarrow$ clarify only if blocked $\rightarrow$ implement Uplink (L1–L5) $\rightarrow$ add Unity tests in `test/test_protocol_parser/test_407_*.c` $\rightarrow$ update spec $\rightarrow$ ask user before implementing Downlink.
