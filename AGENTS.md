# AI Agent Workflow Guidelines: OpenCanbox Core

Embedded C99 CAN-to-UART bridge for **Peugeot 407 (PSA CAN2004 Comfort @ 125 kbps)** $\leftrightarrow$ **Hiworld Android HU (`0x5A 0xA5`, 38,400 8N1)**.

---

## 1. Fast-Path Token Efficiency Rules

1. **Parallel Batching & Hard 120-Line Read Cap:**
   - **NEVER** issue sequential single-tool turns when reads, greps, or edits can be batched (3–6 parallel calls/turn).
   - **NEVER** call `read_file` without `StartLine`/`EndLine` or with a span $>120$ lines on `.c`, `.h`, or `CANBOX_SPEC_*.md`.
   - Never re-read identical line ranges across turns or after `/compact`; rely on `<CONTEXT_SUMMARY>` and thought cache.
2. **3-Turn Fast Path (Specs, Diagnosed Bugs & Post-`/compact`):**
   - Treat `doc/CANBOX_SPEC_*.md` and user RCAs as ground truth (zero exploratory greps or subagent audits on documented bitfields).
   - **Turn 1 (Locate & Read):** Parallel batch `./tools/code_map.py <symbol|CAN_ID|Cmd>` (or `grep -rn "typedef struct <sym>"`), `grep -rn <fn> test/`, and surgical `read_file` spans. Never chain `find`/`grep` across turns.
   - **Turn 2 (Batch Edits):** Prefer `edit_file` over `write_file` (on files $>200$ lines). Use Python/shell one-liners via `run_command` when refactoring $3+$ files or pruning $>30\%$ of a file. Register any new `src/**/*.c` in `src/CMakeLists.txt` in the same turn.
   - **Turn 3 (Test Fast, Gate Once):** Iterate with `./tools/test_fast.sh` (`WaitMsBeforeAsync=10000`) or `./tools/test_fast.sh -e integration_test`. Run `./tools/verify_all.sh` **strictly once** at task completion. Always pass `-s` to PlatformIO (`~/.platformio/penv/bin/pio run -s -e stm32_cbox`). **NEVER** run `.pio/build/native_test/program` (infinite SocketCAN daemon; unit tests live in `native_test_runner`). Skip C builds after `.md`-only edits.
3. **Capped Outputs, Safe Python One-Liners & Single-Call Log Triage:**
   - Cap all search/script outputs (`| head -n 40`, `-m 20`, `-c`) to avoid the 10 KB truncation ceiling. Never run unconstrained greps on `.log` or build dirs.
   - In `python3 -c '...'`, never nest or escape quotes (`\"`) inside f-string `{...}`; bind local variables first or use `%` / `.format()`.
   - For multi-MB logs, never `cat`/view raw files. Run `./tools/triage_canbox_log.py <dump.log>` **once** (covers CAN transitions, `[APP_UART_RX]`, `[APP_UART_TX]`, and timelines; note `[APP_UART_TX]` reflects the binary flashed at capture time) or slice $\pm 1\text{ s}$ around events.
4. **Silent Daemons & Zero Polling:**
   - Start background daemons (`openocd`, loggers) silently (`-d0` or redirect stdout/stderr to file).
   - **NEVER** poll `manage_task status` on daemons or `verify_all.sh`; stop calling tools and await reactive wakeup.
5. **HU Triage Pipeline & External Binary Audits (`research` Subagent):**
   - Trace HU events (`Cmd 0x56`, `0x84`, `0x31`) locally: (1) `doc/CANBOX_SPEC_*.md` $\to$ (2) `src/protocols/hiworld_psa_builders.c` (`vehicle_state_t` field) $\to$ (3) `src/core/can_router.c` (`dirty.*`/heartbeat) $\to$ (4) `src/profiles/psa_407_*.c`. **NEVER** grep/decompile `../QF_Canbus_system` unless a packet is undocumented in `canbox-core`.
   - For external decompiled/ASM audits $>300$ lines (`hiworld-canbus-PSA/`, `payload.asm`, `payload.bin`, `QF_Canbus_system/`), delegate to `invoke_subagent(TypeName="research")`. Never dump `.asm` to `/tmp/*.txt` to page with `read_file`/`sed`.
6. **RT4 PowerPC / IDA / Ghidra Disassembly Rules:**
   - **Jump Tables ($4\times$ offset):** PPC `slwi r, 2` / `rlwinm` indexes 4-byte words; table offsets `0x00, 0x04, 0x08, 0x0C, 0x10` correspond to logical enums `0, 1, 2, 3, 4`.
   - **Bitfield Widths:** `Bits 7:3` = 5 bits (mask `0x1F`, $0\dots 31$). Never truncate with `& 0x07` (turns unmonitored `0x40 >> 3 = 0x08` into false normal `0x00`).
   - **Sentinels:** `Invalid / Default` values (e.g., `0x8` in bits 7:3 $\to$ raw `0x40`, or `0x10` $\to$ `0x80`) mean offline/unmonitored/fault—never decode as normal or synthesize values.
7. **Live Hardware, `can0` (`tools/slcan.sh`) & 2-Turn Ablation:**
   - **Mandatory Skill:** Read `.agents/skills/debug-canbox-firmware-live/SKILL.md` before any live testing across HU (`../QF_Canbus_system`), MCU (`openocd_nuc131.cfg`), and `can0` (BSI or `../simulator`).
   - **No Unapproved Resets:** **NEVER** flash/reset MCU (`openocd`/`nuc131_flasher.py`) or deploy/restart HU app (`build_and_deploy.sh`/`adb_upload.sh`/`am force-stop`) without explicit user approval (wipes live state).
   - **Manage `can0` Strictly via `tools/slcan.sh`** (or `tools/slcan.sh -k` to stop). Never run raw `slcand` or `ip link` manually.
   - **2-Turn `can0` Ablation:** When diagnosing missing-ECU frame dependencies (e.g., `0x15B`, `0x1ED`), **NEVER** run interactive one-off `python3 -c` injections across multiple turns:
     1. *Turn 1:* Run one script on `dump_*.log` to extract all CAN IDs, payloads, and exact periods (`100/200/500/1000 ms`).
     2. *Turn 2:* Run one self-contained Python ablation script on `can0` (using `can_filters=[...]`) performing automated leave-one-out/binary search and printing only the minimal winning subset.

---

## 2. Architecture, Subsystem & Spec Map

| Layer | Files | Responsibility |
| :--- | :--- | :--- |
| **L1: HAL** | `include/hal/*.h`, `src/hal/` | `hal_can`, `hal_uart`, `hal_gpio`, `hal_system`. Zero MCU headers outside `src/hal/`. |
| **L2: Profile** | `src/profiles/profile_psa.c`, `psa_407_*.c`, `psa_source_arbiter.c` | `s_psa_rules[]` table, CAN2004 decoders (`rd4_audio`, `hvac_body`, `telemetry`, `alerts`, `car_config`), Source Arbiter. |
| **L3: Router** | `include/core/can_router.h`, `src/core/can_router.c` | $O(1)$ 256-byte ID bitmask filter, `vehicle_state_t` cache, `dirty.*` + `memcmp` delta dispatch, 60s full resync. |
| **L4/L5: Proto** | `src/protocols/proto_hiworld*.c`, `hiworld_*.c` | Framing (`0x5A 0xA5 <len> <cmd> <payload> <cs>`, `cs = ((len + cmd + sum) - 1) & 0xFF`), `hiworld_psa_builders.c`, `hiworld_connection.c`. |

### Subsystem Lookup (`src/profiles/`, `test/test_protocol_parser/`, `doc/CANBOX_SPEC_HIWORLD_...`)

| Subsystem | CAN IDs (RX / TX) | UART Cmds | Source & Unit Test Files | Spec Suffix (`doc/CANBOX_SPEC_HIWORLD_...`) |
| :--- | :--- | :--- | :--- | :--- |
| **Keys / Stalk / Console** | `0x21F`, `0x221`, `0x3E5` | `0x11`, `0x21` | `psa_407_hvac_body.c`, `test_407_swc_hvac_radar.c` | `407_01_STEERING_STALK_KEYS.md` |
| **HVAC / Climate** | `0x1E3`, `0x1D0`, `0x0F6` | `0x31` | `psa_407_hvac_body.c`, `test_407_swc_hvac_radar.c` | `407_02_DUAL_ZONE_HVAC.md` |
| **TPMS** | `0x268` | `0x66`, `0x65` | `psa_407_telemetry.c`, `test_407_tpms_extended.c` | `407_03_TPMS_TIRE_PRESSURE.md` |
| **Parking Radar (AAS)** | `0x0E1` | `0x41`, `0x43` | `psa_407_hvac_body.c`, `test_407_swc_hvac_radar.c` | `407_04_PARKING_RADAR_AAS.md` |
| **Steering Angle (SAS)** | `0x305` | `0x11` (B8–9) | `psa_407_hvac_body.c`, `test_407_swc_hvac_radar.c` | `407_05_STEERING_ANGLE_SAS.md` |
| **Trip Computer** | `0x2A1`, `0x261`, `0x221`, `0x167` (TX) | `0x13`–`0x15`, `0x1B` | `psa_407_telemetry.c`, `psa_407_car_config.c`, `test_407_trip_doors.c` | `407_06_TRIP_COMPUTER_TELEMETRY.md` |
| **Doors / Body / Speed** | `0x220`, `0x036`, `0x0F6`, `0x128`, `0x0B6` | `0x11`, `0x12`, `0x79` | `psa_407_telemetry.c`, `test_407_trip_doors.c` | `407_07_DOORS_BODY_STATUS.md` |
| **JBL Amplifier / DSP** | `0x1E5`, `0x1A5` | `0x56`, `0x86`, `0xAD` | `psa_407_rd4_audio.c`, `test_407_rd4_media.c` | `407_08_JBL_AMPLIFIER_DSP.md` |
| **RD4 Audio & Arbiter** | `0x165`, `0x225`, `0x265`, `0x2A5`, `0x0A4`, `0x325`–`0x3A6` | `0x84`, `0x85`, `0x97`, `0x60` | `psa_407_rd4_audio.c`, `psa_source_arbiter.c`, `test_407_rd4_media.c`, `test_407_source_arbiter.c` | `407_09_RD4_MFD_MEDIA_TEXT.md` |
| **Clock / MFD Emulation** | `0x3F6` (TX), `0x0DF` (TX), `0x167` (TX), `0x525` | `0xCB`, `0xA6`, `0x9A`, `0xCA` | `psa_407_car_config.c`, `test_407_car_config.c` | `407_10_BSI_SETTINGS_CLOCK_SYNC.md` |
| **Car Config (BSI)** | `0x361`, `0x260`, `0x15B` (TX) | `0x71`, `0x72`, `0x76`, `0x79`, `0x7B`, `0x7D` | `psa_407_car_config.c`, `test_407_car_config.c` | `407_11_CAR_CONFIGURATION_OPTIONS.md` |
| **Alerts & Diagnostics** | `0x120`, `0x1A1` | `0x42`, `0xEA`, `0xFA` | `psa_407_alerts.c`, `test_407_alerts.c` | `407_12_ALERTS_DIAGNOSTICS.md` |
| **Handshake / Link** | — | `0x24`, `0x30`, `0xFE`, `0xFF`, `0xF0` | `hiworld_connection.c`, `proto_hiworld.c`, `test_hiworld_parser.c` | `GENERIC_CONNECTION_PHASE.md` |
| **OEM Firmware Diffs** | All 407 IDs | All 407 Cmds | `profile_psa.c`, `hiworld_psa_builders.c` | `CUSTOMIZATIONS_AND_DIFFERENCES.md` |

### Key State-Machine & Timing Rules
* **RD4 (`0x165` Authority):** `0x165` owns `present`, `sleep`, `power_status`, and `source_mode` (`<0x20` Tuner, `0x20` CD, `0x31` CDC, `0x40..0x41` AUX, `0xFF` OFF). Sub-frames (`0x225`, `0x2A5`, `0x0A4`, `0x3A5`, etc.) update cached fields without powering radio ON or overwriting `source_mode >= 0x20`. `0x265` owns `TA` (`0x80`) when present (`0x225` fallback only). CD (`0x325`/`0x365`/`0x3A5`) and CDC (`0x3A6`) are isolated by `source_mode`.
* **Single-Authority Multi-Frame Isolation:**
  - **HVAC:** `0x1E3` owns all climate fields (`power_on`, `ac_on`, `auto_mode`, `dual_mode`, `front_max_defrost`, `recirculate`, `aqs_auto`, setpoints, airflow, `fan_speed` in `data[6] & 0x0F`, `rear_defrost` in `B1 Bit 3`). `0x1D0` (DLC 7) updates **only** `rear_defrost` (`B0 Bit 6`, `0x40`).
  - **Keys vs Lights:** SWC/stalk keys come strictly from edge-triggered `0x21F`/`0x221`; `0x128` updates **only** `state->lights` (B4).
  - **Body:** Once `0x0F6` is seen, it owns `ignition_on` and `reverse_gear` (`0x036` fallback only). Handbrake comes strictly from `0x036` (never `0x220`).
* **Serial Dispatch Classes (38,400 baud):**
  - **Immediate (0 ms, on change):** Keys (`0x11`, `0x21`), Doors (`0x12`), Radar (`0x41`), Alerts (`0x42`, `0xEA`), Audio/DSP (`0x56`, `0x84`–`0x86`, `0x97`).
  - **Fast Telemetry ($\le 10\text{ Hz}$):** SAS (`0x11` in reverse), speed/RPM.
  - **State-Cached ($\le 2\text{ Hz}$ on change):** HVAC (`0x31`), Trip (`0x13`–`0x15`), TPMS (`0x66`).
  - **Heartbeat & Resync:** `0xFF` ping at $1\text{ Hz}$; full state resync on `0x24`/`0x30` handshake and every $60\text{ s}$.

---

## 3. Strict Engineering Boundaries

* **Pure C99 & Zero Heap:** `-std=c99 -Wall -Wextra -Werror`. No `malloc`/`calloc`/`free` or blocking loops.
* **Lock-Free SPSC:** Power-of-two `ring_buffer_t` only; no mutexes/RTOS primitives in core.
* **Safe Unpacking:** Verify `frame->dlc` before indexing `data[]`. Use `read_be16()`/`read_le16()` (PSA CAN is Big-Endian; guard against signed-byte casts and `0xFF`/`0xFFFF` sentinels).
* **CAN2004 Ground Truth Only:** Zero unverified placeholders (radar is `0x0E1`; `0x260` is BSI profiles, not radar).
* **Docs & Privacy:** No absolute paths (`/home/...`, `file:///...`) in `*.md`. Update `doc/CANBOX_SPEC_HIWORLD_407_*.md` on spec changes; update `doc/TODO_PROGRESS.md` **only** when `[ ]`/`[-]`/`[x]` status or command IDs change.
* **Lifecycle:** Ingest spec $\to$ clarify if blocked $\to$ implement Uplink (L1–L5) $\to$ add Unity tests (`test_407_*.c`) $\to$ update spec $\to$ ask user before implementing Downlink.