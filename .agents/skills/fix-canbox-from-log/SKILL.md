---
name: fix-canbox-from-log
description: >-
  Use this skill when the user provides an E2E dump log (`dump_*.log`) or asks to fix, resolve, debug, or align an application bug or protocol issue based on vehicle or bench logs.
---

# Fix OpenCanbox from E2E Dump Logs

This skill guides the end-to-end resolution of OpenCanbox translation bugs, signal jitter, state flip-flops, and protocol mismatches identified in E2E dump logs (`dump_*.log`) recorded from real vehicles or bench setups.

Follow the 6-phase engineering lifecycle strictly to guarantee root-cause elimination, regression prevention, and embedded firmware compliance.

---

## Workflow Overview

```
[Phase 1: Log Triage & Wire Diffing]
               │
               ▼
[Phase 2: Multi-Layer Protocol & Conflict Diagnosis]
               │
               ▼
[Phase 3: Signal Mapping & Solution Proposal]
               │
               ▼
[Phase 4: Pure C99 Implementation & Decoupling]
               │
               ▼
[Phase 5: Automated Verification (Unity & Target Builds)]
               │
               ▼
[Phase 6: Documentation & Specification Tracking]
```

---

## Fast-Path Execution & Token Efficiency Rules (CRITICAL)

To resolve log-based issues in minimal steps (~15–25 tool calls instead of 100+), strictly follow these fast-path rules:

1. **User-Specified Rule Short-Circuit:**
   - When the user provides a log **and** explicitly states the required behavior (e.g., *"When RD4 radio is off, don't send any CD/Tuner frames"*), **accept the requirement immediately**.
   - Only use the log to extract the raw `[CAN_RX]` byte vectors (e.g., `0x165`, `0x225`, `0x3A5`) needed for your regression test. Do **NOT** spend tool calls reverse-engineering unrelated frames in the log.
2. **`[APP_UART_TX]` in Hardware Logs ≠ Current Local Source Tree:**
   - In bench or vehicle captures, `[APP_UART_TX]` is emitted by the *firmware binary flashed on the physical MCU at capture time*, which may be an older build or test firmware.
   - Never waste steps searching the local C codebase for phantom logic trying to explain why `[APP_UART_TX]` emitted a specific byte in the log if a quick check of the local decoder/router already shows how current code behaves.
3. **Large-Window File Reads (No Micro-Slicing):**
   - `view_file` supports up to **800 lines per call**. Always read 300–800 lines at once when inspecting `can_router.c`, `peugeot_407.c`, `profile_psa.c`, or test files. Never chain 5–10 consecutive 30-line `view_file` calls on the same file.
4. **Batch Parallel Tool Calls:**
   - Whenever you need to inspect multiple files (e.g. `profile_psa.c`, `peugeot_407.c`, and `can_router.c`) or run multiple searches, invoke the tool calls **in parallel within a single turn**.
5. **Direct PSA Architecture Map (Skip Exploratory Grepping):**
   - **Rule Table (`src/profiles/profile_psa.c`):** Maps CAN IDs (`s_psa_rules[]`) to subsystem decoders (e.g., all 12 RD4 CAN IDs route directly to `psa_rd4_decode_frame()`).
   - **Subsystem Decoders & Wire Builders (`src/profiles/peugeot_407.c`):** Implements `psa_rd4_decode_frame()`, detailed decoders (`psa_rd4_process_can_*`, `psa_hvac_process_can_*`), and Hiworld payload builders (`build_hiworld_*`). CAN `0x165` is the sole authority over `state->media.radio` (`present`, `sleep`, `power_status`, `source_mode`).
   - **Delta Dispatch & 60s Resync (`src/core/can_router.c`):** Dispatches frames in `can_router_process_can()` using state-machine rules (e.g., `source_mode < 0x20` for Tuner vs `>= 0x20` for `AUX`/`CD`/`CDC`/`OFF`) and runs the 60-second slow periodic resync in `can_router_periodic_100ms()`.
6. **State-Machine Authority Pattern (No Ad-Hoc Flag Patching):**
   - Authoritative mode frames (e.g., `0x165` for RD4 `source_mode` and `power_status`, `0x1E3` for HVAC logical AQS mode) own the state transition.
   - Background sub-frames (`0x225`, `0x2A5`, `0x0A4`, `0x3A5`) update cached fields without turning a subsystem ON or overwriting `source_mode == 0xFF` (`OFF`) or `>= 0x20` (`AUX`/`CD`/`CDC`).

---

## Phase 1: Log Triage & Wire Diffing

Raw dump logs contain thousands of high-frequency frames. **Never read the entire log file directly into context**. Use the automated diagnostic tools first:

### 1.1 Run Automated Triage
```bash
./tools/triage_canbox_log.py <dump_file.log>
```
Inspect the generated triage report:
- **Chatter & Oscillation Detector:** Identifies alternating state flip-flops (e.g., radar blinking, intake switching between auto and fresh, radio ON/OFF cycling).
- **Silent vs Spurious Commands:** Pinpoints commands emitted by OpenCanbox that OEM never sends, or vice-versa.
- **Flooding:** Highlights commands transmitted at excessive rates (>5x OEM).
- **Wire Format Mismatches:** Detects payload byte length discrepancies.

### 1.2 Run Focused Differential Analysis
Once triage identifies the target command(s) or CAN frame(s), run differential comparison (skip if the user already specified the exact target command/behavior and triage already showed the offending frames):
```bash
# By preset:
./tools/diff_canbox_frames.py <dump_file.log> --preset <hvac|radar|radio|swc|doors|trip|tpms|alerts|bsi_config>

# Or by explicit Command / CAN IDs:
./tools/diff_canbox_frames.py <dump_file.log> -c <CAN_IDS> -p <CMD_HEX>
```

### 1.3 Correlate with User Scenario Notes
When the user specifies actions taken in the car (e.g., *"start from Auto, changed to internal circulation, changed to unfrost front"*):
- Locate each corresponding `[ORIG_CANBOX]` wire frame in the diff output.
- Map each user click to the exact byte values emitted by the real OEM canbox. These bytes are the **proven ground truth**.

---

## Phase 2: Multi-Layer Protocol & Conflict Diagnosis

Trace the exact sequence of events leading to the discrepancy across the OpenCanbox 5-layer architecture:

### 2.1 Identify the Mechanism of Failure
1. **Decoder Clash / State Overwrite (Oscillation):**
   - Two alternating CAN frames (e.g. `0x1D0` and `0x1E3`, or `0x165` and `0x225`/`0x3A5`) update the same canonical state field in conflicting ways.
   - Example: `0x1E3` decodes logical ECU state (`aqs_auto = true`), but `0x1D0` decodes physical flap position (`0x20` forced fresh air) and overwrites `aqs_auto = false`.
   - Result: `can_router.c` detects a state change on *every* frame arrival, emitting rapid serial packets.
2. **Background CAN Traffic When Subsystem is Powered Off or in Another Source:**
   - Idle ECUs (e.g. RD4 tuner `0x225` or CD drive `0x3A5` when audio power `0x165` bit 7 is `0` or in `AUX`) continue broadcasting background status on the comfort bus.
   - Fix: Ensure sub-frame decoders only cache data without overriding authoritative `source_mode` / `power_status`, and gate router dispatch on active `source_mode`.
3. **Missing Feature / Unhandled CAN ID (Silent in App):**
   - OEM canbox produces a serial command, but OpenCanbox is silent because the triggering CAN frame is ignored or missing a rule in `s_<car>_rules[]`.
4. **Spurious Retransmission / Inverted Bit (Mismatched Payload):**
   - OpenCanbox serial packet differs from `ORIG_CANBOX` in specific bit offsets or scaling factors.

### 2.2 Pinpoint the Responsible Layer
- **Layer 2 (Vehicle Profile Decoder - `src/profiles/profile_psa.c` & `src/profiles/peugeot_407.c`):** Incorrect bitmask, mismatch between `profile_psa.c` wrapper and `peugeot_407.c` processor, unhandled power/mode guard, or missing boundary checks.
- **Layer 3 (CAN Router - `src/core/can_router.c`):** Faulty change detection (`memcmp`), missing field decoupling in `vehicle_state_t`, or unguarded event/periodic resync dispatch.
- **Layer 5 (HU Protocol Adapter - `src/protocols/proto_<hu>_adapter.c`):** Wrong payload byte offset, inverted bit flag in serializer, or wrong checksum formula.

---

## Phase 3: Signal Mapping & Solution Proposal

Before implementing code changes, keep internal analysis brief:
1. **Signal Mapping:**
   - Source CAN Frame ID, Byte offset, Bit range, and Physical meaning.
   - Canonical representation in `vehicle_state_t`.
   - Target Head Unit wire byte and bit encoding.
2. **Decoupling Strategy:**
   - If two frames represent different aspects of the same subsystem (e.g. user setting vs physical actuator state, or power-off state vs background idle telemetry), decouple or gate them so one does not clobber the other.
   - For front defrost / demist modes, ensure physical flap overrides do not erase the driver's background intake/auto preference.

---

## Phase 4: Pure C99 Implementation & Decoupling

Apply the fix while adhering strictly to OpenCanbox constraints (`AGENTS.md`):

1. **Pure C99 & Zero Allocations:**
   - Strict `-std=c99 -Wall -Wextra -Werror`.
   - Never use `malloc`, `calloc`, or `free`. Use static or stack buffers with explicit bounds.
2. **Zero MCU Leakage:**
   - Never include hardware-specific headers (`Arduino.h`, `stm32f1xx.h`, `driver/twai.h`) in `src/profiles/`, `src/core/`, or `src/protocols/`.
3. **Boundary Safety:**
   - Always verify `dlc` prior to array indexing (e.g. `if (dlc < 7) return;`).
   - Use safe endian unpackers (`read_be16()`, `read_le16()`, etc.).
4. **State Decoupling & Lockstep Alignment:**
   - Align decoders so receiving repeated frames in any sequence leaves canonical state stable with zero flip-flop jitter.

---

## Phase 5: Automated Verification (Unity & Target Builds)

Always transcribe the real vehicle vectors into regression tests:

### 5.1 Transcribe Test Vectors to C Unity Test
Add or extend unit tests in `test/test_protocol_parser/test_<profile>.c` (or integration tests in `test/test_integration/`):
- Feed the exact raw CAN byte arrays from the dump log into the decoder.
- Assert that the decoded fields match the ground truth.
- Feed the alternating frames back-to-back in loop iterations to verify that state remains stable and no spurious change events are fired.

### 5.2 Run Native Host Test Suite
```bash
~/.platformio/penv/bin/pio test -e native_test_runner
~/.platformio/penv/bin/pio test -e integration_test
```
Ensure all tests pass.

### 5.3 Verify Embedded Target Compilation
Verify that no MCU target breaks and that compilation generates zero warnings:
```bash
~/.platformio/penv/bin/pio run -e stm32_cbox
~/.platformio/penv/bin/pio run -e esp32_cbox
```

---

## Phase 6: Documentation & Progress Tracking

1. **Update Issue Tracker:**
   - Mark the resolved item `[x]` in `Issues.md` with root cause, affected files, and test vector references.
2. **Update Progress Tracker:**
   - Update `doc/TODO_PROGRESS.md` reflecting feature status.
3. **Update Topic Specification Docs:**
   - Update relevant topic docs in `doc/CANBOX_SPEC_*.md` documenting bitfields, wire frames, and customizations.
4. **Enforce Path Privacy:**
   - **Never** include absolute local paths (e.g., `/home/...` or `file:///home/...`) in documentation files (`doc/*.md` or `*.md`). Use repository-relative paths only.
