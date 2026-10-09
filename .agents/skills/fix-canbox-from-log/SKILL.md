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

## Phase 1: Log Triage & Wire Diffing

Raw dump logs contain thousands of high-frequency frames. **Never read the entire log file directly into context**. Use the automated diagnostic tools first:

### 1.1 Run Automated Triage
```bash
./tools/triage_canbox_log.py <dump_file.log>
```
Inspect the generated triage report:
- **Chatter & Oscillation Detector:** Identifies alternating state flip-flops (e.g., radar blinking, intake switching between auto and fresh).
- **Silent vs Spurious Commands:** Pinpoints commands emitted by OpenCanbox that OEM never sends, or vice-versa.
- **Flooding:** Highlights commands transmitted at excessive rates (>5x OEM).
- **Wire Format Mismatches:** Detects payload byte length discrepancies.

### 1.2 Run Focused Differential Analysis
Once triage identifies the target command(s) or CAN frame(s), run differential comparison:
```bash
# By preset:
./tools/diff_canbox_frames.py <dump_file.log> --preset <hvac|radar|swc|doors|trip|tpms|alerts|bsi_config>

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
   - Two alternating CAN frames (e.g. `0x1D0` and `0x1E3`) update the same canonical state field in conflicting ways.
   - Example: `0x1E3` decodes logical ECU state (`aqs_auto = true`), but `0x1D0` decodes physical flap position (`0x20` forced fresh air) and overwrites `aqs_auto = false`.
   - Result: `can_router.c` detects a state change on *every* frame arrival, emitting rapid serial packets.
2. **Missing Feature / Unhandled CAN ID (Silent in App):**
   - OEM canbox produces a serial command, but OpenCanbox is silent because the triggering CAN frame is ignored or missing a rule in `s_<car>_rules[]`.
3. **Spurious Retransmission / Inverted Bit (Mismatched Payload):**
   - OpenCanbox serial packet differs from `ORIG_CANBOX` in specific bit offsets or scaling factors.

### 2.2 Pinpoint the Responsible Layer
- **Layer 2 (Vehicle Profile Decoder - `src/profiles/<car>.c`):** Incorrect bitmask, inverted boolean logic, unhandled mode condition, or missing boundary checks.
- **Layer 3 (CAN Router - `src/core/can_router.c`):** Faulty change detection (`memcmp`), missing field decoupling in `vehicle_state_t`, or improper event dispatch trigger.
- **Layer 5 (HU Protocol Adapter - `src/protocols/proto_<hu>_adapter.c`):** Wrong payload byte offset, inverted bit flag in serializer, or wrong checksum formula.

---

## Phase 3: Signal Mapping & Solution Proposal

Before implementing code changes, summarize the finding:
1. **Signal Mapping:**
   - Source CAN Frame ID, Byte offset, Bit range, and Physical meaning.
   - Canonical representation in `vehicle_state_t`.
   - Target Head Unit wire byte and bit encoding.
2. **Decoupling Strategy:**
   - If two frames represent different aspects of the same subsystem (e.g. user setting vs physical actuator state), decouple them so one does not clobber the other.
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
