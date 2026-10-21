---
name: implement-from-spec
description: >-
  Use this skill when implementing code changes based on generated and verified documentation, specifications (e.g. doc/CANBOX_SPEC_*.md), or design notes, with minimal token usage and no re-verification of documented details.
---

# Fast-Path Implementation from Verified Specification

Implement features, decoders, and protocol builders directly from pre-verified documentation with minimum token consumption and strict turn bounding (3 execution turns max).

## Trigger Criteria
Activate this workflow whenever:
- User points to a specification document (`doc/CANBOX_SPEC_*.md` or design doc) to implement in code.
- Documentation has already been generated or verified, and code changes are needed.
- Explicit low-token / fast-path implementation from spec is requested without re-auditing.

---

## Ground Truth & Zero-Verification Axiom

1. **Spec is Verified Ground Truth:** Treat bitfields, CAN IDs, byte layouts, scaling formulas, sentinels, and enum values in the document as pre-verified ground truth.
2. **Zero Exploratory Research:** Do NOT audit, second-guess, or cross-check documented details against raw CAN dumps, git history, or web search.
3. **No Subagent Audits:** Do NOT launch subagents or run multi-turn audits to re-validate documented logic.
4. **Targeted Symbol Lookup:** Never run unconstrained broad greps. Locate symbols with targeted `grep -n` capped with `-m 10` or direct paths.

---

## The 3-Turn Fast Pipeline

### Turn 1: Surgical Scope & Parallel Read
1. **Spec Scope:** Read only the relevant sections of the target document if not already in context.
2. **Locate Code Touchpoints:** Identify the specific files across the architecture layers:
   - State definitions: `include/core/vehicle_state.h`
   - L2 Decoder: `src/profiles/psa_407_*.c` and routing table in `src/profiles/profile_psa.c`
   - L4/L5 Wire Builder: `include/protocols/hiworld_psa_builders.h` and `src/protocols/hiworld_psa_builders.c`
   - Core Router: `src/core/can_router.c` (dirty flags / dispatch)
   - Unit tests: `test/test_protocol_parser/test_407_*.c`
3. **Parallel Range Reads:** Batch `read_file` calls in a single turn for only the target functions/structs using bounded line ranges (80–150 lines).

### Turn 2: Atomic Batch Code & Test Edits
Apply all implementation changes in parallel in a single turn using `edit_file` / `write_file`:
1. **State & Structs:** Update `vehicle_state_t` fields in `include/core/vehicle_state.h` as specified.
2. **L2 Decoder:** Implement CAN unpacking in `src/profiles/psa_407_*.c`:
   - Enforce bounds check: `if (frame->dlc < EXPECTED_DLC) return false;`
   - Use `read_be16()` / `read_le16()` for multi-byte values (PSA CAN is Big-Endian).
   - Register CAN ID in `s_psa_rules[]` in `src/profiles/profile_psa.c` if adding a new frame.
3. **L4/L5 Protocol Builder:** Implement Hiworld wire frame builder in `src/protocols/hiworld_psa_builders.c`:
   - Follow Hiworld packet format (`0x5A 0xA5 <len> <cmd> <payload...> <cs>`).
   - Use standard checksum: `((len + cmd + sum) - 1) & 0xFF`.
4. **Router & CMake:**
   - Update `dirty.*` delta checks in `src/core/can_router.c` if new state triggers dispatch.
   - If a new `.c` file is created, register it in `src/CMakeLists.txt` in the same turn.
5. **Unit Tests:** Add unit tests in `test/test_protocol_parser/test_407_*.c` asserting inputs/outputs against the test vectors documented in the spec.
6. **Tracker Update:** Update `doc/TODO_PROGRESS.md` if feature checkbox status changes.

### Turn 3: Single-Gate Verification & Zero Polling
1. **Synchronous Fast Test:** Run `./tools/test_fast.sh` (pass `WaitMsBeforeAsync: 10000`) to execute native tests synchronously.
2. **Single Gate Run:** Run `./tools/verify_all.sh` strictly once at completion.
3. **Zero Polling Rule:** If `./tools/verify_all.sh` is backgrounded, **DO NOT loop or poll `manage_task status`**. Stop calling tools and let reactive system wakeup resume execution upon completion.

