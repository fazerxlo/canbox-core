---
name: fast-path-bugfix
description: >-
  Use this skill when the user provides a bug report with specific details (target file, function, CAN/UART payload, or root cause explanation), or explicitly requests minimal-token bug resolution.
---

# Fast-Path Bug Resolution

Solve pre-diagnosed and detailed bug reports with minimum token consumption and strict turn bounding (3 execution turns max).

## Trigger Criteria
Activate this workflow whenever the prompt includes:
- Specific source file (`src/**/*.c`) or function name
- Known CAN ID, log payload bytes, or UART wire frame evidence
- Exact root cause explanation or proposed fix
- Explicit low-token / fast-path resolution requests

---

## The 3-Turn Fast Pipeline

### Turn 1: Surgical Scope & Parallel Read
1. **Resumed Session Fast-Track:** If resuming from a compacted session summary or detailed user prompt where Root Cause Analysis (RCA) and Next Steps are already identified, skip all exploratory investigation. Proceed immediately to reading the target function line ranges.
2. **Zero Exploratory Grepping & External Repo Prohibition:**
   - Never run broad greps (`grep -rn`) across `doc/archive/`, historical git logs, or raw dump files.
   - Never run exploratory greps or file searches across external decompiled codebases (e.g. `QF_Canbus_system/`, `hiworld-canbus-PSA/`, `smali/`) when `doc/CANBOX_SPEC_*.md` or the user spec already documents the command ID, byte offsets, or UI values. Treat `CANBOX_SPEC_*.md` as verified ground truth.
3. **Locate Target Functions:** Run `./tools/code_map.py <symbol|CAN_ID|Cmd>` to get exact `[StartLine-EndLine]` spans across source, headers, unit/integration tests, and specs in one call.
4. **Parallel Range Reads:** Batch `read_file` calls in the same turn for only the target function line ranges ($\le 120$ lines).

### Turn 2: Atomic Batch Edits
Apply all necessary modifications in parallel in a single turn using `replace_file_content`:
1. **L2 Profile / Decoder:** Fix CAN unpacking, sentinel checks, or byte indexing in `src/profiles/`.
2. **L4/L5 Builders / Protocol Adapters:** Fix wire length, payload offsets, padding, or checksums in `src/protocols/` if wire framing is impacted.
3. **Unit & Integration Tests:** Inject real log payload vectors into `test/` to reproduce the fix and assert expected outputs.
4. **Specification Docs:** Update corresponding `doc/CANBOX_SPEC_*.md` tables and packet diagrams.

### Turn 3: Single-Gate Verification & Zero Polling
1. **Synchronous Fast Test:** Run `./tools/test_fast.sh` (pass `WaitMsBeforeAsync: 10000`) so native runner tests finish synchronously.
2. **Single Gate Run:** Run `./tools/verify_all.sh` strictly once at completion.
3. **Zero Polling Rule:** When `./tools/verify_all.sh` runs as a background task, **DO NOT loop or poll `manage_task status`**. Stop calling tools and let the reactive system wakeup resume execution upon completion.

