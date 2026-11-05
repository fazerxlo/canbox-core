---
name: analyze-oem-svt-failures
description: >-
  Audit, diagnose, and fix SVT test run failures (report.md, can.log, uart.jsonl)
  on OEM CAN boxes (Raise, Hiworld, SimpleSoft, OpenCanbox) in <= 3 turns.
---

# Analyze & Fix OEM CAN Box SVT Failures (Strict 3-Turn Budget)

## 0. HARD EXECUTION GUARDRAILS (VIOLATION = CRITICAL FAILURE)

1. **ZERO SINGLE-TOOL READ TURNS:** You MUST batch 3–6 tool calls (`read_file`, `grep`, `run_command`) into a **single turn**. Sequential reads of adjacent line ranges (`1..100` then `101..200`) are strictly forbidden.
2. **USER RCA = GROUND TRUTH:** When the user states a CAN box/DUT handles a feature correctly or directs you to check test/decoder implementation, **DO NOT verify vehicle/DUT behavior**. Strictly forbidden: searching historical logs (`dump_*.log`), older SVT runs, `uart.jsonl`, or external repos (`../psa-can-lab`, `../QF_Canbus_system`). Jump straight to `svt/subsystem-*.py`, `svt/lib/protocols/*.py`, and `svt/duts/*.yaml`.
3. **NEVER PARSE `report.json` OR RAW LOGS:**
   - **NEVER** write `python3 -c` scripts to inspect `report.json` schemas. Always read `report.md` (lines 1–80) or run `tools/triage_svt_failures.py`.
   - **NEVER** read or grep `can.log` or `uart.jsonl` directly without `| head -n 30`.
4. **SINGLE COMPOSITE VERIFY:** After editing, run **one** chained shell command for all syntax, test, and git checks. Never run `py_compile`, `pytest`, `git diff`, or `git status` in separate turns.

---

## 1. Mandatory 3-Turn Fast-Path Workflow

### Case A: User Provides RCA / Points to Test or Protocol Implementation
* **Turn 1 (Parallel Locate & Read — Batch 3–5 Calls in ONE Turn):**
  Issue all required `read_file` calls (or 1 `grep -n` across targets + `read_file` on known files) **simultaneously**:
  - `svt/subsystem-<name>.py` (test assertions & stimulus injection)
  - `svt/lib/protocols/<protocol>.py` (UART frame decoder / key mapping)
  - `svt/duts/<dut>.yaml` (DUT capability flags / skips)
* **Turn 2 (Parallel Batch Edit):**
  Call `edit_file` on all target files in the **same turn**.
* **Turn 3 (Single Composite Verification — 1 Tool Call Only):**
  ```bash
  python3 -m py_compile svt/lib/protocols/*.py svt/subsystem-*.py && ./tools/test_fast.sh && git -C svt status -s && git status -s

```

*Stop immediately after Turn 3 and report results.*

### Case B: Uninvestigated SVT Run Failure

* **Turn 1 (1-Call Triage + Report Read):**
Run the deterministic triage script and inspect the markdown summary in a single turn:
```bash
python3 tools/triage_svt_failures.py --latest && head -n 60 svt/runs/latest/report.md

```


*(For a single test deep-dive: `python3 tools/triage_svt_failures.py --latest --test <TEST_ID>`)*
* **Turn 2 & Turn 3:** Follow **Case A (Turn 1 $\to$ Turn 2 $\to$ Turn 3)** to batch-read, batch-edit, and composite-verify.

---

## 2. Failure Classification & Remediation Matrix

| Category | Diagnostic Signature | Remediation Target |
| --- | --- | --- |
| **`INVALID_TEST_DESIGN`** | DUT awake & TXing valid UART frames, but test checks wrong cmd/field, pulse timing, or expects push without poll. | Fix `svt/subsystem-*.py` or `svt/lib/protocols/<proto>.py`. |
| **`INVALID_CAN_STIMULUS`** | Injected CAN frame clears PSA awake bit (`0x04` in `0x220`) or omits companion frame (`0x1D0` for `0x1E3`). | Fix CAN payload in `svt/subsystem-*.py`. |
| **`DUT_UNSUPPORTED_FEAT`** | Valid CAN stimulus, DUT awake, feature absent in OEM firmware for 407 (e.g. RD4 nav keys, CD changer). | Add `SKIP`/`XFAIL` or capability flag in `svt/duts/<dut>.yaml`. |
| **`DUT_SLEEP_OR_SILENCE`** | UART RX silent > 3.0s; keepalive `0xA7` missing or >17.5s inter-suite bus silence. | Prompt user to power-cycle +12V; run with `--persist-baseline`. |
| **`DUT_REAL_FAILURE`** | Valid CAN + awake DUT + supported feature, but UART payload violates protocol spec. | Fix decoder or document in `doc/SVT_OEM_COMPARISON_AND_COMPATIBILITY.md`. |

---

## 3. Peugeot 407 OEM Protocol Traps (Ground Truth Reference)

* **Climate (`0x1E3` / `0x1D0`):**
* **Dual Mode:** 407 has native independent L/R temps. Raise outputs `temp_driver` & `temp_passenger` (`2E 21 09 A8 03 <T_drv> <T_pass>`) but leaves synthetic `dual_mode` bit (`B0 Bit 2`) = `0`. Never assert `dual_mode is True` on Raise.
* **Outdoor Temp:** Raise sends outdoor temp in **`Cmd 0x36`**, not inside `Cmd 0x21`/`0x31`.
* **Temp Formula:** Hiworld: $T = \text{raw} / 2.0$. Raise: $T = (\text{raw} / 2.0) - 3.0$.
* **Rear Defrost:** Hiworld: `0x31` B1 Bit 3 (`0x08`). Raise: `0x21` B0 Bit 0 (`0x01`). Requires BSI companion `0x1D0` (`B0 Bit 6 = 0x40`).


* **Doors (`0x220`):**
* **Bus Awake Bit (`0x04`):** Nominal closed is `0x220#0400`. Every door injection MUST retain `| 0x04` (`84 00`, `44 00`, `24 00`, `14 00`, `0C 00`, `A4 00`). Sending `80 00` puts the CAN box to sleep.
* **Polling:** Raise requires HU query `2E 8F 01 38 37` to emit `Cmd 0x38` door status; Hiworld pushes `Cmd 0x12` on change.


* **Keys (`0x21F` SWC, `0x221` Stalk, `0x3E5` Console):**
* **RD4 Console (`0x3E5`):** OEM boxes ignore navigation keys (`MENU`, `OK`, `ESC`, Arrows $\to$ `is_nav_key=True`, `supports_rd4_nav_keys: false`), but **decode functional shortcuts**: `DARK` (`0x54` $\to$ `0x07`), `AUDIO` (`0x09` $\to$ `0x31`), `CLIMA` (`0x2F` $\to$ `0x28`), `TRIP` (`0x20` $\to$ `0x40`), `TEL` (`0x29` $\to$ `0x05`).
* **Stalk Tips (`0x221`):** Left tip (`B0 Bit 0 = 0x01`) = PTT/Voice (`0x23` SimpleSoft, `0x40` Hiworld). Right tip (`B0 Bit 3 = 0x08`) = Trip toggle; SimpleSoft emits Virtual EMF CAN frame **`0x167#8200FFFF...`** (`data[0] & 0x80`) instead of a UART key. Tests must check both UART and CAN `0x167`.
* **Pulse-Release (`legacy_keys: true`):** SimpleSoft emits a momentary non-zero key pulse followed immediately by `0x00`. Assert any non-zero key within the capture window, not steady-state hold.


* **Trip & Telemetry (`0x221`, `0x261`, `0x2A1`, `0x0B6`):**
* **Sentinels:** Stationary/uncalculated instantaneous consumption & range emit `0xFFFF` (`--.-`).
* **Powertrain (`0x1A`):** Supported on Hiworld; unsupported on Raise (`powertrain_telemetry_supported: false` $\to$ skip `TRIP-08`).


