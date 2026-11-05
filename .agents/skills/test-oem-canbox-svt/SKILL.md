---
name: test-oem-canbox-svt
description: >-
  Deterministic preflight and execution of SVT suites on physical OEM CAN boxes
  (Raise, Hiworld, SimpleSoft) on PSA 407 hardware (`can0` + `/dev/ttyRealCanbox`).
---

# OEM CAN Box SVT Execution Guide

## 1. Hardware Topology & Zero-Flounder Rules
* **Interfaces:** `can0` = CH340 (`1A86:7523`) @ 125 kbps (manage strictly via `tools/slcan.sh`). `/dev/ttyRealCanbox` = FTDI (`0403:6001`) @ **38,400 8N1** (NEVER toggle to 19,200 baud).
* **17.5s Sleep Rule:** OEM boxes enter deep sleep after ~17.5s without CAN traffic or HU keepalives (`Cmd 0xA7` on Raise, `Cmd 0x24` on Hiworld). Once asleep on the bench, CAN frames cannot wake them. If UART RX is silent >5s, **stop immediately** and ask the user to power-cycle the CAN box.
* **Mandatory Flag:** Always pass `--persist-baseline` to `svt/run_svt.py` so BSI cyclic traffic and HU keepalives never drop between suites.

## 2. Execution Pipeline (Max 2 Commands)

1. **Pre-Flight Check (Mandatory First Step):**
   ```bash
   python3 tools/svt_oem_preflight.py --dut <dut_profile>

```

*If exit code is 1 (`[SLEEPING]` or `[BLOCKED]`), halt and prompt user to power-cycle hardware. Never write custom serial/USB probing scripts.*

2. **Run SVT Suite(s) (`WaitMsBeforeAsync=90000`):**
```bash
python3 svt/run_svt.py --dut <dut_profile> -s <suite_or_all> --persist-baseline

```


3. **Failure Triage:**
If any test fails, follow `.agents/skills/analyze-oem-svt-failures/SKILL.md` (`python3 tools/triage_svt_failures.py --latest`). Never inspect raw `can.log`, `uart.jsonl`, or `report.json` manually.

```

---

### 3. Two Small Additions to `AGENTS.md` (Section 1)
Your `AGENTS.md` is already well-structured for C99 firmware (`src/`, `include/`, `test/`)[cite: 2, 6], but the transcript shows the agent treated Python SVT files (`svt/`) as exempt from the C99 fast-path rules[cite: 2]. Add these two lines to **Section 1 of `AGENTS.md`**[cite: 2, 6]:

1. **Under Rule 1 (Parallel Batching & Hard 120-Line Read Cap)[cite: 2]:**
   ```markdown
   - For Python SVT suites (`svt/**/*.py`, `svt/duts/*.yaml`), batch all file locate/read operations into Turn 1 (3–5 parallel tool calls). Never read `svt/runs/**/report.json` or `uart.jsonl` directly; read `report.md` (lines 1–80) or run `python3 tools/triage_svt_failures.py --latest`.

```

2. Under Rule 2 (Turn 3 — Test Fast, Gate Once):


```markdown
- **Single Composite Verify Command:** Combine all post-edit checks (`py_compile`, `./tools/test_fast.sh`, `git status -s`, `git -C svt status -s`) into a single `&&`-chained command in 1 turn. Never run `git status`/`git diff` across multiple repositories in separate turns.
