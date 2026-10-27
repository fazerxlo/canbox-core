---
name: analyze-can-state-changes
description: >-
  Use this skill when you need to analyze raw CAN bus message logs (huge CSV, candump, or OpenCanbox dump files) to discover which CAN IDs, bytes, and bitfields change in response to RD4 tuner, BSI configuration, HVAC, stalk, or other vehicle status events, with minimal token usage.
---

# Analyze CAN State Changes & Transitions (Token-Optimized)

**NEVER read raw CAN message logs (.csv, candump, or dump_*.log) directly into the agent context.** Raw bus captures easily span 10,000 to 500,000+ lines. Reading them directly exhausts context windows and truncates analysis.

Use `./tools/analyze_can_state_changes.py` to stream-parse the capture, suppress millions of static heartbeat frames, and produce compact bit-level differential tables in <1,000 tokens.

---

## 1. Quick Discovery Command Recipes

### A. Full State Transition Overview (<500 tokens)
Extract all active CAN IDs that experienced payload changes, with their transition counts, state counts, and automatic behavior classification (`DISCRETE_TOGGLE`, `MUX_CAROUSEL`, `COUNTER_CLOCK`, `STREAM_BURST`):
```bash
./tools/analyze_can_state_changes.py <capture_file>
```
*Supports CSV (`Time Stamp,ID,...,D1..D8`), candump logs (`can0 123#...`), and OpenCanbox `dump_*.log`.*

### B. Pinpoint a Known User Action (Toggles / Button Cycles)
If the user toggled a button $N$ times (e.g. 6 toggles = 3 ON/OFF cycles of TA, ESP, Defrost, or A/C):
```bash
./tools/analyze_can_state_changes.py <capture_file> -t 6 -d
```
This filters immediately for CAN IDs that changed exactly 6 times and prints the chronological timeline with exact bit transitions (`bit4(0x10): 0 -> 1`).

### C. Filter Out High-Chatter Carousels & Clocks
To isolate real vehicle setting changes from continuous background carousels (BSI profile carousel `0x260`, Alert log carousel `0x120`, and BSI clock increments `0x3F6`):
```bash
./tools/analyze_can_state_changes.py <capture_file> -x 260 120 3F6 -d
```

### D. Focus on Specific Subsystems
When investigating a specific subsystem (e.g., RD4 Audio/Tuner):
```bash
./tools/analyze_can_state_changes.py <capture_file> -c 165 225 265 2A5 0A4 -d
```

---

## 2. Common Signal Classifications & Interpretations

| Classification | Meaning & PSA CAN2004 Examples | Agent Action |
| :--- | :--- | :--- |
| `DISCRETE_TOGGLE` | Feature toggles between 2 states (`0x265` TA/Stereo, `0x1D0` Defrost button, `0x221` Stalk trip button). | Check flipping bitmask (e.g., `B0:b4(0x10)`); verify against decoder in `src/profiles/`. |
| `MUX_CAROUSEL` | Multiplexed rotating pages (e.g. `0x260` pages `01`, `02`, `03` every 500 ms; `0x120` alert journal). | Normal behavior; pass `-x 260 120` to suppress if looking for discrete user actions. |
| `COUNTER_CLOCK` | Monotonically incrementing values (`0x3F6` BSI clock seconds/minutes, trip odometer `0x167`). | Expected background time/mileage progression; ignore unless debugging clock sync. |
| `STREAM_BURST` | Variable segmented payload streams (`0x0A4` scrolling RDS radiotext, `0x125` station/track lists). | Verify ISO-TP reassembly in `psa_407_rd4_audio.c`. |

---

## 3. Investigating Discrepancies (E.g. Tuner TA vs. Stereo)

When a Head Unit UI or Canbox does not reflect an action correctly:
1. Run `./tools/analyze_can_state_changes.py <file> -d` on the capture.
2. Locate the changing CAN ID (e.g. `0x265`) and inspect the diff:
   - Identify which bit toggles: `B0: 0xA4 -> 0xB4 [bit4(0x10): 0 -> 1]`.
   - Identify static bits in that byte: `0xA4` and `0xB4` both have bit 5 (`0x20`) set!
3. Compare against the profile decoder (`src/profiles/psa_407_rd4_audio.c`):
   - If decoder checked `data[0] & 0x20` instead of `0x10`, it mistakenly bound the feature to the wrong flag (e.g., Stereo/RDS indicator instead of TA priority).
4. Update the profile decoder and corresponding unit tests in `test/test_protocol_parser/`.

