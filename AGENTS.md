# AI Agent Workflow Guidelines: OpenCanbox Core

Embedded C99 automotive firmware & CAN bus bridge for **Peugeot 407 (PSA CAN2004 / AEE2004 Comfort Bus @ 125 kbps)** to **Hiworld Android Head Unit (`0x5A 0xA5`, 38,400 baud 8N1)**.

---

## 1. Fast-Path Token Efficiency Rules

1. **Silent Builds (`-s`):** Always pass `-s` to PlatformIO builds: `~/.platformio/penv/bin/pio run -s -e stm32_cbox`.
2. **Surgical Line-Range Reads & Batching:** Use `grep -n` first and read only the target function range (80–200 lines) in `view_file` rather than loading entire 700-line files. Batch independent `view_file`, `replace_file_content`, and command calls in parallel in a single turn.
3. **Capped Greps & Output Paging:** Always cap search outputs with line limits (`grep -n ... | head -n 20`, `-m 20`, or `-c`). Never run unconstrained broad greps over `.log` or build files that risk flooding context.
4. **Silent Daemons & No Status Polling:** Launch background processes (e.g., OpenOCD, loggers) quietly (`-d0` or redirecting stdout/stderr to a log file). Never poll `manage_task status` on high-throughput daemons or long-running verification commands (`verify_all.sh`); stop calling tools and await reactive system wakeup.
5. **Targeted Log Slicing:** When analyzing multi-megabyte captures, never view or cat raw files. Use `tools/triage_canbox_log.py`, `candump` timestamp ranges, or focused one-line Python parsers bounded to $\pm 1\text{ s}$ around the event.
6. **Scripted Mass Edits:** Use Python/shell one-liners via `run_command` when refactoring across 3+ files or pruning >30% of a file.
7. **Test Fast, Gate Once:** Iterate with `./tools/test_fast.sh` (or `./tools/test_fast.sh -e integration_test`). **NEVER execute `.pio/build/native_test/program` directly** (it is an infinite-loop SocketCAN daemon that hangs; unit tests live in `native_test_runner`). Run `./tools/verify_all.sh` **strictly once** at task completion. Never re-run C builds after editing `*.md` docs.
8. **Single-Call Log Triage:** Run `./tools/triage_canbox_log.py <dump.log>` once (it includes CAN state transitions, `[APP_UART_RX]` downlinks, `[APP_UART_TX]` uplinks, and causal timelines). Avoid writing ad-hoc Python log parsers unless inspecting an unlisted CAN ID. Note: `[APP_UART_TX]` in logs reflects the binary flashed at capture time, not necessarily current local source.
9. **ESP-IDF Registration:** Always register new `src/**/*.c` files in `src/CMakeLists.txt` in the same turn.
10. **Spec Implementation Fast Path:** Treat user specs (`doc/CANBOX_SPEC_*.md`) as pre-verified ground truth. Do not run exploratory greps or subagent audits on documented bitfields/formulas. Read headers/sources/tests in parallel (Turn 1), batch all code edits (Turn 2), and batch test + doc updates (Turn 3).
11. **Pre-Diagnosed Bug Fast Path:** When the user provides target files, functions, log evidence, or root causes, execute strictly via the 3-turn pipeline: Turn 1 (parallel surgical reads of target function + test), Turn 2 (batch edits across source, builders, tests, and specs in a single turn), Turn 3 (run `./tools/test_fast.sh` with `WaitMsBeforeAsync=10000`, then `./tools/verify_all.sh` once with zero status polling). Never run broad exploratory greps or archive scans on diagnosed bugs.

---

## 2. Architecture & Domain Map

| Layer | Files | Responsibility |
| :--- | :--- | :--- |
| **L1: HAL** | `include/hal/*.h`, `src/hal/` | Hardware abstraction (`hal_can`, `hal_uart`, `hal_gpio`, `hal_system`). Zero MCU headers outside `src/hal/`. |
| **L2: Profile** | `src/profiles/profile_psa.c`<br>`src/profiles/psa_407_*.c`<br>`src/profiles/psa_source_arbiter.c` | `s_psa_rules[]` routing table (`profile_psa.c`), CAN2004 decoders (`psa_407_rd4_audio.c`, `psa_407_hvac_body.c`, `psa_407_telemetry.c`, `psa_407_alerts.c`), Source Arbiter (`psa_source_arbiter.c`). |
| **L3: Router** | `include/core/can_router.h`<br>`src/core/can_router.c` | $O(1)$ 256-byte CAN ID bitmask filter, `vehicle_state_t` cache, `dirty.*` + `memcmp` delta dispatch, 60s periodic full resync. |
| **L4/L5: Proto** | `src/protocols/proto_hiworld*.c`<br>`src/protocols/hiworld_*.c` | Hiworld framing (`0x5A 0xA5 <len> <cmd> <payload...> <cs>`, `cs = ((len + cmd + sum) - 1) & 0xFF`), builders (`hiworld_psa_builders.c`), handshake (`hiworld_connection.c`). |

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
