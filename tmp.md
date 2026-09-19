To configure AI coding assistants (such as Claude Code, GitHub Copilot, Gemini Code Assist, or Cursor) to work effectively within this CAN box repository, place structured context and instruction files directly in your project root or workspace settings folder.

---

### Where to Put the Files

Depending on the tools used in VS Code:

| File / Folder Path | Primary Tool / Purpose |
| --- | --- |
| **`AGENTS.md`** (or `agent.md`) | Repository root. General workflow guidelines and strict behavioral rules for autonomous or semi-autonomous AI agents. |
| **`CONTEXT.md`** (or `context.md`) | Repository root. High-level project architecture, memory model, CAN/UART protocol specs, and coding rules. |
| **`.github/copilot-instructions.md`** | GitHub Copilot Workspace custom instructions. |
| **`.cursorrules`** | Cursor IDE workspace context and rule file. |
| **`CLAUDE.md`** | Claude Code CLI context file. |

The cleanest, tool-agnostic structure is keeping `AGENTS.md` and `CONTEXT.md` in the project root, then pointing tool-specific config files (like `.cursorrules` or `.github/copilot-instructions.md`) directly to them.

```text
canbox-core/
├── AGENTS.md                    <-- Role, constraints, commands, testing rules
├── CONTEXT.md                   <-- Hardware model, memory rules, protocols
├── .github/
│   └── copilot-instructions.md  <-- "@workspace see AGENTS.md and CONTEXT.md"
├── include/
├── src/
└── test/

```

---

### File 1: `AGENTS.md`

Place this in the repository root (`canbox-core/AGENTS.md`).

```markdown
# AI Agent Workflow Guidelines: OpenCanbox Core

You are acting as an embedded systems software engineer specializing in automotive firmware, CAN bus reverse engineering, and bare-metal/portable C99 development.

---

## 1. Operating Boundaries & Strict Constraints

* **Pure C99 Only:** All core logic (`src/core/`, `src/protocols/`, `src/profiles/`) must compile under `-std=c99 -Wall -Wextra -Werror` with zero heap allocation (`malloc`, `calloc`, `free` are strictly banned).
* **Zero MCU Leakage:** Never import MCU-specific headers (`Arduino.h`, `stm32f1xx.h`, `driver/twai.h`, `NUC131.h`) inside `src/core/`, `src/protocols/`, or `src/profiles/`. Hardware access must occur strictly through `include/hal/*.h`.
* **Lock-Free SPSC:** All producer-consumer queues must use power-of-two capacities and the bitwise-masked `ring_buffer_t` implementation. Never introduce mutexes or RTOS primitives into the core layer.
* **Non-Destructive Testing:** Never write code that blocks indefinitely (`while (!flag);`) in common paths unless running inside hardware-specific HAL drivers. Desktop targets must run non-blocking loops.

---

## 2. Standard CLI Build & Verification Commands

Before proposing code changes or completing a task, ensure the change compiles and passes tests on the native host:

* **Run Native Test Runner (Unit Tests):**
  ```bash
  pio test -e native_test_runner

```

* **Run Integration Pipeline Tests:**
```bash
pio test -e integration_test

```


* **Build STM32 Firmware Target:**
```bash
pio run -e stm32_cbox

```


* **Build ESP32 Firmware Target:**
```bash
pio run -e esp32_cbox

```


* **Execute Interactive Desktop Simulator:**
```bash
CANBOX_CAN_IFACE="vcan0" pio run -e native_test -t exec

```



---

## 3. Protocol Implementation Rules

When modifying or implementing a Head Unit protocol:

1. Every new protocol must implement the function pointers defined in `include/protocols/hu_protocol_driver.h`.
2. Checksums must be computed incrementally or via static inline helpers without temporary heap allocations.
3. Every packet decoder must handle corrupt framing bytes by falling back to the sync hunt state without losing subsequent valid packets.

---

## 4. Vehicle Profile Rules

When adding or editing a car profile:

1. Unpack multi-byte integers using `read_be16()` or `read_le16()` helpers. Never cast byte pointers directly to multi-byte structures.
2. Boundary-check `frame->dlc` prior to array indexing.
3. Update `vehicle_profile_id_t` in `include/core/vehicle_profile.h` and the registry table in `src/core/vehicle_profile_manager.c`.
4. Add model index mappings in `raise_car_mapping.c` and `hiworld_car_mapping.c`.

```

---

### File 2: `CONTEXT.md`

Place this in the repository root (`canbox-core/CONTEXT.md`).

```markdown
# Project Context: OpenCanbox Core

OpenCanbox Core is a portable C99 firmware project designed to translate automotive CAN bus frames (Comfort/Body CAN) into UART packets consumed by Chinese Android head units (Raise, Hiworld, Bagoo, Simple Soft).

---

## Architecture Topology


```

[Vehicle CAN Bus]
│ (125k / 250k / 500k)
▼
[hal_can (Hardware Driver)] ──► [ring_buffer_t (Lock-Free SPSC)]
│
▼
[can_router_process_can()]
│
▼
[vehicle_profile_manager]
(Matches CAN ID -> profile decoder)
│
▼
[Normalized vehicle_state_t]
(Doors, SWC, Climate, TPMS, Speed)
│
▼
[Delta / Periodic Filter]
│
▼
[hu_protocol_driver (vtable)]
(Active serializer: Raise / Hiworld)
│
▼
[hal_uart (Hardware Driver)] ──► [Android Head Unit]

```

---

## Hardware Abstraction Layer (HAL) Contracts

Hardware access is decoupled using clean C APIs defined in `include/hal/`:

* `hal_can.h`: Non-blocking frame push/pop (`hal_can_send`, `hal_can_receive`), filter initialization, and baud rate configuration.
* `hal_uart.h`: Non-blocking byte-stream read/write (`hal_uart_read_byte`, `hal_uart_write`). Standard Android HU CAN baud rate is **38400 baud, 8N1**.
* `hal_system.h`: Monotonic millisecond counter (`hal_get_tick_ms`), busy delay (`hal_delay_ms`), and soft reboot (`hal_system_reboot`).
* `hal_nvs.h`: Non-volatile storage abstraction for preserving active car selection across ignition cycles.

---

## Protocol Wire Specifications

### 1. Raise Protocol
* **Framing:** `[0x2E] [Cmd] [Len] [Payload(0..N)] [Checksum]`
* **Length:** Count of payload bytes ($N$).
* **Checksum:** Bitwise NOT of 8-bit sum:
  $$\text{Checksum} = (\sim(\text{Cmd} + \text{Len} + \sum \text{Payload})) \ \& \ \text{0xFF}$$
* **Key Commands:**
  * `0x01`: Steering Wheel Control (SWC) keys
  * `0x24`: Door and trunk contacts
  * `0x29`: Vehicle telemetry (Speed, RPM, Steering angle)
  * `0x38`: Extended 10-byte TPMS data
  * `0xCA`: Car model selection (from Android HU)

### 2. Hiworld Protocol
* **Framing:** `[0x5A] [0xA5] [Len] [Cmd] [Payload(0..N)] [Checksum]`
* **Length:** Command byte + Payload byte count ($N + 1$).
* **Checksum:** Truncated 8-bit sum:
  $$\text{Checksum} = (\text{Len} + \text{Cmd} + \sum \text{Payload}) \ \& \ \text{0xFF}$$
* **Key Commands:**
  * `0x11`: Steering Wheel Control keys
  * `0x21`: Door status
  * `0x24`: Car type selection (from Android HU)
  * `0x26`: Steering angle track

---

## Active Memory Model & Normalized State

The canonical vehicle state lives inside a single static structure (`vehicle_state_t` in `src/core/can_router.c`):

```c
typedef struct {
    vehicle_doors_t   doors;
    vehicle_wheel_t   wheel;
    vehicle_climate_t climate;
    vehicle_tpms_t    tpms;
    uint16_t          speed_kmh;
    uint16_t          rpm;
    int16_t           steering_angle_deg;
    bool              reverse_gear;
    bool              handbrake;
} vehicle_state_t;

```

* High-priority events (steering wheel keys, door openings) are transmitted immediately upon state delta detection.
* Low-priority telemetry (speed, RPM, TPMS) is refreshed on a 100 ms periodic timer or throttled to avoid saturating the 38400 baud UART link.

```

---

### Tool-Specific Symlinks / References

To ensure tools like GitHub Copilot or Cursor read these automatically:

* **For GitHub Copilot Workspace (`.github/copilot-instructions.md`):**
  ```markdown
  Read and strictly adhere to `AGENTS.md` and `CONTEXT.md` located in the workspace root. All code must remain pure C99 with zero heap allocation and zero microcontroller headers outside `src/hal/`.

```

* **For Cursor (`.cursorrules`):**
```markdown
You are an embedded C99 engineer. Always refer to CONTEXT.md for protocol specs and AGENTS.md for coding constraints and testing procedures. Run tests with `pio test -e native_test_runner`.

```