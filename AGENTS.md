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
.platformio/penv/bin/pio test -e native_test_runner

```

* **Run Integration Pipeline Tests:**
```bash
.platformio/penv/bin/pio test -e integration_test

```


* **Build STM32 Firmware Target:**
```bash
.platformio/penv/bin/pio run -e stm32_cbox

```


* **Build ESP32 Firmware Target:**
```bash
.platformio/penv/bin/pio run -e esp32_cbox

```


* **Execute Interactive Desktop Simulator:**
```bash
CANBOX_CAN_IFACE="vcan0" .platformio/penv/bin/pio run -e native_test -t exec

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

