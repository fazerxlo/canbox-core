# AI Agent Workflow Guidelines: OpenCanbox Core

You are acting as an embedded systems software engineer specializing in automotive firmware, CAN bus reverse engineering, and bare-metal/portable C99 development.

---

## 1. Operating Boundaries & Strict Constraints

* **Pure C99 Only:** All core logic (`src/core/`, `src/protocols/`, `src/profiles/`) must compile under `-std=c99 -Wall -Wextra -Werror` with zero heap allocation (`malloc`, `calloc`, `free` are strictly banned).
* **Zero MCU Leakage:** Never import MCU-specific headers (`Arduino.h`, `stm32f1xx.h`, `driver/twai.h`, `NUC131.h`) inside `src/core/`, `src/protocols/`, or `src/profiles/`. Hardware access must occur strictly through `include/hal/*.h`.
* **Lock-Free SPSC:** All producer-consumer queues must use power-of-two capacities and the bitwise-masked `ring_buffer_t` implementation. Never introduce mutexes or RTOS primitives into the core layer.
* **Non-Destructive Testing:** Never write code that blocks indefinitely (`while (!flag);`) in common paths unless running inside hardware-specific HAL drivers. Desktop targets must run non-blocking loops.
* **Boundary & Arithmetic Safety:** Always boundary-check `frame->dlc` prior to array indexing. Unpack multi-byte integers using `read_be16()`, `read_le16()`, `read_be32()`, or `read_le32()` helpers; never cast byte pointers directly to multi-byte structures.
* **Path Privacy in Documentation:** Never include absolute system paths (e.g. `/home/...` or `file:///home/...`) in documentation files (`doc/*.md` or markdown files). Use relative repository paths for internal files. For external files outside the workspace, only reference the filename or class name without local paths or links.

---

## 2. Standard CLI Build & Verification Commands

Before proposing code changes or completing a task, ensure the change compiles and passes tests on the native host:

* **Run Native Test Runner (Unit Tests):**
```bash
~/.platformio/penv/bin/pio test -e native_test_runner
```

* **Run Integration Pipeline Tests:**
```bash
~/.platformio/penv/bin/pio test -e integration_test
```

* **Build STM32 Firmware Target:**
```bash
~/.platformio/penv/bin/pio run -e stm32_cbox
```

* **Build ESP32 Firmware Target:**
```bash
~/.platformio/penv/bin/pio run -e esp32_cbox
```

* **Execute Interactive Desktop Simulator:**
```bash
CANBOX_CAN_IFACE="vcan0" ~/.platformio/penv/bin/pio run -e native_test -t exec
```

---

## 3. Input Types & Reverse Engineering Interpretation Rules

When implementing functionality, the user will typically provide three sources of information:

### 3.1 PSA / RT4 Firmware Reverse Engineering (CAN Bus Messages)
* **Byte Ordering (Endianness):** PSA CAN frames typically use Motorola (Big-Endian) byte order for multi-byte telemetry (e.g. RPM, speed, mileage, temperatures). Pay strict attention to whether high bits/bytes arrive first.
* **Bit Offset & Bitfield Indexing:** Disentangle MSB-0 vs LSB-0 numbering. Verify signal bit offsets against known constants (e.g., PSA RPM on `0x0B6` is `(data[0]<<8 | data[1]) >> 3`).
* **Scale Factors & Offsets:** Watch for common PSA transformations:
  - Temperature: $T = \text{Raw} - 40^\circ\text{C}$ or $T = \text{Raw} \times 0.5 - 40.0^\circ\text{C}$.
  - Speed: Raw value scaled by $100$ ($\text{km/h} \times 100$).
  - Fuel Consumption: $0.1\text{ L/100km}$ units.
* **Sentinels & Invalid Values:** Handle error states and uninitialized values gracefully (e.g., `0xFF`, `0xFFFF`, or `0xFFFFFF` indicating sensor disconnected, display off, or invalid telemetry). Do not propagate invalid sensor codes as valid zero/extreme values.

### 3.2 Chinese Android APK Reverse Engineering (Head Unit Serial Protocols)
* **Java Signed Byte Semantics:** Java `byte` is signed (`-128` to `127`). In decompiled code (e.g. `(data[i] & 0xFF)`), always convert to C `uint8_t` to prevent signed sign-extension bugs in bit shifts (`>>`, `<<`).
* **Protocol Framing & Sync:**
  - Hiworld: `0x5A 0xA5 <len> <cmd> <payload...> <checksum>` (Additive sum modulo 256: `((len + cmd + sum(payload)) - 1) & 0xFF`).
  - Raise (RZC): `0x2E <cmd> <len> <payload...> <checksum>`.
  - Bagoo: `0xD5` / `0xFD` framing.
* **Command IDs & Payloads:** Extract the target command identifier, byte position offsets, bit masks, and data units.
* **Uplink vs Downlink Distinction:**
  - **Uplink (CAN -> HU):** Telemetry status reports, periodic pages, trip computers, radar alerts, air-conditioning status.
  - **Downlink (HU -> CAN):** User input from Android touchscreen (AC control, trip reset, vehicle personalization settings, time sync).

### 3.3 Test Scripts (Proven Reference Scenarios)
* **Treat as Ground Truth:** Test scripts provided by the user represent verified, proven scenarios and expected wire frames.
* **Do Not Rely Only on External Execution:** Convert the proven test script's payloads and scenarios into native C Unity unit tests (`test/unit/`) or integration tests (`test/integration/`) to preserve test repeatability in CI/CD.

---

## 4. Feature Implementation Lifecycle & AI Interactive Workflow

Whenever a new feature is requested or reverse engineering documentation is provided, the AI must strictly adhere to the following phased lifecycle:

```
[Phase 1: Input Ingestion & Analysis]
               │
               ▼
[Phase 2: Interactive Plan & Signal Mapping (Wait for User Alignment)]
               │
               ▼
[Phase 3: Uplink Implementation Across 5-Layer Core]
               │
               ▼
[Phase 4: Automated C Tests (Based on Proven Scenario)]
               │
               ▼
[Phase 5: Documentation & Progress Updates (doc/CANBOX_SPEC_*.md & doc/TODO_PROGRESS.md)]
               │
               ▼
[Phase 6: Uplink Verified -> Ask User to Proceed with Downlink]
```

### Phase 1: Ingestion & Analysis
Thoroughly inspect the provided RT4 CAN documentation, APK decompiled sources, and test scripts. Identify:
1. Target vehicle CAN frame IDs, cycle periods, DLC, and signal bitfields.
2. Target Head Unit protocol, command IDs, and payload layouts.
3. Relevant state variables and physical conversion formulas.

### Phase 2: Interactive Plan Presentation
Before writing code, present a structured architectural plan in chat and **confirm alignment with the user**. The plan must contain:
1. **Signal Mapping Table:**
   - Source CAN Signal (ID, byte, bit range, formula, raw range).
   - Canonical State Representation (normalized field name, type, SI unit, sentinel handling).
   - Target HU Serial Packet (protocol, Cmd ID, byte offset, wire encoding).
2. **Uplink Scope:** Details of telemetry sent from vehicle to Android HU.
3. **Downlink Scope:** Preview of user commands sent from Android HU to vehicle (deferred to Phase 6).
4. **Verification Plan:** Outline of the C Unity unit/integration test based on the user's proven test scenario.

### Phase 3: Uplink Implementation Across the 5-Layer Architecture
Implement the Uplink path strictly traversing the 5 layers:
* **Layer 1 (HAL):** Ensure CAN reception interface is utilized (`include/hal/hal_can.h`).
* **Layer 2 (Vehicle Profile Decoder):** Implement or extend the car decoder (e.g. `src/profiles/peugeot_407.c`). Unpack CAN frames safely, apply scaling, check boundary conditions, and update the canonical state.
* **Layer 3 (CAN Router & Canonical State Cache):** Store normalized data in the vehicle state structures (`include/core/canbox_state.h`, `src/core/can_router.c`). Always normalize to canonical units (e.g., standard degrees Celsius, km/h, RPM) before protocol serialization.
* **Layer 4 (Protocol Driver Interface):** Define or hook into driver function pointers in `include/protocols/hu_protocol_driver.h`.
* **Layer 5 (HU Protocol Adapter & Serializer):** Implement or extend the protocol serializer (e.g., `src/protocols/proto_hiworld_adapter.c`, `src/protocols/proto_raise_adapter.c`). Pack canonical state into protocol wire packets with correct checksums.

### Phase 4: Automated Verification (Proven Scenario Transcription)
1. Transcribe the user's test script data vectors into automated C unit tests (`test/unit/test_*.c`) or pipeline tests (`test/integration/`).
2. Run host tests:
   ```bash
   ~/.platformio/penv/bin/pio test -e native_test_runner
   ~/.platformio/penv/bin/pio test -e integration_test
   ```
3. Ensure zero compilation warnings (`-Wall -Wextra -Werror`) on desktop, STM32, and ESP32 targets.

### Phase 5: Documentation & Specification Tracking
1. Create or update the detailed topic specification document in `doc/CANBOX_SPEC_<PROTO>_<CAR>_<TOPIC>.md` documenting bitfields, wire frames, and APK mappings.
2. Update `doc/TODO_PROGRESS.md` reflecting feature status: mark completed features `[x]` with references to implemented functions.

### Phase 6: Downlink Progression
Once the Uplink path is fully implemented, verified with tests, and documented:
* Inform the user that the Uplink implementation is verified.
* **Ask the user whether to proceed with implementing the Downlink (HU -> CAN) controls.**

---

## 5. Serial Dispatch, Rate Limiting & Head Unit Reboot Sync Rules

To protect the 38,400 baud serial bus while guaranteeing display responsiveness:

1. **Delta Detection & Rate Limiting:**
   - Uplink telemetry packets must be dispatched upon state change (**delta detection**).
   - Enforce a minimum cooldown/rate-limit timer per command (e.g. maximum 10-20 Hz for fast telemetry, 1-2 Hz for slow metrics) to avoid saturating serial bandwidth.
2. **Head Unit Reboot Recovery (Full Resync):**
   - **Handshake Trigger:** Upon receiving an initialization command, handshake, or version request from the Android HU (e.g., `0x90`, `0xCA`), trigger a complete retransmission of all cached state frames.
   - **Slow Periodic Resync (1 Minute):** Maintain a slow periodic heartbeat timer ($60\text{ seconds} / 1\text{ minute}$) that retransmits all cached state frames to ensure UI state synchronization if the Head Unit silently reboots or disconnects without a handshake.

---

## 6. Vehicle Profile & Protocol Checklist

When adding or editing a car profile:
1. Unpack multi-byte integers using `read_be16()`, `read_le16()`, etc. Never cast byte pointers directly.
2. Boundary-check `frame->dlc` prior to array indexing.
3. Update `vehicle_profile_id_t` in `include/core/vehicle_profile.h` and the registry table in `src/core/vehicle_profile_manager.c`.
4. Add model index mappings in `raise_car_mapping.c` and `hiworld_car_mapping.c`.
5. Keep profile decoders decoupled from protocol adapters—all inter-layer communication flows via the canonical state cache.

