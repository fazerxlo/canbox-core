# OpenCanbox Core

Portable, pure C99 firmware for custom CAN box adapters interfacing vehicle CAN networks with Chinese Android head units (Raise, Hiworld, Bagoo, Simple Soft).

Designed to compile without modification across multiple microcontroller architectures (**STM32**, **ESP32**, **Arduino/AVR**, **Nuvoton NUC131**) and run natively on **Linux** using SocketCAN and pseudo-terminals (`pty`) for fast desktop testing.

---

## Architecture Overview

```
+-------------------------------------------------------------+
|        Android Head Unit Protocol Engine (Pure C99)         |
|  - Dynamic protocol dispatch (Raise, Hiworld, Bagoo)       |
|  - Serial packet framer, parser, and checksum engines       |
+-------------------------------------------------------------+
                              |
+-------------------------------------------------------------+
|            Vehicle Profile & CAN Router Engine              |
|  - Vehicle matrix decoders (PSA, VAG, Toyota, etc.)         |
|  - Normalized vehicle state tracking & event delta filter   |
+-------------------------------------------------------------+
                              |
+-------------------------------------------------------------+
|             Hardware Abstraction Layer (HAL C API)          |
|  hal_can.h  |  hal_uart.h  |  hal_system.h  |  hal_gpio.h   |
+-------------------------------------------------------------+
        |                 |                 |              |
+---------------+ +---------------+ +---------------+ +---------------+
|  Linux Host   | |   STM32 LL    | |  ESP32 TWAI   | | Nuvoton C_CAN |
| (SocketCAN /  | | (bxCAN /      | | (ESP-IDF /    | | (NuMicro /    |
|    pty)       | |  USART)       | |  UART)        | |  UART)        |
+---------------+ +---------------+ +---------------+ +---------------+

```

---

## Key Features

* **Zero MCU lock-in**: Core translation and state-machine logic contain no platform-specific headers.
* **Dual-layer abstraction**:
* **Vehicle profiles**: Decouples raw manufacturer CAN matrices (IDs, masks, bitfields) from the router.
* **Head unit protocols**: Runtime switching between Raise (`0x2E`), Hiworld (`0x5A 0xA5`), and Bagoo (`0xFD`).


* **Desktop simulation first**: Develop and test end-to-end on Linux without hardware connected using `vcan0` and virtual serial ports.
* **Lock-free buffers**: Single-producer single-consumer (SPSC) ring buffers designed for safe ISR execution on microcontrollers without mutex overhead.

---

## Project Structure

```text
canbox-core/
├── platformio.ini               # Multi-target configuration (Native, STM32, ESP32, AVR)
├── Makefile.nuc131              # Standalone GCC build for Nuvoton NUC131
├── include/
│   ├── core/
│   │   ├── can_router.h         # Normalized state machine & router definitions
│   │   ├── ring_buffer.h        # Lock-free SPSC queue
│   │   └── vehicle_profile.h    # Vehicle matrix interfaces
│   ├── hal/
│   │   ├── hal_can.h            # CAN driver API
│   │   ├── hal_uart.h           # UART driver API
│   │   ├── hal_system.h         # Clocks, delays, reboots
│   │   └── hal_gpio.h           # LEDs, ignition detection
│   └── protocols/
│       ├── canbox_parser.h      # Generic streaming packet parser
│       ├── hu_protocol.h        # Outbound serializer interface
│       ├── hu_protocol_driver.h # Dynamic protocol vtable
│       ├── proto_raise.h        # Raise framing specs
│       └── proto_hiworld.h      # Hiworld framing specs
├── src/
│   ├── main.c                   # Hardware main execution loop
│   ├── core/                    # Hardware-agnostic engine
│   ├── profiles/                # Vehicle CAN decoders (PSA, VAG, etc.)
│   ├── protocols/               # Head unit serializers & parsers
│   └── hal/                     # Hardware driver implementations
│       ├── hal_native/          # Linux desktop drivers
│       ├── hal_stm32/           # STM32 LL bxCAN & USART drivers
│       ├── hal_esp32/           # ESP32 TWAI & UART drivers
│       ├── hal_avr/             # ATmega + MCP2515 drivers
│       └── hal_nuc131/          # Nuvoton NUC131 Bosch C_CAN drivers
└── test/
    ├── test_protocol_parser/    # Unity tests for parser & checksum verification
    └── test_integration/        # Unity tests for multi-driver CAN-to-UART pipeline & scenarios

```

---

## Getting Started: Linux Desktop Simulation

You can compile and run the firmware natively on a Linux development machine using VS Code and PlatformIO.

### 1. Prerequisites

Install CAN utilities and PlatformIO Core:

```bash
sudo apt update
sudo apt install can-utils build-essential
pip install platformio

```

### 2. Configure Virtual CAN & Run Native Target

Create a virtual CAN interface (`vcan0`):

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

```

Start the native simulation binary:

```bash
pio run -e native_test -t exec

```

The application outputs:

```text
[SYS] Linux monotonic clock initialized.
[CAN] Initialized on vcan0
[UART] Head unit UART simulated on: /dev/pts/3
[UART] Access port symlink available at: /tmp/ttyCanbox

```

### 3. Test Interactively

Open two additional terminals:

**Terminal 2 — Monitor Head Unit Serial Output:**

```bash
# Dump raw bytes received on the simulated Android HU UART port
xxd -c 16 < /tmp/ttyCanbox

```

**Terminal 3 — Inject Vehicle CAN Frames:**

```bash
# Inject PSA steering wheel Volume Up press (CAN ID 0x128)
cansend vcan0 128#0100

# Inject PSA door status frame (CAN ID 0x036, Driver door + Trunk open)
cansend vcan0 036#11000000

```

Terminal 2 will output the corresponding Raise binary frames:

* Volume Up: `2e01 0201 00fb`
* Door Status: `2e24 0141 99`

---

## Running Automated Tests

The test suite runs natively on Linux via the Unity framework:

```bash
# Run protocol parser and checksum unit tests
pio test -e native_test_runner

# Run end-to-end multi-driver CAN-to-UART integration and CAN log scenario tests
pio test -e integration_test
```

For instructions on writing and adding scenario tests using real vehicle CAN logs, see [Scenario-Based Integration Testing Guide](test/test_integration/README.md).

---

## Target Build Matrix

| Platform | Board / Environment | Hardware Controller | Command |
| --- | --- | --- | --- |
| **Linux Host** | `native_test` | SocketCAN (`vcan0`) + POSIX `pty` | `pio run -e native_test` |
| **STM32** | `stm32_cbox` | STM32F103 (bxCAN + USART1) | `pio run -e stm32_cbox` |
| **ESP32** | `esp32_cbox` | Built-in TWAI (`GPIO4/5`) + UART1 | `pio run -e esp32_cbox` |
| **Arduino AVR** | `avr_cbox` | ATmega328P + MCP2515 SPI | `pio run -e avr_cbox` |
| **Nuvoton NUC131** | Standalone Makefile | Bosch C_CAN + UART0 | `make -f Makefile.nuc131` |

---

## Adding a New Vehicle Profile

To add support for a new vehicle:

1. **Create profile driver**: Add `src/profiles/profile_<vehicle>.c`.
2. **Define message rules**: Map CAN arbitration IDs and bitfields to the normalized `vehicle_state_t` struct:
```c
static void decode_speed(const can_frame_t *frame, vehicle_state_t *state) {
    state->speed_kmh = (frame->data[2] << 8 | frame->data[3]) / 100;
}

static const profile_can_rule_t s_rules[] = {
    { 0x348, decode_speed }
};

```


3. **Register profile**: Add your profile entry to `include/core/vehicle_profile.h` and the registry table in `src/core/vehicle_profile_manager.c`.
4. **Set active profile**: Call `vehicle_profile_set_active(VEHICLE_PROFILE_<VEHICLE>)` during startup.
