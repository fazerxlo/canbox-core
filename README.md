# OpenCanbox Core

**Portable, deterministic C99 firmware for CAN-to-Android automotive protocol translation and reverse engineering.**

![C99](https://img.shields.io/badge/C99-Strict-blue)
![PlatformIO](https://img.shields.io/badge/PlatformIO-Multi--Platform-orange)
![Linux](https://img.shields.io/badge/Linux-SocketCAN-green)
![STM32](https://img.shields.io/badge/STM32F103-Supported-red)
![ESP32](https://img.shields.io/badge/ESP32--TWAI-Supported-red)
![Nuvoton](https://img.shields.io/badge/NUC131-Supported-red)
![License](https://img.shields.io/badge/License-MIT%2FApache%202.0-green)

---

## Quick Start

**Desktop Testing (Linux + SocketCAN):**
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
pio test -e native_test_runner
```

**STM32F103 Hardware:**
```bash
pio run -e stm32_cbox
```

**ESP32 Hardware:**
```bash
pio run -e esp32_cbox
```

**Nuvoton NUC131 Hardware:**
```bash
pio run -e nuc131_cbox
```

---

## Supported Targets

| Platform | Environment | Controller | HAL | Status |
| :--- | :--- | :--- | :--- | :--- |
| **Linux** | `native_test` | SocketCAN + POSIX pty | `hal_native` | ✅ Tested |
| **STM32F103** | `stm32_cbox` | bxCAN + USART1 + GPIO | `hal_stm32` | ✅ Tested |
| **ESP32** | `esp32_cbox` | TWAI + UART1 + GPIO | `hal_esp32` | ✅ Tested |
| **Nuvoton NUC131** | `nuc131_cbox` | Bosch C_CAN + UART0 + GPIO | `hal_nuc131` | ✅ Tested |

---

## Architecture Overview

### 5-Layer Processing Pipeline

All features traverse a strictly layered architecture, ensuring protocol and vehicle independence:

```
Layer 1: CAN Hardware RX  →  Layer 2: Vehicle Decoder  →  Layer 3: CAN Router
         Layer 4: Protocol Driver  →  Layer 5: Head Unit Adapter  →  UART TX
```

**Portable Core:** Vehicle profiles, protocol drivers, and CAN routing logic remain platform-agnostic.

**Hardware Abstraction:** Vendor-specific clock, CAN, UART, and GPIO are isolated under `src/hal/hal_*/`.

---

## Build Prerequisites

```bash
sudo apt update
sudo apt install build-essential can-utils picocom
pip install platformio
```

### For Nuvoton NUC131 builds:

Clone the custom PlatformIO platform repository:

```bash
git clone https://github.com/fazerxlo/nuvoton_nuc131.git
```

Then place it under your PlatformIO platforms directory:

```bash
~/.platformio/platforms/
```

Or use a local path in `platformio.ini`:

```ini
platform = path/to/nuvoton_nuc131
```

---

## Building

### Unit & Integration Tests (Linux/Native)

```bash
pio test -e native_test_runner
pio test -e integration_test
```

### Hardware Firmware

**STM32F103:**
```bash
pio run -e stm32_cbox
```

**ESP32:**
```bash
pio run -e esp32_cbox
```

**Nuvoton NUC131:**
```bash
pio run -e nuc131_cbox
```

---

## Nuvoton NUC131 Support

### Hardware Mapping

The NUC131 implementation uses the validated pin mapping from the reference firmware (`fazerxlo/canbox/vw_nc03/fw`):

| Function | Pin |
| :--- | :--- |
| CAN0 RX | PD.6 |
| CAN0 TX | PD.7 |
| UART0 RX | PB.0 |
| UART0 TX | PB.1 |
| GPIO LED Status | PA.9 |
| GPIO CAN Standby | PA.12 |
| GPIO Head Unit Power | PA.8 |
| GPIO Reverse Output | PA.13 |
| GPIO Ignition Input | PA.0 |

### Configuration

```ini
[env:nuc131_cbox]
platform = nuvoton
board = nuvoton_nuc131
test_ignore = *
build_flags = 
    -Iinclude
    -Isrc
    -DPLATFORM_NUC131
    -Wall -Wextra -std=c99
build_src_filter = 
    +<main.c>
    +<core/*>
    +<profiles/*>
    +<protocols/*>
    +<hal/hal_nuc131/*>
```

### Flashing

The custom Nuvoton platform requires a custom uploader package (`tool-nuvoton-isp`) or an equivalent OpenOCD-based upload flow. See `fazerxlo/nuvoton_nuc131` for detailed flash instructions.

### Documentation

Full build and flash notes: [`doc/NUC131_SUPPORT.md`](doc/NUC131_SUPPORT.md)

---

## Project Structure

```
canbox-core/
├── README.md                            # This file
├── platformio.ini                       # Multi-target build configuration
├── development_guidelines.md            # Engineering standards
│
├── doc/
│   ├── NUC131_SUPPORT.md               # NUC131 build/flash documentation
│   ├── TODO_PROGRESS.md                # Implementation roadmap
│   └── ...
│
├── include/hal/
│   ├── hal_can.h
│   ├── hal_uart.h
│   ├── hal_gpio.h
│   └── hal_system.h
│
├── src/
│   ├── main.c
│   ├── core/
│   ├── protocols/
│   ├── profiles/
│   └── hal/
│       ├── hal_native/
│       ├── hal_stm32/
│       ├── hal_esp32/
│       └── hal_nuc131/
│
└── tests/
    ├── unit/
    ├── integration/
    └── ...
```

---

## Implementation Status

- ✅ **Peugeot 407 CAN2004** — Engine, Climate, Doors, HVAC, TPMS, Parking Radar
- ✅ **Hiworld, Raise, Bagoo** — Head unit protocol bidirectional implementations
- ✅ **STM32F103, ESP32, Linux HAL** — Fully working portable backends
- ✅ **Nuvoton NUC131 HAL** — Initial support, validated against hardware reference
- ⏳ **PSA CAN2010** — Multi-brand profiles in progress
- ⏳ **VAG PQ35/46** — Golf, Passat, Octavia variants in progress

---

## Development Workflow

### Desktop Simulation

1. Set up virtual CAN:
   ```bash
   sudo modprobe vcan
   sudo ip link add dev vcan0 type vcan
   sudo ip link set up vcan0
   ```

2. Run tests:
   ```bash
   pio test -e native_test_runner
   ```

3. Inject test frames:
   ```bash
   cansend vcan0 128#0100
   ```

### Hardware Deployment

1. Install PlatformIO and target platform dependencies
2. Configure `platformio.ini` for your board
3. Build: `pio run -e <env_name>`
4. Flash using the platform's upload tool

---

## License

Dual **MIT / Apache 2.0** open-source license.

---

## References

- **PlatformIO:** https://platformio.org
- **Linux SocketCAN:** https://github.com/torvalds/linux/tree/master/drivers/net/can
- **STM32F1xx HAL:** https://github.com/STMicroelectronics/STM32CubeF1
- **ESP-IDF:** https://github.com/espressif/esp-idf
- **Nuvoton NUC131 Platform:** https://github.com/fazerxlo/nuvoton_nuc131
- **Reference Firmware (vw_nc03):** https://github.com/fazerxlo/canbox/tree/main/vw_nc03
- **CAN 2.0 Specification:** ISO 11898-1
