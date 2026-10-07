# Nuvoton NUC131 Support & Hardware Integration Notes

This document captures the hardware integration, peripheral mapping, and build architecture for the Nuvoton NUC131 target in `canbox-core`.

---

## 1. Target MCU Overview

* **Microcontroller:** Nuvoton NUC131 (e.g. `NUC131XE3AE`, `NUC131SD2AE`)
* **Core:** ARM Cortex-M0 running at 50 MHz (PLL from internal 22.1184 MHz HIRC / external crystal)
* **Memory:**
  * 16 KB SRAM
  * 128 KB APROM Flash (Application ROM)
  * 4 KB LDROM (ISP Bootloader)
* **Automotive Peripherals:**
  * Bosch C_CAN 2.0B Controller with 32 message objects
  * High-speed UART controllers with 64-byte hardware FIFOs
  * Hardware CRC and system timers

---

## 2. Reference Sources

* **PlatformIO Platform:** [`fazerxlo/nuvoton_nuc131`](https://github.com/fazerxlo/nuvoton_nuc131) — PlatformIO platform definition, compiler scripts, and CMSIS/BSP integration.
* **Reference Commercial Firmware:** `fazerxlo/canbox/vw_nc03` — Verified OEM CAN box firmware layout for VW/PSA aftermarket adapters using NUC131.

---

## 3. Hardware Pinout & Peripheral Mapping

The `canbox-core` HAL implementation (`src/hal/hal_nuc131/`) maps the hardware pins to match automotive CAN box PCB designs:

| Function / Signal | NUC131 Pin | Direction / Mode | HAL Identifier | Notes |
| :--- | :---: | :---: | :--- | :--- |
| **CAN0 RX** | `PD.6` | Input (MFP CAN0) | `hal_can_init()` | Connected to CAN transceiver (TJA1050/SN65) RX |
| **CAN0 TX** | `PD.7` | Output (MFP CAN0) | `hal_can_init()` | Connected to CAN transceiver TX |
| **UART0 RX** | `PB.0` | Input (MFP UART0) | `hal_uart_init()` | Serial stream from Android Head Unit (Rx) |
| **UART0 TX** | `PB.1` | Output (MFP UART0) | `hal_uart_init()` | Serial stream to Android Head Unit (Tx) |
| **Status LED** | `PA.9` | Output Push-Pull | `GPIO_PIN_LED_STATUS` | Onboard heartbeat / diagnostic LED |
| **CAN STBY** | `PA.12` | Output Push-Pull | `GPIO_PIN_CAN_STBY` | Transceiver standby/silent control (`0`=Normal, `1`=Standby) |
| **ACC / HU Power** | `PA.8` | Output Push-Pull | `GPIO_PIN_HEADUNIT_POWER` | Switched $+12\text{V}$ ACC wakeup output to Head Unit |
| **Reverse Trigger**| `PA.13` | Output Push-Pull | `GPIO_PIN_REVERSE_OUT` | Physical $+12\text{V}$ camera switch trigger wire |
| **Illumination** | `PA.14` | Output Push-Pull | `GPIO_PIN_ILL_OUT` | Physical $+12\text{V}$ night dimming / ILL wire |
| **Ignition Sense** | `PA.0` | Input (Pull-Down) | `GPIO_PIN_IGNITION_IN` | Hardware analog $+12\text{V}$ ignition input (via divider/opto) |

---

## 4. HAL Driver Architecture (`src/hal/hal_nuc131/`)

The NUC131 HAL layer satisfies the portability constraints of OpenCanbox Core: all vendor-specific headers (`NUC131.h`, BSP macros) are strictly confined to `src/hal/hal_nuc131/`, with zero leakage into `src/core/`, `src/protocols/`, or `src/profiles/`.

### 4.1 CAN Subsystem (`hal_can_nuc131.c`)
* **Controller:** Built-in Bosch C_CAN IP core.
* **Message Object Allocation:**
  * Message Object `0`: Dedicated to CAN TX frame dispatch.
  * Message Objects `1` through `31`: Configured in FIFO / round-robin reception accepting all CAN standard and extended frames (`mask = 0x0`).
* **Register Interface & Wait Busy Helper:**
  * Uses indexed command request access: `can->IF[iface & 1].CREQ & CAN_IF_CREQ_BUSY_Msk` to safely synchronize transfer between internal RAM and interface registers.
  * Explicit prototype for `CAN_SetRxMsgObjAndMsk()` bridges BSP header omissions.
* **Queue Integration:** Frames received in the CAN ISR or polled path are pushed directly into a power-of-two lock-free SPSC ring buffer (`ring_buffer_t`).

### 4.2 UART Subsystem (`hal_uart_nuc131.c`)
* **Clock & FIFO:** UART0 module clock is derived from PLL with clock divider 1. RX FIFO is configured with 1-byte interrupt trigger threshold (`UART_FCR_RFITL_1BYTE`).
* **ISR Handling:** The RX interrupt handler reads received bytes using the FIFO status macro `!UART_GET_RX_EMPTY(UART0)` and pushes them into an SPSC ring buffer (`s_uart_rx_rb`).
* **Transmission:** Non-blocking byte transfer via `UART_WRITE(UART0, ...)` with FIFO empty polling.

### 4.3 GPIO Subsystem (`hal_gpio_nuc131.c`)
* Digital outputs (`PA.8`, `PA.9`, `PA.12`, `PA.13`, `PA.14`) and input (`PA.0`) configured via `GPIO_SetMode()`.
* Direct atomic bit manipulation using `port->DOUT` and `port->PIN`.

### 4.4 System & Timing (`hal_system_nuc131.c`)
* SysTick timer provides a 1 ms tick counter (`s_system_ticks_ms`).
* Non-blocking uptime queried through `hal_get_time_ms()`.

---

## 5. PlatformIO Configuration (`platformio.ini`)

The project configures the environment `nuc131_cbox` using the custom platform repository:

```ini
[env:nuc131_cbox]
platform = https://github.com/fazerxlo/nuvoton_nuc131.git
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

---

## 6. Build & Flashing

### 6.1 Compiling the Firmware
Compile the NUC131 target using the PlatformIO CLI:

```bash
pio run -e nuc131_cbox
```

### 6.2 Typical Resource Utilization (NUC131XE3AE)
* **RAM:** ~2.3 KB used of 16 KB total (~14%)
* **Flash:** ~56.5 KB used of 128 KB total (~43%)

### 6.3 Flashing Options
1. **Nu-Link (ICP / SWD):** Connect SWD lines (`ICE_CLK`, `ICE_DAT`, `RESET`, `GND`, `VCC`) to a Nuvoton Nu-Link programmer and flash using Nu-Link ICP Programming Tool or OpenOCD.
2. **ISP over UART / USB (`tool-nuvoton-isp`):** When booting into LDROM (holding boot pin or via command), the chip accepts binary firmware over UART0.

---

## 7. Current Status

* ✅ Clean build with zero warnings under `-Wall -Wextra -std=c99`.
* ✅ Complete HAL coverage for CAN, UART, GPIO, and System Tick.
* ✅ Fully decoupled from MCU-specific headers in core and profile layers.
