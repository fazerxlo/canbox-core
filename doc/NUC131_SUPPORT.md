# Nuvoton NUC131 Support & Hardware Integration Notes

This document captures the hardware integration, peripheral mapping, and build architecture for the Nuvoton NUC131 target in `canbox-core`.

---

## 1. Target MCU Overview

* **Microcontroller:** Nuvoton NUC131 (e.g. `NUC131SD2AE`, `NUC131SC2AE`)
* **Core:** ARM Cortex-M0 running at 48–50 MHz (PLL from internal 22.1184 MHz HIRC / external crystal)
* **Memory:**
  * 8 KB – 16 KB SRAM (`0x20000000`–`0x20001FFF`)
  * 64 KB – 68 KB APROM Flash (Application ROM)
  * 4 KB LDROM (ISP Bootloader)
* **Automotive Peripherals:**
  * Bosch C_CAN 2.0B Controller with 32 message objects
  * High-speed UART controllers with 64-byte hardware FIFOs
  * Hardware CRC and system timers

---

## 2. Reference Sources

* **PlatformIO Platform:** [`fazerxlo/nuvoton_nuc131`](https://github.com/fazerxlo/nuvoton_nuc131) — PlatformIO platform definition, compiler scripts, and CMSIS/BSP integration.
* **Bootloader Architecture & Flash Swapping:** `doc/BOOTLOADER_REPLACEMENT.md` — Complete specification of the native Hiworld IAP bootloader, packaging engine (`tools/pack_hiworld_iap.py`), and OpenOCD SWD unlocking procedures.

---

## 3. Hardware Pinout & Peripheral Mapping

The `canbox-core` HAL implementation (`src/hal/hal_nuc131/`) and Hiworld bootloader (`src/hiworld_bootloader/`) map hardware pins to match automotive CAN box PCB designs (specifically Raise PSA-RZ-15-0121-B1):

| Function / Signal | NUC131 Pin | Direction / Mode | HAL Identifier | Notes |
| :--- | :---: | :---: | :--- | :--- |
| **CAN0 RX** | `PD.6` | Input (MFP CAN0) | `hal_can_init()` | Connected to CAN transceiver (TJA1050/SN65) RX (`SYS_GPD_MFP = 0x000000C0`) |
| **CAN0 TX** | `PD.7` | Output (MFP CAN0) | `hal_can_init()` | Connected to CAN transceiver TX |
| **UART0 RX** | `PB.0` | Input (MFP UART0) | `hal_uart_init()` | Serial stream from Android Head Unit (Rx @ 38,400 baud, 8N1) |
| **UART0 TX** | `PB.1` | Output (Push-Pull 1) | `hal_uart_init()` | Serial stream to Android Head Unit (Tx @ 38,400 baud, 8N1) |
| **CAN STBY** | `PC.3` | Output (Push-Pull) | `GPIO_PIN_CAN_STBY` | Transceiver standby active-low control (`0`=Normal/Active, `1`=Standby). Driven `0` to silence buzzer |
| **Transceiver Enable** | `PC.0, PC.1, PC.2, PC.6, PC.7` | Output (Push-Pull) | `GPIOC_DOUT` | Driven HIGH (`1`) push-pull (`GPIOC_DOUT = 0x000000C7`, `GPIOC_PMD = 0x0000F045`) |
| **Buzzer Pins** | `PB.3`, `PB.12` | Output (Push-Pull) | `GPIOB_DOUT` | Held HIGH (`1`) push-pull (`GPIOB_DOUT = 0x0000113F`, `GPIOB_PMD = 0x43030FF4`) to silence parking chime circuit |
| **ACC / HU Power** | `PA.8` | Output (Push-Pull) | `GPIO_PIN_HEADUNIT_POWER` | Switched $+12\text{V}$ ACC wakeup output to Head Unit |
| **Illumination** | `PA.9` | Output (Push-Pull) | `GPIO_PIN_ILL_OUT` | Physical $+12\text{V}$ night dimming / ILL wire |
| **Brake / Handbrake** | `PA.12` | Output (Push-Pull) | `GPIO_PIN_BRAKE_OUT` | Physical Handbrake trigger wire (held `0`) |
| **Reverse Trigger**| `PA.13` | Output (Push-Pull) | `GPIO_PIN_REVERSE_OUT` | Physical $+12\text{V}$ camera switch trigger wire |
| **SWD CLK / DIO** | `ICE_CLK` / `ICE_DAT` | Special Function | In-Circuit Debug | SWD test pads (`SWD DPIDR = 0x0BB11477`) |

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
* Push-Pull digital outputs (`PA.8` ACC, `PA.9` ILL, `PA.12` BRAKE, `PA.13` REVERSE), Open-Drain output (`PC.3` CAN STBY), and input (`PA.0` IGN) configured via `GPIO_SetMode()`.
* Direct atomic bit manipulation using `port->DOUT` and `port->PIN`.

### 4.4 System & Timing (`hal_system_nuc131.c`)
* SysTick timer provides a 1 ms tick counter (`s_system_ticks_ms`).
* Non-blocking uptime queried through `hal_get_time_ms()`.

---

## 5. PlatformIO Configuration (`platformio.ini`)

The project configures the bootloader and application environments using custom linker scripts:

```ini
[env:nuc131_bootloader]
platform = https://github.com/fazerxlo/nuvoton_nuc131.git
board = nuvoton_nuc131
board_build.ldscript = ldscripts/nuc131_bootloader.ld
board_build.custom_startup = yes
test_ignore = *
build_flags = 
    -Iinclude
    -Isrc
    -DPLATFORM_NUC131
    -Wall -Wextra -std=c99
build_src_filter = 
    +<hiworld_bootloader/startup_bootloader.S>
    +<hiworld_bootloader/hiworld_iap_nuc131.c>

[env:raise_nuc131]
platform = https://github.com/fazerxlo/nuvoton_nuc131.git
board = nuvoton_nuc131
board_build.ldscript = ldscripts/nuc131_app.ld
board_build.custom_startup = yes
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
Compile the NUC131 targets using the PlatformIO CLI:

```bash
# Hiworld IAP Bootloader (APROM 0x00000000 - 0x00001FFF, 8 KB)
pio run -e nuc131_bootloader

# canbox-core Application (APROM 0x00002000 - 0x0000FFFF, 56 KB)
pio run -e raise_nuc131

# Pack application into Hiworld IAP container for Head Unit USB update
python3 tools/pack_hiworld_iap.py .pio/build/raise_nuc131/program.bin -o can_app.iap
```

### 6.2 Typical Resource Utilization (NUC131SD2AE)
* **Bootloader Flash:** ~1.85 KB of 8 KB partition (1.4% total MCU flash)
* **Application Flash:** ~15.0 KB of 56 KB partition (11.4% total MCU flash)
* **Application SRAM:** ~1.8 KB of 8 KB available SRAM (11.1%)

### 6.3 Flashing Options
1. **OpenOCD + ST-Link V2 (SWD):**
   - Mass erase factory lock: `openocd -f openocd_nuc131.cfg -c "init; halt; numicro chip_erase; shutdown"`
   - Flash bootloader: `openocd -f openocd_nuc131.cfg -c "init; halt; flash write_image erase .pio/build/nuc131_bootloader/program.bin 0x00000000; shutdown"`
   - Flash application: `openocd -f openocd_nuc131.cfg -c "init; halt; flash write_image erase .pio/build/raise_nuc131/program.bin 0x00002000; reset run; shutdown"`
2. **Head Unit In-Application Programming (IAP):**
   - Copy `can_app.iap` to `/canmcuupdata/can_app.iap` on a USB drive and trigger CAN Update from Head Unit settings.
3. **Serial Firmware Uploader (`tools/flash_hiworld_serial.py`):**
   - Direct PC flashing over UART cable (`/dev/ttyUSB1` @ 38,400 baud):
     ```bash
     ./tools/flash_hiworld_serial.py --port /dev/ttyUSB1
     ```
   - Automatically builds target `raise_nuc131`, packages IAP records, enters bootloader over UART, and flashes firmware in ~6.8 seconds with no ST-Link required.

---

## 7. Current Status

* ✅ Clean build with zero warnings under `-Wall -Wextra -std=c99`.
* ✅ Complete HAL coverage for CAN, UART, GPIO, and System Tick (calibrated 1000 Hz SysTick).
* ✅ Dual-partition memory layout with hardware interrupt bounce trampolines verified on physical board.
* ✅ Transceiver standby and buzzer GPIO states aligned with ground-truth factory MMIO configuration (buzzer completely silenced).
* ✅ Reset pin isolation: `PD7` (CAN TX) and `PC3` (CAN STBY) held HIGH at the first reset instruction to prevent vehicle CAN network disruption on boot/flash.
* ✅ Zero-latency reverse trigger on `PA13` supporting dual Peugeot 407 reverse frames (`0x036` and `0x0F6`).
* ✅ Standalone serial flashing over UART fully working and verified in hardware.

