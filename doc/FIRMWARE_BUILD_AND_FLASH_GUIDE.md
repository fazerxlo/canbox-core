# Firmware Build, Versioning & Flashing Guide

This document explains how to build and flash OpenCanbox firmware for all supported microcontroller architectures (STM32F103, ESP32, Nuvoton NUC131) and desktop simulator targets, including how firmware versioning is embedded into binaries and reported to the Android Head Unit (HU).

---

## 1. Overview of Firmware Versioning

Every build automatically generates and embeds a unique firmware version string into the binary at compile time. This string is reported to the Android Head Unit over UART via Hiworld Protocol Command `0xF0` (`HIWORLD_CMD_VERSION_REPORT`), allowing you to verify the exact build running on the device directly from the Head Unit screen (**Car Settings -> Factory -> CAN Box Info / Settings**).

### 1.1 Default Version Format
By default (development builds), the version string is dynamically generated at compile time with a full date and time stamp:

```text
CANBOX-CORE-V<YYYYMMDD.hhmmss>

```

**Example:**

```text
CANBOX-CORE-V20261009.113534

```

* `<YYYYMMDD>`: 4-digit year, 2-digit month, 2-digit day.
* `<hhmmss>`: 2-digit hour (24-hour format), 2-digit minute, 2-digit second.

### 1.2 Custom Version Overrides (`CANBOX_BUILD_VERSION` / `CANBOX_VERSION`)

You can override the automatic timestamp with a custom version string (e.g. for release tags, release candidates, or custom bench labels) using either the `CANBOX_BUILD_VERSION` or `CANBOX_VERSION` environment variable.

#### Inline Override:

```bash
# Build STM32 firmware with a custom release tag
CANBOX_BUILD_VERSION="CANBOX-CORE-V1.0.0-RC1" pio run -e stm32_cbox

# Or using CANBOX_VERSION
CANBOX_VERSION="CANBOX-CORE-V1.0.0" pio run -e esp32_cbox

```

#### Persistent Session Override:

```bash
export CANBOX_BUILD_VERSION="CANBOX-CORE-BENCH-TEST-407"
pio run -e stm32_cbox

```

### 1.3 How Version Embedding Works

1. PlatformIO executes the SCons pre-build script `tools/generate_version.py` before compilation begins across all environments defined in `platformio.ini`.
2. The script checks for `CANBOX_BUILD_VERSION` or `CANBOX_VERSION` in the environment. If neither is set, it queries the system clock and generates `CANBOX-CORE-V%Y%m%d.%H%M%S`.
3. The script prints the embedded version to the build console:
```text
>>> [canbox-core] Firmware version embedded: CANBOX-CORE-V20261009.113534 <<<

```


4. It passes `-DCANBOX_BUILD_VERSION="<string>"` to the C compiler (`CPPDEFINES`).
5. Core code accesses the string via `canbox_get_version()` in `src/core/canbox_version.c` / `include/core/canbox_version.h`.
6. `src/protocols/proto_hiworld_adapter.c` initializes the Hiworld connection context with this string, which is transmitted to the Android Head Unit upon handshake (`0x24`), explicit query (`0x30`), or boot beacon.

---

## 2. Supported Build Targets & Artifacts

| Environment | Architecture | Board / Target | Output Binaries |
| --- | --- | --- | --- |
| `stm32_cbox` | ARM Cortex-M3 (STM32F103C8T6) | BluePill F103C8 | `.pio/build/stm32_cbox/firmware.bin`<br>

<br>`.pio/build/stm32_cbox/firmware.elf` |
| `esp32_cbox` | Xtensa Dual-Core (ESP32) | NodeMCU / ESP32-WROOM-32 | `.pio/build/esp32_cbox/firmware.bin`<br>

<br>`.pio/build/esp32_cbox/bootloader.bin`<br>

<br>`.pio/build/esp32_cbox/partitions.bin` |
| `nuc131_cbox` | ARM Cortex-M0 (NUC131SD2AE) | Nuvoton NUC131 CAN Box | `.pio/build/nuc131_cbox/firmware.bin` |
| `native_test` | Host x86_64 / Desktop Linux | Virtual CAN Simulator | `.pio/build/native_test/program` |

---

## 3. Building Firmware Binaries

Run the build commands from the root directory of the repository.

### 3.1 Build STM32 Target (`stm32_cbox`)

```bash
# Standard build with automatic timestamp
pio run -e stm32_cbox

# Build with explicit version
CANBOX_BUILD_VERSION="CANBOX-CORE-V1.0.0" pio run -e stm32_cbox
```
The resulting binary will be located at:
```
.pio/build/stm32_cbox/firmware.bin
```

### 3.2 Build ESP32 Target (`esp32_cbox`)

```bash
# Standard build with automatic timestamp
pio run -e esp32_cbox

# Build with explicit version
CANBOX_BUILD_VERSION="CANBOX-CORE-V1.0.0" pio run -e esp32_cbox
```

The resulting binaries will be located at:
```
.pio/build/esp32_cbox/firmware.bin
.pio/build/esp32_cbox/bootloader.bin
.pio/build/esp32_cbox/partitions.bin
```

### 3.3 Build Nuvoton NUC131 Target (`nuc131_cbox`)

```bash
pio run -e nuc131_cbox

```

The resulting binary will be located at:
```
.pio/build/nuc131_cbox/firmware.bin
```

### 3.4 Build Desktop Virtual Simulator (`native_test`)
```bash
pio run -e native_test
```
The executable will be located at:
```
.pio/build/native_test/program
```

---

## 4. Flashing Microcontrollers

### 4.1 STM32F103 (BluePill / Custom PCB)

#### Option A: Direct PlatformIO Upload (via ST-Link V2)

Connect ST-Link V2 (SWDIO, SWCLK, GND, 3.3V) and execute:

```bash
pio run -e stm32_cbox -t upload

```

#### Option B: Standalone Flashing with `openocd`
```bash
st-flash write .pio/build/stm32_cbox/firmware.bin 0x08000000
```

#### Option C: Standalone Flashing with `openocd`

```bash
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "program .pio/build/stm32_cbox/firmware.bin 0x08000000 verify reset exit"
```

#### Option D: UART Bootloader (STM32 DFU / `stm32flash`)
1. Set jumper `BOOT0 = 1`, `BOOT1 = 0`.
2. Connect USB-to-UART adapter to PA9 (TX) and PA10 (RX).
3. Reset board and run:
   ```bash
   stm32flash -w .pio/build/stm32_cbox/firmware.bin -v -g 0x08000000 /dev/ttyUSB0
   ```
4. Restore `BOOT0 = 0` and reset.

---

### 4.2 ESP32 (DevKit / Bench Adapter)

#### Option A: Direct PlatformIO Upload (USB Serial)

Connect ESP32 via USB and execute:

```bash
pio run -e esp32_cbox -t upload
```

#### Option B: Standalone Flashing with `esptool.py`
If uploading pre-built binaries on another machine without PlatformIO:
```bash
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 \
  --before default_reset --after hard_reset write_flash -z \
  --flash_mode dio --flash_freq 40m --flash_size 4MB \
  0x1000  .pio/build/esp32_cbox/bootloader.bin \
  0x8000  .pio/build/esp32_cbox/partitions.bin \
  0x10000 .pio/build/esp32_cbox/firmware.bin
```

---

### 4.3 Nuvoton NUC131 (OEM Hiworld / Raise Hardware)

Most OEM CAN boxes (such as the Raise PSA-RZ-15-0121-B1) ship with Nuvoton NUC131 microcontrollers. These can be flashed easily using a standard **ST-Link V2** and OpenOCD if you do not have a Nuvoton Nu-Link programmer.

#### ⚠️ PRE-REQUISITE: Unlocking Factory-Protected Boards

Factory OEM boards have Hardware Readout Protection (Code Read Protection) activated in the `CONFIG0` register. Standard flashing commands will fail because the internal Flash Memory Controller actively blocks SWD access to the flash memory. **You must mass-erase the chip first to unlock it.**

**IMPORTANT: To unlock a locked MCU, you MUST connect the ST-Link's RST pin to the NUC131's nRST pin. A standard 4-wire SWD connection is not sufficient to halt a locked core.**

1. **Wire the ST-Link V2 to the OEM board (5 wires):**
* ST-Link `SWCLK` -> Board `ICE_CLK`
* ST-Link `SWDIO` -> Board `ICE_DAT`
* ST-Link `GND` -> Board `GND`
* ST-Link `3.3V` -> Board `VDD`
* ST-Link `RST` -> Board `nRST`


2. **Create a custom OpenOCD config (`nuc131_stlink.cfg`):**
```tcl
adapter driver hla
hla_layout stlink
transport select hla_swd
adapter speed 200

# Force OpenOCD to accept ST-Link USB VID/PID
hla_vid_pid 0x0483 0x3748 0x0483 0x374b

# Force hardware reset during connection
reset_config srst_only srst_nogate connect_assert_srst

if { [info exists CPUTAPID] == 0 } {
   set CPUTAPID 0x2ba01477
}

source [find target/numicro.cfg]

```


3. **Run the Mass Erase Command:**
Execute the following single-line command. This command forces a hardware reset, halts the core, executes the chip erase to wipe the lock bit, and then verifies the `CONFIG0` register:
```bash
sudo openocd -f nuc131_stlink.cfg -c "init; reset halt; numicro chip_erase; reset halt; sleep 200; mdw 0x00300000 1; shutdown"

```


4. **Verify the unlock:**
Check the output of the terminal after running the command above. Look for the `mdw` output. If it returns `0x00300000: ffffffff`, the chip is successfully unlocked, the factory firmware is erased, and the board is ready to be flashed.

#### Option A: Flashing with ST-Link V2 (Standalone OpenOCD)

Once the chip is unlocked, you can flash the OpenCanbox firmware using the same configuration file (the `RST` pin connection is no longer strictly required, but recommended):

```bash
sudo openocd -f nuc131_stlink.cfg -c "program .pio/build/nuc131_cbox/firmware.bin 0x00000000 verify reset exit"

```

#### Option B: Flashing with Nu-Link ICE (If Available)

If you possess an official Nuvoton Nu-Link adapter, you can use the built-in target scripts:

```bash
openocd -f interface/nulink.cfg -f target/numicroM0.cfg \
  -c "program .pio/build/nuc131_cbox/firmware.bin 0x00000000 verify reset exit"

```

---

## 5. Verification on Android Head Unit

After flashing the firmware and connecting OpenCanbox to the Android Head Unit UART port (38,400 baud, 8N1):

1. **Power up Head Unit & OpenCanbox.**
2. Open Android Settings or the Factory App (`126` or `3368` password depending on head unit vendor).
3. Navigate to **CAN Type / CAN Bus Protocol Settings**.
4. Select **Hiworld -> PSA Peugeot Citroen -> Peugeot 407**.
5. Check the displayed **CAN Box Version** (or **MCU Version**):
* You should see:
```text
CANBOX-CORE-V20261009.113534

```


*(or your custom `CANBOX_BUILD_VERSION` string)*.



---

## 6. Continuous Integration & Pre-Commit Testing

Before generating release binaries, run the full test suite to guarantee zero regression:

```bash
# Run Unit Tests (92 test cases)
pio test -e native_test_runner

# Run Integration Pipeline Tests (27 test cases)
pio test -e integration_test

```

All tests must report `[PASSED]` with zero compilation warnings under `-Wall -Wextra -std=c99`.
