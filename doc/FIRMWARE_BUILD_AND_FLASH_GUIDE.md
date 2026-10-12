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
| `stm32_cbox` | ARM Cortex-M3 (STM32F103C8T6) | BluePill F103C8 (Generic) | `.pio/build/stm32_cbox/firmware.bin`<br>`.pio/build/stm32_cbox/firmware.elf` |
| `hiworld_stm32f105` | ARM Cortex-M3 (STM32F105RB / STM32F107VC) | OEM Hiworld CAN Box (`0x08007000` IAP) | `.pio/build/hiworld_stm32f105/firmware.bin`<br>`.pio/build/hiworld_stm32f105/firmware.elf` |
| `esp32_cbox` | Xtensa Dual-Core (ESP32) | NodeMCU / ESP32-WROOM-32 | `.pio/build/esp32_cbox/firmware.bin`<br>`.pio/build/esp32_cbox/bootloader.bin`<br>`.pio/build/esp32_cbox/partitions.bin` |
| `nuc131_bootloader` | ARM Cortex-M0 (NUC131SD2AE) | Hiworld IAP Bootloader (`0x00000000` base) | `.pio/build/nuc131_bootloader/program.bin` |
| `raise_nuc131` | ARM Cortex-M0 (NUC131SD2AE) | OEM Raise CAN Box (`0x00002000` app offset) | `.pio/build/raise_nuc131/program.bin` |
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

### 3.2 Build OEM Hiworld STM32F105 Target (`hiworld_stm32f105`)

```bash
# Standard build with automatic timestamp
pio run -e hiworld_stm32f105

# Build with explicit version
CANBOX_BUILD_VERSION="CANBOX-CORE-V1.0.0" pio run -e hiworld_stm32f105
```
The resulting binary configured with `0x08007000` IAP displacement will be located at:
```
.pio/build/hiworld_stm32f105/firmware.bin
```

### 3.3 Build ESP32 Target (`esp32_cbox`)

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

### 3.4 Build Nuvoton NUC131 Targets

```bash
# Hiworld IAP Bootloader (0x00000000 - 0x00001FFF)
pio run -e nuc131_bootloader

# canbox-core Application (0x00002000 - 0x0000FFFF)
pio run -e raise_nuc131

# Pack application into Hiworld IAP update container
python3 tools/pack_hiworld_iap.py .pio/build/raise_nuc131/program.bin -o can_app.iap
```

The resulting binaries will be located at:
```
.pio/build/nuc131_bootloader/program.bin
.pio/build/raise_nuc131/program.bin
```

### 3.5 Build Desktop Virtual Simulator (`native_test`)
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

### 4.2 OEM Hiworld CAN Box (STM32F105 / STM32F107 Connectivity Line)

Commercial Hiworld CAN boxes include a factory resident IAP bootloader occupying `0x08000000` to `0x080067FF` and persistent configuration in Sector 13 (`0x08006800`–`0x08006FFF`). The application entry point is located at **`0x08007000`**.

#### Flashing via ST-Link V2 (Preserving OEM Bootloader):
Connect ST-Link V2 to SWD test pads (`PA13` SWDIO, `PA14` SWCLK, `GND`, `3.3V`):
```bash
# Using st-flash
st-flash write .pio/build/hiworld_stm32f105/firmware.bin 0x08007000

# Or via OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "program .pio/build/hiworld_stm32f105/firmware.bin 0x08007000 verify reset exit"
```

---

### 4.3 ESP32 (DevKit / Bench Adapter)

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

### 4.4 Nuvoton NUC131 (OEM Raise Hardware / NUC131SD2AE)

OEM Raise CAN boxes (such as `PSA-RZ-15-0121-B1`) ship with Nuvoton NUC131 microcontrollers. These are flashed using an **ST-Link V2** and OpenOCD using the dedicated project configuration `openocd_nuc131.cfg`.

#### ⚠️ PRE-REQUISITE: Unlocking Factory-Protected Boards
Factory Raise boards have Security Lock Active in `CONFIG0` (`0xF70FFBB9`, bit 1 = 0), causing all flash reads to return `0xFFFFFFFF`. A one-time mass chip erase clears this lock.

1. **Wire ST-Link V2 to Board SWD Pads:**
   * ST-Link `SWCLK` (Pin 9) $\to$ Board `ICE_CLK`
   * ST-Link `SWDIO` (Pin 7) $\to$ Board `ICE_DAT`
   * ST-Link `GND` (Pin 4/6/8) $\to$ Board `GND`
   * ST-Link `3.3V` (Pin 1) $\to$ Board `VDD`

2. **Mass Chip Erase (One-Time Unlock):**
   ```bash
   openocd -f openocd_nuc131.cfg -c "init; halt; numicro chip_erase; shutdown"
   ```

3. **Flash Hiworld IAP Bootloader (`0x00000000`):**
   ```bash
   openocd -f openocd_nuc131.cfg -c "init; halt; flash write_image erase .pio/build/nuc131_bootloader/program.bin 0x00000000; verify_image .pio/build/nuc131_bootloader/program.bin 0x00000000; shutdown"
   ```

4. **Flash `canbox-core` Application (`0x00002000`):**
   ```bash
   openocd -f openocd_nuc131.cfg -c "init; halt; flash write_image erase .pio/build/raise_nuc131/program.bin 0x00002000; verify_image .pio/build/raise_nuc131/program.bin 0x00002000; reset run; shutdown"
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
# Run Unit Tests (93 test cases)
pio test -e native_test_runner

# Run Integration Pipeline Tests (27 test cases)
pio test -e integration_test
```

All tests must report `[PASSED]` with zero compilation warnings under `-Wall -Wextra -std=c99`.
