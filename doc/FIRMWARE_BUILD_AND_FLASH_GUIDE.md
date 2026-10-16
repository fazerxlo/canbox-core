# Firmware Build, Versioning & Flashing Guide

This document explains how to build, flash, and upgrade OpenCanbox firmware across all supported microcontrollers (STM32F103, STM32F105/107, ESP32, Nuvoton NUC131) and the desktop simulator, including build versioning and in-vehicle Head Unit (HU) upgrades.

---

## 1. Firmware Versioning

Every build automatically embeds a version string into the binary at compile time. This string is reported to the Android Head Unit over UART via Hiworld Command `0xF0` (`HIWORLD_CMD_VERSION_REPORT`), viewable in **Car Settings -> Factory -> CAN Box Info / Settings**.

### 1.1 Default Version Format
Dynamic compile-time timestamp format:
```text
CANBOX-CORE-V<YYYYMMDD.hhmmss>   # e.g. CANBOX-CORE-V20261009.113534
```

### 1.2 Custom Version Overrides (`CANBOX_BUILD_VERSION` / `CANBOX_VERSION`)
Override the automatic timestamp for releases or bench testing:
```bash
# Inline override
CANBOX_BUILD_VERSION="CANBOX-CORE-V1.0.0" pio run -e stm32_cbox

# Or persistent session override
export CANBOX_BUILD_VERSION="CANBOX-CORE-V1.0.0"
pio run -e stm32_cbox
```

### 1.3 How Version Embedding Works
1. PlatformIO runs `tools/generate_version.py` before compilation.
2. The script checks `CANBOX_BUILD_VERSION` or `CANBOX_VERSION`; defaults to `CANBOX-CORE-V%Y%m%d.%H%M%S`.
3. Passes `-DCANBOX_BUILD_VERSION="<string>"` to the C preprocessor.
4. Core code accesses the string via `canbox_get_version()` (`src/core/canbox_version.c`).
5. `src/protocols/proto_hiworld_adapter.c` sends it to the Head Unit on handshake (`0x24`), query (`0x30`), or boot beacon.

---

## 2. Supported Build Targets & Artifacts

| Environment | Architecture | Board / Target | Output Binaries |
| --- | --- | --- | --- |
| `stm32_cbox` | Cortex-M3 (STM32F103C8T6) | BluePill F103C8 (Generic) | `.pio/build/stm32_cbox/firmware.bin` |
| `hiworld_stm32f105` | Cortex-M3 (STM32F105/107) | OEM Hiworld CAN Box (`0x08007000` IAP) | `.pio/build/hiworld_stm32f105/firmware.bin` |
| `esp32_cbox` | Xtensa Dual-Core (ESP32) | NodeMCU / ESP32-WROOM-32 | `.pio/build/esp32_cbox/firmware.bin`<br>`.pio/build/esp32_cbox/bootloader.bin`<br>`.pio/build/esp32_cbox/partitions.bin` |
| `nuc131_bootloader` | Cortex-M0 (NUC131SD2AE) | Hiworld IAP Bootloader (`0x00000000`) | `.pio/build/nuc131_bootloader/program.bin` |
| `raise_nuc131` | Cortex-M0 (NUC131SD2AE) | OEM Raise CAN Box (`0x00002000` app) | `.pio/build/raise_nuc131/program.bin` |
| `native_test` | Host x86_64 / Desktop Linux | Virtual CAN Simulator | `.pio/build/native_test/program` |

---

## 3. Building Firmware Binaries

Run build commands from repository root:

```bash
# 3.1 Generic STM32F103
pio run -e stm32_cbox                                # Output: .pio/build/stm32_cbox/firmware.bin

# 3.2 OEM Hiworld STM32F105 (0x08007000 IAP offset)
pio run -e hiworld_stm32f105                        # Output: .pio/build/hiworld_stm32f105/firmware.bin

# 3.3 ESP32 DevKit
pio run -e esp32_cbox                               # Output: .pio/build/esp32_cbox/{firmware,bootloader,partitions}.bin

# 3.4 Nuvoton NUC131 (Bootloader + App + IAP Container)
pio run -e nuc131_bootloader                        # Bootloader: .pio/build/nuc131_bootloader/program.bin
pio run -e raise_nuc131                             # App: .pio/build/raise_nuc131/program.bin
python3 tools/pack_hiworld_iap.py .pio/build/raise_nuc131/program.bin -o can_app.iap

# 3.5 Desktop Virtual CAN Simulator
pio run -e native_test                              # Output: .pio/build/native_test/program
```

---

## 4. Flashing Microcontrollers & In-Vehicle Upgrades

### 4.1 STM32F103 (BluePill / Custom PCB)
* **ST-Link V2 (PIO):** `pio run -e stm32_cbox -t upload`
* **ST-Link V2 (st-flash):** `st-flash write .pio/build/stm32_cbox/firmware.bin 0x08000000`
* **ST-Link V2 (OpenOCD):**
  ```bash
  openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program .pio/build/stm32_cbox/firmware.bin 0x08000000 verify reset exit"
  ```
* **UART Bootloader (stm32flash):** Set `BOOT0=1, BOOT1=0`, connect UART to PA9 (TX) / PA10 (RX):
  ```bash
  stm32flash -w .pio/build/stm32_cbox/firmware.bin -v -g 0x08000000 /dev/ttyUSB0
  ```
  Restore `BOOT0=0` after flashing.

### 4.2 OEM Hiworld CAN Box (STM32F105 / STM32F107)
Commercial Hiworld units preserve factory IAP bootloader at `0x08000000`–`0x08006FFF`. Application base is **`0x08007000`**. Connect ST-Link V2 to SWD pads (PA13/SWDIO, PA14/SWCLK, GND, 3.3V):
```bash
st-flash write .pio/build/hiworld_stm32f105/firmware.bin 0x08007000
# Or OpenOCD:
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program .pio/build/hiworld_stm32f105/firmware.bin 0x08007000 verify reset exit"
```

### 4.3 ESP32
* **PlatformIO Upload:** `pio run -e esp32_cbox -t upload`
* **esptool.py (Standalone):**
  ```bash
  esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 write_flash -z \
    --flash_mode dio --flash_freq 40m --flash_size 4MB \
    0x1000  .pio/build/esp32_cbox/bootloader.bin \
    0x8000  .pio/build/esp32_cbox/partitions.bin \
    0x10000 .pio/build/esp32_cbox/firmware.bin
  ```

### 4.4 Nuvoton NUC131 (Raise Hardware / NUC131SD2AE)
Raise hardware requires one-time factory chip unlock before flashing via ST-Link V2 (SWCLK $\to$ `ICE_CLK`, SWDIO $\to$ `ICE_DAT`, GND, 3.3V):
1. **Unlock & Mass Erase:**
   ```bash
   openocd -f openocd_nuc131.cfg -c "init; halt; numicro chip_erase; shutdown"
   ```
2. **Start OpenOCD & Flash via FMC Flasher:**
   ```bash
   openocd -f openocd_nuc131.cfg &
   # Flash Bootloader (0x00000000)
   python3 tools/nuc131_flasher.py .pio/build/nuc131_bootloader/program.bin 0x0000
   # Flash Application (0x00002000)
   python3 tools/nuc131_flasher.py .pio/build/raise_nuc131/program.bin 0x2000
   # Reset board
   nc -w 1 127.0.0.1 6666 <<< "reset run"
   ```

### 4.5 Standalone Serial IAP Flashing (UART, No ST-Link)
Once the Hiworld IAP bootloader is resident at `0x00000000`, application updates run over UART (38,400 baud):
```bash
# Auto-compile and flash
./tools/flash_hiworld_serial.py --port /dev/ttyUSB1

# Flash specific pre-built binary
./tools/flash_hiworld_serial.py --bin .pio/build/raise_nuc131/program.bin --no-build --port /dev/ttyUSB1
```
The script handles packaging, sending upgrade trigger (`5A A5 02 E0 00 00 E1`), bootloader handshake, block streaming, and reboot verification in ~7 seconds.

### 4.6 USB App Upgrade via Head Unit UI
> [!NOTE]
> Applicable only to OEM Hiworld CAN boxes or Raise CAN boxes with replaced Hiworld IAP bootloader.

#### 1. File Location & Directory Structure
When looking for update files, the app scans the following paths in order:
- `/storage/udisk1/canmcuupdata/`
- `/storage/udisk2/canmcuupdata/`
- `/storage/udisk3/canmcuupdata/`
- `/mnt/sdcard/canmcuupdata/`

**USB Drive Setup:**
1. Format a USB flash drive as **FAT32**.
2. Create a folder in the root named **`canmcuupdata`**.
3. Place your firmware file inside this folder.
   - For **Hiworld (WC)**, the default filename is **`can_app.iap`** (though the app accepts `.iap`, `.bin`, `.hex`, `.upde`, `.upg`, `.img`).
   - Pack with: `python3 tools/pack_hiworld_iap.py .pio/build/raise_nuc131/program.bin -o can_app.iap`

#### 2. Triggering the Upgrade via UI
1. Insert the USB drive into the Head Unit.
2. In the vehicle app, navigate to **Car Settings / Car Info $\rightarrow$ Version** (`VersionFragment`).
3. The CAN MCU update icon (`btn_canUpdate`) will detect the file and become highlighted (`mCanmcuLocalUpdate = true`).
4. Tap the icon; a confirmation dialog will appear.
5. Tap **Confirm** to start the update process. A progress bar will reflect frame-by-frame flash progress.

---

## 5. Verification on Android Head Unit

Connect OpenCanbox to the Android Head Unit UART port (38,400 baud, 8N1):
1. Power up Head Unit & OpenCanbox.
2. Open Android Settings or Factory App (password `126` or `3368`).
3. Set Protocol: **Hiworld -> PSA Peugeot Citroen -> Peugeot 407**.
4. Check **CAN Box Version** (or **MCU Version**): should display `CANBOX-CORE-V<timestamp>` (or custom `CANBOX_BUILD_VERSION`).

---

## 6. Continuous Integration & Pre-Commit Testing

Run test suites before committing or generating releases:
```bash
./tools/test_fast.sh                        # Fast unit test runner
./tools/verify_all.sh                       # Full 4-target verification gate
```
All targets must compile cleanly (`-std=c99 -Wall -Wextra -Werror`) with zero test failures.
