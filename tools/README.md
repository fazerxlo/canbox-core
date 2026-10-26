# OpenCanbox Developer Tools & Utilities

This directory contains standalone CLI scripts, diagnostic tools, automated test harnesses, and flashing utilities developed for the **OpenCanbox Core** project.

---

## Tool Categories

1. [Firmware Build & Flashing Tools](#1-firmware-build--flashing-tools)
2. [Hardware-in-the-Loop (HIL) & E2E Testing](#2-hardware-in-the-loop-hil--e2e-testing)
3. [Protocol Emulators & Scenario Injectors](#3-protocol-emulators--scenario-injectors)
4. [Log Analysis, Triage & Diagnostics](#4-log-analysis-triage--diagnostics)
5. [Build Automation & Helpers](#5-build-automation--helpers)

---

## 1. Firmware Build & Flashing Tools

### `flash_hiworld_serial.py`
**Automated Wireless/ST-Link-Free Serial Firmware Flasher (Hiworld IAP)**
- **Purpose:** Automatically compiles the NUC131 firmware target, packages it into obfuscated Hiworld IAP records, enters the bootloader over UART (`5A A5 02 E0 00 00 E1`), unlocks flash using security credentials, streams all records with live progress, and verifies clean application reboot.
- **Flashing Time:** ~6.8 seconds at 38,400 baud.
- **Usage:**
  ```bash
  # Flash default NUC131 target over serial (auto-builds raise_nuc131):
  ./tools/flash_hiworld_serial.py --port /dev/ttyUSB1

  # Flash with custom version tag:
  ./tools/flash_hiworld_serial.py --port /dev/ttyUSB1 --version CANBOX-CORE-V1.0.0

  # Flash pre-built .bin without compiling:
  ./tools/flash_hiworld_serial.py --bin .pio/build/raise_nuc131/program.bin --no-build --port /dev/ttyUSB1
  ```
- **Key Arguments:**
  - `--port`: Serial port of the CAN box UART connection (default: `/dev/ttyUSB1`).
  - `--baud`: UART baud rate (default: `38400`).
  - `--bin`: Path to custom binary (default: `.pio/build/raise_nuc131/program.bin`).
  - `--no-build`: Skip PlatformIO build step.
  - `--version`: Embed a custom build version string into the firmware.

---

### `update_canbox_usb.py`
**Automated USB Flash Drive Firmware Updater for Head Unit**
- **Purpose:** One-click script to compile the `raise_nuc131` target, convert the binary into the commercial Hiworld `can_app.iap` container format, discover connected USB drives containing a `canmcuupdata` directory, safely copy and sync the file, and verify checksum integrity.
- **Usage:**
  ```bash
  # Compile raise_nuc131, convert to can_app.iap, and write to connected USB drive:
  ./tools/update_canbox_usb.py

  # Use existing .bin without rebuilding:
  ./tools/update_canbox_usb.py --no-build

  # Target a specific mount path or auto-create 'canmcuupdata' if missing:
  ./tools/update_canbox_usb.py --create-dir
  ```
- **Key Arguments:**
  - `-e`, `--env`: PlatformIO environment (default: `raise_nuc131`).
  - `--no-build`: Skip build step and use existing binary.
  - `-t`, `--target`: Manually specify target USB path or destination directory.
  - `--create-dir`: Automatically create `canmcuupdata/` on the USB drive if absent.
  - `--dry-run`: Test compilation and IAP conversion without writing to USB.

---

### `pack_hiworld_iap.py`
**Hiworld CAN Box Firmware Container Packer (.bin $\to$ .iap)**
- **Purpose:** Takes a raw ARM Cortex-M0 application binary (compiled for offset `0x00002000`) and packages it into the commercial Hiworld `can_app.iap` container format expected by Android Head Units (`/canmcuupdata/can_app.iap`).
- **Features:** Divides binary into 128-byte payload records, applies Hiworld byte-scrambling/obfuscation (`cipher[i] = (plain[i] + i + key[i%4]) & 0xFF`), and calculates 32-bit additive record checksums.
- **Usage:**
  ```bash
  python3 tools/pack_hiworld_iap.py .pio/build/raise_nuc131/program.bin -o can_app.iap
  ```
- **Key Arguments:**
  - `bin_path`: Input `.bin` file.
  - `-o`, `--output`: Output `.iap` container file (default: `can_app.iap`).
  - `-t`, `--template`: Optional OEM `.iap` file to inherit container metadata.

---

### `nuc131_fast_flasher.py`
**High-Speed OpenOCD / SWD Flash Programmer via SRAM Stub**
- **Purpose:** Uses OpenOCD TCL RPC (:6666) to load a 32-byte Cortex-M0 microcode stub into NUC131 SRAM (`0x20000200`), then DMA streams 512-byte pages into SRAM (`0x20000400`) to trigger FMC page erase and programming in hardware.
- **Flashing Time:** Flashes 64 KB in under 2 seconds over ST-Link V2.
- **Usage:**
  ```bash
  # Ensure OpenOCD is running (openocd -f openocd_nuc131.cfg)
  ./tools/nuc131_fast_flasher.py .pio/build/raise_nuc131/program.bin 0x2000
  ```
- **Key Arguments:**
  - `bin_path`: Path to binary image.
  - `base_addr`: Destination flash address (hex, e.g. `0x0000` for bootloader, `0x2000` for application).

---

### `nuc131_flasher.py`
**Direct TCL RPC Flash Programmer**
- **Purpose:** Direct page-by-page FMC register programmer using OpenOCD TCL commands without loading SRAM microcode. Reliable fallback when debugging low-level FMC lock or write cycle behaviors.
- **Usage:**
  ```bash
  ./tools/nuc131_flasher.py .pio/build/nuc131_bootloader/program.bin 0x0000
  ```

---

## 2. Hardware-in-the-Loop (HIL) & E2E Testing

### `test_bsi_workbench.py`
**PSA 407 BSI & SLCAN Automated Workbench Verification Suite**
- **Purpose:** Automates bidirectional verification of OpenCanbox Core against a physical bench (Peugeot 407 BSI, RD4, and SLCAN `can0`). Validates live BSI cyclic traffic (`0x036`, `0x0F6`, `0x361`, `0x165`), tests CAN-to-HU telemetry dispatch, and verifies HU downlink commands (ACK, atomic CAN injection on `0x361`, and immediate optimistic HU telemetry).
- **Usage:**
  ```bash
  # Run full automated test suite:
  ./tools/test_bsi_workbench.py

  # Passively sniff and decode live BSI status:
  ./tools/test_bsi_workbench.py --sniff

  # Test single setting (e.g. drl, wiper, lock, afs, boot):
  ./tools/test_bsi_workbench.py --setting drl
  ```

---

### `canbox_e2e_logger.py`
**Multi-Stream Real-Time Diagnostic Logger & Test Harness**
- **Purpose:** Core E2E development tool. Spawns the desktop simulator or bridges physical USB serial lines, captures CAN bus traffic (`[CAN]`), OpenCanbox transmit (`[APP_UART_TX]`), Head Unit downlink (`[APP_UART_RX]`), and OEM reference box (`[ORIG_CANBOX]`).
- **Features:** 30-second rolling ring buffer lookback. Pressing `L` instantly dumps the last 30 seconds to a timestamped file (`dump_YYYY-MM-DD_HH-MM-SS.log`).
- **Documentation:** See [`README_canbox_e2e_logger.md`](README_canbox_e2e_logger.md).
- **Usage:**
  ```bash
  ./tools/canbox_e2e_logger.py -y
  # Or with manual port selection:
  ./tools/canbox_e2e_logger.py --can /dev/ttyUSB0 --hu /dev/ttyUSB1 --baud 38400
  ```

---

### `canbox_bench_e2e.py`
**Interactive Curses Bench Test Runner**
- **Purpose:** Fullscreen terminal UI (curses) showing real-time SocketCAN traffic, decoded vehicle metrics (RPM, speed, doors, reverse, lights, HVAC), and live Hiworld protocol packet stream simultaneously.
- **Usage:**
  ```bash
  ./tools/canbox_bench_e2e.py --can-interface can0 --hu-port /dev/ttyUSB1
  ```

---

### `canbox_manual_test.py`
**Automated HIL & Scenario Replay Runner**
- **Purpose:** Automates testing on host Linux by setting up `vcan0`, checking build binaries, discovering serial adapters, spawning the `native_test` executable, and replaying recorded CAN logs (.csv) in a continuous loop.
- **Usage:**
  ```bash
  ./tools/canbox_manual_test.py --can-iface vcan0 --csv test/logs/sample_drive.csv
  ```

---

## 3. Protocol Emulators & Scenario Injectors

### `hiworld_reverse_radar_test.py`
**Reverse Gear & Parking Radar Emulator**
- **Purpose:** Emulates Hiworld parking radar distance (`CMD 0x41`), dynamic steering guidelines (`CMD 0x11`), and reverse gear trigger over UART to test Head Unit camera and radar overlay response. Supports optional ADB broadcast trigger (`--adb-trigger`).
- **Usage:**
  ```bash
  ./tools/hiworld_reverse_radar_test.py --port /dev/ttyUSB1 --baud 38400
  ```

---

### `hiworld_doors_test.py`
**Door & Aperture Status Emulator**
- **Purpose:** Generates Hiworld `CMD 0x12` packets simulating open/closed doors (FL, FR, RL, RR, trunk, bonnet) to verify Android UI popups and door overlay graphics.
- **Usage:**
  ```bash
  ./tools/hiworld_doors_test.py --port /dev/ttyUSB1 --door driver --state open
  ```

---

### `hiworld_tpms_test.py` / `tpms_hiworld_serial.py`
**Tire Pressure Monitoring (TPMS) Emulator**
- **Purpose:** Generates Hiworld numeric tire pressures (`CMD 0x66`, in 0.1 bar units) and discrete alarm states (`CMD 0x18`, OK / Low / Puncture) with protocol keep-alives to test the Head Unit TPMS app page.
- **Usage:**
  ```bash
  ./tools/hiworld_tpms_test.py --port /dev/ttyUSB1 --fl 2.4 --fr 2.4 --rl 2.2 --rr 2.2
  ```

---

### `test_can_alerts_interactive.py`
**Interactive PSA CAN Alert Tester (SocketCAN)**
- **Purpose:** Step-by-step interactive tester for vehicle alerts. Iterates through the verified PSA CAN alert catalog (`doc/RT4_CAN_ALERTS_0x1A1.csv`), broadcasts `0x1A1` on SocketCAN, and prompts the user (`[y]`/`[n]`/`[r]`) to verify whether the Head Unit pop-up displays the expected message text.
- **Usage:**
  ```bash
  ./tools/test_can_alerts_interactive.py --can-iface can0
  ```

---

### `replay_serial.py`
**Raw Serial Packet Stream Replayer**
- **Purpose:** Replays recorded binary or hex-dumped serial telemetry logs directly into a physical serial port connected to an Android Head Unit to reproduce bugs deterministically.
- **Usage:**
  ```bash
  ./tools/replay_serial.py --port /dev/ttyUSB1 --file sample_stream.hex --baud 38400
  ```

---

## 4. Log Analysis, Triage & Diagnostics

### `diff_canbox_frames.py`
**Differential Frame Analyzer (OEM vs OpenCanbox)**
- **Purpose:** Analyzes E2E dump logs (`dump_*.log`). Deduplicates repetitive heartbeats (98%+ noise reduction), correlates CAN actions with serial responses, and highlights exact byte/bit differences between the real OEM Canbox and OpenCanbox firmware.
- **Documentation:** See [`README_diff_canbox_frames.md`](README_diff_canbox_frames.md).
- **Usage:**
  ```bash
  ./tools/diff_canbox_frames.py dump_2026-10-09_20-15-00.log -p 31 11
  ```

---

### `triage_canbox_log.py`
**Log Triage & Anomaly Detector**
- **Purpose:** Fast analyzer designed for AI agents and developers. Scans `dump_*.log` files in < 2 seconds to detect:
  1. State flip-flops and oscillations (e.g. flickering door/sensor bits).
  2. Spurious UART frame floods.
  3. Checksum or length framing errors.
  4. CAN-to-UART causality (which CAN frame triggered which UART packet).
- **Usage:**
  ```bash
  ./tools/triage_canbox_log.py dump_2026-10-09_20-15-00.log
  ```

---

### `extract_alert_log.py`
**Alert Log & Fault Journal Extraction Tool**
- **Purpose:** Decodes and verifies PSA CAN2004 vehicle alerts from E2E dump logs:
  - Reassembles 21-byte fault journals from multiplexed CAN `0x120` frames (`0x7C`, `0xBC`, `0xFC`).
  - Decodes real-time rolling alert popups (`0x1A1`).
  - Validates serial alert packets (`CMD 0x42` and `CMD 0xEA`) against CAN ground truth.
- **Usage:**
  ```bash
  ./tools/extract_alert_log.py dump_2026-10-09_20-15-00.log --journal --realtime
  ```

---

## 5. Build Automation & Helpers

### `generate_version.py`
**SCons Pre-Build Version Embedder**
- **Purpose:** Invoked automatically by PlatformIO before every build (`extra_scripts = pre:tools/generate_version.py` in `platformio.ini`).
- **Features:** Queries environment variable `CANBOX_BUILD_VERSION` or `CANBOX_VERSION`. If unset, generates a dynamic build timestamp `CANBOX-CORE-V<YYYYMMDD.hhmmss>` and injects it via compiler define `-DCANBOX_BUILD_VERSION="..."`.
- **Usage:** Automatic during build. Can be invoked manually:
  ```bash
  python3 tools/generate_version.py
  ```

