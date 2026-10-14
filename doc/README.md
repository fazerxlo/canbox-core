# OpenCanbox Core — Documentation Index

This directory contains specifications, hardware reverse-engineering reports, protocol references, and integration guides for the **OpenCanbox Core** automotive firmware project.

---

## 1. Master Status & Implementation Trackers

| Document | Description |
|:---|:---|
| [TODO_PROGRESS.md](TODO_PROGRESS.md) | **Master Progress Matrix:** Live status of all 13 subsystems, command matrices (Uplink/Downlink), CAN IDs, implementation functions, test coverage, and roadmap. |

---

## 2. Hardware Architecture & Low-Power Management

| Document | Description |
|:---|:---|
| [POWER_MANAGEMENT_NUC131.md](POWER_MANAGEMENT_NUC131.md) | Automotive deep-sleep architecture for Nuvoton NUC131, 3-layer hardware isolation, NVIC/EXTI wake-up interrupt routing, and quiescent current safety ($< 35\ \mu\text{A}$). |
| [HARDWARE_GPIO_REVERSE_ILL_ACC.md](HARDWARE_GPIO_REVERSE_ILL_ACC.md) | Hardware pinout and reverse engineering of discrete I/O lines (ACC `PA8`, ILL `PA9`, PARK `PA12`, REV `PA13`), level-shifter circuits, and PCB traces. |
| [NUC131_SUPPORT.md](NUC131_SUPPORT.md) | Nuvoton Cortex-M0 NUC131 architecture guide, memory mapping (LDROM/APROM/SRAM), clock trees, and peripheral configuration. |
| [STM32F103-compatible_SPEC.md](STM32F103-compatible_SPEC.md) | Pinouts, memory layout, and peripheral mapping for STM32F103 / GD32F103 / APM32F103 hardware variants. |
| [BOOTLOADER_REPLACEMENT.md](BOOTLOADER_REPLACEMENT.md) | Dual-bank IAP bootloader specification, vector table relocation, firmware signature verification, and fail-safe flashing over UART. |
| [FIRMWARE_BUILD_AND_FLASH_GUIDE.md](FIRMWARE_BUILD_AND_FLASH_GUIDE.md) | Instructions for cross-compiling, native desktop simulation, OpenOCD flashing via SWD/ST-Link, and command-line usage. |

---

## 3. Hiworld Protocol Specifications (Peugeot 407 / PSA CAN2004)

Detailed topic specifications documenting PSA CAN frame decoders, Head Unit serial wire formats (`0x5A 0xA5`), and Android APK field mappings:

| Topic | Document | Subsystem / Commands |
|:---|:---|:---|
| **01. Steering & Stalk Keys** | [CANBOX_SPEC_HIWORLD_407_01_STEERING_STALK_KEYS.md](CANBOX_SPEC_HIWORLD_407_01_STEERING_STALK_KEYS.md) | Steering wheel buttons, stalk controls (`0x21F`, `0x221`), and radio controls $\rightarrow$ `0x11` / `0x01` |
| **02. Dual-Zone HVAC** | [CANBOX_SPEC_HIWORLD_407_02_DUAL_ZONE_HVAC.md](CANBOX_SPEC_HIWORLD_407_02_DUAL_ZONE_HVAC.md) | Dual-zone climate status, blower, AC, recirc, temperatures $\rightarrow$ `0x31` |
| **03. TPMS Tire Pressure** | [CANBOX_SPEC_HIWORLD_407_03_TPMS_TIRE_PRESSURE.md](CANBOX_SPEC_HIWORLD_407_03_TPMS_TIRE_PRESSURE.md) | 4-wheel tire pressures (`0x3A1`), alert status (`0x1E1`) $\rightarrow$ `0x39` |
| **04. Parking Radar (AAS)** | [CANBOX_SPEC_HIWORLD_407_04_PARKING_RADAR_AAS.md](CANBOX_SPEC_HIWORLD_407_04_PARKING_RADAR_AAS.md) | Native CAN2004 parking distance bars (`0x0E1`) $\rightarrow$ `0x22`, `0x23`, `0x24` |
| **05. Steering Angle (SAS)** | [CANBOX_SPEC_HIWORLD_407_05_STEERING_ANGLE_SAS.md](CANBOX_SPEC_HIWORLD_407_05_STEERING_ANGLE_SAS.md) | Steering angle sensor, calibration, and dynamic trajectory lines $\rightarrow$ `0x26` |
| **06. Trip Computer & Telemetry** | [CANBOX_SPEC_HIWORLD_407_06_TRIP_COMPUTER_TELEMETRY.md](CANBOX_SPEC_HIWORLD_407_06_TRIP_COMPUTER_TELEMETRY.md) | Trip 1, Trip 2, Instant consumption, range, speed, fuel economy $\rightarrow$ `0x33`, `0x34` |
| **07. Doors & Body Status** | [CANBOX_SPEC_HIWORLD_407_07_DOORS_BODY_STATUS.md](CANBOX_SPEC_HIWORLD_407_07_DOORS_BODY_STATUS.md) | Doors, trunk, bonnet, seatbelts, handbrake, exterior lights $\rightarrow$ `0x12` |
| **08. JBL Amplifier DSP** | [CANBOX_SPEC_HIWORLD_407_08_JBL_AMPLIFIER_DSP.md](CANBOX_SPEC_HIWORLD_407_08_JBL_AMPLIFIER_DSP.md) | Premium sound amplifier volume, fader, balance, equalizer $\rightarrow$ `0x81`, `0x82` |
| **09. RD4 / MFD Media Text** | [CANBOX_SPEC_HIWORLD_407_09_RD4_MFD_MEDIA_TEXT.md](CANBOX_SPEC_HIWORLD_407_09_RD4_MFD_MEDIA_TEXT.md) | Radio display / MFD LCD emulation, frequency, source, CD track $\rightarrow$ `0x91`, `0x92` |
| **10. BSI Settings & Clock Sync**| [CANBOX_SPEC_HIWORLD_407_10_BSI_SETTINGS_CLOCK_SYNC.md](CANBOX_SPEC_HIWORLD_407_10_BSI_SETTINGS_CLOCK_SYNC.md) | Vehicle personalization, lighting preferences, date/time sync $\rightarrow$ `0x68`, `0xA6` |
| **11. Car Configuration** | [CANBOX_SPEC_HIWORLD_407_11_CAR_CONFIGURATION_OPTIONS.md](CANBOX_SPEC_HIWORLD_407_11_CAR_CONFIGURATION_OPTIONS.md) | Head unit vehicle parameter menus and configuration items $\rightarrow$ `0x69`, `0x6A` |
| **12. Alerts & Diagnostics** | [CANBOX_SPEC_HIWORLD_407_12_ALERTS_DIAGNOSTICS.md](CANBOX_SPEC_HIWORLD_407_12_ALERTS_DIAGNOSTICS.md) | Real-time cockpit warning popups and diagnostic alerts $\rightarrow$ `0x42`, `0xEA` |

---

## 4. Hiworld Protocol Framework & Lifecycle

| Document | Description |
|:---|:---|
| [CANBOX_SPEC_HIWORLD_GENERIC_CONNECTION_PHASE.md](CANBOX_SPEC_HIWORLD_GENERIC_CONNECTION_PHASE.md) | Protocol connection handshake, keep-alive heartbeat (`0x00`), model discovery (`0x90`, `0xCA`), and baud rate specifications. |
| [CANBOX_SPEC_HIWORLD_CUSTOMIZATIONS_AND_DIFFERENCES.md](CANBOX_SPEC_HIWORLD_CUSTOMIZATIONS_AND_DIFFERENCES.md) | Comparison of protocol dialects, APK-specific quirks, and rate-limiting parameters. |
| [CANBOX_SPEC_HIWORLD_TOYOTA_OVERVIEW.md](CANBOX_SPEC_HIWORLD_TOYOTA_OVERVIEW.md) | Reference notes and protocol analysis for Hiworld Toyota implementations. |

---

## 5. Vehicle Diagnostics & Alert Journal (PSA CAN2004)

| Document | Description |
|:---|:---|
| [RT4_CAN_ALERTS.md](RT4_CAN_ALERTS.md) | Reverse-engineering of PSA CAN alert journal (`0x120`) and real-time warning popups (`0x1A1`). |
| [RT4_CAN_ALERTS_0x1A1.csv](RT4_CAN_ALERTS_0x1A1.csv) | Exhaustive lookup table mapping PSA alert IDs to text descriptions, severity levels, and cockpit icon assignments. |

---

## 6. Raise (RZC) Protocol Specifications

| Document | Description |
|:---|:---|
| [RAISE_PSA_407_SPEC.md](RAISE_PSA_407_SPEC.md) | Master Raise (RZC) protocol specification for PSA Peugeot 407 (`0x2E` framing). |
| [RAISE_SPEC_01_PEUGEOT_407_DATA.md](RAISE_SPEC_01_PEUGEOT_407_DATA.md) | Telemetry status packets and payload field mappings. |
| [RAISE_SPEC_02_GENERIC_PSA_RAISE_EXTENDED.md](RAISE_SPEC_02_GENERIC_PSA_RAISE_EXTENDED.md) | Extended PSA features supported by Raise adapters. |
| [RAISE_SPEC_03_VEHICLE_CONFIG_BSI_SETTINGS.md](RAISE_SPEC_03_VEHICLE_CONFIG_BSI_SETTINGS.md) | BSI vehicle configuration and comfort settings downlink packets. |
| [RAISE_SPEC_04_DOWNLINK_CONTROL_COMMANDS.md](RAISE_SPEC_04_DOWNLINK_CONTROL_COMMANDS.md) | Android Head Unit downlink control commands for Raise protocol. |

---

## 7. Downlink & Vehicle Control Deep-Dives

| Document | Description |
|:---|:---|
| [PEUGEOT_HIWORLD_CLIMA_PROTOCOL_DOWNLINK.md](PEUGEOT_HIWORLD_CLIMA_PROTOCOL_DOWNLINK.md) | Head Unit touchscreen climate controls mapped to PSA CAN messages. |
| [PEUGEOT_HIWORLD_CLIMA_PROTOCOL_UPLINK.md](PEUGEOT_HIWORLD_CLIMA_PROTOCOL_UPLINK.md) | PSA CAN climate state extraction and translation to Head Unit wire frames. |
| [PEUGEOT_RT4_CAR_CONFIG.md](PEUGEOT_RT4_CAR_CONFIG.md) | Multi-Function Display (MFD) configuration menu protocol. |
| [PEUGEOT_RT4_TRIP_RESET.md](PEUGEOT_RT4_TRIP_RESET.md) | Trip computer reset CAN frame sequences and timing. |

---

## 8. Integration & Manual Testing

| Document | Description |
|:---|:---|
| [MANUAL_TESTING_WITH_HEADUNIT.md](MANUAL_TESTING_WITH_HEADUNIT.md) | Bench and vehicle verification procedures with physical Android Head Units, virtual CAN test harnesses, and protocol analyzers. |

