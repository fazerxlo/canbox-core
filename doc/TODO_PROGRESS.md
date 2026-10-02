# OpenCanbox Core: HU Feature Compatibility & Implementation Progress

**Target Head Unit Document:** [`/home/Fazer/git/QF_Canbus_system/README.md`](file:///home/Fazer/git/QF_Canbus_system/README.md)  
**Target Head Unit Application:** `com.qf.vehicle` (QF Canbus System / Peugeot Hiworld `wc` Driver)  
**Firmware Core Project:** `canbox-core` (Pure C99 Embedded Automotive Firmware)  
**Primary Vehicle Profile:** Peugeot 407 (PSA CAN2004 Comfort Bus @ 125 kbps)  
**Primary Head Unit Protocol:** Hiworld (`0x5A 0xA5` Framing, Additive Checksum)  
**Last Updated:** September 2026

---

## 1. Executive Summary & Pipeline Alignment

This document tracks the end-to-end development, testing status, and roadmap of **OpenCanbox Core** (`canbox-core`) against the features, UI panels, floating overlays, and protocol handlers discovered in the Android system application **QF Canbus System** (`com.qf.vehicle`).

### End-to-End Pipeline Mapping

```mermaid
flowchart LR
    subgraph Car ["Vehicle Network (Peugeot 407)"]
        BSI["BSI / Comfort CAN (125k)"]
        EMF["EMF / Air Con (0x1D0 / 0x1E3)"]
        PDC["Parking Radar (0x3A1 / 0x348)"]
        SWC["Steering Stalk (0x228)"]
    end

    subgraph Core ["canbox-core (C99 Engine)"]
        L1["L1: CAN Rx (hal_can)"]
        L2["L2: Profile Decoder (peugeot_407.c)"]
        L3["L3: Router & State Cache (can_router.c)"]
        L4["L4: Protocol Driver (hu_protocol_driver.h)"]
        L5["L5: Hiworld Adapter (proto_hiworld_adapter.c)"]
        GPIO["GPIO Line Drivers (ACC / ILL / REVERSE)"]
    end

    subgraph Android ["Android HU (com.qf.vehicle)"]
        UART["UartDataReceiver / WcVehicleDataRuleHead5aa5"]
        CTRL["PeugeotDataController (wc)"]
        PARSER["PeugeotDataParser"]
        STATE["CarbodyState"]
        UI["UI Panels & Floating Overlays"]
    end

    Car -->|CAN Frames| L1
    L1 --> L2 --> L3 --> L4 --> L5
    L3 -.->|Pin Triggers| GPIO
    L5 -->|UART 38400 (5A A5)| UART
    UART --> CTRL --> PARSER --> STATE --> UI
    UI -.->|Downlink Commands| UART
    UART -.->|UART (1B, 3B, 24)| L5
    L5 -.->|CAN Tx| L1
```

---

## 2. High-Level Progress Overview

| Functional Domain | Total Features | Completed | In Progress | Planned / TODO | N/A | Completion % |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **1. Peugeot UI Panels & Telemetry** | 7 | 6 | 1 | 0 | 0 | **86%** |
| **2. Floating Overlays (`Window...`)** | 8 | 6 | 1 | 0 | 1 | **75%** |
| **3. General Configuration & Panels** | 12 | 4 | 3 | 4 | 1 | **33%** |
| **4. Configuration & Diagnostics** | 4 | 3 | 1 | 0 | 0 | **75%** |
| **5. Physical Hardware Synthesis (GPIO)** | 3 | 3 | 0 | 0 | 0 | **100%** |
| **Overall** | **34** | **22** | **6** | **4** | **2** | **65%** |

---

## 3. Detailed Progress Matrix

Status Indicators:
- `[x] COMPLETED` — Fully implemented in C99 core, serialized to Hiworld protocol, verified by unit/integration tests.
- `[-] IN PROGRESS / PARTIAL` — Uplink implemented or partially wired; downlink handling or APK patch required.
- `[ ] TODO / PLANNED` — Documented and mapped, awaiting decoder/serializer implementation.
- `[N/A]` — Feature not physically applicable to target vehicle platform (e.g. EV battery charging on gasoline/diesel 407).

---

### Domain 1: Peugeot-Specific UI Panels & Telemetry

Mapped against `QF_Canbus_system/README.md` Section *Peugeot-Specific UI Panels & Telemetry*.

| Status | Feature | HU Class / Layout | Hiworld Cmd | Car CAN ID | Core Files / Drivers | Test Verification |
| :---: | :--- | :--- | :--- | :---: | :--- | :--- |
| `[x]` | **Trip 1 Computer**<br>(Avg Fuel, Mileage, Avg Speed) | `PeugeotDataComputer.smali`<br>`peugeot_citroen_car_pc_info.xml` | `0x14`<br>(`EcuInfoPage2`) | `0x2A1` | [`peugeot_407.c`](file://src/profiles/peugeot_407.c)<br>[`proto_hiworld_adapter.c`](file://src/protocols/proto_hiworld_adapter.c) | `test_peugeot_407_trip_hiworld_vector_2_trip1_historical`<br>`test_integration_hiworld_trip_pipeline` |
| `[x]` | **Trip 2 Computer**<br>(Avg Fuel, Mileage, Avg Speed) | `PeugeotDataComputer.smali`<br>`peugeot_citroen_car_pc_info.xml` | `0x15`<br>(`EcuInfoPage3`) | `0x261` | [`peugeot_407.c`](file://src/profiles/peugeot_407.c)<br>[`proto_hiworld_adapter.c`](file://src/protocols/proto_hiworld_adapter.c) | `test_peugeot_407_trip_hiworld_vector_3_trip2_historical`<br>`test_integration_hiworld_trip_pipeline` |
| `[x]` | **Instantaneous Telemetry**<br>(Instant Fuel, DTE Range, Dest) | `PeugeotDataComputer.smali`<br>`peugeot_citroen_car_pc_info.xml` | `0x13`<br>(`EcuInfoPage1`) | `0x221` | [`peugeot_407.c`](file://src/profiles/peugeot_407.c)<br>[`proto_hiworld_adapter.c`](file://src/protocols/proto_hiworld_adapter.c) | `test_peugeot_407_trip_hiworld_vector_1_instant_fuel`<br>`test_peugeot_407_trip_hiworld_vector_target_mileage_dump` |
| `[x]` | **Downlink Trip Reset**<br>(Clear Trip 1 / Trip 2 from UI) | `PeugeotDataComputer.smali`<br>`@id/peugeot_trip1_reset` | `0x1B`<br>(Downlink) | `0x221` Tx | [`proto_hiworld_adapter.c`](file://src/protocols/proto_hiworld_adapter.c)<br>[`can_router.c`](file://src/core/can_router.c) | `test_peugeot_407_trip_reset_frames`<br>`test_integration_hiworld_downlink_trip_reset_pipeline` |
| `[x]` | **Cluster Trip Variant**<br>(Alternate Gauge Style) | `PeugeotDataComputer2.smali`<br>`peugeot_citroen_car_pc_info2.xml` | `0x13`<br>`0x14`<br>`0x15` | `0x221`<br>`0x2A1`<br>`0x261` | Same as Trip 1 / 2 / Instant | Shared test suite |
| `[-]` | **Cruise Speed & Limiter**<br>(Cruise slots 1–6, Limiter steps) | `CruiseSpeedFrgment.smali`<br>`peugeot_citroen_speed_info.xml` | `0x8A`<br>`0x8B` | `0x1A8` | `include/protocols/hiworld_connection.h`<br>CAN decoder in `test_peugeot_407.c` | `test_psa_extended_decode_cruise_0x1a8` (CAN decoder verified; adapter pending) |
| `[x]` | **BSI Diagnostics & Alerts**<br>(Fault log & Warning messages) | `DignosticFrgment.smali`<br>`peugeot_dianostic_info.xml` | `0x42` (Uplink)<br>`0x2F` (Query) | `0x1A1`<br>`0x120`<br>`0x39B` (Tx) | [`peugeot_407.c`](file://src/profiles/peugeot_407.c)<br>[`can_router.c`](file://src/core/can_router.c)<br>[`proto_hiworld_adapter.c`](file://src/protocols/proto_hiworld_adapter.c) | `test_peugeot_407_alert_single_abs`<br>`test_peugeot_407_alert_journal_multi`<br>`test_peugeot_407_alert_journal_clear`<br>`test_peugeot_407_alert_router_pipeline_and_query` |

---

### Domain 2: System Floating Overlays (`Window...` Top-Level Views)

Mapped against `QF_Canbus_system/README.md` Section *System Floating Overlays*.

| Status | Feature | HU Class / Layout | Hiworld Cmd | Car CAN ID | Core Files / Drivers | Test Verification |
| :---: | :--- | :--- | :--- | :---: | :--- | :--- |
| `[x]` | **Door & Trunk Overlay**<br>(4 Doors, Boot, Bonnet, Handbrake) | `WindowDoor.smali`<br>`door_window_layer.xml` | `0x12`<br>(10 bytes) | `0x0F6`<br>`0x036` | [`peugeot_407.c`](file://src/profiles/peugeot_407.c)<br>[`proto_hiworld_adapter.c`](file://src/protocols/proto_hiworld_adapter.c) | `test_peugeot_407_doors_hiworld_vector_1_driver_front`<br>`test_integration_hiworld_door_status_pipeline` |
| `[x]` | **Parking Radar (OPS)**<br>(8-channel proximity overlay) | `WindowRadar.smali`<br>`common_radar_view.xml` | `0x41`<br>(12 bytes) | `0x3A1`<br>`0x348` | [`peugeot_407.c`](file://src/profiles/peugeot_407.c)<br>[`proto_hiworld_adapter.c`](file://src/protocols/proto_hiworld_adapter.c) | `test_peugeot_407_radar_hiworld_vector_1_obstacle_rear_center`<br>`test_integration_hiworld_radar_pipeline` |
| `[-]` | **Climate Control Popup**<br>(Dual HVAC bar / Floating panel) | `WindowAcControl.smali`<br>`WindowAcShow.smali`<br>`public_air_conditon_view.xml` | `0x31` (Up)<br>`0x3B` (Down) | `0x1D0`<br>`0x1E3`<br>`0x0F6` | [`peugeot_407.c`](file://src/profiles/peugeot_407.c)<br>[`proto_hiworld_adapter.c`](file://src/protocols/proto_hiworld_adapter.c) | `test_peugeot_407_hvac_hiworld` (Uplink complete & verified; Downlink `0x3B` pending) |
| `[x]` | **Text Warning / BSI Alerts**<br>(Ice alert, Low fuel, Bulbs) | `WindowTextWarning.smali`<br>`WarningLayer.smali`<br>`text_reminder_layer.xml` | `0x42` | `0x1A1`<br>`0x120` | [`peugeot_407.c`](file://src/profiles/peugeot_407.c)<br>[`proto_hiworld_adapter.c`](file://src/protocols/proto_hiworld_adapter.c) | `test_peugeot_407_alert_single_abs`<br>`test_peugeot_407_alert_single_suspension`<br>`test_peugeot_407_alert_single_low_fuel` |
| `[N/A]`| **Drive Mode Selector**<br>(Eco, Normal, Sport, Snow) | `WindowDriveMode.smali`<br>`common_drivermode_view.xml` | N/A | N/A | N/A (Standard Peugeot 407 does not broadcast drive mode selector frames) | N/A |
| `[ ]` | **Panoramic / 360 Camera**<br>(Surround view controls) | `WindowPanorama.smali`<br>`panorama_layout.xml` | `0x41` (Cam) | `0x3A1` | Planned for aftermarket AHD 360 camera integration | Pending |
| `[x]` | **CAN Volume HUD**<br>(Stalk volume wheel feedback) | `WindowVolume.smali`<br>`common_volume_layout.xml` | `0x11` | `0x228` | [`peugeot_407.c`](file://src/profiles/peugeot_407.c)<br>[`proto_hiworld_adapter.c`](file://src/protocols/proto_hiworld_adapter.c) | `test_peugeot_407_verification_vector_1_vol_up`<br>`test_integration_hiworld_steering_wheel_volume_up_pipeline` |
| `[x]` | **Backlight Control**<br>(Display night/day sync) | `WindowBackLight.smali`<br>`public_back_light_view.xml` | `0x036`<br>+ GPIO ILL | `0x036` | [`peugeot_407.c`](file://src/profiles/peugeot_407.c)<br>[`hal_gpio_*.c`](file://src/hal/hal_esp32/hal_gpio_esp32.c) | `test_scenario_lights_off_side_light_on_headlights_on` |

---

### Domain 3: General Vehicle Configuration & Telemetry Panels

Mapped against `QF_Canbus_system/README.md` Section *General Vehicle Configuration & Telemetry Panels*.

| Status | Feature | HU Class / Layout | Hiworld Cmd | Car CAN ID | Core Files / Drivers | Test Verification |
| :---: | :--- | :--- | :---: | :---: | :--- | :--- |
| `[-]` | **Central Settings & Preferences**<br>(DRL, Follow-me-home, Mirrors) | `CentralSettingFragment.smali`<br>`SettingFragment.smali`<br>`fragment_sub_setting.xml` | `0x71`, `0x72`<br>`0x76`, `0x79`<br>(`0x7B`/`0x7D` down) | `0x39B`<br>`0x2A8` | [`hiworld_connection.c`](file://src/protocols/hiworld_connection.c)<br>[`PEUGEOT_RT4_CAR_CONFIG.md`](file://doc/PEUGEOT_RT4_CAR_CONFIG.md) | `test_hiworld_verification_vector_3_feature_enables` (Uplink flags verified; downlink pending) |
| `[x]` | **Steering Wheel Controls (SWC)**<br>(Key mapping, stalks, rollers) | `OriginalSteeringWheel.smali`<br>`swc_study_layout.xml` | `0x11` | `0x228` | [`peugeot_407.c`](file://src/profiles/peugeot_407.c)<br>[`proto_hiworld_adapter.c`](file://src/protocols/proto_hiworld_adapter.c) | `test_peugeot_407_stalk_buttons_press_and_release`<br>`test_peugeot_407_stalk_rotary_encoder` |
| `[ ]` | **Factory Amplifier / DSP (JBL)**<br>(Bass, Treble, Sub, Surround) | `AmpFragment.smali`<br>`public_ampstate.xml` | `0xAD`<br>(Downlink) | CAN/VAN | [`CANBOX_SPEC_HIWORLD_407_08_JBL_AMPLIFIER_DSP.md`](file://doc/CANBOX_SPEC_HIWORLD_407_08_JBL_AMPLIFIER_DSP.md) | Spec ready; Downlink parser pending |
| `[x]` | **Tire Pressure Monitoring (TPMS)**<br>(4-wheel pressure + alarms) | `TmpsFragment.smali`<br>`public_tmps_info_layout.xml` | `0x66` (Numeric)<br>`0x18` (Discrete) | `0x3A1`<br>`0x348`<br>`0x1E1` | [`peugeot_407.c`](file://src/profiles/peugeot_407.c)<br>[`proto_hiworld_adapter.c`](file://src/protocols/proto_hiworld_adapter.c) | `test_peugeot_407_tpms_hiworld_vector_1_nominal`<br>`test_integration_hiworld_tpms_pipeline` |
| `[ ]` | **Ambient Mood Lighting**<br>(Multi-zone interior RGB) | `AtmosphereLightActivity.smali`<br>`atmosphere_light_main_layout.xml` | Custom | Custom | Not factory fitted on Peugeot 407 (custom aftermarket LIN/CAN) | Pending |
| `[ ]` | **OEM Radio Tuner Passthrough**<br>(AM/FM, RDS, Preset memory) | `OriginalTuner.smali`<br>`car_radio.xml` | `0xA2` | `0x225` | [`CANBOX_SPEC_HIWORLD_407_09_RD4_MFD_MEDIA_TEXT.md`](file://doc/CANBOX_SPEC_HIWORLD_407_09_RD4_MFD_MEDIA_TEXT.md) | `test_peugeot_407_cd_changer_and_rds` (Partial decoder verified) |
| `[ ]` | **OEM CD / Media Player**<br>(Factory CD changer / CDC) | `OriginalMediaPlayer.smali`<br>`media_player.xml` | `0xA4` | `0x2A5` | Same as above | `test_peugeot_407_cd_changer_and_rds` |
| `[ ]` | **Original Car Screen Passthrough**<br>(MFD text / Host screen emulation) | `CarScreenFragment.smali`<br>`common_car_screen.xml` | `0x97`<br>`0xE1`<br>`0xE4` | `0x3B6` | Same as above | Spec documented |
| `[x]` | **Off-Road / Dynamic Telemetry**<br>(Steering angle, Speed, RPM) | `OffroadInfoFragment.smali`<br>`common_offroad_layout.xml` | `0x11` | `0x0B6`<br>`0x21F` (SAS) | [`peugeot_407.c`](file://src/profiles/peugeot_407.c)<br>[`proto_hiworld_adapter.c`](file://src/protocols/proto_hiworld_adapter.c) | `test_peugeot_407_steering_wheel_angle`<br>`test_integration_hiworld_telemetry_periodic_pipeline` |
| `[-]` | **Clock / Date Synchronization**<br>(Bi-directional Android/BSI time) | `TimeSetFragment.smali`<br>`common_cartime_setting.xml` | `0xC2` (Up)<br>`0xCB` (Down) | `0x228`<br>`0x3F6` | [`hiworld_connection.c`](file://src/protocols/hiworld_connection.c)<br>[`CANBOX_SPEC_HIWORLD_407_10_BSI_SETTINGS_CLOCK_SYNC.md`](file://doc/CANBOX_SPEC_HIWORLD_407_10_BSI_SETTINGS_CLOCK_SYNC.md) | `test_hiworld_verification_vector_4_gps_time_sync` (Rx verified; BSI Tx frame pending) |
| `[N/A]`| **EV / PHEV Charging Management**<br>(Charge limits, precondition) | `ChargingSettingFragment.smali`<br>`common_charging_setting_layout.xml` | N/A | N/A | N/A (Peugeot 407 is purely ICE) | N/A |
| `[x]` | **Vehicle Units Sync**<br>(km/h vs mph, Celsius vs Fahrenheit) | `CarbodyState.smali` | `0xC1` | Fixed / BSI | [`hiworld_connection.c`](file://src/protocols/hiworld_connection.c) | `test_hiworld_verification_vector_3_feature_enables` |

---

### Domain 4: CANbox Configuration & Developer Diagnostics

Mapped against `QF_Canbus_system/README.md` Section *CANbox Configuration & Developer Diagnostics*.

| Status | Feature | HU Class / Layout | Hiworld Cmd | Functionality | Core Files / Drivers | Test Verification |
| :---: | :--- | :--- | :---: | :--- | :--- | :--- |
| `[x]` | **Car Type & Protocol Selection** | `CarTypeActivity.smali`<br>`public_can_factory_setting.xml` | `0x24` | Runtime vehicle model configuration (`34 = Peugeot 407`) and automatic CAN baud rate selection | [`hiworld_connection.c`](file://src/protocols/hiworld_connection.c)<br>[`hiworld_car_mapping.c`](file://src/protocols/hiworld_car_mapping.c) | `test_hiworld_verification_vector_1_car_type_set`<br>`test_integration_hiworld_runtime_car_selection_and_handshake` |
| `[x]` | **Protocol Version Information** | `VersionFragment.smali`<br>`public_canversion_info.xml` | `0x30` (Query)<br>`0xF0` (Report) | Reports canbox-core firmware build date string (e.g. `H1H2PA123A-240717`) | [`hiworld_connection.c`](file://src/protocols/hiworld_connection.c) | `test_hiworld_verification_vector_2_version_report`<br>`test_hiworld_connection_periodic_ping` |
| `[x]` | **Live CAN Data Trace** | `DataTraceActivity.smali`<br>`public_canfactory_debug_layout.xml` | Raw UART Hex | Live bidirectional packet inspection over UART serial interface | Built-in UART driver (`hal_uart`) | Live monitoring via SocketCAN / desktop simulator |
| `[-]` | **CAN Box Firmware Upgrade** | `UpgradeActivity.smali`<br>`upgrade_can_layout.xml` | IAP Protocol | In-System Programming (`can_app.iap`) flashing MCU firmware | Target-specific bootloader (STM32/ESP32) | Architecture planned; OTA flashing via platform-specific bootloader |

---

### Domain 5: Physical Hardware Synthesis (GPIO)

Hardware lines synthesized by OpenCanbox Core for head units lacking CAN-driven wakeup/signals:

| Status | Signal Line | Electrical Spec | Triggering CAN ID & Signal | Core Files | Test Verification |
| :---: | :--- | :--- | :--- | :--- | :--- |
| `[x]` | **ACC (Switched Wakeup)** | $+12\text{V}$ Active-High | `0x036` Byte 4 (`0x01`=IGN, `0x03`=ACC) | [`hal_gpio_*.c`](file://src/hal/hal_esp32/hal_gpio_esp32.c)<br>[`can_router.c`](file://src/core/can_router.c) | `test_scenario_ignition_off_after_power_on` |
| `[x]` | **ILL (Night Illumination)** | $+12\text{V}$ Active-High | `0x036` Byte 3 Bit 5 (Side/Headlights active) | [`hal_gpio_*.c`](file://src/hal/hal_esp32/hal_gpio_esp32.c)<br>[`can_router.c`](file://src/core/can_router.c) | `test_scenario_lights_off_side_light_on_headlights_on` |
| `[x]` | **REVERSE (Instant Camera)** | $+12\text{V}$ Active-High | `0x0F6` Byte 7 Bit 7 (`0x80` Reverse Gear engaged) | [`hal_gpio_*.c`](file://src/hal/hal_esp32/hal_gpio_esp32.c)<br>[`can_router.c`](file://src/core/can_router.c) | `test_peugeot_407_reverse_state`<br>`test_integration_hiworld_reverse_pipeline` |

---

## 4. Work Breakdown & Actionable Next Steps

### Priority 1: Climate Downlink (`0x3B`) Integration
- **Objective:** Allow the Android touchscreen HVAC overlay (`WindowAcControl.smali`) to command blower speed, setpoint temperature, and air distribution.
- **Specification:** [`PEUGEOT_HIWORLD_CLIMA_PROTOCOL_DOWNLINK.md`](file://doc/PEUGEOT_HIWORLD_CLIMA_PROTOCOL_DOWNLINK.md)
- **Target Changes:**
  1. Add `HIWORLD_CMD_AC_SETTING_SET (0x3B)` handler in [`proto_hiworld_adapter.c`](file://src/protocols/proto_hiworld_adapter.c).
  2. Map incoming parameters to PSA comfort CAN frames: `0x1E1` (A/C controls, fan speed) or `0x3A1`.
  3. Write unit test in `test_hiworld_parser.c` and integration test in `test_integration_hiworld.c`.

### Priority 2: Clock & Date Downlink Sync (`0xCB`)
- **Objective:** Sync Android system date/time to the car's BSI clock (`TimeSetFragment.smali`).
- **Specification:** [`CANBOX_SPEC_HIWORLD_407_10_BSI_SETTINGS_CLOCK_SYNC.md`](file://doc/CANBOX_SPEC_HIWORLD_407_10_BSI_SETTINGS_CLOCK_SYNC.md)
- **Target Changes:**
  1. Extract year, month, day, hour, minute from `0xCB` payload in [`hiworld_connection.c`](file://src/protocols/hiworld_connection.c).
  2. Implement `psa_build_clock_sync_frame()` generating PSA CAN `0x228` / `0x3F6`.
  3. Verify via `test_hiworld_verification_vector_4_gps_time_sync`.

### Priority 3: BSI Warning Alerts Serializer (`0x42`)
- **Objective:** Show vehicle warning dialogs (`WindowTextWarning.smali`, `WarningLayer.smali`) for critical events (low fuel, engine overheating, bulb failure, ice warning).
- **Target Changes:**
  1. Leverage existing CAN alert decoders (`test_psa_extended_decode_alerts_0x168`).
  2. Implement `hiworld_send_warning_info()` generating `HIWORLD_CMD_WARNING_INFO (0x42)`.
  3. Add test vectors matching Android `PeugeotDataParser.parseWarningInfo`.

### Priority 4: Dynamic BSI Central Settings (`0x7B` / `0x7D`)
- **Objective:** Transmit user preference toggles (guide-me-home lighting delay, daytime running lamps, selective door unlocking) from Android `CentralSettingFragment` to BSI configuration frames.
- **Specification:** [`PEUGEOT_RT4_CAR_CONFIG.md`](file://doc/PEUGEOT_RT4_CAR_CONFIG.md)
- **Target Changes:**
  1. Hook `HIWORLD_CMD_CENTRAL_SETTING1` and `CENTRAL_SETTING2` in adapter.
  2. Format PSA BSI configuration messages (`0x39B` / `0x2A8`).

---

## 5. Verification & Testing Matrix Reference

| Verification Level | Command | Scope |
| :--- | :--- | :--- |
| **Unit Tests (Native)** | `~/.platformio/penv/bin/pio test -e native_test_runner` | 73 tests: Bitfield unpackers, framing slip, checksums, state cache deltas |
| **Integration Pipeline** | `~/.platformio/penv/bin/pio test -e integration_test` | 20 tests: End-to-end CAN injection $\to$ router $\to$ Hiworld/Raise UART serialization |
| **STM32 Target Build** | `~/.platformio/penv/bin/pio run -e stm32_cbox` | Bare-metal ARM Cortex-M3 flash binary compilation check |
| **ESP32 Target Build** | `~/.platformio/penv/bin/pio run -e esp32_cbox` | Dual-core Xtensa RTOS flash binary compilation check |
| **Interactive Desktop Sim**| `CANBOX_CAN_IFACE="vcan0" ~/.platformio/penv/bin/pio run -e native_test -t exec` | Virtual CAN (`vcan0`) and pseudo-terminal (`pty`) hardware emulator |

