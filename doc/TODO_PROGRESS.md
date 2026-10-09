# OpenCanbox Core: Hiworld PSA Peugeot 407 Implementation Progress & Specification Tracker

**Target Vehicle Platform:** Peugeot 407 (PSA CAN2004 / AEE2004 Comfort Bus @ 125 kbps)  
**Target Head Unit Protocol:** Hiworld PSA (`0x5A 0xA5` Framing, Additive Checksum, UART 38,400 baud, 8N1)  
**Target Head Unit Application:** `com.qf.vehicle` (Peugeot Hiworld `wc` Driver / QF Canbus System)  
**Reference Protocol Specifications:** `commands_and_payload_structure.md`, `doc/CANBOX_SPEC_HIWORLD_407_*.md`, `doc/FIRMWARE_BUILD_AND_FLASH_GUIDE.md`  
**Firmware Core Project:** `canbox-core` (Pure C99 Embedded Automotive Firmware, Zero Heap)  
**Last Updated:** October 2026  

---

## 1. Executive Summary & Architecture Pipeline

This document tracks the complete development, verification status, and actionable roadmap of **OpenCanbox Core** (`canbox-core`) tailored **strictly and exclusively to the Hiworld PSA protocol on the Peugeot 407 platform**.

All decoders strictly operate on **verified PSA CAN2004 Comfort Bus frames (@ 125 kbps)**. Speculative CAN2010 definitions (e.g. radar on `0x260`/`0x270` or unconfirmed steering angle frames) and non-Hiworld protocol details (Raise, Bagoo, VAG) are strictly excluded from this tracker.

### End-to-End Pipeline Mapping

```mermaid
flowchart LR
    subgraph Car ["Peugeot 407 CAN2004 Comfort Bus (125 kbps)"]
        BSI_POW["0x036 (Ignition, Lighting, Handbrake)"]
        BSI_FAST["0x0B6 (Engine RPM, Speed)"]
        RADAR["0x0E1 (AAS Parking Radar Proximity)"]
        STALK["0x0F6 / 0x128 (Stalk Keys & Reverse Bit)"]
        ALERTS["0x1A1 (Real-Time Popups) / 0x120 (Journal)"]
        CLIM["0x1D0 / 0x1E3 (Dual HVAC Status)"]
        DOORS["0x220 (Doors, Boot, Bonnet Status)"]
        TRIP["0x221 / 0x2A1 / 0x261 (Trip Computer P0/P1/P2)"]
        TPMS["0x3A1 / 0x361 / 0x1E1 (Tire Pressures & Alarms)"]
        RADIO["0x225 / 0x2A5 / 0x3A6 (RD4 Tuner, RDS & CDC)"]
    end

    subgraph Core ["canbox-core (Pure C99 Engine)"]
        L1["L1: CAN Rx (hal_can)"]
        L2["L2: Profile Decoder (peugeot_407.c)"]
        L3["L3: Router & State Cache (can_router.c)"]
        L4["L4: Protocol Driver Interface (hu_protocol_driver.h)"]
        L5["L5: Hiworld Protocol Adapter (proto_hiworld_adapter.c & hiworld_connection.c)"]
        GPIO["Physical Hardware Synthesis (ACC / ILL / REVERSE)"]
    end

    subgraph HU ["Android Head Unit (com.qf.vehicle) / Host"]
        UART["UART Receiver (5A A5 Framing)"]
        CTRL["PeugeotDataController (wc)"]
        PARSER["PeugeotDataParser"]
        UI["UI Panels, Overlays & Floating Toasts"]
    end

    Car -->|CAN Frames @ 125k| L1
    L1 --> L2 --> L3 --> L4 --> L5
    L3 -.->|Pin Triggers| GPIO
    L5 -->|UART (38400 baud, 5A A5)| UART
    UART --> CTRL --> PARSER --> UI
    UI -.->|Downlink Commands (0x1B, 0x2F, 0x3B, 0xCB)| UART
    UART -.->|UART Downlink| L5
    L5 -.->|Downlink CAN Tx (0x221, 0x39B, 0x228)| L1
    L1 -.->|CAN Frames| Car
```

---

## 2. High-Level Hiworld PSA Progress Overview

| Functional Subsystem | Total Features | Completed `[x]` | In Progress `[-]` | Planned `[ ]` | N/A | Completion % |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **1. Protocol Connection & Handshake** | 6 | 6 | 0 | 0 | 0 | **100%** |
| **2. Steering Wheel Controls & Keys** | 3 | 3 | 0 | 0 | 0 | **100%** |
| **3. Dual-Zone Climate Control (HVAC)** | 3 | 2 | 1 | 0 | 0 | **67%** |
| **4. Doors, Body & Apertures** | 2 | 2 | 0 | 0 | 0 | **100%** |
| **5. Trip Computer & Telemetry** | 5 | 5 | 0 | 0 | 0 | **100%** |
| **6. Parking Radar (OPS / AAS)** | 2 | 2 | 0 | 0 | 0 | **100%** |
| **7. Tire Pressure Monitoring (TPMS)** | 3 | 2 | 0 | 1 | 0 | **67%** |
| **8. BSI Vehicle Alerts & Diagnostics** | 5 | 5 | 0 | 0 | 0 | **100%** |
| **9. Central Personalization & Settings** | 4 | 2 | 2 | 0 | 0 | **50%** |
| **10. Date & Time Synchronization** | 2 | 1 | 1 | 0 | 0 | **50%** |
| **11. RD4 Audio & Multimedia Passthrough**| 4 | 3 | 0 | 1 | 0 | **75%** |
| **12. Dynamic Guidelines & Steering SAS**| 1 | 0 | 0 | 1 | 0 | **0%** (Pending CAN Frame) |
| **13. Powertrain & Hardware GPIO Synthesis**| 4 | 4 | 0 | 0 | 0 | **100%** |
| **14. Dynamic CAN ID Pre-filtering & Router**| 1 | 1 | 0 | 0 | 0 | **100%** |
| **Overall** | **45** | **36** | **4** | **3** | **2** | **80%** |

---

## 3. Hiworld Master Command Implementation Matrix

Cross-referenced directly against `commands_and_payload_structure.md` and Android `com.qf.vehicle` / `wc` driver:

### 3.1 Inbound Telemetry Commands (CAN Box $\to$ Host / Android HU)

| Status | CMD | Name / Category | Wire LEN | Vehicle CAN ID | Core Files / Drivers | Test Verification & Notes |
| :---: | :---: | :--- | :---: | :---: | :--- | :--- |
| `[x]` | `0xF0` | **Version String Report** | Variable | N/A (Firmware) | `src/core/canbox_version.c`<br>`src/protocols/hiworld_connection.c`<br>`src/protocols/proto_hiworld_adapter.c` | `test_hiworld_verification_vector_2_version_report`<br>`test_canbox_embedded_firmware_version`<br>Dynamically embedded `CANBOX-CORE-V<YYYYMMDD.hhmmss>` generated at build time via `tools/generate_version.py` (buffer expanded to 64 bytes) |
| `[x]` | `0x11` | **Steering Wheel Keys (SWC)** | `0x0A` (10) | `0x21F`<br>`0x221`<br>`0x0F6` | `src/profiles/peugeot_407.c`<br>`src/protocols/proto_hiworld_adapter.c` | `test_peugeot_407_stalk_0x21f_buttons_and_rotary`<br>`test_peugeot_407_stalk_tip_0x221_trip_button`<br>`test_integration_hiworld_native_stalk_0x21f_pipeline`<br>`test_integration_hiworld_stalk_tip_0x221_trip_pipeline`<br>`test_peugeot_407_stalk_buttons_press_and_release`<br>`test_peugeot_407_stalk_rotary_encoder`<br>`test_integration_hiworld_steering_wheel_volume_up_pipeline`<br>Spec: `doc/CANBOX_SPEC_HIWORLD_407_01_STEERING_STALK_KEYS.md` |
| `[x]` | `0x12` | **Doors & Apertures Status** | `0x0B` (11) | `0x220`<br>`0x036` | `src/profiles/peugeot_407.c`<br>`src/protocols/proto_hiworld_adapter.c` | `test_peugeot_407_doors_hiworld_vector_1_driver_front`<br>`test_integration_hiworld_door_status_pipeline`<br>10 payload bytes: FL, FR, RL, RR, trunk, bonnet, handbrake |
| `[x]` | `0x13` | **Instantaneous Trip Telemetry** | `0x0B` (11) | `0x221` | `src/profiles/peugeot_407.c`<br>`src/protocols/proto_hiworld_adapter.c` | `test_peugeot_407_trip_hiworld_vector_1_instant_fuel`<br>`test_integration_hiworld_trip_pipeline`<br>Instant fuel ($0.1\text{ L/100km}$), DTE range, target distance |
| `[x]` | `0x14` | **Trip 1 Computer Telemetry** | `0x07` (7) | `0x2A1` | `src/profiles/peugeot_407.c`<br>`src/protocols/proto_hiworld_adapter.c` | `test_peugeot_407_trip_hiworld_vector_2_trip1_historical`<br>Trip 1 avg fuel, avg speed, trip distance |
| `[x]` | `0x15` | **Trip 2 Computer Telemetry** | `0x07` (7) | `0x261` | `src/profiles/peugeot_407.c`<br>`src/protocols/proto_hiworld_adapter.c` | `test_peugeot_407_trip_hiworld_vector_3_trip2_historical`<br>Trip 2 avg fuel, avg speed, trip distance |
| `[x]` | `0x21` | **Console Panel Button Telemetry**| `0x02` (2) | `0x3E5` only | `src/profiles/peugeot_407.c` (`psa_decode_console_0x3e5_ex`)<br>`src/protocols/proto_hiworld_adapter.c` | `test_peugeot_407_console_0x3e5_buttons`<br>`test_peugeot_407_console_0x3e5_edge_cases`<br>`test_integration_hiworld_console_0x3e5_full_pipeline`<br>`test_integration_hiworld_console_0x3e5_ok_pipeline`<br>Spec: `doc/CANBOX_SPEC_HIWORLD_407_01_STEERING_STALK_KEYS.md` section 3.3<br>Verified KeyIDs (OEM Canbox): AUDIO `0x31`, TRIP `0x40`, CLIM `0x28`, DARK `0x07`, OK `0x24`, ESC `0x25`, MENU `0x2E`, UP `0x17`, DOWN `0x18`, LEFT `0x19`, RIGHT `0x1A`. `0x3E5` 2-bit fields: B0 MENU[7:6] CLIM[1:0]; B1 TRIP[7:6] AUDIO[1:0]; B2 OK[7:6] ESC[5:4] DARK[3:2]; B5 UP[7:6] DOWN[5:4] RIGHT[3:2] LEFT[1:0]. Captured in `can_log_buttons*.log`: AUDIO, TRIP, CLIM, DARK, MENU, OK, ESC, UP; DOWN/LEFT/RIGHT bits from simulator docs (not yet captured). TEL decoded (B0[5:4] -> Hiworld `0x05` PHONE, unverified on vehicle, not yet captured); MODE pending (no KeyID). `0x167`/`0x0DF` decoders removed. CMD `0x22` and OEM idle `0x11` frame pending. |
| `[x]` | `0x84` | **RD4 Radio / Audio Source State** | `0x0E` (14) | `0x165`<br>`0x225`<br>`0x2A5`<br>`0x265` | `src/profiles/peugeot_407.c` (`psa_rd4_process_can_0x165`, `psa_rd4_process_can_0x225`, `psa_rd4_process_can_0x2a5`, `build_hiworld_radio_state`)<br>`src/protocols/proto_hiworld_adapter.c`<br>`src/core/can_router.c` | `test_peugeot_407_rd4_vector_1_fm_tuner`<br>`test_peugeot_407_rd4_source_and_wavebands`<br>`test_peugeot_407_rd4_frequency_and_ta_stability`<br>`test_integration_hiworld_rd4_radio_tuner_pipeline`<br>`test_integration_hiworld_rd4_downlink_resume_queries`<br>Spec: `doc/CANBOX_SPEC_HIWORLD_407_09_RD4_MFD_MEDIA_TEXT.md`<br>Decodes Source (`0x165`), Band/Freq/Preset/RDS (`0x225`), PS Name (`0x2A5`), TA (`0x265`); Little-Endian frequency wire format aligned with Android APK (`PeugeotDataParser.smali`); TA indicator stabilized across `0x225`/`0x265` frames (fixes flickering) |
| `[x]` | `0x86` | **RDS Dynamic RadioText Marquee** | $1\dots 64$ | `0x0A4` | `src/profiles/peugeot_407.c` (`psa_rd4_process_can_0x0a4`, `build_hiworld_radio_text`)<br>`src/protocols/proto_hiworld_adapter.c`<br>`src/core/can_router.c` | `test_peugeot_407_rd4_vector_2_isotp_radiotext`<br>`test_integration_hiworld_rd4_radiotext_pipeline`<br>Spec: `doc/CANBOX_SPEC_HIWORLD_407_09_RD4_MFD_MEDIA_TEXT.md`<br>ISO-TP reassembly strips prefix `10 00 00 00`; routes to `tv_radio_text` |
| `[x]` | `0x97` | **CD Changer (CDC) Playback State** | `0x0B` (11) | `0x3A6` | `src/profiles/peugeot_407.c` (`psa_rd4_process_can_0x3a6`, `build_hiworld_media_state`)<br>`src/protocols/proto_hiworld_adapter.c`<br>`src/core/can_router.c` | `test_peugeot_407_cd_changer_and_rds`<br>`test_peugeot_407_rd4_vector_3_cd_changer`<br>`test_integration_hiworld_rd4_cd_changer_pipeline`<br>`test_integration_hiworld_rd4_downlink_resume_queries`<br>Disc index ($1\dots 6$), loaded mask, track, elapsed mm:ss, play modes, play status decoded; Hiworld `0x97` adapter active |
| `[x]` | `0x31` | **Dual-Zone Climate Status** | `0x0D` (13) | `0x1D0`<br>`0x1E3` | `src/profiles/peugeot_407.c`<br>`src/protocols/proto_hiworld_adapter.c` | `test_peugeot_407_hvac_hiworld`<br>Power, AC, auto, dual, defrost, fan (0..8), temps, AQS, airflow |
| `[x]` | `0x41` | **Parking Radar Distance (OPS)** | `0x0D` (13) | `0x0E1` | `src/profiles/peugeot_407.c`<br>`src/protocols/proto_hiworld_adapter.c` | `test_peugeot_407_radar_hiworld_vector_1_obstacle_rear_center`<br>`test_integration_hiworld_radar_pipeline`<br>8 sensor bars (0..4 distance level); strictly CAN2004 `0x0E1` |
| `[x]` | `0x42` | **PSA BSI Vehicle Alert DTCs** | `0x03` / `0x19` | `0x1A1`<br>`0x120` | `src/profiles/peugeot_407.c`<br>`src/protocols/proto_hiworld_adapter.c` | `test_peugeot_407_alert_single_abs`<br>`test_peugeot_407_alert_journal_multi`<br>Matches Section 5 DTC table; controlled via compile flag |
| `[x]` | `0x66` | **TPMS Direct Numeric Pressures** | `0x07` (7) | `0x3A1`<br>`0x361` | `src/profiles/peugeot_407.c`<br>`src/protocols/proto_hiworld_adapter.c` | `test_peugeot_407_tpms_hiworld_vector_1_nominal`<br>`test_integration_hiworld_tpms_pipeline`<br>4-wheel tire pressures in bar / deci-bar |
| `[x]` | `0x71` | **Feature Enable Availability 1** | `0x03` (3) | Fixed / Config | `src/protocols/hiworld_connection.c` | `test_hiworld_verification_vector_3_feature_enables`<br>Reports supported vehicle feature bitmask (`0x60`, `0x12`) |
| `[x]` | `0x72` | **Feature Enable Availability 2** | `0x09` (9) | Fixed / Config | `src/protocols/hiworld_connection.c` | `test_hiworld_verification_vector_3_feature_enables`<br>Reports supported feature bitmask (`0x00, 0x01, 0x80...`) |
| `[x]` | `0x76` | **Central Convenience State 1** | `0x03` (3) | Fixed / BSI | `src/protocols/hiworld_connection.c` | `test_hiworld_verification_vector_3_feature_enables` |
| `[x]` | `0x79` | **Central Convenience State 2** | `0x09` (9) | Fixed / BSI | `src/protocols/hiworld_connection.c` | `test_hiworld_verification_vector_3_feature_enables`<br>Reports dual-zone configuration & vehicle settings |
| `[-]` | `0x7B` | **Convenience & Exterior Settings**| `0x03` (3) | `0x39B`<br>`0x2A8` | `doc/CANBOX_SPEC_HIWORLD_407_11_CAR_CONFIGURATION_OPTIONS.md` | Uplink state decoded; Downlink command mapping pending |
| `[ ]` | `0x7D` | **TPMS Calibration State** | `0x03` (3) | `0x1E1` | `src/protocols/proto_hiworld_adapter.c` | Documented in `commands_and_payload_structure.md` |
| `[-]` | `0x8A` | **Speed Limiter Presets Feedback** | `0x0B` (11) | `0x1A8` | `src/profiles/peugeot_407.c` | `test_psa_extended_decode_cruise_0x1a8`<br>CAN decoder verified; serializer to Hiworld pending |
| `[-]` | `0x8B` | **Cruise Control Presets Feedback**| `0x0B` (11) | `0x1A8` | `src/profiles/peugeot_407.c` | `test_psa_extended_decode_cruise_0x1a8`<br>CAN decoder verified; serializer to Hiworld pending |
| `[-]` | `0x8C` | **Vehicle Clock Sync Feedback** | `0x08` (8) | `0x228` | `src/protocols/hiworld_connection.c` | `test_hiworld_verification_vector_4_gps_time_sync` |
| `[x]` | `0xC1` | **System Units Broadcast** | `0x04` (4) | Fixed / Config | `src/protocols/hiworld_connection.c` | `test_hiworld_verification_vector_3_feature_enables`<br>Units: L/100km, km/h, Celsius (`0x28, 0x20, 0x00`) |
| `[ ]` | `0xC2` | **Date & Time Broadcast** | `0x08` (8) | `0x228` | `src/protocols/hiworld_connection.c` | Uplink vehicle clock broadcast |
| `[-]` | `0xCB` | **Extended Powertrain Telemetry** | `0x0B` (11) | `0x0B6`<br>`0x165` | `src/profiles/peugeot_407.c` | Engine RPM & Speed decoded in `peugeot_407.c`; packaging to `0xCB` planned |
| `[x]` | `0xEA` | **Custom Extended CAN Alerts** | `0x06` / Variable | `0x1A1`<br>`0x120` | `src/profiles/peugeot_407.c`<br>`src/protocols/proto_hiworld_adapter.c` | `test_peugeot_407_custom_cockpit_check_sequence`<br>`test_peugeot_407_custom_summary_table_and_empty_clearance`<br>15-bit PSA DTCs, severity, door masks, CHECK sequence |
| `[x]` | `0xFF` | **Protocol Heartbeat Ping** | `0x02` (2) | Internal Timer | `src/protocols/proto_hiworld_adapter.c` | Periodic link keep-alive (`5A A5 01 FF 01 00`), throttled to 1 Hz (1000 ms / 10 ticks); `test_integration_hiworld_telemetry_periodic_pipeline` |

---

### 3.2 Outbound Downlink & Control Commands (Host / Android HU $\to$ CAN Box)

| Status | CMD | Name / Category | Wire LEN | Vehicle CAN Tx | Core Files / Drivers | Test Verification & Notes |
| :---: | :---: | :--- | :---: | :---: | :--- | :--- |
| `[x]` | `0x1B` | **Trip Computer Page Reset** | `0x03` / `0x05` | `0x221` (Tx) | `src/protocols/proto_hiworld_adapter.c`<br>`src/core/can_router.c` | `test_peugeot_407_trip_reset_frames`<br>`test_integration_hiworld_downlink_trip_reset_pipeline`<br>Resets Trip 1 or Trip 2 from UI button |
| `[x]` | `0x24` | **Car Model & Baud Selection** | `0x02` / `0x03` | N/A (Config) | `src/protocols/hiworld_connection.c`<br>`src/protocols/hiworld_car_mapping.c` | `test_hiworld_verification_vector_1_car_type_set`<br>`test_integration_hiworld_runtime_car_selection_and_handshake`<br>Configures Peugeot 407 (ID 34) & 125 kbps; immediate ACK (`5A A5 01 FF 24 23`) and redundant 70B config burst suppression on active model |
| `[x]` | `0x2F` | **Diagnostic Journal Query** | `0x02` (2) | `0x39B` (Tx) | `src/protocols/proto_hiworld_adapter.c`<br>`src/core/can_router.c` | `test_peugeot_407_custom_downlink_0x2f_response`<br>Android requests fault journal refresh (`forwardType 0x2F`) |
| `[x]` | `0x30` | **Firmware Version Query** | `0x02` (2) | N/A (Internal) | `src/protocols/hiworld_connection.c` | Handled in `hiworld_connection.c`; replies with `0xF0` |
| `[-]` | `0x3B` | **Climate Control Downlink** | `0x0B` (11) | `0x1E1` (Tx) | `doc/PEUGEOT_HIWORLD_CLIMA_PROTOCOL_DOWNLINK.md` | Downlink parser architecture ready; Priority 1 task |
| `[x]` | `0xCB` | **GPS Date & Time Set** | `0x07` (7) | `0x228` (Tx) | `src/protocols/hiworld_connection.c` | `test_hiworld_verification_vector_4_gps_time_sync`<br>Decodes Android GPS clock & transmits PSA `0x228` |
| `[-]` | `0x71` | **Personalization Change Request**| `0x02` / `0x03` | `0x39B` (Tx) | `doc/PEUGEOT_RT4_CAR_CONFIG.md` | Requests BSI feature toggle (DRL, follow-me-home) |
| `[-]` | `0x7B` | **Central Setting Request 1** | `0x03` (3) | `0x39B` (Tx) | `doc/CANBOX_SPEC_HIWORLD_407_11_CAR_CONFIGURATION_OPTIONS.md` | Convenience parameter write |
| `[-]` | `0x7D` | **Central Setting Request 2** | `0x03` (3) | `0x39B` (Tx) | `doc/CANBOX_SPEC_HIWORLD_407_11_CAR_CONFIGURATION_OPTIONS.md` | Convenience parameter write |
| `[ ]` | `0x76` | **Overspeed Warning Limit Set** | `0x03` (3) | `0x39B` (Tx) | Documented in `commands_and_payload_structure.md` | Speed alarm threshold configuration |
| `[ ]` | `0x79` | **Navigation Guidance Display** | `0x08` (8) | `0x3B6` (Tx) | `doc/CANBOX_SPEC_HIWORLD_407_09_RD4_MFD_MEDIA_TEXT.md` | Turn-by-turn arrows to vehicle MFD |
| `[-]` | `0x81` | **Cruise Presets Group 1 Setup** | `0x0A` (10) | `0x39B` (Tx) | Documented in `commands_and_payload_structure.md` | Uploads speed memory presets 1..6 |
| `[x]` | `0x85` | **Historical Alarm Replay Query**| `0x02` (2) | `0x39B` (Tx) | `src/protocols/proto_hiworld_adapter.c` | Triggers alert journal replay (mapped along with `0x2F`) |
| `[x]` | `0x94` | **Software Reset / Link Ping** | `0x01` (1) | N/A (Internal) | `src/protocols/hiworld_connection.c` | Link liveness check / software reboot |
| `[ ]` | `0xC2` | **MCU Firmware Flash Transfer** | `0x0A` (10) | N/A (Bootloader)| Target-specific bootloader (STM32/ESP32 IAP) |

---

## 4. Subsystem Deep-Dive: Implementation & Verification

### 4.1 Steering Column Stalk & Console Keys (`0x11` / `0x21`)
- **CAN Ground Truth (`dump_2026-10-06_16-56-18.log`, `dump_2026-10-06_17-06-48.log`, `dump_2026-10-06_17-13-24.log`, `dump_2026-10-06_17-18-39.log` & `doc/CANBOX_SPEC_HIWORLD_407_01_STEERING_STALK_KEYS.md`):**
  - **COM2000 Stalk (`0x21F`, dlc 3, 50-100 ms) $\to$ CMD `0x11` (`CarBaseInfo`):**
    - Byte 0: Vol+ (`0x08` $\to$ Hiworld `0x01`), Vol- (`0x04` $\to$ Hiworld `0x02`), Mute chord (`0x0C` $\to$ Hiworld `0x03`), Source (`0x02` $\to$ Hiworld `0x0B`), Next Track (`0x80` $\to$ Hiworld `0x08`), Prev Track (`0x40` $\to$ Hiworld `0x09`).
    - Byte 1: Rotary thumbwheel (*molette*) counter (Roll Up $+1 \to$ Hiworld `0x12`, Roll Down $-1 \to$ Hiworld `0x11`).
    - Byte 2: Constant `0x00`.
    - Stalk tips (`0x221` dlc 7): Byte 0 Bit 3 (`0x08`): Trip Computer button (`0xC8` pressed, `0xC0` idle $\to$ Hiworld CMD `0x11` KeyCode `0x14`).
  - **RD4 / RD5 Center Console & Fascia (`0x3E5` only) $\to$ CMD `0x21` (`ControlPanelKey`):**
    - Broadcast as 2-byte payload `[KeyID, PressState]` (`5A A5 02 21 [KeyID] [State] [CS]`).
    - Release packet: `5A A5 02 21 00 00 22`.
    - Verified KeyIDs (OEM Canbox log): AUDIO `0x31`, TRIP `0x40`, CLIM `0x28`, DARK `0x07`, OK `0x24`, ESC `0x25`, MENU `0x2E`, UP `0x17`, DOWN `0x18`, LEFT `0x19`, RIGHT `0x1A`.
    - Code now matches the corrected spec (0x3E5 only; 0x167/0x0DF decoders removed).
    - Raw `0x3E5` evidence in `can_log_buttons.log`: AUDIO=B1[1:0], TRIP=B1[7:6], CLIM=B0[1:0], DARK=B2[3:2]. MENU=B0[7:6], OK=B2[7:6], ESC=B2[5:4] verified in `can_log_buttons1.log`; UP=B5[7:6] verified in `can_log_buttons2.log`. DOWN/LEFT/RIGHT bit positions from simulator docs only (not yet captured). `0x0DF` toggles every ~500 ms independent of keys.
    - Directional keys (UP, DOWN, LEFT, RIGHT): OEM Canbox emits KeyIDs `0x17..0x1A`; B5 field layout from simulator docs. Earlier dumps showed no arrow traffic on CAN, UP is now verified on CAN (B5[7:6]); DOWN/LEFT/RIGHT remain **unverified** until captured.
    - RD4/RD5 internal keys: SOURCE and BAND toggle internal tuner modes on the radio without sending panel pulses.
- **Implemented Decoders:**
  - COM2000 Stalk (`0x21F`): `psa_decode_stalk_0x21f_ex()` and `psa_decode_stalk_0x21f()` in `src/profiles/peugeot_407.c`, registered in `profile_psa.c`.
  - COM2000 Stalk Tip Trip (`0x221`): `psa_decode_stalk_tip_0x221_ex()` and `psa_decode_stalk_tip_0x221()` in `src/profiles/peugeot_407.c`, registered in `profile_psa.c`.
  - ~~RD4/RD5 Fascia Display Mode & Trip (`0x167`)~~: removed (not a key source).
  - RD4/RD5 Fascia Multi-Key Frame (`0x3E5`): `psa_decode_console_0x3e5_ex()` / `psa_decode_console_0x3e5()` in `src/profiles/peugeot_407.c`, registered in `profile_psa.c`. Table-driven 2-bit-field decoder (AUDIO, TRIP, CLIM, DARK, MENU, OK, ESC, UP, DOWN, LEFT, RIGHT).
  - Legacy Aftermarket Stalk (`0x0F6`): `psa_stalk_process_can_ex()` in `src/profiles/peugeot_407.c`.
  - Dispatches: Vol+, Vol-, Mute, Next, Prev, Source, Rotary scroll up/down pulses, Stalk Tip Trip (`WHEEL_KEY_TRIP`), and Fascia buttons (DARK `0x40`, TRIP `0x07`, OK `0x24`, CLIM `0x31`, AUDIO `0x28`, MENU `0x2E`, ESC `0x25`).
- **Hiworld Protocol Serialization:**
  - Stalk keys packed into CMD `0x11` (`HIWORLD_CMD_CAR_BASE_INFO` / `CarBaseInfo`), 10-byte OEM payload layout `[0x23, 0x00, KeyCode, PressState, 0x00, 0x0A, 0x00, 0x00, 0x5E, 0x22]` with automatic pulse release for scroll keys.
  - Console keys packed into CMD `0x21` (`HIWORLD_CMD_CONTROL_PANEL_KEY` / `ControlPanelKey`), 2-byte payload `[KeyID, PressState]` with release frame `5A A5 02 21 00 00 22`.
  - Unit tests: `test_peugeot_407_stalk_0x21f_buttons_and_rotary`, `test_peugeot_407_stalk_tip_0x221_trip_button`, `test_peugeot_407_console_0x3e5_buttons`, `test_peugeot_407_console_0x3e5_edge_cases`, `test_peugeot_407_stalk_buttons_press_and_release`, `test_peugeot_407_stalk_rotary_encoder`.
  - Integration pipelines: `test_integration_hiworld_native_stalk_0x21f_pipeline`, `test_integration_hiworld_stalk_tip_0x221_trip_pipeline`, `test_integration_hiworld_console_0x3e5_full_pipeline`, `test_integration_hiworld_console_0x3e5_ok_pipeline`, `test_integration_hiworld_steering_wheel_volume_up_pipeline`.

### 4.2 Dual-Zone Climate Control (HVAC) (`0x31` Uplink, `0x3B` Downlink)
- **CAN Ground Truth:**
  - `0x1D0` (dlc 4..8, 100 ms): Primary climate state (compressor, power, auto, dual, defrost, fan speed, setpoint temperatures).
  - `0x1E3` (dlc 2..8, 100 ms): Airflow distribution modes and independent Air Quality Sensor (AQS auto-intake).
- **Implemented Decoders:**
  - `psa_hvac_process_can_0x1d0()` & `psa_hvac_process_can_0x1e3()` in `src/profiles/peugeot_407.c`.
  - Air Quality Sensor fix: Proper bit isolation preventing manual fan changes from stomping AQS state.
- **Hiworld Protocol Serialization:**
  - Uplink: CMD `0x31` (`HIWORLD_CMD_CAR_AC_STATE`), 12-byte payload.
  - Downlink (`0x3B`): Downlink architecture documented in `doc/PEUGEOT_HIWORLD_CLIMA_PROTOCOL_DOWNLINK.md`, parser hook planned as Priority 1.
  - Unit test: `test_peugeot_407_hvac_hiworld`.

### 4.3 Doors, Body Status & Apertures (`0x12`)
- **CAN Ground Truth:**
  - `0x220` (dlc 1..8, 100 ms): Individual status for driver door, passenger door, rear-left, rear-right, boot/trunk, bonnet/hood.
  - `0x036` (dlc 2..5): Handbrake status and ignition state.
  - `0x0F6` (dlc 8): Reverse gear bit (`data[7] & 0x80`).
- **Implemented Decoders:**
  - `psa_decode_doors_0x220()` in `src/profiles/peugeot_407.c`.
- **Hiworld Protocol Serialization:**
  - Serialized to CMD `0x12` (`HIWORLD_CMD_DOOR_WINDOW_STATE`), 10 payload bytes matching OEM canbox capture (`[0x00, 0x04, b2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03]`).
  - Unit tests: `test_peugeot_407_doors_hiworld_vector_1_driver_front`, `test_peugeot_407_doors_all_open_with_handbrake`.
  - Integration pipeline: `test_integration_hiworld_door_status_pipeline`.

### 4.4 Trip Computer & Instantaneous Telemetry (`0x13`, `0x14`, `0x15`, `0x1B`)
- **CAN Ground Truth:**
  - `0x221` (dlc 7, 250 ms): Instantaneous fuel consumption ($0.1\text{ L/100km}$), distance to empty (DTE range), remaining destination distance.
  - `0x2A1` (dlc 5..8, 250 ms): Trip 1 accumulated distance, average fuel consumption, average speed.
  - `0x261` (dlc 5..8, 250 ms): Trip 2 accumulated distance, average fuel consumption, average speed.
  - `0x0B6` (dlc 8, 20 ms): Vehicle speed fallback & engine RPM.
- **Implemented Decoders & Downlink Reset:**
  - Decoders: `psa_decode_trip_0x221_profile()`, `psa_decode_trip1_0x2a1_profile()`, `psa_decode_trip2_0x261_profile()`.
  - Downlink Reset: CMD `0x1B` from Android UI received in `src/protocols/proto_hiworld_adapter.c`, triggers `can_router_reset_trip(1/2)` emitting native PSA CAN reset frames.
- **Hiworld Protocol Serialization:**
  - CMD `0x13` (`EcuInfoPage1`): Instantaneous telemetry.
  - CMD `0x14` (`EcuInfoPage2`): Trip 1 statistics.
  - CMD `0x15` (`EcuInfoPage3`): Trip 2 statistics.
  - Unit tests: `test_peugeot_407_trip_hiworld_vector_1_instant_fuel`, `test_peugeot_407_trip_hiworld_vector_2_trip1_historical`, `test_peugeot_407_trip_hiworld_vector_3_trip2_historical`, `test_peugeot_407_trip_reset_frames`.
  - Integration pipeline: `test_integration_hiworld_trip_pipeline`, `test_integration_hiworld_downlink_trip_reset_pipeline`.

### 4.5 Ultrasonic Parking Radar (OPS / AAS) (`0x41`)
- **CAN Ground Truth:**
  - `0x0E1` (dlc 6..8, 100 ms): Strictly CAN2004 AAS parking radar frame.
  - *Architectural Isolation Rule:* CAN ID `0x260` is `MSG_BSI_INF_PROFILS` (BSI User Profiles: Profile 1, Profile 2, Manufacturer; 8 bytes, 250 ms) on CAN2004, NOT parking sensors (which only use `0x260` on CAN2010). Frame `0x260` is completely detached from radar decoders.
- **Implemented Decoders:**
  - `psa_decode_radar_0x0e1_profile()` in `src/profiles/peugeot_407.c`.
  - Maps 4 front zones and 4 rear zones to normalized distance levels ($0 \dots 4$).
- **Hiworld Protocol Serialization:**
  - CMD `0x41` (`HIWORLD_CMD_CAR_RADAR_STATE`), 12-byte payload.
  - Unit tests: `test_peugeot_407_radar_hiworld_vector_1_obstacle_rear_center`, `test_peugeot_407_radar_hiworld_vector_2_quiescent_all_clear`.
  - Integration pipeline: `test_integration_hiworld_radar_pipeline`.

### 4.6 Direct Tire Pressure Monitoring (TPMS) (`0x66`, `0x18`)
- **CAN Ground Truth:**
  - `0x3A1` (dlc 4..8, 1000 ms): Direct numeric tire pressures for FL, FR, RL, RR.
  - `0x361` (dlc 8, 1000 ms): Direct TPMS numeric readings and alarm masks.
  - `0x1E1` (dlc 4..8, 1000 ms): Discrete wheel status states (OK, under-inflation, puncture, missing sensor).
- **Implemented Decoders:**
  - Decoupled state updates: numeric pressures and discrete alarm states are tracked and dispatched independently.
- **Hiworld Protocol Serialization:**
  - CMD `0x66` (`HIWORLD_CMD_TPMS_NUMERIC`): 4-wheel pressures in bar (deci-bar).
  - CMD `0x18` (`HIWORLD_CMD_TPMS_DISCRETE`): 4-wheel alarm states.
  - Unit tests: `test_peugeot_407_tpms_hiworld_vector_1_nominal`, `test_peugeot_407_tpms_hiworld_0x3a1_pressures`, `test_peugeot_407_tpms_hiworld_0x1e1_status_enum`.
  - Integration pipeline: `test_integration_hiworld_tpms_pipeline`.

### 4.7 BSI Diagnostics, Popups & Alert Journal (`0x42`, `0xEA`, `0x2F`, `0x85`)
- **CAN Ground Truth:**
  - `0x1A1` (dlc 8, 100 ms): Immediate vehicle warning popup (`MSG_BSI_CDE_PTR_MESSAGE`).
  - `0x120` (dlc 8, 500 ms): Multi-fault diagnostic journal (`MSG_JOURNAL_ALERTES`) transported via 3-block multiplexing (`0x7C`, `0xBC`, `0xFC`) or fallback ISO-TP.
  - `0x39B` (dlc 8, Tx): Downlink diagnostic query request (`[0x01, 0x01, ...]`).
- **Implemented Decoders & Disambiguation:**
  - Canonical code translation (`psa_can_alarm_id_to_hiworld_code()`) to resolve collision on code `0x0008` (braking fault vs doors).
  - Door mask guard (`door_mask != 0 && door_mask != 0xFF`) to avoid false door open toasts on diagnostic check cycles.
  - 168-bit zero-heap journal reassembler extracting active faults into `vehicle_alert_item_t`.
- **Hiworld Protocol Serialization:**
  - Standard CMD `0x42` (`HIWORLD_CMD_WARNING_INFO`): 2-byte single toast and 24-byte multi-alert table.
  - Custom Extended CMD `0xEA` (`HIWORLD_CMD_EXTENDED_ALERT`): Native 15-bit PSA DTCs, explicit severity (`STOP`, `SERVICE`, `INFO`), door masks, CHECK sequence.
  - Downlink CMD `0x2F` / `0x85`: Head Unit diagnostic log refresh query dispatches cached summary and sends CAN `0x39B`.
  - Unit tests: `test_peugeot_407_alert_single_abs`, `test_peugeot_407_alert_single_suspension`, `test_peugeot_407_alert_single_low_fuel`, `test_peugeot_407_alert_journal_multi`, `test_peugeot_407_alert_journal_block_multiplexed_real_log`, `test_peugeot_407_custom_cockpit_check_sequence`, `test_peugeot_407_custom_summary_table_and_empty_clearance`, `test_peugeot_407_custom_downlink_0x2f_response`.

### 4.8 GPS Clock & Calendar Synchronization (`0xCB` Downlink $\to$ CAN `0x228`)
- **CAN Ground Truth:**
  - `0x228` (dlc 8, 1000 ms): BSI clock broadcast and cluster synchronization (`[Hours, Minutes, Day, Month, Year, Format, 00, 00]`).
- **Implemented Downlink:**
  - Downlink CMD `0xCB` (`HIWORLD_CMD_DATE_TIME_SET`) extracted in `src/protocols/hiworld_connection.c`.
  - Synthesizes PSA CAN `0x228` frame with year offset from 2000, month, day, hour, minute.
  - Unit test: `test_hiworld_verification_vector_4_gps_time_sync`.

### 4.9 Physical Hardware Synthesis & Low-Power Management (GPIO)
- **ACC (Switched Head Unit Power):**
  - Synthesized via `GPIO_PIN_HEADUNIT_POWER` (PB9 on STM32 `volvo_od2`, PA8 on NUC131 `vw_nc03`, GPIO 18 on ESP32).
  - Primary Control Frame: CAN `0x036` (`COMMANDES_BSI`, 100 ms).
    - Byte 4 Bits [2:0] (`PHASE_VIE`):
      - `0x01`: **Ignition ON** (Wakeup / Run `+APC`) — Radio turns ON and stays awake (`VEHICLE_IGNITION_ON`, ACC ON).
      - `0x02`: **Ignition OFF** (Going to sleep) — Initiates shutdown / accessory timer (`VEHICLE_IGNITION_OFF`, ACC OFF).
      - `0x03`: **Wakeup transition** — Brief (~40 ms) pulse during key turn.
      - `0x00`: **Deep Sleep / Standby** — Head unit enters deep sleep mode (`VEHICLE_IGNITION_OFF`, ACC OFF).
    - Byte 2 Bit 7 (`MODE_ECO` = `0x80`):
      - `1`: **Economy Mode** — Forces radio and head unit power OFF immediately to protect battery.
  - RD4 Radio Awakening: CAN `0x165` Byte 0 Bit 7 (`0x80`). When key is OFF, turning on the factory RD4 radio promotes ignition state to `VEHICLE_IGNITION_ACC` to power the head unit. Shuts down when radio switches OFF (`(data[0] & 0xC0) == 0`). Overridden to OFF if Economy Mode (`0x036` Byte 2 Bit 7) is active.
  - Battery Reconnect: BSI sends `0x036` Byte 4 = `0x02` (Ignition OFF) for ~14s before dropping to `0x00` (`dump_connect_battery_only.log`); ACC wire stays strictly 0V throughout.
- **ILL (Night Dimming & Brightness):**
  - Synthesized via `GPIO_PIN_ILL_OUT` (PC13 on STM32 `volvo_od2`, PA9 on NUC131 `vw_nc03`, GPIO 5 on ESP32).
  - Triggers: CAN `0x036` Byte 3 Bit 5 (`0x20` cluster illumination) OR CAN `0x128` Byte 0 (`0x80` sidelights / `0x40` low beam).
  - Brightness Level: CAN `0x036` Byte 3 lower nibble (`0x0F`, 0..15) normalized in `state->lights.brightness`.
- **BRAKE / PARK (Handbrake Output):**
  - Synthesized via `GPIO_PIN_BRAKE_OUT` (PB8 on STM32 `volvo_od2`, PA12 on NUC131 `vw_nc03`).
  - Active State: Initialized to Output Push-Pull, held permanently OFF (`0`).
- **REVERSE (Fast Camera Switch):**
  - Synthesized via `GPIO_PIN_REVERSE_OUT` (PB5 on STM32 `volvo_od2`, PA13 on NUC131 `vw_nc03`, GPIO 4 on ESP32).
  - Triggers: CAN `0x036` Byte 1 Bit 7 / CAN `0x0F6` Byte 7 Bit 7 (`0x80` reverse gear engaged).
- **CAN Transceiver Standby & Sleep (`GPIO_PIN_CAN_STBY`):**
  - Pin PB6 on STM32 (`volvo_od2` / `qemu`), PC3 on NUC131 (`vw_nc03`). Configured in Open-Drain mode (`0` / 0V = Normal high-speed TX/RX mode, `1` / float to pull-up = Standby / Silent low-power mode $< 15\ \mu\text{A}$).
  - Wake-on-CAN: Any CAN frame received via `can_router_process_can()` immediately asserts `false` (Normal Mode) and resets watchdog (`s_can_bus_sleeping = false`).
  - Inactivity Sleep Watchdog: 3.0-second silence watchdog in `can_router_periodic_100ms()` unconditionally forces `ignition_state = VEHICLE_IGNITION_OFF`, drops ACC (`GPIO_PIN_HEADUNIT_POWER`), drops ILL, drops REVERSE, asserts Standby mode (`GPIO_PIN_CAN_STBY = true`), sets `s_can_bus_sleeping = true`, and completely halts all periodic telemetry, door repeats, TPMS updates, and keep-alive heartbeats (`CMD 0xFF`).
- **Verification:**
  - `test_scenario_ignition_off_after_power_on`: Battery connect, key OFF (0x02 hold), deep sleep (0x00), and transceiver standby after 3.0s bus silence.
  - `test_scenario_ignition_silence_watchdog_forces_sleep_from_on`: Unconditional sleep enforcement from active IGN ON, ACC cut, CAN standby assertion, UART silencing, and CAN frame wake-up.
  - `test_scenario_ignition_economy_mode_forces_radio_off`: Economy mode 0x80 override, ACC shutdown, RD4 ignition lock.
  - `test_scenario_lights_off_side_light_on_headlights_on`: ILL output, illumination bit, brightness 15 -> 10, headlights.
  - `test_integration_hiworld_reverse_pipeline`: REVERSE output.

### 4.10 Protocol Connection, Handshake & Resync Lifecycle (`0x24`, `0xF0`, `0xFF`)
- **Downlink Model Configuration (`0x24`):**
  - Handled in `src/protocols/hiworld_connection.c`.
  - Immediate Command ACK: Emits `5A A5 01 FF 24 23` to satisfy Android HU 3000 ms watchdog.
  - De-duplication & Burst Suppression: If `ctx->state == HIWORLD_LINK_ACTIVE` and vehicle model is unchanged, suppresses the redundant 70-byte configuration blast (`0xF0`, `0x71`, `0x72`, `0x76`, `0x79`, `0xC1`), eliminating UART buffer floods.
  - Model Switch: If vehicle model changes, reconfigures CAN controller baud rate dynamically (e.g. 125 kbps vs 500 kbps), switches active profile, and transmits updated configuration burst.
- **Protocol Heartbeat Ping (`0xFF`):**
  - Emitted at 1.0 Hz (1000 ms cadence / 10 periodic ticks) in `src/protocols/proto_hiworld_adapter.c` (`5A A5 01 FF 01 00`).
  - Rate throttled down from 10 Hz to prevent serial queue saturation.
- **Embedded Firmware Version (`0xF0`):**
  - Generated dynamically at build time via `tools/generate_version.py` into macro `CANBOX_BUILD_VERSION` as `CANBOX-CORE-V<YYYYMMDD.hhmmss>`.
  - Stored in expanded 64-byte `ctx->fw_version` buffer in `hiworld_connection_ctx_t`.
  - Sent upon handshake (`0x24`), explicit version query (`0x30`), and initial discovery beacon (`HIWORLD_LINK_WAIT_MODEL`). Displayed in Android Head Unit Factory / CAN settings.
- **Lifecycle Recovery (Scenarios A, B, C, D):**
  - Full operational recovery documented in `doc/CANBOX_SPEC_HIWORLD_GENERIC_CONNECTION_PHASE.md`.
- **Verification:** `test_hiworld_verification_vector_1_car_type_set`, `test_hiworld_verification_vector_2_version_report`, `test_canbox_embedded_firmware_version`, `test_integration_hiworld_runtime_car_selection_and_handshake`, `test_integration_hiworld_telemetry_periodic_pipeline`.

### 4.11 Dynamic CAN ID Pre-filtering & Routing Engine (Layer 3)
- **Static Bitmask Cache (Zero Heap):**
  - 256-byte static bitmask (`uint8_t s_allowed_ids_bitmask[256]`) mapping all 11-bit standard CAN IDs ($0 \dots 2047 / 0x7FF$).
  - Auto-generated on startup in `can_router_init()` and dynamically updated on profile switches via `vehicle_profile_set_active()` $\to$ `can_router_rebuild_filter()`.
- **Constant-Time O(1) Early Rejection:**
  - Rejects unmapped standard CAN frames and 29-bit extended frames before invoking profile decoders or executing differential state comparisons (`memcmp`).
  - Preserves transceiver bus wake-up and inactivity watchdog (`s_can_inactivity_ticks`) upon detecting any bus traffic prior to rejection.
- **Verification:**
  - Unit test `test_can_id_filter_enforcement` verifying profile rule ID acceptance, rejection of unmapped standard IDs (`0x7FF`, `0x111`, `0x260`), rejection of extended frames, and filter rebuild upon profile switching.

### 4.12 RD4 Radio, Tuner, RDS Text & CD Synchronization (`0x84`, `0x86`, `0x97`)
- **CAN Ground Truth (`doc/CANBOX_SPEC_HIWORLD_407_09_RD4_MFD_MEDIA_TEXT.md`):**
  - **RD4 Audio Source (`0x165` ETAT_AUTORADIO, 4 bytes, 50 ms):**
    - Byte 2[7:4] `INPUT_SOURCE`: `0x1` Tuner (FM/AM), `0x2` Internal CD, `0x3` CD Changer (CDC), `0x4` AUX 1, `0x5` AUX 2.
    - Decoded in `psa_rd4_process_can_0x165()` into `radio->source_mode` (`0x00..0x04`, `0x10`, `0x20`, `0x21`, `0x30`, `0x31`, `0xFF`).
  - **RD4 Tuner Status & Frequency (`0x225` ETAT_TUNER, 5-8 bytes, 100 ms):**
    - Byte 0: Indicators (Bit 2 TA `0x80`, Bit 5 RDS `0x20`, Bit 6 SCAN `0x10`, RDTEXT `0x04`), Seeking status (Bit 3 TUN -> `power_status = 0x02`).
    - Byte 1: Preset memory slot (0=manual, 1..6).
    - Byte 2: Band (`0x10`/`0x90` FM1, `0x20`/`0xA0` FM2, `0x40`/`0xC0` FM-AST, `0x50`/`0xD0` AM).
    - Byte 3..4: Frequency uint16_be ($R$): FM: $\text{Freq (0.1 MHz)} = R/2 + 500$; AM: direct kHz.
    - Decoded in `psa_rd4_process_can_0x225()`.
  - **RDS Station Name (`0x2A5` NOM_STATION, 8 bytes, 100 ms):**
    - 8-byte ASCII Program Service (PS) name, space padded. Decoded in `psa_rd4_process_can_0x2a5()`.
  - **Dynamic RDS RadioText (`0x0A4` TEXTE_RADIO, 8 bytes, event stream):**
    - ISO-TP reassembly (Single Frame, First Frame, Consecutive Frames) with automatic 4-byte `10 00 00 00` control prefix stripping.
    - Decoded in `psa_rd4_process_can_0x0a4()` into `radio->radio_text` (up to 64 chars).
  - **CD Changer Status (`0x3A6` CDC_STATUS, 8 bytes, 500 ms):**
    - Disc slot 1..6, track 1..99, total tracks 1..99, elapsed minutes/seconds, playback mode flags (RND, SCAN, RPT).
    - Decoded in `psa_rd4_process_can_0x3a6()` into `cdc->active_disc`, `cdc->discs_loaded_mask`, `cdc->play_status`.
- **Hiworld Protocol Serialization:**
  - `CMD 0x84` (`CarRadioState`, 14 payload bytes / 19 wire bytes): Band/source, frequency in Little-Endian `uint16_le` (0.1 MHz/kHz) matching `PeugeotDataParser.smali::parseCarRadioState`, preset slot, stabilized indicator bitmask (TA bit preserved between `0x225` and `0x265`), power status, 8-byte PS station name. Serialized by `build_hiworld_radio_state()` and `hiworld_send_radio_state()`.
  - `CMD 0x86` (`RadioTextInfo`, $1\dots 64$ payload bytes / $5+\text{LEN}$ wire bytes): Dynamic RDS RadioText string routed to `tv_radio_text` marquee. Serialized by `build_hiworld_radio_text()` and `hiworld_send_radio_text()`.
  - `CMD 0x97` (`CarMediaState`, 11 payload bytes / 16 wire bytes): Active disc, loaded disc mask, track uint16, elapsed mm:ss, play modes, play status, total tracks uint16. Serialized by `build_hiworld_media_state()` and `hiworld_send_media_state()`.
- **Downlink Queries & Resync:**
  - Head Unit resume query `forwardType(0x0F)`: immediately retransmits cached `0x84` radio state and `0x86` radio text.
  - Media resume query `forwardType(0x11)` / `forwardType(0x12)`: immediately retransmits cached `0x97` media state.
  - Slow periodic heartbeat resync (60 seconds / 600 ticks in `can_router_periodic_100ms`).
- **Verification:**
  - Unit tests: `test_peugeot_407_rd4_vector_1_fm_tuner`, `test_peugeot_407_rd4_vector_2_isotp_radiotext`, `test_peugeot_407_rd4_vector_3_cd_changer`, `test_peugeot_407_rd4_source_and_wavebands`, `test_peugeot_407_rd4_frequency_and_ta_stability`.
  - Integration pipeline: `test_integration_hiworld_rd4_radio_tuner_pipeline`, `test_integration_hiworld_rd4_radiotext_pipeline`, `test_integration_hiworld_rd4_cd_changer_pipeline`, `test_integration_hiworld_rd4_downlink_resume_queries`.
  - Host Unit Tests: 98/98 passed. Integration Tests: 31/31 passed. Target builds: STM32 (0 warnings), ESP32 (0 warnings).

---

## 5. Critical PSA Alert DTC Mapping Reference

| Alert ID (Hex) | Dec | Severity | English Warning Text | French OEM Context | Hiworld Std (`0x42`) | Extended (`0xEA`) |
| :---: | :---: | :---: | :--- | :--- | :---: | :---: |
| `0x0001` | 1 | CRITICAL (STOP) | Flat tyre(s) detected | Crevaison détectée | `0x00A0` | `0x0001` (Severity 2) |
| `0x0003` | 3 | CRITICAL (STOP) | Coolant circuit level low | Niveau liquide refroidissement insuffisant | `0x0003` | `0x0003` (Severity 2) |
| `0x0004` | 4 | WARNING | Check engine oil level | Compléter niveau d'huile moteur | `0x0004` | `0x0004` (Severity 1) |
| `0x0005` | 5 | CRITICAL (STOP) | Engine oil pressure low | Pression huile moteur insuffisante | `0x0005` | `0x0005` (Severity 2) |
| `0x0006` | 6 | CRITICAL (STOP) | Engine coolant temp too high | Température eau moteur trop élevée | `0x0006` | `0x0006` (Severity 2) |
| `0x0008` | 8 | CRITICAL (STOP) | Braking system faulty | Système de freinage défaillant | `0x0008` | `0x0008` (Severity 2) |
| `0x000B` | 11 | WARNING | Door(s) open at speed | Portière(s) ouverte(s) en roulant | `0x0011`..`0x0014` | `0x000B` (Severity 1) |
| `0x000D` | 13 | WARNING | Boot open at speed | Coffre ouvert | `0x0014` | `0x000D` (Severity 1) |
| `0x0061` | 97 | WARNING | Power steering faulty | Direction assistée défaillante | `0x0061` | `0x0061` (Severity 1) |
| `0x0067` | 103 | WARNING | Water in diesel fuel filter | Présence d'eau dans filtre à gazole | `0x0067` | `0x0067` (Severity 1) |
| `0x0068` | 104 | WARNING | Brake pads worn | Plaquettes de frein usées | `0x0068` | `0x0068` (Severity 1) |
| `0x0069` | 105 | INFO | Handbrake on | Frein de stationnement serré | `0x0069` | `0x0069` (Severity 0) |
| `0x006D` | 109 | WARNING | ABS braking system faulty | Système ABS défaillant | `0x006D` | `0x006D` (Severity 1) |
| `0x006E` | 110 | WARNING | ESP / ASR system faulty | Système ESP/ASR défaillant | `0x006E` | `0x006E` (Severity 1) |
| `0x006F` | 111 | WARNING | Suspension system faulty | Suspension défaillante | `0x006F` | `0x006F` (Severity 1) |
| `0x0073` | 115 | WARNING | Under-inflation detected | Pression pneumatiques insuffisante | `0x00A0` | `0x0073` (Severity 1) |
| `0x0078` | 120 | WARNING | Airbag system faulty | Airbag(s) défaillant(s) | `0x00F0` | `0x0078` (Severity 1) |
| `0x007B` | 123 | WARNING | Automatic gearbox faulty | Boîte de vitesses automatique défaillante | `0x007B` | `0x007B` (Severity 1) |
| `0x0081` | 129 | WARNING | Particle filter risk clogging | Filtre à particules : risque colmatage | `0x0081` | `0x0081` (Severity 1) |
| `0x0082` | 130 | WARNING | Diesel additive level low | Niveau additif gazole trop faible | `0x0082` | `0x0082` (Severity 1) |
| `0x00E0` | 224 | WARNING | Fuel level low | Niveau carburant faible | `0x00E0` | `0x00E0` (Severity 1) |
| `0x00E1` | 225 | INFO | Screenwash level low | Niveau lave-glace insuffisant | `0x00E1` | `0x00E1` (Severity 0) |
| `0x00EA` | 234 | INFO | Sidelamp bulb faulty | Lampe feux de position défaillante | `0x00EA` | `0x00EA` (Severity 0) |
| `0x00EB` | 235 | INFO | Brake lamp bulb faulty | Lampe stop défaillante | `0x00EB` | `0x00EB` (Severity 0) |
| `0x00F5` | 245 | WARNING | Depollution system faulty | Système antipollution défaillant | `0x0068` | `0x00F5` (Severity 1) |

---

## 6. Actionable Next Implementation Steps (Roadmap)

### Priority 1: Climate Control Downlink (`CMD 0x3B` $\to$ CAN `0x1E1` / `0x3A1`)
- **Goal:** Allow Android touchscreen HVAC overlay (`WindowAcControl.smali`) to command temperature setpoints, blower fan speed, and air distribution.
- **Specification:** `doc/PEUGEOT_HIWORLD_CLIMA_PROTOCOL_DOWNLINK.md`
- **Steps:**
  1. Add `HIWORLD_CMD_AC_SETTING_SET (0x3B)` handler in `src/protocols/proto_hiworld_adapter.c`.
  2. Implement CAN frame builder for PSA comfort climate controls (`0x1E1`).
  3. Add test vectors in `test/test_protocol_parser/test_hiworld_parser.c` and `test/test_integration/test_integration.c`.

### Priority 2: Speed Limiter & Cruise Control Presets Uplink (`CMD 0x8A` / `0x8B`)
- **Goal:** Broadcast active cruise control target speed and stored speed limiter slots 1..6 to Android UI (`CruiseSpeedFrgment.smali`).
- **Specification:** `commands_and_payload_structure.md` Section 4.1 (`CMD 0x8A` / `0x8B`)
- **Steps:**
  1. Leverage existing CAN decoder `psa_decode_cruise_0x1a8()`.
  2. Implement `hiworld_send_cruise_presets()` generating Hiworld `0x8A` / `0x8B` packets.
  3. Validate against test vector `test_psa_extended_decode_cruise_0x1a8`.

### Priority 3: BSI Central Settings Downlink (`CMD 0x7B` / `0x7D` $\to$ CAN `0x39B` / `0x2A8`)
- **Goal:** Transmit user preference toggles (guide-me-home lighting delay, daytime running lamps, selective door unlocking, rear wiper in reverse) from Android `CentralSettingFragment` to BSI configuration frames.
- **Specification:** `doc/CANBOX_SPEC_HIWORLD_407_11_CAR_CONFIGURATION_OPTIONS.md`, `doc/PEUGEOT_RT4_CAR_CONFIG.md`
- **Steps:**
  1. Add downlink command handlers for `0x7B` and `0x7D` in `src/protocols/proto_hiworld_adapter.c`.
  2. Synthesize PSA BSI configuration messages (`0x39B` / `0x2A8`).

### Priority 4: RD4 Radio & CD Changer Media Passthrough (`CMD 0x84`, `0x86`, `0x97`) [COMPLETED]
- **Status:** `[x]` Fully implemented and verified across unit and integration tests.
- **Scope:** Uplink passing OEM radio tuner frequency (`0x225`), RDS station name (`0x2A5`), dynamic RadioText (`0x0A4`), and CD changer playback state (`0x3A6`) to the Android `OriginalTuner` and `OriginalMediaPlayer`, plus `forwardType(0x0F)` / `forwardType(0x11)` resume query handlers.

### Priority 5: Dynamic Guidelines & Steering Angle (SAS) Frame Discovery
- **Status:** Placeholders completely removed.
- **Condition:** In live vehicle testing, cluster illumination (`0x128`) previously caused false trajectory oscillations. Dynamic steering guidelines remain disabled until a verified PSA CAN2004 optical steering angle frame is captured from a Peugeot 407.

---

## 7. Verification & Automated Test Suite Reference

All test commands run on host desktop without hardware connected:

| Test Scope | CLI Command | Current Status | Coverage |
| :--- | :--- | :--- | :--- |
| **Native Unit Tests** | `~/.platformio/penv/bin/pio test -e native_test_runner` | **97 / 97 PASSED** | Hiworld framing, checksums, decoders, serializers, alert table, dynamic CAN ID filter, RD4 tuner/RadioText/CDC |
| **Integration Pipeline** | `~/.platformio/penv/bin/pio test -e integration_test` | **31 / 31 PASSED** | End-to-end CAN $\to$ Router $\to$ Hiworld UART pipeline including RD4 radio, RadioText, CDC & resume queries |
| **STM32 Target Build** | `~/.platformio/penv/bin/pio run -e stm32_cbox` | **BUILD OK** | Bare-metal ARM Cortex-M3 flash binary |
| **ESP32 Target Build** | `~/.platformio/penv/bin/pio run -e esp32_cbox` | **BUILD OK** | Dual-core Xtensa FreeRTOS flash binary |
| **Interactive Desktop Sim** | `CANBOX_CAN_IFACE="vcan0" ~/.platformio/penv/bin/pio run -e native_test -t exec` | **OPERATIONAL** | Virtual CAN (`vcan0`) and pseudo-terminal (`pty`) emulator |

---

