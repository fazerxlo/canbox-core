# OpenCanbox Core: Hiworld PSA Peugeot 407 Progress Tracker

- **Platform:** Peugeot 407 (PSA CAN2004 / AEE2004 Comfort Bus @ 125 kbps)
- **Protocol:** Hiworld PSA (`0x5A 0xA5` Framing, UART 38,400 baud 8N1, APK `com.qf.vehicle` / `wc`)

---

## 1. Subsystem Completion Summary

| Functional Subsystem | Total | `[x]` | `[-]` | `[ ]` | % |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **1. Protocol Connection & Handshake** | 6 | 6 | 0 | 0 | 100% |
| **2. Steering Wheel Controls & Keys** | 3 | 3 | 0 | 0 | 100% |
| **3. Dual-Zone Climate Control (HVAC)** | 3 | 2 | 1 | 0 | 67% |
| **4. Doors, Body & Apertures** | 2 | 2 | 0 | 0 | 100% |
| **5. Trip Computer & Telemetry** | 5 | 5 | 0 | 0 | 100% |
| **6. Parking Radar (OPS / AAS)** | 2 | 2 | 0 | 0 | 100% |
| **7. Tire Pressure Monitoring (TPMS)** | 3 | 2 | 0 | 1 | 67% |
| **8. BSI Vehicle Alerts & Diagnostics** | 5 | 5 | 0 | 0 | 100% |
| **9. Central Personalization & Settings** | 4 | 2 | 2 | 0 | 50% |
| **10. Date & Time Synchronization** | 2 | 1 | 1 | 0 | 50% |
| **11. RD4 Audio, Media, CDC & DSP** | 6 | 6 | 0 | 0 | 100% |
| **12. Dynamic Guidelines & Steering SAS** | 1 | 1 | 0 | 0 | 100% |
| **13. Powertrain & Hardware GPIO Synthesis** | 5 | 5 | 0 | 0 | 100% |
| **14. Dynamic CAN ID Pre-filtering & Router** | 1 | 1 | 0 | 0 | 100% |
| **15. Commercial Hardware & IAP Bootloader** | 2 | 2 | 0 | 0 | 100% |
| **Overall** | **50** | **45** | **3** | **2** | **90%** |

---

## 2. Master Command Implementation Matrix

### 2.1 Uplink Telemetry (CAN Box $\to$ Android HU)

| St | CMD | Name | Wire LEN | CAN IDs | Core Files | Spec / Notes |
| :-: | :-: | :--- | :-: | :-: | :--- | :--- |
| `[x]` | `0xF0` | Version String Report | Var | N/A | `canbox_version.c`, `hiworld_connection.c` | Dynamic `CANBOX-CORE-V<YYYYMMDD.hhmmss>` |
| `[x]` | `0x11` | Steering Wheel Keys (SWC) | `0x0A` | `0x21F`, `0x221`, `0x0F6` | `psa_407_hvac_body.c`, `proto_hiworld_adapter.c` | `CANBOX_SPEC_HIWORLD_407_01_STEERING_STALK_KEYS.md` |
| `[x]` | `0x11` | Dynamic Trajectory (SAS) | `0x0A` | `0x10B`, `0x0F6` | `profile_psa.c`, `can_router.c` | `CANBOX_SPEC_HIWORLD_407_05_STEERING_ANGLE_SAS.md` (`-(raw/10)` deg in reverse) |
| `[x]` | `0x12` | Doors & Apertures | `0x0B` | `0x220`, `0x036` | `psa_407_hvac_body.c`, `hiworld_psa_builders.c` | `CANBOX_SPEC_HIWORLD_407_07_DOORS_BODY_STATUS.md` |
| `[x]` | `0x13` | Instantaneous Trip | `0x0B` | `0x221` | `psa_407_telemetry.c`, `hiworld_psa_builders.c` | `CANBOX_SPEC_HIWORLD_407_06_TRIP_COMPUTER_TELEMETRY.md` |
| `[x]` | `0x14` | Trip 1 Telemetry | `0x07` | `0x2A1` | `psa_407_telemetry.c`, `hiworld_psa_builders.c` | Avg fuel, avg speed, distance |
| `[x]` | `0x15` | Trip 2 Telemetry | `0x07` | `0x261` | `psa_407_telemetry.c`, `hiworld_psa_builders.c` | Avg fuel, avg speed, distance |
| `[x]` | `0x21` | Console Panel Buttons | `0x02` | `0x3E5` | `psa_407_hvac_body.c`, `proto_hiworld_adapter.c` | `CANBOX_SPEC_HIWORLD_407_01_STEERING_STALK_KEYS.md` (§3.3) |
| `[x]` | `0x31` | Dual-Zone Climate (HVAC) | `0x0D` | `0x1D0`, `0x1E3`, `0x0F6` | `psa_407_hvac_body.c`, `hiworld_psa_builders.c` | `CANBOX_SPEC_HIWORLD_407_02_DUAL_ZONE_HVAC.md` |
| `[x]` | `0x41` | Parking Radar (OPS/AAS) | `0x0D` | `0x0E1` | `psa_407_hvac_body.c`, `hiworld_psa_builders.c` | `CANBOX_SPEC_HIWORLD_407_04_PARKING_RADAR_AAS.md` |
| `[x]` | `0x42` | Standard Vehicle Alerts | `0x03`/`0x19` | `0x1A1`, `0x120` | `psa_407_alerts.c`, `proto_hiworld_adapter.c` | `CANBOX_SPEC_HIWORLD_407_12_ALERTS_DIAGNOSTICS.md` |
| `[x]` | `0x56` | RD4 Volume & Audio (DSP) | `0x0D` | `0x1A5`, `0x1E5` | `psa_407_rd4_audio.c`, `hiworld_psa_builders.c` | `CANBOX_SPEC_HIWORLD_407_08_JBL_AMPLIFIER_DSP.md` |
| `[x]` | `0x66` | TPMS Numeric Pressures | `0x07` | `0x3A1`, `0x1E1` | `psa_407_telemetry.c`, `hiworld_psa_builders.c` | `CANBOX_SPEC_HIWORLD_407_03_TPMS_TIRE_PRESSURE.md` |
| `[x]` | `0x71` | Feature Availability 1 | `0x03` | Fixed | `hiworld_connection.c` | `CANBOX_SPEC_HIWORLD_GENERIC_CONNECTION_PHASE.md` |
| `[x]` | `0x72` | Feature Availability 2 | `0x09` | Fixed | `hiworld_connection.c` | `CANBOX_SPEC_HIWORLD_GENERIC_CONNECTION_PHASE.md` |
| `[x]` | `0x76` | Central Convenience 1 | `0x03` | Fixed / BSI | `hiworld_connection.c` | `CANBOX_SPEC_HIWORLD_407_11_CAR_CONFIGURATION_OPTIONS.md` |
| `[x]` | `0x79` | Central Convenience 2 | `0x09` | Fixed / BSI | `hiworld_connection.c` | `CANBOX_SPEC_HIWORLD_407_11_CAR_CONFIGURATION_OPTIONS.md` |
| `[-]` | `0x7B` | Convenience Settings | `0x03` | `0x39B`, `0x2A8` | `hiworld_connection.c` | Uplink decoded; downlink pending |
| `[ ]` | `0x7D` | TPMS Calibration State | `0x03` | `0x1E1` | `proto_hiworld_adapter.c` | Planned |
| `[x]` | `0x84` | RD4 Radio / Source State | `0x0E` | `0x165`, `0x225`, `0x2A5`, `0x265` | `psa_407_rd4_audio.c`, `can_router.c` | `CANBOX_SPEC_HIWORLD_407_09_RD4_MFD_MEDIA_TEXT.md` |
| `[x]` | `0x85` | RD4 Preset Memory List | `0x0D`/`0x31` | `0x225`, `0x125` | `psa_407_rd4_audio.c`, `can_router.c` | Freqs (13B) & ISO-TP Station Names (49B) |
| `[x]` | `0x86` | RDS RadioText Marquee | `1..64` | `0x0A4` | `psa_407_rd4_audio.c`, `can_router.c` | ISO-TP reassembly, strips `10 00 00 00` |
| `[-]` | `0x8A` | Speed Limiter Presets | `0x0B` | `0x1A8` | `psa_407_telemetry.c` | Decoder ready; Hiworld serializer pending |
| `[-]` | `0x8B` | Cruise Control Presets | `0x0B` | `0x1A8` | `psa_407_telemetry.c` | Decoder ready; Hiworld serializer pending |
| `[-]` | `0x8C` | Clock Sync Feedback | `0x08` | `0x228` | `hiworld_connection.c` | `CANBOX_SPEC_HIWORLD_407_10_BSI_SETTINGS_CLOCK_SYNC.md` |
| `[x]` | `0x97` | CD & CDC Playback State | `0x0B` | `0x325`, `0x365`, `0x3A5`, `0x3A6` | `psa_407_rd4_audio.c`, `can_router.c` | `CANBOX_SPEC_HIWORLD_407_09_RD4_MFD_MEDIA_TEXT.md` |
| `[x]` | `0xC1` | System Units Broadcast | `0x04` | Fixed | `hiworld_connection.c` | L/100km, km/h, Celsius (`0x28, 0x20, 0x00`) |
| `[ ]` | `0xC2` | Date & Time Broadcast | `0x08` | `0x228` | `hiworld_connection.c` | Uplink clock broadcast planned |
| `[-]` | `0xCB` | Powertrain Telemetry | `0x0B` | `0x0B6`, `0x165` | `psa_407_telemetry.c` | RPM/Speed decoded; `0xCB` packing planned |
| `[x]` | `0xEA` | Extended CAN Alerts | `0x06`/Var | `0x1A1`, `0x120` | `psa_407_alerts.c`, `proto_hiworld_adapter.c` | 15-bit PSA DTCs, severity, door masks, CHECK |
| `[x]` | `0xFF` | Heartbeat Ping | `0x02` | Timer (1 Hz) | `proto_hiworld_adapter.c` | `5A A5 01 FF 01 00` every 1000 ms |

### 2.2 Downlink Control (Android HU $\to$ CAN Box)

| St | CMD | Name | Wire LEN | CAN Tx | Core Files | Spec / Notes |
| :-: | :-: | :--- | :-: | :-: | :--- | :--- |
| `[x]` | `0x1B` | Trip Computer Reset | `0x03`/`0x05` | `0x221` | `psa_407_telemetry.c`, `proto_hiworld_adapter.c` | Resets Trip 1 (`0x80`) or Trip 2 (`0x40`) |
| `[x]` | `0x24` | Car Model & Baud Select | `0x02`/`0x03` | N/A | `hiworld_connection.c`, `hiworld_car_mapping.c` | ID 34 (Peugeot 407), ACK `5A A5 01 FF 24 23` |
| `[x]` | `0x2F` | Diagnostic Journal Query | `0x02` | `0x39B` | `proto_hiworld_adapter.c`, `can_router.c` | Replays `0x42` & `0xEA` summary |
| `[x]` | `0x30` | Firmware Version Query | `0x02` | N/A | `hiworld_connection.c` | Replies with `0xF0` |
| `[-]` | `0x3B` | Climate Control Downlink | `0x0B` | `0x1E1` | `CANBOX_SPEC_HIWORLD_407_02_DUAL_ZONE_HVAC.md` | Downlink spec ready |
| `[-]` | `0x71` | Personalization Toggle | `0x02`/`0x03` | `0x361` | `CANBOX_SPEC_HIWORLD_407_11_CAR_CONFIGURATION_OPTIONS.md` | DRL, follow-me-home |
| `[ ]` | `0x76` | Overspeed Warning Set | `0x03` | `0x39B` | `CANBOX_SPEC_HIWORLD_407_11_CAR_CONFIGURATION_OPTIONS.md` | Speed threshold |
| `[ ]` | `0x79` | Navigation MFD Guidance | `0x08` | `0x3B6` | `CANBOX_SPEC_HIWORLD_407_09_RD4_MFD_MEDIA_TEXT.md` | Turn arrows to MFD |
| `[-]` | `0x7B` | Central Setting Req 1 | `0x03` | `0x361` | `CANBOX_SPEC_HIWORLD_407_11_CAR_CONFIGURATION_OPTIONS.md` | Convenience param write |
| `[-]` | `0x7D` | Central Setting Req 2 | `0x03` | `0x361` | `CANBOX_SPEC_HIWORLD_407_11_CAR_CONFIGURATION_OPTIONS.md` | Convenience param write |
| `[-]` | `0x81` | Cruise Presets Setup | `0x0A` | `0x39B` | `CANBOX_SPEC_HIWORLD_407_06_TRIP_COMPUTER_TELEMETRY.md` | Speed memory slots 1..6 |
| `[x]` | `0x85` | Alarm Replay Query | `0x02` | `0x39B` | `proto_hiworld_adapter.c` | Replays active alert journal |
| `[x]` | `0x94` | Software Reset / Ping | `0x01` | N/A | `hiworld_connection.c` | Link liveness / reboot |
| `[x]` | `0xAD` | RD4 Audio Source Routing | `0x02`..`0x03` | `0x21F` | `psa_source_arbiter.c`, `can_router.c` | Line In 1 / AUX 1 routing (`0xAD 03 03` / `0xAD 02 01`) |
| `[x]` | `0xCB` | GPS Date & Time Set | `0x07` | `0x228` | `hiworld_connection.c` | `CANBOX_SPEC_HIWORLD_407_10_BSI_SETTINGS_CLOCK_SYNC.md` |
| `[-]` | `0xC2`/`0xE0` | MCU IAP Flash Transfer | Var | N/A | `hiworld_iap_nuc131.c`, `pack_hiworld_iap.py` | NUC131 FMC vector remap & 136B container |
| `[x]` | **ALL** | Universal Downlink ACK | `0x02` | N/A | `proto_hiworld_adapter.c` | `5A A5 01 FF <RxCmd> <(RxCmd-1)&0xFF>` |
