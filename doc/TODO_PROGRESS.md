# OpenCanbox Core: Hiworld PSA Peugeot 407 Progress Tracker

- **Platform:** Peugeot 407 (PSA CAN2004 / AEE2004 Comfort Bus @ 125 kbps)
- **Protocol:** Hiworld PSA (`0x5A 0xA5` Framing, UART 38,400 baud 8N1, APK `com.qf.vehicle` / `wc`)

---

## 1. Subsystem Completion Summary

| Functional Subsystem | Total | `[x]` | `[-]` | `[ ]` | % |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **1. Protocol Connection & Handshake** | 6 | 6 | 0 | 0 | 100% |
| **2. Steering Wheel Controls & Keys** | 4 | 2 | 0 | 2 | 50% |
| **3. Dual-Zone Climate Control (HVAC)** | 2 | 1 | 1 | 0 | 50% |
| **4. Doors, Body & Apertures** | 2 | 2 | 0 | 0 | 100% |
| **5. Trip Computer & Telemetry** | 4 | 4 | 0 | 0 | 100% |
| **6. Parking Radar (OPS / AAS)** | 3 | 2 | 0 | 1 | 67% |
| **7. Tire Pressure Monitoring (TPMS)** | 3 | 2 | 0 | 1 | 67% |
| **8. BSI Vehicle Alerts & Diagnostics** | 5 | 5 | 0 | 0 | 100% |
| **9. Central Personalization & Settings** | 7 | 3 | 1 | 3 | 43% |
| **10. Date, Time & Units Synchronization** | 3 | 1 | 1 | 1 | 33% |
| **11. RD4 Audio, Media, CDC & DSP** | 10 | 8 | 0 | 2 | 80% |
| **12. Dynamic Guidelines & Steering SAS** | 1 | 1 | 0 | 0 | 100% |
| **13. Powertrain & Telemetry Gauges** | 3 | 0 | 3 | 0 | 0% |
| **14. Secondary Display & ID3 Navigation** | 2 | 0 | 0 | 2 | 0% |
| **15. Commercial Hardware & IAP Bootloader** | 2 | 2 | 0 | 0 | 100% |
| **Overall** | **57** | **39** | **6** | **12** | **68%** |

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
| `[x]` | `0x18` | TPMS Discrete Warnings | `0x06` | `0x3A1`, `0x1E1` | `hiworld_psa_builders.c`, `proto_hiworld_adapter.c` | FL, FR, RL, RR status: normal, low, flat, missing |
| `[x]` | `0x21` | Console Panel Buttons | `0x02` | `0x3E5` | `psa_407_hvac_body.c`, `proto_hiworld_adapter.c` | `PanelMap_407_06` (`0x07` Dark, `0x17` FM, `0x28` Clim, `0x40` Trip) |
| `[ ]` | `0x22` | Console Rotary Encoders | `0x02` | `0x3E5` | `hiworld_connection.h` | Vol/tune knob cumulative delta (volume HUD uses `0x56`) |
| `[x]` | `0x31` | Dual-Zone Climate (HVAC) | `0x0D` | `0x1D0`, `0x1E3`, `0x0F6` | `psa_407_hvac_body.c`, `hiworld_psa_builders.c` | Dual independent temps, fan, airflow, forced `SYNC=0` (D0 Bit 2) |
| `[x]` | `0x41` | Parking Radar (OPS/AAS) | `0x0D` | `0x0E1` | `psa_407_hvac_body.c`, `hiworld_psa_builders.c` | 4 Front + 4 Rear distance bars (0..4) |
| `[x]` | `0x42` | Standard Vehicle Alerts | `0x03`/`0x19` | `0x1A1`, `0x120` | `psa_407_alerts.c`, `proto_hiworld_adapter.c` | Canonical Hiworld codes; clear frame `5A A5 02 42 00 00 43` |
| `[x]` | `0x56` | RD4 Volume & Audio (DSP) | `0x0D` | `0x1A5`, `0x1E5` | `psa_407_rd4_audio.c`, `hiworld_psa_builders.c` | Volume Mode `0x01` HUD overlay, Bass/Treble/Bal/Fad/Loudness |
| `[x]` | `0x66` | TPMS Numeric Pressures | `0x07` | `0x3A1`, `0x1E1` | `psa_407_telemetry.c`, `hiworld_psa_builders.c` | 4-wheel pressures in 0.1 Bar increments |
| `[x]` | `0x71` | Feature Availability 1 | `0x03` | Fixed | `hiworld_connection.c` | BSI personalization capabilities mask 1 |
| `[x]` | `0x72` | Feature Availability 2 | `0x09` | Fixed | `hiworld_connection.c` | BSI personalization capabilities mask 2 |
| `[x]` | `0x76` | Central Convenience 1 | `0x03` | Fixed / BSI | `hiworld_connection.c`, `psa_407_car_config.c` | DRL, reverse rear wiper, selective unlock from `0x260` Profile 1 |
| `[x]` | `0x79` | Central Convenience 2 | `0x09` | Fixed / BSI | `hiworld_connection.c`, `psa_407_car_config.c` | Guide-me-home delay, cluster dimmer level from `0x260` Profile 1 |
| `[-]` | `0x7B` | Convenience Settings | `0x03` | `0x39B`, `0x2A8` | `hiworld_connection.c` | Uplink decoded; downlink handled via `0x15B` |
| `[ ]` | `0x7D` | TPMS Calibration State | `0x03` | `0x1E1` | `proto_hiworld_adapter.c` | Planned calibration learning state uplink |
| `[-]` | `0x81` | Speed & RPM Telemetry | `0x06` | `0x0B6`, `0x165` | `psa_407_telemetry.c` | Decoded internally; Hiworld `0x81` frame serializer pending |
| `[-]` | `0x82` | Cruise & Limiter Info | `0x06` | `0x1A8` | `psa_407_telemetry.c` | Decoded internally; Hiworld `0x82` frame serializer pending |
| `[x]` | `0x84` | RD4 Radio / Source State | `0x0E` | `0x165`, `0x225`, `0x2A5`, `0x265` | `psa_407_rd4_audio.c`, `can_router.c` | Tuner band, frequency, presets 1..6, stereo, 8-char RDS PS name |
| `[x]` | `0x85` | RD4 Preset Memory List | `0x0D`/`0x31` | `0x225`, `0x125` | `psa_407_rd4_audio.c`, `can_router.c` | Freqs (13B) & ISO-TP Station Names (49B); AMVAR status pending |
| `[x]` | `0x86` | RDS RadioText Marquee | `1..64` | `0x0A4` | `psa_407_rd4_audio.c`, `can_router.c` | ISO-TP reassembly, strips `10 00 00 00`, up to 64 chars |
| `[-]` | `0x8C` | Clock Sync Feedback | `0x08` | `0x228` | `hiworld_connection.c` | BSI cluster clock read-back to Android planned |
| `[-]` | `0x93` | Factory RD4 Host Info | `0x03` | `0x165` | `hiworld_connection.h` | Active source/power emitted via `0x84`/`0x97` |
| `[x]` | `0x97` | CD & CDC Playback State | `0x0B` | `0x325`, `0x365`, `0x3A5`, `0x3A6` | `psa_407_rd4_audio.c`, `can_router.c` | Track, disc slot 1..6, elapsed min/sec, repeat/shuffle |
| `[x]` | `0xC1` | System Units Broadcast | `0x04` | Fixed | `hiworld_connection.c` | L/100km, km/h, Celsius (`0x28, 0x20, 0x00`) |
| `[ ]` | `0xC2` | Date & Time Broadcast | `0x08` | `0x228` | `hiworld_connection.c` | Uplink clock broadcast planned |
| `[-]` | `0xCB` | Powertrain Telemetry | `0x0B` | `0x0B6`, `0x165` | `psa_407_telemetry.c` | Extended powertrain packet packing planned |
| `[x]` | `0xEA` | Extended CAN Alerts | `0x06`/Var | `0x1A1`, `0x120` | `psa_407_alerts.c`, `proto_hiworld_adapter.c` | 15-bit PSA DTCs, severity, categories, empty clear frame |
| `[x]` | `0xFF` | Heartbeat Ping | `0x02` | Timer (1 Hz) | `proto_hiworld_adapter.c` | `5A A5 01 FF 01 00` every 1000 ms |

---

### 2.2 Downlink Control (Android HU $\to$ CAN Box)

| St | CMD | Name | Wire LEN | CAN Tx | Core Files | Spec / Notes |
| :-: | :-: | :--- | :-: | :-: | :--- | :--- |
| `[x]` | `0x0F` | Query RD4 Radio State | `0x01` | N/A | `proto_hiworld_adapter.c` | Responds immediately with `0x84`, `0x85`, station list trigger |
| `[x]` | `0x1B` | Trip Computer Reset | `0x03`/`0x05` | `0x167` | `psa_407_telemetry.c`, `proto_hiworld_adapter.c` | Resets Trip 1 (`0x80`) or Trip 2 (`0x40`) |
| `[ ]` | `0x21` | Simulated Panel Key | `0x02` | `0x3E5` | `PEUGEOT_HIWORLD_COMMANDS.md` | Virtual console key injection to CAN |
| `[x]` | `0x24` | Car Model & Baud Select | `0x02`/`0x03` | N/A | `hiworld_connection.c`, `hiworld_car_mapping.c` | ID 34 (Peugeot 407), ACK `5A A5 01 FF 24 23` |
| `[x]` | `0x2F` | Diagnostic Journal Query | `0x02` | `0x39B` | `proto_hiworld_adapter.c`, `can_router.c` | Replays `0x42` & `0xEA` summary |
| `[x]` | `0x30` | Firmware Version Query | `0x02` | N/A | `hiworld_connection.c` | Replies with `0xF0` |
| `[-]` | `0x3B` | Climate Control Downlink | `0x0B` | `0x1D0`/`0x1E1` | `psa_407_car_config.c` | Sync toggle (`0x0F`) mapped; touchscreen temp/fan steps pending |
| `[x]` | `0x71` | Personalization Toggle | `0x02`/`0x03` | `0x15B` | `psa_407_car_config.c` | DRL, follow-me-home write |
| `[ ]` | `0x76` | Overspeed Warning Set | `0x03` | `0x39B` | `CANBOX_SPEC_HIWORLD_407_11_CAR_CONFIGURATION_OPTIONS.md` | Speed warning chime threshold |
| `[ ]` | `0x79` | Navigation MFD Guidance | `0x08` | `0x3B6` | `CANBOX_SPEC_HIWORLD_407_09_RD4_MFD_MEDIA_TEXT.md` | Turn arrows & distance to amber EMF-C |
| `[x]` | `0x7B` | Central Setting Req 1 | `0x03` | `0x15B` | `psa_407_car_config.c` | Rear wiper, selective unlock, DRL, follow-me-home |
| `[x]` | `0x7D` | Central Setting Req 2 | `0x03` | `0x15B`/`0x221` | `psa_407_car_config.c` | Remote boot mode, directional xenon, driver profile, TPMS learn |
| `[-]` | `0x8A`/`0x8B` | Cruise Presets Setup | `0x0A` | `0x39B` | `PEUGEOT_HIWORLD_COMMANDS.md` | Speed memory slots $M_1 \dots M_5$ configuration |
| `[ ]` | `0x8C` | AMVAR Sport Mode Switch | `0x02` | `0x39B` | `PEUGEOT_HIWORLD_COMMANDS.md` | Electronically controlled suspension toggle (407 V6) |
| `[x]` | `0x85` | Alarm Replay Query | `0x02` | `0x39B` | `proto_hiworld_adapter.c` | Replays active alert journal |
| `[x]` | `0x94` | Software Reset / Ping | `0x01` | N/A | `hiworld_connection.c` | Link liveness / reboot |
| `[ ]` | `0x9A` | System Language Sync | `0x02` | `0x15B` | `PEUGEOT_HIWORLD_COMMANDS.md` | Sync Android language to instrument cluster / MFD |
| `[x]` | `0xA1` | Host Audio State Sync | `0x03` | N/A | `proto_hiworld_adapter.c` | Handled to sync Tuner / RD4 state |
| `[ ]` | `0xA2` | OEM Tuner Downlink | `0x06` | `0x21F` | `PEUGEOT_HIWORLD_COMMANDS.md` | Frequency tune, band change, preset recall |
| `[ ]` | `0xA4` | CD/CDC Media Downlink | `0x0D` | `0x21F` | `PEUGEOT_HIWORLD_COMMANDS.md` | Play, Pause, Next, Prev, CDC Disc Select |
| `[x]` | `0xAD` | RD4 Audio Source Routing | `0x02`..`0x03` | `0x21F` | `psa_source_arbiter.c`, `can_router.c` | Line In 1 / AUX 1 routing (`0xAD 03 03` / `0xAD 02 01`) |
| `[ ]` | `0xCA` | System Units Downlink | `0x02` | `0x15B` | `PEUGEOT_HIWORLD_COMMANDS.md` | Android settings syncing units to BSI |
| `[x]` | `0xCB` | GPS Date & Time Set | `0x07` | `0x228` | `hiworld_connection.c` | `CANBOX_SPEC_HIWORLD_407_10_BSI_SETTINGS_CLOCK_SYNC.md` |
| `[x]` | `0xC2`/`0xE0` | MCU IAP Flash Transfer | Var | N/A | `hiworld_iap_nuc131.c`, `pack_hiworld_iap.py` | Verified on OpenCanbox hardware (NUC131 FMC vector remap & 136B container) |
| `[ ]` | `0xE1`/`0xE4` | Media Metadata / ID3 to MFD | `13..32` | `0x3B6` | `PEUGEOT_HIWORLD_COMMANDS.md` | Forward song title & artist to car display |
| `[ ]` | `0xF1` | Dedicated RD4 UI Downlink | `0x02`..`0x03` | `0x21F` | `PEUGEOT_HIWORLD_COMMANDS.md` | On-screen Seek, Step, Presets 1..6 translated to stalk frames |
| `[ ]` | `0xF2` | Parking Assist View Mode | `0x02` | N/A | `PEUGEOT_HIWORLD_COMMANDS.md` | Camera overlay view mode switch |
| `[x]` | **ALL** | Universal Downlink ACK | `0x02` | N/A | `proto_hiworld_adapter.c` | `5A A5 01 FF <RxCmd> <(RxCmd-1)&0xFF>` |
