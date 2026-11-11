# SVT Comparison, Potential Firmware Bugs & OEM Compatibility Tracker

**Document Version:** 1.0.0  
**Target Hardware:** OpenCanbox Core (STM32/NUC131) vs Commercial OEM Hiworld Adapter (`H1H2PA123A-240717`)  
**Network:** PSA Peugeot 407 (PSA CAN2004 Comfort Bus @ 125 kbps)  
**Related Test Harness:** System Validation Test (`svt/run_svt.py`)  

---

## 1. Overview & Purpose

This document serves as the authoritative tracker for:
1. **Parity and behavioral differences** discovered by running the 156-scenario SVT suite against physical commercial OEM hardware (`hiworld_oem_407_A`) across three physical bus setups:
   - Standalone / Unterminated
   - Terminated with Matrix cluster LCD
   - Coexisting with the physical Multi-Function Display (EMF-C / LCD)
2. **Potential bugs and architectural regressions** identified in **OpenCanbox Core** (custom firmware) that could cause bus contention, arbitration clashes, or protocol rejection.
3. **Strict backward compatibility guidelines** ensuring OpenCanbox Core remains a 100% transparent, drop-in replacement for OEM commercial adapters (Hiworld, and future Raise adapters) while preserving advanced custom features.

---

## 2. SVT Subsystem Comparison Matrix

The table below summarizes commercial OEM hardware performance in SVT compared to the target capabilities of OpenCanbox Core:

| Subsystem Suite | SVT Scenarios | OEM Pass Rate | Commercial OEM Behavior | OpenCanbox Core Target Behavior | Parity Status |
|---|:---:|:---:|---|---|:---:|
| **`power_management`** | 7 | 3 PASS / 3 SKIP / 1 XFAIL | Passive listener. Wakes within 36 ms on CAN activity. Bus sleep unmonitored. | Active power management, deep sleep on bus silence, instant wake on CAN RX. | ✅ Full Parity |
| **`trip_computer`** | 8 | 6 PASS / 2 SKIP | Decodes `0x13`, `0x14`, `0x15`, `0x1A`. Skips virtual trip resets (`0x167`). | 100% OEM payload parity + 20-frame Virtual MDF trip reset burst (`0x167` 2000 ms @ 100 ms). | ✅ Full Parity |
| **`keys`** | 21 | 21 PASS (100%) | Decodes steering stalk (`0x21F`), wiper tips, and RD4 center console (`0x3E5`). | Identical key code mappings, edge-triggered bursts, gated by injection lock. | ✅ Full Parity |
| **`doors`** | 13 | 12 PASS / 1 FAIL | Decodes driver, passenger, rear doors, trunk, multi-door, and reverse gear. | Identical mapping (`Cmd 0x12`). Diverges only on `DOORS-10` handbrake bitfield. | ⚠️ Minor Divergence |
| **`radar_sas`** | 16 | 11 PASS / 5 FAIL | Decodes rear/front parking radar (`0x41`) and SAS steering angle. | ROM curve discretization (`s_rom_radar_curves[8]`) and quiescent sentinel clearing (`FC FC FE`). | ✅ Full Parity |
| **`climate`** | 10 | 4 PASS / 6 FAIL | Setpoints, LO/HI sentinels, Recirc/AQS, ambient temp pass. Discrete fan/defrost steps. | Full dual-zone HVAC decoding with single-authority arbitration (`0x1E3` vs `0x1D0`). | ⚠️ Fan Step Parity |
| **`fault_injection`** | 8 | 4 PASS / 4 FAIL | Recovers from UART framing noise, rejects bad checksums, handles ISO-TP timeout. | Robust state-machine recovery and lock-free SPSC buffer protection. | ✅ Full Parity |
| **`base_bsi_core`** | 12 | 3 PASS / 9 FAIL | DRL, auto-lights, dimmer pass. Fails on display keepalives and personalization writes. | Emulates complete BSI configuration profile 1 and synchronizes menu state via `0x15B`. | 🌟 OpenCanbox Exclusive |
| **`audio`** | 11 | 3 PASS / 8 FAIL | Internal CD playback and radio OFF pass. Lacks CD text reassembly and CDC arbitration. | Reassembles CD/MP3 text (`0x85`), manages source arbitration between HU and RD4. | 🌟 OpenCanbox Exclusive |
| **`handshake_link`** | 10 | 6 PASS / 4 FAIL | Responds to `0x24`, `0x30`, `0x6A` query, and `0x7D 25 01` injection gate. | Full 4-step connection handshake, periodic `0xFF` link ping, full state resync (`0x6A`). | ✅ Full Parity |
| **`alerts`** | 10 | 2 PASS / 8 FAIL | Legacy sparse alerts (`0x42`) only. Fails extended alert logs (`0xEA`). | Decodes real-time `0x1A1` and reassembles multi-frame ISO-TP journal (`0x120` -> `0xEA`). | 🌟 OpenCanbox Exclusive |
| **`clock_sync`** | 6 | 1 PASS / 5 FAIL | Downlink regional commands pass. Does NOT emulate MFD clock keepalives (`0x3F6`). | Emulates `0x3F6`, forwards Android GPS time (`0xCB`) to CAN `0x228`. | 🌟 OpenCanbox Exclusive |
| **`source_arbiter`** | 6 | 1 PASS / 5 FAIL | Stalk media key handover passes. Does NOT emulate Virtual CDC (`0x3A6`). | Virtual CDC emulator provides closed-loop audio switching on Android AUX. | 🌟 OpenCanbox Exclusive |
| **`tpms`** | 13 | 3 PASS / 10 FAIL | Standby rejection and delta dedup pass. Does NOT synthesize numeric pressures. | Converts discrete CAN `0x1E1` tire alarms into synthetic numeric pressures (Bar). | 🌟 OpenCanbox Exclusive |
| **`jbl_dsp`** | 8 | 0 PASS / 8 FAIL | Zero support for JBL amplifier or DSP equalizer commands. | Decodes and controls JBL amplifier DSP (`0x1E5` / `0x1A5`, `Cmd 0x56` / `0x86`). | 🌟 OpenCanbox Exclusive |

---

## 3. Potential Bugs in Custom Firmware (OpenCanbox Core)

The table below tracks confirmed or suspected edge-case bugs in OpenCanbox Core revealed during physical bus and OEM testing:

| Bug ID | Subsystem | Severity | Description & Root Cause | Affected Files | Status |
|---|---|:---:|---|---|:---:|
| **`BUG-CFW-01`** | `car_config` | **CRITICAL** | **Virtual MFD Bus Collision with Physical Display:** `psa_car_config_periodic_100ms()` unconditionally transmits `0x0DF`, `0x1ED`, `0x15B`, `0x525`, `0x3F6`, and `0x167` every 100–1000 ms even when a physical MFD is actively broadcasting on `can0`. This causes CAN bit-arbitration clashes and error frames. | `src/profiles/psa_407_car_config.c` | **Resolved** |
| **`BUG-CFW-02`** | `car_config` | **HIGH** | **Incomplete MFD Presence Snooping:** Only `0x525` is wired to `psa_car_config_notify_mdf_seen()`. If `0x525` is delayed or absent, broadcasts of `0x15B`, `0x3F6`, or `0x0DF` are ignored, leaving `is_mdf_present()` returning `false`. | `src/profiles/profile_psa.c` | **Resolved** |
| **`BUG-CFW-03`** | `car_config` | **MEDIUM** | **BSI Personalization Contention (`0x15B`):** When a physical MFD is present, physical button presses send `0x15B` to update settings. If Android HU simultaneously attempts to send config downlinks, the firmware must yield authority to the physical MFD rather than transmitting competing `0x15B` frames. | `src/profiles/psa_407_car_config.c` | **Open** |
| **`BUG-CFW-04`** | `telemetry` | **MEDIUM** | **`Cmd 0x1A` Telemetry Framing Discrepancy:** The commercial OEM box emits a 12-byte `WcComDataParser` frame starting with `0x35` with speed at bytes 3–4 (0.1 km/h scale), steering angle at bytes 6–7, and RPM at bytes 9–10. OpenCanbox Core now matches this layout on `Cmd 0x1A` and provides `build_hiworld_telemetry_eb()` (`Cmd 0xEB`) for extended fluids/fuel. | `src/protocols/hiworld_psa_builders.c` | **Resolved** |
| **`BUG-CFW-05`** | `telemetry` | **LOW** | **Trip Reset Arbitration on Physical Stalk vs UART:** When a physical MFD is installed, trip resets are performed by holding the wiper stalk tip button (`0x21F`). Emitting synthetic `0x167` trip resets from UART downlinks simultaneously may cause display desync. | `src/profiles/psa_407_telemetry.c` | **Open** |
| **`BUG-CFW-06`** | `router` | **LOW** | **UART TX Buffer Saturation under Rapid Churn:** During continuous bus activity (e.g. rapid radar or SAS changes), multiple dirty flags triggered simultaneously can flood the 38,400 baud serial channel, resulting in dropped frames if throttling cooldowns are violated. | `src/core/can_router.c` | **Open** |
| **`BUG-CFW-07`** | `protocol` | **MEDIUM** | **Hiworld `CMD 0x6A` Query All & Single Packet Handling:** Commercial firmware responds to `0x6A 00` by sweeping all 25 uplink records and triggering resync. Single queries (`cmd_id`) must immediately serialize and send the target packet. | `src/protocols/proto_hiworld_adapter.c` | **Resolved** |
| **`BUG-CFW-08`** | `protocol` | **MEDIUM** | **Downlink CAN Injection Gate (`0x7D 25 01`):** Commercial box drops console keys and audio control commands until `0x7D {0x25, 0x01}` is received. OpenCanbox Core gates `0x21` and `0xA1..0xA4` with a 5000 ms fallback unlock. | `src/protocols/proto_hiworld_adapter.c` | **Resolved** |
| **`BUG-CFW-09`** | `telemetry` | **MEDIUM** | **BSI Trip Computer Reset Burst (`0x167`):** PSA BSI requires > 1.5s continuous press to clear Trip 1 or Trip 2. A 20-frame burst at 100 ms intervals (2000 ms duration) ensures 100% reliable reset. | `src/profiles/psa_407_car_config.c` | **Resolved** |
| **`BUG-CFW-10`** | `radar` | **LOW** | **Radar Inactive Sentinel & Curve Parity:** Quiescent sentinel `d[3..5] == {0xFC, 0xFC, 0xFE}` must clear all 8 zones to `0xFF`. Raw 3-bit readings mapped via `s_rom_radar_curves[8]`. | `src/profiles/psa_407_hvac_body.c` | **Resolved** |

---

## 4. Backward Compatibility Principles with OEM Canbox

To maintain seamless compatibility with Android Head Unit software (`com.qf.vehicle` / `QF_Canbus_system`):

### 4.1 Dual Operating Modes (Auto-Detection)
OpenCanbox Core must support two operational postures:
1. **Autonomous Virtual MFD Mode (Physical Display Absent):**
   - Active when no MFD frames are detected on `can0`.
   - OpenCanbox Core synthesizes `0x3F6` (clock/trip), `0x167` (trip page), `0x0DF` (heartbeat), `0x525` (diag), and `0x228` (clock sync).
   - Allows full trip computer, clock setting, and BSI configuration directly on the Android touchscreen.
2. **Auto-Companion Mode (Physical Display Present):**
   - Automatically entered within 500 ms of detecting physical MFD traffic (`0x15B`, `0x3F6`, `0x0DF`, `0x525`).
   - Suppresses all CAN transmission of supervisory frames (`0x3F6`, `0x167`, `0x0DF`, `0x525`, `0x1ED`).
   - Acts identically to the commercial OEM Hiworld box: strictly listens to CAN and converts vehicle state into UART uplinks.
   - Forwards physical MFD `0x15B` configuration menu frames to Android UART via `Cmd 0x72`.

### 4.2 Baseline Wire Packet Parity
The custom firmware must guarantee that baseline UART packets match commercial OEM definitions byte-for-byte:
- **`Cmd 0x11`:** Key events (Volume, Track, Source, Rotary, Wiper tips).
- **`Cmd 0x12`:** Doors, trunk, hood, reverse gear.
- **`Cmd 0x13`, `0x14`, `0x15`:** Instantaneous consumption, Trip 1, and Trip 2.
- **`Cmd 0x21`:** RD4 center console buttons (`MENU`, `OK`, `ESC`, `DARK`, `CLIM`, `TRIP`).
- **`Cmd 0x31`:** Dual-zone climate status (temperatures, fan speed, airflow distribution).
- **`Cmd 0x41`:** Front and rear parking radar distances.
- **`Cmd 0x72`:** Vehicle configuration options read from BSI Profile 1.

### 4.3 Additive-Only Extensions
Features unique to OpenCanbox Core must never replace or break baseline packets:
- **Alert Journals (`Cmd 0xEA`):** Emitted as an advanced companion to legacy `Cmd 0x42`. Head units lacking `0xEA` support simply ignore it and consume `0x42`.
- **JBL DSP Control (`Cmd 0x56` / `0x86`):** Emitted only when an active JBL amplifier (`0x1E5`) is detected on CAN.
- **Virtual CDC Emulation (`Cmd 0xAD` / `0x3A6`):** Handled transparently through audio source switching without corrupting standard tuner/CD telemetry.

---

## 5. Architectural Alignment for Raise Canbox Adapters

The OpenCanbox Core codebase is architecturally structured to support Raise adapters alongside Hiworld:

```
                  +---------------------------------------+
                  |         CAN Hardware Interface        |
                  |                (can0)                 |
                  +-------------------+-------------------+
                                      |
                                      v
                  +---------------------------------------+
                  |         L2: PSA 407 Decoders          |
                  |  (psa_407_*.c, psa_source_arbiter.c)  |
                  +-------------------+-------------------+
                                      |
                                      v
                  +---------------------------------------+
                  |      L3: Universal Vehicle State      |
                  |       (vehicle_state_t Cache)         |
                  +---------+-------------------+---------+
                            |                   |
            +---------------+                   +---------------+
            v                                                   v
+-----------------------+                           +-----------------------+
|  L4/L5: Hiworld Proto |                           |   L4/L5: Raise Proto  |
| (hiworld_psa_builders)|                           |  (raise_psa_builders) |
|   Framing: 0x5A 0xA5  |                           |     Framing: 0x2E     |
+-----------------------+                           +-----------------------+
```

### 5.1 Multi-Vendor SVT Strategy
1. **Protocol Decoders:**
   - [`svt/lib/protocols/hiworld.py`](svt/lib/protocols/hiworld.py) decodes Hiworld UART framing into normalized `SemanticEvent`s.
   - [`svt/lib/protocols/raise_proto.py`](svt/lib/protocols/raise_proto.py) decodes Raise UART framing (`0x2E ...`) into identical `SemanticEvent`s.
2. **Unified Scenario Execution:**
   - SVT test scenarios (`subsystem-keys.py`, `subsystem-doors.py`, `subsystem-trip_computer.py`) assert against semantic vehicle state (e.g., `event.name == "trip_instant_fuel"`), making the test suite 100% agnostic to whether the DUT is a Hiworld, OpenCanbox, or Raise adapter.
3. **DUT Target Selection:**
   - Target commercial Hiworld: `python3 svt/run_svt.py --dut hiworld_oem_407_A`
   - Target OpenCanbox Core (Hiworld): `python3 svt/run_svt.py --dut opencanbox_hiworld`
   - Target Commercial Raise: `python3 svt/run_svt.py --dut raise_oem_407`
   - Target OpenCanbox Core (Raise): `python3 svt/run_svt.py --dut opencanbox_raise`

---

## 6. Action Items & Verification Roadmap

- [x] **Step 1 (Fix `BUG-CFW-01`):** Wrap `psa_car_config_periodic_100ms()` transmissions in `src/profiles/psa_407_car_config.c` with `if (!psa_car_config_is_mdf_present())`.
- [x] **Step 2 (Fix `BUG-CFW-02`):** Update `src/profiles/profile_psa.c` to invoke `psa_car_config_notify_mdf_seen()` on receiving CAN IDs `0x15B`, `0x3F6`, `0x0DF`, or `0x525`.
- [x] **Step 3 (Fix `BUG-CFW-04`):** Align `build_hiworld_telemetry_1a()` with OEM `0x35` / `WcComDataParser` layout and add `build_hiworld_telemetry_eb()` (`Cmd 0xEB`).
- [ ] **Step 4 (SVT Run on OpenCanbox Core):** Execute full SVT suite against OpenCanbox Core firmware (`--dut opencanbox_hiworld`) with and without physical MFD to verify zero bus contention.
- [x] **Step 5 (Raise SVT Bringup):** Implemented `svt/lib/protocols/raise_proto.py` and calibrated `svt/duts/raise_oem_407.yaml` for commercial Raise adapter (`PG407S V`).
- [x] **Step 6 (Fix `BUG-CFW-07` through `BUG-CFW-10`):** Implemented `CMD 0x6A` query handler (`0x00` and specific `cmd_id`), `0x7D {0x25, 0x01}` injection safety gate, 20-frame `0x167` trip reset burst, and `0x0E1` `FC FC FE` sentinel clearing with ROM curves.
- [x] **Step 7 (SVT Scenarios for Hiworld Compatibility):** Added `HSHK-08` (Query All), `HSHK-09` (Single Query), `HSHK-10` (Injection Gate) to `svt/subsystem-handshake_link.py`, verified burst counting in `TRIP-05`/`TRIP-06`, and added commercial sentinel checks in `RADAR-05`.

