# OpenCanbox vs Original Hiworld Protocol Specification & Customization Guide

**Document File:** `doc/CANBOX_SPEC_HIWORLD_CUSTOMIZATIONS_AND_DIFFERENCES.md`  
**Document Version:** 1.0.0  
**Target Platform:** Automotive Android Infotainment SoC $\longleftrightarrow$ OpenCanbox Core (C99 MCU Firmware)  
**Vehicle Network:** PSA Peugeot 407 & PSA CAN 2004 Platform (Comfort & Body CAN @ 125 kbps)  
**Protocol Layer:** Hiworld (`0x5A 0xA5` Framing, UART 38,400 baud, 8N1)

---

## Table of Contents
1. [Overview & Purpose](#1-overview--purpose)
2. [Architectural Differences Summary](#2-architectural-differences-summary)
3. [Subsystem Customizations & Differences](#3-subsystem-customizations--differences)
   - 3.1 [Vehicle Health Alerts & Diagnostic Journal (`0x42` / `0x2F`)](#31-vehicle-health-alerts--diagnostic-journal-0x42--0x2f)
   - 3.2 [Dual-Zone Climate Control & Air Quality (`0x31`)](#32-dual-zone-climate-control--air-quality-0x31)
   - 3.3 [Trip Computer & Telemetry Paging (`0x13`, `0x14`, `0x15`)](#33-trip-computer--telemetry-paging-0x13-0x14-0x15)
   - 3.4 [Doors, Body & Safety Status (`0x12`)](#34-doors-body--safety-status-0x12)
   - 3.5 [Parking Radar & Distance Zone Mapping (`0x41`)](#35-parking-radar--distance-zone-mapping-0x41)
   - 3.6 [Steering Stalk & Remote Keys (`0x11`)](#36-steering-stalk--remote-keys-0x11)
4. [Serial Bus Management & Link Reliability](#4-serial-bus-management--link-reliability)
   - 4.1 [Delta Detection vs Continuous Spam](#41-delta-detection-vs-continuous-spam)
   - 4.2 [Head Unit Silent Reboot & State Resync](#42-head-unit-silent-reboot--state-resync)
   - 4.3 [Keep-Alive Heartbeat](#43-keep-alive-heartbeat)
5. [Tracking & Maintenance Workflow for Future Customizations](#5-tracking--maintenance-workflow-for-future-customizations)

---

## 1. Overview & Purpose

Commercial aftermarket CAN box adapters (e.g. factory Hiworld / Simple Soft / Raise dongles) were originally built using generic, one-size-fits-all firmware. When installed in PSA Peugeot 407 vehicles, the OEM adapter exhibits several critical bugs, protocol quirks, and missing features:

1. **Alert text corruption:** Real-time popups and diagnostic sequences show nonsensical warnings (e.g., "Seatbelts buckled" instead of "Diagnosis in progress", or "Bonnet open" instead of "Electronic anti-theft faulty").
2. **Climate control state glitches:** Adjusting fan speed or recirculation triggers unwanted toggles between manual recirculation and automatic Air Quality Sensor (AQS) mode.
3. **Serial bus saturation:** The OEM box spams full trip computer and door frames on every raw CAN arrival, starving the 38,400 baud serial connection and causing UI lag.
4. **Desynchronization upon Android reboot:** If the Android Head Unit silently restarts without cycling the car's ignition, the commercial box stops synchronizing state until the engine is power-cycled.

This document formally records all customizations, bug fixes, architectural enhancements, and wire protocol differences between **OpenCanbox Core** and the **Original Commercial Hiworld Adapter**.

---

## 2. Architectural Differences Summary

| Subsystem | Commercial Hiworld Adapter | OpenCanbox Core Implementation |
|---|---|---|
| **Alert IDs (`0x42`)** | Raw byte passthrough without context; collisions on `0x0008` (Door vs Brake), false door triggers on padding `0xFF` | Context-aware translation to canonical Hiworld codes (`PSA_HIWORLD_ALERT_*`); `door_mask != 0 && door_mask != 0xFF` guards against false door alerts on diagnostic frames |
| **Custom Alerts (`0xEA`)** | Not supported | Full concurrent transmission of Custom Extended Protocol `Cmd 0xEA` (15-bit native CAN IDs, status byte $D2$, 5-byte records, empty clearance `5A A5 04 EA 00 00 00 00 ED`, CHECK debounce reset via `0x0000`) |
| **Alert Journal (`0x120`)** | Often omitted or imperfectly decoded | Full Mode A 3-block multiplexed reassembly (`0x7C`, `0xBC`, `0xFC`) + Mode B fallback ISO-TP; 168-bit zero-heap bitmap extractor |
| **Alert UI Flooding** | Periodic broadcast of 24-byte table triggers continuous floating overlay toasts on Android | Strict edge-triggered single alerts (0 &rarr; 1 transitions only); 24-byte summary sent only on change or `0x2F` query |
| **HVAC Air Quality (`0x31`)** | Frame `0x1E3` bit 4 mistaken for AC compressor; overrides `aqs_auto = false` on manual fan change | Proper bit separation between AQS auto-intake (`0x1E3` bit 4) and AC compressor status; independent AQS state machine |
| **Trip Telemetry (`0x13`-`0x15`)** | Unthrottled continuous broadcast on CAN arrival; no sentinel handling for uninitialized sensors | Delta-detection dispatch with rate-limiting; Motorola BE decoding; `0xFF` / `0xFFFF` sensor sentinels normalized |
| **Door Status (`0x12`)** | Rapid periodic flooding; boot/bonnet discrete bits frequently desynced | State-change dispatch with slow periodic heartbeat; precise individual door, boot, bonnet, and handbrake decoding |
| **Parking Radar (`0x41`)** | Fixed rear-only or erratic front sensor support on older firmware revisions | Bidirectional front & rear 4-sensor bar mapping (0..4); reverse gear interlock; non-blocking radar timeout clear |
| **Serial Bus Protection** | Continuous streaming saturating 38,400 baud bus | SPSC lock-free ring buffer with bitwise masking; cooldown timers per packet type |
| **Reboot Recovery** | No proactive resync; state remains blank if HU reboots while driving | Automatic burst resync on handshake (`0x90`, `0xCA`) + 60-second periodic full-state sync heartbeat |

---

## 3. Subsystem Customizations & Differences

### 3.1 Vehicle Health Alerts & Diagnostic Journal (`0x42` / `0xEA` / `0x2F`)

#### A. Diagnostic Cycle & Real-Time Alert Disambiguation (`0x1A1`)
- **Original Adapter & Android HU Limitation:**
  The commercial box and Android Head Unit (`PeugeotDataController.getWarningString()`) rely on a dedicated sparse-switch for Command `0x42`. Passing raw PSA CAN Alarm IDs (`0x0074`..`0x0085`) causes the Android app to fall back to `unknownInformation` or misfire as Auto Lights (`0x0081`). Furthermore, generic alert `0x0008` arrives with padding `door_mask == 0xFF` or `0x00`, which naive parsers misread as a Front-Left Door open event (`door_mask & 0x01 != 0`).
- **OpenCanbox Core:**
  1. **Canonical Hiworld Code Translation:** `psa_can_alarm_id_to_hiworld_code()` translates native PSA CAN Alarm IDs into canonical Hiworld codes (`PSA_HIWORLD_ALERT_*`) matching the Android APK switch table (`0x0011`..`0x0014` doors, `0x0008` braking error, `0x0069` ABS, `0x0081` auto lights, `0x0083` auto wipers, `0x00F0` airbag, etc.).
  2. **Door Mask Disambiguation:** Frame `0x1A1` uses Alarm ID `0x0008` for both door alerts and braking system faults. OpenCanbox strictly tests `door_mask != 0 && door_mask != 0xFF`:
     - If genuine door bits are present (`0x01`, `0x02`, `0x04`, `0x08`, `0x10`), it translates to specific Hiworld door codes (`0x0011` Front Left, `0x0012` Front Right, `0x0013` Rear, `0x0014` Boot).
     - If `door_mask == 0xFF` or `0x00` (diagnostic padding or brake fault), it maps to `PSA_HIWORLD_ALERT_HANDBRAKE` (`0x0008`), correctly displaying "braking system faulty" on the Android HU without false door alerts.
  3. **Diagnostic Cycle CAN ID Mappings:**
     - `0x007E` -> `PSA_HIWORLD_ALERT_ANTIPOLLUTION` (`0x0068`)
     - `0x0139` -> `PSA_HIWORLD_ALERT_AUTO_WIPERS` (`0x0083`)
     - `0x00C9` -> `PSA_HIWORLD_ALERT_TPMS_UNDER_FL` (`0x00A0`)
     - `0x0078` -> `PSA_HIWORLD_ALERT_AIRBAG` (`0x00F0`)
  4. **Dual-Trigger State Tracking:** An alert is recognized as active when `(display_req || popup_active) && can_alarm_id != 0`. It is cleared only when both flags drop to 0 or `can_alarm_id == 0`.

#### B. Alert Journal Reassembly (`0x120`)
- **Original Adapter:** Often fails to parse the 3 multiplexed blocks on Peugeot 407.
- **OpenCanbox Core:** Implements zero-heap block reassembly:
  - Block 1 (`(data[0] >> 6) == 1`, `0x7C`): Bytes 0..6 of 21-byte alert bitfield.
  - Block 2 (`(data[0] >> 6) == 2`, `0xBC`): Bytes 7..13 of 21-byte alert bitfield.
  - Block 3 (`(data[0] >> 6) == 3`, `0xFC`): Bytes 14..20 of 21-byte alert bitfield.
  - Reverse-lookup mapping from bit index to CAN alarm ID via `Alarm_BitToIndex_Tab` and `Alarm_IndexToPointer_Tab`.

#### C. Custom Extended Alert Protocol (`Cmd 0xEA`) & Independent Transmission Architecture
- **Motivation:**
  Stock Hiworld `Cmd 0x42` uses a compressed 16-bit code space that forces compromises (e.g. mapping different vehicle faults to the same sparse code) and mixes real-time popup toasts with persistent diagnostic lists. In a modified Android Head Unit app (`com.qf.vehicle` with `PsaExtendedAlertManager`), `Cmd 0xEA` provides native 15-bit PSA CAN alert identification, explicit severity categorization, and decoupling between floating popups and the diagnostic journal.
- **Buffer Separation & Compilation Parameter:**
  Transmitting both `0x42` and `0xEA` in the same tick causes both frames to be concatenated back-to-back in a single UART write buffer (`5A A5 02 42 ... 5A A5 05 EA ...`), which can overflow 32/64-byte receiver FIFOs on Android MCUs or cause UI toast collision.
  Therefore, alert frame transmission is controlled by compile-time configuration parameters:
  - `HIWORLD_ENABLE_ALERT_0XEA` (default: `1`): Transmits Custom Extended Protocol `Cmd 0xEA`.
  - `HIWORLD_ENABLE_ALERT_0X42` (default: `0`): Disabled by default so that only `0xEA` is transmitted without buffer concatenation. Set `-DHIWORLD_ENABLE_ALERT_0X42=1` when compiling for legacy head units requiring `0x42`.
- **Custom Extended `Cmd 0xEA` Features:**
  - **Single Toast Popups (`Len = 0x05`):** Carries the 15-bit PSA CAN alarm ID with control byte $D2$. $D2$ formats both Bit 7 (`0x80` Active) and Bit 6 (`0x40` Info / Confirmation requirement `mInfo`), ensuring the toast popup displays without polluting `activeAlerts`.
  - **Summary Tables (`Len = 4 + 5*N`):** Synthesized from CAN `0x120` with 5-byte records containing alert ID, severity (`STOP` 2, `SERVICE` 1, `INFO` 0), category, and detail parameters.
  - **Empty Clearance (`5A A5 04 EA 00 00 00 00 ED`):** Transmitted when all faults clear to blank the diagnostic journal.
  - **Cockpit CHECK Sequence:** Streams Step 1 (`0x00F0` start), rolling faults, Step 3 (`0x0000` inter-alert blanking to reset the Head Unit's debounce filter), and Step 4 (`0x00F1` completion).
  - **Downlink Query `Cmd 0x2F`:** Immediately replies with the cached summary table upon receiving `5A A5 01 2F 00 2F`.

---

### 3.2 Dual-Zone Climate Control & Air Quality (`0x31`)

#### The AQS Air Intake Glitch Fix
- **Original Adapter Bug:**
  In the commercial firmware, frame `0x1E3` byte 0 bit 4 was mistakenly parsed as the Air Conditioning compressor flag (`ac_on`). Whenever the driver adjusted the blower speed manually, frame `0x1E3` was received with bit 4 cleared, causing the firmware to forcefully overwrite `aqs_auto = false` (Auto Air Recirculation turned off).
- **OpenCanbox Core Fix:**
  - `0x1D0` byte 4 bit 4 correctly represents manual Recirculation, and bit 5 represents forced fresh air.
  - `0x1E3` byte 0 bit 4 correctly represents Auto Air Intake (AQS - Air Quality System).
  - **Front Defrost (`0x19` / `UNFROST_FRONT`) Decoupling:** When front defrost is selected, `0x1D0` reports `0x19` in Byte 0 and `0x20` in Byte 4 (physical flap opening to outside air). OpenCanbox decodes `front_max_defrost = true` while preserving the driver's active AQS Auto Intake setting (`aqs_auto = true` as broadcast by `0x1E3` byte 0 bit 4), matching OEM Canbox ground truth and preventing rapid cycling between auto and fresh air.
  - OpenCanbox keeps these states strictly decoupled in `vehicle_climate_t`: adjusting fan speed, temperature, or activating front defrost never causes intake state chatter.

---

### 3.3 Trip Computer & Telemetry Paging (`0x13`, `0x14`, `0x15`)

- **Original Adapter:**
  Transmits 0x13 (Instant), 0x14 (Trip 1), and 0x15 (Trip 2) back-to-back at high rates whenever any CAN frame triggers, consuming up to 60% of UART bandwidth.
- **OpenCanbox Core:**
  - **Delta Detection:** Only broadcasts trip packets when telemetry values change.
  - **Rate Limiting:** Enforces maximum update rates (1-2 Hz for slow metrics).
  - **Downlink Trip Reset:** Translates Android HU reset buttons into PSA Comfort CAN reset frames on `0x221`.
  - **Sensor Sentinel Filtering:** Filters raw values `0xFF`, `0xFFFF`, or `0xFFFFFF` indicating uninitialized/disconnected sensors, preventing erratic UI flashes.

---

### 3.4 Doors, Body & Safety Status (`0x12`)

- **Original Adapter:**
  Repeatedly floods packet `0x12` at 10 Hz even when no doors are changing, causing unnecessary CPU load on the Android SoC.
- **OpenCanbox Core:**
  - Dispatches `0x12` immediately upon state transition (door opened or closed, handbrake changed).
  - Low-priority background sync timer maintains a 1 Hz heartbeat to guarantee display consistency without bus spam.

---

### 3.5 Parking Radar & Distance Zone Mapping (`0x41`)

- **Original Adapter:**
  Only supports rear radar (`0x260`) on older revisions; front radar (`0x270` / `0x0E1`) is either dropped or improperly mapped to side zones.
- **OpenCanbox Core:**
  - Full support for both Rear Radar (`0x260`) and Front Radar (`0x270` / `0x0E1`).
  - Converts raw vehicle distance steps into 4-bar Hiworld visualization (`0x00` clear to `0x04` danger/critical).
  - Gated by reverse gear and speed threshold to avoid false chimes while driving forward above parking speeds.

---

### 3.6 Steering Stalk & Remote Keys (`0x11`)

- **Original Adapter:**
  Rotary encoder dials on the Peugeot stalk (e.g. scroll wheel) often skip steps or suffer from long debounce delays.
- **OpenCanbox Core:**
  - Event-driven dispatch for press and release events (`0x0F6` / `0x128`).
  - Immediate packet emission on rotary encoder step changes with zero delay.

---

## 4. Serial Bus Management & Link Reliability

The serial connection operates at **38,400 baud (3,840 bytes/second)**. An uncontrolled CANbox can easily saturate this bus.

### 4.1 Delta Detection vs Continuous Spam
OpenCanbox enforces a strict state-cache architecture (`can_router.c`):
- Core state is cached in `s_current_state` and compared against `s_last_sent_state`.
- Packets are serialized and dispatched **only when a delta is detected**.
- Periodic emissions are throttled:
  - Fast telemetry (speed, RPM, radar): 10–20 Hz max.
  - Slow telemetry (climate, trip, temperature): 1–2 Hz max.

### 4.2 Head Unit Silent Reboot & State Resync
If the Android Head Unit restarts without cutting vehicle power:
- **Handshake Response:** Receiving Command `0x90` (Handshake) or `0xCA` (Version Query) triggers an immediate full burst of all cached state packets (Doors, Climate, Trip, Alerts, Radar).
- **Slow 60-Second Periodic Resync:** A background timer (`can_router_periodic_100ms`) retransmits all cached state frames every 60 seconds.

### 4.3 Keep-Alive Heartbeat
Every 100 ms, OpenCanbox feeds the serial bus with standard keep-alive frames (`5A A5 01 FF 01 00`) when the bus is idle, preventing Android UART timeout disconnections.

---

## 5. Tracking & Maintenance Workflow for Future Customizations

When adding new customizations or protocol adaptations in the future, adhere to the following workflow:

1. **CAN Frame & APK Analysis:**
   - Record CAN logs on the physical car or simulator (`scenarios/log_canbox.py` or `dump_*.log`).
   - Compare with decompiled Head Unit code (`PeugeotDataParser`, `PeugeotDataController`).
2. **Document Specification:**
   - Add the feature or signal table to the relevant topic spec (`doc/CANBOX_SPEC_HIWORLD_407_*.md`).
   - Update this document (`doc/CANBOX_SPEC_HIWORLD_CUSTOMIZATIONS_AND_DIFFERENCES.md`) detailing the exact difference from the commercial adapter.
3. **C99 Core Implementation:**
   - Profile decoder: `src/profiles/peugeot_407.c` (Layer 2).
   - Canonical cache: `include/core/canbox_state.h`, `src/core/can_router.c` (Layer 3).
   - Hiworld adapter: `src/protocols/proto_hiworld_adapter.c` (Layer 5).
4. **Automated Verification:**
   - Transcribe proven vectors into `test/test_protocol_parser/test_peugeot_407.c` or `test/test_integration/`.
   - Run native host test runner: `pio test -e native_test_runner`.
   - Run integration tests: `pio test -e integration_test`.
   - Verify hardware builds: `pio run -e stm32_cbox` and `pio run -e esp32_cbox`.
5. **Issue & Progress Tracking:**
   - Log progress in `doc/TODO_PROGRESS.md` and `Issues.md`.
