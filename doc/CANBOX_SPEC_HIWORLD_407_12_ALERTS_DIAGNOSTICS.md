# Hiworld (WC / 0x5AA5) Vehicle Alerts & Diagnostic Journal Specification
## Master Catalog: Peugeot 407 Warning Architecture, Real-Time Popups, ISO-TP Alert Journal & Android UI Integration

**Document File:** `CANBOX_SPEC_HIWORLD_407_12_ALERTS_DIAGNOSTICS.md`  
**Document Version:** 1.0.0  
**Target Platform:** Automotive Android Infotainment SoC $\longleftrightarrow$ Pure C99 CAN Box Firmware  
**Vehicle Network:** Peugeot 407 & PSA Platform (CAN 2004 / PSA 15 Comfort & Body CAN @ 125 kbps)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` Framing, Additive Checksum)  
**Command Identifiers:** `0x42` (`HIWORLD_CMD_WARNING_INFO`), `0x2F` (`HIWORLD_CMD_DIAGNOSTIC_QUERY`)  
**Standard Reference:** Decompiled Android Automotive Core (`com.qf.vehicle.band.peugeot.parse.wc.PeugeotDataParser`, `PeugeotDataController`, `DignosticFrgment`) & RT4 Magneti Marelli Firmware SW 8.31

---

# Table of Contents
1. [Overview & Architectural Context](#1-overview--architectural-context)
2. [Vehicle CAN Transport Protocols](#2-vehicle-can-transport-protocols)
   - 2.1 Primary Immediate Alert Frame: `0x1A1` (`MSG_BSI_CDE_PTR_MESSAGE`)
   - 2.2 Historical Alert Journal Transport: `0x120` (`MSG_JOURNAL_ALERTES`)
3. [Hiworld 5AA5 Serial Protocol Serialization](#3-hiworld-5aa5-serial-protocol-serialization)
   - 3.1 Format A: Multi-Alert Summary (`Len = 0x18`)
   - 3.2 Format B: Single Real-Time Alert (`Len = 0x02`)
4. [Master Signal & Alert ID Mapping Table](#4-master-signal--alert-id-mapping-table)
5. [Pure C99 Implementation Reference](#5-pure-c99-implementation-reference)
   - 5.1 Canonical Vehicle State (`include/core/can_router.h`)
   - 5.2 CAN Profile Decoder (`src/profiles/peugeot_407.c`)
   - 5.3 Protocol Driver Interface & Adapter (`src/protocols/proto_hiworld_adapter.c`)
6. [Desktop Test Harness & Verification Vectors](#6-desktop-test-harness--verification-vectors)

---

# 1. Overview & Architectural Context

In Peugeot 407 vehicles equipped with factory infotainment (RD4, RT3, RT4, RT5), health and safety warnings originate from the **Body Systems Interface (BSI)** and dedicated vehicle ECUs (ABS/ESP, Airbag, TPMS, Engine Management). The system supports two distinct presentation modes:

1. **Immediate Modal Alerts:** When a fault or safety alert triggers while driving (e.g. low fuel, puncture, door opened in motion, engine overheat), the BSI broadcasts high-priority frame `0x1A1`. The Head Unit displays a modal dialog or floating toast (`WindowTextWarning`) accompanied by an acoustic chime.
2. **Persistent Alert Log (Journal des Alertes):** Drivers browse the **Alert Log** via the `Vehicle Diagnostics` -> `Diagnostic Information` UI menu (`DignosticFrgment`). The BSI transfers the complete active fault table over segmented CAN frames on `0x120`.

```
+------------------------------------------------------------------------------------+
|                         Peugeot 407 Vehicle Network (125 kbps)                     |
|           [BSI Module (0x1A1 / 0x120) | ABS ECU (0x0E6) | Airbag (0x269)]          |
+------------------------------------------------------------------------------------+
                                         │
                                         ▼ (CAN Confort)
+------------------------------------------------------------------------------------+
|                         OpenCanbox Core (Pure C99 Microcontroller)                 |
|   1. L1 HAL: hal_can receives frames 0x1A1 and 0x120                               |
|   2. L2 Profile: psa_decode_alert_message_0x1a1() & psa_process_journal_0x120()    |
|   3. L3 Router: vehicle_alerts_t canonical state cache                             |
|   4. L4 Driver: hu_protocol_send_alert_single() / hu_protocol_send_alerts_summary()|
|   5. L5 Adapter: proto_hiworld_serialize() with Command 0x42 (Format A or B)       |
+------------------------------------------------------------------------------------+
                                         │
                                         ▼ (UART 38400 8N1: 0x5A 0xA5)
+------------------------------------------------------------------------------------+
|                         Android Head Unit (com.qf.vehicle)                         |
|   - UartDataReceiver -> WcVehicleDataRuleHead5aa5 (Checksum & framing verification)|
|   - PeugeotDataParser.parseWarningInfo() (Unpacks mNumber & 16-bit alert codes)    |
|   - PeugeotDataController.getWarningString() (Maps codes to localized resources)  |
|   - DignosticFrgment (UI list) & WindowTextWarning (Real-time toast overlay)       |
+------------------------------------------------------------------------------------+
```

---

# 2. Vehicle CAN Transport Protocols

### 2.1 Primary Immediate Alert Frame: `0x1A1` (`MSG_BSI_CDE_PTR_MESSAGE`)
- **CAN ID:** `0x1A1` (417 decimal)
- **Periodicity:** 100 ms periodic / asynchronous upon alert state transition
- **DLC:** 8 bytes

#### Frame Layout:
```
 Byte 0            Byte 1            Byte 2            Byte 3            Bytes 4..7
+--------+--------+-----------------+--------+--------+-----------------+-----------------------+
| ACT[7] | ALARM_ID_H[6:0]          | DISP[7]| PRI[6:4]| SOUND[3:0]      | PARAM_DOOR_MASK       | CONTEXT_PARAM_DATA    |
+--------+--------+-----------------+--------+--------+-----------------+-----------------------+
```

- **Byte 0 [7] (`ACT`):** `1` = Alert is currently triggered; `0` = Alert acknowledged or cleared.
- **Byte 0 [6:0] & Byte 1 (`ALARM_ID`):** 15-bit native PSA Alert Identifier: `((Byte 0 & 0x7F) << 8) | Byte 1`.
- **Byte 2 [7] (`DISP`):** `1` = Center screen modal popup requested; `0` = Silent update.
- **Byte 2 [6:4] (`PRI`):** Severity level: `0` = Info, `1` = Minor Caution, `2` = Service, `3` = Critical STOP.
- **Byte 2 [3:0] (`SOUND`):** Acoustic chime melody index (0..15).
- **Byte 3 (`PARAM_DOOR_MASK`):** Door bitfield (`0x01`=FL, `0x02`=FR, `0x04`=RL, `0x08`=RR, `0x10`=Boot).
- **Byte 4 (`PARAM_DETAIL`):** Parameter detail (wheel index, bulb index).

---

### 2.2 Historical Alert Journal Transport: `0x120` (`MSG_JOURNAL_ALERTES`)
- **CAN ID:** `0x120` (288 decimal)
- **Periodicity:** 1000 ms periodic cycle
- **Payload:** 21 bytes (168 consecutive bits) transmitted over 3 multiplexed blocks.
- **Multiplexing Transport:**
  As verified in real car CAN logs (`dump_2026-10-02_12-15-52.log`) and RT4 firmware disassembly (`Network_Event_Task__22C_NETWORK_PRESENTATION`), PSA uses a **2-bit block header** in Byte 0 bits [7:6]:
  1. **Block 1 (`(data[0] >> 6) == 1`, e.g. `0x7C`):** `data[1..7]` &rarr; bytes 0..6 of 21-byte alert bitfield.
  2. **Block 2 (`(data[0] >> 6) == 2`, e.g. `0xBC`):** `data[1..7]` &rarr; bytes 7..13 of 21-byte alert bitfield.
  3. **Block 3 (`(data[0] >> 6) == 3`, e.g. `0xFC`):** `data[1..7]` &rarr; bytes 14..20 of 21-byte alert bitfield.
  *(A fallback ISO-TP multi-frame decoder with `0x10`, `0x21..0x23` is also supported for synthetic test runners).*

Each set bit `k` in `[0..167]` is mapped to an internal Alarm Index via `Alarm_BitToIndex_Tab[k]`, and then resolved to a 15-bit CAN Alarm ID via `Alarm_IndexToPointer_Tab[alarm_idx]`.

---

# 3. Hiworld 5AA5 Serial Protocol Serialization

### 3.1 Format A: Multi-Alert Summary (`Len = 0x18` = 24 bytes)
Used when broadcasting the active fault list or in response to Android UI resume query (`0x2F`):

```
[0x5A, 0xA5, 0x18, 0x42, D0, D1, D2, D3, D4, D5, ..., D23, Checksum]
```
- **Byte 2 (`Len`):** `0x18` (24 bytes)
- **Byte 3 (`CmdID`):** `0x42` (`HIWORLD_CMD_WARNING_INFO`)
- **Bytes 4..6 (`D0..D2`):** Category flags / reserved (`0x00, 0x00, 0x00`)
- **Byte 7 (`D3`):** `mNumber` — Total count of active alerts (0 to 10; `0` clears the UI)
- **Bytes 8..27 (`D4..D23`):** Up to 10 active alert codes (`mOriginalType`) as 16-bit Big-Endian integers, zero-padded.
- **Byte 28 (`Checksum`):** `((sum(bytes[1..27]) - 1) & 0xFF)`

---

### 3.2 Format B: Single Real-Time Alert (`Len = 0x02`)
Used when a single fault or warning triggers while driving:

```
[0x5A, 0xA5, 0x02, 0x42, (code >> 8) & 0xFF, code & 0xFF, Checksum]
```
- **Byte 2 (`Len`):** `0x02` (2 data bytes)
- **Byte 3 (`CmdID`):** `0x42`
- **Bytes 4..5 (`D0..D1`):** 16-bit Big-Endian alert code (`mOriginalType`)
- **Byte 6 (`Checksum`):** `((0x02 + 0x42 + D0 + D1 - 1) & 0xFF)`

---

# 4. Master Signal & Alert ID Mapping Table

| Fault Condition | PSA CAN ID (`0x1A1`/`0x120`) | Hiworld Code (`mOriginalType`) | Android String Res | Description |
|---|---|---|---|---|
| **Low Fuel Level** | `0x000D` | `0x0001` | `vehicle_warning_145` | Low fuel reserve reached |
| **Engine Overheat (STOP)** | `0x0001` | `0x0003` | `vehicle_warning_4` | Engine coolant temperature too high: STOP |
| **Engine Oil Pressure Low (STOP)** | `0x0002` | `0x0004` | `vehicle_warning_5` | Engine oil pressure low: STOP |
| **Brake System Failure (STOP)** | `0x000F` | `0x0005` | `vehicle_warning_6` | Braking system failure: Brake fluid low |
| **Handbrake Applied** | `0x000C` | `0x0008` | `vehicle_warning_42` | Handbrake engaged while driving |
| **Remote Key Battery Flat** | `0x00DF` | `0x000A` | `vehicle_warning_10` | Keyfob battery flat |
| **Directional Headlamps Faulty** | `0x0195` | `0x000B` | `vehicle_warning_11` | Adaptive swiveling xenon headlights fault |
| **Battery Charge / Alternator** | `0x006B` | `0x000D` | `vehicle_warning_13` | Alternator charging fault |
| **ESP / ASR System Faulty** | `0x006A` | `0x000F` | `vehicle_warning_15` | Dynamic stability control unavailable |
| **Front Left Door Open** | `0x0074` | `0x0011` | `vehicle_warning_17` | FL door open |
| **Front Right Door Open** | `0x0085` | `0x0012` | `vehicle_warning_18` | FR door open |
| **Rear Door Open** | `0x0084` / `0x0081` | `0x0013` | `vehicle_warning_19` | Rear door open |
| **Boot Open** | `0x0080` | `0x0014` | `vehicle_warning_20` | Boot / tailgate open |
| **Service Overdue** | `0x0061` | `0x0061` | `vehicle_warning_97` | Service maintenance overdue |
| **DPF / Particle Filter Clogging** | `0x006F` | `0x0064` | `vehicle_warning_100` | FAP / DPF soot clogging risk |
| **Automatic Gearbox Faulty** | `0x0073` / `0x0202` | `0x0067` | `vehicle_warning_103` | Automatic / EGS gearbox fault |
| **Depollution System Faulty** | `0x006E` | `0x0068` | `vehicle_warning_104` | Anti-pollution system fault |
| **ABS Braking System Faulty** | `0x006C` | `0x0069` | `absError` | Anti-lock braking system failure |
| **Electronic Brakeforce Distribution** | `0x0008` (param) | `0x006A` | `vehicle_warning_106` | EBD / REF failure |
| **Suspension Fault (Max 90 km/h)** | `0x009E` | `0x006B` | `SuspensionError` | AMVAR / Hydractive emergency mode |
| **Suspension System Error** | `0x0072` / `0x00D8` | `0x006C` | `SuspensionSystemError` | Suspension electronic error |
| **Automatic Headlamp Control Faulty** | `0x007F` | `0x0081` | `vehicle_warning_129` | Light sensor / auto headlamp error |
| **Automatic Wiping Faulty** | `0x00CB` | `0x0083` | `vehicle_warning_131` | Rain sensor error |
| **TPMS Under-inflated (FL/FR/RR/RL)**| `0x0004` | `0x009A`..`0x009D` | `vehicle_warning_154`..`157` | Tire pressure under-inflated |
| **TPMS Puncture (FL/FR/Rear)** | `0x0005` | `0x009E`..`0x00A0` | `vehicle_warning_158`..`160` | Sudden pressure loss / Puncture |
| **Sidelight Bulb Faulty** | `0x00D3` | `0x00E3` | `vehicle_warning_227` | Sidelight bulb blown |
| **Dipped Beam Bulb Faulty** | `0x013D` | `0x00E5` | `vehicle_warning_229` | Low beam bulb blown |
| **Brake Light Bulb Faulty** | `0x00CE` / `0x0134` | `0x00E7` | `vehicle_warning_231` | Brake light bulb blown |
| **Reversing Lamp Bulb Faulty** | `0x0092` / `0x0095` | `0x00E8` | `vehicle_warning_232` | Reverse / turn indicator bulb blown |
| **Airbags / Pretensioners Faulty** | `0x0078` | `0x00F0` | `vehicle_warning_240` | Airbag ECU fault |
| **Driver Seatbelt Unfastened** | `0x000B` | `0x012F` | `vehicle_warning_303` | Driver seatbelt unbuckled |
| **Passenger Seatbelt Unfastened** | `0x013A` | `0x0130` | `vehicle_warning_304` | Passenger seatbelt unbuckled |
| **Electronic Immobilizer Fault** | `0x0086` | `0x01FA` | `vehicle_warning_506` | Transponder key unlearned / fault |

---

# 5. Pure C99 Implementation Reference

### 5.1 Canonical Vehicle State (`include/core/can_router.h`)
```c
#define CANBOX_MAX_ACTIVE_ALERTS 10

typedef struct {
    uint16_t alert_code;    /* 16-bit Hiworld alert code (mOriginalType) */
    uint16_t can_alarm_id;  /* 15-bit PSA CAN alarm ID */
    uint8_t  severity;      /* 0=Info, 1=Minor, 2=Service, 3=Stop */
    uint8_t  chime_id;      /* Acoustic chime index (0..15) */
    uint8_t  door_mask;     /* Door bitfield */
    uint8_t  param_detail;  /* Parameter detail (wheel, bulb index) */
    bool     is_active;     /* true if alert currently triggered */
    bool     display_req;   /* true if modal dialog popup requested */
} vehicle_alert_item_t;

typedef struct {
    vehicle_alert_item_t realtime_alert;
    uint16_t             active_codes[CANBOX_MAX_ACTIVE_ALERTS];
    uint8_t              active_count;     /* 0..10 */
    bool                 realtime_updated; /* Single alert state changed */
    bool                 journal_updated;  /* Summary list changed */
} vehicle_alerts_t;
```

### 5.2 CAN Profile Decoder (`src/profiles/peugeot_407.c`)
- Unpacks immediate alert messages from `0x1A1` into `vehicle_alert_item_t`.
- Maps native CAN Alarm IDs to canonical Hiworld codes through `psa_can_alarm_id_to_hiworld_code()`.
- Implements lock-free, zero-heap ISO-TP multi-frame reassembler `psa_process_journal_0x120()` with the 168-bit `Alarm_BitToIndex_Tab` reverse-lookup table.

### 5.3 Protocol Driver Interface & Adapter (`src/protocols/proto_hiworld_adapter.c`)
- Implements `hiworld_send_alert_single()` using Command `0x42` with 2-byte payload.
- Implements `hiworld_send_alerts_summary()` using Command `0x42` with 24-byte payload.
- Dispatches active CAN query to car's BSI via `can_router_query_alert_journal()` upon receiving `HIWORLD_CMD_DIAGNOSTIC_QUERY` (`0x2F`).
- Accepts both standard checksum `(sum - 1) & 0xFF`, sync2 variant `(sum + 0xA5 - 1) & 0xFF`, and `0xD4` from Android APK `forwardType(0x2F)`.

### 5.4 CAN Router Dispatch & Rate Limiting (`src/core/can_router.c`)
- **Stop Periodic Spam:** Periodic broadcast of 24-byte `0x42` table is disabled to eliminate non-stop floating popup banners on Android HU.
- **Edge-Triggered Real-Time Popups (`Len = 0x02`):** When a fault code transition (0 &rarr; 1) is detected (new alert chime/fault), emits a single 2-byte Hiworld event frame `5A A5 02 42 <HI> <LO> <CHK>`.
- **Full Table Transmission (`Len = 0x18`):** Emitted strictly on:
  1. Response to HU `0x2F` query (once BSI alert journal collected, or upon 500ms timeout fallback).
  2. Once on firmware startup / first CAN scan.
  3. When an alert is added or cleared from the active alert set.
- **BSI Alert Log CAN Uplink:** Sends Comfort CAN diagnostic frame `0x39B` (DLC 8, `{0x01, 0x01, ...}`) to BSI when the HU queries the alert journal.

---

# 6. Desktop Test Harness & Verification Vectors

Automated unit tests in `test/test_protocol_parser/test_peugeot_407.c`:
1. `test_peugeot_407_alert_single_abs`: Verifies CAN `0x1A1` (`0x006C`) -> Hiworld `0x42` (`0x0069`) serialization.
2. `test_peugeot_407_alert_single_suspension`: Verifies CAN `0x1A1` (`0x009E`) -> Hiworld `0x42` (`0x006B`) serialization.
3. `test_peugeot_407_alert_single_low_fuel`: Verifies CAN `0x1A1` (`0x000D`) -> Hiworld `0x42` (`0x0001`) serialization and alert clear on `POPUP_ACTIVE == 0`.
4. `test_peugeot_407_alert_journal_multi`: Verifies 4-frame segmented ISO-TP transfer on `0x120` (Engine Temp, ABS, DPF) -> 24-byte Hiworld frame with `mNumber = 3`.
5. `test_peugeot_407_alert_journal_clear`: Verifies clear frame with `mNumber = 0`.
6. `test_peugeot_407_alert_router_pipeline_and_query`: Verifies end-to-end CAN router pipeline:
   - Initial scan 24-byte table transmission without unsolicited single alert popups.
   - Suppression of duplicate/unchanged journal transmissions (zero spam).
   - Receipt of UART query `0x2F` (`5A A5 01 2F 01 D4` and `0x30` variant) triggering CAN diagnostic query frame `0x39B` (DLC 8).
   - 500ms timeout expiration and state synchronization.
   - Freedom from periodic spam across 600 periodic ticks.
7. `test_peugeot_407_alert_boundary_and_malformed`: Verifies NULL safety, truncated DLC, and corrupted ISO-TP sequence handling.
8. `test_peugeot_407_alert_journal_block_multiplexed_real_log`: Verifies block-multiplexed transport on `0x120` parsed from real vehicle log.
