# PSA CAN 2004 (AEE2004) Vehicle Alerts & Warning System Master Specification (RT4_CAN_ALERTS.md)

> **Ground Truth Architecture Specification**  
> **Source Platform:** Magneti Marelli RT4 / RT5 Telematics System (CD_RT4-5_SW_8.31)  
> **Target Application:** Portable, pure C99 firmware for custom CAN bus adapters interfacing PSA vehicle CAN networks (CAN 2004 Confort @ 125 kbps) with Chinese Android head units.  
> **Decompiled Source Artifacts:**
> - `Fp_Network_CAN.out` (PowerPC ELF): `Alarm_IndexToPointer_Tab` (0x1db50+0x4d78), `Alarm_Index_Active_Tab` (0x1db50+0x4bd8), `Alarm_BitToIndex_Tab` (0x1db50+0x4958), `get_alarm_index__9C_BCM_CANPUc` (0x00E334), `get_alarm_cpl_status__9C_BCM_CANP16alarm_cpl_status` (0x00E878), `get_alarm_main_status__9C_BCM_CANPUc` (0x00E50c).
> - `mmi_alerts.out` (PowerPC ELF): `_8C_Alerts$ms_uchPictogramLookupTable` (0xc038+0x10c), `Update_Alert_Text_Display__14C_ALERTS_F1_Z2` (0x0074c0), `Req_Disp_Alerts__8C_AlertsUl` (0x00301c).
> - `GraphicBase.800x446.out` & `GraphicBase.480x234.out` (PowerPC ELF): `List_GDO_it_1_ALERTS_F1_Z2` (0x70+0x1324c, 206 entries).
> - `GEN_EN.gph` (GPH Database): `TEXT2006` UTF-16BE Canonical String Table (offset 0x18, 3226 entries).
> - **Real-Vehicle Ground Truth:** 87 distinct vehicle alerts verified directly on a physical Peugeot 407 multi-function display over SocketCAN (`can0`) road-test sessions.

---

## 1. Executive Summary & CAN Signal Unpacking

### 1.1 CAN Frame `0x1A1` Signal Breakdown (Alert Notification Frame)

Frame `0x1A1` is broadcast cyclically (default 100 ms) and event-driven by the BSI to command modal popup windows, acoustic chimes, and alert strings on the multi-function display.

```
       Bit 7     Bit 6     Bit 5     Bit 4     Bit 3     Bit 2     Bit 1     Bit 0
Byte 0: TOGGLE  | <---------------------- ALARM_ID[14:8] ------------------------> |
Byte 1: <------------------------------- ALARM_ID[7:0] ---------------------------> |
Byte 2: DISP_REQ| <--- PRIORITY [2:0] ---> | <------------ SOUND_ID [3:0] -------> |
Byte 3: <------------------------- PARAM_DOOR_MASK [7:0] -------------------------> |
Byte 4: <------------------------- PARAM_DETAIL_1 [7:0] --------------------------> |
Byte 5: <------------------------- PARAM_DETAIL_2 [7:0] --------------------------> |
Byte 6: <------------------------- SEQUENCE_COUNTER / RESERVED -------------------> |
Byte 7: <------------------------- CRC / CHECKSUM / RESERVED ---------------------> |
```

#### Detailed Signal Definitions:
1. **Byte 0 Bit 7 (`MSG_TOGGLE`):**
   - **Sequence Toggle Bit (`1, 0, 1, 0...`)**: Inverts on each new successive incoming alert broadcast (specifically observed during sequential BSI diagnostic checks and consecutive fault cycles) to force the display unit to re-trigger the popup alert presentation even if the same alert category repeats.
   - **Critical Firmware Rule:** Decoders must **NOT** interpret `MSG_TOGGLE == 0` as an alert cancellation or dismissal. Alert clearing is governed strictly by Byte 2 Bit 7 (`DISPLAY_REQUEST == 0`) or `ALARM_ID == 0`.
2. **Byte 0 Bits 6..0 & Byte 1 Bits 7..0 (`ALARM_ID`):**
   - 15-bit CAN Alert Identifier.
   - Exact firmware unpack formula (extracted directly from `get_alarm_index__9C_BCM_CANPUc` at `0x00E378`):
     ```c
     uint16_t alarm_id = ((can_frame[0] & 0x7F) << 8) | can_frame[1];
     ```
3. **Byte 2 Bit 7 (`DISPLAY_REQUEST`):**
   - `1` = Active alert modal popup request. The head unit must display the popup dialog and play the corresponding acoustic chime.
   - `0` = Passive update, background status broadcast, or alert dismissed.
4. **Byte 2 Bits 6..4 (`ALARM_PRIORITY`):**
   - `000b` (0) = Informational notification (No warning icon).
   - `001b` (1) = Minor warning (Amber / Caution).
   - `010b` (2) = Major warning (Amber SERVICE icon).
   - `011b` (3) = Critical STOP warning (Red STOP icon + immediate buzzer).
5. **Byte 2 Bits 3..0 (`SOUND_ID`):**
   - Acoustic chime identifier (`0`..`15`) played via vehicle speakers or instrument cluster buzzer.
6. **Byte 3 (`PARAM_DOOR_MASK`):**
   - Door / Opening status bitmask evaluated when `alarm_id == 0x0008`, `0x000B`, or `0x00DE`:
     - Bit 0 (`0x01`): Front Left Door open
     - Bit 1 (`0x02`): Front Right Door open
     - Bit 2 (`0x04`): Rear Left Door open
     - Bit 3 (`0x08`): Rear Right Door open
     - Bit 4 (`0x10`): Boot / Tailgate open
     - Bit 5 (`0x20`): Rear Screen / Tailgate Glass open
     - Bit 6 (`0x40`): Fuel Filler Flap open
     - Bit 7 (`0x80`): Bonnet / Hood open
   - *Note on `0x0008`:* When `PARAM_DOOR_MASK` is `0x00` or `0xFF` (such as during BSI diagnostic test check), `0x0008` represents **"Braking system faulty"**. When individual bits are active, the vehicle renders the graphical vehicle door opening layout.

---

## 2. Real-Vehicle Ground Truth Verification (87 Confirmed Alerts)

During physical SocketCAN testing on a Peugeot 407 (`can0` @ 125 kbps), 87 alerts were directly confirmed and photographed on the vehicle display:

| CAN ID | Dec | Verified Display Text | Subsystem / Function | Severity |
|:---:|:---:|:---|:---|:---:|
| `0x0000` |   0 | **Diagnosis OK.** | BSI Diagnostics | PASS |
| `0x0001` |   1 | **Engine temperature fault: Stop the vehicle.** | Powertrain (STOP) | PASS |
| `0x0003` |   3 | **Top up coolant level.** | Engine Fluids | PASS |
| `0x0004` |   4 | **Top up engine oil level.** | Engine Fluids | PASS |
| `0x0005` |   5 | **Engine oil pressure fault: Stop the vehicle.** | Powertrain (STOP) | PASS |
| `0x0008` |   8 | **Braking system faulty.** | Braking (STOP) | PASS |
| `0x000A` |  10 | **Air suspension OK: vehicle leveled** | Active Suspension (AMVAR / CSS) | OBSERVED |
| `0x000B` |  11 | **Door(s) open.** | BSI Body / Doors | PASS |
| `0x000F` |  15 | **Risk of particle filter clogging: See handbook.** | DPF / Exhaust | PASS |
| `0x0011` |  17 | **Suspension faulty: Max speed 90 km/h** | Active Suspension | OBSERVED |
| `0x0064` | 100 | **Handbrake cable fault auto handbrake activated.** | Electric Parking Brake | PASS |
| `0x0067` | 103 | **Brake pads worn.** | Braking Wear | PASS |
| `0x0068` | 104 | **Handbrake faulty.** | Parking Brake | PASS |
| `0x0069` | 105 | **Mobile deflector faulty.** | Aerodynamics / Spoiler | PASS |
| `0x006A` | 106 | **ABS system faulty.** | Braking / Safety | PASS |
| `0x006B` | 107 | **ESP/ASR system faulty.** | Traction Control | PASS |
| `0x006C` | 108 | **Suspension faulty.** | Active Suspension | PASS |
| `0x006D` | 109 | **Power steering faulty.** | Steering System | PASS |
| `0x006E` | 110 | **Gearbox fault: Repair needed.** | Automatic Gearbox | PASS |
| `0x006F` | 111 | **Speed control system faulty.** | Cruise Control | PASS |
| `0x0073` | 115 | **Ambient brightness sensor faulty.** | Lighting Sensors | PASS |
| `0x0074` | 116 | **Bulbs blown.** | Exterior Lighting | PASS |
| `0x0075` | 117 | **Automatic headlamp adjustment faulty.** | Xenon / Lighting | PASS |
| `0x0076` | 118 | **Directional headlamps faulty** | Lighting ECU | OBSERVED |
| `0x0079` | 121 | **Active bonnet faulty** | Safety / Pyrotechnics | OBSERVED |
| `0x007A` | 122 | **Automatic gearbox faulty** | Transmission (BVA / BVMP) | OBSERVED |
| `0x007D` | 125 | **Presence of water in diesel filter: Repair needed.** | Fuel System | PASS |
| `0x007E` | 126 | **Engine management system faulty** | Engine Management | OBSERVED |
| `0x007F` | 127 | **Depollution system faulty** | Engine Management (EDC16 / ME7) | OBSERVED |
| `0x0081` | 129 | **Particle filter additive level too low: Repair needed.** | Eolys / DPF | PASS |
| `0x0083` | 131 | **Immobiliser faulty.** | Security | PASS |
| `0x0086` | 134 | **Right-hand sliding side door faulty** | Body / Sliding Doors | OBSERVED |
| `0x0087` | 135 | **Left-hand sliding side door faulty** | Body / Sliding Doors | OBSERVED |
| `0x0089` | 137 | **Space measuring system faulty.** | Parking Space Measurement | PASS |
| `0x008A` | 138 | **Battery charge faulty.** | Electrical / Alternator | PASS |
| `0x0097` | 151 | **Anti-wander lane-crossing warning device faulty** | AFIL Lane Assist | OBSERVED |
| `0x009A` | 154 | **Dipped headlamp bulb faulty** | Exterior Lighting | OBSERVED |
| `0x009B` | 155 | **Main beam headlamp bulb faulty** | Exterior Lighting | OBSERVED |
| `0x009C` | 156 | **Left hand brake light bulb faulty** | Exterior Lighting | OBSERVED |
| `0x009D` | 157 | **Foglamp bulb faulty** | Exterior Lighting | OBSERVED |
| `0x009E` | 158 | **Direction indicators faulty** | Exterior Lighting | OBSERVED |
| `0x009F` | 159 | **Left hand reversing light bulb faulty** | Exterior Lighting | OBSERVED |
| `0x00A0` | 160 | **Sidelamp bulb faulty** | Exterior Lighting | OBSERVED |
| `0x00CD` | 205 | **No cruise control: speed low.** | Speed Limiter / Cruise | PASS |
| `0x00CE` | 206 | **Cruise control activation impossible: enter speed.** | Cruise Control | PASS |
| `0x00D1` | 209 | **Active bonnet deployed.** | Pedestrian Protection | PASS |
| `0x00D3` | 211 | **None of rear passenger seat belts fastened** | Safety / Seatbelt Monitor | OBSERVED |
| `0x00D7` | 215 | **Put automatic gearbox in "P" position.** | Gearbox Interlock | PASS |
| `0x00D8` | 216 | **Risk of black ice.** | Weather / Safety | PASS |
| `0x00D9` | 217 | **Handbrake on !** | Parking Brake | PASS |
| `0x00DE` | 222 | **Front left hand door open** | BSI Body / Doors | OBSERVED |
| `0x00DF` | 223 | **Screen washer fluid level low.** | Driver Convenience | PASS |
| `0x00E0` | 224 | **Fuel level low.** | Fuel Reserve | PASS |
| `0x00E1` | 225 | **Fuel circuit deactivated.** | Crash Cutoff | PASS |
| `0x00E3` | 227 | **Remote control battery spent.** | Key Battery | PASS |
| `0x00E4` | 228 | **Check and re-initialise tyre pressure.** | TPMS System | PASS |
| `0x00E8` | 232 | **Tyre pressure too low** | Under-Inflation (TPMS) | OBSERVED |
| `0x00EA` | 234 | **Hands free starting system faulty.** | Keyless Access | PASS |
| `0x00EB` | 235 | **Starting phase has failed (consult handbook).** | Powertrain / Starter | PASS |
| `0x00EC` | 236 | **Prolonged starting in progress.** | Powertrain / Starter | PASS |
| `0x00ED` | 237 | **Starting impossible : unlock steering.** | Steering Lock | PASS |
| `0x00EF` | 239 | **Remote control not detected.** | Keyless Access | PASS |
| `0x00F0` | 240 | **Diagnosis in progress...** | Cockpit CHECK / BSI | OBSERVED |
| `0x00F1` | 241 | **Diagnosis completed** | Cockpit CHECK / BSI | OBSERVED |
| `0x00F7` | 247 | **Rear LH seat belt not fastened** | Safety / Seatbelt Monitor | OBSERVED |
| `0x00F8` | 248 | **Rear center seat belt not fastened** | Safety / Seatbelt Monitor | OBSERVED |
| `0x00F9` | 249 | **Rear RH seat belt not fastened** | Safety / Seatbelt Monitor | OBSERVED |
| `0x012F` | 303 | **Automatic windscreen wiper activated.** | Rain Sensor | PASS |
| `0x0130` | 304 | **Automatic windscreen wiper deactivated.** | Rain Sensor | PASS |
| `0x0131` | 305 | **Automatic headlamp lighting activated** | Automatic Lighting | OBSERVED |
| `0x0132` | 306 | **Automatic headlights deactivated.** | Automatic Lighting | PASS |
| `0x0133` | 307 | **Self locking doors activated.** | Central Locking | PASS |
| `0x0134` | 308 | **Self locking doors deactivated.** | Central Locking | PASS |
| `0x0137` | 311 | **Child safety activated.** | Child Lock | PASS |
| `0x0138` | 312 | **Child safety deactivated.** | Child Lock | PASS |
| `0x013D` | 317 | **Parking difficult** | Space Measuring System | OBSERVED |
| `0x0198` | 408 | **Stop & Start system faulty** | Powertrain / Stop & Start | OBSERVED |
| `0x01F6` | 502 | **Roof operation impossible: external temperature low.** | Coupe-Cabriolet Roof | PASS |
| `0x01F7` | 503 | **Operation of roof impossible : speed too high.** | Coupe-Cabriolet Roof | PASS |
| `0x01F8` | 504 | **Operation of roof impossible : boot open.** | Coupe-Cabriolet Roof | PASS |
| `0x01FA` | 506 | **Impossible to move roof: screen not deployed.** | Coupe-Cabriolet Roof | PASS |
| `0x01FB` | 507 | **Roof operation complete.** | Coupe-Cabriolet Roof | PASS |
| `0x01FC` | 508 | **Roof operation incomplete.** | Coupe-Cabriolet Roof | PASS |
| `0x01FD` | 509 | **Roof movement impossible: roof locked** | Retractable Hardtop (Coupe-Cabriolet) | OBSERVED |
| `0x01FE` | 510 | **Folding roof mechanism faulty.** | Coupe-Cabriolet Roof | PASS |
| `0x01FF` | 511 | **Roof operation impossible: rear screen open** | Retractable Hardtop | OBSERVED |
| `0x0200` | 512 | **Roof movement impossible: luggage cover not locked** | Retractable Hardtop | OBSERVED |

---

## 3. Reverse-Engineered String Resolution Architecture

Disassembly of Magneti Marelli RT4/RT5 software (SW 8.31) reveals the complete string resolution pipeline from raw CAN frame `0x1A1` to rendered text:

### 3.1 Proven Firmware Pipeline
1. `get_alarm_index__9C_BCM_CAN` (@ `Fp_Network_CAN:0xE334`):
   - Extracts `alarm_id = ((b0 & 0x7F) << 8) | b1`.
   - Performs a linear search across `Alarm_IndexToPointer_Tab[0..207]`.
   - Match is validated if `Alarm_Index_Active_Tab[idx] == 1`; returns `idx` (`0xFF` on miss).
2. `C_Alerts::UpdateAlarmMainStatus` (@ `mmi_alerts:0x24CC`):
   - Stores `idx` in byte offset `this + 982`.
3. `ALERTS_CNTL_ON` (@ `mmi_alerts:0x153C`):
   - Special screen handlers: `idx` 113..115 (roof F11), 140/1 (doors F5), 172..178/35 (exterior bulbs), 61..63/131/12 (TPMS wheels F9), 138/161/41 (seat belts), 162 (space measurement).
   - All standard alerts route to `HandleOtherBSIAlerts` (screen 111 = `C_ALERTS_F1_Z2`).
   - Pictogram/severity is obtained from `ms_uchPictogramLookupTable[idx]`: 0=None, 1=Service, 2=Stop, 3=Belt.
4. `Update_Alert_Text_Display` (@ `mmi_alerts:0x74C0`):
   - Calls `C_COL_NIO::ChangeIndex(idx)` -> `C_GOL_IDTEXT::ChangeTextIndex(idx)`.
   - Text ID is retrieved from `List_GDO_it_1_ALERTS_F1_Z2[1 + idx]` (word 0 is element count = 206).
   - String is fetched from `GEN_EN.gph[text_id]`.

### 3.2 Resolution of Stale GraphicBase Lists (The `+594` and `+710` Offsets)
In the RT4 binaries, `List_GDO_it_1_ALERTS_F1_Z2` for `idx >= 80` was not updated and points to menu and phone brand strings. However, secondary alert blocks in `GEN_EN` line up 1:1 with the CAN alarm indices:
- **`idx 0..79`:** Maps 1:1 to `List_GDO_it_1_ALERTS_F1_Z2[1 + idx]` (texts 475..554).
- **`idx 80..167`:** Maps 1:1 to `GEN_EN[idx + 594]` (texts 674..761). Confirmed on vehicle: `0x198` Stop & Start faulty, `0x64` handbrake cable, `0x68` handbrake, `0x89` space measuring, `0xCD`/`0xCE` cruise control, `0xD1` active bonnet, `0x1F6`..`0x1FE` CC roof.
- **`idx 170..205`:** Maps 1:1 to `GEN_EN[idx + 710]` (texts 880..915). Confirmed on vehicle: `0x97` anti-wander, `0xE8` tyre pressure low, `0xF0`/`0xF1` diagnosis, `0xF7`..`0xF9` rear seat belts, `0x1FF`/`0x200` CC roof, `0x11` suspension 90 km/h.
- **Blown Bulb Block (`idx 172..178`, CAN IDs `0x009A`..`0x00A0`):** When sent with `b3=0x40`, these trigger the individual exterior blown bulb messages (`0x9A` dipped headlamp, `0x9B` main beam, `0x9C` left brake, `0x9D` foglamp, `0x9E` direction indicators, `0x9F` left reversing, `0xA0` sidelamp).

---

## 4. Secondary Warning & Telemetry Frames

In addition to `0x1A1`, vehicle safety systems communicate over dedicated CAN frames:

### 4.1 Frame `0x120` — Historical Alert Journal (`MSG_JOURNAL_ALERTES`)
- **Transport:** Segmented ISO-TP multi-frame session (21 bytes total payload, 168 bits).
- **Evaluation:** Evaluated by `get_alarm_main_status` (0x00E50C). Each bit `k` in `[0..167]` maps to an internal alarm index via `Alarm_BitToIndex_Tab[k]`.

### 4.2 Frame `0x269` — Crash & Impact Notification (`MSG_ETAT_INFO_CRASH`)
- **Periodicity:** Event-driven (0 ms latency).
- **Signals:** Impact severity, pyrotechnic pretensioner firing, inertia fuel cut-off request.

### 4.3 Frame `0x1E1` — Dynamic Tire Pressures (`MSG_DONNEES_ETAT_ROUES`)
- **Periodicity:** 250 ms.
- **Signals:** 4-wheel status (OK, Under-inflated, Puncture STOP, Sensor missing).

### 4.4 Frame `0x161` — Engine Fluid Levels & Temperatures (`MSG_ETAT_BSI_TEMP_NIVEAU`)
- **Periodicity:** 250 ms.
- **Signals:** Coolant temperature, oil pressure STOP, oil level, coolant level.

---

## 5. Master CAN Alert Lookup Table (Full 208-Entry Hardware Specification)

The table below catalogs all 208 alert entries indexed in `Alarm_IndexToPointer_Tab` (`0x00` through `0xCF`), fully cross-referenced with decompiled binaries and physical vehicle ground truth:

| Alarm Index | CAN Alarm ID (Hex / Dec) | Active State | Severity & Pictogram | 0x120 Journal Bit | Canonical English Alert String | Category & Functional Description |
|---|---|---|---|---|---|---|
| `0x00` (  0) | `0x000C` ( 12) | Inactive | ℹ️ INFO (0) | 54, 55, 56, 57 | **Tyre pressure low.** | Tyre Under-Inflation Detection System (TPMS) alert |
| `0x01` (  1) | `0x000B` ( 11) | Active | ⚠️ SERVICE (1) | 10, 11, 12, 13, 14, 15, 16, 125 | **Door(s) open.** | Vehicle body openings & passenger access monitoring |
| `0x02` (  2) | `0x019C` (412) | Active | ℹ️ INFO (0) | - | **Press the park button and the brake pedal.** | Braking system operational warning or hydraulic/wear threshold alert |
| `0x03` (  3) | `0x0008` (  8) | Active | 🛑 STOP (2) | 3 | **Braking system faulty.** | Braking system malfunction / critical brake failure (displayed when door_mask=0x00/0xFF during BSI diagnostic check; if individual door bits set in door_mask, door popup is rendered) |
| `0x04` (  4) | `0x019B` (411) | Active | ℹ️ INFO (0) | - | **To release the parking brake, press the brake pedal.** | Braking system operational warning or hydraulic/wear threshold alert |
| `0x05` (  5) | `0x019A` (410) | Active | ℹ️ INFO (0) | - | **Place gearbox in position "N".** | Transmission / automatic gearbox selector & interlock notice |
| `0x06` (  6) | `0x0002` (  2) | Active | ⚠️ SERVICE (1) | - | **Engine oil temperature too high.** | Powertrain / engine management / mechanical fluid level alert |
| `0x07` (  7) | `0x0001` (  1) | Active | 🛑 STOP (2) | 1 | **Engine temperature fault: Stop the vehicle.** | Powertrain / engine management / mechanical fluid level alert |
| `0x08` (  8) | `0x000F` ( 15) | Active | ⚠️ SERVICE (1) | 27 | **Risk of particle filter clogging: See handbook.** | Powertrain / engine management / mechanical fluid level alert |
| `0x09` (  9) | `0x000A` ( 10) | Active | ⚠️ SERVICE (1) | - | **Air suspension OK: vehicle leveled** | Active suspension status and height leveling confirmation |
| `0x0A` ( 10) | `0x000E` ( 14) | Inactive | ⚠️ SERVICE (1) | - | **FAP additive level low.** | Powertrain / engine management / mechanical fluid level alert |
| `0x0B` ( 11) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Vehicle body openings & passenger access monitoring |
| `0x0C` ( 12) | `0x000D` ( 13) | Active | 🛑 STOP (2) | 59, 60, 61, 62 | **Puncture(s) in tyres detected.** | Tyre Under-Inflation Detection System (TPMS) alert |
| `0x0D` ( 13) | `0x0003` (  3) | Active | ⚠️ SERVICE (1) | 6 | **Top up coolant level.** | Powertrain / engine management / mechanical fluid level alert |
| `0x0E` ( 14) | `0x0005` (  5) | Active | 🛑 STOP (2) | 0 | **Engine oil pressure fault: Stop the vehicle.** | Powertrain / engine management / mechanical fluid level alert |
| `0x0F` ( 15) | `0x0004` (  4) | Active | ⚠️ SERVICE (1) | 8 | **Top up engine oil level.** | Powertrain / engine management / mechanical fluid level alert |
| `0x10` ( 16) | `0x007F` (127) | Active | ⚠️ SERVICE (1) | 25 | **Depollution system faulty** | Engine emissions / depollution anti-pollution system malfunction (exhaust gas treatment / catalytic converter / DPF fault) |
| `0x11` ( 17) | `0x0067` (103) | Active | ⚠️ SERVICE (1) | 21 | **Brake pads worn.** | Braking system operational warning or hydraulic/wear threshold alert |
| `0x12` ( 18) | `0x0000` (  0) | Active | ℹ️ INFO (0) | - | **Diagnosis OK.** | On-board diagnostic system test execution and status |
| `0x13` ( 19) | `0x006E` (110) | Active | ⚠️ SERVICE (1) | 30 | **Gearbox fault: Repair needed.** | Transmission / automatic gearbox selector & interlock notice |
| `0x14` ( 20) | `0x006B` (107) | Active | ⚠️ SERVICE (1) | 17 | **ESP/ASR system faulty.** | Vehicle status notification / configuration change confirmation |
| `0x15` ( 21) | `0x006A` (106) | Active | ⚠️ SERVICE (1) | 26 | **ABS system faulty.** | Vehicle status notification / configuration change confirmation |
| `0x16` ( 22) | `0x006C` (108) | Active | ⚠️ SERVICE (1) | 31 | **Suspension faulty.** | Electronic / active suspension damping status or fault |
| `0x17` ( 23) | `0x0066` (102) | Inactive | 🛑 STOP (2) | - | **Braking system faulty.** | Braking system operational warning or hydraulic/wear threshold alert |
| `0x18` ( 24) | `0xFFFF` (-) | Inactive | ⚠️ SERVICE (1) | - | ***(Unmapped / Inactive)*** | Passive passenger safety / restraint system status |
| `0x19` ( 25) | `0xFFFF` (-) | Inactive | ⚠️ SERVICE (1) | - | ***(Unmapped / Inactive)*** | Passive passenger safety / restraint system status |
| `0x1A` ( 26) | `0x006F` (111) | Active | ⚠️ SERVICE (1) | 33 | **Speed control system faulty.** | Speed limiter / cruise control / overspeed warning |
| `0x1B` ( 27) | `0x007E` (126) | Active | ⚠️ SERVICE (1) | 141 | **Engine management system faulty** | Engine management / depollution system malfunction |
| `0x1C` ( 28) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x1D` ( 29) | `0x0073` (115) | Active | ⚠️ SERVICE (1) | 50 | **Ambient brightness sensor faulty.** | Vehicle status notification / configuration change confirmation |
| `0x1E` ( 30) | `0x0072` (114) | Inactive | ⚠️ SERVICE (1) | - | **Rain sensor faulty.** | Vehicle status notification / configuration change confirmation |
| `0x1F` ( 31) | `0x007D` (125) | Active | ⚠️ SERVICE (1) | 20 | **Presence of water in diesel filter: Repair needed.** | Vehicle status notification / configuration change confirmation |
| `0x20` ( 32) | `0x0087` (135) | Active | ⚠️ SERVICE (1) | 144 | **Left-hand sliding side door faulty** | Vehicle body openings & passenger access monitoring |
| `0x21` ( 33) | `0x0075` (117) | Active | ⚠️ SERVICE (1) | 43 | **Automatic headlamp adjustment faulty.** | Vehicle exterior lighting and bulb failure monitoring |
| `0x22` ( 34) | `0x0086` (134) | Active | ⚠️ SERVICE (1) | 143 | **Right-hand sliding side door faulty** | Vehicle body openings & passenger access monitoring |
| `0x23` ( 35) | `0x0074` (116) | Active | ⚠️ SERVICE (1) | - | **Bulbs blown.** | Vehicle exterior lighting and bulb failure monitoring |
| `0x24` ( 36) | `0x0085` (133) | Inactive | ⚠️ SERVICE (1) | - | **Battery low.** | Electrical power generation & charging circuit status |
| `0x25` ( 37) | `0x0084` (132) | Inactive | ⚠️ SERVICE (1) | - | **Battery charge faulty.** | Electrical power generation & charging circuit status |
| `0x26` ( 38) | `0x0081` (129) | Active | ⚠️ SERVICE (1) | 29 | **Particle filter additive level too low: Repair needed.** | Powertrain / engine management / mechanical fluid level alert |
| `0x27` ( 39) | `0x0080` (128) | Inactive | ⚠️ SERVICE (1) | 24 | **Depollution system faulty.** | Vehicle status notification / configuration change confirmation |
| `0x28` ( 40) | `0x00D9` (217) | Active | ℹ️ INFO (0) | 9 | **Handbrake on !** | Braking system operational warning or hydraulic/wear threshold alert |
| `0x29` ( 41) | `0x00D2` (210) | Active | 💺 BELT (3) | 101, 102 | **Seat belt(s) not fastened.** | Passive passenger safety / restraint system status |
| `0x2A` ( 42) | `0x00D0` (208) | Inactive | ℹ️ INFO (0) | - | **Passenger airbag switched off.** | Passive passenger safety / restraint system status |
| `0x2B` ( 43) | `0x00DF` (223) | Active | ℹ️ INFO (0) | 46 | **Screen washer fluid level low.** | Visibility / wash-wipe equipment status |
| `0x2C` ( 44) | `0x00CB` (203) | Inactive | ℹ️ INFO (0) | - | **Current speed too high.** | Speed limiter / cruise control / overspeed warning |
| `0x2D` ( 45) | `0x00CA` (202) | Inactive | ⚠️ SERVICE (1) | - | **Ignition key left in.** | Vehicle status notification / configuration change confirmation |
| `0x2E` ( 46) | `0x00C9` (201) | Inactive | ℹ️ INFO (0) | - | **Sidelights left on.** | Vehicle exterior lighting and bulb failure monitoring |
| `0x2F` ( 47) | `0xFFFF` (-) | Inactive | 💺 BELT (3) | - | ***(Unmapped / Inactive)*** | Passive passenger safety / restraint system status |
| `0x30` ( 48) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x31` ( 49) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x32` ( 50) | `0x00E4` (228) | Active | ℹ️ INFO (0) | 138 | **Check and re-initialise tyre pressure.** | Tyre Under-Inflation Detection System (TPMS) alert |
| `0x33` ( 51) | `0x00E3` (227) | Active | ℹ️ INFO (0) | 47 | **Remote control battery spent.** | Electrical power generation & charging circuit status |
| `0x34` ( 52) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x35` ( 53) | `0x00D7` (215) | Active | ℹ️ INFO (0) | 106 | **Put automatic gearbox in "P" position.** | Transmission / automatic gearbox selector & interlock notice |
| `0x36` ( 54) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Braking system operational warning or hydraulic/wear threshold alert |
| `0x37` ( 55) | `0x00E0` (224) | Active | ℹ️ INFO (0) | 22 | **Fuel level low.** | Vehicle status notification / configuration change confirmation |
| `0x38` ( 56) | `0xFFFF` (-) | Inactive | ⚠️ SERVICE (1) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x39` ( 57) | `0xFFFF` (-) | Inactive | 💺 BELT (3) | 126 | ***(Unmapped / Inactive)*** | Passive passenger safety / restraint system status |
| `0x3A` ( 58) | `0xFFFF` (-) | Inactive | 💺 BELT (3) | - | ***(Unmapped / Inactive)*** | Passive passenger safety / restraint system status |
| `0x3B` ( 59) | `0xFFFF` (-) | Inactive | 💺 BELT (3) | - | ***(Unmapped / Inactive)*** | Passive passenger safety / restraint system status |
| `0x3C` ( 60) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x3D` ( 61) | `0x00E5` (229) | Active | ⚠️ SERVICE (1) | 52, 148, 149, 150, 151 | **Tyre pressure(s) not monitored.** | Under-inflation detection system inactive or wheel sensors missing / not communicating |
| `0x3E` ( 62) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x3F` ( 63) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x40` ( 64) | `0x0135` (309) | Inactive | ℹ️ INFO (0) | - | **Doors locked.** | Vehicle body openings & passenger access monitoring |
| `0x41` ( 65) | `0x013A` (314) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / System)*** | Vehicle status notification / configuration change confirmation |
| `0x42` ( 66) | `0x0137` (311) | Active | ℹ️ INFO (0) | - | **Child safety activated.** | Vehicle status notification / configuration change confirmation |
| `0x43` ( 67) | `0x0133` (307) | Active | ℹ️ INFO (0) | - | **Self locking doors activated.** | Vehicle body openings & passenger access monitoring |
| `0x44` ( 68) | `0x0131` (305) | Active | ℹ️ INFO (0) | - | **Automatic headlamp lighting activated** | Automatic dusk/darkness headlamp lighting feature enabled |
| `0x45` ( 69) | `0x012F` (303) | Active | ℹ️ INFO (0) | - | **Automatic windscreen wiper activated.** | Visibility / wash-wipe equipment status |
| `0x46` ( 70) | `0x0083` (131) | Active | ⚠️ SERVICE (1) | 37 | **Immobiliser faulty.** | Transponder key immobiliser / electronic anti-theft system communication error |
| `0x47` ( 71) | `0x012E` (302) | Inactive | ℹ️ INFO (0) | - | **"Sport" suspension mode activated.** | Electronic / active suspension damping status or fault |
| `0x48` ( 72) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x49` ( 73) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x4A` ( 74) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x4B` ( 75) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x4C` ( 76) | `0x00E2` (226) | Inactive | ⚠️ SERVICE (1) | - | **LPG fuel refused.** | Vehicle status notification / configuration change confirmation |
| `0x4D` ( 77) | `0x0082` (130) | Inactive | ⚠️ SERVICE (1) | - | **LPG system faulty.** | Vehicle status notification / configuration change confirmation |
| `0x4E` ( 78) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x4F` ( 79) | `0xFFFF` (-) | Inactive | ⚠️ SERVICE (1) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x50` ( 80) | `0x0198` (408) | Active | ⚠️ SERVICE (1) | - | **Stop & Start system faulty** | Powertrain Stop & Start / ECO system fault |
| `0x51` ( 81) | `0x0195` (405) | Inactive | ℹ️ INFO (0) | - | **Use Stop & Start.** | Vehicle exterior lighting and bulb failure monitoring |
| `0x52` ( 82) | `0x0194` (404) | Inactive | ℹ️ INFO (0) | - | **Stop & Start available.** | Vehicle exterior lighting and bulb failure monitoring |
| `0x53` ( 83) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x54` ( 84) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Vehicle exterior lighting and bulb failure monitoring |
| `0x55` ( 85) | `0x0193` (403) | Inactive | ℹ️ INFO (0) | - | **Stop & Start deferred.** | Vehicle exterior lighting and bulb failure monitoring |
| `0x56` ( 86) | `0x0192` (402) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / System)*** | Vehicle exterior lighting and bulb failure monitoring |
| `0x57` ( 87) | `0x0191` (401) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / System)*** | Vehicle exterior lighting and bulb failure monitoring |
| `0x58` ( 88) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x59` ( 89) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x5A` ( 90) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x5B` ( 91) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x5C` ( 92) | `0x0202` (514) | Active | ⚠️ SERVICE (1) | 154 | **Anti-rollback system faulty.** | Vehicle exterior lighting and bulb failure monitoring |
| `0x5D` ( 93) | `0x020B` (523) | Active | ⚠️ SERVICE (1) | 156 | **Inter-vehicles time measuring impossible: initialisation in progress.** | Vehicle exterior lighting and bulb failure monitoring |
| `0x5E` ( 94) | `0x020A` (522) | Active | ℹ️ INFO (0) | - | **Inter-vehicles time measuring impossible: poor visibility.** | Vehicle status notification / configuration change confirmation |
| `0x5F` ( 95) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Braking system operational warning or hydraulic/wear threshold alert |
| `0x60` ( 96) | `0x0209` (521) | Active | ℹ️ INFO (0) | - | **ESP system deactivated.** | Vehicle exterior lighting and bulb failure monitoring |
| `0x61` ( 97) | `0x0206` (518) | Active | ℹ️ INFO (0) | - | **Enhanced traction control: Gravel mode.** | Vehicle exterior lighting and bulb failure monitoring |
| `0x62` ( 98) | `0x0205` (517) | Active | ℹ️ INFO (0) | - | **Enhanced traction control: Snow mode.** | Vehicle status notification / configuration change confirmation |
| `0x63` ( 99) | `0x0204` (516) | Active | ℹ️ INFO (0) | - | **Enhanced traction control: Mud mode** | Vehicle status notification / configuration change confirmation |
| `0x64` (100) | `0x0203` (515) | Active | ℹ️ INFO (0) | - | **Enhanced traction control : Normal mode.** | Vehicle status notification / configuration change confirmation |
| `0x65` (101) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Vehicle exterior lighting and bulb failure monitoring |
| `0x66` (102) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Vehicle exterior lighting and bulb failure monitoring |
| `0x67` (103) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Vehicle exterior lighting and bulb failure monitoring |
| `0x68` (104) | `0x01FB` (507) | Active | ℹ️ INFO (0) | - | **Roof operation complete.** | Retractable Hardtop (Coupe-Cabriolet) |
| `0x69` (105) | `0x01FA` (506) | Active | ℹ️ INFO (0) | - | **Impossible to move roof: screen not deployed.** | Retractable Hardtop (Coupe-Cabriolet) |
| `0x6A` (106) | `0x01F9` (505) | Inactive | ⚠️ SERVICE (1) | - | **Roof mechanism not locked.** | Telematic emergency call & road assistance service status |
| `0x6B` (107) | `0x01F8` (504) | Active | ℹ️ INFO (0) | - | **Operation of roof impossible : boot open.** | Retractable Hardtop (Coupe-Cabriolet) |
| `0x6C` (108) | `0x01F7` (503) | Active | ℹ️ INFO (0) | - | **Operation of roof impossible : speed too high.** | Retractable Hardtop (Coupe-Cabriolet) |
| `0x6D` (109) | `0x01F6` (502) | Active | ℹ️ INFO (0) | - | **Roof operation impossible: external temperature low.** | Retractable Hardtop (Coupe-Cabriolet) |
| `0x6E` (110) | `0x01FE` (510) | Active | ⚠️ SERVICE (1) | - | **Folding roof mechanism faulty.** | Retractable Hardtop (Coupe-Cabriolet) |
| `0x6F` (111) | `0x01F5` (501) | Inactive | ⚠️ SERVICE (1) | - | **Boot mechanism not locked.** | Retractable Hardtop (Coupe-Cabriolet) |
| `0x70` (112) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x71` (113) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x72` (114) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Vehicle body openings & passenger access monitoring |
| `0x73` (115) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Vehicle body openings & passenger access monitoring |
| `0x74` (116) | `0x007C` (124) | Inactive | ⚠️ SERVICE (1) | - | **Roof frame fault.** | Tyre Under-Inflation Detection System (TPMS) alert |
| `0x75` (117) | `0x01FD` (509) | Active | ℹ️ INFO (0) | - | **Roof movement impossible: roof locked** | Retractable Hardtop (Coupe-Cabriolet) |
| `0x76` (118) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Tyre Under-Inflation Detection System (TPMS) alert |
| `0x77` (119) | `0x01FC` (508) | Active | ℹ️ INFO (0) | - | **Roof operation incomplete.** | Retractable Hardtop (Coupe-Cabriolet) |
| `0x78` (120) | `0x0012` ( 18) | Active | ⚠️ SERVICE (1) | 152 | **Suspension faulty.** | Electronic / active suspension damping status or fault |
| `0x79` (121) | `0x0068` (104) | Active | ⚠️ SERVICE (1) | 123 | **Handbrake faulty.** | Parking brake mechanism or electric handbrake actuator fault |
| `0x7A` (122) | `0x0069` (105) | Active | ⚠️ SERVICE (1) | 124 | **Mobile deflector faulty.** | Active aerodynamic spoiler / deflector mechanism fault |
| `0x7B` (123) | `0x006D` (109) | Active | 🛑 STOP (2) | 5 | **Power steering faulty.** | Hydraulic or electric power steering assistance critical failure |
| `0x7C` (124) | `0x0013` ( 19) | Active | ⚠️ SERVICE (1) | 153 | **Power steering faulty.** | Hydraulic or electric power steering assistance service warning |
| `0x7D` (125) | `0x0078` (120) | Active | ⚠️ SERVICE (1) | 23 | **Airbag(s) or pretensioner seat belt(s) faulty.** | Safety / Airbag and pyrotechnic pretensioner system fault |
| `0x7E` (126) | `0x0088` (136) | Active | ⚠️ SERVICE (1) | 91 | **Parking assistance system faulty.** | Parking assistance / AAS ultrasonic sensor system fault |
| `0x7F` (127) | `0x0089` (137) | Active | ⚠️ SERVICE (1) | 92 | **Space measuring system faulty.** | Parking space measurement / lateral slot monitoring fault |
| `0x80` (128) | `0x008A` (138) | Active | ⚠️ SERVICE (1) | 18 | **Battery charge faulty.** | Electrical power generation and alternator charging circuit |
| `0x81` (129) | `0x0076` (118) | Active | ⚠️ SERVICE (1) | 34 | **Directional headlamps faulty** | Directional headlamp system fault |
| `0x82` (130) | `0x008C` (140) | Inactive | ℹ️ INFO (0) | 45 | ***(Unmapped / System)*** | Vehicle status notification / configuration change confirmation |
| `0x83` (131) | `0x008D` (141) | Active | ⚠️ SERVICE (1) | 94, 95, 96, 97 | **Tyre pressure low.** | Tyre Under-Inflation Detection System (TPMS) alert |
| `0x84` (132) | `0x0222` (546) | Active | ℹ️ INFO (0) | 51 | **Disengage the clutch fully.** | Powertrain clutch pedal interlock notification |
| `0x85` (133) | `0x0221` (545) | Active | ℹ️ INFO (0) | - | **ECO deactivated.** | Powertrain Stop & Start / ECO mode status |
| `0x86` (134) | `0x0220` (544) | Active | ℹ️ INFO (0) | - | **ECO activated.** | Powertrain Stop & Start / ECO mode status |
| `0x87` (135) | `0x0091` (145) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / System)*** | Vehicle exterior lighting and bulb failure monitoring |
| `0x88` (136) | `0x00CD` (205) | Active | ℹ️ INFO (0) | - | **No cruise control: speed low.** | Cruise control and speed limiter operational inhibit |
| `0x89` (137) | `0x00D1` (209) | Active | ℹ️ INFO (0) | 100 | **Active bonnet deployed.** | Pyrotechnic pedestrian active bonnet safety deployment |
| `0x8A` (138) | `0x00D4` (212) | Inactive | 💺 BELT (3) | - | ***(Unmapped / System)*** | Vehicle status notification / configuration change confirmation |
| `0x8B` (139) | `0x00D8` (216) | Active | ℹ️ INFO (0) | 107 | **Risk of black ice.** | Outside temperature and road surface ice hazard notification |
| `0x8C` (140) | `0x00DE` (222) | Active | ⚠️ SERVICE (1) | 108, 109, 110, 111, 112, 113, 114, 115 | **Front left hand door open** | Vehicle body openings & passenger access monitoring |
| `0x8D` (141) | `0x00E1` (225) | Active | ℹ️ INFO (0) | - | **Fuel circuit deactivated.** | Vehicle status notification / configuration change confirmation |
| `0x8E` (142) | `0x00E6` (230) | Inactive | ℹ️ INFO (0) | 117 | ***(Unmapped / System)*** | Vehicle status notification / configuration change confirmation |
| `0x8F` (143) | `0x00EC` (236) | Active | ℹ️ INFO (0) | 130 | **Prolonged starting in progress.** | Vehicle status notification / configuration change confirmation |
| `0x90` (144) | `0x00ED` (237) | Active | ℹ️ INFO (0) | - | **Starting impossible : unlock steering.** | Vehicle status notification / configuration change confirmation |
| `0x91` (145) | `0x00EE` (238) | Inactive | ℹ️ INFO (0) | 119 | ***(Unmapped / System)*** | Electronic / active suspension damping status or fault |
| `0x92` (146) | `0x00EF` (239) | Active | ℹ️ INFO (0) | 131 | **Remote control not detected.** | Braking system operational warning or hydraulic/wear threshold alert |
| `0x93` (147) | `0x012D` (301) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / System)*** | Vehicle exterior lighting and bulb failure monitoring |
| `0x94` (148) | `0x0130` (304) | Active | ℹ️ INFO (0) | - | **Automatic windscreen wiper deactivated.** | Visibility / Rain Sensor |
| `0x95` (149) | `0x0132` (306) | Active | ℹ️ INFO (0) | - | **Automatic headlights deactivated.** | Exterior Lighting / Automatic Headlamps |
| `0x96` (150) | `0x0136` (310) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / System)*** | Vehicle status notification / configuration change confirmation |
| `0x97` (151) | `0x0138` (312) | Active | ℹ️ INFO (0) | - | **Child safety deactivated.** | Safety / Child Lock |
| `0x98` (152) | `0x0139` (313) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / System)*** | Automatic rain-sensing windscreen wiper feature disabled |
| `0x99` (153) | `0x013B` (315) | Inactive | ℹ️ INFO (0) | 120 | ***(Unmapped / System)*** | Vehicle status notification / configuration change confirmation |
| `0x9A` (154) | `0x013C` (316) | Inactive | ℹ️ INFO (0) | 121 | ***(Unmapped / System)*** | Vehicle status notification / configuration change confirmation |
| `0x9B` (155) | `0x00DA` (218) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / System)*** | Passive passenger safety / restraint system status |
| `0x9C` (156) | `0x0079` (121) | Active | ⚠️ SERVICE (1) | 90 | **Active bonnet faulty** | Safety / Pyrotechnics |
| `0x9D` (157) | `0x00E7` (231) | Active | ⚠️ SERVICE (1) | 118 | **High speed, check tyre pressures are suitable.** | Passive passenger safety / restraint system status |
| `0x9E` (158) | `0x00E9` (233) | Inactive | ℹ️ INFO (0) | 127 | ***(Unmapped / System)*** | Vehicle body openings & passenger access monitoring |
| `0x9F` (159) | `0x00EA` (234) | Active | ⚠️ SERVICE (1) | 128 | **Hands free starting system faulty.** | Retractable hard-top / cabriolet roof mechanism state |
| `0xA0` (160) | `0x00EB` (235) | Active | ⚠️ SERVICE (1) | 129 | **Starting phase has failed (consult handbook).** | Retractable hard-top / cabriolet roof mechanism state |
| `0xA1` (161) | `0x00D3` (211) | Active | 💺 BELT (3) | 103, 104, 105 | **None of rear passenger seat belts fastened** | Safety / Seatbelt Monitor |
| `0xA2` (162) | `0x013D` (317) | Active | ℹ️ INFO (0) | - | **Parking difficult** | Parking Space Measurement (Lateral Slot) |
| `0xA3` (163) | `0x013E` (318) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / System)*** | Braking system operational warning or hydraulic/wear threshold alert |
| `0xA4` (164) | `0x00CE` (206) | Active | ℹ️ INFO (0) | - | **Cruise control activation impossible: enter speed.** | Speed Limiter / Cruise Control |
| `0xA5` (165) | `0x0134` (308) | Active | ℹ️ INFO (0) | - | **Self locking doors deactivated.** | Body / Central Locking |
| `0xA6` (166) | `0x0064` (100) | Active | ⚠️ SERVICE (1) | 140 | **Handbrake cable fault auto handbrake activated.** | Electric Parking Brake |
| `0xA7` (167) | `0x007A` (122) | Active | ⚠️ SERVICE (1) | 133 | **Automatic gearbox faulty** | Transmission (Automatic Gearbox) |
| `0xA8` (168) | `0x0092` (146) | Active | ⚠️ SERVICE (1) | 134 | **Overtaking assistance system faulty.** | Telematic emergency call & road assistance service status |
| `0xA9` (169) | `0x0095` (149) | Inactive | ℹ️ INFO (0) | - | **Lane monitoring system deactivated.** | Vehicle status notification / configuration change confirmation |
| `0xAA` (170) | `0x0096` (150) | Active | ℹ️ INFO (0) | 136 | **Reminder : Lane monitoring inactive.** | Telematic emergency call & road assistance service status |
| `0xAB` (171) | `0x0097` (151) | Active | ⚠️ SERVICE (1) | 137 | **Anti-wander lane-crossing warning device faulty** | AFIL Lane Departure Assist |
| `0xAC` (172) | `0x009A` (154) | Active | ⚠️ SERVICE (1) | 67, 68 | **Dipped headlamp bulb faulty** | Exterior Lighting / Dipped Beam |
| `0xAD` (173) | `0x009B` (155) | Active | ⚠️ SERVICE (1) | 69, 70 | **Main beam headlamp bulb faulty** | Exterior Lighting / Main Beam |
| `0xAE` (174) | `0x009C` (156) | Active | ⚠️ SERVICE (1) | 71, 72 | **Left hand brake light bulb faulty** | Exterior Lighting / Brake Lamp |
| `0xAF` (175) | `0x009D` (157) | Active | ⚠️ SERVICE (1) | 73, 74, 75, 76 | **Foglamp bulb faulty** | Exterior Lighting / Fog Lamp |
| `0xB0` (176) | `0x009E` (158) | Active | ⚠️ SERVICE (1) | 77, 78, 79, 80 | **Direction indicators faulty** | Exterior Lighting / Indicators |
| `0xB1` (177) | `0x009F` (159) | Active | ⚠️ SERVICE (1) | 81, 82 | **Left hand reversing light bulb faulty** | Exterior Lighting / Reversing Lamp |
| `0xB2` (178) | `0x00A0` (160) | Active | ⚠️ SERVICE (1) | 63, 64, 65, 66 | **Sidelamp bulb faulty** | Exterior Lighting / Sidelamp |
| `0xB3` (179) | `0x00D5` (213) | Inactive | 💺 BELT (3) | - | ***(Unmapped / System)*** | Telematic emergency call & road assistance service status |
| `0xB4` (180) | `0x00D6` (214) | Inactive | 💺 BELT (3) | - | ***(Unmapped / System)*** | Telematic emergency call & road assistance service status |
| `0xB5` (181) | `0x00E8` (232) | Active | ⚠️ SERVICE (1) | 139 | **Tyre pressure too low** | Tyre Under-Inflation Detection System (TPMS) alert |
| `0xB6` (182) | `0x013F` (319) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / System)*** | Telematic emergency call & road assistance service status |
| `0xB7` (183) | `0x0140` (320) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / System)*** | Telematic emergency call & road assistance service status |
| `0xB8` (184) | `0x0196` (406) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / System)*** | Telematic emergency call & road assistance service status |
| `0xB9` (185) | `0x0197` (407) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / System)*** | Telematic emergency call & road assistance service status |
| `0xBA` (186) | `0x0199` (409) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / System)*** | Telematic emergency call & road assistance service status |
| `0xBB` (187) | `0x0011` ( 17) | Active | ⚠️ SERVICE (1) | 142 | **Suspension faulty: Max speed 90 km/h** | Electronic / active suspension damping status or fault |
| `0xBC` (188) | `0x0063` ( 99) | Active | ℹ️ INFO (0) | - | **Automatic handbrake deactivated.** | Telematic emergency call & road assistance service status |
| `0xBD` (189) | `0x00A1` (161) | Active | ℹ️ INFO (0) | - | **Parking lamps activated.** | Telematic emergency call & road assistance service status |
| `0xBE` (190) | `0x00F0` (240) | Active | ℹ️ INFO (0) | - | **Diagnosis in progress...** | BSI full vehicle diagnostic self-test check routine currently running |
| `0xBF` (191) | `0x00F1` (241) | Active | ℹ️ INFO (0) | - | **Diagnosis completed** | BSI full vehicle diagnostic self-test check routine finished with zero active critical faults |
| `0xC0` (192) | `0x00F2` (242) | Active | ℹ️ INFO (0) | - | ***(Unmapped / System)*** | Telematic emergency call & road assistance service status |
| `0xC1` (193) | `0x00F3` (243) | Active | ℹ️ INFO (0) | - | ***(Unmapped / System)*** | Telematic emergency call & road assistance service status |
| `0xC2` (194) | `0x00F4` (244) | Active | ℹ️ INFO (0) | - | ***(Unmapped / System)*** | Vehicle status notification / configuration change confirmation |
| `0xC3` (195) | `0x00F5` (245) | Active | ℹ️ INFO (0) | - | ***(Unmapped / System)*** | Telematic emergency call & road assistance service status |
| `0xC4` (196) | `0x00F6` (246) | Active | ℹ️ INFO (0) | - | ***(Unmapped / System)*** | Telematic emergency call & road assistance service status |
| `0xC5` (197) | `0x00F7` (247) | Active | 💺 BELT (3) | 145 | **Rear LH seat belt not fastened** | Passive passenger safety / seatbelt status |
| `0xC6` (198) | `0x00F8` (248) | Active | 💺 BELT (3) | 146 | **Rear center seat belt not fastened** | Passive passenger safety / seatbelt status |
| `0xC7` (199) | `0x00F9` (249) | Active | 💺 BELT (3) | 147 | **Rear RH seat belt not fastened** | Passive passenger safety / seatbelt status |
| `0xC8` (200) | `0x01FF` (511) | Active | ⚠️ SERVICE (1) | - | **Roof operation impossible: rear screen open** | Retractable Hardtop (Coupe-Cabriolet) |
| `0xC9` (201) | `0x0200` (512) | Active | ⚠️ SERVICE (1) | - | **Roof movement impossible: luggage cover not locked** | Retractable Hardtop (Coupe-Cabriolet) |
| `0xCA` (202) | `0x0201` (513) | Active | ℹ️ INFO (0) | - | **Roof operation complete: lock roof** | Retractable Hardtop (Coupe-Cabriolet) |
| `0xCB` (203) | `0x0061` ( 97) | Active | ℹ️ INFO (0) | - | **Handbrake on.** | Telematic emergency call & road assistance service status |
| `0xCC` (204) | `0x0062` ( 98) | Active | ℹ️ INFO (0) | - | **Handbrake off.** | Telematic emergency call & road assistance service status |
| `0xCD` (205) | `0x007B` (123) | Active | ℹ️ INFO (0) | - | **Press footbrake and set gear lever to "N".** | Telematic emergency call & road assistance service status |
| `0xCE` (206) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0xCF` (207) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
