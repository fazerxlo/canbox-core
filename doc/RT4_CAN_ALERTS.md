# RT4 / RT5 CAN Alert Master Table Specification (`CAN_ALERT_TEABLE.md`)

> **Ground Truth Architecture Specification**  
> **Source Platform:** Magneti Marelli RT4 / RT5 Telematics System (CD_RT4-5_SW_8.31)  
> **Target Application:** Portable, pure C99 firmware for custom CAN bus adapters interfacing PSA vehicle CAN networks (CAN 2004 Confort @ 125 kbps) with Chinese Android head units.  
> **Decompiled Source Artifacts:**
> - `Fp_Network_CAN.out` (PowerPC ELF): `Alarm_IndexToPointer_Tab` (0x1db50+0x4d78), `Alarm_Index_Active_Tab` (0x1db50+0x4bd8), `Alarm_BitToIndex_Tab` (0x1db50+0x4958), `get_alarm_index__9C_BCM_CANPUc` (0x00E334), `get_alarm_cpl_status__9C_BCM_CANP16alarm_cpl_status` (0x00E878), `get_alarm_main_status__9C_BCM_CANPUc` (0x00E50c).
> - `mmi_alerts.out` (PowerPC ELF): `_8C_Alerts$ms_uchPictogramLookupTable` (0xc038+0x10c), `Update_Alert_Text_Display__14C_ALERTS_F1_Z2` (0x0074c0), `Req_Disp_Alerts__8C_AlertsUl` (0x00301c).
> - `GraphicBase.800x446.out` & `GraphicBase.480x234.out` (PowerPC ELF): `List_GDO_it_1_ALERTS_F1_Z2` (0x70+0x1324c, 206 entries).
> - `GEN_EN.gph` (GPH Database): `TEXT2006` UTF-16BE Canonical String Table (offset 0x18, 3226 entries).
> - **Real-Vehicle Ground Truth:** Verified on a Peugeot 407 physical MFD display during BSI diagnostic self-check from E2E log `dump_2026-10-04_20-13-39.log`.

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
   - Door / Opening status bitmask evaluated when `alarm_id == 0x0008`, `0x000B`, or `0x0074`:
     - Bit 0 (`0x01`): Front Left Door open
     - Bit 1 (`0x02`): Front Right Door open
     - Bit 2 (`0x04`): Rear Left Door open
     - Bit 3 (`0x08`): Rear Right Door open
     - Bit 4 (`0x10`): Boot / Tailgate open
     - Bit 5 (`0x20`): Rear Screen / Tailgate Glass open
     - Bit 6 (`0x40`): Fuel Filler Flap open
     - Bit 7 (`0x80`): Bonnet / Hood open
   - *Note on `0x0008`:* When `PARAM_DOOR_MASK` is `0x00` or `0xFF` (such as during BSI diagnostic test check), `0x0008` represents **"Braking system faulty"**. When individual bits are active, the vehicle renders the graphical vehicle door opening layout (`C_ALERTS_F5_Z2`).

---

### 1.2 CAN Frame `0x120` Signal Breakdown (Alert Journal Status)

Frame `0x120` is an ISO-TP segmented multi-frame stream transmitting a 168-bit continuous bitfield representing the historical and current alert journal state stored in the BSI.
- Evaluated in `get_alarm_main_status__9C_BCM_CANPUc` (looping `0..167`):
  ```c
  for (uint8_t bit = 0; bit < 168; bit++) {
      uint8_t byte_idx = bit / 8;
      uint8_t bit_mask = 1 << (7 - (bit % 8));
      if (journal_bytes[byte_idx] & bit_mask) {
          uint16_t alarm_idx = Alarm_BitToIndex_Tab[bit];
          if (alarm_idx != 0xFFFF && Alarm_Index_Active_Tab[alarm_idx]) {
              main_status_bitmask[alarm_idx / 8] |= (1 << (alarm_idx % 8));
          }
      }
  }
  ```

---

## 2. Real-Vehicle Ground Truth Verification (Peugeot 407 MFD Diagnostic Check)

The following 10 key alert mappings were verified on physical vehicle hardware during BSI self-test diagnostics (`dump_2026-10-04_20-13-39.log`) and are synchronized in this specification:

| 15-bit CAN ID (Hex) | Dec | Alarm Index | Verified English Display Text | Verification Source & Context |
|---|---|---|---|---|
| `0x00F0` | 240 | `0xBE` (190) | **Diagnosis in progress...** | Displayed during BSI vehicle self-test diagnostic cycle initialization |
| `0x00F1` | 241 | `0xBF` (191) | **Diagnosis completed** | Displayed when BSI self-test diagnostic check terminates without critical active faults |
| `0x0008` | 8 | `0x03` (3) | **Braking system faulty** | Displayed during BSI diagnostic check (when `PARAM_DOOR_MASK` is `0x00` or `0xFF`) |
| `0x0139` | 313 | `0x98` (152) | **Automatic screen wipe deactivated** | Automatic rain wiper sensor setting toggled OFF via multifunction stalk |
| `0x0130` | 304 | `0x94` (148) | **Automatic headlamp lighting activated** | Automatic dusk/twilight headlamp lighting setting toggled ON |
| `0x0131` | 305 | `0x44` (68) | **Child safety deactivated** | Rear electric child safety switch disengaged |
| `0x0138` | 312 | `0x97` (151) | **Fuel level too low** | Fuel level reached critical low threshold warning |
| `0x007F` | 127 | `0x10` (16) | **Depollution system faulty** | Anti-pollution / exhaust depollution emissions system fault (MIL active) |
| `0x0083` | 131 | `0x46` (70) | **Electronic anti-theft faulty** | Transponder ignition key immobiliser communication fault |
| `0x00E5` | 229 | `0x3D` (61) | **Tyre pressure(s) not monitored** | Under-inflation wheel sensor telemetry missing or module unmonitored |

---

## 3. Master CAN Alert Lookup Table (Full 208-Entry Hardware Specification)

The table below lists all 208 alert entries indexed in `Alarm_IndexToPointer_Tab` (`0x00` through `0xCF`).
- **Alarm Index:** Table array index (`0..207`).
- **CAN Alarm ID:** 15-bit CAN ID extracted from `0x1A1` Bytes 0..1 (`((B0 & 0x7F) << 8) | B1`).
- **Active State:** Validated by `Alarm_Index_Active_Tab[index]`.
- **Pictogram & Severity:** Severity enum extracted from `_8C_Alerts$ms_uchPictogramLookupTable`.
- **Journal Bit(s):** Reverse-mapped bit indices in segmented frame `0x120` from `Alarm_BitToIndex_Tab`.
- **Canonical English Alert Text:** English string extracted from `GEN_EN.gph` via `List_GDO_it_1_ALERTS_F1_Z2` and synchronized with vehicle ground truth.

| Alarm Index | CAN Alarm ID (Hex / Dec) | Active State | Severity & Pictogram | 0x120 Journal Bit | Canonical English Alert String | Category & Functional Description |
|---|---|---|---|---|---|---|
| `0x00` (  0) | `0x000C` (12) | Inactive | ℹ️ INFO (0) | 54, 55, 56, 57 | **Tyre pressure low.** | Tyre Under-Inflation Detection System (TPMS) alert |
| `0x01` (  1) | `0x000B` (11) | Active | ⚠️ SERVICE (1) | 10, 11, 12, 13, 14, 15, 16, 125 | **Door(s) open** | Vehicle body openings & passenger access monitoring |
| `0x02` (  2) | `0x019C` (412) | Active | ℹ️ INFO (0) | - | **Press the park button and the brake pedal.** | Braking system operational warning or hydraulic/wear threshold alert |
| `0x03` (  3) | `0x0008` (8) | Active | 🛑 STOP (2) | 3 | **Braking system faulty** | Braking system malfunction / critical brake failure (displayed when door_mask=0x00/0xFF during BSI diagnostic check; if individual door bits set in door_mask, door popup is rendered) |
| `0x04` (  4) | `0x019B` (411) | Active | ℹ️ INFO (0) | - | **To release the parking brake, press the brake pedal.** | Braking system operational warning or hydraulic/wear threshold alert |
| `0x05` (  5) | `0x019A` (410) | Active | ℹ️ INFO (0) | - | **Place gearbox in position "N".** | Transmission / automatic gearbox selector & interlock notice |
| `0x06` (  6) | `0x0002` (2) | Active | ⚠️ SERVICE (1) | - | **Engine oil temperature too high.** | Powertrain / engine management / mechanical fluid level alert |
| `0x07` (  7) | `0x0001` (1) | Active | 🛑 STOP (2) | 1 | **Engine temperature fault: Stop the vehicle** | Powertrain / engine management / mechanical fluid level alert |
| `0x08` (  8) | `0x000F` (15) | Active | ⚠️ SERVICE (1) | 27 | **Risk of particle filter clogging: See handbook** | Powertrain / engine management / mechanical fluid level alert |
| `0x09` (  9) | `0x000A` (10) | Active | ⚠️ SERVICE (1) | - | ***(Parameter / Bulb dependent)*** | Vehicle exterior lighting and bulb failure monitoring |
| `0x0A` ( 10) | `0x000E` (14) | Inactive | ⚠️ SERVICE (1) | - | **FAP additive level low** | Powertrain / engine management / mechanical fluid level alert |
| `0x0B` ( 11) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **Fuel filler flap not locked.** | Vehicle body openings & passenger access monitoring |
| `0x0C` ( 12) | `0x000D` (13) | Active | 🛑 STOP (2) | 59, 60, 61, 62 | **Puncture(s) in tyres detected.** | Tyre Under-Inflation Detection System (TPMS) alert |
| `0x0D` ( 13) | `0x0003` (3) | Active | ⚠️ SERVICE (1) | 6 | **Top up coolant level** | Powertrain / engine management / mechanical fluid level alert |
| `0x0E` ( 14) | `0x0005` (5) | Active | 🛑 STOP (2) | 0 | **Engine oil pressure fault: Stop the vehicle.** | Powertrain / engine management / mechanical fluid level alert |
| `0x0F` ( 15) | `0x0004` (4) | Active | ⚠️ SERVICE (1) | 8 | **Top up engine oil level.** | Powertrain / engine management / mechanical fluid level alert |
| `0x10` ( 16) | `0x007F` (127) | Active | ⚠️ SERVICE (1) | 25 | **Depollution system faulty** | Engine emissions / depollution anti-pollution system malfunction (exhaust gas treatment / catalytic converter / DPF fault) |
| `0x11` ( 17) | `0x0067` (103) | Active | ⚠️ SERVICE (1) | 21 | **Brake pads worn.** | Braking system operational warning or hydraulic/wear threshold alert |
| `0x12` ( 18) | `0x0000` (0) | Active | ℹ️ INFO (0) | - | **Diagnosis OK** | On-board diagnostic system test execution and status |
| `0x13` ( 19) | `0x006E` (110) | Active | ⚠️ SERVICE (1) | 30 | **Gearbox fault: Repair needed** | Transmission / automatic gearbox selector & interlock notice |
| `0x14` ( 20) | `0x006B` (107) | Active | ⚠️ SERVICE (1) | 17 | **ESP/ASR system faulty.** | Vehicle status notification / configuration change confirmation |
| `0x15` ( 21) | `0x006A` (106) | Active | ⚠️ SERVICE (1) | 26 | **ABS system faulty.** | Vehicle status notification / configuration change confirmation |
| `0x16` ( 22) | `0x006C` (108) | Active | ⚠️ SERVICE (1) | 31 | **Suspension faulty.** | Electronic / active suspension damping status or fault |
| `0x17` ( 23) | `0x0066` (102) | Inactive | 🛑 STOP (2) | - | **Braking system faulty.** | Braking system operational warning or hydraulic/wear threshold alert |
| `0x18` ( 24) | `0xFFFF` (-) | Inactive | ⚠️ SERVICE (1) | - | **Side airbag faulty** | Passive passenger safety / restraint system status |
| `0x19` ( 25) | `0xFFFF` (-) | Inactive | ⚠️ SERVICE (1) | - | **Airbag(s) faulty** | Passive passenger safety / restraint system status |
| `0x1A` ( 26) | `0x006F` (111) | Active | ⚠️ SERVICE (1) | 33 | **Speed control system faulty.** | Speed limiter / cruise control / overspeed warning |
| `0x1B` ( 27) | `0x007E` (126) | Active | ⚠️ SERVICE (1) | 141 | **Engine fault: Repair needed.** | Powertrain / engine management / mechanical fluid level alert |
| `0x1C` ( 28) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **Fault : load-shedding in progress.** | Internal unmapped index / diagnostic placeholder |
| `0x1D` ( 29) | `0x0073` (115) | Active | ⚠️ SERVICE (1) | 50 | **Ambient brightness sensor faulty** | Vehicle status notification / configuration change confirmation |
| `0x1E` ( 30) | `0x0072` (114) | Inactive | ⚠️ SERVICE (1) | - | **Rain sensor faulty** | Vehicle status notification / configuration change confirmation |
| `0x1F` ( 31) | `0x007D` (125) | Active | ⚠️ SERVICE (1) | 20 | **Presence of water in diesel filter: Repair needed.** | Vehicle status notification / configuration change confirmation |
| `0x20` ( 32) | `0x0087` (135) | Active | ⚠️ SERVICE (1) | 144 | **Left rear sliding door faulty.** | Vehicle body openings & passenger access monitoring |
| `0x21` ( 33) | `0x0075` (117) | Active | ⚠️ SERVICE (1) | 43 | **Automatic headlamp adjustment faulty** | Vehicle exterior lighting and bulb failure monitoring |
| `0x22` ( 34) | `0x0086` (134) | Active | ⚠️ SERVICE (1) | 143 | **Right rear sliding door faulty** | Vehicle body openings & passenger access monitoring |
| `0x23` ( 35) | `0x0074` (116) | Active | ⚠️ SERVICE (1) | - | **Bulbs blown.** | Vehicle exterior lighting and bulb failure monitoring |
| `0x24` ( 36) | `0x0085` (133) | Inactive | ⚠️ SERVICE (1) | - | **Battery low.** | Electrical power generation & charging circuit status |
| `0x25` ( 37) | `0x0084` (132) | Inactive | ⚠️ SERVICE (1) | - | **Battery charge faulty.** | Electrical power generation & charging circuit status |
| `0x26` ( 38) | `0x0081` (129) | Active | ⚠️ SERVICE (1) | 29 | **Particle filter additive level too low: Repair needed.** | Powertrain / engine management / mechanical fluid level alert |
| `0x27` ( 39) | `0x0080` (128) | Inactive | ⚠️ SERVICE (1) | 24 | **Depollution system faulty.** | Vehicle status notification / configuration change confirmation |
| `0x28` ( 40) | `0x00D9` (217) | Active | ℹ️ INFO (0) | 9 | **Handbrake on !** | Braking system operational warning or hydraulic/wear threshold alert |
| `0x29` ( 41) | `0x00D2` (210) | Active | 💺 BELT (3) | 101, 102 | **Seat belt(s) not fastened.** | Passive passenger safety / restraint system status |
| `0x2A` ( 42) | `0x00D0` (208) | Inactive | ℹ️ INFO (0) | - | **Passenger airbag switched off.** | Passive passenger safety / restraint system status |
| `0x2B` ( 43) | `0x00DF` (223) | Active | ℹ️ INFO (0) | 46 | **Screen washer fluid level low.** | Visibility / wash-wipe equipment status |
| `0x2C` ( 44) | `0x00CB` (203) | Inactive | ℹ️ INFO (0) | - | **Current speed too high** | Speed limiter / cruise control / overspeed warning |
| `0x2D` ( 45) | `0x00CA` (202) | Inactive | ⚠️ SERVICE (1) | - | **Ignition key left in** | Vehicle status notification / configuration change confirmation |
| `0x2E` ( 46) | `0x00C9` (201) | Inactive | ℹ️ INFO (0) | - | **Sidelights left on** | Vehicle exterior lighting and bulb failure monitoring |
| `0x2F` ( 47) | `0xFFFF` (-) | Inactive | 💺 BELT (3) | - | **Driver's seatbelt not fastened** | Passive passenger safety / restraint system status |
| `0x30` ( 48) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **Shock sensor faulty.** | Internal unmapped index / diagnostic placeholder |
| `0x31` ( 49) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x32` ( 50) | `0x00E4` (228) | Active | ℹ️ INFO (0) | 138 | **Check and re-initialise tyre pressure.** | Tyre Under-Inflation Detection System (TPMS) alert |
| `0x33` ( 51) | `0x00E3` (227) | Active | ℹ️ INFO (0) | 47 | **Remote control battery spent** | Electrical power generation & charging circuit status |
| `0x34` ( 52) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x35` ( 53) | `0x00D7` (215) | Active | ℹ️ INFO (0) | 106 | **Put automatic gearbox in "P" position.** | Transmission / automatic gearbox selector & interlock notice |
| `0x36` ( 54) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **Stoplamp test : brake gently** | Braking system operational warning or hydraulic/wear threshold alert |
| `0x37` ( 55) | `0x00E0` (224) | Active | ℹ️ INFO (0) | 22 | **Fuel level low** | Vehicle status notification / configuration change confirmation |
| `0x38` ( 56) | `0xFFFF` (-) | Inactive | ⚠️ SERVICE (1) | - | **Automatic headlights deactivated** | Internal unmapped index / diagnostic placeholder |
| `0x39` ( 57) | `0xFFFF` (-) | Inactive | 💺 BELT (3) | 126 | **Rear LH passenger seatbelt unfastened.** | Passive passenger safety / restraint system status |
| `0x3A` ( 58) | `0xFFFF` (-) | Inactive | 💺 BELT (3) | - | **Rear RH passenger seatbelt unfastened.** | Passive passenger safety / restraint system status |
| `0x3B` ( 59) | `0xFFFF` (-) | Inactive | 💺 BELT (3) | - | **Front passenger seatbelt not fastened.** | Passive passenger safety / restraint system status |
| `0x3C` ( 60) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **Driving school pedals indication** | Internal unmapped index / diagnostic placeholder |
| `0x3D` ( 61) | `0x00E5` (229) | Active | ⚠️ SERVICE (1) | 52, 148, 149, 150, 151 | **Tyre pressure(s) not monitored** | Under-inflation detection system inactive or wheel sensors missing / not communicating |
| `0x3E` ( 62) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x3F` ( 63) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x40` ( 64) | `0x0135` (309) | Inactive | ℹ️ INFO (0) | - | **Doors locked** | Vehicle body openings & passenger access monitoring |
| `0x41` ( 65) | `0x013A` (314) | Inactive | ℹ️ INFO (0) | - | **ESP system deactivated** | Vehicle status notification / configuration change confirmation |
| `0x42` ( 66) | `0x0137` (311) | Active | ℹ️ INFO (0) | - | **Child safety activated** | Vehicle status notification / configuration change confirmation |
| `0x43` ( 67) | `0x0133` (307) | Active | ℹ️ INFO (0) | - | **Self locking doors activated** | Vehicle body openings & passenger access monitoring |
| `0x44` ( 68) | `0x0131` (305) | Active | ℹ️ INFO (0) | - | **Child safety deactivated** | Electric child safety lock disabled on rear passenger doors / windows |
| `0x45` ( 69) | `0x012F` (303) | Active | ℹ️ INFO (0) | - | **Automatic windscreen wiper activated** | Visibility / wash-wipe equipment status |
| `0x46` ( 70) | `0x0083` (131) | Active | ⚠️ SERVICE (1) | 37 | **Electronic anti-theft faulty** | Transponder key immobiliser / electronic anti-theft system communication error |
| `0x47` ( 71) | `0x012E` (302) | Inactive | ℹ️ INFO (0) | - | **"Sport" suspension mode activated.** | Electronic / active suspension damping status or fault |
| `0x48` ( 72) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x49` ( 73) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x4A` ( 74) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x4B` ( 75) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **Fuel used change in progress** | Internal unmapped index / diagnostic placeholder |
| `0x4C` ( 76) | `0x00E2` (226) | Inactive | ⚠️ SERVICE (1) | - | **LPG fuel refused** | Vehicle status notification / configuration change confirmation |
| `0x4D` ( 77) | `0x0082` (130) | Inactive | ⚠️ SERVICE (1) | - | **LPG system faulty.** | Vehicle status notification / configuration change confirmation |
| `0x4E` ( 78) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **LPG in use** | Internal unmapped index / diagnostic placeholder |
| `0x4F` ( 79) | `0xFFFF` (-) | Inactive | ⚠️ SERVICE (1) | - | **Min level LPG.** | Internal unmapped index / diagnostic placeholder |
| `0x50` ( 80) | `0x0198` (408) | Active | ⚠️ SERVICE (1) | - | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x51` ( 81) | `0x0195` (405) | Inactive | ℹ️ INFO (0) | - | **Foglamp bulb(s) faulty** | Vehicle exterior lighting and bulb failure monitoring |
| `0x52` ( 82) | `0x0194` (404) | Inactive | ℹ️ INFO (0) | - | **Foglamp bulb(s) faulty** | Vehicle exterior lighting and bulb failure monitoring |
| `0x53` ( 83) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | ***(Unmapped / Inactive)*** | Internal unmapped index / diagnostic placeholder |
| `0x54` ( 84) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **Foglamp bulb(s) faulty** | Vehicle exterior lighting and bulb failure monitoring |
| `0x55` ( 85) | `0x0193` (403) | Inactive | ℹ️ INFO (0) | - | **Foglamp bulb(s) faulty** | Vehicle exterior lighting and bulb failure monitoring |
| `0x56` ( 86) | `0x0192` (402) | Inactive | ℹ️ INFO (0) | - | **Headlamp bulbs(s) faulty** | Vehicle exterior lighting and bulb failure monitoring |
| `0x57` ( 87) | `0x0191` (401) | Inactive | ℹ️ INFO (0) | - | **Headlamp bulbs(s) faulty** | Vehicle exterior lighting and bulb failure monitoring |
| `0x58` ( 88) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **Direction indicator(s) faulty.** | Internal unmapped index / diagnostic placeholder |
| `0x59` ( 89) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **Direction indicator(s) faulty.** | Internal unmapped index / diagnostic placeholder |
| `0x5A` ( 90) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **Direction indicator(s) faulty.** | Internal unmapped index / diagnostic placeholder |
| `0x5B` ( 91) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **Direction indicator(s) faulty.** | Internal unmapped index / diagnostic placeholder |
| `0x5C` ( 92) | `0x0202` (514) | Active | ⚠️ SERVICE (1) | 154 | **Right hand reversing light bulb faulty** | Vehicle exterior lighting and bulb failure monitoring |
| `0x5D` ( 93) | `0x020B` (523) | Active | ⚠️ SERVICE (1) | 156 | **Left hand reversing light bulb fault** | Vehicle exterior lighting and bulb failure monitoring |
| `0x5E` ( 94) | `0x020A` (522) | Active | ℹ️ INFO (0) | - | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x5F` ( 95) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **Brake light bulbs faulty** | Braking system operational warning or hydraulic/wear threshold alert |
| `0x60` ( 96) | `0x0209` (521) | Active | ℹ️ INFO (0) | - | **Reversing light bulbs faulty** | Vehicle exterior lighting and bulb failure monitoring |
| `0x61` ( 97) | `0x0206` (518) | Active | ℹ️ INFO (0) | - | **No bulb blown.** | Vehicle exterior lighting and bulb failure monitoring |
| `0x62` ( 98) | `0x0205` (517) | Active | ℹ️ INFO (0) | - | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x63` ( 99) | `0x0204` (516) | Active | ℹ️ INFO (0) | - | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x64` (100) | `0x0203` (515) | Active | ℹ️ INFO (0) | - | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x65` (101) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **Sidelight bulb(s) faul** | Vehicle exterior lighting and bulb failure monitoring |
| `0x66` (102) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **Sidelight bulb(s) faul** | Vehicle exterior lighting and bulb failure monitoring |
| `0x67` (103) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **Sidelight bulb(s) fault.** | Vehicle exterior lighting and bulb failure monitoring |
| `0x68` (104) | `0x01FB` (507) | Active | ℹ️ INFO (0) | - | **Activate regional mode** | Vehicle status notification / configuration change confirmation |
| `0x69` (105) | `0x01FA` (506) | Active | ℹ️ INFO (0) | - | **Customer Contact Cente** | Telematic emergency call & road assistance service status |
| `0x6A` (106) | `0x01F9` (505) | Inactive | ⚠️ SERVICE (1) | - | **Centre Contact Clien** | Telematic emergency call & road assistance service status |
| `0x6B` (107) | `0x01F8` (504) | Active | ℹ️ INFO (0) | - | **Peugeot Assistance** | Telematic emergency call & road assistance service status |
| `0x6C` (108) | `0x01F7` (503) | Active | ℹ️ INFO (0) | - | **Peugeot Assistance** | Telematic emergency call & road assistance service status |
| `0x6D` (109) | `0x01F6` (502) | Active | ℹ️ INFO (0) | - | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x6E` (110) | `0x01FE` (510) | Active | ⚠️ SERVICE (1) | - | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x6F` (111) | `0x01F5` (501) | Inactive | ⚠️ SERVICE (1) | - | **Activate video mod** | Vehicle status notification / configuration change confirmation |
| `0x70` (112) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **Deactivate video mod** | Internal unmapped index / diagnostic placeholder |
| `0x71` (113) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **Video parameters** | Internal unmapped index / diagnostic placeholder |
| `0x72` (114) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **Rear screen open** | Vehicle body openings & passenger access monitoring |
| `0x73` (115) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **More than one door open.** | Vehicle body openings & passenger access monitoring |
| `0x74` (116) | `0x007C` (124) | Inactive | ⚠️ SERVICE (1) | - | **Tyre pressure(s) low** | Tyre Under-Inflation Detection System (TPMS) alert |
| `0x75` (117) | `0x01FD` (509) | Active | ℹ️ INFO (0) | - | **Tyre pressure(s) low** | Tyre Under-Inflation Detection System (TPMS) alert |
| `0x76` (118) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **Puncture(s) in tyres detected.** | Tyre Under-Inflation Detection System (TPMS) alert |
| `0x77` (119) | `0x01FC` (508) | Active | ℹ️ INFO (0) | - | **Tyre pressure(s) not monitored** | Tyre Under-Inflation Detection System (TPMS) alert |
| `0x78` (120) | `0x0012` (18) | Active | ⚠️ SERVICE (1) | 152 | **Stop & Start activated** | Vehicle status notification / configuration change confirmation |
| `0x79` (121) | `0x0068` (104) | Active | ⚠️ SERVICE (1) | 123 | **Stop & Start deactivated** | Vehicle status notification / configuration change confirmation |
| `0x7A` (122) | `0x0069` (105) | Active | ⚠️ SERVICE (1) | 124 | **Copy CD to JBX** | Vehicle status notification / configuration change confirmation |
| `0x7B` (123) | `0x006D` (109) | Active | 🛑 STOP (2) | 5 | **Stop the cop** | Vehicle status notification / configuration change confirmation |
| `0x7C` (124) | `0x0013` (19) | Active | ⚠️ SERVICE (1) | 153 | **Activate Introscan** | Vehicle status notification / configuration change confirmation |
| `0x7D` (125) | `0x0078` (120) | Active | ⚠️ SERVICE (1) | 23 | **Deactivate Introscan** | Vehicle status notification / configuration change confirmation |
| `0x7E` (126) | `0x0088` (136) | Active | ⚠️ SERVICE (1) | 91 | **Activate random mode** | Vehicle status notification / configuration change confirmation |
| `0x7F` (127) | `0x0089` (137) | Active | ⚠️ SERVICE (1) | 92 | **Deactivate rand. mod** | Vehicle status notification / configuration change confirmation |
| `0x80` (128) | `0x008A` (138) | Active | ⚠️ SERVICE (1) | 18 | **Reminder : Lane monitoring inactive.** | Vehicle status notification / configuration change confirmation |
| `0x81` (129) | `0x0076` (118) | Active | ⚠️ SERVICE (1) | 34 | **Anti-wander lane-crossing warning device faulty.** | Vehicle status notification / configuration change confirmation |
| `0x82` (130) | `0x008C` (140) | Inactive | ℹ️ INFO (0) | 45 | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x83` (131) | `0x008D` (141) | Active | ⚠️ SERVICE (1) | 94, 95, 96, 97 | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x84` (132) | `0x0222` (546) | Active | ℹ️ INFO (0) | 51 | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x85` (133) | `0x0221` (545) | Active | ℹ️ INFO (0) | - | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x86` (134) | `0x0220` (544) | Active | ℹ️ INFO (0) | - | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x87` (135) | `0x0091` (145) | Inactive | ℹ️ INFO (0) | - | **Right hand reversing light bulb faulty** | Vehicle exterior lighting and bulb failure monitoring |
| `0x88` (136) | `0x00CD` (205) | Active | ℹ️ INFO (0) | - | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x89` (137) | `0x00D1` (209) | Active | ℹ️ INFO (0) | 100 | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x8A` (138) | `0x00D4` (212) | Inactive | 💺 BELT (3) | - | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x8B` (139) | `0x00D8` (216) | Active | ℹ️ INFO (0) | 107 | **Tyre pressure low.** | Tyre Under-Inflation Detection System (TPMS) alert |
| `0x8C` (140) | `0x00DE` (222) | Active | ⚠️ SERVICE (1) | 108, 109, 110, 111, 112, 113, 114, 115 | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x8D` (141) | `0x00E1` (225) | Active | ℹ️ INFO (0) | - | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x8E` (142) | `0x00E6` (230) | Inactive | ℹ️ INFO (0) | 117 | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x8F` (143) | `0x00EC` (236) | Active | ℹ️ INFO (0) | 130 | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x90` (144) | `0x00ED` (237) | Active | ℹ️ INFO (0) | - | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x91` (145) | `0x00EE` (238) | Inactive | ℹ️ INFO (0) | 119 | **Suspension faulty max. speed : 90 km/h** | Electronic / active suspension damping status or fault |
| `0x92` (146) | `0x00EF` (239) | Active | ℹ️ INFO (0) | 131 | **Automatic handbrake deactivated.** | Braking system operational warning or hydraulic/wear threshold alert |
| `0x93` (147) | `0x012D` (301) | Inactive | ℹ️ INFO (0) | - | **Parking lamps activated.** | Vehicle exterior lighting and bulb failure monitoring |
| `0x94` (148) | `0x0130` (304) | Active | ℹ️ INFO (0) | - | **Automatic headlamp lighting activated** | Automatic dusk/darkness headlamp lighting feature enabled (in diagnostic check: Diagnosis in progress...) |
| `0x95` (149) | `0x0132` (306) | Active | ℹ️ INFO (0) | - | **Diagnosis complete** | On-board diagnostic system test execution and status |
| `0x96` (150) | `0x0136` (310) | Inactive | ℹ️ INFO (0) | - | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x97` (151) | `0x0138` (312) | Active | ℹ️ INFO (0) | - | **Fuel level too low** | Primary fuel reserve threshold reached; immediate refueling required |
| `0x98` (152) | `0x0139` (313) | Inactive | ℹ️ INFO (0) | - | **Automatic screen wipe deactivated** | Automatic rain-sensing windscreen wiper feature disabled |
| `0x99` (153) | `0x013B` (315) | Inactive | ℹ️ INFO (0) | 120 | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x9A` (154) | `0x013C` (316) | Inactive | ℹ️ INFO (0) | 121 | ***(Reserved / System)*** | Vehicle status notification / configuration change confirmation |
| `0x9B` (155) | `0x00DA` (218) | Inactive | ℹ️ INFO (0) | - | **Rear LH passenger seatbelt unfastened.** | Passive passenger safety / restraint system status |
| `0x9C` (156) | `0x0079` (121) | Active | ⚠️ SERVICE (1) | 90 | **Rear cent. passenger seatbelt unfastened** | Passive passenger safety / restraint system status |
| `0x9D` (157) | `0x00E7` (231) | Active | ⚠️ SERVICE (1) | 118 | **Rear RH passenger seatbelt unfastened.** | Passive passenger safety / restraint system status |
| `0x9E` (158) | `0x00E9` (233) | Inactive | ℹ️ INFO (0) | 127 | **Roof operation impossible : screen open.** | Vehicle body openings & passenger access monitoring |
| `0x9F` (159) | `0x00EA` (234) | Active | ⚠️ SERVICE (1) | 128 | **Roof operation impossible : cassette not locked.** | Retractable hard-top / cabriolet roof mechanism state |
| `0xA0` (160) | `0x00EB` (235) | Active | ⚠️ SERVICE (1) | 129 | **Roof operation complete: lock roof** | Retractable hard-top / cabriolet roof mechanism state |
| `0xA1` (161) | `0x00D3` (211) | Active | 💺 BELT (3) | 103, 104, 105 | **Handbrake on** | Braking system operational warning or hydraulic/wear threshold alert |
| `0xA2` (162) | `0x013D` (317) | Active | ℹ️ INFO (0) | - | **Handbrake off.** | Braking system operational warning or hydraulic/wear threshold alert |
| `0xA3` (163) | `0x013E` (318) | Inactive | ℹ️ INFO (0) | - | **Press footbrake and set gear lever to "N".** | Braking system operational warning or hydraulic/wear threshold alert |
| `0xA4` (164) | `0x00CE` (206) | Active | ℹ️ INFO (0) | - | **Rear LH & CENT pass. seatbelts fastened.** | Passive passenger safety / restraint system status |
| `0xA5` (165) | `0x0134` (308) | Active | ℹ️ INFO (0) | - | **Rear RH passenger seatbelts fastened** | Passive passenger safety / restraint system status |
| `0xA6` (166) | `0x0064` (100) | Active | ⚠️ SERVICE (1) | 140 | **Overtaking assistance beeper : O** | Telematic emergency call & road assistance service status |
| `0xA7` (167) | `0x007A` (122) | Active | ⚠️ SERVICE (1) | 133 | **Overtaking assistance beeper: OF** | Telematic emergency call & road assistance service status |
| `0xA8` (168) | `0x0092` (146) | Active | ⚠️ SERVICE (1) | 134 | **Overtaking assistance system faulty.** | Telematic emergency call & road assistance service status |
| `0xA9` (169) | `0x0095` (149) | Inactive | ℹ️ INFO (0) | - | **Lane monitoring system deactivated** | Vehicle status notification / configuration change confirmation |
| `0xAA` (170) | `0x0096` (150) | Active | ℹ️ INFO (0) | 136 | **Peugeot Assistance** | Telematic emergency call & road assistance service status |
| `0xAB` (171) | `0x0097` (151) | Active | ⚠️ SERVICE (1) | 137 | **Peugeot Assistenti** | Vehicle status notification / configuration change confirmation |
| `0xAC` (172) | `0x009A` (154) | Active | ⚠️ SERVICE (1) | 67, 68 | **Peugeot Assistênci** | Vehicle status notification / configuration change confirmation |
| `0xAD` (173) | `0x009B` (155) | Active | ⚠️ SERVICE (1) | 69, 70 | **Peugeot Assistance** | Telematic emergency call & road assistance service status |
| `0xAE` (174) | `0x009C` (156) | Active | ⚠️ SERVICE (1) | 71, 72 | **Peugeot Assistance** | Telematic emergency call & road assistance service status |
| `0xAF` (175) | `0x009D` (157) | Active | ⚠️ SERVICE (1) | 73, 74, 75, 76 | **Peugeot Assistance** | Telematic emergency call & road assistance service status |
| `0xB0` (176) | `0x009E` (158) | Active | ⚠️ SERVICE (1) | 77, 78, 79, 80 | **Peugeot Assistance** | Telematic emergency call & road assistance service status |
| `0xB1` (177) | `0x009F` (159) | Active | ⚠️ SERVICE (1) | 81, 82 | **Peugeot Assistance** | Telematic emergency call & road assistance service status |
| `0xB2` (178) | `0x00A0` (160) | Active | ⚠️ SERVICE (1) | 63, 64, 65, 66 | **Peugeot Assistance** | Telematic emergency call & road assistance service status |
| `0xB3` (179) | `0x00D5` (213) | Inactive | 💺 BELT (3) | - | **Peugeot Assistance** | Telematic emergency call & road assistance service status |
| `0xB4` (180) | `0x00D6` (214) | Inactive | 💺 BELT (3) | - | **Peugeot Assistance** | Telematic emergency call & road assistance service status |
| `0xB5` (181) | `0x00E8` (232) | Active | ⚠️ SERVICE (1) | 139 | **Peugeot Assistance** | Telematic emergency call & road assistance service status |
| `0xB6` (182) | `0x013F` (319) | Inactive | ℹ️ INFO (0) | - | **Peugeot Assistance** | Telematic emergency call & road assistance service status |
| `0xB7` (183) | `0x0140` (320) | Inactive | ℹ️ INFO (0) | - | **Peugeot Assistance** | Telematic emergency call & road assistance service status |
| `0xB8` (184) | `0x0196` (406) | Inactive | ℹ️ INFO (0) | - | **Peugeot Assistance** | Telematic emergency call & road assistance service status |
| `0xB9` (185) | `0x0197` (407) | Inactive | ℹ️ INFO (0) | - | **Peugeot Assistance** | Telematic emergency call & road assistance service status |
| `0xBA` (186) | `0x0199` (409) | Inactive | ℹ️ INFO (0) | - | **Peugeot Assistance** | Telematic emergency call & road assistance service status |
| `0xBB` (187) | `0x0011` (17) | Active | ⚠️ SERVICE (1) | 142 | **Peugeot Assistance** | Telematic emergency call & road assistance service status |
| `0xBC` (188) | `0x0063` (99) | Active | ℹ️ INFO (0) | - | **Peugeot Assistance** | Telematic emergency call & road assistance service status |
| `0xBD` (189) | `0x00A1` (161) | Active | ℹ️ INFO (0) | - | **Peugeot Assistance** | Telematic emergency call & road assistance service status |
| `0xBE` (190) | `0x00F0` (240) | Active | ℹ️ INFO (0) | - | **Diagnosis in progress...** | BSI full vehicle diagnostic self-test check routine currently running |
| `0xBF` (191) | `0x00F1` (241) | Active | ℹ️ INFO (0) | - | **Diagnosis completed** | BSI full vehicle diagnostic self-test check routine finished with zero active critical faults |
| `0xC0` (192) | `0x00F2` (242) | Active | ℹ️ INFO (0) | - | **Citroën Assistance** | Telematic emergency call & road assistance service status |
| `0xC1` (193) | `0x00F3` (243) | Active | ℹ️ INFO (0) | - | **Citroën Assistance** | Telematic emergency call & road assistance service status |
| `0xC2` (194) | `0x00F4` (244) | Active | ℹ️ INFO (0) | - | **Citroën Assistenci** | Vehicle status notification / configuration change confirmation |
| `0xC3` (195) | `0x00F5` (245) | Active | ℹ️ INFO (0) | - | **Citroën Assistance** | Telematic emergency call & road assistance service status |
| `0xC4` (196) | `0x00F6` (246) | Active | ℹ️ INFO (0) | - | **Citroën Assistance** | Telematic emergency call & road assistance service status |
| `0xC5` (197) | `0x00F7` (247) | Active | 💺 BELT (3) | 145 | **Assistenza Citroën** | Vehicle status notification / configuration change confirmation |
| `0xC6` (198) | `0x00F8` (248) | Active | 💺 BELT (3) | 146 | **Citroën Assistance** | Telematic emergency call & road assistance service status |
| `0xC7` (199) | `0x00F9` (249) | Active | 💺 BELT (3) | 147 | **Citroën Assistenti** | Vehicle status notification / configuration change confirmation |
| `0xC8` (200) | `0x01FF` (511) | Active | ⚠️ SERVICE (1) | - | **Citroën Assistênci** | Vehicle status notification / configuration change confirmation |
| `0xC9` (201) | `0x0200` (512) | Active | ⚠️ SERVICE (1) | - | **Citroën Assistance** | Telematic emergency call & road assistance service status |
| `0xCA` (202) | `0x0201` (513) | Active | ℹ️ INFO (0) | - | **Citroën Assistance** | Telematic emergency call & road assistance service status |
| `0xCB` (203) | `0x0061` (97) | Active | ℹ️ INFO (0) | - | **Citroën Assistance** | Telematic emergency call & road assistance service status |
| `0xCC` (204) | `0x0062` (98) | Active | ℹ️ INFO (0) | - | **Citroën Assistance** | Telematic emergency call & road assistance service status |
| `0xCD` (205) | `0x007B` (123) | Active | ℹ️ INFO (0) | - | **Citroën Assistance** | Telematic emergency call & road assistance service status |
| `0xCE` (206) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **Cannot read CD** | Internal unmapped index / diagnostic placeholder |
| `0xCF` (207) | `0xFFFF` (-) | Inactive | ℹ️ INFO (0) | - | **Cannot read CD** | Internal unmapped index / diagnostic placeholder |

---

## 4. Auxiliary Sub-system Alert Displays (`C_ALERTS_F5` .. `F13`)

In addition to the primary modal alert popup (`C_ALERTS_F1_Z2`), the RT4/RT5 firmware manages dedicated multi-widget views for complex vehicle sub-systems:

### 4.1 `C_ALERTS_F5_Z2`: Door & Opening Vehicle Silhouette
Invoked when `alarm_id == 0x000B` (`Door(s) open`) or when opening bits are present in `PARAM_DOOR_MASK` (Byte 3 of `0x1A1`).
- Front Left Door: Bit 0 (`0x01`)
- Front Right Door: Bit 1 (`0x02`)
- Rear Left Door: Bit 2 (`0x04`)
- Rear Right Door: Bit 3 (`0x08`)
- Boot / Tailgate: Bit 4 (`0x10`)
- Rear Window: Bit 5 (`0x20`)
- Fuel Flap: Bit 6 (`0x40`)
- Bonnet / Hood: Bit 7 (`0x80`)

### 4.2 `C_ALERTS_F6_Z2`: Blown Exterior Bulbs (`it_ALERTS_F6_Z2`)
When `alarm_id == 0x0074` (`Bulbs blown.`), `PARAM_DETAIL_1` indexes 33 specific exterior bulb positions:
- `0`: Sidelight bulb(s) faulty
- `1`: Left hand sidelight bulb faulty
- `2`: Right hand sidelight bulb faulty
- `3`: Dipped headlamp bulb(s) faulty
- `4`: Left dipped headlamp bulb faulty
- `5`: Right dipped headlamp bulb faulty
- `6`: Main beam headlamp bulb(s) faulty
- `7`: Direction indicator(s) faulty
- `8`: Brake light bulbs faulty
- `9`: Reversing light bulbs faulty
- `10`: Rear fog lamp bulbs faulty
- `11..32`: Additional localized lighting circuits

### 4.3 `C_ALERTS_F7_Z2` & `C_ALERTS_F9_Z2`: Tyre Pressure & Puncture Monitoring
- `F9_Z2` renders individual wheel icons (FL, FR, RL, RR) with color-coded status:
  - Green / Default: Pressure OK
  - Amber (Warning): `Tyre pressure(s) low`
  - Red (STOP): `Puncture(s) in tyres detected.`
  - Grey / Flashing: `Tyre pressure(s) not monitored` (sensor battery flat or missing)

---

## 5. Portable Pure C99 Firmware Implementation

### 5.1 Lookup Tables & State Machine

```c
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define TOTAL_ALARM_ENTRIES  208

typedef enum {
    ALARM_SEV_INFO     = 0,
    ALARM_SEV_SERVICE  = 1,
    ALARM_SEV_STOP     = 2,
    ALARM_SEV_SEATBELT = 3
} alarm_severity_t;

typedef struct {
    uint16_t         alarm_id;
    uint8_t          alarm_index;
    bool             active;
    alarm_severity_t severity;
    uint8_t          sound_id;
    const char      *alert_text;
} can_alert_entry_t;

/* State machine tracking */
typedef struct {
    uint16_t last_alarm_id;
    bool     last_toggle_bit;
    bool     popup_active;
} can_alert_state_t;

static can_alert_state_t s_alert_state = {0};

/* Exact 208-entry Alarm Index to CAN ID table */
static const uint16_t Alarm_IndexToPointer_Tab[TOTAL_ALARM_ENTRIES] = {
    0x000C, 0x000B, 0x019C, 0x0008, 0x019B, 0x019A, 0x0002, 0x0001,
    0x000F, 0x000A, 0x000E, 0xFFFF, 0x000D, 0x0003, 0x0005, 0x0004,
    0x007F, 0x0067, 0x0000, 0x006E, 0x006B, 0x006A, 0x006C, 0x0066,
    0xFFFF, 0xFFFF, 0x006F, 0x007E, 0xFFFF, 0x0073, 0x0072, 0x007D,
    0x0087, 0x0075, 0x0086, 0x0074, 0x0085, 0x0084, 0x0081, 0x0080,
    0x00D9, 0x00D2, 0x00D0, 0x00DF, 0x00CB, 0x00CA, 0x00C9, 0xFFFF,
    0xFFFF, 0xFFFF, 0x00E4, 0x00E3, 0xFFFF, 0x00D7, 0xFFFF, 0x00E0,
    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x00E5, 0xFFFF, 0xFFFF,
    0x0135, 0x013A, 0x0137, 0x0133, 0x0131, 0x012F, 0x0083, 0x012E,
    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x00E2, 0x0082, 0xFFFF, 0xFFFF,
    0x0198, 0x0195, 0x0194, 0xFFFF, 0xFFFF, 0x0193, 0x0192, 0x0191,
    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x0202, 0x020B, 0x020A, 0xFFFF,
    0x0209, 0x0206, 0x0205, 0x0204, 0x0203, 0xFFFF, 0xFFFF, 0xFFFF,
    0x01FB, 0x01FA, 0x01F9, 0x01F8, 0x01F7, 0x01F6, 0x01FE, 0x01F5,
    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x007C, 0x01FD, 0xFFFF, 0x01FC,
    0x0012, 0x0068, 0x0069, 0x006D, 0x0013, 0x0078, 0x0088, 0x0089,
    0x008A, 0x0076, 0x008C, 0x008D, 0x0222, 0x0221, 0x0220, 0x0091,
    0x00CD, 0x00D1, 0x00D4, 0x00D8, 0x00DE, 0x00E1, 0x00E6, 0x00EC,
    0x00ED, 0x00EE, 0x00EF, 0x012D, 0x0130, 0x0132, 0x0136, 0x0138,
    0x0139, 0x013B, 0x013C, 0x00DA, 0x0079, 0x00E7, 0x00E9, 0x00EA,
    0x00EB, 0x00D3, 0x013D, 0x013E, 0x00CE, 0x0134, 0x0064, 0x007A,
    0x0092, 0x0095, 0x0096, 0x0097, 0x009A, 0x009B, 0x009C, 0x009D,
    0x009E, 0x009F, 0x00A0, 0x00D5, 0x00D6, 0x00E8, 0x013F, 0x0140,
    0x0196, 0x0197, 0x0199, 0x0011, 0x0063, 0x00A1, 0x00F0, 0x00F1,
    0x00F2, 0x00F3, 0x00F4, 0x00F5, 0x00F6, 0x00F7, 0x00F8, 0x00F9,
    0x01FF, 0x0200, 0x0201, 0x0061, 0x0062, 0x007B, 0xFFFF, 0xFFFF,
};

/* Exact 208-entry Active Flag table */
static const uint8_t Alarm_Index_Active_Tab[TOTAL_ALARM_ENTRIES] = {
    0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1,
    1, 1, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0,
    0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0,
    1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1,
    1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1,
    1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
};

/* Exact 206-entry Pictogram Severity table */
static const uint8_t Alert_Pictogram_Tab[206] = {
    0, 1, 0, 2, 0, 0, 1, 2, 1, 1, 1, 0, 2, 1, 2, 1,
    1, 1, 0, 1, 1, 1, 1, 2, 1, 1, 1, 1, 0, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 0, 3, 0, 0, 0, 1, 0, 3,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 3, 3, 3, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 2, 1, 1, 1, 1,
    1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 3, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1,
    1, 3, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1,
    1, 1, 1, 3, 3, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 3, 3, 3, 1, 1, 0, 0, 0, 0,
};

/* 0x1A1 Alert Processing Routine */
void can_alert_process_0x1A1(const uint8_t frame[8], can_alert_entry_t *out_alert)
{
    bool toggle_bit = (frame[0] & 0x80) != 0;
    uint16_t alarm_id = ((frame[0] & 0x7F) << 8) | frame[1];
    bool display_request = (frame[2] & 0x80) != 0;
    uint8_t priority = (frame[2] >> 4) & 0x07;
    uint8_t sound_id = frame[2] & 0x0F;

    /* Detect sequence toggle transition */
    bool is_new_event = (toggle_bit != s_alert_state.last_toggle_bit);
    s_alert_state.last_toggle_bit = toggle_bit;

    if (!display_request || alarm_id == 0) {
        s_alert_state.popup_active = false;
        out_alert->active = false;
        return;
    }

    /* Resolve Alarm Index */
    int resolved_index = -1;
    for (int i = 0; i < TOTAL_ALARM_ENTRIES; i++) {
        if (Alarm_IndexToPointer_Tab[i] == alarm_id && Alarm_Index_Active_Tab[i] == 1) {
            resolved_index = i;
            break;
        }
    }

    out_alert->alarm_id = alarm_id;
    out_alert->alarm_index = (resolved_index >= 0) ? (uint8_t)resolved_index : 0xFF;
    out_alert->active = true;
    out_alert->severity = (resolved_index >= 0 && resolved_index < 206) ? 
                          (alarm_severity_t)Alert_Pictogram_Tab[resolved_index] : 
                          (alarm_severity_t)(priority > 2 ? ALARM_SEV_STOP : priority);
    out_alert->sound_id = sound_id;
}
```
