# Hiworld (WC / 0x5AA5) Vehicle Central Configuration Specification
## Master Catalog: Headunit Car Settings, Personalization, Lighting Options & BSI Overrides
### PSA CAN 2004 (Peugeot 407 / AEE2004 Comfort Bus) $\longleftrightarrow$ Hiworld Android Head Unit Bridge

**Document File:** `CANBOX_SPEC_HIWORLD_407_11_CAR_CONFIGURATION_OPTIONS.md`  
**Document Version:** 2.0.0  
**Target Platform:** Pure C99 Embedded Firmware (`canbox-core` on STM32 / NUC131)  
**Vehicle Network:** Peugeot 407 (PSA CAN 2004 / AEE2004 Comfort Bus @ 125 kbps)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, 38,400 baud 8N1, additive sum checksum)  
**Standard References:**
1. Magneti Marelli RT4 (NaviDrive SW 8.31) Firmware Specification (`RT4_AI_CAN_SETTINGS_GUIDE.md`)
2. Reverse-Engineered Android Automotive Core (`com.qf.vehicle.band.peugeot.parse.wc`, `CentralState.java`)
3. OpenCanbox PSA Architecture (`PEUGEOT_RT4_CAR_CONFIG.md`, `peugeot_407.h`, `can_router.h`)

---

# Table of Contents
1. [Configuration Subsystem Architecture](#1-configuration-subsystem-architecture)
   - 1.1 Bidirectional Settings Pipeline
   - 1.2 Authoritative Role of CAN ID 0x361 (Exclusively BSI Personalization)
   - 1.3 BSI State Caching & NVRAM Persistence Mechanism
2. [Feature Discovery & UI Capability Masks (`Cmd 0x71` & `0x72`)](#2-feature-discovery--ui-capability-masks-cmd-0x71--0x72)
   - 2.1 CentralStateEnable1 (`Cmd 0x71`)
   - 2.2 CentralStateEnable2 (`Cmd 0x72`)
3. [Master Downlink Setting Translation (HU $\to$ CAN Box $\to$ BSI)](#3-master-downlink-setting-translation-hu--can-box--bsi)
   - 3.1 Primary Convenience Settings (`Cmd 0x7B`)
   - 3.2 Extended Personalization Settings (`Cmd 0x7D`)
   - 3.3 Climate Dual/Mono Sync Settings (`Cmd 0x3B` / `0x7D`)
   - 3.4 Regional Measurement Units & Language (`Cmd 0xCA` / `0x9A`)
4. [Master Uplink Telemetry Bitfield Layout (BSI $\to$ CAN Box $\to$ HU)](#4-master-uplink-telemetry-bitfield-layout-bsi--can-box--hu)
   - 4.1 Uplink Page 1 (`Cmd 0x76` - `CentralState1`)
   - 4.2 Uplink Page 2 (`Cmd 0x79` - `CentralState2`)
5. [PSA CAN 2004 Frame Specifications](#5-psa-can-2004-frame-specifications)
   - 5.1 BSI Personalization Menu (`0x361` - `MSG_BSI_INF_CFG_MENU_PERSO`)
   - 5.2 Driver Profile Memory (`0x260` - `MSG_BSI_INF_PROFILS`)
   - 5.3 Power Management & Backlight Dimmer (`0x036` - `MSG_COMMANDES_BSI`)
   - 5.4 Climate Dual/Mono Regulation (`0x1E3` Status & `0x1D0` Command)
6. [Pure C99 Implementation Architecture for `canbox-core`](#6-pure-c99-implementation-architecture-for-canbox-core)
   - 6.1 Data Structures (`peugeot_407.h` & `can_router.h`)
   - 6.2 BSI Frame Decoder & Encoder (`psa_407_car_config.c`)
   - 6.3 Routing Table Integration (`profile_psa.c`)
   - 6.4 Hiworld Telemetry Serializers (`hiworld_psa_builders.c`)
   - 6.5 Downlink Dispatcher (`proto_hiworld_adapter.c`)
7. [Desktop Test Harness & Verification Vectors](#7-desktop-test-harness--verification-vectors)

---

# 1. Configuration Subsystem Architecture

The **Car Settings / Vehicle Personalization** subsystem provides full bidirectional synchronization between the Android Head Unit touchscreen UI and the vehicle's Body Systems Interface (BSI / *Boîtier de Servitude Intelligent*).

```
+-----------------------------------------------------------------------------------+
|                        Android Head Unit (Hiworld / QF)                           |
|       Touchscreen UI: Car Settings, Lighting, Locks, Wipers, Radar, Sync         |
+-----------------------------------------------------------------------------------+
         | Downlink Serial (38,400 baud)                       ^ Uplink Serial
         | Cmd 0x7B (Primary Settings)                         | Cmd 0x71/0x72 (Enables)
         | Cmd 0x7D (Extended Settings)                        | Cmd 0x76 (State 1)
         | Cmd 0xCA (Units) / Cmd 0x9A (Lang)                  | Cmd 0x79 (State 2)
         v                                                     |
+-----------------------------------------------------------------------------------+
|                            OpenCanbox Core Firmware                               |
|                                                                                   |
|  [L4/L5 Downlink Adapter]                                [L4/L5 Telemetry Push]   |
|  - Validates Cmd & SettingID                              - Serializes Cmd 0x76   |
|  - Acknowledges with 0xFF                                 - Serializes Cmd 0x79   |
|  - Calls can_router_set_car_config()                      - Serializes 0x71/0x72  |
|            |                                                         ^            |
|            v                                                         |            |
|  [L3 Router & Cache]                                                 |            |
|  - Maintains s_cached_0x361_raw[6] live state                        |            |
|  - Updates vehicle_state_t.config & dirty flags ---------------------+            |
|            |                                                                      |
|            v                                                                      |
|  [L2 PSA Profile Encoder/Decoder]                                                 |
|  - Mutates specific bits in s_cached_0x361_raw                                    |
|  - Injects CAN ID 0x361 (DLC 6) or 0x1D0 (DLC 7) via HAL                          |
|  - Decodes BSI Personalization Menu frame 0x361 (DLC 6)                           |
+-----------------------------------------------------------------------------------+
         | hal_can_send(&frame)                                ^ CAN RX FIFO
         v                                                     |
+-----------------------------------------------------------------------------------+
|                         PSA CAN 2004 Comfort Bus (125 kbps)                       |
|   - BSI (0x361 DLC 6 periodic 250 ms, 0x260 Profiles, 0x036 Power/Dimmer)         |
|   - Climate ECU (0x1E3 Status periodic 100 ms, 0x1D0 Command)                     |
+-----------------------------------------------------------------------------------+
```

### 1.1 Bidirectional Settings Pipeline
1. **Discovery Handshake:** Upon CAN box connection or vehicle selection (`Cmd 0x24`), OpenCanbox transmits `Cmd 0x71` and `Cmd 0x72` feature enable bitmasks. The Android UI dynamically hides switches unsupported by the Peugeot 407 (e.g., Seat Massage, Fragrance Diffuser, Ionizer).
2. **Periodic BSI Sniffing:** The BSI broadcasts `0x361` (DLC 6) every 250 ms. OpenCanbox decodes current configuration options, updates `vehicle_state_t.config`, and stores the raw bytes in `s_cached_0x361_raw[6]`.
3. **User Interaction (Downlink):** When the driver taps a switch on the Android UI, Android emits `Cmd 0x7B` or `Cmd 0x7D` containing `[SettingID]` and `[Value]`.
4. **Command Acknowledgment:** OpenCanbox returns `5A A5 01 FF <CmdID> <Checksum>` to satisfy the Android serial watchdog.
5. **Atomic BSI CAN Injection:** OpenCanbox copies `s_cached_0x361_raw`, mutates ONLY the target bit/nibble for that setting, and injects the modified 6-byte frame `0x361` onto the CAN bus.
6. **State Echo & Sync:** Within 250 ms, the BSI accepts the change, commits it to internal NVRAM/EEPROM, and broadcasts the updated `0x361` state. OpenCanbox detects the echo, updates the cache, and transmits `Cmd 0x76` / `0x79` uplink telemetry to synchronize the UI switch.

---

### 1.2 Authoritative Role of CAN ID 0x361 (Exclusively BSI Personalization)

> [!IMPORTANT]
> In PSA CAN 2004 (Peugeot 407), live bus captures confirm that **CAN ID `0x361` is exclusively the BSI Vehicle Personalization Configuration frame** (`MSG_BSI_INF_CFG_MENU_PERSO`, DLC 6).
> 
> It is **not** used for TPMS. Tire pressures in PSA CAN 2004 are broadcasted exclusively on CAN ID **`0x1E1`** (discrete status & alarms) and optionally CAN ID **`0x3A1`** (direct numeric pressures in 0.05 Bar/LSB).
> 
> The profile router handles `0x361` directly:
> ```c
> void psa_decode_config_0x361(vehicle_config_t *cfg, const uint8_t *data, uint8_t dlc);
> ```

---

### 1.3 BSI State Caching & NVRAM Persistence Mechanism

The physical BSI is the sole authority for vehicle personalization options.
- If an external tool transmits an arbitrary `0x361` frame without preserving other bitfields, existing options (such as DRL or Follow-Me-Home delay) will be accidentally overwritten or corrupted.
- Therefore, OpenCanbox **strictly maintains a 6-byte live cache** of the latest valid BSI broadcast:
  ```c
  static uint8_t s_cached_0x361_raw[6] = { 0x00, 0x00, 0x00, 0x00, 0x01, 0x00 };
  ```
- Any downlink modification copies `s_cached_0x361_raw`, applies the bitmask change, and transmits.
- If the ignition is OFF or the BSI rejects the setting, the next cyclic BSI broadcast will overwrite `s_cached_0x361_raw` and OpenCanbox will notify Android of the true state, preventing UI desynchronization.

---

# 2. Feature Discovery & UI Capability Masks (`Cmd 0x71` & `0x72`)

During the connection handshake (and after receiving `Cmd 0x24`), OpenCanbox sends `Cmd 0x71` and `Cmd 0x72` to tell Android which configuration options exist on the Peugeot 407.

### 2.1 CentralStateEnable1 (`Cmd 0x71`, Length: 2 bytes)
- **Serial Framing:** `5A A5 02 71 [Byte 0] [Byte 1] [CS]`
- **Byte 0:**
  - `Bit 7` (`0x80`): Ambient Mood Lighting menu supported $\implies$ `1`
  - `Bit 6` (`0x40`): Welcome Light timer supported $\implies$ `0` (Not on standard 407 BSI)
  - `Bit 5` (`0x20`): Parking Radar (AAS) master toggle supported $\implies$ `1`
  - `Bit 4` (`0x10`): Reserved $\implies$ `0`
  - `Bit 3` (`0x08`): Reserved $\implies$ `0`
  - `Bit 2` (`0x04`): Daytime Running Lights (DRL) toggle supported $\implies$ `1`
  - `Bits 1..0` (`0x03`): Follow-Me-Home delay options supported $\implies$ `11`b
  - **Nominal Byte 0 Value:** `0x80 | 0x20 | 0x04 | 0x03 = 0xA7` (or `0x60` for conservative OEM)
- **Byte 1:**
  - `Bit 7` (`0x80`): Auto Rear Wiper in Reverse supported $\implies$ `1`
  - `Bit 6` (`0x40`): Parking Guideline Overlay supported $\implies$ `0` (Handled by SAS `0x11`)
  - `Bit 5` (`0x20`): Automatic Door Locking while Driving supported $\implies$ `1`
  - `Bit 4` (`0x10`): Deadlocking safety active supported $\implies$ `0`
  - `Bit 3` (`0x08`): Selective Boot / Door Unlocking supported $\implies$ `1`
  - `Bits 2..0` (`0x07`): Reserved $\implies$ `0`
  - **Nominal Byte 1 Value:** `0x80 | 0x20 | 0x08 = 0xA8` (or `0x12` for conservative OEM)

---

### 2.2 CentralStateEnable2 (`Cmd 0x72`, Length: 8 bytes)
- **Serial Framing:** `5A A5 08 72 [B0] [B1] [B2] [B3] [B4] [B5] [B6] [B7] [CS]`
- **Byte 0:**
  - `Bit 7` (`0x80`): Remote Trunk / Boot Mode $\implies$ `1`
  - `Bit 6` (`0x40`): Directional / Adaptive Xenon Headlights $\implies$ `1`
  - `Bit 5` (`0x20`): Cornering Lights $\implies$ `0`
  - `Bit 4` (`0x10`): Welcome Lights Switch $\implies$ `0`
  - `Bit 3` (`0x08`): Lane Keep Assist (LKA) $\implies$ `0`
  - `Bit 2` (`0x04`): Driver Fatigue Alert $\implies$ `0`
  - `Bit 1` (`0x02`): TPMS Calibration Reset button $\implies$ `1` (Allows TPMS re-learn)
  - `Bit 0` (`0x01`): Drive Mode Profile $\implies$ `1` (Allows Driver Profile switch)
- **Byte 1:**
  - `Bit 2` (`0x04`): Cluster Backlight Dimmer $\implies$ `1`
- **Byte 5:**
  - `Bit 1` (`0x02`): Dual/Mono Zone Temp Sync $\implies$ `1`
- **Other Bytes (B2..B4, B6..B7):** Set to `0x00` (disables unsupported modern PSA ADAS like AEB, Scent, Massage).

---

# 3. Master Downlink Setting Translation (HU $\to$ CAN Box $\to$ BSI)

When the user taps an option on Android, the headunit sends a serial command to the CAN box.

### 3.1 Primary Convenience Settings (`Cmd 0x7B`)

| Setting Name | Cmd | Setting ID (Hex) | Setting ID (Dec) | Android Payload Values | Target PSA Frame & Bitfield | Injected CAN Action |
|:---|:---:|:---:|:---:|:---|:---|:---|
| **Rear Wiper on Reverse** | `0x7B` | `0x01` | `1` | `0` = Off, `1` = On | `0x361` Byte 0 Bit 5 (`0x20`) | `data[0] = (data[0] & ~0x20) \| (val ? 0x20 : 0x00)` |
| **Selective Door Unlock** | `0x7B` | `0x04` | `4` | `0` = All Doors, `1` = Driver Only | `0x361` Byte 0 Bit 2 (`0x04`) | `data[0] = (data[0] & ~0x04) \| (val ? 0x04 : 0x00)` |
| **Daytime Running (DRL)** | `0x7B` | `0x05` | `5` | `0` = Disabled, `1` = Enabled | `0x361` Byte 0 Bit 6 (`0x40`) | `data[0] = (data[0] & ~0x40) \| (val ? 0x40 : 0x00)` |
| **Follow-Me-Home Delay** | `0x7B` | `0x06` | `6` | `0` = 0s (Off)<br>`1` = 15s<br>`2` = 30s<br>`3` = 60s | `0x361` Byte 0 Bit 7 (`0x80`) &<br>`0x361` Byte 1 Bits 3..0 | If `val == 0`:<br>`data[0] &= ~0x80; data[1] &= 0xF0;`<br>Else:<br>`data[0] \|= 0x80; data[1] = (data[1] & 0xF0) \| (val & 0x0F);` |
| **Ambient Mood Brightness** | `0x7B` | `0x0A` | `10` | `0x80 \| (brightness & 0x0F)`<br>Bit 7: Switch (1=On, 0=Off)<br>Bits 3..0: Level (0..15) | `0x361` Byte 2 Bits 3..0 | If `val & 0x80`:<br>`data[2] = (data[2] & 0xF0) \| (val & 0x0F);`<br>Else:<br>`data[2] = (data[2] & 0xF0);` |
| **Parking Radar (AAS) Toggle**| `0x7B` | `0x0B` | `11` | `0` = Muted/Off, `1` = Enabled | `0x361` Byte 4 (`0x00`/`0x01`) | `data[4] = val ? 0x01 : 0x00;` |
| **Auto Door Lock Driving** | `0x7B` | `0x0C` | `12` | `0` = Off, `1` = On ($>10\text{ km/h}$) | `0x361` Byte 0 Bit 3 (`0x08`) | `data[0] = (data[0] & ~0x08) \| (val ? 0x08 : 0x00)` |

---

### 3.2 Extended Personalization Settings (`Cmd 0x7D`)

| Setting Name | Cmd | Setting ID (Hex) | Setting ID (Dec) | Android Payload Values | Target PSA Frame & Bitfield | Injected CAN Action |
|:---|:---:|:---:|:---:|:---|:---|:---|
| **Remote Boot Mode** | `0x7D` | `0x01` | `1` | `0` = Full Car, `1` = Boot Only | `0x361` Byte 0 Bit 0 (`0x01`) | `data[0] = (data[0] & ~0x01) \| (val ? 0x01 : 0x00)` |
| **Directional Headlights** | `0x7D` | `0x02` | `2` | `0` = Off, `1` = Adaptive Xenon On | `0x361` Byte 0 Bit 1 (`0x02`) | `data[0] = (data[0] & ~0x02) \| (val ? 0x02 : 0x00)` |
| **TPMS Re-Learn Trigger** | `0x7D` | `0x03` | `3` | `1` = Pulse Trigger | `0x221` (Demands EMF) | Injects one-shot `0x221` with Byte 3 = `0x10` |
| **Driver Profile Select** | `0x7D` | `0x0B` | `11` | `0` = Profile 1, `1` = Profile 2, `2` = Profile 3 | `0x260` (`MSG_BSI_INF_PROFILS`) | Injects `0x260#000000[prof]00000000` (`prof` = `val + 1`) |
| **Backlight Dimmer** | `0x7D` | `0x0E` | `14` | `0 \dots 15` | `0x036` (Dimmer Level) | Update cluster dimmer state |
| **Zone Temp Mono/Sync** | `0x7D` | `0x24` | `36` | `0` = Dual, `1` = Mono / SYNC | `0x1D0` (`MSG_CDE_CLIMAV`) | Injects `0x1D0` with Byte 0 Bit 0 (`0x01`) |

---

### 3.3 Climate Dual/Mono Sync Settings (`Cmd 0x3B` / `0x7D`)

The Android Climate UI sends `Cmd 0x3B` with sub-setting `0x0F` to toggle Mono / SYNC mode:
- **Downlink Serial:** `5A A5 02 3B 0F [val] [CS]` (`val == 1` $\implies$ SYNC Mono, `val == 0` $\implies$ Dual).
- **Injected PSA CAN Frame (`0x1D0` - DLC 7):**
  - `Byte 0`: `0x01` (`DUAL_REQ`)
  - `Byte 5`: Driver Temperature setpoint
  - `Byte 6`: Passenger Temperature setpoint (when commanding Mono mode, Byte 6 **must strictly equal Byte 5**).

---

### 3.4 Regional Measurement Units & Language (`Cmd 0xCA` / `0x9A`)
- **Temperature Unit (`Cmd 0xCA`, Setting `0x03`):** `1` = Celsius ($^\circ\text{C}$), `2` = Fahrenheit ($^\circ\text{F}$).
- **Consumption Unit (`Cmd 0xCA`, Setting `0x05`):** `1` = L/100km, `2` = km/L, `3` = MPG (US), `4` = MPG (UK).
- **System Language (`Cmd 0x9A`, Setting `0x01`):** `0x00` = English, `0x01` = Chinese, `0x02` = French, `0x03` = German, `0x04` = Spanish, `0x05` = Italian, `0x06` = Polish, etc.

---

# 4. Master Uplink Telemetry Bitfield Layout (BSI $\to$ CAN Box $\to$ HU)

OpenCanbox broadcasts current settings to the Android Head Unit periodically ($1.0\text{ Hz}$) and immediately upon state change via two primary telemetry packets:

### 4.1 Uplink Page 1 (`Cmd 0x76` - `CentralState1`, Length: 2 bytes)
- **Header:** `5A A5 02 76 [Byte 0] [Byte 1] [CS]`
- **Byte 0:**
  - `Bit 7`: Parking Radar Auto-Active (`1` = On)
  - `Bit 6`: Parking Radar Enabled (`1` = On, mapped from `0x361` Byte 4 `!= 0`)
  - `Bits 5..4`: Welcome Light Timer (`00`=0s, `01`=15s, `10`=30s, `11`=60s)
  - `Bit 3`: Ambient Mood Light Enabled (`1` = On, mapped from `0x361` Byte 2 `> 0`)
  - `Bits 2..0`: Ambient Light Brightness scaled $0 \dots 7$ (mapped from `0x361` Byte 2 `(val & 0x0F) >> 1`)
- **Byte 1:**
  - `Bit 7`: Auto Rear Wiper in Reverse (`1` = On, mapped from `0x361` Byte 0 Bit 5)
  - `Bit 6`: Parking Assist Line Overlay (`0` = Default)
  - `Bit 5`: Automatic Door Lock while Driving (`1` = On, mapped from `0x361` Byte 0 Bit 3)
  - `Bit 4`: Deadlocking Safety Active (`0` = Default)
  - `Bit 3`: Selective Door Unlocking (`1` = Driver Only, `0` = All Doors, mapped from `0x361` Byte 0 Bit 2)
  - `Bit 2`: Daytime Running Lights (DRL) Enabled (`1` = On, mapped from `0x361` Byte 0 Bit 6)
  - `Bits 1..0`: Follow-Me-Home Headlamp Delay:
    - `00`b = 0s (Off)
    - `01`b = 15s
    - `10`b = 30s
    - `11`b = 60s
    *(Mapped from `0x361` Byte 1 Bits 3..0, qualified by Byte 0 Bit 7)*

---

### 4.2 Uplink Page 2 (`Cmd 0x79` - `CentralState2`, Length: 7 or 8 bytes)
- **Header:** `5A A5 07 79 [B0] [B1] [B2] [B3] [B4] [B5] [B6] [CS]`
- **Byte 0:**
  - `Bit 7`: Remote Trunk Only (`1` = Boot only, mapped from `0x361` Byte 0 Bit 0)
  - `Bit 6`: Adaptive / Directional Headlamps (`1` = On, mapped from `0x361` Byte 0 Bit 1)
  - `Bits 5..0`: Cornering (`b5`), Welcome (`b4`), LKA (`b3`), Fatigue (`b2`), Overspeed (`b1`), Theme (`b0`) $\implies$ `0`
- **Byte 1:**
  - `Bit 5`: Drive Mode Profile active (`0` = Normal, `1` = Active profile)
- **Byte 2:**
  - `Bits 5..2`: Cluster Dimmer Backlight level ($0 \dots 15$, mapped from `0x036` Byte 3 Bits 7..4)
  - `Bit 1`: Tailgate Motorized Status
- **Byte 3:**
  - `Bit 2`: Traction Control / ESP Status (`1` = On)
  - `Bit 1`: Power Mirror Auto-Fold (`1` = Enabled, mapped from BSI)
- **Bytes 4..5:**
  - Reserved / Modern ADAS fields (set to `0x00`)
- **Byte 6:**
  - `Bits 2..1`: Zone Temperature Mode:
    - `0x00` = Dual Independent Zones
    - `0x02` = Mono / Synchronized Passenger Zone
    *(Mapped from `0x1E3` Byte 0 Bit 0: `dual_mode ? 0x00 : 0x02`)*

---

# 5. PSA CAN 2004 Frame Specifications

### 5.1 BSI Personalization Menu (`0x361` — `MSG_BSI_INF_CFG_MENU_PERSO`)
- **CAN ID:** `0x361` (Standard 11-bit)
- **DLC:** `6` bytes
- **Period:** 250 ms (BSI cyclic broadcast) / Event-driven on modification
- **Direction:** Bi-directional (BSI $\longleftrightarrow$ OpenCanbox)

#### Byte-by-Byte Layout:
```
Byte 0:
+--------+--------+--------+--------+--------+--------+--------+--------+
| Bit 7  | Bit 6  | Bit 5  | Bit 4  | Bit 3  | Bit 2  | Bit 1  | Bit 0  |
| GUIDE  |  DRL   | WIPER  | RSVD   | AUTOLK | S_BOOT | DIR_LT | T_GATE |
+--------+--------+--------+--------+--------+--------+--------+--------+
  0x80     0x40     0x20     0x10     0x08     0x04     0x02     0x01

Byte 1:
+-----------------------------------+-----------------------------------+
| Bits 7..4: Reserved (0x0)         | Bits 3..0: GUIDE_HOME_TIME        |
+-----------------------------------+-----------------------------------+
                                      0x0 = 0s (Disabled)
                                      0x1 = 15 seconds
                                      0x2 = 30 seconds
                                      0x3 = 60 seconds

Byte 2:
+-----------------------------------+-----------------------------------+
| Bits 7..4: Reserved (0x0)         | Bits 3..0: AMBIENT_LIGHTING       |
+-----------------------------------+-----------------------------------+
                                      0x0 = Off
                                      0x1 .. 0xF = Brightness step 1..15

Byte 3: Reserved (Default 0x00; Driver Profile ID 1..3 on some BSI versions)
Byte 4: PARK_ASSIST_CFG (0x00 = AAS Disabled/Muted, 0x01 = AAS Enabled)
Byte 5: Sequence counter / status (0x00 on standard bus)
```

---

### 5.2 Driver Profile Memory (`0x260` — `MSG_BSI_INF_PROFILS`)
- **CAN ID:** `0x260`
- **DLC:** `8` bytes
- **Period:** 250 ms / Event-driven
- **Byte 3:** `ACTIVE_PROFILE_ID` (`0x01` = Driver Profile 1, `0x02` = Driver Profile 2, `0x03` = Factory Default).

---

### 5.3 Power Management & Backlight Dimmer (`0x036` — `MSG_COMMANDES_BSI`)
- **CAN ID:** `0x036`
- **DLC:** `8` bytes
- **Period:** 50 ms
- **Byte 3 Bits 7..4:** `RHEOSTAT_LEVEL` (0..15 instrument brightness level)
- **Byte 4 Bits 3..0:** `DAY_NIGHT_MODE` (0 = Night amber palette, 1 = Day high contrast, 2 = Auto)
- **Byte 5 Bit 0:** `ECONOMY_MODE` (1 = Active, 0 = Normal)

---

### 5.4 Climate Dual/Mono Regulation (`0x1E3` Status & `0x1D0` Command)
- **`0x1E3` (`MSG_ETAT_CLIMAV_EMF`, 100 ms, DLC 7):**
  - `Byte 0 Bit 0`: `DUAL_ACTIVE` (`1` = Dual independent zones, `0` = Mono / SYNC mode).
  - `Byte 2`: Driver setpoint ($14.0 + \text{raw} \times 0.5\ ^\circ\text{C}$).
  - `Byte 3`: Passenger setpoint (in Mono mode, mirrors Driver setpoint).
- **`0x1D0` (`MSG_CDE_CLIMAV`, Event, DLC 7):**
  - `Byte 0 Bit 0`: `DUAL_REQ` (`0x01` = Toggle Dual/Mono mode).
  - `Byte 5`: Driver setpoint.
  - `Byte 6`: Passenger setpoint (must equal Byte 5 when commanding Mono mode).

---

# 6. Pure C99 Implementation Architecture for `canbox-core`

To maintain strict modularity, deterministic execution, and zero heap allocations, the configuration filter is structured across layers:

### 6.1 Data Structures (`peugeot_407.h` & `can_router.h`)

In `include/profiles/peugeot_407.h`:
```c
typedef struct {
    bool    follow_me_home;         /* 0x361 B0.7 */
    uint8_t follow_me_home_sec;     /* 0, 15, 30, 60 (from 0x361 B1[3:0]) */
    bool    drl_enabled;            /* 0x361 B0.6 */
    bool    rear_wiper_reverse;     /* 0x361 B0.5 */
    bool    auto_door_lock;         /* 0x361 B0.3 */
    bool    selective_boot_unlock;  /* 0x361 B0.2 */
    bool    directional_headlights; /* 0x361 B0.1 */
    bool    motorized_tailgate;     /* 0x361 B0.0 */
    uint8_t ambient_lighting_level; /* 0x361 B2[3:0]: 0..15 */
    bool    parking_radar_enabled;  /* 0x361 B4: 0x01=Active */
    uint8_t active_profile_id;      /* 0x260 B3: 1..3 */
    uint8_t cluster_dimmer;         /* 0x036 B3[7:4]: 0..15 */
} psa_car_config_t;

typedef psa_car_config_t vehicle_config_t;
```

In `include/core/can_router.h`:
```c
/* Added to vehicle_dirty_flags_t: */
typedef struct {
    /* ... existing flags ... */
    bool config;
} vehicle_dirty_flags_t;

/* Added to vehicle_state_t: */
typedef struct {
    /* ... existing subsystems ... */
    vehicle_config_t config;
    vehicle_dirty_flags_t dirty;
} vehicle_state_t;

/* Public Router API for Downlink injection: */
bool can_router_set_car_config(uint8_t cmd_id, uint8_t setting_id, uint8_t value);
```

---

### 6.2 BSI Frame Decoder & Encoder (`src/profiles/psa_407_car_config.c`)

```c
#include "profiles/peugeot_407.h"
#include "hal/hal_can.h"
#include <string.h>

static uint8_t s_cached_0x361_raw[6] = { 0x00, 0x00, 0x00, 0x00, 0x01, 0x00 };
static bool    s_cached_0x361_valid  = false;

/* Convert 4-bit delay code to seconds */
static inline uint8_t delay_code_to_sec(uint8_t code) {
    switch (code & 0x03) {
        case 1:  return 15;
        case 2:  return 30;
        case 3:  return 60;
        default: return 0;
    }
}

/* Convert seconds to 4-bit delay code */
static inline uint8_t delay_sec_to_code(uint8_t sec) {
    if (sec >= 60) return 3;
    if (sec >= 30) return 2;
    if (sec >= 15) return 1;
    return 0;
}

void psa_decode_config_0x361(vehicle_config_t *cfg, const uint8_t *data, uint8_t dlc) {
    if (!cfg || !data || dlc < 6) return;

    memcpy(s_cached_0x361_raw, data, 6);
    s_cached_0x361_valid = true;

    cfg->follow_me_home         = (data[0] & 0x80) != 0;
    cfg->drl_enabled            = (data[0] & 0x40) != 0;
    cfg->rear_wiper_reverse     = (data[0] & 0x20) != 0;
    cfg->auto_door_lock         = (data[0] & 0x08) != 0;
    cfg->selective_boot_unlock  = (data[0] & 0x04) != 0;
    cfg->directional_headlights = (data[0] & 0x02) != 0;
    cfg->motorized_tailgate     = (data[0] & 0x01) != 0;

    cfg->follow_me_home_sec     = cfg->follow_me_home ? delay_code_to_sec(data[1] & 0x0F) : 0;
    cfg->ambient_lighting_level = data[2] & 0x0F;
    cfg->parking_radar_enabled  = (data[4] != 0x00);
}

hal_status_t psa_car_config_apply_downlink(uint8_t cmd_id, uint8_t setting_id, uint8_t val) {
    can_frame_t frame;
    memset(&frame, 0, sizeof(frame));

    if (cmd_id == 0x7B) {
        frame.id  = 0x361;
        frame.dlc = 6;
        memcpy(frame.data, s_cached_0x361_raw, 6);

        switch (setting_id) {
            case 0x01: /* Rear Wiper in Reverse */
                if (val) frame.data[0] |= 0x20;
                else     frame.data[0] &= ~0x20;
                break;
            case 0x04: /* Selective Door Unlocking */
                if (val) frame.data[0] |= 0x04;
                else     frame.data[0] &= ~0x04;
                break;
            case 0x05: /* Daytime Running Lights */
                if (val) frame.data[0] |= 0x40;
                else     frame.data[0] &= ~0x40;
                break;
            case 0x06: /* Follow-Me-Home Headlamp Delay */
                if (val == 0) {
                    frame.data[0] &= ~0x80;
                    frame.data[1] &= 0xF0;
                } else {
                    frame.data[0] |= 0x80;
                    frame.data[1]  = (frame.data[1] & 0xF0) | (val & 0x03);
                }
                break;
            case 0x0A: /* Ambient Lighting Brightness */
                if (val & 0x80) {
                    frame.data[2] = (frame.data[2] & 0xF0) | (val & 0x0F);
                } else {
                    frame.data[2] &= 0xF0;
                }
                break;
            case 0x0B: /* Parking Radar (AAS) Master Toggle */
                frame.data[4] = val ? 0x01 : 0x00;
                break;
            case 0x0C: /* Auto Door Lock while Driving */
                if (val) frame.data[0] |= 0x08;
                else     frame.data[0] &= ~0x08;
                break;
            default:
                return HAL_ERROR;
        }

        /* Update local cache optimistically and inject to BSI */
        memcpy(s_cached_0x361_raw, frame.data, 6);
        return hal_can_send(&frame);
    }
    else if (cmd_id == 0x7D) {
        if (setting_id == 0x01) { /* Remote Boot Mode */
            frame.id = 0x361;
            frame.dlc = 6;
            memcpy(frame.data, s_cached_0x361_raw, 6);
            if (val) frame.data[0] |= 0x01;
            else     frame.data[0] &= ~0x01;
            memcpy(s_cached_0x361_raw, frame.data, 6);
            return hal_can_send(&frame);
        }
        else if (setting_id == 0x02) { /* Directional Headlights */
            frame.id = 0x361;
            frame.dlc = 6;
            memcpy(frame.data, s_cached_0x361_raw, 6);
            if (val) frame.data[0] |= 0x02;
            else     frame.data[0] &= ~0x02;
            memcpy(s_cached_0x361_raw, frame.data, 6);
            return hal_can_send(&frame);
        }
        else if (setting_id == 0x03) { /* TPMS Calibration Pulse */
            frame.id = 0x221;
            frame.dlc = 8;
            frame.data[3] = 0x10;
            return hal_can_send(&frame);
        }
        else if (setting_id == 0x0B) { /* Driver Profile Select */
            frame.id = 0x260;
            frame.dlc = 8;
            frame.data[3] = (val & 0x03) + 1;
            return hal_can_send(&frame);
        }
        else if (setting_id == 0x24) { /* Zone Temp Sync (Mono/Dual) */
            frame.id = 0x1D0;
            frame.dlc = 7;
            frame.data[0] = 0x01; /* DUAL_REQ */
            return hal_can_send(&frame);
        }
    }

    return HAL_ERROR;
}
```

---

### 6.3 Routing Table Integration (`profile_psa.c`)

In `src/profiles/profile_psa.c`, route `0x361` exclusively to configuration decoder:
```c
static void psa_decode_config_0x361_profile(const can_frame_t *frame, vehicle_state_t *state) {
    if (!frame || !state || frame->dlc < 6) return;

    psa_decode_config_0x361(&state->config, frame->data, frame->dlc);
    state->dirty.config = true;
}
```
Add `{ PSA_CAN_ID_BSI_PERSO_361, psa_decode_config_0x361_profile }` to `s_psa_rules[]`.

---

### 6.4 Hiworld Telemetry Serializers (`hiworld_psa_builders.c`)

In `src/protocols/hiworld_psa_builders.c`:
```c
size_t build_hiworld_central_state1(const vehicle_config_t *cfg, uint8_t *out, size_t max_len) {
    if (!cfg || !out || max_len < 7) return 0;

    out[0] = 0x5A;
    out[1] = 0xA5;
    out[2] = 0x02; /* Length: 2 payload bytes */
    out[3] = 0x76; /* Cmd ID: CentralState1 */

    /* Byte 0 */
    uint8_t b0 = 0;
    if (cfg->parking_radar_enabled) b0 |= 0x40;
    if (cfg->ambient_lighting_level > 0) {
        b0 |= 0x08; /* Ambient light active */
        b0 |= ((cfg->ambient_lighting_level >> 1) & 0x07);
    }
    out[4] = b0;

    /* Byte 1 */
    uint8_t b1 = 0;
    if (cfg->rear_wiper_reverse)    b1 |= 0x80;
    if (cfg->auto_door_lock)        b1 |= 0x20;
    if (cfg->selective_boot_unlock) b1 |= 0x08;
    if (cfg->drl_enabled)           b1 |= 0x04;
    if (cfg->follow_me_home) {
        if (cfg->follow_me_home_sec >= 60)      b1 |= 0x03;
        else if (cfg->follow_me_home_sec >= 30) b1 |= 0x02;
        else if (cfg->follow_me_home_sec >= 15) b1 |= 0x01;
    }
    out[5] = b1;

    /* Checksum */
    uint8_t sum = out[2] + out[3] + out[4] + out[5];
    out[6] = (uint8_t)((sum - 1) & 0xFF);

    return 7;
}

size_t build_hiworld_central_state2(const vehicle_config_t *cfg, const vehicle_state_t *st, uint8_t *out, size_t max_len) {
    if (!cfg || !st || !out || max_len < 12) return 0;

    out[0] = 0x5A;
    out[1] = 0xA5;
    out[2] = 0x07; /* Length: 7 payload bytes */
    out[3] = 0x79; /* Cmd ID: CentralState2 */

    /* Byte 0 */
    uint8_t b0 = 0;
    if (cfg->motorized_tailgate)     b0 |= 0x80;
    if (cfg->directional_headlights) b0 |= 0x40;
    out[4] = b0;

    /* Byte 1: Drive profile */
    out[5] = (cfg->active_profile_id > 1) ? 0x20 : 0x00;

    /* Byte 2: Cluster dimmer */
    out[6] = (cfg->cluster_dimmer & 0x0F) << 2;

    /* Byte 3: ESP Traction */
    out[7] = 0x04; /* ESP active by default */

    /* Bytes 4 & 5: Unused ADAS */
    out[8] = 0x00;
    out[9] = 0x00;

    /* Byte 6: Climate Dual/Mono Sync (0x02=Mono, 0x00=Dual) */
    out[10] = (!st->climate.dual_mode) ? 0x02 : 0x00;

    /* Checksum */
    uint8_t sum = 0;
    for (size_t i = 2; i < 11; i++) sum += out[i];
    out[11] = (uint8_t)((sum - 1) & 0xFF);

    return 12;
}
```

---

### 6.5 Downlink Dispatcher (`proto_hiworld_adapter.c`)

In `src/protocols/proto_hiworld_adapter.c`, add handler in `on_hiworld_packet_received`:
```c
if (packet->cmd == HIWORLD_CMD_CENTRAL_SETTING_SET1 || /* 0x7B */
    packet->cmd == HIWORLD_CMD_CENTRAL_SETTING_SET2)   /* 0x7D */
{
    if (packet->payload_len >= 2) {
        uint8_t setting_id = packet->payload[0];
        uint8_t val        = packet->payload[1];
        can_router_set_car_config(packet->cmd, setting_id, val);
    }
}
```

---

# 7. Desktop Test Harness & Verification Vectors

The following automated test vectors verify end-to-end translation between Hiworld downlink serial packets and PSA CAN 2004 frames in `test/test_protocol_parser/`:

### Vector 1: Headunit Configures Follow-Me-Home Headlamps to 30 Seconds
- **Downlink Serial Received (Host $\to$ CAN Box):**
  ```
  Hex Trace: 5A A5 02 7B 06 02 84
  ```
  - Length: `0x02`, Cmd: `0x7B`, Setting ID: `0x06` (`FollowMeHome`), Value: `0x02` (30s)
  - Checksum: `(0x02 + 0x7B + 0x06 + 0x02 - 1) & 0xFF = 0x84`
- **Expected PSA BSI CAN Frame Injected:**
  - `ID: 0x361, DLC: 6`
  - `Payload: 80 02 00 00 01 00` (`Byte 0: 0x80`, `Byte 1: 0x02` = 30s)

---

### Vector 2: Headunit Enables Daytime Running Lights (DRL = ON)
- **Downlink Serial Received:**
  ```
  Hex Trace: 5A A5 02 7B 05 01 82
  ```
  - Cmd: `0x7B`, Setting ID: `0x05` (`DRL`), Value: `0x01` (ON)
  - Checksum: `(0x02 + 0x7B + 0x05 + 0x01 - 1) & 0xFF = 0x82`
- **Expected PSA BSI CAN Frame Injected:**
  - `ID: 0x361, DLC: 6`
  - `Payload: 40 00 00 00 01 00` (`Byte 0: 0x40` = DRL ON)

---

### Vector 3: Headunit Enables Rear Wiper in Reverse Gear
- **Downlink Serial Received:**
  ```
  Hex Trace: 5A A5 02 7B 01 01 7E
  ```
  - Cmd: `0x7B`, Setting ID: `0x01`, Value: `0x01` (ON)
  - Checksum: `(0x02 + 0x7B + 0x01 + 0x01 - 1) & 0xFF = 0x7E`
- **Expected PSA BSI CAN Frame Injected:**
  - `ID: 0x361, DLC: 6`
  - `Payload: 20 00 00 00 01 00` (`Byte 0: 0x20` = Rear Wiper ON)

---

### Vector 4: BSI Broadcasts Configuration State $\to$ CAN Box Uplinks Telemetry
- **CAN Frame Received from BSI:**
  - `ID: 0x361, DLC: 6, Data: E8 02 0A 00 01 00`
  - Decoded State:
    - Guide-Me-Home: `Active (30s)` (`0x80` in B0, `0x02` in B1)
    - DRL: `Active` (`0x40` in B0)
    - Rear Wiper: `Active` (`0x20` in B0)
    - Auto Door Lock: `Active` (`0x08` in B0)
    - Ambient Lighting: `Level 10` (`0x0A` in B2)
    - Parking Radar: `Active` (`0x01` in B4)
- **Expected Uplink Serial Transmitted (`Cmd 0x76`):**
  - `5A A5 02 76 4D A6 6A`
    - `Byte 0: 0x4D` (`0x40` Radar Active | `0x08` Ambient Active | `0x05` Scaled Brightness 10/2)
    - `Byte 1: 0xA6` (`0x80` Rear Wiper | `0x20` Auto Lock | `0x04` DRL | `0x02` Follow 30s)
    - Checksum: `(0x02 + 0x76 + 0x4D + 0xA6 - 1) & 0xFF = 0x6A`

