# Hiworld (WC / 0x5AA5) Vehicle Central Configuration Specification
## Master Catalog: Headunit Car Settings, Personalization, Lighting Options & BSI Overrides
### PSA CAN 2004 (Peugeot 407 / AEE2004 Comfort Bus) $\longleftrightarrow$ Hiworld Android Head Unit Bridge

**Document File:** `CANBOX_SPEC_HIWORLD_407_11_CAR_CONFIGURATION_OPTIONS.md`  
**Document Version:** 2.1.0 (Bench-Verified against EMF-C MDF & Hiworld PA12.20 `H1H2PA123A-240717`)  
**Target Platform:** Pure C99 Embedded Firmware (`canbox-core` on STM32 / ESP32 / NUC131)  
**Vehicle Network:** Peugeot 407 (PSA CAN 2004 / AEE2004 Comfort Bus @ 125 kbps)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, 38,400 baud 8N1, `((len + cmd + sum) - 1) & 0xFF` checksum)  
**Standard References:**
1. Magneti Marelli RT4 (NaviDrive SW 8.31) Firmware Specification (`RT4_AI_CAN_SETTINGS_GUIDE.md`)
2. Reverse-Engineered Android Automotive Core (`com.qf.vehicle.band.peugeot.parse.wc`, `CentralState.java`)
3. OpenCanbox PSA Architecture (`PEUGEOT_RT4_CAR_CONFIG.md`, `peugeot_407.h`, `can_router.h`)
4. Hardware-in-the-Loop Bench Captures (`dump_2026-10-22_14-24-48.log`, `only_mdf_can_log_min2.csv`)

---

# Table of Contents
1. [Configuration Subsystem Architecture](#1-configuration-subsystem-architecture)
   - 1.1 Bidirectional Settings Pipeline
   - 1.2 Menu Availability vs Active Settings (`0x361` vs `0x260` / `0x15B`)
   - 1.3 BSI State Caching, Event-Driven `0x15B` & MDF Coexistence
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
   - 5.1 BSI Personalization Menu Availability Mask (`0x361` - `MSG_BSI_INF_CFG_MENU_PERSO`)
   - 5.2 Driver Profile Memory & Active Settings (`0x260` - `MSG_BSI_INF_PROFILS`)
   - 5.3 Downlink Personalization Modification Request (`0x15B` - `MSG_BSI_PERSO_REQ`)
   - 5.4 Power Management & Backlight Dimmer (`0x036` - `MSG_COMMANDES_BSI`)
   - 5.5 Climate Dual/Mono Regulation (`0x1E3` Status & `0x1D0` Command)
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
| Cmd 0x7D (Extended Settings)                        | Cmd 0x76 (State 1, 2B)
| Cmd 0xCA (Units) / Cmd 0x9A (Lang)                  | Cmd 0x79 (State 2, 8B)
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
|  - Maintains s_cached_profile1_raw[8] & s_cached_0x361_raw[6]         |            |
|  - Updates vehicle_state_t.config & dirty flags ---------------------+            |
|            |                                                                      |
|            v                                                                      |
|  [L2 PSA Profile Encoder/Decoder]                                                 |
|  - Mutates target bits in s_cached_profile1_raw                                    |
|  - Injects event-driven CAN ID 0x15B (DLC 8) or 0x1D0 (DLC 7) via HAL             |
|  - Decodes BSI Profiles frame 0x260 (DLC 8) -> vehicle_config_t                    |
|  - Decodes BSI Menu Availability frame 0x361 (DLC 6) -> feature enables (0x71/72) |
+-----------------------------------------------------------------------------------+
| hal_can_send(&frame)                                ^ CAN RX FIFO
v                                                     |
+-----------------------------------------------------------------------------------+
|                         PSA CAN 2004 Comfort Bus (125 kbps)                       |
|   - BSI (0x361 Equipment Mask, 0x260 Profiles Status, 0x036 Power/Dimmer)         |
|   - Telematic Screen / MDF / CAN Box (0x15B Event-Driven Write Request DLC 8)     |
|   - Climate ECU (0x1E3 Status periodic 100 ms, 0x1D0 Command)                     |
+-----------------------------------------------------------------------------------+

```

### 1.1 Bidirectional Settings Pipeline
1. **Discovery Handshake:** Upon CAN box connection or vehicle selection (`Cmd 0x24`), OpenCanbox transmits `Cmd 0x71` and `Cmd 0x72` feature enable bitmasks. The Android UI dynamically hides switches unsupported by the Peugeot 407 (e.g., Seat Massage, Fragrance Diffuser, Ionizer).
2. **Periodic BSI Sniffing:**
   - The BSI broadcasts `0x361` (DLC 6) equipment availability mask, indicating which personalization menus are telecoded in the vehicle.
   - The BSI cycles driver profiles (1, 2, 3) every 500 ms on `0x260` (DLC 8), so Profile 1 (`data[0] == 0x01`) repeats every 1,500 ms. OpenCanbox decodes active settings from Profile 1, updates `vehicle_state_t.config`, and stores the raw bytes in `s_cached_profile1_raw[8]`.
3. **User Interaction (Downlink):** When the driver taps a switch on the Android UI, Android emits `Cmd 0x7B` or `Cmd 0x7D` containing `[SettingID]` and `[Value]`.
4. **Command Acknowledgment:** OpenCanbox returns `5A A5 01 FF <CmdID> <Checksum>` to satisfy the Android serial watchdog.
5. **Atomic BSI CAN Injection:** OpenCanbox copies `s_cached_profile1_raw`, mutates the target bit for that setting, sets `data[0] = 0x01` (validity bit 2 cleared = pending configuration write request), and injects a single 8-byte frame `0x15B` onto the CAN bus.
6. **State Echo & Sync:** On the next Profile 1 slot of `0x260` (within 500–1500 ms), the BSI commits the change to internal NVRAM/EEPROM and broadcasts the updated `0x260` Profile 1 state. OpenCanbox detects the echo, updates the cache, and transmits `Cmd 0x76` / `0x79` uplink telemetry to synchronize the UI switch.

---

### 1.2 Menu Availability vs Active Settings (`0x361` vs `0x260` / `0x15B`)

> [!IMPORTANT]
> In PSA CAN 2004 (Peugeot 407), there is a strict architectural division:
> - **CAN ID `0x361` (`MSG_BSI_INF_CFG_MENU_PERSO`, DLC 6)** is **Menu Availability (Equipment Mask)**, NOT Current User Settings. It tells displays (EMF-C/RT4) which menus are physically telecoded in the BSI:
>   - Byte 0: `profile_number`
>   - Byte 2: `auto_lighting` (b0), `follow_me_home` (b4), `hinge_panel` (b7)
>   - Byte 3: `drl_present` (b6)
>   - Byte 4: `emf_present` (b4), `aas_disable` (b5)
>   - Byte 5: `tpms_present` (b4..b6)
> - **CAN ID `0x260` (`MSG_BSI_INF_PROFILS`, DLC 8)** broadcasts **Active User Settings**:
>   - The BSI cycles profiles (1, 2, 3) on `0x260` every 500 ms. Profile 1 (`data[0] == 0x01`) contains the true active user settings (DRL, Rear Wiper, Follow-Me-Home delay, Automatic Headlights, Parking Radar AAS toggle, Directional Headlights).
> - **CAN ID `0x15B` (`MSG_BSI_PERSO_REQ`, DLC 8)** is the **Event-Driven Downlink BSI Configuration Frame**:
>   - In PSA CAN 2004, displays command personalization changes to the BSI on CAN ID `0x15B` (DLC 8, with `Byte 0 = 0x01` and validity bit 2 cleared, indicating a pending configuration write request for Profile 1). The BSI does not accept configuration changes written into `0x361`.

---

### 1.3 BSI State Caching, Periodic `0x15B` & Ground Truth Synchronization

Hardware bench captures (`mdf_trace.log`, `dump_mdf_stalk_trip.log`) and live BSI ablation testing on `can0` prove the exact operational model between the factory PSA Multifunction Display (MDF / EMF-C), BSI, and aftermarket CAN box:
- **OEM EMF-C MDF Behavior & `0x167` 10 Hz Presence Gate:**
  - The physical EMF-C display broadcasts **`0x167` (`TRAME_GENERALE_EMF`, DLC 8, nominal `08 06 FF FF 00 00 00 00`) continuously at 100 ms (10 Hz)** and **`0x15B` (`MSG_BSI_PERSO_REQ`, DLC 8) continuously at 500 ms (2.0 Hz)**.
  - **Critical BSI Gate:** The Peugeot 407 BSI monitors `0x167` with a ~500 ms supervision timeout. If `0x167` is absent or transmitted only at 1000 ms (1 Hz), the BSI marks the Multifunction Display as offline and **silently ignores all `0x15B` configuration write requests**. Streaming `0x167` at **100 ms (10 Hz)** alongside `0x15B` (`Byte 0 = 0x01`) at **500 ms (2 Hz)** is necessary and sufficient for the BSI to commit changes to EEPROM.
  - **`0x15B` Byte 0 Bit 2 (`0x04` $\to$ `0x05`) is an Init/Invalidation Flag, NOT a Write Strobe:** At cold power-on before receiving `0x260`, EMF-C emits `0x15B#0500000000000000` (`Byte 0 = 0x05`) to tell the BSI not to overwrite EEPROM with uninitialized RAM. Once synchronized, valid personalization writes always use `Byte 0 = 0x01` (Profile 1, Bit 2 cleared); sending `Byte 0 = 0x05` is ignored by the BSI.
  - **`0x260` 3-Profile Multiplexing (1.5 s Cycle):** The BSI broadcasts `0x260` every 500 ms, cycling round-robin through Profile 1 (`01 03 ...`), Profile 2 (`02 03 ...`), and Profile 3 (`03 03 ...`). Thus, Profile 1 (`data[0] == 0x01`) updates once every **1500 ms (1.5 s)**.
- **OpenCanbox Virtual MDF & Coexistence Architecture:**
  - When no physical MDF is present on the bus (3000 ms timeout on `0x525`), OpenCanbox enters **Virtual MDF mode** and broadcasts the required display supervision frames:
    - `0x167` (`DLC 8`: `08 06 FF FF 00 00 00 00` at **100 ms / 10 Hz** - MFD presence watchdog & active trip page supervisory frame)
    - `0x15B` (`DLC 8`: `01 03 ...` at 500 ms - Profile 1 personalization supervisory demand)
    - `0x0DF` (`DLC 3`: `10 00 50` at 500 ms - display status)
    - `0x1ED` (`DLC 1`: climate sync at 500 ms)
    - `0x525` (`DLC 8`: `05 00 01 00 00 00 00 00` at 1000 ms - presence / diagnostic heartbeat)
    - `0x3F6` (`DLC 7`: `00 00 <cnt> 00 00 C0 01` at 1000 ms - cluster trip supervision)
  - When a physical MDF is connected and broadcasts `0x525`, OpenCanbox suppresses all Virtual MDF frames (`0x15B`, `0x0DF`, `0x1ED`, `0x525`, `0x3F6`, `0x167`) and ISO-TP Flow Control within 100 ms.
- **Ground Truth & Rejection Handling:**
  - BSI `0x260` Profile 1 is the vehicle ground truth.
  - When Android HU sends a downlink (`Cmd 0x7B` / `0x7D`), OpenCanbox immediately transmits `0x15B` and sustains it every 500 ms (with `0x167` at 100 ms) for up to 3.0s (`s_downlink_pending_ticks = 30`, covering two full 1.5s Profile 1 `0x260` cycles).
  - If BSI accepts: BSI updates `0x260` Profile 1 within the next 1.5s cycle (~500–1500 ms), latching is confirmed, and HU receives confirmed settings via `Cmd 0x76` & `0x79`.
  - If BSI rejects: after 3.0s without BSI confirmation, the pending write times out. OpenCanbox reverts its desired state back to the confirmed BSI `0x260` state and pushes the real BSI state to Android HU via `Cmd 0x76` & `0x79`, guaranteeing that the HU UI never remains stuck showing invalid/uncommitted settings.

---

# 2. Feature Discovery & UI Capability Masks (`Cmd 0x71` & `0x72`)

During the connection handshake (after receiving `Cmd 0x24`), OpenCanbox sends `Cmd 0x71` and `Cmd 0x72` to tell Android which configuration options exist on the Peugeot 407.

> [!NOTE]
> **OEM PA12.20 Reference vs Dynamic `0x361` Discovery:**  
> At initial `Cmd 0x24` handshake, the OEM Hiworld PA12.20 (`H1H2PA123A-240717`) transmits conservative static defaults:
> - `Cmd 0x71`: `5A A5 02 71 60 12 E4` (`0x60, 0x12`)
> - `Cmd 0x72`: `5A A5 08 72 00 01 80 00 00 00 00 00 FA` (`0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00`)  
> `canbox-core` initializes with these exact defaults and dynamically expands them when `0x361` arrives from the BSI.

### 2.1 CentralStateEnable1 (`Cmd 0x71`, Length: 2 bytes)
- **Serial Framing:** `5A A5 02 71 [Byte 0] [Byte 1] [CS]`
- **Byte 0:**
  - `Bit 7` (`0x80`): Ambient Mood Lighting menu supported $\implies$ `1`
  - `Bit 6` (`0x40`): Welcome Light timer supported $\implies$ `0` (Set in static PA12.20 default `0x60`)
  - `Bit 5` (`0x20`): Parking Radar (AAS) master toggle supported $\implies$ `1` (if `0x361` B4.5 `0x20` set)
  - `Bit 4` (`0x10`): Reserved $\implies$ `0`
  - `Bit 3` (`0x08`): Reserved $\implies$ `0`
  - `Bit 2` (`0x04`): Daytime Running Lights (DRL) toggle supported $\implies$ `1` (if `0x361` B3.6 `0x40` set)
  - `Bits 1..0` (`0x03`): Follow-Me-Home delay options supported $\implies$ `11`b (if `0x361` B2.4 `0x10` set)
  - **Dynamic Byte 0 Value (for `0x361 = 01 01 91 40 30 10`):** `0x80 | 0x20 | 0x04 | 0x03 = 0xA7` (Init default: `0x60`)
- **Byte 1:**
  - `Bit 7` (`0x80`): Auto Rear Wiper in Reverse supported $\implies$ `1`
  - `Bit 6` (`0x40`): Parking Guideline Overlay supported $\implies$ `0` (Handled by SAS `0x11`)
  - `Bit 5` (`0x20`): Automatic Door Locking while Driving supported $\implies$ `1`
  - `Bit 4` (`0x10`): Deadlocking safety active supported $\implies$ `0`
  - `Bit 3` (`0x08`): Selective Boot / Door Unlocking supported $\implies$ `1`
  - `Bits 2..0` (`0x07`): Reserved $\implies$ `0`
  - **Dynamic Byte 1 Value:** `0x80 | 0x20 | 0x08 = 0xA8` (Init default: `0x12`)

---

### 2.2 CentralStateEnable2 (`Cmd 0x72`, Length: 8 bytes)
- **Serial Framing:** `5A A5 08 72 [B0] [B1] [B2] [B3] [B4] [B5] [B6] [B7] [CS]`
- **Byte 0:**
  - `Bit 7` (`0x80`): Remote Trunk / Boot Mode $\implies$ `1` (if `0x361` B2.7 `0x80` set)
  - `Bit 6` (`0x40`): Directional / Adaptive Xenon Headlights $\implies$ `1` (if `0x361` B2.0 `0x01` set)
  - `Bit 5` (`0x20`): Cornering Lights $\implies$ `0`
  - `Bit 4` (`0x10`): Welcome Lights Switch $\implies$ `0`
  - `Bit 3` (`0x08`): Lane Keep Assist (LKA) $\implies$ `0`
  - `Bit 2` (`0x04`): Driver Fatigue Alert $\implies$ `0`
  - `Bit 1` (`0x02`): TPMS Calibration Reset button $\implies$ `1` (if `0x361` B5[6:4] `0x70` set)
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
| **Rear Wiper on Reverse** | `0x7B` | `0x01` | `1` | `0` = Off, `1` = On | `0x15B` Byte 5 Bit 4 (`0x10`) | `data[5] = (data[5] & ~0x10) \| (val ? 0x10 : 0x00)` |
| **Selective Door Unlock** | `0x7B` | `0x04` | `4` | `0` = All Doors, `1` = Driver Only | `0x15B` Byte 4 Bit 2 (`0x04`) | `data[4] = (data[4] & ~0x04) \| (val ? 0x04 : 0x00)` |
| **Daytime Running (DRL)** | `0x7B` | `0x05` | `5` | `0` = Disabled, `1` = Enabled | `0x15B` Byte 3 Bit 6 (`0x40`) | `data[3] = (data[3] & ~0x40) \| (val ? 0x40 : 0x00)` |
| **Follow-Me-Home Delay** | `0x7B` | `0x06` | `6` | `0` = 0s (Off)<br>`1` = 15s<br>`2` = 30s<br>`3` = 60s | `0x15B` Byte 2 Bit 5 (`0x20`) &<br>`0x15B` Byte 2 Bits 2..0 | `data[2] &= ~0x27;`<br>If `val == 1`: `data[2] \|= (0x20 \| 0x01);`<br>Else if `val == 2`: `data[2] \|= (0x20 \| 0x02);`<br>Else if `val == 3`: `data[2] \|= (0x20 \| 0x04);`<br>Else: `data[2] \|= 0x01;` |
| **Ambient Mood Brightness** | `0x7B` | `0x0A` | `10` | `0x80 \| (brightness & 0x0F)`<br>Bit 7: Switch (1=On, 0=Off)<br>Bits 3..0: Level (0..15) | `0x15B` Byte 2 Bits 3..0 | If `val & 0x80`:<br>`data[2] = (data[2] & 0xF0) \| (val & 0x0F);`<br>Else:<br>`data[2] = (data[2] & 0xF0);` |
| **Parking Radar (AAS) Toggle**| `0x7B` | `0x0B` | `11` | `0` = Muted/Off, `1` = Enabled | `0x15B` Byte 5 Bit 6 (`0x40`) | `data[5] = (data[5] & ~0x40) \| (val ? 0x40 : 0x00)` |
| **Auto Door Lock Driving** | `0x7B` | `0x0C` | `12` | `0` = Off, `1` = On ($>10\text{ km/h}$) | `0x15B` Byte 4 Bit 3 (`0x08`) | `data[4] = (data[4] & ~0x08) \| (val ? 0x08 : 0x00)` |

---

### 3.2 Extended Personalization Settings (`Cmd 0x7D`)

| Setting Name | Cmd | Setting ID (Hex) | Setting ID (Dec) | Android Payload Values | Target PSA Frame & Bitfield | Injected CAN Action |
|:---|:---:|:---:|:---:|:---|:---|:---|
| **Remote Boot Mode** | `0x7D` | `0x01` | `1` | `0` = Full Car, `1` = Boot Only | `0x15B` Byte 5 Bit 7 (`0x80`) | `data[5] = (data[5] & ~0x80) \| (val ? 0x80 : 0x00)` |
| **Directional Headlights** | `0x7D` | `0x02` | `2` | `0` = Off, `1` = Adaptive Xenon On | `0x15B` Byte 2 Bit 7 (`0x80`) | `data[2] = (data[2] & ~0x80) \| (val ? 0x80 : 0x00)` |
| **TPMS Re-Learn Trigger** | `0x7D` | `0x03` | `3` | `1` = Pulse Trigger | `0x221` (Demands EMF) | Injects one-shot `0x221` with Byte 3 = `0x10` |
| **Driver Profile Select** | `0x7D` | `0x0B` | `11` | `0` = Profile 1, `1` = Profile 2, `2` = Profile 3 | `0x15B` (`MSG_BSI_PERSO_REQ`) | Injects `0x15B#0[prof]...` (`prof` = `(val & 0x03) + 1`) |
| **Backlight Dimmer** | `0x7D` | `0x0E` | `14` | `0 \dots 15` | `0x036` (Dimmer Level) | Update cluster dimmer state |
| **Zone Temp Mono/Sync** | `0x7D` | `0x24` | `36` | `0` = Dual, `1` = Mono / SYNC | `0x1ED` (`DEMANDES_AFFICHEUR_CLIM`) | Updates `0x1ED` Byte 0 Bit 4 (`0x10`) |

---

### 3.3 Climate Dual/Mono Sync & A/C Settings (`Cmd 0x3B` / `0x7D`)

The Android Climate UI sends `Cmd 0x3B` (or `Cmd 0x7D`) to control Mono / Dual and A/C compressor cut-off:
- **Downlink Serial (Mono/Dual):** `5A A5 02 3B 0F [val] [CS]` or `5A A5 02 7D 24 [val] [CS]` (`val == 1` $\implies$ SYNC Mono, `val == 0` $\implies$ Dual).
- **Downlink Serial (A/C Switch):** `5A A5 02 3B 02 [val] [CS]` (`val == 1` $\implies$ A/C ON, `val == 0` $\implies$ A/C OFF / Eco).
- **Injected PSA CAN Frame (`0x1ED` - DLC 1, Periodic 500 ms + Event-Driven):**
  - Controlled by Multifunction Display / Telematics unit (`DEMANDES_AFFICHEUR_CLIM`).
  - **Bit 4 (`0x10`):** `1` = Dual Mode, `0` = Mono Mode.
  - **Bit 3 (`0x08`):** `0` = A/C Compressor ON, `1` = A/C Compressor OFF (Eco).
  - Prerequisites: `0x0DF` (`10 00 50`) must be actively broadcast every 500 ms for Climate ECU to accept `0x1ED`. Note that `0x1D0` is emitted by the Climate panel and is NOT used for A/C or Mono/Dual injection.

---

### 3.4 Regional Measurement Units & Language (`Cmd 0xCA` / `0x9A`)
- **Temperature Unit (`Cmd 0xCA`, Setting `0x03`):** `1` = Celsius ($^\circ\text{C}$), `2` = Fahrenheit ($^\circ\text{F}$).
- **Consumption Unit (`Cmd 0xCA`, Setting `0x05`):** `1` = L/100km, `2` = km/L, `3` = MPG (US), `4` = MPG (UK).
- **System Language (`Cmd 0x9A`, Setting `0x01`):** `0x00` = English, `0x01` = Chinese, `0x02` = French, `0x03` = German, `0x04` = Spanish, `0x05` = Italian, `0x06` = Polish, etc.

---

# 4. Master Uplink Telemetry Bitfield Layout (BSI $\to$ CAN Box $\to$ HU)

OpenCanbox broadcasts current settings to the Android Head Unit immediately upon state change and during periodic bulk resync via two primary telemetry packets:

### 4.1 Uplink Page 1 (`Cmd 0x76` - `CentralState1`, Length: 2 bytes)
- **Header:** `5A A5 02 76 [Byte 0] [Byte 1] [CS]`
  *(Verified against Hiworld PA12.20 bench captures: `5A A5 02 76 50 85 4C` when ON, `5A A5 02 76 50 80 47` when Follow-Me-Home & DRL are OFF)*
- **Byte 0:**
  - `Bit 7` (`0x80`): Parking Radar Auto-Active (`1` = On)
  - `Bit 6` (`0x40`): Parking Radar Enabled (`1` = On, mapped from `0x260` Byte 5 Bit 6 `0x40`)
  - `Bit 5` (`0x20`): Welcome Light Status (`0` on standard 407)
  - `Bit 4` (`0x10`): Automatic Headlights / Auto-Lighting Enabled (`1` = On, mapped from `0x260` Byte 2 Bit 4 `0x10`)
  - `Bit 3` (`0x08`): Ambient Mood Light Enabled (`1` = On)
  - `Bits 2..0` (`0x07`): Ambient Light Brightness scaled $0 \dots 7$
- **Byte 1:**
  - `Bit 7` (`0x80`): Auto Rear Wiper in Reverse (`1` = On, mapped from `0x260` Byte 5 Bit 4 `0x10`)
  - `Bit 6` (`0x40`): Parking Assist Line Overlay (`0` = Default)
  - `Bit 5` (`0x20`): Automatic Door Lock while Driving (`1` = On, mapped from `0x260` Byte 4 Bit 3 `0x08`)
  - `Bit 4` (`0x10`): Deadlocking Safety Active (`0` = Default)
  - `Bit 3` (`0x08`): Selective Door Unlocking (`1` = Driver Only, `0` = All Doors, mapped from `0x260` Byte 4 Bit 2 `0x04`)
  - `Bit 2` (`0x04`): Daytime Running Lights (DRL) Enabled (`1` = On, mapped from `0x260` Byte 3 Bit 6 `0x40`)
  - `Bits 1..0` (`0x03`): Follow-Me-Home Headlamp Delay:
    - `00`b = 0s (Off)
    - `01`b = 15s
    - `10`b = 30s
    - `11`b = 60s
    *(Mapped from `0x260` Byte 2 Bits 1..0, or Bits 3..2 when EMF-C MDF encodes `0xB4`, qualified by Byte 2 Bit 5 `0x20`)*

---

### 4.2 Uplink Page 2 (`Cmd 0x79` - `CentralState2`, Length: 8 bytes)
- **Header:** `5A A5 08 79 [B0] [B1] [B2] [B3] [B4] [B5] [B6] [B7] [CS]`
  *(Verified against Hiworld PA12.20 bench capture: `5A A5 08 79 00 00 3C 10 00 00 00 00 CC`)*
- **Byte 0:**
  - `Bit 7` (`0x80`): Remote Trunk Only (`1` = Boot only, mapped from `0x260` Byte 5 Bit 7 `0x80`)
  - `Bit 6` (`0x40`): Adaptive / Directional Headlamps (`1` = On, mapped from `0x260` Byte 2 Bit 7 `0x80`)
  - `Bits 5..0`: Cornering (`b5`), Welcome (`b4`), LKA (`b3`), Fatigue (`b2`), Overspeed (`b1`), Theme (`b0`) $\implies$ `0`
- **Byte 1:**
  - `Bit 5` (`0x20`): Drive Mode Profile active (`0` = Profile 1 / Normal, `1` = Profile > 1)
- **Byte 2:**
  - `Bits 5..2` (`0x3C`): Cluster Dimmer Backlight level ($0 \dots 15 \ll 2$, mapped from `0x036` Byte 3 **Bits 3..0** `data[3] & 0x0F`; e.g., `0x0F << 2 = 0x3C`)
  - `Bit 1`: Tailgate Motorized Status
- **Byte 3:**
  - `Bit 4` (`0x10`): Door / Window / ESP status bit on OEM PA12.20 (`0x10`)
  - `Bit 2` (`0x04`): Traction Control / ESP Status (`1` = On)
  - `Bit 1` (`0x02`): Power Mirror Auto-Fold (`1` = Enabled, mapped from BSI)
- **Bytes 4..5:**
  - Reserved / Modern ADAS fields (set to `0x00`)
- **Byte 6:**
  - `Bits 2..1`: Zone Temperature Mode:
    - `0x00` = Dual Independent Zones
    - `0x02` = Mono / Synchronized Passenger Zone
    *(Mapped from `0x1E3` Byte 0 Bit 0: `dual_mode ? 0x00 : 0x02`)*
- **Byte 7:**
  - Reserved (`0x00`)

---

# 5. PSA CAN 2004 Frame Specifications

### 5.1 BSI Personalization Menu Availability Mask (`0x361` — `MSG_BSI_INF_CFG_MENU_PERSO`)
- **CAN ID:** `0x361` (Standard 11-bit)
- **DLC:** `6` bytes
- **Period:** 500 ms / 1000 ms (BSI cyclic broadcast)
- **Direction:** BSI $\longrightarrow$ Displays / CAN Box
- **Purpose:** Advertises physically telecoded menu availability (feature enables for `Cmd 0x71` & `Cmd 0x72`). Does NOT convey active user settings.

#### Byte-by-Byte Layout (Bench Reference: `01 01 91 40 30 10`):

```

Byte 0: Profile ID (0x01 on standard bench)
Byte 1: Reserved / status (0x01)
Byte 2 (0x91):

* Bit 7 (0x80): hinge_panel (Remote trunk / boot mode present)
* Bit 4 (0x10): follow_me_home (Follow-Me-Home delay menu present)
* Bit 0 (0x01): auto_lighting (Automatic / adaptive headlights present)
Byte 3 (0x40):
* Bit 6 (0x40): drl_present (Daytime Running Lights menu present)
Byte 4 (0x30):
* Bit 5 (0x20): aas_disable (Parking radar AAS toggle menu present)
* Bit 4 (0x10): emf_present (EMF display present)
Byte 5 (0x10):
* Bits 6..4 (0x70): tpms_present (Tire pressure monitoring system present)

```

---

### 5.2 Driver Profile Memory & Active Settings (`0x260` — `MSG_BSI_INF_PROFILS`)
- **CAN ID:** `0x260`
- **DLC:** `8` bytes
- **Period:** 500 ms per slot (BSI cycles Profile 1, Profile 2, Profile 3 $\implies$ 1,500 ms full cycle)
- **Direction:** BSI $\longrightarrow$ Displays / CAN Box
- **Purpose:** Carries active driver personalization settings. Profile 1 (`data[0] == 0x01`) is decoded into `vehicle_config_t`.

#### Byte-by-Byte Layout (Profile 1: `01 03 B1 40 00 D0 00 10` / `01 03 B4 40 00 D0 00 10`):

```

Byte 0: Profile ID (0x01 = Profile 1, 0x02 = Profile 2, 0x03 = Profile 3)
Byte 1: Sub-status (0x03)
Byte 2 (0xB1 / 0xB4 when ON; 0x91 / 0x94 when Follow-Me-Home OFF):

* Bit 7 (0x80): Adaptive / Directional Headlights (1 = ON, 0 = OFF)
* Bit 5 (0x20): Follow-Me-Home Headlamps Master Enable (1 = ON, 0 = OFF)
* Bit 4 (0x10): Automatic Headlights / Auto-Lighting (1 = ON, 0 = OFF) -> Maps to Cmd 0x76 B0.4 (0x10)
* Bits 3..2 (0x0C): Follow-Me-Home Duration Nibble High (used by EMF-C MDF, e.g., 0xB4 when ON / 0x94 when OFF)
* Bits 1..0 (0x03): Follow-Me-Home Delay (00=0s, 01=15s, 10=30s, 11=60s)
Byte 3 (0x40 when ON; 0x00 when OFF):
* Bit 6 (0x40): Daytime Running Lights DRL (1 = ON, 0 = OFF)
Byte 4 (0x00):
* Bit 3 (0x08): Auto Door Lock while Driving (1 = ON)
* Bit 2 (0x04): Selective Door Unlocking (1 = Driver Only)
Byte 5 (0xD0):
* Bit 7 (0x80): Remote Boot Mode / Hinge Panel (1 = Boot Only)
* Bit 6 (0x40): Parking Radar AAS (1 = ON, 0 = Muted)
* Bit 4 (0x10): Rear Wiper in Reverse Gear (1 = ON)
Byte 6: Reserved (0x00)
Byte 7: Profile options mask (0x10)

```

---

### 5.3 Downlink Personalization Modification Request (`0x15B` — `MSG_BSI_PERSO_REQ`)
- **CAN ID:** `0x15B`
- **DLC:** `8` bytes
- **Transmission Mode:** **Event-Driven One-Shot** (transmitted once upon user confirmation in MDF menu or Android Head Unit `Cmd 0x7B`/`0x7D` downlink)
- **Direction:** Displays (EMF-C / RT4) or CAN Box $\longrightarrow$ BSI
- **Byte 0:** `0x01` (Profile 1 with Bit 2 = 0 cleared, requesting BSI to commit the pending profile settings to NVRAM/EEPROM)
- **Bytes 1..7:** Mirrors `0x260` layout with modified setting bits injected into the active profile.

---

### 5.4 Power Management & Backlight Dimmer (`0x036` — `MSG_COMMANDES_BSI`)
- **CAN ID:** `0x036`
- **DLC:** `8` bytes
- **Period:** 50 ms / 100 ms
- **Bench Reference Frame:** `0E 00 00 0F 01 00 00 A0`
- **Byte 1 Bit 7 (`0x80`):** Reverse gear status flag
- **Byte 1 Bit 0 (`0x01`):** Handbrake engaged flag
- **Byte 2 Bit 7 (`0x80`):** `ECONOMY_MODE` (`1` = Active, `0` = Normal)
- **Byte 3 Bit 5 (`0x20`):** `ILLUMINATION_NIGHT` (`1` = Night illumination active)
- **Byte 3 Bits 3..0 (`0x0F`):** `RHEOSTAT_LEVEL` (`0..15` instrument cluster dimmer brightness level; e.g., `0x0F` = 15 $\implies$ `0x0F << 2 = 0x3C` in `Cmd 0x79` Byte 2)
- **Byte 4 Bits 7..4 (`0xF0`):** `POWER_MODE` (Contact state):
  - `0x00`: Contact OFF / Standby
  - `0x10`: Accessory (ACC)
  - `0x20`: Ignition ON (+APC)
  - `0x50`: Engine Running
- **Byte 4 Bits 2..0 (`0x07`):** `PHASE_VIE` (Comfort bus network lifecycle):
  - `0x00`: Deep Sleep / Standby
  - `0x01`: Normal / Bus Awake (+APC active)
  - `0x02`: Shutdown countdown / Preparing sleep
  - `0x03`: Wakeup transition
> [!IMPORTANT]
> **Power Mode Masking Rule:** Never apply an artificial 3-bit mask (`data[4] & 0x07`) to determine ignition. Doing so strips the upper nibble `POWER_MODE` (`0x20` or `0x50`), causing ignition state to read as OFF whenever `PHASE_VIE` transitions away from `0x01` during engine running (`0x50`). Byte 4 must be evaluated as `(data[4] >= 0x20) || (data[4] & 0x07) == 0x01`.

---

### 5.5 Climate Dual/Mono & A/C Regulation (`0x1E3` Status & `0x1ED` / `0x0DF` Command)
- **`0x1E3` (`MSG_ETAT_CLIMAV_EMF`, 200 ms, DLC 7 from Climate ECU):**
  - `Byte 0 Bit 6`: `AC_OFF_STATUS` (`0` = A/C ON, `1` = A/C OFF / Eco).
  - `Byte 0 Bit 0`: `DUAL_ACTIVE` (`1` = Dual independent zones, `0` = Mono / SYNC mode).
  - `Byte 2`: Driver setpoint ($14.0 + \text{raw} \times 0.5\ ^\circ\text{C}$).
  - `Byte 3`: Passenger setpoint (in Mono mode, mirrors Driver setpoint).
- **`0x0DF` (`ETAT_EMF`, 500 ms, DLC 3):**
  - Payload: `10 00 50` (MDF display presence heartbeat; mandatory prerequisite for Climate ECU to accept `0x1ED`).
- **`0x1ED` (`DEMANDES_AFFICHEUR_CLIM`, 500 ms + Event, DLC 1):**
  - `Byte 0 Bit 4`: `DUAL_MODE` (`1` = Dual, `0` = Mono).
  - `Byte 0 Bit 3`: `AC_OFF_REQ` (`0` = A/C ON, `1` = A/C OFF / Eco).
  - Four valid payloads: `0x00` (A/C ON, Mono), `0x08` (A/C OFF, Mono), `0x10` (A/C ON, Dual), `0x18` (A/C OFF, Dual). Note: `0x1D0` is broadcast by the climate console and is NOT injected for MDF A/C or Mono toggles.

---

# 6. Pure C99 Implementation Architecture for `canbox-core`

To maintain strict modularity, deterministic execution, and zero heap allocations, the configuration filter is structured across layers:

### 6.1 Data Structures (`peugeot_407.h` & `can_router.h`)

In `include/profiles/peugeot_407.h`:
```c
typedef struct {
    bool    follow_me_home;         /* 0x260 B2.5 (0x20) */
    uint8_t follow_me_home_sec;     /* 0, 15, 30, 60 (from 0x260 B2[1:0] or B2[3:2]) */
    bool    auto_lighting_enabled;  /* 0x260 B2.4 (0x10) -> Cmd 0x76 B0.4 (0x10) */
    bool    drl_enabled;            /* 0x260 B3.6 (0x40) */
    bool    rear_wiper_reverse;     /* 0x260 B5.4 (0x10) */
    bool    auto_door_lock;         /* 0x260 B4.3 (0x08) */
    bool    selective_boot_unlock;  /* 0x260 B4.2 (0x04) */
    bool    directional_headlights; /* 0x260 B2.7 (0x80) */
    bool    motorized_tailgate;     /* 0x260 B5.7 (0x80) */
    uint8_t ambient_lighting_level; /* Ambient lighting level 0..15 */
    bool    parking_radar_enabled;  /* 0x260 B5.6 (0x40) */
    uint8_t active_profile_id;      /* 0x260 B0: 1..3 */
    uint8_t cluster_dimmer;         /* 0x036 B3[3:0]: 0..15 */
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

static uint8_t s_cached_0x361_raw[6] = { 0x01, 0x01, 0x91, 0x40, 0x30, 0x10 };
static bool    s_cached_0x361_valid  = false;

/* Live cache of active Profile 1 status frame (0x260 / 0x15B) */
static uint8_t s_cached_profile1_raw[8] = { 0x01, 0x03, 0xB1, 0x40, 0x00, 0xD0, 0x00, 0x10 };
static bool    s_cached_profile1_valid  = false;

/* Feature enables (Cmd 0x71 and Cmd 0x72) telecoded in BSI */
static uint8_t s_feature_enable_1[2] = { 0x60, 0x12 };
static uint8_t s_feature_enable_2[8] = { 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* Convert Byte 2 delay code to seconds:
 * Bit 2 (0x04) = 60s
 * Bit 1 (0x02) = 30s
 * Bit 0 (0x01) = 15s
 * Also supports sequential 0x03 = 60s. Default minimum is 15s. */
static inline uint8_t delay_code_to_sec(uint8_t byte2) {
    if (byte2 & 0x04) return 60;
    if ((byte2 & 0x03) == 3) return 60;
    if (byte2 & 0x02) return 30;
    if (byte2 & 0x01) return 15;
    return 15;
}

void psa_decode_config_0x361(const uint8_t *data, uint8_t dlc) {
    if (!data || dlc < 6) return;
    memcpy(s_cached_0x361_raw, data, 6);
    s_cached_0x361_valid = true;

    /* Populate feature enables (Cmd 0x71 & 0x72) from equipment mask */
    /* ... */
}

void psa_decode_config_0x260(vehicle_config_t *cfg, const uint8_t *data, uint8_t dlc) {
    if (!cfg || !data || dlc < 8) return;
    if (data[0] != 0x01) return; /* Profile 1 is the active user settings frame */

    memcpy(s_cached_profile1_raw, data, 8);
    s_cached_profile1_valid = true;

    cfg->active_profile_id      = 1;
    cfg->directional_headlights = (data[2] & 0x80) != 0;
    cfg->follow_me_home         = (data[2] & 0x20) != 0;
    cfg->follow_me_home_sec     = cfg->follow_me_home ? delay_code_to_sec(data[2]) : 0;
    cfg->auto_lighting_enabled  = (data[2] & 0x10) != 0;
    cfg->drl_enabled            = (data[3] & 0x40) != 0;
    cfg->auto_door_lock         = (data[4] & 0x08) != 0;
    cfg->selective_boot_unlock  = (data[4] & 0x04) != 0;
    cfg->motorized_tailgate     = (data[5] & 0x80) != 0;
    cfg->parking_radar_enabled  = (data[5] & 0x40) != 0;
    cfg->rear_wiper_reverse     = (data[5] & 0x10) != 0;
}

hal_status_t psa_car_config_apply_downlink(uint8_t cmd_id, uint8_t setting_id, uint8_t val) {
    can_frame_t frame;
    memset(&frame, 0, sizeof(frame));

    if (cmd_id == 0x7B) {
        frame.id  = PSA_CAN_ID_BSI_PERSO_REQ_15B; /* 0x15B */
        frame.dlc = 8;
        memcpy(frame.data, s_cached_profile1_raw, 8);
        frame.data[0] = 0x01; /* Profile 1 with Bit 2 = 0 cleared */

        /* Mutate target bits in frame.data ... */
        memcpy(s_cached_profile1_raw, frame.data, 8);
        s_cached_profile1_valid = true;
        return hal_can_send(&frame);
    }
    /* ... */
    return HAL_STATUS_ERROR;
}

```

---

### 6.3 Routing Table Integration (`profile_psa.c`)

In `src/profiles/profile_psa.c`:

* Extract the lower nibble (`frame->data[3] & 0x0F`) in `psa_decode_ignition_reverse_0x036` for `cluster_dimmer`.
* Route `0x361` to update feature enable masks (`Cmd 0x71` & `Cmd 0x72`).
* Route `0x260` to decode Profile 1 active configuration into `vehicle_config_t`.

```c
/* Inside psa_decode_ignition_reverse_0x036: */
    if (frame->dlc >= 4) {
        state->lights.illumination = (frame->data[3] & 0x20) != 0;
        state->lights.brightness   = (frame->data[3] & 0x0F);
        uint8_t dimmer = (frame->data[3] & 0x0F); /* Lower nibble of 0x036 Byte 3 (e.g. 0x0F -> 0x3C in 0x79) */
        if (state->config.cluster_dimmer != dimmer) {
            state->config.cluster_dimmer = dimmer;
            state->dirty.config = true;
        }
    }

static void psa_decode_config_0x361_profile(const can_frame_t *frame, vehicle_state_t *state) {
    (void)state;
    if (!frame || frame->dlc < 6) return;

    uint8_t old_feat1[2], old_feat2[8];
    psa_car_config_get_feature_enables(old_feat1, old_feat2);

    psa_decode_config_0x361(frame->data, frame->dlc);

    uint8_t new_feat1[2], new_feat2[8];
    psa_car_config_get_feature_enables(new_feat1, new_feat2);

    if (memcmp(old_feat1, new_feat1, 2) != 0 || memcmp(old_feat2, new_feat2, 8) != 0) {
        hu_protocol_send_feature_enables(new_feat1, new_feat2);
    }
}

static void psa_decode_bsi_profiles_0x260_profile(const can_frame_t *frame, vehicle_state_t *state) {
    if (!frame || !state || frame->dlc < 8) return;

    psa_decode_config_0x260(&state->config, frame->data, frame->dlc);
    state->dirty.config = true;
}

```

Add `{ PSA_CAN_ID_BSI_PERSO_361, psa_decode_config_0x361_profile }` and `{ PSA_CAN_ID_BSI_PROFILES_260, psa_decode_bsi_profiles_0x260_profile }` to `s_psa_rules[]`.

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
    if (cfg->auto_lighting_enabled) b0 |= 0x10; /* Automatic headlights (0x260 B2.4) -> 0x50 with radar */
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
        else                                    b1 |= 0x01;
    }
    out[5] = b1;

    /* Checksum */
    uint8_t sum = (uint8_t)(out[2] + out[3] + out[4] + out[5]);
    out[6] = (uint8_t)((sum - 1) & 0xFF);

    return 7;
}

size_t build_hiworld_central_state2(const vehicle_config_t *cfg, const vehicle_state_t *st, uint8_t *out, size_t max_len) {
    if (!cfg || !out || max_len < 13) return 0;

    out[0] = 0x5A;
    out[1] = 0xA5;
    out[2] = 0x08; /* Length: 8 payload bytes (matches OEM PA12.20) */
    out[3] = 0x79; /* Cmd ID: CentralState2 */

    /* Byte 0 */
    uint8_t b0 = 0;
    if (cfg->motorized_tailgate)     b0 |= 0x80;
    if (cfg->directional_headlights) b0 |= 0x40;
    out[4] = b0;

    /* Byte 1: Drive profile */
    out[5] = (cfg->active_profile_id > 1) ? 0x20 : 0x00;

    /* Byte 2: Cluster dimmer (0x036 B3[3:0] << 2, e.g. 0x0F << 2 = 0x3C) */
    out[6] = (uint8_t)((cfg->cluster_dimmer & 0x0F) << 2);

    /* Byte 3: ESP Traction */
    out[7] = 0x04; /* ESP active by default */

    /* Bytes 4 & 5: Unused ADAS */
    out[8] = 0x00;
    out[9] = 0x00;

    /* Byte 6: Climate Dual/Mono Sync (0x02=Mono, 0x00=Dual) */
    out[10] = (st && !st->climate.dual_mode) ? 0x02 : 0x00;

    /* Byte 7: Reserved */
    out[11] = 0x00;

    /* Checksum */
    uint8_t sum = 0;
    for (size_t i = 2; i < 12; i++) sum = (uint8_t)(sum + out[i]);
    out[12] = (uint8_t)((sum - 1) & 0xFF);

    return 13;
}

```

---

### 6.5 Downlink Dispatcher (`proto_hiworld_adapter.c`)

In `src/protocols/proto_hiworld_adapter.c`, handle `0x7B`, `0x7D`, and `0x3B` in `on_hiworld_packet_received`:

```c
if (packet->cmd == HIWORLD_CMD_CENTRAL_SETTING1 || /* 0x7B */
    packet->cmd == HIWORLD_CMD_CENTRAL_SETTING2 || /* 0x7D */
    packet->cmd == HIWORLD_CMD_AC_SETTING_SET)     /* 0x3B */
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

* **Downlink Serial Received (Host $\to$ CAN Box):**
```
Hex Trace: 5A A5 02 7B 06 02 84

```


* Length: `0x02`, Cmd: `0x7B`, Setting ID: `0x06` (`FollowMeHome`), Value: `0x02` (30s)
* Checksum: `(0x02 + 0x7B + 0x06 + 0x02 - 1) & 0xFF = 0x84`


* **Expected PSA BSI CAN Frame Injected:**
* `ID: 0x15B, DLC: 8`
* `Payload: 01 03 [B2] 40 00 D0 00 10` (`Byte 0: 0x01` Profile 1 with write req, `Byte 2: 0x20 | 0x02` = 30s)



---

### Vector 2: Headunit Enables Daytime Running Lights (DRL = ON)

* **Downlink Serial Received:**
```
Hex Trace: 5A A5 02 7B 05 01 82

```


* Cmd: `0x7B`, Setting ID: `0x05` (`DRL`), Value: `0x01` (ON)
* Checksum: `(0x02 + 0x7B + 0x05 + 0x01 - 1) & 0xFF = 0x82`


* **Expected PSA BSI CAN Frame Injected:**
* `ID: 0x15B, DLC: 8`
* `Payload: 01 03 B1 40 00 D0 00 10` (`Byte 0: 0x01`, `Byte 3: 0x40` = DRL ON)



---

### Vector 3: Headunit Enables Rear Wiper in Reverse Gear

* **Downlink Serial Received:**
```
Hex Trace: 5A A5 02 7B 01 01 7E

```


* Cmd: `0x7B`, Setting ID: `0x01`, Value: `0x01` (ON)
* Checksum: `(0x02 + 0x7B + 0x01 + 0x01 - 1) & 0xFF = 0x7E`


* **Expected PSA BSI CAN Frame Injected:**
* `ID: 0x15B, DLC: 8`
* `Payload: 01 03 B1 40 00 D0 00 10` (`Byte 0: 0x01`, `Byte 5: bit 4 (0x10)` = Rear Wiper ON)



---

### Vector 4: BSI Broadcasts Configuration State $\to$ CAN Box Uplinks Telemetry

* **CAN Frame Received from BSI (`0x260` Active Settings):**
* **State A (Guide-Me-Home & DRL ON):** `ID: 0x260, DLC: 8, Data: 01 03 B1 40 00 D0 00 10` (or `01 03 B4 40 00 D0 00 10` from EMF-C MDF)
* **State B (Guide-Me-Home & DRL OFF):** `ID: 0x260, DLC: 8, Data: 01 03 91 00 00 D0 00 10` (or `01 03 94 00 00 D0 00 10` from EMF-C MDF)
* **Decoded State (State A):**
* Adaptive Lighting: `Active` (`0x80` in B2)
* Follow-Me-Home: `Active (15s)` (`0x20` in B2, `0x01` in B2[1:0] or `0x04` in B2[3:2])
* Automatic Headlights: `Active` (`0x10` in B2)
* DRL: `Active` (`0x40` in B3)
* Parking Radar AAS: `Active` (`0x40` in B5)
* Rear Wiper in Reverse: `Active` (`0x10` in B5)




* **Expected Uplink Serial Transmitted (`Cmd 0x76` — matches PA12.20 `ORIG_CANBOX`):**
* **State A (ON):** `5A A5 02 76 50 85 4C`
* `Byte 0: 0x50` (`0x40` Radar Active | `0x10` Auto Headlights Active)
* `Byte 1: 0x85` (`0x80` Rear Wiper | `0x04` DRL | `0x01` Follow-Me-Home 15s)
* Checksum: `(0x02 + 0x76 + 0x50 + 0x85 - 1) & 0xFF = 0x4C`


* **State B (OFF):** `5A A5 02 76 50 80 47`
* `Byte 0: 0x50` (`0x40` Radar Active | `0x10` Auto Headlights Active)
* `Byte 1: 0x80` (`0x80` Rear Wiper | DRL OFF | Follow-Me-Home OFF)
* Checksum: `(0x02 + 0x76 + 0x50 + 0x80 - 1) & 0xFF = 0x47`




* **CAN Frame Received from BSI (`0x361` Menu Availability Mask):**
* `ID: 0x361, DLC: 6, Data: 01 01 91 40 30 10`
* **Feature Enables Computed:**
* Static Init (PA12.20 Baseline): `Cmd 0x71 = 60 12`, `Cmd 0x72 = 00 01 80 00 00 00 00 00`
* Dynamic `0x361` Expansion: `Cmd 0x71 = A7 A8`, `Cmd 0x72 = C3 04 00 00 00 02 00 00`

