# Hiworld (WC / 0x5AA5) Vehicle Central Configuration Specification
## Master Catalog: Headunit Car Settings, Personalization, Lighting Options & BSI Overrides

**Document File:** `CANBOX_SPEC_HIWORLD_407_11_CAR_CONFIGURATION_OPTIONS.md`  
**Document Version:** 1.0.0  
**Target Platform:** Automotive Android Infotainment SoC $\longleftrightarrow$ Pure C99 CAN Box Firmware  
**Vehicle Network:** Peugeot 407 & PSA Platform (CAN 2004 / PSA 15 Comfort & Body CAN)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Standard Reference:** Decompiled Android Automotive Core (`com.qf.vehicle.band.peugeot.parse.wc`, `CentralState.java`)

---

# Table of Contents
1. [Configuration Subsystem Architecture](#1-configuration-subsystem-architecture)
   - 1.1 Bidirectional Settings Pipeline
   - 1.2 Downlink Command Structure (`Cmd 0x7B`, `0x7D`, `0xCA`, `0x9A`)
   - 1.3 Uplink State Feedback (`Cmd 0x76`, `0x79`, `0xC1`)
   - 1.4 Feature Discovery & Availability Masks (`Cmd 0x71`, `0x72`)
2. [Exhaustive Catalog of Vehicle Configuration Options](#2-exhaustive-catalog-of-vehicle-configuration-options)
   - 2.1 Domain 1: Lighting & Visibility (Follow-Me-Home, Welcome, DRL, Ambient)
   - 2.2 Domain 2: Locking, Security & Access (Auto-Lock, Selective Unlock, Boot, Mirrors)
   - 2.3 Domain 3: Wipers & Driving Convenience (Reverse Wipe, Rain Sensor, S&S, TPMS)
   - 2.4 Domain 4: ADAS & Driver Assistance (Blind Spot, Fatigue, LDW, TSR, AEB, ESP)
   - 2.5 Domain 5: Interior Comfort, Wellness & Seats (Massage, Fragrance, Ionizer, Sync)
   - 2.6 Domain 6: Regional Units & System Language
3. [Master Downlink Setting ID Translation Table](#3-master-downlink-setting-id-translation-table)
4. [Master Uplink Telemetry Bitfield Layout](#4-master-uplink-telemetry-bitfield-layout)
5. [Pure C99 Configuration Engine Implementation](#5-pure-c99-configuration-engine-implementation)
   - 5.1 Configuration Data Model (`hiworld_car_config.h`)
   - 5.2 Downlink Parser & Uplink Serializer (`hiworld_car_config.c`)
6. [Desktop Test Harness & Verification Vectors](#6-desktop-test-harness--verification-vectors)

---

# 1. Configuration Subsystem Architecture

When the user interacts with the **Car Settings / Vehicle Personalization** app on the Android touchscreen, the headunit sends discrete configuration downlink frames to the CAN box adapter. The CAN box decodes the setting index, validates parameters, maps them to native PSA BSI CAN frames (`0x221`, `0x228`, `0x1E0`, `0x1A1`), and broadcasts updated state telemetry back to Android.

```
+------------------------------------------------------------------------------------+
|                         Android Headunit "Car Settings" UI                         |
|     [Follow-Me-Home: 30s | DRL: ON | Auto-Lock: ON | Mirror Fold: ON | Units: C]   |
+------------------------------------------------------------------------------------+
                         │ (Downlink Cmd 0x7B / 0x7D) ▲ (Uplink Feedback Cmd 0x76/0x79)
                         ▼                            │
+------------------------------------------------------------------------------------+
|                         Hiworld CAN Box Microcontroller                            |
|   1. Decodes Sub-Setting Index & Param Value (hiworld_car_config_parser)           |
|   2. Updates internal `hiworld_car_config_t` state model                           |
|   3. Synthesizes PSA BSI Configuration Frame (CAN ID 0x221 / 0x228)                |
|   4. Broadcasts Feature Availability Masks (Cmd 0x71 / 0x72)                       |
+------------------------------------------------------------------------------------+
                         │ (PSA CAN Bus: 125 kbps)    ▲ (BSI Status CAN 0x221/0x0F6)
                         ▼                            │
+------------------------------------------------------------------------------------+
|                         Peugeot 407 Built-in Systems Interface                     |
|           [Executes Relays, Headlamp Timers, Actuators, Mirror Solenoids]          |
+------------------------------------------------------------------------------------+
```

### 1.1 Bidirectional Settings Pipeline
1. **Discovery / Capability Handshake:** CAN box emits `Cmd 0x71` & `Cmd 0x72` feature masks; Android hides unsupported switches.
2. **User Tap:** Android emits `Cmd 0x7B` or `Cmd 0x7D` with `[SettingID]` and `[Value]`.
3. **BSI Injection:** CAN box injects the corresponding CAN bits into PSA CAN ID `0x221`.
4. **State Feedback:** CAN box reads vehicle confirmation and replies with `Cmd 0x76` / `0x79` to synchronize the UI toggle switch.

---

# 2. Exhaustive Catalog of Vehicle Configuration Options

---

### 2.1 Domain 1: Lighting & Visibility

#### 1. Follow-Me-Home Headlamp Delay (`WithMeGoHomeTime` / Walk-Home Headlights)
Keeps dipped low-beam headlights illuminated after locking the vehicle to guide passengers in darkness.
- **Downlink Framing:** `Cmd 0x7B` (`123` dec), `SettingID = 0x06`
- **Available Options:**
  - `0`: **Disabled / 0 Seconds** (Headlights extinguish immediately)
  - `1`: **15 Seconds** delay
  - `2`: **30 Seconds** delay (Default standard)
  - `3`: **60 Seconds** delay
- **PSA CAN Mapping (`0x221` Byte 4):** Bits 7..6: `00`=0s, `01`=15s, `10`=30s, `11`=60s.

#### 2. Welcome / Greeting Lighting Duration (`WelcomeLightTime`)
Turns on sidelights, puddle lamps, and interior courtesy lights upon remote key unlock.
- **Downlink Framing:** `Cmd 0x7B`, `SettingID = 0x09`
- **Available Options:**
  - `0`: **Disabled / 0 Seconds**
  - `1`: **15 Seconds**
  - `2`: **30 Seconds**
  - `3`: **60 Seconds**

#### 3. Welcome Lighting Master Toggle (`WelcomeLights_Cb`)
- **Downlink Framing:** `Cmd 0x7D` (`125` dec), `SettingID = 0x05`
- **Available Options:**
  - `0`: **OFF** (Welcome lights disabled)
  - `1`: **ON** (Welcome lights enabled)

#### 4. Daytime Running Lights (`DayRunCarLight_Cb` / DRL)
Controls the dedicated front daytime running lamps while the engine is running.
- **Downlink Framing:** `Cmd 0x7B`, `SettingID = 0x05`
- **Available Options:**
  - `0`: **Disabled / OFF**
  - `1`: **Enabled / ON**
- **PSA CAN Mapping (`0x221` Byte 2):** Bit 7 (`0x80`).

#### 5. Adaptive Directional Headlights / Cornering Lights (`TurnBigLight_Cb` / `TurnAssistance_Cb`)
Swivels headlamps into turns or illuminates fog lights at low speeds when turning the steering wheel.
- **Downlink Framing:** `Cmd 0x7D`, `SettingID = 0x02` (Directional Headlights) / `SettingID = 0x04` (Cornering Assistance)
- **Available Options:** `0` = Off, `1` = On.
- **PSA CAN Mapping (`0x221` Byte 2):** Bit 0 (`0x01`).

#### 6. Automatic High Beam Assist (`AutoHighLight_Cb`)
Automatically switches between high beam and low beam based on oncoming traffic sensors.
- **Downlink Framing:** `Cmd 0x7D`, `SettingID = 0x17` (`23` dec)
- **Available Options:** `0` = Off, `1` = On.

#### 7. Headlamp Extinguishing Delay (`DelayedExtinguishingDoorClose_List` / `EngineCloseDelay_List`)
Delay timer after closing doors or turning off ignition.
- **Downlink Framing:** `Cmd 0x7D`, `SettingID = 0x1E` (`30` dec) / `SettingID = 0x20` (`32` dec)
- **Available Options:** `0` = 0s, `1` = 15s, `2` = 30s, `3` = 60s.

#### 8. Ambient Mood Lighting Master Switch & Brightness (`AtmosphereSwitch_Cb`, `AtmosphereLightBrightness_List`)
Interior footwell and center console LED ambient lighting.
- **Downlink Framing:** `Cmd 0x7B`, `SettingID = 0x0A` (`10` dec)
- **Payload Layout:** `[Bit 7: Switch (1=ON, 0=OFF)] | [Bits 3..0: Brightness Level (0..15)]`
- **Available Options:** Switch (`ON`/`OFF`), Brightness (`0` = Off to `15` = Maximum).

#### 9. Ambient Mood Mode / Theme Color (`Atmosphere_mode_List` / `ThemeColor_List`)
- **Downlink Framing:** `Cmd 0x7D`, `SettingID = 0x09` / `SettingID = 0x0A`
- **Available Options:**
  - Mode: `0` = Normal, `1` = Relax / Soft, `2` = Dynamic / Boost.
  - Theme: `0` = Blue / Cyan, `1` = Amber / Red, `2` = White / Silver, `3` = Green.

#### 10. Instrument Cluster Backlight Brightness (`MeterBackLight_List`)
- **Downlink Framing:** `Cmd 0x7D`, `SettingID = 0x0E`
- **Available Options:** `0 \dots 15` (Discrete brightness levels).

---

### 2.2 Domain 2: Locking, Security & Access

#### 1. Automatic Driving Door Lock (`DoorAutoLock_Cb` / Drive-Lock)
Automatically locks all doors and tailgate once vehicle speed exceeds $10\text{ km/h}$ ($6\text{ mph}$).
- **Downlink Framing:** `Cmd 0x7B`, `SettingID = 0x0C` (`12` dec)
- **Available Options:**
  - `0`: **OFF** (Doors remain unlocked while driving)
  - `1`: **ON** (Doors lock automatically above 10 km/h)
- **PSA CAN Mapping (`0x221` Byte 1):** Bit 4 (`0x10`).

#### 2. Selective Door Unlocking (`UnlockDoorWhenRemoteOrManulEnter_List`)
Controls whether the first remote unlock press unlocks all doors or driver's door only.
- **Downlink Framing:** `Cmd 0x7B`, `SettingID = 0x04`
- **Available Options:**
  - `0`: **All Doors** unlocked simultaneously
  - `1`: **Driver's Door Only** unlocked (Second press unlocks remaining doors)

#### 3. Remote Boot Opening Mode (`OnlyRemoteOpenTrunk_Cb`)
- **Downlink Framing:** `Cmd 0x7D`, `SettingID = 0x01`
- **Available Options:**
  - `0`: Unlocks trunk and all doors
  - `1`: Unlocks and releases trunk lid only (keeps cabin doors locked)

#### 4. Automatic Power Mirror Folding on Lock (`DisableAutoFoldingRearviewMirror_Cb` / `RearMirrorAutoFolder_Cb`)
Automatically folds side mirrors against the body when locking via remote key fob.
- **Downlink Framing:** `Cmd 0x7D`, `SettingID = 0x1A` (`26` dec) / `SettingID = 0x23` (`35` dec)
- **Available Options:**
  - `0`: **Disabled** (Mirrors stay unfolded)
  - `1`: **Enabled** (Mirrors fold on lock, unfold on ignition)
- **PSA CAN Mapping (`0x221` Byte 4):** Bit 3 (`0x08`).

#### 5. Hands-Free Keyless Boot / Kick Sensor (`TrunkNokeyUnlock_Cb` / `TrunkAutoOpen_Cb`)
Motorized tailgate hands-free bumper foot sensor.
- **Downlink Framing:** `Cmd 0x7D`, `SettingID = 0x15` (`21` dec) / `SettingID = 0x16` (`22` dec)
- **Available Options:** `0` = Disabled, `1` = Enabled.

---

### 2.3 Domain 3: Wipers & Driving Convenience

#### 1. Automatic Rear Wiper in Reverse Gear (`AutoRearWiperWhenReverse_Cb`)
Activates the rear window wiper automatically when reverse gear is engaged while front wipers are operating.
- **Downlink Framing:** `Cmd 0x7B`, `SettingID = 0x01`
- **Available Options:**
  - `0`: **Disabled / OFF**
  - `1`: **Enabled / ON**
- **PSA CAN Mapping (`0x221` Byte 1):** Bit 7 (`0x80`).

#### 2. Automatic Rain-Sensing Wipers (`AutomaticWiper_Cb`)
- **Downlink Framing:** `Cmd 0x7B`, `SettingID = 0x0D`
- **Available Options:** `0` = Manual intermittent, `1` = Automatic rain sensor.

#### 3. Tire Pressure Calibration Trigger (`Tire_PressureCalibration_Cb` / TPMS Re-learn)
Stores current tire pressures as the new reference baseline in the BSI.
- **Downlink Framing:** `Cmd 0x7D`, `SettingID = 0x03`
- **Action:** One-shot pulse (`Value = 1`). CAN Box injects `CAN ID 0x221#0000001000000000`.

#### 4. Stop & Start (S&S) System Master Override (`IntelligenceStartStop_Cb`)
- **Downlink Framing:** `Cmd 0x7D`, `SettingID = 0x01` with `Cmd 0x8C` (`-116` signed)
- **Available Options:**
  - `0`: **Deactivated / Inhibited** (Engine will not auto-stop)
  - `1`: **Active / Ready** (Standard eco auto-stop enabled)

#### 5. Drive Mode Profile (`DriveMode_List`)
- **Downlink Framing:** `Cmd 0x7D`, `SettingID = 0x0B` (`11` dec)
- **Available Options:** `0` = Normal / Standard, `1` = Eco Mode, `2` = Sport Mode, `3` = Snow / Off-road.

---

### 2.4 Domain 4: ADAS & Driver Assistance Systems

#### 1. Parking Radar Acoustic & Visual Master Switch (`RadarSwitch_Cb` / `Parking_AutoActive_Cb`)
- **Downlink Framing:** `Cmd 0x7B`, `SettingID = 0x0B` (Switch) / `Cmd 0x7D`, `SettingID = 0x08` (Auto Active)
- **Available Options:** `0` = Parking radar disabled/muted, `1` = Enabled.

#### 2. Blind Spot Monitoring System (`BlindWarning_Cb` / SAM)
Ultrasonic/radar rear quarter blind spot indicator lamps in side mirrors.
- **Downlink Framing:** `Cmd 0x7D`, `SettingID = 0x02`
- **Available Options:** `0` = Off, `1` = On.

#### 3. Driver Fatigue Alert (`FatigueDrivingWarning_Cb`)
Evaluates driving duration ($>2\text{ hours}$) and steering micro-corrections to display coffee cup break prompt.
- **Downlink Framing:** `Cmd 0x7D`, `SettingID = 0x07`
- **Available Options:** `0` = Off, `1` = On.

#### 4. Lane Departure Warning & Lane Keep (`LaneKeepAssistance_Cb` / LDW)
- **Downlink Framing:** `Cmd 0x7D`, `SettingID = 0x06`
- **Available Options:** `0` = Off, `1` = Visual/Audio warning only, `2` = Active steering intervention.

#### 5. Traffic Sign Recognition (`TrafficSignRecognitionSystem_Cb` / TSR Speed Limits)
- **Downlink Framing:** `Cmd 0x7D`, `SettingID = 0x22` (`34` dec)
- **Available Options:** `0` = Off, `1` = Display road signs on cluster.

#### 6. Active Emergency Braking Sensitivity (`EmergencyBrakingSensitivity_List` / AEB Forward Collision)
- **Downlink Framing:** `Cmd 0x7D`, `SettingID = 0x21` (`33` dec)
- **Available Options:**
  - `0`: **Late / Close** (Warns only on immediate hazard)
  - `1`: **Normal** (Standard safety distance)
  - `2`: **Early / Far** (Conservative long-range warning)

#### 7. Electronic Stability Program / Traction Control (`TractionControlSystem_Cb` / ESP)
- **Downlink Framing:** `Cmd 0x7D`, `SettingID = 0x19` (`25` dec)
- **Available Options:** `0` = ESP OFF (Inhibited up to 50 km/h), `1` = ESP ON.

---

### 2.5 Domain 5: Interior Comfort, Wellness & Seats

#### 1. Seat Massage Mode & Intensity (`SeatMassageMode_List` / `SeatMassageSpeed_List`)
- **Downlink Framing:** `Cmd 0x7D`, `SettingID = 0x1D` (`29` dec) for Mode / `SettingID = 0x1F` (`31` dec) for Speed
- **Available Options:**
  - **Mode:** `0` = Wave, `1` = Lumbar, `2` = Stretch, `3` = Shoulders.
  - **Speed / Intensity:** `0` = Off, `1` = Low, `2` = Medium, `3` = High.

#### 2. Interior Fragrance / Aromatherapy Diffuser (`Aromatherapy_List` / `AromatherapyConcentration_List`)
- **Downlink Framing:** `Cmd 0x7D`, `SettingID = 0x0D` (`13` dec) for Scent / `SettingID = 0x0E` (`14` dec) for Intensity
- **Available Options:**
  - **Scent:** `0` = Off, `1` = Scent 1 (Harmony), `2` = Scent 2 (Energizing), `3` = Scent 3 (Relax).
  - **Intensity:** `0` = Low, `1` = Medium, `2` = Strong.

#### 3. Air Ionizer / Anion Purifier (`AnionPurifier_List`)
- **Downlink Framing:** `Cmd 0x7D`, `SettingID = 0x0C` (`12` dec)
- **Available Options:** `0` = Off, `1` = Clean Mode, `2` = Relax Mode.

#### 4. Dual vs. Mono Zone Temperature Locking (`ZoneTemperature_List`)
- **Downlink Framing:** `Cmd 0x7D`, `SettingID = 0x24` (`36` dec)
- **Available Options:** `0` = Dual Independent Zones, `1` = Mono / Synchronized Passenger Zone.

---

### 2.6 Domain 6: Regional Measurement Units & System Language

#### 1. Temperature Measurement Unit (`TempUnit_List`)
- **Downlink Framing:** `Cmd 0xCA` (`-54` signed / `204` dec), `SettingID = 0x03`
- **Available Options:**
  - `1`: **Celsius ($^\circ\text{C}$)**
  - `2`: **Fahrenheit ($^\circ\text{F}$)**

#### 2. Fuel Economy Consumption Unit (`OilUnit_List`)
- **Downlink Framing:** `Cmd 0xCA`, `SettingID = 0x05`
- **Available Options:**
  - `1`: **$\text{L/100km}$** (Metric Standard)
  - `2`: **$\text{km/L}$** (Japanese/Italian format)
  - `3`: **$\text{MPG (US)}$**
  - `4`: **$\text{MPG (UK)}$**

#### 3. System Language (`ForwardLanguageSetting` / `Cmd 0x9A`)
- **Downlink Framing:** `Cmd 0x9A` (`-102` signed / `154` dec), `SettingID = 0x01`
- **Available Language Codes:**
  - `0x00`: **English**
  - `0x01`: **Simplified Chinese (简体中文)**
  - `0x02`: **French (Français)**
  - `0x03`: **German (Deutsch)**
  - `0x04`: **Spanish (Español)**
  - `0x05`: **Italian (Italiano)**
  - `0x06`: **Polish (Polski)**
  - `0x07`: **Russian (Русский)**
  - `0x08`: **Portuguese (Português)**
  - `0x09`: **Dutch (Nederlands)**
  - `0x0A`: **Turkish (Türkçe)**
  - `0x0B`: **Arabic (العربية)**

---

# 3. Master Downlink Setting ID Translation Table

| Function / Setting Name | Downlink Cmd ID | Sub-Setting Index (Hex) | Sub-Setting Index (Dec) | Valid Payload Values | Target PSA CAN Frame |
|:---|:---:|:---:|:---:|:---|:---|
| **Follow-Me-Home Delay** | `0x7B` (`123`) | `0x06` | `6` | `0`=0s, `1`=15s, `2`=30s, `3`=60s | `0x221` Byte 4 Bits 7..6 |
| **Welcome Light Duration**| `0x7B` (`123`) | `0x09` | `9` | `0`=0s, `1`=15s, `2`=30s, `3`=60s | `0x221` Byte 4 Bits 5..4 |
| **Welcome Lights Switch** | `0x7D` (`125`) | `0x05` | `5` | `0`=Off, `1`=On | `0x221` BSI Internal |
| **Daytime Running (DRL)** | `0x7B` (`123`) | `0x05` | `5` | `0`=Off, `1`=On | `0x221` Byte 2 Bit 7 (`0x80`)|
| **Auto Door Lock Driving**| `0x7B` (`123`) | `0x0C` | `12` | `0`=Off, `1`=On ($>10\text{ km/h}$) | `0x221` Byte 1 Bit 4 (`0x10`)|
| **Selective Door Unlock** | `0x7B` (`123`) | `0x04` | `4` | `0`=All, `1`=Driver Only | `0x221` Byte 7 Bit 7 |
| **Auto Rear Wiper Reverse**| `0x7B` (`123`) | `0x01` | `1` | `0`=Off, `1`=On | `0x221` Byte 1 Bit 7 (`0x80`)|
| **Ambient Light & Bright**| `0x7B` (`123`) | `0x0A` | `10` | `0x80 \| (Brightness & 0x0F)` | `0x221` Byte 3 Bits 3..0 |
| **TPMS Reset Trigger** | `0x7D` (`125`) | `0x03` | `3` | `1` (One-shot calibration pulse) | `0x221` Byte 3 Bit 4 (`0x10`)|
| **Mirror Auto-Fold Lock** | `0x7D` (`125`) | `0x23` | `35` | `0`=Disabled, `1`=Enabled | `0x221` Byte 4 Bit 3 (`0x08`)|
| **Directional Headlights** | `0x7D` (`125`) | `0x02` | `2` | `0`=Off, `1`=On | `0x221` Byte 2 Bit 0 (`0x01`)|
| **Auto High Beam Assist** | `0x7D` (`125`) | `0x17` | `23` | `0`=Off, `1`=On | `0x221` BSI Flag |
| **Driver Fatigue Warning** | `0x7D` (`125`) | `0x07` | `7` | `0`=Off, `1`=On | `0x221` BSI Flag |
| **Lane Keep Assist (LKA)**| `0x7D` (`125`) | `0x06` | `6` | `0`=Off, `1`=Warn, `2`=Active | `0x221` BSI Flag |
| **AEB Collision Distance**| `0x7D` (`125`) | `0x21` | `33` | `0`=Close, `1`=Norm, `2`=Far | `0x221` BSI Flag |
| **Fragrance Scent Select** | `0x7D` (`125`) | `0x0D` | `13` | `0`=Off, `1`=Scent1, `2`=2, `3`=3| `0x221` BSI Flag |
| **Fragrance Intensity** | `0x7D` (`125`) | `0x0E` | `14` | `0`=Low, `1`=Med, `2`=High | `0x221` BSI Flag |
| **Anion Air Purifier** | `0x7D` (`125`) | `0x0C` | `12` | `0`=Off, `1`=Clean, `2`=Relax | `0x221` BSI Flag |
| **Temperature Unit** | `0xCA` (`204`) | `0x03` | `3` | `1`=Celsius, `2`=Fahrenheit | Cluster Config |
| **Fuel Economy Unit** | `0xCA` (`204`) | `0x05` | `5` | `1`=L/100km, `2`=km/L, `3`=MPG | Cluster Config |
| **System Language** | `0x9A` (`154`) | `0x01` | `1` | `0`=EN, `1`=CN, `2`=FR, `3`=DE | Cluster / MFD Language |

---

# 4. Master Uplink Telemetry Bitfield Layout

The CAN box periodically (every 1000 ms) and on-change transmits the full vehicle setting state across two primary telemetry frames:

### 4.1 Uplink Page 1 (`Cmd 0x76` / `118` dec - `CentralState1`)
- **Header:** `0x5A 0xA5 0x02 76 [Byte 0] [Byte 1] [Checksum]`
- **Byte 0:**
  - `Bit 7`: Parking Radar Auto-Active (`0x80`)
  - `Bit 6`: Parking Radar Sound Enabled (`0x40`)
  - `Bits 5..4`: Welcome Light Timer (`00`=0s, `01`=15s, `10`=30s, `11`=60s)
  - `Bit 3`: Ambient Mood Light Enabled (`0x08`)
  - `Bits 2..0`: Ambient Light Brightness level ($0 \dots 7$)
- **Byte 1:**
  - `Bit 7`: Auto Rear Wiper in Reverse (`0x80`)
  - `Bit 6`: Parking Assist Line Overlay (`0x40`)
  - `Bit 5`: Automatic Door Lock while Driving (`0x20`)
  - `Bit 4`: Deadlocking Safety Active (`0x10`)
  - `Bit 3`: Selective Door Unlocking (`0x08`: 1=Driver only, 0=All)
  - `Bit 2`: Daytime Running Lights Enabled (`0x04`)
  - `Bits 1..0`: Follow-Me-Home Headlamp Delay (`00`=0s, `01`=15s, `10`=30s, `11`=60s)

### 4.2 Uplink Page 2 (`Cmd 0x79` / `121` dec - `CentralState`)
- **Header:** `0x5A 0xA5 0x07 79 [B0] [B1] [B2] [B3] [B4] [B5] [B6] [Checksum]`
- **Byte 0:** Remote Trunk Only (`b7`), Adaptive Headlamps (`b6`), Cornering Lights (`b5`), Welcome Lights (`b4`), Lane Keep Assist (`b3`), Fatigue Warning (`b2`), Overspeed Alert (`b1`), Theme Color (`b0`).
- **Byte 1:** Ambient Mode (`b7..b6`), Drive Mode (`b5`), Anion Purifier (`b4..b3`), Fragrance Scent (`b2..b1`).
- **Byte 2:** Fragrance Concentration (`b7..b6`), Cluster Backlight (`b5..b2`), Electric Tailgate (`b1`), Auto Tailgate Open (`b0`).
- **Byte 3:** Keyless Tailgate Unlock (`b5`), Auto High Beam (`b4`), Touch Sensitivity (`b3`), Traction Control ESP (`b2`), Mirror Auto-Fold (`b1`), Headlight Delay Switch (`b0`).
- **Byte 4:** Seat Massage Mode (`b5..b3`), Engine Close Delay (`b2..b0`).
- **Byte 5:** Seat Massage Speed (`b7..b6`), Door Close Extinguish Delay (`b5..b4`), AEB Collision Sensitivity (`b3..b2`), Traffic Sign Recognition TSR (`b1`), Mirror Fold Enabled (`b0`).
- **Byte 6:** Zone Temp Sync (`b2..b1`), Seat Position Tips (`b0`).

---

# 5. Pure C99 Configuration Engine Implementation

### 5.1 Configuration Data Model (`hiworld_car_config.h`)

```c
#ifndef HIWORLD_CAR_CONFIG_H
#define HIWORLD_CAR_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Full Vehicle Configuration State Model */
typedef struct {
    /* Lighting & Visibility */
    uint8_t follow_me_home_sec;   /* 0, 15, 30, 60 */
    uint8_t welcome_light_sec;    /* 0, 15, 30, 60 */
    bool    welcome_lights_switch;
    bool    drl_enabled;          /* Daytime Running Lights */
    bool    adaptive_headlights;
    bool    cornering_lights;
    bool    auto_high_beam;
    bool    ambient_light_switch;
    uint8_t ambient_brightness;   /* 0..15 */
    uint8_t ambient_mode;         /* 0=Normal, 1=Relax, 2=Boost */
    uint8_t theme_color;          /* 0=Blue, 1=Red, 2=White, 3=Green */
    uint8_t cluster_backlight;    /* 0..15 */

    /* Locking & Security */
    bool    auto_lock_driving;    /* Lock > 10 km/h */
    bool    selective_unlocking;  /* true=Driver only, false=All */
    bool    remote_trunk_only;
    bool    auto_mirror_fold;     /* Fold mirrors on remote lock */
    bool    handsfree_tailgate;

    /* Wipers & Driving */
    bool    rear_wiper_reverse;
    bool    rain_sensor_wipers;
    bool    start_stop_active;
    uint8_t drive_mode;           /* 0=Normal, 1=Eco, 2=Sport, 3=Snow */

    /* ADAS & Safety */
    bool    radar_system_enabled;
    bool    radar_auto_active;
    bool    blind_spot_sam;
    bool    driver_fatigue_alert;
    uint8_t lane_keep_assist;     /* 0=Off, 1=Warn, 2=Active */
    bool    traffic_sign_tsr;
    uint8_t aeb_sensitivity;      /* 0=Late, 1=Normal, 2=Early */
    bool    esp_traction_control;

    /* Interior & Seats */
    uint8_t seat_massage_mode;    /* 0=Off, 1=Wave, 2=Lumbar, 3=Stretch */
    uint8_t seat_massage_speed;   /* 0=Off, 1=Low, 2=Med, 3=High */
    uint8_t fragrance_scent;      /* 0=Off, 1..3 */
    uint8_t fragrance_intensity;  /* 0=Low, 1=Med, 2=High */
    uint8_t anion_purifier;       /* 0=Off, 1=Clean, 2=Relax */
    bool    zone_temp_mono_sync;

    /* Units & Language */
    uint8_t unit_temp;            /* 0=Celsius, 1=Fahrenheit */
    uint8_t unit_fuel;            /* 0=L/100km, 1=km/L, 2=MPG US, 3=MPG UK */
    uint8_t language_id;          /* 0=EN, 1=CN, 2=FR, 3=DE, 4=ES, etc. */
} hiworld_car_config_t;

typedef void (*hiworld_uart_tx_fn)(const uint8_t *buf, size_t len);
typedef void (*hiworld_can_tx_fn)(uint32_t id, const uint8_t *data, uint8_t dlc);

typedef struct {
    hiworld_car_config_t state;
    hiworld_uart_tx_fn   uart_tx;
    hiworld_can_tx_fn    can_tx;
} hiworld_config_ctx_t;

void hiworld_config_init(hiworld_config_ctx_t *ctx, hiworld_uart_tx_fn uart_tx, hiworld_can_tx_fn can_tx);
void hiworld_config_process_downlink(hiworld_config_ctx_t *ctx, uint8_t cmd_id, const uint8_t *payload, uint8_t len);
void hiworld_config_send_telemetry_page1(hiworld_config_ctx_t *ctx);
void hiworld_config_send_telemetry_page2(hiworld_config_ctx_t *ctx);
void hiworld_config_sync_to_psa_can(hiworld_config_ctx_t *ctx);

#endif /* HIWORLD_CAR_CONFIG_H */
```

---

### 5.2 Downlink Parser & Uplink Serializer (`hiworld_car_config.c`)

```c
#include "hiworld_car_config.h"
#include <string.h>

static uint8_t calc_cs(const uint8_t *frame) {
    uint8_t len = frame[2];
    uint8_t sum = 0;
    for (size_t i = 2; i < (size_t)(len + 4); i++) {
        sum += frame[i];
    }
    return (uint8_t)((sum - 1) & 0xFF);
}

void hiworld_config_init(hiworld_config_ctx_t *ctx, hiworld_uart_tx_fn uart_tx, hiworld_can_tx_fn can_tx) {
    memset(ctx, 0, sizeof(hiworld_config_ctx_t));
    ctx->uart_tx = uart_tx;
    ctx->can_tx  = can_tx;

    /* Defaults: Follow-Me-Home 30s, DRL ON, Auto-Lock ON, Mirror Fold ON */
    ctx->state.follow_me_home_sec    = 30;
    ctx->state.welcome_light_sec     = 30;
    ctx->state.welcome_lights_switch = true;
    ctx->state.drl_enabled           = true;
    ctx->state.auto_lock_driving     = true;
    ctx->state.auto_mirror_fold      = true;
    ctx->state.rear_wiper_reverse    = true;
    ctx->state.radar_system_enabled  = true;
    ctx->state.radar_auto_active     = true;
    ctx->state.esp_traction_control  = true;
    ctx->state.unit_temp             = 0; /* Celsius */
    ctx->state.unit_fuel             = 0; /* L/100km */
    ctx->state.language_id           = 0; /* English */
}

/* Transmit Uplink Telemetry Page 1 (Cmd 0x76) */
void hiworld_config_send_telemetry_page1(hiworld_config_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[7];
    p[0] = 0x5A;
    p[1] = 0xA5;
    p[2] = 0x02; /* Length: 2 bytes */
    p[3] = 0x76; /* Cmd ID */

    /* Byte 0 */
    uint8_t b0 = 0;
    if (ctx->state.radar_auto_active)    b0 |= 0x80;
    if (ctx->state.radar_system_enabled) b0 |= 0x40;
    if (ctx->state.welcome_light_sec == 60) b0 |= 0x30;
    else if (ctx->state.welcome_light_sec == 30) b0 |= 0x20;
    else if (ctx->state.welcome_light_sec == 15) b0 |= 0x10;
    if (ctx->state.ambient_light_switch) b0 |= 0x08;
    b0 |= (ctx->state.ambient_brightness & 0x07);
    p[4] = b0;

    /* Byte 1 */
    uint8_t b1 = 0;
    if (ctx->state.rear_wiper_reverse)   b1 |= 0x80;
    if (ctx->state.auto_lock_driving)    b1 |= 0x20;
    if (ctx->state.selective_unlocking)  b1 |= 0x08;
    if (ctx->state.drl_enabled)          b1 |= 0x04;
    if (ctx->state.follow_me_home_sec == 60) b1 |= 0x03;
    else if (ctx->state.follow_me_home_sec == 30) b1 |= 0x02;
    else if (ctx->state.follow_me_home_sec == 15) b1 |= 0x01;
    p[5] = b1;

    p[6] = calc_cs(p);
    ctx->uart_tx(p, 7);
}

/* Transmit Uplink Telemetry Page 2 (Cmd 0x79) */
void hiworld_config_send_telemetry_page2(hiworld_config_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[12];
    p[0] = 0x5A;
    p[1] = 0xA5;
    p[2] = 0x07; /* Length: 7 bytes */
    p[3] = 0x79; /* Cmd ID */

    /* Byte 0 */
    uint8_t b0 = 0;
    if (ctx->state.remote_trunk_only)     b0 |= 0x80;
    if (ctx->state.adaptive_headlights)   b0 |= 0x40;
    if (ctx->state.cornering_lights)      b0 |= 0x20;
    if (ctx->state.welcome_lights_switch) b0 |= 0x10;
    if (ctx->state.driver_fatigue_alert)  b0 |= 0x04;
    b0 |= (ctx->state.theme_color & 0x01);
    p[4] = b0;

    /* Byte 1 */
    uint8_t b1 = (ctx->state.ambient_mode & 0x03) << 6;
    if (ctx->state.drive_mode > 0) b1 |= 0x20;
    b1 |= (ctx->state.anion_purifier & 0x03) << 3;
    b1 |= (ctx->state.fragrance_scent & 0x03) << 1;
    p[5] = b1;

    /* Byte 2 */
    uint8_t b2 = (ctx->state.fragrance_intensity & 0x03) << 6;
    b2 |= (ctx->state.cluster_backlight & 0x0F) << 2;
    p[6] = b2;

    /* Byte 3 */
    uint8_t b3 = 0;
    if (ctx->state.handsfree_tailgate)    b3 |= 0x20;
    if (ctx->state.auto_high_beam)        b3 |= 0x10;
    if (ctx->state.esp_traction_control)  b3 |= 0x04;
    if (ctx->state.auto_mirror_fold)      b3 |= 0x02;
    p[7] = b3;

    /* Byte 4 */
    p[8] = ((ctx->state.seat_massage_mode & 0x07) << 3);

    /* Byte 5 */
    uint8_t b5 = (ctx->state.seat_massage_speed & 0x03) << 6;
    b5 |= (ctx->state.aeb_sensitivity & 0x03) << 2;
    if (ctx->state.traffic_sign_tsr)  b5 |= 0x02;
    if (ctx->state.auto_mirror_fold)  b5 |= 0x01;
    p[9] = b5;

    /* Byte 6 */
    p[10] = (ctx->state.zone_temp_mono_sync ? 0x02 : 0x00);

    p[11] = calc_cs(p);
    ctx->uart_tx(p, 12);
}

/* Synchronize Configuration State to Vehicle PSA BSI Frame (CAN ID 0x221) */
void hiworld_config_sync_to_psa_can(hiworld_config_ctx_t *ctx) {
    if (!ctx->can_tx) return;

    uint8_t can_data[8] = {0};

    /* Byte 1: Rear Wiper & Auto Lock */
    if (ctx->state.rear_wiper_reverse)   can_data[1] |= 0x80;
    if (ctx->state.auto_lock_driving)    can_data[1] |= 0x10;
    if (ctx->state.radar_system_enabled) can_data[1] |= 0x08;

    /* Byte 2: DRL & Directional Lighting */
    if (ctx->state.drl_enabled)          can_data[2] |= 0x80;
    if (ctx->state.cornering_lights || ctx->state.adaptive_headlights) can_data[2] |= 0x01;

    /* Byte 3: Ambient Lighting & TPMS Reset */
    can_data[3] = (ctx->state.ambient_brightness & 0x0F);

    /* Byte 4: Follow-Me-Home & Mirror Fold */
    if (ctx->state.follow_me_home_sec == 60) can_data[4] |= 0xC0;
    else if (ctx->state.follow_me_home_sec == 30) can_data[4] |= 0x80;
    else if (ctx->state.follow_me_home_sec == 15) can_data[4] |= 0x40;
    if (ctx->state.auto_mirror_fold) can_data[4] |= 0x08;

    /* Byte 7: Selective Unlocking */
    if (ctx->state.selective_unlocking) can_data[7] |= 0x80;

    ctx->can_tx(0x221, can_data, 8);
}

/* Parse Downlink Touchscreen Settings from Android */
void hiworld_config_process_downlink(hiworld_config_ctx_t *ctx, uint8_t cmd_id, const uint8_t *payload, uint8_t len) {
    if (len < 2) return;

    uint8_t setting_id = payload[0];
    uint8_t val        = payload[1];

    if (cmd_id == 0x7B) {
        /* Primary Settings (Cmd 0x7B / 123 dec) */
        switch (setting_id) {
            case 0x01: /* Auto Rear Wiper in Reverse */
                ctx->state.rear_wiper_reverse = (val != 0);
                break;
            case 0x04: /* Selective Door Unlocking */
                ctx->state.selective_unlocking = (val != 0);
                break;
            case 0x05: /* Daytime Running Lights (DRL) */
                ctx->state.drl_enabled = (val != 0);
                break;
            case 0x06: /* Follow-Me-Home Headlamp Delay */
                ctx->state.follow_me_home_sec = (val == 3) ? 60 : (val == 2) ? 30 : (val == 1) ? 15 : 0;
                break;
            case 0x09: /* Welcome Light Duration */
                ctx->state.welcome_light_sec = (val == 3) ? 60 : (val == 2) ? 30 : (val == 1) ? 15 : 0;
                break;
            case 0x0A: /* Ambient Light Switch & Brightness */
                ctx->state.ambient_light_switch = ((val & 0x80) != 0);
                ctx->state.ambient_brightness   = (val & 0x0F);
                break;
            case 0x0B: /* Parking Radar Master Switch */
                ctx->state.radar_system_enabled = (val != 0);
                break;
            case 0x0C: /* Auto Lock when Driving */
                ctx->state.auto_lock_driving = (val != 0);
                break;
            default:
                break;
        }
    } 
    else if (cmd_id == 0x7D) {
        /* Extended Settings (Cmd 0x7D / 125 dec) */
        switch (setting_id) {
            case 0x01: /* Remote Trunk Only */
                ctx->state.remote_trunk_only = (val != 0);
                break;
            case 0x02: /* Directional Headlamps */
                ctx->state.adaptive_headlights = (val != 0);
                break;
            case 0x03: /* TPMS Calibration Pulse */
                if (ctx->can_tx) {
                    uint8_t tpms_pulse[8] = { 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00 };
                    ctx->can_tx(0x221, tpms_pulse, 8);
                }
                break;
            case 0x04: /* Cornering Light Assistance */
                ctx->state.cornering_lights = (val != 0);
                break;
            case 0x05: /* Welcome Lights Switch */
                ctx->state.welcome_lights_switch = (val != 0);
                break;
            case 0x06: /* Lane Keep Assist */
                ctx->state.lane_keep_assist = val;
                break;
            case 0x07: /* Driver Fatigue Warning */
                ctx->state.driver_fatigue_alert = (val != 0);
                break;
            case 0x08: /* Parking Radar Auto Active */
                ctx->state.radar_auto_active = (val != 0);
                break;
            case 0x09: /* Atmosphere Mode */
                ctx->state.ambient_mode = val;
                break;
            case 0x0B: /* Drive Mode */
                ctx->state.drive_mode = val;
                break;
            case 0x0C: /* Anion Air Purifier */
                ctx->state.anion_purifier = val;
                break;
            case 0x0D: /* Fragrance Scent Select */
                ctx->state.fragrance_scent = val;
                break;
            case 0x0E: /* Fragrance Intensity */
                ctx->state.fragrance_intensity = val;
                break;
            case 0x17: /* Auto High Beam */
                ctx->state.auto_high_beam = (val != 0);
                break;
            case 0x19: /* Traction Control ESP */
                ctx->state.esp_traction_control = (val != 0);
                break;
            case 0x1D: /* Seat Massage Mode */
                ctx->state.seat_massage_mode = val;
                break;
            case 0x1F: /* Seat Massage Speed */
                ctx->state.seat_massage_speed = val;
                break;
            case 0x21: /* AEB Collision Sensitivity */
                ctx->state.aeb_sensitivity = val;
                break;
            case 0x22: /* Traffic Sign Recognition TSR */
                ctx->state.traffic_sign_tsr = (val != 0);
                break;
            case 0x23: /* Auto Mirror Fold on Remote Lock */
                ctx->state.auto_mirror_fold = (val != 0);
                break;
            case 0x24: /* Zone Temperature Sync */
                ctx->state.zone_temp_mono_sync = (val != 0);
                break;
            default:
                break;
        }
    }
    else if (cmd_id == 0xCA) {
        /* Regional Units (Cmd 0xCA / 204 dec) */
        if (setting_id == 0x03) {
            ctx->state.unit_temp = (val == 2) ? 1 : 0; /* 1=C, 2=F */
        } else if (setting_id == 0x05) {
            ctx->state.unit_fuel = (val >= 1) ? (val - 1) : 0; /* 1=L/100, 2=km/L, 3=MPG */
        }
    }
    else if (cmd_id == 0x9A) {
        /* Language Selection (Cmd 0x9A / 154 dec) */
        if (setting_id == 0x01) {
            ctx->state.language_id = val;
        }
    }

    /* Synchronize updated state to BSI CAN and acknowledge to Android UI */
    hiworld_config_sync_to_psa_can(ctx);
    hiworld_config_send_telemetry_page1(ctx);
    hiworld_config_send_telemetry_page2(ctx);
}
```

---

# 6. Desktop Test Harness & Verification Vectors

### Vector 1: Headunit Configures Follow-Me-Home Headlights to 30 Seconds
- **Downlink Serial Received (Host $\to$ CAN Box):**
  ```
  Hex Trace: 5A A5 02 7B 06 02 84
  ```
  - Preamble: `5A A5`
  - Length: `0x02`
  - Command: `0x7B` (`Command.ForwardCentralState1`)
  - Payload: Setting ID = `0x06` (`WithMeGoHomeTime`), Value = `0x02` (30s)
  - Checksum: `(0x02 + 0x7B + 0x06 + 0x02 - 1) & 0xFF = 0x84`
- **Expected PSA BSI CAN Frame Injected:**
  - `ID 0x221#0000000080000000` (`Byte 4: 0x80` = 30s)

### Vector 2: Headunit Enables Daytime Running Lights (DRL = ON)
- **Downlink Serial Received:**
  ```
  Hex Trace: 5A A5 02 7B 05 01 82
  ```
  - Command: `0x7B`, Setting ID = `0x05` (`DayRunCarLight`), Value = `0x01` (ON)
  - Checksum: `(0x02 + 0x7B + 0x05 + 0x01 - 1) & 0xFF = 0x82`
- **Expected PSA BSI CAN Frame Injected:**
  - `ID 0x221#0000800000000000` (`Byte 2: 0x80` = DRL ON)

### Vector 3: Headunit Enables Auto Mirror Fold on Lock
- **Downlink Serial Received:**
  ```
  Hex Trace: 5A A5 02 7D 23 01 A2
  ```
  - Command: `0x7D` (`Command.ForwardCentralState2`), Setting ID = `0x23` (`RearMirrorAutoFolder`), Value = `0x01` (ON)
  - Checksum: `(0x02 + 0x7D + 0x23 + 0x01 - 1) & 0xFF = 0xA2`
- **Expected PSA BSI CAN Frame Injected:**
  - `ID 0x221#0000000008000000` (`Byte 4: 0x08` = Mirror Fold Enabled)

### Vector 4: Headunit Triggers TPMS Calibration Reset
- **Downlink Serial Received:**
  ```
  Hex Trace: 5A A5 02 7D 03 01 82
  ```
  - Command: `0x7D`, Setting ID = `0x03` (`Tire_PressureCalibration`), Value = `0x01`
  - Checksum: `(0x02 + 0x7D + 0x03 + 0x01 - 1) & 0xFF = 0x82`
- **Expected PSA BSI CAN Frame Injected:**
  - `ID 0x221#0000001000000000` (`Byte 3: 0x10` = Calibration Pulse)

