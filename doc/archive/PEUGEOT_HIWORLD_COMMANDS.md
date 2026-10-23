# Hiworld protocol commands for Peugeot 407


Here is the complete specification of **Hiworld protocol commands supported by the Android Head Unit (`com.qf.vehicle`) for the Peugeot 407 (`peugeot_407_06_all` / `peugeot_407_06_keep_host`)**, categorized into **Uplink (CAN Box $\to$ Android)** and **Downlink (Android $\to$ CAN Box)**, followed by a **checklist of features commonly missing in custom firmware**.

---

### Hiworld Packet Framing & Checksum

All communication over UART (typically **38400 baud, 8N1**) follows the standard Hiworld framing:
```text
[0x5A] [0xA5] [LEN] [CMD] [DATA_0 ... DATA_N-1] [CHECKSUM]
```
- **SOF Header:** `0x5A 0xA5`
- **LEN:** Number of payload bytes including `CMD` and all `DATA` bytes (`LEN = 1 + N`).
- **Checksum:** Sum of all bytes starting from index 1 (`0xA5`) up to the last data byte, minus 1, masked to 8 bits:
  $$\text{Checksum} = \left(\sum_{i=1}^{\text{LEN}+3} \text{packet}[i] - 1\right) \ \& \ \text{0xFF}$$

---

### 1. Uplink Commands (CAN Box $\to$ Android Head Unit)

These are frames your custom firmware sends to the Android Head Unit to update the virtual instrument cluster, climate overlay, OEM radio screen, and vehicle settings.

| Command Hex | Constant Name | Payload Size | Purpose & Peugeot 407 Functionality |
| :---: | :--- | :---: | :--- |
| **`0x11`** | `CarBaseInfo` | 4–6 bytes | **Steering Wheel Angle & Trajectory Lines:** `D0:D1` = 16-bit signed steering angle for camera guidelines. <br>**SWC / Stalk Keys:** `D2` = Key code (`0x01` Vol+, `0x02` Vol-, `0x03` Mute, `0x04` Voice, `0x05`/`0x40` Tel, `0x08` Prev, `0x09` Next, `0x0B` Mode, `0x14` Trip), `D3` = Press (`0x01`) / Release (`0x00`). |
| **`0x12`** | `DoorWindowState` | 4–6 bytes | **Doors, Trunk, Bonnet & Windows:** Bitmask for Driver door, Passenger door, Rear-Left, Rear-Right, Trunk/Tailgate, Bonnet/Hood, Sunroof, and electric window states. |
| **`0x13`** | `EcuInfoPage1` | 6 bytes | **Trip Computer (Page 0 / Instantaneous):** Cruising range / distance to empty (km), Instant fuel consumption (L/100km or MPG), Remaining distance to destination. |
| **`0x14`** | `EcuInfoPage2` | 6 bytes | **Trip Computer (Page 1 / Trip 1):** Trip 1 distance (km), Trip 1 average fuel consumption, Trip 1 average speed (km/h). |
| **`0x15`** | `EcuInfoPage3` | 6 bytes | **Trip Computer (Page 2 / Trip 2):** Trip 2 distance (km), Trip 2 average fuel consumption, Trip 2 average speed (km/h). |
| **`0x18`** | `TyreDiscrete` | 4–6 bytes | **TPMS Discrete Warnings:** 4-wheel status (FL, FR, RL, RR): Normal (`0`), Low Pressure (`1`), Puncture/Flat (`2`), Sensor Missing/Battery Flat (`3`). |
| **`0x21`** | `ControlPanelKey` | 2 bytes | **RD4 Center Console Physical Keys:** `D0` = Key code, `D1` = Press (`0x01`) / Release (`0x00`). <br>Mapped via **`PanelMap_407_06`**: `0x07` DARK/DISPLAY, `0x17` FM, `0x18` AM, `0x19` NEXT, `0x1A` PREV, `0x24` HOME, `0x25` BACK, `0x28` CLIM/SETTINGS, `0x2B` TEL/PHONE, `0x2E` SETUP, `0x31` MUSIC, `0x40` TRIP. |
| **`0x22`** | `ControlPanelKnob` | 2 bytes | **RD4 Rotary Encoders:** `D0` = Knob ID (`0x01` Volume Knob, `0x02` Selection/Tune Dial), `D1` = Cumulative counter (delta determines clockwise/counter-clockwise and step count). |
| **`0x31`** | `CarAcState` | 12 bytes | **Dual-Zone Climate Control (HVAC):** <br>• `D0`: Bit 6 = Power, Bit 3 = AUTO, **Bit 2 = SYNC/Mono** (must be 0 for independent dual-zone on 407), Bit 0 = A/C compressor.<br>• `D1`: Recirculation.<br>• `D2`: Defrost front / Rear windshield demister.<br>• `D4`: Airflow direction (Upper nibble = Passenger, Lower nibble = Driver).<br>• `D5`: Blower fan speed (`0x00..0x08`).<br>• `D6`/`D7`: Left / Right target temperatures (`0xFE`=LO, `0xFF`=HI, or half-degrees C).<br>• `D11`: Outside ambient temperature ($(\text{val} \times 0.5) - 40.0^\circ\text{C}$). |
| **`0x41`** | `CarRadarState` | 4–8 bytes | **PDC Parking Sensors:** Distance bar levels (0..4 or 0..8) for 4 Rear sensors (RL, RML, RMR, RR) and 4 Front sensors (FL, FML, FMR, FR). |
| **`0x42`** | `WarningInfo` | Variable | **Active BSI Warning Alerts:** Big-endian 16-bit alert codes (`0x0001` Low Fuel, `0x0003` Engine Temp, `0x0005` Brake Fluid, `0x0008` Handbrake, `0x0069` ABS, `0x006B` AMVAR Suspension fault, `0x009A` TPMS). Sending `5A A5 02 42 00 00 43` clears active popups. |
| **`0x56`** | `DspStatePopUp` | 6–8 bytes | **RD4 Audio / DSP HUD Pop-up:** <br>• Mode byte: `1` Volume, `2` Bass, `3` Treble, `4` Balance, `5` Fader, `6` Loudness, `7` Speed-volume, `8` EQ.<br>• Volume level (`0x00..0x1E` / 0..30), Mute bit, Bass/Treble (-7..+7), Balance/Fader (-7..+7). |
| **`0x66`** | `TyreNumeric` | 8–12 bytes | **TPMS Numeric Pressures:** 4-wheel pressures in 0.1 Bar increments + tire temperatures. |
| **`0x71`** / **`0x72`** | `CentralStateEnable1/2`| 4–8 bytes | **BSI Personalization Feature Masks:** Informs Android which vehicle settings menus are supported by the car's BSI configuration. |
| **`0x76`** / **`0x79`** | `CentralState` | 4–8 bytes | **Vehicle Personalization States:** Reverse rear wiper enable, selective trunk unlocking, guide-me-home delay, Daytime Running Lights (DRL), radar mute. |
| **`0x81`** | `RecruiseSpeedValueInfo`| 4–6 bytes | **Vehicle Speed & Engine Telemetry:** Current vehicle speed (km/h), Engine RPM, Odometer/gear. |
| **`0x82`** | `CruiseSpeedInfo` | 4–6 bytes | **Cruise Control & Speed Limiter:** Current programmed cruise speed, limiter speed, and engagement status (Active / Paused / Off). |
| **`0x84`** | `CarRadioState` | 14+ bytes | **RD4 OEM Tuner Status:** Current band (`0x00` FM1, `0x01` FM2, `0x04` FMAST, `0x10` AM, `0x20` AUX, `0x30` CD, `0x40` OFF), tuned frequency (e.g. 102.50 MHz), preset channel (1..6), Stereo flag, TA/RDS flags, 8-character RDS station name (PS). |
| **`0x85`** | `CarRadioPreFrequency` / `SportModeInfo` | 2–14 bytes | **RD4 Preset Frequencies / Suspension:** If length > 6: Preset 1..6 station frequencies. If length $\le$ 6: Sport mode status (AMVAR electronically controlled damping on 407 V6). |
| **`0x86`** | `RadioTextInfo` | Variable | **RD4 Dedicated RDS Radio Text:** RDS PS (8 chars), PTY program genre code, full scrolling Radio Text (RT string up to 64 chars). |
| **`0x93`** | `HostInfo` | 2–4 bytes | **Factory RD4 Source & Power:** Current active audio path (Radio, CD, AUX 1, AUX 2, Phone), Power ON/OFF, Mute state. |
| **`0x97`** | `CarMediaState` | 11 bytes | **RD4 Internal CD / CDC Status:** Track number, disc slot (1..6 for CDC), elapsed minutes and seconds, total tracks, Repeat mode, Random/Shuffle state. |
| **`0xC1`** | `UnitInfo` | 2–4 bytes | **Vehicle Units:** Distance unit (km / miles), Consumption (L/100km, MPG US, MPG UK, km/L), Temperature (°C / °F), Pressure (Bar / PSI / kPa). |
| **`0xEA`** | `ExtendedAlert` | Variable | **Extended BSI Diagnostic Journal:** Historic & stored fault log displayed in `DiagnosticFragment`. |
| **`0xF0`** | `CanBoxVersion` | Variable ASCII | **Firmware Version:** ASCII string reporting MCU firmware (e.g., `WC-PSA-V1.02.003`), displayed in Car Info &rarr; Software Version. |

---

### 2. Downlink Commands (Android Head Unit $\to$ CAN Box / Car)

These frames are emitted by the Android Head Unit. Your custom firmware needs to intercept these, parse the parameters, and (where applicable) inject the corresponding frames onto the vehicle's CAN bus (Comfort CAN 125 kbps).

| Command Hex | Constant Name | Payload Size | Purpose & Expected CAN Box Action |
| :---: | :--- | :---: | :--- |
| **`0x0F`** | `QueryRadio` | 1 byte | **Poll RD4 State:** Sent on boot or app resume (`5A A5 01 0F CS`). Custom firmware should reply with `0x84` (Tuner State), `0x56` (DSP), and `0x97` (CD State). |
| **`0x1B`** | `ForwardEcuSetting` | 4 bytes | **Trip Computer Control:** Reset Trip 1, Reset Trip 2, or cycle the active trip display page. |
| **`0x21`** | `ForwardKeyBean` | 2 bytes | **Simulated Console Key:** Android HU transmits virtual panel key press/release. |
| **`0x24`** | `ForwardCarType` | 2 bytes | **Car Model ID Handshake:** Transmitted by Android on boot to set car type in CAN box. |
| **`0x3B`** | `ForwardAcSetting` | 2 bytes | **Climate Control Commands:** Touch commands from the HU AC screen (Blower up/down, Temp up/down, Mode, AUTO, AC toggle, Defrost). Translated to HVAC CAN frames. |
| **`0x7B`** | `ForwardCarMediaState` | 2 bytes | **RD4 CD Toggle Commands:** `Data[0] = 0x0E` (Scan toggle), `Data[0] = 0x0F` (Random/Shuffle toggle). |
| **`0x7D`** | `ForwardCentralState2` | 2 bytes | **Vehicle Personalization Adjustments:** Writes user setting changes (DRL, reverse wiper, guide-me-home delay) to BSI. |
| **`0x8A`** / **`0x8B`** | `ForwardSpeedValueCmd` / `ForwardCruiseSpeedCmd` | 2–10 bytes | **Cruise / Speed Limiter Presets:** Setting cruise limiter memory values. |
| **`0x8C`** | `ForwardSportModeCmd` | 2 bytes | **AMVAR Suspension Switch:** Toggles Sport suspension mode on 407 V6. |
| **`0x9A`** | `ForwardLanguageSetting`| 2 bytes | **System Language Sync:** Synchronizes instrument cluster / MFD language to Android language (English, French, Spanish, German, Italian, etc.). |
| **`0xA1`** | `ForwardHostState` | 3 bytes | **Host Power & Source Master Setting:** Android signals power state or master audio routing. |
| **`0xA2`** | `ForwardTunerCmd` | 6 bytes | **OEM Tuner Downlink:** Frequency change, band change (FM1/FM2/AM), direct frequency seek, or preset recall. |
| **`0xA4`** | `ForwardCdCmd` | 13 bytes | **CD/CDC Media Controls:** Play, Pause, Next Track, Previous Track, Fast Forward, Fast Rewind, CDC Disc Select. |
| **`0xAD`** | `ForwardDspStateSetting`| 2 bytes | **DSP Audio Downlink:** Volume setting, equalizer mode, balance/fader adjustments, and AUX source activation. |
| **`0xCA`** | `ForwardUnitSetting` | 2 bytes | **Units Downlink:** Syncs distance, consumption, and temperature units from Android settings to BSI. |
| **`0xCB`** | `ForwardDateTimeSetting`| 10 bytes | **Clock / Date Synchronization:** Android broadcasts time periodically (`Year`, `Month`, `Day`, `Hour`, `Minute`, `Second`, 12/24H mode) to set the dashboard/MFD clock. |
| **`0xE1`** | `ForwardHostSourceInfo` | 13 bytes | **Android Media Metadata to Car Display:** Transmits current Android track title, artist name, and radio frequency so the car's instrument cluster or secondary display shows the current song. |
| **`0xE4`** | `ForwardId3Info` | 32 bytes | **ID3 Tag Broadcast:** UTF-8 track / artist string transmitted to vehicle display. |
| **`0xF1`** | `Downlink RD4 Control` | 2–3 bytes | **Dedicated Bidirectional RD4 UI Control:** Specifically implemented for Peugeot 407 Original Tuner & CD UI (Step Up/Down `0x01`/`0x02`, Seek Up/Down `0x03`/`0x04`, Preset 1..6 `0x07`). Firmware translates this to SWC stalk frames on `CAN 0x21F`. |
| **`0xF2`** | `ForwardAmvParkingState`| 2 bytes | **Parking Assist View Mode:** Switches PDC camera overlays. |

---

### 3. Checklist: What is Most Commonly Missing in Custom Firmware

When building custom Peugeot 407 Hiworld firmware (e.g. on Nuvoton NUC131 / STM32 / Raise hardware), verify the following:

1. **RD4 Bidirectional Control Downlink (`Cmd 0xF1` & `0x0F`):**
   - **Symptom:** Radio frequency, station name, and CD track appear on Android, but tapping on-screen buttons (Seek, Next, Presets 1..6) does nothing.
   - **Fix:** Custom firmware must listen for `Cmd 0xF1` frames on UART, decode the action type, and transmit the corresponding simulated COM2000 steering wheel stalk command on **`CAN ID 0x21F`** (e.g. `0x21F [05 ...]` for Seek Down, `0x21F [04 ...]` for Seek Up). Also respond to `Cmd 0x0F` with immediate `0x84` telemetry.
2. **Date & Time Synchronization (`Cmd 0xCB`):**
   - **Symptom:** The car's dashboard / instrument cluster clock is flashing or incorrect, with no OEM screen left to configure it.
   - **Fix:** Listen for `Cmd 0xCB` (`5A A5 0A CB Year Month Day Hour Min ...`) from Android and translate it into the PSA BSI time-setting CAN frames (`0x228` / `0x268`).
3. **Dual-Zone HVAC SYNC / Mono Logic (`Cmd 0x31`, `D0` Bit 2):**
   - **Symptom:** Adjusting temperature forces passenger side to always match driver side.
   - **Fix:** On the Peugeot 407, driver and passenger zones are independent by default. Ensure **Bit 2 of `D0` is `0`** unless MONO/SYNC is explicitly active.
4. **Active Alert Dismissal & Clear Frames (`Cmd 0x42` / `0xEA`):**
   - **Symptom:** Warning popup (e.g. "Handbrake applied" or "Door Open") stays stuck on screen even after releasing handbrake or closing door.
   - **Fix:** When alerts clear, send `5A A5 02 42 00 00 43` (Alert Code 0) to notify the Head Unit that the alert journal is empty.
5. **Dynamic Trajectory Lines (`Cmd 0x11`, `D0:D1`):**
   - **Symptom:** Reverse camera guidelines do not bend when turning the wheel.
   - **Fix:** Extract CAN frame `0x0A5` / `0x0E6` (Steering Angle Sensor), convert to signed 16-bit angle, and pack into `D0` (MSB) and `D1` (LSB) of `Cmd 0x11`.
6. **Key Code Mapping for 407 Console (`Cmd 0x21`):**
   - **Symptom:** Pressing RD4 physical buttons triggers the wrong action on Android.
   - **Fix:** Align console button key codes with `PanelMap_407_06` (`0x07` Dark, `0x17` FM, `0x18` AM, `0x28` Clim, `0x2B` Phone, `0x40` Trip).
7. **Volume HUD Popup Trigger (`Cmd 0x56`):**
   - **Symptom:** Turning the physical RD4 volume knob changes audio level, but no on-screen volume overlay pops up.
   - **Fix:** Send `Cmd 0x56` with Mode `0x01` and the new volume level (`0x00..0x1E`) whenever RD4 volume changes.