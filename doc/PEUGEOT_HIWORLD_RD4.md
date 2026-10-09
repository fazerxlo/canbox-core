### RD4 Radio, Source Display, CD Changer & RDS Radio Text Implementation Complete

All code modifications, protocol decoders, UI bindings, auto-switch hooks, Radio Text marquee rendering, and test tools have been implemented, tested, and assembled into the signed APK.

---

### 1. Summary of Changes Implemented

| Component | Target File | Action Taken |
|---|---|---|
| **Data Model** | [`CarRadioState.smali`](file:///home/Fazer/git/QF_Canbus_system/QF_Canbus_system/smali_classes2/com/qf/vehicle/entity/CarRadioState.smali) | Added `.field public mStationName:Ljava/lang/String;` and `.field public mRadioText:Ljava/lang/String;` initialized to `""` in `<init>()`. |
| **UART Protocol Whitelist** | [`PeugeotDataDefine$Handle.smali`](file:///home/Fazer/git/QF_Canbus_system/QF_Canbus_system/smali/com/qf/vehicle/band/peugeot/parse/wc/PeugeotDataDefine$Handle.smali) | Registered `CarRadioState:B = -0x7ct` (`0x84`) and `RadioTextInfo:B = -0x7at` (`0x86`). Expanded `ids` and `stringIds` arrays from `0x1c` to `0x1e` to route packets from the UART reader. |
| **Packet Decoder** | [`PeugeotDataParser.smali`](file:///home/Fazer/git/QF_Canbus_system/QF_Canbus_system/smali/com/qf/vehicle/band/peugeot/parse/wc/PeugeotDataParser.smali) | Added routing for opcodes `-0x7ct` (`0x84`) and `-0x7at` (`0x86`). Implemented `parseCarRadioState(...)`: decodes audio source/band, frequency, preset slot, RDS indicators (ST, RDS, TA, SCAN, REG, RDTEXT, AUTO.P), power status, 8-byte ASCII RDS station name, and optional trailing Radio Text string (`Data[14..N]`). Implemented `parseRadioText(...)`: decodes standalone `0x86` payloads directly into `mRadioText`. |
| **UI Marquee View** | [`res/layout/car_radio.xml`](file:///home/Fazer/git/QF_Canbus_system/QF_Canbus_system/res/layout/car_radio.xml), [`res/values/ids.xml`](file:///home/Fazer/git/QF_Canbus_system/QF_Canbus_system/res/values/ids.xml), [`res/values/public.xml`](file:///home/Fazer/git/QF_Canbus_system/QF_Canbus_system/res/values/public.xml) | Declared view ID `tv_radio_text` (`0x7f08098d`). Inserted full-width marquee `TextView` (20sp, cyan `#33ffff`, single-line, horizontally scrolling marquee) above preset buttons. |
| **UI Binding & Animation** | [`OriginalTuner.smali`](file:///home/Fazer/git/QF_Canbus_system/QF_Canbus_system/smali/com/qf/vehicle/activities/OriginalTuner.smali) | Bound `tv_radio_text = findViewById(0x7f08098d)` in `initView()`. Implemented dedicated helper method `updateRadioText()V` called during `update()`: displays `mRadioText`, triggers `setSelected(true)` to start hardware marquee scrolling, or hides view (`GONE`) when empty/off. |
| **Controller & UI Cache** | [`PeugeotDataController.smali`](file:///home/Fazer/git/QF_Canbus_system/QF_Canbus_system/smali/com/qf/vehicle/band/peugeot/parse/wc/PeugeotDataController.smali) | Added field `lastAudioSource:B = -1`. Updated `forwardType(B)` to immediately respond to resume queries: `0x0F` forwards cached `mCarRadioInfo` to `OriginalTuner`, and `0x11`/`0x12` forwards cached `mCarCdPlayInfo` to `OriginalMediaPlayer`. |
| **Auto-Switch Callback** | [`PeugeotDataController$3.smali`](file:///home/Fazer/git/QF_Canbus_system/QF_Canbus_system/smali/com/qf/vehicle/band/peugeot/parse/wc/PeugeotDataController$3.smali) | Implemented `notifyCarRadioState`: forwards state to UI and automatically switches active app on source change: Radio / AUX &rarr; [`OriginalTuner`](file:///home/Fazer/git/QF_Canbus_system/QF_Canbus_system/smali/com/qf/vehicle/activities/OriginalTuner.smali); CD / CDC &rarr; [`OriginalMediaPlayer`](file:///home/Fazer/git/QF_Canbus_system/QF_Canbus_system/smali/com/qf/vehicle/activities/OriginalMediaPlayer.smali) (debounced against `lastAudioSource`). In `notifyCarMediaState`, active playback triggers auto-switch to `OriginalMediaPlayer`. |
| **App Launcher Visibility** | [`PeugeotConstant.smali`](file:///home/Fazer/git/QF_Canbus_system/QF_Canbus_system/smali/com/qf/vehicle/band/peugeot/PeugeotConstant.smali) | Unconditionally called `VehicleConfigUtil.setUserCarOriginalRadio(context, true)` and enabled `setCarMediaPlayerEnable(context, true)` for all standard Peugeot 407 profiles (`peugeot_407_06_all`). |
| **Changelog Record** | [`CHANGELOG.md`](file:///home/Fazer/git/QF_Canbus_system/CHANGELOG.md) | Added Sections 19, 20, and 21 documenting all modified components and rationale per project guidelines. |

---

### 2. Bit-Level CAN Box UART Protocol Specification

Detailed field descriptions and diagrams are documented below.

#### Frame 1: RD4 Radio / Audio Source Frame (`Cmd 0x84`, Wire Length: 19+ bytes)
```
[0x5A] [0xA5] [LEN] [0x84] [DATA_0 .. DATA_13] [OPTIONAL DATA_14..N] [CHECKSUM]
```
* **`LEN`:** `0x0E` (14 bytes) for standard packet without Radio Text, or `14 + strlen(RadioText)` for extended packet.
* **`Data[0]` (Audio Source / Band):**
  * Radio: `0x00` = FM1, `0x01` = FM2, `0x02` = FM3, `0x04` = FM-AST, `0x10` = AM/MW, `0x11` = AM1, `0x12` = AM2, `0x13` = AM-AST
  * External: `0x20` = Line In (AUX 1), `0x21` = Line In (AUX 2), `0x30` = CD, `0x31` = CDC, `0xFF` = Standby / OFF
* **`Data[1..2]` (Frequency `uint16_be`):**
  * FM: Frequency $\times 10$ ($102.50\text{ MHz} \rightarrow 1025 \rightarrow \mathtt{0x0401}$).
  * AM: Frequency in kHz ($1440\text{ kHz} \rightarrow \mathtt{0x05A0}$).
  * AUX / CD / OFF: Send `0x0000`.
* **`Data[3]` (Preset Slot):** `0x00` = Manual, `0x01 .. 0x06` = Preset 1 to 6.
* **`Data[4]` (Indicators Bitmask):**
  * Bit 7 (`0x80`) = `TA` (Traffic Announcement)
  * Bit 6 (`0x40`) = `ST` (Stereo)
  * Bit 5 (`0x20`) = `RDS` (RDS sync active)
  * Bit 4 (`0x10`) = `SCAN`
  * Bit 3 (`0x08`) = `REG` (Regional)
  * Bit 2 (`0x04`) = `RDTEXT` (Radio Text available)
  * Bit 1 (`0x02`) = `AUTO.P` (Auto-store)
* **`Data[5]` (Power / Status):** `0x00` = Standby / Off, `0x01` = Normal / Playing, `0x02` = Seeking, `0x03` = Mute.
* **`Data[6..13]` (Station Name):** 8 ASCII bytes representing RDS PS name (e.g. `"RMF FM  "` &rarr; `52 4D 46 20 46 4D 20 20`).
* **`Data[14..N]` (Optional Radio Text):** Variable length UTF-8/ASCII string containing RDS Radio Text (song title, artist, program info). Up to 64 characters from PSA CAN `0x0A4`.
* **Checksum:** `((LEN + CMD + sum(DATA)) - 1) & 0xFF`

#### Frame 2: Standalone RDS Radio Text Frame (`Cmd 0x86`, Wire Length: `5 + LEN` bytes)
```
[0x5A] [0xA5] [LEN] [0x86] [DATA_0 .. DATA_N] [CHECKSUM]
```
* **Opcode:** `0x86` (`-0x7at`).
* **`LEN`:** Length of Radio Text string (typically 1 to 64 bytes).
* **`Data[0..N]`:** UTF-8/ASCII bytes of current Radio Text (song title & artist).
* **Checksum:** `((LEN + 0x86 + sum(DATA)) - 1) & 0xFF`

> [!NOTE]
> `0x85` (`-0x7bt`) is reserved by `PeugeotDataParser` for `SportModeInfo` (Peugeot 508 Sport/Eco Mode). Do not use `0x85` for Radio Text.

#### Frame 3: CD Changer / Media Frame (`Cmd 0x97`, Wire Length: 16 bytes)
```
[0x5A] [0xA5] [0x0B] [0x97] [DATA_0 .. DATA_10] [CHECKSUM]
```
* **`Data[0]`:** Active disc slot `1..6`.
* **`Data[1]`:** Discs loaded bitmask (Bits 0..5 = Slots 1..6).
* **`Data[2]`:** Disc format bitmask (0 = Audio CD, 1 = MP3).
* **`Data[3..4]`:** Current track number (`uint16_be`, `1..999`).
* **`Data[5..6]`:** Elapsed minutes (`0..59`) and seconds (`0..59`).
* **`Data[7]`:** Playback modes (Bits: Repeat, Random, Scan).
* **`Data[8]`:** Playback status (`0x01` = Play, `0x02` = Pause, `0x00` = Stop).
* **`Data[9..10]`:** Total tracks on current disc (`uint16_be`).

---

### 3. PSA CAN Bus Mapping Reference for CAN Box Firmware

When implementing CAN box firmware (e.g. STM32, ESP32, Arduino CAN Shield):

| PSA CAN ID | Protocol / Format | Extracted Fields | Hiworld Mapping |
|---|---|---|---|
| **`0x2A5`** | 8 bytes standard | RD4 Source, Frequency, Preset, Flags, 8-byte Station Name (PS) | Pack into `Cmd 0x84` (`Data[0..13]`) |
| **`0x0A4`** | ISO-TP Multi-frame (up to 64 bytes) | Dynamic RDS Radio Text (RT) - song title, artist, radio show | Append to `Cmd 0x84` (`Data[14..N]`) or transmit via `Cmd 0x86` |
| **`0x2E5`** | 8 bytes standard | RD4 Audio Volume, Balance, Fader, Bass, Treble, Loudness | Pack into `Cmd 0x82` (`SoundEffectInfo`) |
| **`0x1E5`** | 8 bytes standard | CD Changer status, active disc, track number, elapsed time | Pack into `Cmd 0x97` (`MediaPlayState`) |

---

### 4. Simulation & Test Generator (`send_peugeot_radio.py`)

The Python test script [`scenarios/send_peugeot_radio.py`](file:///home/Fazer/git/QF_Canbus_system/scenarios/send_peugeot_radio.py) supports serial transmission over USB-UART as well as ADB shell injection:

```bash
# 1. FM Radio with RDS Station Name and Radio Text (Song title) appended to Cmd 0x84:
python3 scenarios/send_peugeot_radio.py --port /dev/ttyUSB0 --baud 38400 --band FM1 --freq 102.50 --station "RMF FM" --rt "Queen - Bohemian Rhapsody"

# 2. Standalone Radio Text update via Cmd 0x86:
python3 scenarios/send_peugeot_radio.py --port /dev/ttyUSB0 --baud 38400 --rt "Queen - Bohemian Rhapsody" --rt-mode standalone

# 3. Switch to Line Input (AUX 1):
python3 scenarios/send_peugeot_radio.py --port /dev/ttyUSB0 --band AUX1

# 4. CD Changer Playback Status (Cmd 0x97): Disc 1, Track 5, 02:35, 18 tracks:
python3 scenarios/send_peugeot_radio.py --port /dev/ttyUSB0 --cdc --cdc-disc 1 --cdc-track 5 --cdc-min 2 --cdc-sec 35 --cdc-total 18

# 5. Turn RD4 Audio OFF / Standby:
python3 scenarios/send_peugeot_radio.py --port /dev/ttyUSB0 --band OFF

# 6. Dry-run frame inspection (no serial port needed):
python3 scenarios/send_peugeot_radio.py --dry-run --band FM1 --freq 102.50 --station "RMF FM" --rt "Queen - Bohemian Rhapsody"
```

---

### 5. Application Launch & Auto-Switching Triggers

This section documents the exact mechanisms, byte conditions, and code paths that trigger [`OriginalTuner`](file:///home/Fazer/git/QF_Canbus_system/QF_Canbus_system/smali/com/qf/vehicle/activities/OriginalTuner.smali) and [`OriginalMediaPlayer`](file:///home/Fazer/git/QF_Canbus_system/QF_Canbus_system/smali/com/qf/vehicle/activities/OriginalMediaPlayer.smali) to open on the Android Head Unit.

#### 5.1 Overview: Launch Modes
There are two ways the applications are launched:
1. **Manual User Launch:** Via launcher icons ("Original Radio" / "Car CD") or vehicle settings apps.
2. **Automatic CAN Event Launch (Auto-Switch):** Triggered automatically in the background when physical buttons on the vehicle RD4 radio/steering wheel change the audio source or when CD playback begins.

```
                           +--------------------------+
                           |  PSA RD4 CAN Bus Event   |
                           +--------------------------+
                                        |
                                        v
                           +--------------------------+
                           | Hiworld UART (0x84/0x97) |
                           +--------------------------+
                                        |
                                        v
                           +--------------------------+
                           |    PeugeotDataParser     |
                           +--------------------------+
                                        |
                                        v
                           +--------------------------+
                           | PeugeotDataController$3  |
                           +--------------------------+
                                /                    \
           [Radio / AUX / 0x84]                       [CD / CDC / 0x97]
                  /                                           \
                 v                                             v
+----------------------------------+          +----------------------------------+
| AppUtil.startCarApp(...)         |          | AppUtil.startCarApp(...)         |
| -> OriginalTuner                 |          | -> OriginalMediaPlayer           |
+----------------------------------+          +----------------------------------+
```

---

#### 5.2 Enabling App Visibility in the ROM
By default on many QingFang ROMs, original OEM car apps are hidden unless explicit car profile flags are enabled. In [`PeugeotConstant.smali`](file:///home/Fazer/git/QF_Canbus_system/QF_Canbus_system/smali/com/qf/vehicle/band/peugeot/PeugeotConstant.smali):
- `VehicleConfigUtil.setUserCarOriginalRadio(context, true)`: Enables the OEM Tuner activity (`OriginalTuner`) in the system app launcher and assigns task affinity `com.qf.vehicle.tuner`.
- `VehicleConfigUtil.setCarMediaPlayerEnable(context, true)`: Enables the OEM CD/Media Player activity (`OriginalMediaPlayer`).
- Both activities are declared with `android:launchMode="singleTask"` in `AndroidManifest.xml`, ensuring only one instance exists and preventing duplicate screen stacking.

---

#### 5.3 Auto-Switching Rules for `OriginalTuner` (Radio vs External Sources)

The automatic trigger is implemented in [`PeugeotDataController$3.smali::notifyCarRadioState(CarRadioState)`](file:///home/Fazer/git/QF_Canbus_system/QF_Canbus_system/smali/com/qf/vehicle/band/peugeot/parse/wc/PeugeotDataController$3.smali):

1. **Incoming Frame:** `Cmd 0x84` is received over UART and parsed into `CarRadioState`.
2. **Power Evaluation:**
   - Byte `Data[5]` (`mCurState`): If `0` (Power is OFF/Standby), or `Data[0]` is `0xFF` (`-1`), the method sets `lastAudioSource = -1` and **does NOT launch** any activity.
3. **External Sources Bypassed (`mBandType >= 0x20`):**
   - For external audio sources (`0x20` Line In AUX 1, `0x21` Line In AUX 2, `0x30` CD, `0x31` CDC), the application **does NOT automatically pop up** `OriginalTuner` or take over the foreground screen. It strictly updates `lastAudioSource = mBandType` and delivers the data to any already-opened UI listeners.
4. **Radio Bands Trigger (`mBandType < 0x20`):**
   - Only when an actual radio tuner band is active (FM1, FM2, FM3, FMAST, AM, AM1, AM2, AMAST):
     - The controller checks the previous audio source (`lastAudioSource`):
       - If `lastAudioSource` was external (`>= 0x20`) or off/standby (`-1`):
         ```smali
         :cond_launch_tuner
         iget-object p1, p0, Lcom/qf/vehicle/band/peugeot/parse/wc/PeugeotDataController$3;->this$0:Lcom/qf/vehicle/band/peugeot/parse/wc/PeugeotDataController;
         iput-byte v1, p1, Lcom/qf/vehicle/band/peugeot/parse/wc/PeugeotDataController;->lastAudioSource:B
         const-string p1, "com.qf.vehicle.activities.OriginalTuner"
         invoke-static {v0, p1}, Lcom/qf/vehicle/utils/AppUtil;->startCarApp(Landroid/content/Context;Ljava/lang/String;)V
         ```
         **Trigger Action:** Brings `OriginalTuner` to the foreground seamlessly when switching to radio.
       - If the user was already listening to a radio tuner band (e.g. switching from FM1 to FM2), `lastAudioSource` is simply updated without restarting or reloading the activity.

---

#### 5.4 Auto-Launch Policy for CD / CDC Media Player

Previously, `Cmd 0x84` (`0x30`/`0x31`) or `Cmd 0x97` playback status (`0x01`) automatically launched `OriginalMediaPlayer`. In order to prevent unwanted popups when listening to auxiliary sources or operating other HU applications (e.g., Navigation, Car Settings):
- Auto-launch on media packets has been disabled in `PeugeotDataController$3.smali`.
- `OriginalMediaPlayer` can still be launched manually from the car apps launcher, and updates its state in real-time when visible.

---

#### 5.5 Instant State Synchronization on Activity Resume (`onResume`)

When either activity is brought to the foreground (whether by auto-switch or manual user click), it does not need to wait for the next periodic UART frame to render data:
- In `OriginalTuner::onResume()`: Dispatches request query `forwardType(0x0F)` to `PeugeotDataController`.
  - `PeugeotDataController::forwardType(0x0F)` immediately pushes the cached `mCarRadioInfo` instance to the activity listener.
  - The UI immediately populates frequency, preset dials, RDS station name, indicators, and Radio Text.
- In `OriginalMediaPlayer::onResume()`: Dispatches request queries `forwardType(0x11)` and `forwardType(0x12)`.
  - `PeugeotDataController` immediately pushes cached `mCarCdPlayInfo`, rendering active disc slot, track, and elapsed time with zero latency.

