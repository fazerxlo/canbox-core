The **PSA RD4 radio** (produced primarily by Blaupunkt / Bosch / Continental) is the standard audio head unit used across PSA Peugeot-Citroën vehicles built on the **CAN2004 (AEE2004)** architecture (e.g., Peugeot 407, 207, 307 facelift, 308, Citroën C4, C5 II).

It operates on the **PSA Comfort / Infotainment CAN bus (`CAN-INFO` / `CAN Confort`) at 125 kbps**.

---

### Architectural Context: RD4 vs. RT3/RT4/RT6

Unlike systems with integrated colour displays and navigation (such as RT4/RT5/WIP Nav), the **RD4 has no built-in screen**. It is paired with an external monochrome Multifunction Display (**EMF-A, EMF-C, or EMF-CT**). 

Because of this split architecture:
1. **RD4 broadcasts display frames** over CAN so the EMF screen can render tuner frequencies, station names, RDS text, audio settings, and volume bars.
2. **RD4 receives control frames** from steering column stalks and front panel buttons.
3. **RD4 receives vehicle status frames** for ignition power management, anti-theft VIN pairing, and audio synthesis (warning chimes and parking sensor beeps).

Detailed signal specifications in the project are documented in [doc/CAN2004_radio.md](file:///home/Fazer/git/simulator/doc/CAN2004_radio.md) and defined in [signal-db/radio.yaml](file:///home/Fazer/git/simulator/signal-db/radio.yaml).

---

## 1. Frames Transmitted by the RD4 (Head Unit $\to$ Display / Bus)

### `0x165` — Audio Source / Input Status (`ETAT_AUTORADIO`)
* **Period:** 50 ms  
* **DLC:** 4 bytes  
* **Purpose:** Informs the EMF display which audio source is currently active.

| Byte | Bit(s) | Signal | Values / Meaning |
|:---:|:---:|---|---|
| **0** | 7:0 | Status byte 1 | Constant `0xCC` |
| **1** | 7:0 | Status byte 2 | Constant `0x54` |
| **2** | 7:4 | **INPUT_SOURCE** | `0x1` = Tuner (FM/AM)<br>`0x2` = Internal CD<br>`0x3` = CD Changer (CDC)<br>`0x4` = AUX 1 (Glovebox / Phone)<br>`0x5` = AUX 2<br>`0x6` = USB Media<br>`0x7` = Bluetooth Audio |
| **2** | 3:0 | Sub-status | Typically `0x00` or `0x01` |
| **3** | 7:0 | Status byte 4 | Constant `0x02` |

---

### `0x1A5` — Radio Volume (`VOLUME_RADIO`)
* **Period:** 100 ms (or event-driven on knob turn / stalk press)  
* **DLC:** 1 byte  
* **Purpose:** Transmits current master volume level and animation triggers.

| Byte | Bit(s) | Signal | Description |
|:---:|:---:|---|---|
| **0** | 7:5 | **VOLFLAG** | `0xE0` = Volume stable / idle.<br>`0x00` = Volume changing (triggers the on-screen volume pop-up bar on the EMF for ~2 seconds). |
| **0** | 4:0 | **VOLUME** | Volume step from **0 to 30** (`0x00` to `0x1E`). |

> [!NOTE]
> On RT4 color navigation units, `0x1A5` was repurposed for CD/Jukebox elapsed playback time. On the RD4, it is strictly the volume step.

---

### `0x1E5` — Audio Tone Settings & Equalizer (`REGLAGES_SON`)
* **Period:** 100 ms  
* **DLC:** 7 bytes  
* **Purpose:** Carries audio tone adjustments (balance, fader, bass, treble, loudness, auto-volume) and indicates which audio adjustment menu is open on screen.

| Byte | Bit(s) | Signal | Description / Formula |
|:---:|:---:|---|---|
| **0** | 7 | Menu: L/R Balance | `1` = Left/Right balance menu open on EMF |
| **0** | 6:0 | Left / Right Balance | `0x3F` = Centre (0); `< 0x3F` = Left; `> 0x3F` = Right (range -9..+9) |
| **1** | 7 | Menu: F/R Fader | `1` = Front/Rear fader menu open |
| **1** | 6:0 | Front / Rear Fader | `0x3F` = Centre (0); `> 0x3F` = Front; `< 0x3F` = Rear (e.g., `0x46` = Front +7) |
| **2** | 7 | Menu: Bass | `1` = Bass tone adjustment menu open |
| **2** | 6:0 | Bass Level | `0x3F` = Flat (0 dB); `< 0x3F` = Cut; `> 0x3F` = Boost (range -9..+9, e.g. `0x48` = +9) |
| **3** | 7:0 | Reserved / Neutral | Typically `0x3F` (or `0x00`) |
| **4** | 7 | Menu: Treble | `1` = Treble adjustment menu open |
| **4** | 6:0 | Treble Level | `0x3F` = Flat (0 dB); `< 0x3F` = Cut; `> 0x3F` = Boost (range -9..+9, e.g. `0x48` = +9) |
| **5** | 7 | Menu: Loudness | `1` = Loudness menu open |
| **5** | 6 | Loudness State | `1` = Loudness enabled, `0` = Disabled |
| **5** | 4 | Menu: Auto-Volume | `1` = Speed-dependent auto-volume menu open |
| **5** | 2:0 | Auto-Volume Threshold | `0x07` = Enabled / Level 7; `0x00` = Disabled |
| **6** | 6 | Menu: Equalizer | `1` = Musical ambiance menu open |
| **6** | 5:0 | Equalizer Preset | `0x03` = None / Flat<br>`0x07` = Classical<br>`0x0B` = Jazz-Blues<br>`0x0F` = Pop-Rock<br>`0x13` = Vocal<br>`0x17` = Techno |

---

### `0x225` — FM/AM Tuner Status & Frequency (`ETAT_TUNER`)
* **Period:** 100 ms  
* **DLC:** 5 to 8 bytes  
* **Purpose:** Carries active frequency, tuner waveband, preset memory, and RDS reception flags.

| Byte | Bit(s) | Signal | Description |
|:---:|:---:|---|---|
| **0** | 7 | LIST | Station list mode active |
| **0** | 6 | SCAN | Scan tuning mode active |
| **0** | 5 | RDS | RDS subcarrier locked / RDS data available |
| **0** | 4 | PTY | Program Type (PTY) standby mode active |
| **0** | 3 | TUN | Tuner actively seeking / tuning |
| **0** | 2 | TA | Traffic Announcement priority active |
| **0** | 1:0 | TUNDIR | Tuning direction (`0` = None, `1` = Up, `2` = Down) |
| **1** | 7:0 | MEMORY | Preset station number (`0` = manual tuning, `1`..`6` or `0x10`..`0x60`) |
| **2** | 7:0 | BAND | `0x10` / `0x90` = FM Band 1<br>`0x20` / `0xA0` = FM Band 2<br>`0x40` / `0xC0` = FM Auto-Store (AST)<br>`0x50` / `0xD0` = AM (Medium Wave) |
| **3–4** | 15:0 | FREQUENCY | 16-bit uint (Big-Endian):<br>$$\text{Frequency (MHz)} = (\text{raw} \times 0.05) + 50.0$$<br>*(e.g., raw `0x0398` = 920 $\to$ $920 \times 0.05 + 50.0 = 96.0\text{ MHz}$)* |

---

### `0x265` — RDS & Station Info Flags (`INFO_TUNER`)
* **Period:** 100 ms  
* **DLC:** 4 bytes  
* **Byte 0:** Status flags: Bit 5 = TA (Traffic Announcement), Bit 4 = TP (Traffic Programme), Bit 2 = RDS PS name valid, Bit 0 = PTY valid.  
* **Byte 1:** Head unit hardware flags (`0xE0`).  
* **Byte 2:** Substatus (`0x01`).  
* **Byte 3:** Media mode: `0x00` = FM Tuner, `0x01` = CD / CDC.

---

### `0x2A5` — RDS Station Name (`NOM_STATION`)
* **Period:** 100 ms (or event-driven on RDS PS decode)  
* **DLC:** 8 bytes  
* **Payload:** 8 ASCII characters carrying the RDS Program Service (PS) name (e.g., `" RMF FM "`, `"RADIO 1 "`). Left-aligned and padded with spaces or `0x00`.

---

### `0x0A4` — RDS RadioText (`TEXTE_RADIO`)
* **Period:** Event-driven multi-frame stream (~100–500 ms)  
* **DLC:** 8 bytes per segment  
* **Transport:** Segmented using **ISO 15765-2 (ISO-TP)** to transmit free-form station RadioText up to 64 characters (scrolling song titles, artist info):
  - **Single Frame (SF)** (`0x0N`): When text is $\le 7$ characters.
  - **First Frame (FF)** (`0x1H 0xLL`): Total length header + first 6 characters.
  - **Consecutive Frame (CF)** (`0x2N`): Sequence chunks of 7 characters each.
  - Real captures show an internal 4-byte control prefix (`10 00 00 00`) prepended before the 64-byte text payload.

---

### Other RD4 Transmission Frames
* **`0x1E0` (Head Unit Presence / Heartbeat):** Period 100 ms; observed bench payload `24 00 00 00 20`.
* **`0x125` (CD Track & Station Lists):** ISO-TP segmented frame broadcasting the current station list or CD track titles to the EMF scroll menu.
* **`0x325`, `0x365`, `0x3A5` (CD Drive Status):** Disc presence in slot, CD mechanism status, track number and elapsed playback time.

---

## 2. Frames Received & Processed by the RD4 (Inputs)

| CAN ID | Transmitter | Name / Function | Key Signals Processed by RD4 |
|:---:|:---:|---|---|
| **`0x036`** | BSI | Power Management & Illumination | **Byte 4:** Power mode (`0x01` ACC, `0x02` IGN, `0x05` Engine run). RD4 powers on/off based on this.<br>**Byte 2 bit 7:** Economy Mode (turns off radio after timeout to prevent battery drain).<br>**Byte 3:** Dimmer level and day/night palette flag. |
| **`0x21F`** | COM2000 (Stalk) | Steering Column Remote Controls | **Byte 0:** Key mask (`0x08` Vol +, `0x04` Vol -, `0x02` Source toggle, `0x80` Next/Seek +, `0x40` Prev/Seek -).<br>**Byte 1:** Scroll wheel encoder pulses. |
| **`0x3E5`** | Steering Wheel / Console | Panel Buttons | Button presses for `MENU`, `MODE`, `AUDIO`, `OK`, `ESC`, `TRIP`, `CLIM`, and arrow keys (`UP`, `DOWN`, `LEFT`, `RIGHT`). |
| **`0x2B6`** | BSI | Chassis VIN / VIS Broadcast | BSI broadcasts bytes 10–17 of the VIN (chassis number). The RD4 compares this against its internal EEPROM. If missing or mismatched, **the RD4 emits a periodic anti-theft beep** every few seconds. |
| **`0x1A1`** | BSI | Vehicle Warnings & Chimes | Requests audio chime synthesis through car speakers for door open warnings, ice alert, overspeed warning, etc. |
| **`0x0E1`** | AAS ECU | Parking Sensor Assistance | Sensor distance zones and buzzer cadence (`BUZZER_CADENCE`); RD4 synthesizes parking radar beeps through front and rear speakers. |
| **`0x131` / `0x1A0`** | CDC / Emulator | CD Changer Bus | Commands and status for external Blaupunkt 5-CD changer or digital audio interfaces (e.g. Yatour). |

---

## Summary Reference Table

| CAN ID | Dir | Name / Purpose | Period | Key Payload Content |
|:---:|:---:|---|:---:|---|
| **`0x0A4`** | RD4 $\to$ EMF | RDS RadioText | Event | ISO-TP multi-frame (up to 64 chars text) |
| **`0x165`** | RD4 $\to$ EMF | Source Status | 50 ms | Byte 2[7:4]: TUN (1), CD (2), CDC (3), AUX1 (4), AUX2 (5), BT (7) |
| **`0x1A5`** | RD4 $\to$ EMF | Master Volume | 100 ms | Byte 0[4:0]: Volume 0–30; Byte 0[7:5]: `0xE0` stable / `0x00` changing |
| **`0x1E0`** | RD4 $\to$ Bus | Presence Heartbeat | 100 ms | `24 00 00 00 20` |
| **`0x1E5`** | RD4 $\to$ EMF | Tone Settings & Menus | 100 ms | Bal, Fad, Bass, Treble, Loudness, Ambiance & active menu flags |
| **`0x225`** | RD4 $\to$ EMF | Tuner & Frequency | 100 ms | Band (FM1/FM2/AST/AM), Preset 1–6, Frequency ($raw \times 0.05 + 50$) |
| **`0x265`** | RD4 $\to$ EMF | Tuner Flags | 100 ms | RDS valid, TA/TP flags, source indicators |
| **`0x2A5`** | RD4 $\to$ EMF | Station Name (RDS PS) | 100 ms | 8 ASCII characters (station name) |
| **`0x036`** | BSI $\to$ RD4 | Power / Illumination | 50 ms | Ignition state, Economy mode, Dimmer |
| **`0x21F`** | Stalk $\to$ RD4 | Remote Stalk Buttons | 100 ms | Vol+/-, Source, Track+/-, Rotary scroll wheel |
| **`0x2B6`** | BSI $\to$ RD4 | VIN Anti-Theft | 500 ms | Chassis serial (prevents anti-theft beep) |