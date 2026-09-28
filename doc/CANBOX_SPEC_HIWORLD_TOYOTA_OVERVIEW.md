# Toyota Hiworld (wc) Protocol Overview

This document provides a comprehensive mapping of the Hiworld UART command headers (`0x5A 0xA5`) used by the Toyota parser (`ToyotaDataParser`) inside `QF_Canbus.apk`.

If you select `[Hiworld] - [Toyota]` in the factory settings, the Head Unit will expect incoming CAN box frames to use the IDs in the **Receive Table**, and it will send commands using the IDs in the **Transmit Table**.

## 1. Receive (CAN Box -> Head Unit)
These are the packet IDs your C99 firmware must generate and send over the UART to update the Android UI.

| Hex ID | Decimal (Java) | Internal Method | Description |
| :--- | :--- | :--- | :--- |
| `0x11` | 17 | `parseSteeringWheelKeyAngle` | Steering wheel buttons, steering angle, SOS button. |
| `0x13` | 19 | `parseBasicTripInfo` | Basic Trip Computer (Instant fuel, Range, Avg Speed). |
| `0x16` | 22 | `parseHistoryFuelConsumption` | Historical fuel consumption logs. |
| `0x17` | 23 | `parsePast15MinutesFuelConsumption`| 15-minute rolling fuel consumption data. |
| `0x1F` | 31 | `parseVehicheData` | Speed, RPM telemetry. |
| `0x21` | 33 | `parsePanelKey` | Physical dashboard multimedia keys. |
| `0x22` | 34 | `parseKnob` | Rotary knob events. |
| `0x23` | 35 | `parseAcPanel` | Physical AC panel keys. |
| `0x31` | 49 | `parseAcState2` | Extended AC (Rear AC, seat heaters, coolers). |
| `0x32` | 50 | `parseCarBodyState` | Handbrake, seatbelts, engine RPM, battery voltage, doors. |
| `0x37` | 55 | `parseAcState3` | Advanced airflow routing. |
| `0x41` | 65 | `parseRadarState` | Parking sensors (Front/Rear proximity zones). |
| `0x48` | 72 | `parseTyreState` | **Numeric TPMS** (Status, FL, FR, RL, RR, Spare). |
| `0x62` | 98 | `parseCentralState` | Car settings states (Lights, Locks, Wipers, Radar switch). |
| `0x82` | -126 | `parseAcState` | Basic Climate Control (Fan speed, Temp, AC clutch). |
| `0x84` | -124 | `parseCarRadioState` | OEM Radio screen status. |
| `0x86` | -122 | `parseCarMediaState` | CD/Media player screen status. |
| `0xA6` | -90 | `parseDspState` | OEM Amplifier / DSP equalizer status. |
| `0xA8` | -88 | `parseCarMediaTextInfo` | RDS text / ID3 tags for radio/CD. |
| `0xC5` | -59 | `parseBtKeyInfo` | Bluetooth telephony keys. |
| `0xE0` | -32 | `parseSourceInfo` | Current active audio source. |
| `0xE8` | -24 | `parseCemeraState` | Reverse Camera / 360 camera triggers. |
| `0xF0` | -16 | `parseCanboxVersion` | CAN box firmware version string. |


## 2. Transmit (Head Unit -> CAN Box)
When the user interacts with the Android UI (tapping buttons, moving sliders), the Head Unit sends these commands down to your CAN box.

| Hex ID | Decimal (Java) | Internal Constant | Description |
| :--- | :--- | :--- | :--- |
| `0x24` | 36 | `ForwardCarType` | Initialization / Car variant selected. |
| `0x2A` | 42 | `ForwardAcPanel` | AC physical panel commands. |
| `0x2C` | 44 | `ForwardTouchInfo` | Screen touch coordinates (sometimes used for 360 camera). |
| `0x3D` | 61 | `ForwardAcSetting` | AC adjustments from the touchscreen. |
| `0x6A` | 106 | `ForwardCentralSetting` | Car settings adjustments (Lights, Locks). |
| `0x91` | -111 | `ForwardCarSource` | Change active audio source. |
| `0x92` | -110 | `ForwardMediaTrackName` | Android track name to display on OEM screen. |
| `0x93` | -109 | `ForwardMediaAlbumName` | Android album name to display on OEM screen. |
| `0x94` | -108 | `ForwardMediaArtistName` | Android artist name to display on OEM screen. |
| `0x97` | -105 | `ForwardMediaFoldName` | Android folder name. |
| `0x9A` | -102 | `ForwardSystemLanguage` | Sync language to CAN box / Dashboard. |
| `0xAD` | -83 | `ForwardDspStateSetting` | **DSP Adjustments** (EQ sliders, Balance, Fader). |
| `0xC4` | -60 | `ForwardBtSourceName` | Bluetooth metadata. |
| `0xCB` | -53 | `ForwardSystemTimeSetting` | Time synchronization. |
| `0xCD` | -51 | `ForwardBtSourceNumber` | Bluetooth caller ID/number. |
| `0xF2` | -14 | `ForwardCdSourceSetting` | CD player commands (Next, Prev, Play, Pause). |
| `0xF3` | -13 | `ForwardSrcControl` | Source control triggers. |
| `0xFA` | -6 | `ForwardCemeraSetting` | Camera view settings. |
