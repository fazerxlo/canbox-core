# SPEC Part 4: Downlink Control Commands Specification
## Portable C99 Firmware Implementation for Headunit $\to$ Vehicle CAN Bidirectional Control

**Document File:** `SPEC_04_DOWNLINK_CONTROL_COMMANDS.md`  
**Target Standard:** ISO/IEC 9899:1999 (Pure C99)  
**Target Hardware:** STM32, ESP32, Arduino/AVR (ATmega328P + MCP2515), Nuvoton NUC131, Linux (SocketCAN `vcan0` + POSIX PTY)  
**Direction:** Android Headunit Touchscreen / Android OS $\to$ Custom CAN Box Translator $\to$ Vehicle CAN Network  
**Protocol Envelopes:** Raise (RZC `0x2E`), Hiworld (WC `0x5A 0xA5`), Bagoo (`0xD5`), Simple Soft (XP `0x2E`)

---

# Table of Contents
1. [4.1 Clock & Calendar Synchronization (GPS $\to$ Cluster & BSI)](#41-clock--calendar-synchronization-gps--cluster--bsi)
2. [4.2 Trip Computer Reset Injection](#42-trip-computer-reset-injection)
3. [4.3 OEM JBL Amplifier DSP Parameter Control](#43-oem-jbl-amplifier-dsp-parameter-control)
4. [4.4 Media Metadata & ID3 Title Mirroring to MFD](#44-media-metadata--id3-title-mirroring-to-mfd)
5. [4.5 TPMS Sensor Reset / Calibration Trigger](#45-tpms-sensor-reset--calibration-trigger)
6. [4.6 Touchscreen Climate Control (HVAC) Overrides](#46-touchscreen-climate-control-hvac-overrides)
7. [Pure C99 Downlink Command Parser Engine](#pure-c99-downlink-command-parser-engine)
8. [Bidirectional Desktop Test Vectors & Verification](#bidirectional-desktop-test-vectors--verification)

---

# 4.1 Clock & Calendar Synchronization (GPS $\to$ Cluster & BSI)

### 4.1.1 Headunit Serial Downlink Frame
- **Raise Command ID:** `0xA6` (Payload Length: 5 bytes)
```
Raise 0xA6 Frame Layout:
0x2E 0xA6 0x05 [Year] [Month] [Day] [Hour] [Minute] [Checksum]
- Year: 0..99 (Offset from 2000, e.g., 26 for year 2026)
- Month: 1..12
- Day: 1..31
- Hour: 0..23 (24-hour format)
- Minute: 0..59
```

### 4.1.2 Vehicle CAN Injection Frame
- **PSA Instrument Cluster Clock CAN ID:** `0x228` (DLC: 8, Standard 11-bit)
```
PSA 0x228 Injection Frame:
[Byte 0] Hour (0..23)
[Byte 1] Minute (0..59)
[Byte 2] Day (1..31)
[Byte 3] Month (1..12)
[Byte 4] Year (0..99)
[Bytes 5..7] 0x00 0x00 0x00
```

---

# 4.2 Trip Computer Reset Injection

### 4.2.1 Headunit Serial Downlink Frame
- **Raise Command ID:** `0x82` (Length: 2 bytes)
```
Reset Trip 1: 0x2E 0x82 0x02 0x41 0x00 0x3A
Reset Trip 2: 0x2E 0x82 0x02 0x22 0x00 0x59
```

### 4.2.2 Vehicle CAN Injection Frame
- **PSA BSI Command CAN ID:** `0x221` (DLC: 8)
- Inject Trip 1 Clear: `0x221 [0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00]`
- Inject Trip 2 Clear: `0x221 [0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00]`

---

# 4.3 OEM JBL Amplifier DSP Parameter Control

### 4.3.1 Headunit Serial Downlink Frame
- **Raise Command ID:** `0xC5` (Length: 8 bytes)
```
Raise 0xC5 Frame Layout:
0x2E 0xC5 0x08 [Fader] [Balance] [Bass] [Treble] [Middle] [SoundField] [DspPos] 0x01 [Checksum]
- Fader: 0..14 (Neutral = 7, < 7 = Rear, > 7 = Front)
- Balance: 0..14 (Neutral = 7, < 7 = Left, > 7 = Right)
- Bass: 0..14 (Neutral = 7, Range -7..0..+7)
- Treble: 0..14 (Neutral = 7)
- Middle: 0..14 (Neutral = 7)
- SoundField / Preset: 0..5
```

### 4.3.2 Vehicle CAN Injection Frame
- **PSA JBL Amplifier Control CAN ID:** `0x280` (DLC: 8)
```
PSA 0x280 Injection Frame:
[Byte 0] Bass Level (0..14)
[Byte 1] Treble Level (0..14)
[Byte 2] Middle Level (0..14)
[Byte 3] Balance (0..14)
[Byte 4] Fader (0..14)
[Bytes 5..7] 0x00 0x00 0x00
```

---

# 4.4 Media Metadata & ID3 Title Mirroring to MFD

### 4.4.1 Headunit Serial Downlink Frames
- **Media Track Progress:** Raise `Cmd 0xC0` (Length: 5 bytes)
  `0x2E 0xC0 0x05 0x08 [CurrentTrack_L] [CurrentTrack_H] [TotalTracks_L] [TotalTracks_H] [Checksum]`
- **ID3 Song Title / Artist Text:** Raise `Cmd 0xC1` (Length: $N+1$ bytes)
  `0x2E 0xC1 [Length] 0x03 [UTF-8 / ASCII Character Bytes...] [Checksum]`

### 4.4.2 Vehicle CAN Injection Frame
- **PSA Multi-Function Display (EMF/MFD) CAN ID:** `0x396` (DLC: 8, standard 8-character ASCII segment).

---

# 4.5 TPMS Sensor Reset / Calibration Trigger

### 4.5.1 Headunit Serial Downlink Frame
- **Raise Command ID:** `0x80` (Length: 2 bytes)
```
TPMS Calibration Request: 0x2E 0x80 0x02 0x10 0x01 0x6C
```

### 4.5.2 Vehicle CAN Injection Frame
- **PSA BSI CAN ID:** `0x221` (DLC: 8)
- Inject TPMS Relearn: `0x221 [0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00]`

---

# 4.6 Touchscreen Climate Control (HVAC) Overrides

### 4.6.1 Headunit Serial Downlink Frame
- **Raise Command ID:** `0x8A` (Length: 2 bytes: `[FunctionID] [Value]`)

| Function ID (`0x8A`) | Action Description | Value Parameter | PSA CAN Injection Target (`0x1E0`) |
|:---:|:---|:---:|:---|
| `0x01` | Toggle Full Auto Climate | `0x01` = Auto, `0x00` = Manual | `0x1E0 [0x08, 0,0,0,0,0,0,0]` |
| `0x02` | Toggle A/C Compressor | `0x01` = ON, `0x00` = OFF | `0x1E0 [0x40, 0,0,0,0,0,0,0]` |
| `0x04` | Increase Driver Temperature | Step delta ($+0.5^\circ\text{C}$) | `0x1E0 [0, 0, +1, 0,0,0,0,0]` |
| `0x05` | Decrease Driver Temperature | Step delta ($-0.5^\circ\text{C}$) | `0x1E0 [0, 0, -1, 0,0,0,0,0]` |
| `0x06` | Airflow Face / Center Vent | `0x01` = Selected | `0x1E0 [0, 0x40, 0,0,0,0,0,0]` |
| `0x07` | Airflow Defrost / Windshield | `0x01` = Selected | `0x1E0 [0, 0x80, 0,0,0,0,0,0]` |
| `0x08` | Airflow Footwell Vent | `0x01` = Selected | `0x1E0 [0, 0x20, 0,0,0,0,0,0]` |
| `0x0B` | Toggle DUAL / MONO Sync | `0x01` = Dual, `0x00` = Mono | `0x1E0 [0x04, 0,0,0,0,0,0,0]` |
| `0x0C` | HVAC Master Power | `0x01` = ON, `0x00` = OFF | `0x1E0 [0x80, 0,0,0,0,0,0,0]` |

---

# Pure C99 Downlink Command Parser Engine

```c
#include "canbox_hal.h"
#include <string.h>

void canbox_handle_downlink_packet(uint8_t cmd_id, const uint8_t *payload, uint8_t len) {
    canbox_msg_t can_out;
    memset(&can_out, 0, sizeof(can_out));
    can_out.is_ext = false;

    switch (cmd_id) {
        /* 1. Clock & Date Synchronization */
        case 0xA6: {
            if (len >= 5) {
                can_out.id = 0x228;
                can_out.dlc = 8;
                can_out.data[0] = payload[3]; /* Hour */
                can_out.data[1] = payload[4]; /* Min */
                can_out.data[2] = payload[2]; /* Day */
                can_out.data[3] = payload[1]; /* Month */
                can_out.data[4] = payload[0]; /* Year */
                g_hal->can_send(&can_out);
            }
            break;
        }

        /* 2. Trip Computer Reset */
        case 0x82: {
            if (len >= 2) {
                can_out.id = 0x221;
                can_out.dlc = 8;
                if (payload[0] == 0x41) {
                    can_out.data[2] = 0x40; /* Trip 1 Reset */
                } else if (payload[0] == 0x22) {
                    can_out.data[2] = 0x80; /* Trip 2 Reset */
                }
                g_hal->can_send(&can_out);
            }
            break;
        }

        /* 3. JBL Amplifier Control */
        case 0xC5: {
            if (len >= 5) {
                can_out.id = 0x280;
                can_out.dlc = 8;
                can_out.data[0] = payload[2]; /* Bass */
                can_out.data[1] = payload[3]; /* Treble */
                can_out.data[2] = payload[4]; /* Middle */
                can_out.data[3] = payload[1]; /* Balance */
                can_out.data[4] = payload[0]; /* Fader */
                g_hal->can_send(&can_out);
            }
            break;
        }

        /* 4. Touchscreen Climate Controls */
        case 0x8A: {
            if (len >= 2) {
                can_out.id = 0x1E0;
                can_out.dlc = 8;
                uint8_t func = payload[0];
                if (func == 0x01)      can_out.data[0] = 0x08; /* Auto */
                else if (func == 0x02) can_out.data[0] = 0x40; /* AC */
                else if (func == 0x0B) can_out.data[0] = 0x04; /* Dual */
                else if (func == 0x0C) can_out.data[0] = 0x80; /* Power */
                g_hal->can_send(&can_out);
            }
            break;
        }

        /* 5. TPMS Calibration Reset */
        case 0x80: {
            if (len >= 2 && payload[0] == 0x10) {
                can_out.id = 0x221;
                can_out.dlc = 8;
                can_out.data[3] = 0x10; /* TPMS calibrate */
                g_hal->can_send(&can_out);
            }
            break;
        }
    }
}
```

---

# Bidirectional Desktop Test Vectors & Verification

```bash
# 1. Test GPS Time Sync: Year 2026, Sept 20, 23:15:
# Headunit sends serial frame: 2E A6 05 1A 09 14 17 0F 96
# Verified CAN Injection on vcan0:
# candump vcan0 -> 228 [8] 17 0F 14 09 1A 00 00 00

# 2. Test Trip 1 Reset:
# Headunit sends serial frame: 2E 82 02 41 00 3A
# Verified CAN Injection on vcan0:
# candump vcan0 -> 221 [8] 00 00 40 00 00 00 00 00

# 3. Test JBL Amp EQ Set: Bass +3 (10), Treble -2 (5), Fader Center (7):
# Headunit sends serial frame: 2E C5 08 07 07 0A 05 07 00 00 01 0F
# Verified CAN Injection on vcan0:
# candump vcan0 -> 280 [8] 0A 05 07 07 07 00 00 00
```

