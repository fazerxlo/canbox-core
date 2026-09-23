# CAN Box Protocol Specification: BSI Vehicle Central Settings & Clock Sync
## Topic 10: Peugeot 407 Personalization, BSI Setup & GPS Clock Sync
**Document File:** `CANBOX_SPEC_HIWORLD_407_10_BSI_SETTINGS_CLOCK_SYNC.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** `https://github.com/fazerxlo/canbox/tree/main/doc/CAN2004_lights.md`, `CAN_messages.md`, `CAN2004_0x0F6.md`

---

# 1. Functional Domain & Architecture Overview

The Peugeot 407 Built-in Systems Interface (BSI) manages central vehicle personalization options, exterior lighting states, and cluster calendar/clock synchronization:
- **`0x128` (Cluster & Lighting Telemetry, D5 / Byte 4):** Definitively encodes the 4 exterior lighting states (`0x80` Side lights, `0xC0` Headlights low beam, `0xE0` Full beam, `0xA0` Transient full beam).
- **`0x036` (Dashboard Illumination):** Controls instrument cluster backlighting and auto-dimming when side lights are on.
- **`0x228` (Clock & Calendar Synchronization):** Broadcasts 8-byte date/time frames (`[Hours, Minutes, Day, Month, Year, Format, 00, 00]`) to synchronize the instrument cluster and dashboard clock.
- **Android GPS Clock Downlink (`Cmd 0xCB`):** After removing the factory RD4 radio, the CAN adapter decodes Android GPS Date/Time packets and synthesizes native PSA `0x228` frames to keep the vehicle cluster clock accurate.

```
+------------------------------------------------------------------------------------+
|                         Peugeot 407 BSI & Instrument Cluster                       |
|           [Lighting States (0x128), Illumination (0x036), Clock Sync (0x228)]      |
+------------------------------------------------------------------------------------+
                         │ (Uplink 0x128 / 0x036)   ▲ (Downlink Clock 0x228)
                         ▼                          │
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Captures lighting states from CAN 0x128 Byte 4 (D5)                           |
|   2. Decodes Android GPS Time Sync Packets (Hiworld Cmd 0xCB)                      |
|   3. Synthesizes PSA CAN 0x228 Clock Sync Frames to set Cluster Date/Time          |
|   4. Decodes Android "Car Settings" menu overrides (Hiworld Cmd 0x7B / 0x7D)       |
+------------------------------------------------------------------------------------+
                         │ (UART Telemetry)         ▲ (UART Commands)
                         ▼                          │
+------------------------------------------------------------------------------------+
|                     Android Headunit Car Settings & System Time                    |
|          (Provides toggle switches for BSI options and automatic GPS Time Sync)    |
+------------------------------------------------------------------------------------+
```

---

# 2. PSA CAN Bus Bitfield Specification

### 2.1 PSA Exterior Lighting State (`0x128` Byte 4 / D5)
- **CAN ID:** `0x128` (DLC: 8, Period: 100 ms)

```
+--------+--------+------------------------------------+-----------------------------+
| Byte   | Bit    | Function / Signal Name             | Bit Mask / Value Definition |
+--------+--------+------------------------------------+-----------------------------+
| Byte 4 | Bit 7  | Lights Switch Engaged (Any on)     | 0x80 (Side lights ON)       |
|        | Bit 6  | Low Beam (Headlights) Active       | 0x40 (Headlights ON: 0xC0)  |
|        | Bit 5  | High Beam (Full Beam) Active       | 0x20 (Full Beam ON: 0xE0)   |
|        | Bits 4..0| Reserved                         | 0x00                        |
+--------+--------+------------------------------------+-----------------------------+
```

| State | Byte 4 Value | Bits Set | Meaning |
|:---|:---:|:---:|:---|
| **OFF / AUTO** | `0x00` | `00000000` | All lights off / daytime |
| **Side Lights** | `0x80` | `10000000` | Parking / side lights on |
| **Headlights** | `0xC0` | `11000000` | Low beam dipped headlights on |
| **Full Beam** | `0xE0` | `11100000` | High beam / main headlights on |
| **Transient** | `0xA0` | `10100000` | Stalk spring-return transient |

### 2.2 PSA Instrument Cluster Clock & Calendar Frame (`0x228`)
- **CAN ID:** `0x228` (Standard 11-bit Identifier, DLC: 8)

```
+--------+--------+------------------------------------+-----------------------------+
| Byte   | Bit    | Function / Signal Name             | Value / Encoding Definition |
+--------+--------+------------------------------------+-----------------------------+
| Byte 0 | 7..0   | Hours (24-Hour Format)             | 0..23 Hours                 |
+--------+--------+------------------------------------+-----------------------------+
| Byte 1 | 7..0   | Minutes                            | 0..59 Minutes               |
+--------+--------+------------------------------------+-----------------------------+
| Byte 2 | 7..0   | Day of Month                       | 1..31 Days                  |
+--------+--------+------------------------------------+-----------------------------+
| Byte 3 | 7..0   | Month                              | 1..12 (1 = Jan, 12 = Dec)   |
+--------+--------+------------------------------------+-----------------------------+
| Byte 4 | 7..0   | Year (Offset from 2000)            | 0..99 (e.g. 26 = Year 2026) |
+--------+--------+------------------------------------+-----------------------------+
| Byte 5 | 7..0   | Time Format Flags                  | 0x00 = 24H, 0x01 = 12H AM/PM|
+--------+--------+------------------------------------+-----------------------------+
| Byte 6..7|7..0  | Reserved Padding                   | 0x00, 0x00                  |
+--------+--------+------------------------------------+-----------------------------+
```

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Android GPS Clock Broadcast Downlink (`Cmd 0xCB` / `Command.ForwardDateTimeSetting`)
When Android synchronizes system time with the vehicle:
- **Sync Header:** `0x5A 0xA5`
- **Length ($L$):** `0x06` (6 payload bytes)
- **Command ID:** `0xCB` (`203` unsigned / `-53` signed / `Command.ForwardDateTimeSetting`)
- **Payload Layout (6 Bytes):**
  - `Byte 0`: Year (0..99, e.g. `0x1A` = 2026)
  - `Byte 1`: Month (1..12)
  - `Byte 2`: Day (1..31)
  - `Byte 3`: Hour (0..23)
  - `Byte 4`: Minute (0..59)
  - `Byte 5`: Format Mode (`0x01` = 24H mode, `0x00` = 12H mode)
- **Checksum:** `((Length + CmdID + sum(Payload)) - 1) & 0xFF`
- **Total Frame Wire Length:** 11 bytes (`5A A5 06 CB [6 Bytes] Checksum`)

---

# 4. Pure C99 Firmware Implementation

```c
#ifndef CANBOX_BSI_CLOCK_H
#define CANBOX_BSI_CLOCK_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define PSA_CAN_ID_CLOCK_SYNC 0x228

typedef void (*canbox_can_tx_fn)(uint32_t id, const uint8_t *data, uint8_t dlc);

/* Process Inbound Android Time Setting (Cmd 0xCB) -> Inject PSA CAN 0x228 */
static inline void psa_bsi_handle_android_time(const uint8_t *payload, uint8_t len, canbox_can_tx_fn can_tx) {
    if (!can_tx || len < 6) return;

    uint8_t year   = payload[0]; /* 0..99 */
    uint8_t month  = payload[1]; /* 1..12 */
    uint8_t day    = payload[2]; /* 1..31 */
    uint8_t hour   = payload[3]; /* 0..23 */
    uint8_t minute = payload[4]; /* 0..59 */
    uint8_t format = payload[5]; /* 1=24H */

    uint8_t can_frame[8];
    can_frame[0] = hour;
    can_frame[1] = minute;
    can_frame[2] = day;
    can_frame[3] = month;
    can_frame[4] = year;
    can_frame[5] = (format == 1) ? 0x00 : 0x01;
    can_frame[6] = 0x00;
    can_frame[7] = 0x00;

    can_tx(PSA_CAN_ID_CLOCK_SYNC, can_frame, 8);
}

#endif /* CANBOX_BSI_CLOCK_H */
```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: Headunit Synchronizes GPS Clock (23 Sept 2026, 16:30, 24H)
- **Downlink Serial Received (Host $\to$ CAN Box):**
  - Frame: `5A A5 06 CB 1A 09 17 10 1E 01 4F`
  - Checksum Calculation: `(0x06 + 0xCB + 0x1A + 0x09 + 0x17 + 0x10 + 0x1E + 0x01 - 1) & 0xFF = 0x4F`
- **Expected PSA CAN Injection (`0x228`):**
  - Frame: `228#101E17091A000000`
