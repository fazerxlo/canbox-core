# CAN Box Protocol Specification: BSI Vehicle Central Settings & Clock Sync
## Topic 10: Peugeot 407 Personalization, BSI Setup & GPS Clock Sync
**Document File:** `CANBOX_SPEC_HIWORLD_407_10_BSI_SETTINGS_CLOCK_SYNC.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** `https://github.com/fazerxlo/canbox/tree/main/doc/CAN2004_lights.md`, `CAN_messages.md`, `CAN2004_0x0F6.md`

---

# 1. Functional Domain & Architecture Overview

The Peugeot 407 Built-in Systems Interface (BSI) manages all vehicle convenience features, personalization options, and cluster date/time synchronization:
- **Vehicle Personalization Options:** Daytime running lights (DRL), follow-me-home headlamp timers, auto door locking on driving, automatic rear wiper in reverse, and auto-folding wing mirrors.
- **GPS Date & Time Clock Synchronization:** Because aftermarket Android installations remove the factory RD4 headunit, the instrument cluster and dashboard clock lose manual clock adjustment buttons. The CAN box decodes Android GPS Date/Time broadcast packets (`Cmd 0xA6`) and injects native PSA Clock Sync CAN frames (`CAN ID 0x228`) to synchronize the car's internal calendar and cluster clock.
The Peugeot 407 Built-in Systems Interface (BSI) manages central vehicle personalization options, exterior lighting states, and cluster calendar/clock synchronization:
- **`0x128` (Cluster & Lighting Telemetry, D5 / Byte 4):** Definitively encodes the 4 exterior lighting states (`0x80` Side lights, `0xC0` Headlights low beam, `0xE0` Full beam, `0xA0` Transient full beam).
- **`0x036` (Dashboard Illumination):** Controls instrument cluster backlighting and auto-dimming when side lights are on.
- **`0x228` (Clock & Calendar Synchronization):** Broadcasts 8-byte date/time frames (`[Hours, Minutes, Day, Month, Year, Format, 00, 00]`) to synchronize the instrument cluster and dashboard clock.
- **Android GPS Clock Downlink (`Cmd 0xA6`):** After removing the factory RD4 radio, the CAN adapter decodes Android GPS Date/Time packets and synthesizes native PSA `0x228` frames to keep the vehicle cluster clock accurate.

```
+------------------------------------------------------------------------------------+
|                         Peugeot 407 BSI & Instrument Cluster                       |
|           [Vehicle Settings, Lighting Timers, Units, Cluster Clock Calendar]       |
|           [Lighting States (0x128), Illumination (0x036), Clock Sync (0x228)]      |
+------------------------------------------------------------------------------------+
                         │ (Uplink 0x221 Settings)  ▲ (Downlink Clock 0x228 / BSI 0x221)
                         │ (Uplink 0x128 / 0x036)   ▲ (Downlink Clock 0x228)
                         ▼                          │
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Captures BSI configuration states & encodes Hiworld 0x38 Telemetry            |
|   1. Captures lighting states from CAN 0x128 Byte 4 (D5)                           |
|   2. Decodes Android GPS Time Sync Packets (Hiworld Cmd 0xA6)                      |
|   3. Synthesizes PSA CAN 0x228 Clock Sync Frames to set Cluster Date/Time          |
|   4. Decodes Android "Car Settings" menu overrides (Hiworld Cmd 0x80 / 0xC6)       |
|   4. Decodes Android "Car Settings" menu overrides (Hiworld Cmd 0x80)              |
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

### 2.1 PSA Instrument Cluster Clock & Calendar Frame (`0x228`)
- **CAN ID:** `0x228` (Standard 11-bit Identifier)
- **DLC:** 8 bytes
- **Transmission:** Event-triggered on sync request or periodic 60 s
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

### 2.2 BSI Personalization Settings (`0x221`)
- **CAN ID:** `0x221` (DLC: 8)
- `Byte 1 Bit 7`: Auto rear wiper in reverse ($1 = \text{On}$).
- `Byte 1 Bit 4`: Auto door lock when driving $> 10\text{ km/h}$ ($1 = \text{On}$).
- `Byte 2 Bit 7`: Daytime Running Lights DRL ($1 = \text{On}$).
- `Byte 2 Bit 0`: Adaptive directional cornering headlights ($1 = \text{On}$).
- `Byte 4 Bits 7..6`: Follow-Me-Home Headlamp Delay (`00`=0s, `01`=15s, `10`=30s, `11`=60s).
- `Byte 4 Bit 3`: Auto power mirror folding on remote locking ($1 = \text{On}$).

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Downlink Android GPS Date/Time Sync (`Cmd 0xA6`)
When Android updates its network/GPS time or on system boot:
- **Sync Header:** `0x5A 0xA5`
- **Length:** `0x06` (1-byte Cmd + 5-byte Payload)
- **Command ID:** `0xA6`
- **Sync Header:** `0x5A 0xA5` | **Length:** `0x06` | **Cmd:** `0xA6`
- **Payload Layout (5 Bytes):**
  - `Byte 0`: Year ($0 \dots 99$, Year minus 2000)
  - `Byte 1`: Month ($1 \dots 12$)
  - `Byte 2`: Day ($1 \dots 31$)
  - `Byte 3`: Hour ($0 \dots 23$)
  - `Byte 4`: Minute ($0 \dots 59$)
- **Checksum:** 8-bit sum modulo 256 over `Length + CmdID + Payload`.

### 3.2 Downlink BSI Feature Settings Override (`Cmd 0x80`)
When the user flips a switch in the Android Car Settings menu:
- **Format:** `5A A5 04 80 [SettingIndex] [Value] [CS]`
- **Setting Indices:**
  - `0x01`: DRL Toggle (`0x01`=Enable, `0x00`=Disable)
  - `0x02`: Follow-Me-Home Duration (`0`=0s, `1`=15s, `2`=30s, `3`=60s)
  - `0x03`: Auto Lock Driving Toggle
  - `0x04`: Auto Rear Wiper Toggle
  - `0x05`: Auto Mirror Fold Toggle
### 3.2 Downlink BSI Personalization Settings (`Cmd 0x80`)
- **Format:** `5A A5 04 80 [SettingID] [Value] [CS]`

---

# 4. Pure C99 Firmware Implementation

```c
#ifndef CANBOX_BSI_H
#define CANBOX_BSI_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

typedef enum {
    PSA_LIGHTS_OFF        = 0,
    PSA_LIGHTS_SIDE       = 1,
    PSA_LIGHTS_HEADLIGHTS = 2,
    PSA_LIGHTS_FULL_BEAM  = 3
} psa_lights_state_t;

typedef struct {
    bool    drl_enabled;
    uint8_t follow_me_home_sec; /* 0, 15, 30, 60 */
    bool    auto_lock_drive;
    bool    auto_mirror_fold;
    bool    rear_wiper_reverse;
    bool    cornering_lights;
    
    /* Cluster Clock Telemetry */
    psa_lights_state_t lights_state;
    uint8_t year;   /* 0..99 */
    uint8_t month;  /* 1..12 */
    uint8_t day;    /* 1..31 */
    uint8_t hour;   /* 0..23 */
    uint8_t minute; /* 0..59 */
} psa_bsi_state_t;

typedef void (*canbox_uart_tx_fn)(const uint8_t *buf, size_t len);
typedef void (*canbox_can_tx_fn)(uint32_t id, const uint8_t *data, uint8_t dlc);

typedef struct {
    psa_bsi_state_t   state;
    canbox_uart_tx_fn uart_tx;
    canbox_can_tx_fn  can_tx;
} psa_bsi_ctx_t;

static inline void psa_bsi_init(psa_bsi_ctx_t *ctx, canbox_uart_tx_fn uart_tx, canbox_can_tx_fn can_tx) {
    memset(ctx, 0, sizeof(psa_bsi_ctx_t));
    ctx->uart_tx = uart_tx;
    ctx->can_tx  = can_tx;
}

/* Process PSA CAN 0x128 (Lighting & Warning Status) */
static inline void psa_bsi_process_can_0x128(psa_bsi_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 5) return;

    uint8_t d5 = data[4];
    if (!(d5 & 0x80)) {
        ctx->state.lights_state = PSA_LIGHTS_OFF;
    } else if (d5 & 0x20) {
        ctx->state.lights_state = PSA_LIGHTS_FULL_BEAM; /* 0xE0 or 0xA0 */
    } else if (d5 & 0x40) {
        ctx->state.lights_state = PSA_LIGHTS_HEADLIGHTS; /* 0xC0 */
    } else {
        ctx->state.lights_state = PSA_LIGHTS_SIDE; /* 0x80 */
    }
}

/* Process Android Downlink GPS Time Sync (Cmd 0xA6) -> Inject PSA CAN 0x228 */
static inline void psa_bsi_process_downlink_time(psa_bsi_ctx_t *ctx, const uint8_t *payload, size_t len) {
    if (!ctx->can_tx || len < 5) return;

    ctx->state.year   = payload[0];
    ctx->state.month  = payload[1];
    ctx->state.day    = payload[2];
    ctx->state.hour   = payload[3];
    ctx->state.minute = payload[4];

    /* Build PSA CAN 0x228 Frame */
    uint8_t can_data[8];
    memset(can_data, 0, sizeof(can_data));
    can_data[0] = ctx->state.hour;
    can_data[1] = ctx->state.minute;
    can_data[2] = ctx->state.day;
    can_data[3] = ctx->state.month;
    can_data[4] = ctx->state.year;
    can_data[5] = 0x00; /* 24-hour mode */

    ctx->can_tx(0x228, can_data, 8);
}

/* Process Android Downlink BSI Configuration Override (Cmd 0x80) */
static inline void psa_bsi_process_downlink_config(psa_bsi_ctx_t *ctx, uint8_t setting_id, uint8_t value) {
    if (!ctx->can_tx) return;

    /* Build PSA BSI Configuration Frame on CAN 0x221 */
    uint8_t can_data[8];
    memset(can_data, 0, sizeof(can_data));

    switch (setting_id) {
        case 0x01: /* DRL */
            ctx->state.drl_enabled = (value != 0);
            can_data[2] = ctx->state.drl_enabled ? 0x80 : 0x00;
            break;

        case 0x02: /* Follow Me Home */
            ctx->state.follow_me_home_sec = value;
            if (value == 60) can_data[4] = 0xC0;
            else if (value == 30) can_data[4] = 0x80;
            else if (value == 15) can_data[4] = 0x40;
            break;

        case 0x03: /* Auto Lock */
            ctx->state.auto_lock_drive = (value != 0);
            can_data[1] = ctx->state.auto_lock_drive ? 0x10 : 0x00;
            break;

        case 0x04: /* Rear Wiper */
            ctx->state.rear_wiper_reverse = (value != 0);
            can_data[1] = ctx->state.rear_wiper_reverse ? 0x80 : 0x00;
            break;

        case 0x05: /* Mirror Fold */
            ctx->state.auto_mirror_fold = (value != 0);
            can_data[4] = ctx->state.auto_mirror_fold ? 0x08 : 0x00;
            break;
    }

    ctx->can_tx(0x221, can_data, 8);
}

#endif /* CANBOX_BSI_H */
```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: Android Syncs Date/Time: 22 Sept 2026, 14:35 (24H)
### Vector 1: Headlights Switched to Low Beam (0xC0 in 0x128 Byte 4)
- **CAN ID `0x128` Injection:**
  ```bash
  cansend vcan0 128#B0C00000C080B001
  ```
- **Decoded Lighting State:**
  - `lights_state = PSA_LIGHTS_HEADLIGHTS (2)`

### Vector 2: Android Syncs Date/Time: 22 Sept 2026, 14:35 (24H)
- **UART Downlink Received (`Cmd 0xA6`):**
  - Payload: Year 26 (`0x1A`), Month 9 (`0x09`), Day 22 (`0x16`), Hour 14 (`0x0E`), Min 35 (`0x23`)
  - Frame: `5A A5 06 A6 1A 09 16 0E 23 12`
- **Expected CAN Frame Injected onto Instrument Cluster:**
  - ID: `0x228`, DLC: 8, Data: `0E 23 16 09 1A 00 00 00`

### Vector 2: Android Configures Follow-Me-Home to 30 Seconds
- **UART Downlink Received (`Cmd 0x80`):**
  - Frame: `5A A5 04 80 02 1E A4`
- **Expected CAN Frame Injected onto BSI:**
  - ID: `0x221`, DLC: 8, Data: `00 00 00 00 80 00 00 00`

