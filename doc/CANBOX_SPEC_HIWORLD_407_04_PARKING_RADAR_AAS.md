# CAN Box Protocol Specification: Ultrasonic Parking Sensors (AAS)
## Topic 04: Peugeot 407 Front & Rear Parking Radar Telemetry
**Document File:** `CANBOX_SPEC_HIWORLD_407_04_PARKING_RADAR_AAS.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)

---

# 1. Functional Domain & Architecture Overview

The Peugeot 407 Parking Assistance ECU (AAS - *Aide au Stationnement*) monitors front and rear ultrasonic sensors. Sensor obstacle distances are computed in real time and broadcast across the PSA Comfort CAN bus on CAN IDs `0x260` (Rear AAS) and `0x270` (Front AAS). 

The CAN box translates these raw distance zones into serialized headunit radar packets, allowing the Android infotainment screen to render graphical distance arcs and trigger corresponding parking buzzer audio profiles.

```
+------------------------------------------------------------------------------------+
|                         Peugeot 407 Parking Assist ECU (AAS)                       |
|           [Rear 4-Sensor Zone (0x260) / Front 4-Sensor Zone (0x270)]               |
+------------------------------------------------------------------------------------+
                                          │
                                 [PSA CAN 0x260 / 0x270]
                                          ▼
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Captures periodic 100 ms radar frames                                         |
|   2. Converts Peugeot raw distance steps (0..4, 0xFF) to Headunit Radar Format     |
|   3. Gates radar packet transmission based on Reverse Gear or AAS active state     |
|   4. Dispatches Hiworld Radar Telemetry (Cmd 0x22 / 0x30 / 0x32)                   |
+------------------------------------------------------------------------------------+
                                          │
                   [UART Serial: 38400 baud, 8N1 / Hiworld Protocol]
                                          ▼
+------------------------------------------------------------------------------------+
|                     Android Headunit Reverse Camera & Radar UI                     |
|           (Displays overlay parking arcs over rear camera video stream)            |
+------------------------------------------------------------------------------------+
```

---

# 2. PSA CAN Bus Bitfield Specification

### 2.1 Rear Parking Radar (`0x260`)
- **CAN ID:** `0x260` (Standard 11-bit Identifier)
- **DLC:** 8 bytes (Bytes 0–2 active distance data, Byte 3 system status)
- **Transmission Cycle:** Periodic 100 ms while active

```
+--------+--------+------------------------------------+-----------------------------+
| Byte   | Bit    | Function / Signal Name             | Value / Distance Definition |
+--------+--------+------------------------------------+-----------------------------+
| Byte 0 | 7..0   | Rear Left Outer Sensor             | 0x00..0x04, 0xFF = Inactive |
+--------+--------+------------------------------------+-----------------------------+
| Byte 1 | 7..0   | Rear Center Sensors (Inner Left/Rt)| 0x00..0x04, 0xFF = Inactive |
+--------+--------+------------------------------------+-----------------------------+
| Byte 2 | 7..0   | Rear Right Outer Sensor            | 0x00..0x04, 0xFF = Inactive |
+--------+--------+------------------------------------+-----------------------------+
| Byte 3 | Bit 7  | AAS System Active State            | 1 = Active / 0 = Inactive   |
|        | Bit 0  | AAS Fault / Mute State             | 1 = System Fault            |
+--------+--------+------------------------------------+-----------------------------+
```

### 2.2 Front Parking Radar (`0x270`)
- **CAN ID:** `0x270` (Standard 11-bit Identifier)
- **DLC:** 8 bytes
- **Transmission Cycle:** Periodic 100 ms while active

```
+--------+--------+------------------------------------+-----------------------------+
| Byte   | Bit    | Function / Signal Name             | Value / Distance Definition |
+--------+--------+------------------------------------+-----------------------------+
| Byte 0 | 7..0   | Front Left Outer Sensor            | 0x00..0x04, 0xFF = Inactive |
+--------+--------+------------------------------------+-----------------------------+
| Byte 1 | 7..0   | Front Center Sensors               | 0x00..0x04, 0xFF = Inactive |
+--------+--------+------------------------------------+-----------------------------+
| Byte 2 | 7..0   | Front Right Outer Sensor           | 0x00..0x04, 0xFF = Inactive |
+--------+--------+------------------------------------+-----------------------------+
```

### 2.3 Distance Threshold & Acoustic Tone Mapping:
| Step Code | Physical Distance ($d$) | Android Graphic Arcs | Acoustic Beep Profile |
|:---:|:---:|:---:|:---|
| `0x00` | $d > 120\text{ cm}$ (Zone 1) | 1 Green Arc | Slow intermittent tone ($1\text{ Hz}$) |
| `0x01` | $90\text{ cm} < d \le 120\text{ cm}$ (Zone 2) | 3 Yellow Arcs | Moderate intermittent tone ($2\text{ Hz}$) |
| `0x02` | $60\text{ cm} < d \le 90\text{ cm}$ (Zone 3) | 5 Orange Arcs | Fast intermittent tone ($4\text{ Hz}$) |
| `0x03` | $30\text{ cm} < d \le 60\text{ cm}$ (Zone 4) | 7 Red-Orange Arcs | Rapid intermittent tone ($8\text{ Hz}$) |
| `0x04` | $d \le 30\text{ cm}$ (Zone 5) | 10 Solid Red Arcs | Continuous tone |
| `0xFF` | Inactive / No Obstacle | 0 Arcs (Hidden) | Mute / Silent |

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Hiworld Unified Radar Telemetry (`Cmd 0x22` / `0x32`)
- **Sync Header:** `0x5A 0xA5`
- **Length:** `0x09` (1-byte Cmd + 8-byte Payload)
- **Command ID:** `0x22` (Combined Front & Rear Radar)
- **Payload Layout (8 Bytes):**
  - `Byte 0`: Rear Left Outer ($0 \dots 4$, `0xFF` = clear)
  - `Byte 1`: Rear Left Center ($0 \dots 4$)
  - `Byte 2`: Rear Right Center ($0 \dots 4$)
  - `Byte 3`: Rear Right Outer ($0 \dots 4$)
  - `Byte 4`: Front Left Outer ($0 \dots 4$)
  - `Byte 5`: Front Left Center ($0 \dots 4$)
  - `Byte 6`: Front Right Center ($0 \dots 4$)
  - `Byte 7`: Front Right Outer ($0 \dots 4$)
- **Checksum:** 8-bit sum modulo 256 over `Length + CmdID + Payload`.

### 3.2 Raise Radar Compatibility Format:
- **Front Radar (`Cmd 0x30`):** `0x2E 0x30 0x06 [FL] [FC] [FR] [RL] [RC] [RR] [Checksum]`
- **Rear Radar (`Cmd 0x32`):** `0x2E 0x32 0x07 0x00 [RL] [RC] [RR] [FL] [FC] [FR] [Checksum]`

---

# 4. Pure C99 Firmware Implementation

```c
#ifndef CANBOX_RADAR_H
#define CANBOX_RADAR_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

typedef struct {
    uint8_t rear_left_outer;
    uint8_t rear_center;
    uint8_t rear_right_outer;
    uint8_t front_left_outer;
    uint8_t front_center;
    uint8_t front_right_outer;
    bool    system_active;
    bool    system_fault;
} psa_radar_state_t;

typedef void (*canbox_uart_tx_fn)(const uint8_t *buf, size_t len);

typedef struct {
    psa_radar_state_t state;
    canbox_uart_tx_fn uart_tx;
} psa_radar_ctx_t;

static inline void psa_radar_init(psa_radar_ctx_t *ctx, canbox_uart_tx_fn uart_tx) {
    memset(ctx, 0, sizeof(psa_radar_ctx_t));
    ctx->state.rear_left_outer   = 0xFF;
    ctx->state.rear_center       = 0xFF;
    ctx->state.rear_right_outer  = 0xFF;
    ctx->state.front_left_outer  = 0xFF;
    ctx->state.front_center      = 0xFF;
    ctx->state.front_right_outer = 0xFF;
    ctx->uart_tx = uart_tx;
}

/* Transmit Hiworld Radar Packet (Cmd 0x22) */
static inline void psa_radar_send_hiworld(psa_radar_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[12];
    p[0] = 0x5A;
    p[1] = 0xA5;
    p[2] = 0x09; /* Length: 1 Cmd + 8 Payload */
    p[3] = 0x22; /* Cmd ID */

    p[4] = ctx->state.rear_left_outer;
    p[5] = ctx->state.rear_center;
    p[6] = ctx->state.rear_center;
    p[7] = ctx->state.rear_right_outer;

    p[8]  = ctx->state.front_left_outer;
    p[9]  = ctx->state.front_center;
    p[10] = ctx->state.front_center;
    p[11] = ctx->state.front_right_outer;

    uint8_t sum = 0;
    for (size_t i = 2; i <= 11; i++) {
        sum += p[i];
    }
    p[12] = sum;

    ctx->uart_tx(p, 13);
}

/* Process PSA CAN 0x260 (Rear Radar) */
static inline void psa_radar_process_can_0x260(psa_radar_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 3) return;

    ctx->state.rear_left_outer  = data[0];
    ctx->state.rear_center      = data[1];
    ctx->state.rear_right_outer = data[2];

    if (dlc >= 4) {
        ctx->state.system_active = (data[3] & 0x80) ? true : false;
        ctx->state.system_fault  = (data[3] & 0x01) ? true : false;
    }

    psa_radar_send_hiworld(ctx);
}

/* Process PSA CAN 0x270 (Front Radar) */
static inline void psa_radar_process_can_0x270(psa_radar_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 3) return;

    ctx->state.front_left_outer  = data[0];
    ctx->state.front_center      = data[1];
    ctx->state.front_right_outer = data[2];

    psa_radar_send_hiworld(ctx);
}

#endif /* CANBOX_RADAR_H */
```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: Close Obstacle Rear Center (Zone 4 = 0x03)
- **CAN ID `0x260` Injection:**
  ```bash
  cansend vcan0 260#FFFFFF03FFFFFF80
  ```
- **Expected UART Output (Hiworld `0x22`):**
  - Frame: `5A A5 09 22 FF 03 03 FF FF FF FF FF 26`

### Vector 2: Critical Stop Obstacle Rear Right (Zone 5 = 0x04)
- **CAN ID `0x260` Injection:**
  ```bash
  cansend vcan0 260#FFFFFF04FFFFFF80
  ```
- **Expected UART Output (Hiworld `0x22`):**
  - Frame: `5A A5 09 22 FF 04 04 FF FF FF FF FF 28`

