# CAN Box Protocol Specification: Ultrasonic Parking Sensors (AAS)
## Topic 04: Peugeot 407 Front & Rear Parking Radar Telemetry
**Document File:** `CANBOX_SPEC_HIWORLD_407_04_PARKING_RADAR_AAS.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** `signal-db/parktronic.yaml` (`Msg0E1`), `QF_Canbus.apk` (`PeugeotDataParser.java` / `VehicleReceiver.java`)

---

# 1. Functional Domain & Architecture Overview

The Peugeot 407 Parking Assistance ECU (AAS - *Aide au Stationnement*) monitors front and rear ultrasonic sensors. In native Peugeot 407 CAN2004 Comfort bus architecture, sensor obstacle distances and zone activations are multiplexed onto CAN ID **`0x0E1`** (7 bytes, periodic 100 ms). On newer PSA architectures (AEE2010 / Peugeot 308/508) or certain aftermarket gateways, distances are separated into CAN IDs `0x260` (Rear AAS) and `0x270` (Front AAS).

The CAN box translates these raw distance zones into serialized headunit radar packets (**`Cmd 0x41`** / `Handle.CarRadarState`), allowing the Android infotainment screen to render graphical distance arcs and trigger corresponding parking buzzer audio profiles.

```
+------------------------------------------------------------------------------------+
|                         Peugeot 407 Parking Assist ECU (AAS)                       |
|           [Native CAN2004: 0x0E1 (7 bytes) / AEE2010: 0x260 & 0x270]               |
+------------------------------------------------------------------------------------+
                                          │
                                 [PSA CAN 0x0E1]
                                          ▼
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Captures periodic 100 ms radar frames (0x0E1)                                 |
|   2. Converts Peugeot raw distance steps (0..4/7, 0xFF) to Headunit Radar Format   |
|   3. Pulls physical BACK trigger wire (+12V/GND) on Reverse Gear                   |
|   4. Dispatches Hiworld Radar Telemetry (Cmd 0x41, Length 0x08)                    |
+------------------------------------------------------------------------------------+
                                          │
                   [UART Serial: 38400 baud, 8N1 / Hiworld Protocol]
                   [Hardware: Dedicated BACK / REVERSE Trigger Wire]
                                          ▼
+------------------------------------------------------------------------------------+
|                     Android Headunit Reverse Camera & Radar UI                     |
|           (Displays overlay parking arcs over rear camera video stream)            |
+------------------------------------------------------------------------------------+
```

> [!NOTE]
> **Reverse Window Activation Requirement:** On Android Head Units (e.g., TS10, UIS7862, Topway), the reverse camera view (`QF_BackCar.apk`) is launched by the MCU hardware monitoring the physical `BACK` / `REVERSE` wire (triggering `com.qf.action.BACKCAR_START` or KeyEvent 289). Radar telemetry packets (`Cmd 0x41`) are processed by `QF_Canbus.apk` and rendered as overlays while the reverse window is active (or if Front Running Radar is enabled).

---

# 2. PSA CAN Bus Bitfield Specification

### 2.1 Native Peugeot 407 CAN2004 Parking Radar Frame (`0x0E1`)
- **CAN ID:** `0x0E1` (Standard 11-bit Identifier)
- **DLC:** 7 bytes
- **Transmission Cycle:** Periodic 100 ms while active
- **Inactive / Quiescent Frame:** `24 00 3F FC FC FC 00` (or zeroed sensors)

```
+--------+--------+------------------------------------+-----------------------------+
| Byte   | Bit    | Function / Signal Name             | Value / Distance Definition |
+--------+--------+------------------------------------+-----------------------------+
| Byte 0 | 7..0   | Header / Constant                  | Always 0x24                 |
+--------+--------+------------------------------------+-----------------------------+
| Byte 1 | Bit 6  | Rear Radar Active                  | 0x40 (1 = Active)           |
|        | Bit 4  | Front Radar Active                 | 0x10 (1 = Active)           |
|        | Bits   | Other zone/status flags            | 0x00                        |
+--------+--------+------------------------------------+-----------------------------+
| Byte 2 | 7..0   | Constant Mask                      | Always 0x3F                 |
+--------+--------+------------------------------------+-----------------------------+
| Byte 3 | 7..5   | Rear Left (RL) Sensor Zone         | 0..7 (0=closest, 7=clear)   |
|        | 4..2   | Rear Center (RC) Sensor Zone       | 0..7 (0=closest, 7=clear)   |
|        | 1..0   | Reserved                           | 0                           |
+--------+--------+------------------------------------+-----------------------------+
| Byte 4 | 7..5   | Rear Right (RR) Sensor Zone        | 0..7 (0=closest, 7=clear)   |
|        | 4..2   | Front Left (FL) Sensor Zone        | 0..7 (0=closest, 7=clear)   |
|        | 1..0   | Reserved                           | 0                           |
+--------+--------+------------------------------------+-----------------------------+
| Byte 5 | 7..5   | Front Center (FC) Sensor Zone      | 0..7 (0=closest, 7=clear)   |
|        | 4..2   | Front Right (FR) Sensor Zone       | 0..7 (0=closest, 7=clear)   |
|        | Bit 1  | Display Active Flag                | 1 = Display AAS on screen   |
|        | Bit 0  | Reserved                           | 0                           |
+--------+--------+------------------------------------+-----------------------------+
| Byte 6 | 7..0   | Trailer / Status Byte              | Always 0x00                 |
+--------+--------+------------------------------------+-----------------------------+
```

### 2.2 Alternate / AEE2010 Parking Radar Frames (`0x260` / `0x270`)
- **Rear Radar (`0x260`):**
  - `Byte 0`: Rear Left Outer (0x00..0x04, 0xFF = Inactive)
  - `Byte 1`: Rear Center Inner (0x00..0x04, 0xFF = Inactive)
  - `Byte 2`: Rear Right Outer (0x00..0x04, 0xFF = Inactive)
  - `Byte 3`: Bit 7 = System Active (1=Active), Bit 0 = System Fault
- **Front Radar (`0x270`):**
  - `Byte 0`: Front Left Outer
  - `Byte 1`: Front Center Inner
  - `Byte 2`: Front Right Outer

### 2.3 Distance Threshold & Acoustic Tone Mapping:
| Step Code | Physical Distance ($d$) | Android Graphic Arcs | Acoustic Beep Profile |
|:---:|:---:|:---:|:---|
| `0x00` | $d \le 30\text{ cm}$ (Zone 1 / Critical) | 10 Solid Red Arcs | Continuous tone |
| `0x01` | $30\text{ cm} < d \le 60\text{ cm}$ (Zone 2) | 7 Red-Orange Arcs | Rapid intermittent tone ($8\text{ Hz}$) |
| `0x02` | $60\text{ cm} < d \le 90\text{ cm}$ (Zone 3) | 5 Orange Arcs | Fast intermittent tone ($4\text{ Hz}$) |
| `0x03` | $90\text{ cm} < d \le 120\text{ cm}$ (Zone 4) | 3 Yellow Arcs | Moderate intermittent tone ($2\text{ Hz}$) |
| `0x04` | $d > 120\text{ cm}$ (Zone 5) | 1 Green Arc | Slow intermittent tone ($1\text{ Hz}$) |
| `0xFF` / `0x07` | Inactive / No Obstacle | 0 Arcs (Hidden) | Mute / Silent |

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Hiworld Unified Radar Telemetry (`Cmd 0x41` / `Handle.CarRadarState`)
- **Sync Header:** `0x5A 0xA5`
- **Length ($L$):** `0x08` (8 sensor payload bytes)
- **Command ID:** `0x41` (`65` decimal / `Handle.CarRadarState`)
- **Payload Layout (8 Bytes):**
  - `Byte 0`: Rear Left Outer (RL) ($0 \dots 6$, `0xFF` = clear)
  - `Byte 1`: Rear Left Center (RML) ($0 \dots 6$)
  - `Byte 2`: Rear Right Center (RMR) ($0 \dots 6$)
  - `Byte 3`: Rear Right Outer (RR) ($0 \dots 6$)
  - `Byte 4`: Front Left Outer (FL) ($0 \dots 6$, `0xFF` = clear)
  - `Byte 5`: Front Left Center (FML) ($0 \dots 6$)
  - `Byte 6`: Front Right Center (FMR) ($0 \dots 6$)
  - `Byte 7`: Front Right Outer (FR) ($0 \dots 6$)
- **Value Semantics:**
  - `0x00..0x06`: Obstacle proximity zone (0 = zone 1 / closest, 6 = zone 7 / farthest)
  - `0xFF`: Inactive / No obstacle detected (sensor arc hidden)
- **Checksum:** `((Length + CmdID + sum(Payload)) - 1) & 0xFF`
- **Total Frame Wire Length:** 13 bytes (`5A A5 08 41 [8 Bytes] Checksum`)

> [!NOTE]
> In the Hiworld protocol dictionary, `Cmd 0x22` is reserved for physical rotary control knobs (`Handle.ControlPanelKnob`), whereas `Cmd 0x41` is dedicated to radar state telemetry (`Handle.CarRadarState`).

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

#define HIWORLD_SOF1             0x5A
#define HIWORLD_SOF2             0xA5
#define HIWORLD_CMD_RADAR_STATE  0x41

typedef struct {
    uint8_t rear_left_outer;
    uint8_t rear_left_center;
    uint8_t rear_right_center;
    uint8_t rear_right_outer;
    uint8_t front_left_outer;
    uint8_t front_left_center;
    uint8_t front_right_center;
    uint8_t front_right_outer;
    bool    rear_active;
    bool    front_active;
    bool    display_active;
    bool    system_fault;
} psa_radar_state_t;

typedef void (*canbox_uart_tx_fn)(const uint8_t *buf, size_t len);

typedef struct {
    psa_radar_state_t state;
    canbox_uart_tx_fn uart_tx;
} psa_radar_ctx_t;

static inline void psa_radar_init(psa_radar_ctx_t *ctx, canbox_uart_tx_fn uart_tx) {
    memset(ctx, 0, sizeof(psa_radar_ctx_t));
    ctx->state.rear_left_outer    = 0xFF;
    ctx->state.rear_left_center   = 0xFF;
    ctx->state.rear_right_center  = 0xFF;
    ctx->state.rear_right_outer   = 0xFF;
    ctx->state.front_left_outer   = 0xFF;
    ctx->state.front_left_center  = 0xFF;
    ctx->state.front_right_center = 0xFF;
    ctx->state.front_right_outer  = 0xFF;
    ctx->uart_tx = uart_tx;
}

/* Transmit Hiworld Radar Packet (Cmd 0x41) */
static inline void psa_radar_send_hiworld(psa_radar_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[13];
    p[0] = HIWORLD_SOF1;
    p[1] = HIWORLD_SOF2;
    p[2] = 0x08;                    /* Length: 8 Payload bytes */
    p[3] = HIWORLD_CMD_RADAR_STATE; /* Cmd ID: 0x41 */

    p[4] = ctx->state.rear_left_outer;
    p[5] = ctx->state.rear_left_center;
    p[6] = ctx->state.rear_right_center;
    p[7] = ctx->state.rear_right_outer;

    p[8]  = ctx->state.front_left_outer;
    p[9]  = ctx->state.front_left_center;
    p[10] = ctx->state.front_right_center;
    p[11] = ctx->state.front_right_outer;

    uint8_t sum = 0;
    for (size_t i = 2; i <= 11; i++) {
        sum += p[i];
    }
    p[12] = (uint8_t)((sum - 1) & 0xFF);

    ctx->uart_tx(p, 13);
}

/* Map 3-bit PSA sensor zone (0..7) to Hiworld distance step (0..6, 0xFF) */
static inline uint8_t psa_radar_map_zone(uint8_t raw3bit) {
    if (raw3bit >= 7 || raw3bit == 0) return 0xFF; /* 0xFF = clear / inactive */
    /* PSA raw: 1 = closest (zone 0 in Hiworld) .. 6 = farthest (zone 5 in Hiworld) */
    return raw3bit - 1;
}

/* Process Native PSA CAN 0x0E1 (Peugeot 407 Parktronic) */
static inline void psa_radar_process_can_0x0e1(psa_radar_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 6) return;

    ctx->state.rear_active    = (data[1] >> 6) & 1;
    ctx->state.front_active   = (data[1] >> 4) & 1;
    ctx->state.display_active = (data[5] & 0x02) ? true : false;

    if (!ctx->state.display_active && !ctx->state.rear_active && !ctx->state.front_active) {
        ctx->state.rear_left_outer    = 0xFF;
        ctx->state.rear_left_center   = 0xFF;
        ctx->state.rear_right_center  = 0xFF;
        ctx->state.rear_right_outer   = 0xFF;
        ctx->state.front_left_outer   = 0xFF;
        ctx->state.front_left_center  = 0xFF;
        ctx->state.front_right_center = 0xFF;
        ctx->state.front_right_outer  = 0xFF;
    } else {
        uint8_t rl = (data[3] >> 5) & 0x07;
        uint8_t rc = (data[3] >> 2) & 0x07;
        uint8_t rr = (data[4] >> 5) & 0x07;
        uint8_t fl = (data[4] >> 2) & 0x07;
        uint8_t fc = (data[5] >> 5) & 0x07;
        uint8_t fr = (data[5] >> 2) & 0x07;

        ctx->state.rear_left_outer    = psa_radar_map_zone(rl);
        ctx->state.rear_left_center   = psa_radar_map_zone(rc);
        ctx->state.rear_right_center  = psa_radar_map_zone(rc);
        ctx->state.rear_right_outer   = psa_radar_map_zone(rr);

        ctx->state.front_left_outer   = psa_radar_map_zone(fl);
        ctx->state.front_left_center  = psa_radar_map_zone(fc);
        ctx->state.front_right_center = psa_radar_map_zone(fc);
        ctx->state.front_right_outer  = psa_radar_map_zone(fr);
    }

    psa_radar_send_hiworld(ctx);
}

/* Process AEE2010 PSA CAN 0x260 (Rear Radar) */
static inline void psa_radar_process_can_0x260(psa_radar_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 3) return;

    ctx->state.rear_left_outer   = data[0];
    ctx->state.rear_left_center  = data[1];
    ctx->state.rear_right_center = data[1];
    ctx->state.rear_right_outer  = data[2];

    if (dlc >= 4) {
        ctx->state.rear_active  = (data[3] & 0x80) ? true : false;
        ctx->state.system_fault = (data[3] & 0x01) ? true : false;
    }

    psa_radar_send_hiworld(ctx);
}

/* Process AEE2010 PSA CAN 0x270 (Front Radar) */
static inline void psa_radar_process_can_0x270(psa_radar_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 3) return;

    ctx->state.front_left_outer   = data[0];
    ctx->state.front_left_center  = data[1];
    ctx->state.front_right_center = data[1];
    ctx->state.front_right_outer  = data[2];

    psa_radar_send_hiworld(ctx);
}

#endif /* CANBOX_RADAR_H */
```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: Native CAN 0x0E1 Obstacle Rear Center
- **CAN ID `0x0E1` Injection:**
  ```bash
  cansend vcan0 0E1#24403F0C000200
  ```
- **Expected UART Output (Hiworld `0x41`):**
  - Frame: `5A A5 08 41 FF 02 02 FF FF FF FF FF 46`

### Vector 2: Quiescent / All Clear
- **CAN ID `0x0E1` Injection:**
  ```bash
  cansend vcan0 0E1#24003FFCFCFC00
  ```
- **Expected UART Output (Hiworld `0x41`):**
  - Frame: `5A A5 08 41 FF FF FF FF FF FF FF FF 40`
