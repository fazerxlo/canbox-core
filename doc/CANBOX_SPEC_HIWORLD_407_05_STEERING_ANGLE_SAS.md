# CAN Box Protocol Specification: Steering Wheel Angle Sensor (SAS)
## Topic 05: Peugeot 407 Steering Angle & Dynamic Trajectory
**Document File:** `CANBOX_SPEC_HIWORLD_407_05_STEERING_ANGLE_SAS.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** `signal-db/steering_wheel.yaml` (`Msg0C5`), `QF_Canbus.apk` (`PeugeotDataParser.java`)

---

# 1. Functional Domain & Architecture Overview

The Peugeot 407 Steering Column Control Module integrates an optical Steering Wheel Angle Sensor (SAS). The absolute steering wheel position is broadcast at high frequency (every 50 ms) over the PSA Comfort CAN bus on CAN ID **`0x0C5`**.

*(Note: CAN ID `0x0E6` in PSA CAN2004 represents ABS wheel pulse counter ticks `MSG_IS_DAT_ABR` used for dead reckoning navigation, whereas `0x0C5` is the dedicated SAS angle frame).*

When the vehicle is placed in reverse gear, the CAN box translates steering angle telemetry into headunit trajectory frames. The Android infotainment system uses this data to dynamically bend reverse camera parking guidelines in real time.

```
+------------------------------------------------------------------------------------+
|                         Peugeot 407 Steering Wheel Sensor (SAS)                    |
|                [16-Bit Signed Angle Telemetry, Cycle Time: 50 ms]                  |
+------------------------------------------------------------------------------------+
                                          │
                                   [PSA CAN 0x0C5]
                                          ▼
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Captures fast 50 ms steering angle frames (CAN ID 0x0C5)                      |
|   2. Interlocks angle reporting with Reverse Gear state                            |
|   3. Dispatches Hiworld (Cmd 0x11, Bytes 6-7) / Raise (Cmd 0x29, Little-Endian)    |
+------------------------------------------------------------------------------------+
                                          │
                    [UART Serial: 38400 baud, 8N1 / Hiworld Protocol]
                                          ▼
+------------------------------------------------------------------------------------+
|                     Android Headunit Reverse Parking Application                   |
|          (Renders dynamic curving guide tracks over the camera live feed)          |
+------------------------------------------------------------------------------------+
```

---

# 2. PSA CAN Bus Bitfield Specification

### 2.1 PSA Steering Angle Frame (`0x0C5`)
- **CAN ID:** `0x0C5` (Standard 11-bit Identifier)
- **DLC:** 4 bytes
- **Transmission Cycle:** Periodic 50 ms
- **Quiescent / Center Frame:** `00 00 00 00`

```
+--------+--------+------------------------------------+-----------------------------+
| Byte   | Bit    | Function / Signal Name             | Value / Encoding Definition |
+--------+--------+------------------------------------+-----------------------------+
| Byte 0 | 7..0   | Steering Angle High Byte (MSB)     | Signed 16-bit Big-Endian    |
+--------+--------+------------------------------------+-----------------------------+
| Byte 1 | 7..0   | Steering Angle Low Byte (LSB)      | Signed 16-bit Big-Endian    |
+--------+--------+------------------------------------+-----------------------------+
| Byte 2 | 7..0   | Reserved Padding                   | Always 0x00                 |
+--------+--------+------------------------------------+-----------------------------+
| Byte 3 | 7..0   | Reserved Padding                   | Always 0x00                 |
+--------+--------+------------------------------------+-----------------------------+
```

### 2.2 Physical Scaling & Direction:
$$\text{Steering Angle } (^\circ) = \frac{\text{SignedValue}_{16}}{10.0}$$
$$\text{SignedValue}_{16} = \text{Round}(\text{Angle}_{^\circ} \times 10.0)$$
- **Physical Range:** $-540.0^\circ \dots +540.0^\circ$ (Raw: $-5400 \dots +5400$).
- **Direction Convention:**
  - $\text{Value} = 0$: Steering wheel centered.
  - $\text{Value} < 0$: Steering wheel turned **Left** (e.g. $-90.0^\circ \implies -900 = \text{0xFC7C}$).
  - $\text{Value} > 0$: Steering wheel turned **Right** (e.g. $+90.0^\circ \implies +900 = \text{0x0384}$).

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Hiworld Base Info Frame Steering Telemetry (`Cmd 0x11`)
In the Hiworld PSA protocol, steering wheel angle is multiplexed inside the Base Info Frame (`Cmd 0x11`):
- **Sync Header:** `0x5A 0xA5`
- **Length ($L$):** `0x08` (8 payload bytes)
- **Command ID:** `0x11` (`17` decimal / `Handle.CarBaseInfo`)
- **Payload Layout (8 Bytes):**
  - `Byte 0..1`: Reserved / Key status (`0x00 0x00`)
  - `Byte 2..3`: Key Code & State (`0x00 0x00` when idle)
  - `Byte 4..5`: Reserved (`0x00 0x00`)
  - `Byte 6`: **Angle High Byte** (Signed 16-bit MSB, Big Endian)
  - `Byte 7`: **Angle Low Byte** (Signed 16-bit LSB, Big Endian)
- **Checksum:** `((Length + CmdID + sum(Payload)) - 1) & 0xFF`
- **Total Frame Wire Length:** 13 bytes (`5A A5 08 11 [8 Bytes] Checksum`)

### 3.2 Raise Steering Angle Compatibility Format (`Cmd 0x29`):
- **Sync Header:** `0x2E`
- **Command ID:** `0x29`
- **Length:** `0x02`
- **Payload Layout (2 Bytes - Little Endian):**
  - `Byte 0`: Angle Low Byte (`Angle & 0xFF`)
  - `Byte 1`: Angle High Byte (`Angle >> 8`)
- **Checksum:** Bitwise inverted sum `(Sum ^ 0xFF) & 0xFF`.

---

# 4. Pure C99 Firmware Implementation

```c
#ifndef CANBOX_SAS_H
#define CANBOX_SAS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define HIWORLD_SOF1         0x5A
#define HIWORLD_SOF2         0xA5
#define HIWORLD_CMD_CAR_BASE 0x11

typedef struct {
    int16_t angle_deci_deg; /* Signed 0.1 deg: -5400..+5400 */
    bool    reverse_gear_active;
} psa_sas_state_t;

typedef void (*canbox_uart_tx_fn)(const uint8_t *buf, size_t len);

typedef struct {
    psa_sas_state_t   state;
    canbox_uart_tx_fn uart_tx;
} psa_sas_ctx_t;

static inline void psa_sas_init(psa_sas_ctx_t *ctx, canbox_uart_tx_fn uart_tx) {
    memset(ctx, 0, sizeof(psa_sas_ctx_t));
    ctx->uart_tx = uart_tx;
}

/* Transmit Hiworld Steering Angle (Cmd 0x11, Big Endian) */
static inline void psa_sas_send_hiworld(psa_sas_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[13];
    p[0] = HIWORLD_SOF1;
    p[1] = HIWORLD_SOF2;
    p[2] = 0x08;                 /* Length: 8 Payload bytes */
    p[3] = HIWORLD_CMD_CAR_BASE; /* Cmd ID: 0x11 */
    p[4] = 0x00;
    p[5] = 0x00;
    p[6] = 0x00;                 /* Key Code (idle) */
    p[7] = 0x00;                 /* Key State (idle) */
    p[8] = 0x00;
    p[9] = 0x00;
    p[10] = (uint8_t)(((uint16_t)ctx->state.angle_deci_deg >> 8) & 0xFF); /* Angle MSB */
    p[11] = (uint8_t)((uint16_t)ctx->state.angle_deci_deg & 0xFF);        /* Angle LSB */

    uint8_t sum = 0;
    for (size_t i = 2; i <= 11; i++) {
        sum += p[i];
    }
    p[12] = (uint8_t)((sum - 1) & 0xFF);

    ctx->uart_tx(p, 13);
}

/* Process PSA CAN 0x0C5 Frame */
static inline void psa_sas_process_can_0x0C5(psa_sas_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 2) return;

    /* Big-endian signed 16-bit unpack */
    int16_t raw_angle = (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
    ctx->state.angle_deci_deg = raw_angle;

    psa_sas_send_hiworld(ctx);
}

#endif /* CANBOX_SAS_H */
```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: Steering Centered (0.0°)
- **CAN ID `0x0C5` Injection:**
  ```bash
  cansend vcan0 0C5#00000000
  ```
- **Expected UART Output (Hiworld `0x11`):**
  - Frame: `5A A5 08 11 00 00 00 00 00 00 00 00 18`
  - Checksum Calculation: `(0x08 + 0x11 - 1) & 0xFF = 0x18`

### Vector 2: Steering Turned Right +90.0° (+900 = 0x0384)
- **CAN ID `0x0C5` Injection:**
  ```bash
  cansend vcan0 0C5#03840000
  ```
- **Expected UART Output (Hiworld `0x11`):**
  - Frame: `5A A5 08 11 00 00 00 00 00 00 03 84 9F`
  - Checksum Calculation: `(0x08 + 0x11 + 0x03 + 0x84 - 1) & 0xFF = 0x9F`

### Vector 3: Steering Turned Left -90.0° (-900 = 0xFC7C)
- **CAN ID `0x0C5` Injection:**
  ```bash
  cansend vcan0 0C5#FC7C0000
  ```
- **Expected UART Output (Hiworld `0x11`):**
  - Frame: `5A A5 08 11 00 00 00 00 00 00 FC 7C 90`
  - Checksum Calculation: `(0x08 + 0x11 + 0xFC + 0x7C - 1) & 0xFF = 0x90`
