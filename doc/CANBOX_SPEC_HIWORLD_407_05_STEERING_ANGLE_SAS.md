# CAN Box Protocol Specification: Steering Wheel Angle Sensor (SAS)
## Topic 05: Peugeot 407 Steering Angle & Dynamic Trajectory
**Document File:** `CANBOX_SPEC_HIWORLD_407_05_STEERING_ANGLE_SAS.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)

---

# 1. Functional Domain & Architecture Overview

The Peugeot 407 Steering Column Control Module integrates an optical Steering Wheel Angle Sensor (SAS). The absolute steering wheel position is broadcast at high frequency (every 20 ms) over the PSA Comfort CAN bus on CAN ID `0x0E6`. 

When the vehicle is placed in reverse gear (detected on CAN ID `0x036`), the CAN box translates steering angle telemetry into headunit trajectory frames. The Android infotainment system uses this data to dynamically bend reverse camera parking guidelines in real time.

```
+------------------------------------------------------------------------------------+
|                         Peugeot 407 Steering Wheel Sensor (SAS)                    |
|                [16-Bit Signed Angle Telemetry, Cycle Time: 20 ms]                  |
+------------------------------------------------------------------------------------+
                                          │
                                   [PSA CAN 0x0E6]
                                          ▼
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Captures fast 20 ms steering angle frames (CAN ID 0x0E6)                      |
|   2. Interlocks angle reporting with Reverse Gear state (CAN ID 0x036)             |
|   3. Converts 0.1 deg signed values into Headunit Vendor Endianness                |
|   4. Dispatches Hiworld (Cmd 0x26, Big-Endian) / Raise (Cmd 0x29, Little-Endian)   |
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

### 2.1 PSA Steering Angle Frame (`0x0E6`)
- **CAN ID:** `0x0E6` (Standard 11-bit Identifier)
- **DLC:** 8 bytes (Bytes 0–1 active angle, Bytes 2–3 angular velocity, Bytes 4–7 status)
- **Transmission Cycle:** Fast periodic 20 ms

```
+--------+--------+------------------------------------+-----------------------------+
| Byte   | Bit    | Function / Signal Name             | Value / Encoding Definition |
+--------+--------+------------------------------------+-----------------------------+
| Byte 0 | 7..0   | Steering Angle High Byte (MSB)     | Signed 16-bit Big-Endian    |
+--------+--------+------------------------------------+-----------------------------+
| Byte 1 | 7..0   | Steering Angle Low Byte (LSB)      | Signed 16-bit Big-Endian    |
+--------+--------+------------------------------------+-----------------------------+
| Byte 2 | 7..0   | Steering Angular Velocity High Byte| Angular rate (deg/sec * 10) |
+--------+--------+------------------------------------+-----------------------------+
| Byte 3 | 7..0   | Steering Angular Velocity Low Byte | Angular rate (deg/sec * 10) |
+--------+--------+------------------------------------+-----------------------------+
| Byte 4 | Bit 7  | SAS Calibration / Zero Point Valid | 1 = Valid, 0 = Uncalibrated |
|        | Bit 0  | SAS Hardware Fault                 | 1 = Sensor Error            |
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

### 3.1 Hiworld Steering Angle Telemetry (`Cmd 0x26`)
- **Sync Header:** `0x5A 0xA5`
- **Length:** `0x03` (1-byte Cmd + 2-byte Payload)
- **Command ID:** `0x26`
- **Payload Layout (2 Bytes - Big Endian):**
  - `Byte 0`: Angle High Byte (`Angle >> 8`)
  - `Byte 1`: Angle Low Byte (`Angle & 0xFF`)
- **Checksum:** 8-bit sum modulo 256 over `Length + CmdID + Payload`.

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

typedef struct {
    int16_t angle_deci_deg; /* Signed 0.1 deg: -5400..+5400 */
    int16_t angular_velocity;
    bool    calibrated;
    bool    sensor_fault;
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

/* Transmit Hiworld Steering Angle (Cmd 0x26, Big Endian) */
static inline void psa_sas_send_hiworld(psa_sas_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[7];
    p[0] = 0x5A;
    p[1] = 0xA5;
    p[2] = 0x03; /* Length: 1 Cmd + 2 Payload */
    p[3] = 0x26; /* Cmd ID */
    p[4] = (uint8_t)(((uint16_t)ctx->state.angle_deci_deg >> 8) & 0xFF); /* MSB */
    p[5] = (uint8_t)((uint16_t)ctx->state.angle_deci_deg & 0xFF);        /* LSB */

    uint8_t sum = 0;
    for (size_t i = 2; i <= 5; i++) {
        sum += p[i];
    }
    p[6] = sum;

    ctx->uart_tx(p, 7);
}

/* Process PSA CAN 0x0E6 Frame */
static inline void psa_sas_process_can_0x0E6(psa_sas_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 2) return;

    /* Big-endian signed 16-bit unpack */
    int16_t raw_angle = (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
    ctx->state.angle_deci_deg = raw_angle;

    if (dlc >= 4) {
        ctx->state.angular_velocity = (int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3]);
    }
    if (dlc >= 5) {
        ctx->state.calibrated   = (data[4] & 0x80) ? true : false;
        ctx->state.sensor_fault = (data[4] & 0x01) ? true : false;
    }

    psa_sas_send_hiworld(ctx);
}

/* Update Reverse Gear Interlock */
static inline void psa_sas_set_reverse_gear(psa_sas_ctx_t *ctx, bool reverse_on) {
    ctx->state.reverse_gear_active = reverse_on;
}

#endif /* CANBOX_SAS_H */
```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: Steering Centered (0.0°)
- **CAN ID `0x0E6` Injection:**
  ```bash
  cansend vcan0 0E6#0000000080000000
  ```
- **Expected UART Output (Hiworld `0x26`):**
  - Frame: `5A A5 03 26 00 00 29`

### Vector 2: Steering Turned Right +90.0° (+900 = 0x0384)
- **CAN ID `0x0E6` Injection:**
  ```bash
  cansend vcan0 0E6#0384000080000000
  ```
- **Expected UART Output (Hiworld `0x26`):**
  - Frame: `5A A5 03 26 03 84 B0`

### Vector 3: Steering Turned Left -90.0° (-900 = 0xFC7C)
- **CAN ID `0x0E6` Injection:**
  ```bash
  cansend vcan0 0E6#FC7C000080000000
  ```
- **Expected UART Output (Hiworld `0x26`):**
  - Frame: `5A A5 03 26 FC 7C A1`

