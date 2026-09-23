# CAN Box Protocol Specification: Tire Pressure Monitoring System (TPMS)
## Topic 03: Peugeot 407 Direct & Discrete TPMS Telemetry
**Document File:** `CANBOX_SPEC_HIWORLD_407_03_TPMS_TIRE_PRESSURE.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** `signal-db/tyres.yaml` (`Msg1E1`, `Msg361`, `Msg3A1`), `QF_Canbus.apk` (`PeugeotDataParser.java`)

---

# 1. Functional Domain & Architecture Overview

The Peugeot 407 features active TPMS wheel transmitters in all four road wheels (and optionally the spare wheel). The TPMS ECU / BSI receives 433 MHz RF broadcasts from each wheel sensor and bridges tire pressure and diagnostic alarm states onto the PSA Comfort CAN bus via:
- **`0x1E1` (Wheel Status Enum / `MSG_DONNEES_ETAT_ROUES`, 250 ms):** Wheel state enum (OK, under-inflated, puncture, sensor missing) for FL, FR, RR, RL, spare wheel, and overall system state.
- **`0x361` (Direct Measurements & Status, 500 ms):** 2 bytes per wheel (FL, FR, RR, RL) carrying a 2-bit diagnostic status and a 14-bit physical pressure scaled by 0.1 Bar/LSB.
- **`0x3A1` (Direct Pressures / `MSG_DONNEES_PRESSION_ROUES`, 250 ms):** 1 byte per wheel scaled by 0.05 Bar/LSB.
- **`0x168` Byte 1 (Combine Warning Overlay):** Bits 7..6 carry instrument cluster TPMS warning lamp illumination flags.

```
+------------------------------------------------------------------------------------+
|                         Peugeot 407 Direct TPMS Wheel Sensors                      |
|             [0x1E1 Status Enum, 0x361 (14-bit 0.1 Bar), 0x3A1 (0.05 Bar)]          |
+------------------------------------------------------------------------------------+
                                          │
                               [PSA CAN 0x361 / 0x1E1]
                                          ▼
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Extracts 4-wheel numeric pressures (0.1 Bar / 0.05 Bar) from 0x361 / 0x3A1    |
|   2. Classifies discrete alarm states (Under-inflation, Puncture, Battery Low)     |
|   3. Dispatches Hiworld 0x18 (Discrete) & 0x66 / 0x68 (Numeric TPMS)               |
+------------------------------------------------------------------------------------+
                                          │
                    [UART Serial: 38400 baud, 8N1 / Hiworld Protocol]
                                          ▼
+------------------------------------------------------------------------------------+
|                       Android Headunit Vehicle Info / TPMS App                     |
|           (Renders 4-wheel car model with real-time numeric Bar/PSI)               |
+------------------------------------------------------------------------------------+
```

---

# 2. PSA CAN Bus Bitfield Specification

### 2.1 PSA Direct Tire Telemetry Frame (`0x361`)
- **CAN ID:** `0x361` (Standard 11-bit Identifier, DLC: 8, Period: 500 ms)
- **Slot Encoding (2 Bytes per Wheel):**
  - Bytes 0–1: Front Left (FL)
  - Bytes 2–3: Front Right (FR)
  - Bytes 4–5: Rear Right (RR)
  - Bytes 6–7: Rear Left (RL)

```
+---------------+--------+------------------------------------+-----------------------------+
| Byte Range    | Bit    | Function / Signal Name             | Bit Mask / Value Definition |
+---------------+--------+------------------------------------+-----------------------------+
| High Byte     | 15..14 | Wheel Diagnostic State             | 00 = OK                     |
|               |        |                                    | 01 = UNDER_INFLATED         |
|               |        |                                    | 10 = PUNCTURE               |
|               |        |                                    | 11 = SENSOR_FAULT / NO_DATA |
| High + Low    | 13..0  | Physical Tire Pressure Raw         | 0..16383 (0.1 Bar per LSB)  |
|               |        |                                    | 0x3FFF = Sensor Missing     |
+---------------+--------+------------------------------------+-----------------------------+
```

### 2.2 PSA Wheel Status Enum Frame (`0x1E1`)
- **CAN ID:** `0x1E1` (Standard 11-bit Identifier, DLC: 8, Period: 250 ms)
- `Byte 0 (FL)`: `(state & 0x07) << 3`
- `Byte 1 (FR)`: `(state & 0x07) << 3`
- `Byte 2 (RR)`: `(state & 0x07) << 3`
- `Byte 3 (RL)`: `(state & 0x07) << 3`
- `Byte 4 (Spare)`: `(state & 0x07) << 3`
- `Byte 5`: `tpms_system_state` (0x20 = nominal)

### 2.3 PSA Direct Pressures in 0.05 Bar (`0x3A1`)
- **CAN ID:** `0x3A1` (DLC: 8, Period: 250 ms)
- `Byte 0 (FL)`: $\text{Round}(\text{Pressure}_{\text{bar}} / 0.05)$
- `Byte 1 (FR)`: $\text{Round}(\text{Pressure}_{\text{bar}} / 0.05)$
- `Byte 2 (RR)`: $\text{Round}(\text{Pressure}_{\text{bar}} / 0.05)$
- `Byte 3 (RL)`: $\text{Round}(\text{Pressure}_{\text{bar}} / 0.05)$

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Hiworld Discrete TPMS Alarm (`Cmd 0x18`)
- **Sync Header:** `0x5A 0xA5`
- **Length ($L$):** `0x04` (4 payload bytes)
- **Command ID:** `0x18`
- **Payload Layout (4 Bytes):**
  - `Byte 0`: Front Left Alarm (`0x00` = Normal, `0x01` = Low Pressure, `0x02` = Puncture, `0x03` = Offline)
  - `Byte 1`: Front Right Alarm
  - `Byte 2`: Rear Left Alarm
  - `Byte 3`: Rear Right Alarm
- **Checksum:** `((Length + CmdID + sum(Payload)) - 1) & 0xFF`

### 3.2 Hiworld Universal Direct Numeric Pressures (`Cmd 0x66`)
- **Sync Header:** `0x5A 0xA5`
- **Length ($L$):** `0x06` (6 payload bytes)
- **Command ID:** `0x66`
- **Payload Layout (6 Bytes):**
  - `Byte 0`: Mode Flag (`0x01` = Live numeric reading)
  - `Byte 1`: FL Pressure in $0.1\text{ Bar}$ (e.g. `24` = $2.4\text{ Bar}$)
  - `Byte 2`: FR Pressure in $0.1\text{ Bar}$
  - `Byte 3`: RL Pressure in $0.1\text{ Bar}$
  - `Byte 4`: RR Pressure in $0.1\text{ Bar}$
  - `Byte 5`: Pressure Unit (`0x00` = Bar, `0x01` = PSI, `0x02` = kPa)
- **Checksum:** `((Length + CmdID + sum(Payload)) - 1) & 0xFF`

---

# 4. Pure C99 Firmware Implementation

```c
#ifndef CANBOX_TPMS_H
#define CANBOX_TPMS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define HIWORLD_SOF1              0x5A
#define HIWORLD_SOF2              0xA5
#define HIWORLD_CMD_TPMS_NUMERIC  0x66
#define HIWORLD_CMD_TPMS_DISCRETE 0x18

typedef struct {
    uint8_t fl_press_bar_deci; /* 0.1 Bar: 24 = 2.4 Bar */
    uint8_t fr_press_bar_deci;
    uint8_t rl_press_bar_deci;
    uint8_t rr_press_bar_deci;
    uint8_t fl_state;          /* 0=OK, 1=LOW, 2=PUNCTURE, 3=FAULT */
    uint8_t fr_state;
    uint8_t rl_state;
    uint8_t rr_state;
} psa_tpms_state_t;

typedef void (*canbox_uart_tx_fn)(const uint8_t *buf, size_t len);

typedef struct {
    psa_tpms_state_t  state;
    canbox_uart_tx_fn uart_tx;
} psa_tpms_ctx_t;

static inline void psa_tpms_init(psa_tpms_ctx_t *ctx, canbox_uart_tx_fn uart_tx) {
    memset(ctx, 0, sizeof(psa_tpms_ctx_t));
    ctx->state.fl_press_bar_deci = 24; /* 2.4 Bar default */
    ctx->state.fr_press_bar_deci = 24;
    ctx->state.rr_press_bar_deci = 22; /* 2.2 Bar default */
    ctx->state.rl_press_bar_deci = 22;
    ctx->uart_tx = uart_tx;
}

/* Transmit Hiworld Numeric TPMS Frame (Cmd 0x66) */
static inline void psa_tpms_send_numeric(psa_tpms_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[11];
    p[0] = HIWORLD_SOF1;
    p[1] = HIWORLD_SOF2;
    p[2] = 0x06;                     /* Length: 6 Payload bytes */
    p[3] = HIWORLD_CMD_TPMS_NUMERIC; /* Cmd 0x66 */
    p[4] = 0x01;                     /* Mode: Live */
    p[5] = ctx->state.fl_press_bar_deci;
    p[6] = ctx->state.fr_press_bar_deci;
    p[7] = ctx->state.rl_press_bar_deci;
    p[8] = ctx->state.rr_press_bar_deci;
    p[9] = 0x00;                     /* Unit: Bar */

    uint8_t sum = 0;
    for (size_t i = 2; i <= 9; i++) {
        sum += p[i];
    }
    p[10] = (uint8_t)((sum - 1) & 0xFF);

    ctx->uart_tx(p, 11);
}

/* Transmit Hiworld Discrete TPMS Alarm Frame (Cmd 0x18) */
static inline void psa_tpms_send_discrete(psa_tpms_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[9];
    p[0] = HIWORLD_SOF1;
    p[1] = HIWORLD_SOF2;
    p[2] = 0x04;                      /* Length: 4 Payload bytes */
    p[3] = HIWORLD_CMD_TPMS_DISCRETE; /* Cmd 0x18 */
    p[4] = ctx->state.fl_state;
    p[5] = ctx->state.fr_state;
    p[6] = ctx->state.rl_state;
    p[7] = ctx->state.rr_state;

    uint8_t sum = 0;
    for (size_t i = 2; i <= 7; i++) {
        sum += p[i];
    }
    p[8] = (uint8_t)((sum - 1) & 0xFF);

    ctx->uart_tx(p, 9);
}

/* Process PSA CAN 0x361 Frame */
static inline void psa_tpms_process_can_0x361(psa_tpms_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 8) return;

    uint16_t fl_raw = ((uint16_t)data[0] << 8) | data[1];
    uint16_t fr_raw = ((uint16_t)data[2] << 8) | data[3];
    uint16_t rr_raw = ((uint16_t)data[4] << 8) | data[5];
    uint16_t rl_raw = ((uint16_t)data[6] << 8) | data[7];

    ctx->state.fl_state = (fl_raw >> 14) & 0x03;
    ctx->state.fr_state = (fr_raw >> 14) & 0x03;
    ctx->state.rr_state = (rr_raw >> 14) & 0x03;
    ctx->state.rl_state = (rl_raw >> 14) & 0x03;

    if (ctx->state.fl_state != 3 && (fl_raw & 0x3FFF) != 0x3FFF) {
        ctx->state.fl_press_bar_deci = (uint8_t)(fl_raw & 0x3FFF);
    }
    if (ctx->state.fr_state != 3 && (fr_raw & 0x3FFF) != 0x3FFF) {
        ctx->state.fr_press_bar_deci = (uint8_t)(fr_raw & 0x3FFF);
    }
    if (ctx->state.rr_state != 3 && (rr_raw & 0x3FFF) != 0x3FFF) {
        ctx->state.rr_press_bar_deci = (uint8_t)(rr_raw & 0x3FFF);
    }
    if (ctx->state.rl_state != 3 && (rl_raw & 0x3FFF) != 0x3FFF) {
        ctx->state.rl_press_bar_deci = (uint8_t)(rl_raw & 0x3FFF);
    }

    psa_tpms_send_numeric(ctx);
    psa_tpms_send_discrete(ctx);
}

/* Process PSA CAN 0x3A1 Frame (0.05 Bar LSB) */
static inline void psa_tpms_process_can_0x3a1(psa_tpms_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 4) return;

    ctx->state.fl_press_bar_deci = (uint8_t)((data[0] * 5 + 5) / 10);
    ctx->state.fr_press_bar_deci = (uint8_t)((data[1] * 5 + 5) / 10);
    ctx->state.rr_press_bar_deci = (uint8_t)((data[2] * 5 + 5) / 10);
    ctx->state.rl_press_bar_deci = (uint8_t)((data[3] * 5 + 5) / 10);

    psa_tpms_send_numeric(ctx);
}

#endif /* CANBOX_TPMS_H */
```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: Nominal Pressures on CAN 0x361 (FL=2.4, FR=2.4, RR=2.2, RL=2.2 Bar)
- **CAN ID `0x361` Injection:**
  ```bash
  cansend vcan0 361#0018001800160016
  ```
- **Expected UART Output (Hiworld `0x66`):**
  - Frame: `5A A5 06 66 01 18 18 16 16 00 D2`
