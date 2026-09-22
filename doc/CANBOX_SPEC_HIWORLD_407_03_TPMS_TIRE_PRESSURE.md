# CAN Box Protocol Specification: Tire Pressure Monitoring System (TPMS)
## Topic 03: Peugeot 407 Direct & Discrete TPMS Telemetry
**Document File:** `CANBOX_SPEC_HIWORLD_407_03_TPMS_TIRE_PRESSURE.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)

---

# 1. Functional Domain & Architecture Overview

The Peugeot 407 features active TPMS wheel transmitters in all four road wheels (and optionally the spare wheel). The TPMS ECU / BSI receives 433 MHz RF broadcasts from each wheel sensor and bridges tire pressure, internal temperature, and alarm states onto the PSA Comfort CAN bus via CAN ID `0x385` and `0x221`.

```
+------------------------------------------------------------------------------------+
|                         Peugeot 407 Direct TPMS Wheel Sensors                      |
|                  [FL, FR, RL, RR Pressure (Bar), Temp (C), Alarms]                 |
+------------------------------------------------------------------------------------+
                                          │
                                 [PSA CAN 0x385 / 0x221]
                                          ▼
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Extracts 4-wheel numeric pressures (0.1 Bar) & temperatures (deg C)          |
|   2. Classifies discrete alarm states (Under-inflation, Puncture, Battery Low)    |
|   3. Dispatches Hiworld 0x18 (Discrete) & 0x66 / 0x68 (Numeric TPMS)               |
|   4. Accepts Android Calibration Reset Downlink (Cmd 0x80) -> Injects CAN 0x221    |
+------------------------------------------------------------------------------------+
                                          │
                   [UART Serial: 38400 baud, 8N1 / Hiworld Protocol]
                                          ▼
+------------------------------------------------------------------------------------+
|                       Android Headunit Vehicle Info / TPMS App                     |
|           (Renders 4-wheel car model with real-time numeric Bar/PSI & C)           |
+------------------------------------------------------------------------------------+
```

---

# 2. PSA CAN Bus Bitfield Specification

### 2.1 PSA Direct Tire Telemetry Frame (`0x385`)
- **CAN ID:** `0x385` (Standard 11-bit Identifier)
- **DLC:** 8 bytes
- **Cycle Rate:** Periodic 1000 ms or immediate on sudden pressure drop ($> 0.2\text{ Bar/min}$)

```
+--------+--------+------------------------------------+-----------------------------+
| Byte   | Bit    | Function / Signal Name             | Bit Mask / Value Definition |
+--------+--------+------------------------------------+-----------------------------+
| Byte 0 | 7..0   | Front Left (FL) Pressure Raw       | 0..250 (0.1 Bar per LSB)    |
+--------+--------+------------------------------------+-----------------------------+
| Byte 1 | 7..0   | Front Right (FR) Pressure Raw      | 0..250 (0.1 Bar per LSB)    |
+--------+--------+------------------------------------+-----------------------------+
| Byte 2 | 7..0   | Rear Left (RL) Pressure Raw        | 0..250 (0.1 Bar per LSB)    |
+--------+--------+------------------------------------+-----------------------------+
| Byte 3 | 7..0   | Rear Right (RR) Pressure Raw       | 0..250 (0.1 Bar per LSB)    |
+--------+--------+------------------------------------+-----------------------------+
| Byte 4 | 7..0   | Front Left (FL) Temperature Raw    | Raw - 40 = Degrees Celsius  |
+--------+--------+------------------------------------+-----------------------------+
| Byte 5 | 7..0   | Front Right (FR) Temperature Raw   | Raw - 40 = Degrees Celsius  |
+--------+--------+------------------------------------+-----------------------------+
| Byte 6 | 7..0   | Rear Left (RL) Temperature Raw     | Raw - 40 = Degrees Celsius  |
+--------+--------+------------------------------------+-----------------------------+
| Byte 7 | 7..0   | Rear Right (RR) Temperature Raw    | Raw - 40 = Degrees Celsius  |
+--------+--------+------------------------------------+-----------------------------+
```

### 2.2 PSA Discrete Alarm States Frame (`0x221` Byte 3)
```
+--------+--------+------------------------------------+-----------------------------+
| Byte 3 | Bit 7  | Spare Wheel Pressure Alarm         | 1 = Warning Active          |
|        | Bit 6  | Sensor Battery Low Warning         | 1 = Battery Low             |
|        | Bit 5  | Sensor RF Missing / Offline        | 1 = No Signal               |
|        | Bit 4  | Calibration / Learning Mode Active | 1 = In Progress             |
|        | Bit 3  | Rear Right (RR) Low / Puncture     | 1 = Fault / 0 = Normal      |
|        | Bit 2  | Rear Left (RL) Low / Puncture      | 1 = Fault / 0 = Normal      |
|        | Bit 1  | Front Right (FR) Low / Puncture    | 1 = Fault / 0 = Normal      |
|        | Bit 0  | Front Left (FL) Low / Puncture     | 1 = Fault / 0 = Normal      |
+--------+--------+------------------------------------+-----------------------------+
```

### 2.3 Physical Formulas:
$$\text{Pressure (Bar)} = \frac{\text{RawByte}}{10.0} \quad (\text{e.g. } 24 \implies 2.4\text{ Bar})$$
$$\text{Pressure (PSI)} = \text{Pressure (Bar)} \times 14.5038$$
$$\text{Pressure (kPa)} = \text{Pressure (Bar)} \times 100.0$$
$$\text{Temperature } (^\circ\text{C}) = \text{RawByte} - 40 \quad (\text{e.g. } 65 \implies +25^\circ\text{C})$$

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Hiworld Discrete TPMS Alarm (`Cmd 0x18`)
- **Sync Header:** `0x5A 0xA5`
- **Length:** `0x05` (1-byte Cmd + 4-byte Payload)
- **Command ID:** `0x18`
- **Payload Layout (4 Bytes):**
  - `Byte 0`: Front Left Alarm (`0x00`=Normal, `0x01`=Low Pressure, `0x02`=Puncture, `0x03`=Offline)
  - `Byte 1`: Front Right Alarm
  - `Byte 2`: Rear Left Alarm
  - `Byte 3`: Rear Right Alarm
- **Checksum:** 8-bit sum modulo 256 over `Length + CmdID + Payload`.

### 3.2 Hiworld Universal Direct Numeric Pressures (`Cmd 0x66`)
- **Length:** `0x07` (1-byte Cmd + 6-byte Payload)
- **Command ID:** `0x66`
- **Payload Layout (6 Bytes):**
  - `Byte 0`: Mode Flag (`0x01` = Live numeric reading)
  - `Byte 1`: FL Pressure in $0.1\text{ Bar}$ (e.g. `24` = $2.4\text{ Bar}$)
  - `Byte 2`: FR Pressure in $0.1\text{ Bar}$
  - `Byte 3`: RL Pressure in $0.1\text{ Bar}$
  - `Byte 4`: RR Pressure in $0.1\text{ Bar}$
  - `Byte 5`: Pressure Unit (`0x00` = Bar, `0x01` = PSI, `0x02` = kPa)

### 3.3 Hiworld Direct Tire Temperatures & Classification (`Cmd 0x68`)
- **Length:** `0x09` (1-byte Cmd + 8-byte Payload)
- **Command ID:** `0x68`
- **Payload Layout (8 Bytes):**
  - `Byte 0..3`: Tire Temperatures (FL, FR, RL, RR) as signed $^\circ\text{C}$ offset by $+40$ (e.g. $25^\circ\text{C} = 65 = \text{0x41}$).
  - `Byte 4..7`: Sensor Battery & RF Health status.

### 3.4 TPMS Calibration Reset Downlink (`Cmd 0x80`)
When the user clicks "Calibrate / Re-learn TPMS" in Android settings:
- **UART Command:** `5A A5 03 80 10 00 93`
- **CAN Action:** CAN box injects BSI calibration trigger frame into CAN ID `0x221`.

---

# 4. Pure C99 Firmware Implementation

```c
#ifndef CANBOX_TPMS_H
#define CANBOX_TPMS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

typedef enum {
    TPMS_ALARM_OK       = 0,
    TPMS_ALARM_LOW      = 1,
    TPMS_ALARM_PUNCTURE = 2,
    TPMS_ALARM_OFFLINE  = 3,
    TPMS_ALARM_BAT_LOW  = 4
} tpms_alarm_t;

typedef struct {
    uint16_t pressure_dbar[4]; /* FL=0, FR=1, RL=2, RR=3 in 0.1 Bar units */
    int16_t  temperature_c[4]; /* FL, FR, RL, RR in deg C */
    uint8_t  alarm_state[4];   /* tpms_alarm_t enum */
    bool     calibrating;
} psa_tpms_state_t;

typedef void (*canbox_uart_tx_fn)(const uint8_t *buf, size_t len);
typedef void (*canbox_can_tx_fn)(uint32_t id, const uint8_t *data, uint8_t dlc);

typedef struct {
    psa_tpms_state_t state;
    canbox_uart_tx_fn uart_tx;
    canbox_can_tx_fn  can_tx;
} psa_tpms_ctx_t;

static inline void psa_tpms_init(psa_tpms_ctx_t *ctx, canbox_uart_tx_fn uart_tx, canbox_can_tx_fn can_tx) {
    memset(ctx, 0, sizeof(psa_tpms_ctx_t));
    ctx->uart_tx = uart_tx;
    ctx->can_tx  = can_tx;
}

/* Transmit Hiworld Numeric Pressure Telemetry (Cmd 0x66) */
static inline void psa_tpms_send_numeric_hiworld(psa_tpms_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[10];
    p[0] = 0x5A;
    p[1] = 0xA5;
    p[2] = 0x07; /* Length: 1 Cmd + 6 Payload */
    p[3] = 0x66; /* Cmd ID */
    p[4] = 0x01; /* Mode: Live readings */
    p[5] = (uint8_t)(ctx->state.pressure_dbar[0]);
    p[6] = (uint8_t)(ctx->state.pressure_dbar[1]);
    p[7] = (uint8_t)(ctx->state.pressure_dbar[2]);
    p[8] = (uint8_t)(ctx->state.pressure_dbar[3]);
    p[9] = 0x00; /* Unit: 0.1 Bar */

    uint8_t sum = 0;
    for (size_t i = 2; i <= 9; i++) sum += p[i];
    p[10] = sum;

    ctx->uart_tx(p, 11);
}

/* Transmit Hiworld Discrete Alarm State Telemetry (Cmd 0x18) */
static inline void psa_tpms_send_discrete_hiworld(psa_tpms_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[8];
    p[0] = 0x5A;
    p[1] = 0xA5;
    p[2] = 0x05; /* Length: 1 Cmd + 4 Payload */
    p[3] = 0x18; /* Cmd ID */
    p[4] = ctx->state.alarm_state[0];
    p[5] = ctx->state.alarm_state[1];
    p[6] = ctx->state.alarm_state[2];
    p[7] = ctx->state.alarm_state[3];

    uint8_t sum = 0;
    for (size_t i = 2; i <= 7; i++) sum += p[i];
    p[8] = sum;

    ctx->uart_tx(p, 9);
}

/* Process PSA CAN ID 0x385 (Direct Pressures & Temperatures) */
static inline void psa_tpms_process_can_0x385(psa_tpms_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 8) return;

    for (int i = 0; i < 4; i++) {
        ctx->state.pressure_dbar[i] = data[i];
        ctx->state.temperature_c[i] = (int16_t)data[i + 4] - 40;
    }

    psa_tpms_send_numeric_hiworld(ctx);
}

/* Process PSA CAN ID 0x221 (Discrete Alarm Flags) */
static inline void psa_tpms_process_can_0x221(psa_tpms_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 4) return;
    uint8_t b3 = data[3];

    ctx->state.alarm_state[0] = (b3 & 0x01) ? TPMS_ALARM_LOW : TPMS_ALARM_OK;
    ctx->state.alarm_state[1] = (b3 & 0x02) ? TPMS_ALARM_LOW : TPMS_ALARM_OK;
    ctx->state.alarm_state[2] = (b3 & 0x04) ? TPMS_ALARM_LOW : TPMS_ALARM_OK;
    ctx->state.alarm_state[3] = (b3 & 0x08) ? TPMS_ALARM_LOW : TPMS_ALARM_OK;
    ctx->state.calibrating    = (b3 & 0x10) ? true : false;

    psa_tpms_send_discrete_hiworld(ctx);
}

/* Handle Android Downlink TPMS Calibration Command */
static inline void psa_tpms_process_downlink(psa_tpms_ctx_t *ctx, uint8_t cmd_param) {
    if (!ctx->can_tx) return;
    (void)cmd_param;

    /* Inject TPMS Re-calibration request into PSA BSI (CAN ID 0x221) */
    uint8_t can_data[8] = { 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00 };
    ctx->can_tx(0x221, can_data, 8);
}

#endif /* CANBOX_TPMS_H */
```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: Normal Pressures (FL=2.4 Bar, FR=2.4 Bar, RL=2.2 Bar, RR=2.2 Bar) @ 25°C
- **CAN ID `0x385` Injection:**
  - Pressures: `24 24 22 22` (`0x18 0x18 0x16 0x16`)
  - Temperatures ($25^\circ\text{C} + 40 = 65 = \text{0x41}$): `41 41 41 41`
  ```bash
  cansend vcan0 385#1818161641414141
  ```
- **Expected UART Output (Hiworld Numeric `0x66`):**
  - Frame: `5A A5 07 66 01 18 18 16 16 00 D0`

### Vector 2: Front Left Puncture / Low Pressure Alarm
- **CAN ID `0x221` Injection:**
  ```bash
  cansend vcan0 221#0000000100000000
  ```
- **Expected UART Output (Hiworld Discrete `0x18`):**
  - Frame: `5A A5 05 18 01 00 00 00 1E`

