# CAN Box Protocol Specification: Dual-Zone Climate Control (HVAC)
## Topic 02: Peugeot 407 Dual-Zone Climate & Touchscreen Overrides
**Document File:** `CANBOX_SPEC_HIWORLD_407_02_DUAL_ZONE_HVAC.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** `https://github.com/fazerxlo/canbox/tree/main/doc/CAN2004_clima.md`, `CAN_messages.md`

---

# 1. Functional Domain & Architecture Overview

The Peugeot 407 features an electronic dual-zone automatic climate control system. The HVAC ECU broadcasts comprehensive system telemetry (temperatures, blower level, distribution, compressor status) over the PSA Comfort CAN bus on ID `0x1D0`. The CAN adapter translates this data into headunit serial telemetry and accepts downlink control commands from the Android touchscreen (e.g. temperature adjustments, fan speed changes, mode toggling) to inject corresponding CAN control frames into the vehicle network.
The Peugeot 407 automatic climate control system drives three CAN frames on the PSA Comfort CAN bus:
- **`0x1D0` (Climate Panel State, 500 ms):** Transmits operational mode, fan speed, dual air distribution, recirculation state, rear defrost, and zone temperature indices.
- **`0x1E3` (Climate EMF / Display State, 200 ms):** Carries A/C compressor state, dual mode, front demist, cabin air recycling popup notification triggers (`intake_notify`), and display indices.
- **`0x12D` (Climate Command / Compressor Capability, 500 ms):** Carries compressor clutch state (`0x80`=engaged, `0x00`=off).

The CAN adapter decodes `0x1D0` and `0x1E3` into standardized Hiworld HVAC frames (`Cmd 0x31`) and accepts touchscreen overrides (`Cmd 0x8A`) to inject corresponding control frames back to the vehicle.

```
+------------------------------------------------------------------------------------+
|                         Peugeot 407 HVAC Electronic Unit                           |
|       [Driver/Pass Temps, 8-Speed Fan, Dual/Mono, Auto, AQS, Air Distribution]     |
|         [Panel State (0x1D0), EMF Display (0x1E3), Compressor State (0x12D)]       |
+------------------------------------------------------------------------------------+
                         │ (Uplink 0x1D0)           ▲ (Downlink 0x1E0)
                         │ (Uplink 0x1D0 / 0x1E3)   ▲ (Downlink Overrides)
                         ▼                          │
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Parses PSA CAN ID 0x1D0 (200 ms periodic / on state change)                   |
|   2. Formats & serializes Hiworld / Raise HVAC telemetry packets                   |
|   3. Decodes Android touchscreen downlink commands (Hiworld 0x8A)                  |
|   4. Synthesizes PSA CAN 0x1E0 control frames                                      |
|   1. Parses CAN 0x1D0 (Modes: 0x08 Auto, 0x28 Manual, 0x19 Demist, 0xA8 Standby)  |
|   2. Converts Peugeot 23-step Temp Index (0..22) to 0.5 deg C units / LO / HI      |
|   3. Serializes Hiworld 0x31 HVAC telemetry for Android popup bar                  |
|   4. Decodes Android touch adjustments (Cmd 0x8A) -> CAN requests                  |
+------------------------------------------------------------------------------------+
                         │ (UART Telemetry)         ▲ (UART Commands)
                         ▼                          │
+------------------------------------------------------------------------------------+
|                     Android Headunit Climate Overlay / App                         |
|     (Displays popup bar on HVAC change; accepts user touch input adjustments)      |
+------------------------------------------------------------------------------------+
```

---

# 2. PSA CAN Bus Bitfield Specification

### 2.1 Uplink Telemetry Frame (`0x1D0`)
- **CAN ID:** `0x1D0` (Standard 11-bit Identifier)
- **DLC:** 8 bytes
- **Transmission Cycle:** Periodic 200 ms or immediately upon user button/rotary actuation
### 2.1 PSA Climate Panel State Frame (`0x1D0`)
- **CAN ID:** `0x1D0` (DLC: 8, Period: 500 ms)
- **Standby Frame (Ignition ON, Fan=0):** `A8 00 0F 00 00 <TempLeft> <TempRight> 00`
- **Active Byte Layout:**

```
+--------+--------+------------------------------------+-----------------------------+
| Byte   | Bit    | Function / Signal Name             | Bit Mask / Value Definition |
| Byte   | Bit    | Function / Signal Name             | Value / Encoding Definition |
+--------+--------+------------------------------------+-----------------------------+
| Byte 0 | Bit 7  | System Power                       | 0x80 (1 = ON, 0 = OFF)      |
|        | Bit 6  | A/C Compressor Request             | 0x40 (1 = Active / ON)      |
|        | Bit 5  | Air Recirculation                  | 0x20 (1 = Recirc, 0 = Fresh)|
|        | Bit 4  | AQS Auto-Recirculation             | 0x10 (1 = Auto AQS Active)  |
|        | Bit 3  | Auto Climate Regulation            | 0x08 (1 = Auto Mode)        |
|        | Bit 2  | Dual-Zone Mode                     | 0x04 (1 = Dual, 0 = Mono)   |
|        | Bit 0  | Rear Screen / Mirror Demist        | 0x01 (1 = Active)           |
| Byte 0 | 7..0   | Climate Mode Flags                 | 0x08 = AUTO mode            |
|        |        |                                    | 0x28 = Manual mode          |
|        |        |                                    | 0x19 = Front Demist active  |
|        |        |                                    | 0xA8 = Standby (System Off) |
+--------+--------+------------------------------------+-----------------------------+
| Byte 1 | Bit 7  | Driver Windshield Defrost Vent     | 0x80 (1 = Open)             |
|        | Bit 6  | Driver Center / Face Vent          | 0x40 (1 = Open)             |
|        | Bit 5  | Driver Footwell Vent               | 0x20 (1 = Open)             |
|        | Bit 4  | Auto Blower Intensity High         | 0x10 (1 = Fast Profile)     |
|        | Bit 3..0| Blower Fan Speed Level            | 0..8 (0 = Off, 1..8 Speed)  |
| Byte 1 | 7..0   | Constant Padding                   | 0x00                        |
+--------+--------+------------------------------------+-----------------------------+
| Byte 2 | Bits 7..0| Driver Set Temperature Raw Code  | 0x00=LO, 0xFF=HI, 28..60    |
| Byte 2 | 3..0   | Fan Speed Raw Nibble               | 0x0F = Off / Standby        |
|        |        |                                    | 0x00..0x07 = Fan Level 1..8 |
+--------+--------+------------------------------------+-----------------------------+
| Byte 3 | Bits 7..0| Passenger Set Temperature Raw Code| 0x00=LO, 0xFF=HI, 28..60    |
| Byte 3 | 7..4   | Left Zone Air Distribution Code    | See Air Distribution Table  |
|        | 3..0   | Right Zone Air Distribution Code   | See Air Distribution Table  |
+--------+--------+------------------------------------+-----------------------------+
| Byte 4 | Bit 7  | Max Front Windshield Demist        | 0x80 (1 = Active)           |
|        | Bit 3  | Max A/C Fast Cool                  | 0x08 (1 = Active)           |
|        | Bit 0  | Temperature Scale Unit             | 0x01 (0 = Celsius, 1 = Fahr)|
| Byte 4 | Bit 5  | Explicit Intake Fresh Air          | 1 = Forced Fresh Air (0x20) |
|        | Bit 4  | Cabin Air Recirculation (Recycle)  | 1 = Cabin Recirc (0x10)     |
|        | Bit 0  | Rear Screen Demist (Heated Mirrors)| 1 = Rear Demist Active(0x01)|
+--------+--------+------------------------------------+-----------------------------+
| Byte 5 | Bits 7..0| Reserved / Rear Zone Flags       | 0x00                        |
| Byte 5 | 7..0   | Left Zone Set Temperature Index    | Index 0..22 (See Temp Table)|
+--------+--------+------------------------------------+-----------------------------+
| Byte 6 | Bit 7  | Passenger Windshield Defrost Vent  | 0x80 (1 = Open)             |
|        | Bit 6  | Passenger Center / Face Vent       | 0x40 (1 = Open)             |
|        | Bit 5  | Passenger Footwell Vent            | 0x20 (1 = Open)             |
|        | Bits 4..0| Reserved                         | 0x00                        |
| Byte 6 | 7..0   | Right Zone Set Temperature Index   | Index 0..22 (See Temp Table)|
+--------+--------+------------------------------------+-----------------------------+
| Byte 7 | Bits 7..0| Checksum / Reserved              | 0x00                        |
| Byte 7 | 7..0   | Constant Padding                   | 0x00                        |
+--------+--------+------------------------------------+-----------------------------+
```

### 2.2 Temperature Scaling Equations
$$\text{Temperature } (^\circ\text{C}) = \frac{\text{RawByte}}{2.0}$$
$$\text{RawByte} = \text{Round}(T_{^\circ\text{C}} \times 2.0)$$
- **Range:** $14.0^\circ\text{C} \dots 30.0^\circ\text{C}$ (Raw $28 \dots 60$, Step: $0.5^\circ\text{C}$).
- **Special Values:**
  - `0x00` = Display **"LO"** ($< 14.0^\circ\text{C}$)
  - `0xFF` = Display **"HI"** ($> 30.0^\circ\text{C}$)
### 2.2 Temperature Index Table (`0x1D0` Bytes 5 & 6)
| Index | Display | Raw Android Celsius | Index | Display | Raw Android Celsius |
|:---:|:---:|:---:|:---:|:---:|:---:|
| `0` | **MIN / LO** | `0x00` (LO) | `12` | **21.5 °C** | $43 = \text{0x2B}$ |
| `1` | **14.0 °C** | $28 = \text{0x1C}$ | `13` | **22.0 °C** | $44 = \text{0x2C}$ |
| `2` | **15.0 °C** | $30 = \text{0x1E}$ | `14` | **22.5 °C** | $45 = \text{0x2D}$ |
| `3` | **16.0 °C** | $32 = \text{0x20}$ | `15` | **23.0 °C** | $46 = \text{0x2E}$ |
| `4` | **17.0 °C** | $34 = \text{0x22}$ | `16` | **23.5 °C** | $47 = \text{0x2F}$ |
| `5` | **18.0 °C** | $36 = \text{0x24}$ | `17` | **24.0 °C** | $48 = \text{0x30}$ |
| `6` | **18.5 °C** | $37 = \text{0x25}$ | `18` | **25.0 °C** | $50 = \text{0x32}$ |
| `7` | **19.0 °C** | $38 = \text{0x26}$ | `19` | **26.0 °C** | $52 = \text{0x34}$ |
| `8` | **19.5 °C** | $39 = \text{0x27}$ | `20` | **27.0 °C** | $54 = \text{0x36}$ |
| `9` | **20.0 °C** | $40 = \text{0x28}$ | `21` | **28.0 °C** | $56 = \text{0x38}$ |
| `10` | **20.5 °C** | $41 = \text{0x29}$ | `22` | **MAX / HI** | `0xFF` (HI) |
| `11` | **21.0 °C** | $42 = \text{0x2A}$ | | | |

### 2.3 Air Distribution Codes (`0x1D0` Byte 3)
| Nibble Code | Vent Target | Android UI Flags (`DrvUp`, `DrvFace`, `DrvDown`) |
|:---:|:---|:---|
| `0x00` | **AUTO** (Automatic Distribution) | Auto distribution |
| `0x02` | **Down** (Floor / Footwell vents) | `DrvDown = 1` |
| `0x03` | **Front** (Windshield defrost vent) | `DrvUp = 1` |
| `0x04` | **Up / Face** (Dashboard center vents)| `DrvFace = 1` |
| `0x05` | **Front + Down** (Defrost + Floor) | `DrvUp = 1`, `DrvDown = 1` |
| `0x06` | **Up + Down** (Face + Floor) | `DrvFace = 1`, `DrvDown = 1` |
| `0x07` | **All** (Face + Floor + Defrost) | `DrvUp = 1`, `DrvFace = 1`, `DrvDown = 1` |

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Hiworld HVAC Telemetry Frame (`Cmd 0x31`)
- **Header:** `0x5A 0xA5`
- **Length:** `0x08` (1-byte Cmd + 7-byte Payload)
- **Command ID:** `0x31`
- **Header:** `0x5A 0xA5` | **Length:** `0x08` | **Command ID:** `0x31`
- **Payload Layout (7 Bytes):**
  - `Byte 0`: `[Power:b7] [AC:b6] [Recirc:b5] [AQS:b4] [Auto:b3] [Dual:b2] [RearDefrost:b0]`
  - `Byte 1`: `[DrvUp:b7] [DrvFace:b6] [DrvDown:b5] [BlowerSpeed:b3..b0 (0..8)]`
  - `Byte 2`: `[DriverTemp: Raw 0x00, 0xFF, 28..60]`
  - `Byte 3`: `[PassTemp: Raw 0x00, 0xFF, 28..60]`
  - `Byte 4`: `[FrontMaxDefrost:b7] [MaxAC:b3] [TempUnit:b0 (0=C, 1=F)]`
  - `Byte 5`: `[RearPower:b7] [Reserved:b6..b0]`
  - `Byte 2`: Driver Set Temp ($0\text{x00}=\text{LO}, 0\text{xFF}=\text{HI}, 28 \dots 60$)
  - `Byte 3`: Passenger Set Temp ($0\text{x00}=\text{LO}, 0\text{xFF}=\text{HI}, 28 \dots 60$)
  - `Byte 4`: `[FrontMaxDefrost:b7] [MaxAC:b3] [TempUnit:b0]`
  - `Byte 5`: `0x00`
  - `Byte 6`: `[PassUp:b7] [PassFace:b6] [PassDown:b5] [AutoProfile:b4..b3]`
- **Checksum:** 8-bit sum modulo 256 over `Length + CmdID + Payload (Bytes 2..9)`.
- **Checksum:** 8-bit sum modulo 256 over `Length + CmdID + Payload`.

### 3.2 Touchscreen Downlink Control (`Cmd 0x8A`)
When the user adjusts settings on the Android touchscreen, the headunit sends `Cmd 0x8A`:
- **Format:** `0x5A 0xA5 0x03 0x8A [FunctionCode] [Value] [Checksum]`
- **Function Codes:**
  - `0x01`: Power Toggle (`0x01`=On, `0x00`=Off)
  - `0x02`: A/C Toggle (`0x01`=On, `0x00`=Off)
  - `0x03`: Auto Mode Toggle (`0x01`=Enable)
  - `0x04`: Dual / Mono Toggle
  - `0x05`: Blower Speed Set (`Value` = $0 \dots 8$)
  - `0x06`: Driver Temp Increment (+0.5°C) / Decrement (-0.5°C)
  - `0x07`: Passenger Temp Increment / Decrement
  - `0x08`: Air Distribution Driver (`0x01`=Up, `0x02`=Face, `0x03`=Down, `0x04`=Auto)
  - `0x09`: Max Front Defrost Toggle
  - `0x0A`: Rear Screen Demist Toggle

---

# 4. Pure C99 Firmware Implementation

```c
#ifndef CANBOX_HVAC_H
#define CANBOX_HVAC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* HVAC Data Model */
/* Lookup table converting Peugeot Index (0..22) to 0.5C Raw (28..60, 0x00=LO, 0xFF=HI) */
static const uint8_t PSA_INDEX_TO_RAW[23] = {
    0x00, /* 0: LO */
    28,   /* 1: 14.0C */
    30,   /* 2: 15.0C */
    32,   /* 3: 16.0C */
    34,   /* 4: 17.0C */
    36,   /* 5: 18.0C */
    37,   /* 6: 18.5C */
    38,   /* 7: 19.0C */
    39,   /* 8: 19.5C */
    40,   /* 9: 20.0C */
    41,   /* 10: 20.5C */
    42,   /* 11: 21.0C */
    43,   /* 12: 21.5C */
    44,   /* 13: 22.0C */
    45,   /* 14: 22.5C */
    46,   /* 15: 23.0C */
    47,   /* 16: 23.5C */
    48,   /* 17: 24.0C */
    50,   /* 18: 25.0C */
    52,   /* 19: 26.0C */
    54,   /* 20: 27.0C */
    56,   /* 21: 28.0C */
    0xFF  /* 22: HI */
};

typedef struct {
    bool    power;
    bool    ac_compressor;
    bool    recirculation;
    bool    aqs_auto;
    bool    auto_mode;
    bool    dual_mode;
    bool    rear_defrost;
    bool    front_max_defrost;
    bool    ac_max;
    bool    temp_unit_fahrenheit;
    uint8_t blower_speed;       /* 0..8 */
    uint8_t auto_profile;       /* 0=Soft, 1=Normal, 2=Fast */
    uint8_t blower_speed; /* 0..8 */
    
    /* Airflow Distribution */
    bool    driver_wind_up;
    bool    driver_wind_face;
    bool    driver_wind_down;
    bool    pass_wind_up;
    bool    pass_wind_face;
    bool    pass_wind_down;

    /* Temperatures: Raw (0x00=LO, 0xFF=HI, 28..60 for 14.0C..30.0C) */
    uint8_t driver_temp_raw;
    uint8_t pass_temp_raw;
} psa_hvac_state_t;

/* Serial Output Callback */
typedef void (*canbox_uart_tx_fn)(const uint8_t *buf, size_t len);
typedef void (*canbox_can_tx_fn)(uint32_t id, const uint8_t *data, uint8_t dlc);

typedef struct {
    psa_hvac_state_t state;
    psa_hvac_state_t  state;
    canbox_uart_tx_fn uart_tx;
    canbox_can_tx_fn  can_tx;
} psa_hvac_ctx_t;

/* Initialize HVAC Context */
static inline void psa_hvac_init(psa_hvac_ctx_t *ctx, canbox_uart_tx_fn uart_tx, canbox_can_tx_fn can_tx) {
static inline void psa_hvac_init(psa_hvac_ctx_t *ctx, canbox_uart_tx_fn uart_tx) {
    memset(ctx, 0, sizeof(psa_hvac_ctx_t));
    ctx->uart_tx = uart_tx;
    ctx->can_tx  = can_tx;
}

/* Serialize and transmit Hiworld HVAC packet (0x31) */
static inline void psa_hvac_send_hiworld(psa_hvac_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[11];
    uint8_t p[12];
    p[0] = 0x5A;
    p[1] = 0xA5;
    p[2] = 0x08; /* Length: 1 Cmd + 7 Payload */
    p[3] = 0x31; /* Cmd: HVAC */

    /* Byte 0 */
    uint8_t b0 = 0;
    if (ctx->state.power)         b0 |= 0x80;
    if (ctx->state.ac_compressor) b0 |= 0x40;
    if (ctx->state.recirculation) b0 |= 0x20;
    if (ctx->state.aqs_auto)      b0 |= 0x10;
    if (ctx->state.auto_mode)     b0 |= 0x08;
    if (ctx->state.dual_mode)     b0 |= 0x04;
    if (ctx->state.rear_defrost)  b0 |= 0x01;
    p[4] = b0;

    /* Byte 1 */
    uint8_t b1 = ctx->state.blower_speed & 0x0F;
    if (ctx->state.driver_wind_up)   b1 |= 0x80;
    if (ctx->state.driver_wind_face) b1 |= 0x40;
    if (ctx->state.driver_wind_down) b1 |= 0x20;
    p[5] = b1;

    /* Byte 2 & 3: Temperatures */
    p[6] = ctx->state.driver_temp_raw;
    p[7] = ctx->state.pass_temp_raw;

    /* Byte 4 */
    uint8_t b4 = 0;
    if (ctx->state.front_max_defrost)     b4 |= 0x80;
    if (ctx->state.ac_max)                b4 |= 0x08;
    if (ctx->state.temp_unit_fahrenheit)  b4 |= 0x01;
    if (ctx->state.front_max_defrost) b4 |= 0x80;
    p[8] = b4;

    /* Byte 5 */
    p[9] = 0x00;

    /* Byte 6 */
    uint8_t b6 = 0;
    if (ctx->state.pass_wind_up)   b6 |= 0x80;
    if (ctx->state.pass_wind_face) b6 |= 0x40;
    if (ctx->state.pass_wind_down) b6 |= 0x20;
    b6 |= (ctx->state.auto_profile & 0x03) << 3;
    p[10] = b6;

    /* Checksum: Sum over p[2]..p[10] */
    uint8_t sum = 0;
    for (size_t i = 2; i <= 10; i++) {
        sum += p[i];
    }
    for (size_t i = 2; i <= 10; i++) sum += p[i];
    p[11] = sum;

    ctx->uart_tx(p, 12);
}

/* Parse PSA CAN ID 0x1D0 Frame */
static inline void psa_hvac_process_can(psa_hvac_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
static inline void psa_hvac_decode_distribution(uint8_t code, bool *up, bool *face, bool *down) {
    *up = false; *face = false; *down = false;
    switch (code) {
        case 0x02: *down = true; break;
        case 0x03: *up = true; break;
        case 0x04: *face = true; break;
        case 0x05: *up = true; *down = true; break;
        case 0x06: *face = true; *down = true; break;
        case 0x07: *up = true; *face = true; *down = true; break;
        default: break; /* Auto / 0x00 */
    }
}

/* Process PSA CAN 0x1D0 (Climate Panel State) */
static inline void psa_hvac_process_can_0x1D0(psa_hvac_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 7) return;

    ctx->state.power         = (data[0] & 0x80) ? true : false;
    ctx->state.ac_compressor = (data[0] & 0x40) ? true : false;
    ctx->state.recirculation = (data[0] & 0x20) ? true : false;
    ctx->state.aqs_auto      = (data[0] & 0x10) ? true : false;
    ctx->state.auto_mode     = (data[0] & 0x08) ? true : false;
    ctx->state.dual_mode     = (data[0] & 0x04) ? true : false;
    ctx->state.rear_defrost  = (data[0] & 0x01) ? true : false;
    uint8_t mode = data[0];
    if (mode == 0xA8) {
        /* Standby / Off */
        ctx->state.power = false;
        ctx->state.blower_speed = 0;
    } else {
        ctx->state.power = true;
        ctx->state.auto_mode = (mode == 0x08);
        ctx->state.front_max_defrost = (mode == 0x19);

    ctx->state.driver_wind_up   = (data[1] & 0x80) ? true : false;
    ctx->state.driver_wind_face = (data[1] & 0x40) ? true : false;
    ctx->state.driver_wind_down = (data[1] & 0x20) ? true : false;
    ctx->state.blower_speed     = data[1] & 0x0F;
        /* Fan Speed */
        uint8_t raw_fan = data[2] & 0x0F;
        ctx->state.blower_speed = (raw_fan == 0x0F) ? 0 : (raw_fan + 1);

    ctx->state.driver_temp_raw = data[2];
    ctx->state.pass_temp_raw   = data[3];
        /* Air Distribution */
        psa_hvac_decode_distribution((data[3] >> 4) & 0x0F, &ctx->state.driver_wind_up, &ctx->state.driver_wind_face, &ctx->state.driver_wind_down);
        psa_hvac_decode_distribution(data[3] & 0x0F, &ctx->state.pass_wind_up, &ctx->state.pass_wind_face, &ctx->state.pass_wind_down);

    ctx->state.front_max_defrost = (data[4] & 0x80) ? true : false;
    ctx->state.ac_max            = (data[4] & 0x08) ? true : false;
    ctx->state.temp_unit_fahrenheit = (data[4] & 0x01) ? true : false;
        /* Recirc & Rear Demist */
        ctx->state.recirculation = (data[4] & 0x10) ? true : false;
        ctx->state.rear_defrost  = (data[4] & 0x01) ? true : false;

    ctx->state.pass_wind_up   = (data[6] & 0x80) ? true : false;
    ctx->state.pass_wind_face = (data[6] & 0x40) ? true : false;
    ctx->state.pass_wind_down = (data[6] & 0x20) ? true : false;
        /* Temperature Indices */
        uint8_t idx_l = data[5] <= 22 ? data[5] : 11;
        uint8_t idx_r = data[6] <= 22 ? data[6] : 11;
        ctx->state.driver_temp_raw = PSA_INDEX_TO_RAW[idx_l];
        ctx->state.pass_temp_raw   = PSA_INDEX_TO_RAW[idx_r];
    }

    psa_hvac_send_hiworld(ctx);
}

/* Parse Downlink Touchscreen Overrides from Android Headunit */
static inline void psa_hvac_process_downlink(psa_hvac_ctx_t *ctx, uint8_t func_code, uint8_t value) {
    if (!ctx->can_tx) return;

    /* Build PSA CAN 0x1E0 Climate Control Request Frame */
    uint8_t can_data[8];
    memset(can_data, 0, sizeof(can_data));
    can_data[0] = func_code;
    can_data[1] = value;

    ctx->can_tx(0x1E0, can_data, 8);
}

#endif /* CANBOX_HVAC_H */
```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: Climate Active, 21.5°C Driver, 22.0°C Passenger, Auto ON, AC ON, Fan Level 4
- **CAN Input:**
  - $21.5^\circ\text{C} \implies 43 = \text{0x2B}$
  - $22.0^\circ\text{C} \implies 44 = \text{0x2C}$
  - `Byte 0`: `0xC8` (Power=1, AC=1, Auto=1)
  - `Byte 1`: `0x44` (Face Vent=1, Fan Speed=4)
### Vector 1: AUTO Mode, Fan Level 3 (raw 0x02), 21.0°C (Index 11 = 0x0B) Both Zones
- **CAN ID `0x1D0` Injection:**
  ```bash
  cansend vcan0 1D0#C8442B2C00004000
  cansend vcan0 1D0#08000200000B0B00
  ```
- **Expected UART Output (Hiworld):**
  - Frame bytes: `5A A5 08 31 C8 44 2B 2C 00 00 40 E6`
- **Expected UART Output (Hiworld `0x31`):**
  - Driver & Passenger Temp: $21.0^\circ\text{C} = 42 = \text{0x2A}$
  - Frame: `5A A5 08 31 88 03 2A 2A 00 00 00 48`

### Vector 2: Max Front Windshield Defrost Enabled
- **CAN Input:**
### Vector 2: Standby Mode (Fan Dragged to 0)
- **CAN ID `0x1D0` Injection:**
  ```bash
  cansend vcan0 1D0#C8882B2C80008000
  cansend vcan0 1D0#A8000F00000B0B00
  ```
- **Expected UART Output (Hiworld):**
  - Frame bytes: `5A A5 08 31 C8 88 2B 2C 80 00 80 6E`

### Vector 3: Android Touchscreen Set Blower Speed to Level 6 Downlink
- **UART Downlink Received:** `5A A5 03 8A 05 06 98`
- **Expected CAN Frame Injected:**
  - ID: `0x1E0`, DLC: 8, Data: `05 06 00 00 00 00 00 00`

  - Power=0, Fan=0: `5A A5 08 31 00 00 2A 2A 00 00 00 C0`
