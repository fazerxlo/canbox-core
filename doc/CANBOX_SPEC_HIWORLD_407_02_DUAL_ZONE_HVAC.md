# CAN Box Protocol Specification: Dual-Zone Climate Control (HVAC)
## Topic 02: Peugeot 407 Dual-Zone Climate & Touchscreen Overrides
**Document File:** `CANBOX_SPEC_HIWORLD_407_02_DUAL_ZONE_HVAC.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** `https://github.com/fazerxlo/canbox/tree/main/doc/CAN2004_clima.md`, `CAN_messages.md`

---

# 1. Functional Domain & Architecture Overview

The Peugeot 407 automatic climate control system drives three CAN frames on the PSA Comfort CAN bus:
- **`0x1D0` (Climate Panel State, 500 ms):** Transmits operational mode, fan speed, dual air distribution, recirculation state, rear defrost, and zone temperature indices.
- **`0x1E3` (Climate EMF / Display State, 200 ms):** Carries A/C compressor state, dual mode, front demist, cabin air recycling popup notification triggers (`intake_notify`), and display indices.
- **`0x12D` (Climate Command / Compressor Capability, 500 ms):** Carries compressor clutch state (`0x80` = engaged, `0x00` = off).

The CAN adapter decodes `0x1D0` and `0x1E3` into standardized Hiworld HVAC frames (`Cmd 0x31`) and accepts touchscreen overrides (`Cmd 0x3B` / `0x8A`) to inject corresponding control frames back to the vehicle.

```
+------------------------------------------------------------------------------------+
|                         Peugeot 407 HVAC Electronic Unit                           |
|       [Driver/Pass Temps, 8-Speed Fan, Dual/Mono, Auto, AQS, Air Distribution]     |
|         [Panel State (0x1D0), EMF Display (0x1E3), Compressor State (0x12D)]       |
+------------------------------------------------------------------------------------+
                         │ (Uplink 0x1D0 / 0x1E3)   ▲ (Downlink Overrides)
                         ▼                          │
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Parses CAN 0x1D0 (Modes: 0x08 Auto, 0x28 Manual, 0x19 Demist, 0xA8 Standby)  |
|   2. Converts Peugeot 23-step Temp Index (0..22) to 0.5 deg C units / LO / HI      |
|   3. Serializes Hiworld 0x31 HVAC telemetry for Android popup bar                  |
|   4. Decodes Android touch adjustments (Cmd 0x3B) -> CAN requests                  |
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

### 2.1 PSA Climate Panel State Frame (`0x1D0`)
- **CAN ID:** `0x1D0` (DLC: 8, Period: 500 ms)
- **Standby Frame (Ignition ON, Fan=0):** `A8 00 0F 00 00 <TempLeft> <TempRight> 00`
- **Active Byte Layout:**

```
+--------+--------+------------------------------------+-----------------------------+
| Byte   | Bit    | Function / Signal Name             | Value / Encoding Definition |
+--------+--------+------------------------------------+-----------------------------+
| Byte 0 | 7..0   | Climate Mode Flags                 | 0x08 = AUTO mode            |
|        |        |                                    | 0x28 = Manual mode          |
|        |        |                                    | 0x19 = Front Demist active  |
|        |        |                                    | 0xA8 = Standby (System Off) |
+--------+--------+------------------------------------+-----------------------------+
| Byte 1 | 7..0   | Constant Padding                   | 0x00                        |
+--------+--------+------------------------------------+-----------------------------+
| Byte 2 | 3..0   | Fan Speed Raw Nibble               | 0x0F = Off / Standby        |
|        |        |                                    | 0x00..0x07 = Fan Level 1..8 |
+--------+--------+------------------------------------+-----------------------------+
| Byte 3 | 7..4   | Left Zone Air Distribution Code    | See Air Distribution Table  |
|        | 3..0   | Right Zone Air Distribution Code   | See Air Distribution Table  |
+--------+--------+------------------------------------+-----------------------------+
| Byte 4 | Bit 5  | Explicit Intake Fresh Air          | 1 = Forced Fresh Air (0x20) |
|        | Bit 4  | Cabin Air Recirculation (Recycle)  | 1 = Cabin Recirc (0x10)     |
|        | Bit 0  | Rear Screen / Mirror Demist        | 1 = Rear Demist Active      |
+--------+--------+------------------------------------+-----------------------------+
| Byte 5 | 7..0   | Driver Zone Temperature Index      | Index 0..22 (See Temp Table)|
+--------+--------+------------------------------------+-----------------------------+
| Byte 6 | 7..0   | Passenger Zone Temperature Index   | Index 0..22 (See Temp Table)|
+--------+--------+------------------------------------+-----------------------------+
| Byte 7 | 7..0   | Reserved Padding                   | Always 0x00                 |
+--------+--------+------------------------------------+-----------------------------+
```

### 2.2 Temperature Index Table (`0x1D0` Bytes 5 & 6)
| Index | Display | Raw Android Celsius | Index | Display | Raw Android Celsius |
|:---:|:---:|:---:|:---:|:---:|:---:|
| `0` | **MIN / LO** | `0xFE` (-2 / LO) | `12` | **21.5 °C** | $43 = \text{0x2B}$ |
| `1` | **14.0 °C** | $28 = \text{0x1C}$ | `13` | **22.0 °C** | $44 = \text{0x2C}$ |
| `2` | **15.0 °C** | $30 = \text{0x1E}$ | `14` | **22.5 °C** | $45 = \text{0x2D}$ |
| `3` | **16.0 °C** | $32 = \text{0x20}$ | `15` | **23.0 °C** | $46 = \text{0x2E}$ |
| `4` | **17.0 °C** | $34 = \text{0x22}$ | `16` | **23.5 °C** | $47 = \text{0x2F}$ |
| `5` | **18.0 °C** | $36 = \text{0x24}$ | `17` | **24.0 °C** | $48 = \text{0x30}$ |
| `6` | **18.5 °C** | $37 = \text{0x25}$ | `18` | **25.0 °C** | $50 = \text{0x32}$ |
| `7` | **19.0 °C** | $38 = \text{0x26}$ | `19` | **26.0 °C** | $52 = \text{0x34}$ |
| `8` | **19.5 °C** | $39 = \text{0x27}$ | `20` | **27.0 °C** | $54 = \text{0x36}$ |
| `9` | **20.0 °C** | $40 = \text{0x28}$ | `21` | **28.0 °C** | $56 = \text{0x38}$ |
| `10` | **20.5 °C** | $41 = \text{0x29}$ | `22` | **MAX / HI** | `0xFF` (-1 / HI) |
| `11` | **21.0 °C** | $42 = \text{0x2A}$ | | | |

### 2.3 Air Distribution Codes (`0x1D0` Byte 3)
| Nibble Code | Vent Target | Hiworld Code | Android Distribution Flags |
|:---:|:---|:---:|:---|
| `0x00` / `0x01` | **AUTO** | `0` | Auto distribution |
| `0x02` | **Down** (Floor / Footwell) | `3` | Floor vent |
| `0x03` | **Up** (Windshield defrost) | `11` | Windshield defrost vent |
| `0x04` | **Face** (Dashboard center) | `6` | Face / Center vent |
| `0x05` | **Up + Down** (Defrost + Floor) | `12` | Up + Down |
| `0x06` | **Face + Down** (Face + Floor) | `5` | Face + Down |
| `0x07` | **All** (Face + Floor + Defrost) | `14` | Face + Floor + Up |

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Hiworld HVAC Telemetry Frame (`Cmd 0x31` / `Handle.CarAcState`)
- **Sync Header:** `0x5A 0xA5`
- **Length ($L$):** `0x0C` (12 payload bytes)
- **Command ID:** `0x31` (`49` decimal / `Handle.CarAcState`)
- **Payload Layout (12 Bytes):**
  - `Byte 0`:
    - `Bit 6`: Power ($1 = \text{On}, 0 = \text{Off}$)
    - `Bit 5`: AC Max ($1 = \text{Max AC}$)
    - `Bit 3`: Auto Mode ($1 = \text{Auto}$)
    - `Bit 2`: Sync / Dual ($1 = \text{Dual/Sync}$)
    - `Bit 1..0`: AC Compressor ($1 = \text{On}, 0 = \text{Off}$)
  - `Byte 1`:
    - `Bit 4`: Cabin Air Recirculation ($1 = \text{Recirc}, 0 = \text{Fresh}$)
    - `Bit 3`: AQS ($1 = \text{Auto Air Quality Sensor}$)
  - `Byte 2`:
    - `Bit 5`: Rear Window Defrost ($1 = \text{On}$)
    - `Bit 4`: Front Window Defrost ($1 = \text{On}$)
  - `Byte 3`:
    - `Bit 1..0`: Auto Blower Intensity ($0 \dots 3$)
  - `Byte 4`:
    - `Bit 7..4`: Passenger Air Distribution Mode
    - `Bit 3..0`: Driver Air Distribution Mode
  - `Byte 5`: Blower Fan Speed ($0 \dots 8$)
  - `Byte 6`: Left (Driver) Temperature ($\text{Raw} / 2.0 = ^\circ\text{C}$, `0xFE` = LO, `0xFF` = HI)
  - `Byte 7`: Right (Passenger) Temperature ($\text{Raw} / 2.0 = ^\circ\text{C}$, `0xFE` = LO, `0xFF` = HI)
  - `Byte 8..10`: Reserved (`0x00 0x00 0x00`)
  - `Byte 11`: Outside Ambient Temperature ($\text{Raw} \times 0.5 - 40.0^\circ\text{C}$)
- **Checksum:** `((Length + CmdID + sum(Payload)) - 1) & 0xFF`
- **Total Frame Wire Length:** 17 bytes (`5A A5 0C 31 [12 Bytes] Checksum`)

---

# 4. Pure C99 Firmware Implementation

```c
#ifndef CANBOX_HVAC_H
#define CANBOX_HVAC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define HIWORLD_SOF1            0x5A
#define HIWORLD_SOF2            0xA5
#define HIWORLD_CMD_CAR_AC_STATE 0x31

typedef struct {
    bool    power;
    bool    ac_compressor;
    bool    ac_max;
    bool    auto_mode;
    bool    dual_mode;
    bool    recirculation;
    bool    aqs_auto;
    bool    front_max_defrost;
    bool    rear_defrost;
    uint8_t blower_speed;
    uint8_t driver_wind_mode;
    uint8_t pass_wind_mode;
    uint8_t driver_temp_raw;
    uint8_t pass_temp_raw;
    uint8_t outdoor_temp_raw;
} psa_hvac_state_t;

typedef void (*canbox_uart_tx_fn)(const uint8_t *buf, size_t len);

typedef struct {
    psa_hvac_state_t  state;
    canbox_uart_tx_fn uart_tx;
} psa_hvac_ctx_t;

/* Initialize HVAC Context */
static inline void psa_hvac_init(psa_hvac_ctx_t *ctx, canbox_uart_tx_fn uart_tx) {
    memset(ctx, 0, sizeof(psa_hvac_ctx_t));
    ctx->uart_tx = uart_tx;
}

/* Serialize and transmit Hiworld HVAC packet (0x31) */
static inline void psa_hvac_send_hiworld(psa_hvac_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[17];
    p[0] = HIWORLD_SOF1;
    p[1] = HIWORLD_SOF2;
    p[2] = 0x0C;                     /* Length: 12 Payload bytes */
    p[3] = HIWORLD_CMD_CAR_AC_STATE; /* Cmd: 0x31 */

    /* Byte 0 */
    uint8_t b0 = 0;
    if (ctx->state.power)         b0 |= 0x40;
    if (ctx->state.ac_max)        b0 |= 0x20;
    if (ctx->state.auto_mode)     b0 |= 0x08;
    if (ctx->state.dual_mode)     b0 |= 0x04;
    if (ctx->state.ac_compressor) b0 |= 0x01;
    p[4] = b0;

    /* Byte 1 */
    uint8_t b1 = 0;
    if (ctx->state.recirculation) b1 |= 0x10;
    if (ctx->state.aqs_auto)      b1 |= 0x08;
    p[5] = b1;

    /* Byte 2 */
    uint8_t b2 = 0;
    if (ctx->state.rear_defrost)      b2 |= 0x20;
    if (ctx->state.front_max_defrost) b2 |= 0x10;
    p[6] = b2;

    /* Byte 3 */
    p[7] = 0x00;

    /* Byte 4 */
    p[8] = ((ctx->state.pass_wind_mode & 0x0F) << 4) | (ctx->state.driver_wind_mode & 0x0F);

    /* Byte 5: Fan Speed */
    p[9] = ctx->state.blower_speed;

    /* Bytes 6 & 7: Temperatures */
    p[10] = ctx->state.driver_temp_raw;
    p[11] = ctx->state.pass_temp_raw;

    /* Bytes 8..10: Reserved */
    p[12] = 0x00;
    p[13] = 0x00;
    p[14] = 0x00;

    /* Byte 11: Outdoor Temp */
    p[15] = ctx->state.outdoor_temp_raw;

    /* Checksum: Sum over p[2]..p[15] minus 1 */
    uint8_t sum = 0;
    for (size_t i = 2; i <= 15; i++) {
        sum += p[i];
    }
    p[16] = (uint8_t)((sum - 1) & 0xFF);

    ctx->uart_tx(p, 17);
}

#endif /* CANBOX_HVAC_H */
```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: Auto AC 21.0°C Dual Mode, Fan Speed 3
- **Expected UART Output (Hiworld `0x31`):**
  - Frame: `5A A5 0C 31 4D 00 00 00 00 03 2A 2A 00 00 00 78 D6`
  - Checksum Calculation: `((0x0C + 0x31 + 0x4D + 0x03 + 0x2A + 0x2A + 0x78) - 1) & 0xFF = 0xD6`
