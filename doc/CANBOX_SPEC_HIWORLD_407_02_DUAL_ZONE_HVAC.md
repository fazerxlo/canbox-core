# CAN Box Protocol Specification: Dual-Zone Climate Control (HVAC)
## Topic 02: Peugeot 407 Dual-Zone Climate & Touchscreen Overrides
**Document File:** `CANBOX_SPEC_HIWORLD_407_02_DUAL_ZONE_HVAC.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Ground Truth (Primary):** `RT4_CAN_2004.md` (reverse-engineered from Magneti Marelli RT4 firmware SW 8.31) + bench-verified `signal-db/clim.yaml`  
**Cross-Referenced:** `https://github.com/fazerxlo/canbox/tree/main/doc/CAN2004_clima.md`, `CAN_messages.md`

---

# 1. Functional Domain & Architecture Overview

The Peugeot 407 automatic climate control system drives three CAN frames on the PSA Comfort CAN bus:
- **`0x1E3` (Climate EMF / Display Status, 100 ms):** Broadcast by the Climate ECU (`MSG_ETAT_CLIMAV_EMF`). In accordance with the RT4 Demand-vs-Status rule (`Fp_Network_CAN.out` / `mmi_airc.out`), **`0x1E3` is the sole authoritative source** for all HVAC UI presentation (power, A/C compressor state, dual/mono mode, blower fan speed bars, zone temperature setpoints, airflow distribution, recirculation/AQS intake, and front max defrost).
- **`0x1D0` (Climate Panel Demand, 100 ms):** Sent by the physical fascia panel (`MSG_CDE_CLIMAV`) when a user adjusts controls, or injected by the CAN adapter on downlink (`Cmd 0x3B` / `0x7D`). Because panel demands can be refused or clamped by the Climate ECU (e.g. in Battery Economy mode or when refrigerant pressure is missing on a bench), `0x1D0` is **never** used to overwrite HVAC display state; only **Rear Window Defrost** (Byte 4 Bit 6, `0x40`) is extracted from `0x1D0` as it is not carried in `0x1E3`.
- **`0x12D` (Climate Compressor State, 100 ms):** Carries compressor clutch state (`0x80` = engaged, `0x00` = off).

The CAN adapter decodes `0x1E3` (plus `rear_defrost` from `0x1D0` and `outdoor_temp_raw` from `0x0F6`) into standardized Hiworld HVAC frames (`Cmd 0x31`) and accepts touchscreen overrides (`Cmd 0x3B` / `0x7D`) to inject corresponding `0x1D0` demand frames back to the vehicle.

---

# 2. PSA CAN Bus Bitfield Specification

### 2.1 PSA Climate Panel State Frame (`0x1D0`)
- **CAN ID:** `0x1D0` (DLC: 8 bytes, Period: 100 ms)
- **Direction:** Climate Panel (Façade Climatisation) → BSI / CAN adapter (uplink / status broadcast)
- **Note on DLC:** The RT4 re-emits `0x1D0` toward the Climate ECU as a 7-byte *demand* frame. The 8-byte format described here is the *panel broadcast* direction verified on the bench.
- **Standby Frame (Ignition ON, Fan=0):** `A8 00 0F 00 00 <TempLeft> <TempRight> 00`
- **Active Byte Layout:**

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

> **Note:** `0x1D0` uses this raw 0..22 index directly. `0x1E3` uses a **separate** display encoding: raw `0x00` = LO, `0x01..0x15` (1..21) = 14.5 °C .. 27.0 °C via formula `14.0 + raw × 0.5`, `0x16` = HI. See Section 2.4.

### 2.3 Air Distribution Codes (`0x1D0` Byte 3 & `0x1E3` Bytes 4–5)

> **Source:** RT4 firmware reverse-engineering (`RT4_CAN_2004.md` §4.3). The previous table was wrong (codes were shifted by 1). Both `0x1D0` (panel broadcast nibble) and `0x1E3` (EMF display upper nibble) use the same code set.

| Nibble Code | Vent Target | Hiworld Code | Android Distribution Flags |
|:---:|:---|:---:|:---|
| `0x00` | **AUTO** | `0` | Auto distribution |
| `0x01` | **Windshield / Screen** (defrost vent) | `11` | Windshield defrost vent |
| `0x02` | **Face** (Dashboard center vents) | `6` | Face / Center vent |
| `0x03` | **Feet / Floor** (Footwell) | `3` | Floor vent |
| `0x04` | **Bi-level** (Screen + Feet) | `12` | Windshield + Floor |
| `0x05` | **Face + Feet** (Vents + Footwell) | `5` | Face + Floor |

---

### 2.4 PSA Climate EMF / Display State Frame (`0x1E3`)
- **CAN ID:** `0x1E3` (DLC: 7 bytes, Period: 100 ms per RT4 firmware)
- **Direction:** Climate ECU → RT4 / Multifunction Display (EMF)
- **Description:** Authoritative dual-zone HVAC system status broadcast from the Climate ECU to the multifunction display / RT4. Carries system power, A/C compressor state, setpoints, air distribution, and blower fan bar levels.

**Byte Layout Table:**

| Byte | Bits | Mask | Signal / Field | Encoding & Description |
|:---:|:---:|:---:|:---|:---|
| **0** | 7 | `0x80` | `RECIRC_STATUS` | `1` = Recirculation active, `0` = Fresh air |
| **0** | 6 | `0x40` | `AC_OFF` | `1` = A/C compressor disabled (ECO mode), `0` = A/C active / authorized |
| **0** | 5 | `0x20` | `CLIM_OFF` | `1` = HVAC system powered OFF, `0` = HVAC active |
| **0** | 4 | `0x10` | `AUTO_AIRFLOW` | `1` = Automated airflow pattern active |
| **0** | 3 | `0x08` | `AUTO_BLOWER` | `1` = Automated fan blower regulation active |
| **0** | 2 | `0x04` | `HIDE_FAN_BARS` | `1` = Hide blower fan bar graph on display |
| **0** | 0 | `0x01` | `DUAL_ACTIVE` | `1` = Dual independent zones, `0` = Mono / SYNC |
| **1** | 7 | `0x80` | `UNFROST_FRONT` | `1` = Rapid front demist / defrost active |
| **1** | 6:4 | `0x70` | `TEMP_OFFSET` | Temperature calibration offset (`0x3` = $+3.0\ ^\circ\text{C}$ neutral factory default) |
| **2** | 7:0 | `0xFF` | `DISPLAY_TEMP_L` | Driver temperature setpoint: `0x00` = LO, `0x01..0x15` = 14.0–28.0 °C (`peugeot_temp_map`), `0x16` = HI |
| **3** | 7:0 | `0xFF` | `DISPLAY_TEMP_R` | Passenger temperature setpoint (mirrors Driver in Mono mode) |
| **4** | 7:4 | `0xF0` | `AIR_FLOW_L` | Driver air distribution pattern code (see Section 2.3) |
| **5** | 7:4 | `0xF0` | `AIR_FLOW_R` | Passenger air distribution pattern code (see Section 2.3) |
| **6** | 3:0 | `0x0F` | `DISPLAY_FAN_BARS`| Blower fan level bar graph ($0\dots 8$ segments, `0` = off) |

**Standby Frame (Ignition ON, Climate Off):** `20 00 00 00 00 00 00` (or `1C 40 0B 0B 00 00 0F 00` with AC off)

**Byte 0 Mode Bit Combinations (Bench-Verified):**

| Byte 0 value | Meaning |
|:---:|:---|
| `0x1C` | AUTO blower + Auto intake + Mono (bits 3+2+4) |
| `0x1D` | AUTO blower + Auto intake + Dual (bits 3+2+4+0) |
| `0x15` | Manual blower + Auto intake (AQS) + Dual (bits 4+2+0) |
| `0x05` | Manual blower + Fresh air + Dual (bits 2+0) |
| `0x85` | Manual blower + Cabin recirc + Dual (bits 7+2+0) |
| `0x87` | Recirc + intake notify pulse (one-shot popup) |
| `0x17` | Auto intake + intake notify pulse (one-shot popup) |
| `0x07` | Fresh air + intake notify pulse (one-shot popup) |

---

### 2.5 Climate Compressor State Frame (`0x12D`)
- **CAN ID:** `0x12D` (DLC: 8 bytes, Period: 100 ms per RT4 firmware)
- **Direction:** BSI / Engine ECU → Climate ECU / CAN adapter

| Byte | Signal | Encoding |
|---|---|---|
| 0 | `COMPRESSOR_STATUS` | `0x80` = clutch engaged; `0x00` = compressor off |
| 1 | `EVAPORATOR_TEMP` | raw − 40 °C; `0x32` = 10 °C (default bench value) |
| 2 | `BLOWER_VOLTAGE` | raw × 0.1 V; `0x32` = 5.0 V (default bench value) |
| 3–4 | `REFRIGERANT_PRESS` | raw × 0.1 bar |
| 5–6 | Reserved | `0x00 0x00` |
| 7 | `FLAGS` | `0x80` = climate loop authorized |

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

## Rear Defrost Detection (`vehicle_climate_t.rear_defrost`)

Rear defrost is decoded only from `0x1D0`: Byte 4 Bit 6 (`(data[4] >> 6) & 1`), dlc >= 5. `0x128` and `0x0F6` are intentionally not used.

A cleared bit clears the flag. It is serialized in Hiworld `0x31` payload[2] bit 5 (`0x20`). Implemented in `psa_hvac_process_can_0x1d0()`; test `test_peugeot_407_rear_defrost_detection`.

## Downlink Climate Commands (`0x1ED` / `0x0DF`)

The physical center-console HVAC panel has no physical buttons for A/C Compressor ON/OFF (Eco) or Mono/Dual sync. These functions are commanded via CAN ID `0x1ED` (`DEMANDES_AFFICHEUR_CLIM`, DLC 1) by the Multifunction Display (EMF/RT4) or OpenCanbox (Virtual MDF):
- **Prerequisite:** `0x0DF` (`10 00 50`, DLC 3) broadcast every 500 ms.
- **`0x1ED` Byte 0 Bitfield:**
  - **Bit 4 (`0x10`):** `1` = Dual Mode, `0` = Mono Mode.
  - **Bit 3 (`0x08`):** `0` = A/C ON, `1` = A/C OFF (Eco).
  - Valid payloads: `0x00` (A/C ON, Mono), `0x08` (A/C OFF, Mono), `0x10` (A/C ON, Dual), `0x18` (A/C OFF, Dual).
- **HU Mapping:**
  - Hiworld `Cmd 0x3B` Setting `0x02` (A/C): `1` = ON (clear bit 3), `0` = OFF (set bit 3).
  - Hiworld `Cmd 0x3B` Setting `0x0F` / `Cmd 0x7D` Setting `0x24` (Mono): `1` = Mono (clear bit 4), `0` = Dual (set bit 4).
  - Raise `Cmd 0x8A` Setting `0x02` (A/C toggle/set), Setting `0x0B` (Dual toggle/set).
