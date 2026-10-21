# CAN Box Protocol Specification: Tire Pressure Monitoring System (TPMS)
## Topic 03: Peugeot 407 Direct & Discrete TPMS Telemetry
**Document File:** `CANBOX_SPEC_HIWORLD_407_03_TPMS_TIRE_PRESSURE.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** `signal-db/tyres.yaml` (`Msg1E1`, `Msg361`, `Msg3A1`), `QF_Canbus.apk` (`PeugeotDataParser.java`), OEM EMF-C (MDF) Hardware Verification

---

# 1. Functional Domain & Architecture Overview

The Peugeot 407 features active TPMS (DSG — *Détection de Sous-Gonflage*) wheel transmitters in all four road wheels (and optionally the spare wheel). The TPMS ECU / BSI receives 433 MHz RF broadcasts from each wheel sensor and bridges tire pressure and diagnostic alarm states onto the PSA Comfort CAN bus via:
- **`0x1E1` (Wheel Status Enum / `MSG_DONNEES_ETAT_ROUES`, 250 ms):** Primary discrete wheel state frame (OK, under-inflated, puncture, sensor missing, battery low, or unmonitored `0x80`) for FL, FR, RR, RL, spare wheel, and overall system state. Verified directly on OEM Peugeot 407 Multifunction Display (MDF / EMF-C) and RT4.
- **`0x361` (Direct Measurements & Status, 500 ms, DLC 8):** 2 bytes per wheel (FL, FR, RR, RL) carrying a 2-bit diagnostic status and a 14-bit physical pressure scaled by 0.1 Bar/LSB. *(Note: On vehicles without direct pressure reporting enabled, `0x361` with `DLC: 6` is a BSI parameter frame and must be ignored by the TPMS decoder).*
- **`0x3A1` (Direct Pressures / `MSG_DONNEES_PRESSION_ROUES`, 250 ms):** 1 byte per wheel scaled by 0.05 Bar/LSB.
- **`0x168` Byte 1 (Combine Warning Overlay):** Bits 7..6 carry instrument cluster TPMS warning lamp illumination flags.

---

# 2. PSA CAN Bus Bitfield Specification

### 2.1 PSA Direct Tire Telemetry Frame (`0x361`)
- **CAN ID:** `0x361` (Standard 11-bit Identifier, **DLC: 8 required**, Period: 500 ms)
- **Slot Encoding (2 Bytes per Wheel, Big-Endian `uint16_t`):**
  - `Bytes 0–1`: Front Left (FL)
  - `Bytes 2–3`: Front Right (FR)
  - `Bytes 4–5`: Rear Right (RR)
  - `Bytes 6–7`: Rear Left (RL)
- **Bitfield per 16-bit Wheel Word:**
  - `Bits 15..14` (`(word >> 14) & 0x03`):
    - `0` (`00b`): Normal / OK
    - `1` (`01b`): Under-inflated (Low Pressure)
    - `2` (`10b`): Punctured (Flat)
    - `3` (`11b`): Sensor Fault / Offline
  - `Bits 13..0` (`word & 0x3FFF`):
    - Pressure in $0.1\text{ Bar/LSB}$ (e.g., `0x0018` = $24 = 2.4\text{ Bar}$; `0x3FFF` = Invalid).

### 2.2 PSA Wheel Status Enum Frame (`0x1E1` / `MSG_DONNEES_ETAT_ROUES`)
- **CAN ID:** `0x1E1` (Standard 11-bit Identifier, DLC: 8, Period: 250 ms)
- **Wheel Byte Indexing (Note PSA Rear Axle Order: RR before RL):**
  - `Byte 0`: Front Left (FL)
  - `Byte 1`: Front Right (FR)
  - `Byte 2`: Rear Right (RR)
  - `Byte 3`: Rear Left (RL)
  - `Byte 4`: Spare Wheel
  - `Byte 5`: `tpms_system_state` (`0x20` = nominal monitoring active, `0x80` = system unmonitored/standby)
  - `Bytes 6–7`: Reserved (`0x00 0x00`)
- **Per-Wheel Bitfield Layout (`Bytes 0..4`):**
  - **`Bit 7` (`0x80`):** Unmonitored / Standby / Sensor Offline Flag (`1` = wheel not monitored / standby on bench `0x80`).
  - **`Bits 5..3` (`(raw >> 3) & 0x07`):** 3-bit discrete wheel status enum (verified on OEM MDF & RT4):

| Enum `(raw >> 3) & 0x07` | Raw Byte (`enum << 3`) | Wheel Status Description | MDF / Cluster Behavior | Hiworld `Cmd 0x18` Code |
| :---: | :---: | :--- | :--- | :---: |
| **`0`** | `0x00` | **Normal / OK** | Normal tire graphic | `0x00` (Normal) |
| **`1`** | `0x08` | **Under-inflated (Low Pressure)** | Amber `SERVICE` + wheel icon | `0x01` (Low) |
| **`2`** | `0x10` | **Punctured / Flat** | Red `STOP` + flat tire icon | `0x02` (Puncture) |
| **`3`** | `0x18` | **Sensor Missing / Not Detected** | Missing sensor warning | `0x03` (Fault/Offline) |
| **`4`** | `0x20` | **Sensor Battery Low / HW Fault** | Service / battery warning | `0x03` (Fault/Offline) |
| *(Bit 7 = 1)* | `0x80` | **Not Monitored / Standby** | Unmonitored (`80 80 80 80 80 80`) | Suppressed at boot / `0x03` |

### 2.2.1 Two-Tier Hybrid TPMS Architecture
1. **Tier 1 (Basic Mode — Default on Peugeot 407 when only `0x1E1` is present):**
   - Discrete wheel statuses from `0x1E1` are mapped to Hiworld `Cmd 0x18` (`0`=Normal, `1`=Low, `2`=Puncture, `3`=Fault).
   - Because standard Peugeot 407 configurations do not broadcast numeric pressures unless enabled via Diagbox, `0x1E1` also synthesizes representative pressures in `pressure_bar_deci[]` dispatched via `Cmd 0x66` so the Android Head Unit TPMS UI displays sensible pressures alongside the tire alert graphics:
     - Normal (`enum = 0`): Front = `24` ($2.4\text{ Bar}$), Rear = `22` ($2.2\text{ Bar}$)
     - Low Pressure (`enum = 1`): `17` ($1.7\text{ Bar}$)
     - Puncture (`enum = 2`): `8` ($0.8\text{ Bar}$)
     - Missing / Fault (`enum >= 3` or `0x80`): `0` ($0.0\text{ Bar}$ / `--`)
2. **Tier 2 (Direct Numeric Mode — `0x361` DLC 8 or `0x3A1` Active):**
   - Upon receiving a valid direct numeric frame (`0x361` with `DLC >= 8` or `0x3A1` with `DLC >= 4`), the decoder latches `s_has_direct_pressures = true`.
   - Once latched, `0x1E1` updates `alarm_state[]` only and **stops overwriting** `pressure_bar_deci[]`, preserving exact physical sensor measurements from `0x361` / `0x3A1`.
3. **Standby / Unequipped Guard (`1E1#8080808080800000`):**
   - On vehicles without TPMS or on bench/startup before sensor wake-up, the BSI broadcasts `80 80 80 80 80 80 00 00` (Bit 7 `0x80` set on all wheels and `Byte 5 = 0x80`).
   - If `tpms->valid` is `false` and all four road wheels have Bit 7 set (`(data[x] & 0x80) != 0`), the frame is ignored so no false `0.0 Bar` or fault frames are sent to the Head Unit.

### 2.3 PSA Direct Pressures in 0.05 Bar (`0x3A1`)
- **CAN ID:** `0x3A1` (DLC: 8, Period: 250 ms)
- `Byte 0 (FL)`: $\text{Round}(\text{Pressure}_{\text{bar}} / 0.05) \implies \text{deci\_bar} = (\text{raw} \times 5 + 5) / 10$
- `Byte 1 (FR)`: $\text{Round}(\text{Pressure}_{\text{bar}} / 0.05)$
- `Byte 2 (RR)`: $\text{Round}(\text{Pressure}_{\text{bar}} / 0.05)$
- `Byte 3 (RL)`: $\text{Round}(\text{Pressure}_{\text{bar}} / 0.05)$

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Hiworld Discrete TPMS Alarm (`Cmd 0x18`)
- **Sync Header:** `0x5A 0xA5`
- **Length ($L$):** `0x04` (4 payload bytes)
- **Command ID:** `0x18`
- **Payload Layout (4 Bytes — Note Hiworld Rear Axle Order: RL before RR):**
  - `Byte 0`: Front Left Alarm (`0x00` = Normal, `0x01` = Low Pressure, `0x02` = Puncture, `0x03` = Offline/Fault)
  - `Byte 1`: Front Right Alarm
  - `Byte 2`: **Rear Left** Alarm
  - `Byte 3`: **Rear Right** Alarm
- **Checksum:** `((Length + CmdID + sum(Payload)) - 1) & 0xFF`

### 3.2 Hiworld Universal Direct Numeric Pressures (`Cmd 0x66`)
- **Sync Header:** `0x5A 0xA5`
- **Length ($L$):** `0x06` (6 payload bytes)
- **Command ID:** `0x66`
- **Payload Layout (6 Bytes — Note Hiworld Rear Axle Order: RL before RR):**
  - `Byte 0`: Mode Flag (`0x01` = Live numeric reading)
  - `Byte 1`: FL Pressure in $0.1\text{ Bar}$ (e.g. `24` = $2.4\text{ Bar}$)
  - `Byte 2`: FR Pressure in $0.1\text{ Bar}$
  - `Byte 3`: **RL** Pressure in $0.1\text{ Bar}$
  - `Byte 4`: **RR** Pressure in $0.1\text{ Bar}$
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
    uint8_t pressure_bar_deci[4]; /* [0]=FL, [1]=FR, [2]=RL, [3]=RR (0.1 Bar: 24 = 2.4 Bar) */
    uint8_t alarm_state[4];       /* [0]=FL, [1]=FR, [2]=RL, [3]=RR (0=OK, 1=LOW, 2=PUNCTURE, 3=FAULT) */
    bool    valid;
    bool    has_direct_pressures; /* Latched true once 0x361 (DLC>=8) or 0x3A1 is received */
} psa_tpms_state_t;

typedef void (*canbox_uart_tx_fn)(const uint8_t *buf, size_t len);

typedef struct {
    psa_tpms_state_t  state;
    canbox_uart_tx_fn uart_tx;
} psa_tpms_ctx_t;

static inline void psa_tpms_init(psa_tpms_ctx_t *ctx, canbox_uart_tx_fn uart_tx) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(psa_tpms_ctx_t));
    ctx->state.pressure_bar_deci[0] = 24; /* FL: 2.4 Bar default */
    ctx->state.pressure_bar_deci[1] = 24; /* FR: 2.4 Bar default */
    ctx->state.pressure_bar_deci[2] = 22; /* RL: 2.2 Bar default */
    ctx->state.pressure_bar_deci[3] = 22; /* RR: 2.2 Bar default */
    ctx->uart_tx = uart_tx;
}

/* Decode single wheel byte from PSA 0x1E1:
 * Bit 7 (0x80) = Unmonitored / Sensor Offline
 * Bits 5..3 ((raw >> 3) & 0x07): 0=OK, 1=LOW, 2=PUNCTURE, 3=MISSING, 4=BATTERY_LOW */
static inline uint8_t psa_tpms_decode_0x1e1_wheel(uint8_t raw_byte, uint8_t nominal_deci_bar, uint8_t *out_synth_bar) {
    if ((raw_byte & 0x80) != 0) {
        if (out_synth_bar) *out_synth_bar = 0;
        return 3; /* Unmonitored / Offline */
    }

    uint8_t state_enum = (uint8_t)((raw_byte >> 3) & 0x07);
    switch (state_enum) {
        case 0: /* 0x00: Normal / OK */
            if (out_synth_bar) *out_synth_bar = nominal_deci_bar;
            return 0;
        case 1: /* 0x08: Under-inflated / Low Pressure */
            if (out_synth_bar) *out_synth_bar = 17; /* 1.7 Bar synthetic low */
            return 1;
        case 2: /* 0x10: Punctured / Flat */
            if (out_synth_bar) *out_synth_bar = 8;  /* 0.8 Bar synthetic puncture */
            return 2;
        case 3: /* 0x18: Sensor Missing */
        case 4: /* 0x20: Sensor Battery Low / Fault */
        default:
            if (out_synth_bar) *out_synth_bar = 0;  /* 0.0 Bar / Offline */
            return 3;
    }
}

/* Transmit Hiworld Numeric TPMS Frame (Cmd 0x66) */
static inline void psa_tpms_send_numeric(psa_tpms_ctx_t *ctx) {
    if (!ctx || !ctx->uart_tx || !ctx->state.valid) return;

    uint8_t p[11];
    p[0] = HIWORLD_SOF1;
    p[1] = HIWORLD_SOF2;
    p[2] = 0x06;                     /* Length: 6 Payload bytes */
    p[3] = HIWORLD_CMD_TPMS_NUMERIC; /* Cmd 0x66 */
    p[4] = 0x01;                     /* Mode: Live */
    p[5] = ctx->state.pressure_bar_deci[0]; /* FL */
    p[6] = ctx->state.pressure_bar_deci[1]; /* FR */
    p[7] = ctx->state.pressure_bar_deci[2]; /* RL */
    p[8] = ctx->state.pressure_bar_deci[3]; /* RR */
    p[9] = 0x00;                     /* Unit: Bar */

    uint8_t sum = 0;
    for (size_t i = 2; i <= 9; i++) {
        sum = (uint8_t)(sum + p[i]);
    }
    p[10] = (uint8_t)((sum - 1) & 0xFF);

    ctx->uart_tx(p, 11);
}

/* Transmit Hiworld Discrete TPMS Alarm Frame (Cmd 0x18) */
static inline void psa_tpms_send_discrete(psa_tpms_ctx_t *ctx) {
    if (!ctx || !ctx->uart_tx || !ctx->state.valid) return;

    uint8_t p[9];
    p[0] = HIWORLD_SOF1;
    p[1] = HIWORLD_SOF2;
    p[2] = 0x04;                      /* Length: 4 Payload bytes */
    p[3] = HIWORLD_CMD_TPMS_DISCRETE; /* Cmd 0x18 */
    p[4] = ctx->state.alarm_state[0]; /* FL */
    p[5] = ctx->state.alarm_state[1]; /* FR */
    p[6] = ctx->state.alarm_state[2]; /* RL */
    p[7] = ctx->state.alarm_state[3]; /* RR */

    uint8_t sum = 0;
    for (size_t i = 2; i <= 7; i++) {
        sum = (uint8_t)(sum + p[i]);
    }
    p[8] = (uint8_t)((sum - 1) & 0xFF);

    ctx->uart_tx(p, 9);
}

/* Process PSA CAN 0x361 Frame (Direct Pressures + 2-bit Status; DLC >= 8 required) */
static inline void psa_tpms_process_can_0x361(psa_tpms_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (!ctx || !data || dlc < 8) return;

    uint16_t fl_raw = ((uint16_t)data[0] << 8) | data[1];
    uint16_t fr_raw = ((uint16_t)data[2] << 8) | data[3];
    uint16_t rr_raw = ((uint16_t)data[4] << 8) | data[5];
    uint16_t rl_raw = ((uint16_t)data[6] << 8) | data[7];

    ctx->state.alarm_state[0] = (uint8_t)((fl_raw >> 14) & 0x03); /* FL */
    ctx->state.alarm_state[1] = (uint8_t)((fr_raw >> 14) & 0x03); /* FR */
    ctx->state.alarm_state[2] = (uint8_t)((rl_raw >> 14) & 0x03); /* RL */
    ctx->state.alarm_state[3] = (uint8_t)((rr_raw >> 14) & 0x03); /* RR */

    ctx->state.pressure_bar_deci[0] = (ctx->state.alarm_state[0] != 3 && (fl_raw & 0x3FFF) != 0x3FFF) ? (uint8_t)(fl_raw & 0x3FFF) : 0;
    ctx->state.pressure_bar_deci[1] = (ctx->state.alarm_state[1] != 3 && (fr_raw & 0x3FFF) != 0x3FFF) ? (uint8_t)(fr_raw & 0x3FFF) : 0;
    ctx->state.pressure_bar_deci[2] = (ctx->state.alarm_state[2] != 3 && (rl_raw & 0x3FFF) != 0x3FFF) ? (uint8_t)(rl_raw & 0x3FFF) : 0;
    ctx->state.pressure_bar_deci[3] = (ctx->state.alarm_state[3] != 3 && (rr_raw & 0x3FFF) != 0x3FFF) ? (uint8_t)(rr_raw & 0x3FFF) : 0;

    ctx->state.has_direct_pressures = true;
    ctx->state.valid = true;

    psa_tpms_send_numeric(ctx);
    psa_tpms_send_discrete(ctx);
}

/* Process PSA CAN 0x3A1 Frame (0.05 Bar LSB) */
static inline void psa_tpms_process_can_0x3a1(psa_tpms_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (!ctx || !data || dlc < 4) return;

    ctx->state.pressure_bar_deci[0] = (uint8_t)((data[0] * 5 + 5) / 10); /* FL */
    ctx->state.pressure_bar_deci[1] = (uint8_t)((data[1] * 5 + 5) / 10); /* FR */
    ctx->state.pressure_bar_deci[2] = (uint8_t)((data[3] * 5 + 5) / 10); /* RL (Byte 3) */
    ctx->state.pressure_bar_deci[3] = (uint8_t)((data[2] * 5 + 5) / 10); /* RR (Byte 2) */

    ctx->state.has_direct_pressures = true;
    ctx->state.valid = true;

    psa_tpms_send_numeric(ctx);
}

/* Process PSA CAN 0x1E1 Frame (Wheel Status Enum + Tier 1 Synthetic Pressure Fallback) */
static inline void psa_tpms_process_can_0x1e1(psa_tpms_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (!ctx || !data || dlc < 4) return;

    /* Guard against standby / unequipped 80 80 80 80 frame before TPMS wakes up */
    if (!ctx->state.valid &&
        (data[0] & 0x80) != 0 && (data[1] & 0x80) != 0 &&
        (data[2] & 0x80) != 0 && (data[3] & 0x80) != 0) {
        return;
    }

    uint8_t synth_fl, synth_fr, synth_rl, synth_rr;

    /* PSA 0x1E1 Byte order: Byte 0=FL, Byte 1=FR, Byte 2=RR, Byte 3=RL
     * Hiworld array order:  [0]=FL,    [1]=FR,    [2]=RL,    [3]=RR */
    ctx->state.alarm_state[0] = psa_tpms_decode_0x1e1_wheel(data[0], 24, &synth_fl); /* FL */
    ctx->state.alarm_state[1] = psa_tpms_decode_0x1e1_wheel(data[1], 24, &synth_fr); /* FR */
    ctx->state.alarm_state[2] = psa_tpms_decode_0x1e1_wheel(data[3], 22, &synth_rl); /* RL */
    ctx->state.alarm_state[3] = psa_tpms_decode_0x1e1_wheel(data[2], 22, &synth_rr); /* RR */

    if (!ctx->state.has_direct_pressures) {
        ctx->state.pressure_bar_deci[0] = synth_fl;
        ctx->state.pressure_bar_deci[1] = synth_fr;
        ctx->state.pressure_bar_deci[2] = synth_rl;
        ctx->state.pressure_bar_deci[3] = synth_rr;
        ctx->state.valid = true;
        psa_tpms_send_numeric(ctx);
    }

    ctx->state.valid = true;
    psa_tpms_send_discrete(ctx);
}

#endif /* CANBOX_TPMS_H */

```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: Nominal Pressures on CAN 0x361 (FL=2.4, FR=2.4, RR=2.2, RL=2.2 Bar)

* **CAN ID `0x361` Injection:**
```bash
cansend vcan0 361#0018001800160016

```


* **Expected UART Output:**
* **Numeric (`Cmd 0x66`):** `5A A5 06 66 01 18 18 16 16 00 C8`
* **Discrete (`Cmd 0x18`):** `5A A5 04 18 00 00 00 00 1B`



### Vector 2: Asymmetric Pressures with Rear-Left Low Warning (FL=2.4, FR=2.3, RL=1.2, RR=2.1 Bar)

* **CAN ID `0x361` Injection:**
* FL: 2.4 Bar (State 0 OK) -> `0x0018`
* FR: 2.3 Bar (State 0 OK) -> `0x0017`
* RR: 2.1 Bar (State 0 OK) -> `0x0015`
* RL: 1.2 Bar (State 1 UNDER_INFLATED = `0x4000`) -> `0x400C`


```bash
cansend vcan0 361#001800170015400C

```


* **Expected UART Output:**
* **Numeric (`Cmd 0x66`):** `5A A5 06 66 01 18 17 0C 15 00 BC`
* **Discrete (`Cmd 0x18`):** `5A A5 04 18 00 00 01 00 1C` (RL alarm = `0x01` LOW)



### Vector 3: Front-Right Puncture / Flat Tyre (FL=2.4, FR=0.4, RL=2.4, RR=2.4 Bar)

* **CAN ID `0x361` Injection:**
* FL: 2.4 Bar (State 0 OK) -> `0x0018`
* FR: 0.4 Bar (State 2 PUNCTURE = `0x8000`) -> `0x8004`
* RR: 2.4 Bar (State 0 OK) -> `0x0018`
* RL: 2.4 Bar (State 0 OK) -> `0x0018`


```bash
cansend vcan0 361#0018800400180018

```


* **Expected UART Output:**
* **Numeric (`Cmd 0x66`):** `5A A5 06 66 01 18 04 18 18 00 B8`
* **Discrete (`Cmd 0x18`):** `5A A5 04 18 00 02 00 00 1D` (FR alarm = `0x02` PUNCTURE)



### Vector 4: Basic Mode (`0x1E1` Only — FL Under-inflated, No `0x361`/`0x3A1`)

* **CAN ID `0x1E1` Injection (`fl=1` -> `1 << 3 = 0x08`):**
```bash
cansend vcan0 1E1#0800000000200000

```


* **Expected UART Output:**
* **Numeric (`Cmd 0x66`, Synthetic):** `5A A5 06 66 01 11 18 16 16 00 C1` (`FL=1.7, FR=2.4, RL=2.2, RR=2.2 Bar`)
* **Discrete (`Cmd 0x18`):** `5A A5 04 18 01 00 00 00 1C` (`FL alarm = 0x01 LOW`)





