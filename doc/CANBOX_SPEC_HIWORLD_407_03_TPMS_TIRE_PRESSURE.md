# CAN Box Protocol Specification: Tire Pressure Monitoring System (TPMS)
## Topic 03: Peugeot 407 Direct & Discrete TPMS Telemetry
**Document File:** `CANBOX_SPEC_HIWORLD_407_03_TPMS_TIRE_PRESSURE.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** `signal-db/tyres.yaml` (`Msg1E1`, `Msg3A1`), `QF_Canbus.apk` (`PeugeotDataParser.java`), OEM EMF-C (MDF) Hardware Verification

---

# 1. Functional Domain & Architecture Overview

The Peugeot 407 features active TPMS (DSG — *Détection de Sous-Gonflage*) wheel transmitters in all four road wheels (and optionally the spare wheel). The TPMS ECU / BSI receives 433 MHz RF broadcasts from each wheel sensor and bridges tire pressure and diagnostic alarm states onto the PSA Comfort CAN bus via:
- **`0x1E1` (Wheel Status Enum / `MSG_DONNEES_ETAT_ROUES`, 250 ms):** Primary discrete wheel state frame (OK, under-inflated, puncture, sensor missing, battery low, or unmonitored `0x80`) for FL, FR, RR, RL, spare wheel, and overall system state. Verified directly on OEM Peugeot 407 Multifunction Display (MDF / EMF-C) and RT4.
- **`0x3A1` (Direct Pressures / `MSG_DONNEES_PRESSION_ROUES`, 250 ms):** 1 byte per wheel scaled by 0.05 Bar/LSB ($2.4\text{ Bar} \implies 48$).
- **`0x168` Byte 1 (Combine Warning Overlay):** Bits 7..6 carry instrument cluster TPMS warning lamp illumination flags.

> [!NOTE]
> **CAN ID `0x361` Exclusion**:
> Real vehicle bus captures and firmware reverse engineering confirm that CAN ID `0x361` (DLC 6) is **exclusively** the BSI Personalization Menu (`MSG_BSI_INF_CFG_MENU_PERSO`) and is **never** used for TPMS telemetry. Direct numeric tire pressures in PSA CAN 2004 are broadcasted exclusively on CAN ID **`0x3A1`**.

---

# 2. PSA CAN Bus Bitfield Specification

### 2.1 PSA Wheel Status Enum Frame (`0x1E1` / `MSG_DONNEES_ETAT_ROUES`)
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
  - **`Bits 7:3` (`(raw >> 3) & 0x1F`):** 5-bit discrete wheel status enum (verified on OEM RT4 disassembly `get_wheel_status__9C_BCM_CANP17type_wheel_status` & EMF-C):
  - Note: In 32-bit PowerPC disassembly jump tables, enum values are scaled by $4$ (`slwi r, 2`), producing table offsets `0x00, 0x04, 0x08, 0x0C, 0x10`. The RT4 default/invalid value is `0x8` (`0x8 << 3 = 0x40`), representing unmonitored wheels.

| Enum `(raw >> 3) & 0x1F` | Raw Byte (`enum << 3`) | PPC Jump Table Offset | Wheel Status Description | MDF / Cluster Behavior | Hiworld `Cmd 0x18` Code |
| :---: | :---: | :---: | :--- | :--- | :---: |
| **`0`** | `0x00` | `0x00` | **Normal / OK** | Normal tire graphic | `0x00` (Normal) |
| **`1`** | `0x08` | `0x04` | **Under-inflated (Low Pressure)** | Amber `SERVICE` + wheel icon | `0x01` (Low) |
| **`2`** | `0x10` | `0x08` | **Punctured / Flat** | Red `STOP` + flat tire icon | `0x02` (Puncture) |
| **`3`** | `0x18` | `0x0C` | **Sensor Missing / Not Detected** | Missing sensor warning | `0x03` (Fault/Offline) |
| **`4`** | `0x20` | `0x10` | **Sensor Battery Low / HW Fault** | Service / battery warning | `0x03` (Fault/Offline) |
| **`8`** | `0x40` | `0x20` / default | **Default / Invalid (Unmonitored)** | Unmonitored (bench / offline sensor) | `0x03` (Fault/Offline) |
| **`16`** | `0x80` | `0x40` / default | **Unequipped / Standby (Spare)** | Spare offline / unmonitored | `0x03` (Fault/Offline) |

### 2.2 Two-Tier Hybrid TPMS Architecture
1. **Tier 1 (Basic Mode — Default on Peugeot 407 when only `0x1E1` is present):**
   - Discrete wheel statuses from `0x1E1` are mapped to Hiworld `Cmd 0x18` (`0`=Normal, `1`=Low, `2`=Puncture, `3`=Fault).
   - Because standard Peugeot 407 configurations do not broadcast numeric pressures unless enabled via Diagbox, `0x1E1` also synthesizes representative pressures in `pressure_bar_deci[]` dispatched via `Cmd 0x66` so the Android Head Unit TPMS UI displays sensible pressures alongside the tire alert graphics:
     - Normal (`enum = 0`): Front = `24` ($2.4\text{ Bar}$), Rear = `22` ($2.2\text{ Bar}$)
     - Low Pressure (`enum = 1`): `15` ($1.5\text{ Bar}$)
     - Puncture / Flat (`enum = 2`): `0` ($0.0\text{ Bar}$)
     - Missing / Fault (`enum >= 3` or `0x80`): `0xFF` (displays `--.-` on Head Unit)
2. **Tier 2 (Direct Numeric Mode — `0x3A1` Active):**
   - Upon receiving a valid direct numeric frame (`0x3A1` with `DLC >= 4`), direct mode is active with a 3000 ms timeout window (`s_last_3a1_time_ms`).
   - While `0x3A1` is active, `0x1E1` updates `alarm_state[]` for `Cmd 0x18` only and **stops overwriting** `pressure_bar_deci[]`, preserving exact physical sensor measurements from `0x3A1`. If `0x3A1` ceases for > 3000 ms, `0x1E1` resumes synthesizing pressures for `Cmd 0x66`.
3. **Standby / Unequipped Guard (`1E1#8080808080800000`):**
   - On vehicles without TPMS or on bench/startup before sensor wake-up, the BSI broadcasts `80 80 80 80 80 80 00 00` (Bit 7 `0x80` set on all wheels and `Byte 5 = 0x80`).
   - If `tpms->valid` is `false` and all four road wheels have Bit 7 set (`(data[x] & 0x80) != 0`), the frame is ignored so no false `0.0 Bar` or fault frames are sent to the Head Unit.

### 2.3 PSA Direct Pressures in 0.05 Bar (`0x3A1`)

#### `0x3A1` — Direct Wheel Pressure Values (`MSG_DONNEES_PRESSION_ROUES`)
- **Period:** 250 ms  
- **Direction:** TPMS ECU -> RT4 / Display  
- **Length:** 8 bytes (minimum DLC: 4 accepted)  
- **Producer / Handler Function:** `get_wheel_pressure__9C_BCM_CANP19type_wheel_pressure` (offset `0x01261C`) & Case 17 (`0x17748`)

| Byte | Bits | Signal Name | Scaling / Formula | Invalid / Default | Description & Values |
|---|---|---|---|---|---|
| 0 | 7:0 | PRESSURE_FL | raw × 0.05 bar (or raw × 10 kPa) | 0xFF | Front Left tire pressure (e.g., `48` = 2.40 bar) |
| 1 | 7:0 | PRESSURE_FR | raw × 0.05 bar | 0xFF | Front Right tire pressure |
| 2 | 7:0 | PRESSURE_RR | raw × 0.05 bar | 0xFF | Rear Right tire pressure |
| 3 | 7:0 | PRESSURE_RL | raw × 0.05 bar | 0xFF | Rear Left tire pressure |
| 4-7 | 31:0 | RESERVED | raw | 0x00000000 | Reserved |

**Conversion to Decibar (0.1 Bar/LSB):**
- $\text{deci\_bar} = \begin{cases} 0\text{xFF} & \text{if } \text{raw} = 0\text{xFF (Invalid/Sensor Offline } \implies \text{displays --.-)} \\ (\text{raw} \times 5 + 5) / 10 & \text{otherwise (integer rounding)} \end{cases}$

> [!NOTE]
> Note the axle layout mapping: on PSA CAN 2004 bus, Byte 2 is **Rear Right (RR)** and Byte 3 is **Rear Left (RL)**. Hiworld protocol expects `RL` at byte index 3 and `RR` at byte index 4. The decoder explicitly swaps the rear axle positions into their correct logical slots.

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
- **Payload Layout:**
  - `Byte 0`: Pressure Reporting Mode / Status (`0x01` = Active numeric monitoring)
  - `Byte 1`: Front Left Pressure in $0.1\text{ Bar/LSB}$ (e.g., $24 = 2.4\text{ Bar}$)
  - `Byte 2`: Front Right Pressure in $0.1\text{ Bar/LSB}$
  - `Byte 3`: **Rear Left Pressure** in $0.1\text{ Bar/LSB}$
  - `Byte 4`: **Rear Right Pressure** in $0.1\text{ Bar/LSB}$
  - `Byte 5`: Pressure Unit (`0x00` = Bar, `0x01` = PSI, `0x02` = kPa)
- **Checksum:** `((Length + CmdID + sum(Payload)) - 1) & 0xFF`

---

# 4. Pure C99 Reference Implementation

```c
#ifndef CANBOX_TPMS_H
#define CANBOX_TPMS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define HIWORLD_SOF1                0x5A
#define HIWORLD_SOF2                0xA5
#define HIWORLD_CMD_TPMS_DISCRETE   0x18
#define HIWORLD_CMD_TPMS_NUMERIC    0x66

typedef struct {
    uint8_t alarm_state[4];        /* 0:FL, 1:FR, 2:RL, 3:RR */
    uint8_t pressure_bar_deci[4];  /* 0:FL, 1:FR, 2:RL, 3:RR (0.1 Bar) */
    bool    has_direct_pressures;
    bool    valid;
} psa_tpms_state_t;

typedef void (*tpms_uart_tx_fn)(const uint8_t *buf, size_t len);

typedef struct {
    psa_tpms_state_t state;
    tpms_uart_tx_fn  uart_tx;
} psa_tpms_ctx_t;

static inline void psa_tpms_init(psa_tpms_ctx_t *ctx, tpms_uart_tx_fn tx_fn) {
    if (!ctx) return;
    ctx->uart_tx = tx_fn;
    ctx->state.valid = false;
    ctx->state.has_direct_pressures = false;
    for (int i = 0; i < 4; i++) {
        ctx->state.alarm_state[i] = 0;
        ctx->state.pressure_bar_deci[i] = (i < 2) ? 24 : 22;
    }
}

/* Transmit Hiworld Numeric Pressure Frame (Cmd 0x66) */
static inline void psa_tpms_send_numeric(psa_tpms_ctx_t *ctx) {
    if (!ctx || !ctx->uart_tx || !ctx->state.valid) return;

    uint8_t p[11];
    p[0] = HIWORLD_SOF1;
    p[1] = HIWORLD_SOF2;
    p[2] = 0x06;                     /* Length: 6 Payload bytes */
    p[3] = HIWORLD_CMD_TPMS_NUMERIC; /* Cmd 0x66 */
    p[4] = 0x01;                     /* Valid mode */
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

/* Process PSA CAN 0x3A1 Frame (0.05 Bar LSB, 0xFF = Invalid/Offline) */
static inline void psa_tpms_process_can_0x3a1(psa_tpms_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (!ctx || !data || dlc < 4) return;

    ctx->state.pressure_bar_deci[0] = (data[0] == 0xFF) ? 0 : (uint8_t)((data[0] * 5 + 5) / 10); /* FL */
    ctx->state.pressure_bar_deci[1] = (data[1] == 0xFF) ? 0 : (uint8_t)((data[1] * 5 + 5) / 10); /* FR */
    ctx->state.pressure_bar_deci[2] = (data[3] == 0xFF) ? 0 : (uint8_t)((data[3] * 5 + 5) / 10); /* RL (Byte 3) */
    ctx->state.pressure_bar_deci[3] = (data[2] == 0xFF) ? 0 : (uint8_t)((data[2] * 5 + 5) / 10); /* RR (Byte 2) */

    ctx->state.has_direct_pressures = true;
    ctx->state.valid = true;

    psa_tpms_send_numeric(ctx);
}

/* Decode single wheel from 0x1E1 byte */
static inline uint8_t psa_tpms_decode_0x1e1_wheel(uint8_t raw, uint8_t nominal_deci, uint8_t *out_synth_bar) {
    uint8_t code = (uint8_t)((raw >> 3) & 0x1F);
    switch (code) {
        case 0:
            if (out_synth_bar) *out_synth_bar = nominal_deci;
            return 0; /* OK */
        case 1:
            if (out_synth_bar) *out_synth_bar = 15;
            return 1; /* Low */
        case 2:
            if (out_synth_bar) *out_synth_bar = 0;
            return 2; /* Puncture / Flat */
        case 3:
        case 4:
        case 8:  /* RT4 Default / Invalid: Unmonitored 0x40 */
        case 16: /* Unequipped 0x80 */
        default:
            if (out_synth_bar) *out_synth_bar = 0;
            return 3; /* Fault / Offline */
    }
}

/* Process PSA CAN 0x1E1 Frame (Wheel Status Enum + Tier 1 Synthetic Pressure Fallback) */
static inline void psa_tpms_process_can_0x1e1(psa_tpms_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (!ctx || !data || dlc < 4) return;

    /* Guard against standby / unequipped / unmonitored frames before TPMS wakes up */
    if (!ctx->state.valid &&
        ((data[0] & 0xC0) != 0) && ((data[1] & 0xC0) != 0) &&
        ((data[2] & 0xC0) != 0) && ((data[3] & 0xC0) != 0)) {
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

### Vector 1: Nominal Direct Pressures on CAN 0x3A1 (FL=2.4, FR=2.4, RR=2.2, RL=2.2 Bar)

* **CAN ID `0x3A1` Injection (`raw = bar / 0.05`):**
  - FL: 2.4 Bar $\implies 48 = 0x30$
  - FR: 2.4 Bar $\implies 48 = 0x30$
  - RR: 2.2 Bar $\implies 44 = 0x2C$
  - RL: 2.2 Bar $\implies 44 = 0x2C$
```bash
cansend vcan0 3A1#30302C2C
```
* **Expected UART Output:**
  - **Numeric (`Cmd 0x66`):** `5A A5 06 66 01 18 18 16 16 00 C8`
  - **Discrete (`Cmd 0x18`):** `5A A5 04 18 00 00 00 00 1B`

---

### Vector 2: Basic Mode (`0x1E1` Only — FL Under-inflated, FR/RR/RL Nominal)

* **CAN ID `0x1E1` Injection (`FL=1` low $\to 1 \ll 3 = 0x08$):**
```bash
cansend vcan0 1E1#0800000000200000
```
* **Expected UART Output:**
  - **Numeric (`Cmd 0x66`, Synthetic):** `5A A5 06 66 01 0F 18 16 16 00 BF` (`FL=1.5, FR=2.4, RL=2.2, RR=2.2 Bar`)
  - **Discrete (`Cmd 0x18`):** `5A A5 04 18 01 00 00 00 1C` (`FL alarm = 0x01 LOW`)
