# CAN Box Protocol Specification: Steering Wheel Angle Sensor (SAS)
## Topic 05: Peugeot 407 Steering Angle & Dynamic Trajectory
**Document File:** `CANBOX_SPEC_HIWORLD_407_05_STEERING_ANGLE_SAS.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 / 307 FL / Citroën C5 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld PA12.20 / `H1H2PA123A-240717` (`0x5A 0xA5` sync header, `(sum - 1) & 0xFF` checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** Hardware-in-the-loop verification on Hiworld PA12.20 (`0x10B`), `QF_Canbus.apk` (`PeugeotDataParser.java`)

---

# 1. Functional Domain & Architecture Overview

> [!NOTE]
> **Status Notice (October 2026) — Confirmed via Hardware-in-the-Loop (Hiworld PA12.20):**  
> In PSA CAN2004 vehicles (Peugeot 407, 307 Restyling, Citroën C5), the Steering Column Module (`COM2003` / `COM2005`) broadcasts raw steering angle data on the high-speed **CAN IS (500 kbps)** bus under CAN ID **`0x305`**. The **BSI** acts as a gateway and routes this telemetry onto the **CAN Comfort (125 kbps)** bus under CAN ID **`0x10B`**. Earlier assumptions regarding `0x0C5` and `0x0E6` (`MSG_IS_DAT_ABR` ABS wheel ticks) have been superseded by direct hardware verification against a physical Hiworld PA12.20 decoder.

The Peugeot 407 Steering Column Control Module (`COM2005`) integrates an optical/magnetic Steering Wheel Angle Sensor (SAS / CAV). The BSI gateway translates the 500 kbps `0x305` frame into the 125 kbps Comfort CAN frame **`0x10B`** every 50 ms.

When the vehicle is running (`0x036` Ignition ON) and placed in reverse gear (`0x0F6` Byte 7 `0x80`), the CAN box translates the 16-bit signed value from `0x10B` (Bytes 0–1, in $0.1^\circ$ units) by dividing by `10` and inverting the sign into whole degrees ($1^\circ$ units) inside Hiworld **`Cmd 0x11`** (Bytes 6–7) and **`Cmd 0x1A`** (Bytes 6–7).


```

+------------------------------------------------------------------------------------+
|              Peugeot 407 Steering Column (COM2005) -> BSI Gateway                  |
|      [CAN IS 0x305 @ 500k] ---> [BSI Gateway] ---> [CAN Comfort 0x10B @ 125k]      |
+------------------------------------------------------------------------------------+
│
[PSA CAN 0x10B]
▼
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Captures 50 ms steering angle frames (CAN ID 0x10B, DLC 8)                    |
|   2. Verifies Byte 3 status flags (0x00 = valid) & converts 0.1° to -1.0° steps    |
|   3. Dispatches Hiworld (Cmd 0x11, Bytes 6-7, LEN=0x0A) / Raise (Cmd 0x29, LE)     |
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

### 2.1 PSA Comfort CAN Steering Angle Frame (`0x10B`)
- **CAN ID (Comfort 125 kbps):** `0x10B` (Standard 11-bit Identifier, routed by BSI from CAN IS `0x305`)
- **DLC:** 8 bytes (7 or 8 bytes accepted)
- **Transmission Cycle:** Periodic 50 ms
- **Quiescent / Center Frame:** `00 00 00 00 00 00 00 00`


```

+--------+--------+------------------------------------+-----------------------------+
| Byte   | Bit    | Function / Signal Name             | Value / Encoding Definition |
+--------+--------+------------------------------------+-----------------------------+
| Byte 0 | 7..0   | Steering Angle High Byte (MSB)     | Signed 16-bit Big-Endian    |
+--------+--------+------------------------------------+-----------------------------+
| Byte 1 | 7..0   | Steering Angle Low Byte (LSB)      | Signed 16-bit Big-Endian    |
+--------+--------+------------------------------------+-----------------------------+
| Byte 2 | 7..0   | Rotation Speed                     | Unsigned 8-bit (4 deg/s/bit)|
+--------+--------+------------------------------------+-----------------------------+
| Byte 3 | 7..0   | Sensor Status / Error Flags        | Must be 0x00 for valid SAS  |
+--------+--------+------------------------------------+-----------------------------+
| Byte 4 | 7..0   | Reserved / Counter                 | 0x00                        |
+--------+--------+------------------------------------+-----------------------------+
| Byte 5 | 7..0   | Reserved Padding                   | 0x00                        |
+--------+--------+------------------------------------+-----------------------------+
| Byte 6 | 7..0   | Reserved Padding                   | 0x00                        |
+--------+--------+------------------------------------+-----------------------------+
| Byte 7 | 7..0   | Reserved Padding                   | 0x00                        |
+--------+--------+------------------------------------+-----------------------------+

```

> [!IMPORTANT]
> **Byte 3 Validation Gate:** Hiworld PA12.20 validates the status byte (`Byte 3`) of `0x10B`. Non-zero error bits in `Byte 3..7` cause the decoder to discard the frame and freeze or zero out the UART steering angle output. Setting `Bytes 2..7` to `0x00` guarantees acceptance.

### 2.2 Physical Scaling & Direction (PSA CAN vs. Hiworld UART):
On the PSA CAN bus (`0x10B` / `0x305`), the angle is encoded in **tenths of a degree ($0.1^\circ$)**, where **Left (CCW) is positive** and **Right (CW) is negative**:
$$\text{CAN\_Raw}_{16} = \text{Round}(\text{Angle}_{^\circ} \times 10.0)$$
$$\text{UART\_Angle}_{16} = -\left(\frac{\text{CAN\_Raw}_{16}}{10}\right)$$

- **Physical Range:** $-780.0^\circ \dots +780.0^\circ$ (Typical lock-to-lock: $-540.0^\circ \dots +540.0^\circ$, Raw: $-5400 \dots +5400$).
- **Direction & Polarity Inversion:**
  - **Center ($0.0^\circ$):** $\text{CAN\_Raw} = 0$ (`0x0000`) $\implies \text{Hiworld UART} = 0^\circ$ (`0x0000`).
  - **Left Turn ($+90.0^\circ$ CCW):** $\text{CAN\_Raw} = +900$ (`0x0384`) $\implies \text{Hiworld UART} = -90^\circ$ (`0xFFA6`).
  - **Right Turn ($-90.0^\circ$ CW):** $\text{CAN\_Raw} = -900$ (`0xFC7C`) $\implies \text{Hiworld UART} = +90^\circ$ (`0x005A`).

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Hiworld Base Info Frame Steering Telemetry (`Cmd 0x11`)
In the Hiworld PSA protocol (verified on `PA12.20` / `H1H2PA123A-240717`), steering wheel angle is multiplexed inside the **10-byte** Base Info Frame (`Cmd 0x11`) and mirrored in the Extended Engine/Chassis Frame (`Cmd 0x1A`, Bytes 6–7):
- **Sync Header:** `0x5A 0xA5`
- **Length ($L$):** `0x0A` (10 payload bytes; 8-byte legacy layout also supported by `PeugeotDataParser.java`)
- **Command ID:** `0x11` (`17` decimal / `Handle.CarBaseInfo`)
- **Payload Layout (10 Bytes):**
  - `Byte 0`: Status Flags (`Bit 0` `0x01` = ACC, `Bit 1` `0x02` = ILL, `Bit 2` `0x04` = Park Brake, `Bit 5` `0x20` = Reverse Gear; e.g., `0x27` when running in reverse)
  - `Byte 1`: Reserved (`0x00`)
  - `Byte 2`: Steering Wheel Button Key Code (`0x00` when idle, mapped from `0x21F`)
  - `Byte 3`: Key State / Speed Info (`0x00` or `0x01`)
  - `Byte 4..5`: Reserved (`0x00 0x00`)
  - `Byte 6`: **Steering Angle High Byte** (`int16_t` MSB, Big Endian, whole degrees: $-\text{CAN\_Raw} / 10$)
  - `Byte 7`: **Steering Angle Low Byte** (`int16_t` LSB, Big Endian, whole degrees: $-\text{CAN\_Raw} / 10$)
  - `Byte 8..9`: Door / Configuration Tail Flags (e.g., `0x5E 0x22` on PA12.20)
- **Checksum:** `((Length + CmdID + sum(Payload)) - 1) & 0xFF`
- **Total Frame Wire Length:** 15 bytes (`5A A5 0A 11 [10 Bytes] Checksum`)
- **UART Flow Control:** Hiworld PA12.20 expects a 6-byte host ACK (`5A A5 01 FF 11 0F`) to maintain continuous 20 Hz angle streaming without retransmissions.

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

#define PSA_CAN_ID_SAS_COMFORT 0x10BU
#define HIWORLD_SOF1           0x5AU
#define HIWORLD_SOF2           0xA5U
#define HIWORLD_CMD_CAR_BASE   0x11U

typedef struct {
    int16_t can_raw_deci_deg; /* PSA 0x10B raw 0.1 deg: +Left, -Right (-5400..+5400) */
    int16_t uart_angle_deg;   /* Hiworld 0x11 whole deg: -Left, +Right (-540..+540)  */
    uint8_t base_status_b0;   /* Bit 0: ACC (0x01), Bit 1: ILL (0x02), Bit 5: REV (0x20) */
} psa_sas_state_t;

typedef void (*canbox_uart_tx_fn)(const uint8_t *buf, size_t len);

typedef struct {
    psa_sas_state_t   state;
    canbox_uart_tx_fn uart_tx;
} psa_sas_ctx_t;

static inline void psa_sas_init(psa_sas_ctx_t *ctx, canbox_uart_tx_fn uart_tx) {
    memset(ctx, 0, sizeof(psa_sas_ctx_t));
    ctx->state.base_status_b0 = 0x27U; /* Default: ACC + ILL + PARK + REV active */
    ctx->uart_tx = uart_tx;
}

/* Transmit Hiworld PA12.20 Base Info & Steering Angle (Cmd 0x11, 10B Payload, Big Endian) */
static inline void psa_sas_send_hiworld(psa_sas_ctx_t *ctx) {
    if (!ctx || !ctx->uart_tx) return;

    uint16_t u_angle = (uint16_t)ctx->state.uart_angle_deg;

    uint8_t p[15];
    p[0]  = HIWORLD_SOF1;
    p[1]  = HIWORLD_SOF2;
    p[2]  = 0x0AU;                      /* Length: 10 Payload bytes (PA12.20 format) */
    p[3]  = HIWORLD_CMD_CAR_BASE;       /* Cmd ID: 0x11 */
    p[4]  = ctx->state.base_status_b0;  /* Byte 0: Status flags (e.g. 0x27) */
    p[5]  = 0x00U;                      /* Byte 1: Reserved */
    p[6]  = 0x00U;                      /* Byte 2: SWC Key Code (idle) */
    p[7]  = 0x00U;                      /* Byte 3: SWC Key State */
    p[8]  = 0x00U;                      /* Byte 4: Reserved */
    p[9]  = 0x00U;                      /* Byte 5: Reserved */
    p[10] = (uint8_t)((u_angle >> 8) & 0xFFU); /* Byte 6: Angle MSB (whole deg) */
    p[11] = (uint8_t)(u_angle & 0xFFU);        /* Byte 7: Angle LSB (whole deg) */
    p[12] = 0x5EU;                      /* Byte 8: PA12.20 config tail */
    p[13] = 0x22U;                      /* Byte 9: PA12.20 config tail */

    uint8_t sum = 0U;
    for (size_t i = 2U; i <= 13U; i++) {
        sum = (uint8_t)(sum + p[i]);
    }
    p[14] = (uint8_t)((sum - 1U) & 0xFFU);

    ctx->uart_tx(p, sizeof(p));
}

/* Process PSA Comfort CAN 0x10B Frame */
static inline bool psa_sas_process_can_0x10B(psa_sas_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (!ctx || !data || dlc < 4U) return false;

    /* Reject frame if sensor error flags are set in Byte 3 */
    if (data[3] != 0x00U) return false;

    /* Unpack Big-Endian signed 16-bit angle (0.1 deg units, +Left / -Right) */
    int16_t raw_deci_deg = (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
    if (raw_deci_deg == (int16_t)0x7FFF) return false;

    /* Convert to Hiworld whole degrees and invert polarity (-Left / +Right) */
    int16_t new_uart_deg = -(raw_deci_deg / 10);
    bool changed = (new_uart_deg != ctx->state.uart_angle_deg);

    ctx->state.can_raw_deci_deg = raw_deci_deg;
    ctx->state.uart_angle_deg   = new_uart_deg;

    if (changed) {
        psa_sas_send_hiworld(ctx);
    }
    return true;
}

#endif /* CANBOX_SAS_H */

```

---

# 5. Verification Vectors & Simulation Harness

> [!TIP]
> Ensure background ignition (`0x036#0E000020010000A0`) and reverse gear (`0x0F6#8E58000000000080`) frames are transmitted periodically on `can0` when testing a physical Hiworld PA12.20 box.

### Vector 1: Steering Centered (0.0°)

* **CAN ID `0x10B` Injection:**
```bash
cansend can0 10B#0000000000000000

```


* **Expected UART Output (Hiworld PA12.20 `Cmd 0x11`):**
* Frame: `5A A5 0A 11 27 00 00 00 00 00 00 00 5E 22 C1`
* Decoded Angle (`Data[6..7]`): `0x0000` ($0^\circ$)
* Checksum Calculation: `(0x0A + 0x11 + 0x27 + 0x5E + 0x22 - 1) & 0xFF = 0xC1`



### Vector 2: Steering Turned Left +21.0° (CAN Raw = +210 = `0x00D2` -> UART = -21° = `0xFFEB`)

* **CAN ID `0x10B` Injection:**
```bash
cansend can0 10B#00D2000000000000

```


* **Expected UART Output (Hiworld PA12.20 `Cmd 0x11`):**
* Frame: `5A A5 0A 11 27 00 00 00 00 00 FF EB 5E 22 AB`
* Decoded Angle (`Data[6..7]`): `0xFFEB` ($-21^\circ$)
* Checksum Calculation: `(0x0A + 0x11 + 0x27 + 0xFF + 0xEB + 0x5E + 0x22 - 1) & 0xFF = 0xAB`



### Vector 3: Steering Turned Left +60.0° (CAN Raw = +600 = `0x0258` -> UART = -60° = `0xFFC4`)

* **CAN ID `0x10B` Injection:**
```bash
cansend can0 10B#0258000000000000

```


* **Expected UART Output (Hiworld PA12.20 `Cmd 0x11`):**
* Frame: `5A A5 0A 11 27 00 00 00 00 00 FF C4 5E 22 84`
* Decoded Angle (`Data[6..7]`): `0xFFC4` ($-60^\circ$)
* Checksum Calculation: `(0x0A + 0x11 + 0x27 + 0xFF + 0xC4 + 0x5E + 0x22 - 1) & 0xFF = 0x84`



### Vector 4: Steering Turned Right -90.0° (CAN Raw = -900 = `0xFC7C` -> UART = +90° = `0x005A`)

* **CAN ID `0x10B` Injection:**
```bash
cansend can0 10B#FC7C000000000000

```


* **Expected UART Output (Hiworld PA12.20 `Cmd 0x11`):**
* Frame: `5A A5 0A 11 27 00 00 00 00 00 00 5A 5E 22 1B`
* Decoded Angle (`Data[6..7]`): `0x005A` ($+90^\circ$)
* Checksum Calculation: `(0x0A + 0x11 + 0x27 + 0x00 + 0x5A + 0x5E + 0x22 - 1) & 0xFF = 0x1B`

