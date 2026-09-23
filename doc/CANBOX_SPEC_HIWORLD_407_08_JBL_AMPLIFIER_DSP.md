# CAN Box Protocol Specification: JBL Premium Amplifier & Audio DSP
## Topic 08: Peugeot 407 JBL Sound System & Digital Signal Processing
**Document File:** `CANBOX_SPEC_HIWORLD_407_08_JBL_AMPLIFIER_DSP.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** `https://github.com/fazerxlo/canbox/tree/main/doc/CAN2004_amp.md`, `CAN_messages.md`

---

# 1. Functional Domain & Architecture Overview

High-trim Peugeot 407 vehicles feature a digital JBL 10-speaker sound system powered by an external CAN-connected multi-channel audio amplifier. The amplifier operates on the PSA Comfort CAN bus, receiving tone adjustments (Bass, Treble, Equalizer, Balance, Fader, Dynamic Loudness, Speed-Dependent Volume) and broadcasting current internal gain settings.

```
+------------------------------------------------------------------------------------+
|                         Peugeot 407 JBL Digital Audio Amplifier                    |
|             [10-Speaker Multichannel DSP, Bass, Treble, EQ, Balance, Fader]        |
+------------------------------------------------------------------------------------+
                                          │
                                [PSA CAN 0x1E5 / 0x280]
                                          ▼
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Captures amplifier state telemetry from CAN ID 0x1E5                          |
|   2. Converts discrete gain steps (0..14) to Hiworld DSP packets                   |
|   3. Decodes Android equalizer adjustments (Cmd 0xAD / 0xC5) -> CAN ID 0x280       |
+------------------------------------------------------------------------------------+
                                          │
                    [UART Serial: 38400 baud, 8N1 / Hiworld Protocol]
                                          ▼
+------------------------------------------------------------------------------------+
|                     Android Headunit Equalizer / DSP Application                   |
|           (Renders graphic sliders for Bass/Treble/Bal/Fad & EQ profiles)          |
+------------------------------------------------------------------------------------+
```

---

# 2. PSA CAN Bus Bitfield Specification

### 2.1 PSA Amplifier Telemetry Frame (`0x1E5`)
- **CAN ID:** `0x1E5` (Standard 11-bit Identifier)
- **DLC:** 8 bytes
- **Transmission Cycle:** Periodic 500 ms or immediate on adjustment

```
+--------+--------+------------------------------------+-----------------------------+
| Byte   | Bit    | Function / Signal Name             | Value / Range Definition    |
+--------+--------+------------------------------------+-----------------------------+
| Byte 0 | 7..0   | Fixed Padding Byte                 | 0x00                        |
+--------+--------+------------------------------------+-----------------------------+
| Byte 1 | 7..0   | Bass Level                         | 0..14 (7 = Center / 0 dB)   |
+--------+--------+------------------------------------+-----------------------------+
| Byte 2 | 7..0   | Treble Level                       | 0..14 (7 = Center / 0 dB)   |
+--------+--------+------------------------------------+-----------------------------+
| Byte 3 | 7..0   | Balance (Left / Right)             | 0..14 (0=Left, 7=Mid, 14=Rt)|
+--------+--------+------------------------------------+-----------------------------+
| Byte 4 | 7..0   | Fader (Rear / Front)               | 0..14 (0=Rear, 7=Mid, 14=Fr)|
+--------+--------+------------------------------------+-----------------------------+
| Byte 5 | 7..0   | EQ Preset Profile                  | 0=Off, 1=Pop, 2=Classic,    |
|        |        |                                    | 3=Elec, 4=Jazz, 5=Vocal     |
+--------+--------+------------------------------------+-----------------------------+
| Byte 6 | Bit 4  | Loudness Dynamic Boost             | 0x10 (1 = ON, 0 = OFF)      |
|        | Bit 3..0| Speed Volume Compensation Level   | 0..3 (0=Off, 1=Low, 3=High) |
+--------+--------+------------------------------------+-----------------------------+
| Byte 7 | 7..0   | Master JBL Amplifier Volume        | 0..30 (Discrete gain steps) |
+--------+--------+------------------------------------+-----------------------------+
```

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Hiworld Amplifier Feedback Telemetry (`Cmd 0x56`)
- **Sync Header:** `0x5A 0xA5`
- **Length ($L$):** `0x08` (8 payload bytes)
- **Command ID:** `0x56`
- **Payload Layout (8 Bytes):**
  - `Byte 0`: `0x00` (Status)
  - `Byte 1`: Bass ($0 \dots 14$, $7 = \text{Center}$)
  - `Byte 2`: Treble ($0 \dots 14$)
  - `Byte 3`: Balance ($0 \dots 14$)
  - `Byte 4`: Fader ($0 \dots 14$)
  - `Byte 5`: EQ Preset Profile ($0 \dots 5$)
  - `Byte 6`: Flags: Loudness (`0x10`), Speed Volume (`0..3`)
  - `Byte 7`: Master Volume ($0 \dots 30$)
- **Checksum:** `((Length + CmdID + sum(Payload)) - 1) & 0xFF`
- **Total Frame Wire Length:** 13 bytes (`5A A5 08 56 [8 Bytes] Checksum`)

---

# 4. Pure C99 Firmware Implementation

```c
#ifndef CANBOX_JBL_H
#define CANBOX_JBL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define HIWORLD_SOF1        0x5A
#define HIWORLD_SOF2        0xA5
#define HIWORLD_CMD_DSP_FEEDBACK 0x56

typedef struct {
    uint8_t master_volume; /* 0..30 */
    uint8_t bass;          /* 0..14 (7=Center) */
    uint8_t treble;        /* 0..14 */
    uint8_t middle;        /* 0..14 */
    uint8_t balance;       /* 0..14 */
    uint8_t fader;         /* 0..14 */
    uint8_t eq_preset;     /* 0..5 */
    bool    loudness;
    uint8_t speed_volume;  /* 0..3 */
} psa_jbl_state_t;

typedef void (*canbox_uart_tx_fn)(const uint8_t *buf, size_t len);

typedef struct {
    psa_jbl_state_t   state;
    canbox_uart_tx_fn uart_tx;
} psa_jbl_ctx_t;

static inline void psa_jbl_init(psa_jbl_ctx_t *ctx, canbox_uart_tx_fn uart_tx) {
    memset(ctx, 0, sizeof(psa_jbl_ctx_t));
    ctx->state.bass    = 7;
    ctx->state.treble  = 7;
    ctx->state.middle  = 7;
    ctx->state.balance = 7;
    ctx->state.fader   = 7;
    ctx->uart_tx = uart_tx;
}

/* Transmit Hiworld JBL Telemetry Packet (Cmd 0x56) */
static inline void psa_jbl_send_hiworld(psa_jbl_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[13];
    p[0] = HIWORLD_SOF1;
    p[1] = HIWORLD_SOF2;
    p[2] = 0x08;                     /* Length: 8 Payload bytes */
    p[3] = HIWORLD_CMD_DSP_FEEDBACK; /* Cmd ID: 0x56 */
    p[4] = 0x00;
    p[5] = ctx->state.bass;
    p[6] = ctx->state.treble;
    p[7] = ctx->state.balance;
    p[8] = ctx->state.fader;
    p[9] = ctx->state.eq_preset;
    p[10] = (ctx->state.loudness ? 0x10 : 0x00) | (ctx->state.speed_volume & 0x03);
    p[11] = ctx->state.master_volume;

    uint8_t sum = 0;
    for (size_t i = 2; i <= 11; i++) {
        sum += p[i];
    }
    p[12] = (uint8_t)((sum - 1) & 0xFF);

    ctx->uart_tx(p, 13);
}

#endif /* CANBOX_JBL_H */
```
