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

---

# 2. PSA CAN Bus Bitfield Specification

### 2.1 PSA Amplifier Telemetry Frame (`0x1E5`)
- **CAN ID:** `0x1E5` (Standard 11-bit Identifier)
- **DLC:** 8 bytes
- **Transmission Cycle:** Periodic 500 ms or immediate on adjustment

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Hiworld Amplifier Feedback Telemetry (`Cmd 0x56`)
- **Sync Header:** `0x5A 0xA5`
- **Length ($L$):** `0x08` (8 payload bytes)
- **Command ID:** `0x56`
- **Payload Layout (8 Bytes):**
  - `Byte 0`: Status bitfield:
    - **Bit 7 (`0x80`):** Mute active (`1` = Muted, `0` = Unmuted)
    - **Bits 3:0 (`0x0F`):** Active adjustment mode / OSD popup trigger:
      - `0x00`: Idle / Dismiss OSD / Background telemetry sync (never triggers popup)
      - `0x01`: Master Volume HUD popup
      - `0x02`: Bass OSD popup
      - `0x03`: Treble OSD popup
      - `0x04`: Balance OSD popup
      - `0x05`: Fader OSD popup
      - `0x06`: Loudness OSD popup
      - `0x07`: Speed-dependent volume OSD popup
      - `0x08`: Equalizer preset OSD popup
    *(Note: During periodic background resync and `Cmd 0x24` step 2 ping responses, Bits 3:0 must strictly be set to `0x00` to prevent triggering unsolicited Volume HUD popups on the Android Head Unit).*
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
