# CAN Box Protocol Specification: OEM JBL Sound Amplifier (DSP)
## Topic 08: Peugeot 407 Premium JBL Amplifier Control & Feedback
**Document File:** `CANBOX_SPEC_HIWORLD_407_08_JBL_AMPLIFIER_DSP.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** `https://github.com/fazerxlo/canbox/tree/main/doc/CAN2004_radio.md`, `CAN_messages.md`

---

# 1. Functional Domain & Architecture Overview

High-trim Peugeot 407 vehicles equipped with the OEM JBL Hi-Fi audio system feature a dedicated digital signal processor (DSP) multi-channel power amplifier in the boot. The amplifier interfaces directly with the PSA Comfort CAN bus:
- **Feedback (`0x1A0`):** The JBL amplifier broadcasts current DSP settings, internal volume, and diagnostic status.
- **Downlink Control (`0x280`):** The headunit adjusts equalizer gains, fader/balance, loudness, and volume compensation by injecting CAN frames to ID `0x280`.
The Peugeot 407 JBL Hi-Fi audio system and RD4 radio headunit broadcast volume, tone adjustments, equalizer curves, and power amplifier telemetry over PSA Comfort CAN:
- **`0x1A5` (Radio Volume):** 1-byte periodic volume frame (`VOLUME_RADIO`). Bits 4:0 carry master volume ($0 \dots 30$). Bits 7:5 carry the `VOLFLAG` (`0xE0` = stable, `0x00` = volume changing).
- **`0x1E5` (Audio Settings):** 7-byte periodic frame (`REGLAGES_SON`) carrying L/R Balance, F/R Fader, Bass, Treble, Loudness, Automatic-Volume, and Ambiance / Equalizer presets offset from center `0x3F`.
- **`0x1A0` / `0x280`:** Direct JBL DSP Amplifier telemetry and downlink command frames.

The CAN adapter bridges Android Equalizer/DSP settings into native JBL CAN frames.

```
+------------------------------------------------------------------------------------+
|                         Peugeot 407 OEM JBL DSP Power Amplifier                    |
|             [10-Speaker Multichannel DSP: EQ, Bass, Treble, Fader, Vol]            |
+------------------------------------------------------------------------------------+
                         │ (Feedback CAN 0x1A0)     ▲ (Control CAN 0x280)
                         ▼                          │
                         │ (CAN 0x1A5 / 0x1E5 / 0x1A0) ▲ (Downlink 0x280)
                         ▼                             │
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Captures JBL Amp status frames (CAN ID 0x1A0)                                 |
|   2. Dispatches Hiworld Amplifier Telemetry (Cmd 0x56)                             |
|   3. Decodes Android DSP App adjustments (Hiworld Cmd 0xC5)                        |
|   4. Synthesizes PSA CAN 0x280 DSP control frames                                  |
|   1. Decodes native CAN 0x1A5 (0..30 Volume) & 0x1E5 Audio Parameters (0x3F center)|
|   2. Converts Equalizer Ambiance codes (Pop, Classic, Jazz, Techno, Vocal)         |
|   3. Dispatches Hiworld 0x56 Amplifier Telemetry Packets                           |
|   4. Accepts Android Downlink Cmd 0xC5 -> Synthesizes CAN 0x280 control frames     |
+------------------------------------------------------------------------------------+
                         │ (UART Telemetry)         ▲ (UART Commands)
                         ▼                          │
                         │ (UART Telemetry)            ▲ (UART Commands)
                         ▼                             │
+------------------------------------------------------------------------------------+
|                     Android Headunit DSP & Equalizer Application                   |
|         (Provides touchscreen sliders for 14-step EQ, balance/fader grid)          |
+------------------------------------------------------------------------------------+
```

---

# 2. PSA CAN Bus Bitfield Specification

### 2.1 JBL Amplifier Feedback Frame (`0x1A0`)
- **CAN ID:** `0x1A0` (DLC: 8, Cycle: 200 ms)
### 2.1 PSA Radio Volume Frame (`0x1A5`)
- **CAN ID:** `0x1A5` (DLC: 1, Period: 100 ms)
- `Bits 7:5`: `VOLFLAG` (`0xE0` = stable idle, `0x00` = active change)
- `Bits 4:0`: `VOLUME` ($0 \dots 30$ linear steps)

### 2.2 PSA Audio Settings Frame (`0x1E5`)
- **CAN ID:** `0x1E5` (DLC: 7, Period: 100 ms)
- **Center Baseline:** `0x3F` ($63$ decimal = flat / center $0\text{ dB}$). Range $-9 \dots +9$ (`0x36`..`0x48`).

```
+--------+--------+------------------------------------+-----------------------------+
| Byte   | Bit    | Function / Signal Name             | Value / Range Definition    |
+--------+--------+------------------------------------+-----------------------------+
| Byte 0 | 7..0   | Fixed Padding Byte                 | 0x00                        |
| Byte 0 | Bit 7  | Balance Menu Active Flag           | 1 = Menu open               |
|        | 6..0   | Left / Right Balance               | Value - 0x3F (-9..0..+9)    |
+--------+--------+------------------------------------+-----------------------------+
| Byte 1 | 7..0   | Bass Level                         | 0..14 (7=Center, -7..0..+7) |
| Byte 1 | Bit 7  | Fader Menu Active Flag             | 1 = Menu open               |
|        | 6..0   | Front / Rear Fader                 | Value - 0x3F (-9..0..+9)    |
+--------+--------+------------------------------------+-----------------------------+
| Byte 2 | 7..0   | Treble Level                       | 0..14 (7=Center, -7..0..+7) |
| Byte 2 | Bit 7  | Bass Menu Active Flag              | 1 = Menu open               |
|        | 6..0   | Bass Tone Level                    | Value - 0x3F (-9..0..+9)    |
+--------+--------+------------------------------------+-----------------------------+
| Byte 3 | 7..0   | Balance (Left / Right)             | 0..14 (0=Left, 7=Mid, 14=Rt)|
| Byte 3 | 7..0   | Fixed Zero Padding                 | 0x00                        |
+--------+--------+------------------------------------+-----------------------------+
| Byte 4 | 7..0   | Fader (Rear / Front)               | 0..14 (0=Rear, 7=Mid, 14=Fr)|
| Byte 4 | Bit 7  | Treble Menu Active Flag            | 1 = Menu open               |
|        | 6..0   | Treble Tone Level                  | Value - 0x3F (-9..0..+9)    |
+--------+--------+------------------------------------+-----------------------------+
| Byte 5 | 7..0   | EQ Preset Profile                  | 0=Off, 1=Pop, 2=Classic,    |
|        |        |                                    | 3=Elec, 4=Jazz, 5=Vocal     |
| Byte 5 | Bit 6  | Loudness Dynamic Boost             | 1 = ON, 0 = OFF             |
|        | Bit 2:0| Speed-Dependent Volume Comp        | 0x07 = Enabled, 0x00 = Off  |
+--------+--------+------------------------------------+-----------------------------+
| Byte 6 | Bit 4  | Loudness Dynamic Boost             | 0x10 (1 = ON, 0 = OFF)      |
|        | Bit 3..0| Speed Volume Compensation Level   | 0..3 (0=Off, 1=Low, 3=High) |
| Byte 6 | Bit 5:0| Ambiance / Equalizer Preset        | See Preset Table below      |
+--------+--------+------------------------------------+-----------------------------+
| Byte 7 | 7..0   | Master JBL Amplifier Volume        | 0..30 (Discrete gain steps) |
+--------+--------+------------------------------------+-----------------------------+
```

### 2.2 JBL Amplifier Downlink Control Frame (`0x280`)
- **CAN ID:** `0x280` (DLC: 8, Event-driven upon slider drag)
- `Byte 0`: Bass level ($0 \dots 14$)
- `Byte 1`: Treble level ($0 \dots 14$)
- `Byte 2`: Middle level ($0 \dots 14$)
- `Byte 3`: Balance ($0 \dots 14$)
- `Byte 4`: Fader ($0 \dots 14$)
- `Byte 5`: EQ Preset ($0 \dots 5$)
- `Byte 6`: Loudness & Speed Volume flags
- `Byte 7`: Master Volume level ($0 \dots 30$)
### Ambiance / Equalizer Preset Codes (`0x1E5` Byte 6):
| CAN Code | Preset Name | Hiworld EQ Preset ID |
|:---:|:---|:---:|
| `0x03` | **None / Custom** | `0` |
| `0x07` | **Classical** | `2` |
| `0x0B` | **Jazz / Blues** | `4` |
| `0x0F` | **Pop / Rock** | `1` |
| `0x13` | **Vocal** | `5` |
| `0x17` | **Techno / Electronic** | `3` |

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Hiworld Amplifier Feedback Telemetry (`Cmd 0x56`)
- **Sync Header:** `0x5A 0xA5` | **Length:** `0x09` | **Cmd:** `0x56`
- **Payload Layout (8 Bytes):**
  - `Byte 0`: `0x00` (Status)
  - `Byte 1`: Bass ($0 \dots 14$, $7 = 0\text{ dB}$)
  - `Byte 0`: `0x00`
  - `Byte 1`: Bass ($0 \dots 14$, $7 = \text{Center}$)
  - `Byte 2`: Treble ($0 \dots 14$)
  - `Byte 3`: Balance ($0 \dots 14$)
  - `Byte 4`: Fader ($0 \dots 14$)
  - `Byte 5`: EQ Preset Profile ($0 \dots 5$)
  - `Byte 6`: Flags: Loudness (`0x10`), Speed Volume (`0x00..0x03`)
  - `Byte 6`: Loudness (`0x10`) & Speed Volume (`0..3`)
  - `Byte 7`: Master Volume ($0 \dots 30$)
- **Checksum:** 8-bit sum modulo 256 over `Length + CmdID + Payload`.

### 3.2 Downlink Audio Adjustments from Android (`Cmd 0xC5`)
When the user drags equalizer sliders on the Android UI:
- **UART Command:** `5A A5 09 C5 [Fader] [Bal] [Bass] [Treb] [Mid] [EQ] [Flags] [Vol] [CS]`
- **CAN Action:** Serializer unpacks fields and sends CAN frame to ID `0x280`.

---

# 4. Pure C99 Firmware Implementation

```c
#ifndef CANBOX_JBL_H
#define CANBOX_JBL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

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
typedef void (*canbox_can_tx_fn)(uint32_t id, const uint8_t *data, uint8_t dlc);

typedef struct {
    psa_jbl_state_t   state;
    canbox_uart_tx_fn uart_tx;
    canbox_can_tx_fn  can_tx;
} psa_jbl_ctx_t;

static inline void psa_jbl_init(psa_jbl_ctx_t *ctx, canbox_uart_tx_fn uart_tx, canbox_can_tx_fn can_tx) {
static inline void psa_jbl_init(psa_jbl_ctx_t *ctx, canbox_uart_tx_fn uart_tx) {
    memset(ctx, 0, sizeof(psa_jbl_ctx_t));
    ctx->state.bass    = 7;
    ctx->state.treble  = 7;
    ctx->state.middle  = 7;
    ctx->state.balance = 7;
    ctx->state.fader   = 7;
    ctx->uart_tx = uart_tx;
    ctx->can_tx  = can_tx;
}

/* Transmit Hiworld JBL Telemetry Packet (Cmd 0x56) */
static inline void psa_jbl_send_hiworld(psa_jbl_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[12];
    p[0] = 0x5A;
    p[1] = 0xA5;
    p[2] = 0x09; /* Length: 1 Cmd + 8 Payload */
    p[3] = 0x56; /* Cmd ID */
    p[4] = 0x00;
    p[5] = ctx->state.bass;
    p[6] = ctx->state.treble;
    p[7] = ctx->state.balance;
    p[8] = ctx->state.fader;
    p[9] = ctx->state.eq_preset;
    p[10] = (ctx->state.loudness ? 0x10 : 0x00) | (ctx->state.speed_volume & 0x03);
    p[11] = ctx->state.master_volume;

    uint8_t sum = 0;
    for (size_t i = 2; i <= 11; i++) sum += p[i];
    p[12] = sum;

    ctx->uart_tx(p, 13);
}

/* Process Inbound PSA CAN 0x1A0 (JBL Feedback) */
static inline void psa_jbl_process_can_0x1A0(psa_jbl_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 8) return;

    ctx->state.bass          = data[1];
    ctx->state.treble        = data[2];
    ctx->state.balance       = data[3];
    ctx->state.fader         = data[4];
    ctx->state.eq_preset     = data[5];
    ctx->state.loudness      = (data[6] & 0x10) ? true : false;
    ctx->state.speed_volume  = data[6] & 0x03;
    ctx->state.master_volume = data[7];

/* Process PSA CAN 0x1A5 (Volume) */
static inline void psa_jbl_process_can_0x1A5(psa_jbl_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 1) return;
    ctx->state.master_volume = data[0] & 0x1F;
    psa_jbl_send_hiworld(ctx);
}

/* Process Android Downlink DSP Adjustment (Cmd 0xC5) */
static inline void psa_jbl_process_downlink_0xC5(psa_jbl_ctx_t *ctx, const uint8_t *payload, size_t len) {
    if (!ctx->can_tx || len < 8) return;
/* Process PSA CAN 0x1E5 (Audio Settings) */
static inline void psa_jbl_process_can_0x1E5(psa_jbl_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 7) return;

    uint8_t can_data[8];
    can_data[0] = payload[2]; /* Bass */
    can_data[1] = payload[3]; /* Treble */
    can_data[2] = payload[4]; /* Middle */
    can_data[3] = payload[1]; /* Balance */
    can_data[4] = payload[0]; /* Fader */
    can_data[5] = payload[5]; /* EQ Preset */
    can_data[6] = payload[6]; /* Loudness & Speed Volume */
    can_data[7] = payload[7]; /* Volume */
    /* Unpack 0x3F center offset (-7..+7 mapping to 0..14) */
    int8_t bal = (int8_t)(data[0] & 0x7F) - 0x3F;
    int8_t fad = (int8_t)(data[1] & 0x7F) - 0x3F;
    int8_t bas = (int8_t)(data[2] & 0x7F) - 0x3F;
    int8_t trb = (int8_t)(data[4] & 0x7F) - 0x3F;

    ctx->can_tx(0x280, can_data, 8);
    ctx->state.balance = (uint8_t)(bal + 7);
    ctx->state.fader   = (uint8_t)(fad + 7);
    ctx->state.bass    = (uint8_t)(bas + 7);
    ctx->state.treble  = (uint8_t)(trb + 7);

    ctx->state.loudness = (data[5] & 0x40) ? true : false;
    ctx->state.speed_volume = ((data[5] & 0x07) == 0x07) ? 1 : 0;

    /* Map Ambiance Code */
    uint8_t amb = data[6] & 0x3F;
    switch (amb) {
        case 0x07: ctx->state.eq_preset = 2; break; /* Classical */
        case 0x0B: ctx->state.eq_preset = 4; break; /* Jazz */
        case 0x0F: ctx->state.eq_preset = 1; break; /* Pop */
        case 0x13: ctx->state.eq_preset = 5; break; /* Vocal */
        case 0x17: ctx->state.eq_preset = 3; break; /* Techno */
        default:   ctx->state.eq_preset = 0; break; /* None */
    }

    psa_jbl_send_hiworld(ctx);
}

#endif /* CANBOX_JBL_H */
```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: Flat Tone Settings (Bass=7, Treble=7, Bal=7, Fader=7, Loudness=ON, Vol=18)
- **CAN ID `0x1A0` Injection:**
### Vector 1: Volume Set to Level 15 (0x0F) with Stable Flag (0xEF)
- **CAN ID `0x1A5` Injection:**
  ```bash
  cansend vcan0 1A0#0007070707001012
  cansend vcan0 1A5#EF
  ```
- **Expected UART Output (Hiworld `0x56`):**
  - Frame: `5A A5 09 56 00 07 07 07 07 00 10 12 A9`
  - Frame: `5A A5 09 56 00 07 07 07 07 00 00 0F 83`

### Vector 2: Android Downlink Boost Bass to +4 (Raw 11 = 0x0B)
- **UART Downlink Received (`Cmd 0xC5`):**
  - `5A A5 09 C5 07 07 0B 07 07 00 10 12 BC`
- **Expected CAN Frame Injected:**
  - ID: `0x280`, DLC: 8, Data: `0B 07 07 07 07 00 10 12`

### Vector 2: Audio Settings Flat (0x3F), Pop-Rock EQ (0x0F), Loudness ON (0x40)
- **CAN ID `0x1E5` Injection:**
  ```bash
  cansend vcan0 1E5#3F3F3F003F400F
  ```
- **Expected UART Output (Hiworld `0x56`):**
  - Frame: `5A A5 09 56 00 07 07 07 07 01 10 0F 95`
