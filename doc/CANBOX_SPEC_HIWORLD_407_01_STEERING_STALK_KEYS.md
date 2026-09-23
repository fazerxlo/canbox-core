# CAN Box Protocol Specification: Steering Column Stalk & Button Controls
## Topic 01: Peugeot 407 Multifunction Stalk & Rotary Encoder
**Document File:** `CANBOX_SPEC_HIWORLD_407_01_STEERING_STALK_KEYS.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** `https://github.com/fazerxlo/canbox/tree/main/doc/CAN2004_sterringkeys.md`, `CAN2004_autowp_comparison.md`, `CAN2004_radio.md`

---

# 1. Functional Domain & Architecture Overview

In Peugeot 407 (PSA CAN2004 architecture), the steering column multifunction remote stalk communicates button presses, track changes, source selection, and rotary encoder wheel ticks over the Comfort CAN bus at 125 kbps:
- **Native PSA CAN2004 Remote Frame (`0x21F`):** A 3-byte momentary command frame (`[cmd, aux, reserved]`) sent by the steering stalk switch module.
- **Steering Wheel Panel Frame (`0x3E5`):** A 6-byte panel frame used in certain high-trim steering wheels.
- **Generic CAN Box Alias (`0x0F6`):** Many commercial Chinese CAN adapters normalize stalk inputs into internal 0x0F6 event frames.

```
+------------------------------------------------------------------------------------+
|                         Peugeot 407 Steering Column Stalk                          |
|             [Vol+, Vol-, Seek+, Seek-, Source, OK, ESC, Menu, Tel, Scroll]         |
+------------------------------------------------------------------------------------+
                                          │
                                [PSA CAN 0x21F / 0x0F6]
                                          ▼
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Captures native 3-byte CAN ID 0x21F or 0x0F6 remote frames                    |
|   2. Debounces key press/release transitions                                       |
|   3. Tracks rotary scroll wheel increments/decrements                              |
|   4. Translates actions to Hiworld (0x5AA5 / Cmd 0x11) & Raise (0x2E / Cmd 0x02)   |
+------------------------------------------------------------------------------------+
                                          │
                    [UART Serial: 38400 baud, 8N1 / Hiworld Protocol]
                                          ▼
+------------------------------------------------------------------------------------+
|                     Android Headunit Infotainment Subsystem                        |
|       (Processes Key Codes: Volume, Media Control, Android Navigation, Apps)       |
+------------------------------------------------------------------------------------+
```

---

# 2. PSA CAN Bus Bitfield Specification

### 2.1 Native PSA CAN2004 Steering Remote Frame (`0x21F`)
- **CAN ID:** `0x21F` (Standard 11-bit Identifier)
- **DLC:** 3 bytes
- **Frame Format:** `[Byte 0: Command Mask] [Byte 1: Aux / Pulse Counter] [Byte 2: 0x00]`
- **Transmission Cycle:** Event-driven upon button press/release

```
+--------+--------+------------------------------------+-----------------------------+
| Byte   | Bit    | Function / Signal Name             | Bit Mask / Value Definition |
+--------+--------+------------------------------------+-----------------------------+
| Byte 0 | Bit 7  | Next Track / Seek+ (Forward)       | 0x80 (1 = Pressed)          |
|        | Bit 6  | Previous Track / Seek- (Backward)  | 0x40 (1 = Pressed)          |
|        | Bit 3  | Volume Up                          | 0x08 (1 = Pressed)          |
|        | Bit 2  | Volume Down                        | 0x04 (1 = Pressed)          |
|        | Bit 1  | Source / Mode Toggle               | 0x02 (1 = Pressed)          |
|        | Bit 0  | Idle / Release State               | 0x00                        |
+--------+--------+------------------------------------+-----------------------------+
| Byte 1 | 7..0   | Secondary / Scroll Pulse Counter   | Value stays ~0x09/0x0B      |
+--------+--------+------------------------------------+-----------------------------+
| Byte 2 | 7..0   | Reserved Padding                   | Always 0x00                 |
+--------+--------+------------------------------------+-----------------------------+
```

### 2.2 Alternate CAN ID `0x0F6` Byte & Bit Layout:
```
+--------+--------+------------------------------------+-----------------------------+
| Byte   | Bit    | Function / Signal Name             | Bit Mask / Value Definition |
+--------+--------+------------------------------------+-----------------------------+
| Byte 0 | Bit 7  | Dark Screen Button                 | 0x80 (1 = Pressed)          |
|        | Bit 6  | Mode / Source Toggle               | 0x40 (1 = Pressed)          |
|        | Bit 5  | ESC / Back Button                  | 0x20 (1 = Pressed)          |
|        | Bit 4  | Scroll Wheel Center Click (OK)     | 0x10 (1 = Pressed)          |
|        | Bit 3  | Volume Up                          | 0x08 (1 = Pressed)          |
|        | Bit 2  | Volume Down                        | 0x04 (1 = Pressed)          |
|        | Bit 1  | Next Track / Seek+                 | 0x02 (1 = Pressed)          |
|        | Bit 0  | Previous Track / Seek-             | 0x01 (1 = Pressed)          |
+--------+--------+------------------------------------+-----------------------------+
| Byte 1 | Bit 6  | Menu Button                        | 0x40 (1 = Pressed)          |
|        | Bit 3..0| Rotary Scroll Encoder Value       | 0x00..0x0F (4-bit counter)  |
+--------+--------+------------------------------------+-----------------------------+
| Byte 2 | Bit 1  | Telephone Hangup / End Call        | 0x02 (1 = Pressed)          |
|        | Bit 0  | Telephone Answer / Call Pick-up    | 0x01 (1 = Pressed)          |
+--------+--------+------------------------------------+-----------------------------+
```

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Hiworld Base Info Frame (`Cmd 0x11` / `Handle.CarBaseInfo`)
- **Sync Header:** `0x5A 0xA5`
- **Length ($L$):** `0x08` (8 payload bytes)
- **Command ID:** `0x11` (`17` dec / `Handle.CarBaseInfo`)
- **Payload Layout (8 Bytes):**
  - `Byte 0..1`: Reserved / Status (`0x00 0x00`)
  - `Byte 2`: **Key Code** (`0x00` = No key / Release)
  - `Byte 3`: **Key State** (`0x01` = Pressed, `0x00` = Released)
  - `Byte 4..5`: Reserved (`0x00 0x00`)
  - `Byte 6`: **Steering Angle High Byte** (Signed 16-bit MSB, $0.1^\circ$ resolution)
  - `Byte 7`: **Steering Angle Low Byte** (Signed 16-bit LSB)
- **Checksum:** `((Length + CmdID + sum(Payload)) - 1) & 0xFF`
- **Total Frame Wire Length:** 13 bytes (`5A A5 08 11 [8 Bytes] Checksum`)

### Master Key Code Translation Table:
| Button / Action | PSA CAN `0x21F` Mask | Hiworld KeyID (Byte 2) | Raise KeyID | Bagoo KeyID | Android Function |
|:---|:---:|:---:|:---:|:---|
| **Volume Up** | `Byte 0: 0x08` | `0x01` | `0x14` | `0x01` | Increase master volume |
| **Volume Down** | `Byte 0: 0x04` | `0x02` | `0x15` | `0x02` | Decrease master volume |
| **Next Track (Seek+)** | `Byte 0: 0x80` | `0x03` | `0x18` | `0x03` | Media next / radio seek up |
| **Prev Track (Seek-)** | `Byte 0: 0x40` | `0x04` | `0x17` | `0x04` | Media prev / radio seek down |
| **Source / Mode Toggle**| `Byte 0: 0x02` | `0x07` | `0x11` | `0x07` | Cycle Radio $\to$ USB $\to$ BT |
| **Scroll Click (OK)** | Stalk Center / `0x3E5` | `0x09` | `0x19` | `0x09` | Enter / Select item |
| **Menu Button** | Stalk Panel / `0x3E5` | `0x0A` | `0x54` | `0x0A` | Open BSI / System menu |
| **ESC / Back** | Stalk Panel / `0x3E5` | `0x0B` | `0x60` | `0x0B` | Return / Back navigation |
| **Rotary Scroll Up** | Stalk Wheel (+1) | `0x13` | `0x42` | `0x13` | List scroll up tick |
| **Rotary Scroll Down** | Stalk Wheel (-1) | `0x14` | `0x43` | `0x14` | List scroll down tick |
| **Telephone Answer** | Stalk End Pin | `0x19` | `0x32` | `0x19` | Pick up incoming call |
| **Telephone Hangup** | Stalk End Pin (Long) | `0x15` | `0x31` | `0x15` | Terminate / Reject call |
| **Dark Screen** | MFD Panel Button | `0x38` | `0x20` | `0x38` | Toggle display backlight |

---

# 4. Pure C99 Firmware Implementation

```c
#ifndef CANBOX_STALK_H
#define CANBOX_STALK_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define HIWORLD_SOF1             0x5A
#define HIWORLD_SOF2             0xA5
#define HIWORLD_CMD_CAR_BASE     0x11

/* Supported Vendor Types */
typedef enum {
    CANBOX_VENDOR_HIWORLD = 0,
    CANBOX_VENDOR_RAISE   = 1,
    CANBOX_VENDOR_BAGOO   = 2,
    CANBOX_VENDOR_SIMPLE  = 3
} canbox_vendor_t;

/* Internal Stalk State Structure */
typedef struct {
    uint8_t prev_mask_21f;
    uint8_t prev_counter_21f;
    int16_t steering_angle;
} psa_stalk_state_t;

/* Serial Output Callback Type */
typedef void (*canbox_uart_tx_fn)(const uint8_t *buf, size_t len);

/* Stalk Engine Context */
typedef struct {
    psa_stalk_state_t state;
    canbox_vendor_t   vendor;
    canbox_uart_tx_fn tx_fn;
} psa_stalk_ctx_t;

/* Initialize Stalk Context */
static inline void psa_stalk_init(psa_stalk_ctx_t *ctx, canbox_vendor_t vendor, canbox_uart_tx_fn tx_fn) {
    memset(ctx, 0, sizeof(psa_stalk_ctx_t));
    ctx->vendor = vendor;
    ctx->tx_fn = tx_fn;
}

/* Build and transmit a serialized key pulse packet */
static inline void psa_stalk_emit_key(psa_stalk_ctx_t *ctx, uint8_t hiworld_id, uint8_t raise_id, uint8_t state) {
    if (!ctx->tx_fn) return;

    uint8_t packet[13];
    size_t len = 0;

    switch (ctx->vendor) {
        case CANBOX_VENDOR_HIWORLD: {
            /* Format: 0x5A 0xA5 [Len=0x08] [Cmd=0x11] [B0] [B1] [KeyID] [State] [B4] [B5] [AngH] [AngL] [Checksum] */
            packet[0] = HIWORLD_SOF1;
            packet[1] = HIWORLD_SOF2;
            packet[2] = 0x08;                /* Length: 8 Payload bytes */
            packet[3] = HIWORLD_CMD_CAR_BASE;/* Cmd 0x11 */
            packet[4] = 0x00;
            packet[5] = 0x00;
            packet[6] = (state != 0) ? hiworld_id : 0x00; /* Key Code */
            packet[7] = state;                             /* Key State (1=press, 0=release) */
            packet[8] = 0x00;
            packet[9] = 0x00;
            packet[10] = (uint8_t)(((uint16_t)ctx->state.steering_angle >> 8) & 0xFF);
            packet[11] = (uint8_t)((uint16_t)ctx->state.steering_angle & 0xFF);

            uint8_t sum = 0;
            for (size_t i = 2; i <= 11; i++) {
                sum += packet[i];
            }
            packet[12] = (uint8_t)((sum - 1) & 0xFF);
            len = 13;
            break;
        }

        case CANBOX_VENDOR_RAISE: {
            /* Format: 0x2E [Cmd=0x02] [Len=2] [KeyID] [State] [Checksum] */
            packet[0] = 0x2E;
            packet[1] = 0x02;
            packet[2] = 0x02;
            packet[3] = raise_id;
            packet[4] = state;
            packet[5] = (uint8_t)((packet[1] + packet[2] + packet[3] + packet[4]) ^ 0xFF);
            len = 6;
            break;
        }

        case CANBOX_VENDOR_BAGOO: {
            /* Format: 0xD5 [Cmd=0x02] [Len=2] [KeyID] [State] [Checksum] */
            packet[0] = 0xD5;
            packet[1] = 0x02;
            packet[2] = 0x02;
            packet[3] = hiworld_id;
            packet[4] = state;
            packet[5] = (uint8_t)(packet[1] + packet[2] + packet[3] + packet[4]);
            len = 6;
            break;
        }

        default:
            break;
    }

    if (len > 0) {
        ctx->tx_fn(packet, len);
    }
}

/* Process Native PSA CAN2004 Remote Stalk Frame (0x21F) */
static inline void psa_stalk_process_can_0x21F(psa_stalk_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 1) return;

    uint8_t curr = data[0];
    uint8_t prev = ctx->state.prev_mask_21f;

    #define DISPATCH_KEY(mask, hw_id, rz_id) do { \
        if (((curr) & (mask)) && !((prev) & (mask))) { \
            psa_stalk_emit_key(ctx, (hw_id), (rz_id), 1); \
        } else if (!((curr) & (mask)) && ((prev) & (mask))) { \
            psa_stalk_emit_key(ctx, (hw_id), (rz_id), 0); \
        } \
    } while(0)

    DISPATCH_KEY(0x08, 0x01, 0x14); /* Volume Up */
    DISPATCH_KEY(0x04, 0x02, 0x15); /* Volume Down */
    DISPATCH_KEY(0x02, 0x07, 0x11); /* Source */
    DISPATCH_KEY(0x80, 0x03, 0x18); /* Next / Forward */
    DISPATCH_KEY(0x40, 0x04, 0x17); /* Previous / Backward */

    #undef DISPATCH_KEY

    ctx->state.prev_mask_21f = curr;
    if (dlc >= 2) {
        ctx->state.prev_counter_21f = data[1];
    }
}

#endif /* CANBOX_STALK_H */
```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: Volume Up Press & Release on Real PSA CAN2004 (`0x21F`)
- **CAN Injection:**
  ```bash
  cansend vcan0 21F#080900   # Volume Up Pressed
  cansend vcan0 21F#000900   # Volume Up Released
  ```
- **Expected UART Output (Hiworld):**
  - Press: `5A A5 08 11 00 00 01 01 00 00 00 00 1A`
  - Release: `5A A5 08 11 00 00 00 00 00 00 00 00 18`

### Vector 2: Next Track Press & Release (`0x21F`)
- **CAN Injection:**
  ```bash
  cansend vcan0 21F#800B00   # Next Track Pressed
  cansend vcan0 21F#000B00   # Next Track Released
  ```
- **Expected UART Output (Hiworld):**
  - Press: `5A A5 08 11 00 00 03 01 00 00 00 00 1C`
  - Release: `5A A5 08 11 00 00 00 00 00 00 00 00 18`
