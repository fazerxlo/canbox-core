# CAN Box Protocol Specification: Steering Column Stalk & Button Controls
## Topic 01: Peugeot 407 Multifunction Stalk & Rotary Encoder
**Document File:** `CANBOX_SPEC_HIWORLD_407_01_STEERING_STALK_KEYS.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** `https://github.com/fazerxlo/canbox/tree/main/doc/CAN2004_sterringkeys.md`, `CAN2004_autowp_comparison.md`, `CAN2004_radio.md`

---

# 1. Functional Domain & Architecture Overview

The Peugeot 407 steering column multifunction stalk communicates button events, rotary wheel state changes, and telephony commands to the central BSI (Built-in Systems Interface) via the PSA Comfort CAN bus at 125 kbps. 
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
                                   [PSA CAN 0x0F6]
                               [PSA CAN 0x21F / 0x3E5]
                                          ▼
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Filters & debounces CAN ID 0x0F6 (50 ms cycle or event-triggered)             |
|   2. Tracks rotary encoder delta (4-bit signed counter)                            |
|   3. Translates state transitions into Headunit Key Pulse Packets                  |
|   1. Captures native 3-byte CAN ID 0x21F remote frames                             |
|   2. Debounces key press/release transitions (D1 mask, D2 auxiliary counter)       |
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

- **CAN ID:** `0x0F6` (Standard 11-bit Identifier)
- **DLC:** 8 bytes (Bytes 0–2 active, Bytes 3–7 reserved/padding)
- **Transmission Frequency:** Periodic 50 ms or immediate on-event state transition
### 2.1 Native PSA CAN2004 Steering Remote Frame (`0x21F`)
- **CAN ID:** `0x21F` (Standard 11-bit Identifier)
- **DLC:** 3 bytes
- **Frame Format:** `[Byte 0: Command Mask] [Byte 1: Aux / Pulse Counter] [Byte 2: 0x00]`
- **Transmission Cycle:** Event-driven upon button press/release

### CAN ID `0x0F6` Byte & Bit Layout:
```
+--------+--------+------------------------------------+-----------------------------+
| Byte   | Bit    | Function / Signal Name             | Bit Mask / Value Definition |
+--------+--------+------------------------------------+-----------------------------+
| Byte 0 | Bit 7  | Dark Screen Button                 | 0x80 (1 = Pressed)          |
|        | Bit 6  | Mode / Source Toggle               | 0x40 (1 = Pressed)          |
|        | Bit 5  | ESC / Back Button                  | 0x20 (1 = Pressed)          |
|        | Bit 4  | Scroll Wheel Center Click (OK)     | 0x10 (1 = Pressed)          |
| Byte 0 | Bit 7  | Next Track / Seek+ (Forward)       | 0x80 (1 = Pressed)          |
|        | Bit 6  | Previous Track / Seek- (Backward)  | 0x40 (1 = Pressed)          |
|        | Bit 3  | Volume Up                          | 0x08 (1 = Pressed)          |
|        | Bit 2  | Volume Down                        | 0x04 (1 = Pressed)          |
|        | Bit 1  | Next Track / Seek+                 | 0x02 (1 = Pressed)          |
|        | Bit 0  | Previous Track / Seek-             | 0x01 (1 = Pressed)          |
|        | Bit 1  | Source / Mode Toggle               | 0x02 (1 = Pressed)          |
|        | Bit 0  | Idle / Release State               | 0x00                        |
+--------+--------+------------------------------------+-----------------------------+
| Byte 1 | Bit 6  | Menu Button                        | 0x40 (1 = Pressed)          |
|        | Bit 3..0| Rotary Scroll Encoder Value       | 0x00..0x0F (4-bit counter)  |
| Byte 1 | 7..0   | Secondary / Scroll Pulse Counter   | Value stays ~0x09/0x0B      |
+--------+--------+------------------------------------+-----------------------------+
| Byte 2 | Bit 1  | Telephone Hangup / End Call        | 0x02 (1 = Pressed)          |
|        | Bit 0  | Telephone Answer / Call Pick-up    | 0x01 (1 = Pressed)          |
| Byte 2 | 7..0   | Reserved Padding                   | Always 0x00                 |
+--------+--------+------------------------------------+-----------------------------+
```

### Rotary Encoder Processing:
The rotary scroll wheel in Byte 1 (Bits 3..0) increments when rotated upward and decrements when rotated downward (modulo 16).
$$\Delta = (\text{Current}_{\text{Bits 3..0}} - \text{Previous}_{\text{Bits 3..0}}) \pmod{16}$$
- If $\Delta \in [1, 7]$, scroll wheel rotated **Up** by $\Delta$ steps.
- If $\Delta \in [8, 15]$ (equivalent to $-8 \dots -1$), scroll wheel rotated **Down** by $(16 - \Delta)$ steps.
### 2.2 Pulse Sequence Pattern (Observed from Real Vehicle Traces)
- **Press:** `[Mask, Counter, 00]` (e.g. `08 09 00` = Volume Up pressed)
- **Release:** `[00, Counter, 00]` (e.g. `00 09 00` = Volume Up released)

---

# 3. Headunit Serial Protocol Mappings

### Hiworld Protocol Packet Format (`Cmd 0x11`):
- **Sync Header:** `0x5A 0xA5`
- **Length:** `0x03` (1-byte Cmd + 2-byte Payload)
- **Command ID:** `0x11`
- **Payload:** `[KeyID]` `[State: 0x01 = Press, 0x00 = Release]`
- **Checksum:** 8-bit sum modulo 256 over `Length + CmdID + Payload`.

### Master Key Code Translation Table:
| Button / Action | PSA CAN `0x0F6` Mask | Hiworld KeyID | Raise KeyID | Bagoo KeyID | SimpleSoft KeyID | Android Function |
| Button / Action | PSA CAN `0x21F` Mask | Hiworld KeyID | Raise KeyID | Bagoo KeyID | SimpleSoft KeyID | Android Function |
|:---|:---:|:---:|:---:|:---:|:---|
| **Volume Up** | `Byte 0: 0x08` | `0x01` | `0x14` | `0x01` | `0x14` | Increase master volume |
| **Volume Down** | `Byte 0: 0x04` | `0x02` | `0x15` | `0x02` | `0x15` | Decrease master volume |
| **Next Track (Seek+)** | `Byte 0: 0x02` | `0x03` | `0x18` | `0x03` | `0x18` | Media next / radio seek up |
| **Prev Track (Seek-)** | `Byte 0: 0x01` | `0x04` | `0x17` | `0x04` | `0x17` | Media prev / radio seek down |
| **Source / Mode Toggle**| `Byte 0: 0x40` | `0x07` | `0x11` | `0x07` | `0x11` | Cycle Radio $\to$ USB $\to$ BT |
| **Scroll Click (OK)** | `Byte 0: 0x10` | `0x09` | `0x19` | `0x09` | `0x19` | Enter / Select item |
| **Menu Button** | `Byte 1: 0x40` | `0x0A` | `0x54` | `0x0A` | `0x54` | Open BSI / System menu |
| **ESC / Back** | `Byte 0: 0x20` | `0x0B` | `0x60` | `0x0B` | `0x60` | Return / Back navigation |
| **Rotary Scroll Up** | `Byte 1: +1` | `0x13` | `0x42` | `0x13` | `0x42` | List scroll up tick |
| **Rotary Scroll Down** | `Byte 1: -1` | `0x14` | `0x43` | `0x14` | `0x43` | List scroll down tick |
| **Telephone Answer** | `Byte 2: 0x01` | `0x19` | `0x32` | `0x19` | `0x32` | Pick up incoming call |
| **Telephone Hangup** | `Byte 2: 0x02` | `0x15` | `0x31` | `0x15` | `0x31` | Terminate / Reject call |
| **Dark Screen** | `Byte 0: 0x80` | `0x38` | `0x20` | `0x38` | `0x20` | Toggle display backlight |
| **Next Track (Seek+)** | `Byte 0: 0x80` | `0x03` | `0x18` | `0x03` | `0x18` | Media next / radio seek up |
| **Prev Track (Seek-)** | `Byte 0: 0x40` | `0x04` | `0x17` | `0x04` | `0x17` | Media prev / radio seek down |
| **Source / Mode Toggle**| `Byte 0: 0x02` | `0x07` | `0x11` | `0x07` | `0x11` | Cycle Radio $\to$ USB $\to$ BT |
| **Scroll Click (OK)** | Stalk Center / `0x3E5` | `0x09` | `0x19` | `0x09` | `0x19` | Enter / Select item |
| **Menu Button** | Stalk Panel / `0x3E5` | `0x0A` | `0x54` | `0x0A` | `0x54` | Open BSI / System menu |
| **ESC / Back** | Stalk Panel / `0x3E5` | `0x0B` | `0x60` | `0x0B` | `0x60` | Return / Back navigation |
| **Rotary Scroll Up** | Stalk Wheel (+1) | `0x13` | `0x42` | `0x13` | `0x42` | List scroll up tick |
| **Rotary Scroll Down** | Stalk Wheel (-1) | `0x14` | `0x43` | `0x14` | `0x43` | List scroll down tick |
| **Telephone Answer** | Stalk End Pin | `0x19` | `0x32` | `0x19` | `0x32` | Pick up incoming call |
| **Telephone Hangup** | Stalk End Pin (Long) | `0x15` | `0x31` | `0x15` | `0x31` | Terminate / Reject call |
| **Dark Screen** | MFD Panel Button | `0x38` | `0x20` | `0x38` | `0x20` | Toggle display backlight |

---

# 4. Pure C99 Firmware Implementation

```c
#ifndef CANBOX_STALK_H
#define CANBOX_STALK_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* Supported Vendor Types */
typedef enum {
    CANBOX_VENDOR_HIWORLD = 0,
    CANBOX_VENDOR_RAISE   = 1,
    CANBOX_VENDOR_BAGOO   = 2,
    CANBOX_VENDOR_SIMPLE  = 3
} canbox_vendor_t;

/* Internal Stalk State Structure */
typedef struct {
    uint8_t prev_b0;
    uint8_t prev_b1;
    uint8_t prev_b2;
    uint8_t prev_rotary;
    uint8_t prev_mask_21f;
    uint8_t prev_counter_21f;
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

    uint8_t packet[8];
    size_t len = 0;

    switch (ctx->vendor) {
        case CANBOX_VENDOR_HIWORLD:
            /* Format: 0x5A 0xA5 [Len=3] [Cmd=0x11] [KeyID] [State] [Checksum] */
            packet[0] = 0x5A;
            packet[1] = 0xA5;
            packet[2] = 0x03;
            packet[3] = 0x11;
            packet[4] = hiworld_id;
            packet[5] = state;
            packet[6] = (uint8_t)(packet[2] + packet[3] + packet[4] + packet[5]);
            len = 7;
            break;

        case CANBOX_VENDOR_RAISE:
            /* Format: 0x2E [Cmd=0x02] [Len=2] [KeyID] [State] [Checksum] */
            packet[0] = 0x2E;
            packet[1] = 0x02;
            packet[2] = 0x02;
            packet[3] = raise_id;
            packet[4] = state;
            packet[5] = (uint8_t)((packet[1] + packet[2] + packet[3] + packet[4]) ^ 0xFF);
            len = 6;
            break;

        case CANBOX_VENDOR_BAGOO:
            /* Format: 0xD5 [Cmd=0x02] [Len=2] [KeyID] [State] [Checksum] */
            packet[0] = 0xD5;
            packet[1] = 0x02;
            packet[2] = 0x02;
            packet[3] = hiworld_id;
            packet[4] = state;
            packet[5] = (uint8_t)(packet[1] + packet[2] + packet[3] + packet[4]);
            len = 6;
            break;

        case CANBOX_VENDOR_SIMPLE:
            /* Format: 0x2E [Cmd=0x02] [Len=2] [KeyID] [State] [Checksum] */
            packet[0] = 0x2E;
            packet[1] = 0x02;
            packet[2] = 0x02;
            packet[3] = raise_id;
            packet[4] = state;
            packet[5] = (uint8_t)(packet[1] + packet[2] + packet[3] + packet[4]);
            len = 6;
            break;
    }

    ctx->tx_fn(packet, len);
}

/* Process Inbound PSA CAN ID 0x0F6 Frame */
static inline void psa_stalk_process_can(psa_stalk_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 2) return;
/* Process Native PSA CAN2004 Remote Stalk Frame (0x21F) */
static inline void psa_stalk_process_can_0x21F(psa_stalk_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 1) return;

    uint8_t b0 = data[0];
    uint8_t b1 = data[1];
    uint8_t b2 = (dlc >= 3) ? data[2] : 0;
    uint8_t curr = data[0];
    uint8_t prev = ctx->state.prev_mask_21f;

    #define DISPATCH_KEY(curr, prev, mask, hw_id, rz_id) do { \
    #define DISPATCH_KEY(mask, hw_id, rz_id) do { \
        if (((curr) & (mask)) && !((prev) & (mask))) { \
            psa_stalk_emit_key(ctx, (hw_id), (rz_id), 1); \
        } else if (!((curr) & (mask)) && ((prev) & (mask))) { \
            psa_stalk_emit_key(ctx, (hw_id), (rz_id), 0); \
        } \
    } while(0)

    /* Byte 0 Buttons */
    DISPATCH_KEY(b0, ctx->state.prev_b0, 0x08, 0x01, 0x14); /* Volume Up */
    DISPATCH_KEY(b0, ctx->state.prev_b0, 0x04, 0x02, 0x15); /* Volume Down */
    DISPATCH_KEY(b0, ctx->state.prev_b0, 0x02, 0x03, 0x18); /* Next Track */
    DISPATCH_KEY(b0, ctx->state.prev_b0, 0x01, 0x04, 0x17); /* Prev Track */
    DISPATCH_KEY(b0, ctx->state.prev_b0, 0x40, 0x07, 0x11); /* Source / Mode */
    DISPATCH_KEY(b0, ctx->state.prev_b0, 0x10, 0x09, 0x19); /* OK / Confirm */
    DISPATCH_KEY(b0, ctx->state.prev_b0, 0x20, 0x0B, 0x60); /* ESC / Back */
    DISPATCH_KEY(b0, ctx->state.prev_b0, 0x80, 0x38, 0x20); /* Dark Screen */
    DISPATCH_KEY(0x08, 0x01, 0x14); /* Volume Up */
    DISPATCH_KEY(0x04, 0x02, 0x15); /* Volume Down */
    DISPATCH_KEY(0x02, 0x07, 0x11); /* Source */
    DISPATCH_KEY(0x80, 0x03, 0x18); /* Next / Forward */
    DISPATCH_KEY(0x40, 0x04, 0x17); /* Previous / Backward */

    /* Byte 1 Buttons */
    DISPATCH_KEY(b1, ctx->state.prev_b1, 0x40, 0x0A, 0x54); /* Menu */
    #undef DISPATCH_KEY

    /* Byte 2 Buttons */
    DISPATCH_KEY(b2, ctx->state.prev_b2, 0x01, 0x19, 0x32); /* Tel Answer */
    DISPATCH_KEY(b2, ctx->state.prev_b2, 0x02, 0x15, 0x31); /* Tel Hangup */

    /* Rotary Scroll Encoder (Bits 3..0) */
    uint8_t curr_rotary = b1 & 0x0F;
    uint8_t prev_rotary = ctx->state.prev_rotary;
    int8_t delta = (int8_t)((curr_rotary - prev_rotary) & 0x0F);
    
    if (delta > 0 && delta <= 7) {
        while (delta--) {
            psa_stalk_emit_key(ctx, 0x13, 0x42, 1);
            psa_stalk_emit_key(ctx, 0x13, 0x42, 0);
        }
    } else if (delta >= 8 && delta <= 15) {
        int8_t steps = 16 - delta;
        while (steps--) {
            psa_stalk_emit_key(ctx, 0x14, 0x43, 1);
            psa_stalk_emit_key(ctx, 0x14, 0x43, 0);
        }
    ctx->state.prev_mask_21f = curr;
    if (dlc >= 2) {
        ctx->state.prev_counter_21f = data[1];
    }

    ctx->state.prev_b0 = b0;
    ctx->state.prev_b1 = b1;
    ctx->state.prev_b2 = b2;
    ctx->state.prev_rotary = curr_rotary;

    #undef DISPATCH_KEY
}

#endif /* CANBOX_STALK_H */
```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: Volume Up Key Press & Release
### Vector 1: Volume Up Press & Release on Real PSA CAN2004 (`0x21F`)
- **CAN Injection:**
  ```bash
  cansend vcan0 0F6#0800000000000000  # Press
  cansend vcan0 0F6#0000000000000000  # Release
  cansend vcan0 21F#080900   # Volume Up Pressed
  cansend vcan0 21F#000900   # Volume Up Released
  ```
- **Expected UART Output (Hiworld):**
  - Press: `5A A5 03 11 01 01 16`
  - Release: `5A A5 03 11 01 00 15`

### Vector 2: Rotary Wheel Scroll Up 1 Tick
- **CAN Injection (Value rolls from 0 to 1):**
### Vector 2: Next Track Press & Release (`0x21F`)
- **CAN Injection:**
  ```bash
  cansend vcan0 0F6#0001000000000000
  cansend vcan0 21F#800B00   # Next Track Pressed
  cansend vcan0 21F#000B00   # Next Track Released
  ```
- **Expected UART Output (Hiworld Pulse):**
  - Press: `5A A5 03 11 13 01 28`
  - Release: `5A A5 03 11 13 00 27`
- **Expected UART Output (Hiworld):**
  - Press: `5A A5 03 11 03 01 18`
  - Release: `5A A5 03 11 03 00 17`

### Vector 3: Telephone Answer Key Press
### Vector 3: Source Key Press & Release (`0x21F`)
- **CAN Injection:**
  ```bash
  cansend vcan0 0F6#0000010000000000
  cansend vcan0 21F#020900   # Source Pressed
  cansend vcan0 21F#000900   # Source Released
  ```
- **Expected UART Output (Hiworld):**
  - Press: `5A A5 03 11 19 01 2E`

  - Press: `5A A5 03 11 07 01 1C`
  - Release: `5A A5 03 11 07 00 1B`
