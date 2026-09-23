# CAN Box Protocol Specification: Doors, Trunk, Hood & Central Body Status
## Topic 07: Peugeot 407 Body Openings & Central Electronics Telemetry
**Document File:** `CANBOX_SPEC_HIWORLD_407_07_DOORS_BODY_STATUS.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** `https://github.com/fazerxlo/canbox/tree/main/doc/CAN2004_doors.md`, `CAN_messages.md`

---

# 1. Functional Domain & Architecture Overview

The Peugeot 407 BSI broadcasts body opening events and physical microswitch contacts over the Comfort CAN bus using two primary mechanisms:
- **`0x220` (Door & Body Openings, Event-Driven):** The authoritative 2-byte frame carrying individual microswitch contact states for all 4 doors, boot/tailgate, bonnet/hood, estate tailgate rear window, and fuel flap.
- **`0x221` (Generic CAN Box Alias):** Many commercial CAN box firmware models listen for `0x220` and re-map to internal `0x221` representations.

The CAN adapter decodes these discrete opening flags and dispatches Hiworld (`Cmd 0x12`) / Raise (`Cmd 0x38`) door status packets to the Android headunit, which renders an animated 3D vehicle opening popup overlay.

```
+------------------------------------------------------------------------------------+
|                         Peugeot 407 BSI (Body Control Module)                      |
|           [FL Door, FR Door, RL Door, RR Door, Trunk, Hood, Rear Window, Fuel]     |
+------------------------------------------------------------------------------------+
                                          │
                                [PSA CAN 0x220 / 0x221]
                                          ▼
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Filters & extracts discrete door opening bitmasks from CAN ID 0x220           |
|   2. Decodes rear window (SW estate) and fuel filler flap states                   |
|   3. Serializes Hiworld 0x12 Door Status Packets                                   |
+------------------------------------------------------------------------------------+
                                          │
                    [UART Serial: 38400 baud, 8N1 / Hiworld Protocol]
                                          ▼
+------------------------------------------------------------------------------------+
|                     Android Headunit Door Popup Application                        |
|            (Pops up graphic vehicle model displaying opened doors/hood/trunk)      |
+------------------------------------------------------------------------------------+
```

---

# 2. PSA CAN Bus Bitfield Specification

### 2.1 Authoritative PSA CAN2004 Door Opening Frame (`0x220`)
- **CAN ID:** `0x220` (Standard 11-bit Identifier)
- **DLC:** 2 bytes
- **Transmission Cycle:** Event-driven on state transition

```
+--------+--------+------------------------------------+-----------------------------+
| Byte   | Bit    | Function / Signal Name             | Bit Mask / Value Definition |
+--------+--------+------------------------------------+-----------------------------+
| Byte 0 | Bit 7  | Front Left Door (Driver)           | 0x80 (1 = Open, 0 = Closed) |
|        | Bit 6  | Front Right Door (Passenger)       | 0x40 (1 = Open, 0 = Closed) |
|        | Bit 5  | Rear Left Door                     | 0x20 (1 = Open, 0 = Closed) |
|        | Bit 4  | Rear Right Door                    | 0x10 (1 = Open, 0 = Closed) |
|        | Bit 3  | Trunk / Boot Lid                   | 0x08 (1 = Open, 0 = Closed) |
|        | Bit 2  | Engine Hood / Bonnet               | 0x04 (1 = Open, 0 = Closed) |
|        | Bit 1  | Rear Tailgate Window (SW Estate)   | 0x02 (1 = Open, 0 = Closed) |
|        | Bit 0  | Fuel Filler Flap                   | 0x01 (1 = Open, 0 = Closed) |
+--------+--------+------------------------------------+-----------------------------+
| Byte 1 | Bit 7  | Auto Rear Wiper in Reverse Active  | 0x80 (1 = Enabled)          |
|        | Bit 4  | Auto Central Locking on Drive      | 0x10 (1 = Active)           |
|        | Bit 3  | Parking Radar System Enabled       | 0x08 (1 = Active)           |
|        | Bit 0  | Handbrake Physical Switch          | 0x01 (1 = Pulled / Active)  |
+--------+--------+------------------------------------+-----------------------------+
```

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Hiworld Door & Window Frame (`Cmd 0x12` / `Handle.DoorWindowState`)
- **Sync Header:** `0x5A 0xA5`
- **Length ($L$):** `0x03` (3 payload bytes)
- **Command ID:** `0x12` (`18` decimal / `Handle.DoorWindowState`)
- **Payload Layout (3 Bytes):**
  - `Byte 0..1`: Reserved (`0x00 0x00`)
  - `Byte 2`:
    - `Bit 7`: Driver Front Door ($1 = \text{Open}$)
    - `Bit 6`: Passenger Front Door ($1 = \text{Open}$)
    - `Bit 5`: Rear Left Door ($1 = \text{Open}$)
    - `Bit 4`: Rear Right Door ($1 = \text{Open}$)
    - `Bit 3`: Trunk / Boot ($1 = \text{Open}$)
    - `Bit 2`: Engine Hood / Bonnet ($1 = \text{Open}$)
- **Checksum:** `((Length + CmdID + sum(Payload)) - 1) & 0xFF`
- **Total Frame Wire Length:** 8 bytes (`5A A5 03 12 [3 Bytes] Checksum`)

---

# 4. Pure C99 Firmware Implementation

```c
#ifndef CANBOX_DOORS_H
#define CANBOX_DOORS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define HIWORLD_SOF1        0x5A
#define HIWORLD_SOF2        0xA5
#define HIWORLD_CMD_DOOR    0x12

typedef struct {
    bool door_front_left;
    bool door_front_right;
    bool door_rear_left;
    bool door_rear_right;
    bool trunk_open;
    bool hood_open;
    bool handbrake_pulled;
} psa_doors_state_t;

typedef void (*canbox_uart_tx_fn)(const uint8_t *buf, size_t len);

typedef struct {
    psa_doors_state_t state;
    canbox_uart_tx_fn uart_tx;
} psa_doors_ctx_t;

static inline void psa_doors_init(psa_doors_ctx_t *ctx, canbox_uart_tx_fn uart_tx) {
    memset(ctx, 0, sizeof(psa_doors_ctx_t));
    ctx->uart_tx = uart_tx;
}

/* Transmit Hiworld Door State Frame (Cmd 0x12) */
static inline void psa_doors_send_hiworld(psa_doors_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[8];
    p[0] = HIWORLD_SOF1;
    p[1] = HIWORLD_SOF2;
    p[2] = 0x03;             /* Length: 3 Payload bytes */
    p[3] = HIWORLD_CMD_DOOR; /* Cmd 0x12 */
    p[4] = 0x00;
    p[5] = 0x00;

    uint8_t b2 = 0;
    if (ctx->state.door_front_left)  b2 |= 0x80;
    if (ctx->state.door_front_right) b2 |= 0x40;
    if (ctx->state.door_rear_left)   b2 |= 0x20;
    if (ctx->state.door_rear_right)  b2 |= 0x10;
    if (ctx->state.trunk_open)       b2 |= 0x08;
    if (ctx->state.hood_open)        b2 |= 0x04;
    p[6] = b2;

    uint8_t sum = 0;
    for (size_t i = 2; i <= 6; i++) {
        sum += p[i];
    }
    p[7] = (uint8_t)((sum - 1) & 0xFF);

    ctx->uart_tx(p, 8);
}

/* Process PSA CAN 0x220 Frame */
static inline void psa_doors_process_can_0x220(psa_doors_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 1) return;

    uint8_t b0 = data[0];
    ctx->state.door_front_left  = (b0 & 0x80) ? true : false;
    ctx->state.door_front_right = (b0 & 0x40) ? true : false;
    ctx->state.door_rear_left   = (b0 & 0x20) ? true : false;
    ctx->state.door_rear_right  = (b0 & 0x10) ? true : false;
    ctx->state.trunk_open       = (b0 & 0x08) ? true : false;
    ctx->state.hood_open        = (b0 & 0x04) ? true : false;

    if (dlc >= 2) {
        ctx->state.handbrake_pulled = (data[1] & 0x01) ? true : false;
    }

    psa_doors_send_hiworld(ctx);
}

#endif /* CANBOX_DOORS_H */
```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: Driver Front Door Open
- **CAN ID `0x220` Injection:**
  ```bash
  cansend vcan0 220#8000
  ```
- **Expected UART Output (Hiworld `0x12`):**
  - Frame: `5A A5 03 12 00 00 80 94`
  - Checksum Calculation: `(0x03 + 0x12 + 0x00 + 0x00 + 0x80 - 1) & 0xFF = 0x94`
