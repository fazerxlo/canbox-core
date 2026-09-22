# CAN Box Protocol Specification: Doors, Trunk, Hood & Central Body Status
## Topic 07: Peugeot 407 Body Openings & Central Electronics Telemetry
**Document File:** `CANBOX_SPEC_HIWORLD_407_07_DOORS_BODY_STATUS.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** `https://github.com/fazerxlo/canbox/tree/main/doc/CAN2004_doors.md`, `CAN_messages.md`

---

# 1. Functional Domain & Architecture Overview

The Peugeot 407 Built-in Systems Interface (BSI) monitors the state of all door microswitches, the boot/tailgate release latch, the engine bonnet contact, and central locking actuators. The BSI broadcasts this central body state periodically (and immediately upon any door opening or closing) across the PSA Comfort CAN bus on CAN ID `0x221`.
The Peugeot 407 BSI broadcasts body opening events and physical microswitch contacts over the Comfort CAN bus using two primary mechanisms:
- **`0x220` (Door & Body Openings, Event-Driven):** The authoritative, real-time 2-byte frame carrying individual microswitch contact states for all 4 doors, boot/tailgate, bonnet/hood, estate tailgate rear window, and fuel flap.
- **`0x1A1` (BSI Display Popup / Warning):** An 8-byte frame driving cluster and MFD warning popups.
- **`0x221` (Generic CAN Box Alias):** Many commercial CAN box firmware models listen for `0x220` and re-map to internal `0x221` representations.

The CAN adapter decodes these discrete opening flags and dispatches Hiworld / Raise door status packets to the Android headunit, which renders an animated 3D vehicle opening popup overlay.

```
+------------------------------------------------------------------------------------+
|                         Peugeot 407 BSI (Body Control Module)                      |
|           [FL Door, FR Door, RL Door, RR Door, Trunk, Hood, Handbrake, Locks]      |
|           [FL Door, FR Door, RL Door, RR Door, Trunk, Hood, Rear Window, Fuel]     |
+------------------------------------------------------------------------------------+
                                          │
                                   [PSA CAN 0x221]
                               [PSA CAN 0x220 / 0x221]
                                          ▼
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Filters & extracts discrete door opening bitmasks from CAN ID 0x221           |
|   2. Interlocks Handbrake, Lighting and Central Locking states                     |
|   1. Filters & extracts discrete door opening bitmasks from CAN ID 0x220           |
|   2. Decodes rear window (SW estate) and fuel filler flap states                   |
|   3. Serializes Hiworld 0x24 / 0x38 Door Status Packets                            |
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

### 2.1 PSA Body Openings & Status Frame (`0x221`)
- **CAN ID:** `0x221` (Standard 11-bit Identifier)
- **DLC:** 8 bytes
- **Transmission Cycle:** Periodic 100 ms or immediate on microswitch edge trigger
### 2.1 Authoritative PSA CAN2004 Door Opening Frame (`0x220`)
- **CAN ID:** `0x220` (Standard 11-bit Identifier)
- **DLC:** 2 bytes
- **Transmission Cycle:** Event-driven (sent immediately upon any door or latch opening/closing)

```
+--------+--------+------------------------------------+-----------------------------+
| Byte   | Bit    | Function / Signal Name             | Bit Mask / Value Definition |
+--------+--------+------------------------------------+-----------------------------+
| Byte 0 | Bit 7  | Driver Front Door (Left)           | 0x80 (1 = Open, 0 = Closed) |
|        | Bit 6  | Passenger Front Door (Right)       | 0x40 (1 = Open, 0 = Closed) |
| Byte 0 | Bit 7  | Front Left Door (Driver)           | 0x80 (1 = Open, 0 = Closed) |
|        | Bit 6  | Front Right Door (Passenger)       | 0x40 (1 = Open, 0 = Closed) |
|        | Bit 5  | Rear Left Door                     | 0x20 (1 = Open, 0 = Closed) |
|        | Bit 4  | Rear Right Door                    | 0x10 (1 = Open, 0 = Closed) |
|        | Bit 3  | Boot / Trunk / Tailgate            | 0x08 (1 = Open, 0 = Closed) |
|        | Bit 2  | Engine Bonnet / Hood               | 0x04 (1 = Open, 0 = Closed) |
|        | Bit 1  | Sunroof / Panorama Shade State     | 0x02 (1 = Open)             |
|        | Bit 0  | Fuel Flap Open Flag                | 0x01 (1 = Open)             |
|        | Bit 3  | Trunk / Boot Lid                   | 0x08 (1 = Open, 0 = Closed) |
|        | Bit 2  | Engine Hood / Bonnet               | 0x04 (1 = Open, 0 = Closed) |
|        | Bit 1  | Rear Tailgate Window (SW Estate)   | 0x02 (1 = Open, 0 = Closed) |
|        | Bit 0  | Fuel Filler Flap                   | 0x01 (1 = Open, 0 = Closed) |
+--------+--------+------------------------------------+-----------------------------+
| Byte 1 | Bit 7  | Auto Rear Wiper in Reverse Active  | 0x80 (1 = Enabled)          |
|        | Bit 4  | Auto Central Locking on Drive      | 0x10 (1 = Active)           |
|        | Bit 3  | Parking Radar System Enabled       | 0x08 (1 = Active)           |
|        | Bit 0  | Handbrake Physical Switch          | 0x01 (1 = Pulled / Active)  |
| Byte 1 | Bit 7  | Vehicle Type (CAR_TYPE)            | 0 = 5-door saloon, 1 = 3-dr |
|        | Bit 6  | Spare Wheel Carrier Arm Status     | 1 = Open                    |
|        | Bits 5..0| Reserved / Unused                | 0x00                        |
+--------+--------+------------------------------------+-----------------------------+
| Byte 2 | Bit 7  | Daytime Running Lights (DRL)       | 0x80 (1 = Enabled)          |
|        | Bit 0  | Cornering / Directional Lights     | 0x01 (1 = Enabled)          |
+--------+--------+------------------------------------+-----------------------------+
```

### Observed Bit Patterns on Real Peugeot 407:
- All closed: `00 00`
- Front left (driver) open: `80 00`
- Front right open: `40 00`
- Trunk / boot open: `08 00`
- Bonnet open: `04 00`
- Tailgate glass (SW) open: `02 00`
- All 4 doors + trunk open: `F8 00`

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Hiworld Doors & Body Frame (`Cmd 0x24` / `0x38`)
- **Sync Header:** `0x5A 0xA5`
- **Length:** `0x03` (1-byte Cmd + 2-byte Payload) or `0x09` (Extended BSI)
- **Command ID:** `0x24` (Compact Doors) / `0x38` (Extended Body)
- **Compact Payload Layout (`Cmd 0x24`, 2 Bytes):**
  - `Byte 0`:
    - `Bit 7`: Driver Front Door ($1 = \text{Open}$)
    - `Bit 6`: Passenger Front Door ($1 = \text{Open}$)
    - `Bit 5`: Rear Left Door ($1 = \text{Open}$)
    - `Bit 4`: Rear Right Door ($1 = \text{Open}$)
    - `Bit 3`: Trunk / Boot ($1 = \text{Open}$)
    - `Bit 2`: Engine Hood / Bonnet ($1 = \text{Open}$)
  - `Byte 1`: Handbrake ($1 = \text{Active}$) & Lock status.
### 3.1 Hiworld Doors & Body Frame (`Cmd 0x24`)
- **Sync Header:** `0x5A 0xA5` | **Length:** `0x03` | **Cmd:** `0x24`
- **Payload Layout (2 Bytes):**
  - `Byte 0`: `[FL:b7] [FR:b6] [RL:b5] [RR:b4] [Trunk:b3] [Hood:b2] [0:b1..b0]`
  - `Byte 1`: Handbrake / Lock state (`0x01` = Handbrake pulled)
- **Checksum:** 8-bit sum modulo 256 over `Length + CmdID + Payload`.

### 3.2 Raise Doors & Extended Body Compatibility (`Cmd 0x38`):
- **Header:** `0x2E 0x38 0x08 [B0] [B1] [B2] [B3] [B4] [B5] [B6] [B7] [Checksum]`
- Bitwise inverted sum `(Sum ^ 0xFF) & 0xFF`.

---

# 4. Pure C99 Firmware Implementation

```c
#ifndef CANBOX_DOORS_H
#define CANBOX_DOORS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

typedef struct {
    bool door_front_left;
    bool door_front_right;
    bool door_rear_left;
    bool door_rear_right;
    bool trunk_open;
    bool hood_open;
    bool rear_window_open;
    bool fuel_flap_open;
    bool handbrake_pulled;
    bool auto_lock_drive;
    bool drl_active;
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

/* Transmit Hiworld Doors Packet (Cmd 0x24) */
static inline void psa_doors_send_hiworld(psa_doors_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[7];
    p[0] = 0x5A;
    p[1] = 0xA5;
    p[2] = 0x03; /* Length: 1 Cmd + 2 Payload */
    p[3] = 0x24; /* Cmd ID */

    /* Byte 0: Openings */
    uint8_t b0 = 0;
    if (ctx->state.door_front_left)  b0 |= 0x80;
    if (ctx->state.door_front_right) b0 |= 0x40;
    if (ctx->state.door_rear_left)   b0 |= 0x20;
    if (ctx->state.door_rear_right)  b0 |= 0x10;
    if (ctx->state.trunk_open)       b0 |= 0x08;
    if (ctx->state.trunk_open || ctx->state.rear_window_open) b0 |= 0x08;
    if (ctx->state.hood_open)        b0 |= 0x04;
    p[4] = b0;

    /* Byte 1: Handbrake & System Flags */
    uint8_t b1 = 0;
    if (ctx->state.handbrake_pulled) b1 |= 0x01;
    p[5] = b1;

    uint8_t sum = 0;
    for (size_t i = 2; i <= 5; i++) sum += p[i];
    p[6] = sum;

    ctx->uart_tx(p, 7);
}

/* Process PSA CAN ID 0x221 Frame */
static inline void psa_doors_process_can_0x221(psa_doors_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 2) return;
/* Process Native PSA CAN2004 0x220 Frame */
static inline void psa_doors_process_can_0x220(psa_doors_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 1) return;

    uint8_t b0 = data[0];
    uint8_t b1 = data[1];

    ctx->state.door_front_left  = (b0 & 0x80) ? true : false;
    ctx->state.door_front_right = (b0 & 0x40) ? true : false;
    ctx->state.door_rear_left   = (b0 & 0x20) ? true : false;
    ctx->state.door_rear_right  = (b0 & 0x10) ? true : false;
    ctx->state.trunk_open       = (b0 & 0x08) ? true : false;
    ctx->state.hood_open        = (b0 & 0x04) ? true : false;
    ctx->state.rear_window_open = (b0 & 0x02) ? true : false;
    ctx->state.fuel_flap_open   = (b0 & 0x01) ? true : false;

    ctx->state.handbrake_pulled = (b1 & 0x01) ? true : false;
    ctx->state.auto_lock_drive  = (b1 & 0x10) ? true : false;

    if (dlc >= 3) {
        ctx->state.drl_active = (data[2] & 0x80) ? true : false;
    }

    psa_doors_send_hiworld(ctx);
}

#endif /* CANBOX_DOORS_H */
```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: Driver Door Open + Handbrake Pulled
- **CAN ID `0x221` Injection:**
### Vector 1: Front Left Door Open on PSA CAN2004 (`0x220`)
- **CAN ID `0x220` Injection:**
  ```bash
  cansend vcan0 221#8001000000000000
  cansend vcan0 220#8000
  ```
- **Expected UART Output (Hiworld `0x24`):**
  - Frame: `5A A5 03 24 80 01 A8`
  - Frame: `5A A5 03 24 80 00 A7`

### Vector 2: Trunk / Tailgate Open
- **CAN ID `0x221` Injection:**
### Vector 2: Trunk Lid Open (`0x220`)
- **CAN ID `0x220` Injection:**
  ```bash
  cansend vcan0 221#0800000000000000
  cansend vcan0 220#0800
  ```
- **Expected UART Output (Hiworld `0x24`):**
  - Frame: `5A A5 03 24 08 00 2F`

