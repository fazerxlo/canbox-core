# CAN Box Protocol Specification: Steering Column Stalk & Button Controls
## Topic 01: Peugeot 407 Multifunction Stalk, Stalk Tips & Rotary Encoder
**Document File:** `doc/CANBOX_SPEC_HIWORLD_407_01_STEERING_STALK_KEYS.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum, 10-byte CMD `0x11` payload)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** Real E2E Vehicle Dumps (`dump_2026-10-06_16-56-18.log` & `dump_2026-10-06_17-06-48.log`) with attached OEM Hiworld Canbox, cross-referenced with `doc/PEUGEOT_RT4_SWC.md`

---

# 1. Functional Domain & Architecture Overview

In Peugeot 407 (PSA CAN2004 architecture), the steering column multifunction remote stalk (COM2000 / COM2005) communicates audio buttons, track changes, source selection, and thumbwheel rotary ticks over the Comfort CAN bus at 125 kbps:

- **Native PSA CAN2004 Remote Stalk Frame (`0x21F`):** A 3-byte command frame (`x_commandes_volant` / `CDE_RADIO`) broadcast by COM2000. It conveys Volume (+/-), chorded Mute, Source, Next/Prev Seek, and the rotary thumbwheel position counter.
- **Native PSA CAN2004 Stalk Tips Frame (`0x221`):** A 7-byte frame (`INFOS_GEN_ODB`) transmitting push-button presses on the stalk tips: Trip Computer (ODB) on Byte 0 Bit 3 (`0x08`), toggling `0xC0` (idle) to `0xC8` (pressed).
- **Native PSA CAN2004 RD4 Console Frame (`0x3E5`):** A 6-byte frame carrying RD4 fascia keys (AUDIO, TRIP, CLIM, DARK, MENU, OK, ESC, arrows) as 2-bit fields; see section 3.3.
- **Generic CAN Box Alias (`0x0F6`):** An aftermarket normalized frame definition used in legacy Chinese CAN boxes (e.g. Raise, Bagoo).

```
+------------------------------------------------------------------------------------+
|                         Peugeot 407 Steering Column Stalk                          |
|             [Vol+, Vol-, Mute, Seek+, Seek-, Source, Voice PTT, Trip, Scroll]      |
+------------------------------------------------------------------------------------+
                                          │
                  [PSA CAN 0x21F (Stalk) / 0x221 (Tips) / 0x3E5 (RD4 Console)]
                                          ▼
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Captures native 3-byte CAN ID 0x21F stalk frames                              |
|   2. Decodes Vol+, Vol-, Source, Seek+, Seek- from Byte 0                          |
|   3. Filters mechanical contact skew to cleanly decode chorded MUTE (0x0C)         |
|   4. Tracks Byte 1 rotary thumbwheel counter increments/decrements                 |
|   5. Captures Byte 0 Bit 3 trip button presses from CAN ID 0x221                   |
|   6. Captures RD4 console keys from CAN ID 0x3E5 only                   |
|   7. Translates actions to Hiworld OEM format (0x5AA5 / Cmd 0x11 & Cmd 0x21)       |
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
- **CAN ID:** `0x21F` (543 dec, `x_commandes_volant` / `CDE_RADIO`)
- **DLC:** 3 bytes
- **Source ECU:** COM2000 / COM2005 (Steering column switch module)
- **Transmission Cycle:** Event-driven on state transition; periodically repeated (~50–100 ms) while a button is held down. When released, an idle frame (`00 <counter> 00`) is transmitted.

#### Byte Layout
```text
+-------------------+-------------------+-------------------+
|      Byte 0       |      Byte 1       |      Byte 2       |
|  [All Key Flags]  |  [Scroll Counter] |   [Padding 0x00]  |
+-------------------+-------------------+-------------------+
```

#### Byte 0: Primary Button Bitmask
Proven by real vehicle captures (`dump_2026-10-06_16-56-18.log` & `dump_2026-10-06_17-06-48.log`):

| Bit | Hex Mask | Function / Signal Name | Verified Wire Value | Description |
| :---: | :---: | :--- | :---: | :--- |
| **Bit 7** | `0x80` | **Next Track / Seek Up** | `21F#800100` | Pressed when stalk pulled forward / seek up |
| **Bit 6** | `0x40` | **Previous Track / Seek Down**| `21F#400100` | Pressed when stalk pulled backward / seek down |
| **Bit 3** | `0x08` | **Volume Up (`VOL +`)** | `21F#080200` | Pulled up / pressed |
| **Bit 2** | `0x04` | **Volume Down (`VOL -`)** | `21F#040200` | Pulled down / pressed |
| **Bit 3 + 2**| `0x0C` | **MUTE / Unmute** | `21F#0C0200` | Vol+ and Vol- pulled simultaneously (`0x08 \| 0x04`) |
| **Bit 1** | `0x02` | **SRC (Source / Mode)** | `21F#020200` | Stalk end button / source toggle |
| **Bit 0** | `0x01` | **Idle / Unused** | `21F#000200` | Idle state |

> **Contact Skew & Debouncing Rule for MUTE:**  
> In real vehicle logs, pulling both volume paddles simultaneously exhibits mechanical contact skew: Vol+ contact (`0x08`) typically touches ~50–80 ms before Vol- joins to produce `0x0C` (e.g. `1791299204.999`: `0x08` $\to$ `1791299205.080`: `0x0C`).  
> Similarly, during release, one paddle releases slightly earlier (e.g. `1791299205.215`: `0x04` $\to$ `1791299205.243`: `0x00`).  
> Firmware must implement a short edge debounce window (~80 ms) or contact-skew filter so that a chorded MUTE action is not preceded or succeeded by a spurious single-step volume event.

#### Byte 1: Rotary Scroll Wheel / Thumbwheel (*Molette*) Position Counter
- Transmits an incremental rolling counter indicating absolute thumbwheel angular steps:
  - Rolling **Up**: Counter increments (`0x01` $\to$ `0x02` $\to$ `0x03` $\to \dots$).
  - Rolling **Down**: Counter decrements (`0x03` $\to$ `0x02` $\to$ `0x01` $\to \dots$).
- Firmware computes the signed difference (`delta = (int8_t)(curr - prev)` with modulo roll-over compensation):
  - `delta > 0`: Emits **Scroll Up** pulse (Hiworld Key `0x12`).
  - `delta < 0`: Emits **Scroll Down** pulse (Hiworld Key `0x11`).

#### Byte 2: Reserved Padding
- Constant `0x00` in native PSA CAN2004 Peugeot 407 operation.

---

### 2.2 Native PSA CAN2004 Stalk Tip Buttons (`0x221`)
- **CAN ID:** `0x221` (545 dec, `INFOS_GEN_ODB`)
- **DLC:** 7 bytes
- **Source ECU:** COM2000 / BSI
- **Byte 0 Bitfield (Trip Computer ODB):**
  - **Bit 3 (`0x08`):** Push-button on the tip of the right wiper stalk.
    - Idle State: `data[0] = 0xC0` (`221#c0ffffffffffff`)
    - Pressed State: `data[0] = 0xC8` (`221#c8ffffffffffff`)
    - Cycles trip computer display: Instantaneous Telemetry $\to$ Trip 1 $\to$ Trip 2 $\to$ Off.

---

### 2.3 Native PSA CAN2004 RD4 Console Buttons (`0x3E5`)
- **CAN ID:** `0x3E5` (997 dec), **DLC:** 6 bytes
- **Source:** RD4 radio fascia keypad. Only this frame is used for RD4 fascia keys (AUDIO, TRIP, CLIM, DARK, MENU, OK, ESC, arrows).
- Full 2-bit field layout and verification evidence: see section 3.3.
- `0x167` (`COMMANDES_EMF`, EMF display page) and `0x0DF` (display state) are **not** key sources.

---

### 2.4 Legacy Aftermarket CAN Box Alias (`0x0F6`)
For compatibility with aftermarket Chinese CAN bus adapters that pre-process stalk inputs into internal `0x0F6` frames:
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

### 3.1 Hiworld Base Info Frame (`Cmd 0x11` / `CarBaseInfo`)
The real OEM Hiworld Canbox broadcasts **10 payload bytes** on CMD `0x11`:

```
5A A5 0A 11 [B0] [B1] [KeyID] [State] [B4] [B5] [B6] [B7] [B8] [B9] [Checksum]
```

- **Sync Header:** `0x5A 0xA5`
- **Length ($L$):** `0x0A` (10 payload bytes)
- **Command ID:** `0x11` (`17` dec / `CarBaseInfo`)
- **Payload Field Mapping:**
  - `Byte 0`: Base status flags (`0x23` nominal: ACC active, ignition valid)
  - `Byte 1`: Status (`0x00`)
  - `Byte 2`: **Key Code** (`0x00` = Idle / Release, non-zero = Active Key)
  - `Byte 3`: **Key State** (`0x01` = Pressed, `0x00` = Released)
  - `Byte 4..5`: Telemetry flags / constants (`0x00 0x0A`)
  - `Byte 6..7`: Reserved (`0x00 0x00`)
  - `Byte 8..9`: Constant telemetry padding (`0x5E 0x22`)
- **Checksum:** `((Length + CmdID + sum(Payload)) - 1) & 0xFF`
- **Total Frame Length on Wire:** 15 bytes

### 3.2 Steering Wheel Stalk Key Code Translation Table (`Cmd 0x11`)
Directly verified against real OEM Hiworld transmissions and Android `com.qf.vehicle` driver mapping:

| Button / Physical Action | Source CAN Signal | Hiworld KeyID (Byte 2) | Android QF App Action | Raise KeyID | Bagoo KeyID |
|:---|:---|:---:|:---:|:---:|:---:|
| **Volume Up** | `0x21F` Byte 0: `0x08` | `0x01` | Volume + | `0x14` | `0x01` |
| **Volume Down** | `0x21F` Byte 0: `0x04` | `0x02` | Volume - | `0x15` | `0x02` |
| **Mute / Unmute** | `0x21F` Byte 0: `0x0C` (Vol+ & Vol-) | `0x03` | Audio Mute | `0x12` | `0x05` |
| **Source (SRC)** | `0x21F` Byte 0: `0x02` | `0x0B` | Media Source Cycle | `0x11` | `0x07` |
| **Next Track (Seek+)** | `0x21F` Byte 0: `0x80` | `0x08` | Next Track / Seek Up | `0x18` | `0x03` |
| **Prev Track (Seek-)** | `0x21F` Byte 0: `0x40` | `0x09` | Previous Track / Seek Down | `0x17` | `0x04` |
| **Rotary Scroll Up** | `0x21F` Byte 1 Counter $+1$ | `0x12` | Scroll Up (Pulse) | `0x42` | `0x13` |
| **Rotary Scroll Down** | `0x21F` Byte 1 Counter $-1$ | `0x11` | Scroll Down (Pulse) | `0x43` | `0x14` |
| **Stalk Tip Trip Button (ODB)** | `0x221` Byte 0: `0x08` (`0xC8`) | `0x14` | Trip Page Cycle / Stalk Key `0x14` | — | — |
| **Key Released (Idle)** | `0x21F` Byte 0: `0x00` / `0x221` Bit 3: `0` | `0x00` | All Keys Released | `0x00` | `0x00` |

---

### 3.3 RD4 Center Console & Fascia Buttons (`Cmd 0x21` / `ControlPanelKey`)
Verified against (a) a capture of the original OEM Hiworld Canbox while each RD4 fascia button was pressed (wire frames below), and (b) `can_log_buttons.log`, a raw CAN capture of the same buttons. The OEM Hiworld CAN box reports fascia key events via dedicated **CMD `0x21`** with a **2-byte payload**, separate from stalk telemetry (`Cmd 0x11`):

```
5A A5 02 21 [KeyID] [PressState] [Checksum]
```

- **Sync Header:** `0x5A 0xA5`
- **Length ($L$):** `0x02` (2 payload bytes)
- **Command ID:** `0x21` (`33` dec / `ControlPanelKey`)
- **KeyID:** Identifier of the physical RD4 fascia button.
- **PressState:** `0x01` = Pressed, `0x00` = Released.
- **Release Frame:** When a button is released, `5A A5 02 21 00 00 22` is emitted (KeyID `0x00`, state `0x00`).
- **Checksum:** `((Length + CmdID + KeyID + PressState) - 1) & 0xFF`.

#### RD4 Fascia Key Translation Table (verified OEM Hiworld Canbox output)

| Physical RD4 Fascia Button | Hiworld KeyID | Wire Packet (Press) | Wire Packet (Release) |
|:---|:---:|:---|:---|
| **AUDIO** | `0x31` | `5A A5 02 21 31 01 54` | `5A A5 02 21 00 00 22` |
| **TRIP** | `0x40` | `5A A5 02 21 40 01 63` | `5A A5 02 21 00 00 22` |
| **CLIM** | `0x28` | `5A A5 02 21 28 01 4B` | `5A A5 02 21 00 00 22` |
| **DARK** | `0x07` | `5A A5 02 21 07 01 2A` | `5A A5 02 21 00 00 22` |
| **OK** | `0x24` | `5A A5 02 21 24 01 47` | `5A A5 02 21 00 00 22` |
| **ESC** | `0x25` | `5A A5 02 21 25 01 48` | `5A A5 02 21 00 00 22` |
| **TEL / PHONE** | `0x05` (HU key table `PHONE`; not seen in the OEM Canbox log) | `5A A5 02 21 05 01 28` | `5A A5 02 21 00 00 22` |
| **MENU** | `0x2E` | `5A A5 02 21 2E 01 51` | `5A A5 02 21 00 00 22` |
| **UP** | `0x17` | `5A A5 02 21 17 01 3A` | `5A A5 02 21 00 00 22` |
| **DOWN** | `0x18` | `5A A5 02 21 18 01 3B` | `5A A5 02 21 00 00 22` |
| **LEFT** | `0x19` | `5A A5 02 21 19 01 3C` | `5A A5 02 21 00 00 22` |
| **RIGHT** | `0x1A` | `5A A5 02 21 1A 01 3D` | `5A A5 02 21 00 00 22` |

> [!IMPORTANT]
> Earlier revisions of this document had AUDIO/CLIM and TRIP/DARK KeyIDs swapped, and LEFT/RIGHT swapped. The table above is the corrected, verified mapping.

#### RD4 Source Frame: CAN ID `0x3E5` ONLY
For the RD4 radio fascia, **all** key events come from CAN ID **`0x3E5`** (DLC 6). No other CAN ID (`0x167`, `0x0DF`, `0x0F6`, ...) is a key source for RD4 fascia buttons.

Every button occupies a **2-bit field**; a non-zero field value means pressed. Observed pressed value is `01` (e.g. `0x01`, `0x04`, `0x40`); the decoder must test the whole field (e.g. `byte & 0xC0 != 0`), not a single bit.

| Byte | Bits 7:6 | Bits 5:4 | Bits 3:2 | Bits 1:0 |
|:---:|:---:|:---:|:---:|:---:|
| **Byte 0** | MENU | TEL (not yet captured) | *(unused)* | CLIM |
| **Byte 1** | TRIP | MODE (unverified) | *(unused)* | AUDIO |
| **Byte 2** | OK | ESC | **DARK** | *(unused)* |
| **Byte 3-4** | always `0x00` | | | |
| **Byte 5** | UP | DOWN | RIGHT | LEFT |

Evidence from `can_log_buttons.log` (button press order: AUDIO, TRIP, CLIM, DARK, DARK):

| Press | `0x3E5` frame (idle = `000000000000`) | Decoded |
|:---|:---|:---|
| 1 | `000100000000` | Byte 1 bits 1:0 = AUDIO |
| 2 | `004000000000` | Byte 1 bits 7:6 = TRIP |
| 3 | `010000000000` | Byte 0 bits 1:0 = CLIM |
| 4 | `000004000000` | Byte 2 bits 3:2 = DARK |
| 5 | `000004000000` | Byte 2 bits 3:2 = DARK |

`can_log_buttons1.log` (press order: MENU, OK, ESC):

| Press | `0x3E5` frame | Decoded |
|:---|:---|:---|
| 1 | `400000000000` | Byte 0 bits 7:6 = MENU |
| 2 | `000040000000` | Byte 2 bits 7:6 = OK |
| 3 | `000010000000` | Byte 2 bits 5:4 = ESC |

`can_log_buttons2.log` (arrows): only **one** `0x3E5` key frame was captured, `000000000040` = Byte 5 bits 7:6 = **UP** (press ~390 ms, then idle). No DOWN/LEFT/RIGHT frames are present in this capture, so those three fields remain unverified on CAN.

Each press is a single `0x3E5` frame with the button field set, followed ~350-450 ms later by the idle frame `000000000000` (release). MENU, OK and ESC are verified in `can_log_buttons1.log`, and UP in `can_log_buttons2.log`. DOWN, LEFT and RIGHT bit positions come only from the simulator CAN2004 radio documentation (their Hiworld KeyIDs are verified by the OEM Canbox log).

- **TEL:** `0x3E5` Byte 0 bits 5:4 (simulator doc; not yet captured on this car). Hiworld code `0x05` (`PHONE`) from the HU key table. Decoded, but unverified on vehicle.
- **MODE:** Byte 1 bits 5:4 from the simulator doc, no Hiworld code known; **pending**, not decoded.
- **BAND & SOURCE:** handled internally by the RD4/RD5 radio; no frames on the Comfort CAN bus.
- **`0x0DF` / `0x167`:** not key frames. `0x0DF` is a display/lighting state frame (it toggles `0x10`/`0x90` on the first byte every ~500 ms regardless of key presses, as seen in all button logs) and `0x167` is the EMF display page frame. Neither is decoded as a button.
- **Wiper Stalk Tip Trip Button:** broadcast via CAN ID `0x221` Byte 0 Bit 3 (`0x08`), emitting **CMD `0x11` Key `0x14`** (`5A A5 0A 11 23 00 14 01 00 0A 00 00 5E 22 DC`).

#### Hiworld Idle / Keep-Alive Frames from the OEM Canbox (pending, undocumented)
After a period with no key press the OEM Canbox sends:
```
5A A5 02 11 03 00 00 00 00 0A 00 00 5E 22 A7   (CMD 0x11, base info; first byte 0x03)
5A A5 02 21 00 00 22                           (panel key release)
5A A5 02 22 00 00 23                           (CMD 0x22, meaning unknown)
```
Their purpose (note the `02` length byte on the `0x11` frame in this capture) is not yet understood; CMD `0x22` is **pending** and not implemented.

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
#define HIWORLD_BASE_PAYLOAD_LEN 10

/* Supported Vendor Types */
typedef enum {
    CANBOX_VENDOR_HIWORLD = 0,
    CANBOX_VENDOR_RAISE   = 1,
    CANBOX_VENDOR_BAGOO   = 2
} canbox_vendor_t;

/* Internal Stalk State Structure */
typedef struct {
    uint8_t prev_b0_21f;
    uint8_t prev_counter_21f;
    uint8_t active_key_code;
    bool    mute_latched;
    uint8_t prev_b0_221;
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
    if (!ctx) return;
    memset(ctx, 0, sizeof(psa_stalk_ctx_t));
    ctx->vendor = vendor;
    ctx->tx_fn = tx_fn;
}

/* Transmit a serialized key packet (10-byte payload for Hiworld) */
static inline void psa_stalk_emit_key(psa_stalk_ctx_t *ctx, uint8_t hiworld_id, uint8_t raise_id, uint8_t state) {
    if (!ctx || !ctx->tx_fn) return;

    uint8_t packet[16];
    size_t len = 0;

    switch (ctx->vendor) {
        case CANBOX_VENDOR_HIWORLD: {
            /* 10-Byte Payload: [23] [00] [KeyID] [State] [00] [0A] [00] [00] [5E] [22] */
            packet[0] = HIWORLD_SOF1;
            packet[1] = HIWORLD_SOF2;
            packet[2] = HIWORLD_BASE_PAYLOAD_LEN; /* 0x0A (10 payload bytes) */
            packet[3] = HIWORLD_CMD_CAR_BASE;     /* 0x11 */
            packet[4] = 0x23;                     /* Status flags */
            packet[5] = 0x00;
            packet[6] = (state != 0) ? hiworld_id : 0x00;
            packet[7] = state;
            packet[8] = 0x00;
            packet[9] = 0x0A;
            packet[10] = 0x00;
            packet[11] = 0x00;
            packet[12] = 0x5E;
            packet[13] = 0x22;

            uint8_t sum = 0;
            for (size_t i = 2; i <= 13; i++) {
                sum += packet[i];
            }
            packet[14] = (uint8_t)((sum - 1) & 0xFF);
            len = 15;
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
    if (!ctx || !data || dlc < 2) return;

    uint8_t b0 = data[0];
    uint8_t counter = data[1];

    /* 1. MUTE Detection: Simultaneous Vol+ (0x08) & Vol- (0x04) */
    bool is_mute = ((b0 & 0x0C) == 0x0C);
    if (is_mute && !ctx->state.mute_latched) {
        psa_stalk_emit_key(ctx, 0x03, 0x12, 1); /* Mute Press */
        ctx->state.mute_latched = true;
        ctx->state.active_key_code = 0x03;
    } else if (!is_mute && ctx->state.mute_latched) {
        psa_stalk_emit_key(ctx, 0x00, 0x00, 0); /* Mute Release */
        ctx->state.mute_latched = false;
        ctx->state.active_key_code = 0;
    }

    /* 2. Primary Buttons (Evaluated when not in chorded Mute) */
    if (!is_mute && !ctx->state.mute_latched) {
        #define CHECK_KEY(mask, hw_id, rz_id) do { \
            if ((b0 & (mask)) && !(ctx->state.prev_b0_21f & (mask))) { \
                psa_stalk_emit_key(ctx, (hw_id), (rz_id), 1); \
                ctx->state.active_key_code = (hw_id); \
            } else if (!(b0 & (mask)) && (ctx->state.prev_b0_21f & (mask)) && ctx->state.active_key_code == (hw_id)) { \
                psa_stalk_emit_key(ctx, 0x00, 0x00, 0); \
                ctx->state.active_key_code = 0; \
            } \
        } while (0)

        CHECK_KEY(0x80, 0x08, 0x18); /* Next Track / Seek Up */
        CHECK_KEY(0x40, 0x09, 0x17); /* Prev Track / Seek Down */
        CHECK_KEY(0x08, 0x01, 0x14); /* Volume Up */
        CHECK_KEY(0x04, 0x02, 0x15); /* Volume Down */
        CHECK_KEY(0x02, 0x0B, 0x11); /* Source / Mode */

        #undef CHECK_KEY
    }

    /* 3. Rotary Thumbwheel Counter (Byte 1) Delta Tracking */
    if (ctx->state.prev_counter_21f != 0 && counter != ctx->state.prev_counter_21f) {
        int8_t delta = (int8_t)(counter - ctx->state.prev_counter_21f);
        if (delta > 0) {
            psa_stalk_emit_key(ctx, 0x12, 0x42, 1); /* Scroll Up Pulse */
            psa_stalk_emit_key(ctx, 0x00, 0x00, 0);
        } else if (delta < 0) {
            psa_stalk_emit_key(ctx, 0x11, 0x43, 1); /* Scroll Down Pulse */
            psa_stalk_emit_key(ctx, 0x00, 0x00, 0);
        }
    }

    ctx->state.prev_b0_21f = b0;
    ctx->state.prev_counter_21f = counter;
}

/* Process Native PSA CAN2004 Stalk Tips Frame (0x221) */
static inline void psa_stalk_process_can_0x221(psa_stalk_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (!ctx || !data || dlc < 1) return;

    uint8_t b0 = data[0];
    uint8_t prev = ctx->state.prev_b0_221;

    /* Bit 3: Trip Computer (ODB) Button (0xC8 = pressed, 0xC0 = idle) */
    if ((b0 & 0x08) && !(prev & 0x08)) {
        /* Edge detected: Trip button pressed */
    } else if (!(b0 & 0x08) && (prev & 0x08)) {
        /* Edge detected: Trip button released */
    }

    ctx->state.prev_b0_221 = b0;
}

#endif /* CANBOX_STALK_H */
```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: Volume Up Press & Release (`0x21F`)
- **CAN Injection:**
  ```bash
  cansend vcan0 21F#080200   # Volume Up Pressed
  cansend vcan0 21F#000200   # Volume Up Released
  ```
- **Expected UART Output (OEM Hiworld 10-Byte Frame):**
  - Press: `5A A5 0A 11 23 00 01 01 00 0A 00 00 5E 22 C9`
  - Release: `5A A5 0A 11 23 00 00 00 00 0A 00 00 5E 22 C7`

### Vector 2: Volume Down Press & Release (`0x21F`)
- **CAN Injection:**
  ```bash
  cansend vcan0 21F#040200   # Volume Down Pressed
  cansend vcan0 21F#000200   # Volume Down Released
  ```
- **Expected UART Output:**
  - Press: `5A A5 0A 11 23 00 02 01 00 0A 00 00 5E 22 CA`
  - Release: `5A A5 0A 11 23 00 00 00 00 0A 00 00 5E 22 C7`

### Vector 3: Chorded Mute (Vol+ & Vol- Together) (`0x21F`)
- **CAN Injection:**
  ```bash
  cansend vcan0 21F#0C0200   # MUTE Pressed (0x08 | 0x04)
  cansend vcan0 21F#000200   # MUTE Released
  ```
- **Expected UART Output:**
  - Press: `5A A5 0A 11 23 00 03 01 00 0A 00 00 5E 22 CB`
  - Release: `5A A5 0A 11 23 00 00 00 00 0A 00 00 5E 22 C7`

### Vector 4: Source (SRC) Press & Release (`0x21F`)
- **CAN Injection:**
  ```bash
  cansend vcan0 21F#020200   # Source Pressed
  cansend vcan0 21F#000200   # Source Released
  ```
- **Expected UART Output:**
  - Press: `5A A5 0A 11 23 00 0B 01 00 0A 00 00 5E 22 D3`
  - Release: `5A A5 0A 11 23 00 00 00 00 0A 00 00 5E 22 C7`

### Vector 5: Next Track Press & Release (`0x21F` Byte 0)
- **CAN Injection:**
  ```bash
  cansend vcan0 21F#800100   # Next Track Pressed
  cansend vcan0 21F#000100   # Next Track Released
  ```
- **Expected UART Output:**
  - Press: `5A A5 0A 11 23 00 08 01 00 0A 00 00 5E 22 D0`
  - Release: `5A A5 0A 11 23 00 00 00 00 0A 00 00 5E 22 C7`

### Vector 6: Previous Track Press & Release (`0x21F` Byte 0)
- **CAN Injection:**
  ```bash
  cansend vcan0 21F#400100   # Prev Track Pressed
  cansend vcan0 21F#000100   # Prev Track Released
  ```
- **Expected UART Output:**
  - Press: `5A A5 0A 11 23 00 09 01 00 0A 00 00 5E 22 D1`
  - Release: `5A A5 0A 11 23 00 00 00 00 0A 00 00 5E 22 C7`

### Vector 7: Rotary Scroll Wheel Step Up (`0x21F` Byte 1)
- **CAN Injection:**
  ```bash
  cansend vcan0 21F#000200   # Counter advances 0x01 -> 0x02
  ```
- **Expected UART Output (Pulse Up):**
  - Pulse Press: `5A A5 0A 11 23 00 12 01 00 0A 00 00 5E 22 DA`
  - Pulse Release: `5A A5 0A 11 23 00 00 00 00 0A 00 00 5E 22 C7`

### Vector 8: Rotary Scroll Wheel Step Down (`0x21F` Byte 1)
- **CAN Injection:**
  ```bash
  cansend vcan0 21F#000200   # Counter decrements 0x03 -> 0x02
  ```
- **Expected UART Output (Pulse Down):**
  - Pulse Press: `5A A5 0A 11 23 00 11 01 00 0A 00 00 5E 22 D9`
  - Pulse Release: `5A A5 0A 11 23 00 00 00 00 0A 00 00 5E 22 C7`

### Vector 9: Trip Computer (ODB) Stalk Tip (`0x221` Byte 0)
- **CAN Injection:**
  ```bash
  cansend vcan0 221#C8FFFFFFFFFFFF   # Wiper stalk tip pressed (0xC8)
  cansend vcan0 221#C0FFFFFFFFFFFF   # Wiper stalk tip released (0xC0)
  ```
