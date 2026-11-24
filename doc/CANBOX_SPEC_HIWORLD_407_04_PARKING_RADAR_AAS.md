# CAN Box Protocol Specification: Ultrasonic Parking Sensors (AAS)
## Topic 04: Peugeot 407 Front & Rear Parking Radar Telemetry
**Document File:** `CANBOX_SPEC_HIWORLD_407_04_PARKING_RADAR_AAS.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** `signal-db/parktronic.yaml` (`Msg0E1`), `QF_Canbus.apk` (`PeugeotDataParser.java` / `VehicleReceiver.java`)

---

# 1. Functional Domain & Architecture Overview

The Peugeot 407 Parking Assistance ECU (AAS - *Aide au Stationnement*) monitors front and rear ultrasonic sensors. In native Peugeot 407 CAN2004 Comfort bus architecture, sensor obstacle distances and zone activations are multiplexed onto CAN ID **`0x0E1`** (7 bytes, periodic 100 ms).

> [!IMPORTANT]
> **CAN2004 vs CAN2010 (AEE2010) Architecture Separation:**  
> On Peugeot 407 (CAN2004), CAN ID **`0x260`** is `MSG_BSI_INF_PROFILS` (BSI user profile configuration options: Profile 1, Profile 2, Manufacturer; 8 bytes, 250 ms), **NOT** parking sensors. Parking sensors on `0x260` (Rear AAS) and `0x270` (Front AAS) belong strictly to CAN2010 / AEE2010 (e.g. Peugeot 308, 508, RCZ). In `canbox-core`, CAN2004 decoding uses `0x0E1` exclusively and ignores `0x260` for AAS to prevent profile bytes from triggering false obstacle alarms or radar display flickering.

The CAN box translates these raw distance zones into serialized headunit radar packets (**`Cmd 0x41`** / `Handle.CarRadarState`), allowing the Android infotainment screen to render graphical distance arcs and trigger corresponding parking buzzer audio profiles.

> [!NOTE]
> **Reverse Window Activation Requirement:** On Android Head Units (e.g., TS10, UIS7862, Topway), the reverse camera view (`QF_BackCar.apk`) is launched by the MCU hardware monitoring the physical `BACK` / `REVERSE` wire (triggering `com.qf.action.BACKCAR_START` or KeyEvent 289). Radar telemetry packets (`Cmd 0x41`) are processed by `QF_Canbus.apk` and rendered as overlays while the reverse window is active (or if Front Running Radar is enabled).

---

# 2. PSA CAN Bus Bitfield Specification

### 2.1 Native Peugeot 407 CAN2004 Parking Radar Frame (`0x0E1`)
- **CAN ID:** `0x0E1` (Standard 11-bit Identifier)
- **DLC:** 7 bytes
- **Transmission Cycle:** Periodic 50–100 ms (confirmed in RT4 SW 8.31 `Fp_Network_CAN.out` / `mmi_alerts.out` `C_ParkingMMI::UpdateParkingStatus`)
- **Inactive / Quiescent Frame:** `D8 00 3F FC FC FC 00` (`B0=0xD8`: Rear=6 Off, Front=6 Off; `B1=0x00` beep off; `B2=0x3F` silence; `B3..B5=FC FC FC`: all 6 zones=7 clear, `B5[1]=0` popup hidden). When `B5[1]=1` with all zones clear (`FC FC FE`), the popup is shown with all 8 UART slots cleared to `0xFF`.
- **Byte Layout (`B0..B6`):**
  - `B0[7:5]`: Rear AAS status (`4`=Active `0x80`, `5`=Standby, `6`=Idle/Off `0xC0`, `3`=Fault, `1`=Inhibited)
  - `B0[4:2]`: Front AAS status (`4`=Active `0x10`, `5`=Standby, `6`=Idle/Off `0x18`, `1`=Fault) — e.g. `0x98`=Rear active, `0xD0`=Front active, `0x90`=Both active, `0xD8`=Both idle
  - `B1[7:6]`: Audio speaker side (`0`=Silence, `1`=Left, `2`=Right, `3`=Both); `B1[5]`: Audio zone (`0`=Rear 500 Hz, `1`=Front 800 Hz); `B1[4]`: Audio beep enable (`1`=ON)
  - `B2[7:6]`: Beep ON duration (`0`=90 ms, `1`=160 ms, `2`=220 ms, `3`=300 ms); `B2[5:0]`: Beep pause (`* 8 ms`, `0x00`=continuous, `0x3F`=silence)
  - `B3[7:5]` / `B3[4:2]`: Rear Left / Rear Middle (`0`=closest/STOP .. `6`=far, `7`=clear)
  - `B4[7:5]` / `B4[4:2]`: Rear Right / Front Left (`0`=closest/STOP .. `6`=far, `7`=clear)
  - `B5[7:5]` / `B5[4:2]`: Front Middle / Front Right (`0`=closest/STOP .. `6`=far, `7`=clear); `B5[1]`: Graphic radar popup request (`1`=show, `0`=hide)
  - `B6`: Unused (`0x00`)

### 2.2 Alternate / AEE2010 Parking Radar Frames (`0x260` / `0x270`) — CAN2010 Only
*(Note: Not applicable to Peugeot 407 CAN2004. On CAN2004, `0x260` is `MSG_BSI_INF_PROFILS` broadcast by the BSI at 250 ms. These frames are only used on CAN2010 / AEE2010 platforms.)*
- **Rear Radar (`0x260`):**
  - `Byte 0`: Rear Left Outer (0x00..0x04, 0xFF = Inactive)
  - `Byte 1`: Rear Center Inner (0x00..0x04, 0xFF = Inactive)
  - `Byte 2`: Rear Right Outer (0x00..0x04, 0xFF = Inactive)
  - `Byte 3`: Bit 7 = System Active (1=Active), Bit 0 = System Fault
- **Front Radar (`0x270`):**
  - `Byte 0`: Front Left Outer
  - `Byte 1`: Front Center Inner
  - `Byte 2`: Front Right Outer

### 2.3 Distance Threshold & Acoustic Tone Mapping:
| Step Code | Physical Distance ($d$) | Android Graphic Arcs | Acoustic Beep Profile |
|:---:|:---:|:---:|:---|
| `0x00` | $d \le 30\text{ cm}$ (Zone 1 / Critical) | 10 Solid Red Arcs | Continuous tone |
| `0x01` | $30\text{ cm} < d \le 60\text{ cm}$ (Zone 2) | 7 Red-Orange Arcs | Rapid intermittent tone ($8\text{ Hz}$) |
| `0x02` | $60\text{ cm} < d \le 90\text{ cm}$ (Zone 3) | 5 Orange Arcs | Fast intermittent tone ($4\text{ Hz}$) |
| `0x03` | $90\text{ cm} < d \le 120\text{ cm}$ (Zone 4) | 3 Yellow Arcs | Moderate intermittent tone ($2\text{ Hz}$) |
| `0x04` | $d > 120\text{ cm}$ (Zone 5) | 1 Green Arc | Slow intermittent tone ($1\text{ Hz}$) |
| `0xFF` / `0x07` | Inactive / No Obstacle | 0 Arcs (Hidden) | Mute / Silent |

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Hiworld Unified Radar Telemetry (`Cmd 0x41` / `Handle.CarRadarState`)
- **Sync Header:** `0x5A 0xA5`
- **Length ($L$):** `0x0C` (12 payload bytes: 8 sensor distance bytes + 4 configuration bytes)
- **Command ID:** `0x41` (`65` decimal / `Handle.CarRadarState`)
- **Payload Layout (12 Bytes):**
  - `Byte 0`: Rear Left Outer (RL) ($0 \dots 6$, `0xFF` = clear)
  - `Byte 1`: Rear Left Center (RML) ($0 \dots 6$)
  - `Byte 2`: Rear Right Center (RMR) ($0 \dots 6$)
  - `Byte 3`: Rear Right Outer (RR) ($0 \dots 6$)
  - `Byte 4`: Front Left Outer (FL) ($0 \dots 6$, `0xFF` = clear)
  - `Byte 5`: Front Left Center (FML) ($0 \dots 6$)
  - `Byte 6`: Front Right Center (FMR) ($0 \dots 6$)
  - `Byte 7`: Front Right Outer (FR) ($0 \dots 6$)
  - `Byte 8`: Radar active / enabled flag (`0x01`)
  - `Byte 9`: Reserved (`0x00`)
  - `Byte 10`: 6-sensor configuration mask (`0x3F`)
  - `Byte 11`: Distance scale / max zone steps (`0x05`)
- **Value Semantics:**
  - `0x00`: Zone 0 (closest / critical proximity / touching obstacle)
  - `0x01..0x06`: Obstacle proximity zones (increasing distance)
  - `0xFF`: Inactive / No obstacle detected (sensor arc hidden)
- **Checksum:** `((Length + CmdID + sum(Payload)) - 1) & 0xFF`
- **Total Frame Wire Length:** 17 bytes (`5A A5 0C 41 [8 Bytes] 01 00 3F 05 Checksum`)

> [!NOTE]
> In the Hiworld protocol dictionary, `Cmd 0x22` is reserved for physical rotary control knobs (`Handle.ControlPanelKnob`), whereas `Cmd 0x41` is dedicated to radar state telemetry (`Handle.CarRadarState`). OEM Hiworld canbox sends 12 payload bytes (`0x0C`) with trailing metadata `01 00 3F 05`.
>
> **Cadence & Companion Commands (`C-009` Verified):**
> - **Delta-Only `0x41` Cadence:** Both OEM `H1H2PAF23A` firmware (48 ms delta scan, excluded from 528 ms heartbeat) and `QF_Canbus_system` (`WindowRadar` has no inactivity timeout; `VehicleSender.showRadarLayer()` drops unchanged frames) confirm delta-only `0x41` emission is sufficient; periodic keepalives are unnecessary. `QF_Canbus_system` `PeugeotDataParser` decodes `B0..B7` (`0x00`=closest level `1`, `0x01..0x06`=`2,4,5,7,8,10`, `0xFF`=clear `0`) and ignores `B8..B11`.
> - **`Cmd 0xD7` (Redundant OEM Mirror):** Older OEM `H1H2PA123A` boxes also emit 9-byte `0xD7` (`01 [8 zones]`), parsed by `WcComDataParser` into the exact same 8 radar fields. Absent in `H1H2PAF23A` and unneeded in `canbox-core`.
> - **`Cmd 0x43` (Unused):** Absent in both `QF_Canbus_system` and OEM Hiworld PSA firmware.
> - **`Cmd 0xF2` (Not 407 AAS):** 2-byte downlink (`forwardAmvParkingState`) writes only AEE2010 Visiopark/360° camera CAN `0x1A9` (never `0x15B`). On Peugeot 407, AAS enable/disable uses `Cmd 0x7B [0x0B, state]` $\to$ CAN `0x15B`.

### 3.2 Raise Radar Compatibility Format:
- **Front Radar (`Cmd 0x30`):** `0x2E 0x30 0x06 [FL] [FC] [FR] [RL] [RC] [RR] [Checksum]`
- **Rear Radar (`Cmd 0x32`):** `0x2E 0x32 0x07 0x00 [RL] [RC] [RR] [FL] [FC] [FR] [Checksum]`

---

# 4. Pure C99 Firmware Implementation

```c
#ifndef CANBOX_RADAR_H
#define CANBOX_RADAR_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define HIWORLD_SOF1             0x5A
#define HIWORLD_SOF2             0xA5
#define HIWORLD_CMD_RADAR_STATE  0x41

typedef struct {
    uint8_t rear_left_outer;
    uint8_t rear_left_center;
    uint8_t rear_right_center;
    uint8_t rear_right_outer;
    uint8_t front_left_outer;
    uint8_t front_left_center;
    uint8_t front_right_center;
    uint8_t front_right_outer;
    bool    rear_active;
    bool    front_active;
    bool    display_active;
    bool    system_fault;
} psa_radar_state_t;

typedef void (*canbox_uart_tx_fn)(const uint8_t *buf, size_t len);

typedef struct {
    psa_radar_state_t state;
    canbox_uart_tx_fn uart_tx;
} psa_radar_ctx_t;

static inline void psa_radar_init(psa_radar_ctx_t *ctx, canbox_uart_tx_fn uart_tx) {
    memset(ctx, 0, sizeof(psa_radar_ctx_t));
    ctx->state.rear_left_outer    = 0xFF;
    ctx->state.rear_left_center   = 0xFF;
    ctx->state.rear_right_center  = 0xFF;
    ctx->state.rear_right_outer   = 0xFF;
    ctx->state.front_left_outer   = 0xFF;
    ctx->state.front_left_center  = 0xFF;
    ctx->state.front_right_center = 0xFF;
    ctx->state.front_right_outer  = 0xFF;
    ctx->uart_tx = uart_tx;
}

/* Transmit Hiworld Radar Packet (Cmd 0x41) */
static inline void psa_radar_send_hiworld(psa_radar_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[17];
    p[0] = HIWORLD_SOF1;
    p[1] = HIWORLD_SOF2;
    p[2] = 0x0C;                    /* Length: 12 Payload bytes */
    p[3] = HIWORLD_CMD_RADAR_STATE; /* Cmd ID: 0x41 */

    p[4] = ctx->state.rear_left_outer;
    p[5] = ctx->state.rear_left_center;
    p[6] = ctx->state.rear_right_center;
    p[7] = ctx->state.rear_right_outer;

    p[8]  = ctx->state.front_left_outer;
    p[9]  = ctx->state.front_left_center;
    p[10] = ctx->state.front_right_center;
    p[11] = ctx->state.front_right_outer;

    p[12] = 0x01;                   /* Radar active / enabled flag */
    p[13] = 0x00;                   /* Reserved */
    p[14] = 0x3F;                   /* 6-sensor configuration mask */
    p[15] = 0x05;                   /* Distance scale / max zone steps */

    uint8_t sum = 0;
    for (size_t i = 2; i <= 15; i++) {
        sum += p[i];
    }
    p[16] = (uint8_t)((sum - 1) & 0xFF);

    ctx->uart_tx(p, 17);
}

/* Map 3-bit PSA sensor zone (0..7) to Hiworld distance step (0..6, 0xFF) */
static inline uint8_t psa_radar_map_zone(uint8_t raw3bit) {
    if (raw3bit >= 7) return 0xFF; /* 0xFF = clear / inactive */
    /* PSA raw: 0 = closest (zone 0 in Hiworld) .. 6 = farthest (zone 6 in Hiworld) */
    return raw3bit;
}

/* Process Native PSA CAN 0x0E1 (Peugeot 407 Parktronic) */
static inline void psa_radar_process_can_0x0e1(psa_radar_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 6) return;

    ctx->state.rear_active    = (data[1] >> 6) & 1;
    ctx->state.front_active   = (data[1] >> 4) & 1;
    ctx->state.display_active = (data[5] & 0x02) ? true : false;

    if (!ctx->state.display_active && !ctx->state.rear_active && !ctx->state.front_active) {
        ctx->state.rear_left_outer    = 0xFF;
        ctx->state.rear_left_center   = 0xFF;
        ctx->state.rear_right_center  = 0xFF;
        ctx->state.rear_right_outer   = 0xFF;
        ctx->state.front_left_outer   = 0xFF;
        ctx->state.front_left_center  = 0xFF;
        ctx->state.front_right_center = 0xFF;
        ctx->state.front_right_outer  = 0xFF;
    } else {
        uint8_t rl = (data[3] >> 5) & 0x07;
        uint8_t rc = (data[3] >> 2) & 0x07;
        uint8_t rr = (data[4] >> 5) & 0x07;
        uint8_t fl = (data[4] >> 2) & 0x07;
        uint8_t fc = (data[5] >> 5) & 0x07;
        uint8_t fr = (data[5] >> 2) & 0x07;

        ctx->state.rear_left_outer    = psa_radar_map_zone(rl);
        ctx->state.rear_left_center   = psa_radar_map_zone(rc);
        ctx->state.rear_right_center  = psa_radar_map_zone(rc);
        ctx->state.rear_right_outer   = psa_radar_map_zone(rr);

        ctx->state.front_left_outer   = psa_radar_map_zone(fl);
        ctx->state.front_left_center  = psa_radar_map_zone(fc);
        ctx->state.front_right_center = psa_radar_map_zone(fc);
        ctx->state.front_right_outer  = psa_radar_map_zone(fr);
    }

    psa_radar_send_hiworld(ctx);
}

/* Process AEE2010 PSA CAN 0x260 (Rear Radar) */
static inline void psa_radar_process_can_0x260(psa_radar_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 3) return;

    ctx->state.rear_left_outer   = data[0];
    ctx->state.rear_left_center  = data[1];
    ctx->state.rear_right_center = data[1];
    ctx->state.rear_right_outer  = data[2];

    if (dlc >= 4) {
        ctx->state.rear_active  = (data[3] & 0x80) ? true : false;
        ctx->state.system_fault = (data[3] & 0x01) ? true : false;
    }

    psa_radar_send_hiworld(ctx);
}

/* Process AEE2010 PSA CAN 0x270 (Front Radar) */
static inline void psa_radar_process_can_0x270(psa_radar_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 3) return;

    ctx->state.front_left_outer   = data[0];
    ctx->state.front_left_center  = data[1];
    ctx->state.front_right_center = data[1];
    ctx->state.front_right_outer  = data[2];

    psa_radar_send_hiworld(ctx);
}

#endif /* CANBOX_RADAR_H */
```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: Native CAN 0x0E1 Obstacle
- **CAN ID `0x0E1` Injection (from `dump_2026-09-29_21-00-15.log`):**
  ```bash
  cansend vcan0 0E1#24403F04202600
  ```
- **Expected UART Output (Hiworld `0x41`):**
  - Frame: `5A A5 0C 41 00 01 01 01 00 01 01 01 01 00 3F 05 97`

### Vector 2: Quiescent / All Clear
- **CAN ID `0x0E1` Injection (from `dump_2026-09-29_20-44-32.log`):**
  ```bash
  cansend vcan0 0E1#24003FFCFCFC00
  ```
- **Expected UART Output (Hiworld `0x41`):**
  - Frame: `5A A5 0C 41 FF FF FF FF FF FF FF FF 01 00 3F 05 89`
