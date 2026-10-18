# SPEC Part 1: Peugeot 407 Specific CAN-Bus Telemetry Specification
## Portable C99 Firmware Implementation for Multi-MCU & Linux Desktop Simulation

**Document File:** `SPEC_01_PEUGEOT_407_DATA.md`  
**Target Standard:** ISO/IEC 9899:1999 (Pure C99)  
**Hardware Compatibility:** STM32 (bxCAN/FDCAN), ESP32 (TWAI), Arduino/AVR (ATmega328P + MCP2515), Nuvoton NUC131 (Cortex-M0 CAN), Linux (SocketCAN `vcan0` + POSIX PTY)  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard Identifiers)  
**Headunit Protocols:** Raise (RZC `0x2E`), Hiworld (WC `0x5A 0xA5`), Bagoo (`0xD5`), Simple Soft (XP `0x2E`)

---

# Table of Contents
1. [1.1 Steering Column Stalk & Buttons](#11-steering-column-stalk--buttons)
2. [1.2 Dual-Zone Climate Control (HVAC)](#12-dual-zone-climate-control-hvac)
3. [1.3 Ultrasonic Parking Sensors (Front & Rear AAS)](#13-ultrasonic-parking-sensors-front--rear-aas)
4. [1.4 Trip Computer & Engine Telemetry](#14-trip-computer--engine-telemetry)
5. [1.5 Doors & Body Status](#15-doors--body-status)
6. [1.6 Steering Wheel Angle & Dynamic Trajectory](#16-steering-wheel-angle--dynamic-trajectory)
7. [1.7 OEM JBL Sound Amplifier (DSP)](#17-oem-jbl-sound-amplifier-dsp)
8. [1.8 RD4 Radio & CD Changer Media Data](#18-rd4-radio--cd-changer-media-data)
9. [Pure C99 Architecture & Hardware Abstraction Layer (HAL)](#pure-c99-architecture--hardware-abstraction-layer-hal)
10. [Linux Desktop Simulation & Verification Vector Harness](#linux-desktop-simulation--verification-vector-harness)

---

# 1.1 Steering Column Stalk & Buttons

### 1.1.1 CAN Frame & Protocol Specification
- **PSA CAN ID:** `0x0F6` (DLC: 8, Cycle: 50 ms or Immediate on Event)
- **Raise Command ID:** `0x02` (Payload Length: 2 bytes: `[KeyID] [State: 1=Down, 0=Up]`)
- **Hiworld Command ID:** `0x11` (Payload Length: 2 bytes)
- **SimpleSoft Command ID:** `0x02` (Payload Length: 2 bytes)

#### CAN ID `0x0F6` Bitfield Mapping:
```
+--------+------------------------------------------------------------------------------------+
| Byte 0 | Bit 7: Dark Screen Button (0x80)        Bit 3: Volume Up (0x08)                    |
|        | Bit 6: Source / Mode Toggle (0x40)      Bit 2: Volume Down (0x04)                  |
|        | Bit 5: ESC / Back Button (0x20)         Bit 1: Next Track / Seek+ (0x02)           |
|        | Bit 4: Scroll Wheel Click / OK (0x10)   Bit 0: Prev Track / Seek- (0x01)           |
+--------+------------------------------------------------------------------------------------+
| Byte 1 | Bit 6: Menu Button (0x40)                                                          |
|        | Bits 3..0: Rotary Scroll Encoder 4-bit Signed Step Delta (Up = +1, Down = -1)     |
+--------+------------------------------------------------------------------------------------+
| Byte 2 | Bit 1: Telephone Hangup Button (0x02)   Bit 0: Telephone Answer Button (0x01)      |
+--------+------------------------------------------------------------------------------------+
```

#### Headunit Key Code Translation Table:
| Signal / Button | PSA CAN `0x0F6` Mask | Raise KeyID (Hex) | Hiworld KeyID | SimpleSoft KeyID | Android Headunit Action |
|:---|:---:|:---:|:---:|:---:|:---|
| **Volume Up** | Byte 0 `0x08` | `0x14` | `0x01` | `0x14` | Increase master audio volume |
| **Volume Down** | Byte 0 `0x04` | `0x15` | `0x02` | `0x15` | Decrease master audio volume |
| **Next Track (Seek+)** | Byte 0 `0x02` | `0x18` | `0x03` | `0x18` | Next media track / tuner seek up |
| **Previous Track (Seek-)** | Byte 0 `0x01` | `0x17` | `0x04` | `0x17` | Prev media track / tuner seek down |
| **Source / Mode Toggle** | Byte 0 `0x40` | `0x11` | `0x07` | `0x11` | Cycle Radio $\to$ USB $\to$ BT $\to$ AUX |
| **Scroll Wheel Click (OK)**| Byte 0 `0x10` | `0x19` | `0x09` | `0x19` | Enter / Confirm selection |
| **Dark Screen Button** | Byte 0 `0x80` | `0x20` | `0x38` | `0x20` | Headunit screen blackout |
| **ESC / Back Button** | Byte 0 `0x20` | `0x60` | `0x0B` | `0x60` | Return / Back navigation |
| **Menu Button** | Byte 1 `0x40` | `0x54` | `0x0A` | `0x54` | Open BSI / Vehicle Settings menu |
| **Rotary Scroll Up** | Byte 1 Bits 0..3 (+1) | `0x42` | `0x13` | `0x42` | List scroll up / Rotary tick |
| **Rotary Scroll Down** | Byte 1 Bits 0..3 (-1) | `0x43` | `0x14` | `0x43` | List scroll down / Rotary tick |
| **Telephone Answer** | Byte 2 `0x01` | `0x32` | `0x19` | `0x32` | Answer incoming phone call |
| **Telephone Hangup** | Byte 2 `0x02` | `0x31` | `0x15` | `0x31` | End active call / Reject call |

### 1.1.2 Pure C99 Implementation
```c
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t prev_b0;
    uint8_t prev_b1;
    uint8_t prev_b2;
} psa_stalk_state_t;

static psa_stalk_state_t g_stalk_state;

void psa_stalk_process_can(const uint8_t *data, uint8_t dlc, void (*send_key)(uint8_t key_id, uint8_t state)) {
    if (dlc < 2) return;
    uint8_t b0 = data[0];
    uint8_t b1 = data[1];
    uint8_t b2 = (dlc >= 3) ? data[2] : 0;

    #define DISPATCH_KEY(curr, prev, mask, key_code) do { \
        if (((curr) & (mask)) && !((prev) & (mask))) send_key((key_code), 1); \
        if (!((curr) & (mask)) && ((prev) & (mask))) send_key((key_code), 0); \
    } while(0)

    DISPATCH_KEY(b0, g_stalk_state.prev_b0, 0x08, 0x14); /* Vol + */
    DISPATCH_KEY(b0, g_stalk_state.prev_b0, 0x04, 0x15); /* Vol - */
    DISPATCH_KEY(b0, g_stalk_state.prev_b0, 0x02, 0x18); /* Next */
    DISPATCH_KEY(b0, g_stalk_state.prev_b0, 0x01, 0x17); /* Prev */
    DISPATCH_KEY(b0, g_stalk_state.prev_b0, 0x40, 0x11); /* Source */
    DISPATCH_KEY(b0, g_stalk_state.prev_b0, 0x10, 0x19); /* OK / Confirm */
    DISPATCH_KEY(b0, g_stalk_state.prev_b0, 0x80, 0x20); /* Dark */
    DISPATCH_KEY(b0, g_stalk_state.prev_b0, 0x20, 0x60); /* ESC */
    DISPATCH_KEY(b1, g_stalk_state.prev_b1, 0x40, 0x54); /* Menu */
    DISPATCH_KEY(b2, g_stalk_state.prev_b2, 0x01, 0x32); /* Tel Answer */
    DISPATCH_KEY(b2, g_stalk_state.prev_b2, 0x02, 0x31); /* Tel Hangup */

    /* Rotary Encoder Scroll delta */
    int8_t scroll_curr = (int8_t)(b1 & 0x0F);
    int8_t scroll_prev = (int8_t)(g_stalk_state.prev_b1 & 0x0F);
    int8_t delta = scroll_curr - scroll_prev;
    if (delta > 0 || delta < -7) {
        send_key(0x42, 1); send_key(0x42, 0); /* Scroll Up Pulse */
    } else if (delta < 0 || delta > 7) {
        send_key(0x43, 1); send_key(0x43, 0); /* Scroll Down Pulse */
    }

    g_stalk_state.prev_b0 = b0;
    g_stalk_state.prev_b1 = b1;
    g_stalk_state.prev_b2 = b2;
}
```

---

# 1.2 Dual-Zone Climate Control (HVAC)

### 1.2.1 CAN Frame & Protocol Specification
- **PSA CAN ID:** `0x1D0` (DLC: 8, Cycle: 200 ms / On Change)
- **Raise Command ID:** `0x21` (Length: 7 bytes)
- **Hiworld Command ID:** `0x31` (Length: 7 bytes)
- **SimpleSoft Command ID:** `0x21` (Length: 7 bytes)

#### CAN ID `0x1D0` Bitfield Mapping:
```
+--------+------------------------------------------------------------------------------------+
| Byte 0 | Bit 7: System Power (1=ON, 0=OFF)       Bit 3: Auto Climate Mode (1=Active)        |
|        | Bit 6: A/C Compressor (1=ON, 0=OFF)     Bit 2: DUAL Mode (1=Dual, 0=Mono/Sync)     |
|        | Bit 5: Air Recirculation (1=Internal)   Bit 0: Rear Heated Screen / Demist (1=ON)  |
|        | Bit 4: Air Quality Sensor (AQS Auto-Recirculation)                                  |
+--------+------------------------------------------------------------------------------------+
| Byte 1 | Bit 7: Driver Windshield Defrost Vent   Bits 3..0: Blower Fan Speed (0=Off, 1..8)  |
|        | Bit 6: Driver Center / Face Vent        Bit 4: Auto Blower Intensity               |
|        | Bit 5: Driver Footwell Vent                                                         |
+--------+------------------------------------------------------------------------------------+
| Byte 2 | Driver Set Temperature Code (Raw: 28..60 in 0.5 deg C increments; 0x00=LO, 0xFF=HI)|
+--------+------------------------------------------------------------------------------------+
| Byte 3 | Passenger Set Temperature Code (Raw: 28..60; 0x00=LO, 0xFF=HI)                      |
+--------+------------------------------------------------------------------------------------+
| Byte 4 | Bit 7: Max Front Windshield Defrost / Demist (1=Active)   Bit 3: Max A/C Active    |
+--------+------------------------------------------------------------------------------------+
| Byte 6 | Bit 7: Passenger Windshield Defrost Vent  Bit 5: Passenger Footwell Vent           |
|        | Bit 6: Passenger Center / Face Vent                                                |
+--------+------------------------------------------------------------------------------------+
```

#### Temperature Scaling Formulas:
$$\text{Temperature } (^\circ\text{C}) = \frac{\text{RawByte}}{2.0}$$
$$\text{RawByte} = \text{Round}(T_{^\circ\text{C}} \times 2.0)$$
* Special bounds: `0x00` = Display `"LO"`, `0xFF` = Display `"HI"`.
* Example: $21.5^\circ\text{C} \implies 43 = \text{0x2B}$; $22.0^\circ\text{C} \implies 44 = \text{0x2C}$.

### 1.2.2 Pure C99 Implementation
```c
typedef struct {
    bool    power;
    bool    ac_compressor;
    bool    recirculation;
    bool    aqs_auto;
    bool    auto_mode;
    bool    dual_mode;
    bool    rear_defrost;
    bool    front_max_defrost;
    bool    ac_max;
    uint8_t fan_speed;      /* 0..8 */
    uint8_t driver_temp_raw;/* 0x00=LO, 0xFF=HI, 28..60 */
    uint8_t pass_temp_raw;
    bool    driver_wind_up;
    bool    driver_wind_face;
    bool    driver_wind_down;
    bool    pass_wind_up;
    bool    pass_wind_face;
    bool    pass_wind_down;
} hvac_state_t;

size_t build_raise_hvac_packet(const hvac_state_t *st, uint8_t *out_buf, size_t max_len) {
    if (max_len < 11) return 0;
    out_buf[0] = 0x2E;
    out_buf[1] = 0x21;
    out_buf[2] = 0x07; /* Length */

    uint8_t d0 = 0;
    if (st->power)         d0 |= 0x80;
    if (st->ac_compressor) d0 |= 0x40;
    if (st->recirculation) d0 |= 0x20;
    if (st->aqs_auto)      d0 |= 0x10;
    if (st->auto_mode)     d0 |= 0x08;
    if (st->dual_mode)     d0 |= 0x04;
    if (st->rear_defrost)  d0 |= 0x01;
    out_buf[3] = d0;

    uint8_t d1 = (st->fan_speed & 0x0F);
    if (st->driver_wind_up)   d1 |= 0x80;
    if (st->driver_wind_face) d1 |= 0x40;
    if (st->driver_wind_down) d1 |= 0x20;
    out_buf[4] = d1;

    out_buf[5] = st->driver_temp_raw;
    out_buf[6] = st->pass_temp_raw;

    uint8_t d4 = 0;
    if (st->front_max_defrost) d4 |= 0x80;
    if (st->ac_max)            d4 |= 0x08;
    out_buf[7] = d4;

    out_buf[8] = 0x00; /* Rear Power / Flags */

    uint8_t d6 = 0;
    if (st->pass_wind_up)   d6 |= 0x80;
    if (st->pass_wind_face) d6 |= 0x40;
    if (st->pass_wind_down) d6 |= 0x20;
    out_buf[9] = d6;

    /* Checksum: (Sum ^ 0xFF) & 0xFF */
    uint8_t sum = 0;
    for (size_t i = 1; i <= 9; i++) {
        sum += out_buf[i];
    }
    out_buf[10] = (uint8_t)(sum ^ 0xFF);
    return 11;
}
```

---

# 1.3 Ultrasonic Parking Sensors (Front & Rear AAS)

### 1.3.1 CAN Frame & Protocol Specification
- **PSA Rear AAS CAN ID:** `0x260` (DLC: 8, Cycle: 100 ms)
- **PSA Front AAS CAN ID:** `0x270` (DLC: 8, Cycle: 100 ms)
- **Raise Front Radar Command ID:** `0x30` (Length: 6 bytes)
- **Raise Rear Radar Command ID:** `0x32` (Length: 7 bytes)

#### Distance Steps & Android Graphic Mapping:
| CAN Step Value | Physical Distance | Android UI Visualization | Audio Tone |
|:---:|:---:|:---:|:---|
| `0x00` | $> 120\text{ cm}$ | 1 Green Arc | Slow beep ($1\text{ Hz}$) |
| `0x01` | $90 \dots 120\text{ cm}$ | 3 Yellow Arcs | Medium beep ($2\text{ Hz}$) |
| `0x02` | $60 \dots 90\text{ cm}$ | 5 Orange Arcs | Fast beep ($4\text{ Hz}$) |
| `0x03` | $30 \dots 60\text{ cm}$ | 7 Red-Orange Arcs | Rapid beep ($8\text{ Hz}$) |
| `0x04` | $< 30\text{ cm}$ | 10 Solid Red Arcs | Continuous tone |
| `0xFF` | Inactive / Clear | 0 Arcs / Hidden | Mute |

```c
/* Pure C99 Parking Radar Formatter */
size_t build_raise_rear_radar(uint8_t rl, uint8_t rc, uint8_t rr,
                              uint8_t fl, uint8_t fc, uint8_t fr,
                              uint8_t *out_buf, size_t max_len) {
    if (max_len < 11) return 0;
    out_buf[0] = 0x2E;
    out_buf[1] = 0x32;
    out_buf[2] = 0x07; /* Length */
    out_buf[3] = 0x00; /* Flag */
    out_buf[4] = rl;
    out_buf[5] = rc;
    out_buf[6] = rr;
    out_buf[7] = fl;
    out_buf[8] = fc;
    out_buf[9] = fr;

    uint8_t sum = 0;
    for (size_t i = 1; i <= 9; i++) sum += out_buf[i];
    out_buf[10] = (uint8_t)(sum ^ 0xFF);
    return 11;
}
```

---

# 1.4 Trip Computer & Engine Telemetry

### 1.4.1 CAN Frame & Protocol Specification
- **Instant Fuel & DTE Range:** PSA CAN ID `0x165` (DLC: 8, Cycle: 500 ms) $\to$ Raise `Cmd 0x33` (Length: 6 bytes)
- **Trip 1 Statistics:** PSA CAN ID `0x1A5` (DLC: 8, Cycle: 1000 ms) $\to$ Raise `Cmd 0x34` (Length: 6 bytes)
- **Trip 2 Statistics:** PSA CAN ID `0x2A5` (DLC: 8, Cycle: 1000 ms) $\to$ Raise `Cmd 0x35` (Length: 6 bytes)
- **Outside Ambient Temp:** PSA CAN ID `0x165` Byte 6 $\to$ Raise `Cmd 0x36` (Length: 1 byte)
- **Speed, RPM, Reverse:** PSA CAN ID `0x036` (DLC: 8, Cycle: 50 ms) $\to$ Raise `Cmd 0x40` (Length: 1 byte)

#### Detailed Scaling & Bit Manipulation Table:
| Parameter Name | PSA CAN ID & Byte Offsets | Raise Frame Layout & Offsets | Physical Formula / Encoding |
|:---|:---|:---|:---|
| **Instant Fuel** | `0x165` Bytes 0..1 (Big-Endian) | `Cmd 0x33` Bytes 0..1 (Big-Endian) | $\text{Fuel } (\text{L/100km}) = \frac{\text{Raw}_{16}}{10.0}$; $> 3000 \implies \text{Invalid}$ |
| **Cruising Range (DTE)**| `0x165` Bytes 2..3 (Big-Endian) | `Cmd 0x33` Bytes 2..3 (Big-Endian) | $\text{Range } (\text{km}) = \text{Raw}_{16}$; $> 2000 \implies \text{Invalid}$ |
| **Dest Distance** | `0x165` Bytes 4..5 (Big-Endian) | `Cmd 0x33` Bytes 4..5 (Big-Endian) | $\text{Dest } (\text{km}) = \text{Raw}_{16}$ |
| **Trip 1 Distance**| `0x1A5` Bytes 0..1 (Big-Endian) | `Cmd 0x34` Bytes 4..5 (Big-Endian) | $\text{Dist } (\text{km}) = \frac{\text{Raw}_{16}}{10.0}$ |
| **Trip 1 Avg Speed**| `0x1A5` Byte 4 | `Cmd 0x34` Bytes 2..3 (Big-Endian) | $\text{Speed } (\text{km/h}) = \text{Raw}_{16}$ |
| **Trip 1 Avg Fuel** | `0x1A5` Bytes 2..3 (Big-Endian) | `Cmd 0x34` Bytes 0..1 (Big-Endian) | $\text{AvgFuel } (\text{L/100km}) = \frac{\text{Raw}_{16}}{10.0}$ |
| **Outside Temp** | `0x165` Byte 6 | `Cmd 0x36` Byte 0 | Bit 7 = Sign (1 = Neg, 0 = Pos), Bits 6..0 = Magnitude ($^\circ\text{C}$) |
| **Reverse Gear** | `0x036` Byte 1 Bit 7 (`0x80`) | `Cmd 0x40` Byte 0 | `0x80` = Reverse Active (Camera On), `0x00` = Reverse Off |
| **Handbrake** | `0x221` Byte 1 Bit 0 (`0x01`) | `Cmd 0x38` Byte 1 Bit 0 | `1` = Handbrake Engaged, `0` = Released |

```c
/* Pure C99 Trip Computer Formatter */
size_t build_raise_trip1(uint16_t avg_fuel_dkl, uint16_t avg_spd_kmh, uint16_t dist_dkm, uint8_t *out) {
    out[0] = 0x2E;
    out[1] = 0x34;
    out[2] = 0x06;
    out[3] = (uint8_t)(avg_fuel_dkl >> 8);
    out[4] = (uint8_t)(avg_fuel_dkl & 0xFF);
    out[5] = (uint8_t)(avg_spd_kmh >> 8);
    out[6] = (uint8_t)(avg_spd_kmh & 0xFF);
    out[7] = (uint8_t)(dist_dkm >> 8);
    out[8] = (uint8_t)(dist_dkm & 0xFF);
    uint8_t sum = 0;
    for (size_t i = 1; i <= 8; i++) sum += out[i];
    out[9] = (uint8_t)(sum ^ 0xFF);
    return 10;
}
```

---

# 1.5 Doors & Body Status

### 1.5.1 CAN Frame & Protocol Specification
- **PSA CAN ID:** `0x221` (DLC: 8, Cycle: 100 ms)
- **Raise Command ID:** `0x38` (Length: 8 bytes)

```
Raise 0x38 Payload Byte 0:
[Bit 7] Driver Front Door (1=Open, 0=Closed)
[Bit 6] Passenger Front Door (1=Open)
[Bit 5] Rear Left Door (1=Open)
[Bit 4] Rear Right Door (1=Open)
[Bit 3] Trunk / Tailgate (1=Open)
[Bit 2] Engine Bonnet / Hood (1=Open)
```

---

# 1.6 Steering Wheel Angle & Dynamic Trajectory

### 1.6.1 CAN Frame & Protocol Specification
- **PSA CAN ID:** `0x0E6` (DLC: 8, Cycle: 20 ms)
- **Raise Command ID:** `0x29` (Length: 2 bytes: Little-Endian signed 16-bit)
- **Hiworld Command ID:** `0x26` (Length: 2 bytes: Big-Endian signed 16-bit)

#### Scaling Formula:
$$\text{Steering Angle } (^\circ) = \frac{\text{SignedValue}_{16}}{10.0}$$
* Range: $-5400 \dots +5400 \implies -540.0^\circ \dots +540.0^\circ$.
* Negative: Turned Left; Positive: Turned Right; $0$: Centered.

```c
size_t build_raise_steering_angle(int16_t angle_deci_deg, uint8_t *out) {
    out[0] = 0x2E;
    out[1] = 0x29;
    out[2] = 0x02;
    out[3] = (uint8_t)(angle_deci_deg & 0xFF);        /* Little Endian */
    out[4] = (uint8_t)((angle_deci_deg >> 8) & 0xFF);
    out[5] = (uint8_t)((out[1] + out[2] + out[3] + out[4]) ^ 0xFF);
    return 6;
}
```

---

# 1.7 OEM JBL Sound Amplifier (DSP)

### 1.7.1 CAN Frame & Protocol Specification
- **PSA Feedback CAN ID:** `0x1A0` (DLC: 8, Cycle: 200 ms)
- **Raise Command ID:** `0x56` (Length: 8 bytes)

```
Raise 0x56 Payload Layout:
[Byte 0] 0x00 (Fixed padding)
[Byte 1] Bass Level (0..14, Neutral=7, represents -7..0..+7)
[Byte 2] Treble Level (0..14, Neutral=7)
[Byte 3] Balance (0..14, 7=Center, <7=Left, >7=Right)
[Byte 4] Fader (0..14, 7=Center, <7=Rear, >7=Front)
[Byte 5] EQ Preset (0=Custom/Off, 1=Pop, 2=Classic, 3=Electronic, 4=Jazz, 5=Vocal)
[Byte 6] Flags: Loudness (Bit 4: 0x10), Speed-Dependent Volume Comp (Bits 3..0: 0..3)
[Byte 7] Master Amplifier Volume Level (0..30)
```

---

# 1.8 RD4 Radio & CD Changer Media Data

### 1.8.1 CAN Frame & Protocol Specification
- **CD Changer Info:** PSA CAN ID `0x3A6` $\to$ Raise `Cmd 0x54` (Length: 7 bytes)
  - `Byte 0`: `0x02` (CD Player Mode)
  - `Byte 1`: Disc Slot ($1 \dots 6$)
  - `Byte 2`: Track Number ($1 \dots 99$)
  - `Byte 3`: Total Disc Tracks ($1 \dots 99$)
  - `Byte 4`: Elapsed Minutes ($0 \dots 59$)
  - `Byte 5`: Elapsed Seconds ($0 \dots 59$)
  - `Byte 6`: Play Flags (`0x01`=Random, `0x02`=Scan, `0x04`=Repeat)
- **Radio Station RDS Name:** PSA CAN ID `0x396` $\to$ Raise `Cmd 0x55` (Length: 8 bytes ASCII)
  - Transmits Program Service (PS) Name (e.g. `"BBC R1  "`).

---

# Pure C99 Architecture & Hardware Abstraction Layer (HAL)

### Universal C99 HAL Interface (`canbox_hal.h`)
```c
#ifndef CANBOX_HAL_H
#define CANBOX_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
    bool     is_ext;
} canbox_msg_t;

typedef struct {
    bool (*can_init)(uint32_t baud);
    bool (*can_send)(const canbox_msg_t *msg);
    bool (*can_recv)(canbox_msg_t *msg);
    bool (*uart_send)(const uint8_t *buf, size_t len);
    bool (*uart_recv_byte)(uint8_t *byte);
    uint32_t (*get_millis)(void);
    void (*delay_ms)(uint32_t ms);
} canbox_hal_t;

#endif
```

---

# Linux Desktop Simulation & Verification Vector Harness

### Verification Vectors for `cansend` & Expected Serial Outputs:
```bash
# 1. Volume Up button press:
cansend vcan0 0F6#0800000000000000
# Expected UART Output (Hex): 2E 02 02 14 01 E8

# 2. Climate: 21.5 C Driver, 22.0 C Pass, AC ON, Auto ON:
cansend vcan0 1D0#C8452B2C00004000
# Expected UART Output (Hex): 2E 21 07 C8 45 2B 2C 00 00 40 8E

# 3. Rear Parking Radar Zone 4 Obstacle:
cansend vcan0 260#0303030000000000
# Expected UART Output (Hex): 2E 32 07 00 03 03 03 FF FF FF C2

# 4. Driver Door Open:
cansend vcan0 221#8000000000000000
# Expected UART Output (Hex): 2E 38 08 80 00 00 00 00 00 00 00 57
```

