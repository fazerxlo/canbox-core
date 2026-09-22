# CAN Box Protocol Specification: Trip Computer & Engine Telemetry
## Topic 06: Peugeot 407 Trip Computer, Fuel Economy & Dynamics
**Document File:** `CANBOX_SPEC_HIWORLD_407_06_TRIP_COMPUTER_TELEMETRY.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** `https://github.com/fazerxlo/canbox/tree/main/doc/CAN2004_0x0F6.md`, `CAN2004_radio.md`, `CAN_messages.md`

---

# 1. Functional Domain & Architecture Overview

The Peugeot 407 BSI and Engine Management ECU compute comprehensive journey metrics, fuel consumption statistics, and vehicle dynamics across multiple CAN frames:
- `0x165`: Instantaneous fuel consumption, cruising range (Distance-to-Empty), destination distance, and outside air temperature.
- `0x1A5`: Trip Computer 1 statistics (Distance traveled, average speed, average fuel economy).
- `0x2A5`: Trip Computer 2 statistics.
- `0x036`: Engine RPM, vehicle cluster speed, and reverse gear status.
- `0x221`: Parking brake (handbrake) physical state.
In Peugeot 407 (PSA CAN2004 architecture), engine telemetry, odometer readings, ambient temperatures, and trip metrics are broadcast across the following core frames:
- **`0x0F6` (BSI Slow Data, 500 ms):** Transmits engine coolant temperature ($\text{Raw} - 40^\circ\text{C}$), total vehicle odometer (24-bit uint $\times 0.1\text{ km}$), external ambient air temperature ($\text{Raw} \times 0.5 - 40^\circ\text{C}$), reverse gear status (Byte 7 Bit 7), wipers, and turn signals (blinkers).
- **`0x036` (Ignition & Illumination, 100 ms):** Transmits vehicle power state (`Byte 4: 0x01=IGN, 0x03=ACC`), dashboard illumination enabled (Byte 3 Bit 5), and luminosity level.
- **`0x161` (Engine Fluids):** Transmits oil temperature, oil level, and fuel tank level.
- **`0x221` / `0x2A1` / `0x261` (Trip Metrics):** Transmit instantaneous fuel consumption, Trip 1 statistics, and Trip 2 statistics.

The CAN box translates these parameters into Hiworld trip computer packets and accepts touchscreen trip reset commands from the Android system to clear Trip 1 or Trip 2.

```
+------------------------------------------------------------------------------------+
|                         Peugeot 407 BSI & Engine ECU                               |
|        [Instant Fuel, Range, Trip 1, Trip 2, RPM, Speed, Outside Temp, Reverse]    |
|                         Peugeot 407 BSI & Engine Management                        |
|        [BSI Slow Data (0x0F6), Power/Ignition (0x036), Trip Metrics (0x221)]       |
+------------------------------------------------------------------------------------+
                         │ (Uplink 0x165/1A5/2A5/036) ▲ (Downlink Reset 0x221)
                         ▼                            │
                                          │
                            [PSA CAN 0x0F6 / 0x036 / 0x221]
                                          ▼
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Aggregates multi-frame telemetry streams into unified `psa_trip_state_t`      |
|   2. Converts units (L/100km, km/h, km, signed deg C) to Hiworld UART packets      |
|   3. Translates Android Headunit Trip Reset requests (Cmd 0x82) to BSI CAN 0x221   |
|   1. Extracts Coolant Temp, Odometer, Ambient Temp, Reverse from 0x0F6             |
|   2. Decodes ACC/IGN power modes & dashboard dimming from 0x036                    |
|   3. Translates trip data to Hiworld 0x33 (Instant/DTE), 0x34 (Trip1), 0x35 (Trip2)|
|   4. Dispatches Hiworld 0x36 Outside Temp & 0x40 Reverse Camera Trigger            |
+------------------------------------------------------------------------------------+
                         │ (UART Telemetry)           ▲ (UART Reset Commands)
                         ▼                            │
                                          │
                   [UART Serial: 38400 baud, 8N1 / Hiworld Protocol]
                                          ▼
+------------------------------------------------------------------------------------+
|                     Android Headunit Vehicle Info & Dashboard App                  |
|          (Displays live digital gauges, trip graphs, and outside temperature)      |
+------------------------------------------------------------------------------------+
```

---

# 2. PSA CAN Bus Bitfield Specification

### 2.1 Instant Fuel, Range & Ambient Temperature (`0x165`)
- **CAN ID:** `0x165` (DLC: 8, Cycle: 500 ms)
### 2.1 BSI Slow Data Frame (`0x0F6` - `BSI_SLOW_DATA`)
- **CAN ID:** `0x0F6` (DLC: 8, Period: 500 ms)

```
+--------+--------+------------------------------------+-----------------------------+
| Byte   | Bit    | Function / Signal Name             | Value / Encoding Definition |
+--------+--------+------------------------------------+-----------------------------+
| Byte 0 | 7..0   | Instant Fuel High Byte (MSB)       | 0.1 L/100km (Big-Endian)    |
| Byte 1 | 7..0   | Instant Fuel Low Byte (LSB)        | > 3000 = Invalid/Coast/Idle |
| Byte 0 | 7..0   | BSI Status Byte                    | Real bus typical: 0x88      |
+--------+--------+------------------------------------+-----------------------------+
| Byte 2 | 7..0   | Distance to Empty (DTE) MSB        | Cruising Range (km, Big-End)|
| Byte 3 | 7..0   | Distance to Empty (DTE) LSB        | > 2000 = Invalid/Dash       |
| Byte 1 | 7..0   | Coolant Temperature                | Raw - 40 (Degrees Celsius)  |
+--------+--------+------------------------------------+-----------------------------+
| Byte 4 | 7..0   | Distance to Destination MSB        | Target Distance (km, Big-End|
| Byte 5 | 7..0   | Distance to Destination LSB        | Target Distance (km)        |
| Byte 2 | 7..0   | Total Odometer High Byte (MSB)     | 24-bit unsigned integer     |
| Byte 3 | 7..0   | Total Odometer Middle Byte         | Scale: 0.1 km per LSB       |
| Byte 4 | 7..0   | Total Odometer Low Byte (LSB)      | 0xFFFFFF = Invalid sentinel |
+--------+--------+------------------------------------+-----------------------------+
| Byte 6 | Bit 7  | Outside Temp Sign Bit              | 1 = Negative, 0 = Positive  |
|        | Bit 6..0| Outside Temp Magnitude            | Degrees Celsius (0..127 C)  |
| Byte 5 | 7..0   | External Ambient Temperature Raw   | Raw * 0.5 - 40.0 deg C      |
+--------+--------+------------------------------------+-----------------------------+
| Byte 7 | 7..0   | Reserved / Status Flags            | 0x00                        |
| Byte 6 | 7..0   | Filtered Ambient Temperature       | Raw * 0.5 - 40.0 deg C      |
+--------+--------+------------------------------------+-----------------------------+
```

### 2.2 Trip 1 (`0x1A5`) & Trip 2 (`0x2A5`) Statistics
- **CAN ID:** `0x1A5` (Trip 1) / `0x2A5` (Trip 2) (DLC: 8, Cycle: 1000 ms)

```
| Byte 7 | Bit 7  | Reverse Gear Status                | 1 = Reverse, 0 = Forward    |
|        | Bit 6  | Front Wipers Status                | 1 = Wiping, 0 = Off         |
|        | Bit 1:0| Turn Signal Blinkers Status        | 0=None, 1=Rt, 2=Lt, 3=Hazard|
+--------+--------+------------------------------------+-----------------------------+
| Byte   | Bit    | Function / Signal Name             | Value / Encoding Definition |
+--------+--------+------------------------------------+-----------------------------+
| Byte 0 | 7..0   | Trip Distance High Byte (MSB)      | 0.1 km units (Big-Endian)   |
| Byte 1 | 7..0   | Trip Distance Low Byte (LSB)       | e.g. 1254 = 125.4 km        |
+--------+--------+------------------------------------+-----------------------------+
| Byte 2 | 7..0   | Average Fuel High Byte (MSB)       | 0.1 L/100km (Big-Endian)    |
| Byte 3 | 7..0   | Average Fuel Low Byte (LSB)        | e.g. 68 = 6.8 L/100km       |
+--------+--------+------------------------------------+-----------------------------+
| Byte 4 | 7..0   | Average Speed (km/h)               | Raw integer (0..255 km/h)   |
+--------+--------+------------------------------------+-----------------------------+
| Byte 5..7|7..0  | Reserved / Checksum                | 0x00                        |
+--------+--------+------------------------------------+-----------------------------+
```

### 2.3 Engine Dynamics & Reverse Gear (`0x036`)
- **CAN ID:** `0x036` (DLC: 8, Cycle: 50 ms)
- `Bytes 0..1`: Engine RPM ($\text{RPM} = \text{Raw}_{16} / 8.0$, Big-Endian).
- `Byte 1 Bit 7`: Reverse Gear ($1 = \text{Engaged}, 0 = \text{Neutral/Forward}$).
- `Bytes 2..3`: Vehicle Speed ($\text{Speed} = \text{Raw}_{16} / 100.0\text{ km/h}$, Big-Endian).
### 2.2 Ignition & Dashboard Illumination (`0x036`)
- **CAN ID:** `0x036` (DLC: 8, Period: 100 ms)
- `Byte 3 Bit 5`: Dashboard Illumination Enabled ($1 = \text{On}$).
- `Byte 3 Bits 3:0`: Cluster Luminosity level ($0 \dots 15$).
- `Byte 4`: Power Mode (`0x01` = Ignition ON, `0x03` = Accessory ACC ON, `0x00` = Key Off).

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Hiworld Instantaneous Telemetry (`Cmd 0x33`)
- **Sync Header:** `0x5A 0xA5` | **Length:** `0x07` | **Cmd:** `0x33`
- **Payload Layout (6 Bytes - Big Endian):**
  - `Bytes 0..1`: Instant Fuel in $0.1\text{ L/100km}$ (e.g. `0x0044` = $6.8\text{ L/100km}$).
  - `Bytes 2..3`: Cruising Range / DTE in $\text{km}$ (e.g. `0x0280` = $640\text{ km}$).
  - `Bytes 4..5`: Destination Distance in $\text{km}$.
  - `Bytes 0..1`: Instant Fuel in $0.1\text{ L/100km}$
  - `Bytes 2..3`: Cruising Range (DTE) in $\text{km}$
  - `Bytes 4..5`: Destination Distance in $\text{km}$
- **Checksum:** 8-bit sum modulo 256 over `Length + CmdID + Payload`.

### 3.2 Hiworld Trip 1 (`Cmd 0x34`) & Trip 2 (`Cmd 0x35`)
- **Length:** `0x07` | **Cmd:** `0x34` (Trip 1) / `0x35` (Trip 2)
- **Payload Layout (6 Bytes - Big Endian):**
  - `Bytes 0..1`: Average Fuel in $0.1\text{ L/100km}$.
  - `Bytes 2..3`: Average Speed in $\text{km/h}$ (MSB in Byte 2, LSB in Byte 3).
  - `Bytes 4..5`: Distance in $0.1\text{ km}$.
### 3.2 Hiworld Ambient Outside Temperature (`Cmd 0x36`)
- **Sync Header:** `0x5A 0xA5` | **Length:** `0x02` | **Cmd:** `0x36`
- **Payload:** 1 byte signed temperature in $^\circ\text{C}$ (e.g. $+21^\circ\text{C} = \text{0x15}$, $-4^\circ\text{C} = \text{0xFC}$ or sign-magnitude $\text{0x84}$).

### 3.3 Hiworld Ambient Outside Temperature (`Cmd 0x36`)
- **Length:** `0x02` | **Cmd:** `0x36`
- **Payload (1 Byte):** Signed integer in $^\circ\text{C}$ or PSA Sign/Magnitude format ($+21^\circ\text{C} = \text{0x15}$, $-5^\circ\text{C} = \text{0x85}$).
### 3.3 Hiworld Reversing Camera Trigger (`Cmd 0x40`)
- **Sync Header:** `0x5A 0xA5` | **Length:** `0x02` | **Cmd:** `0x40`
- **Payload:** `0x80` = Reverse Active (Camera On), `0x00` = Reverse Inactive.

### 3.4 Downlink Trip Reset Command (`Cmd 0x82`)
When the user taps "Reset Trip 1" or "Reset Trip 2" on Android:
- **Trip 1 Reset UART:** `5A A5 03 82 41 00 C6` $\implies$ CAN Box sends `0x221#0000400000000000`
- **Trip 2 Reset UART:** `5A A5 03 82 22 00 A7` $\implies$ CAN Box sends `0x221#0000800000000000`

---

# 4. Pure C99 Firmware Implementation

```c
#ifndef CANBOX_TRIP_H
#define CANBOX_TRIP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

typedef struct {
    uint16_t instant_fuel_dkl; /* 0.1 L/100km */
    uint16_t range_km;         /* Distance to empty */
    uint16_t dest_dist_km;
    
    /* Trip 1 */
    uint16_t trip1_dist_dkm;   /* 0.1 km */
    uint16_t trip1_avg_speed;  /* km/h */
    uint16_t trip1_avg_fuel;   /* 0.1 L/100km */

    /* Trip 2 */
    uint16_t trip2_dist_dkm;
    uint16_t trip2_avg_speed;
    uint16_t trip2_avg_fuel;

    /* Dynamics */
    int16_t  coolant_temp_c;
    uint32_t total_odometer_km;
    int8_t   outside_air_temp_c;
    uint16_t vehicle_speed_kmh;
    uint16_t engine_rpm;
    bool     reverse_gear;
    bool     handbrake_pulled;
    bool     ignition_on;
    bool     accessory_on;
    bool     dash_lights_on;
    uint8_t  dash_luminosity;
    uint8_t  blinkers_state;
} psa_trip_state_t;

typedef void (*canbox_uart_tx_fn)(const uint8_t *buf, size_t len);
typedef void (*canbox_can_tx_fn)(uint32_t id, const uint8_t *data, uint8_t dlc);

typedef struct {
    psa_trip_state_t  state;
    canbox_uart_tx_fn uart_tx;
    canbox_can_tx_fn  can_tx;
} psa_trip_ctx_t;

static inline void psa_trip_init(psa_trip_ctx_t *ctx, canbox_uart_tx_fn uart_tx, canbox_can_tx_fn can_tx) {
static inline void psa_trip_init(psa_trip_ctx_t *ctx, canbox_uart_tx_fn uart_tx) {
    memset(ctx, 0, sizeof(psa_trip_ctx_t));
    ctx->uart_tx = uart_tx;
    ctx->can_tx  = can_tx;
}

/* Transmit Hiworld Instant Telemetry (Cmd 0x33) */
static inline void psa_trip_send_instant_hiworld(psa_trip_ctx_t *ctx) {
/* Transmit Hiworld Outside Temperature (Cmd 0x36) */
static inline void psa_trip_send_temp_hiworld(psa_trip_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[11];
    uint8_t p[6];
    p[0] = 0x5A;
    p[1] = 0xA5;
    p[2] = 0x07; /* Length: 1 Cmd + 6 Payload */
    p[3] = 0x33; /* Cmd ID */
    p[4] = (uint8_t)(ctx->state.instant_fuel_dkl >> 8);
    p[5] = (uint8_t)(ctx->state.instant_fuel_dkl & 0xFF);
    p[6] = (uint8_t)(ctx->state.range_km >> 8);
    p[7] = (uint8_t)(ctx->state.range_km & 0xFF);
    p[8] = (uint8_t)(ctx->state.dest_dist_km >> 8);
    p[9] = (uint8_t)(ctx->state.dest_dist_km & 0xFF);
    p[2] = 0x02; /* Length: 1 Cmd + 1 Payload */
    p[3] = 0x36; /* Cmd ID */
    p[4] = (uint8_t)ctx->state.outside_air_temp_c;
    p[5] = (uint8_t)(p[2] + p[3] + p[4]);

    uint8_t sum = 0;
    for (size_t i = 2; i <= 9; i++) sum += p[i];
    p[10] = sum;

    ctx->uart_tx(p, 11);
    ctx->uart_tx(p, 6);
}

/* Transmit Hiworld Trip 1 Statistics (Cmd 0x34) */
static inline void psa_trip_send_trip1_hiworld(psa_trip_ctx_t *ctx) {
/* Transmit Hiworld Reverse Trigger (Cmd 0x40) */
static inline void psa_trip_send_reverse_hiworld(psa_trip_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[11];
    uint8_t p[6];
    p[0] = 0x5A;
    p[1] = 0xA5;
    p[2] = 0x07;
    p[3] = 0x34;
    p[4] = (uint8_t)(ctx->state.trip1_avg_fuel >> 8);
    p[5] = (uint8_t)(ctx->state.trip1_avg_fuel & 0xFF);
    p[6] = (uint8_t)(ctx->state.trip1_avg_speed >> 8);
    p[7] = (uint8_t)(ctx->state.trip1_avg_speed & 0xFF);
    p[8] = (uint8_t)(ctx->state.trip1_dist_dkm >> 8);
    p[9] = (uint8_t)(ctx->state.trip1_dist_dkm & 0xFF);
    p[2] = 0x02;
    p[3] = 0x40;
    p[4] = ctx->state.reverse_gear ? 0x80 : 0x00;
    p[5] = (uint8_t)(p[2] + p[3] + p[4]);

    uint8_t sum = 0;
    for (size_t i = 2; i <= 9; i++) sum += p[i];
    p[10] = sum;

    ctx->uart_tx(p, 11);
    ctx->uart_tx(p, 6);
}

/* Process PSA CAN ID 0x165 Frame */
static inline void psa_trip_process_can_0x165(psa_trip_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 7) return;
/* Process PSA CAN 0x0F6 (BSI Slow Data) */
static inline void psa_trip_process_can_0x0F6(psa_trip_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 8) return;

    ctx->state.instant_fuel_dkl = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    ctx->state.range_km         = ((uint16_t)data[2] << 8) | (uint16_t)data[3];
    ctx->state.dest_dist_km     = ((uint16_t)data[4] << 8) | (uint16_t)data[5];
    ctx->state.coolant_temp_c = (int16_t)data[1] - 40;

    /* Temperature: Sign/Magnitude conversion */
    uint8_t t_raw = data[6];
    int8_t temp = (int8_t)(t_raw & 0x7F);
    if (t_raw & 0x80) temp = -temp;
    ctx->state.outside_air_temp_c = temp;
    uint32_t odo_raw = ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 8) | (uint32_t)data[4];
    if (odo_raw != 0xFFFFFF) {
        ctx->state.total_odometer_km = odo_raw / 10;
    }

    psa_trip_send_instant_hiworld(ctx);
}
    if (data[5] != 0xFF) {
        ctx->state.outside_air_temp_c = (int8_t)((float)data[5] * 0.5f - 40.0f);
        psa_trip_send_temp_hiworld(ctx);
    }

/* Process PSA CAN ID 0x1A5 Frame (Trip 1) */
static inline void psa_trip_process_can_0x1A5(psa_trip_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 5) return;
    bool rev = (data[7] & 0x80) ? true : false;
    if (rev != ctx->state.reverse_gear) {
        ctx->state.reverse_gear = rev;
        psa_trip_send_reverse_hiworld(ctx);
    }

    ctx->state.trip1_dist_dkm  = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    ctx->state.trip1_avg_fuel  = ((uint16_t)data[2] << 8) | (uint16_t)data[3];
    ctx->state.trip1_avg_speed = data[4];

    psa_trip_send_trip1_hiworld(ctx);
    ctx->state.blinkers_state = data[7] & 0x03;
}

/* Handle Android Downlink Trip Reset */
static inline void psa_trip_process_downlink_reset(psa_trip_ctx_t *ctx, uint8_t trip_page) {
    if (!ctx->can_tx) return;
/* Process PSA CAN 0x036 (Ignition & Illumination) */
static inline void psa_trip_process_can_0x036(psa_trip_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 5) return;

    uint8_t can_data[8] = {0};
    if (trip_page == 0x41) {
        can_data[2] = 0x40; /* Trip 1 Reset mask */
    } else if (trip_page == 0x22) {
        can_data[2] = 0x80; /* Trip 2 Reset mask */
    }
    ctx->can_tx(0x221, can_data, 8);
    ctx->state.dash_lights_on  = (data[3] & 0x20) ? true : false;
    ctx->state.dash_luminosity = data[3] & 0x0F;
    ctx->state.ignition_on     = (data[4] == 0x01);
    ctx->state.accessory_on    = (data[4] == 0x03);
}

#endif /* CANBOX_TRIP_H */
```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: Instant Fuel 6.8 L/100km (68 = 0x0044), Range 640 km (0x0280), Outside Temp +21°C (0x15)
- **CAN ID `0x165` Injection:**
### Vector 1: BSI Slow Data with Outside Temp 21.0°C (Raw 122 = 0x7A), Coolant 90°C (130 = 0x82), Reverse ON
- **CAN ID `0x0F6` Injection:**
  ```bash
  cansend vcan0 165#0044028000001500
  cansend vcan0 0F6#8882FFFFFF7A7A80
  ```
- **Expected UART Output (Hiworld Instant `0x33`):**
  - Frame: `5A A5 07 33 00 44 02 80 00 00 00`
- **Expected UART Output (Hiworld Temp `0x36` & Reverse `0x40`):**
  - Temp Frame: `5A A5 02 36 15 53` ($+21^\circ\text{C}$)
  - Reverse Frame: `5A A5 02 40 80 C2` (Reverse Active)

### Vector 2: Trip 1 Distance 125.4 km (1254 = 0x04E6), Avg Speed 58 km/h (0x003A), Avg Fuel 7.2 L/100km (72 = 0x0048)
- **CAN ID `0x1A5` Injection:**
### Vector 2: Ignition ON + Dash Lights ON (Luminosity 10 = 0x0A)
- **CAN ID `0x036` Injection:**
  ```bash
  cansend vcan0 1A5#04E600483A000000
  cansend vcan0 036#0E00002A010000A0
  ```
- **Expected UART Output (Hiworld Trip 1 `0x34`):**
  - Frame: `5A A5 07 34 00 48 00 3A 04 E6 A7`

- **Decoded Dynamics State:**
  - `dash_lights_on = true`, `dash_luminosity = 10`, `ignition_on = true`
