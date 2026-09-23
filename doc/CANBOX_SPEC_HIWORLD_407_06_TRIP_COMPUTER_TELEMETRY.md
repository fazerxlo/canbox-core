# CAN Box Protocol Specification: Trip Computer & Engine Telemetry
## Topic 06: Peugeot 407 Trip Computer, Fuel Economy & Dynamics
**Document File:** `CANBOX_SPEC_HIWORLD_407_06_TRIP_COMPUTER_TELEMETRY.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** `signal-db/trip.yaml` (`Msg221`, `Msg2A1`, `Msg261`), `signal-db/bsi.yaml` (`Msg0B6`, `Msg0F6`, `Msg161`), `QF_Canbus.apk` (`PeugeotDataParser.java`)

---

# 1. Functional Domain & Architecture Overview

In Peugeot 407 (PSA CAN2004 architecture), engine telemetry, odometer readings, ambient temperatures, and trip metrics are broadcast across the following core frames:
- **`0x0B6` (Fast Dynamic Data, 50 ms):** Engine RPM (bits 15..3 = RPM $\times 8$) and vehicle speed (uint16 scaled by 100 in km/h).
- **`0x0F6` (BSI Slow Data, 500 ms):** Engine coolant temperature ($\text{Raw} - 40^\circ\text{C}$), external ambient air temperature ($\text{Raw} \times 0.5 - 40^\circ\text{C}$), reverse gear status (Byte 7 Bit 7), wipers, and turn signals (blinkers).
- **`0x161` (BSI Gauges, 500 ms):** Oil temperature ($\text{Raw} - 40^\circ\text{C}$), fuel level percentage (Byte 3), and oil level (Byte 6).
- **`0x036` (Ignition & Illumination, 100 ms):** Power state (`Byte 4: 0x01=IGN, 0x03=ACC`), dashboard illumination enabled (Byte 3 Bit 5), and luminosity level.
- **`0x221` (Instantaneous Trip, 1000 ms):** Instantaneous fuel consumption ($0.1\text{ L/100km}$), cruising range / autonomy (km), distance ($0.1\text{ km}$), and stalk com button flags.
- **`0x2A1` (Trip 1 Historical, 1000 ms):** Mean speed (uint8 / uint16 km/h), distance (km), and average consumption ($0.1\text{ L/100km}$).
- **`0x261` (Trip 2 Historical, 1000 ms):** Same format as Trip 1.

The CAN box translates these parameters into Hiworld trip computer packets (`Cmd 0x13`, `0x14`, `0x15`) and accepts touchscreen trip reset commands from the Android system to clear Trip 1 or Trip 2.

```
+------------------------------------------------------------------------------------+
|                         Peugeot 407 BSI & Engine Management                        |
|        [BSI Slow Data (0x0F6), Power/Ignition (0x036), Trip Metrics (0x221)]       |
|        [Fast Dynamics (0x0B6), Historical Trip1 (0x2A1), Historical Trip2 (0x261)]  |
+------------------------------------------------------------------------------------+
                                          │
                  [PSA CAN 0x0B6 / 0x0F6 / 0x221 / 0x2A1 / 0x261]
                                          ▼
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Extracts Coolant Temp, Ambient Temp, Reverse from 0x0F6                       |
|   2. Decodes RPM and Vehicle Speed from 0x0B6                                      |
|   3. Translates trip data to Hiworld 0x13 (Instant/DTE), 0x14 (Trip1), 0x15 (Trip2)|
|   4. Dispatches BSI Personalization & Trip computer pages                          |
+------------------------------------------------------------------------------------+
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

### 2.1 Fast Dynamic Telemetry Frame (`0x0B6`)
- **CAN ID:** `0x0B6` (DLC: 8, Period: 50 ms)
- `Bytes 0..1`: Engine RPM $\times 8$ (`(data[0]<<8 | data[1]) >> 3` gives RPM in rpm; `0xFFFF` = invalid/off).
- `Bytes 2..3`: Vehicle Speed in $\text{km/h} \times 100$ (`((data[2]<<8 | data[3]) / 100.0)`).
- `Byte 7`: Trailer constant (`0xD0`).

### 2.2 BSI Slow Data Frame (`0x0F6` - `BSI_SLOW_DATA`)
- **CAN ID:** `0x0F6` (DLC: 8, Period: 500 ms)
- `Byte 0`: BSI Status byte (`0x88` typical).
- `Byte 1`: Coolant Temperature in $^\circ\text{C} = \text{Raw} - 40$.
- `Byte 2..4`: Operational flags / Odometer ($0.1\text{ km}$ LSB, `0xFFFFFF` = invalid).
- `Byte 5`: External Ambient Temperature Raw in $^\circ\text{C} = \text{Raw} \times 0.5 - 40.0$.
- `Byte 6`: Filtered Ambient Temperature ($\text{Raw} \times 0.5 - 40.0$).
- `Byte 7`:
  - `Bit 7`: Reverse Gear ($1 = \text{Reverse}, 0 = \text{Forward}$)
  - `Bit 6`: Front Wipers ($1 = \text{Wiping}, 0 = \text{Off}$)
  - `Bit 1:0`: Blinkers ($0 = \text{None}, 1 = \text{Right}, 2 = \text{Left}, 3 = \text{Hazard}$)

### 2.3 Trip Computer Instantaneous Frame (`0x221`)
- **CAN ID:** `0x221` (DLC: 7, Period: 1000 ms)
- `Byte 0`: Bit 7 = Hide Fuel, Bit 6 = Hide Dist, Bit 3 = Com Stalk Right, Bit 0 = Com Stalk Left.
- `Bytes 1..2`: Instantaneous Fuel Consumption ($0.1\text{ L/100km}$, Big-Endian uint16).
- `Bytes 3..4`: Cruising Range / Autonomy (km, Big-Endian uint16).
- `Bytes 5..6`: Instantaneous Distance ($0.1\text{ km}$, Big-Endian uint16).

### 2.4 Trip Computer Historical Records 1 & 2 (`0x2A1` & `0x261`)
- **CAN IDs:** `0x2A1` (Trip 1), `0x261` (Trip 2) (DLC: 7, Period: 1000 ms)
- `Byte 0`: Mean Speed ($\text{km/h}$, uint8).
- `Bytes 1..2`: Distance Traveled (km, Big-Endian uint16).
- `Bytes 3..4`: Average Consumption ($0.1\text{ L/100km}$, Big-Endian uint16).
- `Bytes 5..6`: Average Speed ($\text{km/h}$, Big-Endian uint16).

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Hiworld Instantaneous Telemetry (`Cmd 0x13` / `Handle.EcuInfoPage1`)
- **Sync Header:** `0x5A 0xA5`
- **Length ($L$):** `0x04` (4 payload bytes)
- **Command ID:** `0x13` (`19` decimal / `Handle.EcuInfoPage1`)
- **Payload Layout (4 Bytes - Big Endian):**
  - `Byte 0..1`: Instantaneous Fuel Consumption ($0.1\text{ L/100km}$, e.g. `0x0044` = $6.8\text{ L/100km}$)
  - `Byte 2..3`: Cruising Range / Distance-to-Empty ($\text{km}$, e.g. `0x0280` = $640\text{ km}$)
- **Checksum:** `((Length + CmdID + sum(Payload)) - 1) & 0xFF`
- **Wire Frame:** `5A A5 04 13 [4 Bytes] Checksum`

### 3.2 Hiworld Trip 1 Telemetry (`Cmd 0x14` / `Handle.EcuInfoPage2`)
- **Sync Header:** `0x5A 0xA5`
- **Length ($L$):** `0x06` (6 payload bytes)
- **Command ID:** `0x14` (`20` decimal / `Handle.EcuInfoPage2`)
- **Payload Layout (6 Bytes):**
  - `Byte 0..1`: Average Fuel Consumption ($0.1\text{ L/100km}$)
  - `Byte 2`: Reserved (`0x00`)
  - `Byte 3`: Average Speed ($\text{km/h}$, `0xFF` = 0)
  - `Byte 4..5`: Trip Distance Traveled ($\text{km}$)
- **Checksum:** `((Length + CmdID + sum(Payload)) - 1) & 0xFF`

### 3.3 Hiworld Trip 2 Telemetry (`Cmd 0x15` / `Handle.EcuInfoPage3`)
- **Sync Header:** `0x5A 0xA5`
- **Length ($L$):** `0x06` (6 payload bytes)
- **Command ID:** `0x15` (`21` decimal / `Handle.EcuInfoPage3`)
- **Payload Layout (6 Bytes):**
  - `Byte 0..1`: Average Fuel Consumption ($0.1\text{ L/100km}$)
  - `Byte 2`: Reserved (`0x00`)
  - `Byte 3`: Average Speed ($\text{km/h}$, `0xFF` = 0)
  - `Byte 4..5`: Trip Distance Traveled ($\text{km}$)
- **Checksum:** `((Length + CmdID + sum(Payload)) - 1) & 0xFF`

---

# 4. Pure C99 Firmware Implementation

```c
#ifndef CANBOX_TRIP_H
#define CANBOX_TRIP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define HIWORLD_SOF1        0x5A
#define HIWORLD_SOF2        0xA5
#define HIWORLD_CMD_ECU_P0  0x13
#define HIWORLD_CMD_ECU_P1  0x14
#define HIWORLD_CMD_ECU_P2  0x15

typedef struct {
    uint16_t rpm;
    uint16_t speed_kmh;
    int8_t   coolant_c;
    int8_t   ambient_c;
    bool     reverse_active;
    
    uint16_t instant_fuel_deci; /* 0.1 L/100km */
    uint16_t range_km;          /* Distance to Empty */
    
    uint16_t trip1_avg_fuel;    /* 0.1 L/100km */
    uint8_t  trip1_avg_speed;   /* km/h */
    uint16_t trip1_distance_km; /* km */

    uint16_t trip2_avg_fuel;    /* 0.1 L/100km */
    uint8_t  trip2_avg_speed;   /* km/h */
    uint16_t trip2_distance_km; /* km */
} psa_trip_state_t;

typedef void (*canbox_uart_tx_fn)(const uint8_t *buf, size_t len);

typedef struct {
    psa_trip_state_t  state;
    canbox_uart_tx_fn uart_tx;
} psa_trip_ctx_t;

static inline void psa_trip_init(psa_trip_ctx_t *ctx, canbox_uart_tx_fn uart_tx) {
    memset(ctx, 0, sizeof(psa_trip_ctx_t));
    ctx->uart_tx = uart_tx;
}

/* Transmit Hiworld Instantaneous Telemetry (Cmd 0x13) */
static inline void psa_trip_send_instant(psa_trip_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[9];
    p[0] = HIWORLD_SOF1;
    p[1] = HIWORLD_SOF2;
    p[2] = 0x04;               /* Length: 4 Payload bytes */
    p[3] = HIWORLD_CMD_ECU_P0; /* Cmd 0x13 */
    p[4] = (uint8_t)(ctx->state.instant_fuel_deci >> 8);
    p[5] = (uint8_t)(ctx->state.instant_fuel_deci & 0xFF);
    p[6] = (uint8_t)(ctx->state.range_km >> 8);
    p[7] = (uint8_t)(ctx->state.range_km & 0xFF);

    uint8_t sum = 0;
    for (size_t i = 2; i <= 7; i++) {
        sum += p[i];
    }
    p[8] = (uint8_t)((sum - 1) & 0xFF);

    ctx->uart_tx(p, 9);
}

/* Transmit Hiworld Trip 1 Telemetry (Cmd 0x14) */
static inline void psa_trip_send_trip1(psa_trip_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[11];
    p[0] = HIWORLD_SOF1;
    p[1] = HIWORLD_SOF2;
    p[2] = 0x06;               /* Length: 6 Payload bytes */
    p[3] = HIWORLD_CMD_ECU_P1; /* Cmd 0x14 */
    p[4] = (uint8_t)(ctx->state.trip1_avg_fuel >> 8);
    p[5] = (uint8_t)(ctx->state.trip1_avg_fuel & 0xFF);
    p[6] = 0x00;
    p[7] = ctx->state.trip1_avg_speed;
    p[8] = (uint8_t)(ctx->state.trip1_distance_km >> 8);
    p[9] = (uint8_t)(ctx->state.trip1_distance_km & 0xFF);

    uint8_t sum = 0;
    for (size_t i = 2; i <= 9; i++) {
        sum += p[i];
    }
    p[10] = (uint8_t)((sum - 1) & 0xFF);

    ctx->uart_tx(p, 11);
}

/* Transmit Hiworld Trip 2 Telemetry (Cmd 0x15) */
static inline void psa_trip_send_trip2(psa_trip_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[11];
    p[0] = HIWORLD_SOF1;
    p[1] = HIWORLD_SOF2;
    p[2] = 0x06;               /* Length: 6 Payload bytes */
    p[3] = HIWORLD_CMD_ECU_P2; /* Cmd 0x15 */
    p[4] = (uint8_t)(ctx->state.trip2_avg_fuel >> 8);
    p[5] = (uint8_t)(ctx->state.trip2_avg_fuel & 0xFF);
    p[6] = 0x00;
    p[7] = ctx->state.trip2_avg_speed;
    p[8] = (uint8_t)(ctx->state.trip2_distance_km >> 8);
    p[9] = (uint8_t)(ctx->state.trip2_distance_km & 0xFF);

    uint8_t sum = 0;
    for (size_t i = 2; i <= 9; i++) {
        sum += p[i];
    }
    p[10] = (uint8_t)((sum - 1) & 0xFF);

    ctx->uart_tx(p, 11);
}

/* Process PSA CAN 0x0B6 (RPM & Speed) */
static inline void psa_trip_process_can_0x0b6(psa_trip_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 4) return;
    uint16_t raw_rpm = ((uint16_t)data[0] << 8) | data[1];
    uint16_t raw_spd = ((uint16_t)data[2] << 8) | data[3];

    ctx->state.rpm = (raw_rpm == 0xFFFF) ? 0 : (raw_rpm >> 3);
    ctx->state.speed_kmh = (raw_spd == 0xFFFF) ? 0 : (raw_spd / 100);
}

/* Process PSA CAN 0x221 (Instantaneous Trip) */
static inline void psa_trip_process_can_0x221(psa_trip_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 7) return;

    ctx->state.instant_fuel_deci = ((uint16_t)data[1] << 8) | data[2];
    ctx->state.range_km          = ((uint16_t)data[3] << 8) | data[4];

    psa_trip_send_instant(ctx);
}

/* Process PSA CAN 0x2A1 (Trip 1 Historical) */
static inline void psa_trip_process_can_0x2a1(psa_trip_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 5) return;

    ctx->state.trip1_distance_km = ((uint16_t)data[1] << 8) | data[2];
    ctx->state.trip1_avg_fuel    = ((uint16_t)data[3] << 8) | data[4];
    ctx->state.trip1_avg_speed   = (dlc >= 7 && (data[5] || data[6])) ? (uint8_t)(((uint16_t)data[5] << 8) | data[6]) : data[0];

    psa_trip_send_trip1(ctx);
}

/* Process PSA CAN 0x261 (Trip 2 Historical) */
static inline void psa_trip_process_can_0x261(psa_trip_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 5) return;

    ctx->state.trip2_distance_km = ((uint16_t)data[1] << 8) | data[2];
    ctx->state.trip2_avg_fuel    = ((uint16_t)data[3] << 8) | data[4];
    ctx->state.trip2_avg_speed   = (dlc >= 7 && (data[5] || data[6])) ? (uint8_t)(((uint16_t)data[5] << 8) | data[6]) : data[0];

    psa_trip_send_trip2(ctx);
}

#endif /* CANBOX_TRIP_H */
```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: Instant Fuel 6.8 L/100km (68 = 0x0044), Range 640 km (0x0280)
- **CAN ID `0x221` Injection:**
  ```bash
  cansend vcan0 221#00004402800000
  ```
- **Expected UART Output (Hiworld `0x13`):**
  - Frame: `5A A5 04 13 00 44 02 80 DD`
  - Checksum Calculation: `(0x04 + 0x13 + 0x00 + 0x44 + 0x02 + 0x80 - 1) & 0xFF = 0xDD`
