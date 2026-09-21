# SPEC Part 2: Generic PSA & Raise Extended Protocol Specification
## Portable C99 Firmware Implementation for TPMS, ADAS, Start-Stop & Cruise Control

**Document File:** `SPEC_02_GENERIC_PSA_RAISE_EXTENDED.md`  
**Target Standard:** ISO/IEC 9899:1999 (Pure C99)  
**Target Hardware:** STM32, ESP32, Arduino/AVR (MCP2515), Nuvoton NUC131, Linux (SocketCAN `vcan0`)  
**Protocol Envelopes:** Raise (RZC `0x2E`), Hiworld (WC `0x5A 0xA5`), Bagoo (`0xD5`), Simple Soft (XP `0x2E`)

---

# Table of Contents
1. [2.1 Direct TPMS Numeric Readings & Fault Classification](#21-direct-tpms-numeric-readings--fault-classification)
2. [2.2 Stop & Start (S&S) Telemetry & Timer](#22-stop--start-ss-telemetry--timer)
3. [2.3 Cruise Control & Speed Memory Presets](#23-cruise-control--speed-memory-presets)
4. [2.4 Driver Assistance & ADAS Features](#24-driver-assistance--adas-features)
5. [Pure C99 Implementation Engine](#pure-c99-implementation-engine)
6. [Simulation & Verification Vectors](#simulation--verification-vectors)

---

# 2.1 Direct TPMS Numeric Readings & Fault Classification

### 2.1.1 Protocol Specifications & Packet Formats
- **Discrete TPMS Alarm Frame:** Raise `Cmd 0x18` (Length: 4 bytes)
- **Universal Numeric TPMS Pressures:** Raise `Cmd 0x66` (Length: 6 bytes)
- **TPMS Temperatures & Detailed Alarms:** Raise `Cmd 0x68` (Length: 8 bytes)

```
Raise 0x18 Frame (Discrete Alarms):
0x2E 0x18 0x04 [FL_Alarm] [FR_Alarm] [RL_Alarm] [RR_Alarm] [Checksum]
- 0x00: Pressure Normal (OK)
- 0x01: Pressure Low / Puncture / Sensor Fault

Raise 0x66 Frame (Direct Numeric Pressures):
0x2E 0x66 0x06 [Mode] [FL_Pres] [FR_Pres] [RL_Pres] [RR_Pres] [Unit] [Checksum]
- Byte 0 (Mode): 0x01 = Current Real-Time Pressure, 0x00 = Stored Baseline
- Bytes 1..4 (FL, FR, RL, RR): Raw pressure integer
- Byte 5 (Unit):
    * 0x00 = Bar (Scaling: Raw * 0.1 Bar, e.g. 24 = 2.4 Bar)
    * 0x01 = PSI (Scaling: Raw * 0.5 PSI, e.g. 64 = 32.0 PSI)
    * 0x02 = kPa (Scaling: Raw * 10.0 kPa, e.g. 24 = 240 kPa)

Raise 0x68 Frame (Tire Temperatures & Alarm Classification):
0x2E 0x68 0x08 [FL_Temp] [FR_Temp] [RL_Temp] [RR_Temp] [FL_Type] [FR_Type] [RL_Type] [RR_Type] [CS]
- Bytes 0..3 (FL..RR Temp): Raw + 40 (Offset = -40 deg C, Range: -40 C .. +215 C)
- Bytes 4..7 (FL..RR Type):
    * 0x00 = Normal
    * 0x01 = Low Pressure Warning
    * 0x02 = Rapid Leak / Puncture
    * 0x03 = System Offline / Sensor Signal Lost
    * 0x04 = Low Sensor Battery
```

### 2.1.2 Pure C99 Data Structure & Serializer
```c
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef struct {
    uint16_t pressure_dbar[4]; /* FL, FR, RL, RR in 0.1 Bar */
    int16_t  temperature_c[4]; /* FL, FR, RL, RR in deg C (-40..+215) */
    uint8_t  alarm_code[4];    /* 0=OK, 1=Low, 2=Puncture, 3=Lost, 4=LowBat */
} canbox_tpms_state_t;

size_t build_raise_tpms_numeric(const canbox_tpms_state_t *tpms, uint8_t *out) {
    out[0] = 0x2E;
    out[1] = 0x66;
    out[2] = 0x06;
    out[3] = 0x01; /* Real-time mode */
    out[4] = (uint8_t)tpms->pressure_dbar[0];
    out[5] = (uint8_t)tpms->pressure_dbar[1];
    out[6] = (uint8_t)tpms->pressure_dbar[2];
    out[7] = (uint8_t)tpms->pressure_dbar[3];
    out[8] = 0x00; /* Unit: Bar * 0.1 */

    uint8_t sum = 0;
    for (size_t i = 1; i <= 8; i++) sum += out[i];
    out[9] = (uint8_t)(sum ^ 0xFF);
    return 10;
}

size_t build_raise_tpms_temp_alarms(const canbox_tpms_state_t *tpms, uint8_t *out) {
    out[0] = 0x2E;
    out[1] = 0x68;
    out[2] = 0x08;
    for (int i = 0; i < 4; i++) {
        int16_t t = tpms->temperature_c[i] + 40;
        out[3 + i] = (t < 0) ? 0 : ((t > 255) ? 255 : (uint8_t)t);
    }
    for (int i = 0; i < 4; i++) {
        out[7 + i] = tpms->alarm_code[i];
    }
    uint8_t sum = 0;
    for (size_t i = 1; i <= 10; i++) sum += out[i];
    out[11] = (uint8_t)(sum ^ 0xFF);
    return 12;
}
```

---

# 2.2 Stop & Start (S&S) Telemetry & Timer

### 2.2.1 Protocol Specification
- **Raise Command ID:** `0x71` (Length: 5 bytes)
- **Hiworld Command ID:** `0x61` (Length: 5 bytes)

```
Raise 0x71 Payload Layout:
[Byte 0] System State: 0x01 = Active / Available, 0x00 = Inhibited / Deactivated
[Bytes 1..4] Cumulative Engine-Off Duration in Seconds (32-bit Big-Endian integer)
```
The Android headunit converts the 32-bit second counter into `HH:MM:SS` display format in the Eco-driving menu.

### 2.2.2 Pure C99 Implementation
```c
size_t build_raise_start_stop(bool is_active, uint32_t stop_time_sec, uint8_t *out) {
    out[0] = 0x2E;
    out[1] = 0x71;
    out[2] = 0x05;
    out[3] = is_active ? 0x01 : 0x00;
    out[4] = (uint8_t)((stop_time_sec >> 24) & 0xFF);
    out[5] = (uint8_t)((stop_time_sec >> 16) & 0xFF);
    out[6] = (uint8_t)((stop_time_sec >> 8) & 0xFF);
    out[7] = (uint8_t)(stop_time_sec & 0xFF);

    uint8_t sum = 0;
    for (size_t i = 1; i <= 7; i++) sum += out[i];
    out[8] = (uint8_t)(sum ^ 0xFF);
    return 9;
}
```

---

# 2.3 Cruise Control & Speed Memory Presets

### 2.3.1 Protocol Specification
- **Raise Command ID:** `0x72` (Length: 7 bytes)
- **Hiworld Command ID:** `0x62` (Length: 7 bytes)

```
Raise 0x72 Payload Layout:
[Byte 0] Cruise Control Status (0x01 = Active / Engaged, 0x00 = Standby / OFF)
[Byte 1] Target Cruise Speed (km/h)
[Byte 2] Memorized Speed Preset M1 (km/h, e.g. 50)
[Byte 3] Memorized Speed Preset M2 (km/h, e.g. 70)
[Byte 4] Memorized Speed Preset M3 (km/h, e.g. 90)
[Byte 5] Memorized Speed Preset M4 (km/h, e.g. 110)
[Byte 6] Memorized Speed Preset M5 (km/h, e.g. 130)
```

```c
size_t build_raise_cruise_memory(bool active, uint8_t target_spd, const uint8_t presets[5], uint8_t *out) {
    out[0] = 0x2E;
    out[1] = 0x72;
    out[2] = 0x07;
    out[3] = active ? 0x01 : 0x00;
    out[4] = target_spd;
    memcpy(&out[5], presets, 5);

    uint8_t sum = 0;
    for (size_t i = 1; i <= 9; i++) sum += out[i];
    out[10] = (uint8_t)(sum ^ 0xFF);
    return 11;
}
```

---

# 2.4 Driver Assistance & ADAS Features

### 2.4.1 Protocol Specification
- **Raise Command ID:** `0x70` (Length: 6 bytes)
- **Hiworld Command ID:** `0x60` (Length: 6 bytes)

```
Raise 0x70 Payload Layout:
[Byte 0] Blind Spot Monitoring (SAM):
         * Bit 7 (0x80): Left / Right Blind Spot Indicator Active in Side Mirrors
[Byte 1] Driver Fatigue Alert (Coffee Cup Warning):
         * Bit 7 (0x80): Rest Recommendation Warning Active
[Byte 2] Lane Departure Warning (LDW):
         * 0x00: Normal
         * 0x01: Left Line Departure Warning
         * 0x02: Right Line Departure Warning
[Byte 3] Traffic Sign Recognition (TSR Speed Limit in km/h: 30, 50, 70, 90, 110, 120, 130)
[Byte 4] ESP / Traction Control:
         * 0x01: ESP System Engaged / Intervening
         * 0x00: ESP Standby
[Byte 5] Automatic Emergency Braking (AEB) & Collision Risk:
         * 0x00: Safe
         * 0x01: Visual & Acoustic Collision Risk Alert (Level 1)
         * 0x02: Emergency Autonomous Braking Active (Level 2)
```

```c
typedef struct {
    bool    blind_spot_warning;
    bool    fatigue_coffee_cup;
    uint8_t lane_departure_state; /* 0=None, 1=Left, 2=Right */
    uint8_t speed_limit_tsr;      /* km/h */
    bool    esp_active;
    uint8_t aeb_risk_level;       /* 0=None, 1=Risk, 2=Braking */
} canbox_adas_state_t;

size_t build_raise_adas(const canbox_adas_state_t *adas, uint8_t *out) {
    out[0] = 0x2E;
    out[1] = 0x70;
    out[2] = 0x06;
    out[3] = adas->blind_spot_warning ? 0x80 : 0x00;
    out[4] = adas->fatigue_coffee_cup ? 0x80 : 0x00;
    out[5] = adas->lane_departure_state;
    out[6] = adas->speed_limit_tsr;
    out[7] = adas->esp_active ? 0x01 : 0x00;
    out[8] = adas->aeb_risk_level;

    uint8_t sum = 0;
    for (size_t i = 1; i <= 8; i++) sum += out[i];
    out[9] = (uint8_t)(sum ^ 0xFF);
    return 10;
}
```

---

# Pure C99 Implementation Engine

```c
#include "canbox_hal.h"

void canbox_extended_telemetry_tick(const canbox_tpms_state_t *tpms,
                                    const canbox_adas_state_t *adas,
                                    bool ss_active, uint32_t ss_time,
                                    canbox_uart_id_t uart_id) {
    uint8_t tx_buf[32];
    size_t len;

    /* 1. Direct Numeric TPMS */
    len = build_raise_tpms_numeric(tpms, tx_buf);
    g_hal->uart_send(tx_buf, len);

    /* 2. ADAS Systems */
    len = build_raise_adas(adas, tx_buf);
    g_hal->uart_send(tx_buf, len);

    /* 3. Stop & Start */
    len = build_raise_start_stop(ss_active, ss_time, tx_buf);
    g_hal->uart_send(tx_buf, len);
}
```

---

# Simulation & Verification Vectors

```bash
# 1. TPMS: 4 Wheels @ 2.4 Bar (24 dec = 0x18 hex):
# Expected Raise Frame: 2E 66 06 01 18 18 18 18 00 90

# 2. ADAS: TSR 90 km/h Limit, Blind Spot Active:
# Expected Raise Frame: 2E 70 06 80 00 00 5A 00 00 B9

# 3. Stop & Start: Active, 125 seconds engine off (0x0000007D):
# Expected Raise Frame: 2E 71 05 01 00 00 00 7D 0C
```

