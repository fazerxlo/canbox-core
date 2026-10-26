# SPEC Part 3: Vehicle Configuration & Central Settings Specification
## Portable C99 Firmware Implementation for PSA BSI Parameters & Measurement Units

**Document File:** `SPEC_03_VEHICLE_CONFIG_BSI_SETTINGS.md`  
**Target Standard:** ISO/IEC 9899:1999 (Pure C99)  
**Target Hardware:** STM32, ESP32, Arduino/AVR, Nuvoton NUC131, Linux (SocketCAN `vcan0` + POSIX PTY)  
**Vehicle System:** PSA Built-in Systems Interface (BSI Central ECU)  
**Protocol Envelopes:** Raise (RZC `0x2E`), Hiworld (WC `0x5A 0xA5`), Bagoo (`0xD5`), Simple Soft (XP `0x2E`)

---

# Table of Contents
1. [3.1 Lighting & Visibility Settings](#31-lighting--visibility-settings)
2. [3.2 Locking & Electric Mirrors Configuration](#32-locking--electric-mirrors-configuration)
3. [3.3 Wiper & Driving Assistance Settings](#33-wiper--driving-assistance-settings)
4. [3.4 Display & Measurement Units Customization](#34-display--measurement-units-customization)
5. [Pure C99 Architecture & State Serializer](#pure-c99-architecture--state-serializer)
6. [Desktop Test Vectors & CAN Injection Guide](#desktop-test-vectors--can-injection-guide)

---

# 3.1 Lighting & Visibility Settings

### 3.1.1 CAN Frame & Raise Protocol Framing
- **PSA CAN ID:** `0x221` (DLC: 8, Cycle: 100 ms) & `0x341` (BSI Configuration status)
- **Raise Command ID:** `0x38` (Central Configuration Frame, Length: 8 bytes)

```
Raise 0x38 Lighting Bit Positions:
[Byte 2]
  - Bit 7 (0x80): Daytime Running Lights (DRL) Active (1 = ON, 0 = OFF)
  - Bit 0 (0x01): Adaptive Cornering / Directional Headlights (1 = ON, 0 = OFF)
[Byte 3]
  - Bits 3..0 (0x0F): Ambient Mood Lighting Brightness Level (0..15)
[Byte 4]
  - Bits 7..6 (0xC0): Follow-Me-Home Headlight Delay:
      * 00b (0x00): 0 seconds (Disabled)
      * 01b (0x40): 15 seconds
      * 10b (0x80): 30 seconds
      * 11b (0xC0): 60 seconds
  - Bits 5..4 (0x30): Welcome / Greeting Lighting Duration:
      * 00b: 0 seconds
      * 01b: 15 seconds
      * 10b: 30 seconds
```

---

# 3.2 Locking & Electric Mirrors Configuration

### 3.2.1 Protocol Mapping & Control Flags
- **Raise Command ID:** `0x38`

```
Raise 0x38 Locking & Mirrors Bit Positions:
[Byte 1]
  - Bit 4 (0x10): Automatic Central Door Locking upon driving (> 10 km/h) (1=ON, 0=OFF)
[Byte 4]
  - Bit 3 (0x08): Automatic Electric Mirror Folding on Key Lock / Unlock (1=ON, 0=OFF)
[Byte 7]
  - Bit 7 (0x80): Selective Unlocking Mode:
      * 1: Driver Door Only unlocked on first key click
      * 0: All Doors unlocked simultaneously
  - Bit 2 (0x04): Motorized Boot / Tailgate Hands-Free Kick Sensor (1=Enabled, 0=Disabled)
```

---

# 3.3 Wiper & Driving Assistance Settings

### 3.3.1 Protocol Mapping
```
Raise 0x38 Wiper & Driving Bit Positions:
[Byte 1]
  - Bit 7 (0x80): Automatic Rear Window Wiper engagement when reversing (1=Active, 0=Disabled)
[Byte 7]
  - Bit 0 (0x01): Automatic Rain Sensing Windshield Wipers Enable (1=ON, 0=OFF)
```

---

# 3.4 Display & Measurement Units Customization

### 3.4.1 Protocol Mapping
- **Raise Command ID:** `0x39` (Units Telemetry, Length: 2 bytes) or embedded in `0x21` / `0x38`.

```
Raise 0x39 Payload Layout:
[Byte 0]
  - Bits 7..6: Distance Measurement Unit:
      * 00b (0x00): Kilometers (km)
      * 01b (0x40): Statute Miles (mi)
  - Bits 5..4: Fuel Consumption Economy Unit:
      * 00b (0x00): Liters per 100 km (L/100km)
      * 01b (0x10): Kilometers per Liter (km/L)
      * 10b (0x20): Miles per Gallon US (MPG US)
      * 11b (0x30): Miles per Gallon UK (MPG UK)
[Byte 1]
  - Bit 7: Temperature Measurement Unit:
      * 0: Celsius (deg C)
      * 1: Fahrenheit (deg F)
  - Bits 5..4: Tire Pressure Measurement Unit:
      * 00b (0x00): Bar
      * 01b (0x10): PSI
      * 10b (0x20): Kilopascals (kPa)
```

---

# Pure C99 Architecture & State Serializer

```c
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef struct {
    /* Lighting */
    bool    drl_active;
    bool    cornering_lights;
    uint8_t follow_me_home_sec; /* 0, 15, 30, 60 */
    uint8_t welcome_light_sec;  /* 0, 15, 30 */
    uint8_t mood_light_lvl;     /* 0..15 */

    /* Locking & Mirrors */
    bool    auto_lock_driving;
    bool    auto_mirror_folding;
    bool    selective_unlock_driver;
    bool    motorized_boot_kick;

    /* Wipers */
    bool    rear_wiper_in_reverse;
    bool    rain_sensor_wipers;

    /* Measurement Units */
    uint8_t unit_distance; /* 0=km, 1=miles */
    uint8_t unit_fuel;     /* 0=L/100km, 1=km/L, 2=MPG US, 3=MPG UK */
    uint8_t unit_temp;     /* 0=C, 1=F */
    uint8_t unit_pressure; /* 0=Bar, 1=PSI, 2=kPa */
} bsi_vehicle_config_t;

size_t build_raise_bsi_config(const bsi_vehicle_config_t *cfg, uint8_t *out) {
    out[0] = 0x2E;
    out[1] = 0x38;
    out[2] = 0x08; /* Length */
    memset(&out[3], 0, 8);

    /* Byte 1 */
    if (cfg->rear_wiper_in_reverse) out[4] |= 0x80;
    if (cfg->auto_lock_driving)     out[4] |= 0x10;

    /* Byte 2 */
    if (cfg->drl_active)        out[5] |= 0x80;
    if (cfg->cornering_lights)  out[5] |= 0x01;

    /* Byte 3 */
    out[6] |= (cfg->mood_light_lvl & 0x0F);

    /* Byte 4 */
    if (cfg->follow_me_home_sec == 60)      out[7] |= 0xC0;
    else if (cfg->follow_me_home_sec == 30) out[7] |= 0x80;
    else if (cfg->follow_me_home_sec == 15) out[7] |= 0x40;

    if (cfg->welcome_light_sec == 30)      out[7] |= 0x20;
    else if (cfg->welcome_light_sec == 15) out[7] |= 0x10;

    if (cfg->auto_mirror_folding) out[7] |= 0x08;

    /* Byte 7 */
    if (cfg->selective_unlock_driver) out[10] |= 0x80;
    if (cfg->motorized_boot_kick)     out[10] |= 0x04;
    if (cfg->rain_sensor_wipers)      out[10] |= 0x01;

    uint8_t sum = 0;
    for (size_t i = 1; i <= 10; i++) sum += out[i];
    out[11] = (uint8_t)(sum ^ 0xFF);
    return 12;
}

size_t build_raise_units_config(const bsi_vehicle_config_t *cfg, uint8_t *out) {
    out[0] = 0x2E;
    out[1] = 0x39;
    out[2] = 0x02;
    out[3] = ((cfg->unit_distance & 0x03) << 6) | ((cfg->unit_fuel & 0x03) << 4);
    out[4] = ((cfg->unit_temp & 0x01) << 7) | ((cfg->unit_pressure & 0x03) << 4);

    uint8_t sum = 0;
    for (size_t i = 1; i <= 4; i++) sum += out[i];
    out[5] = (uint8_t)(sum ^ 0xFF);
    return 6;
}
```

---

# Desktop Test Vectors & CAN Injection Guide

```bash
# 1. Enable DRL, Follow-Me-Home 30s, Mirror Folding, Auto Lock:
# Expected Raise Frame: 2E 38 08 00 10 80 00 88 00 00 00 E7

# 2. Set Measurement Units: Miles, MPG US, Fahrenheit, PSI:
# Expected Raise Frame: 2E 39 02 60 90 A4
```

