### Architectural Overview in `canbox-core`

```
+-------------------------------------------------------------------------+
|                  Android Head Unit (Raise / Hiworld)                    |
|  Touchscreen UI: AC Mono/Dual button, Vehicle Settings (DRL, Auto-Lock) |
+-------------------------------------------------------------------------+
       | Downlink UART (e.g., Hiworld 0x3B / Raise 0x8A)  ^ Uplink UART (0x31)
       v                                                  |
+-------------------------------------------------------------------------+
|                        canbox-core (MCU / Linux)                        |
|  - Downlink Parser: maps HU serial command -> PSA CAN frame injection   |
|  - Vehicle Profile: decodes PSA CAN frames -> vehicle_state_t           |
|  - Uplink Serializer: pushes state changes to HU                        |
+-------------------------------------------------------------------------+
       | hal_can_send()                                   ^ CAN IRQ / RX FIFO
       v                                                  |
+-------------------------------------------------------------------------+
|                       PSA CAN 2004 Comfort Bus                          |
|         BSI (0x361 / 0x260) <-----> Climate ECU (0x1D0 / 0x1E3)         |
+-------------------------------------------------------------------------+
```

---

## Part 1: Climate Control — Reading & Changing Mono / Dual Mode

In PSA CAN 2004, climate communication uses two primary frames:
- **`0x1E3` (`MSG_ETAT_CLIMAV_EMF`)**: Telemetry broadcasted by Climate ECU to RT4/Display (100 ms period). **Used to READ status**.
- **`0x1D0` (`MSG_CDE_CLIMAV`)**: Command frame sent by the control fascia / head unit to Climate ECU and BSI. **Used to CHANGE settings**.

---

### 1.1 Reading Climate State & Mono/Dual Mode (`0x1E3`)

The Climate ECU broadcasts `0x1E3` every 100 ms (DLC = 7):

| Byte | Bits | Field Name | Description |
|:---:|:---:|:---|:---|
| **0** | **0** | **`DUAL_ACTIVE`** | **`1` = Dual Mode** (Independent driver/pass temp)<br>**`0` = Mono / SYNC Mode** (Synchronized) |
| 0 | 6 | `AC_OFF` | `1` = A/C Compressor OFF (ECO mode), `0` = A/C ON |
| 0 | 5 | `CLIM_OFF` | `1` = HVAC System Powered OFF, `0` = Powered ON |
| 0 | 4 | `AUTO_AIRFLOW` | `1` = Automatic air distribution active |
| 0 | 3 | `AUTO_BLOWER` | `1` = Automatic fan speed active |
| 0 | 7 | `RECIRC_STATUS`| `1` = Cabin air recirculation ON |
| 1 | 7 | `DEFROST_FRONT`| `1` = Max windshield demisting ON |
| **2** | 7:0 | **`DISPLAY_TEMP_L`** | Driver setpoint: `0x00`=LO, `0x01..0x15` ($14.0 + \text{raw} \times 0.5\ ^\circ\text{C}$), `0x16`=HI |
| **3** | 7:0 | **`DISPLAY_TEMP_R`** | Passenger setpoint (In Mono mode, always equals `DISPLAY_TEMP_L`) |
| 4 | 7:4 | `AIR_FLOW_L` | Driver airflow direction (1=Screen, 2=Vents, 3=Feet, 4=Screen+Feet, 5=Vents+Feet) |
| 5 | 7:4 | `AIR_FLOW_R` | Passenger airflow direction |
| 6 | 3:0 | `FAN_SPEED` | Blower speed level (`0` = Off, `1..7`, `15` = Max) |

#### C99 Decoder for `peugeot_407.c`:
```c
void psa_decode_climate_0x1e3(const uint8_t *data, uint8_t dlc, vehicle_climate_t *climate) {
    if (!climate || !data || dlc < 4) return;

    /* Mono vs Dual mode: Byte 0, Bit 0 */
    climate->dual_mode = (data[0] & 0x01) != 0;

    climate->power_on   = (data[0] & 0x20) == 0; // Bit 5 is CLIM_OFF
    climate->ac_on      = (data[0] & 0x40) == 0; // Bit 6 is AC_OFF
    climate->auto_mode  = (data[0] & 0x08) != 0; // Bit 3 is AUTO_BLOWER
    climate->recirculate= (data[0] & 0x80) != 0;

    /* Temperature setpoints */
    climate->temp_driver    = data[2];
    climate->temp_passenger = data[3];

    if (dlc >= 7) {
        climate->fan_speed = data[6] & 0x0F;
    }
}
```

---

### 1.2 Changing Mono / Dual Mode & Climate Commands (`0x1D0`)

To change climate settings, your device sends frame **`0x1D0`** (DLC = 7) onto CAN CONF:

| Byte | Bits | Field Name | Mask / Value |
|:---:|:---:|:---|:---|
| **0** | **0** | **`DUAL_REQ`** | **`0x01` = Request Dual / Mono Toggle** |
| 0 | 3 | `AUTO_REQ` | `0x08` = Request AUTO mode |
| 0 | 4 | `AC_OFF_REQ` | `0x10` = Request A/C OFF toggle |
| 0 | 5 | `RECIRC_REQ` | `0x20` = Request Recirculation toggle |
| 0 | 7 | `DEFROST_MAX` | `0x80` = Request Max Defrost |
| 1 | 7:0 | `RESERVED` | `0x00` |
| 2 | 3:0 | `FAN_SPEED` | Desired fan speed (`0..7`, `0xF` for max, `0` for no change) |
| 3 | 7:0 | `AIR_DIR` | `(left_dir << 4) | (right_dir & 0x0F)` |
| 4 | 7:0 | `RECIRC_DEFROST`| Recirculation mode & rear defrost |
| 5 | 7:0 | `TEMP_DRIVER` | Driver setpoint (`0x00`=LO, `0x01..0x15`, `0x16`=HI) |
| 6 | 7:0 | `TEMP_PASS` | Passenger setpoint (**Must match Byte 5 when commanding Mono mode**) |

#### C99 Injection Function:
```c
#include "hal/hal_can.h"

hal_status_t psa_climate_set_dual_mode(bool enable_dual, uint8_t driver_temp, uint8_t pass_temp) {
    can_frame_t frame;
    memset(&frame, 0, sizeof(frame));

    frame.id  = 0x1D0;
    frame.dlc = 7;

    /* Bit 0 is DUAL_REQ */
    frame.data[0] = 0x01; 
    frame.data[1] = 0x00;
    frame.data[2] = 0x00; // Keep current blower
    frame.data[3] = 0x00; // Keep distribution
    frame.data[4] = 0x00;

    frame.data[5] = driver_temp;
    /* When forcing Mono, passenger setpoint must follow driver */
    frame.data[6] = enable_dual ? pass_temp : driver_temp;

    return hal_can_send(&frame);
}
```

---

### 1.3 How to Check If Climate Change Was Applied

1. **CAN Feedback**: Monitor incoming `0x1E3` messages from the Climate ECU.
   - Within **100 ms – 200 ms** (1 to 2 cycles), Byte 0 Bit 0 will transition:
     $$\text{Dual Active} \implies (\text{data}[0] \ \& \ 0x01) == 1$$
     $$\text{Mono Active} \implies (\text{data}[0] \ \& \ 0x01) == 0$$
   - In Mono mode, verify that `data[3]` (Passenger temp) immediately equals `data[2]` (Driver temp).
2. **Android HU State Push**: When `state->climate.dual_mode` updates, `can_router.c` detects the change and pushes the updated packet to Android (Hiworld `0x31`, Byte 4, Bit 2).

---

## Part 2: Vehicle Personalization Configuration

Vehicle configuration (lighting, locking, wipers) is stored in the **BSI (Body Computer)** and synchronized via CAN ID **`0x361`** (`MSG_BSI_INF_CFG_MENU_PERSO`).

---

### 2.1 Reading Current Car Configuration (`0x361`)

The BSI broadcasts `0x361` periodically every **250 ms** (DLC = 6):

```
Byte 0: [ D7 | D6 | D5 | D4 | D3 | D2 | D1 | D0 ]
          |    |    |    |    |    |    |    +-- MOTORIZED_BOOT (Tailgate mode)
          |    |    |    |    |    |    +------- DIR_HEADLIGHTS (Directional Xenon)
          |    |    |    |    |    +------------ SELECTIVE_BOOT (Selective unlocking)
          |    |    |    |    +----------------- AUTO_DOOR_LOCK (Drive lock > 10 km/h)
          |    |    |    +---------------------- Reserved (0)
          |    |    +--------------------------- REAR_WIPER_REV (Rear wipe on reverse)
          |    +-------------------------------- DRL_ENABLE (Daytime running lights)
          +------------------------------------- GUIDE_ME_HOME (Follow-me-home lights)

Byte 1: Bits 3:0 = GUIDE_HOME_TIME (0 = 0s, 1 = 15s, 2 = 30s, 3 = 60s)
Byte 2: AMBIENT_LIGHTING (Footwell / mood brightness 0 to 15)
Byte 3: PROFILE_ID (Active profile: 1, 2, or 3)
```

#### C99 Configuration Structure & Reader:
```c
typedef struct {
    bool    follow_me_home;
    bool    daytime_running_lights;
    bool    rear_wiper_reverse;
    bool    auto_door_lock;
    bool    selective_boot_unlock;
    bool    directional_headlights;
    uint8_t follow_me_home_delay_sec; // 0, 15, 30, 60
    uint8_t ambient_lighting_level;   // 0..15
    uint8_t profile_id;               // 1..3
} psa_car_config_t;

static psa_car_config_t s_car_config;
static uint8_t s_cached_0x361_raw[6];

void psa_decode_config_0x361(const uint8_t *data, uint8_t dlc) {
    if (!data || dlc < 4) return;

    memcpy(s_cached_0x361_raw, data, (dlc < 6) ? dlc : 6);

    s_car_config.follow_me_home         = (data[0] & 0x80) != 0;
    s_car_config.daytime_running_lights = (data[0] & 0x40) != 0;
    s_car_config.rear_wiper_reverse     = (data[0] & 0x20) != 0;
    s_car_config.auto_door_lock         = (data[0] & 0x08) != 0;
    s_car_config.selective_boot_unlock  = (data[0] & 0x04) != 0;
    s_car_config.directional_headlights = (data[0] & 0x02) != 0;

    uint8_t delay_code = data[1] & 0x0F;
    s_car_config.follow_me_home_delay_sec = (delay_code == 1) ? 15 : (delay_code == 2) ? 30 : (delay_code == 3) ? 60 : 0;
    s_car_config.ambient_lighting_level   = data[2] & 0x0F;
    s_car_config.profile_id               = data[3];
}
```

---

### 2.2 Changing Car Configuration Settings

As verified in the reverse-engineered RT4 routine [`set_mmi_config`](#L143-L162), changing a configuration parameter requires transmitting `0x361` with the modified bits onto the CAN bus. The BSI receives this frame, commits the value to NVRAM, and begins broadcasting the updated state.

#### C99 Config Writer Function:
```c
hal_status_t psa_car_config_set_option(uint8_t bit_mask, bool enable) {
    can_frame_t frame;
    memset(&frame, 0, sizeof(frame));

    frame.id  = 0x361;
    frame.dlc = 6;

    /* Start from cached live BSI state */
    memcpy(frame.data, s_cached_0x361_raw, 6);

    /* Modify the requested bit */
    if (enable) {
        frame.data[0] |= bit_mask;
    } else {
        frame.data[0] &= (uint8_t)(~bit_mask);
    }

    return hal_can_send(&frame);
}

// Convenience helpers:
hal_status_t psa_set_drl(bool enable) {
    return psa_car_config_set_option(0x40, enable);
}

hal_status_t psa_set_auto_door_lock(bool enable) {
    return psa_car_config_set_option(0x08, enable);
}

hal_status_t psa_set_rear_wiper_reverse(bool enable) {
    return psa_car_config_set_option(0x20, enable);
}

hal_status_t psa_set_follow_me_home(bool enable, uint8_t seconds) {
    can_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.id = 0x361;
    frame.dlc = 6;
    memcpy(frame.data, s_cached_0x361_raw, 6);

    if (enable) {
        frame.data[0] |= 0x80;
        uint8_t code = (seconds >= 60) ? 3 : (seconds >= 30) ? 2 : 1;
        frame.data[1] = (frame.data[1] & 0xF0) | code;
    } else {
        frame.data[0] &= ~0x80;
        frame.data[1] &= 0xF0;
    }

    return hal_can_send(&frame);
}
```

---

### 2.3 How to Check If Car Configuration Change Was Applied

1. **BSI Echo Verification**:
   - The BSI processes the transmitted command and broadcasts the updated values in its next periodic `0x361` frame within **250 ms**.
   - Your driver compares the received `0x361` with the desired state:
     ```c
     bool verified = ((received_data[0] & bit_mask) != 0) == target_state;
     ```
2. **Timeout Handling**:
   - Start a software timer for **600 ms** (allows up to 2 BSI broadcast periods). If the bit has not updated within this window, report a CAN write timeout.
3. **Physical Verification**:
   - **DRL**: Start engine with light stalk at `0`; front parking/DRL lights illuminate.
   - **Auto-Lock**: Drive beyond 10 km/h; door locks trigger.
   - **Reverse Wiper**: Turn on front wipers, engage reverse; rear wiper sweeps.

---

## Part 3: Handling Android Downlink Commands in `canbox-core`

When user clicks on the Android Head Unit touchscreen, the HU sends a serial packet over UART to your MCU:

### A. Hiworld Protocol (`5A A5`)
- **Mono / Dual Sync Toggle**:
  - Android sends Command `0x3B` (`ForwardAcSetting`), length `0x02`, data `[0x0F, val]`.
  - When `val == 0x01` $\rightarrow$ SYNC ON (Mono mode).
  - When `val == 0x00` $\rightarrow$ SYNC OFF (Dual mode).
- **MCU Action**:
  ```c
  if (cmd_id == 0x3B && len >= 2 && payload[0] == 0x0F) {
      bool mono_mode = (payload[1] == 0x01);
      psa_climate_set_dual_mode(!mono_mode, climate->temp_driver, climate->temp_passenger);
  }
  ```

### B. Raise Protocol (`0x2E`)
- **Mono / Dual Toggle**:
  - Android sends Command `0x8A`, length `0x02`, data `[0x0B, 0x01]`.
- **MCU Action**:
  ```c
  if (cmd_id == 0x8A && len >= 2 && payload[0] == 0x0B) {
      bool dual_target = (payload[1] == 0x01);
      psa_climate_set_dual_mode(dual_target, climate->temp_driver, climate->temp_passenger);
  }
  ```

---

### Summary Checklist for Your Implementation

| Operation | Action | CAN Frame | Byte/Bit | Verification |
|---|---|---|---|---|
| **Read Climate State** | Listen to CAN | `0x1E3` (100 ms) | Byte 0, Bit 0 (`0x01`) | `1` = Dual, `0` = Mono |
| **Change Mono / Dual** | Send CAN | `0x1D0` (Event) | Byte 0, Bit 0 (`0x01`), Byte 5/6 temps | Next `0x1E3` reflects new Bit 0 within 100 ms |
| **Read Car Config** | Listen to CAN | `0x361` (250 ms) | Byte 0 (DRL, Auto-Lock, Follow-Me, Wiper) | Cached in MCU memory |
| **Change Car Config** | Send CAN | `0x361` (Event) | Cache modified bit $\rightarrow$ transmit | Next `0x361` reflects updated bits within 250 ms |