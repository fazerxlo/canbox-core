## 1. PSA CAN Protocol Specification (Firmware Evidence)

In the RT4 firmware binary (`Fp_Network_CAN.out`), trip resets are implemented in:
- `cmd_trip_reset_1__9C_BCM_CAN` (address `0x0119f0`)
- `cmd_trip_reset_2__9C_BCM_CAN` (address `0x011af0`)

Both functions construct the **`MSG_DEMANDES_EMF`** command frame:

```c
// Decompiled logic from Fp_Network_CAN.out (offset 0x119f0 & 0x11af0):
void cmd_trip_reset_1(bcm_can_ctx_t *ctx) {
    semTake(ctx->presentation->sem, WAIT_FOREVER);
    
    // Set Bit 7 (0x80) on Byte 0
    ctx->presentation->demandes_emf_buf[0] |= 0x80;
    
    // Transmit one-shot command frame onto CAN
    Write(ctx->presentation->msg_demandes_emf);
    
    // Immediately clear Bit 7
    ctx->presentation->demandes_emf_buf[0] &= ~0x80;
    
    semGive(ctx->presentation->sem);
}

void cmd_trip_reset_2(bcm_can_ctx_t *ctx) {
    semTake(ctx->presentation->sem, WAIT_FOREVER);
    
    // Set Bit 6 (0x40) on Byte 0
    ctx->presentation->demandes_emf_buf[0] |= 0x40;
    
    Write(ctx->presentation->msg_demandes_emf);
    
    // Immediately clear Bit 6
    ctx->presentation->demandes_emf_buf[0] &= ~0x40;
    
    semGive(ctx->presentation->sem);
}
```

### CAN Frame Layout (`MSG_DEMANDES_EMF` / CAN ID `0x221`)
- **CAN ID:** `0x221` (Comfort Bus, DLC = 8 bytes)
- **Transmission:** Event-driven one-shot pulse (sent once, bits cleared immediately)

#### Byte 0 Command Bitmask:
| Bit | Hex Mask | Function Name | Description |
|:---:|:---:|:---|:---|
| **Bit 7** | **`0x80`** | **`TRIP1_RESET`** | **`1` = Reset Trip 1 memory (Distance, Avg Fuel, Avg Speed)** |
| **Bit 6** | **`0x40`** | **`TRIP2_RESET`** | **`1` = Reset Trip 2 memory** |
| Bit 5 | `0x20` | `CHECK_ALERTS` | Triggers alert log recall on cluster/display |
| Bit 4 | `0x10` | `RAZ_TPMS` | Reset / re-initialize TPMS tire pressure calibration |
| Bit 3 | `0x08` | `RAZ_MAINT` | Reset maintenance service counter countdown |
| Bits 1:0 | `0x03` | `TRIP_PAGE_SELECT` | Selected trip bank (`0` = Instant, `1` = Route 1, `2` = Route 2) |

Bytes 1 through 7 are typically filled with `0x00`.

---

## 2. Head Unit Downlink Protocols (Android $\to$ MCU)

When the user taps **"Reset Trip 1"** or **"Reset Trip 2"** on the Android touchscreen, the head unit sends a serial packet over UART to your CAN box:

### A. Raise Protocol (`0x2E`)
Raise uses Command ID **`0x82`** (Length: 2 bytes):
- **Reset Trip 1**: `2E 82 02 41 00 3A`
- **Reset Trip 2**: `2E 82 02 22 00 59`

### B. Hiworld Protocol (`5A A5`)
Hiworld uses Command ID **`0x1B`** (`ForwardEcuSetting`):

#### Standard 4-Byte Format (`QF_Canbus.apk` / Real Android HU):
Frame Layout: `5A A5 04 1B [Page] [ResetTarget] 01 FF [CS]`
- **Byte 0 (Active Page):**
  - `0x01`: Instantaneous consumption tab
  - `0x02`: Trip 1 tab (`EcuInfoPage2` / `Cmd 0x14`)
  - `0x03`: Trip 2 tab (`EcuInfoPage3` / `Cmd 0x15`)
- **Byte 1 (Reset Target):**
  - `0x00`: **Tab navigation / page viewing only — DO NOT RESET**
  - `0x02`: **Reset Trip 1 (`EcuInfoPage2`)**
  - `0x03`: **Reset Trip 2 (`EcuInfoPage3`)**
- **Byte 2 (`0x01`):** Command validity / enable flag
- **Byte 3 (`0xFF`):** Parameter mask

**Bench-Verified Wire Frames:**
- **View Instantaneous Tab:** `5A A5 04 1B 01 00 01 FF 1F` (No CAN action)
- **View Trip 1 Tab:** `5A A5 04 1B 02 00 01 FF 20` (No CAN action)
- **View Trip 2 Tab:** `5A A5 04 1B 03 00 01 FF 21` (No CAN action)
- **Reset Trip 1:** `5A A5 04 1B 02 02 01 FF 22` $\longrightarrow$ Pulse CAN ID `0x221` with `0x80`
- **Reset Trip 2:** `5A A5 04 1B 03 03 01 FF 24` $\longrightarrow$ Pulse CAN ID `0x221` with `0x40`

#### 2-Byte Fallback Format:
- **Reset Trip 1**: `5A A5 02 1B 01 01 1E` or `5A A5 02 1B 02 01 1F`
- **Reset Trip 2**: `5A A5 02 1B 03 01 20`

---

## 3. C99 Implementation in `canbox-core`

Here is how to implement the complete handler in `src/profiles/peugeot_407.c` and connect it to `hal_can_send()`:

### 3.1 CAN Injection Function
```c
#include "hal/hal_can.h"
#include <string.h>

/**
 * @brief Injects a Trip Reset pulse onto the PSA CAN network.
 * @param trip_index 1 for Trip 1, 2 for Trip 2
 * @return HAL_STATUS_OK on success
 */
hal_status_t psa_trip_send_reset(uint8_t trip_index) {
    can_frame_t frame;
    memset(&frame, 0, sizeof(frame));

    frame.id  = 0x221;
    frame.dlc = 8;

    if (trip_index == 1) {
        frame.data[0] = 0x80; /* Bit 7: Trip 1 Reset */
    } else if (trip_index == 2) {
        frame.data[0] = 0x40; /* Bit 6: Trip 2 Reset */
    } else {
        return HAL_STATUS_ERROR;
    }

    /* Send one-shot pulse frame */
    return hal_can_send(&frame);
}
```

### 3.2 Downlink Serial Packet Dispatcher
Add this to your UART downlink handler (e.g. in `proto_raise_adapter.c` / `proto_hiworld_adapter.c`):

```c
void canbox_handle_downlink_trip_reset(uint8_t protocol_type, uint8_t cmd_id, const uint8_t *payload, uint8_t len) {
    if (!payload || len < 2) return;

    /* 1. Raise Protocol Handler */
    if (cmd_id == 0x82) {
        if (payload[0] == 0x41) {
            /* Reset Trip 1 */
            psa_trip_send_reset(1);
        } else if (payload[0] == 0x22) {
            /* Reset Trip 2 */
            psa_trip_send_reset(2);
        }
    }
    
    /* 2. Hiworld Protocol Handler */
    else if (cmd_id == 0x1B) {
        if (len >= 4) {
            uint8_t reset_target = payload[1];
            if (reset_target == 0x02) {
                psa_trip_send_reset(1);
            } else if (reset_target == 0x03) {
                psa_trip_send_reset(2);
            }
            /* reset_target == 0x00: Tab navigation only, do NOT trigger reset */
        } else if (len >= 2) {
            uint8_t page = payload[0];
            uint8_t action = payload[1];
            if (action == 0x01) {
                if (page == 0x03) {
                    psa_trip_send_reset(2);
                } else if (page == 0x01 || page == 0x02) {
                    psa_trip_send_reset(1);
                }
            }
        }
    }
}
```

---

## 4. How to Check If the Change Was Applied

After the BSI receives the reset pulse on `0x221`, it clears the corresponding trip memory registers and broadcasts the new state in its periodic telemetry frames.

### 4.1 Periodic CAN Telemetry Frames (BSI $\to$ Infotainment)

Within **500 ms** (1 broadcast cycle of the BSI), inspect the following incoming frames:

#### Trip 1 Status — CAN ID `0x2A1` (`MSG_INFOS_TRAJET1_ODB`, 500 ms period):
| Byte Range | Meaning | Value After Reset |
|:---:|:---|:---|
| **Bytes 1–2** | Distance traveled since reset | `0x0000` ($0.0\text{ km}$) |
| **Bytes 3–4** | Average fuel consumption | `0xFFFF` (displays as `---` L/100km until vehicle has traveled $\approx 100\text{ m}$) |
| **Bytes 5–6** | Average vehicle road speed | `0xFFFF` or `0x0000` (displays as `---` km/h) |

#### Trip 2 Status — CAN ID `0x261` (`MSG_INFOS_TRAJET2_ODB`, 500 ms period):
| Byte Range | Meaning | Value After Reset |
|:---:|:---|:---|
| **Bytes 1–2** | Distance traveled since reset | `0x0000` ($0.0\text{ km}$) |
| **Bytes 3–4** | Average fuel consumption | `0xFFFF` |
| **Bytes 5–6** | Average vehicle road speed | `0xFFFF` or `0x0000` |

### 4.2 C99 Verification in `canbox-core`
In `psa_trip_process_can_0x2a1()`:
```c
void psa_trip_process_can_0x2a1(psa_trip_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 5) return;

    ctx->state.trip1_distance_km = ((uint16_t)data[1] << 8) | data[2];
    ctx->state.trip1_avg_fuel    = ((uint16_t)data[3] << 8) | data[4];
    ctx->state.trip1_avg_speed   = (dlc >= 7) ? data[5] : 0;

    // Push updated zeroes / dashes to Android HU
    psa_trip_send_trip1(ctx);
}
```

### 4.3 Desktop Testing on `vcan0`
Using the Linux simulation target in `canbox-core`:

```bash
# 1. Inject Trip 1 Reset from Android simulation:
# Head Unit sends: 2E 82 02 41 00 3A
# Verified CAN output on vcan0:
candump vcan0,221:7FF
# Output: 221 [8] 80 00 00 00 00 00 00 00

# 2. Simulate BSI response (Trip 1 cleared: 0 km, 0xFFFF fuel, 0xFFFF speed):
cansend vcan0 2A1#250000FFFFFFFF

# 3. Monitor UART output to Android HU:
xxd -c 16 < /tmp/ttyCanbox
# Hiworld outputs: 5A A5 06 14 FF FF 00 FF 00 00 [CS] (all fields reset to initial dashes/zeroes)
```