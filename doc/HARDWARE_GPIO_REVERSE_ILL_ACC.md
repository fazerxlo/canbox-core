# Hardware GPIO Specification: REVERSE, ILL (Illumination), and ACC (Accessory)

**Document File:** `HARDWARE_GPIO_REVERSE_ILL_ACC.md`  
**Target Architecture:** Pure C99 Embedded CAN Translator & Desktop Simulator (`canbox-core`)  
**Target Hardware:** STM32F103 (BluePill / OEM CAN Box) & ESP32-WROOM-32  
**Vehicle Integration:** Peugeot 407 / PSA CAN2004 & Generic Automotive CAN Bus  
**Document Version:** 1.0  
**Date:** 2026-09-29  

---

# 1. Executive Summary & Functional Domain

Modern aftermarket Android Head Units (e.g., UIS7862, TS10, Topway, Allwinner T507) use a hybrid interface approach:
1. **Digital UART Stream (Rx/Tx):** Serial packets (Hiworld, Raise, Bagoo) for non-critical telemetry, door status, HVAC, steering wheel key events, and trip computer data.
2. **Physical Dedicated GPIO Lines (+12V / GND logic):** Three discrete, low-latency analog/digital trigger wires:
   - **REVERSE (BACK / CAM_LINE):** Instantaneous hardware switch to reverse camera video feed.
   - **ILL (ILLUMINATION):** Nighttime backlight dimming and button illumination.
   - **ACC (ACCESSORY / 12V SWITCHED):** Operating system wake, run, and sleep power-sequencing trigger.

In many modern vehicles (including the Peugeot 407 CAN2004 architecture), physical ACC, ILL, and REVERSE analog signals do **not** exist at the factory radio harness (which only provides permanent $+12\text{V}$ BATT and Chassis GND). The CAN box micro-controller must synthesize these physical $+12\text{V}$ signals from real-time CAN bus frames and drive dedicated GPIO lines.

```
+─────────────────────────────────────────────────────────────────────────────────────────+
│                                  Vehicle CAN Bus (PSA)                                  │
│         [CAN ID 0x036: Ignition/Power/Reverse]      [CAN ID 0x0F6: Reverse Gear]        │
│         [CAN ID 0x128: Exterior Lights/Headlamps]   [CAN ID 0x228: Clock/Status]        │
+─────────────────────────────────────────────────────────────────────────────────────────+
                                             │
                                     [CAN Transceiver]
                                     (TJA1050 / SN65)
                                             │
                                             ▼
+─────────────────────────────────────────────────────────────────────────────────────────+
│                      OpenCanbox Core Firmware (Pure C99 Engine)                         │
│                                                                                         │
│   ┌────────────────────────────────┐         ┌──────────────────────────────────────┐   │
│   │   CAN Router / Profile Parser  │         │          HAL GPIO Subsystem          │   │
│   │   - 0x036 Byte 4 -> Ignition   │ ──────► │   hal_gpio_write(PIN_ACC, true)      │   │
│   │   - 0x128 Byte 0 -> Lights     │ ──────► │   hal_gpio_write(PIN_ILL, true)      │   │
│   │   - 0x036/0x0F6 -> Reverse     │ ──────► │   hal_gpio_write(PIN_REVERSE, true)  │   │
│   └────────────────────────────────┘         └──────────────────────────────────────┘   │
+─────────────────────────────────────────────────────────────────────────────────────────+
                                             │
                    ┌────────────────────────┼────────────────────────┐
                    │ (MCU 3.3V GPIO)        │ (MCU 3.3V GPIO)        │ (MCU 3.3V GPIO)
                    ▼                        ▼                        ▼
       ┌────────────────────────┐┌────────────────────────┐┌────────────────────────┐
       │   High-Side Switch 1   ││   High-Side Switch 2   ││   High-Side Switch 3   │
       │   (NPN + P-MOSFET)     ││   (NPN + P-MOSFET)     ││   (NPN + P-MOSFET)     │
       └────────────────────────┘└────────────────────────┘└────────────────────────┘
                    │                        │                        │
                    ▼ (+12V Switched)        ▼ (+12V Switched)        ▼ (+12V Switched)
     [ACC Output Line (Red)]   [ILL Output Line (Orange)] [BACK Output Line (Pink/Brown)]
                    │                        │                        │
                    └────────────────────────┼────────────────────────┘
                                             │
                                             ▼
+─────────────────────────────────────────────────────────────────────────────────────────+
│                       Android Head Unit 16-Pin / 20-Pin Harness                         │
│   - Pin A4 / Red Wire    : ACC In (+12V = Wake / OS Boot; 0V = Deep Sleep)              │
│   - Pin A6 / Orange Wire : ILL In (+12V = Night mode, screen dimming, LED backlights)   │
│   - Pin A5 / Brown Wire  : REVERSE In (+12V = Fast video MUX switch to AHD/CVBS CAM)     │
+─────────────────────────────────────────────────────────────────────────────────────────+
```

---

# 2. Detailed Signal Specifications

### 2.1 REVERSE Trigger (`GPIO_PIN_REVERSE_OUT`)

#### Purpose & Head Unit Reaction
- **Low-Latency Camera Switch:** Android OS serial polling latency ($\approx 100\text{ ms} - 300\text{ ms}$) is too slow for safety-critical reverse gear engagement. The Head Unit MCU monitors the physical `BACK` / `REVERSE` wire via an internal hardware interrupt.
- When $+12\text{V}$ is sensed:
  1. The MCU immediately routes analog CVBS/AHD video input to the screen.
  2. Broadcasts Android Intent `com.qf.action.BACKCAR_START` or injects `KeyEvent 289`.
  3. Launches reverse camera application (`QF_BackCar.apk`).
- When the pin drops to $0\text{V}$:
  1. Broadcasts `com.qf.action.BACKCAR_STOP`.
  2. Restores navigation/media application.

#### Vehicle CAN Sources (PSA Peugeot 407)
| CAN ID | Byte Index | Bit Mask | Signal Name | Logic Definition |
|:---|:---:|:---:|:---|:---|
| **`0x036`** | Byte 1 | `0x80` (`Bit 7`) | Reverse Engaged (High Speed) | `1` = Reverse Gear Active, `0` = Inactive |
| **`0x0F6`** | Byte 7 | `0x80` (`Bit 7`) | BSI Reverse Indicator (Slow Data) | `1` = Reverse Gear Active, `0` = Inactive |

#### Electrical Characteristics
- **Standard Harness Wire Color:** Brown, Pink, or Violet with White stripe (marked `REVERSE` or `BACK`).
- **Output Type:** Switched $+12\text{V}$ (High-Side Driver).
- **Active State:** $+10.5\text{V} \dots +14.8\text{V}$ (Battery Voltage, High).
- **Inactive State:** $0\text{V} \dots +1.0\text{V}$ (Floating or pulled to GND via $10\text{ k}\Omega$).
- **Maximum Load Current:** $10\text{ mA} - 50\text{ mA}$ (high-impedance MCU optocoupler/resistor divider input inside the Head Unit).

---

### 2.2 ILLUMINATION Trigger (`GPIO_PIN_ILL_OUT`)

#### Purpose & Head Unit Reaction
- **Automatic Day/Night Mode:** Controls screen brightness and button backlight illumination.
- When $+12\text{V}$ is sensed:
  1. Head Unit enters Night Mode (dimmed display brightness according to user night setting).
  2. Activates capacitive/physical button LED backlights (RGB or single-color).
  3. Tells navigation apps (e.g., Google Maps, Waze) to switch to dark color scheme.
- When $0\text{V}$ is sensed:
  1. Restores maximum screen brightness for direct daylight readability.
  2. Deactivates button LEDs.

#### Vehicle CAN Sources (PSA Peugeot 407)
| CAN ID | Byte Index | Bit Mask | Signal Name | Logic Definition |
|:---|:---:|:---:|:---|:---|
| **`0x128`** | Byte 0 | `0x80` (`Bit 7`) | Sidelights (Position/Parking) | `1` = Sidelights ON, `0` = OFF |
| **`0x128`** | Byte 0 | `0x40` (`Bit 6`) | Headlights (Low / Dipped Beam) | `1` = Headlights ON, `0` = OFF |
| **`0x036`** | Byte 3 | `0x20` (`Bit 5`) | Instrument Cluster Illumination | `1` = Dashboard Backlight Active |

> [!NOTE]
> The `ILL` signal must activate if **either** sidelights (`0x128` Bit 7) or low-beam headlights (`0x128` Bit 6) are active.

#### Electrical Characteristics
- **Standard Harness Wire Color:** Orange or Orange with White stripe (marked `ILL`, `ILLUMINATION`, or `LAMP`).
- **Output Type:** Switched $+12\text{V}$ (High-Side Driver).
- **Active State:** $+10.5\text{V} \dots +14.8\text{V}$ (High).
- **Inactive State:** $0\text{V} \dots +1.0\text{V}$ (Low).
- **Maximum Load Current:** $20\text{ mA} - 100\text{ mA}$ (typically drives internal MCU sense pin or optocoupler).

---

### 2.3 ACC (Accessory / Ignition) Trigger (`GPIO_PIN_HEADUNIT_POWER` / `ACC`)

#### Purpose & Head Unit Reaction
- **Power Lifecycle Control:** Controls whether the Android Head Unit is awake or in deep sleep.
- Modern Android Head Units remain permanently connected to $+12\text{V}$ Constant Battery (`BATT+` / Yellow Wire) to preserve clock, memory, and allow ultra-fast 1-second cold boot.
- When ACC $+12\text{V}$ is asserted:
  1. Head Unit core power rail regulators (Buck DC/DC converters) are enabled.
  2. Main SoC exits suspend-to-RAM (`STR`) within 1.0 second.
- When ACC drops to $0\text{V}$:
  1. Android Head Unit displays "Entering Sleep Mode".
  2. OS saves active state to RAM and powers down display, audio power amplifier, and Wi-Fi/Bluetooth modules.
  3. Parasitic quiescent current drops below $5\text{ mA}$.

#### Vehicle CAN Sources & Wakeup Triggers
| CAN ID / Source | Byte Index | Value / Mask | Signal Name | Logic Definition |
|:---|:---:|:---:|:---|:---|
| **`0x036`** | Byte 4 | `0x03` | Ignition Key ACC Position | `true` = ACC Active |
| **`0x036`** | Byte 4 | `0x01` | Ignition Key IGN / Engine Run | `true` = IGN Active |
| **`0x036`** | Byte 4 | `0x00` | Ignition Key Removed / OFF | `false` = Standby / OFF |
| **Physical Pin `PB12`** | — | High Level | Hardware Ignition Wire Sense | Fallback if CAN is asleep |

#### Electrical Characteristics
- **Standard Harness Wire Color:** Red (marked `ACC` or `SWITCHED +12V`).
- **Output Type:** Switched $+12\text{V}$ (High-Side MOSFET or Solid-State Relay).
- **Active State:** $+10.5\text{V} \dots +14.8\text{V}$ (High).
- **Inactive State:** $0\text{V}$ (Low).
- **Continuous Current Rating:** Must support **$500\text{ mA} - 1.5\text{ A}$ continuous** (some Head Units pull operating current for internal peripherals from the ACC line).

---

# 3. Hardware Schematic & Electronic Topologies

Microcontroller GPIO pins operate strictly at **$3.3\text{V}$ CMOS logic** ($V_{OH} \approx 3.3\text{V}$, $I_{max} \approx 8 - 20\text{ mA}$). Connecting MCU pins directly to $+12\text{V}$ automotive wiring will permanently destroy the microcontroller. 

### 3.1 Recommended High-Side Driver (+12V Active High Switch)

The standard automotive driver topology uses an NPN pre-driver transistor (or N-channel small-signal MOSFET) controlling a high-side P-channel power MOSFET.

```
       +12V_BATT (Automotive Raw +12V ... +14.8V)
            │
            ├───[TVS: SMAJ14A]─── GND (Automotive Spike Protection)
            │
            ├──[R1: 10k]──────────┐
            │                     │
            ├───────────────┐     │
            │ (Source)      │     │
       ┌────┴────┐          │     │
       │ P-MOSFET│          │     │
       │ AO3401A │          │     │
       │ (SI2301)│          │     │
       └────┬────┘          │     │
            │ (Drain)       ├───[Gate]
            │               │
            │          ┌────┴────┐
            │          │   NPN   │
            │          │ 2N3904  │
            │          │(BC847)  │
            │          └────┬────┘
            │               │ (Base)
            │               ├──[R2: 1k]───◄ MCU GPIO (3.3V Logic)
            │               │
            │              [R3: 10k]
            │               │
            │              GND
            │
            ├───[PTC Fuse: 500mA - 1.5A]
            │
            ├───[Schottky Clamp Diode: SS14 / 1N5819 to GND]
            │
            ▼
   OUTPUT TRIGGER WIRE (REVERSE / ILL / ACC to Android Head Unit)
```

#### Component Values & Design Rationale:
1. **P-MOSFET (AO3401A / SI2301 / IRLML6402):**
   - $V_{DS} = -30\text{V}$, $I_D \ge -4\text{A}$, $R_{DS(on)} \le 50\text{ m}\Omega$.
   - Negligible voltage drop ($< 25\text{ mV}$ at $500\text{ mA}$).
2. **NPN Pre-Driver (2N3904 / MMBT3904 / BC847):**
   - Translates $3.3\text{V}$ MCU logic to $+12\text{V}$ rail switching.
   - When MCU pin is `HIGH` ($3.3\text{V}$), base current turns NPN `ON`, pulling P-MOSFET gate to GND ($V_{GS} \approx -12\text{V}$), saturating the P-MOSFET.
   - When MCU pin is `LOW` ($0\text{V}$), NPN turns `OFF`, $R_1$ pulls Gate to $+12\text{V}$ ($V_{GS} = 0\text{V}$), shutting P-MOSFET completely `OFF`.
3. **Pull-Up Resistor ($R_1 = 10\text{ k}\Omega$):** Ensures clean turn-off and prevents gate float.
4. **Base Resistor ($R_2 = 1\text{ k}\Omega$):** Sets base current $I_B \approx \frac{3.3\text{V} - 0.7\text{V}}{1000\Omega} = 2.6\text{ mA}$, driving NPN into deep saturation.
5. **Base Bleeder ($R_3 = 10\text{ k}\Omega$):** Prevents spurious output activation during MCU reset/floating states.
6. **PTC Resettable Fuse:** Protects the board traces against direct short circuits to car chassis ground.
7. **TVS Diode (SMAJ14A):** Clamps ISO 7637-2 automotive load dumps and inductive switching spikes.

---

### 3.2 Optional Optocoupler Isolation Topology

For harsh commercial vehicle environments or galvanic isolation:

```
                      +12V_BATT
                          │
                     ┌────┴────┐
                     │ Collector│
                     │ Opto-Tx  │ (PC817 / EL817)
                     │ Emitter  │
                     └────┬────┘
                          ├───[R_pull: 1k / 1W]─── OUTPUT (+12V)
                          │
                         GND (Via Head Unit Load)

 MCU GPIO (3.3V) ───[R_limit: 220R]───►[Opto LED+]
                                       [Opto LED-]─── GND_MCU
```

---

# 4. Microcontroller Pinout Mapping

### 4.1 STM32F103 (BluePill & Custom PCB Targets)

| Signal Name | HAL Enum Identifier | STM32 Pin | GPIO Mode | Active State | Description |
|:---|:---|:---:|:---|:---:|:---|
| **ACC / HU POWER** | `GPIO_PIN_HEADUNIT_POWER` | **`PB1`** (or `PB2`) | Push-Pull Output | HIGH (3.3V &rarr; +12V) | Switched 12V ACC output to Head Unit |
| **REVERSE OUT** | `GPIO_PIN_REVERSE_OUT` | **`PB5`** | Push-Pull Output | HIGH (3.3V &rarr; +12V) | Camera trigger wire |
| **ILLUMINATION** | `GPIO_PIN_ILL_OUT` | **`PB7`** | Push-Pull Output | HIGH (3.3V &rarr; +12V) | Night dimming trigger wire |
| **IGNITION SENSE** | `GPIO_PIN_IGNITION_IN` | **`PB12`** | Input (Pull-Down) | HIGH (+12V In) | Hardware analog ACC/IGN monitor |
| **CAN TRANSCEIVER**| `GPIO_PIN_CAN_STBY` | **`PB0`** | Push-Pull Output | LOW (0V) | Transceiver standby (0 = Normal, 1 = Standby) |
| **STATUS LED** | `GPIO_PIN_LED_STATUS` | **`PC13`** | Push-Pull Output | LOW (Active LOW) | Onboard heartbeat/diagnostic LED |

---

### 4.2 ESP32 Target (`esp32_cbox`)

| Signal Name | HAL Enum Identifier | ESP32 Pin | GPIO Mode | Active State | Description |
|:---|:---|:---:|:---|:---:|:---|
| **REVERSE OUT** | `GPIO_PIN_REVERSE_OUT` | **`GPIO_NUM_4`** | Output | HIGH | Physical BACK trigger wire |
| **ILLUMINATION** | `GPIO_PIN_ILL_OUT` | **`GPIO_NUM_5`** | Output | HIGH | Night dimming trigger wire |
| **ACC / HU POWER** | `GPIO_PIN_HEADUNIT_POWER` | **`GPIO_NUM_18`** | Output | HIGH | Switched 12V ACC output |
| **IGNITION SENSE** | `GPIO_PIN_IGNITION_IN` | **`GPIO_NUM_34`** | Input (Input-Only)| HIGH | Hardware ignition sense |
| **STATUS LED** | `GPIO_PIN_LED_STATUS` | **`GPIO_NUM_2`** | Output | HIGH | Onboard diagnostic LED |

---

# 5. Core Firmware Architecture & Software Drivers

### 5.1 HAL Header Interface (`include/hal/hal_gpio.h`)

```c
#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <stdbool.h>
#include "hal_system.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GPIO_PIN_LED_STATUS = 0,
    GPIO_PIN_CAN_STBY,       /* Transceiver standby/silent control */
    GPIO_PIN_IGNITION_IN,    /* Accessory/Ignition 12V detect (via opto/divider) */
    GPIO_PIN_HEADUNIT_POWER, /* Switched +12V ACC supply enable */
    GPIO_PIN_REVERSE_OUT,    /* Physical BACK / REVERSE trigger wire */
    GPIO_PIN_ILL_OUT         /* Physical ILLUMINATION trigger wire */
} hal_gpio_pin_t;

hal_status_t hal_gpio_init(void);
void         hal_gpio_write(hal_gpio_pin_t pin, bool state);
bool         hal_gpio_read(hal_gpio_pin_t pin);
void         hal_gpio_toggle(hal_gpio_pin_t pin);

#ifdef __cplusplus
}
#endif

#endif /* HAL_GPIO_H */
```

---

### 5.2 CAN Routing & Synchronous Dispatch (`src/core/can_router.c`)

The CAN router monitors state updates from vehicle profiles and immediately drives the corresponding GPIO pins without waiting for periodic timer ticks:

```c
/* ==========================================================================
 * CAN ROUTER DISPATCH EXCERPT
 * ========================================================================== */

/* 1. Instantaneous Reverse Gear Update */
if (s_current_state.reverse_gear != s_last_sent_state.reverse_gear) {
    /* Drive physical BACK trigger wire */
    hal_gpio_write(GPIO_PIN_REVERSE_OUT, s_current_state.reverse_gear);
    
    /* Forward over serial protocol if supported */
    hu_protocol_send_reverse(s_current_state.reverse_gear);
    
    s_last_sent_state.reverse_gear = s_current_state.reverse_gear;
}

/* 2. Instantaneous Illumination / Headlights Update */
bool ill_active = s_current_state.lights.side_light || s_current_state.lights.headlights;
bool prev_ill   = s_last_sent_state.lights.side_light || s_last_sent_state.lights.headlights;

if (ill_active != prev_ill) {
    /* Drive physical ILL trigger wire */
    hal_gpio_write(GPIO_PIN_ILL_OUT, ill_active);
    
    s_last_sent_state.lights.side_light = s_current_state.lights.side_light;
    s_last_sent_state.lights.headlights = s_current_state.lights.headlights;
}

/* 3. Accessory / Ignition Power Control */
bool acc_active = (s_current_state.ignition_state == VEHICLE_IGNITION_ON ||
                   s_current_state.ignition_state == VEHICLE_IGNITION_ACC);

if (acc_active != (s_last_sent_state.ignition_state == VEHICLE_IGNITION_ON ||
                   s_last_sent_state.ignition_state == VEHICLE_IGNITION_ACC)) {
    /* Enable or disable switched +12V ACC to Head Unit */
    hal_gpio_write(GPIO_PIN_HEADUNIT_POWER, acc_active);
}
```

---

# 6. Timing, Debouncing, and Power Sequencing

### 6.1 Engine Cranking Dip Protection (Crank Immunity)
When the starter motor engages, the car battery voltage frequently collapses from $12.6\text{V}$ down to $8.0\text{V} - 9.5\text{V}$ for $500\text{ ms} - 1500\text{ ms}$, and the vehicle BSI temporarily broadcasts ignition state `0x00` or `0x03` (`CRANK`).
- **Hazard:** Dropping the `ACC` GPIO pin immediately will cause the Android Head Unit to initiate a shutdown or reboot cycle right when the engine starts.
- **Solution:** Implement a **2.5-second hold-off timer** on ACC drop. If ignition re-asserts within 2.5 seconds, ACC output remains solidly held at $+12\text{V}$.

```
Battery 12V ──┐                      ┌──────────────────────────────
              │      Cranking Dip    │
              └────────┐ (8.5V) ┌────┘
                       └────────┘
CAN Ignition ─┐                 ┌───────────────────────────────────
State         │                 │
              └─────────────────┘
                  ◄─── 1.2s ───►

ACC GPIO ───────────────────────────────────────────────────────────
Output (Held) (Hold-off Timer prevents glitchy power-down)
```

---

### 6.2 Reverse Gear Shift Hysteresis
Drivers frequently shift rapidly through reverse (e.g. from `Drive` &rarr; `Neutral` &rarr; `Reverse` &rarr; `Park`).
- **Debounce Policy:**
  - **Reverse Assertion ($0 \to 1$):** Immediate response ($< 10\text{ ms}$) to ensure immediate video camera availability.
  - **Reverse De-assertion ($1 \to 0$):** **$500\text{ ms} - 1000\text{ ms}$ trailing delay**. When performing a 3-point turn (shifting between `R` and `D`), this delay keeps the rear camera active without annoying black screen flicker between shifts.

---

### 6.3 Illumination Tunnel / Overpass Hysteresis
Vehicles equipped with automatic headlights (light sensor) may toggle headlights ON and OFF rapidly when passing under highway bridges or tree canopies.
- **Debounce Policy:**
  - **Lights ON:** Immediate assertion.
  - **Lights OFF:** **$1.5\text{ s}$ trailing delay** before de-asserting the ILL wire, avoiding screen flickering.

---

# 7. Verification, Diagnostics & Test Harness

### 7.1 Bench Test Procedures (Oscilloscope / Multimeter)

1. Connect a digital multimeter (DC Volts) or oscilloscope channel between the CAN box `REVERSE` wire output and chassis GND.
2. In native simulator / bench harness, inject reverse gear CAN frames:
   ```bash
   # Engage Reverse Gear on Peugeot 407 (0x036 Byte 1 = 0x80)
   cansend vcan0 036#0E80000001000000

   # Verify Multimeter reads +12.0V ... +14.4V
   ```
3. Disengage Reverse Gear:
   ```bash
   # Neutral / Park (0x036 Byte 1 = 0x00)
   cansend vcan0 036#0E00000001000000

   # Verify Multimeter falls to 0.0V
   ```
4. Test Illumination Trigger:
   ```bash
   # Turn Sidelights ON (0x128 Byte 0 = 0x80)
   cansend vcan0 128#8000000000000000

   # Verify ILL Output Pin rises to +12.0V
   ```

### 7.2 Native Integration Unit Test Examples (`test_integration_hiworld.c`)

```c
void test_integration_gpio_reverse_and_ill(void) {
    // 1. Initial State: Reverse OFF, ILL OFF
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_REVERSE_OUT));
    TEST_ASSERT_FALSE(hal_gpio_read(GPIO_PIN_ILL_OUT));

    // 2. Engage Reverse via CAN 0x036
    can_frame_t rev_frame = {
        .id = 0x036,
        .dlc = 8,
        .data = { 0x0E, 0x80, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&rev_frame);
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_REVERSE_OUT));

    // 3. Turn on Headlights via CAN 0x128
    can_frame_t light_frame = {
        .id = 0x128,
        .dlc = 8,
        .data = { 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    can_router_process_can(&light_frame);
    TEST_ASSERT_TRUE(hal_gpio_read(GPIO_PIN_ILL_OUT));
}
```

---

# 8. Standard Head Unit Wire Harness Reference

Standard ISO 10487 / Chinese Android Head Unit pin assignments for the power connector:

```
               ┌─────────────────────────────────────┐
               │  [1]   [2]   [3]   [4]   [5]   [6]  │
               │  [7]   [8]   [9]  [10]  [11]  [12]  │
               │ [13]  [14]  [15]  [16]  [17]  [18]  │
               └─────────────────────────────────────┘
```

| Pin # | Wire Color | Label | Function | OpenCanbox Core Driver Source |
|:---:|:---|:---|:---|:---|
| **1** | **Black** | `GND` | System Chassis Ground ($0\text{V}$) | Direct Vehicle Chassis / Battery Negative |
| **2** | **Yellow** | `BATT+` | Constant Unswitched Battery ($+12\text{V}$) | Direct Vehicle Battery Fuse Link |
| **3** | **Red** | `ACC` | Switched Ignition ($+12\text{V}$) | **`GPIO_PIN_HEADUNIT_POWER` (PB1 / GPIO 18)** |
| **4** | **Orange** | `ILL` | Illumination / Dimming ($+12\text{V}$) | **`GPIO_PIN_ILL_OUT` (PB7 / GPIO 5)** |
| **5** | **Brown / Pink**| `BACK / REV`| Reverse Camera Trigger ($+12\text{V}$) | **`GPIO_PIN_REVERSE_OUT` (PB5 / GPIO 4)** |
| **6** | **Blue** | `ANT / AMP`| Remote Amplifier Turn-on ($+12\text{V}$ Out) | Generated by Head Unit to external amplifier |
| **7** | **Orange/Black**| `KEY1` | Steering Key Analog In 1 (Resistive) | Synthesized by DAC/Resistor Matrix (if non-CAN) |
| **8** | **Brown/Black** | `KEY2` | Steering Key Analog In 2 (Resistive) | Synthesized by DAC/Resistor Matrix (if non-CAN) |
| **9** | **Blue/White** | `UART_TX`| Serial Data from CAN Box to HU | STM32 USART1 TX (`PA9`) / ESP32 UART2 TX |
| **10**| **Green/White**| `UART_RX`| Serial Data from HU to CAN Box | STM32 USART1 RX (`PA10`) / ESP32 UART2 RX |
