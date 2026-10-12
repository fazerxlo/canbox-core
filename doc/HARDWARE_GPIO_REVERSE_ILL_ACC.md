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
| **`0x036`** | Byte 3 | `0x20` (`Bit 5`) | Instrument Cluster Illumination | `1` = Night Backlight Active, `0` = Daytime |
| **`0x036`** | Byte 3 | `0x0F` (`Bits 3..0`) | Cluster Dimming Brightness | Normalized level $0 \dots 15$ ($15 = \text{max}$) |

> [!NOTE]
> The physical `ILL` hardware output line (`GPIO_PIN_ILL_OUT`) is asserted active if **any** of the following conditions are met:
> `ill_active = side_light || headlights || illumination;`
> Brightness level is simultaneously tracked in the canonical vehicle state (`state->lights.brightness`) and forwarded to Head Unit protocols supporting variable backlight dimming.

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

#### Declarative Power Logic
Firmware evaluates switched ACC power via a pure boolean function:
```c
if (economy_mode) {
    ACC = OFF;
} else if (radio_present) {
    /* If OEM RD4 radio is present on the bus:
     * Radio sleep mode ((data[0] & 0xC0) == 0) unconditionally forces ACC OFF.
     * Radio active (data[0] & 0x80) asserts ACC ON.
     * Otherwise follow ignition switch. */
    if (radio_sleep) {
        ACC = OFF;
    } else if (radio_on) {
        ACC = ON;
    } else {
        ACC = ignition_on ? ON : OFF;
    }
} else {
    /* If OEM RD4 radio is not present (aftermarket HU installed), follow ignition switch */
    ACC = ignition_on ? ON : OFF;
}
```
- **`ignition_on`:** CAN `0x036` Byte 4 (`0x01` Run `+APC` or `0x03` Crank pulse).
- **`radio_present`:** Becomes `true` upon receiving CAN `0x165`.
- **`radio_on`:** CAN `0x165` Byte 0 Bit 7 (`0x80` RD4 factory radio audio active).
- **`radio_sleep`:** CAN `0x165` Byte 0 Bits [7:6] == `0x00` (`(data[0] & 0xC0) == 0`, e.g. `0x08`, factory radio in deep sleep / standby).
- **`economy_mode`:** CAN `0x036` Byte 2 Bit 7 (`0x80` BSI battery protection).

#### Vehicle CAN Sources & Wakeup Triggers
| CAN ID / Source | Byte Index | Value / Mask | Signal Name | Logic Definition |
|:---|:---:|:---:|:---|:---|
| **`0x036`** | Byte 4 Bits [2:0] | `0x01` | `PHASE_VIE`: Ignition ON (Run `+APC`) | `true` = `ignition_on` (ACC ON if radio not in sleep) |
| **`0x036`** | Byte 4 Bits [2:0] | `0x02` | `PHASE_VIE`: Ignition OFF (Going to sleep) | `false` = `ignition_on` (Shutdown / ACC OFF unless radio) |
| **`0x036`** | Byte 4 Bits [2:0] | `0x03` | `PHASE_VIE`: Wakeup Transition (~40 ms) | `true` = `ignition_on` (Key turn transition pulse) |
| **`0x036`** | Byte 4 Bits [2:0] | `0x00` | `PHASE_VIE`: Deep Sleep / Standby | `false` = `ignition_on` (Deep Sleep / ACC OFF) |
| **`0x036`** | Byte 2 Bit 7 | `0x80` | `MODE_ECO`: Economy Mode | `1` = `economy_mode` (Blocks radio ACC to protect battery) |
| **`0x165`** | Byte 0 Bit 7 | `0x80` | RD4 Factory Radio Power State | `1` = `radio_on` (Powers ACC when ignition OFF & eco OFF) |
| **`0x165`** | Byte 0 Bits [7:6] | `0x00` | RD4 Deep Sleep / Standby State | `(data[0] & 0xC0) == 0` = `radio_sleep` (Forces ACC OFF) |
| **Physical Pin `PB12`** | — | High Level | Hardware Ignition Wire Sense | Fallback if CAN is asleep |

#### Real-World Operational Scenarios (CAN Log Ground Truth)
1. **Battery Reconnect Initial Wake (`dump_connect_battery_only.log`):**
   - When $+12\text{V}$ battery power is physically connected without ignition key, the PSA BSI initializes and transmits `0x036` Byte 4 = `0x02` (`PHASE_VIE = 0x02`, Ignition OFF) for approximately $14\text{ seconds}$.
   - Because the ignition key is absent, the radio and physical ACC wire stay strictly at $0\text{V}$ (OFF).
   - At $t = 1791462266.040\text{ s}$, `0x036` Byte 4 transitions to `0x00` (`Deep Sleep / Standby`).
   - Immediately following this drop, all periodic CAN frames cease, and the vehicle comfort bus enters silent sleep. The 3.0-second bus inactivity watchdog asserts transceiver standby mode (`GPIO_PIN_CAN_STBY = true`) and completely silences UART comms.
2. **Key OFF + RD4 Radio Knob Power-On (`dump_ign_off_rd4_on_off.log`):**
   - With the vehicle key removed (`IGN OFF`), pressing the power knob on the factory RD4 radio wakes the comfort CAN bus.
   - The BSI transmits `0x036` Byte 4 = `0x01` (`VEHICLE_IGNITION_ON`), and the RD4 radio transmits `0x165` with `data[0] = 0xC8` (`Bit 7 = 1` radio active, `Bit 6 = 1` display active).
   - The decoder promotes this state to `VEHICLE_IGNITION_ACC`, immediately asserting `GPIO_PIN_HEADUNIT_POWER = true` (+12V switched ACC) so that the aftermarket Android Head Unit powers up alongside the OEM audio system.
   - When the user presses the RD4 knob again to turn off the radio, `0x165` transitions to `0x48` then `0x08` (`(data[0] & 0xC0) == 0`), display frame `0x1E0` zeros out, and comfort bus frames stop.
   - The 3.0-second bus silence watchdog drops `GPIO_PIN_HEADUNIT_POWER` to `false` and places the CAN transceiver into standby.
3. **Economy Mode Battery Protection (`MODE_ECO`):**
   - When BSI senses battery charge dropping below threshold, it sets `0x036` Byte 2 Bit 7 (`0x80`).
   - Firmware immediately forces `ignition_state = VEHICLE_IGNITION_OFF`, dropping `GPIO_PIN_HEADUNIT_POWER` to $0\text{V}$ and preventing RD4 knob power-up from drawing current until the engine is started.

#### Electrical Characteristics
- **Standard Harness Wire Color:** Red (marked `ACC` or `SWITCHED +12V`).
- **Output Type:** Switched $+12\text{V}$ (High-Side MOSFET or Solid-State Relay).
- **Active State:** $+10.5\text{V} \dots +14.8\text{V}$ (High).
- **Inactive State:** $0\text{V}$ (Low).
- **Continuous Current Rating:** Must support **$500\text{ mA} - 1.5\text{ A}$ continuous** (some Head Units pull operating current for internal peripherals from the ACC line).

---

### 2.4 CAN Transceiver Standby & Sleep Control (`GPIO_PIN_CAN_STBY`)

#### Purpose & Low-Power Sleep Management
- Minimizes parasitic battery drain when the vehicle comfort bus is sleeping and the ignition key is removed.
- Prevents the CAN box from draining the car battery over prolonged parking periods by placing the physical CAN transceiver into silent/standby mode.

#### Transceiver IC Compatibility & Electrical Logic
- **Compatible Transceivers:** TJA1050, TJA1040, SN65HVD230, MCP2551, VP230.
- **Hardware Pin Assignment:**
  - STM32F103: Pin **`PB0`** (Push-Pull Output).
  - ESP32: Pin **`GPIO_NUM_21`** (or designated standby pin).
- **Electrical Truth Table:**
  | Logical State (`bool`) | MCU Output Level | Transceiver Mode | Receiver State | Transmitter State | Quiescent Current ($I_{CC}$) |
  |:---:|:---:|:---:|:---:|:---:|:---:|
  | **`false`** | **`0V` (LOW)** | **Normal Mode** | Active | Active (High-Speed TX/RX) | $\approx 5\text{ mA} - 15\text{ mA}$ |
  | **`true`** | **`3.3V` (HIGH)** | **Standby / Silent** | Listen-only / Sleep | Disabled | $< 15\ \mu\text{A}$ |

#### Dynamic Power Management in Firmware (`can_router.c`)
1. **Wake-on-CAN:**
   - Whenever any incoming CAN message arrives via `can_router_process_can()`, the router immediately asserts:
     ```c
     hal_gpio_write(GPIO_PIN_CAN_STBY, false); /* Normal Mode (0V / LOW) */
     s_can_inactivity_ticks = 0;
     ```
2. **Inactivity Sleep Watchdog:**
   - In `can_router_periodic_100ms()`, `s_can_inactivity_ticks` increments every 100 ms.
   - If no CAN frames arrive for **3.0 seconds** (30 consecutive ticks), the bus is physically dormant or disconnected. The watchdog unconditionally forces sleep:
     ```c
     /* 3.0 seconds of total bus silence */
     s_current_state.ignition_state = VEHICLE_IGNITION_OFF;
     s_last_sent_state.ignition_state = VEHICLE_IGNITION_OFF;
     hal_gpio_write(GPIO_PIN_HEADUNIT_POWER, false); /* Drop ACC +12V */
     hal_gpio_write(GPIO_PIN_ILL_OUT, false);        /* Drop ILL +12V */
     hal_gpio_write(GPIO_PIN_REVERSE_OUT, false);    /* Drop REVERSE */
     hal_gpio_write(GPIO_PIN_CAN_STBY, true);        /* Standby Mode (3.3V / HIGH) */
     s_can_bus_sleeping = true;
     ```
   - All periodic transmissions, TPMS updates, door repeats, and keep-alive heartbeats (`CMD 0xFF`) are halted immediately during sleep (`if (s_can_bus_sleeping) return;`). UART traffic is completely silenced until a CAN frame wakes the bus.

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

### 4.1 STM32F103 (OEM CAN Box `volvo_od2` / `qemu`)

| Signal Name | HAL Enum Identifier | STM32 Pin | GPIO Mode | Active State | Description |
|:---|:---|:---:|:---|:---:|:---|
| **CAN TRANSCEIVER**| `GPIO_PIN_CAN_STBY` | **`PB6`** | Output (Open-Drain)| `0` = Active, `1` = Standby | Transceiver standby/silent control (ZL1040/TJA1040) |
| **ACC / HU POWER** | `GPIO_PIN_HEADUNIT_POWER` | **`PB9`** | Push-Pull Output | HIGH (3.3V &rarr; +12V) | Switched 12V ACC output trigger to Head Unit |
| **ILLUMINATION** | `GPIO_PIN_ILL_OUT` | **`PC13`** | Push-Pull Output | HIGH (3.3V &rarr; +12V) | Night dimming trigger wire |
| **BRAKE / PARK** | `GPIO_PIN_BRAKE_OUT` | **`PB8`** | Push-Pull Output | LOW (Permanently OFF) | Handbrake / Parking brake trigger to Head Unit |
| **REVERSE OUT** | `GPIO_PIN_REVERSE_OUT` | **`PB5`** | Push-Pull Output | HIGH (3.3V &rarr; +12V) | Camera trigger wire |
| **IGNITION SENSE** | `GPIO_PIN_IGNITION_IN` | **`PB12`** | Input (Pull-Down) | HIGH (+12V In) | Hardware analog ACC/IGN monitor |

---

### 4.2 NUC131 Target (OEM CAN Box `vw_nc03`)

| Signal Name | HAL Enum Identifier | NUC131 Pin | GPIO Mode | Active State | Description |
|:---|:---|:---:|:---|:---:|:---|
| **CAN TRANSCEIVER**| `GPIO_PIN_CAN_STBY` | **`PC3`** | Output (Open-Drain)| `0` = Active, `1` = Standby | Transceiver standby/silent control |
| **ACC / HU POWER** | `GPIO_PIN_HEADUNIT_POWER` | **`PA8`** | Push-Pull Output | HIGH (3.3V &rarr; +12V) | Switched 12V ACC output trigger to Head Unit |
| **ILLUMINATION** | `GPIO_PIN_ILL_OUT` | **`PA9`** | Push-Pull Output | HIGH (3.3V &rarr; +12V) | Night dimming trigger wire |
| **BRAKE / PARK** | `GPIO_PIN_BRAKE_OUT` | **`PA12`** | Push-Pull Output | LOW (Permanently OFF) | Handbrake / Parking brake trigger to Head Unit |
| **REVERSE OUT** | `GPIO_PIN_REVERSE_OUT` | **`PA13`** | Push-Pull Output | HIGH (3.3V &rarr; +12V) | Camera trigger wire |
| **IGNITION SENSE** | `GPIO_PIN_IGNITION_IN` | **`PA0`** | Input (Pull-Down) | HIGH (+12V In) | Hardware analog ACC/IGN monitor |

---

### 4.3 ESP32 Target (`esp32_cbox`)

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
    GPIO_PIN_HEADUNIT_POWER, /* Switched +12V ACC supply enable (ACC) */
    GPIO_PIN_REVERSE_OUT,    /* Physical BACK / REVERSE trigger wire */
    GPIO_PIN_ILL_OUT,        /* Physical ILLUMINATION trigger wire */
    GPIO_PIN_BRAKE_OUT       /* Physical HANDBRAKE / PARK trigger wire */
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
bool ill_active = s_current_state.lights.side_light || s_current_state.lights.headlights || s_current_state.lights.illumination;
bool prev_ill   = s_last_sent_state.lights.side_light || s_last_sent_state.lights.headlights || s_last_sent_state.lights.illumination;

if (ill_active != prev_ill) {
    /* Drive physical ILL trigger wire */
    hal_gpio_write(GPIO_PIN_ILL_OUT, ill_active);
    
    s_last_sent_state.lights.side_light   = s_current_state.lights.side_light;
    s_last_sent_state.lights.headlights   = s_current_state.lights.headlights;
    s_last_sent_state.lights.illumination = s_current_state.lights.illumination;
}

/* 3. Accessory / Ignition Power Control */
bool acc_active = (s_current_state.ignition_state == VEHICLE_IGNITION_ON ||
                   s_current_state.ignition_state == VEHICLE_IGNITION_ACC);

if (acc_active != (s_last_sent_state.ignition_state == VEHICLE_IGNITION_ON ||
                   s_last_sent_state.ignition_state == VEHICLE_IGNITION_ACC)) {
    /* Enable or disable switched +12V ACC to Head Unit */
    hal_gpio_write(GPIO_PIN_HEADUNIT_POWER, acc_active);
}

/* 4. CAN Transceiver Wake-up & Inactivity Watchdog */
/* Inside can_router_process_can(): */
hal_gpio_write(GPIO_PIN_CAN_STBY, false); /* Wake transceiver on any frame */
s_can_inactivity_ticks = 0;

/* Inside can_router_periodic_100ms(): */
s_can_inactivity_ticks++;
if (s_can_inactivity_ticks >= 30) { /* 3.0 seconds bus silence */
    if (s_current_state.ignition_state != VEHICLE_IGNITION_ON) {
        s_current_state.ignition_state = VEHICLE_IGNITION_OFF;
        hal_gpio_write(GPIO_PIN_HEADUNIT_POWER, false);
        hal_gpio_write(GPIO_PIN_ILL_OUT, false);
        hal_gpio_write(GPIO_PIN_CAN_STBY, true); /* Enter low-power standby */
    }
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
