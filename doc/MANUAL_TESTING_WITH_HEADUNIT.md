# Manual Head Unit Verification & CAN Log Replay Guide

This guide walks you through connecting your Linux development laptop directly to an Android Head Unit (HU) via a USB-to-UART adapter, replaying real recorded automotive CAN logs, and manually verifying vehicle animations, climate controls, lights, telemetry, and door popups on the Head Unit screen.

---

## 1. Hardware Setup & Physical Wiring

To connect your laptop to the Head Unit, use a standard **USB-to-UART serial dongle** (such as FT232RL, CP2102, CH340, or PL2303).

### 1.1 Pin Connections

Connect the USB dongle to the CANBox / Protocol connector on the back of your Android Head Unit:

```
+---------------------------+                    +---------------------------+
|      Laptop / Linux       |                    |     Android Head Unit     |
|   (USB Serial Adapter)    |                    |    (CANBox / UART Port)   |
|                           |                    |                           |
|       GND [Pin] ----------+--------------------+---------- GND [Pin]       |
|       TXD [Pin] ---------\ /-------------------+---------- RXD [Pin]       |
|       RXD [Pin] ----------X--------------------+---------- TXD [Pin]       |
+---------------------------+ \                  +---------------------------+
                               \
                                +---------------- 12V Power & ACC (to bench PSU)
```

> [!IMPORTANT]
> 1. **Common Ground (GND):** You **must** connect GND of the USB-UART adapter to the Head Unit GND pin to establish a common voltage reference.
> 2. **Crossed RX/TX:** Connect Dongle **TX $\to$ HU RX**, and Dongle **RX $\to$ HU TX**.
> 3. **Logic Levels:** Most Android Head Units accept standard **3.3V or 5V TTL** logic. If your USB adapter has a 3.3V/5V selector jumper, set it to **3.3V** or **5V** according to your HU specification (3.3V is safe for both).
> 4. **Power Supply:** Power the Head Unit with a 12V DC bench power supply (connect +12V Constant `B+` and `ACC` to +12V, `GND` to PSU Ground).

---

## 2. Android Head Unit Factory Configuration

Before sending data, ensure your Head Unit is configured to listen for the simulated CAN box protocol:

1. Open **Settings $\to$ Factory Settings** (or **Car Settings $\to$ Extra Settings**; enter your HU factory PIN, usually `8888`, `3368`, `1617`, `1234`, or `0000`).
2. Navigate to **CANBus / Protocol Settings / Model Selection**.
3. Select:
   - **Protocol Provider / CAN Brand:** `Raise` (or `SimpleSoft` / `RZC`)
   - **Car Brand:** `PSA` / `Peugeot` / `Citroën`
   - **Model:** `407` (or `Generic PSA 2004+`)
4. Save and allow the Head Unit to restart the CAN service.
5. Default UART communication parameters configured in OpenCanbox Core:
   - **Baud Rate:** `38400`
   - **Data Bits:** `8`
   - **Parity:** `None`
   - **Stop Bits:** `1` (8N1)

---

## 3. Source Code Architecture & What Was Changed

### Do we need source code changes for the HU to connect to the laptop?
**Yes, and they have been implemented:**

1. **Physical Serial Device Support (`src/hal/hal_native/hal_uart_linux.c`):**
   - *Previous state:* `hal_uart_init()` only created a virtual pseudo-terminal (`/tmp/ttyCanbox`).
   - *Updated:* Now checks the `CANBOX_UART_DEVICE` environment variable (e.g., `/dev/ttyUSB0`). If set, it opens the real hardware serial device directly, sets the baud rate (`38400`), configures raw 8N1 mode, and communicates directly with the Head Unit. If unset, it falls back to virtual PTY mode.

2. **Configurable CAN Interface (`src/hal/hal_native/hal_can_linux.c`):**
   - *Updated:* Checks `CANBOX_CAN_IFACE` (defaults to `vcan0` or can be pointed to physical `can0` if using a hardware CAN dongle).

3. **CPU Throttling on Linux Host (`src/main.c`):**
   - *Updated:* Added an idle sleep (`hal_delay_ms(1)`) on Linux to prevent the desktop process from pegging a CPU core at 100% while waiting for CAN/UART frames.

---

## 4. Running the Automated Python Test Harness

The Python runner [`tools/canbox_manual_test.py`](file:///home/Fazer/git/canbox-core/tools/canbox_manual_test.py) manages the complete test lifecycle:
* Sets up the virtual CAN interface (`vcan0`).
* Detects your USB serial dongle (e.g., `/dev/ttyUSB0`).
* Launches the OpenCanbox Core application in native mode.
* Replays the recorded `.csv` CAN log chronologically in an infinite loop.

### 4.1 Quick Start: Run Test with Default Lights Scenario

Plug in your USB serial adapter and execute:

```bash
python3 tools/canbox_manual_test.py --auto-setup-vcan
```

The script will:
1. Auto-create `vcan0` (prompting for `sudo` if necessary).
2. Auto-detect `/dev/ttyUSB0` (or list available ports).
3. Build the native target `.pio/build/native_test/program` if not already built.
4. Launch the application connected to `/dev/ttyUSB0`.
5. Stream CAN frames in an infinite loop.

### 4.2 Specifying Custom Serial Port and CAN Log

To explicitly pass the serial port and select a specific scenario log:

```bash
python3 tools/canbox_manual_test.py \
  --serial-port /dev/ttyUSB0 \
  --csv test/test_integration/data/lights_off_side_light_on_headlights_on.csv \
  --speed 1.0 \
  --auto-setup-vcan
```

### 4.3 Available Command-Line Options

| Flag | Shorthand | Default | Description |
|:---|:---|:---|:---|
| `--serial-port` | `-p` | Auto-detect | Path to USB serial device (e.g. `/dev/ttyUSB0`, `/dev/ttyACM0`) |
| `--csv` | `-c` | `lights_off_...csv` | Path to 14-column CAN log CSV capture |
| `--baudrate` | `-b` | `38400` | UART Baud rate (Raise protocol uses 38400) |
| `--iface` | `-i` | `vcan0` | CAN interface name |
| `--speed` | `-s` | `1.0` | Replay speed multiplier (e.g. `2.0` = 2x speed) |
| `--no-loop` | | `False` | Play log once and stop (omit to loop infinitely) |
| `--loop-delay` | | `0.5` | Delay in seconds between replay iterations |
| `--no-app` | | `False` | Replay CAN log only without spawning native binary |
| `--auto-setup-vcan`| | `False` | Automatically run `ip link` setup commands with `sudo` |

---

## 5. Available Test Scenario CAN Logs

The following pre-recorded CAN captures are available in [`test/test_integration/data/`](file:///home/Fazer/git/canbox-core/test/test_integration/data/):

### Scenario 1: Lights Off $\to$ Side Lights $\to$ Headlights
* **File:** `test/test_integration/data/lights_off_side_light_on_headlights_on.csv`
* **What to observe on the HU:**
  - Screen brightness / Night mode triggers when side lights and dipped beam headlights turn ON.
  - Head Unit illumination / light icon updates in the top status bar.

```bash
python3 tools/canbox_manual_test.py --csv test/test_integration/data/lights_off_side_light_on_headlights_on.csv
```

### Scenario 2: Power On $\to$ Ignition Off $\to$ Sleep
* **File:** `test/test_integration/data/ignition_off_after_power_on.csv`
* **What to observe on the HU:**
  - ACC / Ignition status transitions.
  - Heartbeat status keeping the Android MCU link active during ignition.

```bash
python3 tools/canbox_manual_test.py --csv test/test_integration/data/ignition_off_after_power_on.csv
```

---

## 6. Manual Verification Checklist on Android HU

When the script is running, verify the following interactive behaviors on the Head Unit:

- [ ] **UART Link Heartbeat:** The Head Unit should not display "CANBus Disconnected" or "No CAN Box".
- [ ] **Lighting / Dimmer:** Status bar headlights icon toggles, backlight dims when headlights are active.
- [ ] **Door Popup:** Opening a door in the CAN log displays the door overlay vehicle graphic.
- [ ] **Climate / HVAC Popup:** Temperature, fan speed, dual-zone AC, and defrost indicators reflect CAN state.
- [ ] **Steering Wheel Key Controls:** Volume Up/Down, Mode, Mute, Next/Prev track respond on the HU.
- [ ] **Trip & Telemetry:** Speed (km/h) and RPM indicators on the HU dashboard app move according to playback.
- [ ] **Parking Radar / Distance Bars:** Proximity sensors show colored distance bars on the reverse / radar screen.

---

## 7. Troubleshooting & Diagnostics

### 7.1 "Permission Denied" on `/dev/ttyUSB0`
If your user does not have permission to access the serial port:
```bash
sudo usermod -aG dialout $USER
sudo chmod 666 /dev/ttyUSB0
```

### 7.2 Monitor Raw UART Data Sent to HU
To observe the exact binary packets being transmitted to the Head Unit in real time:
```bash
# Using hexdump to inspect raw UART stream from a second terminal
sudo minicom -D /dev/ttyUSB0 -b 38400 -H
```

### 7.3 Monitor CAN Messages on `vcan0`
To inspect the CAN stream generated by the replayer:
```bash
candump vcan0
```

### 7.4 Resetting the Virtual CAN Interface
If `vcan0` gets into a bad state:
```bash
sudo ip link delete vcan0 type vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

