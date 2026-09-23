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
|       GND [Pin4] ----------+--------------------+------ GND [Pin]          |
|       TXD [Pin2] ---------\ /-------------------+------ RXD [Pin Violet]   |
|       RXD [Pin3] ----------X--------------------+------ TXD [Pin Green ]   |
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
   - **Baud Rate:** `19200` (Standard for Peugeot 407 Raise / PSA protocol)
   - **Voltage Level:** `3.3V TTL`
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

The Python runner [`tools/canbox_manual_test.py`](file://tools/canbox_manual_test.py) manages the complete test lifecycle:
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
| `--time-unit` | `-u` | `auto` | Timestamp unit (`auto`, `us`, `ms`, `s`, `ns`) |
| `--protocol` | `-P` | `bagoo` | HU protocol driver (`bagoo`, `raise`, `hiworld`) |
| `--baudrate` | `-b` | `19200` | UART Baud rate (default `19200` for Bagoo/Raise Peugeot) |
| `--iface` | `-i` | `vcan0` | CAN interface name |
| `--speed` | `-s` | `1.0` | Replay speed multiplier (e.g. `2.0` = 2x speed) |
| `--no-loop` | | `False` | Play log once and stop (omit to loop infinitely) |
| `--loop-delay` | | `0.5` | Delay in seconds between replay iterations |
| `--no-app` | | `False` | Replay CAN log only without spawning native binary |
| `--auto-setup-vcan`| | `False` | Automatically run `ip link` setup commands with `sudo` |

---

## 5. Direct Serial Packet Replay (`tools/replay_serial.py`)

If you want to test the Head Unit's reaction directly to recorded CANBox serial packets without running the full CAN translator / simulator on `vcan0`, use [`tools/replay_serial.py`](file://tools/replay_serial.py).

This tool streams raw binary (`.bin`) or hex-dumped text (`.txt`) serial packets directly over USB-UART to the Head Unit's MCU, automatically parsing individual frames (using `0xFD` / `0x2E` headers) and pacing transmissions with configurable inter-packet delays.

### 5.1 Quick Start Examples

```bash
# 1. Single-pass replay of the Peugeot 407 dump (auto-detects USB serial adapter @ 38400 baud)
python3 tools/replay_serial.py test_data/dump.bin

# 2. Continuous loop replay with 1.0s pause between cycles
python3 tools/replay_serial.py test_data/dump.bin --loop --loop-delay 1.0

# 3. Explicit serial port, baud rate, and 50ms inter-packet delay
python3 tools/replay_serial.py test_data/dump.bin -p /dev/ttyUSB0 -b 38400 -d 0.05

# 4. Replay directly from a hex-formatted text dump
python3 tools/replay_serial.py test_data/dump.txt --loop
```

### 5.2 Command-Line Options

| Option | Short | Default | Description |
|:---|:---:|:---|:---|
| `file` | | `test_data/dump.bin` | Path to binary (`.bin`) or hex text (`.txt`) file to replay |
| `--port` | `-p` | Auto-detect | Serial port device (e.g. `/dev/ttyUSB0`, `/dev/ttyACM0`) |
| `--baud` | `-b` | `38400` | Serial baud rate (Raise/RZC protocol standard: 38400) |
| `--packet-delay` | `-d` | `0.05` | Delay in seconds between individual packets (allows HU MCU to process) |
| `--loop` | `-l` | `False` | Loop packet playback indefinitely |
| `--loop-delay` | | `1.0` | Delay in seconds between loop iterations |
| `--raw-stream` | | `False` | Stream entire binary payload at once without packet parsing |

---

## 6. Available Test Scenario CAN Logs

The following pre-recorded CAN captures are available in [`test/test_integration/data/`](file://test/test_integration/data/):

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

## 7. Manual Verification Checklist on Android HU

When replaying dumps or streaming CAN logs, verify the following interactive behaviors on the Head Unit:

- [ ] **UART Link Heartbeat:** The Head Unit should not display "CANBus Disconnected" or "No CAN Box".
- [ ] **Trip Computer Screen (`0x34` / `0x35`):** Displays Average Fuel Consumption (7.3 L/100km), Average Speed (37 km/h), and Trip Distance (56.9 km).
- [ ] **Trip Reset Animations:** Fuel/Speed display `--.-` followed by all fields clearing (`---`) upon trip reset.
- [ ] **Ambient Temperature (`0x36`):** Status bar displays outside temperature (~$5^\circ\text{C}$).
- [ ] **Lighting / Dimmer (`0x38`):** DRL icon, Adaptive Headlights, and interior mood lighting update.
- [ ] **Vehicle Settings & BSI Menu (`0x7D`):** Options in the Car Settings application reflect the vehicle configuration.
- [ ] **Steering Wheel Key Controls (`0x01` / `0x02`):** Volume Up/Down, Mode, Mute, Next/Prev track respond on the HU.

---

## 8. Troubleshooting & Diagnostics

### 8.1 "Permission Denied" on `/dev/ttyUSB0`
If your user does not have permission to access the serial port:
```bash
sudo usermod -aG dialout $USER
sudo chmod 666 /dev/ttyUSB0
```

### 8.2 Monitor Raw UART Data Sent to HU
To observe the exact binary packets being transmitted to the Head Unit in real time:
```bash
# Using hexdump to inspect raw UART stream from a second terminal
sudo minicom -D /dev/ttyUSB0 -b 38400 -H
```

### 8.3 Monitor CAN Messages on `vcan0`
To inspect the CAN stream generated by the replayer:
```bash
candump vcan0
```

### 8.4 Resetting the Virtual CAN Interface
If `vcan0` gets into a bad state:
```bash
sudo ip link delete vcan0 type vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```


