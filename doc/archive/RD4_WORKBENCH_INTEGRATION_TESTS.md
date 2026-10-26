# PSA RD4 Workbench Automated Integration Tests

This specification and user guide documents the automated bench integration test suite for verifying factory PSA RD4 radios (Blaupunkt / Continental / Bosch) paired with `canbox-core` firmware.

---

## 1. System Overview & Workbench Architecture

When testing the RD4 on a workbench without vehicle BSI interference, the CAN shield is connected directly to the RD4 radio. The test runner connects to both **SocketCAN (`can0`)** and the **Head Unit UART serial interface** (`/dev/ttyUSB*` @ 38,400 baud).

```
                      +---------------------------------------+
                      |   Automated Workbench Test Runner     |
                      |    (tools/test_rd4_workbench.py)      |
                      +---------------------------------------+
                             |                         |
              HU Downlink Injection             SocketCAN Sniffing &
             & Uplink Telemetry Verify            Autonomous BSI Sim
               (38,400 baud Serial)                 (125 kbps CAN)
                             |                         |
                             v                         v
                      +-------------+           +--------------+
                      | canbox-core |<--------->| RD4 Radio    |
                      +-------------+  CAN 21F  +--------------+
                                       CAN 165
```

---

## 2. Hardware Wiring & Bench Setup

### 2.1 RD4 Quadlock Connector Pinout

| Pin | Quadlock Block | Function | Connection |
|:---:|:---:|:---|:---|
| **Pin 12** | Part A (Power) | Chassis Ground (GND) | Power Supply (-) & CAN Ground |
| **Pin 15** | Part A (Power) | Battery Continuous (+12V BAT) | Power Supply (+12V, 2–3A) |
| **Pin 10** | Part A (Power) | CAN High (CAN-CONFORT) | CAN Shield / Transceiver CAN_H |
| **Pin 13** | Part A (Power) | CAN Low (CAN-CONFORT) | CAN Shield / Transceiver CAN_L |
| **Pin 4**  | Part B (Audio) | AUX 2 / CDC Audio Left (+) | 3.5mm Jack / Audio Line Out |
| **Pin 10** | Part B (Audio) | AUX 2 / CDC Audio Right (+) | 3.5mm Jack / Audio Line Out |
| **Pin 11** | Part B (Audio) | Audio Signal Ground (GND) | 3.5mm Jack Shield / Audio GND |

> [!IMPORTANT]
> **CAN Bus Termination:** Ensure a 120 $\Omega$ termination resistor is placed between CAN_H (Pin 10) and CAN_L (Pin 13). Without termination, RD4 transceivers will suffer packet collisions and frame errors.

### 2.2 USB Interfaces

1. **CAN Adapter (`can0`):** USB-to-CAN adapter (e.g. slcan CH340, Candlelight, CANable) configured for **125 kbps** (`speed 125000`).
2. **Head Unit Serial (`/dev/ttyUSB4`):** USB-to-UART adapter (PL2303, FT232, CH340) connected to the CAN Box's HU UART port (TX $\to$ RX, RX $\to$ TX, GND $\to$ GND) at **38,400 baud**.

---

## 3. Autonomous BSI Wakeup Simulator

When the RD4 is connected on the workbench **without the car's BSI**, the radio will power off within 30 seconds (Economy Mode) unless valid BSI wakeup packets are present.

The test runner incorporates an autonomous background BSI keeper thread (`BsiSimulator`) that transmits every 100 ms:
- **`CAN 0x036` (`MSG_BSI_RAP_VOL`):** `0E 00 00 2A 01 00 00 A0` (Ignition ON, engine running status)
- **`CAN 0x0F6` (`MSG_BSI_LENT`):** `80 00 00 00 00 00 00 00` (Economy Mode disabled)

*(If a real BSI is connected to the CAN bus during testing, use the `--no-bsi` flag to disable the software simulator).*

---

## 4. Test Scenarios

### Test 1: Pre-flight Verification
* **Check A (CAN Connectivity):** Listens for `CAN 0x165` to verify RD4 is powered on (`power_status == 1`) and active.
* **Check B (UART Connectivity):** Transmits ping packet `5A A5 02 24 22 00 47` to CAN box and expects ACK `0xFF 0x24`.

### Test 2: Random Source Switching (Closed-Loop Convergence & Overshoot Protection)
* **Execution:** Executes $N$ randomized hops across sources (`Tuner`, `CD`, `CDC`, `AUX`).
* **Verifications:**
  1. **Immediate ACK:** Asserts CAN box emits `5A A5 01 FF AD AC` within 150 ms.
  2. **Pulse Formation:** Confirms `CAN 0x21F` emits single 50 ms atomic pulses (`02 00 00` $\to$ `00 00 00`) with $\ge 150\text{ ms}$ inter-pulse settle windows.
  3. **Hardware Ground-Truth:** Confirms `CAN 0x165` byte 2 high nibble switches to the target source within 3.5 seconds.
  4. **Overshoot Guard:** Holds observation for 1.2 seconds after target acquisition to verify the radio **does not slide past** or execute extra pulses.

### Test 3: Tuner Station List & Frequency Preservation (`0x0F`)
* **Execution:** Puts RD4 into Tuner mode and records the current FM frequency from `CAN 0x225`. Injects Station List Query `5A A5 01 0F 0F`.
* **Verifications:**
  1. Asserts `CAN 0x21F` emits rotary scroll up (`00 01 00`) followed ~80 ms later by restore down (`00 00 00`).
  2. Asserts CAN box pushes `0x84` and `0x85` (presets/list) over UART.
  3. Re-reads `CAN 0x225` and confirms frequency is **identical** to the pre-test value (confirming the thumbwheel scroll did not de-tune the station).

### Test 4: Rapid Mid-Flight Retargeting (Stress Test)
* **Execution:** Sends `Switch to CD` (`0x03 0x02`), and 30 ms later sends `Switch to CDC` (`0x02 0x01`).
* **Verifications:** Asserts arbiter updates target mid-flight, steps cleanly to CDC without deadlocking, and locks at CDC.

### Test 5: CDC Playback State Toggling (`0x162` Status Heartbeat)
* **Execution:**
  1. Sends `5A A5 02 AD 02 01 B1` (CDC Active / Play) $\to$ Asserts `CAN 0x162` Byte 1 has bit 1 set and status icon = 3 (Play).
  2. Sends `5A A5 02 AD 02 00 B0` (CDC Standby / Pause) $\to$ Asserts `CAN 0x162` Byte 1 has status icon = 2 (Pause) and bit 1 cleared.

### Test 6: Physical Button Override & 800 ms Debounce Lockout (Rule F3)
* **Execution:** Injects physical stalk press on `CAN 0x21F` while on CDC.
* **Verifications:** Asserts RD4 steps to the next source, CAN box suppresses uplink during the 800 ms debounce window, and emits UART `0x84` with the new source once settled.

---

## 5. Usage & CLI Options

```bash
# Standard test run with auto-detected serial ports:
./tools/test_rd4_workbench.py -y

# Explicit interface and serial port configuration:
./tools/test_rd4_workbench.py --can can0 --hu /dev/ttyUSB4

# Custom number of randomized source switching hops (e.g. 15 hops):
./tools/test_rd4_workbench.py --can can0 --hu /dev/ttyUSB4 --hops 15

# Testing with real vehicle BSI connected (disables software BSI simulation):
./tools/test_rd4_workbench.py --can can0 --hu /dev/ttyUSB4 --no-bsi
```

---

## 6. Log Analysis & Post-Mortem Debugging

Every test run writes a microsecond-timestamped log file: `rd4_workbench_test_YYYY-MM-DD_HH-MM-SS.log`.

### Log Prefixes

| Tag | Origin | Description |
|:---|:---|:---|
| `[BSI_SIM]` | Test Script | Status of 0x036/0x0F6 ignition keeper frames |
| `[CAN_TX]` | Test Script / Canbox | CAN frames transmitted onto `can0` |
| `[CAN_RX]` | RD4 / Bus | CAN frames received from the physical RD4 radio |
| `[HU_TX]` | Test Script | Downlink packets injected into CAN Box HU port |
| `[HU_RX]` | CAN Box | Uplink telemetry frames emitted by CAN Box |
| `[TEST_RESULT]` | Test Engine | Assertion outcome and reason |

### Example Test Log Snippet
```text
1792244222.183 [HU_TX] RAW:5aa502ad0201b1
1792244222.183 [HU_RX] CMD:0xFF RAW:5aa501ffadac
1792244222.183 [CAN_RX] ID:21F DLC:3 DATA:020000
1792244222.218 [CAN_RX] ID:165 DLC:4 DATA:c8d02000
1792244222.233 [CAN_RX] ID:21F DLC:3 DATA:000000
1792244222.279 [CAN_RX] ID:165 DLC:4 DATA:c8d03000
1792244222.279 [HU_RX] CMD:0x84 RAW:5aa50e843100000000012020202020202020c3
1792244222.279 [TEST_RESULT] Hop 1: TUNER -> CDC: PASS - Clean convergence in 0.10s (no overshoot)
```

