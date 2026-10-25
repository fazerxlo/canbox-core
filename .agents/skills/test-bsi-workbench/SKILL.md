---
name: test-bsi-workbench
description: >-
  Verify and test OpenCanbox Core firmware and profile implementations on the physical Peugeot 407
  workbench using SLCAN (can0), BSI, and RD4 radio. Includes interface bringup, bus sniffing, CAN-to-HU
  uplink validation, HU-to-CAN downlink injection, and guide for AI agents to author new test scenarios.
---

# PSA 407 BSI & SLCAN Workbench Verification Guide

This skill guides testing and validating **any** OpenCanbox Core subsystem (Personalization, Climate/HVAC, Trip Computer, Doors, Radio, SWC, Diagnostics) directly on the physical vehicle test bench.

---

## 0. Fast-Path Workbench Topology & Zero-Flounder Rules

### Persistent USB Hardware Identification (Auto-Discovery by VID:PID)
Port enumeration `/dev/ttyUSB*` changes when devices are replugged. All tools resolve ports automatically via `tools/canbox_usb_ports.py` using persistent USB hardware IDs:

| Role / Interface | Hardware Chip & USB VID:PID | Protocol & Parameters | Purpose & Auto-Discovery Behavior |
| :--- | :--- | :--- | :--- |
| **CAN Bus (SLCAN)** | **QinHeng CH340 (`1A86:7523`)** | PSA Comfort CAN @ **125 kbps** (`slcand -S 1000000 <dev> can0`) | Real Peugeot 407 BSI & RD4 bus. Automatically detected and excluded from UART probers. |
| **Canbox UART** | **FTDI FT232R (`0403:6001`)** | **38,400 baud, 8N1** (Raise `0x2E` or Hiworld `0x5A`) | Physical canbox UART TX/RX. Auto-selected by all tools when `--port` is omitted. |
| **Head Unit UART** | **Prolific PL2303 (`067B:2303`)** | **38,400 baud, 8N1** (Hiworld / Raise) | Android Head Unit serial connection. |
| **Baud Rate Rule** | Fixed **38,400 bps** | **NEVER toggle or probe down to 19,200 bps** | Peugeot 407 Raise & Hiworld canboxes operate strictly at 38,400 baud. Toggling baud causes framing break errors (`0x00` bytes). |

### Essential CAN Keepalives & Interlocks (Autonomous Wakeup Baseline)
1. **Sleeping ECUs Baseline:** If ECUs are asleep or the engine ECU is absent, broadcast continuously at 25–40 ms:
   - `0x036`: Phase de vie `0x01` (Active/Running): `0E 00 00 2A 01 00 00 A0`
   - `0x0F6`: Economy mode OFF (`0x88`), Contact ON (+APC): `88 00 00 00 00 50 00 20`
   - `0x128`: Key in ignition (+APC): `03 01 00 00 00 00 00 00`
   - `0x15B`: BSI power state: `01 03 B1 40 00 D0 00 10`
   - `0x168`: Instrument cluster status: `00 00 00 00 00 00 00 00`
2. **Climate (HVAC) Canbox Requirement:**
   - Commercial canboxes (Raise, Hiworld) **require CAN ID `0x1E3`** (`MSG_ETAT_CLIMAV_EMF`: `1C 40 0B 0B 00 00 07`) to report climate. `0x1D0` alone is silent.
3. **High-Power Consumer Interlock (Rear Defrost / Heated Mirrors):**
   - The BSI **completely blocks** Rear Defrost and Heated Mirrors if engine is stopped (`0x0B6` = `FF FF FF FF`).
   - **Must inject running engine:** `0x0B6#1C200000000000D0` (900 RPM) and `0x12D#8032320064000080` (Compressor authorized) for BSI to accept Rear Defrost button presses.
4. **Verified Rear Defrost CAN Ground Truth:**
   - **Fascia Panel Button Request (`0x1D0`):** **Byte 0 Bit 6** (`0x40`): `0x29` (OFF) $\longleftrightarrow$ `0x69` (ON).
   - **Climate ECU Status Indicator (`0x1E3`):** **Byte 1 Bit 3** (`0x08`): `0x00` (OFF) $\longleftrightarrow$ `0x08` (ON).
   - Serialized in Hiworld `0x31` `payload[2]` bit 5 (`0x20`) and Raise `0x21` `payload[0]` bit 0 (`0x01`).

---

## 1. Workbench Toolchain Reference (Token-Friendly Execution)

| Script | Purpose & Usage | Token-Friendly CLI Command |
| :--- | :--- | :--- |
| **`tools/probe_canbox_uart.py`** | Instant baud rate & protocol detection without port conflicts | `python3 tools/probe_canbox_uart.py --port /dev/ttyUSB0` |
| **`tools/workbench_live_sniffer.py`** | Real-time dual CAN delta + UART stream sniffer with delta-filtering | `python3 tools/workbench_live_sniffer.py` |
| **`tools/test_clima_canbox.py`** | Validates climate telemetry reporting across temperature sweeps | `python3 tools/test_clima_canbox.py` |
| **`tools/workbench_defrost_live_test.py`** | Live test with 900 RPM engine simulation for high-power button presses | `python3 tools/workbench_defrost_live_test.py` |
| **`tools/workbench_defrost_verifier.py`** | Automated 3-cycle state toggle auto-verifier & fuzzer | `python3 tools/workbench_defrost_verifier.py --port /dev/ttyUSB0 -v` |
| **`tools/update_canbox_usb.py`** | Flashes `can_app.iap` to connected USB drive with auto-sync and unmount | `python3 tools/update_canbox_usb.py` |

---

## 2. Workbench Architecture & Topology

```
+-------------------------------------------------------------------------+
|                              Host Linux PC                              |
|                                                                         |
|  [Test Harness]                 [OpenCanbox Core (Native/MCU)]          |
|  tools/test_bsi_workbench.py    .pio/build/native_test/program          |
|       |                                |                                |
|       | PTY / UART (/tmp/ttyCanbox)    | PTY / UART                     |
|       +--------------------------------+                                |
|       |                                                                 |
|       | SocketCAN (can0)                                                |
|       +--------------------------------+                                |
|                                        |                                |
|                                   [slcand -s6]                          |
|                                        |                                |
|                                   /dev/ttyUSB4                          |
|                                (CH340 USB-Serial)                       |
+----------------------------------------|--------------------------------+
                                         | CAN H / CAN L (125 kbps)
                                         v
+-------------------------------------------------------------------------+
|                         Physical Workbench                              |
|  - Peugeot 407 BSI (0x036, 0x0F6, 0x260, 0x361, 0x120)                   |
|  - Climate ECU / Façade (0x1D0, 0x1E3, 0x12D)                           |
|  - Blaupunkt RD4 Radio (0x165, 0x225, 0x2A5)                            |
+-------------------------------------------------------------------------+
```

---

## 2. Prerequisite: SLCAN (`can0`) Bringup & Recovery

PSA CAN2004 Comfort bus operates at **125 kbps** (SLCAN speed `-s6` in standard LAWICEL adapters with 16 MHz clock, or `-s4` on standard SLCAN).

### Step 1: Clean Stale Daemons & Setup Interface
If multiple `slcand` daemons exist or `can0` is down, reset and bind:

```bash
# 1. Clean any duplicate or hung slcand processes
sudo killall slcand 2>/dev/null

# 2. Attach serial device to network interface (e.g. /dev/ttyUSB4)
sudo slcan_attach -f -s6 -o /dev/ttyUSB4
sudo slcand -S 1000000 /dev/ttyUSB4 can0
sudo ip link set dev can0 up

# 3. Confirm can0 is UP
ip link show can0
```

### Step 2: Validate Live BSI Bus Activity
Ensure the BSI and radio are actively transmitting on `can0`:

```bash
# Sniff and decode active frames using the workbench tool:
./tools/test_bsi_workbench.py --sniff
```

Expected active frames when BSI and RD4 are powered:
* **`0x036` (Ignition/Dimmer):** `0E 00 00 0F 01 00 00 A0` (ACC=True, Dimmer=15)
* **`0x0F6` (Slow Telemetry):** `88 FF FF FF FF FF FF 20` (Economy mode inactive)
* **`0x260` (BSI Active Config Profile):** `01 00 B4 00 00 40 00 00` (Profile 1: FMH=15s, AutoLight=1, Radar=1)
* **`0x361` (BSI Equipment Availability Mask):** `01 01 91 40 30 10` (FMH/DRL/AAS supported)
* **`0x165` (RD4 Audio Source):** `C8 C0 40 00` (Source Tuner / Power ON)

> [!NOTE]
> If the bus goes quiet (0 frames), the BSI has entered economy/sleep mode. Broadcast ignition keepalive `0x036` (`0E 00 00 0F 01 00 00 A0`) and `0x0F6` (`88 FF FF FF FF FF FF 20`) to restore active operation.

---

## 3. The Universal Two-Way Verification Pattern

Every subsystem on the workbench translates bidirectionally between the vehicle CAN bus and the Head Unit serial protocol:

```
[Vehicle CAN Bus: can0] <--- (1) Inbound CAN / (4) Injected CAN ---> [canbox-core] <--- (2) Uplink / (3) Downlink ---> [HU UART: /tmp/ttyCanbox]
```

### Direction 1: Uplink Testing (CAN Bus $\to$ `canbox-core` $\to$ HU Telemetry)
Tests that when an ECU on the CAN bus updates state, `canbox-core` decodes it and outputs the corresponding serial telemetry to the Head Unit:
1. **Transmit CAN Frame:** Inject standard 11-bit CAN frame on `can0` via `harness.send_can_frame(can_id, payload)`.
2. **Sniff UART Stream:** Read `/tmp/ttyCanbox` via `harness.read_uart_packets(duration=0.3)`.
3. **Assert Protocol Wire Format:** Check that the expected Hiworld command (`0x12`, `0x13`, `0x31`, `0x76`, `0x79`, `0x84`, etc.) is received with the expected decoded bitfields and valid checksum.

### Direction 2: Downlink Testing (HU Downlink $\to$ `canbox-core` $\to$ CAN Bus & HU Uplink)
Tests that when the user performs a touchscreen action, `canbox-core` processes the command, injects the corresponding CAN frame, and updates the Head Unit:
1. **Send Downlink Packet:** Write framed Hiworld packet (`5A A5 <len> <cmd> <payload...> <cs>`) to UART via `harness.send_hu_packet(cmd, payload)`.
2. **Assert Immediate ACK:** Verify `canbox-core` replies with ACK frame `5A A5 01 FF <Cmd> <Checksum>`.
3. **Assert CAN Injection:** Sniff `can0` via `harness.read_can_frame(target_can_id, timeout_sec=0.2)` to ensure `canbox-core` injected the expected CAN frame onto the comfort bus.
4. **Assert Optimistic Uplink:** Verify that `canbox-core` immediately emits updated state telemetry on UART so the Android UI reflects the new switch or setpoint without lag.

---

## 4. How an AI Agent Writes a New Workbench Scenario

The test harness [`tools/test_bsi_workbench.py`](file:///home/Fazer/git/canbox-core/tools/test_bsi_workbench.py) uses a modular Scenario Registry pattern. When asked to implement or verify a new subsystem, follow these steps:

### Step 1: Subclass `BaseWorkbenchScenario`
In [`tools/test_bsi_workbench.py`](file:///home/Fazer/git/canbox-core/tools/test_bsi_workbench.py), create a new scenario class implementing the `run(self, harness: BsiWorkbenchHarness, filter_key: str = None) -> bool` method.

### Step 2: Implement Uplink & Downlink Tests
Use the following canonical template:

```python
class MySubsystemScenario(BaseWorkbenchScenario):
    name = "mysubsystem"
    description = "Short Description of Subsystem (CAN_IDs <-> HU_Cmds)"

    def run(self, harness: BsiWorkbenchHarness, filter_key: str = None) -> bool:
        print("\n" + "=" * 72)
        print("SCENARIO: MY SUBSYSTEM")
        print("=" * 72)

        all_passed = True

        # --- Test 1: Uplink (CAN -> Core -> HU) ---
        print("\n--- Test A1: CAN Frame 0xXYZ -> canbox-core -> HU 0xAB ---")
        can_payload = bytes([0x01, 0x02, 0x03, 0x04])
        harness.send_can_frame(0xXYZ, can_payload)

        pkts = harness.read_uart_packets(duration=0.3)
        relevant_pkts = [p for p in pkts if p[0] == 0xAB]

        uplink_ok = False
        if relevant_pkts:
            payload = relevant_pkts[-1][1]
            # Assert specific bitfield or value
            uplink_ok = (payload[0] == 0x01)

        flag = "[PASS]" if uplink_ok else "[FAIL]"
        if not uplink_ok: all_passed = False
        print(f"  {flag} CAN 0xXYZ -> HU 0xAB Telemetry (Match: {uplink_ok})")

        # --- Test 2: Downlink (HU -> Core -> CAN) ---
        print("\n--- Test B1: HU Downlink 0xCD -> canbox-core -> CAN 0xUVW ---")
        harness.drain_can(0.05)
        # Send Hiworld Downlink: Cmd 0xCD, Payload [0x01, 0x02]
        harness.send_hu_packet(0xCD, bytes([0x01, 0x02]))

        # 1. Assert ACK (Cmd 0xFF acknowledging Cmd 0xCD)
        rx_pkts = harness.read_uart_packets(duration=0.25)
        ack_ok = any(p[0] == 0xFF and len(p[1]) >= 1 and p[1][0] == 0xCD for p in rx_pkts)

        # 2. Assert CAN Injection on can0
        injected_can = harness.read_can_frame(0xUVW, timeout_sec=0.2)
        can_inj_ok = (injected_can is not None and injected_can[0] == 0x01)

        # 3. Assert Immediate Optimistic HU Telemetry
        echo_pkts = [p for p in rx_pkts if p[0] == 0xAB]
        echo_ok = len(echo_pkts) > 0

        pass_all = ack_ok and can_inj_ok and echo_ok
        flag = "[PASS]" if pass_all else "[FAIL]"
        if not pass_all: all_passed = False
        print(f"  {flag} HU 0xCD -> CAN 0xUVW (ACK={ack_ok}, CAN_Inj={can_inj_ok}, HU_Echo={echo_ok})")

        return all_passed
```

### Step 3: Register in `SCENARIO_REGISTRY`
Add the instantiated class to `SCENARIO_REGISTRY` at the bottom of [`tools/test_bsi_workbench.py`](file:///home/Fazer/git/canbox-core/tools/test_bsi_workbench.py):

```python
SCENARIO_REGISTRY: Dict[str, BaseWorkbenchScenario] = {
    "config": CarConfigScenario(),
    "climate": ClimateScenario(),
    "trip": TripScenario(),
    "doors": DoorsScenario(),
    "mysubsystem": MySubsystemScenario(),
}
```

### Step 4: Execute & Verify
Run the standalone scenario directly from the CLI:

```bash
./tools/test_bsi_workbench.py --scenario mysubsystem
```

---

## 5. Subsystem Reference Catalog

### 5.1 Dual-Zone Climate Control (HVAC)
* **CAN Status:** `0x1E3` (DLC 7, 100 ms) — EMF Climate State (Byte 0 Bit 0: Dual Mode, Byte 0 Bit 6: AC Off, Bytes 2/3: Temps).
* **CAN Demand:** `0x1D0` (DLC 7/8, 100 ms) — Climate Panel Request (Byte 0 Bit 0: `DUAL_REQ`, Byte 4: Defrost/Recirc).
* **HU Telemetry:** `Cmd 0x31` (12 payload bytes) — Byte 0 Bit 2: Dual Mode active, Byte 0 Bit 0: AC on.
* **HU Downlink:** `Cmd 0x3B` Setting `0x0F` — Toggle Dual/Mono temperature sync.
* **Command:** `./tools/test_bsi_workbench.py --scenario climate`

### 5.2 Car Personalization Options
* **CAN Status:** `0x260` (DLC 8, 1500 ms per profile) — BSI Active Personalization Settings (`MSG_BSI_INF_CFG_PERSO_ETAT`).
* **CAN Equipment Mask:** `0x361` (DLC 6, 500 ms) — Menu Availability (`MSG_BSI_INF_CFG_MENU_PERSO`).
* **CAN Downlink Injection:** `0x15B` (DLC 8, Event-driven) — Mutates setting via Profile 1 pending request (`MSG_BSI_DEM_CFG_PERSO`).
* **HU Telemetry:** `Cmd 0x76` (`CentralState1`), `Cmd 0x79` (`CentralState2`), `Cmd 0x71`/`0x72` (Feature Enables).
* **HU Downlink:** `Cmd 0x7B` (Primary options: DRL, Wiper, AutoLock, FMH, Ambient, Radar) & `Cmd 0x7D` (Tailgate, AFS).
* **Command:** `./tools/test_bsi_workbench.py --scenario config`

### 5.3 Trip Computer & Resets
* **CAN Status:** `0x221` (DLC 7, 1000 ms) — Instantaneous fuel, range, destination distance.
* **CAN Resets:** `0x221` pulsed with `0x80` (Trip 1 Clear) or `0x40` (Trip 2 Clear).
* **HU Telemetry:** `Cmd 0x13` (Instant), `Cmd 0x14` (Trip 1), `Cmd 0x15` (Trip 2).
* **HU Downlink:** `Cmd 0x1B` Setting `0x02` (Trip 1 Reset) / `0x03` (Trip 2 Reset).
* **Command:** `./tools/test_bsi_workbench.py --scenario trip`

### 5.4 Doors & Trunk Body Status
* **CAN Status:** `0x220` (DLC 2) — Byte 0 Bit 7: Driver door, Bit 6: Pass door, Bit 3: Trunk.
* **HU Telemetry:** `Cmd 0x12` (10 payload bytes) — Byte 2 carries door opening masks.
* **Command:** `./tools/test_bsi_workbench.py --scenario doors`

---

## 6. Workbench CLI Usage Reference

| Command | Action |
| :--- | :--- |
| `./tools/test_bsi_workbench.py` | Runs default (`config`) scenario suite |
| `./tools/test_bsi_workbench.py --scenario all` | Runs ALL registered scenarios (`config`, `climate`, `trip`, `doors`) |
| `./tools/test_bsi_workbench.py --scenario climate` | Runs Climate/HVAC test scenario only |
| `./tools/test_bsi_workbench.py --scenario trip` | Runs Trip Computer test scenario only |
| `./tools/test_bsi_workbench.py --scenario config --setting drl` | Filters specific setting within a scenario |
| `./tools/test_bsi_workbench.py --sniff` | Passively sniffs and decodes all active BSI CAN frames |
| `./tools/test_bsi_workbench.py --hu /dev/ttyUSB0` | Tests against physical MCU hardware instead of native PTY |
