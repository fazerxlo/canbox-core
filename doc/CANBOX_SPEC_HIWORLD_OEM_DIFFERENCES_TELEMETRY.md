# Hiworld OEM Commercial Firmware vs OpenCanbox Core — Telemetry Uplink Alignment & Protocol Differences

**Document File:** `doc/CANBOX_SPEC_HIWORLD_OEM_DIFFERENCES_TELEMETRY.md`  
**Document Version:** 1.0.0  
**Target Platform:** PSA Peugeot 407 & PSA CAN2004 Architecture (Comfort Bus @ 125 kbps)  
**Protocol Layer:** Hiworld UART Protocol (`0x5A 0xA5` Framing, 38,400 baud 8N1)  
**Reference Binary:** Commercial Hiworld Dongle `H1H2PA123A-240717` (Vehicle Model `0x22` / Group 3)

---

## 1. Executive Summary & Purpose

When integrating automotive head units with Peugeot 407 (PSA CAN2004), telemetry data flows between the CAN bus, the CAN box microcontroller, and the Android infotainment system. Discrepancies between commercial aftermarket firmware (factory Hiworld dongles) and OpenCanbox Core can lead to incorrect gauge displays, mismatched telemetry resolution, or test failures in automated System Verification Tests (SVT).

This specification documents the exact wire layouts, scaling factors, sentinel definitions, and architectural behaviors for:
1. **Powertrain Telemetry Uplink (`Cmd 0x1A`)**
2. **Trip Computer Telemetry Uplinks (`Cmd 0x13`, `0x14`, `0x15`)**
3. **Downlink Trip Reset Controls (`Cmd 0x1B`)**
4. **Virtual MDF Display Dependencies & Bus Emulation**

---

## 2. Powertrain Telemetry Uplink (`Cmd 0x1A`) Alignment

### 2.1 Side-by-Side Byte Layout

Both implementations use Hiworld framing with length `0x0C` (12 payload bytes):
`0x5A 0xA5 0x0C 0x1A <D0..D11> <Checksum>`

| Byte | Commercial Hiworld OEM Firmware (`ref_0x22_uplink.c` / `payload.asm`) | OpenCanbox Core Legacy / Initial Layout (`hiworld_psa_builders.c`) | Notes & Alignment Strategy |
|:---:|:---|:---|:---|
| **D0** | **Header Validity Flags (`0x21` or `0x35`):**<br>• Bit 0: ACC/IGN state (`0x01`)<br>• Bit 2: Speed/RPM valid latch<br>• Bit 5: Telemetry block active (`0x20`) | **Status Flags Bitmask:**<br>• Bit 0: Ignition ON<br>• Bit 1: Engine Running (`RPM > 400`)<br>• Bit 2: Reverse Gear<br>• Bit 3: Handbrake Active<br>• Bit 4: Fuel Low Warning | OEM firmware sets Bit 0 & Bit 5 (`0x21`) baseline; when powertrain data is active, Bit 2/4 are asserted (`0x35`). |
| **D1** | **Power / Direction Flags:**<br>• Bit 0: Ignition / ACC (`status_flags & 0x01`)<br>• Bit 1: Reverse Gear (`(status_flags >> 2) & 0x01`)<br>• Bits 7..2: Reserved (`0x00`) | **Gear Enum:**<br>`state->gear` (0=P, 1=R, 2=N, 3=D, 4=M1...) | OEM dedicates D1 strictly to reverse + ignition; gear latches are placed in D2. |
| **D2** | **Gear Position Latches:**<br>• Bit 3: `latch_gear_111 & 0x01`<br>• Bit 2: `latch_gear_112 & 0x01`<br>• Bits 1..0: `latch_gear_110 & 0x03` | **Vehicle Speed High Byte (km/h):**<br>`(uint8_t)(state->speed_kmh >> 8)` | In OEM firmware, D2 is the raw BSI gear selector nibble. |
| **D3** | **Vehicle Speed High Byte:**<br>`(uint8_t)(speed_0p1kmh >> 8)` | **Vehicle Speed Low Byte (km/h):**<br>`(uint8_t)(state->speed_kmh & 0xFF)` | **Resolution Difference:**<br>OEM encodes speed in **0.1 km/h units** (`raw_0B6_spd / 10`).<br>E.g., $110.0\text{ km/h} \to 1100 = \text{0x044C}$. |
| **D4** | **Vehicle Speed Low Byte:**<br>`(uint8_t)(speed_0p1kmh & 0xFF)` | **Engine RPM High Byte:**<br>`(uint8_t)(state->rpm >> 8)` | |
| **D5** | **MFD Display Mode Latch:**<br>Mode index (`0x05`) | **Engine RPM Low Byte (in `Cmd 0xEB`):**<br>`(uint8_t)(state->rpm & 0xFF)` | In OEM (`Cmd 0x1A`), carries current display mode (`0x05`). |
| **D6** | **Steering Wheel Angle High Byte (`WcComDataParser`):**<br>`(uint8_t)(u_angle >> 8)` from CAN `0x10B` (`-deg`) | **Coolant Temperature (°C, in `Cmd 0xEB`):**<br>`(uint8_t)(state->coolant_temp_c)` | Stock QF `WcComDataParser.parseBasicInfo()` parses D6..D7 as `notifySteeringWheelAngle`! |
| **D7** | **Steering Wheel Angle Low Byte (`WcComDataParser`):**<br>`(uint8_t)(u_angle & 0xFF)` from CAN `0x10B` (`-deg`) | **Oil Temperature (°C, in `Cmd 0xEB`):**<br>`(uint8_t)(state->oil_temp_c)` | OpenCanbox Core now formats `Cmd 0x1A` identically to OEM `WcComDataParser` and provides `Cmd 0xEB` for extended fluids/fuel. |
| **D8** | **Reverse Camera Trigger (`WcComDataParser`):**<br>`0x01` in Reverse, `0x00` otherwise | **Fuel Level Liters (in `Cmd 0xEB`):**<br>`state->fuel_liters` | |
| **D9** | **Engine RPM High Byte:**<br>`(uint8_t)(rpm >> 8)` | **Fuel Percentage (0..100%):**<br>`state->fuel_percentage` | In OEM firmware, RPM is extracted from CAN `0x0B6`: `((d[0] << 8) | d[1]) >> 3`. |
| **D10** | **Engine RPM Low Byte:**<br>`(uint8_t)(rpm & 0xFF)` | **Instant Fuel Consumption High Byte:**<br>`(uint8_t)(instant_fuel_deci >> 8)` | E.g., $2400\text{ RPM} = \text{0x0960}$. |
| **D11** | **Padding / Reserved (`0x00`)** | **Instant Fuel Consumption Low Byte:**<br>`(uint8_t)(instant_fuel_deci & 0xFF)` | |

---

## 3. Trip Computer Uplinks (`Cmd 0x13`, `0x14`, `0x15`)

### 3.1 Packet Opcodes & Allocations
- **Cmd `0x13` (Length = 10): Instantaneous Telemetry**
  - Instantaneous fuel consumption (0.1 L/100km)
  - Cruising range / distance to empty (km)
  - Remaining distance to destination (km)
  - External ambient temperature
- **Cmd `0x14` (Length = 10): Trip 1 Historical Telemetry**
  - Average fuel consumption (0.1 L/100km)
  - Average speed (km/h)
  - Trip distance traveled (km)
- **Cmd `0x15` (Length = 10): Trip 2 Historical Telemetry**
  - Average fuel consumption (0.1 L/100km)
  - Average speed (km/h)
  - Trip distance traveled (km)

### 3.2 Stationary & Uncalculated Sentinel Alignment
When the vehicle is stationary or when trip counters are cleared/uncalculated:
- PSA CAN frame `0x221` sets instantaneous consumption to `0xFFFF` and destination distance to `0xFFFF`.
- **OEM Firmware Behavior:** Directly packages `0xFF` / `0xFFFF` into the corresponding payload fields of `Cmd 0x13`. The Android APK (`com.qf.vehicle`) receives `0xFFFF` and displays `--.-` on the UI.
- **OpenCanbox Core Behavior:** Normalized to forward `0xFFFF` without replacing it with synthetic zeros, matching OEM behavior and passing automated SVT tests.

---

## 4. Downlink Trip Reset Controls (`Cmd 0x1B`) & Virtual MDF Coexistence

### 4.1 Downlink Packet Wire Formats
The Android Head Unit issues `Cmd 0x1B` when the user presses reset on the trip computer screens:
- **Android QF Application Transmission:**
  - Reset Trip 1: `5A A5 04 1B 02 02 01 FF <CS>`
  - Reset Trip 2: `5A A5 04 1B 03 03 01 FF <CS>`
  - Tab Switch / Navigation: `5A A5 04 1B 01 00 01 FF <CS>`

### 4.2 Handling Differences
- **Hiworld OEM Firmware:**
  - Evaluates `if (len == 3U)` in `case 0x1BU` (`ref_0x22_uart_rx_can_tx.c`).
  - Does **not** inject CAN frame `0x167` or synthesize stalk button pulses.
  - Relies completely on the factory physical Multifunction Display (EMF-A / EMF-C) connected on the CAN bus to handle trip calculations and reset logic.
- **OpenCanbox Core:**
  - Supports both standard 3-byte and 4-byte QF frames.
  - Implements **Virtual MDF Emulation Mode**: when no physical EMF display is present on the bus, OpenCanbox emulates display presence and injects CAN `0x167` reset commands, enabling Android touchscreen resets even on cars where the factory screen has been removed.

---

## 5. Architectural Comparison: Uplink Cadence & Throttling

| Characteristic | Commercial Hiworld OEM Firmware | OpenCanbox Core Implementation |
|---|---|---|
| **Sampling & Refresh Timer** | Hardcoded 41.3 ms timer (every 5th tick of an 8.26 ms hardware interrupt) latches CAN values into shadow buffers (`ref_0x22_uplink.c`). | Event-driven on CAN RX with 10 ms rate-limit filter. |
| **Uplink Dispatcher** | 8.26 ms main loop evaluates `memcmp` against shadow buffers across all 19 subsystem commands. | Delta-detection dispatch via `s_current_state` and `s_last_sent_state` in `can_router.c`. |
| **Bus Saturation Protection** | Does not rate-limit bursty multi-packet emissions during high CAN bus load; relies on 38,400 baud hardware FIFO. | Rate-limited transmission slots: fast telemetry ($\le 10\text{ Hz}$), slow telemetry ($\le 1\text{–}2\text{ Hz}$). |
| **Android Silent Reboot Sync** | Lacks persistent full-resync mechanism after silent Android MCU soft reboot. | Dual-tier resync: immediate burst upon handshake `Cmd 0x24`/`0x30`, followed by a 60-second periodic state refresh. |

---

## 6. SVT Test Suite Adaptation & Compatibility Layer

To support regression testing across both OpenCanbox firmware and physical commercial Hiworld hardware (`hiworld_oem_407_A`), the SVT framework (`svt/lib/protocols/hiworld.py`) incorporates dual-mode decoding:

1. **Automatic Header & Layout Detection:**
   ```python
   # Detect OEM 0x1A layout vs OpenCanbox layout:
   # OEM uses D0=0x21/0x35 and places RPM at D9..D10
   is_oem = (payload[0] in (0x21, 0x35) or (len(payload) >= 11 and payload[9] > 0 and payload[4] == 0))
   if is_oem:
       # OEM layout: Speed in 0.1 km/h at bytes 3..4, RPM at bytes 9..10
       raw_spd = (payload[3] << 8) | payload[4]
       speed_kmh = int(round(raw_spd / 10.0))
       rpm = (payload[9] << 8) | payload[10]
   else:
       # OpenCanbox layout: Speed in integer km/h at bytes 2..3, RPM at bytes 4..5
       speed_kmh = (payload[2] << 8) | payload[3]
       rpm = (payload[4] << 8) | payload[5]
   ```

2. **DUT Capability Gating:**
   DUTs are configured via YAML profile (`svt/duts/hiworld_oem_407_A.yaml`). Features not natively supported by the commercial dongle (e.g. `virtual_mdf: false`) are gracefully skipped (`[SKIP]`), ensuring 100% actionable pass/fail reporting.

