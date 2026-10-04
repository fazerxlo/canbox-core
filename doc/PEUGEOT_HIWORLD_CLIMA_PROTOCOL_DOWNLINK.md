# Hiworld Peugeot Protocol: Sending Climate State (0x31) to Head Unit (HU)

This specification describes how a CANbox, MCU, ESP32, or CAN emulator must construct and transmit the climate state packet (**`0x31`**) to the Android Head Unit (HU) running the Hiworld (`wc`) protocol driver.

---

## 1. Frame Layout & Protocol Wrapper

Every message sent from the CANbox to the HU uses the **Hiworld 5AA5 protocol frame**:

```
 0     1     2     3     4     5     6     7     8     9    10    11    12    13    14    15    16
+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
| 5A  | A5  | Len | 31  | D0  | D1  | D2  | D3  | D4  | D5  | D6  | D7  | D8  | D9  | D10 | D11 | Chk |
+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
```

* **Header (SOF):** `0x5A 0xA5`
* **Length (`Len`):** `0x0C` (12 data bytes `D0` .. `D11`)
* **Command ID:** `0x31` ([`PeugeotDataDefine$Handle.CarAcState`](#L22))
* **Payload Offset:** Data starts at **array index 4** (`D0 = packet[4]`)
* **Total Frame Size:** 17 bytes (2 header + 1 length + 1 command ID + 12 data + 1 checksum)

### Checksum Algorithm
The checksum byte at the end of the packet is computed by summing all bytes starting from `packet[1]` (`0xA5`) up through the last data byte, subtracting `1`, and masking to 8 bits:

$$\text{Checksum} = \left(\sum_{i=1}^{\text{len}+3} \text{packet}[i] - 1\right) \ \& \ \text{0xFF}$$

In C/C++ / Arduino:
```c
uint8_t calculate_checksum(const uint8_t *packet, uint8_t total_length) {
    uint8_t sum = 0;
    for (int i = 1; i < total_length - 1; i++) {
        sum += packet[i];
    }
    return (uint8_t)(sum - 1);
}
```

---

## 2. Complete Data Byte Map (CANbox -> HU)

| Data Byte | Packet Index | Description | Key Fields |
|:---:|:---:|:---|:---|
| **D0** | `packet[4]` | Basic Controls & Mode | Power, A/C Max, Rear Power, Auto, **Mono/Dual (SYNC)**, A/C |
| **D1** | `packet[5]` | Circulation & Air Quality | Recirculation (Cycle), AQS |
| **D2** | `packet[6]` | Defrost / Fog Clear | Rear Window Heater, Front Windshield Defrost |
| **D3** | `packet[7]` | Blower Profile Level | Auto Wind Strength Profile (Soft / Normal / Fast) |
| **D4** | `packet[8]` | **Airflow Direction** | **Upper 4 bits: Right Airflow** / **Lower 4 bits: Left Airflow** |
| **D5** | `packet[9]` | Fan / Blower Speed | Speed level `0x00 .. 0x08` |
| **D6** | `packet[10]` | Left (Driver) Temperature | Encoded temperature (`0xFE`=LO, `0xFF`=HI, or `val / 2.0 °C`) |
| **D7** | `packet[11]` | Right (Pass) Temperature | Encoded temperature (`0xFE`=LO, `0xFF`=HI, or `val / 2.0 °C`) |
| **D8** | `packet[12]` | Reserved | Typically `0x00` |
| **D9** | `packet[13]` | Reserved | Typically `0x00` |
| **D10** | `packet[14]` | Reserved | Typically `0x00` |
| **D11** | `packet[15]` | Outside Ambient Temperature | `(val * 0.5) - 40.0 °C` (Send `0x00` if not available) |

---

## 3. Deep Dive: Mono / Dual (SYNC) State

Mono/Dual status is read directly from **Data Byte 0 (`packet[4]`)**, specifically **Bit 2 (`0x04`)**:

```
Data Byte 0 (packet[4]) Bitfield:
 +-------+-------+-------+-------+-------+-------+-------+-------+
 | Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
 |   -   | Power | AcMax | RearPw| Auto  | SYNC  |      A/C      |
 +-------+-------+-------+-------+-------+-------+-------+-------+
```

### Exact Bit Definitions:
* **Bit 2 (`0x04`) — Mono / Dual (SYNC):**
  * **`1` = MONO / SYNC Active** (Passenger temperature is linked to driver)
  * **`0` = DUAL / SYNC Inactive** (Driver and passenger temperatures operate independently)
* **Bit 6 (`0x40`) — Climate Power:**
  * `1` = System ON, `0` = System OFF
* **Bit 5 (`0x20`) — A/C Max:**
  * `1` = Max A/C ON, `0` = OFF
* **Bit 4 (`0x10`) — Rear Climate Power:**
  * `1` = Rear ON, `0` = Rear OFF
* **Bit 3 (`0x08`) — AUTO Mode:**
  * `1` = Auto mode ON, `0` = Manual mode
* **Bits 1..0 (`0x03`) — A/C Compressor:**
  * `0x01` = A/C compressor ON
  * `0x00` = A/C compressor OFF

### Head Unit Smali Verification ([`PeugeotDataParser.smali#L4042-L4060`](#L4042-L4060)):
```smali
aget-byte v1, p1, p0       # v1 = packet[4]
const/4 v7, 0x2
shr-int/2addr v1, v7       # v1 = packet[4] >> 2
and-int/2addr v1, v3       # v1 = (packet[4] >> 2) & 1
if-ne v1, v3, :cond_4
move v1, v3                # true
goto :goto_4
:cond_4
move v1, v4                # false
:goto_4
iput-boolean v1, v0, Lcom/qf/vehicle/entity/AcState;->mSync:Z
```

---

## 4. Deep Dive: Left and Right Airflow Direction

Airflow distribution is encoded in **Data Byte 4 (`packet[8]`)**.

> [!WARNING]
> **Common Bug Alert:**
> 1. **Nibble Order:** The **Upper 4 bits are RIGHT (Passenger)** and the **Lower 4 bits are LEFT (Driver)**. If your arrows are mirrored or wrong, this nibble order is usually reversed in the firmware.
> 2. **Values are NOT Bitmasks:** Hiworld uses specific enumeration numbers (e.g. `0x06` for face, `0x0B` for windshield), **not** single bit flags (`0x01`, `0x02`, `0x04`).

```
Data Byte 4 (packet[8]):
+-------------------------------+-------------------------------+
|      Bits 7 .. 4 (High)       |       Bits 3 .. 0 (Low)       |
|    Right / Passenger Vents    |     Left / Driver Vents       |
+-------------------------------+-------------------------------+
```

### Supported Airflow Nibble Codes:

| Nibble Value (Hex) | Decimal | Active Direction | Smali Variables Set to `true` |
|:---:|:---:|:---|:---|
| **`0x03`** | 3 | **Floor / Footwell** | `mWindDown` |
| **`0x05`** | 5 | **Face + Floor** (Bi-Level) | `mWindParallel` + `mWindDown` |
| **`0x06`** | 6 | **Face / Center Vents** | `mWindParallel` |
| **`0x0B`** | 11 | **Windshield / Defrost** | `mWindUp` |
| **`0x0C`** | 12 | **Windshield + Floor** | `mWindUp` + `mWindDown` |
| **`0x0D`** | 13 | **Windshield + Face** | `mWindUp` + `mWindParallel` |
| **`0x0E`** | 14 | **Windshield + Face + Floor** | `mWindUp` + `mWindParallel` + `mWindDown` |
| `0x00` (or other) | 0 | None / Auto default | All direction booleans `false` |

### Head Unit Smali Verification ([`PeugeotDataParser.smali#L4174-L4328`](#L4174-L4328)):

#### Right (Passenger) Airflow Parsing:
```smali
aget-byte v4, p1, v1       # v4 = packet[8]
shr-int/lit8 v4, v4, 0x4   # v4 = packet[8] >> 4 (Upper Nibble)
and-int/lit8 v4, v4, 0xf   # v4 = (packet[8] >> 4) & 0x0F
# Compared against 3, 5, 6, and switch [0x0B .. 0x0E] -> sets mRightWindUp/Parallel/Down
```

#### Left (Driver) Airflow Parsing:
```smali
aget-byte v1, p1, v1       # v1 = packet[8]
and-int/lit8 v1, v1, 0xf   # v1 = packet[8] & 0x0F (Lower Nibble)
iput-byte v1, v0, Lcom/qf/vehicle/entity/AcState;->mWindMode:B
# Compared against 3, 5, 6, and switch [0x0B .. 0x0E] -> sets mWindUp/Parallel/Down
```

### Combined Data Byte 4 Examples:
* **Both sides to Face vents (`0x06` left, `0x06` right):**
  $$\text{Byte 4} = (0x6 \ll 4) \mid 0x6 = \mathbf{0x66}$$
* **Both sides to Footwell (`0x03` left, `0x03` right):**
  $$\text{Byte 4} = (0x3 \ll 4) \mid 0x3 = \mathbf{0x33}$$
* **Both sides to Windshield / Defrost (`0x0B` left, `0x0B` right):**
  $$\text{Byte 4} = (0xB \ll 4) \mid 0xB = \mathbf{0xBB}$$
* **Both sides to Bi-Level Face + Footwell (`0x05` left, `0x05` right):**
  $$\text{Byte 4} = (0x5 \ll 4) \mid 0x5 = \mathbf{0x55}$$
* **Dual Airflow Test (Left to Face `0x06`, Right to Footwell `0x03`):**
  $$\text{Byte 4} = (0x3 \ll 4) \mid 0x6 = \mathbf{0x36}$$
  *(Note: Driver = lower nibble `0x6`, Passenger = upper nibble `0x3`)*

---

## 5. Other Payload Fields (D1 .. D11)

### Data Byte 1 (`packet[5]`): Circulation & AQS
* **Bit 4 (`0x10`) — Recirculation:**
  * `(val >> 4) & 1 == 1`: Internal Recirculation ON
  * `0`: Fresh Air / Outside Intake
* **Bit 3 (`0x08`) — AQS (Air Quality System):**
  * `(val >> 3) & 1 == 1`: AQS Active

### Data Byte 2 (`packet[6]`): Defrosters
* **Bit 5 (`0x20`) — Rear Window Defroster / Mirror Heater:**
  * `(val >> 5) & 1 == 1`: ON
* **Bit 4 (`0x10`) — Front Windshield Fast Defrost:**
  * `(val >> 4) & 1 == 1`: ON

### Data Byte 3 (`packet[7]`): Auto Blower Intensity Level
* **Bits 1..0 (`0x03`):**
  * `0`: Soft
  * `1`: Normal
  * `2`: Fast

### Data Byte 5 (`packet[9]`): Blower / Fan Speed
* Raw integer fan level from `0x00` (Off) to `0x08` (Max).

### Data Bytes 6 & 7 (`packet[10]`, `packet[11]`): Left & Right Temperatures
Encoded in half-degree Celsius steps:
* `0xFE`: Displays **"Low"** (`LO`)
* `0xFF`: Displays **"High"** (`HI`)
* Any other value: Temperature in °C = $\frac{\text{val}}{2.0}$
  * `0x28` (40) $\rightarrow$ **20.0 °C**
  * `0x29` (41) $\rightarrow$ **20.5 °C**
  * `0x2A` (42) $\rightarrow$ **21.0 °C**
  * `0x2B` (43) $\rightarrow$ **21.5 °C**
  * `0x2C` (44) $\rightarrow$ **22.0 °C**

### Data Byte 11 (`packet[15]`): Outside Ambient Temperature
* Temperature in °C = $(\text{val} \times 0.5) - 40.0$
  * Set to `0x00` or omit if unneeded.

---

## 6. Complete Raw Frame Examples to Test

### Example 1: Standard Dual Mode, Face Vents, 21.5°C Driver, 23.0°C Passenger
* Power ON (`0x40`), AC ON (`0x01`), **Dual Mode (Sync Bit 2 = 0)** $\rightarrow$ **D0 = `0x41`**
* Recirculation Fresh Air $\rightarrow$ **D1 = `0x00`**
* Defrosters OFF $\rightarrow$ **D2 = `0x00`**
* Wind Strength Normal $\rightarrow$ **D3 = `0x01`**
* Airflow: Left Face (`0x6`), Right Face (`0x6`) $\rightarrow$ **D4 = `0x66`**
* Fan Speed 3 $\rightarrow$ **D5 = `0x03`**
* Left Temp: 21.5 °C ($21.5 \times 2 = 43 = 0x2B$) $\rightarrow$ **D6 = `0x2B`**
* Right Temp: 23.0 °C ($23.0 \times 2 = 46 = 0x2E$) $\rightarrow$ **D7 = `0x2E`**
* Reserved bytes $\rightarrow$ **D8..D10 = `0x00 0x00 0x00`**
* Outside Temp: 22.0 °C ($(22 + 40) \times 2 = 124 = 0x7C$) $\rightarrow$ **D11 = `0x7C`**

```text
Payload: 41 00 00 01 66 03 2B 2E 00 00 00 7C
Full Frame:
5A A5 0C 31 41 00 00 01 66 03 2B 2E 00 00 00 7C C7
```
*(Checksum: `A5 + 0C + 31 + 41 + 00 + 00 + 01 + 66 + 03 + 2B + 2E + 00 + 00 + 00 + 7C - 1 = 0x02C7 -> 0xC7`)*

---

### Example 2: MONO Mode ON (Sync Enabled), Bi-Level Vents, 20.0°C Both Sides
* Power ON (`0x40`), AC ON (`0x01`), **Mono Mode ON (Sync Bit 2 = 1, `0x04`)** $\rightarrow$ **D0 = `0x45`** (`0x40 | 0x04 | 0x01`)
* Airflow: Left Bi-Level (`0x5`), Right Bi-Level (`0x5`) $\rightarrow$ **D4 = `0x55`**
* Fan Speed 4 $\rightarrow$ **D5 = `0x04`**
* Left Temp: 20.0 °C ($20 \times 2 = 40 = 0x28$) $\rightarrow$ **D6 = `0x28`**
* Right Temp: 20.0 °C ($20 \times 2 = 40 = 0x28$) $\rightarrow$ **D7 = `0x28`**

```text
Payload: 45 00 00 01 55 04 28 28 00 00 00 00
Full Frame:
5A A5 0C 31 45 00 00 01 55 04 28 28 00 00 00 00 93
```
*(Checksum: `A5 + 0C + 31 + 45 + 00 + 00 + 01 + 55 + 04 + 28 + 28 + 00 + 00 + 00 + 00 - 1 = 0x0193 -> 0x93`)*

---

## 7. Checklist for Debugging Common Bugs

1. **Mono / Dual not toggling on HU?**
   * Check **Byte 4 (`D0`), Bit 2 (`0x04`)**.
   * It must be `1` for Mono (Sync ON) and `0` for Dual. Do NOT put this in Byte 1 or Byte 2.
2. **Airflow arrows not lighting up?**
   * Check **Byte 8 (`D4`)**.
   * Are you sending bitmasks instead of enum values? Remember:
     * Face is `6`, **not** `2` or `1`.
     * Feet is `3`, **not** `4`.
     * Defrost is `0x0B` (11), **not** `1`.
3. **Airflow inverted between driver and passenger?**
   * Upper nibble `D4[7..4]` is **Right / Passenger**.
   * Lower nibble `D4[3..0]` is **Left / Driver**.
4. **HU ignoring the entire frame?**
   * Check length byte: `packet[2] = 0x0C` (12 data bytes).
   * Check command ID: `packet[3] = 0x31`.
   * Check checksum: Must be 1-byte sum starting from index 1 (`0xA5`) to index 15, minus 1.
