# Peugeot Hiworld (WC) Climate Protocol Specification

This document details the reverse-engineered CAN communication protocol for Peugeot climate control under the Hiworld CANbus box implementation (**`wc`** supplier), extracted from `QF_Canbus_system`.

---

## 1. Protocol Overview & Frame Format

* **Supplier ID / Rule:** `WcVehicleDataRuleHead5aa5` (`canprovider_hiworld_5aa5`)
* **Header / SOF:** `0x5A 0xA5`
* **Parser:** [`PeugeotDataParser.smali`](file:///home/Fazer/git/QF_Canbus_system/smali/com/qf/vehicle/band/peugeot/parse/wc/PeugeotDataParser.smali)
* **Controller:** [`PeugeotDataController.smali`](file:///home/Fazer/git/QF_Canbus_system/smali/com/qf/vehicle/band/peugeot/parse/wc/PeugeotDataController.smali)
* **Definitions:** [`PeugeotDataDefine.smali`](file:///home/Fazer/git/QF_Canbus_system/smali/com/qf/vehicle/band/peugeot/parse/wc/PeugeotDataDefine.smali)

### Frame Layout
All frames follow the standard Hiworld `5A A5` frame format:

| Offset | Field | Value / Description |
|:---:|:---|:---|
| `0` | Header 1 | `0x5A` |
| `1` | Header 2 | `0xA5` |
| `2` | Length | Data payload length `N` |
| `3` | Command ID | CAN message identifier (e.g. `0x3B`, `0x31`) |
| `4 .. 4+N-1` | Data Payload | `N` bytes of payload |
| `4+N` | Checksum | `(sum(Byte[1 .. 3+N]) - 1) & 0xFF` |

### Checksum Algorithm
Implemented in [`WcVehicleDataRuleHead5aa5.calCheckSum(...)`](file:///home/Fazer/git/QF_Canbus_system/smali_classes2/com/qf/vehicle/supplier/wc/WcVehicleDataRuleHead5aa5.smali#L44-L75):
```java
byte calCheckSum(byte[] packet, int startOffset, int endOffset) {
    int sum = 0;
    for (int i = startOffset + 1; i < endOffset - 1; i++) {
        sum += packet[i];
    }
    return (byte)((sum - 1) & 0xFF);
}
```

---

## 2. Mono / Dual (SYNC) Control

In PSA / Peugeot systems, temperature synchronization (Mono/Dual) is controlled via the **SYNC** function.

### A. Sending the Command (Headunit -> CAN Box)
* **Command ID:** `0x3B` ([`PeugeotDataDefine$Command.ForwardAcSetting`](file:///home/Fazer/git/QF_Canbus_system/smali/com/qf/vehicle/band/peugeot/parse/wc/PeugeotDataDefine$Command.smali#L18))
* **Length:** `0x02`
* **Data Byte 0 (Control Function):** `0x0F` (15 decimal)
* **Data Byte 1 (Value):**
  * `0x01` = **Mono / Sync ON** (Passenger temperature follows driver)
  * `0x00` = **Dual / Sync OFF** (Independent left & right temperatures)

#### Smali Logic ([`PeugeotDataParser.smali#L305-L322`](file:///home/Fazer/git/QF_Canbus_system/smali/com/qf/vehicle/band/peugeot/parse/wc/PeugeotDataParser.smali#L305-L322)):
```smali
:pswitch_3
const/16 v0, 0xf          # Function code 0x0F
aput-byte v0, v1, v2      # Data[0] = 0x0F
add-int/2addr v2, v5      # index++
iget-boolean p2, p2, Lcom/qf/vehicle/entity/AcState;->mSync:Z
xor-int/2addr p2, v5      # Toggle mSync bit: mSync ^ 1
int-to-byte p2, p2
aput-byte p2, v1, v2      # Data[1] = inverted sync bit
```

#### Raw Frame Examples:
* **Enable Mono (Sync ON):**
  ```text
  5A A5 02 3B 0F 01 4C
  ```
  *(Checksum: `(0x02 + 0x3B + 0x0F + 0x01 - 1) & 0xFF = 0x4C`)*

* **Disable Mono / Enable Dual (Sync OFF):**
  ```text
  5A A5 02 3B 0F 00 4B
  ```
  *(Checksum: `(0x02 + 0x3B + 0x0F + 0x00 - 1) & 0xFF = 0x4B`)*

---

### B. Receiving / Reading Status (CAN Box -> Headunit)
* **Command ID:** `0x31` ([`PeugeotDataDefine$Handle.CarAcState`](file:///home/Fazer/git/QF_Canbus_system/smali/com/qf/vehicle/band/peugeot/parse/wc/PeugeotDataDefine$Handle.smali#L22))
* **Offset:** Data Byte 0 (Byte 4 of overall packet)
* **Bit Position:** Bit 2 (`0x04`)

#### Smali Logic ([`PeugeotDataParser.smali#L4042-L4061`](file:///home/Fazer/git/QF_Canbus_system/smali/com/qf/vehicle/band/peugeot/parse/wc/PeugeotDataParser.smali#L4042-L4061)):
```smali
aget-byte v1, p1, p0       # Data Byte 0
const/4 v7, 0x2
shr-int/2addr v1, v7       # Shift right by 2 bits
and-int/2addr v1, v3       # Mask with 0x01
...
iput-boolean v1, v0, Lcom/qf/vehicle/entity/AcState;->mSync:Z
```

* `Bit 2 == 1`: **Mono Mode** (Synchronized)
* `Bit 2 == 0`: **Dual Mode** (Independent)

---

## 3. Airflow Direction Control

### A. Independent Left vs. Right Direction Capability
> [!IMPORTANT]
> **Peugeot / PSA vehicles do NOT support independent left and right airflow direction.**
>
> 1. **Vehicle Hardware:** PSA dual-zone HVAC units have separate left/right blend doors for **temperature mixing**, but share a single set of distribution doors/stepper motors (defrost, face, floor) across the entire cabin.
> 2. **Hiworld Protocol:** Command `0x3B` defines only single global airflow distribution codes (`0x08`, `0x09`, `0x0A`).
> 3. **Application Layer:** Generic right-side wind setting types (`RightWindUp = 0x5F`, `RightWindParallel = 0x60`, `RightWindDown = 0x61`) are intentionally unmapped in `PeugeotDataParser.forwardAcState`.

---

### B. Sending Airflow Commands (Headunit -> CAN Box)
* **Command ID:** `0x3B`
* **Length:** `0x02`

| Function ID | Smali Switch Case | Airflow Direction | Data 0 | Data 1 |
|:---:|:---:|:---|:---:|:---:|
| `0x0E` (14) | `:pswitch_2` | **Wind Up** (Windshield / Defrost) | `0x08` | `mWindUp ^ 1` (`0x01` / `0x00`) |
| `0x0F` (15) | `:pswitch_1` | **Wind Parallel** (Face / Center vents) | `0x09` | `mWindParallel ^ 1` (`0x01` / `0x00`) |
| `0x10` (16) | `:pswitch_0` | **Wind Down** (Floor / Footwell) | `0x0A` | `mWindDown ^ 1` (`0x01` / `0x00`) |

#### Smali Logic ([`PeugeotDataParser.smali#L248-L304`](file:///home/Fazer/git/QF_Canbus_system/smali/com/qf/vehicle/band/peugeot/parse/wc/PeugeotDataParser.smali#L248-L304)):
```smali
:pswitch_0                 # SettingType 0x10 (Wind Down)
const/16 v0, 0xa           # Code 0x0A
aput-byte v0, v1, v2
add-int/2addr v2, v5
iget-boolean p2, p2, Lcom/qf/vehicle/entity/AcState;->mWindDown:Z
xor-int/2addr p2, v5       # Toggle value
aput-byte p2, v1, v2

:pswitch_1                 # SettingType 0x0F (Wind Parallel)
const/16 v0, 0x9           # Code 0x09
aput-byte v0, v1, v2
add-int/2addr v2, v5
iget-boolean p2, p2, Lcom/qf/vehicle/entity/AcState;->mWindParallel:Z
xor-int/2addr p2, v5       # Toggle value
aput-byte p2, v1, v2

:pswitch_2                 # SettingType 0x0E (Wind Up)
const/16 v0, 0x8           # Code 0x08
aput-byte v0, v1, v2
add-int/2addr v2, v5
iget-boolean p2, p2, Lcom/qf/vehicle/entity/AcState;->mWindUp:Z
xor-int/2addr p2, v5       # Toggle value
aput-byte p2, v1, v2
```

#### Raw Frame Examples:
* **Toggle Wind Up (Defrost):**
  ```text
  5A A5 02 3B 08 01 45
  ```
* **Toggle Wind Parallel (Face):**
  ```text
  5A A5 02 3B 09 01 46
  ```
* **Toggle Wind Down (Floor):**
  ```text
  5A A5 02 3B 0A 01 47
  ```

---

### C. Receiving / Reading Airflow Status (CAN Box -> Headunit)
* **Command ID:** `0x31`
* **Offset:** Data Byte 4 (Byte 8 of overall packet)

#### Nibble Structure ([`PeugeotDataParser.smali#L4174-L4328`](file:///home/Fazer/git/QF_Canbus_system/smali/com/qf/vehicle/band/peugeot/parse/wc/PeugeotDataParser.smali#L4174-L4328)):
* **Bits 7..4 (Upper nibble):** Passenger / Right-side reported mode
* **Bits 3..0 (Lower nibble):** Driver / Left-side reported mode

*(Note: In Peugeot cars, both nibbles report identical values because the mechanical flaps are coupled.)*

| Nibble Value (Hex) | Active Airflow Modes |
|:---:|:---|
| `0x03` | Floor / Footwell only (`mWindDown`) |
| `0x05` | Face + Floor (`mWindParallel` + `mWindDown`) |
| `0x06` | Face only (`mWindParallel`) |
| `0x0B` | Windshield only (`mWindUp`) |
| `0x0C` | Windshield + Floor (`mWindUp` + `mWindDown`) |
| `0x0D` | Windshield + Face (`mWindUp` + `mWindParallel`) |
| `0x0E` | Windshield + Face + Floor (`mWindUp` + `mWindParallel` + `mWindDown`) |

---

## 4. Summary of CAN Commands (Command ID `0x3B`)

For reference, the complete command table implemented in [`PeugeotDataParser.forwardAcState`](file:///home/Fazer/git/QF_Canbus_system/smali/com/qf/vehicle/band/peugeot/parse/wc/PeugeotDataParser.smali#L180-L613) is:

| Function Code (Data[0]) | Value (Data[1]) | Description |
|:---:|:---:|:---|
| `0x01` | `0x00` / `0x01` | Power OFF / ON |
| `0x02` | `0x00` / `0x01` | A/C Compressor OFF / ON |
| `0x03` | `0x00` / `0x01` | A/C MAX OFF / ON |
| `0x04` | `0x00` / `0x01` | AUTO Mode OFF / ON |
| `0x05` | `0x00` / `0x01` | Front Defrost (Fog Clear) OFF / ON |
| `0x06` | `0x00` / `0x01` | Rear Window Defogger OFF / ON |
| `0x07` | `0x00` / `0x01` | Recirculation (Cycle) OFF / ON |
| `0x08` | `0x00` / `0x01` | Wind Up (Windshield) |
| `0x09` | `0x00` / `0x01` | Wind Parallel (Face) |
| `0x0A` | `0x00` / `0x01` | Wind Down (Footwell) |
| `0x0B` | `0x01` / `0x02` | Left Temp (1 = Decrease, 2 = Increase) |
| `0x0C` | `0x01` / `0x02` | Right Temp (1 = Decrease, 2 = Increase) |
| `0x0D` | `0x01` / `0x02` | Blower Speed (1 = Decrease, 2 = Increase) |
| `0x0E` | `0x00 .. 0x07` | Blower Intensity Level (Wind Strength) |
| `0x0F` | `0x00` / `0x01` | **Mono / Dual (Sync OFF / ON)** |
| `0x10` | `0x00` / `0x01` | AQS (Air Quality Sensor) OFF / ON |
| `0x11` | `0x00` / `0x01` | Rear Climate Power OFF / ON |
