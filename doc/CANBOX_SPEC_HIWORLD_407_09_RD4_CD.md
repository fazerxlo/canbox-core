# Peugeot RD4 CD Player & CD Changer (CDC) Firmware Uplink Specification

## Document Target: CAN Box Firmware Developer & Firmware AI Agents
**Applicable Vehicle Platforms:** PSA Peugeot 407, 307, 207, 308, 607, 807, Citroën C4, C5 (RD4 Audio Architecture)  
**Target Head Unit Application:** `com.qf.vehicle.activities.OriginalMediaPlayer` ("Car CD" UI)  
**Target Hardware Protocol:** Hiworld PSA CAN Box UART Protocol (38400 baud, 8N1)  
**Reference Codebase:** `QF_Canbus_system` (`PeugeotDataParser.smali`, `OriginalMediaPlayer.smali`, `CarMediaState.smali`)

---

## 1. Architectural Overview

In PSA vehicles equipped with the RD4 audio system, CD audio playback is handled either by the **single internal in-dash CD drive** inside the RD4 head unit, or by an **optional external 6-disc CD Changer (CDC)** (typically manufactured by Blaupunkt/Clarion and mounted in the glovebox or boot).

The CAN Box MCU sits between the vehicle's low-speed body CAN network (**CAN-CONFORT at 125 kbps**) and the Android Head Unit (connected via **UART serial at 38400 baud**).

```
 +-------------------------------------------------------+
 |                 PSA Vehicle Bus (125 kbps)            |
 |                                                       |
 |   [RD4 In-Dash CD / External 6-Disc CDC]              |
 |          |                                            |
 |          | CAN 0x165 (RD4 Audio Source: 0x2 = CD)     |
 |          | CAN 0x365 (CD Disc Info: Total Tracks)     |
 |          | CAN 0x3A5 (CD Track & Playback Elapsed)    |
 |          | CAN 0x325 (CD Drive / Tray Mechanism)      |
 |          | CAN 0x3A6 (Optional 6-Disc CDC Telemetry)  |
 |          | CAN 0x0A4 (CD-Text & MP3 ID3 Tags)         |
 +----------|--------------------------------------------+
            v
 +-------------------------------------------------------+
 |             CAN Box MCU (Firmware Agent)              |
 |  - Reads CAN-CONFORT frames                           |
 |  - Tracks CD playback state machine                   |
 |  - Packages data into Hiworld UART frames             |
 +-------------------------------------------------------+
            |
            | UART Serial (38400 baud, 8N1)
            | Cmd 0x97 (CarCdPlayInfo: Disc, Track, Time, Status)
            | Cmd 0x84 (Audio Source: 0x30 = CD, 0x31 = CDC)
            v
 +-------------------------------------------------------+
 |         Android Head Unit (com.qf.vehicle)            |
 |                                                       |
 |   PeugeotDataParser -> CarMediaState                  |
 |          |                                            |
 |          v                                            |
 |   OriginalMediaPlayer (media_player.xml)              |
 |   - Active Disc & Track (e.g. "5/18")                 |
 |   - Elapsed Duration (e.g. "02:35")                   |
 |   - Status: "Play", "Pause", "Loading", "Eject"       |
 |   - 6 Disc Magazine Slot Statuses (In/Out/Play)       |
 +-------------------------------------------------------+
```

---

## 2. Head Unit UI Layout & Data Bindings (`OriginalMediaPlayer`)

The Android application activity `com.qf.vehicle.activities.OriginalMediaPlayer` binds directly to `CarMediaState` updated by `PeugeotDataParser.smali::parseCarMediaState`.

| UI Element ID | Resource Name | Target View | Rendered Format / Semantics |
|---|---|---|---|
| `0x7f080324` | `@id/curIndex` | `TextView` | Track Index: `"  <TrackIndex>/<TotalTracks>"` (e.g. `"  5/18"`). If total is 0, renders `"  <TrackIndex>"`. |
| `0x7f080321` | `@id/curPlayDuration` | `TextView` | Elapsed Time: `"  <Minutes>:<Seconds>"` formatted with zero-padded seconds (e.g. `"  02:35"`). |
| `0x7f080328` | `@id/curWorkMode` | `TextView` | Status String: `"  <Status>"` (e.g. `"  Play"`, `"  Pause"`, `"  Fast forward"`, `"  Loading"`). |
| `0x7f080753` | `@id/playPause` | `ImageView` | Playback State Icon: **Level 1** = Play (active green icon), **Level 0** = Paused. |
| `0x7f080479`..`47e` | `@id/iconDisk1`..`6` | `ImageView` | Active Disc Indicator: Level 1 for the currently active slot `mDiskIndex`, Level 0 for inactive slots. |
| `0x7f0808b2`..`8b7` | `@id/stateDisk1`..`6` | `TextView` | Slot Presence: Shows `"in disc"` (`0x7f0b0f3a`) when loaded, `"out disc"` (`0x7f0b1260`) when empty, or `"play"` (`0x7f0b12d2`) when playing. |
| `0x7f080357` | `@id/diskPrandParent`| `LinearLayout`| 6-disc magazine slot container (visible when `mCd6DisksEnable == 1`). |
| `0x7f0807c4` | `@id/randomParent` | `ImageView` | Track Shuffle / Random icon (`mRandom == 1`). |
| `0x7f0802c6` | `@id/cycleParent` | `ImageView` | Track Repeat icon (`mRepeat == 1`). |

---

## 3. Hiworld UART Wire Protocol Specification

### 3.1 Packet Framing
CD Changer / Media packets use opcode **`Cmd 0x97`** with an 11-byte payload:

```
+------------+------------+------------+------------+-----------------------+------------+
| SYNC 1     | SYNC 2     | LENGTH     | CMD        | DATA PAYLOAD (11 B)   | CHECKSUM   |
| 0x5A       | 0xA5       | 0x0B       | 0x97       | Data[0] .. Data[10]   | CS         |
+------------+------------+------------+------------+-----------------------+------------+
```

* **Total Wire Length:** 16 bytes.
* **Length Byte (`LEN`):** Always `0x0B` (11 decimal).
* **Command Opcode (`CMD`):** `0x97` (`-0x69t`).
* **Checksum Formula:**
  $$\text{Checksum} = \left( \left( \text{LEN} + \text{CMD} + \sum_{i=0}^{10} \text{Data}[i] \right) - 1 \right) \ \& \ \mathtt{0xFF}$$

---

### 3.2 Byte-by-Byte Payload Definition (`Data[0 .. 10]`)

| Byte Index | Field Name | Type | Allowed Range | Description & Bit Assignments |
|---|---|---|---|---|
| **`Data[0]`** | **Active Disc Index** | `uint8` | `0x00 .. 0x06` | Lower 4 bits (`& 0x0F`): Active disc slot.<br>• **Single In-Dash CD:** Set to `0x00` (as verified in OEM CAN box captures) or `0x01`.<br>• **6-Disc CDC:** Set to `0x01 .. 0x06`. |
| **`Data[1]`** | **Discs Loaded Mask** | `uint8` | `0x00 .. 0x3F` | Bitmask of loaded disc slots (`1 = Loaded / Present`, `0 = Empty`):<br>• Bit 0 (`0x01`): Disc Slot 1 Loaded<br>• Bit 1 (`0x02`): Disc Slot 2 Loaded<br>• Bit 2 (`0x04`): Disc Slot 3 Loaded<br>• Bit 3 (`0x08`): Disc Slot 4 Loaded<br>• Bit 4 (`0x10`): Disc Slot 5 Loaded<br>• Bit 5 (`0x20`): Disc Slot 6 Loaded<br>*Single In-Dash CD:* Set to `0x00` (single internal drive has no magazine slots) or `0x01`. *CDC full:* `0x3F`. |
| **`Data[2]`** | **Disc Format Mask** | `uint8` | `0x00 .. 0x3F` | Bitmask of disc audio formats (`0 = Standard CD-DA Audio`, `1 = MP3 CD`):<br>• Bit 0 (`0x01`): Disc 1 is MP3<br>• Bit 1 (`0x02`): Disc 2 is MP3<br>• ... Bit 5 (`0x20`): Disc 6 is MP3<br>*Standard Audio CD:* `0x00`. |
| **`Data[3]`** | **Track Number Lo** | `uint8` | `0x01 .. 0xFF` | **Little-endian Low Byte** of current track number (`uint16_le`). |
| **`Data[4]`** | **Track Number Hi** | `uint8` | `0x00 .. 0x03` | **Little-endian High Byte** of current track number (`uint16_le`).<br>$\text{Track} = \text{Data}[3] \mid (\text{Data}[4] \ll 8)$. |
| **`Data[5]`** | **Elapsed Minutes** | `uint8` | `0 .. 59` | Current playback minutes on active track. |
| **`Data[6]`** | **Elapsed Seconds** | `uint8` | `0 .. 59` | Current playback seconds on active track (`0 .. 59`). |
| **`Data[7]`** | **Playback Modes** | `uint8` | Bitmask | Playback mode flags:<br>• Bit 7 (`0x80`): Track Scan (`SCAN`)<br>• Bit 6 (`0x40`): Disc Scan<br>• Bit 5 (`0x20`): Track Repeat (Repeat 1 / Loop Track)<br>• Bit 4 (`0x10`): Disc Repeat (Repeat All Discs)<br>• Bit 3 (`0x08`): Track Random (Shuffle active disc)<br>• Bit 2 (`0x04`): Disc Random (Shuffle all discs)<br>• Bits 1..0: Reserved (`0`). |
| **`Data[8]`** | **Playback Status** | `uint8` | Enum | Raw status code mapped to UI string & Play/Pause icon (see Section 3.3).<br>• **`0x02` = Play / Playing**<br>• **`0x01` = Pause**<br>• **`0x06` = Stop**<br>• **`0x03` = Fast Forward**<br>• **`0x0D` = Loading / Reading**<br>• **`0x0C` = Eject** |
| **`Data[9]`** | **Total Tracks Lo** | `uint8` | `0x01 .. 0xFF` | **Little-endian Low Byte** of total tracks on active disc (`uint16_le`). |
| **`Data[10]`**| **Total Tracks Hi** | `uint8` | `0x00 .. 0x03` | **Little-endian High Byte** of total tracks on active disc (`uint16_le`).<br>$\text{Total} = \text{Data}[9] \mid (\text{Data}[10] \ll 8)$. |

---

### 3.3 Critical Traps & Protocol Idiosyncrasies

#### Trap 1: Endianness (`uint16_le` Little-Endian)
The decompiled parsing method [`PeugeotDataParser.smali::parseCarMediaState`](file:///home/Fazer/git/QF_Canbus_system/QF_Canbus_system/smali/com/qf/vehicle/band/peugeot/parse/wc/PeugeotDataParser.smali#L5281) calls:
```smali
invoke-static {p1, v1, 2, true}, Lcom/qf/vehicle/utils/JavaDecodeUtil;->byteArrToInt([BIIZ)I
```
Passing `true` as the 4th parameter (`isLittleEndian`) instructs `JavaDecodeUtil` to calculate:
$$\text{Value} = \text{byte}[0] \ | \ (\text{byte}[1] \ll 8)$$
> [!CAUTION]
> **Do NOT send big-endian words!**  
> If Track 5 is transmitted as big-endian `0x00 0x05`, the Head Unit decodes it as $0 \mid (5 \ll 8) = \mathbf{1280}$. Always transmit `Data[3] = 0x05`, `Data[4] = 0x00`.

#### Trap 2: Playback Status Code Wire Mapping (`Data[8]`)
`PeugeotDataParser.smali` translates the raw byte in `Data[8]` through internal `packed-switch :pswitch_data_0` before passing it to `OriginalMediaPlayer`. Here is the exact mapping:

| CAN Box UART Wire `Data[8]` | Internal Parser Enum | UI Display String (`curWorkMode`) | Play/Pause Icon Level (`playPause`) |
|:---:|:---:|---|:---:|
| **`0x02`** | `3` | **`"Play"`** | **Level 1 (Green / Active)** |
| **`0x01`** | `2` | **`"Pause"`** | **Level 0 (Paused)** |
| **`0x06`** | `7` | **`"Stop"`** | Level 0 |
| **`0x03`** | `22` (`0x16`) | **`"Fast forward"`** | Level 0 |
| **`0x00`** | `1` | **`"read"`** (TOC read) | Level 0 |
| **`0x0D`** | `27` (`0x1B`) | **`"Loading"`** | Level 0 |
| **`0x0C`** | `13` (`0x0D`) | **`"Eject"`** | Level 0 |
| **`0x0A`** | `11` (`0x0B`) | **`"Disk changing(User)"`** | Level 0 |
| **`0x0B`** | `12` (`0x0C`) | **`"Disk changing(Internal)"`** | Level 0 |
| **`0x0E`** | `20` (`0x14`) | **`"Error"`** | Level 0 |

> [!IMPORTANT]
> To indicate normal playback, the CAN box **MUST transmit `Data[8] = 0x02`**, NOT `0x01`. Sending `0x01` will cause the UI to display `"Pause"`.

---

## 4. Multi-Packet Synchronization: Audio Source Transition (`Cmd 0x84`)

The Head Unit manages audio source context using `Cmd 0x84`. When the user presses the **`SOURCE`** button on the RD4 or steering stalk to select CD playback, the CAN box must transmit both `Cmd 0x84` and `Cmd 0x97`:

```
                       +------------------------------+
                       | User Presses SOURCE -> CD    |
                       +------------------------------+
                                      |
                 +--------------------+--------------------+
                 |                                         |
                 v                                         v
   +---------------------------+             +---------------------------+
   |   Hiworld Frame Cmd 0x84  |             |   Hiworld Frame Cmd 0x97  |
   |   Audio Source Frame      |             |   Media Playback Frame    |
   +---------------------------+             +---------------------------+
   | Data[0] = 0x30 (CD)       |             | Data[0]  = 0x01 (Disc 1)  |
   | Data[1..2] = 0x0000       |             | Data[1]  = 0x01 (Present) |
   | Data[3] = 0x00            |             | Data[3..4] = 0x05, 0x00   |
   | Data[5] = 0x01 (Power On) |             | Data[5..6] = 0x02, 0x23   |
   | Data[6..13] = "CD      "  |             | Data[8]  = 0x02 (Play)    |
   +---------------------------+             +---------------------------+
```

### Source Codes in `Cmd 0x84` `Data[0]`:
* `0x30`: **Internal RD4 In-Dash CD Player** (Station name ASCII: `"CD      "`).
* `0x31`: **External CD Changer (CDC)** (Station name ASCII: `"CDC     "`).
* `0x00 .. 0x03`: Radio Tuner (FM1, FM2, FM3, etc. - switches foreground context to Radio).
* `0x20` / `0x21`: Line-In AUX 1 / AUX 2.
* `0xFF`: Power OFF / Standby.

---

## 5. Vehicle CAN-CONFORT Bus Reverse-Engineering (125 kbps)

On PSA vehicles (Peugeot 407/307/607/807, Citroën C4/C5), audio frames are broadcast on the Comfort CAN bus at **125 kbps (11-bit standard identifiers)**:

### 5.1 CAN ID `0x165` — RD4 Audio Source Mode (`ETAT_AUTORADIO`)
Broadcast periodically by the RD4 head unit every 50 ms (DLC: 4 bytes):

| Byte | Bits | Field | Values / Meaning | Conversion to Hiworld UART |
|:---:|:---:|---|---|---|
| **0** | `7..0` | Status Byte 1 | Typically `0xC8` / `0xCC` | Power status |
| **1** | `7..0` | Status Byte 2 | Typically `0xC0` / `0x54` | Audio system flags |
| **2** | `7..4` | **INPUT_SOURCE** | `0x1` = Tuner (FM/AM)<br>`0x2` = **Internal CD**<br>`0x3` = **CD Changer (CDC)**<br>`0x4` = AUX 1<br>`0x5` = AUX 2 | • If `0x2`: Set `Cmd 0x84 Data[0] = 0x30` (`CD`)<br>• If `0x3`: Set `Cmd 0x84 Data[0] = 0x31` (`CDC`) |
| **2** | `3..0` | Sub-status | Typically `0x0` | Internal mode flags |
| **3** | `7..0` | Status Byte 4 | Typically `0x00` / `0x02` | Mute / audio path state |

---

### 5.2 CAN ID `0x365` — RD4 Internal CD Disc Information (`INFO_CD_DISC`)
Broadcast periodically when an internal audio/MP3 CD is loaded (DLC: 5 bytes):

| Byte | Field | Allowed Values | Conversion to Hiworld UART |
|:---:|---|---|---|
| **0** | **Total Tracks on Disc** | `0x01 .. 0x63` (1..99) | Direct count of tracks on the disc (e.g. `0x0C` = 12 tracks).<br>Pack into `Cmd 0x97 Data[9] = Byte 0`, `Data[10] = 0x00` (`uint16_le`). |
| **1** | CD Format / Mechanism Status | Typically `0x26` | Disc format & type indicators |
| **2** | Disc Attributes / Audio Mode | Typically `0x02` | `0x00` = Data, `0x02` = CD-DA Audio |
| **3** | Reserved | `0x00` | Reserved |
| **4** | Reserved | `0x00` | Reserved |

---

### 5.3 CAN ID `0x3A5` — RD4 Internal CD Track & Playback Telemetry (`ETAT_CD_PLAY`)
Broadcast periodically every 1,000 ms (1.0 Hz) during playback (DLC: 6 bytes):

| Byte | Field | Allowed Values | Conversion to Hiworld UART |
|:---:|---|---|---|
| **0** | **Current Track Number** | `0x01 .. 0x63` (1..99) | Direct current track number (e.g. `0x05` = Track 5, `0x03` = Track 3).<br>Pack into `Cmd 0x97 Data[3] = Byte 0`, `Data[4] = 0x00` (`uint16_le`). |
| **1** | Track Index / Sequence | Typically `0x03` | Internal track sequence / sub-index. |
| **2** | **Drive Status & Play Modes** | Bitmask / Status code | • `0x19`, `0x1E`: Normal Play &rarr; Map to `Cmd 0x97 Data[8] = 0x02` (`Play`), `Data[7] = 0x00` (Normal Mode).<br>• `0x25`: Fast Forward / Seeking &rarr; Map to `Cmd 0x97 Data[8] = 0x02` (`Play`).<br>• `0x01`: Pause &rarr; Map to `Cmd 0x97 Data[8] = 0x01` (`Pause`).<br>• `0x00`: Stop &rarr; Map to `Cmd 0x97 Data[8] = 0x06` (`Stop`). |
| **3** | **Elapsed Minutes** | `0x00 .. 0x3B` (0..59) | Pack into `Cmd 0x97 Data[5] = Byte 3`. |
| **4** | **Elapsed Seconds** | `0x00 .. 0x3B` (0..59) | Pack into `Cmd 0x97 Data[6] = Byte 4` (increments by 1 every second). |
| **5** | Sub-status | Typically `0x00` | Mechanism substatus. |

---

### 5.4 CAN ID `0x325` — RD4 Internal CD Tray / Drive Mechanism Status (`MECANISME_CD`)
Broadcast on CD insertion, ejection, and mechanism state changes (DLC: 3 bytes):

| Byte | Field | Values / Meaning |
|:---:|---|---|
| **0** | Mechanism Status | `0x00` = Idle / Normal, `0x80` = Tray / Disc State Active |
| **1** | Disc Presence | `0x03` = Disc Loaded / Present in drive slot |
| **2** | Reserved | `0x00` |

---

### 5.5 CAN ID `0x3A6` — External 6-Disc CD Changer (CDC) State (`CDC_STATUS`)
Emitted periodically (500 ms) when the optional external glovebox/boot multi-disc changer is installed (DLC: 8 bytes):

| Byte | Field | Allowed Values | Conversion to Hiworld UART |
|:---:|---|---|---|
| **0** | Reserved | `0x00` | Reserved |
| **1** | **Active Disc Slot** | `0x01 .. 0x06` | Pack into `Cmd 0x97 Data[0] = Byte 1`. |
| **2** | **Current Track Number** | `0x01 .. 0x63` | Pack into `Cmd 0x97 Data[3] = Byte 2`, `Data[4] = 0x00` (`uint16_le`). |
| **3** | **Total Tracks on Disc** | `0x01 .. 0x63` | Pack into `Cmd 0x97 Data[9] = Byte 3`, `Data[10] = 0x00` (`uint16_le`). |
| **4** | **Elapsed Minutes** | `0x00 .. 0x3B` | Pack into `Cmd 0x97 Data[5] = Byte 4`. |
| **5** | **Elapsed Seconds** | `0x00 .. 0x3B` | Pack into `Cmd 0x97 Data[6] = Byte 5`. |
| **6** | **Play Mode Flags** | Bitmask | Bit 0: Random (`RND`), Bit 1: Scan, Bit 2: Repeat (`RPT`). Pack into `Cmd 0x97 Data[7]`. |
| **7** | Reserved | `0x00` | Reserved |

---

### 5.6 CAN ID `0x0A4` — CD-Text & MP3 Track Titles
When an MP3 CD or CD-Text disc is inserted into an RD4 N2 (MP3-capable model), song titles and artists are broadcast using **ISO-TP segmented multi-frame transport** (CAN IDs `0x0A4` / `0x0E4`).  
The CAN box can extract the ASCII/UTF-8 track title and transmit it via `Cmd 0x86` (Radio Text / Media Title) or extended metadata fields.

---

## 6. Reference C Firmware Implementation

The following clean C implementation can be compiled directly into STM32 / ESP32 CAN box firmware:

```c
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define HIWORLD_SYNC_1          0x5A
#define HIWORLD_SYNC_2          0xA5
#define HIWORLD_CMD_CD_PLAY     0x97
#define HIWORLD_CMD_AUDIO_SRC   0x84

/* Playback Status Codes for Cmd 0x97 Data[8] */
typedef enum {
    HIWORLD_CD_STATUS_READ    = 0x00,
    HIWORLD_CD_STATUS_PAUSE   = 0x01,
    HIWORLD_CD_STATUS_PLAY    = 0x02,
    HIWORLD_CD_STATUS_FF      = 0x03,
    HIWORLD_CD_STATUS_STOP    = 0x06,
    HIWORLD_CD_STATUS_EJECT   = 0x0C,
    HIWORLD_CD_STATUS_LOADING = 0x0D,
    HIWORLD_CD_STATUS_ERROR   = 0x0E
} hiworld_cd_status_t;

/* Playback Mode Flags for Cmd 0x97 Data[7] */
#define HIWORLD_CD_MODE_SCAN_TRACK    (1 << 7)
#define HIWORLD_CD_MODE_SCAN_DISC     (1 << 6)
#define HIWORLD_CD_MODE_REPEAT_TRACK  (1 << 5)
#define HIWORLD_CD_MODE_REPEAT_DISC   (1 << 4)
#define HIWORLD_CD_MODE_RANDOM_TRACK  (1 << 3)
#define HIWORLD_CD_MODE_RANDOM_DISC   (1 << 2)

/**
 * Calculates the standard Hiworld checksum.
 * Formula: ((Length + Cmd + sum(Data)) - 1) & 0xFF
 */
static uint8_t hiworld_calc_checksum(uint8_t len, uint8_t cmd, const uint8_t *payload) {
    uint16_t sum = len + cmd;
    for (uint8_t i = 0; i < len; i++) {
        sum += payload[i];
    }
    return (uint8_t)((sum - 1) & 0xFF);
}

/**
 * Builds a 16-byte Hiworld CD Media Status packet (Cmd 0x97).
 *
 * @param out_buf       Output buffer (must be at least 16 bytes).
 * @param disc_slot     Active disc slot (1..6).
 * @param loaded_mask   Loaded disc slots bitmask (bit 0 = disc 1, etc.).
 * @param mp3_mask      MP3 format bitmask (bit 0 = disc 1 is MP3).
 * @param track_num     Current track (1..999, Little-Endian).
 * @param minutes       Elapsed minutes (0..59).
 * @param seconds       Elapsed seconds (0..59).
 * @param mode_flags    Playback modes (repeat, random, scan).
 * @param status        Playback status enum (0x02 = Play, 0x01 = Pause, etc.).
 * @param total_tracks  Total tracks on current disc (1..999, Little-Endian).
 * @return              Total bytes written (16).
 */
int build_hiworld_cd_packet(
    uint8_t *out_buf,
    uint8_t disc_slot,
    uint8_t loaded_mask,
    uint8_t mp3_mask,
    uint16_t track_num,
    uint8_t minutes,
    uint8_t seconds,
    uint8_t mode_flags,
    hiworld_cd_status_t status,
    uint16_t total_tracks
) {
    uint8_t payload[11];

    payload[0]  = disc_slot & 0x0F;
    payload[1]  = loaded_mask & 0x3F;
    payload[2]  = mp3_mask & 0x3F;
    payload[3]  = (uint8_t)(track_num & 0xFF);        /* Little-endian Lo */
    payload[4]  = (uint8_t)((track_num >> 8) & 0xFF); /* Little-endian Hi */
    payload[5]  = minutes;
    payload[6]  = seconds;
    payload[7]  = mode_flags;
    payload[8]  = (uint8_t)status;
    payload[9]  = (uint8_t)(total_tracks & 0xFF);        /* Little-endian Lo */
    payload[10] = (uint8_t)((total_tracks >> 8) & 0xFF); /* Little-endian Hi */

    out_buf[0] = HIWORLD_SYNC_1;
    out_buf[1] = HIWORLD_SYNC_2;
    out_buf[2] = 0x0B;  /* Payload length */
    out_buf[3] = HIWORLD_CMD_CD_PLAY;
    memcpy(&out_buf[4], payload, 11);
    out_buf[15] = hiworld_calc_checksum(0x0B, HIWORLD_CMD_CD_PLAY, payload);

    return 16;
}
```

---

## 7. Concrete Wire Frame Examples & Hex Dumps

### Example 1: Single In-Dash CD, Playing Track 5 of 18 at 02:35 (Normal Playback)
* Disc Slot: `1` (`0x01`)
* Discs Loaded: Disc 1 Present (`0x01`)
* Format: Audio CD (`0x00`)
* Track: `5` &rarr; `0x05 0x00` (`uint16_le`)
* Elapsed: `2 min 35 sec` &rarr; `0x02 0x23`
* Mode: Normal (`0x00`)
* Status: Playing (`0x02`)
* Total Tracks: `18` &rarr; `0x12 0x00` (`uint16_le`)

**Raw Hex Wire Frame (16 bytes):**
```
5A A5 0B 97 01 01 00 05 00 02 23 00 02 12 00 E1
```
*Checksum check:* `(0x0B + 0x97 + 0x01 + 0x01 + 0x00 + 0x05 + 0x00 + 0x02 + 0x23 + 0x00 + 0x02 + 0x12 + 0x00) - 1 = 0x0E2 - 1 = 0xE1`.

---

### Example 2: 6-Disc CDC, All Slots Full, Disc 3 Playing Track 12 of 24 at 00:45, Repeat Track Active
* Disc Slot: `3` (`0x03`)
* Discs Loaded: Slots 1..6 Full (`0x3F`)
* Format: Audio CD (`0x00`)
* Track: `12` (`0x000C`) &rarr; `0x0C 0x00` (`uint16_le`)
* Elapsed: `0 min 45 sec` &rarr; `0x00 0x2D`
* Mode: Repeat Track (`0x20` / Bit 5)
* Status: Playing (`0x02`)
* Total Tracks: `24` (`0x0018`) &rarr; `0x18 0x00` (`uint16_le`)

**Raw Hex Wire Frame (16 bytes):**
```
5A A5 0B 97 03 3F 00 0C 00 00 2D 20 02 18 00 52
```

---

### Example 3: Single In-Dash CD Paused at 01:10 on Track 2
* Status: Paused (`0x01`)
* Track: `2` &rarr; `0x02 0x00`
* Elapsed: `1 min 10 sec` &rarr; `0x01 0x0A`

**Raw Hex Wire Frame (16 bytes):**
```
5A A5 0B 97 01 01 00 02 00 01 0A 00 01 12 00 C2
```

---

### Example 4: CD Changer Magazine Loading / Changing to Disc 4
* Disc Slot: `4` (`0x04`)
* Status: Loading / Changing Disc (`0x0D` &rarr; maps to `"Loading"`)

**Raw Hex Wire Frame (16 bytes):**
```
5A A5 0B 97 04 3F 00 01 00 00 00 00 0D 00 00 EA
```

---

### Example 5: In-Dash CD Ejected / No Disc Present
* Disc Slot: `1` (`0x01`)
* Discs Loaded: `0` (`0x00`)
* Status: Eject (`0x0C`)

**Raw Hex Wire Frame (16 bytes):**
```
5A A5 0B 97 01 00 00 00 00 00 00 00 0C 00 00 BE
```

---

### Example 6: Ground Truth Vehicle Log Vector (`dump_2026-10-10_11-12-22.log`)
* Context: RD4 Single In-Dash CD, Track 3 of 12, 03:12 Elapsed, Playing:
  - CAN `0x165` (dlc 4): `C8 C0 20 00` &rarr; Source `0x2` (Internal CD)
  - CAN `0x365` (dlc 5): `0C 26 02 00 00` &rarr; Total Tracks = 12 (`0x0C`)
  - CAN `0x3A5` (dlc 6): `03 03 19 03 0C 00` &rarr; Track 3, Playing, 03 min 12 sec
* Disc Slot: `0` (`0x00` - single drive)
* Discs Loaded: `0` (`0x00` - no changer magazine)
* Format: Audio CD (`0x00`)
* Track: `3` &rarr; `0x03 0x00` (`uint16_le`)
* Elapsed: `3 min 12 sec` &rarr; `0x03 0x0C`
* Modes: Intro/Repeat/Random OFF &rarr; `0x00`
* Status: Playing (`0x02`)
* Total Tracks: `12` &rarr; `0x0C 0x00` (`uint16_le`)

**Raw Hex Wire Frame (16 bytes):**
```
5A A5 0B 97 00 00 00 03 00 03 0C 00 02 0C 00 C1
```
*Checksum check:* `(0x0B + 0x97 + 0x00 + 0x00 + 0x00 + 0x03 + 0x00 + 0x03 + 0x0C + 0x00 + 0x02 + 0x0C + 0x00) - 1 = 0x0C2 - 1 = 0xC1`.

---

## 8. Timing, Rate Limiting & Transmission Policy

1. **Periodic Refresh:**
   * When CD audio is actively playing, transmit `Cmd 0x97` once per second (`1.0 Hz`) to increment the elapsed seconds display on the Head Unit.
2. **Immediate Event-Driven Dispatch:**
   * Transmit `Cmd 0x97` **immediately** upon detecting:
     * Track Skip (Next / Prev track button pressed).
     * Play/Pause toggle.
     * Disc selection change.
     * Disc eject or insertion.
3. **Audio Source Switching:**
   * When switching to CD mode:
     1. Transmit `Cmd 0x84` with `Data[0] = 0x30` (or `0x31`) and `station = "CD      "`.
     2. Wait 50 ms.
     3. Transmit `Cmd 0x97` with current playback state.

---

## 9. Testing & Validation Workflow

### 9.1 Using the Python Test Tool (`send_peugeot_radio.py`)

The workspace includes [`scenarios/send_peugeot_radio.py`](file:///home/Fazer/git/QF_Canbus_system/scenarios/send_peugeot_radio.py) configured to transmit CD frames:

```bash
# 1. Inspect raw hex bytes without hardware (Dry-run):
python3 scenarios/send_peugeot_radio.py \
  --dry-run \
  --cdc \
  --cdc-disc 1 \
  --cdc-track 5 \
  --cdc-min 2 \
  --cdc-sec 35 \
  --cdc-total 18 \
  --cdc-status 2

# 2. Transmit to physical Head Unit over USB-UART:
python3 scenarios/send_peugeot_radio.py \
  --port /dev/ttyUSB0 \
  --baud 38400 \
  --cdc \
  --cdc-disc 1 \
  --cdc-track 5 \
  --cdc-min 2 \
  --cdc-sec 35 \
  --cdc-total 18 \
  --cdc-status 2

# 3. Simulate CDC with 6 loaded discs and Track 12 playing:
python3 scenarios/send_peugeot_radio.py \
  --port /dev/ttyUSB0 \
  --cdc \
  --cdc-disc 3 \
  --cdc-discs-mask 0x3F \
  --cdc-track 12 \
  --cdc-min 0 \
  --cdc-sec 45 \
  --cdc-total 24 \
  --cdc-status 2

# 4. Transmit via ADB shell injection:
python3 scenarios/send_peugeot_radio.py \
  --adb \
  --adb-device-port /dev/ttyS1 \
  --cdc \
  --cdc-disc 1 \
  --cdc-track 5 \
  --cdc-min 2 \
  --cdc-sec 35 \
  --cdc-total 18 \
  --cdc-status 2
```

### 9.2 Android Logcat Verification
Monitor how `com.qf.vehicle` parses and routes the incoming frame:

```bash
adb logcat -v time -s PeugeotDataParser:D CarDataChannel:D OriginalMediaPlayer:D
```

Expected log output on valid packet reception:
```text
PeugeotDataParser: parseCarMediaState: disc=1, track=5/18, time=02:35, status=3
OriginalMediaPlayer: updateMedia: track=5/18, duration=02:35, workMode=Play
```

---

## 10. Summary Checklist for Firmware Developers

- [ ] **Baud Rate:** Set UART to `38400 baud`, 8 data bits, no parity, 1 stop bit (`8N1`).
- [ ] **Frame Opcode:** Use `0x97` with `LEN = 0x0B` (total wire length: 16 bytes).
- [ ] **Endianness Check:** Ensure Track Number (`Data[3..4]`) and Total Tracks (`Data[9..10]`) are **Little-Endian (`uint16_le`)**.
- [ ] **Play Status Value:** Send `0x02` for **Play**, `0x01` for **Pause**, `0x06` for **Stop**, `0x0D` for **Loading**.
- [ ] **Source Pairing:** Pair CD status with `Cmd 0x84` (`0x30` for internal CD, `0x31` for external CDC).
- [ ] **Single CD Drive Handling:** For internal CD drive, set `Data[0] = 0x01` and `Data[1] = 0x01` (or `0x00` when ejected).
- [ ] **Rate Limiting:** Transmit 1 Hz periodic during active playback, plus immediately on event changes.

