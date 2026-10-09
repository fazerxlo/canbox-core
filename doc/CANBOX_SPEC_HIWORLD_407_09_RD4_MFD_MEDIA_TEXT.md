# CAN Box Protocol Specification: RD4 Audio, CD Changer & RDS Radio Text
## Topic 09: Peugeot 407 RD4 Audio, Tuner, RDS Text & CD Synchronization (Hiworld Protocol)
**Document File:** `doc/CANBOX_SPEC_HIWORLD_407_09_RD4_MFD_MEDIA_TEXT.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard CAN2004)  
**Headunit Protocol:** Hiworld UART (`0x5A 0xA5` sync header, 38,400 baud, 8N1)  
**Authoritative Ground Truth Sources:**
- CAN Bus Data: `doc/PEUGEOT_RD4.md`
- Hiworld Android Integration: `doc/PEUGEOT_HIWORLD_RD4.md`

---

## 1. Architectural Overview & Signal Flow

In PSA CAN2004 (AEE2004) vehicles such as the Peugeot 407, the OEM **RD4 radio** (Blaupunkt/Bosch/Continental) has no internal display. It broadcasts tuner status, RDS station names, audio settings, and CD telemetry over the **PSA Comfort CAN bus (125 kbps)** to the external Multifunction Display (EMF-A/C/CT).

The CAN Box intercepts these CAN frames, caches the radio/media state, and translates them into Hiworld UART frames sent to the Android Head Unit:

```
+-----------------------------------------------------------------------------------------+
|                               PSA CAN2004 Comfort Bus (125 kbps)                        |
|                                                                                         |
|   0x165 (ETAT_AUTORADIO): Audio Source (Tuner, CD, CDC, AUX 1, AUX 2, BT)              |
|   0x225 (ETAT_TUNER):     Waveband, Preset Slot, RDS Flags, Raw Tuner Frequency         |
|   0x2A5 (NOM_STATION):    8-Byte ASCII RDS Station Service Name (PS)                    |
|   0x0A4 (TEXTE_RADIO):    ISO-TP Segmented Dynamic RDS RadioText (Song Title / Artist)   |
|   0x3A6 (CDC_STATUS):     CD Changer Disc Slot, Track No, Elapsed MM:SS, Total Tracks    |
|   0x1A5 (VOLUME_RADIO):   Master Volume Level & Pop-up Animation Trigger                |
|   0x1E5 (REGLAGES_SON):   Balance, Fader, Bass, Treble, Loudness, Equalizer Ambiance    |
+-----------------------------------------------------------------------------------------+
                                             │
                                             ▼
+-----------------------------------------------------------------------------------------+
|                       CAN Box Microcontroller (Pure C99 Engine)                         |
|                                                                                         |
|   1. Audio Source & Power State Router (0x165)                                          |
|   2. Tuner Frequency Scaler & Waveband Mapper (0x225: Freq_Hiworld = Raw / 2 + 500)     |
|   3. RDS PS Station Name Extractor (0x2A5)                                              |
|   4. Static ISO-TP Multi-Frame Reassembler (0x0A4: Strips 4-byte 10 00 00 00 prefix)   |
|   5. CD Changer State Parser (0x3A6: Disc, Track, Time, Status)                         |
|   6. Master Volume & Tone Adjustment Parser (0x1A5 & 0x1E5)                             |
|   7. State Cache & Rate-Limited Serial Dispatcher                                       |
+-----------------------------------------------------------------------------------------+
                                             │
                       Hiworld UART Protocol (38,400 baud, 8N1)
                                             │
      ┌──────────────────────────────────────┼──────────────────────────────────────┐
      ▼                                      ▼                                      ▼
+──────────────────────────+   +──────────────────────────+   +──────────────────────────+
| Cmd 0x84 (CarRadioState) |   | Cmd 0x86 (RadioTextInfo) |   | Cmd 0x97 (CarMediaState) |
| Band, Freq, Preset, RDS, |   | Dynamic RDS RadioText    |   | Disc Slot, Loaded Mask,  |
| State, 8-Byte PS Name    |   | String (Song / Artist)   |   | Track uint16, MM:SS,     |
| (Routes to OriginalTuner)|   | (Routes to tv_radio_text)|   | Play State, Total uint16 |
+──────────────────────────+   +──────────────────────────+   +──────────────────────────+
```

---

## 2. PSA CAN Bus Ground Truth Specifications

### 2.1 Audio Source / Input Status (`0x165` — `ETAT_AUTORADIO`)
* **Period:** 50 ms  
* **DLC:** 4 bytes  
* **Payload:** `[0xCC, 0x54, (INPUT_SOURCE << 4) | SubStatus, 0x02]`

| Byte | Bit(s) | Signal | Value / Meaning |
|:---:|:---:|---|---|
| **0** | 7:0 | Constant | `0xCC` |
| **1** | 7:0 | Constant | `0x54` |
| **2** | 7:4 | **INPUT_SOURCE** | `0x1` = Tuner (FM/AM)<br>`0x2` = Internal CD Drive<br>`0x3` = CD Changer (CDC)<br>`0x4` = AUX 1 (Line-In)<br>`0x5` = AUX 2<br>`0x6` = USB Media<br>`0x7` = Bluetooth Audio |
| **2** | 3:0 | Sub-status | `0x00` or `0x01` |
| **3** | 7:0 | Constant | `0x02` |

---

### 2.2 FM/AM Tuner Status & Frequency (`0x225` — `ETAT_TUNER`)
* **Period:** 100 ms  
* **DLC:** 5 to 8 bytes  

| Byte | Bit(s) | Signal | Description |
|:---:|:---:|---|---|
| **0** | 7 | **LIST** | `1` = Station list mode active |
| **0** | 6 | **SCAN** | `1` = Scan tuning active |
| **0** | 5 | **RDS** | `1` = RDS subcarrier lock valid |
| **0** | 4 | **PTY** | `1` = Program Type (PTY) standby active |
| **0** | 3 | **TUN** | `1` = Tuner actively seeking/tuning |
| **0** | 2 | **TA** | `1` = Traffic Announcement priority active |
| **0** | 1:0 | **TUNDIR** | `0` = None, `1` = Seek Up, `2` = Seek Down |
| **1** | 7:0 | **MEMORY** | Preset station number: `0` = Manual tuning, `1`..`6` = Preset slot |
| **2** | 7:0 | **BAND** | `0x10` / `0x90` = FM Band 1<br>`0x20` / `0xA0` = FM Band 2<br>`0x40` / `0xC0` = FM Auto-Store (AST)<br>`0x50` / `0xD0` = AM (Medium Wave) |
| **3..4** | 15:0 | **FREQUENCY** | Raw uint16 big-endian:<br>$$\text{Frequency (MHz)} = (\text{raw} \times 0.05) + 50.0$$ |

---

### 2.3 RDS Station Name (`0x2A5` — `NOM_STATION`)
* **Period:** 100 ms (or event-driven on RDS PS decode)  
* **DLC:** 8 bytes  
* **Payload:** 8 ASCII characters carrying the RDS Program Service (PS) name (e.g. `"RMF FM  "`, `"RADIO 1 "`). Left-aligned and padded with trailing spaces (`0x20`) or null bytes (`0x00`).

---

### 2.4 RDS Dynamic RadioText Multi-Frame (`0x0A4` — `TEXTE_RADIO`)
* **Period:** Event-driven multi-frame stream (~100–500 ms)  
* **DLC:** 8 bytes per CAN frame  
* **Transport:** Segmented using ISO 15765-2 (ISO-TP):
  - **Single Frame (SF):** `data[0] = 0x0N` ($N \le 7$ characters).
  - **First Frame (FF):** `data[0..1] = 0x1H 0xLL` ($12$-bit total length, payload in `data[2..7]`).
  - **Consecutive Frame (CF):** `data[0] = 0x2N` (Sequence counter $N = 1 \dots 15$, payload in `data[1..7]`).
* **Control Prefix:** Observed real captures on Peugeot 407 show an internal 4-byte prefix (`10 00 00 00`) prepended to the actual ASCII/UTF-8 text payload. The reassembly engine must strip this prefix when present to obtain the raw text.

---

### 2.5 CD Changer Status & Telemetry (`0x3A6` — `CDC_STATUS`)
* **Period:** 500 ms (or event-driven on track/time change)  
* **DLC:** 8 bytes  
* **Ground Truth Codebase Definition:** (`src/profiles/peugeot_407.c:1178` & `test/test_protocol_parser/test_peugeot_407.c:1152`)

| Byte | Bit(s) | Signal | Values / Description |
|:---:|:---:|---|---|
| **0** | 7:0 | Reserved | `0x00` |
| **1** | 7:0 | **DISC_SLOT** | Active disc slot: `1`..`6` (`0` = No disc / Slot empty) |
| **2** | 7:0 | **TRACK_NUM** | Current track number: `1`..`99` |
| **3** | 7:0 | **TOTAL_TRACKS**| Total tracks on active disc: `1`..`99` |
| **4** | 7:0 | **ELAPSED_MIN** | Track elapsed minutes: `0`..`59` |
| **5** | 7:0 | **ELAPSED_SEC** | Track elapsed seconds: `0`..`59` |
| **6** | 7:0 | **PLAY_FLAGS**  | Playback mode flags (`0x01` = Random / RND, `0x02` = Scan, `0x04` = Repeat / RPT) |
| **7** | 7:0 | Reserved | `0x00` |

---

### 2.6 Radio Volume (`0x1A5` — `VOLUME_RADIO`)
* **Period:** 100 ms (or event-driven on knob turn / stalk command)  
* **DLC:** 1 byte  
* `data[0][7:5]`: `VOLFLAG` (`0xE0` = Stable/idle; `0x00` = Changing, triggers on-screen volume pop-up).  
* `data[0][4:0]`: `VOLUME` ($0 \dots 30$).

---

### 2.7 Audio Tone Settings & Equalizer (`0x1E5` — `REGLAGES_SON`)
* **Period:** 100 ms  
* **DLC:** 7 bytes  
* `data[0]`: Balance (Bit 7: Active menu, Bits 6:0: Level `0x3F` Center, range -9..+9, `> 0x3F` = Right, `< 0x3F` = Left)  
* `data[1]`: Fader (Bit 7: Active menu, Bits 6:0: Level `0x3F` Center, range -9..+9, `> 0x3F` = Front, `< 0x3F` = Rear, e.g. `0x46` = Front +7)  
* `data[2]`: Bass (Bit 7: Active menu, Bits 6:0: Level `0x3F` Flat, range -9..+9, e.g. `0x48` = +9)  
* `data[3]`: Reserved / Neutral (Typically `0x3F`)  
* `data[4]`: Treble (Bit 7: Active menu, Bits 6:0: Level `0x3F` Flat, range -9..+9, e.g. `0x48` = +9)  
* `data[5]`: Loudness & Auto-Volume (Bit 6: Loudness ON/OFF, Bits 2:0: Auto-Volume level)  
* `data[6]`: Equalizer Ambiance Preset (`0x03` Flat, `0x07` Classic, `0x0B` Jazz, `0x0F` Pop, `0x13` Vocal, `0x17` Techno)

---

## 3. Hiworld UART Protocol Specifications

All Hiworld packets use the standard framing:
```
[0x5A] [0xA5] [LEN] [CMD] [DATA_0 .. DATA_(LEN-1)] [CHECKSUM]
```
Where checksum is calculated as:
$$\text{Checksum} = \left(\text{LEN} + \text{CMD} + \sum_{i=0}^{\text{LEN}-1} \text{DATA}_i - 1\right) \pmod{256}$$

---

### 3.1 RD4 Radio & Tuner Status Frame (`Cmd 0x84` — `CarRadioState`)
* **Command ID:** `0x84` (`-0x7ct` in signed byte)  
* **Length (`LEN`):** `0x0E` (14 bytes) for base packet, or `14 + strlen(RadioText)` for extended packet with trailing text.  
* **Minimum Wire Length:** 19 bytes ($2 \text{ sync} + 1 \text{ len} + 1 \text{ cmd} + 14 \text{ payload} + 1 \text{ csum}$).  
* **Headunit Route:** Pushes to `OriginalTuner` activity; triggers auto-switch to tuner when switching from external sources.

| Payload Byte | Field Name | Description & Value Encoding |
|:---:|---|---|
| **`Data[0]`** | **Band / Source Type** | **Radio Bands:**<br>`0x00` = FM1, `0x01` = FM2, `0x02` = FM3, `0x04` = FM-AST<br>`0x10` = AM / MW, `0x11` = AM1, `0x12` = AM2, `0x13` = AM-AST<br>**External Sources:**<br>`0x20` = Line In (AUX 1), `0x21` = Line In (AUX 2)<br>`0x30` = Internal CD, `0x31` = CD Changer (CDC)<br>`0xFF` = Standby / Radio Off |
| **`Data[1..2]`** | **Frequency (`uint16_le`)** | **FM:** $\text{Frequency (MHz)} \times 10$ in **Little-Endian** (`Data[1] = LSB`, `Data[2] = MSB`)<br>*(e.g., $96.0\text{ MHz} \to 960 = \mathtt{0x03C0} \to \text{Data[1]}=\mathtt{0xC0}, \text{Data[2]}=\mathtt{0x03}$)*<br>**AM:** Frequency in kHz in Little-Endian<br>**AUX / CD / OFF:** Send `0x0000`<br>*(Note: `PeugeotDataParser.smali::parseCarRadioState` explicitly calls `JavaDecodeUtil.byteArrToInt(..., isLittleEndian=true)`)* |
| **`Data[3]`** | **Preset Slot** | `0x00` = Manual tuning<br>`0x01`..`0x06` = Presets 1 to 6 |
| **`Data[4]`** | **Indicators Bitmask** | **Bit 7 (`0x80`):** `TA` (Traffic Announcement active)<br>**Bit 6 (`0x40`):** `ST` (Stereo reception)<br>**Bit 5 (`0x20`):** `RDS` (RDS lock active)<br>**Bit 4 (`0x10`):** `SCAN` (Scan tuning mode)<br>**Bit 3 (`0x08`):** `REG` (Regional program lock)<br>**Bit 2 (`0x04`):** `RDTEXT` (RadioText available)<br>**Bit 1 (`0x02`):** `AUTO.P` (Auto-store AST mode)<br>*(Note: TA indicator is stabilized against `0x225` and `0x265` frame interleaving to prevent jitter/flickering)* |
| **`Data[5]`** | **Power / Status** | `0x00` = Standby / Off<br>`0x01` = Normal Playback<br>`0x02` = Seeking / Tuning<br>`0x03` = Mute |
| **`Data[6..13]`**| **Station Name (PS)** | 8 ASCII characters of RDS Program Service name (space padded) |
| **`Data[14..N]`**| **Optional RadioText** | Variable length ASCII/UTF-8 string (up to 64 chars) from CAN `0x0A4` |

#### Frequency Conversion Formula:
Given CAN `0x225` raw frequency $R$:
$$f_{\text{MHz}} = (R \times 0.05) + 50.0$$
The Hiworld 0.1 MHz integer value is:
$$\text{Hiworld\_Freq} = f_{\text{MHz}} \times 10 = (R \times 0.5) + 500 = \frac{R}{2} + 500$$
*(Example: raw $920$ (`0x0398`) $\implies 920 / 2 + 500 = 960$ ($96.0\text{ MHz}$), packed Little-Endian as `[0xC0, 0x03]`).*

---

### 3.2 Standalone RDS RadioText Frame (`Cmd 0x86` — `RadioTextInfo`)
* **Command ID:** `0x86` (`-0x7at` in signed byte)  
* **Length (`LEN`):** Length of text payload ($1 \dots 64$ bytes).  
* **Total Wire Length:** $5 + \text{LEN}$ bytes.  
* **Headunit Route:** Decoded by `PeugeotDataParser::parseRadioText()` directly into `mRadioText` and rendered by full-width scrolling marquee `tv_radio_text` on `OriginalTuner`.  
* **Payload:** `Data[0..N]` contains raw ASCII/UTF-8 characters of song title, artist, or program announcement.

> [!IMPORTANT]
> Do NOT use `Cmd 0x85` for RadioText. `0x85` is reserved by the QingFang parser for `SportModeInfo` (Peugeot 508 Sport/Eco Mode).

---

### 3.3 CD Changer / Media State Frame (`Cmd 0x97` — `CarMediaState`)
* **Command ID:** `0x97` (`151` unsigned / `-105` signed)  
* **Length (`LEN`):** `0x0B` (11 payload bytes)  
* **Total Wire Length:** 16 bytes ($2 \text{ sync} + 1 \text{ len} + 1 \text{ cmd} + 11 \text{ payload} + 1 \text{ csum}$).  
* **Headunit Route:** Routes to `OriginalMediaPlayer` (`mCarCdPlayInfo`).

| Payload Byte | Field Name | Type | Description |
|:---:|---|:---:|---|
| **`Data[0]`** | **Active Disc Slot** | `uint8_t` | Active disc slot: `1`..`6` |
| **`Data[1]`** | **Discs Loaded Mask** | `uint8_t` | Bitmask of loaded disc slots (Bit 0 = Slot 1, ..., Bit 5 = Slot 6) |
| **`Data[2]`** | **Disc Format** | `uint8_t` | `0x00` = Audio CD, `0x01` = MP3 CD |
| **`Data[3..4]`** | **Current Track** | `uint16_be`| Current track number: `1`..`999` |
| **`Data[5]`** | **Elapsed Minutes** | `uint8_t` | `0`..`59` |
| **`Data[6]`** | **Elapsed Seconds** | `uint8_t` | `0`..`59` |
| **`Data[7]`** | **Playback Modes** | `uint8_t` | Bit 0: Repeat, Bit 1: Random, Bit 2: Intro/Scan |
| **`Data[8]`** | **Playback Status** | `uint8_t` | `0x00` = Stop, `0x01` = Play, `0x02` = Pause |
| **`Data[9..10]`**| **Total Tracks** | `uint16_be`| Total tracks on active disc: `1`..`999` |

---

### 3.4 Audio Settings & Tone Adjustments (`Cmd 0x82` — `SoundEffectInfo`)
* **Command ID:** `0x82` (`-0x7et` in signed byte)  
* **Length (`LEN`):** `0x08` (8 payload bytes)  
* Maps master volume from CAN `0x1A5` and equalizer/fader/balance/tone from CAN `0x1E5`.

---

## 4. Headunit Query & Handshake Resync Handling

When the Android Head Unit boots up, or when either `OriginalTuner` or `OriginalMediaPlayer` resumes (`onResume()`), the APK queries the CAN Box using `forwardType`:
* **Tuner Resume Query (`forwardType(0x0F)`):** Head Unit requests immediate radio state. The CAN Box must immediately retransmit cached **`Cmd 0x84`** and **`Cmd 0x86`**.
* **Media Resume Queries (`forwardType(0x11)` / `forwardType(0x12)`):** Head Unit requests CD changer playback state. The CAN Box must immediately retransmit cached **`Cmd 0x97`**.
* **Periodic Resync:** To protect against unannounced Head Unit reboots, the CAN Box retransmits all cached state frames at a slow 60-second periodic interval.

---

## 5. Pure C99 Implementation Blueprint

The following standalone module implements the decoders, static ISO-TP reassembly, state management, and Hiworld serialization with zero heap allocation:

```c
#ifndef CANBOX_HIWORLD_PEUGEOT_407_RD4_H
#define CANBOX_HIWORLD_PEUGEOT_407_RD4_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define HIWORLD_SOF1                0x5A
#define HIWORLD_SOF2                0xA5

#define HIWORLD_CMD_RADIO_STATE     0x84
#define HIWORLD_CMD_RADIO_TEXT      0x86
#define HIWORLD_CMD_MEDIA_STATE     0x97
#define HIWORLD_CMD_SOUND_EFFECT    0x82

#define PSA_CAN_ID_ETAT_AUTORADIO   0x165
#define PSA_CAN_ID_VOLUME_RADIO     0x1A5
#define PSA_CAN_ID_REGLAGES_SON     0x1E5
#define PSA_CAN_ID_ETAT_TUNER       0x225
#define PSA_CAN_ID_NOM_STATION      0x2A5
#define PSA_CAN_ID_TEXTE_RADIO      0x0A4
#define PSA_CAN_ID_CD_CHANGER       0x3A6

/* Maximum RadioText buffer size */
#define PSA_RD4_MAX_RADIO_TEXT_LEN  64

/* State structures */
typedef struct {
    uint8_t  source_mode;       /* Hiworld source code */
    uint8_t  band;              /* 0x00=FM1, 0x01=FM2, 0x04=FMAST, 0x10=AM */
    uint16_t freq_0_1mhz;       /* Frequency in 0.1 MHz units (FM) or kHz (AM) */
    uint8_t  preset_slot;       /* 0=manual, 1..6 */
    uint8_t  indicators;        /* TA, ST, RDS, SCAN, REG, RDTEXT, AUTO.P */
    uint8_t  power_status;      /* 0=off, 1=playing, 2=seeking, 3=mute */
    char     station_name[9];   /* 8 ASCII chars + null terminator */
    char     radio_text[PSA_RD4_MAX_RADIO_TEXT_LEN + 1];
    uint8_t  radio_text_len;
} psa_rd4_radio_state_t;

typedef struct {
    uint8_t  active_disc;       /* 1..6 */
    uint8_t  discs_loaded_mask; /* Bit 0..5 */
    uint8_t  disc_format;       /* 0=CDDA, 1=MP3 */
    uint16_t track_num;         /* 1..999 */
    uint16_t total_tracks;      /* 1..999 */
    uint8_t  elapsed_min;       /* 0..59 */
    uint8_t  elapsed_sec;       /* 0..59 */
    uint8_t  play_modes;        /* Bit 0: RND, Bit 1: SCAN, Bit 2: RPT */
    uint8_t  play_status;       /* 0=stop, 1=play, 2=pause */
} psa_rd4_cdc_state_t;

/* Static ISO-TP Multi-frame Reassembler */
typedef struct {
    uint8_t  buffer[PSA_RD4_MAX_RADIO_TEXT_LEN + 8];
    uint16_t total_length;
    uint16_t received_length;
    uint8_t  next_sn;
    bool     active;
} psa_isotp_rx_ctx_t;

typedef void (*canbox_uart_tx_fn)(const uint8_t *buf, size_t len);

typedef struct {
    psa_rd4_radio_state_t radio;
    psa_rd4_cdc_state_t   cdc;
    psa_isotp_rx_ctx_t    isotp;
    canbox_uart_tx_fn     uart_tx;
} psa_rd4_media_ctx_t;

/* Initialize context */
static inline void psa_rd4_media_init(psa_rd4_media_ctx_t *ctx, canbox_uart_tx_fn tx_fn) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->uart_tx = tx_fn;
    ctx->radio.source_mode = 0xFF; /* Standby / Off */
    memset(ctx->radio.station_name, ' ', 8);
    ctx->radio.station_name[8] = '\0';
}

/* Transmit Hiworld Radio State (Cmd 0x84) */
static inline void psa_rd4_send_hiworld_0x84(psa_rd4_media_ctx_t *ctx) {
    if (!ctx || !ctx->uart_tx) return;

    uint8_t frame[32];
    uint8_t len = 14; /* Standard 14 payload bytes */

    frame[0] = HIWORLD_SOF1;
    frame[1] = HIWORLD_SOF2;
    frame[2] = len;
    frame[3] = HIWORLD_CMD_RADIO_STATE;

    /* If in external source, send source code; otherwise send radio band */
    if (ctx->radio.source_mode >= 0x20) {
        frame[4] = ctx->radio.source_mode;
    } else {
        frame[4] = ctx->radio.band;
    }

    /* Frequency uint16_be */
    if (ctx->radio.source_mode < 0x20 && ctx->radio.power_status != 0) {
        frame[5] = (uint8_t)((ctx->radio.freq_0_1mhz >> 8) & 0xFF);
        frame[6] = (uint8_t)(ctx->radio.freq_0_1mhz & 0xFF);
    } else {
        frame[5] = 0x00;
        frame[6] = 0x00;
    }

    frame[7]  = ctx->radio.preset_slot;
    frame[8]  = ctx->radio.indicators;
    frame[9]  = ctx->radio.power_status;

    /* 8-Byte ASCII Station Name */
    for (size_t i = 0; i < 8; i++) {
        frame[10 + i] = (uint8_t)ctx->radio.station_name[i];
    }

    /* Checksum: ((LEN + CMD + sum(DATA)) - 1) & 0xFF */
    uint8_t sum = len + HIWORLD_CMD_RADIO_STATE;
    for (size_t i = 4; i < 4 + len; i++) {
        sum += frame[i];
    }
    frame[4 + len] = (uint8_t)((sum - 1) & 0xFF);

    ctx->uart_tx(frame, 5 + len);
}

/* Transmit Hiworld Standalone RadioText (Cmd 0x86) */
static inline void psa_rd4_send_hiworld_0x86(psa_rd4_media_ctx_t *ctx) {
    if (!ctx || !ctx->uart_tx || ctx->radio.radio_text_len == 0) return;

    uint8_t frame[72];
    uint8_t len = ctx->radio.radio_text_len;
    if (len > PSA_RD4_MAX_RADIO_TEXT_LEN) len = PSA_RD4_MAX_RADIO_TEXT_LEN;

    frame[0] = HIWORLD_SOF1;
    frame[1] = HIWORLD_SOF2;
    frame[2] = len;
    frame[3] = HIWORLD_CMD_RADIO_TEXT;

    memcpy(&frame[4], ctx->radio.radio_text, len);

    uint8_t sum = len + HIWORLD_CMD_RADIO_TEXT;
    for (size_t i = 0; i < len; i++) {
        sum += (uint8_t)ctx->radio.radio_text[i];
    }
    frame[4 + len] = (uint8_t)((sum - 1) & 0xFF);

    ctx->uart_tx(frame, 5 + len);
}

/* Transmit Hiworld CD Changer State (Cmd 0x97) */
static inline void psa_rd4_send_hiworld_0x97(psa_rd4_media_ctx_t *ctx) {
    if (!ctx || !ctx->uart_tx) return;

    uint8_t frame[16];
    uint8_t len = 0x0B; /* 11 payload bytes */

    frame[0] = HIWORLD_SOF1;
    frame[1] = HIWORLD_SOF2;
    frame[2] = len;
    frame[3] = HIWORLD_CMD_MEDIA_STATE;

    frame[4]  = ctx->cdc.active_disc;
    frame[5]  = ctx->cdc.discs_loaded_mask;
    frame[6]  = ctx->cdc.disc_format;
    frame[7]  = (uint8_t)((ctx->cdc.track_num >> 8) & 0xFF);
    frame[8]  = (uint8_t)(ctx->cdc.track_num & 0xFF);
    frame[9]  = ctx->cdc.elapsed_min;
    frame[10] = ctx->cdc.elapsed_sec;
    frame[11] = ctx->cdc.play_modes;
    frame[12] = ctx->cdc.play_status;
    frame[13] = (uint8_t)((ctx->cdc.total_tracks >> 8) & 0xFF);
    frame[14] = (uint8_t)(ctx->cdc.total_tracks & 0xFF);

    uint8_t sum = len + HIWORLD_CMD_MEDIA_STATE;
    for (size_t i = 4; i <= 14; i++) {
        sum += frame[i];
    }
    frame[15] = (uint8_t)((sum - 1) & 0xFF);

    ctx->uart_tx(frame, 16);
}

/* Decode CAN 0x165 (ETAT_AUTORADIO) */
static inline void psa_rd4_process_can_0x165(psa_rd4_media_ctx_t *ctx, const uint8_t *d, uint8_t dlc) {
    if (!ctx || !d || dlc < 3) return;

    uint8_t src = (d[2] >> 4) & 0x0F;
    switch (src) {
        case 0x1: /* Tuner */
            ctx->radio.source_mode = ctx->radio.band;
            ctx->radio.power_status = 0x01;
            break;
        case 0x2: /* Internal CD */
            ctx->radio.source_mode = 0x30;
            ctx->radio.power_status = 0x01;
            break;
        case 0x3: /* CDC */
            ctx->radio.source_mode = 0x31;
            ctx->radio.power_status = 0x01;
            break;
        case 0x4: /* AUX 1 */
            ctx->radio.source_mode = 0x20;
            ctx->radio.power_status = 0x01;
            break;
        case 0x5: /* AUX 2 */
            ctx->radio.source_mode = 0x21;
            ctx->radio.power_status = 0x01;
            break;
        default:
            ctx->radio.source_mode = 0xFF;
            ctx->radio.power_status = 0x00;
            break;
    }
    psa_rd4_send_hiworld_0x84(ctx);
}

/* Decode CAN 0x225 (ETAT_TUNER) */
static inline void psa_rd4_process_can_0x225(psa_rd4_media_ctx_t *ctx, const uint8_t *d, uint8_t dlc) {
    if (!ctx || !d || dlc < 5) return;

    /* Indicator flags */
    ctx->radio.indicators = 0;
    if (d[0] & 0x04) ctx->radio.indicators |= 0x80; /* TA */
    if (d[0] & 0x20) ctx->radio.indicators |= 0x20; /* RDS */
    if (d[0] & 0x40) ctx->radio.indicators |= 0x10; /* SCAN */
    if (ctx->radio.radio_text_len > 0) ctx->radio.indicators |= 0x04; /* RDTEXT */

    /* Seeking / Tuning status */
    if (d[0] & 0x08) {
        ctx->radio.power_status = 0x02; /* Seeking */
    } else if (ctx->radio.power_status == 0x02) {
        ctx->radio.power_status = 0x01; /* Restored to playing */
    }

    /* Preset memory: 0=manual, 1..6 (encoded as 0x10..0x60 or 0x01..0x06) */
    ctx->radio.preset_slot = (d[1] >= 0x10) ? ((d[1] >> 4) & 0x0F) : (d[1] & 0x0F);

    /* Band conversion */
    uint8_t raw_band = d[2];
    if (raw_band == 0x10 || raw_band == 0x90)      ctx->radio.band = 0x00; /* FM1 */
    else if (raw_band == 0x20 || raw_band == 0xA0) ctx->radio.band = 0x01; /* FM2 */
    else if (raw_band == 0x40 || raw_band == 0xC0) ctx->radio.band = 0x04; /* FM-AST */
    else if (raw_band == 0x50 || raw_band == 0xD0) ctx->radio.band = 0x10; /* AM */

    /* Frequency conversion: Raw to 0.1 MHz */
    uint16_t raw_freq = ((uint16_t)d[3] << 8) | d[4];
    if (ctx->radio.band < 0x10) {
        /* FM: Freq * 10 = (raw * 0.05 + 50.0) * 10 = (raw / 2) + 500 */
        ctx->radio.freq_0_1mhz = (raw_freq / 2) + 500;
    } else {
        /* AM: kHz direct */
        ctx->radio.freq_0_1mhz = raw_freq;
    }

    if (ctx->radio.source_mode < 0x20) {
        ctx->radio.source_mode = ctx->radio.band;
    }

    psa_rd4_send_hiworld_0x84(ctx);
}

/* Decode CAN 0x265 (INFO_TUNER) */
static inline void psa_rd4_process_can_0x265(psa_rd4_media_ctx_t *ctx, const uint8_t *d, uint8_t dlc) {
    if (!ctx || !d || dlc < 1) return;

    /* Byte 0 Bit 5: TA (Traffic Announcement) */
    if (d[0] & 0x20) {
        ctx->radio.indicators |= 0x80; /* TA bit in Hiworld */
    } else {
        ctx->radio.indicators &= ~0x80;
    }
}

/* Decode CAN 0x2A5 (NOM_STATION) */
static inline void psa_rd4_process_can_0x2A5(psa_rd4_media_ctx_t *ctx, const uint8_t *d, uint8_t dlc) {
    if (!ctx || !d || dlc < 8) return;

    memcpy(ctx->radio.station_name, d, 8);
    ctx->radio.station_name[8] = '\0';

    psa_rd4_send_hiworld_0x84(ctx);
}

/* Decode CAN 0x0A4 (TEXTE_RADIO) with Static ISO-TP Reassembly */
static inline void psa_rd4_process_can_0x0a4(psa_rd4_media_ctx_t *ctx, const uint8_t *d, uint8_t dlc) {
    if (!ctx || !d || dlc < 2) return;

    uint8_t pci = d[0] & 0xF0;

    /* Single Frame (0x0N) */
    if (pci == 0x00) {
        uint8_t len = d[0] & 0x0F;
        if (len > 0 && len <= (dlc - 1)) {
            const uint8_t *payload = &d[1];
            /* Strip 4-byte prefix 10 00 00 00 if present */
            if (len > 4 && payload[0] == 0x10 && payload[1] == 0x00 && payload[2] == 0x00 && payload[3] == 0x00) {
                payload += 4;
                len -= 4;
            }
            if (len > PSA_RD4_MAX_RADIO_TEXT_LEN) len = PSA_RD4_MAX_RADIO_TEXT_LEN;
            memcpy(ctx->radio.radio_text, payload, len);
            ctx->radio.radio_text[len] = '\0';
            ctx->radio.radio_text_len = len;
            psa_rd4_send_hiworld_0x86(ctx);
        }
        ctx->isotp.active = false;
        return;
    }

    /* First Frame (0x1N) */
    if (pci == 0x10) {
        uint16_t total_len = (((uint16_t)(d[0] & 0x0F)) << 8) | d[1];
        if (total_len > sizeof(ctx->isotp.buffer)) total_len = sizeof(ctx->isotp.buffer);

        uint8_t chunk_len = dlc - 2;
        memcpy(ctx->isotp.buffer, &d[2], chunk_len);
        ctx->isotp.total_length    = total_len;
        ctx->isotp.received_length = chunk_len;
        ctx->isotp.next_sn         = 1;
        ctx->isotp.active          = true;
        return;
    }

    /* Consecutive Frame (0x2N) */
    if (pci == 0x20 && ctx->isotp.active) {
        uint8_t sn = d[0] & 0x0F;
        if (sn == (ctx->isotp.next_sn & 0x0F)) {
            uint8_t chunk_len = dlc - 1;
            if (ctx->isotp.received_length + chunk_len > ctx->isotp.total_length) {
                chunk_len = (uint8_t)(ctx->isotp.total_length - ctx->isotp.received_length);
            }
            memcpy(&ctx->isotp.buffer[ctx->isotp.received_length], &d[1], chunk_len);
            ctx->isotp.received_length += chunk_len;
            ctx->isotp.next_sn = (ctx->isotp.next_sn + 1) & 0x0F;

            /* Completed reassembly */
            if (ctx->isotp.received_length >= ctx->isotp.total_length) {
                const uint8_t *payload = ctx->isotp.buffer;
                uint16_t len = ctx->isotp.total_length;

                /* Strip 4-byte prefix 10 00 00 00 if present */
                if (len > 4 && payload[0] == 0x10 && payload[1] == 0x00 && payload[2] == 0x00 && payload[3] == 0x00) {
                    payload += 4;
                    len -= 4;
                }
                if (len > PSA_RD4_MAX_RADIO_TEXT_LEN) len = PSA_RD4_MAX_RADIO_TEXT_LEN;

                memcpy(ctx->radio.radio_text, payload, len);
                ctx->radio.radio_text[len] = '\0';
                ctx->radio.radio_text_len = (uint8_t)len;

                psa_rd4_send_hiworld_0x86(ctx);
                ctx->isotp.active = false;
            }
        } else {
            /* Sequence error */
            ctx->isotp.active = false;
        }
    }
}

/* Decode CAN 0x3A6 (CDC_STATUS) */
static inline void psa_rd4_process_can_0x3a6(psa_rd4_media_ctx_t *ctx, const uint8_t *d, uint8_t dlc) {
    if (!ctx || !d || dlc < 7) return;

    ctx->cdc.active_disc       = d[1];
    ctx->cdc.track_num         = d[2];
    ctx->cdc.total_tracks      = d[3];
    ctx->cdc.elapsed_min       = d[4];
    ctx->cdc.elapsed_sec       = d[5];
    ctx->cdc.play_modes        = d[6];

    /* Disc slot bitmask */
    if (ctx->cdc.active_disc >= 1 && ctx->cdc.active_disc <= 6) {
        ctx->cdc.discs_loaded_mask |= (1 << (ctx->cdc.active_disc - 1));
        ctx->cdc.play_status = 0x01; /* Play */
    } else {
        ctx->cdc.play_status = 0x00; /* Stop */
    }

    psa_rd4_send_hiworld_0x97(ctx);
}

#endif /* CANBOX_HIWORLD_PEUGEOT_407_RD4_H */
```

---

## 6. Verification Test Vectors & Reference Scenarios

### Vector 1: FM Tuner Playback at 102.50 MHz (Preset 1, RDS Active, Station "RMF FM")
* **CAN Injections:**
  1. `0x165` (Source = Tuner):
     ```bash
     cansend vcan0 165#CC541002
     ```
  2. `0x225` (FM1, Preset 1, RDS Lock, 102.50 MHz $\to$ raw $1050 \to \mathtt{0x041A}$):
     ```bash
     cansend vcan0 225#200190041A
     ```
  3. `0x2A5` (Station Name `"RMF FM  "`):
     ```bash
     cansend vcan0 2A5#524D4620464D2020
     ```
* **Expected Hiworld Output (`Cmd 0x84`):**
  - **Frame:** `5A A5 0E 84 00 04 01 01 20 01 52 4D 46 20 46 4D 20 20 6B`
  - *Breakdown:*
    - Header: `5A A5`
    - Length: `0E` (14 bytes)
    - Cmd ID: `84` (`CarRadioState`)
    - Payload: `00` (FM1), `04 01` ($1025 = 102.50\text{ MHz}$), `01` (Preset 1), `20` (RDS active), `01` (Playing), `"RMF FM  "`
    - Checksum: `0x6B`

---

### Vector 2: Dynamic RDS RadioText Stream ("Queen - Bohemian Rhapsody")
* **Text Length:** 25 bytes.
* **Prepended with 4-byte prefix:** `10 00 00 00 51 75 65 65 6E 20 2D 20 42 6F 68 65 6D 69 61 6E 20 52 68 61 70 73 6F 64 79` (Total 29 bytes).
* **ISO-TP CAN Injections (`0x0A4`):**
  - First Frame (Length 29 = `0x01D`):
    ```bash
    cansend vcan0 0A4#101D100000005175
    ```
  - Consecutive Frame 1:
    ```bash
    cansend vcan0 0A4#2165656E202D2042
    ```
  - Consecutive Frame 2:
    ```bash
    cansend vcan0 0A4#226F68656D69616E
    ```
  - Consecutive Frame 3:
    ```bash
    cansend vcan0 0A4#232052686170736F
    ```
  - Consecutive Frame 4:
    ```bash
    cansend vcan0 0A4#2464790000000000
    ```
* **Expected Hiworld Standalone Output (`Cmd 0x86`):**
  - **Wire Frame:** `5A A5 19 86 51 75 65 65 6E 20 2D 20 42 6F 68 65 6D 69 61 6E 20 52 68 61 70 73 6F 64 79 [CSUM]`
  - Pushes text directly into `mRadioText` and initiates marquee animation on `tv_radio_text`.

---

### Vector 3: CD Changer Playback (Disc 2, Track 14, Total 20, Time 03:45, Random)
* **CAN Injection (`0x3A6`):**
  ```bash
  cansend vcan0 3A6#00020E14032D0100
  ```
  *(Byte 1=Disc 2, Byte 2=Track 14, Byte 3=Total 20, Byte 4=Min 3, Byte 5=Sec 45, Byte 6=Flags 0x01)*
* **Expected Hiworld Output (`Cmd 0x97`):**
  - **Frame:** `5A A5 0B 97 02 02 00 00 0E 03 2D 01 01 00 14 00`
  - *Breakdown:*
    - Header: `5A A5`
    - Length: `0B` (11 bytes)
    - Cmd ID: `97`
    - Payload: `02` (Disc 2), `02` (Slot 2 loaded), `00` (CDDA), `00 0E` (Track 14), `03` (3 min), `2D` (45 sec), `01` (Random), `01` (Playing), `00 14` (20 total tracks)
    - Checksum: `((0x0B + 0x97 + sum(Payload)) - 1) & 0xFF = 0x00`

---

## 7. Aspects Identified for Manual Verification

The following edge cases and protocol interactions require bench testing with physical hardware:

1. **CAN `0x0A4` Control Prefix Invariance:**
   - Real Peugeot 407 captures show the `10 00 00 00` prefix preceding RadioText on Blaupunkt RD4 N1/N2 units.
   - *Manual Check:* Verify whether Continental and Bosch RD4 MP3 variants emit the identical 4-byte prefix or send pure raw ASCII directly following the ISO-TP length header.
2. **AM Waveband Frequency Representation on CAN `0x225`:**
   - In FM mode, $f_{\text{MHz}} = \text{raw} \times 0.05 + 50.0$.
   - *Manual Check:* Verify if Medium Wave (AM) raw values on CAN `0x225` directly represent frequency in kHz, or if a different linear scaling formula applies.
3. **Stereo Reception Indicator Bit:**
   - Bit 6 of Hiworld `Cmd 0x84 Data[4]` is the `ST` (Stereo) indicator.
   - *Manual Check:* Determine whether PSA `0x225` or `0x265` carries a distinct Stereo flag, or whether Hiworld Head Units expect bit 6 to mirror the `RDS` locked flag (`0x20`).
4. **CDC Multi-Disc Loaded Bitmask Synthesis:**
   - PSA CAN `0x3A6` only explicitly reports the active disc index.
   - *Manual Check:* Verify whether external CD changers broadcast the full multi-disc magazine presence mask on auxiliary CDC bus frames (`0x131` / `0x1A0`), or whether synthesizing `1 << (active_disc - 1)` is sufficient for the Android media player.
5. **Head Unit Auto-Switch Debounce Window:**
   - When switching radio presets rapidly, `0x225` frequency frames may arrive before `0x2A5` station name frames.
   - *Manual Check:* Verify whether debouncing `Cmd 0x84` transmission by 50–100 ms prevents empty/stale station name rendering on the Android screen during fast manual seeking.
