# CAN Box Protocol Specification: RD4 Audio, CD Changer & Multi-Function Display (MFD)
## Topic 09: Peugeot 407 Radio, CD Text & MFD Screen Synchronization
**Document File:** `CANBOX_SPEC_HIWORLD_407_09_RD4_MFD_MEDIA_TEXT.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** `signal-db/radio.yaml` (`Msg0A4`, `Msg165`, `Msg1A5`, `Msg1E5`, `Msg225`, `Msg2A5`, `Msg3E5`), `QF_Canbus.apk` (`PeugeotDataParser.java`)

---

# 1. Functional Domain & Architecture Overview

The Peugeot 407 RD4 headunit and Multi-Function Display (MFD Type C/Monochrome/Color) exchange CD track playback status, radio station RDS names, tuner frequency, and audio settings across PSA Comfort CAN frames:
- **`0x0A4` (RDS RadioText, 500 ms):** Transmits up to 64-character RDS RadioText strings using ISO-TP (ISO 15765-2) multi-frame segmentation (Single Frame `0x0N`, First Frame `0x1N`, Consecutive Frame `0x2N`).
- **`0x165` (Radio Input Source, 50 ms):** `[0xCC, 0x54, (input_code << 4), 0x02]` indicating active audio source (Tuner, CD, AUX, etc.).
- **`0x1A5` (Radio Volume Level, 100 ms):** `[volflag | (volume & 0x1F)]` for audio gain / master volume.
- **`0x1E5` (Audio Settings, 100 ms):** Balance, Fader, Bass, Treble, Loudness boost, and Equalizer Ambiance profiles.
- **`0x225` (FM Tuner Status, 100 ms):** Band (FM1/FM2/AST/AM), preset memory index, RDS flags, and tuned frequency ($\text{Display MHz} = \text{Raw} \times 0.05 + 50.0$).
- **`0x2A5` (RDS Station PS Name, 100 ms):** 8-byte ASCII string representing Programme Service name.
- **`0x3A6` (CD Playback Telemetry, 500 ms):** Current Disc ($1 \dots 6$), Track ($1 \dots 99$), and elapsed time ($MM:SS$).
- **`0x3E5` (Fascia Control Buttons, 50 ms):** Front panel buttons (Menu, Tel, Clim, Trip, Mode, Audio, OK, ESC, directional navigation).

The CAN box translates this media telemetry into Hiworld media state frames (`Cmd 0x97` / `0x93`) and forwards Android ID3 track title text (`Cmd 0xE4`) onto the vehicle MFD screen.

```
+------------------------------------------------------------------------------------+
|                         Peugeot 407 RD4 Radio & MFD Screen                         |
|             [CD Track, Disc, Duration, RDS Station Name, Tuner Freq]               |
|      [ISO-TP RadioText (0x0A4), Tuner (0x225), Station (0x2A5), CD (0x3A6)]        |
+------------------------------------------------------------------------------------+
                                          │
                  [PSA CAN 0x0A4 / 0x225 / 0x2A5 / 0x3A6 / 0x165]
                                          ▼
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Captures CD playback status from CAN ID 0x3A6                                 |
|   2. Decodes RDS Station name from 0x2A5 & ISO-TP RadioText from 0x0A4             |
|   3. Decodes Tuner frequency from 0x225 (MHz = raw * 0.05 + 50)                    |
|   4. Serializes Hiworld Media frames (Cmd 0x97 / 0x93)                             |
|   5. Accepts Android song text (Cmd 0xE4) and injects onto MFD display             |
+------------------------------------------------------------------------------------+
                                          │
                    [UART Serial: 38400 baud, 8N1 / Hiworld Protocol]
                                          ▼
+------------------------------------------------------------------------------------+
|                     Android Headunit Media Application                             |
|           (Displays RD4 Radio/CD playback information & status)                    |
+------------------------------------------------------------------------------------+
```

---

# 2. PSA CAN Bus Bitfield Specification

### 2.1 PSA Tuner Frequency & Band (`0x225`)
- **CAN ID:** `0x225` (DLC: 5, Period: 100 ms)
- `Byte 0`: Bit 7 = LIST, Bit 6 = SCAN, Bit 5 = RDS, Bit 4 = PTY, Bit 3 = TUN, Bit 2 = TA, Bits 1:0 = TUNDIR.
- `Byte 1`: Preset memory index ($1 \dots 6$).
- `Byte 2`: Band code (`0x90` = FM1, `0xA0` = FM2, `0xC0` = FM-AST, `0xD0` = AM/MW).
- `Bytes 3..4`: Raw frequency uint16 big-endian ($\text{Frequency MHz} = \text{Raw} \times 0.05 + 50.0$).

### 2.2 PSA RDS Station Name (`0x2A5`)
- **CAN ID:** `0x2A5` (DLC: 8, Period: 100 ms)
- `Bytes 0..7`: 8-byte ASCII Station Name (e.g. `"RMF FM  "`, `"RADIO ZET"`).

### 2.3 PSA CD Changer / Playback Status Frame (`0x3A6`)
- **CAN ID:** `0x3A6` (DLC: 8, Period: 500 ms)
- `Byte 0`: Disc Number ($1 \dots 6$, 0 = No disc).
- `Byte 1`: Track Number ($1 \dots 99$).
- `Byte 2`: Track Elapsed Minutes ($0 \dots 59$).
- `Byte 3`: Track Elapsed Seconds ($0 \dots 59$).
- `Byte 4`: Playback Mode Flags (`0x01` = RND, `0x02` = SCAN, `0x04` = RPT).

### 2.4 PSA RDS RadioText Multi-Frame (`0x0A4`)
- **CAN ID:** `0x0A4` (DLC: 8, Period: 500 ms)
- Uses ISO 15765-2 (ISO-TP) framing:
  - Single Frame: `data[0] = 0x0N` (length N 1..7, payload in bytes 1..N).
  - First Frame: `data[0..1] = 0x1H 0xLL` (total length, payload in bytes 2..7).
  - Consecutive Frame: `data[0] = 0x2N` (sequence N 1..15, payload in bytes 1..7).

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Hiworld Media Telemetry Frame (`Cmd 0x97` / `Handle.CarMediaState`)
- **Sync Header:** `0x5A 0xA5`
- **Length ($L$):** `0x07` (7 payload bytes)
- **Command ID:** `0x97` (`151` unsigned / `-105` signed / `Handle.CarMediaState`)
- **Payload Layout (7 Bytes):**
  - `Byte 0`: Source Type (`0x01`=Tuner, `0x02`=CD, `0x03`=CDC, `0x05`=BT, `0x06`=USB, `0x07`=AUX)
  - `Byte 1`: Disc Number ($1 \dots 6$)
  - `Byte 2`: Track Number ($1 \dots 99$)
  - `Byte 3`: Total Tracks ($1 \dots 99$)
  - `Byte 4`: Elapsed Minutes ($0 \dots 59$)
  - `Byte 5`: Elapsed Seconds ($0 \dots 59$)
  - `Byte 6`: Play Flags (`0x01`=RND, `0x02`=SCAN, `0x04`=RPT)
- **Checksum:** `((Length + CmdID + sum(Payload)) - 1) & 0xFF`
- **Total Frame Wire Length:** 12 bytes (`5A A5 07 97 [7 Bytes] Checksum`)

---

# 4. Pure C99 Firmware Implementation

```c
#ifndef CANBOX_MEDIA_H
#define CANBOX_MEDIA_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define HIWORLD_SOF1             0x5A
#define HIWORLD_SOF2             0xA5
#define HIWORLD_CMD_MEDIA_STATE  0x97

typedef struct {
    uint8_t source_mode;    /* 1=TUN, 2=CD, 3=CDC, 5=BT, 6=USB, 7=AUX */
    uint8_t cd_disc_num;    /* 1..6 */
    uint8_t cd_track_num;   /* 1..99 */
    uint8_t cd_total_tracks;
    uint8_t cd_elapsed_min;
    uint8_t cd_elapsed_sec;
    uint8_t cd_play_flags;  /* Bit 0: RND, Bit 1: SCAN, Bit 2: RPT */
    char    station_name[9];
    uint16_t tuner_freq_raw; /* freq = raw * 0.05 + 50 */
} psa_media_state_t;

typedef void (*canbox_uart_tx_fn)(const uint8_t *buf, size_t len);

typedef struct {
    psa_media_state_t state;
    canbox_uart_tx_fn uart_tx;
} psa_media_ctx_t;

static inline void psa_media_init(psa_media_ctx_t *ctx, canbox_uart_tx_fn uart_tx) {
    memset(ctx, 0, sizeof(psa_media_ctx_t));
    ctx->uart_tx = uart_tx;
}

/* Transmit Hiworld Media Telemetry (Cmd 0x97) */
static inline void psa_media_send_hiworld(psa_media_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[12];
    p[0] = HIWORLD_SOF1;
    p[1] = HIWORLD_SOF2;
    p[2] = 0x07;                    /* Length: 7 Payload bytes */
    p[3] = HIWORLD_CMD_MEDIA_STATE; /* Cmd ID: 0x97 */
    p[4] = ctx->state.source_mode;
    p[5] = ctx->state.cd_disc_num;
    p[6] = ctx->state.cd_track_num;
    p[7] = ctx->state.cd_total_tracks;
    p[8] = ctx->state.cd_elapsed_min;
    p[9] = ctx->state.cd_elapsed_sec;
    p[10] = ctx->state.cd_play_flags;

    uint8_t sum = 0;
    for (size_t i = 2; i <= 10; i++) {
        sum += p[i];
    }
    p[11] = (uint8_t)((sum - 1) & 0xFF);

    ctx->uart_tx(p, 12);
}

/* Process PSA CAN 0x3A6 (CD Changer Telemetry) */
static inline void psa_media_process_can_0x3A6(psa_media_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 5) return;
    ctx->state.source_mode     = 0x02; /* CD */
    ctx->state.cd_disc_num     = data[0];
    ctx->state.cd_track_num    = data[1];
    ctx->state.cd_elapsed_min  = data[2];
    ctx->state.cd_elapsed_sec  = data[3];
    ctx->state.cd_play_flags   = data[4];
    psa_media_send_hiworld(ctx);
}

/* Process PSA CAN 0x2A5 (RDS Station Name) */
static inline void psa_media_process_can_0x2A5(psa_media_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    uint8_t len = (dlc < 8) ? dlc : 8;
    memcpy(ctx->state.station_name, data, len);
    ctx->state.station_name[len] = '\0';
}

/* Process PSA CAN 0x225 (Tuner Status) */
static inline void psa_media_process_can_0x225(psa_media_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 5) return;
    ctx->state.source_mode    = 0x01; /* Tuner */
    ctx->state.tuner_freq_raw = ((uint16_t)data[3] << 8) | data[4];
}

#endif /* CANBOX_MEDIA_H */
```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: CD Track 12, Disc 1, Time 03:45
- **CAN ID `0x3A6` Injection:**
  ```bash
  cansend vcan0 3A6#010C032D00000000
  ```
- **Expected UART Output (Hiworld `0x97`):**
  - Frame: `5A A5 07 97 02 01 0C 00 03 2D 00 DB`
