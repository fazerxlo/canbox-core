# CAN Box Protocol Specification: RD4 Radio, CD & MFD Media Text Mirroring
## Topic 09: Peugeot 407 Media Metadata & Dashboard Screen Mirroring
**Document File:** `CANBOX_SPEC_HIWORLD_407_09_RD4_MFD_MEDIA_TEXT.md`  
**Target Platform:** Pure C99 Embedded CAN Translator & Desktop Simulator  
**Vehicle Network:** Peugeot 407 (PSA Comfort CAN Bus @ 125 kbps, 11-bit Standard ID)  
**Primary Driver Protocol:** Hiworld (`0x5A 0xA5` sync header, additive sum checksum)  
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)
**Cross-Compatible Protocols:** Raise (RZC `0x2E`), Bagoo (`0xD5`/`0xFD`), Simple Soft (XP `0x2E`)  
**Cross-Referenced Ground Truth:** `https://github.com/fazerxlo/canbox/tree/main/doc/CAN2004_radio.md`, `CAN_messages.md`

---

# 1. Functional Domain & Architecture Overview

The Peugeot 407 infotainment bus links the factory RD4 radio/CD player with the center dashboard amber Multi-Function Display (MFD - *Écran Multi-Fonctions EMF-C*):
- **OEM Media Uplink:** CD changer playback status (CAN ID `0x3A6`) and Radio RDS station names (CAN ID `0x396`) are broadcast over CAN.
- **Android Media Downlink Mirroring:** When an aftermarket Android headunit replaces the factory RD4 radio, the CAN adapter accepts ID3 song metadata (Title, Artist, Track Number) from Android via serial `Cmd 0xC1` / `0xCB` and formats it into native PSA CAN `0x396` frames. This preserves song and track title display on the factory dashboard MFD screen.
In the Peugeot 407 infotainment system, the factory RD4 radio headunit communicates with the central Multi-Function Display (EMF-C / EMF-A) over PSA Comfort CAN:
- **`0x165` (Radio Source / Mode):** Periodic 50 ms frame (`ETAT_AUTORADIO`). Byte 2 high nibble encodes the active source (`0x1`=Tuner, `0x2`=CD, `0x3`=CDC, `0x4`=AUX1, `0x5`=AUX2, `0x6`=USB, `0x7`=BT).
- **`0x225` (FM Tuner Status):** Periodic frame carrying frequency ($\text{Freq} = \text{Raw} \times 0.05 + 50.0\text{ MHz}$) and band (`0x01`=FM1, `0x02`=FM2, `0x04`=AST).
- **`0x2A5` (Radio Station Name / RDS PS):** Broadcasts 8-byte ASCII station name text (e.g. `"RMF FM  "`).
- **`0x0A4` / `0x125`:** ISO-TP style streaming transport for extended RadioText (RT) and CD track titles.
- **Android ID3 Downlink Mirroring:** When an aftermarket Android unit is installed, the CAN box translates Android song title packets (`Cmd 0xC1`) into native PSA `0x2A5` / `0x0A4` frames to display the song title on the dashboard screen.

```
+------------------------------------------------------------------------------------+
|                         Peugeot 407 Multi-Function Display (MFD)                   |
|                  [OEM Center Amber Screen / Instrument Cluster MFD]                |
+------------------------------------------------------------------------------------+
                         ▲ (Downlink ID3 Text CAN 0x396)
                         ▲ (Downlink ID3 Text CAN 0x2A5 / 0x0A4)
                         │
+------------------------------------------------------------------------------------+
|                         CAN Box Microcontroller (C99 Engine)                       |
|   1. Captures RD4 Radio / CD status frames (CAN ID 0x396 / 0x3A6)                  |
|   2. Dispatches Hiworld Media Telemetry (Cmd 0x54 / 0x55) to Android               |
|   3. Decodes Android Music Player ID3 Song Text packets (Hiworld Cmd 0xC1)         |
|   4. Converts UTF-8 / ASCII strings to PSA MFD format (CAN ID 0x396)               |
|   1. Decodes native RD4 Source (0x165), FM Tuner (0x225) and RDS Name (0x2A5)      |
|   2. Dispatches Hiworld Media Telemetry (Cmd 0x54 / 0x55)                          |
|   3. Accepts Android Downlink Cmd 0xC1 ID3 Metadata Push                           |
|   4. Synthesizes PSA CAN 0x2A5 / 0x0A4 text frames for the factory MFD screen      |
+------------------------------------------------------------------------------------+
                         │ (UART Telemetry)         ▲ (UART ID3 Metadata)
                         ▼                          │
+------------------------------------------------------------------------------------+
|                     Android Headunit Media Player / Bluetooth Music                |
|             (Transmits active track name, artist, and elapsed playback time)       |
+------------------------------------------------------------------------------------+
```

---

# 2. PSA CAN Bus Bitfield Specification

### 2.1 CD Changer / Disc Playback Status (`0x3A6`)
- **CAN ID:** `0x3A6` (DLC: 8, Cycle: 500 ms)
### 2.1 PSA Radio Source Frame (`0x165`)
- **CAN ID:** `0x165` (DLC: 4, Period: 50 ms)
- `Byte 2 Bits 7:4`: Active Source Code:
  - `0x1`: FM/AM Tuner (`TUN`)
  - `0x2`: Internal CD (`CD`)
  - `0x3`: CD Changer (`CDC`)
  - `0x4`: AUX 1 / Phone
  - `0x5`: AUX 2
  - `0x6`: USB Media
  - `0x7`: Bluetooth Audio (`BT`)

```
+--------+--------+------------------------------------+-----------------------------+
| Byte   | Bit    | Function / Signal Name             | Value / Encoding Definition |
+--------+--------+------------------------------------+-----------------------------+
| Byte 0 | 7..0   | Source Mode                        | 0x02 = CD Mode, 0x01 = Radio|
+--------+--------+------------------------------------+-----------------------------+
| Byte 1 | 7..0   | Disc Magazine Slot Number          | 1..6 (Current active slot)  |
+--------+--------+------------------------------------+-----------------------------+
| Byte 2 | 7..0   | Track Number                       | 1..99                       |
+--------+--------+------------------------------------+-----------------------------+
| Byte 3 | 7..0   | Total Tracks on Disc               | 1..99                       |
+--------+--------+------------------------------------+-----------------------------+
| Byte 4 | 7..0   | Playback Elapsed Minutes           | 0..59 Minutes               |
+--------+--------+------------------------------------+-----------------------------+
| Byte 5 | 7..0   | Playback Elapsed Seconds           | 0..59 Seconds               |
+--------+--------+------------------------------------+-----------------------------+
| Byte 6 | Bit 2  | Repeat Playback Active (RPT)       | 0x04 (1 = Active)           |
|        | Bit 1  | Track Intro Scan Active (SCAN)     | 0x02 (1 = Active)           |
|        | Bit 0  | Random / Shuffle Active (RND)      | 0x01 (1 = Active)           |
+--------+--------+------------------------------------+-----------------------------+
| Byte 7 | 7..0   | Status Flags / CD Loading State    | 0x00 = Normal Playing       |
+--------+--------+------------------------------------+-----------------------------+
```
### 2.2 PSA FM Tuner Status Frame (`0x225`)
- **CAN ID:** `0x225` (DLC: 5, Period: 100 ms)
- `Byte 2`: FM Band (`0x01`=FM1, `0x02`=FM2, `0x04`=AST, `0x08`=AM)
- `Bytes 3..4`: Tuner Frequency Raw 16-bit unsigned (Big-Endian):
  $$\text{Frequency (MHz)} = \text{Raw}_{16} \times 0.05 + 50.0$$
  $$\text{Raw}_{16} = \frac{\text{Frequency (MHz)} - 50.0}{0.05}$$
  *(Example: $96.0\text{ MHz} \implies \text{Raw} = 920 = \text{0x0398}$)*

### 2.2 RDS Radio Station Name & MFD Text Push (`0x396`)
- **CAN ID:** `0x396` (DLC: 8, Event-driven or periodic 1000 ms)
- `Bytes 0..7`: 8 bytes of ISO 8859-1 / ASCII text representing radio station PS name (e.g. `"RMF FM  "` or `"TRACK 01"`).
### 2.3 PSA RDS Station Name Frame (`0x2A5`)
- **CAN ID:** `0x2A5` (DLC: 8, Period: 500 ms)
- `Bytes 0..7`: 8 bytes of ISO 8859-1 / ASCII text representing station PS name.

---

# 3. Headunit Serial Protocol Mappings

### 3.1 Hiworld CD Changer Telemetry (`Cmd 0x54`)
### 3.1 Hiworld CD / Media Status Telemetry (`Cmd 0x54`)
- **Sync Header:** `0x5A 0xA5` | **Length:** `0x08` | **Cmd:** `0x54`
- **Payload Layout (7 Bytes):**
  - `Byte 0`: Mode (`0x02` = CD Player)
  - `Byte 1`: Disc Number ($1 \dots 6$)
  - `Byte 0`: Mode (`0x01` = Radio, `0x02` = CD, `0x06` = USB, `0x07` = BT)
  - `Byte 1`: Disc Slot Number ($1 \dots 6$)
  - `Byte 2`: Track Number ($1 \dots 99$)
  - `Byte 3`: Total Tracks ($1 \dots 99$)
  - `Byte 4`: Elapsed Minutes ($0 \dots 59$)
  - `Byte 5`: Elapsed Seconds ($0 \dots 59$)
  - `Byte 6`: Play Flags (`0x01`=RND, `0x02`=SCAN, `0x04`=RPT)
- **Checksum:** 8-bit sum modulo 256 over `Length + CmdID + Payload`.

### 3.2 Hiworld Radio Station RDS Name (`Cmd 0x55`)
### 3.2 Hiworld Radio Station Name (`Cmd 0x55`)
- **Sync Header:** `0x5A 0xA5` | **Length:** `0x09` | **Cmd:** `0x55`
- **Payload:** 8 bytes ASCII characters.

### 3.3 Downlink Android Song Title Push (`Cmd 0xC1` / `0xCB`)
When Android plays a song, it pushes ID3 metadata to the CAN box:
- **Format:** `5A A5 [Len] C1 [TextType: 0x01=Title, 0x02=Artist] [ASCII String...] [CS]`
- **Action:** CAN Box slices the string into 8-byte chunks and injects them onto PSA CAN ID `0x396`.

---

# 4. Pure C99 Firmware Implementation

```c
#ifndef CANBOX_MEDIA_H
#define CANBOX_MEDIA_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

typedef struct {
    uint8_t cd_disc_num;    /* 1..6 */
    uint8_t cd_track_num;   /* 1..99 */
    uint8_t cd_total_tracks;
    uint8_t cd_elapsed_min;
    uint8_t cd_elapsed_sec;
    uint8_t cd_play_flags;  /* Bit 0: RND, Bit 1: SCAN, Bit 2: RPT */
    char    rds_ps_name[9]; /* 8 chars + null */
    uint8_t source_mode;    /* 1=TUN, 2=CD, 3=CDC, 4=AUX1, 5=AUX2, 6=USB, 7=BT */
    uint16_t tuner_freq_khz;/* Frequency in kHz */
    char    rds_ps_name[9]; /* 8 chars null-terminated */
    uint8_t cd_track_num;
    uint8_t cd_disc_num;
} psa_media_state_t;

typedef void (*canbox_uart_tx_fn)(const uint8_t *buf, size_t len);
typedef void (*canbox_can_tx_fn)(uint32_t id, const uint8_t *data, uint8_t dlc);

typedef struct {
    psa_media_state_t state;
    canbox_uart_tx_fn uart_tx;
    canbox_can_tx_fn  can_tx;
} psa_media_ctx_t;

static inline void psa_media_init(psa_media_ctx_t *ctx, canbox_uart_tx_fn uart_tx, canbox_can_tx_fn can_tx) {
    memset(ctx, 0, sizeof(psa_media_ctx_t));
    ctx->uart_tx = uart_tx;
    ctx->can_tx  = can_tx;
}

/* Transmit Hiworld CD Changer Telemetry (Cmd 0x54) */
static inline void psa_media_send_cd_hiworld(psa_media_ctx_t *ctx) {
/* Transmit Hiworld RDS Station Name Packet (Cmd 0x55) */
static inline void psa_media_send_rds_hiworld(psa_media_ctx_t *ctx) {
    if (!ctx->uart_tx) return;

    uint8_t p[11];
    uint8_t p[12];
    p[0] = 0x5A;
    p[1] = 0xA5;
    p[2] = 0x08; /* Length: 1 Cmd + 7 Payload */
    p[3] = 0x54; /* Cmd ID */
    p[4] = 0x02; /* CD Mode */
    p[5] = ctx->state.cd_disc_num;
    p[6] = ctx->state.cd_track_num;
    p[7] = ctx->state.cd_total_tracks;
    p[8] = ctx->state.cd_elapsed_min;
    p[9] = ctx->state.cd_elapsed_sec;
    p[10] = ctx->state.cd_play_flags;
    p[2] = 0x09; /* Length: 1 Cmd + 8 Payload */
    p[3] = 0x55; /* Cmd ID */
    memcpy(&p[4], ctx->state.rds_ps_name, 8);

    uint8_t sum = 0;
    for (size_t i = 2; i <= 10; i++) sum += p[i];
    p[11] = sum;
    for (size_t i = 2; i <= 11; i++) sum += p[i];
    p[12] = sum;

    ctx->uart_tx(p, 12);
    ctx->uart_tx(p, 13);
}

/* Process PSA CAN 0x3A6 (CD Changer Telemetry) */
static inline void psa_media_process_can_0x3A6(psa_media_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 7) return;
/* Process PSA CAN 0x165 (Radio Source) */
static inline void psa_media_process_can_0x165(psa_media_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 3) return;
    ctx->state.source_mode = (data[2] >> 4) & 0x0F;
}

    ctx->state.cd_disc_num     = data[1];
    ctx->state.cd_track_num    = data[2];
    ctx->state.cd_total_tracks = data[3];
    ctx->state.cd_elapsed_min  = data[4];
    ctx->state.cd_elapsed_sec  = data[5];
    ctx->state.cd_play_flags   = data[6];
/* Process PSA CAN 0x225 (FM Tuner Frequency) */
static inline void psa_media_process_can_0x225(psa_media_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 5) return;
    uint16_t raw = ((uint16_t)data[3] << 8) | (uint16_t)data[4];
    ctx->state.tuner_freq_khz = (uint16_t)((raw * 50) + 50000);
}

    psa_media_send_cd_hiworld(ctx);
/* Process PSA CAN 0x2A5 (RDS Station Name Text) */
static inline void psa_media_process_can_0x2A5(psa_media_ctx_t *ctx, const uint8_t *data, uint8_t dlc) {
    if (dlc < 8) return;
    memcpy(ctx->state.rds_ps_name, data, 8);
    ctx->state.rds_ps_name[8] = '\0';
    psa_media_send_rds_hiworld(ctx);
}

/* Process Android Downlink Song Title Push (Cmd 0xC1) -> Forward to PSA MFD CAN 0x396 */
/* Process Android Downlink Song Title Push (Cmd 0xC1) -> Forward to PSA MFD CAN 0x2A5 */
static inline void psa_media_process_downlink_title(psa_media_ctx_t *ctx, const uint8_t *payload, size_t len) {
    if (!ctx->can_tx || len < 2) return;

    /* Extract up to 8 ASCII characters */
    uint8_t can_data[8];
    memset(can_data, ' ', sizeof(can_data));

    size_t str_len = len - 1; /* Skip TextType prefix */
    size_t str_len = len - 1; /* Skip TextType byte */
    if (str_len > 8) str_len = 8;
    memcpy(can_data, &payload[1], str_len);

    ctx->can_tx(0x396, can_data, 8);
    ctx->can_tx(0x2A5, can_data, 8);
}

#endif /* CANBOX_MEDIA_H */
```

---

# 5. Verification Vectors & Simulation Harness

### Vector 1: CD Disc 1, Track 14, Total 20, Elapsed 02:45, Random Mode Active
- **CAN ID `0x3A6` Injection:**
### Vector 1: FM Station `"BBC R1  "` Received on PSA CAN2004 (`0x2A5`)
- **CAN ID `0x2A5` Injection:**
  ```bash
  cansend vcan0 3A6#02010E14022D0100
  cansend vcan0 2A5#4242432052312020
  ```
- **Expected UART Output (Hiworld `0x54`):**
  - Frame: `5A A5 08 54 02 01 0E 14 02 2D 01 B0`
- **Expected UART Output (Hiworld `0x55`):**
  - Frame: `5A A5 09 55 42 42 43 20 52 31 20 20 04`

### Vector 2: Android Pushes Song Title `"RADIO 1 "` Downlink
- **UART Downlink Received (`Cmd 0xC1`):**
  - `5A A5 0A C1 01 52 41 44 49 4F 20 31 1D`
- **Expected CAN Frame Injected onto MFD:**
  - ID: `0x396`, DLC: 8, Data: `52 41 44 49 4F 20 31 20` (`"RADIO 1 "`)

### Vector 2: Tuner at 96.0 MHz (Raw 920 = 0x0398) on FM1 (`0x225`)
- **CAN ID `0x225` Injection:**
  ```bash
  cansend vcan0 225#0000010398
  ```
- **Decoded Frequency:**
  - $\text{Freq} = 920 \times 0.05 + 50.0 = 96.0\text{ MHz}$
