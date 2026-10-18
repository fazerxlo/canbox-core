# PSA RD4 Volume & Audio Settings (DSP) Specification & Firmware Implementation Guide

This document specifies the reverse-engineered **PSA Peugeot/Citroën RD4 Audio & Volume protocol** and standardizes the UART bridge protocol (`Cmd 0x56`) between the **CANbox microcontroller firmware** and the **Android Head Unit (`com.qf.vehicle`)**.

---

## 1. Comfort CAN Bus Specifications (PSA CAN Confort)

| Parameter | Specification |
|---|---|
| **Bus Standard** | ISO 11898 Fault-tolerant / Low-Speed CAN |
| **Baud Rate** | **125 kbps** (`CAN_125KBPS`) |
| **Identifier Type** | Standard 11-bit (`0x000` – `0x7FF`) |
| **Volume CAN ID** | **`0x1A5`** (421 Decimal), DLC $\ge 1$ |
| **Sound Settings CAN ID** | **`0x1E5`** (485 Decimal), DLC $\ge 7$ |
| **Transmitting Node** | RD4 Audio Head Unit (Blaupunkt / Continental / Bosch) |

---

## 2. CAN Frame Details

### 2.1 Volume Frame: `0x1A5`
Broadcast when the RD4 volume knob is rotated, stalk `VOL+` / `VOL-` buttons are pressed, or periodically to maintain state.

```
  Bit 7      Bit 6      Bit 5      Bit 4      Bit 3      Bit 2      Bit 1      Bit 0
┌───────────────────────┬─────────────────────────────────────────────────────────┐
│     ACTIVITY STATE    │                 VOLUME LEVEL (0 to 30)                  │
└───────────────────────┴─────────────────────────────────────────────────────────┘
```
- **Bits [4:0] (`& 0x1F`):** Master volume level ($0 \dots 30$).
- **Bits [7:5] (`& 0xE0`):** Activity / Rotation status:
  - `0x00` (`0b000`): Volume knob is actively rotating / changing $\to$ `active_mode = 1`, `is_muted = false`.
  - `0xE0` (`0b111`): Volume knob is stopped / stationary $\to$ `active_mode = 0` (dismisses volume HUD), `is_muted = false`.
  - `0x80` (`0b100`): Isolated bit 7 $\to$ explicit mute active (`is_muted = true`).

### 2.2 Sound Settings & Equalizer Frame: `0x1E5`
Broadcast when the RD4 Audio/Music button or arrow keys are pressed, or periodically.

```
  Byte 0       Byte 1       Byte 2       Byte 3       Byte 4       Byte 5       Byte 6
┌────────────┬────────────┬────────────┬────────────┬────────────┬────────────┬────────────┐
│ [Act][Bal] │ [Act][Fad] │ [Act][Bass]│  Reserved  │ [Act][Treb]│ [Act][Loud]│ [Act][Eq]  │
│  L/R Bal   │  F/R Bal   │    Bass    │    0x00    │   Treble   │  AutoVol   │  Preset    │
└────────────┴────────────┴────────────┴────────────┴────────────┴────────────┴────────────┘
```

#### Bias-63 Decoding for Bass, Treble, Balance, Fader:
$$\text{Level}_{-9\dots+9} = (\text{RawByte} \ \& \ 0\text{x}7\text{F}) - 63$$
- Center is `63` (`0x3F`) $\rightarrow 0$.
- Range: `-9` (`54` / `0x36`) to `+9` (`72` / `0x48`).

#### Active Mode Detection (which parameter is currently being navigated/edited):
- `(Byte0 & 0x80) != 0`: Left / Right Balance active
- `(Byte1 & 0x80) != 0`: Front / Rear Fader active
- `(Byte2 & 0x80) != 0`: Bass active
- `(Byte4 & 0x80) != 0`: Treble active
- `(Byte5 & 0x80) != 0`: Loudness active (`(Byte5 & 0x40) != 0` $\rightarrow$ ON)
- `(Byte5 & 0x10) != 0`: Auto-Volume active (`(Byte5 & 0x07) == 0x07` $\rightarrow$ ON)
- `(Byte6 & 0x40) != 0`: Equalizer active
- **Menu Closed / Inactive:** When all active mode bits are `0`, the menu is closed.

#### Equalizer Presets (`Byte6 & 0xBF`):
- `0x00` $\rightarrow$ None / Custom (`0`)
- `0x07` $\rightarrow$ Classical (`1`)
- `0x0B` $\rightarrow$ Jazz-Blues (`2`)
- `0x0F` $\rightarrow$ Pop-Rock (`3`)
- `0x13` $\rightarrow$ Vocals (`4`)
- `0x17` $\rightarrow$ Techno (`5`)

---

## 3. CANbox-to-Head-Unit UART Protocol: `Cmd 0x56`

The CANbox microcontroller combines volume and audio settings into a single **13-byte wire telemetry frame** sent over UART:

```
[0x5A] [0xA5] [0x08] [0x56] [DATA_0 .. DATA_7] [CHECKSUM]
```

### 3.1 Field Definitions

| Wire Offset | Field | Description | Range / Values |
|---|---|---|---|
| 0 | `Header[0]` | Sync Header Byte 1 | `0x5A` |
| 1 | `Header[1]` | Sync Header Byte 2 | `0xA5` |
| 2 | `Length` | Payload Byte Count | `0x08` (8 bytes) |
| 3 | `Command` | Telemetry Opcode | **`0x56`** |
| 4 | `Data[0]` | Status & Active Mode | Bit 7: Mute (`1`=Muted, `0`=Unmuted)<br>Bits [3:0]: Active Mode ($0 \dots 8$) |
| 5 | `Data[1]` | Bass | $0 \dots 18$ ($9 = \text{Center}$, $0 = -9$, $18 = +9$) |
| 6 | `Data[2]` | Treble | $0 \dots 18$ ($9 = \text{Center}$, $0 = -9$, $18 = +9$) |
| 7 | `Data[3]` | Balance | $0 \dots 18$ ($9 = \text{Center}$, $0 = \text{Left 9}$, $18 = \text{Right 9}$) |
| 8 | `Data[4]` | Fader | $0 \dots 18$ ($9 = \text{Center}$, $0 = \text{Rear 9}$, $18 = \text{Front 9}$) |
| 9 | `Data[5]` | EQ Preset Profile | $0 = \text{None}$, $1 = \text{Classical}$, $2 = \text{Jazz-Blues}$, $3 = \text{Pop-Rock}$, $4 = \text{Vocals}$, $5 = \text{Techno}$ |
| 10 | `Data[6]` | Flags | Bit 4 (`0x10`): Loudness (`1`=ON, `0`=OFF)<br>Bit 0 (`0x01`): Speed-Dependent Volume (`1`=ON, `0`=OFF) |
| 11 | `Data[7]` | Master Volume | $0 \dots 30$ |
| 12 | `Checksum` | Framing Checksum | `((Length + Cmd + sum(Data)) - 1) & 0xFF` |

### 3.2 Active Mode Mapping (`Data[0] & 0x0F`):
- `0` = None / Passive sync (RD4 audio menu is closed, volume not actively turning)
- `1` = Master Volume active (knob rotated or SWC VOL pressed)
- `2` = Bass adjustment active
- `3` = Treble adjustment active
- `4` = Balance (Left / Right) adjustment active
- `5` = Fader (Front / Rear) adjustment active
- `6` = Loudness adjustment active
- `7` = Speed-dependent Volume adjustment active
- `8` = Equalizer Preset adjustment active

### 3.3 Formula: Converting CAN `0x1E5` to UART Scale ($0 \dots 18$):
$$\text{uart\_val} = \text{clamp}\Big(\big((\text{can\_byte} \ \& \ 0\text{x}7\text{F}) - 63\big) + 9,\ 0,\ 18\Big) = \text{clamp}\Big((\text{can\_byte} \ \& \ 0\text{x}7\text{F}) - 54,\ 0,\ 18\Big)$$

---

## 4. Prompt for CANbox Firmware AI Agent

Copy and paste the prompt below into the AI agent working on the CANbox microcontroller firmware:

````markdown
You are programming the embedded C/C++ firmware for a PSA CANbox microcontroller (STM32, GD32, or Arduino MCP2515) connected between PSA Comfort CAN (125 kbps) and an Android Head Unit (UART 38400 or 115200 baud).

### TASK OBJECTIVE
Capture Volume (CAN `0x1A5`) and Sound/Equalizer Settings (CAN `0x1E5`) from the PSA RD4 radio and transmit unified telemetry frames (`Cmd 0x56`, 13 bytes total) over UART to the Head Unit.

### 1. CAN INPUT FRAMES
- **`0x1A5` (Volume, DLC >= 1):**
  - `data[0] & 0x1F`: Master Volume ($0 \dots 30$).
  - `data[0] & 0x80`: Mute flag (`0x80` = Muted, `0x00` = Unmuted).
- **`0x1E5` (Sound Settings, DLC >= 7):**
  - `data[0]`: Left/Right Balance (Bit 7 = active mode, Bits [6:0] = raw value with bias 63, range 54..72).
  - `data[1]`: Front/Rear Fader (Bit 7 = active mode, Bits [6:0] = raw value with bias 63, range 54..72).
  - `data[2]`: Bass (Bit 7 = active mode, Bits [6:0] = raw value with bias 63, range 54..72).
  - `data[4]`: Treble (Bit 7 = active mode, Bits [6:0] = raw value with bias 63, range 54..72).
  - `data[5]`: Bit 7 = Loudness active, Bit 6 = Loudness state (`0x40`=ON, `0x00`=OFF); Bit 4 = Speed Volume active, Bits [2:0] = Speed Volume state (`0x07`=ON, `0x00`=OFF).
  - `data[6]`: Bit 6 = Equalizer active, Bits [5:0] & 0xBF = Preset (`0x00`=None, `0x07`=Classical, `0x0B`=Jazz-Blues, `0x0F`=Pop-Rock, `0x13`=Vocals, `0x17`=Techno).

### 2. UART OUTPUT FRAME (`Cmd 0x56`, 13 Bytes Wire Length)
Format:
```
[0x5A] [0xA5] [0x08] [0x56] [D0] [D1] [D2] [D3] [D4] [D5] [D6] [D7] [CHECKSUM]
```
Where:
- **`D0`:** `(isMuted ? 0x80 : 0x00) | (activeMode & 0x0F)`
  - Active Mode IDs:
    - `0` = None (menu closed, idle)
    - `1` = Volume changed / active
    - `2` = Bass active
    - `3` = Treble active
    - `4` = Balance active
    - `5` = Fader active
    - `6` = Loudness active
    - `7` = Speed Volume active
    - `8` = Equalizer active
- **`D1` (Bass):** Clamped $0 \dots 18$ ($9 = \text{Center}$): `clamp((data[2] & 0x7F) - 54, 0, 18)`
- **`D2` (Treble):** Clamped $0 \dots 18$ ($9 = \text{Center}$): `clamp((data[4] & 0x7F) - 54, 0, 18)`
- **`D3` (Balance):** Clamped $0 \dots 18$ ($9 = \text{Center}$): `clamp((data[0] & 0x7F) - 54, 0, 18)`
- **`D4` (Fader):** Clamped $0 \dots 18$ ($9 = \text{Center}$): `clamp((data[1] & 0x7F) - 54, 0, 18)`
- **`D5` (EQ Preset):**
  - `0` = None (`0x00`)
  - `1` = Classical (`0x07`)
  - `2` = Jazz-Blues (`0x0B`)
  - `3` = Pop-Rock (`0x0F`)
  - `4` = Vocals (`0x13`)
  - `5` = Techno (`0x17`)
- **`D6` (Flags):** `(loudnessOn ? 0x10 : 0x00) | (speedVolOn ? 0x01 : 0x00)`
- **`D7` (Master Volume):** $0 \dots 30$
- **`CHECKSUM`:** `((0x08 + 0x56 + D0 + D1 + D2 + D3 + D4 + D5 + D6 + D7) - 1) & 0xFF`

### 3. TRANSMISSION TIMING RULES
1. **Immediate On Event:** Transmit immediately whenever `0x1A5` is received with an actively rotating knob or changed volume (`activeMode = 1`).
2. **Immediate On Setting Change:** Transmit immediately whenever `0x1E5` indicates an active editing mode transition (`activeMode = 2..8`) or parameter value changes.
3. **Anti-Flicker Delta Dispatch:** While actively in an editing mode (`activeMode >= 2`), suppress redundant UART transmissions on repeated identical CAN frames to eliminate UI jitter.
4. **Menu Dismissal (Direct Vehicle Ground Truth):** In CAN `0x1E5`, the RD4 radio explicitly sets the active-mode bit while a menu is open and clears all active bits to 0 upon menu timeout or user exit. When all active mode bits drop to 0 (or volume knob stops rotating on `0x1A5` with `0xE0`), transmit one frame with `activeMode = 0` so the Head Unit knows to dismiss the HUD. No artificial backup timers are needed.
5. **Gated Heartbeat / Passive Sync:** Transmit periodically (every 1000 ms) with `activeMode = 0` strictly when `activeMode == 0`. Gated and suppressed during active editing (`activeMode != 0`) to prevent the periodic timer from closing active menus.

### 4. C REFERENCE IMPLEMENTATION
```c
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef struct {
    uint8_t masterVolume;   // 0..30
    bool isMuted;           // true/false
    uint8_t bass;           // 0..18, 9 = Center
    uint8_t treble;         // 0..18, 9 = Center
    uint8_t balance;        // 0..18, 9 = Center
    uint8_t fader;          // 0..18, 9 = Center
    uint8_t eqPreset;       // 0..5
    bool loudness;          // true/false
    bool speedVolume;       // true/false (ON/OFF only)
    uint8_t activeMode;     // 0..8
} RD4_AudioState;

static RD4_AudioState g_audioState = {
    .masterVolume = 15,
    .isMuted = false,
    .bass = 9,
    .treble = 9,
    .balance = 9,
    .fader = 9,
    .eqPreset = 0,
    .loudness = false,
    .speedVolume = false,
    .activeMode = 0
};

static uint8_t clamp18(int val) {
    if (val < 0) return 0;
    if (val > 18) return 18;
    return (uint8_t)val;
}

void parse_RD4_volume_frame(const uint8_t* can_data, uint8_t dlc) {
    if (dlc < 1) return;
    uint8_t new_vol = can_data[0] & 0x1F;
    if (new_vol > 30) new_vol = 30;
    bool new_mute = (can_data[0] & 0x80) != 0;

    bool changed = (new_vol != g_audioState.masterVolume) || (new_mute != g_audioState.isMuted);
    g_audioState.masterVolume = new_vol;
    g_audioState.isMuted = new_mute;

    if (changed) {
        g_audioState.activeMode = 1; // Volume active
        send_uart_dsp_telemetry();
    }
}

void parse_RD4_audio_settings_frame(const uint8_t* can_data, uint8_t dlc) {
    if (dlc < 7) return;

    // Decode levels (bias 63 to scale 0..18 where 9 is center)
    g_audioState.balance = clamp18(((int)(can_data[0] & 0x7F) - 63) + 9);
    g_audioState.fader   = clamp18(((int)(can_data[1] & 0x7F) - 63) + 9);
    g_audioState.bass    = clamp18(((int)(can_data[2] & 0x7F) - 63) + 9);
    g_audioState.treble  = clamp18(((int)(can_data[4] & 0x7F) - 63) + 9);

    // Switches (ON/OFF only)
    g_audioState.loudness    = (can_data[5] & 0x40) != 0;
    g_audioState.speedVolume = ((can_data[5] & 0x07) == 0x07);

    // EQ Preset
    uint8_t eq_raw = can_data[6] & 0xBF;
    switch (eq_raw) {
        case 0x07: g_audioState.eqPreset = 1; break; // Classical
        case 0x0B: g_audioState.eqPreset = 2; break; // Jazz-Blues
        case 0x0F: g_audioState.eqPreset = 3; break; // Pop-Rock
        case 0x13: g_audioState.eqPreset = 4; break; // Vocals
        case 0x17: g_audioState.eqPreset = 5; break; // Techno
        default:   g_audioState.eqPreset = 0; break; // None
    }

    // Detect active editing mode
    uint8_t prev_mode = g_audioState.activeMode;
    if (can_data[2] & 0x80)      g_audioState.activeMode = 2; // Bass
    else if (can_data[4] & 0x80) g_audioState.activeMode = 3; // Treble
    else if (can_data[0] & 0x80) g_audioState.activeMode = 4; // Balance
    else if (can_data[1] & 0x80) g_audioState.activeMode = 5; // Fader
    else if (can_data[5] & 0x80) g_audioState.activeMode = 6; // Loudness
    else if (can_data[5] & 0x10) g_audioState.activeMode = 7; // Speed Volume
    else if (can_data[6] & 0x40) g_audioState.activeMode = 8; // Equalizer
    else                         g_audioState.activeMode = 0; // None / Closed

    send_uart_dsp_telemetry();
}

void send_uart_dsp_telemetry(void) {
    uint8_t packet[13];
    packet[0] = 0x5A;
    packet[1] = 0xA5;
    packet[2] = 0x08; // Length
    packet[3] = 0x56; // Opcode Cmd 0x56

    packet[4] = (g_audioState.isMuted ? 0x80 : 0x00) | (g_audioState.activeMode & 0x0F);
    packet[5] = g_audioState.bass;
    packet[6] = g_audioState.treble;
    packet[7] = g_audioState.balance;
    packet[8] = g_audioState.fader;
    packet[9] = g_audioState.eqPreset;
    packet[10] = (g_audioState.loudness ? 0x10 : 0x00) | (g_audioState.speedVolume ? 0x01 : 0x00);
    packet[11] = g_audioState.masterVolume;

    // Checksum calculation: ((Length + Cmd + sum(Payload)) - 1) & 0xFF
    uint16_t sum = packet[2] + packet[3];
    for (int i = 4; i < 12; i++) {
        sum += packet[i];
    }
    packet[12] = (uint8_t)((sum - 1) & 0xFF);

    uart_write_bytes(packet, 13);
}
```
````

