# PSA RD4 Bidirectional Audio Input Switching & SWC Specification

## Target Audience: AI Agents, Android App Developers & Firmware Engineers
**Platforms:** PSA Peugeot 407, 307, 308, Citroën C4, C5 (RD4 Audio Architecture)  
**Components:**
1. **Android Head Unit:** `com.qf.vehicle` (`PsaAudioRouteManager`, `PeugeotDataParser`, `PeugeotDataController`, `PeugeotDataDefine`)
2. **CAN Box Firmware:** `canbox-core` (`src/profiles/psa_virtual_cdc.c`, `src/core/can_router.c`, `src/protocols/proto_hiworld.c`, `src/protocols/proto_hiworld_adapter.c`)
3. **Vehicle Hardware:** Factory RD4 Radio (Blaupunkt / Continental / Bosch) + Quadlock Part B (Pins 4, 10, 11)

---

## 1. System Architecture & Closed-Loop Audio Management

```mermaid
flowchart TD
    subgraph AndroidHU ["Android Head Unit (com.qf.vehicle)"]
        AudioApp["HU Media (YouTube Music / Spotify)"]
        PhoneApp["Bluetooth Phone Call (HFP)"]
        TunerUI["OriginalTuner UI (FM/AM Screen)"]
        NaviApp["Navigation (Waze / Google Maps)"]
        RouteMgr["PsaAudioRouteManager (mDesiredRd4Source & Focus Arbiter)"]
        KeyMap["PeugeotDataDefine (SWC Vol & Source -> 0x7F NONE)"]
        Parser["PeugeotDataParser (mConfirmedRd4Source Tracker)"]
        UART_TX["UART Transmit (38400 baud)"]
        UART_RX_HU["UART Receive (38400 baud)"]
    end

    subgraph CANBox ["Custom CAN Box (canbox-core)"]
        UART_RX["proto_hiworld_adapter.c"]
        Arbiter["psa_source_arbiter_t (Algorithm 1 State Filter)"]
        Stalk_Inject["Virtual Stalk Pulse: CAN 0x21F (SOURCE)"]
        Periodic_TX["100ms CAN Scheduler (0x162, 0x1A2, 0x1E2, 0x1A0...)"]
        UART_TX_CB["can_router.c (Cmd 0x84 / 0x97 Push Gate)"]
    end

    subgraph CarAudio ["Peugeot RD4 Vehicle System (CAN-CONFORT @ 125 kbps)"]
        RD4["Factory RD4 Radio"]
        Stalk["Physical Steering Stalk & RD4 Front Panel"]
        QuadlockB["Quadlock Part B (Pins 4, 10, 11)"]
        Speakers["Car Amplifier & Speakers"]
    end

    %% Audio & Phone Routes
    AudioApp -->|Media Playback Started| RouteMgr
    PhoneApp -->|Call Active / Ended| RouteMgr
    TunerUI -->|onResume: Check mConfirmedRd4Source| RouteMgr

    %% Downlinks
    RouteMgr -->|5A A5 02 AD 02 01 B1 (Select CDC)| Parser
    RouteMgr -->|5A A5 02 AD 02 00 B0 (CDC Standby)| Parser
    RouteMgr -->|5A A5 02 AD 03 01 B3 (Select Tuner)| Parser
    RouteMgr -->|5A A5 02 AD 03 02 B4 (Select CD)| Parser
    Parser --> UART_TX
    UART_TX --> UART_RX
    UART_RX --> Arbiter

    %% Stalk physical keys
    Stalk -->|Physical SRC Button| RD4
    RD4 -->|CAN 0x165 (actual_rd4_source)| Arbiter

    %% CAN Bus Control
    Arbiter -->|Rule F2: Closed-Loop Pulses| Stalk_Inject
    Arbiter -->|Authentic Blaupunkt CDC Frames| Periodic_TX
    Stalk_Inject -->|CAN 0x21F| RD4
    Periodic_TX -->|CAN 0x162 Byte 5 = 0x06| RD4

    %% Uplinks (Filtered)
    Arbiter -->|Rule F1 & F3: Settled State Only| UART_TX_CB
    UART_TX_CB -->|Cmd 0x84 (Single Settled Source)| UART_RX_HU
    UART_RX_HU --> Parser
    Parser -->|Update mConfirmedRd4Source| RouteMgr

    %% Physical Audio Output
    AudioApp -.->|Analog Line Out| QuadlockB
    PhoneApp -.->|Analog Line Out| QuadlockB
    QuadlockB -->|Un-mutes Part B on CDC| RD4
    RD4 --> Speakers
```

---

## 2. Downlink Protocol Framing (HU $\to$ CAN Box)

```text
[ 0x5A, 0xA5, <LEN>, <CMD>, <SUBCMD>, <PARAM>, <CHECKSUM> ]
```
Where `CHECKSUM = ((LEN + CMD + sum(PAYLOAD)) - 1) & 0xFF`.

| Packet (Hex) | Action | Trigger Event |
|:---|:---|:---|
| `5A A5 02 AD 02 01 B1` | **Switch RD4 to CDC** | Media playback started OR Phone call connected |
| `5A A5 02 AD 02 00 B0` | **CDC Standby** | Media paused OR Tuner UI opened |
| `5A A5 02 AD 03 01 B3` | **Switch RD4 to Tuner** | Tuner UI opened OR Call ended (pre-call source was Tuner) |
| `5A A5 02 AD 03 02 B4` | **Switch RD4 to CD** | Call ended (pre-call source was Internal CD) |
| `5A A5 01 0F 0F` | **Query Radio State & Station List** | Tuner UI opened (`forwardType(0x0F)`) $\to$ Immediately re-sends cached `0x84` (zero CAN cost) & arbitrates thumbwheel scroll for `0x125` list (deferred if seeking) |

---

## 3. Firmware State Filter & Arbiter Architecture (`psa_virtual_cdc.c`)

The CAN box acts as a **state filter and arbiter** between vehicle `CAN 0x165` and serial UART `Cmd 0x84`. Raw `0x165` source oscillations are **never** forwarded directly to UART when an automated seeking or physical debounce transition is in progress.

### 3.1 State Machine Variables

The static context maintains zero dynamic memory allocations (`malloc`, `calloc`, `free` are strictly banned):

```c
typedef enum {
    SRC_SYNC_IDLE = 0,          /* In sync, normal operation */
    SRC_SYNC_SEEKING,           /* Actively pulsing 0x21F to reach HU target */
    SRC_SYNC_WAIT_CONFIRM,      /* Pulse sent, waiting up to 350ms for 0x165 update */
    SRC_SYNC_PHYSICAL_DEBOUNCE  /* User pressed physical SRC button, waiting to settle */
} psa_src_sync_state_t;

typedef struct {
    psa_src_sync_state_t sync_state;
    uint8_t actual_rd4_source;      /* Ground-truth from CAN 0x165 (1=Tuner, 2=CD, 3=CDC, 4=AUX) */
    uint8_t reported_hu_source;     /* Last stable source sent to HU via Cmd 0x84 */
    uint8_t target_source;          /* Target requested by HU Cmd 0xAD (0 if none) */
    uint8_t last_pulse_source;      /* Source at time pulse was injected */
    uint8_t retry_count;            /* Max 5 pulses per request */
    uint32_t state_timer_ms;        /* Timestamp for non-blocking debounce & timeouts */
    bool pending_station_list_trigger; /* Defer 0x0F rotary scroll trigger until Tuner settled */
} psa_source_arbiter_t;
```

### 3.2 Canonical Source Mapping
* `PSA_RD4_SRC_TUNER (0x01)`: CAN `0x165` Byte 2 high nibble `0x1` (`source_mode < 0x20`)
* `PSA_RD4_SRC_CD    (0x02)`: CAN `0x165` Byte 2 high nibble `0x2` (`source_mode == 0x20` or legacy `0x30`)
* `PSA_RD4_SRC_CDC   (0x03)`: CAN `0x165` Byte 2 high nibble `0x3` (`source_mode == 0x31`)
* `PSA_RD4_SRC_AUX   (0x04)`: CAN `0x165` Byte 2 high nibble `0x4` (`source_mode == 0x40`) or `0x5` (`source_mode == 0x41`)
* `PSA_RD4_SRC_OFF   (0x00)`: CAN `0x165` Byte 0 bit 7 `0x00` (`power_status == 0x00`, `source_mode == 0xFF`)

---

### 3.3 Firmware Operating Rules (F1 – F5)

#### Rule F1 — Uplink Mute During Active Seeking (`SRC_SYNC_SEEKING` / `SRC_SYNC_WAIT_CONFIRM`)
* When HU sends `Cmd 0xAD` requesting a target source (`1 = Tuner`, `2 = CD`, `3 = CDC`):
  * If `actual_rd4_source == target_source`: Target is already reached. Immediately emit `Cmd 0x84` and remain in `SRC_SYNC_IDLE`.
  * If `actual_rd4_source != target_source`: Set `target_source`, set `retry_count = 0`, transition to `SRC_SYNC_SEEKING`, inject initial pulse, and advance to `SRC_SYNC_WAIT_CONFIRM`.
* **CRITICAL:** While `sync_state != SRC_SYNC_IDLE`, `can_router.c` **must not** emit UART `Cmd 0x84` or `Cmd 0x97` source packets to the HU. Intermediate cycling frames (e.g. CD briefly showing up while stepping from Tuner to CDC) are completely suppressed.

#### Rule F2 — Closed-Loop Pulse Stepping & Safety Fallback
* Pulses on COM2000 virtual stalk (`CAN 0x21F`) follow:
  1. **Press:** `21F#02<cnt>00` (Byte 0 = `0x02` SRC button, Byte 1 = rolling rotary counter `<cnt>`).
  2. **Hold:** 60 ms duration.
  3. **Release:** `21F#00<cnt>00` (Byte 0 = `0x00` Idle).
* After release, record `last_pulse_source` and enter `SRC_SYNC_WAIT_CONFIRM` for up to **350 ms**.
* Stale `0x165` frames arriving with `new_source == last_pulse_source` are ignored (waiting for RD4 internal processing).
* When a **changed** `0x165` source arrives:
  * If `actual_rd4_source == target_source`: Target reached! Set `reported_hu_source = actual_rd4_source`, clear `target_source = 0`, transition to `SRC_SYNC_IDLE`, and emit confirmed `Cmd 0x84`.
  * If `actual_rd4_source != target_source`: Wait 80 ms settle debounce, increment `retry_count`, and inject the next pulse.
* **Safety Fallback (Missing Disc / Drive Empty):** If `retry_count >= 5` (e.g. HU requested `CD`, but the slot is empty so RD4 skips `CD`), abort seeking, transition to `SRC_SYNC_IDLE`, set `reported_hu_source = actual_rd4_source`, and transmit `Cmd 0x84` with the real vehicle source.

#### Rule F3 — Physical Button Settle Filter (`SRC_SYNC_PHYSICAL_DEBOUNCE`)
* When `sync_state == SRC_SYNC_IDLE` and `CAN 0x165` reports `actual_rd4_source != reported_hu_source`:
  * The driver pressed the physical RD4 front panel `SOURCE` button or the steering wheel stalk button.
  * Do **not** send `Cmd 0x84` immediately. Transition to `SRC_SYNC_PHYSICAL_DEBOUNCE` and arm an **800 ms settle timer**.
  * Every time `actual_rd4_source` changes again (driver rapidly clicking through sources), reset the 800 ms timer.
  * Only after `actual_rd4_source` remains stable continuously for **800 ms**:
    1. Set `reported_hu_source = actual_rd4_source`.
    2. Transition to `SRC_SYNC_IDLE`.
    3. Emit a single, final `Cmd 0x84` to the Android HU.

#### Rule F4 — Latest Command Wins (Preemption)
* If a new `Cmd 0xAD` arrives while already in `SRC_SYNC_SEEKING`, `SRC_SYNC_WAIT_CONFIRM`, or `SRC_SYNC_PHYSICAL_DEBOUNCE`:
  * Overwrite `target_source = new_target`.
  * Reset `retry_count = 0`.
  * If `actual_rd4_source == target_source`: Conclude immediately and emit `Cmd 0x84`.
  * Else: Continue seeking the new target without emitting intermediate `Cmd 0x84` packets.

#### Rule F5 — Cmd 0x0F Station List Arbitration & Immediate Cached State Re-emission
* **Exclusively Gated by Cmd 0x0F:** Station list refresh/request pulses on CAN `0x21F` are **only** generated upon explicit receipt of HU query `5A A5 01 0F 0F`. Switching source to Tuner (via `Cmd 0xAD` or physical front panel / stalk button) does **not** automatically trigger a station list scroll.
* **Immediate 0x84 Emission:** When Android HU sends `Cmd 0x0F` (`5A A5 01 0F 0F`), firmware immediately emits the cached `Cmd 0x84` (current frequency, band, and RDS PS station name) to the serial bus without generating any CAN traffic. Even on cold boot if the protocol cache is uninitialized, the serializer falls back to canonical state cache or zeroed baseline to guarantee immediate response.
* **Deferred 0x21F Rotary Scroll Queue:** In `psa_virtual_cdc_trigger_station_list()`:
  * If the source arbiter is currently busy (`sync_state != SRC_SYNC_IDLE`, e.g. pulsing `SOURCE` on CAN `0x21F` to satisfy a concurrent `Cmd 0xAD`), the thumbwheel scroll is **not** transmitted immediately. Instead, a flag `pending_station_list_trigger = true` is set.
  * Once `CAN 0x165` confirms that the RD4 has settled on Tuner (`rd4_src_nibble == 1`, i.e., `0x10`) and `sync_state` returns to `SRC_SYNC_IDLE`, the firmware automatically executes the Scroll UP (`cnt + 1`) $\to$ 80 ms $\to$ Scroll DOWN (`cnt - 1`) sequence cleanly.
  * If seeking aborted or settled on any non-Tuner source (e.g., missing CD fallback), `pending_station_list_trigger` is cleared without emitting scroll frames.

---

## 4. Android Head Unit Architecture (`com.qf.vehicle`)

### 4.1 State Tracking Variables
In `PsaAudioRouteManager` / `PeugeotDataParser`:
* `mConfirmedRd4Source`: The authoritative source currently active in the vehicle (updated strictly by incoming `Cmd 0x84`).
* `mDesiredRd4Source`: The pending target source requested by Android (set when sending `Cmd 0xAD`, cleared when confirmed or timed out).

### 4.2 Uplink Handling (`Cmd 0x84` Ingestion)
When `PeugeotDataParser` receives `Cmd 0x84`:
```java
int newSource = packet.getRd4Source(); // 1=Tuner, 2=CD, 3=CDC, 4=AUX
mConfirmedRd4Source = newSource;

if (mDesiredRd4Source != null) {
    if (newSource == mDesiredRd4Source) {
        // Target reached successfully!
        mDesiredRd4Source = null;
    } else {
        // Target aborted by firmware fallback (e.g. CD slot empty)
        mDesiredRd4Source = null;
    }
} else {
    // Passive switch: Driver operated physical RD4 / Stalk button
    handlePassiveSourceChange(newSource);
}
```

### 4.3 Passive Source Change Actions (`mDesiredRd4Source == null`)
Because `canbox-core` already debounces physical button presses for 800 ms, `newSource` represents the settled driver intent:

1. **Passively Switched to Tuner (`newSource == SOURCE_TUNER`):**
   * If YouTube Music or media is active, request `AUDIOFOCUS_LOSS_TRANSIENT` or dispatch `KEYCODE_MEDIA_PAUSE`.
   * **Do not** force full-screen `OriginalTuner` takeover over active navigation (Waze / Google Maps). Show a non-intrusive floating toast/HUD banner or only bring Tuner UI to foreground if the current foreground app is an OEM media activity or the Home Launcher.
   * `OriginalTuner.onResume()` checks:
     ```java
     if (mConfirmedRd4Source == SOURCE_TUNER) {
         // Radio is ALREADY active! Do NOT send redundant 0xAD 03 01!
         return;
     }
     ```
     This completely eliminates the ping-pong feedback loop.

2. **Passively Switched to CD (`newSource == SOURCE_CD`):**
   * Request media pause (`KEYCODE_MEDIA_PAUSE`).
   * Optionally display track banner or bring `OriginalMediaPlayer` to foreground if permitted by the navigation safety policy.

3. **Passively Switched to CDC (`newSource == SOURCE_CDC`):**
   * Do not open Tuner or CD UI.
   * Dispatch `KEYCODE_MEDIA_PLAY` or abandon AudioFocus transient loss so YouTube Music automatically resumes playing over Quadlock Part B.

---

## 5. Summary of Common Edge Cases & Protections

| Scenario | Risk Without State Filter | Closed-Loop Resolution |
|:---|:---|:---|
| **Tuner $\to$ CDC Transition** | RD4 temporarily enters CD mode for 200 ms; HU opens CD player and pauses media. | Rule F1 suppresses intermediate `0x84` frames; HU only receives `0x84 CDC` when locked. |
| **Rapid Stalk Multi-Click** | Driver clicks SRC 3 times in 500 ms; HU launches and dismisses 3 different activities. | Rule F3 800 ms settle timer resets on each click; only the final source is dispatched. |
| **Missing CD in Slot** | Radio cannot enter CD mode; blind pulsing loops infinitely. | Rule F2 safety fallback caps retries at 5 and returns true vehicle source. |
| **Driver Operates Physical Stalk** | Android `onResume()` sends `Cmd 0xAD` back to vehicle, causing infinite oscillation. | Android checks `mConfirmedRd4Source == newSource` before issuing `Cmd 0xAD`. |
| **Active Navigation (Waze/Maps)** | Physical source change abruptly pushes navigation off-screen. | HUD/Toast banner policy prevents stealing window focus from navigation apps. |
| **Cmd 0x0F Arrives During Source Switch (0xAD)** | 0x21F rotary scroll frames collide with 0x21F SOURCE button pulses; RD4 scrolls while in CD/CDC without loading tuner list. | Rule F5 emits cached 0x84 immediately and defers 0x21F rotary scroll until RD4 settles on Tuner (`sync_state == SRC_SYNC_IDLE`). |
