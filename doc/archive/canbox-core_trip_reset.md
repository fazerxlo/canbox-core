### 1. How Trip Reset Works via Android Head Unit (Downlink)

```
[Android Head Unit]
       │ UART Downlink: Cmd 0x1B (Reset Trip 1 or Trip 2)
       ▼
[OpenCanbox Core]
       │ 1. proto_hiworld_adapter parses packet
       │ 2. psa_trip_send_reset() builds CAN 0x167 with reset bit
       │ 3. 300 ms sustained pulse via psa_car_config_periodic_100ms()
       ▼
[CAN Bus 0x167] ──► [BSI / Engine ECU] (Resets internal trip counters)
                           │
                           ▼
[CAN Bus 0x2A1 / 0x261] ──► [OpenCanbox] ──► [HU UART: Cmd 0x14 / 0x15] (Zeros displayed)
```

1. **Touchscreen Event (Android HU $\rightarrow$ Canbox):**
   The Android HU trip app transmits a Hiworld Downlink packet [`Cmd 0x1B`](file:///home/Fazer/git/canbox-core/doc/CANBOX_SPEC_HIWORLD_407_06_TRIP_COMPUTER_TELEMETRY.md#L105-L120):
   * Reset Trip 1: `5A A5 04 1B 02 02 01 FF 22` (`payload[1] == 0x02`)
   * Reset Trip 2: `5A A5 04 1B 03 03 01 FF 24` (`payload[1] == 0x03`)
2. **Translation to CAN Bus Frame `0x167`:**
   In [`proto_hiworld_adapter.c`](file:///home/Fazer/git/canbox-core/src/protocols/proto_hiworld_adapter.c#L66-L86) and [`psa_trip_send_reset()`](file:///home/Fazer/git/canbox-core/src/profiles/psa_407_telemetry.c#L183-L217), OpenCanbox prepares CAN ID **`0x167`** (`COMMANDES_EMF`):
   * **Trip 1 Reset:** `8A 06 FF FF 00 00 00 00`  
     * `0x8A` = Page Trip 1 (`0x0A`) ORed with Trip 1 Reset bit (`0x80`).
   * **Trip 2 Reset:** `4C 06 FF FF 00 00 00 00`  
     * `0x4C` = Page Trip 2 (`0x0C`) ORed with Trip 2 Reset bit (`0x40`).
3. **Timed Reset Pulse:**
   * OpenCanbox immediately transmits this frame onto the CAN bus and sets `s_mdf_reset_ticks = 3` ([`psa_407_car_config.c`](file:///home/Fazer/git/canbox-core/src/profiles/psa_407_car_config.c#L384-L396)).
   * In the 100 ms periodic loop, the reset bit is held for **300 ms**, then cleared back to idle (`0x0A` or `0x0C`).
4. **BSI Response & Uplink Update:**
   * The BSI detects the reset pulse on CAN `0x167`, resets trip distance, average speed, and average consumption counters.
   * The BSI broadcasts updated zeroed counters on **`0x2A1`** (Trip 1) and **`0x261`** (Trip 2).
   * OpenCanbox decodes these frames and emits Hiworld Uplink telemetry packets [`Cmd 0x14`](file:///home/Fazer/git/canbox-core/doc/CANBOX_SPEC_HIWORLD_407_06_TRIP_COMPUTER_TELEMETRY.md#L79-L90) / [`Cmd 0x15`](file:///home/Fazer/git/canbox-core/doc/CANBOX_SPEC_HIWORLD_407_06_TRIP_COMPUTER_TELEMETRY.md#L92-L103) so the Android screen immediately refreshes with `0 km`, `0.0 L/100km`, `-- km/h`.

---

### 2. What is Emulated so the Cluster Matrix Sees Trip Pages

In OEM vehicles, the central instrument cluster (COMBINE matrix display between dials) listens to the Multifunction Display (EMF) to know what to render.

When the physical EMF is removed, OpenCanbox checks if physical EMF diagnostic frame `0x525` is absent via [`psa_car_config_is_mdf_present()`](file:///home/Fazer/git/canbox-core/src/profiles/psa_407_car_config.c#L113-L116) and activates **Virtual EMF emulation** with periodic CAN broadcasts:

| Emulated CAN ID | Rate | Payload | Purpose for Cluster Matrix |
| :--- | :--- | :--- | :--- |
| **`0x167`** | 1000 ms (or on page change) | `[Page_Mask] 06 FF FF 00 00 00 00` | Tells the cluster matrix which trip page to show:<br>• `0x09`: Instantaneous (range, instant consumption)<br>• `0x0A`: Trip 1 (distance, avg consumption, avg speed)<br>• `0x0C`: Trip 2 |
| **`0x0DF`** | 500 ms | `10 00 50` | EMF presence / display status heartbeat. Without this, cluster and BSI log EMF missing faults. |
| **`0x3F6`** | 1000 ms | `00 00 [ctr++] 00 00 C0 01` | EMF trip clock / display counter keepalive. |
| **`0x525`** | 1000 ms | `05 00 01 00 00 00 00 00` | EMF diagnostic identification heartbeat. |

Because `0x167` is continuously broadcast by OpenCanbox with the active `s_mdf_trip_page_mask`, the cluster matrix displays the corresponding trip computer view.

---

### 3. How the Stalk SWC Button Cycles Pages & Triggers Resets

In Peugeot 407, the trip button is the **push-button on the tip of the wiper stalk** (COM2000 / COM2005):

```
       [Wiper Stalk Tip Button Pressed]
                     │
                     ▼
             [CAN Bus ID 0x221]
           (Byte 0 Bit 3: 0x08 == 1)
                     │
                     ▼
           [psa_trip_decode_frame()]
          /                         \
  Short Press (< 2s)          Long Press (>= 2s)
         │                            │
  psa_car_config_cycle_trip_page()    psa_trip_send_reset(active_trip)
         │                            │
  • Instant -> Trip 1 -> Trip 2       • Injects 0x8A (Trip 1) or 0x4C (Trip 2)
  • Transmits 0x167 with new page     • 300 ms pulse to BSI
  • Starts 7s auto-revert timer
```

Detailed in [`psa_407_telemetry.c` lines 124–160](file:///home/Fazer/git/canbox-core/src/profiles/psa_407_telemetry.c#L124-L160):

1. **Button Detection (`0x221`):**
   * Stalk button state is carried in `0x221` Byte 0 Bit 3: `(data[0] & 0x08) != 0`.
2. **Short Press (Falling edge under 2 seconds):**
   * Calls [`psa_car_config_cycle_trip_page()`](file:///home/Fazer/git/canbox-core/src/profiles/psa_407_car_config.c#L398-L411).
   * Cycles `s_mdf_trip_page_mask`: `Instant (0x09)` $\rightarrow$ `Trip 1 (0x0A)` $\rightarrow$ `Trip 2 (0x0C)` $\rightarrow$ `Instant (0x09)`.
   * Immediately transmits the updated CAN `0x167` so the cluster matrix switches pages in real time.
   * Starts a **7-second inactivity timer** (`s_mdf_trip_inactivity_ticks = 70`). If no further button press occurs within 7 seconds, it auto-reverts back to `Instant (0x09)` and sends `0x167` (matching OEM Peugeot factory behavior).
   * Flags `dirty.trip_page` to keep the Android HU synchronized with the stalk selection.
3. **Long Press (Held $\ge 2000\text{ ms}$):**
   * Detects sustained hold while the button is active.
   * Checks current active page mask:
     * If currently on **Trip 1 (`0x0A`)**: calls `psa_trip_send_reset(1)`, injecting `0x167` with `0x8A`.
     * If currently on **Trip 2 (`0x0C`)**: calls `psa_trip_send_reset(2)`, injecting `0x167` with `0x4C`.
     * If currently on **Instant (`0x09`)**: does nothing (instantaneous trip has no counters to reset).
   * The BSI receives the reset bit from `0x167`, clears the counters, and both the cluster matrix and Android screen show the reset values.