# Project Context: OpenCanbox Core

OpenCanbox Core is a portable C99 firmware project designed to translate automotive CAN bus frames (Comfort/Body CAN) into UART packets consumed by Chinese Android head units (Raise, Hiworld, Bagoo, Simple Soft).

---

## Architecture Topology


```

[Vehicle CAN Bus]
│ (125k / 250k / 500k)
▼
[hal_can (Hardware Driver)] ──► [ring_buffer_t (Lock-Free SPSC)]
│
▼
[can_router_process_can()]
│
▼
[vehicle_profile_manager]
(Matches CAN ID -> profile decoder)
│
▼
[Normalized vehicle_state_t]
(Doors, SWC, Climate, TPMS, Speed)
│
▼
[Delta / Periodic Filter]
│
▼
[hu_protocol_driver (vtable)]
(Active serializer: Raise / Hiworld)
│
▼
[hal_uart (Hardware Driver)] ──► [Android Head Unit]

```

---

## Hardware Abstraction Layer (HAL) Contracts

Hardware access is decoupled using clean C APIs defined in `include/hal/`:

* `hal_can.h`: Non-blocking frame push/pop (`hal_can_send`, `hal_can_receive`), filter initialization, and baud rate configuration.
* `hal_uart.h`: Non-blocking byte-stream read/write (`hal_uart_read_byte`, `hal_uart_write`). Standard Android HU CAN baud rate is **38400 baud, 8N1**.
* `hal_system.h`: Monotonic millisecond counter (`hal_get_tick_ms`), busy delay (`hal_delay_ms`), and soft reboot (`hal_system_reboot`).
* `hal_nvs.h`: Non-volatile storage abstraction for preserving active car selection across ignition cycles.

---

## Protocol Wire Specifications

### 1. Raise Protocol
* **Framing:** `[0x2E] [Cmd] [Len] [Payload(0..N)] [Checksum]`
* **Length:** Count of payload bytes ($N$).
* **Checksum:** Bitwise NOT of 8-bit sum:
  $$\text{Checksum} = (\sim(\text{Cmd} + \text{Len} + \sum \text{Payload})) \ \& \ \text{0xFF}$$
* **Key Commands:**
  * `0x01`: Steering Wheel Control (SWC) keys
  * `0x24`: Door and trunk contacts
  * `0x29`: Vehicle telemetry (Speed, RPM, Steering angle)
  * `0x38`: Extended 10-byte TPMS data
  * `0xCA`: Car model selection (from Android HU)

### 2. Hiworld Protocol
* **Framing:** `[0x5A] [0xA5] [Len] [Cmd] [Payload(0..N)] [Checksum]`
* **Length:** Command byte + Payload byte count ($N + 1$).
* **Checksum:** Truncated 8-bit sum:
  $$\text{Checksum} = (\text{Len} + \text{Cmd} + \sum \text{Payload}) \ \& \ \text{0xFF}$$
* **Key Commands:**
  * `0x11`: Steering Wheel Control keys
  * `0x21`: Door status
  * `0x24`: Car type selection (from Android HU)
  * `0x26`: Steering angle track

---

## Active Memory Model & Normalized State

The canonical vehicle state lives inside a single static structure (`vehicle_state_t` in `src/core/can_router.c`):

```c
typedef struct {
    vehicle_doors_t   doors;
    vehicle_wheel_t   wheel;
    vehicle_climate_t climate;
    vehicle_tpms_t    tpms;
    uint16_t          speed_kmh;
    uint16_t          rpm;
    int16_t           steering_angle_deg;
    bool              reverse_gear;
    bool              handbrake;
} vehicle_state_t;

```

* High-priority events (steering wheel keys, door openings) are transmitted immediately upon state delta detection.
* Low-priority telemetry (speed, RPM, TPMS) is refreshed on a 100 ms periodic timer or throttled to avoid saturating the 38400 baud UART link.
