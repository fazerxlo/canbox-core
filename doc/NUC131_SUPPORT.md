# NUV131 / NUC131 Support Notes

This document captures the practical integration points for the Nuvoton NUC131 target in `canbox-core`.

## Reference sources

- `fazerxlo/nuvoton_nuc131` — PlatformIO platform and chip BSP
- `fazerxlo/canbox/vw_nc03/fw` — working NUV131 firmware implementation (CAN + UART + GPIO mapping)

## Hardware mapping

The reference firmware uses these pins:

- CAN0 RX: PD.6
- CAN0 TX: PD.7
- UART0 RX: PB.0
- UART0 TX: PB.1
- GPIO outputs: PA.8, PA.9, PA.12, PA.13

The generic `canbox-core` HAL interface maps these to the same logical functions:

- `GPIO_PIN_LED_STATUS`
- `GPIO_PIN_CAN_STBY`
- `GPIO_PIN_HEADUNIT_POWER`
- `GPIO_PIN_REVERSE_OUT`
- `GPIO_PIN_IGNITION_IN`

## PlatformIO setup

Use the custom platform from `fazerxlo/nuvoton_nuc131`:

```ini
[env:nuc131_cbox]
platform = nuvoton
board = nuvoton_nuc131
test_ignore = *
build_flags = 
    -Iinclude
    -Isrc
    -DPLATFORM_NUC131
    -Wall -Wextra -std=c99
build_src_filter = 
    +<main.c>
    +<core/*>
    +<profiles/*>
    +<protocols/*>
    +<hal/hal_nuc131/*>
```

## Build prerequisites

1. Clone the custom platform repo:

```bash
git clone https://github.com/fazerxlo/nuvoton_nuc131.git
```

2. Place it under your PlatformIO platforms directory:

```bash
~/.platformio/platforms/
```

3. Build the firmware:

```bash
pio run -e nuc131_cbox
```

## Flashing

The custom platform repo explicitly requires a custom uploader package, `tool-nuvoton-isp`, or an equivalent OpenOCD-based method. See `fazerxlo/nuvoton_nuc131` for the exact upload tool configuration.

## Implementation notes

- The NUC131 backend follows the same lifecycle as the STM32 / ESP32 HALs.
- Core logic remains portable and should not depend on Nuvoton-specific types.
- HAL implementations isolate vendor-specific register access and pin configuration.

## Current status

The repo contains a working HAL scaffold for the NUC131 target and the matching PlatformIO environment to compile it. Actual flashing and on-device validation depend on an installed Nuvoton uploader or OpenOCD toolchain.
