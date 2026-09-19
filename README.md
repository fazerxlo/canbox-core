# OpenCanbox Core

Portable, pure C99 firmware for custom CAN box adapters interfacing vehicle CAN networks with Chinese Android head units (Raise, Hiworld, Bagoo, Simple Soft).

## Project Structure
canbox-core/
├── platformio.ini
├── include/
│   ├── core/
│   ├── hal/
│   └── protocols/
├── src/
│   ├── core/
│   ├── profiles/
│   ├── protocols/
│   └── hal/
└── test/
    ├── test_protocol_parser/
    └── test_integration/
