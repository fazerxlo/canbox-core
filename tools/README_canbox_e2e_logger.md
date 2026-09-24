# OpenCanbox E2E Test & Logger (`canbox_e2e_logger.py`)

This tool is a comprehensive Hardware-in-the-Loop (HIL) testing and logging harness for the OpenCanbox Core project. 

It compiles the native desktop simulator and bridges it directly to your physical USB adapters to provide a real-time translating layer between a CAN bus and a Head Unit (HU). While bridging the traffic, it runs parallel diagnostic listeners to capture and log the precise interactions between all hardware components.

## Features

- **Automated `slcan` Setup:** Automatically attaches and brings up a SocketCAN interface (`can0`) using a standard USB-to-UART serial adapter.
- **Transparent UART Proxying (PTY):** The script creates a virtual pseudoterminal (`/dev/pts/X`) for the OpenCanbox app to connect to, intercepting all hexadecimal bytes sent and received to the physical Head Unit adapter.
- **Native SocketCAN Sniffing:** Sniffs the physical CAN interface independently of the application output.
- **Original Canbox Reference:** Optionally allows you to run a parallel logger against an original OEM Canbox to cross-reference behavior.
- **Interactive Device Menu:** Intelligently discovers available `/dev/ttyUSB*` and `/dev/ttyACM*` adapters with smart, auto-excluding defaults.
- **Time-Travel Dump (30s Lookback):** Records all parallel data streams (`[CAN]`, `[APP_UART_TX]`, `[APP_UART_RX]`, `[APP]`, and `[ORIG_CANBOX]`) into a temporary ring buffer. Pressing `L` instantly dumps the last 30 seconds of this data into a persistent timestamped file (e.g., `dump_2026-09-24_12-25-30.log`).

## Requirements

The script is intended to be run natively on Linux.

- **can-utils**: Required for `slcand` and `slcan_attach`.
- **Python 3**:
  - `pyserial` (`pip install pyserial`)
- **PlatformIO**: Used to compile the `native_test` C binary on the fly.

## Usage

Make sure the script is executable:
```bash
chmod +x tools/canbox_e2e_logger.py
```

Run the logger:
```bash
./tools/canbox_e2e_logger.py
```

### Interface Configuration

When the script starts, it will ask you to map your connected USB adapters:
1. **CAN Adapter:** The adapter connected to the vehicle bench (slcan).
2. **Head Unit Adapter:** The adapter connected to the Android Head Unit.
3. **Original Canbox (Optional):** The adapter connected to a reference OEM Canbox.

*Tip: You can just press `Enter` to accept the default available port or hit `c` to enter a custom path manually.*

### Runtime Controls

Once running, the console will display a live-updating statistics bar showing traffic volume across all active interfaces.

- **`L`**: Extract the last 30 seconds of logs and save them to a `dump_YYYY-MM-DD_HH-MM-SS.log` file in the current directory.
- **`q` or `Esc`**: Safely tear down the PTY proxies, terminate the native application, securely delete the temporary log files, and exit.

## Log Format Example

When a dump is requested, the resulting log will merge all events synchronously:
```
1790245082.340 [APP] Physical serial device opened: /dev/pts/12 (Baud: 38400)
1790245082.500 [CAN] ID:036 DLC:8 DATA:0000000000000000
1790245082.505 [APP_UART_TX] 55a501020000
1790245082.600 [APP_UART_RX] 55a502020000
1790245082.650 [ORIG_CANBOX] RX: 55a501020000
```

