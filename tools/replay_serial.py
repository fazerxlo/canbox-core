#!/usr/bin/env python3
"""
OpenCanbox Core - Serial Packet Replayer for Head Units
======================================================
Replays raw binary or hex-dumped CANBox serial telemetry packets to a connected
USB-Serial adapter (/dev/ttyUSB*, /dev/ttyACM*) interfacing with an Android Head Unit.
"""

import argparse
import glob
import os
import sys
import time

try:
    import serial
except ImportError:
    print("[ERROR] pyserial is required. Install via: pip install pyserial")
    sys.exit(1)


def find_serial_ports():
    """Scans system for connected USB-to-UART serial adapters."""
    return sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))


def parse_packets(raw_bytes):
    """Splits raw stream of CANbox bytes into individual frames based on 0xFD framing."""
    packets = []
    i = 0
    while i < len(raw_bytes):
        if raw_bytes[i] == 0xFD:
            if i + 1 >= len(raw_bytes):
                break
            length = raw_bytes[i + 1]
            total_len = length + 1  # 0xFD (1) + length bytes
            if i + total_len <= len(raw_bytes):
                packet = raw_bytes[i : i + total_len]
                packets.append(packet)
                i += total_len
            else:
                # Incomplete packet at the end
                packets.append(raw_bytes[i:])
                break
        else:
            i += 1
    return packets


def main():
    parser = argparse.ArgumentParser(
        description="Replay CANBox serial frames to USB-Serial port connected to HU."
    )
    parser.add_argument(
        "file",
        nargs="?",
        default="test_data/dump.bin",
        help="Path to binary (.bin) or hex text (.txt) file to replay (default: test_data/dump.bin)",
    )
    parser.add_argument(
        "--port",
        "-p",
        default=None,
        help="Serial port (e.g., /dev/ttyUSB0). If omitted, auto-discovery is performed.",
    )
    parser.add_argument(
        "--baud",
        "-b",
        type=int,
        default=19200,
        help="Serial baud rate (default: 19200 for Peugeot 407 Raise/RZC)",
    )
    parser.add_argument(
        "--packet-delay",
        "-d",
        type=float,
        default=0.05,
        help="Delay in seconds between individual packets (default: 0.05s / 50ms)",
    )
    parser.add_argument(
        "--loop",
        "-l",
        action="store_true",
        help="Loop replay indefinitely",
    )
    parser.add_argument(
        "--loop-delay",
        type=float,
        default=1.0,
        help="Delay in seconds between loops when --loop is active (default: 1.0s)",
    )
    parser.add_argument(
        "--raw-stream",
        action="store_true",
        help="Send raw binary file at once instead of packet-by-packet",
    )

    args = parser.parse_args()

    # Load data
    if not os.path.isfile(args.file):
        print(f"[ERROR] File not found: {args.file}")
        sys.exit(1)

    with open(args.file, "rb") as f:
        content = f.read()

    # Check if file is hex text or raw binary
    if args.file.endswith(".txt") or b"0x" in content[:20]:
        import re
        tokens = re.findall(r"0x[0-9A-Fa-f]+", content.decode("utf-8", errors="ignore"))
        raw_bytes = bytes([int(x, 16) for x in tokens])
    else:
        raw_bytes = content

    print(f"[*] Loaded {len(raw_bytes)} bytes from '{args.file}'")

    # Select Port
    port = args.port
    if not port:
        ports = find_serial_ports()
        if ports:
            port = ports[0]
            print(f"[INFO] Auto-detected serial port: {port}")
        else:
            print("[ERROR] No USB serial ports (/dev/ttyUSB* or /dev/ttyACM*) found.")
            print("        Please specify manually with --port <device>")
            sys.exit(1)

    # Open Serial Connection
    try:
        ser = serial.Serial(port=port, baudrate=args.baud, timeout=1.0)
        print(f"[OK] Opened {port} @ {args.baud} 8N1")
    except Exception as e:
        print(f"[ERROR] Failed to open serial port {port}: {e}")
        sys.exit(1)

    packets = parse_packets(raw_bytes)
    if not packets or args.raw_stream:
        print(f"[*] Streaming mode: raw binary stream ({len(raw_bytes)} bytes)")
    else:
        print(f"[*] Packet mode: {len(packets)} frames parsed from stream")

    try:
        loop_cnt = 0
        while True:
            loop_cnt += 1
            print(f"\n--- [Replay Loop #{loop_cnt}] ---")

            if packets and not args.raw_stream:
                for idx, pkt in enumerate(packets, 1):
                    ser.write(pkt)
                    ser.flush()
                    print(f"[{idx:02d}/{len(packets):02d}] Sent: {pkt.hex(' ')}")
                    if args.packet_delay > 0:
                        time.sleep(args.packet_delay)
            else:
                ser.write(raw_bytes)
                ser.flush()
                print(f"[OK] Sent {len(raw_bytes)} bytes")

            if not args.loop:
                print("\n[OK] Single replay complete.")
                break

            time.sleep(args.loop_delay)

    except KeyboardInterrupt:
        print("\n[*] Replay stopped by user.")
    finally:
        ser.close()
        print("[*] Serial port closed.")


if __name__ == "__main__":
    main()

