#!/usr/bin/env python3
"""
Hiworld Protocol Doors Serial Emulator
======================================
Sends Left Front (FL) and Left Rear (RL) doors open status packets to an
Android Head Unit over serial interface (/dev/ttyUSB1 @ 38400 baud by default).

Hiworld 0x12 Door Status Framing:
  Header: 0x5A 0xA5
  Length: 0x03 (3 payload bytes)
  Cmd ID: 0x12
  Payload: [0x00, 0x00, 0xA0]
    - Byte 0..1: 0x00 0x00 (Reserved)
    - Byte 2:
        Bit 7 (0x80): Front Left / Driver Door (OPEN)
        Bit 5 (0x20): Rear Left Door           (OPEN)
        -> 0x80 | 0x20 = 0xA0
  Checksum: ((Len + Cmd + sum(Payload)) - 1) & 0xFF
            = (0x03 + 0x12 + 0x00 + 0x00 + 0xA0 - 1) & 0xFF = 0xB4
  Total Frame: 5A A5 03 12 00 00 A0 B4
"""

import argparse
import sys
import time
import signal

try:
    import serial
except ImportError:
    print("[ERROR] 'pyserial' package is required.")
    print("        Install it with: pip install pyserial")
    sys.exit(1)


HIWORLD_SOF1 = 0x5A
HIWORLD_SOF2 = 0xA5

CMD_CAR_BASE_INFO    = 0x11
CMD_DOOR_WINDOW      = 0x12
CMD_FEATURE_ENABLE1  = 0x71
CMD_FEATURE_ENABLE2  = 0x72
CMD_VERSION_REPORT   = 0xF0
CMD_HEARTBEAT        = 0xFF


def build_hiworld_packet(cmd: int, payload: list or bytes or bytearray) -> bytes:
    """Builds Hiworld packet with additive sum - 1 checksum."""
    p_bytes = bytes(payload)
    length = len(p_bytes)
    total_sum = length + cmd + sum(p_bytes)
    checksum = (total_sum - 1) & 0xFF
    return bytes([HIWORLD_SOF1, HIWORLD_SOF2, length, cmd]) + p_bytes + bytes([checksum])


def main():
    parser = argparse.ArgumentParser(
        description="Hiworld Serial Protocol Door Status Emulator (FL & RL Open)"
    )
    parser.add_argument(
        "--port", "-p",
        default="/dev/ttyUSB1",
        help="Serial port path (default: /dev/ttyUSB1)"
    )
    parser.add_argument(
        "--baud", "-b",
        type=int,
        default=38400,
        help="Baud rate (default: 38400)"
    )
    parser.add_argument(
        "--rate", "-r",
        type=float,
        default=5.0,
        help="Broadcast frequency in Hz (default: 5.0 Hz / 200ms interval)"
    )
    parser.add_argument(
        "--doors",
        choices=["fl_rl", "fl", "rl", "all_open", "all_closed"],
        default="fl_rl",
        help="Doors state to transmit (default: fl_rl)"
    )
    parser.add_argument(
        "--mode",
        choices=["10byte_real", "3byte", "1byte_hi", "1byte_low", "all_variants"],
        default="10byte_real",
        help="Packet format variant to send (default: 10byte_real matching real PSA Hiworld canbox)"
    )
    parser.add_argument(
        "--no-handshake",
        action="store_true",
        help="Skip initial version report and feature enable handshake"
    )
    args = parser.parse_args()

    print("=" * 72)
    print("      HIWORLD PROTOCOL DOORS STATUS SERIAL EMULATOR")
    print("=" * 72)
    print(f"Target Port    : {args.port}")
    print(f"Baud Rate      : {args.baud} 8N1")
    print(f"Broadcast Rate : {args.rate} Hz ({1000/args.rate:.0f} ms interval)")
    print(f"Door Selection : {args.doors.upper()}")
    print(f"Frame Format   : {args.mode}")
    print("=" * 72)

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )
        print(f"[OK] Opened serial device: {args.port}")
    except Exception as e:
        print(f"[ERROR] Failed to open {args.port}: {e}")
        print("Tip: Check device path (e.g. /dev/ttyUSB0 or /dev/ttyUSB1) and permissions.")
        sys.exit(1)

    # 1. Send Handshake
    if not args.no_handshake:
        print("\n[INFO] Sending initial handshake packets...")
        pkt_version = build_hiworld_packet(CMD_VERSION_REPORT, b"HW_PSA_V2.04.01")
        pkt_feat1   = build_hiworld_packet(CMD_FEATURE_ENABLE1, [0xFF, 0xFF])
        pkt_feat2   = build_hiworld_packet(CMD_FEATURE_ENABLE2, [0xFB, 0xFF])

        for name, pkt in [("Version Report (0xF0)", pkt_version),
                          ("Feature Enable 1 (0x71)", pkt_feat1),
                          ("Feature Enable 2 (0x72)", pkt_feat2)]:
            ser.write(pkt)
            ser.flush()
            print(f"  -> Sent {name:25s}: {pkt.hex(' ')}")
            time.sleep(0.05)

    # 2. Door Bitmask calculation
    # Standard Bit mapping (Byte 2):
    # Bit 7: FL (Driver) (0x80)
    # Bit 6: FR (Passenger) (0x40)
    # Bit 5: RL (0x20)
    # Bit 4: RR (0x10)
    # Bit 3: Trunk (0x08)
    # Bit 2: Hood / Base Active Flag (0x04)
    if args.doors == "fl_rl":
        mask_std = 0x80 | 0x20  # 0xA0
        mask_low = 0x01 | 0x04  # 0x05 (if low-bit indexed)
    elif args.doors == "fl":
        mask_std = 0x80
        mask_low = 0x01
    elif args.doors == "rl":
        mask_std = 0x20
        mask_low = 0x04
    elif args.doors == "all_open":
        mask_std = 0xFC
        mask_low = 0x3F
    else:  # all_closed
        mask_std = 0x00
        mask_low = 0x00

    # 10-byte real canbox payload: [0x00, 0x04, mask_std | 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03]
    b2_real = mask_std | 0x04
    payload_10byte = [0x00, 0x04, b2_real, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03]
    pkt_10byte    = build_hiworld_packet(CMD_DOOR_WINDOW, payload_10byte)
    pkt_3byte     = build_hiworld_packet(CMD_DOOR_WINDOW, [0x00, 0x00, mask_std])
    pkt_1byte_hi  = build_hiworld_packet(CMD_DOOR_WINDOW, [mask_std])
    pkt_1byte_low = build_hiworld_packet(CMD_DOOR_WINDOW, [mask_low])
    pkt_heartbeat = build_hiworld_packet(CMD_HEARTBEAT, [0x01])

    print(f"\n[INFO] Generated Door Status Packets:")
    print(f"  - Real 10-byte PSA Canbox Frame (Cmd 0x12, len 10): {pkt_10byte.hex(' ')}")
    print(f"  - Standard 3-byte (Cmd 0x12, payload 00 00 {mask_std:02X}):        {pkt_3byte.hex(' ')}")
    print(f"  - Heartbeat Keep-Alive (Cmd 0xFF):                             {pkt_heartbeat.hex(' ')}")

    interval = 1.0 / args.rate
    print(f"\n[INFO] Transmitting periodically every {interval*1000:.0f} ms. Press Ctrl+C to stop.\n")

    running = True

    def signal_handler(sig, frame):
        nonlocal running
        print("\n[INFO] Stopping transmission...")
        running = False

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    count = 0
    try:
        while running:
            if args.mode == "10byte_real":
                ser.write(pkt_10byte)
            elif args.mode == "3byte":
                ser.write(pkt_3byte)
            elif args.mode == "1byte_hi":
                ser.write(pkt_1byte_hi)
            elif args.mode == "1byte_low":
                ser.write(pkt_1byte_low)
            elif args.mode == "all_variants":
                ser.write(pkt_10byte)
                ser.write(pkt_3byte)

            # Send heartbeat every ~1 second
            if count % int(max(1, args.rate)) == 0:
                ser.write(pkt_heartbeat)

            ser.flush()

            # Read any responses from HU
            if ser.in_waiting > 0:
                rx = ser.read(ser.in_waiting)
                print(f"[HU Response] << {rx.hex(' ')}")

            count += 1
            time.sleep(interval)

    finally:
        ser.close()
        print("[OK] Serial port closed.")


if __name__ == "__main__":
    main()

