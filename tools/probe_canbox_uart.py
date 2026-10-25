#!/usr/bin/env python3
"""
Canbox UART Baud Rate & Protocol Auto-Detection Tool
===================================================
Probes connected USB-UART serial adapters to discover:
1. Baud Rate (38,400, 19,200, 9,600, 57,600, 115,200)
2. Protocol Dialect (Hiworld 0x5A 0xA5, Raise 0x2E, SimpleSoft 0xFD, Bagoo 0xD5)
3. Active Telemetry Stream & Frame Integrity (Checksum verification)

Safely broadcasts PSA Comfort CAN wake-up keep-alives on can0 in the background
to prevent OEM canboxes from entering low-power sleep mode during probing.
"""

import argparse
import glob
import os
import re
import socket
import struct
import subprocess
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

try:
    import serial
except ImportError:
    print("[ERROR] pyserial is required. Run 'pip install pyserial'.")
    sys.exit(1)

CAN_FRAME_FMT = "=IB3x8s"

# Standard PSA CAN2004 Keep-Alive / Wake-up Frames
WAKE_FRAMES = [
    (0x036, bytes([0x0E, 0x00, 0x00, 0x2A, 0x01, 0x00, 0x00, 0xA0])),  # Ignition ON (Phase de vie = 0x01 Awake)
    (0x0F6, bytes([0x88, 0x00, 0x00, 0x00, 0x00, 0x50, 0x00, 0x20])),  # Economy OFF (+APC / Ignition ON, 24°C)
    (0x128, bytes([0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])),  # Key in ignition / Contact ON
    (0x15B, bytes([0x01, 0x03, 0xB1, 0x40, 0x00, 0xD0, 0x00, 0x10])),  # BSI Power state
    (0x168, bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])),  # Body / cluster status
    (0x1D0, bytes([0x08, 0x80, 0x07, 0x00, 0x00, 0x0B, 0x0B])),  # HVAC Panel State (Dual-zone, 21.0°C)
    (0x1E3, bytes([0x1C, 0x40, 0x0B, 0x0B, 0x00, 0x00, 0x07])),  # Climate ECU Display State (EMF, 21.0°C)
    (0x12D, bytes([0x80, 0x32, 0x32, 0x00, 0x64, 0x00, 0x00, 0x80])),  # AC Compressor state
    (0x228, bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])),  # BSI status
    (0x261, bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])),  # Trip 1 telemetry
    (0x2A1, bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])),  # Trip 2 telemetry
]

TEST_BAUDRATES = [38400, 19200, 9600, 57600, 115200]


try:
    from canbox_usb_ports import find_slcan_device, find_canbox_uart_device
except ImportError:
    import sys
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from canbox_usb_ports import find_slcan_device, find_canbox_uart_device


def get_available_serial_ports() -> List[str]:
    """Finds all available USB serial ports, prioritizing Canbox UART (FTDI 0403:6001)."""
    slcan_dev = find_slcan_device()
    canbox_dev = find_canbox_uart_device()
    candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    ports = [p for p in candidates if p != slcan_dev]
    if canbox_dev in ports:
        ports.remove(canbox_dev)
        ports.insert(0, canbox_dev)
    return ports


def can_keepalive_worker(stop_event: threading.Event, can_iface: str):
    """Broadcasts safe keep-alives at 25 Hz while probing UART."""
    try:
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((can_iface,))
    except Exception as e:
        print(f"[CAN] Note: Cannot bind to {can_iface} ({e}). Probing without CAN wake-up.")
        return

    # Wakeup transition pulse (Phase de vie = 0x03 Wakeup)
    wake_pulse = struct.pack(CAN_FRAME_FMT, 0x036, 8, bytes([0x0E, 0x00, 0x00, 0x2A, 0x03, 0x00, 0x00, 0xA0]))
    for _ in range(5):
        try:
            sock.send(wake_pulse)
        except OSError:
            pass
        time.sleep(0.03)

    while not stop_event.is_set():
        for can_id, data in WAKE_FRAMES:
            frame = struct.pack(CAN_FRAME_FMT, can_id, len(data), data.ljust(8, b"\x00"))
            try:
                sock.send(frame)
            except OSError:
                pass
        time.sleep(0.04)

    sock.close()


def check_hiworld_frame(buf: bytes) -> List[Dict]:
    """Scans for valid Hiworld (0x5A 0xA5 <len> <cmd> ... <cs>) frames."""
    hits = []
    i = 0
    while i <= len(buf) - 5:
        if buf[i] == 0x5A and buf[i + 1] == 0xA5:
            length = buf[i + 2]
            if length <= 64 and (i + length + 5 <= len(buf)):
                pkt = buf[i : i + length + 5]
                cmd = pkt[3]
                cs = pkt[-1]
                calc_cs = (sum(pkt[2:-1]) - 1) & 0xFF
                if cs == calc_cs:
                    hits.append({
                        "proto": "Hiworld",
                        "cmd": cmd,
                        "len": length,
                        "payload": pkt[4:-1].hex(" "),
                        "raw": pkt.hex(" "),
                    })
                    i += len(pkt)
                    continue
        i += 1
    return hits


def check_bagoo_frame(buf: bytes) -> List[Dict]:
    """Scans for valid Bagoo / SimpleSoft (0xFD <total_len> <cmd> ... <cs>) frames."""
    hits = []
    i = 0
    while i <= len(buf) - 4:
        if buf[i] == 0xFD:
            total_len = buf[i + 1]
            if 3 <= total_len <= 64 and (i + total_len + 1 <= len(buf)):
                pkt = buf[i : i + total_len + 1]
                cmd = pkt[2]
                payload = pkt[3:-1]
                cs = pkt[-1]
                calc_cs = (total_len + cmd + sum(payload)) & 0xFF
                if cs == calc_cs:
                    hits.append({
                        "proto": "Bagoo / SimpleSoft (0xFD)",
                        "cmd": cmd,
                        "len": len(payload),
                        "payload": payload.hex(" "),
                        "raw": pkt.hex(" "),
                    })
                    i += len(pkt)
                    continue
        i += 1
    return hits


def check_raise_frame(buf: bytes) -> List[Dict]:
    """Scans for valid Raise (0x2E <cmd> <len> ... <cs>) frames."""
    hits = []
    i = 0
    while i <= len(buf) - 4:
        if buf[i] == 0x2E:
            cmd = buf[i + 1]
            length = buf[i + 2]
            if length <= 64 and (i + length + 4 <= len(buf)):
                pkt = buf[i : i + length + 4]
                cs = pkt[-1]
                calc_cs = (sum(pkt[1:-1]) & 0xFF) ^ 0xFF
                if cs == calc_cs:
                    hits.append({
                        "proto": "Raise (RZC)",
                        "cmd": cmd,
                        "len": length,
                        "payload": pkt[3:-1].hex(" "),
                        "raw": pkt.hex(" "),
                    })
                    i += len(pkt)
                    continue
        i += 1
    return hits


def probe_port(port: str, baud: int, duration: float = 1.2) -> Tuple[bytes, List[Dict], List[Dict], List[Dict]]:
    """Opens port at baud, collects bytes for duration, and searches for valid frames."""
    try:
        with serial.Serial(port, baud, timeout=0.05) as ser:
            ser.reset_input_buffer()
            rx = bytearray()
            t0 = time.time()
            while time.time() - t0 < duration:
                chunk = ser.read(128)
                if chunk:
                    rx.extend(chunk)
                time.sleep(0.01)
            raw = bytes(rx)
            hi_hits = check_hiworld_frame(raw)
            raise_hits = check_raise_frame(raw)
            bagoo_hits = check_bagoo_frame(raw)
            return raw, hi_hits, raise_hits, bagoo_hits
    except Exception as e:
        return b"", [], [], []


def main():
    parser = argparse.ArgumentParser(description="Canbox UART Baud Rate & Protocol Prober")
    parser.add_argument("-p", "--port", default=None, help="UART serial port to probe (auto-detects if omitted)")
    parser.add_argument("-c", "--can", default="can0", help="CAN interface for wake-up broadcast (default: can0)")
    parser.add_argument("-d", "--duration", type=float, default=1.5, help="Sample duration per baud rate in seconds (default: 1.5)")
    args = parser.parse_args()

    ports = [args.port] if args.port else get_available_serial_ports()
    if not ports:
        print("[ERROR] No free serial ports detected (/dev/ttyUSB*).")
        slcan_dev = find_slcan_device()
        if slcan_dev:
            print(f"        (Note: {slcan_dev} is currently occupied by slcand).")
        sys.exit(1)

    print("=====================================================")
    print("      OpenCanbox UART Baud & Protocol Prober         ")
    print("=====================================================")
    slcan_dev = find_slcan_device()
    if slcan_dev:
        print(f"[INFO] SLCAN detected on {slcan_dev} (can0). Excluding from UART probe.")
    print(f"[INFO] Probing serial ports: {', '.join(ports)}")

    # Start CAN keep-alive thread
    stop_event = threading.Event()
    can_thread = threading.Thread(target=can_keepalive_worker, args=(stop_event, args.can), daemon=True)
    can_thread.start()

    best_match = None

    try:
        for port in ports:
            print(f"\n--- Probing Port: {port} ---")
            for baud in TEST_BAUDRATES:
                sys.stdout.write(f"  Testing {baud:6d} baud ... ")
                sys.stdout.flush()

                raw, hi_hits, raise_hits, bagoo_hits = probe_port(port, baud, duration=args.duration)

                if hi_hits:
                    sys.stdout.write(f"MATCH! [Hiworld] - {len(hi_hits)} valid frame(s)\n")
                    best_match = {"port": port, "baud": baud, "proto": "hiworld", "hits": hi_hits}
                    for h in hi_hits[:3]:
                        print(f"    • Cmd 0x{h['cmd']:02X} (len {h['len']}): {h['raw']}")
                    break
                elif raise_hits:
                    sys.stdout.write(f"MATCH! [Raise (RZC 2E)] - {len(raise_hits)} valid frame(s)\n")
                    best_match = {"port": port, "baud": baud, "proto": "raise", "hits": raise_hits}
                    for h in raise_hits[:3]:
                        print(f"    • Cmd 0x{h['cmd']:02X} (len {h['len']}): {h['raw']}")
                    break
                elif bagoo_hits:
                    sys.stdout.write(f"MATCH! [Bagoo / SimpleSoft (0xFD)] - {len(bagoo_hits)} valid frame(s)\n")
                    best_match = {"port": port, "baud": baud, "proto": "bagoo", "hits": bagoo_hits}
                    for h in bagoo_hits[:3]:
                        print(f"    • Cmd 0x{h['cmd']:02X} (len {h['len']}): {h['raw']}")
                    break
                elif raw:
                    sys.stdout.write(f"Received {len(raw)} bytes (corrupt or unaligned checksums)\n")
                    hex_sample = raw[:40].hex(" ")
                    print(f"    Sample: {hex_sample}")
                else:
                    sys.stdout.write("0 bytes (silent)\n")

            if best_match:
                break

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    finally:
        stop_event.set()
        can_thread.join(timeout=0.5)

    print("\n==================== CONCLUSION ====================")
    if best_match:
        print(f"[SUCCESS] Confirmed Canbox Configuration:")
        print(f"  • Port:     {best_match['port']}")
        print(f"  • Baudrate: {best_match['baud']} bps")
        print(f"  • Protocol: {best_match['proto'].upper()}")
        print(f"\nTo run the defrost verifier with these verified settings:")
        print(f"  ./tools/workbench_defrost_verifier.py --port {best_match['port']} --baud {best_match['baud']} --proto {best_match['proto']}")
    else:
        print("[FAIL] No valid Hiworld or Raise frames could be decoded.")
        print("Suggestions:")
        print("  1. Verify UART wiring (TX of canbox -> RX of USB adapter, GND connected).")
        print("  2. Check canbox power: Quadlock Pin 15 (+12V) and Pin 12 (GND).")
        print("  3. Check CAN bus wires: Quadlock Pin 13 (CAN_H) and Pin 10 (CAN_L) connected to can0.")
    print("=====================================================")


if __name__ == "__main__":
    main()

