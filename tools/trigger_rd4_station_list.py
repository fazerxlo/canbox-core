#!/usr/bin/env python3
"""
Peugeot 407 RD4 Station List Trigger & Decoder Utility
======================================================
Triggers and reassembles the memorized radio station list from the Blaupunkt RD4
radio over CAN bus (can0) using the verified single rotary tick method.

Protocol Flow:
  1. Reads baseline rotary counter from CAN 0x21F (COM2000).
  2. Injects a single tick forward: CAN 0x21F [0x00, counter + 1, 0x00].
  3. Intercepts CAN 0x125 ISO-TP First Frame (0x10 0x3A ...).
  4. Transmits ISO-TP Flow Control on CAN 0x11F [0x30, 0x00, 0x00] (emulating EMF display).
  5. Reassembles Consecutive Frames (0x21 .. 0x28).
  6. Decodes all 6 memorized preset station names (8 ASCII bytes + delimiter).
"""

import argparse
import select
import socket
import struct
import sys
import time

CAN_FRAME_FMT = "=IB3x8s"
CAN_ID_SWC_STALK_21F = 0x21F
CAN_ID_STATION_LIST_125 = 0x125
CAN_ID_EMF_FC_11F = 0x11F


def pack_can(can_id: int, data: bytes) -> bytes:
    return struct.pack(CAN_FRAME_FMT, can_id & 0x7FF, len(data), data.ljust(8, b"\x00"))


def parse_can(raw: bytes):
    cid_raw, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
    return (cid_raw & 0x7FF), min(dlc, 8), data[:dlc]


def trigger_station_list(iface: str = "can0", timeout: float = 1.5):
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((iface,))

    # 1. Read current stalk counter from bus
    counter = 0x10
    t0 = time.time()
    while time.time() - t0 < 0.3:
        pkt = sock.recv(16)
        cid, dlc, data = parse_can(pkt)
        if cid == CAN_ID_SWC_STALK_21F and dlc >= 2:
            counter = data[1]
            break

    next_cnt = (counter + 1) & 0xFF
    print(f"[*] Reading COM2000 baseline counter: 0x{counter:02X}")
    print(f"[*] Sending Rotary Trigger Tick on CAN 0x21F [00 {next_cnt:02X} 00]...")
    sock.send(pack_can(CAN_ID_SWC_STALK_21F, bytes([0x00, next_cnt, 0x00])))

    # 2. Capture and reassemble ISO-TP 0x125
    isotp_buf = bytearray()
    total_len = 0
    t_end = time.time() + timeout

    while time.time() < t_end:
        r, _, _ = select.select([sock], [], [], 0.05)
        if not r:
            continue
        pkt = sock.recv(16)
        if len(pkt) != 16:
            continue
        cid, dlc, data = parse_can(pkt)

        if cid == CAN_ID_STATION_LIST_125:
            pci = data[0] & 0xF0
            if pci == 0x10:  # First Frame
                total_len = (((data[0] & 0x0F) << 8) | data[1])
                isotp_buf = bytearray(data[2:])
                # Send Flow Control CTS on 0x11F
                sock.send(pack_can(CAN_ID_EMF_FC_11F, bytes([0x30, 0x00, 0x00])))
            elif pci == 0x20 and isotp_buf:  # Consecutive Frame
                isotp_buf.extend(data[1:])
                if len(isotp_buf) >= total_len:
                    break

    if not isotp_buf:
        print("[FAIL] No 0x125 station list received from RD4.")
        return False

    print(f"[✓] Successfully reassembled CAN 0x125 ({len(isotp_buf)} bytes):\n")
    print("=" * 45)
    print(" RD4 MEMORIZED PRESET STATIONS (CAN 0x125)")
    print("=" * 45)

    # Decode 6 entries (offset 4 + i*9, 8 ASCII chars each)
    for i in range(6):
        offset = 4 + (i * 9)
        if offset + 8 <= len(isotp_buf):
            raw_name = bytes(isotp_buf[offset:offset + 8])
            name = "".join(chr(b) if 0x20 <= b <= 0x7E else " " for b in raw_name).strip()
            print(f"  Slot {i + 1}:  [ {name:<10s} ]  (hex: {raw_name.hex()})")

    print("=" * 45)
    return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Trigger RD4 Memorized Station List on CAN")
    parser.add_argument("-i", "--interface", default="can0", help="SocketCAN interface (default: can0)")
    args = parser.parse_args()

    ok = trigger_station_list(args.interface)
    sys.exit(0 if ok else 1)

