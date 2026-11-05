#!/usr/bin/env python3
"""
svt_oem_preflight.py — Fast Pre-Flight Hardware Readiness Checker for OEM CAN Boxes

Performs a deterministic 3-second non-destructive check of:
1. SocketCAN interface (`can0`) link state & traffic.
2. Coexisting physical bench nodes (EMF matrix display, RD4 radio).
3. Canbox UART link (`/dev/ttyRealCanbox` or auto-detected FTDI port @ 38,400 baud).
4. Physical CAN box wake/sleep state via protocol-specific keepalive and queries.

Exit codes:
- 0: DUT is ACTIVE and responsive. Ready for SVT execution.
- 1: DUT is ASLEEP, offline, or hardware port unavailable.
"""

import os
import sys
import time
import socket
import struct
import argparse
import select
from typing import Optional, Tuple, List, Dict, Any

# Ensure svt & tools are importable
repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

from svt.lib.harness import load_dut_profile
from tools.canbox_usb_ports import find_canbox_uart_device

BOLD = "\033[1m"
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
RESET = "\033[0m"

KNOWN_NODES = {
    0x15B: "EMF Display (BSI power/display)",
    0x167: "EMF Display (General frame)",
    0x1ED: "EMF Display (Trip status)",
    0x3F6: "EMF Display (Clock broadcast)",
    0x165: "RD4 Radio (Status/power)",
    0x225: "RD4 Radio (Audio state)",
    0x265: "RD4 Radio (Tuner/RDS)",
    0x3E5: "Steering Stalk / Remotes",
}

def check_can_bus(interface: str, probe_duration: float = 1.0) -> Tuple[bool, List[int], Dict[str, Any]]:
    """Inspects CAN interface and sniffs for coexisting physical nodes without hanging."""
    can_ids_seen = set()
    traffic_detected = False
    details = {"frame_count": 0, "error": None}

    try:
        s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        s.setblocking(False)
        s.bind((interface,))
    except Exception as e:
        details["error"] = str(e)
        return False, [], details

    t_end = time.time() + probe_duration
    try:
        while time.time() < t_end:
            r, _, _ = select.select([s], [], [], max(0.01, t_end - time.time()))
            if not r:
                break
            while True:
                try:
                    cf, _ = s.recvfrom(16)
                    can_id, can_dlc, data = struct.unpack("=IB3x8s", cf)
                    can_id = can_id & 0x1FFFFFFF
                    can_ids_seen.add(can_id)
                    details["frame_count"] += 1
                    traffic_detected = True
                except BlockingIOError:
                    break
    finally:
        s.close()

    return True, sorted(list(can_ids_seen)), details

def probe_canbox_uart(
    port: str,
    baud: int,
    protocol: str,
    timeout: float = 3.0,
    can_interface: Optional[str] = "can0"
) -> Tuple[bool, List[str], Optional[str]]:
    """Opens UART, transmits HU keepalive/query, and listens for CAN box frames."""
    import serial

    # If CAN socket is available, send wake burst and keepalive
    can_sock = None
    if can_interface:
        try:
            can_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            can_sock.bind((can_interface,))
            for _ in range(5):
                can_sock.send(struct.pack("=IB3x8s", 0x036, 8, bytes.fromhex("0E00002A030000A0")))
                time.sleep(0.01)
        except Exception:
            can_sock = None

    packets_received = []
    try:
        ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
    except Exception as e:
        if can_sock:
            can_sock.close()
        return False, [], str(e)

    try:
        # Transmit protocol-specific HU keepalive
        if protocol == "raise":
            # Cmd 0xA7 Model keepalive: [0x2E, 0xA7, 0x02, 0x03, 0x01, 0x52]
            # Cmd 0x8F Query outdoor temp (0x36): [0x2E, 0x8F, 0x01, 0x36, 0x39]
            # Cmd 0x8F Query climate status (0x21): [0x2E, 0x8F, 0x01, 0x21, 0x4E]
            ser.write(bytes([0x2E, 0xA7, 0x02, 0x03, 0x01, 0x52]))
            ser.write(bytes([0x2E, 0x8F, 0x01, 0x36, 0x39]))
            ser.write(bytes([0x2E, 0x8F, 0x01, 0x21, 0x4E]))
            ser.flush()
        elif protocol == "hiworld":
            # Cmd 0x24 Handshake: [0x5A, 0xA5, 0x02, 0x24, 0x01, 0x26]
            ser.write(bytes([0x5A, 0xA5, 0x02, 0x24, 0x01, 0x26]))
            ser.flush()
        elif protocol in ("simplesoft", "bagoo", "xp"):
            # Cmd 0xA7 Model keepalive: [0xFD, 0x05, 0xA7, 0x03, 0x01, 0xB0]
            # Cmd 0x8F Query outdoor temp (0x36): [0xFD, 0x04, 0x8F, 0x36, 0xC9]
            # Cmd 0x8F Query climate status (0x21): [0xFD, 0x04, 0x8F, 0x21, 0xB4]
            ser.write(bytes([0xFD, 0x05, 0xA7, 0x03, 0x01, 0xB0]))
            ser.write(bytes([0xFD, 0x04, 0x8F, 0x36, 0xC9]))
            ser.write(bytes([0xFD, 0x04, 0x8F, 0x21, 0xB4]))
            ser.flush()

        t_end = time.time() + timeout
        buf = bytearray()
        while time.time() < t_end:
            if can_sock:
                try:
                    can_sock.send(struct.pack("=IB3x8s", 0x036, 8, bytes.fromhex("0E00002A010000A0")))
                    can_sock.send(struct.pack("=IB3x8s", 0x0F6, 8, bytes.fromhex("8800000000500020")))
                    can_sock.send(struct.pack("=IB3x8s", 0x128, 8, bytes.fromhex("91E000000080B001")))
                    can_sock.send(struct.pack("=IB3x8s", 0x0B6, 8, bytes.fromhex("1C200000000000D0")))
                    can_sock.send(struct.pack("=IB3x8s", 0x1E3, 8, bytes.fromhex("1C400B0B000007")))
                except Exception:
                    pass
            chunk = ser.read(64)
            if chunk:
                buf.extend(chunk)
                # Parse frames
                if protocol == "raise":
                    while len(buf) >= 4:
                        if buf[0] != 0x2E:
                            buf.pop(0)
                            continue
                        cmd = buf[1]
                        length = buf[2]
                        total_len = 3 + length + 1
                        if len(buf) < total_len:
                            break
                        frame_bytes = bytes(buf[:total_len])
                        del buf[:total_len]
                        packets_received.append(f"Cmd 0x{cmd:02X} ({frame_bytes.hex().upper()})")
                elif protocol == "hiworld":
                    while len(buf) >= 5:
                        if buf[0] != 0x5A or buf[1] != 0xA5:
                            buf.pop(0)
                            continue
                        length = buf[2]
                        cmd = buf[3]
                        total_len = length + 4
                        if len(buf) < total_len:
                            break
                        frame_bytes = bytes(buf[:total_len])
                        del buf[:total_len]
                        packets_received.append(f"Cmd 0x{cmd:02X} ({frame_bytes.hex().upper()})")
                elif protocol in ("simplesoft", "bagoo", "xp"):
                    while len(buf) >= 4:
                        if buf[0] != 0xFD:
                            buf.pop(0)
                            continue
                        total_len = buf[1]
                        if total_len < 3 or total_len > 64:
                            buf.pop(0)
                            continue
                        if len(buf) < total_len + 1:
                            break
                        frame_bytes = bytes(buf[:total_len + 1])
                        cmd = frame_bytes[2]
                        del buf[:total_len + 1]
                        packets_received.append(f"Cmd 0x{cmd:02X} ({frame_bytes.hex().upper()})")
            time.sleep(0.05)

    finally:
        ser.close()
        if can_sock:
            can_sock.close()

    is_active = len(packets_received) > 0
    return is_active, packets_received, None

def main():
    parser = argparse.ArgumentParser(description="Fast Pre-Flight Sanity Checker for OEM CAN Boxes")
    parser.add_argument("--dut", default="raise_oem_407", help="DUT profile YAML or name (default: raise_oem_407)")
    parser.add_argument("--port", default=None, help="Serial port override (e.g. /dev/ttyRealCanbox)")
    parser.add_argument("--interface", default="can0", help="CAN interface name (default: can0)")
    parser.add_argument("--timeout", type=float, default=2.5, help="UART listen timeout in seconds (default: 2.5s)")
    args = parser.parse_args()

    print(f"\n{BOLD}{CYAN}=== OEM CAN Box SVT Pre-Flight Sanity Check ==={RESET}")

    dut_cfg = load_dut_profile(args.dut, repo_root)
    dut_name = dut_cfg.get("name", args.dut)
    protocol = dut_cfg.get("protocol", "raise").lower()
    baud = dut_cfg.get("baud", 38400)

    serial_port = args.port
    if not serial_port:
        serial_port = dut_cfg.get("serial_port")
    if not serial_port or not os.path.exists(serial_port):
        # Auto-detect via canbox profile
        dev = find_canbox_uart_device()
        if dev:
            serial_port = dev
        else:
            serial_port = "/dev/ttyRealCanbox"

    # Step 1: CAN Interface & Sniff Check
    can_ok, can_ids, can_details = check_can_bus(args.interface, probe_duration=1.0)
    coexisting_nodes = []
    for cid in can_ids:
        if cid in KNOWN_NODES:
            coexisting_nodes.append(f"0x{cid:03X} ({KNOWN_NODES[cid]})")

    # Step 2: UART Probe & Sleep Detection
    port_exists = os.path.exists(serial_port)
    uart_ok, packets, uart_err = False, [], "Port does not exist"
    if port_exists:
        uart_ok, packets, uart_err = probe_canbox_uart(
            serial_port, baud, protocol, timeout=args.timeout,
            can_interface=args.interface if can_ok else None
        )

    # Print Summary Table
    print(f"--------------------------------------------------------------------------------")
    print(f"{'CHECK':<28s} {'STATUS':<10s} {'DETAILS'}")
    print(f"--------------------------------------------------------------------------------")

    # Row 1: CAN Interface
    if can_ok:
        can_stat = f"{GREEN}ONLINE{RESET}"
        can_info = f"{args.interface} active ({can_details['frame_count']} frames detected in 1.0s)"
    else:
        can_stat = f"{RED}OFFLINE{RESET}"
        can_info = f"Failed to bind {args.interface}: {can_details.get('error')}"
    print(f"  {'CAN Bus Interface':<26s} {can_stat:<19s} {can_info}")

    # Row 2: Physical Coexisting Nodes
    if coexisting_nodes:
        node_stat = f"{YELLOW}DETECTED{RESET}"
        node_info = ", ".join(coexisting_nodes)
    else:
        node_stat = f"{CYAN}STANDALONE{RESET}"
        node_info = "No physical EMF or RD4 nodes streaming on bus"
    print(f"  {'Coexisting Bus Nodes':<26s} {node_stat:<19s} {node_info}")

    # Row 3: UART Port
    if port_exists:
        p_stat = f"{GREEN}FOUND{RESET}"
        p_info = f"{serial_port} @ {baud} baud (8N1)"
    else:
        p_stat = f"{RED}MISSING{RESET}"
        p_info = f"{serial_port} not found in /dev/"
    print(f"  {'Canbox UART Port':<26s} {p_stat:<19s} {p_info}")

    # Row 4: DUT Responsiveness
    if uart_ok:
        dut_stat = f"{GREEN}ACTIVE{RESET}"
        unique_cmds = sorted(list(set(p.split()[1] for p in packets)))
        dut_info = f"Received {len(packets)} frames ({', '.join(unique_cmds)})"
    else:
        dut_stat = f"{RED}SLEEPING{RESET}"
        dut_info = f"No serial response within {args.timeout:.1f}s. {uart_err or 'DUT is in deep sleep.'}"
    print(f"  {'DUT Wake / Comm State':<26s} {dut_stat:<19s} {dut_info}")
    print(f"--------------------------------------------------------------------------------")

    # Final Verdict & Guidance
    if can_ok and port_exists and uart_ok:
        print(f"\n{BOLD}{GREEN}>>> [PASS] OEM CAN Box is AWAKE and READY for SVT execution! <<<{RESET}\n")
        return 0
    else:
        print(f"\n{BOLD}{RED}>>> [BLOCKED] OEM CAN Box is NOT READY for SVT execution! <<<{RESET}")
        if not can_ok:
            print(f"  {YELLOW}Action:{RESET} Bring up {args.interface} via `{BOLD}tools/slcan.sh{RESET}` (or restart with `tools/slcan.sh -k && tools/slcan.sh`).")
        if not port_exists:
            print(f"  {YELLOW}Action:{RESET} Check USB connection for Canbox UART interface ({serial_port}).")
        if port_exists and not uart_ok:
            print(f"  {YELLOW}Action:{RESET} {BOLD}POWER CYCLE THE CAN BOX.{RESET} The physical OEM CAN box has entered sleep.")
            print(f"          Disconnect and reconnect the CAN box power supply, then re-run this check.")
        print()
        return 1

if __name__ == "__main__":
    sys.exit(main())
