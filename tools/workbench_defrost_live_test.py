#!/usr/bin/env python3
"""
Peugeot 407 Rear Defrost Live Workbench Confirmation Script
===========================================================
Emulates ONLY the Engine Running signals on CAN (`0x0B6` 900 RPM + `0x12D` Alternator/Compressor OK)
so the physical BSI & Climate ECU unlock the Rear Window Defrost high-load interlock without
colliding with any physical BSI or HVAC frames.

When you press the physical Rear Defrost (or Front Defrost) button on the HVAC fascia panel,
this script immediately detects, decodes, and confirms:
  - Frame ID (e.g. 0x1D0, 0x1E3)
  - Exact DLC (Data Length Code)
  - Exact Byte index, Bit position (0..7), and Hex Bitmask
  - Full payload transition (OLD -> NEW) and Defrost State (ENABLED / DISABLED)
"""

import os
import sys
import glob
import time
import socket
import struct
import select
import argparse
import datetime
import threading
import subprocess
from typing import Dict, List, Optional, Tuple

try:
    import serial
except ImportError:
    serial = None

# ANSI formatting
BOLD = "\033[1m"
CYAN = "\033[36m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
RED = "\033[31m"
MAGENTA = "\033[35m"
RESET = "\033[0m"

CAN_FRAME_FMT = "=IB3x8s"

CLIMA_MONITOR_IDS = {
    0x1D0: "MSG_CDE_CLIMAV (Fascia Panel Demand)",
    0x1E3: "MSG_ETAT_CLIMAV_EMF (Climate ECU Display Status)",
    0x217: "BSI Comfort / Heated Rear Screen Status",
    0x128: "SIGNALISATION_COMBINES (Cluster Indicators)",
    0x168: "Cluster Warning Lamps",
}


def find_ch340_can_device() -> Optional[str]:
    if os.path.exists("/dev/ttyCANShield"):
        return "/dev/ttyCANShield"
    for dev_path in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
        name = os.path.basename(dev_path)
        p = os.path.realpath(f"/sys/class/tty/{name}/device")
        while p and p != "/":
            vid_p = os.path.join(p, "idVendor")
            pid_p = os.path.join(p, "idProduct")
            if os.path.exists(vid_p) and os.path.exists(pid_p):
                try:
                    vid = int(open(vid_p).read().strip(), 16)
                    pid = int(open(pid_p).read().strip(), 16)
                    if vid == 0x1A86 and pid == 0x7523:
                        return dev_path
                except Exception:
                    pass
            p = os.path.dirname(p)
    return None


def ensure_can0_up(interface: str = "can0") -> bool:
    res = subprocess.run(["ip", "link", "show", interface], capture_output=True, text=True)
    if res.returncode == 0 and "UP" in res.stdout:
        subprocess.run(["ip", "link", "set", interface, "txqueuelen", "1000"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    dev = find_ch340_can_device() or "/dev/ttyCANShield"
    if os.path.exists(dev):
        print(f"{CYAN}[CAN INIT]{RESET} Bringing up {interface} @ 125 kbps (-s4) on {dev}...")
        subprocess.run(["slcand", "-o", "-c", "-s4", "-S", "1000000", dev, interface],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.2)
        subprocess.run(["ip", "link", "set", interface, "txqueuelen", "1000", "up"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.1)
    res = subprocess.run(["ip", "link", "show", interface], capture_output=True, text=True)
    return res.returncode == 0 and "UP" in res.stdout


def calc_hiworld_cs(length: int, cmd: int, payload: bytes) -> int:
    return ((length + cmd + sum(payload)) - 1) & 0xFF


def calc_raise_cs(cmd: int, payload: bytes) -> int:
    s = (cmd + len(payload) + sum(payload)) & 0xFF
    return (~s) & 0xFF


def describe_byte_changes(old_b: bytes, new_b: bytes) -> List[Tuple[int, int, int, int, List[Tuple[int, int]]]]:
    """
    Returns list of (byte_idx, old_val, new_val, xor_mask, [(bit_num, new_bit_val), ...])
    """
    changes = []
    limit = min(len(old_b), len(new_b))
    for idx in range(limit):
        if old_b[idx] != new_b[idx]:
            xor = old_b[idx] ^ new_b[idx]
            bits = []
            for b in range(7, -1, -1):
                if xor & (1 << b):
                    bits.append((b, (new_b[idx] >> b) & 1))
            changes.append((idx, old_b[idx], new_b[idx], xor, bits))
    return changes


def main():
    parser = argparse.ArgumentParser(description="Peugeot 407 Engine-Running Rear Defrost Live Confirmation Tool")
    parser.add_argument("--can", default="can0", help="SocketCAN interface (default: can0)")
    parser.add_argument("--port", default=None, help="Optional Canbox UART port (auto-detected if omitted)")
    parser.add_argument("--with-bsi-wake", action="store_true",
                        help="Also broadcast 0x036/0x0F6/0x128 if physical BSI is disconnected")
    parser.add_argument("--all-ids", action="store_true",
                        help="Monitor all CAN IDs on bus instead of only HVAC/body IDs")
    args = parser.parse_args()

    ensure_can0_up(args.can)

    uart_port = args.port
    if not uart_port:
        try:
            sys.path.insert(0, os.path.dirname(__file__))
            from canbox_usb_ports import find_canbox_uart_device
            uart_port = find_canbox_uart_device()
        except Exception:
            uart_port = "/dev/ttyUSB0"

    print("=" * 78)
    print(f"{BOLD}{CYAN}  PEUGEOT 407 REAR DEFROST LIVE CONFIRMATION TOOL{RESET}")
    print(f"  CAN Interface : {BOLD}{args.can}{RESET} (125 kbps PSA Comfort CAN)")
    print(f"  Engine Emul.  : {GREEN}0x0B6 = 1C 20 00 00 00 00 00 D0 (900 RPM, Alternator OK){RESET}")
    print(f"                  {GREEN}0x12D = 80 32 32 00 64 00 00 80 (Compressor/Load Authorized){RESET}")
    if args.with_bsi_wake:
        print(f"  BSI Wakeup    : {YELLOW}ENABLED (0x036, 0x0F6, 0x128){RESET}")
    else:
        print(f"  BSI Wakeup    : {DIM if hasattr(sys, 'ps1') else ''}OFF (Physical BSI owns 0x036/0x0F6/0x128 - zero collision){RESET}")
    print("=" * 78)

    stop_event = threading.Event()

    try:
        can_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        can_sock.bind((args.can,))
    except Exception as e:
        print(f"{RED}[FATAL] Cannot open SocketCAN '{args.can}': {e}{RESET}")
        sys.exit(1)

    ser = None
    if serial is not None and uart_port and os.path.exists(uart_port):
        try:
            ser = serial.Serial(uart_port, 38400, timeout=0.02)
            print(f"[UART] Monitoring Canbox UART on {uart_port} @ 38400 8N1")
        except Exception as e:
            print(f"[UART] Note: UART {uart_port} not opened ({e}) - running CAN-only mode.")

    # Background CAN broadcaster: Engine Running ONLY (unless --with-bsi-wake is passed)
    def engine_sim_worker():
        tx_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        tx_sock.bind((args.can,))
        # 900 RPM: 900 * 8 = 7200 = 0x1C20
        frame_0b6 = struct.pack(CAN_FRAME_FMT, 0x0B6, 8, bytes([0x1C, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0]))
        # Compressor/Alternator load loop authorized
        frame_12d = struct.pack(CAN_FRAME_FMT, 0x12D, 8, bytes([0x80, 0x32, 0x32, 0x00, 0x64, 0x00, 0x00, 0x80]))
        # Optional BSI wakeup if testing HVAC panel standalone without BSI
        frame_036 = struct.pack(CAN_FRAME_FMT, 0x036, 8, bytes([0x0E, 0x00, 0x00, 0x2A, 0x01, 0x00, 0x00, 0xA0]))
        frame_0f6 = struct.pack(CAN_FRAME_FMT, 0x0F6, 8, bytes([0x88, 0x00, 0x00, 0x00, 0x00, 0x50, 0x00, 0x20]))
        frame_128 = struct.pack(CAN_FRAME_FMT, 0x128, 8, bytes([0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))

        cycle = 0
        while not stop_event.is_set():
            try:
                tx_sock.send(frame_0b6)
                if cycle % 2 == 0:
                    tx_sock.send(frame_12d)
                if args.with_bsi_wake and cycle % 2 == 0:
                    tx_sock.send(frame_036)
                    tx_sock.send(frame_0f6)
                    tx_sock.send(frame_128)
                cycle += 1
            except OSError:
                pass
            time.sleep(0.030)
        tx_sock.close()

    t_engine = threading.Thread(target=engine_sim_worker, daemon=True)
    t_engine.start()

    # Optional UART listener (supports both Hiworld 0x31 and Raise 0x21)
    def uart_worker():
        if not ser:
            return
        rx_buf = bytearray()
        last_hw_31 = None
        last_rz_21 = None
        last_hs = 0.0
        while not stop_event.is_set():
            now = time.time()
            if now - last_hs > 3.0:
                try:
                    ser.write(bytes([0x5A, 0xA5, 0x02, 0x24, 0x22, 0x00, 0x47]))
                except Exception:
                    pass
                last_hs = now
            try:
                chunk = ser.read(64)
                if not chunk:
                    continue
                rx_buf.extend(chunk)
                while len(rx_buf) >= 4:
                    # Hiworld 0x5A 0xA5
                    if rx_buf[0] == 0x5A and len(rx_buf) >= 5 and rx_buf[1] == 0xA5:
                        length = rx_buf[2]
                        if length > 32:
                            rx_buf.pop(0)
                            continue
                        total_len = length + 5
                        if len(rx_buf) < total_len:
                            break
                        cmd = rx_buf[3]
                        payload = bytes(rx_buf[4:4 + length])
                        cs = rx_buf[4 + length]
                        if cs == calc_hiworld_cs(length, cmd, payload):
                            del rx_buf[:total_len]
                            if cmd == 0x31 and len(payload) >= 3 and payload != last_hw_31:
                                last_hw_31 = payload
                                rear = bool(payload[2] & 0x20)
                                front = bool(payload[2] & 0x10)
                                status = f"{GREEN}ON (B2 Bit 5 = 1){RESET}" if rear else f"{YELLOW}OFF (B2 Bit 5 = 0){RESET}"
                                print(f"  {MAGENTA}[UART HIWORLD 0x31]{RESET} RearDefrost={status} | FrontDefrost={front} | Raw: {payload.hex(' ').upper()}")
                            continue
                        else:
                            rx_buf.pop(0)
                            continue
                    # Raise 0x2E
                    elif rx_buf[0] == 0x2E:
                        cmd = rx_buf[1]
                        length = rx_buf[2]
                        if length > 32:
                            rx_buf.pop(0)
                            continue
                        total_len = length + 4
                        if len(rx_buf) < total_len:
                            break
                        payload = bytes(rx_buf[3:3 + length])
                        cs = rx_buf[3 + length]
                        if cs == calc_raise_cs(cmd, payload):
                            del rx_buf[:total_len]
                            if cmd == 0x21 and len(payload) >= 1 and payload != last_rz_21:
                                last_rz_21 = payload
                                rear = bool(payload[0] & 0x01)
                                front = bool(payload[4] & 0x80) if len(payload) > 4 else False
                                status = f"{GREEN}ON (B0 Bit 0 = 1){RESET}" if rear else f"{YELLOW}OFF (B0 Bit 0 = 0){RESET}"
                                print(f"  {MAGENTA}[UART RAISE 0x21]{RESET}   RearDefrost={status} | FrontDefrost={front} | Raw: {payload.hex(' ').upper()}")
                            continue
                        else:
                            rx_buf.pop(0)
                            continue
                    else:
                        rx_buf.pop(0)
            except Exception:
                time.sleep(0.01)

    t_uart = threading.Thread(target=uart_worker, daemon=True)
    t_uart.start()

    print(f"\n{GREEN}[READY]{RESET} Engine Running simulation is active on {args.can}.")
    print(f"{BOLD}[ACTION]{RESET} Press the physical {BOLD}Rear Defrost{RESET} button on the HVAC panel now (press to enable, press again to disable).")
    print(f"         Press Ctrl+C to stop.\n")
    print("-" * 78)

    last_can: Dict[int, bytes] = {}
    ignored_tx_ids = {0x0B6, 0x12D}
    if args.with_bsi_wake:
        ignored_tx_ids.update({0x036, 0x0F6, 0x128})

    try:
        while not stop_event.is_set():
            r, _, _ = select.select([can_sock], [], [], 0.05)
            if not r:
                continue
            frame_raw = can_sock.recv(16)
            if not frame_raw:
                continue
            can_id_raw, dlc, data = struct.unpack(CAN_FRAME_FMT, frame_raw)
            can_id = can_id_raw & 0x7FF

            if can_id in ignored_tx_ids:
                continue
            if not args.all_ids and can_id not in CLIMA_MONITOR_IDS:
                continue

            payload = data[:dlc]
            now_str = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

            if can_id not in last_can:
                last_can[can_id] = payload
                name = CLIMA_MONITOR_IDS.get(can_id, "CAN Frame")
                print(f"[{now_str}] [BASELINE] ID=0x{can_id:03X} | DLC={dlc} | Payload={payload.hex(' ').upper()} ({name})")
                continue

            if last_can[can_id] != payload:
                old_p = last_can[can_id]
                last_can[can_id] = payload
                name = CLIMA_MONITOR_IDS.get(can_id, "CAN Frame")
                changes = describe_byte_changes(old_p, payload)

                print(f"\n[{now_str}] {BOLD}{CYAN}⚡ CAN DELTA DETECTED{RESET} — {BOLD}ID: 0x{can_id:03X}{RESET} | {BOLD}DLC: {dlc}{RESET} ({name})")
                print(f"   Payload : {old_p.hex(' ').upper()}  -->  {BOLD}{payload.hex(' ').upper()}{RESET}")

                for b_idx, old_v, new_v, xor_m, bits in changes:
                    bit_desc = ", ".join(
                        f"Bit {b_num} (0x{1 << b_num:02X}): {1 - b_val}->{BOLD}{b_val}{RESET}"
                        for b_num, b_val in bits
                    )
                    label = ""
                    if can_id == 0x1D0 and b_idx == 0 and (xor_m & 0x40):
                        is_on = bool(new_v & 0x40)
                        state_str = f"{GREEN}REAR DEFROST ENABLED (ON){RESET}" if is_on else f"{YELLOW}REAR DEFROST DISABLED (OFF){RESET}"
                        label = f"  ==> 🎯 {BOLD}CONFIRMED [0x1D0 DLC={dlc} Byte 0 Bit 6 (0x40)]: {state_str}{RESET}"
                    elif can_id == 0x1D0 and b_idx == 0 and (xor_m & 0x10):
                        is_on = bool(new_v & 0x10)
                        label = f"  ==> [0x1D0 Byte 0 Bit 4 (0x10): Front Max Defrost {'ON' if is_on else 'OFF'}]"
                    elif can_id == 0x1E3 and b_idx == 1 and (xor_m & 0x08):
                        is_on = bool(new_v & 0x08)
                        state_str = f"{GREEN}REAR DEFROST LED/STATUS ON{RESET}" if is_on else f"{YELLOW}REAR DEFROST LED/STATUS OFF{RESET}"
                        label = f"  ==> 🎯 {BOLD}CONFIRMED [0x1E3 DLC={dlc} Byte 1 Bit 3 (0x08)]: {state_str}{RESET}"
                    elif can_id == 0x1E3 and b_idx == 1 and (xor_m & 0x80):
                        is_on = bool(new_v & 0x80)
                        label = f"  ==> [0x1E3 Byte 1 Bit 7 (0x80): Front Max Defrost {'ON' if is_on else 'OFF'}]"

                    print(f"   Change  : {BOLD}Byte {b_idx}{RESET}: 0x{old_v:02X} -> 0x{new_v:02X} (XOR Mask: 0x{xor_m:02X}) | {bit_desc}{label}")

    except KeyboardInterrupt:
        print(f"\n{CYAN}[STOP] Stopping engine simulation and closing interfaces.{RESET}")
    finally:
        stop_event.set()
        can_sock.close()
        if ser:
            ser.close()


if __name__ == "__main__":
    main()
