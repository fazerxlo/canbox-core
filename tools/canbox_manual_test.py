#!/usr/bin/env python3
"""
OpenCanbox Core - Manual Hardware-in-the-Loop (HIL) & Scenario Replay Runner
===========================================================================
This tool facilitates manual end-to-end testing between a Head Unit (HU)
and the OpenCanbox Core application running on Linux.

It performs:
1. Environment Setup (vcan0 interface, build artifact checks)
2. Hardware USB Serial Port Discovery / Configuration
3. Spawning the OpenCanbox Core desktop simulator
4. Replaying CAN logs (.csv) in real-time in a continuous loop over SocketCAN
"""

import argparse
import csv
import glob
import os
import signal
import socket
import struct
import subprocess
import sys
import time

# SocketCAN constants
CAN_RAW = 1
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_LOG_PATH = os.path.join(
    PROJECT_ROOT,
    "test",
    "test_integration",
    "data",
    "lights_off_side_light_on_headlights_on.csv",
)
NATIVE_BINARY_PATH = os.path.join(PROJECT_ROOT, ".pio", "build", "native_test", "program")


def parse_can_frame(can_id, data_bytes, is_extended=False, is_rtr=False):
    """Packs CAN frame into Linux struct can_frame (16 bytes)."""
    can_id_flagged = can_id & (CAN_EFF_MASK if is_extended else CAN_SFF_MASK)
    if is_extended:
        can_id_flagged |= CAN_EFF_FLAG
    if is_rtr:
        can_id_flagged |= CAN_RTR_FLAG

    dlc = len(data_bytes)
    data_padded = bytes(data_bytes).ljust(8, b"\x00")
    # Struct layout: uint32 can_id, uint8 can_dlc, uint8 __pad, uint8 __res0, uint8 __res1, uint8 data[8]
    return struct.pack("=IB3x8s", can_id_flagged, dlc, data_padded)


def find_serial_ports():
    """Scans system for connected USB-to-UART serial adapters."""
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    return ports


def check_and_setup_vcan(iface="vcan0", auto_setup=False):
    """Ensures the virtual CAN network interface exists and is up."""
    # Check if interface exists in sysfs
    if os.path.exists(f"/sys/class/net/{iface}"):
        print(f"[OK] CAN interface '{iface}' is present.")
        return True

    print(f"[WARN] Virtual CAN interface '{iface}' does not exist.")
    setup_cmds = [
        ["sudo", "modprobe", "vcan"],
        ["sudo", "ip", "link", "add", "dev", iface, "type", "vcan"],
        ["sudo", "ip", "link", "set", "up", iface],
    ]

    if auto_setup:
        print(f"[*] Attempting automatic configuration of '{iface}' via sudo...")
        try:
            for cmd in setup_cmds:
                subprocess.check_call(cmd)
            print(f"[OK] Successfully initialized '{iface}'.")
            return True
        except subprocess.CalledProcessError as e:
            print(f"[ERROR] Failed to automatically setup '{iface}': {e}")

    cmd_str = f"sudo modprobe vcan && sudo ip link add dev {iface} type vcan && sudo ip link set up {iface}"
    print("\n[ACTION REQUIRED] Please create the interface manually by running:")
    print(f"  {cmd_str}\n")
    return False


def ensure_app_built():
    """Ensures the native desktop binary is compiled."""
    if os.path.isfile(NATIVE_BINARY_PATH) and os.access(NATIVE_BINARY_PATH, os.X_OK):
        return True

    print("[*] Binary not found. Building 'native_test' target...")
    pio_path = os.path.expanduser("~/.platformio/penv/bin/pio")
    pio_cmd = pio_path if os.path.isfile(pio_path) else "pio"

    try:
        subprocess.check_call(
            [pio_cmd, "run", "-e", "native_test"],
            cwd=PROJECT_ROOT,
        )
        print("[OK] Build successful.")
        return True
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        print(f"[ERROR] Failed to compile native binary: {e}")
        return False


class CanCsvReader:
    """Reads and parses the project standard 14-column CAN capture CSV."""

    def __init__(self, filepath):
        self.filepath = filepath
        self.frames = []
        self._load()

    def _load(self):
        if not os.path.isfile(self.filepath):
            raise FileNotFoundError(f"CAN log CSV file not found: {self.filepath}")

        with open(self.filepath, "r", encoding="utf-8", errors="ignore") as f:
            reader = csv.reader(f)
            header = next(reader, None)

            for row in reader:
                if not row or len(row) < 7:
                    continue
                try:
                    ts_us = int(row[0].strip())
                    can_id = int(row[1].strip(), 16)
                    is_extended = row[2].strip().lower() in ("true", "1")
                    # Dir=row[3], Bus=row[4]
                    dlc = int(row[5].strip())
                    dlc = min(max(dlc, 0), 8)
                    data = []
                    for i in range(dlc):
                        byte_str = row[6 + i].strip()
                        data.append(int(byte_str, 16) if byte_str else 0)

                    self.frames.append({
                        "ts_us": ts_us,
                        "can_id": can_id,
                        "is_extended": is_extended,
                        "data": data,
                    })
                except (ValueError, IndexError):
                    continue

        if not self.frames:
            raise ValueError(f"No valid CAN frames found in CSV: {self.filepath}")

        print(f"[OK] Loaded {len(self.frames)} CAN frames from {os.path.basename(self.filepath)}")


def replay_can_loop(
    frames,
    iface="vcan0",
    loop=True,
    speed=1.0,
    inter_loop_delay=0.5,
    stop_event_check=None,
):
    """Streams CAN frames sequentially over Linux SocketCAN respecting timestamps."""
    can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, CAN_RAW)
    try:
        can_socket.bind((iface,))
    except OSError as e:
        print(f"[ERROR] Failed to bind to SocketCAN interface '{iface}': {e}")
        return

    loop_count = 0
    total_sent = 0
    start_time = time.time()

    print(f"[*] Starting CAN log playback on interface '{iface}' (Speed: {speed}x)...")

    try:
        while True:
            loop_count += 1
            loop_start_sim_us = frames[0]["ts_us"]
            loop_start_real = time.perf_counter()

            for frame in frames:
                if stop_event_check and stop_event_check():
                    return

                # Calculate target delay relative to start of loop
                sim_delta_s = (frame["ts_us"] - loop_start_sim_us) / 1_000_000.0 / speed
                real_delta_s = time.perf_counter() - loop_start_real
                sleep_needed = sim_delta_s - real_delta_s

                if sleep_needed > 0.0005:
                    time.sleep(sleep_needed)

                raw_frame = parse_can_frame(
                    frame["can_id"],
                    frame["data"],
                    is_extended=frame["is_extended"],
                )
                try:
                    can_socket.send(raw_frame)
                    total_sent += 1
                except OSError as e:
                    print(f"[WARN] CAN send error: {e}")

            elapsed = time.time() - start_time
            rate = total_sent / max(elapsed, 0.001)
            print(
                f"\r[CAN Replay] Loop #{loop_count} complete | Total Frames: {total_sent} | "
                f"Rate: {rate:.1f} fps | Elapsed: {elapsed:.1f}s",
                end="",
                flush=True,
            )

            if not loop:
                print("\n[OK] Single pass replay finished.")
                break

            if inter_loop_delay > 0:
                time.sleep(inter_loop_delay)

    finally:
        can_socket.close()


def main():
    parser = argparse.ArgumentParser(
        description="OpenCanbox Core - Manual HU Test & CAN Log Replay Harness"
    )
    parser.add_argument(
        "--csv",
        "-c",
        default=DEFAULT_LOG_PATH,
        help=f"Path to CAN log CSV file (default: {os.path.basename(DEFAULT_LOG_PATH)})",
    )
    parser.add_argument(
        "--serial-port",
        "-p",
        default=None,
        help="Path to physical USB serial dongle (e.g. /dev/ttyUSB0). If omitted, auto-discovery is performed.",
    )
    parser.add_argument(
        "--protocol",
        "-P",
        default="bagoo",
        choices=["bagoo", "raise", "hiworld"],
        help="Head Unit protocol dialect (default: bagoo for Peugeot 407 / PSA 15)",
    )
    parser.add_argument(
        "--baudrate",
        "-b",
        type=int,
        default=19200,
        help="UART Baud rate (default: 19200 for Peugeot 407 Raise/Bagoo)",
    )
    parser.add_argument(
        "--iface",
        "-i",
        default="vcan0",
        help="CAN interface name (default: vcan0)",
    )
    parser.add_argument(
        "--speed",
        "-s",
        type=float,
        default=1.0,
        help="CAN replay playback speed multiplier (default: 1.0)",
    )
    parser.add_argument(
        "--no-loop",
        action="store_true",
        help="Replay CAN log once instead of infinite loop",
    )
    parser.add_argument(
        "--loop-delay",
        type=float,
        default=0.5,
        help="Delay in seconds between replay loops (default: 0.5s)",
    )
    parser.add_argument(
        "--no-app",
        action="store_true",
        help="Do not start the OpenCanbox Core application (CAN replay only)",
    )
    parser.add_argument(
        "--auto-setup-vcan",
        action="store_true",
        help="Attempt to auto-create vcan0 using sudo if missing",
    )

    args = parser.parse_args()

    print("=" * 72)
    print("      OPENCANBOX CORE - MANUAL HARDWARE TEST & LOG REPLAYER")
    print("=" * 72)

    # 1. Check / Setup vcan interface
    if not check_and_setup_vcan(args.iface, auto_setup=args.auto_setup_vcan):
        sys.exit(1)

    # 2. Setup Serial Connection & Discovery
    serial_port = args.serial_port
    if not serial_port:
        available_ports = find_serial_ports()
        if available_ports:
            print(f"[INFO] Discovered USB serial adapters: {', '.join(available_ports)}")
            serial_port = available_ports[0]
            print(f"[*] Auto-selected serial port: {serial_port}")
        else:
            print("[WARN] No USB serial adapters (/dev/ttyUSB* or /dev/ttyACM*) detected.")
            print("       If running in virtual PTY simulation mode, the app will create /tmp/ttyCanbox.")
            serial_port = None

    # 3. Ensure native app is compiled
    app_process = None
    if not args.no_app:
        if not ensure_app_built():
            sys.exit(1)

        app_env = os.environ.copy()
        app_env["CANBOX_CAN_IFACE"] = args.iface
        app_env["CANBOX_HU_PROTOCOL"] = args.protocol
        if serial_port:
            app_env["CANBOX_UART_DEVICE"] = serial_port
            app_env["CANBOX_UART_BAUD"] = str(args.baudrate)

        print(f"[*] Starting OpenCanbox Core binary: {NATIVE_BINARY_PATH}")
        print(f"    - CAN Interface : {args.iface}")
        print(f"    - HU Protocol   : {args.protocol.upper()}")
        print(f"    - Serial Target : {serial_port if serial_port else '/tmp/ttyCanbox (PTY)'}")
        print(f"    - Baud Rate     : {args.baudrate}")

        app_process = subprocess.Popen(
            [NATIVE_BINARY_PATH],
            cwd=PROJECT_ROOT,
            env=app_env,
        )
        time.sleep(0.3)

    # 4. Load CSV CAN Log
    try:
        reader = CanCsvReader(args.csv)
    except Exception as e:
        print(f"[ERROR] {e}")
        if app_process:
            app_process.terminate()
        sys.exit(1)

    # 5. Handle Clean Shutdown on Ctrl+C
    stopped = False

    def handle_sigint(signum, frame):
        nonlocal stopped
        stopped = True
        print("\n[*] Stopping replay and terminating subprocesses...")
        if app_process:
            app_process.terminate()
            try:
                app_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                app_process.kill()
        print("[OK] Clean shutdown complete.")
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)
    signal.signal(signal.SIGTERM, handle_sigint)

    # 6. Replay CAN Log Loop
    replay_can_loop(
        reader.frames,
        iface=args.iface,
        loop=not args.no_loop,
        speed=args.speed,
        inter_loop_delay=args.loop_delay,
        stop_event_check=lambda: stopped,
    )

    if app_process:
        app_process.terminate()


if __name__ == "__main__":
    main()

