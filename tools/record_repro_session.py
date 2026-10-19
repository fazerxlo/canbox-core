#!/usr/bin/env python3
"""
Record Reproduction Session
===========================
Synchronously captures and correlates:
1. CAN bus frames on `can0` (microsecond resolution)
2. Android Head Unit logcat over ADB (UART Uplink/Downlink + Audio Routing)
3. NUC131 ST-Link / OpenOCD CPU state (detecting halts, watchdogs, resets)

Live highlights RD4 resets (0x5E0, 0x165 00000000, 0880, 0890), source switches,
and UART 0xAD commands. Saves raw candump and correlated session log.
"""

import argparse
import datetime
import os
import re
import select
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

CAN_FRAME_FMT = "=IB3x8s"
CAN_IDS_RD4 = {0x165, 0x131, 0x1A5, 0x1E5, 0x225, 0x265, 0x2A5, 0x325, 0x365, 0x3A5, 0x3E5, 0x525, 0x5E0}
CAN_IDS_CDC = {0x162, 0x1A2, 0x1E2, 0x122, 0x522}
CAN_IDS_SWC = {0x21F}

g_running = True

def signal_handler(sig, frame):
    global g_running
    g_running = False

def openocd_poll(host="127.0.0.1", port=6666):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.5)
        s.connect((host, port))
        s.sendall(b"NuMicro.cpu curstate\x1a")
        res = bytearray()
        while True:
            chunk = s.recv(1024)
            if not chunk:
                break
            res += chunk
            if b"\x1a" in chunk:
                break
        s.close()
        return res.decode("utf-8", errors="ignore").rstrip("\x1a").strip()
    except Exception:
        return "disconnected"

def decode_165(data):
    if len(data) < 4:
        return "short"
    d0, d1, d2, d3 = data[0], data[1], data[2], data[3]
    if d0 == 0x00 and d1 == 0x00:
        return "[REBOOT: Watchdog Reset (00 00)]"
    if d0 == 0x08 and d1 == 0x90:
        return "[FAULT: Audio Controller Crash (08 90)]"
    if d0 == 0x08 and d1 == 0x80:
        return "[FAULT: Internal Bus Reset Stage (08 80)]"
    src_str = "Unknown"
    if d2 < 0x20:
        src_str = f"Tuner ({d2:02X})"
    elif d2 in (0x20, 0x30):
        src_str = f"CD ({d2:02X})"
    elif d2 == 0x31:
        src_str = "CDC"
    elif d2 in (0x40, 0x41):
        src_str = f"AUX ({d2:02X})"
    elif d2 == 0xFF:
        src_str = "OFF"
    mute_str = "Active" if (d0 & 0x80) else "Muted"
    return f"Src:{src_str} ({mute_str}, B0={d0:02X} B1={d1:02X})"

def main():
    global g_running
    parser = argparse.ArgumentParser(description="Record RD4 / CANbox reproduction session")
    parser.add_argument("--can", default="can0", help="SocketCAN interface (default: can0)")
    parser.add_argument("--adb", default="192.168.1.33:9876", help="ADB target device (default: 192.168.1.33:9876)")
    parser.add_argument("--out", default=None, help="Output log prefix")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    ts_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_can_path = f"repro_can_{ts_str}.log"
    log_unified_path = f"repro_session_{ts_str}.log"
    log_adb_path = f"repro_adb_{ts_str}.log"

    print("======================================================================")
    print(" RD4 / OpenCanbox Live Session Recorder & ST-Link Monitor")
    print("======================================================================")
    print(f"[*] CAN Interface:      {args.can}")
    print(f"[*] ADB Target:         {args.adb}")
    print(f"[*] OpenOCD TCL Port:   127.0.0.1:6666")
    print(f"[*] Session Log:        {log_unified_path}")
    print(f"[*] Raw CAN Dump:       {log_can_path}")
    print(f"[*] Raw ADB Logcat:     {log_adb_path}")

    # Check OpenOCD initial state
    init_state = openocd_poll()
    print(f"[*] OpenOCD Target State: {init_state}")

    # Open CAN socket
    try:
        can_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        can_sock.bind((args.can,))
        can_sock.setblocking(False)
    except Exception as e:
        print(f"[!] Failed to bind to SocketCAN {args.can}: {e}")
        sys.exit(1)

    f_can = open(log_can_path, "w", buffering=1)
    f_unified = open(log_unified_path, "w", buffering=1)
    f_adb = open(log_adb_path, "w", buffering=1)

    # Start ADB logcat subprocess
    adb_cmd = ["adb", "-s", args.adb, "logcat", "-v", "time"]
    adb_proc = None
    try:
        adb_proc = subprocess.Popen(adb_cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
    except Exception as e:
        print(f"[!] Warning: Failed to spawn adb logcat: {e}")

    def adb_reader():
        global g_running
        if not adb_proc or not adb_proc.stdout:
            return
        pattern_downlink = re.compile(r"(writeData-->\[|APP2MCU - writeToUart: \[)([^\]]+)\]")
        pattern_uplink = re.compile(r"MCU2APP : \[([^\]]+)\]")
        pattern_route = re.compile(r"PsaAudioRoute:\s+(.+)")

        for line in iter(adb_proc.stdout.readline, ""):
            if not g_running:
                break
            if not line:
                continue
            f_adb.write(line)
            # Filter relevant audio / UART lines for unified log
            now_str = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            m_dl = pattern_downlink.search(line)
            m_ul = pattern_uplink.search(line)
            m_rt = pattern_route.search(line)

            if m_dl:
                payload = m_dl.group(2).strip()
                tag = f"[{now_str}] [HU_UART_TX] {payload}"
                f_unified.write(tag + "\n")
                if "ad " in payload.lower() or "5a a5 02 ad" in payload.lower():
                    print(f"\033[96m{tag}\033[0m")
            elif m_ul:
                payload = m_ul.group(1).strip()
                # Check for 5A A5
                if "5a a5" in payload.lower():
                    tag = f"[{now_str}] [CANBOX_UART_TX] {payload}"
                    f_unified.write(tag + "\n")
                    if " 97 " in payload.lower() or " 84 " in payload.lower():
                        print(f"\033[94m{tag}\033[0m")
            elif m_rt:
                tag = f"[{now_str}] [HU_ROUTE] {m_rt.group(1).strip()}"
                f_unified.write(tag + "\n")
                print(f"\033[93m{tag}\033[0m")

    adb_thread = threading.Thread(target=adb_reader, daemon=True)
    adb_thread.start()

    print("\n[+] RECORDER ARMED & RUNNING. Ready for reproduction!")
    print("[+] Please toggle the RD4 Source or SWC button now to reproduce the reset.")
    print("[+] Press Ctrl+C when finished to stop recording and print analysis summary.\n")

    last_poll_time = 0
    last_165_str = None
    last_131_str = None

    try:
        while g_running:
            r, _, _ = select.select([can_sock], [], [], 0.05)
            now_ts = time.time()
            now_str = datetime.datetime.fromtimestamp(now_ts).strftime("%H:%M:%S.%f")[:-3]

            # Poll OpenOCD state every 1 second
            if now_ts - last_poll_time >= 1.0:
                last_poll_time = now_ts
                st = openocd_poll()
                if st != "running":
                    msg = f"[{now_str}] [STLINK_WARN] NUC131 CPU State: {st}"
                    f_unified.write(msg + "\n")
                    print(f"\033[91m{msg}\033[0m")

            if r:
                try:
                    raw_data = can_sock.recv(64)
                    if len(raw_data) >= 16:
                        can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw_data[:16])
                        data_bytes = data[:dlc]
                        hex_data = " ".join(f"{b:02X}" for b in data_bytes)
                        candump_line = f"({now_ts:.6f}) {args.can} {can_id:03X}#{data_bytes.hex().upper()}"
                        f_can.write(candump_line + "\n")

                        # Check interesting frames
                        if can_id == 0x165:
                            decoded = decode_165(data_bytes)
                            f_unified.write(f"[{now_str}] [CAN 0x165] {hex_data:<14} | {decoded}\n")
                            if hex_data != last_165_str:
                                last_165_str = hex_data
                                if "REBOOT" in decoded or "FAULT" in decoded:
                                    print(f"\033[91;1m[{now_str}] [CAN 0x165 RESET!] {hex_data} -> {decoded}\033[0m")
                                else:
                                    print(f"[{now_str}] [CAN 0x165] {hex_data:<14} | {decoded}")

                        elif can_id == 0x5E0:
                            f_unified.write(f"[{now_str}] [CAN 0x5E0 RESET FRAME] {hex_data}\n")
                            print(f"\033[91;1m[{now_str}] [CAN 0x5E0 RESET DIAGNOSTIC FRAME DETECTED!] {hex_data}\033[0m")

                        elif can_id == 0x131:
                            desc = "Deck Active" if (len(data_bytes) > 0 and data_bytes[0] == 0x01) else "Deck Standby"
                            f_unified.write(f"[{now_str}] [CAN 0x131] {hex_data:<14} | {desc}\n")
                            if hex_data != last_131_str:
                                last_131_str = hex_data
                                print(f"[{now_str}] [CAN 0x131] {hex_data:<14} | {desc}")

                        elif can_id == 0x162:
                            f_unified.write(f"[{now_str}] [CAN 0x162 CDC] {hex_data}\n")
                            print(f"\033[95m[{now_str}] [CAN 0x162 CDC Emulation Frame] {hex_data}\033[0m")

                        elif can_id == 0x21F:
                            if len(data_bytes) >= 1 and data_bytes[0] != 0x00:
                                f_unified.write(f"[{now_str}] [CAN 0x21F STALK KEY] {hex_data}\n")
                                print(f"\033[92m[{now_str}] [CAN 0x21F Stalk Button Pressed] {hex_data}\033[0m")

                except BlockingIOError:
                    pass
                except Exception as e:
                    pass

    finally:
        print("\n[*] Stopping recording session...")
        g_running = False
        if adb_proc:
            try:
                adb_proc.terminate()
                adb_proc.wait(timeout=1.0)
            except Exception:
                adb_proc.kill()
        can_sock.close()
        f_can.close()
        f_unified.close()
        f_adb.close()

        print(f"[+] Recording saved:")
        print(f"    - Unified log: {log_unified_path}")
        print(f"    - CAN raw log: {log_can_path}")
        print(f"    - ADB raw log: {log_adb_path}")

        # Run triage if tool exists
        if os.path.exists("tools/triage_canbox_log.py"):
            print("\n[*] Auto-triaging session log...")
            try:
                subprocess.run([sys.executable, "tools/triage_canbox_log.py", log_unified_path])
            except Exception as e:
                print(f"[!] Triage failed: {e}")

if __name__ == "__main__":
    main()

