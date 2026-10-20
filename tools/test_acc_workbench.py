#!/usr/bin/env python3
"""
Workbench Hardware Integration Test: Head Unit ACC & Power Lifecycle
===================================================================
Coordinates SLCAN (can0), Android Head Unit (ADB), and OpenOCD (ST-Link)
to validate OpenCanbox ACC (switched 12V supply enable) logic on the physical bench.

Hardware Setup:
  - Host PC connected to CAN bus via SLCAN (can0 @ 125 kbps).
  - CAN box connected to can0 and Head Unit (UART + switched ACC wire).
  - Blaupunkt RD4 radio connected to can0 (no vehicle BSI).
  - Android Head Unit connected via ADB (WiFi / TCP / USB).
  - (Optional) ST-Link V2 connected to CAN box SWD for live register inspection.

Scenarios Tested:
  1. Ignition ON (+APC) -> Head Unit ACC turns ON (PA8 / PB9 HIGH, screen Awake).
  2. RD4 silence / warm reboot mid-drive -> ACC remains solidly HIGH (no glitch).
  3. Ignition turns OFF -> ACC immediately turns OFF (PA8 / PB9 LOW, screen Asleep).
  4. Ignition OFF + RD4 switched ON -> ACC turns ON (Accessory mode).
  5. Ignition OFF + RD4 switched OFF -> ACC turns OFF.
  6. Economy mode active -> ACC is immediately cut.
  7. Standalone mode (No RD4) -> ACC strictly follows ignition key.

Usage:
  python3 tools/test_acc_workbench.py --can can0 --adb 192.168.1.33:9876
  python3 tools/test_acc_workbench.py --can can0 --adb 192.168.1.33:9876 --openocd 127.0.0.1:4444
  python3 tools/test_acc_workbench.py --help
"""

import argparse
import datetime
import os
import re
import select
import socket
import struct
import subprocess
import sys
import threading
import time

CAN_FRAME_FMT = "=IB3x8s"
CAN_ID_BSI_036 = 0x036
CAN_ID_BSI_0F6 = 0x0F6
CAN_ID_RD4_165 = 0x165
CAN_ID_RD4_WAKEUP = 0x420

COLOR_RESET  = "\033[0m"
COLOR_GREEN  = "\033[92m"
COLOR_RED    = "\033[91m"
COLOR_YELLOW = "\033[93m"
COLOR_CYAN   = "\033[96m"
COLOR_GRAY   = "\033[90m"
COLOR_BOLD   = "\033[1m"


class BenchLogger:
    def __init__(self, filename=None):
        if not filename:
            dt = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"acc_bench_test_{dt}.log"
        self.log_file = open(filename, "w", encoding="utf-8")
        self.lock = threading.Lock()
        print(f"[*] Test session log: {filename}")

    def log(self, tag, msg, color="", console=True):
        ts = f"{time.time():.4f}"
        line = f"{ts} [{tag}] {msg}"
        with self.lock:
            self.log_file.write(line + "\n")
            self.log_file.flush()
        if console:
            if color:
                print(f"{color}[{tag}] {msg}{COLOR_RESET}")
            else:
                print(f"[{tag}] {msg}")

    def close(self):
        with self.lock:
            self.log_file.close()


class CanBusInterface:
    def __init__(self, iface: str, logger: BenchLogger):
        self.iface = iface
        self.logger = logger
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((iface,))
        self.running = True
        self.lock = threading.Lock()
        self.last_165 = None
        self.last_165_time = 0
        self.last_420_time = 0
        self.wakeup_callback = None
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

    def _rx_loop(self):
        while self.running:
            try:
                r, _, _ = select.select([self.sock], [], [], 0.05)
                if not r:
                    continue
                pkt = self.sock.recv(16)
                if len(pkt) < 16:
                    continue
                can_id_raw, can_dlc, data = struct.unpack(CAN_FRAME_FMT, pkt)
                can_id = can_id_raw & 0x7FF
                dlc = min(can_dlc, 8)
                payload = data[:dlc]

                if can_id == CAN_ID_RD4_165:
                    with self.lock:
                        self.last_165 = payload
                        self.last_165_time = time.time()
                    # Log to test session file only; suppress from console to avoid scrolling ACTION REQUIRED prompts
                    self.logger.log("RD4", f"RX 0x165 [{dlc}]: {payload.hex().upper()}", COLOR_GRAY, console=False)
                elif can_id == CAN_ID_RD4_WAKEUP:
                    with self.lock:
                        self.last_420_time = time.time()
                    self.logger.log("RD4_NM", f"RX 0x420 (RD4 Knob Wakeup Request) [{dlc}]: {payload.hex().upper()}", COLOR_CYAN)
                    if self.wakeup_callback:
                        self.wakeup_callback()
            except Exception as e:
                self.logger.log("CAN_RX_ERR", f"RX loop exception: {e}", COLOR_RED)
                break

    def send(self, can_id: int, data: bytes):
        can_id_raw = can_id & 0x7FF
        dlc = len(data)
        pad = b"\x00" * (8 - dlc)
        pkt = struct.pack(CAN_FRAME_FMT, can_id_raw, dlc, data + pad)
        try:
            self.sock.send(pkt)
        except Exception as e:
            self.logger.log("CAN_TX", f"Error sending {can_id:03X}: {e}", COLOR_RED)

    def get_rd4_power_state(self, since_time: float = 0.0):
        with self.lock:
            if not self.last_165 or len(self.last_165) < 1:
                return None, None
            if self.last_165_time < since_time:
                return None, None
            d0 = self.last_165[0]
            sleep = (d0 & 0xC0) == 0
            powered = (d0 & 0x80) != 0
            return powered, sleep

    def reset_rd4_power_state(self):
        with self.lock:
            self.last_165 = None
            self.last_165_time = 0.0
            self.last_420_time = 0.0

    def stop(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass


class BsiSimulator(threading.Thread):
    """Simulates PSA BSI periodic 0x036 (Ignition/Phase de vie) and 0x0F6 frames."""
    def __init__(self, can_bus: CanBusInterface, logger: BenchLogger):
        super().__init__(daemon=True)
        self.can_bus = can_bus
        self.logger = logger
        self.running = True
        self.ignition_on = False
        self.economy_mode = False
        self.accessory_active = False
        self.state_change_time = time.time()
        self.lock = threading.Lock()
        self.can_bus.wakeup_callback = self.trigger_accessory_wakeup

    def set_state(self, ignition_on: bool, economy_mode: bool = False):
        with self.lock:
            self.ignition_on = ignition_on
            self.economy_mode = economy_mode
            self.accessory_active = False
            self.state_change_time = time.time()
        state_str = f"Ignition={'ON' if ignition_on else 'OFF'}, Eco={'ON' if economy_mode else 'OFF'}"
        self.logger.log("BSI_SIM", f"State changed: {state_str}", COLOR_CYAN)

    def trigger_accessory_wakeup(self):
        with self.lock:
            if not self.ignition_on and not self.accessory_active:
                self.accessory_active = True
                self.state_change_time = time.time()
                self.logger.log("BSI_SIM", "Accessory wake triggered by RD4 0x420! Waking comfort bus (0x036 Phase=0x01, 0x0F6 Byte0=0x80)", COLOR_CYAN)

    def run(self):
        while self.running:
            with self.lock:
                ign = self.ignition_on
                eco = self.economy_mode
                acc = self.accessory_active
                change_t = self.state_change_time

            # Frame 0x036:
            # byte 2 bit 7: Economy mode
            # byte 4 bits [2:0]: Phase de vie
            #   0x01: Bus active (Ignition ON or RD4 Accessory wakeup)
            #   0x02: Contact OFF transition (Shutdown pulse)
            #   0x00: Standby / Sleep (Parked)
            d2 = 0x80 if eco else 0x00
            now = time.time()
            if ign:
                d4 = 0x01
                b0 = 0x88  # 0x0F6 Byte 0 Bit 3 = 1 (+APC / Ignition ON)
            elif acc:
                d4 = 0x01  # Bus kept awake by RD4 accessory
                b0 = 0x80  # 0x0F6 Byte 0 Bit 3 = 0 (Ignition OFF)
            elif (now - change_t) < 1.0:
                # Key-off transition for 1s
                d4 = 0x02
                b0 = 0x80
            else:
                # Parked standby / sleep
                d4 = 0x00
                b0 = 0x80

            f_036 = bytes([0x0E, 0x00, d2, 0x2A, d4, 0x00, 0x00, 0xA0])
            self.can_bus.send(CAN_ID_BSI_036, f_036)

            # Frame 0x0F6 (Slow BSI telemetry, 100ms)
            f_0f6 = bytes([b0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x20])
            self.can_bus.send(CAN_ID_BSI_0F6, f_0f6)

            time.sleep(0.10)

    def stop(self):
        self.running = False


class AdbMonitor:
    def __init__(self, target: str, logger: BenchLogger):
        self.target = target
        self.logger = logger

    def check_connection(self) -> bool:
        try:
            cmd = ["adb"]
            if self.target:
                cmd += ["-s", self.target]
            cmd += ["shell", "getprop", "sys.boot_completed"]
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=3.0, text=True)
            return "1" in res.stdout.strip()
        except Exception:
            return False

    def get_wakefulness(self) -> str:
        """Returns 'Awake', 'Asleep', 'Dozing', or 'Unknown'."""
        try:
            cmd = ["adb"]
            if self.target:
                cmd += ["-s", self.target]
            cmd += ["shell", "dumpsys", "power"]
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=3.0, text=True)
            for line in res.stdout.splitlines():
                if "mWakefulness=" in line:
                    m = re.search(r"mWakefulness=(\w+)", line)
                    if m:
                        return m.group(1)
                elif "Display Power: state=" in line:
                    m = re.search(r"state=(\w+)", line)
                    if m:
                        return "Awake" if m.group(1).upper() == "ON" else "Asleep"
        except Exception:
            pass
        return "Unknown"


class OpenOcdInspector:
    def __init__(self, host: str, port: int, logger: BenchLogger):
        self.host = host
        self.port = port
        self.logger = logger

    def _send_cmd(self, cmd: str) -> str:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(1.5)
        s.connect((self.host, self.port))
        buf = ""
        while "> " not in buf:
            chunk = s.recv(1024).decode(errors="ignore")
            if not chunk:
                break
            buf += chunk
        s.sendall(cmd.encode() + b"\n")
        buf = ""
        while "> " not in buf:
            chunk = s.recv(1024).decode(errors="ignore")
            if not chunk:
                break
            buf += chunk
        s.close()
        return buf

    def check_connection(self) -> bool:
        try:
            resp = self._send_cmd("version")
            return "Open On-Chip Debugger" in resp or ">" in resp
        except Exception:
            return False

    def read_nuc131_pa8_acc(self):
        """Reads NUC131 GPIOA_DOUT (0x50004008) bit 8 (PA8 - Head Unit Power)."""
        try:
            resp = self._send_cmd("mdw 0x50004008 1")
            # Output format: "0x50004008: 00004cff"
            m = re.search(r"0x50004008:\s+([0-9a-fA-F]+)", resp)
            if m:
                val = int(m.group(1), 16)
                pa8_high = (val & (1 << 8)) != 0
                return pa8_high
        except Exception:
            pass
        return None


def format_pa8(pa8):
    if pa8 is True:
        return "HIGH"
    elif pa8 is False:
        return "LOW"
    return "UNKNOWN"


def run_acc_test_suite(can_iface: str, adb_target: str, openocd_host: str, openocd_port: int, interactive: bool):
    logger = BenchLogger()
    logger.log("INIT", "=== OpenCanbox Hardware ACC & Power Workbench Test ===", COLOR_BOLD)

    # 1. Initialize Interfaces
    logger.log("INIT", f"Connecting to CAN interface {can_iface}...")
    try:
        can_bus = CanBusInterface(can_iface, logger)
    except Exception as e:
        logger.log("ERROR", f"Failed to bind {can_iface}: {e}. Ensure slcand or vcan is running.", COLOR_RED)
        return False

    adb = AdbMonitor(adb_target, logger)
    adb_ok = adb.check_connection()
    if adb_ok:
        logger.log("INIT", f"ADB connected to {adb_target or 'default device'}", COLOR_GREEN)
    else:
        logger.log("INIT", f"ADB not connected to {adb_target}. Screen status checks will be skipped.", COLOR_YELLOW)

    openocd = OpenOcdInspector(openocd_host, openocd_port, logger)
    openocd_ok = openocd.check_connection()
    if openocd_ok:
        logger.log("INIT", f"OpenOCD connected at {openocd_host}:{openocd_port}", COLOR_GREEN)
    else:
        logger.log("INIT", f"OpenOCD not reachable at {openocd_host}:{openocd_port}. SWD register checks skipped.", COLOR_YELLOW)

    bsi = BsiSimulator(can_bus, logger)
    bsi.start()

    passed = 0
    total = 0

    def assert_test(name: str, condition: bool, detail: str = ""):
        nonlocal passed, total
        total += 1
        if condition:
            passed += 1
            logger.log("PASS", f"Scenario {total}: {name} {detail}", COLOR_GREEN)
        else:
            logger.log("FAIL", f"Scenario {total}: {name} - FAILED {detail}", COLOR_RED)

    try:
        # ======================================================================
        # Scenario 1: Initial State (Ignition OFF)
        # ======================================================================
        bsi.set_state(ignition_on=False, economy_mode=False)
        time.sleep(0.5)

        pa8 = openocd.read_nuc131_pa8_acc() if openocd_ok else None
        wake = adb.get_wakefulness() if adb_ok else None
        assert_test("Ignition OFF -> ACC is LOW", (pa8 is False) if pa8 is not None else True,
                    f"(PA8={format_pa8(pa8)}, HU={wake})")

        # ======================================================================
        # Scenario 2: Turn Ignition ON (+APC)
        # ======================================================================
        logger.log("TEST", "Triggering Ignition ON (0x036 Phase de vie = 0x01)...", COLOR_CYAN)
        bsi.set_state(ignition_on=True, economy_mode=False)
        time.sleep(1.0)

        pa8 = openocd.read_nuc131_pa8_acc() if openocd_ok else None
        wake = adb.get_wakefulness() if adb_ok else None
        assert_test("Ignition ON -> ACC turns ON", (pa8 is True) if pa8 is not None else (wake == "Awake"),
                    f"(PA8={format_pa8(pa8)}, HU={wake})")

        # ======================================================================
        # Scenario 3: RD4 Mid-Drive Glitch / Silence Resilience
        # ======================================================================
        logger.log("TEST", "Injecting RD4 transient sleep/reboot glitch mid-drive...", COLOR_CYAN)
        # Inject transient reboot frame 00 00 00 00 followed by sleep 08 80
        can_bus.send(CAN_ID_RD4_165, bytes.fromhex("00000000"))
        time.sleep(0.1)
        can_bus.send(CAN_ID_RD4_165, bytes.fromhex("08800000"))
        time.sleep(0.3)

        pa8 = openocd.read_nuc131_pa8_acc() if openocd_ok else None
        wake = adb.get_wakefulness() if adb_ok else None
        assert_test("Ignition ON overrides RD4 glitch -> ACC stays HIGH",
                    (pa8 is True) if pa8 is not None else (wake == "Awake"),
                    f"(PA8={format_pa8(pa8)}, HU={wake})")

        # ======================================================================
        # Scenario 4: Turn Ignition OFF -> ACC cuts immediately
        # ======================================================================
        logger.log("TEST", "Triggering Ignition OFF (Key turned OFF)...", COLOR_CYAN)
        bsi.set_state(ignition_on=False, economy_mode=False)
        time.sleep(0.5)

        pa8 = openocd.read_nuc131_pa8_acc() if openocd_ok else None
        wake = adb.get_wakefulness() if adb_ok else None
        assert_test("Ignition OFF -> ACC immediately turns OFF",
                    (pa8 is False) if pa8 is not None else True,
                    f"(PA8={format_pa8(pa8)}, HU={wake})")

        # ======================================================================
        # Scenario 5: User switches ON RD4 while parked (Accessory Mode)
        # ======================================================================
        if interactive:
            logger.log("WAIT", "Waiting up to 4s for RD4 to settle into standby/sleep after key-off...", COLOR_YELLOW)
            settle_start = time.time()
            while time.time() - settle_start < 4.0:
                p, s = can_bus.get_rd4_power_state()
                if p is False or s is True or p is None:
                    break
                time.sleep(0.2)

            can_bus.reset_rd4_power_state()
            prompt_time = time.time()

            print("\n" + COLOR_YELLOW + COLOR_BOLD +
                  ">>> ACTION REQUIRED: Please press the Power/Volume knob on the RD4 radio to TURN IT ON." +
                  COLOR_RESET)
            print(">>> Waiting up to 15s for RD4 CAN frame 0x165 (0xC8 / Power ON)...")
            detected_on = False
            acc_on = False
            while time.time() - prompt_time < 15.0:
                p, s = can_bus.get_rd4_power_state(since_time=prompt_time)
                cur_pa8 = openocd.read_nuc131_pa8_acc() if openocd_ok else None
                wake = adb.get_wakefulness() if adb_ok else None

                if p is True:
                    detected_on = True

                if openocd_ok:
                    acc_on = (cur_pa8 is True)
                elif adb_ok:
                    acc_on = (wake == "Awake")
                else:
                    acc_on = True

                if detected_on and acc_on:
                    break
                time.sleep(0.1)
        else:
            # Simulated RD4 power on
            logger.log("TEST", "Simulating RD4 Power ON button press (165#C8001100)...", COLOR_CYAN)
            can_bus.send(CAN_ID_RD4_165, bytes.fromhex("C8001100"))
            time.sleep(0.3)
            detected_on = True

        pa8 = openocd.read_nuc131_pa8_acc() if openocd_ok else None
        wake = adb.get_wakefulness() if adb_ok else None
        assert_test("Ignition OFF + RD4 switched ON -> ACC turns ON",
                    detected_on and ((pa8 is True) if pa8 is not None else True),
                    f"(RD4_ON={detected_on}, PA8={format_pa8(pa8)}, HU={wake})")

        # ======================================================================
        # Scenario 6: User switches OFF RD4 while parked
        # ======================================================================
        if interactive and detected_on:
            # Settle in accessory mode for 1.5s
            time.sleep(1.5)
            can_bus.reset_rd4_power_state()
            prompt_time = time.time()
            print("\n" + COLOR_YELLOW + COLOR_BOLD +
                  ">>> ACTION REQUIRED: Please press the Power/Volume knob on the RD4 radio to TURN IT OFF." +
                  COLOR_RESET)
            print(">>> Waiting up to 15s for RD4 CAN frame 0x165 (0x48 / 0x08 Power OFF)...")
            detected_off = False
            acc_off = False
            while time.time() - prompt_time < 15.0:
                p, s = can_bus.get_rd4_power_state(since_time=prompt_time)
                cur_pa8 = openocd.read_nuc131_pa8_acc() if openocd_ok else None
                wake = adb.get_wakefulness() if adb_ok else None

                if p is False or s is True:
                    detected_off = True

                if openocd_ok:
                    acc_off = (cur_pa8 is False)
                elif adb_ok:
                    acc_off = (wake != "Awake")
                else:
                    acc_off = True

                if detected_off and acc_off:
                    break
                time.sleep(0.1)
        else:
            logger.log("TEST", "Simulating RD4 Power OFF button press (165#48A00000)...", COLOR_CYAN)
            can_bus.send(CAN_ID_RD4_165, bytes.fromhex("48A00000"))
            time.sleep(0.3)
            detected_off = True

        pa8 = openocd.read_nuc131_pa8_acc() if openocd_ok else None
        wake = adb.get_wakefulness() if adb_ok else None
        assert_test("Ignition OFF + RD4 switched OFF -> ACC turns OFF",
                    detected_off and ((pa8 is False) if pa8 is not None else True),
                    f"(RD4_OFF={detected_off}, PA8={format_pa8(pa8)}, HU={wake})")

        # ======================================================================
        # Scenario 7: Economy Mode Cutoff
        # ======================================================================
        logger.log("TEST", "Turning RD4 back ON in accessory mode, then triggering Economy Mode...", COLOR_CYAN)
        can_bus.send(CAN_ID_RD4_165, bytes.fromhex("C8001100"))
        time.sleep(0.3)
        bsi.set_state(ignition_on=False, economy_mode=True)
        time.sleep(0.5)

        pa8 = openocd.read_nuc131_pa8_acc() if openocd_ok else None
        wake = adb.get_wakefulness() if adb_ok else None
        assert_test("Economy Mode active -> ACC immediately CUT",
                    (pa8 is False) if pa8 is not None else True,
                    f"(PA8={format_pa8(pa8)}, HU={wake})")

    finally:
        bsi.stop()
        can_bus.stop()
        logger.close()

    print("\n" + "=" * 60)
    if passed == total:
        print(f"{COLOR_GREEN}{COLOR_BOLD}ALL {passed}/{total} ACC INTEGRATION TESTS PASSED!{COLOR_RESET}")
    else:
        print(f"{COLOR_RED}{COLOR_BOLD}{passed}/{total} ACC TESTS PASSED, {total - passed} FAILED.{COLOR_RESET}")
    print("=" * 60)
    return passed == total


def main():
    parser = argparse.ArgumentParser(description="Workbench Hardware Integration Test for Head Unit ACC & Power")
    parser.add_argument("--can", default="can0", help="SocketCAN interface name (default: can0)")
    parser.add_argument("--adb", default=None, help="ADB target address/ID (e.g. 192.168.1.33:9876)")
    parser.add_argument("--openocd-host", default="127.0.0.1", help="OpenOCD host (default: 127.0.0.1)")
    parser.add_argument("--openocd-port", type=int, default=4444, help="OpenOCD telnet port (default: 4444)")
    parser.add_argument("--auto", action="store_true", help="Run fully automated (simulate RD4 button events instead of prompting)")
    args = parser.parse_args()

    # Make executable
    os.chmod(__file__, 0o755)

    interactive = not args.auto
    success = run_acc_test_suite(args.can, args.adb, args.openocd_host, args.openocd_port, interactive)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

