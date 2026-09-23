#!/usr/bin/env python3
"""
OpenCanbox Core - Workbench Hardware-in-the-Loop (HIL) E2E Test Runner
======================================================================
Facilitates manual end-to-end testing between a real vehicle bench
(BSI, BMS, cluster console, speed, RPM, steering wheel stalk/keys)
connected to Linux via SocketCAN (can0 / slcan) and an Android Head Unit
connected via physical USB-to-UART (/dev/ttyUSB1 @ 38400 8N1).

Default Configuration:
- CAN Interface     : can0
- Head Unit UART    : /dev/ttyUSB1
- Baud Rate         : 38400 (8N1)
- HU Protocol       : hiworld (or raise / bagoo)
- Vehicle Profile   : psa (Peugeot 407 / PSA 2004)
"""

import argparse
import curses
import glob
import os
import queue
import select
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

# SocketCAN constants
CAN_RAW = 1
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
NATIVE_BINARY_PATH = os.path.join(PROJECT_ROOT, ".pio", "build", "native_test", "program")


def find_serial_ports():
    """Scans system for connected USB-to-UART serial adapters."""
    return sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))


def is_can_interface_up(iface: str) -> bool:
    """Checks if CAN network interface is UP in sysfs."""
    operstate_path = f"/sys/class/net/{iface}/operstate"
    if os.path.exists(operstate_path):
        try:
            with open(operstate_path, "r") as f:
                state = f.read().strip()
                return state in ("up", "unknown")
        except Exception:
            return True
    return os.path.exists(f"/sys/class/net/{iface}")


def setup_slcan(slcan_dev: str, iface: str = "can0", slcan_speed: str = "s6", uart_baud: int = 1000000) -> bool:
    """
    Sets up SLCAN socketcan interface using:
    sudo slcan_attach -f -<speed> -o <slcan_dev>
    sudo slcand -S <uart_baud> <slcan_dev> <iface>
    sudo ip link set dev <iface> up
    """
    speed_flag = f"-{slcan_speed.lstrip('-')}"
    print(f"[*] Initializing SLCAN adapter on {slcan_dev} -> {iface} (CAN Speed: {speed_flag}, UART Baud: {uart_baud})...")

    attach_cmd = ["sudo", "slcan_attach", "-f", speed_flag, "-o", slcan_dev]
    slcand_cmd = ["sudo", "slcand", "-S", str(uart_baud), slcan_dev, iface]
    iplink_cmd = ["sudo", "ip", "link", "set", "dev", iface, "up"]

    try:
        # Run slcan_attach
        print(f"    [1/3] Running: {' '.join(attach_cmd)}")
        subprocess.run(attach_cmd, check=True)
        time.sleep(0.2)

        # Run slcand
        print(f"    [2/3] Running: {' '.join(slcand_cmd)}")
        subprocess.run(slcand_cmd, check=True)
        time.sleep(0.3)

        # Bring interface UP
        print(f"    [3/3] Running: {' '.join(iplink_cmd)}")
        subprocess.run(iplink_cmd, check=True)
        time.sleep(0.2)

        if is_can_interface_up(iface):
            print(f"[OK] CAN interface '{iface}' successfully attached and UP.")
            return True
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Failed to setup SLCAN on {slcan_dev}: {e}")
    except FileNotFoundError as e:
        print(f"[ERROR] Required tool not found in PATH: {e}")
        print("        Ensure can-utils is installed (sudo apt install can-utils)")
    return False


def teardown_slcan(iface: str = "can0"):
    """Brings down the CAN interface and terminates slcand."""
    print(f"[*] Tearing down SLCAN interface {iface}...")
    try:
        subprocess.run(["sudo", "ip", "link", "set", "dev", iface, "down"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(["sudo", "killall", "slcand"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"[OK] SLCAN interface {iface} closed.")
    except Exception:
        pass


def ensure_app_built() -> bool:
    """Ensures the native desktop binary is compiled."""
    if os.path.isfile(NATIVE_BINARY_PATH) and os.access(NATIVE_BINARY_PATH, os.X_OK):
        return True

    print("[*] OpenCanbox Core native binary not found. Compiling 'native_test' target...")
    pio_path = os.path.expanduser("~/.platformio/penv/bin/pio")
    pio_cmd = pio_path if os.path.isfile(pio_path) else "pio"

    try:
        subprocess.check_call(
            [pio_cmd, "run", "-e", "native_test"],
            cwd=PROJECT_ROOT,
        )
        print("[OK] Native binary compiled successfully.")
        return True
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        print(f"[ERROR] Failed to compile native binary: {e}")
        return False


def parse_can_frame(raw_bytes):
    """Unpacks Linux struct can_frame (16 bytes)."""
    can_id_raw, can_dlc, data = struct.unpack("=IB3x8s", raw_bytes)
    is_extended = (can_id_raw & CAN_EFF_FLAG) != 0
    is_remote = (can_id_raw & CAN_RTR_FLAG) != 0
    can_id = can_id_raw & (CAN_EFF_MASK if is_extended else CAN_SFF_MASK)
    dlc = min(can_dlc, 8)
    return can_id, is_extended, is_remote, list(data[:dlc])


class BenchStateDecoder:
    """Decodes automotive CAN frames from the PSA / Peugeot bench in real time."""

    def __init__(self):
        self.lock = threading.Lock()
        self.speed_kmh = 0
        self.rpm = 0
        self.steering_angle_deg = 0
        self.active_stalk_key = "NONE"
        self.stalk_pressed = False
        self.wheel_btn = "NONE"
        self.driver_door = False
        self.passenger_door = False
        self.rear_left_door = False
        self.rear_right_door = False
        self.trunk = False
        self.hood = False
        self.handbrake = False
        self.reverse_gear = False
        self.side_light = False
        self.headlights = False
        self.high_beam = False
        self.front_fog = False
        self.rear_fog = False
        self.hvac_power = False
        self.hvac_ac = False
        self.hvac_auto = False
        self.hvac_recirc = False
        self.hvac_fan_speed = 0
        self.hvac_temp_driver = 0
        self.hvac_temp_pass = 0
        self.radar_rear = [0, 0, 0]  # RL, RC, RR
        self.radar_front = [0, 0, 0] # FL, FC, FR
        self.prev_stalk_b0 = 0
        self.prev_stalk_b1 = 0
        self.prev_stalk_b2 = 0
        self.recent_events = []
        self.total_frames = 0
        self.can_id_counts = {}

    def log_event(self, text: str):
        ts_str = time.strftime("%H:%M:%S")
        with self.lock:
            self.recent_events.append(f"[{ts_str}] {text}")
            if len(self.recent_events) > 10:
                self.recent_events.pop(0)

    def clear_events(self):
        with self.lock:
            self.recent_events.clear()

    def process_can_frame(self, can_id: int, dlc: int, data: list):
        with self.lock:
            self.total_frames += 1
            self.can_id_counts[can_id] = self.can_id_counts.get(can_id, 0) + 1

        # 0x0B6: Engine RPM and Speed (PSA CAN2004 / CAN2010)
        if can_id == 0x0B6 and dlc >= 4:
            rpm_raw = (data[0] << 8) | data[1]
            speed_raw = (data[2] << 8) | data[3]
            rpm = rpm_raw >> 3
            speed = speed_raw >> 7
            with self.lock:
                self.rpm = rpm
                self.speed_kmh = speed

        # 0x280: Engine RPM (VAG TP2.0 / MQB)
        elif can_id == 0x280 and dlc >= 4:
            rpm = ((data[3] << 8) | data[2]) // 4
            with self.lock:
                self.rpm = rpm

        # 0x5A0 / 0x320: Vehicle Speed (VAG)
        elif can_id == 0x5A0 and dlc >= 2:
            speed = data[1]
            with self.lock:
                self.speed_kmh = speed
        elif can_id == 0x320 and dlc >= 3:
            speed = ((data[2] << 8) | data[1]) // 100
            with self.lock:
                self.speed_kmh = speed

        # 0x0C2: Steering Wheel Angle (VAG)
        elif can_id == 0x0C2 and dlc >= 2:
            raw_angle = (data[1] << 8) | data[0]
            if raw_angle >= 0x8000:
                raw_angle -= 0x10000
            with self.lock:
                self.steering_angle_deg = raw_angle // 10

        # 0x5C0: Steering Wheel Buttons (VAG)
        elif can_id == 0x5C0 and dlc >= 1:
            btn = data[0]
            btn_map = {0x01: "VOL_UP", 0x02: "VOL_DOWN", 0x03: "NEXT", 0x04: "PREV", 0x06: "SRC", 0x07: "MUTE"}
            btn_str = btn_map.get(btn, "NONE" if btn == 0 else f"BTN_0x{btn:02X}")
            with self.lock:
                if btn_str != self.wheel_btn:
                    if btn_str != "NONE":
                        self.log_event(f"VAG Wheel Key: {btn_str}")
                    self.wheel_btn = btn_str

        # 0x470: Doors Status (VAG)
        elif can_id == 0x470 and dlc >= 1:
            d0 = data[0]
            with self.lock:
                self.driver_door = bool(d0 & 0x01)
                self.passenger_door = bool(d0 & 0x02)
                self.rear_left_door = bool(d0 & 0x04)
                self.rear_right_door = bool(d0 & 0x08)
                self.trunk = bool(d0 & 0x10)
                self.hood = bool(d0 & 0x20)

        # 0x0E6 / 0x0E8: Steering Wheel Angle (PSA)
        elif can_id in (0x0E6, 0x0E8) and dlc >= 2:
            raw_angle = (data[0] << 8) | data[1]
            if raw_angle >= 0x8000:
                raw_angle -= 0x10000
            angle = int(raw_angle / 10)
            with self.lock:
                self.steering_angle_deg = angle

        # 0x0F6: Stalk Column Buttons (PSA)
        elif can_id == 0x0F6 and dlc >= 2:
            b0 = data[0]
            b1 = data[1]
            b2 = data[2] if dlc >= 3 else 0

            key_name = "NONE"
            pressed = False

            if (b0 & 0x08): key_name, pressed = "VOL_UP", True
            elif (b0 & 0x04): key_name, pressed = "VOL_DOWN", True
            elif (b0 & 0x02): key_name, pressed = "NEXT", True
            elif (b0 & 0x01): key_name, pressed = "PREV", True
            elif (b0 & 0x40): key_name, pressed = "SRC", True
            elif (b0 & 0x10): key_name, pressed = "OK", True
            elif (b0 & 0x80): key_name, pressed = "DARK", True
            elif (b0 & 0x20): key_name, pressed = "ESC", True
            elif (b1 & 0x40): key_name, pressed = "MENU", True
            elif (b2 & 0x01): key_name, pressed = "TEL_ANSWER", True
            elif (b2 & 0x02): key_name, pressed = "TEL_HANGUP", True

            # Scroll delta
            scroll_curr = b1 & 0x0F
            scroll_prev = self.prev_stalk_b1 & 0x0F
            delta = scroll_curr - scroll_prev
            if (0 < delta <= 7) or delta < -7:
                self.log_event("Stalk Scroll: UP")
            elif (delta < 0 and delta >= -7) or delta > 7:
                self.log_event("Stalk Scroll: DOWN")

            with self.lock:
                if pressed and (self.active_stalk_key != key_name or not self.stalk_pressed):
                    self.log_event(f"Stalk Button PRESSED: {key_name}")
                elif not pressed and self.stalk_pressed:
                    self.log_event(f"Stalk Button RELEASED: {self.active_stalk_key}")
                self.active_stalk_key = key_name
                self.stalk_pressed = pressed
                self.prev_stalk_b0 = b0
                self.prev_stalk_b1 = b1
                self.prev_stalk_b2 = b2

        # 0x128: Steering Wheel Buttons & Lighting Status (PSA)
        elif can_id == 0x128 and dlc >= 1:
            btn = data[0]
            btn_map = {
                0x01: "VOL_UP",
                0x02: "VOL_DOWN",
                0x04: "NEXT",
                0x08: "PREV",
                0x10: "SRC",
                0x20: "MUTE",
            }
            btn_str = btn_map.get(btn, "NONE" if btn == 0 else f"BTN_0x{btn:02X}")
            with self.lock:
                if btn_str != self.wheel_btn:
                    if btn_str != "NONE":
                        self.log_event(f"Wheel Key: {btn_str}")
                    self.wheel_btn = btn_str

                if dlc >= 5:
                    flags = data[4]
                    self.side_light = bool(flags & (1 << 7))
                    self.headlights = bool(flags & (1 << 6))
                    self.high_beam  = bool(flags & (1 << 5))
                    self.front_fog  = bool(flags & (1 << 4))
                    self.rear_fog   = bool(flags & (1 << 3))

        # 0x036: Doors, Ignition, Reverse Gear & Handbrake (PSA)
        elif can_id == 0x036 and dlc >= 3:
            d0 = data[0]
            d1 = data[1]
            with self.lock:
                prev_rev = self.reverse_gear
                self.driver_door     = bool(d0 & (1 << 0))
                self.passenger_door  = bool(d0 & (1 << 1))
                self.rear_left_door  = bool(d0 & (1 << 2))
                self.rear_right_door = bool(d0 & (1 << 3))
                self.trunk           = bool(d0 & (1 << 4))
                self.hood            = bool(d0 & (1 << 5))
                self.reverse_gear    = bool(d1 & (1 << 7))
                self.handbrake       = bool(d1 & (1 << 0))
                if self.reverse_gear != prev_rev:
                    self.log_event(f"Reverse Gear: {'ENGAGED (R)' if self.reverse_gear else 'DISENGAGED'}")

        # 0x221: Extended Body & Doors status (PSA)
        elif can_id == 0x221 and dlc >= 1:
            d0 = data[0]
            with self.lock:
                self.driver_door     = bool(d0 & 0x80)
                self.passenger_door  = bool(d0 & 0x40)
                self.rear_left_door  = bool(d0 & 0x20)
                self.rear_right_door = bool(d0 & 0x10)
                self.trunk           = bool(d0 & 0x08)
                self.hood            = bool(d0 & 0x04)
                if dlc >= 2:
                    self.handbrake   = bool(data[1] & 0x01)

        # 0x1D0: Climate Control (HVAC) (PSA)
        elif can_id == 0x1D0 and dlc >= 4:
            with self.lock:
                self.hvac_power       = bool(data[0] & 0x80)
                self.hvac_ac          = bool(data[0] & 0x40)
                self.hvac_recirc      = bool(data[0] & 0x20)
                self.hvac_auto        = bool(data[0] & 0x08)
                self.hvac_fan_speed   = data[1] & 0x0F
                self.hvac_temp_driver = data[2]
                self.hvac_temp_pass   = data[3]

        # 0x260: Ultrasonic Rear Parking Sensors (PSA)
        elif can_id == 0x260 and dlc >= 3:
            with self.lock:
                self.radar_rear = [data[0], data[1], data[2]]

        # 0x270: Ultrasonic Front Parking Sensors (PSA)
        elif can_id == 0x270 and dlc >= 3:
            with self.lock:
                self.radar_front = [data[0], data[1], data[2]]


class SocketCanListener(threading.Thread):
    """Background listener on SocketCAN interface."""

    def __init__(self, iface: str, decoder: BenchStateDecoder, record_fp=None):
        super().__init__(daemon=True)
        self.iface = iface
        self.decoder = decoder
        self.record_fp = record_fp
        self.running = True
        self.sock = None
        self.frame_rate = 0
        self._fps_counter = 0
        self._fps_time = time.time()

    def run(self):
        try:
            self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, CAN_RAW)
            self.sock.bind((self.iface,))
            self.sock.setblocking(False)
        except OSError as e:
            self.decoder.log_event(f"SocketCAN Error: {e}")
            return

        while self.running:
            try:
                r, _, _ = select.select([self.sock], [], [], 0.05)
                if not r:
                    continue

                # Drain all available frames from the non-blocking socket buffer
                while self.running:
                    try:
                        raw_frame = self.sock.recv(16)
                    except (BlockingIOError, socket.error):
                        break

                    if not raw_frame or len(raw_frame) != 16:
                        break

                    can_id, is_ext, is_rtr, data = parse_can_frame(raw_frame)
                    self.decoder.process_can_frame(can_id, len(data), data)
                    self._fps_counter += 1

                    if self.record_fp:
                        ts = time.time()
                        data_hex = [f"{b:02X}" for b in data]
                        padded = data_hex + [""] * (8 - len(data_hex))
                        row = [f"{ts:.6f}", f"{can_id:X}", str(is_ext), "0", "0", str(len(data))] + padded
                        self.record_fp.write(",".join(row) + "\n")

                now = time.time()
                if now - self._fps_time >= 1.0:
                    self.frame_rate = self._fps_counter / (now - self._fps_time)
                    self._fps_counter = 0
                    self._fps_time = now

            except Exception as e:
                if self.running:
                    time.sleep(0.01)

        if self.sock:
            self.sock.close()

    def stop(self):
        self.running = False


def safe_addstr(win, y, x, text, attr=0):
    """Safely adds a string to curses window without exceeding window bounds."""
    max_y, max_x = win.getmaxyx()
    if 0 <= y < max_y and 0 <= x < max_x:
        avail_len = max_x - x - 1
        if avail_len > 0:
            try:
                win.addstr(y, x, text[:avail_len], attr)
            except curses.error:
                pass


def curses_dashboard_loop(stdscr, decoder: BenchStateDecoder, can_listener: SocketCanListener, args, app_proc):
    """Renders real-time in-place Curses terminal dashboard with zero scrolling."""
    try:
        curses.curs_set(0)
    except Exception:
        pass
    stdscr.nodelay(True)
    stdscr.timeout(100) # 100ms refresh rate

    has_colors = curses.has_colors()
    if has_colors:
        try:
            curses.start_color()
        except Exception:
            pass

        try:
            curses.use_default_colors()
            bg = -1
        except Exception:
            bg = curses.COLOR_BLACK

        try:
            curses.init_pair(1, curses.COLOR_CYAN, bg)   # Section headers
            curses.init_pair(2, curses.COLOR_GREEN, bg)  # Status OK / Active
            curses.init_pair(3, curses.COLOR_YELLOW, bg) # Dynamic values
            curses.init_pair(4, curses.COLOR_RED, bg)    # Alerts / Reverse
            curses.init_pair(5, curses.COLOR_WHITE, curses.COLOR_BLUE if curses.can_change_color() else bg) # Top Banner
            curses.init_pair(6, curses.COLOR_MAGENTA, bg)
        except Exception:
            has_colors = False

    ATTR_HEADER = (curses.color_pair(1) | curses.A_BOLD) if has_colors else curses.A_BOLD
    ATTR_OK     = (curses.color_pair(2) | curses.A_BOLD) if has_colors else curses.A_BOLD
    ATTR_VAL    = (curses.color_pair(3) | curses.A_BOLD) if has_colors else 0
    ATTR_ALERT  = (curses.color_pair(4) | curses.A_BOLD) if has_colors else curses.A_REVERSE
    ATTR_BANNER = (curses.color_pair(5) | curses.A_BOLD) if has_colors else curses.A_REVERSE
    ATTR_EVENT  = curses.A_NORMAL

    while True:
        # Check input keys
        try:
            ch = stdscr.getch()
            if ch in (ord('q'), ord('Q'), 27): # 'q' or ESC
                break
            elif ch in (ord('c'), ord('C')):
                decoder.clear_events()
        except Exception:
            pass

        stdscr.erase()
        max_y, max_x = stdscr.getmaxyx()

        if max_y < 10 or max_x < 30:
            safe_addstr(stdscr, 0, 0, "Terminal window too small.", curses.A_BOLD)
            stdscr.refresh()
            time.sleep(0.1)
            continue

        # Format Status Strings
        status_app = f"RUNNING (PID: {app_proc.pid})" if (app_proc and app_proc.poll() is None) else "OFFLINE"
        can_fps = f"{can_listener.frame_rate:.1f} fps" if can_listener else "0.0 fps"

        with decoder.lock:
            speed = decoder.speed_kmh
            rpm = decoder.rpm
            angle = decoder.steering_angle_deg
            stalk_key = f"{decoder.active_stalk_key} (PRESSED)" if decoder.stalk_pressed else (decoder.wheel_btn if decoder.wheel_btn != "NONE" else "IDLE")
            
            # Doors
            doors_open = []
            if decoder.driver_door:     doors_open.append("Driver")
            if decoder.passenger_door:  doors_open.append("Passenger")
            if decoder.rear_left_door:  doors_open.append("Rear-Left")
            if decoder.rear_right_door: doors_open.append("Rear-Right")
            if decoder.trunk:           doors_open.append("Trunk")
            if decoder.hood:            doors_open.append("Hood")
            doors_str = ", ".join(doors_open) if doors_open else "All Closed"

            rev_active = decoder.reverse_gear
            rev_str = "REVERSE (R) ON" if rev_active else "FORWARD / NEUTRAL"
            brake_str = "ENGAGED" if decoder.handbrake else "RELEASED"

            # Lights
            lights_on = []
            if decoder.side_light: lights_on.append("Side")
            if decoder.headlights: lights_on.append("Dipped/Low")
            if decoder.high_beam:  lights_on.append("HighBeam")
            if decoder.front_fog:  lights_on.append("FrontFog")
            if decoder.rear_fog:   lights_on.append("RearFog")
            lights_str = ", ".join(lights_on) if lights_on else "OFF"

            # Climate
            def fmt_temp(t):
                if t == 0x00: return "LO"
                if t == 0xFF: return "HI"
                return f"{t * 0.5:.1f}°C"

            hvac_str = (
                f"Power: {'ON' if decoder.hvac_power else 'OFF'} | "
                f"AC: {'ON' if decoder.hvac_ac else 'OFF'} | "
                f"Auto: {'ON' if decoder.hvac_auto else 'OFF'} | "
                f"Fan: {decoder.hvac_fan_speed}/8 | "
                f"L: {fmt_temp(decoder.hvac_temp_driver)} R: {fmt_temp(decoder.hvac_temp_pass)}"
            )

            radar_str = (
                f"Rear: [L:{decoder.radar_rear[0]} C:{decoder.radar_rear[1]} R:{decoder.radar_rear[2]}] | "
                f"Front: [L:{decoder.radar_front[0]} C:{decoder.radar_front[1]} R:{decoder.radar_front[2]}]"
            )

            total_pkts = decoder.total_frames
            events_copy = list(decoder.recent_events)

        # Draw Title Banner
        banner_text = " OPENCANBOX CORE - WORKBENCH HARDWARE-IN-THE-LOOP (HIL) "
        safe_addstr(stdscr, 0, 0, banner_text.center(max_x - 1), ATTR_BANNER)

        # Draw Interface Status
        safe_addstr(stdscr, 1, 1, f"CAN Interface : {args.can_iface:<10}", ATTR_HEADER)
        safe_addstr(stdscr, 1, 26, f"Status: UP  |  Traffic: {can_fps:<8}  |  Total Frames: {total_pkts}", ATTR_OK)

        safe_addstr(stdscr, 2, 1, f"HU Serial Port: {args.hu_port:<10}", ATTR_HEADER)
        safe_addstr(stdscr, 2, 26, f"Baud: {args.hu_baud} 8N1  |  Protocol: {args.protocol.upper()}", ATTR_VAL)

        safe_addstr(stdscr, 3, 1, f"Vehicle Profile: {args.profile.upper():<9}", ATTR_HEADER)
        safe_addstr(stdscr, 3, 26, f"Core App: {status_app}", ATTR_OK if "RUNNING" in status_app else ATTR_ALERT)

        sep_line = "─" * (max_x - 2)
        safe_addstr(stdscr, 4, 0, sep_line, curses.A_DIM)

        # Section 1: Telemetry
        safe_addstr(stdscr, 5, 1, "[1] TELEMETRY & CLUSTER", ATTR_HEADER)
        safe_addstr(stdscr, 6, 5, f"Speed          : {speed:3d} km/h", ATTR_VAL)
        safe_addstr(stdscr, 6, 38, f"Engine RPM      : {rpm:4d} RPM", ATTR_VAL)
        safe_addstr(stdscr, 7, 5, f"Steering Angle : {angle:+4d}°", ATTR_VAL)
        safe_addstr(stdscr, 7, 38, f"Active Key/Stalk: {stalk_key}", ATTR_ALERT if decoder.stalk_pressed else ATTR_VAL)

        # Section 2: Body & Doors
        safe_addstr(stdscr, 8, 1, "[2] BODY & DOORS", ATTR_HEADER)
        safe_addstr(stdscr, 9, 5, f"Doors Status   : {doors_str}", ATTR_ALERT if doors_open else ATTR_VAL)
        safe_addstr(stdscr, 10, 5, f"Reverse Gear   : {rev_str:<18}", ATTR_ALERT if rev_active else ATTR_VAL)
        safe_addstr(stdscr, 10, 38, f"Handbrake       : {brake_str}", ATTR_VAL)

        # Section 3: Lights & HVAC
        safe_addstr(stdscr, 11, 1, "[3] LIGHTS & CLIMATE", ATTR_HEADER)
        safe_addstr(stdscr, 12, 5, f"Lights         : {lights_str}", ATTR_VAL)
        safe_addstr(stdscr, 13, 5, f"HVAC (Climate) : {hvac_str}", ATTR_VAL)

        # Section 4: Parking Radar
        safe_addstr(stdscr, 14, 1, "[4] PARKING RADAR SENSORS", ATTR_HEADER)
        safe_addstr(stdscr, 15, 5, f"Sensors (0=Far, 6=Close): {radar_str}", ATTR_VAL)

        safe_addstr(stdscr, 16, 0, sep_line, curses.A_DIM)

        # Section 5: Recent Events Log
        safe_addstr(stdscr, 17, 1, "[RECENT BENCH EVENTS / CAN & HU PROTOCOL ACTIVITY]", ATTR_HEADER)
        row_idx = 18
        max_event_rows = min(6, max_y - 21) if max_y > 21 else 3
        slice_events = events_copy[-max_event_rows:] if events_copy else []
        for evt in slice_events:
            safe_addstr(stdscr, row_idx, 3, f"> {evt}", ATTR_EVENT)
            row_idx += 1
        if not slice_events:
            safe_addstr(stdscr, row_idx, 3, "(Listening for live CAN bench traffic...)", curses.A_DIM)
            row_idx += 1

        # Footer Controls
        footer_y = max(row_idx + 1, min(max_y - 2, 25))
        safe_addstr(stdscr, footer_y - 1, 0, sep_line, curses.A_DIM)
        safe_addstr(stdscr, footer_y, 1, " Controls: [q] Exit Bench  |  [c] Clear Events Log  |  [Ctrl+C] Quit", ATTR_HEADER)

        stdscr.refresh()
        time.sleep(0.08)


def main():
    parser = argparse.ArgumentParser(
        description="OpenCanbox Core - Automotive Workbench HIL Manual Testing Harness"
    )
    parser.add_argument(
        "--can-iface", "-i",
        default="can0",
        help="CAN network interface name (default: can0)"
    )
    parser.add_argument(
        "--hu-port", "-p",
        default="/dev/ttyUSB1",
        help="Path to Head Unit USB-to-UART serial port (default: /dev/ttyUSB1)"
    )
    parser.add_argument(
        "--hu-baud", "-b",
        type=int,
        default=38400,
        help="Head Unit serial baud rate (default: 38400 for Hiworld / Raise)"
    )
    parser.add_argument(
        "--protocol", "-P",
        default="hiworld",
        choices=["hiworld", "raise", "bagoo"],
        help="Head Unit protocol dialect (default: hiworld)"
    )
    parser.add_argument(
        "--profile", "-M",
        default="psa",
        choices=["psa", "vag"],
        help="Vehicle CAN profile decoding rules (default: psa)"
    )
    parser.add_argument(
        "--setup-slcan", "-s",
        action="store_true",
        help="Automatically attach and bring up SLCAN adapter on --slcan-dev"
    )
    parser.add_argument(
        "--slcan-dev",
        default=None,
        help="USB serial device of SLCAN adapter (e.g. /dev/ttyUSB0)"
    )
    parser.add_argument(
        "--slcan-speed",
        default="s6",
        help="SLCAN CAN speed flag (default: s6 for 500k, s5 for 250k, s4 for 125k)"
    )
    parser.add_argument(
        "--slcan-uart-baud",
        type=int,
        default=1000000,
        help="UART baud rate between host and SLCAN dongle (default: 1000000)"
    )
    parser.add_argument(
        "--teardown-slcan",
        action="store_true",
        help="Tear down SLCAN interface (ip link set can0 down) upon script exit"
    )
    parser.add_argument(
        "--no-app",
        action="store_true",
        help="Do not spawn OpenCanbox Core desktop process (run in monitor-only mode)"
    )
    parser.add_argument(
        "--record-can",
        default=None,
        help="File path to record live incoming CAN traffic to CSV"
    )
    parser.add_argument(
        "--stream",
        action="store_true",
        help="Output raw streaming logs instead of in-place Curses dashboard"
    )

    args = parser.parse_args()

    # 1. SLCAN Setup if requested
    if args.setup_slcan or args.slcan_dev:
        slcan_port = args.slcan_dev
        if not slcan_port:
            ports = find_serial_ports()
            candidate_ports = [p for p in ports if p != args.hu_port]
            if candidate_ports:
                slcan_port = candidate_ports[0]
            elif ports:
                slcan_port = ports[0]
            else:
                print("[ERROR] No USB serial adapters found for SLCAN.")
                sys.exit(1)
        
        setup_slcan(slcan_port, iface=args.can_iface, slcan_speed=args.slcan_speed, uart_baud=args.slcan_uart_baud)

    # 2. Check CAN Interface
    if not is_can_interface_up(args.can_iface):
        print(f"[WARN] CAN interface '{args.can_iface}' is not currently UP.")
        print(f"       If using SLCAN, run with: --setup-slcan --slcan-dev /dev/ttyUSB0")
        print(f"       Or bring up manually:")
        print(f"         sudo slcan_attach -f -s6 -o /dev/ttyUSB0")
        print(f"         sudo slcand -S 1000000 /dev/ttyUSB0 {args.can_iface}")
        print(f"         sudo ip link set dev {args.can_iface} up\n")

    # 3. Check Head Unit Serial Port
    if not os.path.exists(args.hu_port):
        available_ports = find_serial_ports()
        print(f"[WARN] Target Head Unit serial port '{args.hu_port}' was not found.")
        if available_ports:
            print(f"       Available serial adapters: {', '.join(available_ports)}")
            print(f"       You can specify with: --hu-port {available_ports[0]}")
        else:
            print("       No USB serial adapters detected. Please plug in the Head Unit USB-UART adapter.")

    # 4. Compile & Launch OpenCanbox Core desktop simulator
    app_proc = None
    if not args.no_app:
        if not ensure_app_built():
            sys.exit(1)

        app_env = os.environ.copy()
        app_env["CANBOX_CAN_IFACE"] = args.can_iface
        app_env["CANBOX_HU_PROTOCOL"] = args.protocol
        app_env["CANBOX_VEHICLE_PROFILE"] = args.profile
        if os.path.exists(args.hu_port):
            app_env["CANBOX_UART_DEVICE"] = args.hu_port
            app_env["CANBOX_UART_BAUD"] = str(args.hu_baud)

        app_proc = subprocess.Popen(
            [NATIVE_BINARY_PATH],
            cwd=PROJECT_ROOT,
            env=app_env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        time.sleep(0.3)

    # 5. Start Background CAN Bus Sniffer & Decoder
    decoder = BenchStateDecoder()
    record_file = None
    if args.record_can:
        record_file = open(args.record_can, "w", encoding="utf-8")
        record_file.write("timestamp,can_id,is_ext,is_rtr,err,dlc,d0,d1,d2,d3,d4,d5,d6,d7\n")

    can_listener = SocketCanListener(args.can_iface, decoder, record_fp=record_file)
    can_listener.start()

    # 6. Execute Curses Dashboard or Fallback Streaming Mode
    try:
        if not args.stream and sys.stdout.isatty():
            curses.wrapper(curses_dashboard_loop, decoder, can_listener, args, app_proc)
        else:
            print("[*] Running in streaming log mode (Press Ctrl+C to stop)...")
            while True:
                time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[ERROR] UI Dashboard error: {e}")
    finally:
        print("\n[*] Shutting down HIL bench runner...")
        can_listener.stop()
        can_listener.join(timeout=1.0)
        
        if record_file:
            record_file.close()

        if app_proc and app_proc.poll() is None:
            print("[*] Terminating OpenCanbox Core native app...")
            app_proc.terminate()
            try:
                app_proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                app_proc.kill()

        if args.teardown_slcan:
            teardown_slcan(args.can_iface)

        print("[OK] Clean shutdown complete.")


if __name__ == "__main__":
    main()
