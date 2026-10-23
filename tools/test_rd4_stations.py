#!/usr/bin/env python3
"""
Peugeot 407 RD4 Tuner Station Switcher & Verification Script (Rotary SWC)
========================================================================
Communicates directly with the vehicle CAN bus via SocketCAN (can0 / slcan)
to test changing RD4 memorized tuner stations using the COM2000 Steering
Stalk Rotary Wheel (CAN ID 0x21F Byte 1).

Monitored Frames:
  - CAN 0x165: RD4 Audio Source & Power Status
  - CAN 0x225: Tuner Band, Preset Slot (1..6), Frequency
  - CAN 0x2A5: RDS Station Name (8-byte ASCII)
  - CAN 0x125: ISO-TP Memorized Station List
  - CAN 0x21F: COM2000 Steering Stalk Controls (Rotary Counter in Byte 1)

Test Scenario:
  1. Confirm RD4 is on Station #1 (Preset P1).
  2. Switch to Station #3 using Rotary Wheel ticks and verify selection.
  3. Switch back to Station #1 using Rotary Wheel ticks and verify selection.

Usage:
  # Run standard test scenario:
  python3 tools/test_rd4_stations.py

  # Target specific preset (1..6):
  python3 tools/test_rd4_stations.py --target 3
  python3 tools/test_rd4_stations.py --target 1

  # Manual step up / step down:
  python3 tools/test_rd4_stations.py --step-up
  python3 tools/test_rd4_stations.py --step-down

  # Live Tuner & Station List Monitor:
  python3 tools/test_rd4_stations.py --monitor
"""

import argparse
import select
import socket
import struct
import sys
import threading
import time

CAN_FRAME_FMT = "=IB3x8s"

CAN_ID_SWC_STALK_21F   = 0x21F
CAN_ID_RD4_SOURCE_165  = 0x165
CAN_ID_RD4_STATUS_225  = 0x225
CAN_ID_RD4_STATION_2A5 = 0x2A5
CAN_ID_RD4_LIST_125    = 0x125


def pack_can_frame(can_id: int, data: bytes) -> bytes:
    dlc = len(data)
    padded = data.ljust(8, b"\x00")
    return struct.pack(CAN_FRAME_FMT, can_id & 0x7FF, dlc, padded)


def parse_can_frame(raw_bytes: bytes):
    can_id_raw, can_dlc, data = struct.unpack(CAN_FRAME_FMT, raw_bytes)
    can_id = can_id_raw & 0x7FF
    dlc = min(can_dlc, 8)
    return can_id, dlc, data[:dlc]


class RD4RotaryController:
    def __init__(self, interface: str = "can0"):
        self.interface = interface
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.sock.bind((interface,))
        except OSError as e:
            print(f"[ERROR] Could not bind to interface '{interface}': {e}")
            sys.exit(1)

        self.stalk_counter = 0x10
        self.external_stalk_active = False
        self.external_stalk_last_seen = 0.0

        self.current_preset = None
        self.current_freq_mhz = 0.0
        self.current_band = "FM1"
        self.current_station_name = ""
        self.current_source = "Unknown"
        self.power_on = False

        self.stations = {}  # slot (1..6) -> name
        self.lock = threading.Lock()

    def send_can(self, can_id: int, data: bytes):
        raw = pack_can_frame(can_id, data)
        try:
            self.sock.send(raw)
        except OSError as e:
            print(f"[WARN] Failed to send CAN frame {can_id:03X}#: {e}")

    def sync_state(self, duration: float = 0.5):
        """Listens on CAN bus to learn current state, presets, and stalk counter."""
        t_end = time.time() + duration
        external_count = 0
        while time.time() < t_end:
            r, _, _ = select.select([self.sock], [], [], 0.05)
            if not r:
                continue
            raw = self.sock.recv(16)
            if len(raw) != 16:
                continue
            can_id, dlc, data = parse_can_frame(raw)
            if can_id == CAN_ID_SWC_STALK_21F and dlc >= 2:
                external_count += 1
                self.stalk_counter = data[1]
                self.external_stalk_last_seen = time.time()
            self._process_frame(can_id, dlc, data)

        if external_count >= 2:
            self.external_stalk_active = True

    def _process_frame(self, can_id: int, dlc: int, data: bytes):
        if can_id == CAN_ID_RD4_SOURCE_165 and dlc >= 3:
            src_raw = (data[2] >> 4) & 0x0F
            src_map = {
                0x1: "Tuner (FM/AM)",
                0x2: "Internal CD",
                0x3: "CD Changer (CDC)",
                0x4: "Line In (AUX 1)",
                0x5: "Line In (AUX 2)",
            }
            self.current_source = src_map.get(src_raw, f"Source 0x{src_raw:X}")
            self.power_on = (data[0] & 0xC0) == 0xC0

        elif can_id == CAN_ID_RD4_STATUS_225 and dlc >= 5:
            # Preset slot: 0=manual, 1..6
            self.current_preset = (data[1] >> 4) & 0x0F if data[1] >= 0x10 else (data[1] & 0x0F)

            raw_band = data[2]
            is_fm = True
            if raw_band in (0x10, 0x90):
                self.current_band = "FM1"
            elif raw_band in (0x20, 0xA0):
                self.current_band = "FM2"
            elif raw_band in (0x40, 0xC0):
                self.current_band = "FM-AST"
            elif raw_band in (0x50, 0xD0):
                self.current_band = "AM"
                is_fm = False

            raw_freq = (data[3] << 8) | data[4]
            if is_fm and raw_freq > 0:
                self.current_freq_mhz = ((raw_freq // 2) + 500) / 10.0
            elif not is_fm and raw_freq > 0:
                self.current_freq_mhz = float(raw_freq)

            if self.current_preset and 1 <= self.current_preset <= 6 and self.current_station_name:
                self.stations[self.current_preset] = self.current_station_name

        elif can_id == CAN_ID_RD4_STATION_2A5:
            name = "".join(chr(b) if 0x20 <= b <= 0x7E else " " for b in data[:dlc]).strip()
            if name:
                self.current_station_name = name
                if self.current_preset and 1 <= self.current_preset <= 6:
                    self.stations[self.current_preset] = name

    def get_state_summary(self) -> str:
        preset_str = f"P{self.current_preset}" if (self.current_preset and 1 <= self.current_preset <= 6) else "Manual"
        station_str = f"'{self.current_station_name}'" if self.current_station_name else "---"
        return f"{self.current_band} | {preset_str} | {self.current_freq_mhz:.1f} MHz | Station: {station_str}"

    def rotary_step_up(self):
        """Emits a single rotary wheel step UP (+1) on CAN 0x21F."""
        self.stalk_counter = (self.stalk_counter + 1) & 0xFF
        self.send_can(CAN_ID_SWC_STALK_21F, bytes([0x00, self.stalk_counter, 0x00]))

    def rotary_step_down(self):
        """Emits a single rotary wheel step DOWN (-1) on CAN 0x21F."""
        self.stalk_counter = (self.stalk_counter - 1) & 0xFF
        self.send_can(CAN_ID_SWC_STALK_21F, bytes([0x00, self.stalk_counter, 0x00]))

    def wait_for_preset(self, target_preset: int, timeout: float = 2.0) -> bool:
        t_end = time.time() + timeout
        while time.time() < t_end:
            r, _, _ = select.select([self.sock], [], [], 0.05)
            if r:
                raw = self.sock.recv(16)
                if len(raw) == 16:
                    can_id, dlc, data = parse_can_frame(raw)
                    self._process_frame(can_id, dlc, data)
                    if self.current_preset == target_preset:
                        time.sleep(0.2)
                        self.sync_state(0.3)
                        return True
        return False

    def select_station(self, target_preset: int) -> bool:
        """
        Navigates to target_preset (1..6) using rotary wheel ticks on CAN 0x21F.
        """
        if not (1 <= target_preset <= 6):
            print(f"[ERROR] Invalid preset {target_preset}. Must be 1..6.")
            return False

        self.sync_state(0.4)
        print(f"[*] Current status: {self.get_state_summary()}")
        if self.current_preset == target_preset:
            print(f"[✓] Already at Station #{target_preset}.")
            return True

        curr = self.current_preset if (self.current_preset and 1 <= self.current_preset <= 6) else 1

        # Calculate shortest path in 6-preset ring
        delta = (target_preset - curr) % 6
        step_dir = +1 if delta <= 3 else -1
        steps = delta if delta <= 3 else (6 - delta)

        dir_str = "UP (+1)" if step_dir > 0 else "DOWN (-1)"
        print(f"[*] Navigating from Station #{curr} -> #{target_preset} ({steps} step(s) {dir_str} via Rotary)...")

        for i in range(1, steps + 1):
            if step_dir > 0:
                print(f"  [Step {i}/{steps}] Rotary Step UP -> Counter: 0x{((self.stalk_counter + 1) & 0xFF):02X}")
                self.rotary_step_up()
            else:
                print(f"  [Step {i}/{steps}] Rotary Step DOWN -> Counter: 0x{((self.stalk_counter - 1) & 0xFF):02X}")
                self.rotary_step_down()

            time.sleep(0.080)
            self.sync_state(0.2)

        # Wait for settling
        self.wait_for_preset(target_preset, timeout=1.5)
        self.sync_state(0.4)
        print(f"[*] Result after rotary navigation: {self.get_state_summary()}")
        return (self.current_preset == target_preset)


def run_scenario(controller: RD4RotaryController) -> bool:
    print("=" * 70)
    print(" PEUGEOT 407 RD4 TUNER ROTARY STATION VERIFICATION SCENARIO ")
    print("=" * 70)

    # Step 0: Initial Synchronization
    print("\n[Step 0] Synchronizing with RD4 on CAN bus (can0)...")
    controller.sync_state(0.8)
    print(f"Initial State: {controller.get_state_summary()}")

    if controller.external_stalk_active:
        print("\n  [INFO] Hardware COM2000 detected on bus transmitting 0x21F [00 {:02X} 00]".format(controller.stalk_counter))
        print("  [NOTE] If physical COM2000 is broadcasting, ensure no contention or disconnect stalk for pure software simulation.")

    # Ensure starting at Station #1
    if controller.current_preset != 1:
        print("\n[*] Initializing radio to Station #1 via Rotary...")
        controller.select_station(1)
    else:
        print("[✓] Radio is currently at Station #1.")

    print(f"Confirmed State: {controller.get_state_summary()}")
    time.sleep(1.0)

    # Step 1: Change to Station #3
    print("\n" + "-" * 70)
    print("[Step 1] Changing Tuner to Station #3 using Rotary Wheel...")
    print("-" * 70)
    success_3 = controller.select_station(3)
    if not success_3:
        print(f"[FAIL] Station #3 was NOT selected! Current state: {controller.get_state_summary()}")
    else:
        print(f"[PASS] Station #3 successfully selected: {controller.get_state_summary()}")

    time.sleep(1.5)

    # Step 2: Change back to Station #1
    print("\n" + "-" * 70)
    print("[Step 2] Changing Tuner back to Station #1 using Rotary Wheel...")
    print("-" * 70)
    success_1 = controller.select_station(1)
    if not success_1:
        print(f"[FAIL] Station #1 was NOT restored! Current state: {controller.get_state_summary()}")
    else:
        print(f"[PASS] Station #1 successfully restored: {controller.get_state_summary()}")

    overall = success_3 and success_1
    print("\n" + "=" * 70)
    result_text = "ALL ROTARY STATION CHANGES VERIFIED SUCCESSFULLY" if overall else "ROTARY TEST FINISHED (Check Stalk Contention)"
    print(f" [TEST RESULT] {result_text} ")
    print("=" * 70)
    return overall


def main():
    parser = argparse.ArgumentParser(description="Peugeot RD4 Tuner Rotary Station Switcher & Verification")
    parser.add_argument("-i", "--interface", default="can0", help="SocketCAN interface (default: can0)")
    parser.add_argument("--target", type=int, choices=range(1, 7), help="Target specific preset station (1..6) using rotary")
    parser.add_argument("--step-up", action="store_true", help="Send 1 rotary step UP (+1)")
    parser.add_argument("--step-down", action="store_true", help="Send 1 rotary step DOWN (-1)")
    parser.add_argument("--monitor", action="store_true", help="Monitor RD4 tuner state & station list in real time")
    args = parser.parse_args()

    controller = RD4RotaryController(interface=args.interface)

    if args.monitor:
        print(f"[*] Monitoring RD4 radio on {args.interface} (Ctrl+C to stop)...")
        last_str = ""
        while True:
            controller.sync_state(0.2)
            s = controller.get_state_summary()
            if s != last_str:
                ts = time.strftime("%H:%M:%S")
                print(f"[{ts}] {s}")
                last_str = s

    if args.step_up:
        controller.sync_state(0.3)
        controller.rotary_step_up()
        print(f"[*] Emitted Rotary Step UP. New counter: 0x{controller.stalk_counter:02X}")
        controller.sync_state(0.5)
        print(f"[*] Current status: {controller.get_state_summary()}")
        sys.exit(0)

    if args.step_down:
        controller.sync_state(0.3)
        controller.rotary_step_down()
        print(f"[*] Emitted Rotary Step DOWN. New counter: 0x{controller.stalk_counter:02X}")
        controller.sync_state(0.5)
        print(f"[*] Current status: {controller.get_state_summary()}")
        sys.exit(0)

    if args.target:
        ok = controller.select_station(args.target)
        sys.exit(0 if ok else 1)

    ok = run_scenario(controller)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
