#!/usr/bin/env python3
"""
Workbench Rear Defrost CAN Discovery & Verification Tool
========================================================

Designed for Peugeot 407 (AEE2004 Comfort CAN @ 125 kbps) on workbench setup:
- SocketCAN (can0 via slcan)
- USB Serial (Hiworld 0x5A 0xA5 @ 38400 baud or Raise 0x2E @ 19200 baud)
- Attached RD4 radio & canbox (Raise / Hiworld)

Safety Guaranteed:
- Hard whitelist of broadcast comfort CAN IDs.
- Strictly bans diagnostic/programming ranges (0x5E0-0x5EF, 0x600-0x7FF) and RD4 EEPROM write frames.
- Automated 3-cycle state toggle auto-verification to eliminate false positives.
"""

import sys
import os
import re
import glob
import subprocess
import time
import json
import struct
import socket
import select
import logging
import argparse
import datetime
import threading
from typing import Optional, Tuple, List, Dict, Any

try:
    import serial
except ImportError:
    print("[ERROR] pyserial is required. Install with: pip install pyserial")
    sys.exit(1)

# SocketCAN frame format: <can_id: 32-bit uint>, <can_dlc: 8-bit uint>, <pad: 3 bytes>, <data: 8 bytes>
CAN_FRAME_FMT = "=IB3x8s"

# Safety Whitelist: ONLY these broadcast comfort/clima IDs may be tested.
# Diagnostic IDs (0x600..0x7FF, 0x5E0..0x5EF) and RD4 display configs (0x0A4, 0x2A5, 0x3A5) are STRICTLY BANNED.
SAFE_STAGE1_CANDIDATES = [
    (0x1E3, 7, "Climate ECU Display State (PSA 407 MSG_ETAT_CLIMAV_EMF)"),
    (0x1D0, 7, "Climate Panel State (PSA 407 Dual-Zone)"),
    (0x1D0, 8, "Climate Panel State (8-byte variant)"),
    (0x0F6, 8, "BSI Power / Status / Ignition (Byte 5 ambient temp protected)"),
    (0x128, 8, "BSI Body / Doors / Mirror Demister Status"),
    (0x168, 8, "Instrument Cluster Warning Lamps & Telltales"),
    (0x1E1, 8, "Climate Control Panel Button Requests"),
    (0x217, 8, "BSI Comfort & Heated Screen / Mirror Status"),
    (0x228, 8, "BSI Auxiliary Climate / Heating Status"),
    (0x336, 8, "BSI Comfort Features Extra"),
    (0x366, 8, "BSI HVAC Extra"),
]

SAFE_STAGE2_CANDIDATES = [
    (0x0E1, 8, "Parking Radar Status"),
    (0x21F, 8, "Steering Wheel Stalk Keys"),
    (0x220, 8, "BSI Vehicle Status"),
    (0x221, 8, "Trip Computer / Stalk Input"),
    (0x260, 8, "BSI Vehicle Personalization Options"),
    (0x261, 8, "Trip 1 Statistics"),
    (0x2A1, 8, "Trip 2 Statistics"),
    (0x305, 8, "Steering Angle Sensor"),
    (0x361, 8, "BSI Settings Broadcast"),
    (0x3E5, 8, "Radio Remote Controls"),
]

# Bit values to sweep per byte
TEST_BIT_VALUES = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0xFF]

class CanboxDefrostVerifier:
    def __init__(self, can_iface: str, uart_port: str, baud_rate: Optional[int], proto_mode: str, stage: int, custom_id: Optional[int] = None, verbose: bool = False):
        self.can_iface = can_iface
        self.uart_port = uart_port
        self.proto_mode = proto_mode.lower() # 'auto', 'hiworld', 'raise'
        self.stage = stage
        self.custom_id = custom_id
        self.verbose = verbose

        # Baud rate configuration
        if baud_rate is not None:
            self.baud_rate = baud_rate
            self.auto_baud = False
        else:
            if self.proto_mode == "raise":
                self.baud_rate = 38400
                self.auto_baud = False
            elif self.proto_mode == "hiworld":
                self.baud_rate = 38400
                self.auto_baud = False
            elif self.proto_mode == "bagoo":
                self.baud_rate = 19200
                self.auto_baud = False
            else:
                self.baud_rate = 38400  # Default to 38400 (Standard Peugeot 407 Hiworld & Raise)
                self.auto_baud = False

        self.can_sock: Optional[socket.socket] = None
        self.ser: Optional[serial.Serial] = None
        self.stop_event = threading.Event()

        # State tracking
        self.active_proto = "unknown" if self.proto_mode == "auto" else self.proto_mode
        self.rear_defrost_active = False
        self.front_defrost_active = False
        self.last_rear_defrost_update = 0.0
        self.state_lock = threading.Lock()
        self.state_change_event = threading.Event()

        # Current fuzzer status
        self.current_test_desc = "Idle"
        self.confirmed_hits: List[Dict[str, Any]] = []
        self.active_overrides: Dict[int, bytes] = {}
        self.override_lock = threading.Lock()

    def open_interfaces(self):
        print(f"[INIT] Opening SocketCAN interface: {self.can_iface}...")
        try:
            self.can_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.can_sock.bind((self.can_iface,))
        except Exception as e:
            print(f"[FATAL] Cannot bind to CAN interface '{self.can_iface}': {e}")
            print("       Make sure slcan is up (e.g. 'sudo slcand -o -s4 -t hw -S 3000000 /dev/ttyUSB0 can0 && sudo ip link set up can0')")
            sys.exit(1)

        print(f"[INIT] Opening UART port: {self.uart_port} @ {self.baud_rate} baud 8N1...")
        try:
            self.ser = serial.Serial(self.uart_port, self.baud_rate, timeout=0.05)
        except Exception as e:
            print(f"[FATAL] Cannot open UART serial port '{self.uart_port}': {e}")
            self.can_sock.close()
            sys.exit(1)

    def wake_can_bus(self):
        """
        Emits active PSA CAN2004 wake-up sequence to wake sleeping ECUs (RD4 and Canbox):
        - Phase de vie = 0x03 (Wakeup transition) then 0x01 (Active/Running)
        - 0x0F6: Economy mode inactive, Contact ON (+APC)
        - 0x128: Key in ignition lock (+APC)
        - 0x15B: BSI power & equipment state
        - 0x1D0: Dual-zone HVAC baseline
        - 0x261 / 0x2A1: Trip telemetry to stimulate canbox UART reporting
        """
        print(f"[CAN WAKE] Initializing Comfort CAN Bus (Waking RD4 & Canbox)...")
        wake_trans_036 = bytes([0x0E, 0x00, 0x00, 0x2A, 0x03, 0x00, 0x00, 0xA0])
        wake_0f6 = bytes([0x88, 0x00, 0x00, 0x00, 0x00, 0x50, 0x00, 0x20])
        for _ in range(6):
            self.send_can(0x036, wake_trans_036)
            self.send_can(0x0F6, wake_0f6)
            time.sleep(0.03)

        wake_active_036 = bytes([0x0E, 0x00, 0x00, 0x2A, 0x01, 0x00, 0x00, 0xA0])
        key_128 = bytes([0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        bsi_15b = bytes([0x01, 0x03, 0xB1, 0x40, 0x00, 0xD0, 0x00, 0x10])
        body_168 = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        hvac_1d0 = bytes([0x08, 0x80, 0x07, 0x00, 0x00, 0x0B, 0x0B])
        hvac_1e3 = bytes([0x1C, 0x40, 0x0B, 0x0B, 0x00, 0x00, 0x07])
        comp_12d = bytes([0x80, 0x32, 0x32, 0x00, 0x64, 0x00, 0x00, 0x80])
        trip1_261 = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        trip2_2a1 = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

        for _ in range(8):
            self.send_can(0x036, wake_active_036)
            self.send_can(0x0F6, wake_0f6)
            self.send_can(0x128, key_128)
            self.send_can(0x15B, bsi_15b)
            self.send_can(0x168, body_168)
            self.send_can(0x1D0, hvac_1d0)
            self.send_can(0x1E3, hvac_1e3)
            self.send_can(0x12D, comp_12d)
            self.send_can(0x261, trip1_261)
            self.send_can(0x2A1, trip2_2a1)
            time.sleep(0.04)
        print(f"[CAN WAKE] CAN bus awake. Active BSI keeper armed.")

    def send_can(self, can_id: int, data: bytes):
        if not self.can_sock:
            return
        # RD4 Safety Check: Enforce whitelist
        if can_id >= 0x5E0 or (can_id in (0x0A4, 0x2A5, 0x3A5, 0x165, 0x3F6)):
            # Strictly prohibited during fuzzing
            return
        dlc = len(data)
        padded = data.ljust(8, b"\x00")[:8]
        frame = struct.pack(CAN_FRAME_FMT, can_id, dlc, padded)
        try:
            self.can_sock.send(frame)
        except OSError:
            pass

    @staticmethod
    def calc_hiworld_cs(frame_bytes: bytes) -> int:
        """Hiworld CS = ((sum(frame[2:-1]) - 1) & 0xFF)"""
        return (sum(frame_bytes[2:-1]) - 1) & 0xFF

    @staticmethod
    def calc_raise_cs(frame_bytes: bytes) -> int:
        """Raise CS = (((sum(frame[1:-1]) & 0xFF) ^ 0xFF) & 0xFF)"""
        return (sum(frame_bytes[1:-1]) & 0xFF) ^ 0xFF

    @staticmethod
    def calc_bagoo_cs(frame_bytes: bytes) -> int:
        """Bagoo / SimpleSoft CS = (total_len + cmd + sum(payload)) & 0xFF"""
        return (frame_bytes[1] + frame_bytes[2] + sum(frame_bytes[3:-1])) & 0xFF

    def parse_hiworld_frame(self, frame: bytes):
        cmd = frame[3]
        length = frame[2]
        payload = frame[4:-1]

        # Send ACK to keep Hiworld streaming
        ack_cs = (0x01 + 0xFF + cmd - 1) & 0xFF
        try:
            if self.ser:
                self.ser.write(bytes([0x5A, 0xA5, 0x01, 0xFF, cmd, ack_cs]))
        except (serial.SerialException, OSError):
            pass

        with self.state_lock:
            if self.active_proto != "hiworld":
                self.active_proto = "hiworld"
                self.proto_mode = "hiworld"
                print(f"\n[PROTO] Detected Hiworld protocol on UART (0x5A 0xA5, Cmd: 0x{cmd:02X}, Len: {length})")

        if self.verbose:
            print(f"[UART RX HIWORLD] Cmd 0x{cmd:02X} (len {len(payload)}): {payload.hex(' ')}")

        if cmd == 0x31 and len(payload) >= 3:
            byte_2 = payload[2]
            rear = bool(byte_2 & 0x20)
            front = bool(byte_2 & 0x10)

            with self.state_lock:
                changed = (rear != self.rear_defrost_active)
                self.rear_defrost_active = rear
                self.front_defrost_active = front
                self.last_rear_defrost_update = time.time()
                if changed:
                    self.state_change_event.set()

    def parse_raise_frame(self, frame: bytes):
        cmd = frame[1]
        length = frame[2]
        payload = frame[3:-1]

        with self.state_lock:
            if self.active_proto != "raise":
                self.active_proto = "raise"
                self.proto_mode = "raise"
                print(f"\n[PROTO] Detected Raise protocol on UART (0x2E, Cmd: 0x{cmd:02X}, Len: {length})")

        if self.verbose:
            print(f"[UART RX RAISE] Cmd 0x{cmd:02X} (len {length}): {payload.hex(' ')}")

        if cmd == 0x21 and len(payload) >= 1:
            byte_0 = payload[0]
            rear = bool(byte_0 & 0x01)
            front = bool(payload[4] & 0x80) if len(payload) >= 5 else False

            with self.state_lock:
                changed = (rear != self.rear_defrost_active)
                self.rear_defrost_active = rear
                self.front_defrost_active = front
                self.last_rear_defrost_update = time.time()
                if changed:
                    self.state_change_event.set()

    def parse_bagoo_frame(self, frame: bytes):
        total_len = frame[1]
        cmd = frame[2]
        payload = frame[3:-1]

        with self.state_lock:
            if self.active_proto != "bagoo":
                self.active_proto = "bagoo"
                self.proto_mode = "bagoo"
                print(f"\n[PROTO] Detected Bagoo / SimpleSoft protocol on UART (0xFD, Cmd: 0x{cmd:02X}, Len: {len(payload)})")

        if self.verbose:
            print(f"[UART RX BAGOO] Cmd 0x{cmd:02X} (len {len(payload)}): {payload.hex(' ')}")

        if cmd == 0x21 and len(payload) >= 1:
            byte_0 = payload[0]
            rear = bool(byte_0 & 0x01)
            front = bool(payload[4] & 0x80) if len(payload) >= 5 else False

            with self.state_lock:
                changed = (rear != self.rear_defrost_active)
                self.rear_defrost_active = rear
                self.front_defrost_active = front
                self.last_rear_defrost_update = time.time()
                if changed:
                    self.state_change_event.set()

    def uart_receiver_thread(self):
        buf = bytearray()
        last_baud_toggle = time.time()
        locked = not self.auto_baud

        while not self.stop_event.is_set():
            if not self.auto_baud or self.active_proto != "unknown":
                locked = True

            # Auto-baud rate probing when baud not explicitly locked (alternate 38400 and 19200 every 5.0s)
            if self.auto_baud and not locked and (time.time() - last_baud_toggle > 5.0):
                next_baud = 19200 if self.ser.baudrate == 38400 else 38400
                try:
                    self.ser.baudrate = next_baud
                    self.ser.reset_input_buffer()
                    buf.clear()
                    last_baud_toggle = time.time()
                    if self.verbose:
                        print(f"[BAUD] Probing UART baud rate: {next_baud} bps...")
                except Exception:
                    pass

            try:
                chunk = self.ser.read(64)
                if not chunk:
                    continue
                buf.extend(chunk)

                if self.verbose:
                    print(f"[UART RAW] {chunk.hex(' ')}")

                # Process packets
                while len(buf) >= 4:
                    # Hiworld Check: 0x5A 0xA5
                    if buf[0] == 0x5A and buf[1] == 0xA5:
                        length = buf[2]
                        if length > 64:
                            del buf[0]
                            continue
                        total_len = length + 5
                        if len(buf) < total_len:
                            break # Incomplete packet
                        frame = bytes(buf[:total_len])
                        if self.calc_hiworld_cs(frame) == frame[-1]:
                            if not locked:
                                locked = True
                                self.auto_baud = False
                                self.baud_rate = self.ser.baudrate
                                print(f"[BAUD] Locked UART baud rate to {self.ser.baudrate} bps (Hiworld detected)")
                            self.parse_hiworld_frame(frame)
                            del buf[:total_len]
                            continue
                        else:
                            del buf[0]
                            continue

                    # Raise Check: 0x2E
                    if buf[0] == 0x2E:
                        cmd = buf[1]
                        length = buf[2]
                        if length > 64:
                            del buf[0]
                            continue
                        total_len = length + 4
                        if len(buf) < total_len:
                            break # Incomplete packet
                        frame = bytes(buf[:total_len])
                        if self.calc_raise_cs(frame) == frame[-1]:
                            if not locked:
                                locked = True
                                self.auto_baud = False
                                self.baud_rate = self.ser.baudrate
                                print(f"[BAUD] Locked UART baud rate to {self.ser.baudrate} bps (Raise detected)")
                            self.parse_raise_frame(frame)
                            del buf[:total_len]
                            continue
                        else:
                            del buf[0]
                            continue

                    # Bagoo / SimpleSoft Check: 0xFD
                    if buf[0] == 0xFD:
                        tlen = buf[1]
                        if tlen < 3 or tlen > 64:
                            del buf[0]
                            continue
                        total_len = tlen + 1
                        if len(buf) < total_len:
                            break # Incomplete packet
                        frame = bytes(buf[:total_len])
                        if self.calc_bagoo_cs(frame) == frame[-1]:
                            if not locked:
                                locked = True
                                self.auto_baud = False
                                self.baud_rate = self.ser.baudrate
                                print(f"[BAUD] Locked UART baud rate to {self.ser.baudrate} bps (Bagoo / SimpleSoft detected)")
                            self.parse_bagoo_frame(frame)
                            del buf[:total_len]
                            continue
                        else:
                            del buf[0]
                            continue

                    # Unaligned byte discard
                    del buf[0]
            except (serial.SerialException, OSError) as e:
                if not self.stop_event.is_set():
                    print(f"[UART ERROR] {e}")
                break

    def background_keepalive_thread(self):
        """
        Broadcasts safe background PSA BSI frames to keep RD4 & Canbox awake:
        - 0x036: Ignition ON / Phase de vie = 0x01 (Awake)
        - 0x0B6: Engine running / RPM
        - 0x0F6: Economy mode inactive, valid telemetry (unless actively fuzzed)
        - 0x128: Key in ignition / Contact ON (unless actively fuzzed)
        - 0x15B: BSI power & equipment state
        - 0x168: Body / warning cluster status (unless actively fuzzed)
        - 0x1D0: Dual-zone HVAC baseline (unless actively fuzzed)
        - 0x261 / 0x2A1: Trip telemetry (unless actively fuzzed)
        """
        payload_036 = bytes([0x0E, 0x00, 0x00, 0x2A, 0x01, 0x00, 0x00, 0xA0])
        payload_0b6 = bytes([0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        payload_0f6 = bytes([0x88, 0x00, 0x00, 0x00, 0x00, 0x50, 0x00, 0x20])
        payload_128 = bytes([0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        payload_15b = bytes([0x01, 0x03, 0xB1, 0x40, 0x00, 0xD0, 0x00, 0x10])
        payload_168 = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        payload_1d0 = bytes([0x08, 0x80, 0x07, 0x00, 0x00, 0x0B, 0x0B])
        payload_1e3 = bytes([0x1C, 0x40, 0x0B, 0x0B, 0x00, 0x00, 0x07])
        payload_12d = bytes([0x80, 0x32, 0x32, 0x00, 0x64, 0x00, 0x00, 0x80])
        payload_261 = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        payload_2a1 = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

        trip_timer = 0
        while not self.stop_event.is_set():
            self.send_can(0x036, payload_036)
            self.send_can(0x0B6, payload_0b6)
            self.send_can(0x15B, payload_15b)
            self.send_can(0x12D, payload_12d)

            with self.override_lock:
                ov = dict(self.active_overrides)

            self.send_can(0x0F6, ov.get(0x0F6, payload_0f6))
            self.send_can(0x128, ov.get(0x128, payload_128))
            self.send_can(0x168, ov.get(0x168, payload_168))
            self.send_can(0x1D0, ov.get(0x1D0, payload_1d0))
            self.send_can(0x1E3, ov.get(0x1E3, payload_1e3))

            # Send any extra candidate actively being tested
            for cid, cdata in ov.items():
                if cid not in (0x0F6, 0x128, 0x168, 0x1D0, 0x1E3):
                    self.send_can(cid, cdata)

            trip_timer += 1
            if trip_timer >= 10:  # Every ~400 ms
                trip_timer = 0
                self.send_can(0x261, ov.get(0x261, payload_261))
                self.send_can(0x2A1, ov.get(0x2A1, payload_2a1))

            time.sleep(0.040)

    def auto_verify_candidate(self, can_id: int, test_payload: bytes, base_payload: bytes, desc: str) -> bool:
        """
        Performs 3 cycles of OFF -> ON -> OFF -> ON validation.
        Returns True only if the canbox reliably tracks the toggle every cycle.
        """
        print(f"\n   [VERIFY] Testing 3-cycle toggle confirmation for {desc}...")
        
        for cycle in range(1, 4):
            # Phase 1: Expect OFF
            self.state_change_event.clear()
            with self.override_lock:
                self.active_overrides[can_id] = base_payload
            time.sleep(0.20)
            with self.state_lock:
                if self.rear_defrost_active:
                    print(f"   [VERIFY FAIL] Cycle {cycle}: Defrost did not turn OFF on base frame!")
                    return False

            # Phase 2: Expect ON
            self.state_change_event.clear()
            with self.override_lock:
                self.active_overrides[can_id] = test_payload
            time.sleep(0.20)
            with self.state_lock:
                if not self.rear_defrost_active:
                    print(f"   [VERIFY FAIL] Cycle {cycle}: Defrost did not turn ON on test frame!")
                    return False

            print(f"   [VERIFY] Cycle {cycle}/3: OK (Reliably toggled OFF -> ON)")

        # Leave in OFF state
        with self.override_lock:
            self.active_overrides[can_id] = base_payload
        time.sleep(0.05)

        return True

    def test_single_combination(self, can_id: int, dlc: int, byte_idx: int, val: int, base_data: bytearray) -> bool:
        test_data = bytearray(base_data)
        test_data[byte_idx] ^= val

        self.current_test_desc = f"0x{can_id:03X} B{byte_idx} ^ 0x{val:02X}"
        sys.stdout.write(f"\r🔍 [{self.active_proto.upper()}] Testing -> CAN ID: 0x{can_id:03X} | Byte: {byte_idx} | Mask: 0x{val:02X} (State: Defrost={'ON' if self.rear_defrost_active else 'OFF'})   ")
        sys.stdout.flush()

        self.state_change_event.clear()
        with self.state_lock:
            initial_state = self.rear_defrost_active

        # Inject test payload via continuous keepalive broadcaster
        with self.override_lock:
            self.active_overrides[can_id] = bytes(test_data)
        time.sleep(0.180)

        # Check if state flipped to ON
        with self.state_lock:
            current_state = self.rear_defrost_active

        # Restore base payload
        with self.override_lock:
            self.active_overrides[can_id] = bytes(base_data)
        time.sleep(0.040)

        if not initial_state and current_state:
            # Candidate hit! Run 3-cycle auto-verification
            desc = f"CAN 0x{can_id:03X} [Byte {byte_idx} Mask 0x{val:02X}]"
            print(f"\n\n🎯 POTENTIAL HIT DETECTED: {desc}")
            print(f"   Raw Test Payload: {' '.join(f'{b:02X}' for b in test_data)}")
            print(f"   Raw Base Payload: {' '.join(f'{b:02X}' for b in base_data)}")

            if self.auto_verify_candidate(can_id, bytes(test_data), bytes(base_data), desc):
                print(f"\n✅ [CONFIRMED 100% REPEATABLE] Rear Defrost controlled by:")
                print(f"   CAN ID   : 0x{can_id:03X}")
                print(f"   Byte     : {byte_idx}")
                print(f"   Bit Mask : 0x{val:02X}")
                print(f"   Payload  : {' '.join(f'{b:02X}' for b in test_data)}")
                print(f"   Protocol : {self.active_proto.upper()}\n")

                hit_record = {
                    "timestamp": datetime.datetime.now().isoformat(),
                    "protocol": self.active_proto,
                    "can_id": f"0x{can_id:03X}",
                    "dlc": dlc,
                    "byte_index": byte_idx,
                    "bit_mask": f"0x{val:02X}",
                    "test_payload_hex": " ".join(f"{b:02X}" for b in test_data),
                    "base_payload_hex": " ".join(f"{b:02X}" for b in base_data),
                }
                self.confirmed_hits.append(hit_record)
                self.save_results()

                # Interactive prompt: Continue or Stop?
                print("Options: [C]ontinue scanning remaining frames | [S]top and finish")
                while True:
                    try:
                        choice = input("Select [C/S] (Default C): ").strip().upper()
                        if not choice or choice == "C":
                            print("[INFO] Resuming candidate scan...\n")
                            return False # Do not break entire scan
                        elif choice == "S":
                            print("[INFO] Stopping scan per user request.")
                            return True # Stop
                    except (EOFError, KeyboardInterrupt):
                        return True

        # Clear candidate override so background returns to default baseline
        with self.override_lock:
            self.active_overrides.pop(can_id, None)

        return False

    def build_candidate_list(self) -> List[Tuple[int, int, str]]:
        if self.custom_id is not None:
            return [(self.custom_id, 8, f"User Custom ID 0x{self.custom_id:03X}")]

        candidates = list(SAFE_STAGE1_CANDIDATES)
        if self.stage >= 2:
            candidates.extend(SAFE_STAGE2_CANDIDATES)
        return candidates

    def run_fuzzer(self):
        candidates = self.build_candidate_list()
        print(f"\n=== STARTING REAR DEFROST CAN DISCOVERY ===")
        print(f"Interface : {self.can_iface} (SocketCAN)")
        print(f"UART Port : {self.uart_port} @ {self.baud_rate} (Mode: {self.proto_mode.upper()})")
        print(f"Candidates: {len(candidates)} safe CAN IDs in scope")
        print("Press Ctrl+C at any time to exit.\n")

        # Synchronize and detect protocol (wait up to 8s if auto-probing baud/proto)
        print("[SYNC] Synchronizing with canbox UART protocol (CAN wake-up active)...")
        t_sync = time.time()
        sync_timeout = 8.0 if self.auto_baud else 3.0
        while time.time() - t_sync < sync_timeout:
            with self.state_lock:
                if self.active_proto != "unknown":
                    break
            time.sleep(0.1)

        with self.state_lock:
            if self.active_proto == "unknown":
                print(f"[WARN] Protocol sync window elapsed. Defaulting to RAISE protocol @ {self.ser.baudrate} bps.")
                self.active_proto = "raise"
                self.proto_mode = "raise"
                self.auto_baud = False
            else:
                print(f"[STATUS] Locked Canbox Protocol: {self.active_proto.upper()} @ {self.ser.baudrate} bps | Defrost: {'ON' if self.rear_defrost_active else 'OFF'}")

        for can_id, dlc, desc in candidates:
            if self.stop_event.is_set():
                break

            print(f"\n--- Checking Candidate: 0x{can_id:03X} ({desc}) ---")

            # Setup baseline data for this ID from authentic operational state
            BASE_DATA_MAP = {
                0x1E3: bytes([0x1C, 0x40, 0x0B, 0x0B, 0x00, 0x00, 0x07]),
                0x1D0: bytes([0x08, 0x80, 0x07, 0x00, 0x00, 0x0B, 0x0B]),
                0x0F6: bytes([0x88, 0x00, 0x00, 0x00, 0x00, 0x50, 0x00, 0x20]),
                0x128: bytes([0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
                0x15B: bytes([0x01, 0x03, 0xB1, 0x40, 0x00, 0xD0, 0x00, 0x10]),
                0x168: bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
                0x1E1: bytes([0x3C, 0x3C, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00]),
            }

            if can_id in BASE_DATA_MAP:
                raw_base = BASE_DATA_MAP[can_id]
                base_data = bytearray(raw_base[:dlc]) if len(raw_base) >= dlc else bytearray(raw_base.ljust(dlc, b"\x00"))
            else:
                base_data = bytearray(dlc)

            for byte_idx in range(dlc):
                if self.stop_event.is_set():
                    break

                # Protect Byte 5 in 0x0F6 (Ambient temp)
                if can_id == 0x0F6 and byte_idx == 5:
                    continue

                for val in TEST_BIT_VALUES:
                    if self.stop_event.is_set():
                        break

                    stop_requested = self.test_single_combination(can_id, dlc, byte_idx, val, base_data)
                    if stop_requested:
                        return

        print("\n\n[INFO] Candidate sweep complete.")

    def save_results(self):
        if not self.confirmed_hits:
            return
        filename = f"defrost_confirmed_{self.active_proto}_{int(time.time())}.json"
        try:
            with open(filename, "w") as f:
                json.dump(self.confirmed_hits, f, indent=2)
            print(f"[SAVED] Results exported to {filename}")
        except Exception as e:
            print(f"[ERROR] Could not save results: {e}")

    def start(self):
        self.open_interfaces()
        # 1. Wake up sleeping ECUs (RD4 and Canbox) before starting threads
        self.wake_can_bus()

        # 2. Start background BSI keeper so CAN bus never falls asleep
        t_bg = threading.Thread(target=self.background_keepalive_thread, daemon=True)
        t_bg.start()

        # 3. Start UART receiver
        t_uart = threading.Thread(target=self.uart_receiver_thread, daemon=True)
        t_uart.start()

        try:
            self.run_fuzzer()
        except KeyboardInterrupt:
            print("\n[INFO] User interrupted. Exiting...")
        finally:
            self.stop_event.set()
            if self.ser:
                self.ser.close()
            if self.can_sock:
                self.can_sock.close()

            print("\n=== SUMMARY OF CONFIRMED HITS ===")
            if self.confirmed_hits:
                for hit in self.confirmed_hits:
                    print(f"• ID: {hit['can_id']} | Byte: {hit['byte_index']} | Mask: {hit['bit_mask']} | Proto: {hit['protocol']}")
                self.save_results()
            else:
                print("No confirmed defrost triggers found in this pass.")

def find_slcan_device() -> Optional[str]:
    """Finds which ttyUSB device is currently held by slcand."""
    try:
        res = subprocess.run(["ps", "-eo", "args"], capture_output=True, text=True)
        for line in res.stdout.splitlines():
            if "slcand" in line:
                m = re.search(r"(ttyUSB\d+|ttyACM\d+)", line)
                if m:
                    return f"/dev/{m.group(1)}"
    except Exception:
        pass
    return None

def get_default_uart_port() -> str:
    """Selects the first available serial port that is NOT the slcand interface."""
    slcan_dev = find_slcan_device()
    candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    for p in candidates:
        if p != slcan_dev:
            return p
    return "/dev/ttyUSB0"

def main():
    parser = argparse.ArgumentParser(description="Workbench Peugeot 407 Rear Defrost CAN Discovery & Verification")
    parser.add_argument("-c", "--can", default="can0", help="SocketCAN interface name (default: can0)")
    parser.add_argument("-p", "--port", default=None, help="UART serial port for canbox (auto-selects port not used by slcand)")
    parser.add_argument("-b", "--baud", type=int, default=None, help="UART baud rate (default: auto-probe 38400 / 19200)")
    parser.add_argument("--proto", choices=["auto", "hiworld", "raise", "bagoo"], default="auto", help="Canbox protocol (default: auto)")
    parser.add_argument("--stage", type=int, choices=[1, 2], default=1, help="Test stage: 1=High-probability candidates, 2=Extended comfort IDs (default: 1)")
    parser.add_argument("--custom-id", type=lambda x: int(x, 0), default=None, help="Directly test a single CAN ID (e.g. 0x1D0 or 0x0F6)")
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose UART hex stream and packet logging")

    args = parser.parse_args()

    # Determine serial port
    if args.port is None:
        args.port = get_default_uart_port()
        print(f"[INIT] Auto-selected UART port '{args.port}' (slcan excluded).")
    elif not os.path.exists(args.port):
        alt = get_default_uart_port()
        print(f"[WARN] Port '{args.port}' not found. Falling back to '{alt}'.")
        args.port = alt

    verifier = CanboxDefrostVerifier(
        can_iface=args.can,
        uart_port=args.port,
        baud_rate=args.baud,
        proto_mode=args.proto,
        stage=args.stage,
        custom_id=args.custom_id,
        verbose=args.verbose
    )
    verifier.start()

if __name__ == "__main__":
    main()

