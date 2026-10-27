#!/usr/bin/env python3
"""
Peugeot 407 Dual-Zone Climate Control (HVAC) Physical Workbench Verifier
========================================================================
Automated verification tool for PSA Comfort CAN2004 dual-zone climate:
- Manages keepalive (0x036, 0x0F6, 0x128, 0x15B, 0x168)
- Manages 900 RPM engine & compressor authorization (0x0B6, 0x12D)
- Manages Hiworld handshake (0x24 -> 407 model code 0x22)
- Performs dual-zone temperature sweep (LO, 14.0 - 28.0 C, HI)
- Performs fan speed & distribution sweeps
- Performs climate flags & toggle tests
- Supports interactive delta sniffing for physical fascia operation
"""

import sys
import os
import time
import socket
import struct
import threading
import datetime
import argparse
import serial
from typing import Dict, List, Optional, Tuple

try:
    from canbox_usb_ports import find_canbox_uart_device
except ImportError:
    def find_canbox_uart_device():
        return "/dev/ttyUSB0"

CAN_FRAME_FMT = "=IB3x8s"

def calc_hiworld_cs(length: int, cmd: int, payload: bytes) -> int:
    return ((length + cmd + sum(payload)) - 1) & 0xFF

def calc_raise_cs(cmd: int, payload: bytes) -> int:
    s = (cmd + len(payload) + sum(payload)) & 0xFF
    return (~s) & 0xFF

class ClimateWorkbenchTester:
    def __init__(self, can_iface: str = "can0", uart_port: Optional[str] = None, baud_rate: int = 38400, verbose: bool = False):
        self.can_iface = can_iface
        self.uart_port = uart_port or find_canbox_uart_device()
        self.baud_rate = baud_rate
        self.verbose = verbose
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

        # CAN and Serial handles
        self.can_sock: Optional[socket.socket] = None
        self.ser: Optional[serial.Serial] = None

        # Dynamic CAN frame table (periodically broadcasted @ 35 ms)
        self.active_can_frames: Dict[int, bytes] = {
            0x036: bytes([0x0E, 0x00, 0x00, 0x2A, 0x01, 0x00, 0x00, 0xA0]), # Wakeup / Active
            0x0B6: bytes([0x1C, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0]), # 900 RPM engine
            0x0F6: bytes([0x88, 0x00, 0x00, 0x00, 0x00, 0x50, 0x00, 0x20]), # Economy OFF, Ext temp 24C
            0x128: bytes([0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]), # Ignition (+APC)
            0x12D: bytes([0x80, 0x32, 0x32, 0x00, 0x64, 0x00, 0x00, 0x80]), # Compressor authorized
            0x15B: bytes([0x01, 0x03, 0xB1, 0x40, 0x00, 0xD0, 0x00, 0x10]), # BSI power state
            0x168: bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]), # Cluster
        }

        # Latest received Clima state
        self.last_hiworld_clima: Optional[bytes] = None
        self.last_raise_clima: Optional[bytes] = None
        self.clima_update_event = threading.Event()

        # All received packets log
        self.received_clima_packets: List[Tuple[float, str, bytes]] = []

    def open(self):
        self.can_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.can_sock.bind((self.can_iface,))
        self.ser = serial.Serial(self.uart_port, self.baud_rate, timeout=0.02)
        print(f"[OK] CAN ({self.can_iface}) & UART ({self.uart_port} @ {self.baud_rate}) opened.")

    def close(self):
        self.stop_event.set()
        if self.ser:
            self.ser.close()
        if self.can_sock:
            self.can_sock.close()

    def send_can(self, can_id: int, data: bytes):
        if not self.can_sock:
            return
        f = struct.pack(CAN_FRAME_FMT, can_id, len(data), data.ljust(8, b'\x00'))
        try:
            self.can_sock.send(f)
        except Exception:
            pass

    def send_hiworld(self, cmd: int, payload: bytes):
        if not self.ser:
            return
        length = len(payload)
        cs = calc_hiworld_cs(length, cmd, payload)
        pkt = bytes([0x5A, 0xA5, length, cmd]) + payload + bytes([cs])
        self.ser.write(pkt)

    def set_can_override(self, can_id: int, data: Optional[bytes]):
        with self.lock:
            if data is None:
                self.active_can_frames.pop(can_id, None)
            else:
                self.active_can_frames[can_id] = data

    def start_threads(self):
        # 1. CAN Broadcaster thread
        def can_worker():
            while not self.stop_event.is_set():
                with self.lock:
                    items = list(self.active_can_frames.items())
                for cid, data in items:
                    self.send_can(cid, data)
                time.sleep(0.035)

        t_can = threading.Thread(target=can_worker, daemon=True)
        t_can.start()

        # 2. UART Handshake / Keepalive thread (Sends 0x24 model code every 3000 ms)
        def uart_tx_worker():
            # Send initial 407 model selection: Model 0x22 (Peugeot 407), Variant 0x00
            time.sleep(0.1)
            self.send_hiworld(0x24, bytes([0x22, 0x00]))
            while not self.stop_event.is_set():
                time.sleep(3.0)
                self.send_hiworld(0x24, bytes([0x22, 0x00]))

        t_tx = threading.Thread(target=uart_tx_worker, daemon=True)
        t_tx.start()

        # 3. UART Receiver thread
        def uart_rx_worker():
            rx_buf = bytearray()
            while not self.stop_event.is_set():
                chunk = self.ser.read(64)
                if not chunk:
                    continue
                rx_buf.extend(chunk)

                while len(rx_buf) >= 4:
                    # Hiworld Check: 0x5A 0xA5 <len> <cmd> <payload...> <cs>
                    if len(rx_buf) >= 5 and rx_buf[0] == 0x5A and rx_buf[1] == 0xA5:
                        l = rx_buf[2]
                        if l > 32:
                            rx_buf.pop(0)
                            continue
                        total_len = l + 5
                        if len(rx_buf) < total_len:
                            break
                        cmd = rx_buf[3]
                        payload = bytes(rx_buf[4:4 + l])
                        cs = rx_buf[4 + l]
                        if cs == calc_hiworld_cs(l, cmd, payload):
                            del rx_buf[:total_len]
                            if cmd == 0x31:
                                self.last_hiworld_clima = payload
                                self.received_clima_packets.append((time.time(), "HIWORLD_0x31", payload))
                                self.clima_update_event.set()
                                if self.verbose:
                                    print(f"  [UART 0x31] {payload.hex(' ')}")
                            elif self.verbose and cmd in (0x11, 0x12, 0x42, 0x71, 0x72, 0x79):
                                print(f"  [UART 0x{cmd:02X}] {payload.hex(' ')}")
                            continue
                        else:
                            rx_buf.pop(0)
                            continue

                    # Raise Check: 0x2E <cmd> <len> <payload...> <cs>
                    elif rx_buf[0] == 0x2E:
                        cmd = rx_buf[1]
                        l = rx_buf[2]
                        if l > 32:
                            rx_buf.pop(0)
                            continue
                        total_len = 3 + l + 1
                        if len(rx_buf) < total_len:
                            break
                        payload = bytes(rx_buf[3:3 + l])
                        cs = rx_buf[3 + l]
                        if cs == calc_raise_cs(cmd, payload):
                            del rx_buf[:total_len]
                            if cmd == 0x21:
                                self.last_raise_clima = payload
                                self.received_clima_packets.append((time.time(), "RAISE_0x21", payload))
                                self.clima_update_event.set()
                                if self.verbose:
                                    print(f"  [UART RAISE 0x21] {payload.hex(' ')}")
                            continue
                        else:
                            rx_buf.pop(0)
                            continue
                    else:
                        rx_buf.pop(0)

        t_rx = threading.Thread(target=uart_rx_worker, daemon=True)
        t_rx.start()

    def wait_for_clima(self, timeout: float = 1.0) -> Optional[bytes]:
        self.clima_update_event.clear()
        if self.clima_update_event.wait(timeout):
            return self.last_hiworld_clima
        return self.last_hiworld_clima

    def run_sweep(self):
        print("\n" + "=" * 65)
        print("  STEP 1: INITIAL CLIMATE TELEMETRY SANITY CHECK")
        print("=" * 65)
        # Establish baseline 0x1D0 and 0x1E3
        base_1d0 = bytes([0x29, 0x80, 0x07, 0x11, 0x20, 0x0B, 0x0B])
        base_1e3 = bytes([0x1C, 0x40, 0x0B, 0x0B, 0x00, 0x00, 0x07])
        self.set_can_override(0x1D0, base_1d0)
        self.set_can_override(0x1E3, base_1e3)

        clima = self.wait_for_clima(timeout=2.0)
        if clima:
            print(f"[SUCCESS] Canbox actively transmitting Hiworld 0x31: {clima.hex(' ')}")
        else:
            print("[WARN] No immediate 0x31 packet. Checking ongoing reception...")

        # ---------------------------------------------------------------------
        print("\n" + "=" * 65)
        print("  STEP 2: DUAL-ZONE TEMPERATURE SWEEPS (INDICES 0..22)")
        print("=" * 65)
        # Test all 23 indices: LO (0), 14.0C (1) .. 28.0C (21), HI (22)
        temp_sweep_results = []
        for idx in range(23):
            # Inject on both 0x1D0 and 0x1E3
            data_1d0 = bytes([0x29, 0x80, 0x07, 0x11, 0x20, idx, idx])
            data_1e3 = bytes([0x1C, 0x40, idx, idx, 0x00, 0x00, 0x07])
            self.set_can_override(0x1D0, data_1d0)
            self.set_can_override(0x1E3, data_1e3)
            time.sleep(0.25)
            clima = self.wait_for_clima(timeout=0.3)
            if clima and len(clima) >= 8:
                dr_temp = clima[6]
                ps_temp = clima[7]
                temp_sweep_results.append((idx, dr_temp, ps_temp, clima.hex(' ')))
                display_str = "LO" if idx == 0 else ("HI" if idx == 22 else f"{13.5 + idx * 0.5:.1f} C")
                print(f"  Index {idx:2d} ({display_str:7s}) -> Hiworld Driver=0x{dr_temp:02X} ({dr_temp/2.0:.1f}C) Pass=0x{ps_temp:02X} ({ps_temp/2.0:.1f}C) | Raw: {clima.hex(' ')}")
            else:
                print(f"  Index {idx:2d} -> [NO PACKET]")

        # Test Asymmetric dual-zone
        print("\n--- Testing Asymmetric Dual-Zone Temperatures ---")
        asym_pairs = [(0, 22), (1, 21), (7, 15), (11, 14)] # (LO, HI), (14C, 28C), (19C, 23C), (21C, 22.5C)
        for l_idx, r_idx in asym_pairs:
            data_1d0 = bytes([0x29, 0x80, 0x07, 0x11, 0x20, l_idx, r_idx])
            data_1e3 = bytes([0x1D, 0x40, l_idx, r_idx, 0x00, 0x00, 0x07]) # 0x1D = Dual mode active
            self.set_can_override(0x1D0, data_1d0)
            self.set_can_override(0x1E3, data_1e3)
            time.sleep(0.25)
            clima = self.wait_for_clima(timeout=0.3)
            if clima and len(clima) >= 8:
                dr_temp = clima[6]
                ps_temp = clima[7]
                print(f"  Left Idx {l_idx:2d}, Right Idx {r_idx:2d} -> Hiworld Driver=0x{dr_temp:02X} ({dr_temp/2.0:.1f}C), Pass=0x{ps_temp:02X} ({ps_temp/2.0:.1f}C)")

        # ---------------------------------------------------------------------
        print("\n" + "=" * 65)
        print("  STEP 3: BLOWER FAN SPEED SWEEP (0..8 + AUTO)")
        print("=" * 65)
        # 0x1D0 Byte 2 Fan bits: 0x0F = OFF/0, 0x00 = Speed 1 .. 0x07 = Speed 8
        fan_tests = [
            (0x0F, "Fan OFF / Level 0"),
            (0x00, "Fan Speed 1"),
            (0x01, "Fan Speed 2"),
            (0x02, "Fan Speed 3"),
            (0x03, "Fan Speed 4"),
            (0x04, "Fan Speed 5"),
            (0x05, "Fan Speed 6"),
            (0x06, "Fan Speed 7"),
            (0x07, "Fan Speed 8 (MAX)"),
        ]
        for fan_code, desc in fan_tests:
            data_1d0 = bytes([0x29, 0x80, fan_code, 0x11, 0x20, 0x0B, 0x0B])
            data_1e3 = bytes([0x1C, 0x40, 0x0B, 0x0B, 0x00, 0x00, fan_code if fan_code != 0x0F else 0x00])
            self.set_can_override(0x1D0, data_1d0)
            self.set_can_override(0x1E3, data_1e3)
            time.sleep(0.25)
            clima = self.wait_for_clima(timeout=0.3)
            if clima and len(clima) >= 6:
                uart_fan = clima[5]
                print(f"  {desc:20s} (0x1D0 Byte 2=0x{fan_code:02X}) -> Hiworld Byte 5 = {uart_fan} | Raw: {clima.hex(' ')}")

        # ---------------------------------------------------------------------
        print("\n" + "=" * 65)
        print("  STEP 4: AIR DISTRIBUTION MODES (0x1D0 Byte 3 / 0x1E3 Bytes 4-5)")
        print("=" * 65)
        # Test vent distribution codes: 0x00..0x08
        dist_modes = [
            (0x00, "AUTO distribution"),
            (0x01, "Windshield / Demist"),
            (0x02, "Face / Center Vents"),
            (0x03, "Floor / Feet Vents"),
            (0x04, "Bi-level (Screen + Floor)"),
            (0x05, "Face + Floor"),
            (0x06, "Windshield + Floor (alt)"),
            (0x07, "Windshield + Face"),
            (0x08, "Floor / Down (alt)"),
        ]
        for mode_code, desc in dist_modes:
            # 0x1D0: upper nibble = Left, lower nibble = Right
            b3 = (mode_code << 4) | mode_code
            data_1d0 = bytes([0x29, 0x80, 0x07, b3, 0x20, 0x0B, 0x0B])
            # 0x1E3: Byte 4 upper nibble = Left, Byte 5 upper nibble = Right
            data_1e3 = bytes([0x1C, 0x40, 0x0B, 0x0B, mode_code << 4, mode_code << 4, 0x07])
            self.set_can_override(0x1D0, data_1d0)
            self.set_can_override(0x1E3, data_1e3)
            time.sleep(0.25)
            clima = self.wait_for_clima(timeout=0.3)
            if clima and len(clima) >= 5:
                b4 = clima[4]
                dr_mode = b4 & 0x0F
                ps_mode = (b4 >> 4) & 0x0F
                print(f"  {desc:26s} (CAN Code 0x{mode_code:02X}) -> Hiworld Byte 4: 0x{b4:02X} (Dr={dr_mode:2d}, Pass={ps_mode:2d}) | Raw: {clima.hex(' ')}")

        # ---------------------------------------------------------------------
        print("\n" + "=" * 65)
        print("  STEP 5: CLIMATE FLAGS & TOGGLES")
        print("=" * 65)
        flag_tests = [
            # Description, 1D0, 1E3
            ("A/C Compressor ON", 
             bytes([0x29, 0x80, 0x07, 0x11, 0x20, 0x0B, 0x0B]),
             bytes([0x1C, 0x40, 0x0B, 0x0B, 0x00, 0x00, 0x07])),
            ("A/C Compressor OFF", 
             bytes([0x29, 0x80, 0x07, 0x11, 0x20, 0x0B, 0x0B]),
             bytes([0x1C, 0x00, 0x0B, 0x0B, 0x00, 0x00, 0x07])),
            ("Auto Mode ON", 
             bytes([0x29, 0x80, 0x07, 0x00, 0x20, 0x0B, 0x0B]),
             bytes([0x1C, 0x40, 0x0B, 0x0B, 0x00, 0x00, 0x07])),
            ("Auto Mode OFF (Manual)", 
             bytes([0x29, 0x80, 0x07, 0x11, 0x20, 0x0B, 0x0B]),
             bytes([0x10, 0x40, 0x0B, 0x0B, 0x10, 0x10, 0x07])),
            ("Dual Mode ON (Split)", 
             bytes([0x29, 0x80, 0x07, 0x11, 0x20, 0x0B, 0x0E]),
             bytes([0x1D, 0x40, 0x0B, 0x0E, 0x10, 0x10, 0x07])), # Bit 0 in 1E3
            ("Mono Mode ON (Sync)", 
             bytes([0x29, 0x80, 0x07, 0x11, 0x20, 0x0B, 0x0B]),
             bytes([0x1C, 0x40, 0x0B, 0x0B, 0x10, 0x10, 0x07])), # Bit 0 = 0 in 1E3
            ("Cabin Recirculation ON", 
             bytes([0x29, 0x80, 0x07, 0x11, 0x10, 0x0B, 0x0B]), # 1D0 b4=0x10
             bytes([0x85, 0x40, 0x0B, 0x0B, 0x00, 0x00, 0x07])), # 1E3 b0=0x85
            ("Auto Air Intake (AQS) ON", 
             bytes([0x29, 0x80, 0x07, 0x11, 0x00, 0x0B, 0x0B]), # 1D0 b4=0x00
             bytes([0x15, 0x40, 0x0B, 0x0B, 0x00, 0x00, 0x07])), # 1E3 b0=0x15
            ("Forced Fresh Air ON", 
             bytes([0x29, 0x80, 0x07, 0x11, 0x20, 0x0B, 0x0B]), # 1D0 b4=0x20
             bytes([0x05, 0x40, 0x0B, 0x0B, 0x00, 0x00, 0x07])), # 1E3 b0=0x05
            ("Front Windshield Demist ON", 
             bytes([0x19, 0x80, 0x07, 0x11, 0x20, 0x0B, 0x0B]), # 1D0 b0=0x19
             bytes([0x14, 0xB0, 0x0B, 0x0B, 0x10, 0x10, 0x07])), # 1E3 b1=0xB0
            ("Rear Defrost Request ON (1D0 b0 bit 6)", 
             bytes([0x69, 0x80, 0x07, 0x11, 0x20, 0x0B, 0x0B]), # 0x29 | 0x40 = 0x69
             bytes([0x1C, 0x40, 0x0B, 0x0B, 0x00, 0x00, 0x07])),
            ("Rear Defrost Indicator ON (1E3 b1 bit 3)", 
             bytes([0x29, 0x80, 0x07, 0x11, 0x20, 0x0B, 0x0B]),
             bytes([0x1C, 0x48, 0x0B, 0x0B, 0x00, 0x00, 0x07])), # 0x40 | 0x08 = 0x48
            ("Rear Defrost Both ON (1D0+1E3)", 
             bytes([0x69, 0x80, 0x07, 0x11, 0x20, 0x0B, 0x0B]),
             bytes([0x1C, 0x48, 0x0B, 0x0B, 0x00, 0x00, 0x07])),
        ]
        for desc, d_1d0, d_1e3 in flag_tests:
            self.set_can_override(0x1D0, d_1d0)
            self.set_can_override(0x1E3, d_1e3)
            time.sleep(0.3)
            clima = self.wait_for_clima(timeout=0.3)
            if clima:
                b0 = clima[0] if len(clima) > 0 else 0
                b1 = clima[1] if len(clima) > 1 else 0
                b2 = clima[2] if len(clima) > 2 else 0
                pwr = bool(b0 & 0x40)
                ac = bool(b0 & 0x01)
                auto = bool(b0 & 0x08)
                dual = bool(b0 & 0x04)
                recirc = bool(b1 & 0x10)
                aqs = bool(b1 & 0x08)
                f_def = bool(b2 & 0x10)
                r_def = bool(b2 & 0x20)
                print(f"  {desc:38s} -> B0:0x{b0:02X} B1:0x{b1:02X} B2:0x{b2:02X} (Pwr={pwr} AC={ac} Auto={auto} Dual={dual} Recirc={recirc} AQS={aqs} FrontDef={f_def} RearDef={r_def})")

        print("\n" + "=" * 65)
        print("  SWEEP COMPLETE")
        print("=" * 65)

def main():
    parser = argparse.ArgumentParser(description="Peugeot 407 Dual-Zone Climate Workbench Verifier")
    parser.add_argument("--can", default="can0", help="CAN interface (default: can0)")
    parser.add_argument("--port", default=None, help="UART serial port (auto-discovered if omitted)")
    parser.add_argument("--baud", type=int, default=38400, help="Baud rate (default: 38400)")
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose UART logging")
    args = parser.parse_args()

    tester = ClimateWorkbenchTester(can_iface=args.can, uart_port=args.port, baud_rate=args.baud, verbose=args.verbose)
    try:
        tester.open()
        tester.start_threads()
        tester.run_sweep()
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    finally:
        tester.close()

if __name__ == "__main__":
    main()

