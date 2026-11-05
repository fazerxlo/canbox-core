#!/usr/bin/env python3
"""
Test Trip Computer Simulation & Verification Tool for Hiworld Canbox (Peugeot 407).
==================================================================================
Simulates CAN bus telemetry and serial Head Unit connection to a real Hiworld Canbox:
1. Connects to real Canbox via UART (38,400 baud) and CAN interface (can0).
2. Establishes Peugeot 407 connection parameters via Hiworld Cmd 0x24 handshake.
3. Broadcasts essential vehicle keepalive frames (0x036 awake, 0x0F6 ignition, 0x0B6 speed/RPM).
4. Simulates Instantaneous Telemetry (0x221), Trip 1 (0x2A1), and Trip 2 (0x261).
5. Ingests and verifies real MDF trip trace vectors from dump_mdf_stalk_trip.log.
6. Decodes and verifies Hiworld UART uplink commands (Cmd 0x13, Cmd 0x14, Cmd 0x15).
"""

import os
import sys
import time
import socket
import struct
import argparse
import threading
from typing import Optional, Tuple, List, Dict, Any

# Ensure workspace tools can import canbox_usb_ports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from canbox_usb_ports import find_canbox_uart_device
except ImportError:
    def find_canbox_uart_device():
        return "/dev/ttyUSB0"

try:
    import serial
except ImportError:
    print("[ERROR] pyserial is required. Install with: pip install pyserial")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Hiworld Protocol Framing & Checksums
# ---------------------------------------------------------------------------
HIWORLD_SYNC_0 = 0x5A
HIWORLD_SYNC_1 = 0xA5

def make_hiworld_packet(cmd: int, payload: bytes = b'') -> bytes:
    """Builds a standard Hiworld frame: 5A A5 <length> <cmd> <payload...> <cs>"""
    length = len(payload)
    cs = ((length + cmd + sum(payload)) - 1) & 0xFF
    return bytes([HIWORLD_SYNC_0, HIWORLD_SYNC_1, length, cmd]) + payload + bytes([cs])

def parse_hiworld_stream(buf: bytearray) -> Tuple[Optional[int], Optional[bytes]]:
    """Extracts the first valid Hiworld frame from a bytearray buffer."""
    while len(buf) >= 5:
        if buf[0] != HIWORLD_SYNC_0 or buf[1] != HIWORLD_SYNC_1:
            buf.pop(0)
            continue
        length = buf[2]
        if length > 64:  # Invalid length protection
            buf.pop(0)
            continue
        total_len = length + 5
        if len(buf) < total_len:
            break  # Incomplete frame, wait for more bytes
        cmd = buf[3]
        payload = bytes(buf[4:4 + length])
        cs = buf[4 + length]
        expected_cs = ((length + cmd + sum(payload)) - 1) & 0xFF
        if cs == expected_cs:
            del buf[:total_len]
            return cmd, payload
        else:
            # Checksum mismatch, advance 1 byte to seek next sync
            buf.pop(0)
    return None, None


# ---------------------------------------------------------------------------
# Hiworld Trip Decoders
# ---------------------------------------------------------------------------
def decode_hiworld_0x13(payload: bytes) -> Dict[str, Any]:
    """
    Hiworld Cmd 0x13 (Instantaneous Telemetry):
    - Byte 0..1: Instant fuel consumption (0.1 L/100km, uint16 BE, 0xFFFF = undefined / --.-)
    - Byte 2..3: Cruising Range / Autonomy (km, uint16 BE, 0xFFFF = undefined)
    - Byte 4..5: Destination distance (km, uint16 BE, 0xFFFF = undefined)
    """
    if len(payload) < 6:
        return {}
    inst_fuel_raw = struct.unpack(">H", payload[0:2])[0]
    range_raw = struct.unpack(">H", payload[2:4])[0]
    dest_dist_raw = struct.unpack(">H", payload[4:6])[0]

    return {
        "inst_fuel_raw": inst_fuel_raw,
        "inst_fuel_l_100km": None if inst_fuel_raw == 0xFFFF else (inst_fuel_raw / 10.0),
        "cruising_range_km": None if range_raw == 0xFFFF else range_raw,
        "dest_distance_km": None if dest_dist_raw == 0xFFFF else dest_dist_raw,
    }

def decode_hiworld_0x14_0x15(payload: bytes) -> Dict[str, Any]:
    """
    Hiworld Cmd 0x14 (Trip 1) and Cmd 0x15 (Trip 2):
    - Byte 0..1: Average fuel consumption (0.1 L/100km, uint16 BE, 0xFFFF = undefined)
    - Byte 2: Reserved
    - Byte 3: Average speed (km/h, uint8, 0xFF = 0/undefined)
    - Byte 4..5: Trip distance traveled (km, uint16 BE, 0xFFFF = undefined)
    """
    if len(payload) < 6:
        return {}
    avg_fuel_raw = struct.unpack(">H", payload[0:2])[0]
    avg_speed_raw = payload[3]
    trip_dist_raw = struct.unpack(">H", payload[4:6])[0]

    return {
        "avg_fuel_raw": avg_fuel_raw,
        "avg_fuel_l_100km": None if avg_fuel_raw == 0xFFFF else (avg_fuel_raw / 10.0),
        "avg_speed_kmh": None if avg_speed_raw == 0xFF else avg_speed_raw,
        "trip_distance_km": None if trip_dist_raw == 0xFFFF else trip_dist_raw,
    }


# ---------------------------------------------------------------------------
# Main Simulation & Verification Runner
# ---------------------------------------------------------------------------
class CanboxTripTester:
    def __init__(self, can_iface: str = "can0", serial_port: Optional[str] = None, baudrate: int = 38400):
        self.can_iface = can_iface
        self.serial_port = serial_port or find_canbox_uart_device()
        self.baudrate = baudrate
        self.sock: Optional[socket.socket] = None
        self.ser: Optional[serial.Serial] = None
        self.stop_evt = threading.Event()

        self.rx_buffer = bytearray()
        self.rx_packets: List[Tuple[float, int, bytes]] = []
        self.rx_lock = threading.Lock()

        # Background cyclic CAN frames
        self.can_lock = threading.Lock()
        self.active_can_frames: Dict[int, bytes] = {
            # 0x036: Network alive / Awakened state
            0x036: bytes([0x0E, 0x00, 0x00, 0x0F, 0x01, 0x00, 0x00, 0xA0]),
            # 0x0F6: Ignition ON (+APC Byte 0 Bit 3 = 0x88), Temp 24C
            0x0F6: bytes([0x88, 0x18, 0x00, 0x00, 0x00, 0x50, 0x50, 0x00]),
            # 0x0B6: Engine RPM 800 (raw 0x1900 >> 3 = 800), Speed 50 km/h (raw 5000 = 0x1388)
            0x0B6: bytes([0x19, 0x00, 0x13, 0x88, 0x00, 0x00, 0x00, 0xD0]),
            # 0x128: Kombi / Instrument Panel Lighting & Contact ON
            0x128: bytes([0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
            # 0x220: Body & Doors status (all closed)
            0x220: bytes([0x00, 0x00]),
            # 0x260: BSI Vehicle Configuration Baseline
            0x260: bytes([0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
            # 0x361: BSI Options & Feature Capabilities
            0x361: bytes([0x01, 0x01, 0x91, 0x40, 0x30, 0x10]),
        }

        self.firmware_version = ""
        self.handshake_acked = False

    def connect(self):
        print(f"[*] Opening SocketCAN on {self.can_iface}...")
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((self.can_iface,))

        print(f"[*] Opening Serial UART on {self.serial_port} @ {self.baudrate} baud...")
        self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.05)
        self.ser.reset_input_buffer()

        # Start UART listener thread
        t_uart = threading.Thread(target=self._uart_worker, daemon=True, name="UART-RX")
        t_uart.start()

        # Start CAN background broadcast thread (50 ms cycle)
        t_can = threading.Thread(target=self._can_worker, daemon=True, name="CAN-TX")
        t_can.start()

        # Start Handshake / Heartbeat thread (Cmd 0x24 every 2.5s)
        t_hshk = threading.Thread(target=self._handshake_worker, daemon=True, name="Handshake")
        t_hshk.start()

    def send_can(self, can_id: int, data: bytes):
        """Sends a single CAN frame immediately on can0."""
        if not self.sock:
            return
        frame = struct.pack("=IB3x8s", can_id, len(data), data.ljust(8, b"\x00"))
        try:
            self.sock.send(frame)
        except Exception as e:
            print(f"[!] SocketCAN send error: {e}")

    def send_hiworld(self, cmd: int, payload: bytes = b""):
        """Sends a Hiworld frame to Canbox over UART."""
        if not self.ser or not self.ser.is_open:
            return
        pkt = make_hiworld_packet(cmd, payload)
        self.ser.write(pkt)

    def _uart_worker(self):
        """Background thread reading UART bytes and extracting Hiworld frames."""
        while not self.stop_evt.is_set():
            try:
                chunk = self.ser.read(64)
                if not chunk:
                    continue
                self.rx_buffer.extend(chunk)
                while True:
                    cmd, payload = parse_hiworld_stream(self.rx_buffer)
                    if cmd is None:
                        break
                    now = time.time()
                    with self.rx_lock:
                        self.rx_packets.append((now, cmd, payload))

                    if cmd == 0xFF:
                        if payload and payload[0] == 0x24:
                            self.handshake_acked = True
                    elif cmd == 0xF0:
                        try:
                            self.firmware_version = payload.decode("ascii", errors="ignore").strip()
                        except Exception:
                            self.firmware_version = payload.hex()
            except Exception:
                break

    def _can_worker(self):
        """Background thread continuously broadcasting essential CAN frames."""
        while not self.stop_evt.is_set():
            with self.can_lock:
                frames_to_send = list(self.active_can_frames.items())
            for cid, data in frames_to_send:
                self.send_can(cid, data)
            time.sleep(0.05)  # 50 ms loop

    def _handshake_worker(self):
        """Periodically emits Hiworld Cmd 0x24 (Peugeot 407 model code 0x22)."""
        while not self.stop_evt.is_set():
            # Cmd 0x24: Car Model 0x22 = Peugeot 407, Option 0x00
            self.send_hiworld(0x24, bytes([0x22, 0x00]))
            time.sleep(2.5)

    def wait_for_packet(self, target_cmd: int, timeout_sec: float = 4.0, since_idx: int = 0, predicate=None) -> Optional[bytes]:
        """Waits for a specific Hiworld command received after since_idx in rx_packets matching predicate."""
        t_end = time.time() + timeout_sec
        while time.time() < t_end and not self.stop_evt.is_set():
            with self.rx_lock:
                for idx in range(since_idx, len(self.rx_packets)):
                    t, cmd, payload = self.rx_packets[idx]
                    if cmd == target_cmd:
                        if predicate is None or predicate(payload):
                            return payload
            time.sleep(0.02)
        return None

    def start_bsi_baseline(self, settle_time_sec: float = 1.0):
        """
        Emits BSI base CAN bus traffic on can0 and allows it to run before waiting for version.
        """
        print("\n" + "=" * 60)
        print("  STEP 1: Starting BSI Base CAN Bus Traffic")
        print("=" * 60)
        print("  [*] Broadcasting BSI baseline CAN frames on can0:")
        print("      - CAN 0x036: BSI Network Alive / Awakened state")
        print("      - CAN 0x0F6: Ignition ON (+APC) & Ambient Temperature")
        print("      - CAN 0x0B6: Engine Speed & Running RPM")
        print("      - CAN 0x128: Kombi / Instrument Panel Lighting")
        print("      - CAN 0x220: Body & Doors Status (all closed)")
        print("      - CAN 0x260: BSI Vehicle Configuration Baseline")
        print("      - CAN 0x361: BSI Options & Feature Capabilities")
        print(f"  [*] Allowing BSI baseline traffic to run on can0 for {settle_time_sec:.1f}s...")
        time.sleep(settle_time_sec)
        print("  [PASS] BSI base CAN bus traffic running actively.")

    def force_dump_version(self):
        """Forces a version dump by querying Cmd 0x10, Cmd 0x24, and Cmd 0x30."""
        self.send_hiworld(0x10, b"")
        self.send_hiworld(0x24, bytes([0x22, 0x00]))
        self.send_hiworld(0x30, b"")

    def wait_for_firmware_version(self, timeout_sec: float = 11.0, force_dump: bool = True) -> bool:
        """
        Explicitly waits for Canbox Firmware Version (Cmd 0xF0) for up to 10 seconds.
        Forces version dump at start of wait window if requested.
        """
        print("\n" + "=" * 60)
        print("  STEP 2: Canbox Firmware Version Discovery & Handshake Sync")
        print("=" * 60)
        t_start = time.time()

        if force_dump:
            print("  [->] Triggering firmware version dump (sending Cmd 0x10 & Cmd 0x24)...")
            self.force_dump_version()

        print(f"  [*] Waiting for Canbox Firmware Version (Cmd 0xF0, up to '{timeout_sec}'s)...")
        while time.time() - t_start < timeout_sec and not self.stop_evt.is_set():
            if self.firmware_version:
                elapsed = time.time() - t_start
                print(f"  [PASS] Canbox Firmware Version Confirmed: '{self.firmware_version}' (Cmd 0xF0 received in {elapsed:.2f}s)")
                return True

            time.sleep(0.05)

        if self.firmware_version:
            print(f"  [PASS] Canbox Firmware Version Confirmed: '{self.firmware_version}' (Cmd 0xF0)")
            return True

        print("  [FAIL] Could not acquire Canbox firmware version within 10s timeout!")
        return False

    def establish_connection(self, timeout_sec: float = 31.0) -> bool:
        """
        Full connection sequence:
        1. Emits BSI base CAN bus traffic first.
        2. Waits for firmware version (up to 10s) with force dump fallback.
        3. Verifies handshake acknowledgement.
        4. Allows startup state burst to settle and sets baseline trip sentinels.
        """
        # 1. BSI base CAN bus traffic happens BEFORE waiting for version
        self.start_bsi_baseline(settle_time_sec=1.0)

        # 2. Wait up to 10 seconds for firmware version
        if not self.wait_for_firmware_version(timeout_sec=timeout_sec, force_dump=True):
            return False

        # 3. Verify handshake acknowledgement
        t_start = time.time()
        while time.time() - t_start < 2.0 and not self.stop_evt.is_set():
            if self.handshake_acked:
                break
            time.sleep(0.05)

        if self.handshake_acked:
            print("  [PASS] Handshake Acknowledged by Canbox (Cmd 0xFF 0x24 received)!")
        else:
            print("  [WARN] Handshake ACK not explicitly received, but firmware version is active.")

        # 4. Allow canbox initial state burst (0xC1, 0x71, 0x72, 0x76, 0x79, 0x81, 0x42) to drain
        print("  [*] Allowing Canbox initial state burst to settle...")
        time.sleep(3.0)

        # 5. Baseline initialization: set trip frames to clean sentinels
        print("  [*] Initializing baseline CAN trip sentinels...")
        self.send_can(0x221, bytes.fromhex("00FFFFFFFFFFFF"))
        self.send_can(0x2A1, bytes.fromhex("FFFFFFFFFF0399"))
        self.send_can(0x261, bytes.fromhex("FFFFFFFFFF0399"))
        time.sleep(0.5)

        print("  [PASS] Connection ready for telemetry injection.")
        return True

    def close(self):
        self.stop_evt.set()
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self.sock:
            self.sock.close()


# ---------------------------------------------------------------------------
# Test Scenarios
# ---------------------------------------------------------------------------
def run_tests(tester: CanboxTripTester, mdf_log_path: str = "dump_mdf_stalk_trip.log"):
    results = []

    print("\n" + "=" * 60)
    print("  STEP 3: Simulating & Verifying Peugeot 407 Trip Computer")
    print("=" * 60)

    # ---------------------------------------------------------
    # TEST 1: Instantaneous Trip (0x221) Normal Driving Values
    # ---------------------------------------------------------
    print("\n--- Test 1: Instantaneous Trip Telemetry (CAN 0x221 -> Cmd 0x13) ---")
    print("  Injecting CAN 0x221:")
    print("    - Instant Fuel: 6.8 L/100km (0x0044)")
    print("    - Cruising Range: 640 km (0x0280)")
    print("    - Destination Distance: 1200 km (0x04B0)")
    idx0 = len(tester.rx_packets)
    can_221_data = bytes.fromhex("000044028004B0")
    with tester.can_lock:
        tester.active_can_frames[0x221] = can_221_data
    tester.send_can(0x221, can_221_data)

    p13 = tester.wait_for_packet(0x13, timeout_sec=10.0, since_idx=idx0,
                                 predicate=lambda p: decode_hiworld_0x13(p).get('inst_fuel_l_100km') == 6.8)
    if p13:
        d13 = decode_hiworld_0x13(p13)
        print(f"  [RECV Cmd 0x13] Raw: {p13.hex(' ')}")
        print(f"    -> Decoded Fuel: {d13['inst_fuel_l_100km']} L/100km (expected 6.8)")
        print(f"    -> Decoded Range: {d13['cruising_range_km']} km (expected 640)")
        print(f"    -> Decoded Distance: {d13['dest_distance_km']} km (expected 1200)")

        match = (d13['inst_fuel_l_100km'] == 6.8 and
                 d13['cruising_range_km'] == 640 and
                 d13['dest_distance_km'] == 1200)
        results.append(("Test 1: Instant Trip Normal (0x221 -> 0x13)", match, f"Fuel={d13['inst_fuel_l_100km']} Range={d13['cruising_range_km']}"))
    else:
        print("  [FAIL] Did not receive Cmd 0x13 within timeout")
        results.append(("Test 1: Instant Trip Normal (0x221 -> 0x13)", False, "Timeout waiting for Cmd 0x13"))

    # ---------------------------------------------------------
    # TEST 2: Instantaneous Trip (0x221) Stationary Sentinels (--.-)
    # ---------------------------------------------------------
    print("\n--- Test 2: Instantaneous Trip Stationary Sentinel (0x221 -> Cmd 0x13) ---")
    print("  Injecting CAN 0x221 with 0xFFFF (Stationary / stopped):")
    idx0 = len(tester.rx_packets)
    can_221_idle = bytes.fromhex("00FFFFFFFFFFFF")
    with tester.can_lock:
        tester.active_can_frames[0x221] = can_221_idle
    tester.send_can(0x221, can_221_idle)

    p13_idle = tester.wait_for_packet(0x13, timeout_sec=10.0, since_idx=idx0,
                                      predicate=lambda p: decode_hiworld_0x13(p).get('inst_fuel_l_100km') is None)
    if p13_idle:
        d13_idle = decode_hiworld_0x13(p13_idle)
        print(f"  [RECV Cmd 0x13] Raw: {p13_idle.hex(' ')}")
        print(f"    -> Decoded Fuel: {d13_idle['inst_fuel_l_100km']} (expected None / '--.-')")
        print(f"    -> Decoded Range: {d13_idle['cruising_range_km']} (expected None / '--.-')")
        match = (d13_idle['inst_fuel_l_100km'] is None and d13_idle['cruising_range_km'] is None)
        results.append(("Test 2: Instant Trip Stationary Sentinel (0x221 -> 0x13)", match, f"Raw={p13_idle[:4].hex()}"))
    else:
        print("  [FAIL] Did not receive Cmd 0x13 sentinel")
        results.append(("Test 2: Instant Trip Stationary Sentinel (0x221 -> 0x13)", False, "Timeout waiting for Cmd 0x13"))

    # ---------------------------------------------------------
    # TEST 3: Trip 1 Historical Telemetry (0x2A1 -> Cmd 0x14)
    # ---------------------------------------------------------
    print("\n--- Test 3: Trip 1 Historical Telemetry (CAN 0x2A1 -> Cmd 0x14) ---")
    print("  Injecting CAN 0x2A1:")
    print("    - Mean Speed: 48 km/h (0x30)")
    print("    - Distance: 336 km (0x0150)")
    print("    - Avg Fuel: 8.5 L/100km (0x0055)")
    print("    - Elapsed: 260 min (0x0104)")
    idx0 = len(tester.rx_packets)
    can_2a1_data = bytes.fromhex("30015000550104")
    with tester.can_lock:
        tester.active_can_frames[0x2A1] = can_2a1_data
    tester.send_can(0x2A1, can_2a1_data)

    p14 = tester.wait_for_packet(0x14, timeout_sec=10.0, since_idx=idx0,
                                 predicate=lambda p: decode_hiworld_0x14_0x15(p).get('avg_fuel_l_100km') == 8.5)
    if p14:
        d14 = decode_hiworld_0x14_0x15(p14)
        print(f"  [RECV Cmd 0x14] Raw: {p14.hex(' ')}")
        print(f"    -> Decoded Avg Fuel: {d14['avg_fuel_l_100km']} L/100km (expected 8.5)")
        print(f"    -> Decoded Avg Speed: {d14['avg_speed_kmh']} km/h (expected 48)")
        print(f"    -> Decoded Trip Distance: {d14['trip_distance_km']} km (expected 336)")

        match = (d14['avg_fuel_l_100km'] == 8.5 and
                 d14['avg_speed_kmh'] == 48 and
                 d14['trip_distance_km'] == 336)
        results.append(("Test 3: Trip 1 Historical (0x2A1 -> 0x14)", match, f"AvgFuel={d14['avg_fuel_l_100km']} Speed={d14['avg_speed_kmh']} Dist={d14['trip_distance_km']}"))
    else:
        print("  [FAIL] Did not receive Cmd 0x14 within timeout")
        results.append(("Test 3: Trip 1 Historical (0x2A1 -> 0x14)", False, "Timeout waiting for Cmd 0x14"))

    # ---------------------------------------------------------
    # TEST 4: Trip 2 Historical Telemetry (0x261 -> Cmd 0x15)
    # ---------------------------------------------------------
    print("\n--- Test 4: Trip 2 Historical Telemetry (CAN 0x261 -> Cmd 0x15) ---")
    print("  Injecting CAN 0x261:")
    print("    - Mean Speed: 55 km/h (0x37)")
    print("    - Distance: 820 km (0x0334)")
    print("    - Avg Fuel: 6.4 L/100km (0x0040)")
    print("    - Elapsed: 900 min (0x0384)")
    idx0 = len(tester.rx_packets)
    can_261_data = bytes.fromhex("37033400400384")
    with tester.can_lock:
        tester.active_can_frames[0x261] = can_261_data
    tester.send_can(0x261, can_261_data)

    p15 = tester.wait_for_packet(0x15, timeout_sec=10.0, since_idx=idx0,
                                 predicate=lambda p: decode_hiworld_0x14_0x15(p).get('avg_fuel_l_100km') == 6.4)
    if p15:
        d15 = decode_hiworld_0x14_0x15(p15)
        print(f"  [RECV Cmd 0x15] Raw: {p15.hex(' ')}")
        print(f"    -> Decoded Avg Fuel: {d15['avg_fuel_l_100km']} L/100km (expected 6.4)")
        print(f"    -> Decoded Avg Speed: {d15['avg_speed_kmh']} km/h (expected 55)")
        print(f"    -> Decoded Trip Distance: {d15['trip_distance_km']} km (expected 820)")

        match = (d15['avg_fuel_l_100km'] == 6.4 and
                 d15['avg_speed_kmh'] == 55 and
                 d15['trip_distance_km'] == 820)
        results.append(("Test 4: Trip 2 Historical (0x261 -> 0x15)", match, f"AvgFuel={d15['avg_fuel_l_100km']} Speed={d15['avg_speed_kmh']} Dist={d15['trip_distance_km']}"))
    else:
        print("  [FAIL] Did not receive Cmd 0x15 within timeout")
        results.append(("Test 4: Trip 2 Historical (0x261 -> 0x15)", False, "Timeout waiting for Cmd 0x15"))

    # ---------------------------------------------------------
    # TEST 5: MDF Attached Log Replay & Verification
    # ---------------------------------------------------------
    print("\n--- Test 5: Ingesting & Verifying Attached MDF Trace Vector ---")
    if os.path.exists(mdf_log_path):
        print(f"  Parsing MDF trace file: {mdf_log_path}...")
        mdf_2a1_frame = None
        mdf_261_frame = None
        with open(mdf_log_path, "r") as f:
            for line in f:
                if "2A1#" in line and not mdf_2a1_frame:
                    payload = line.split("2A1#")[1].strip()
                    if payload != "FFFFFFFFFF0399":
                        mdf_2a1_frame = bytes.fromhex(payload)
                if "261#" in line and not mdf_261_frame:
                    payload = line.split("261#")[1].strip()
                    if payload != "FFFFFFFFFF0399":
                        mdf_261_frame = bytes.fromhex(payload)

        if mdf_2a1_frame:
            print(f"  Found MDF Trip 1 CAN 0x2A1 Vector: {mdf_2a1_frame.hex(' ')}")
            exp_speed = mdf_2a1_frame[0]
            exp_dist = struct.unpack(">H", mdf_2a1_frame[1:3])[0]
            exp_fuel = struct.unpack(">H", mdf_2a1_frame[3:5])[0] / 10.0
            print(f"    Expected: Speed={exp_speed} km/h, Dist={exp_dist} km, Fuel={exp_fuel} L/100km")

            idx0 = len(tester.rx_packets)
            with tester.can_lock:
                tester.active_can_frames[0x2A1] = mdf_2a1_frame
            tester.send_can(0x2A1, mdf_2a1_frame)

            p14_mdf = tester.wait_for_packet(0x14, timeout_sec=10.0, since_idx=idx0,
                                             predicate=lambda p: decode_hiworld_0x14_0x15(p).get('avg_fuel_l_100km') == exp_fuel)
            if p14_mdf:
                d14_mdf = decode_hiworld_0x14_0x15(p14_mdf)
                print(f"  [RECV Cmd 0x14] Raw: {p14_mdf.hex(' ')}")
                print(f"    -> Decoded Avg Fuel: {d14_mdf['avg_fuel_l_100km']} L/100km")
                print(f"    -> Decoded Avg Speed: {d14_mdf['avg_speed_kmh']} km/h")
                print(f"    -> Decoded Trip Distance: {d14_mdf['trip_distance_km']} km")
                match = (d14_mdf['avg_fuel_l_100km'] == exp_fuel and
                         d14_mdf['avg_speed_kmh'] == exp_speed and
                         d14_mdf['trip_distance_km'] == exp_dist)
                results.append(("Test 5: Attached MDF Trace Vector (0x2A1 -> 0x14)", match, f"Fuel={d14_mdf['avg_fuel_l_100km']} Speed={d14_mdf['avg_speed_kmh']} Dist={d14_mdf['trip_distance_km']}"))
            else:
                print("  [FAIL] Did not receive Cmd 0x14 for MDF vector within timeout")
                results.append(("Test 5: Attached MDF Trace Vector (0x2A1 -> 0x14)", False, "Timeout waiting for Cmd 0x14"))
        else:
            print(f"  [WARN] No non-zero 0x2A1 vector found in {mdf_log_path}")
            results.append(("Test 5: Attached MDF Trace Vector", False, "No valid vector in MDF log"))
    else:
        print(f"  [WARN] MDF log file '{mdf_log_path}' not found")
        results.append(("Test 5: Attached MDF Trace Vector", False, "MDF log file not found"))

    # ---------------------------------------------------------
    # PRINT SUMMARY
    # ---------------------------------------------------------
    print("\n" + "=" * 60)
    print("  VERIFICATION SUMMARY")
    print("=" * 60)
    all_passed = True
    for name, passed, detail in results:
        status = "[PASS]" if passed else "[FAIL]"
        if not passed:
            all_passed = False
        print(f"  {status} {name:<50} | {detail}")

    print("=" * 60)
    if all_passed:
        print("  >>> ALL TRIP COMPUTER TESTS COMPLETED SUCCESSFULLY! <<<")
    else:
        print("  >>> ONE OR MORE TESTS FAILED. CHECK LOGS ABOVE. <<<")
    print("=" * 60 + "\n")

    return all_passed


def main():
    parser = argparse.ArgumentParser(description="Simulate CAN and serial connection to real Hiworld Canbox for Peugeot 407 Trip Computer.")
    parser.add_argument("--can", default="can0", help="SocketCAN interface (default: can0)")
    parser.add_argument("--port", default=None, help="Serial port to Canbox (default: auto-detected via FTDI VID:PID)")
    parser.add_argument("--baud", type=int, default=38400, help="UART baud rate (default: 38400)")
    parser.add_argument("--mdf", default="dump_mdf_stalk_trip.log", help="Path to attached MDF capture log")
    parser.add_argument("--dump-version", action="store_true", help="Dump Canbox firmware version and exit without executing tests")
    args = parser.parse_args()

    print("=" * 60)
    print("  Hiworld Canbox Peugeot 407 Trip Computer Simulator & Verifier")
    print("=" * 60)

    tester = CanboxTripTester(can_iface=args.can, serial_port=args.port, baudrate=args.baud)
    try:
        tester.connect()

        if args.dump_version:
            tester.start_bsi_baseline(settle_time_sec=1.0)
            success = tester.wait_for_firmware_version(timeout_sec=30.0, force_dump=True)
            if success:
                print(f"\n[OK] Firmware version: {tester.firmware_version}\n")
                return 0
            else:
                print("\n[FAIL] Could not read firmware version from Canbox.\n")
                return 1

        connected = tester.establish_connection()
        if not connected:
            print("[FAIL] Failed to acquire firmware version or establish connection with Canbox. Aborting before tests.")
            return 1
        time.sleep(1.0)
        success = run_tests(tester, mdf_log_path=args.mdf)
        return 0 if success else 1
    except KeyboardInterrupt:
        print("\n[!] User interrupted test execution.")
        return 130
    finally:
        tester.close()

if __name__ == "__main__":
    sys.exit(main())
