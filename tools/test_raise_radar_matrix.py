#!/usr/bin/env python3
"""
tools/test_raise_radar_matrix.py
=============================================================================
Automated Radar Matrix & Architectural Probe for Commercial Raise CAN Box
(Peugeot 407 / PSA CAN2004 Comfort @ 125 kbps).

Probes candidate triggers across multi-dimensional test axes:
1. Multi-CAN Radar IDs:
   - 0x0E1: Native PSA CAN2004 AAS (7 bytes)
   - 0x260: AEE2010 Rear AAS (8 bytes)
   - 0x270: AEE2010 Front AAS (8 bytes)
   - 0x260 + 0x270: Dual AEE2010 AAS
   - 0x1E1: Alternate PSA AAS
   - 0x0E0 / 0x0E2: Adjacent AAS frames
2. Acoustic & Display Flags:
   - 0x0E1 Byte 5 Bit 1: display_active flag (MDF screen request)
   - 0x0E1 Byte 6: buzzer frequencies (mute, intermittent, continuous 0x07/0x26)
   - 0x120: BSI acoustic chime & alert sound frame
   - 0x1A1: EMF/MDF pop-up alert window opened ("Aide au stationnement")
   - 0x167: EMF/MDF display state window open (00 B4 00 / 00 B8 00)
3. Car Configuration & Personalization:
   - 0x361: AAS present in vehicle characteristics (Byte 4 Bit 5 = 0x20)
   - 0x260: AAS enabled by user in Profile 1 (Byte 5 Bit 6 = 0x40) vs disabled
   - 0x15B: User personalization toggle downlink
4. Reverse Gear Conditions:
   - Dual-frame reverse: 0x0F6 (B7=0xA0) + 0x036 (B1=0x80)
   - Forward gear / neutral (for front parking radar)

Monitors /dev/ttyRealCanbox @ 38400 baud for Raise Cmd 0x30 / Cmd 0x32 / any UART packet.
=============================================================================
"""

import sys
import os
import time
import datetime
import socket
import struct
import argparse
import threading
from typing import Dict, List, Tuple, Optional, Any

# Ensure workspace can import canbox_usb_ports if available
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from canbox_usb_ports import find_canbox_uart_device
except ImportError:
    def find_canbox_uart_device():
        return "/dev/ttyRealCanbox"

try:
    import serial
except ImportError:
    print("[ERROR] pyserial is required. Install with: pip install pyserial")
    sys.exit(1)

CAN_FRAME_FMT = "=IB3x8s"
RAISE_SYNC = 0x2E

def raise_checksum(cmd: int, length: int, payload: bytes) -> int:
    return ((cmd + length + sum(payload)) & 0xFF) ^ 0xFF

def raise_packet(cmd: int, payload: bytes = b"") -> bytes:
    length = len(payload)
    cs = raise_checksum(cmd, length, payload)
    return bytes([RAISE_SYNC, cmd, length]) + payload + bytes([cs])

def decode_raise_packet(pkt: bytes) -> Dict[str, Any]:
    if len(pkt) < 4 or pkt[0] != RAISE_SYNC:
        return {"valid": False}
    cmd = pkt[1]
    length = pkt[2]
    if len(pkt) < 4 + length:
        return {"valid": False}
    payload = pkt[3:3 + length]
    expected_cs = pkt[3 + length]
    actual_cs = raise_checksum(cmd, length, payload)
    if expected_cs != actual_cs:
        return {"valid": False, "checksum_err": True}
    return {
        "valid": True,
        "cmd": cmd,
        "length": length,
        "payload": payload,
        "hex": pkt[:4 + length].hex().upper()
    }


class RaiseRadarTester:
    def __init__(self, can_iface: str = "can0", serial_port: str = "/dev/ttyRealCanbox", baud: int = 38400, log_file: Optional[str] = None):
        self.can_iface = can_iface
        self.serial_port = serial_port
        self.baud = baud
        self.stop_evt = threading.Event()
        self.log_file = log_file
        self.log_fp = None
        self.log_lock = threading.Lock()

        if self.log_file:
            self.log_fp = open(self.log_file, "w", encoding="utf-8")
            self._write_header()

        # CAN Sockets
        self.can_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.can_sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_LOOPBACK, 0)
        except (AttributeError, OSError):
            pass
        self.can_sock.bind((self.can_iface,))

        # Separate CAN RX Socket (to capture bus responses/other ECU frames if present)
        try:
            self.can_rx_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.can_rx_sock.bind((self.can_iface,))
            self.can_rx_sock.settimeout(0.05)
        except Exception as e:
            self.can_rx_sock = None
            self.log("WARN", f"Failed to open CAN RX socket: {e}")

        # Serial Port
        self.ser = serial.Serial(self.serial_port, self.baud, timeout=0.05)

        # Baseline cyclic frames: {can_id: (data_bytes, period_sec, last_tx)}
        self.cyclic_frames: Dict[int, List[Any]] = {}
        self.cyclic_lock = threading.Lock()

        # Received UART packets
        self.received_packets: List[Dict[str, Any]] = []
        self.rx_lock = threading.Lock()

    def _write_header(self):
        if not self.log_fp:
            return
        dt_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.log_fp.write("# ==============================================================================\n")
        self.log_fp.write("# RAISE OEM CAN BOX RADAR MATRIX TEST LOG\n")
        self.log_fp.write(f"# Started: {dt_str}\n")
        self.log_fp.write(f"# CAN Interface: {self.can_iface} | Serial: {self.serial_port} @ {self.baud} baud\n")
        self.log_fp.write("# Compatible with ./tools/triage_canbox_log.py\n")
        self.log_fp.write("# ==============================================================================\n")
        self.log_fp.flush()

    def log(self, tag: str, message: str):
        if not self.log_fp:
            return
        now = time.time()
        with self.log_lock:
            self.log_fp.write(f"{now:.3f} [{tag}] {message}\n")
            self.log_fp.flush()

    def send_can(self, can_id: int, data: bytes):
        dlc = len(data)
        data_padded = data.ljust(8, b"\x00")
        frame = struct.pack(CAN_FRAME_FMT, can_id, dlc, data_padded)
        try:
            self.can_sock.send(frame)
            self.log("CAN", f"ID:{can_id:03X} DLC:{dlc} DATA:{data.hex().upper()}")
        except OSError as e:
            self.log("CAN_ERR", f"TX Failed ID:{can_id:03X}: {e}")

    def patch_cyclic(self, can_id: int, data: bytes, period_ms: int = 100):
        with self.cyclic_lock:
            self.cyclic_frames[can_id] = [data, period_ms / 1000.0, 0.0]

    def remove_cyclic(self, can_id: int):
        with self.cyclic_lock:
            if can_id in self.cyclic_frames:
                del self.cyclic_frames[can_id]

    def _can_thread_fn(self):
        while not self.stop_evt.is_set():
            now = time.time()
            to_send = []
            with self.cyclic_lock:
                for can_id, entry in list(self.cyclic_frames.items()):
                    data, period, last_tx = entry
                    if now - last_tx >= period:
                        entry[2] = now
                        to_send.append((can_id, data))
            for can_id, data in to_send:
                self.send_can(can_id, data)
            time.sleep(0.010)

    def _can_rx_thread_fn(self):
        if not self.can_rx_sock:
            return
        while not self.stop_evt.is_set():
            try:
                cf, _ = self.can_rx_sock.recvfrom(16)
                if len(cf) < 16:
                    continue
                can_id, can_dlc, data_padded = struct.unpack(CAN_FRAME_FMT, cf)
                can_id &= socket.CAN_EFF_MASK
                data = data_padded[:can_dlc]
                self.log("CAN_RX", f"ID:{can_id:03X} DLC:{can_dlc} DATA:{data.hex().upper()}")
            except socket.timeout:
                continue
            except OSError:
                break
            except Exception:
                pass

    def _uart_thread_fn(self):
        buf = bytearray()
        last_keepalive = 0.0
        while not self.stop_evt.is_set():
            # Send periodic HU keepalive / version query (Cmd 0x7F / 0xCA) every 1.5s
            now = time.time()
            if now - last_keepalive >= 1.5:
                last_keepalive = now
                try:
                    pkt = raise_packet(0x7F, b"")
                    self.ser.write(pkt)  # Version query
                    self.log("APP_UART_TX", pkt.hex().upper())
                except Exception as e:
                    self.log("UART_ERR", f"TX keepalive failed: {e}")

            try:
                chunk = self.ser.read(64)
                if chunk:
                    buf.extend(chunk)
            except Exception:
                time.sleep(0.02)
                continue

            # Parse Raise framing: 0x2E <cmd> <len> <payload> <cs>
            while len(buf) >= 4:
                if buf[0] != RAISE_SYNC:
                    buf.pop(0)
                    continue
                length = buf[2]
                total_len = 4 + length
                if len(buf) < total_len:
                    break
                pkt_bytes = bytes(buf[:total_len])
                del buf[:total_len]

                res = decode_raise_packet(pkt_bytes)
                if res.get("valid"):
                    with self.rx_lock:
                        self.received_packets.append(res)
                    self.log("APP_UART_RX", pkt_bytes.hex().upper())
                    cmd_val = res["cmd"]
                    cmd_desc = ""
                    if cmd_val == 0x32:
                        cmd_desc = " [RADAR_REAR]"
                    elif cmd_val == 0x30:
                        cmd_desc = " [RADAR_FRONT]"
                    elif cmd_val == 0x21:
                        cmd_desc = " [WHEEL_BUTTONS]"
                    elif cmd_val == 0x7F:
                        cmd_desc = " [VERSION_INFO]"
                    self.log("UART_RX_INFO", f"Cmd:0x{cmd_val:02X} Len:{res['length']} Payload:{res['payload'].hex().upper()}{cmd_desc}")
                elif res.get("checksum_err"):
                    self.log("UART_ERR", f"Checksum error in packet: {pkt_bytes.hex().upper()}")

            time.sleep(0.005)

    def start(self):
        # Initial standard baseline
        self.patch_cyclic(0x036, bytes.fromhex("0E00000F010000A0"), 100) # Network awake
        self.patch_cyclic(0x0F6, bytes.fromhex("88FFFFFFFFFFFF20"), 100) # Ignition ON, Reverse OFF
        self.patch_cyclic(0x0B6, bytes.fromhex("1C200000000000D0"), 50)  # Speed / RPM
        self.patch_cyclic(0x128, bytes.fromhex("0301000000000000"), 100) # Contact / Lights
        self.patch_cyclic(0x220, bytes.fromhex("00000000"), 200)         # Doors closed
        self.patch_cyclic(0x361, bytes.fromhex("010191403010"), 500)     # BSI Config: AAS present
        self.patch_cyclic(0x260, bytes.fromhex("0100104008D00010"), 250) # BSI Profile 1: AAS enabled

        self.t_can = threading.Thread(target=self._can_thread_fn, daemon=True)
        if self.can_rx_sock:
            self.t_can_rx = threading.Thread(target=self._can_rx_thread_fn, daemon=True)
            self.t_can_rx.start()
        self.t_uart = threading.Thread(target=self._uart_thread_fn, daemon=True)
        self.t_can.start()
        self.t_uart.start()
        time.sleep(0.5)

    def stop(self):
        self.stop_evt.set()
        time.sleep(0.2)
        try:
            self.ser.close()
            self.can_sock.close()
            if self.can_rx_sock:
                self.can_rx_sock.close()
        except Exception:
            pass
        if self.log_fp:
            try:
                self.log("INFO", "=== Raise OEM CAN Box Radar Matrix Test Ended ===")
                self.log_fp.close()
            except Exception:
                pass
            self.log_fp = None

    def run_matrix_vector(self, name: str, stimulus_fn, duration_sec: float = 1.2) -> Dict[str, Any]:
        """Applies stimulus, monitors UART for radar frames, and returns results."""
        self.log("TEST_VECTOR", f"START >>> {name}")
        with self.rx_lock:
            self.received_packets.clear()

        stimulus_fn(self)
        t_start = time.time()
        time.sleep(duration_sec)

        with self.rx_lock:
            pkts = list(self.received_packets)

        # Categorize responses
        radar_rear = [p for p in pkts if p["cmd"] == 0x32]
        radar_front = [p for p in pkts if p["cmd"] == 0x30]
        radar_any = [p for p in pkts if p["cmd"] in (0x30, 0x32)]
        other_cmds = sorted(list(set(p["cmd"] for p in pkts if p["cmd"] not in (0x30, 0x32, 0x21, 0x33, 0x34, 0x35, 0x36, 0x38, 0x7F))))

        hit = len(radar_any) > 0 or len(other_cmds) > 0
        status_str = "TRIGGER!" if hit else "SILENT"
        self.log("TEST_VECTOR", f"END   <<< {name} | Status: {status_str} | Radar pkts: {len(radar_any)} {[p['hex'] for p in radar_any]} | Other cmds: {[f'0x{c:02X}' for c in other_cmds]}")

        return {
            "name": name,
            "duration": round(time.time() - t_start, 2),
            "radar_rear_count": len(radar_rear),
            "radar_front_count": len(radar_front),
            "radar_packets": [p["hex"] for p in radar_any],
            "other_cmds": [f"0x{c:02X}" for c in other_cmds],
            "total_pkts": len(pkts),
            "hit": hit
        }


# =============================================================================
# Test Matrix Vector Definitions
# =============================================================================

def set_reverse(tester: RaiseRadarTester, active: bool):
    if active:
        tester.patch_cyclic(0x0F6, bytes.fromhex("88FFFFFFFFFFFFA0"), 100)
        tester.patch_cyclic(0x036, bytes.fromhex("0E80000F010000A0"), 100)
    else:
        tester.patch_cyclic(0x0F6, bytes.fromhex("88FFFFFFFFFFFF20"), 100)
        tester.patch_cyclic(0x036, bytes.fromhex("0E00000F010000A0"), 100)

def define_test_vectors() -> List[Tuple[str, str, Any]]:
    """Returns list of (ID, Description, stimulus_function)."""
    vectors = []

    # -------------------------------------------------------------------------
    # AXIS 1: NATIVE CAN2004 0x0E1 VARIATIONS (Reverse Engaged)
    # -------------------------------------------------------------------------
    def v_01(t: RaiseRadarTester):
        set_reverse(t, True)
        # 0x0E1: Vector 1 (Rear active, display active, obstacles on rear & front)
        t.patch_cyclic(0x0E1, bytes.fromhex("24403F04202600"), 100)
    vectors.append(("E1-01", "0x0E1 Vector 1 (Display bit ON, Rear Active, Reverse ON)", v_01))

    def v_02(t: RaiseRadarTester):
        set_reverse(t, True)
        # 0x0E1: Display bit OFF (B5 bit 1 = 0), only rear active (B1 bit 6 = 1)
        t.patch_cyclic(0x0E1, bytes.fromhex("24403F04202400"), 100)
    vectors.append(("E1-02", "0x0E1 Display bit OFF (Rear Active only)", v_02))

    def v_03(t: RaiseRadarTester):
        set_reverse(t, True)
        # 0x0E1: Critical Zone 0 (<30cm) obstacle with continuous tone flag (B6 = 0x07)
        t.patch_cyclic(0x0E1, bytes.fromhex("24403F001CFE07"), 100)
    vectors.append(("E1-03", "0x0E1 Critical Proximity (Zone 0) + Buzzer Tone B6=0x07", v_03))

    def v_04(t: RaiseRadarTester):
        set_reverse(t, True)
        # 0x0E1: Alternate buzzer tone patterns (B6 = 0x26, 0x01, 0x04)
        t.patch_cyclic(0x0E1, bytes.fromhex("24403F04202626"), 100)
    vectors.append(("E1-04", "0x0E1 Acoustic Beeping Flag (Byte 6 = 0x26)", v_04))

    # -------------------------------------------------------------------------
    # AXIS 2: EXTERNAL DISPLAY & POP-UP ALERTS (MFD Window Open)
    # -------------------------------------------------------------------------
    def v_05(t: RaiseRadarTester):
        set_reverse(t, True)
        t.patch_cyclic(0x0E1, bytes.fromhex("24403F04202600"), 100)
        # 0x1A1: Popup warning window open (Bit 7 / Category)
        t.patch_cyclic(0x1A1, bytes.fromhex("8000000000000000"), 200)
    vectors.append(("POP-01", "0x0E1 Obstacle + CAN 0x1A1 Popup Window Open (Byte 0 = 0x80)", v_05))

    def v_06(t: RaiseRadarTester):
        set_reverse(t, True)
        t.patch_cyclic(0x0E1, bytes.fromhex("24403F04202600"), 100)
        # 0x167: EMF Window State Active (00 B4 00 / 00 B8 00)
        t.patch_cyclic(0x167, bytes.fromhex("00B400"), 100)
    vectors.append(("POP-02", "0x0E1 Obstacle + CAN 0x167 EMF Window Active (00 B4 00)", v_06))

    def v_07(t: RaiseRadarTester):
        set_reverse(t, True)
        t.patch_cyclic(0x0E1, bytes.fromhex("24403F04202600"), 100)
        # 0x120: BSI Alert Chime / Acoustic Sound Active
        t.patch_cyclic(0x120, bytes.fromhex("FC000000000F0200"), 500)
    vectors.append(("POP-03", "0x0E1 Obstacle + CAN 0x120 Acoustic Alert Chime Active", v_07))

    # -------------------------------------------------------------------------
    # AXIS 3: AEE2010 RADAR FRAMES (0x260 / 0x270)
    # -------------------------------------------------------------------------
    def v_08(t: RaiseRadarTester):
        set_reverse(t, True)
        t.remove_cyclic(0x0E1)
        # 0x260: AEE2010 Rear AAS (Zone 1 RL=1, RC=1, RR=1, Active bit 7 = 0x80)
        t.patch_cyclic(0x260, bytes.fromhex("0101018000000000"), 100)
    vectors.append(("AEE-01", "0x260 AEE2010 Rear Radar (Zone 1, Active bit 0x80)", v_08))

    def v_09(t: RaiseRadarTester):
        set_reverse(t, True)
        t.remove_cyclic(0x0E1)
        # 0x260: AEE2010 Rear AAS Critical (Zone 0 = 00 00 00 80)
        t.patch_cyclic(0x260, bytes.fromhex("0000008000000000"), 100)
    vectors.append(("AEE-02", "0x260 AEE2010 Rear Radar Critical (Zone 0, Active bit 0x80)", v_09))

    def v_10(t: RaiseRadarTester):
        set_reverse(t, False) # Front radar operates while driving forward
        t.remove_cyclic(0x0E1)
        # 0x270: AEE2010 Front AAS (Zone 2 FL=2, FC=2, FR=2, Active bit 0x80)
        t.patch_cyclic(0x270, bytes.fromhex("0202028000000000"), 100)
    vectors.append(("AEE-03", "0x270 AEE2010 Front Radar (Zone 2, Forward Drive)", v_10))

    def v_11(t: RaiseRadarTester):
        set_reverse(t, True)
        t.remove_cyclic(0x0E1)
        # Both 0x260 Rear and 0x270 Front simultaneously
        t.patch_cyclic(0x260, bytes.fromhex("0101018000000000"), 100)
        t.patch_cyclic(0x270, bytes.fromhex("0202028000000000"), 100)
    vectors.append(("AEE-04", "Dual 0x260 (Rear) + 0x270 (Front) AEE2010 Frames", v_11))

    # -------------------------------------------------------------------------
    # AXIS 4: CAR CONFIGURATION & USER PERSONALIZATION
    # -------------------------------------------------------------------------
    def v_12(t: RaiseRadarTester):
        set_reverse(t, True)
        t.patch_cyclic(0x0E1, bytes.fromhex("24403F04202600"), 100)
        # BSI Config: AAS equipment enabled (0x361 B4 = 0x30)
        t.patch_cyclic(0x361, bytes.fromhex("010191403010"), 500)
        # BSI Profile 1: User toggle ON (0x260 B5 = 0xD0, bit 6 = 0x40)
        t.patch_cyclic(0x260, bytes.fromhex("0100104008D00010"), 250)
        # Downlink request active on 0x15B
        t.patch_cyclic(0x15B, bytes.fromhex("0100104008400000"), 250)
    vectors.append(("CFG-01", "0x0E1 + BSI AAS Enabled (0x361=30, 0x260=D0, 0x15B=40)", v_12))

    def v_13(t: RaiseRadarTester):
        set_reverse(t, True)
        # User toggled AAS to MUTED/DISABLED in Profile 1 (0x260 B5 bit 6 = 0 -> 0x90)
        t.patch_cyclic(0x0E1, bytes.fromhex("24403F04202600"), 100)
        t.patch_cyclic(0x260, bytes.fromhex("0100104008900010"), 250)
    vectors.append(("CFG-02", "0x0E1 + BSI AAS Muted by User (0x260 B5 bit 6 = 0)", v_13))

    # -------------------------------------------------------------------------
    # AXIS 5: ALTERNATE & ADJACENT RADAR CAN IDS
    # -------------------------------------------------------------------------
    def v_14(t: RaiseRadarTester):
        set_reverse(t, True)
        t.remove_cyclic(0x0E1)
        # 0x1E1: Alternate PSA AAS
        t.patch_cyclic(0x1E1, bytes.fromhex("24403F04202600"), 100)
    vectors.append(("ALT-01", "CAN 0x1E1 (Alternate PSA AAS ID Vector)", v_14))

    def v_15(t: RaiseRadarTester):
        set_reverse(t, True)
        t.remove_cyclic(0x0E1)
        # 0x0E0 / 0x0E2: Adjacent CAN IDs
        t.patch_cyclic(0x0E0, bytes.fromhex("24403F04202600"), 100)
        t.patch_cyclic(0x0E2, bytes.fromhex("24403F04202600"), 100)
    vectors.append(("ALT-02", "CAN 0x0E0 / 0x0E2 (Adjacent IDs Stimulus)", v_15))

    return vectors


# =============================================================================
# Main Test Harness
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="Multi-Axis Radar Matrix Test for Raise OEM CAN Box")
    parser.add_argument("--can-iface", default="can0", help="CAN interface name (default: can0)")
    parser.add_argument("--serial-port", default=find_canbox_uart_device(), help="Serial device path")
    parser.add_argument("--baud", type=int, default=38400, help="Serial baud rate (default: 38400)")
    parser.add_argument("--duration", type=float, default=1.2, help="Stimulus hold duration per test (seconds)")
    parser.add_argument("--vector", default=None, help="Specific vector ID to test (e.g. E1-01, AEE-01)")
    parser.add_argument("--log-file", "-l", default=None, help="Path to log file (default: raise_radar_matrix_<timestamp>.log)")
    parser.add_argument("--no-log", action="store_true", help="Disable logging to file")
    args = parser.parse_args()

    log_file = None
    if not args.no_log:
        if args.log_file:
            log_file = args.log_file
        else:
            dt_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            log_file = f"raise_radar_matrix_{dt_str}.log"

    print("================================================================================")
    print("        RAISE OEM CAN BOX: MULTI-AXIS RADAR ARCHITECTURAL PROBE                 ")
    print(f"  CAN: {args.can_iface} | Serial: {args.serial_port} @ {args.baud} 8N1")
    if log_file:
        print(f"  Log File: {log_file}")
    print("================================================================================")

    tester = RaiseRadarTester(can_iface=args.can_iface, serial_port=args.serial_port, baud=args.baud, log_file=log_file)
    try:
        tester.start()
    except Exception as e:
        print(f"[ERROR] Failed to start tester: {e}")
        sys.exit(1)

    vectors = define_test_vectors()
    if args.vector:
        vectors = [v for v in vectors if v[0].lower() == args.vector.lower()]
        if not vectors:
            print(f"[ERROR] Unknown vector ID '{args.vector}'")
            tester.stop()
            sys.exit(1)

    results = []
    print(f"\nRunning {len(vectors)} test vectors (hold: {args.duration}s each)...")
    print("-" * 80)
    print(f"{'ID':<8} {'STATUS':<10} {'RADAR PKTS':<12} {'DESCRIPTION'}")
    print("-" * 80)

    try:
        for vid, desc, fn in vectors:
            res = tester.run_matrix_vector(f"{vid}: {desc}", fn, duration_sec=args.duration)
            hit = res["hit"]
            status_str = "[TRIGGER!]" if hit else "[SILENT]"
            pkt_count = res["radar_rear_count"] + res["radar_front_count"]
            print(f"{vid:<8} {status_str:<10} {pkt_count:<12} {desc}")
            if hit:
                print(f"         >>> UART Radar Packets: {res['radar_packets']}")
                if res["other_cmds"]:
                    print(f"         >>> Other UART Cmds:    {res['other_cmds']}")
            results.append((vid, desc, res))
    except KeyboardInterrupt:
        print("\n[!] Test interrupted by user (Ctrl+C)")
    finally:
        tester.stop()

    print("-" * 80)
    print("\n=== SUMMARY OF FINDINGS ===")
    hits = [r for r in results if r[2]["hit"]]
    if hits:
        print(f"[!] SUCCESS: Discovered {len(hits)} active triggers for Raise radar!")
        for vid, desc, r in hits:
            print(f"  - {vid} ({desc}): Radar={r['radar_packets']}, Other={r['other_cmds']}")
    else:
        print("[-] ZERO radar frames emitted across all test vectors.")
        print("    Confirms commercial Raise firmware (PG407S) lacks radar decoder for tested variants.")
    print("================================================================================")
    if log_file and os.path.exists(log_file):
        print(f"\n[+] Full CAN and UART message log saved to: {log_file}")
        print(f"    Review directly with text viewer or triage with: ./tools/triage_canbox_log.py {log_file}")
        print("================================================================================")

if __name__ == "__main__":
    main()

