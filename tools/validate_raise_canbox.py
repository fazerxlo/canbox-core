#!/usr/bin/env python3
"""
tools/validate_raise_canbox.py
Simulates BSI traffic on can0 and probes the attached CAN box:
1. Starts periodic BSI frames on can0.
2. Monitors can0 for any external CAN frames (e.g. Virtual MDF emulation: 0x167, 0x15B, 0x0DF, 0x3F6, 0x525).
3. Monitors /dev/ttyRealCanbox at candidate baud rates (38400, 19200, etc.) for incoming traffic.
4. Sends periodic Head Unit handshake / keepalive queries.
"""

import sys
import os
import time
import socket
import struct
import threading
from typing import Dict, List, Set, Tuple

try:
    import serial
except ImportError:
    print("Error: pyserial required")
    sys.exit(1)

CAN_FRAME_FMT = "=IB3x8s"

# Essential PSA CAN2004 BSI periodic frames
BSI_PERIODIC_FRAMES: List[Tuple[int, bytes, float]] = [
    # (CAN_ID, Data, Cadence in seconds)
    (0x036, bytes([0x0E, 0x00, 0x00, 0x0F, 0x01, 0x00, 0x00, 0xA0]), 0.100),  # Network awake
    (0x0F6, bytes([0x88, 0x18, 0x00, 0x00, 0x00, 0x50, 0x50, 0x00]), 0.100),  # Ignition ON, Temp 24C
    (0x0B6, bytes([0x19, 0x00, 0x13, 0x88, 0x00, 0x00, 0x00, 0xD0]), 0.050),  # Engine RPM 800, Speed 50 km/h
    (0x128, bytes([0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]), 0.100),  # Contact ON
    (0x220, bytes([0x00, 0x00]), 0.200),                                      # Doors all closed
    (0x260, bytes([0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]), 0.500),  # BSI Profile 1
    (0x361, bytes([0x01, 0x01, 0x91, 0x40, 0x30, 0x10]), 0.500),              # BSI Options
    (0x1E3, bytes([0x1C, 0x40, 0x0B, 0x0B, 0x00, 0x00, 0x07]), 0.250),        # HVAC state
    (0x221, bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]), 0.200),  # Trip instantaneous
    (0x2A1, bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]), 0.500),  # Trip 1
    (0x261, bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]), 0.500),  # Trip 2
]

# Raise protocol framing helpers
RAISE_SYNC = 0x2E
def raise_checksum(cmd: int, length: int, payload: bytes) -> int:
    return ((cmd + length + sum(payload)) & 0xFF) ^ 0xFF

def raise_packet(cmd: int, payload: bytes = b"") -> bytes:
    length = len(payload)
    cs = raise_checksum(cmd, length, payload)
    return bytes([RAISE_SYNC, cmd, length]) + payload + bytes([cs])

# Hiworld protocol framing helpers
def hiworld_packet(cmd: int, payload: bytes = b"") -> bytes:
    length = len(payload)
    cs = ((length + cmd + sum(payload)) - 1) & 0xFF
    return bytes([0x5A, 0xA5, length, cmd]) + payload + bytes([cs])


class CanboxValidator:
    def __init__(self, can_iface: str = "can0", serial_port: str = "/dev/ttyRealCanbox"):
        self.can_iface = can_iface
        self.serial_port = serial_port
        self.stop_evt = threading.Event()
        
        # Sockets
        self.tx_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.tx_sock.bind((self.can_iface,))
        
        self.rx_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        # Disable receiving own messages on rx_sock
        self.rx_sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
        self.rx_sock.bind((self.can_iface,))
        self.rx_sock.settimeout(0.1)

        self.external_can_frames: Dict[int, List[bytes]] = {}
        self.external_can_counts: Dict[int, int] = {}
        self.lock = threading.Lock()

    def start_can_broadcast(self):
        def can_tx_worker():
            last_tx = {cid: 0.0 for cid, _, _ in BSI_PERIODIC_FRAMES}
            while not self.stop_evt.is_set():
                now = time.time()
                for cid, data, cadence in BSI_PERIODIC_FRAMES:
                    if now - last_tx[cid] >= cadence:
                        last_tx[cid] = now
                        frame = struct.pack(CAN_FRAME_FMT, cid, len(data), data.ljust(8, b"\x00"))
                        try:
                            self.tx_sock.send(frame)
                        except Exception as e:
                            pass
                time.sleep(0.01)

        t_tx = threading.Thread(target=can_tx_worker, daemon=True, name="CAN-TX")
        t_tx.start()

    def start_can_listener(self):
        def can_rx_worker():
            while not self.stop_evt.is_set():
                try:
                    data = self.rx_sock.recv(16)
                    can_id, dlc, raw = struct.unpack("=IB3x8s", data)
                    can_id &= 0x1FFFFFFF  # Clear flags
                    payload = raw[:dlc]
                    with self.lock:
                        if can_id not in self.external_can_counts:
                            self.external_can_counts[can_id] = 0
                            self.external_can_frames[can_id] = []
                            print(f"[CAN-RX] *** NEW EXTERNAL CAN ID DETECTED: 0x{can_id:03X} (DLC {dlc}): {payload.hex(' ')} ***")
                        self.external_can_counts[can_id] += 1
                        if len(self.external_can_frames[can_id]) < 5:
                            self.external_can_frames[can_id].append(payload)
                except socket.timeout:
                    continue
                except Exception as e:
                    break

        t_rx = threading.Thread(target=can_rx_worker, daemon=True, name="CAN-RX")
        t_rx.start()

    def probe_serial(self, baudrate: int, duration: float = 12.0) -> Tuple[bytes, List[Dict]]:
        print(f"\n[UART] Probing {self.serial_port} @ {baudrate} baud for {duration} seconds...")
        rx_bytes = bytearray()
        packets_found: List[Dict] = []
        try:
            ser = serial.Serial(self.serial_port, baudrate, timeout=0.1)
        except Exception as e:
            print(f"[UART] Failed to open {self.serial_port}: {e}")
            return b"", []

        t_start = time.time()
        last_hs = 0.0

        while (time.time() - t_start) < duration:
            now = time.time()
            # Send periodic Head Unit handshake queries every 2 seconds
            if now - last_hs >= 2.0:
                last_hs = now
                # Raise Car Model Select (PSA 407 = 0x01, 0x02)
                ser.write(raise_packet(0xCA, bytes([0x01, 0x02])))
                # Raise Firmware Version Request
                ser.write(raise_packet(0x7F, b""))
                # Also send Hiworld Handshake just in case
                ser.write(hiworld_packet(0x24, bytes([0x01, 0x02])))
                ser.flush()

            chunk = ser.read(128)
            if chunk:
                rx_bytes.extend(chunk)
                t_rel = now - t_start
                print(f"[UART +{t_rel:.1f}s] Received {len(chunk)} bytes (Total: {len(rx_bytes)}). Hex: {chunk.hex(' ')}")

                # Check for Raise packets: 2E <cmd> <len> ... <cs>
                i = 0
                while i <= len(rx_bytes) - 4:
                    if rx_bytes[i] == RAISE_SYNC:
                        cmd = rx_bytes[i + 1]
                        length = rx_bytes[i + 2]
                        if length <= 64 and (i + length + 4 <= len(rx_bytes)):
                            pkt = bytes(rx_bytes[i : i + length + 4])
                            payload = pkt[3:-1]
                            cs = pkt[-1]
                            if cs == raise_checksum(cmd, length, payload):
                                packets_found.append({
                                    "proto": "Raise",
                                    "cmd": hex(cmd),
                                    "payload": payload.hex(' '),
                                    "raw": pkt.hex(' ')
                                })
                                print(f"  --> [DECODED RAISE] Cmd 0x{cmd:02X} (len {length}): {payload.hex(' ')}")
                                i += len(pkt)
                                continue
                    # Check for Hiworld packets: 5A A5 <len> <cmd> ... <cs>
                    if rx_bytes[i] == 0x5A and i + 1 < len(rx_bytes) and rx_bytes[i + 1] == 0xA5:
                        length = rx_bytes[i + 2]
                        if length <= 64 and (i + length + 5 <= len(rx_bytes)):
                            pkt = bytes(rx_bytes[i : i + length + 5])
                            cmd = pkt[3]
                            payload = pkt[4:-1]
                            cs = pkt[-1]
                            calc_cs = ((length + cmd + sum(payload)) - 1) & 0xFF
                            if cs == calc_cs:
                                packets_found.append({
                                    "proto": "Hiworld",
                                    "cmd": hex(cmd),
                                    "payload": payload.hex(' '),
                                    "raw": pkt.hex(' ')
                                })
                                print(f"  --> [DECODED HIWORLD] Cmd 0x{cmd:02X} (len {length}): {payload.hex(' ')}")
                                i += len(pkt)
                                continue
                    i += 1

            time.sleep(0.02)

        ser.close()
        return bytes(rx_bytes), packets_found


def main():
    validator = CanboxValidator()
    print("=================================================================")
    print("  Canbox Hardware & Protocol Validation with Active BSI Broadcast")
    print("=================================================================")
    print("[*] Starting BSI CAN bus simulation on can0...")
    validator.start_can_broadcast()
    validator.start_can_listener()
    
    # Give the canbox up to 12s on 38400 baud first
    raw, pkts = validator.probe_serial(38400, duration=12.0)
    
    # If nothing or corrupted, probe 19200
    if len(pkts) == 0:
        print("[!] No decoded packets at 38400 baud. Probing 19200 baud...")
        raw_192, pkts_192 = validator.probe_serial(19200, duration=8.0)
        if len(pkts_192) > 0:
            raw, pkts = raw_192, pkts_192
        else:
            # Also test 115200 and 9600
            print("[!] Probing 115200 baud...")
            raw_115, pkts_115 = validator.probe_serial(115200, duration=5.0)
            if len(pkts_115) > 0:
                raw, pkts = raw_115, pkts_115
            else:
                print("[!] Probing 9600 baud...")
                raw_96, pkts_96 = validator.probe_serial(9600, duration=5.0)
                if len(pkts_96) > 0:
                    raw, pkts = raw_96, pkts_96

    validator.stop_evt.set()
    time.sleep(0.2)

    print("\n=================================================================")
    print("  SUMMARY OF RESULTS")
    print("=================================================================")
    print(f"1. CAN BUS EXTERNAL TRANSMISSIONS DETECTED:")
    if validator.external_can_counts:
        for cid, count in validator.external_can_counts.items():
            samples = [f.hex(' ') for f in validator.external_can_frames[cid]]
            print(f"   • CAN ID 0x{cid:03X}: {count} frames received. Samples: {samples}")
            if cid in (0x167, 0x15B, 0x0DF, 0x3F6, 0x525):
                print(f"     ==> [CONFIRMED] CAN ID 0x{cid:03X} is an EMF / MDF DISPLAY FRAME! The canbox IS EMULATING MDF!")
    else:
        print("   • No external CAN transmissions observed on can0.")

    print(f"\n2. SERIAL UART TRAFFIC:")
    print(f"   • Total bytes captured: {len(raw)}")
    print(f"   • Decoded valid packets: {len(pkts)}")
    for p in pkts[:10]:
        print(f"     - {p['proto']} Cmd {p['cmd']}: {p['payload']} (Raw: {p['raw']})")
    print("=================================================================")

if __name__ == "__main__":
    main()

