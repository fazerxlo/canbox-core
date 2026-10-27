#!/usr/bin/env python3
"""
Interactive Peugeot 407 Dual-Zone Climate Workbench Delta Sniffer
================================================================
Maintains 40 ms keepalive and 900 RPM engine simulation on can0.
Maintains Hiworld 0x24 model handshake on UART /dev/ttyUSB0.
Logs every CAN delta on 0x1D0, 0x1E3, 0x12D and decodes Hiworld 0x31 UART packets in real-time.
"""

import sys
import os
import time
import socket
import struct
import threading
import datetime
import serial

try:
    from canbox_usb_ports import find_canbox_uart_device
except ImportError:
    def find_canbox_uart_device():
        return "/dev/ttyUSB0"

CAN_FRAME_FMT = "=IB3x8s"

def calc_hiworld_cs(length: int, cmd: int, payload: bytes) -> int:
    return ((length + cmd + sum(payload)) - 1) & 0xFF

def main():
    can_iface = "can0"
    uart_port = find_canbox_uart_device()
    baud_rate = 38400

    print("=" * 70)
    print("  PEUGEOT 407 WORKBENCH INTERACTIVE CLIMATE SNIFFER")
    print(f"  CAN: {can_iface} | UART: {uart_port} @ {baud_rate} baud")
    print("=" * 70)
    print("  [ACTIVE] Autonomous keepalive (0x036, 0x0F6, 0x128, 0x15B, 0x168)")
    print("  [ACTIVE] 900 RPM engine simulation & AC authorization (0x0B6, 0x12D)")
    print("  [ACTIVE] Hiworld 0x24 vehicle model selection (Peugeot 407)")
    print("\n  >> Operate the physical climate buttons and rotary dials now! <<\n")

    stop_event = threading.Event()

    can_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    can_sock.bind((can_iface,))

    ser = serial.Serial(uart_port, baud_rate, timeout=0.02)

    # Keepalive baseline frames
    keepalive_frames = {
        0x036: bytes([0x0E, 0x00, 0x00, 0x2A, 0x01, 0x00, 0x00, 0xA0]),
        0x0B6: bytes([0x1C, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0]),
        0x0F6: bytes([0x88, 0x00, 0x00, 0x00, 0x00, 0x50, 0x00, 0x20]),
        0x128: bytes([0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
        0x12D: bytes([0x80, 0x32, 0x32, 0x00, 0x64, 0x00, 0x00, 0x80]),
        0x15B: bytes([0x01, 0x03, 0xB1, 0x40, 0x00, 0xD0, 0x00, 0x10]),
        0x168: bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
    }

    def keepalive_worker():
        while not stop_event.is_set():
            for cid, data in keepalive_frames.items():
                f = struct.pack(CAN_FRAME_FMT, cid, len(data), data.ljust(8, b'\x00'))
                try:
                    can_sock.send(f)
                except Exception:
                    pass
            time.sleep(0.035)

    def handshake_worker():
        # Send 0x24 on startup and every 3.0s
        while not stop_event.is_set():
            pkt = bytes([0x5A, 0xA5, 0x02, 0x24, 0x22, 0x00, 0x47])
            ser.write(pkt)
            time.sleep(3.0)

    def uart_rx_worker():
        rx_buf = bytearray()
        while not stop_event.is_set():
            chunk = ser.read(64)
            if not chunk:
                continue
            rx_buf.extend(chunk)
            while len(rx_buf) >= 5:
                if rx_buf[0] != 0x5A or rx_buf[1] != 0xA5:
                    rx_buf.pop(0)
                    continue
                l = rx_buf[2]
                if l > 32:
                    rx_buf.pop(0)
                    continue
                total_len = l + 5
                if len(rx_buf) < total_len:
                    break
                cmd = rx_buf[3]
                p = bytes(rx_buf[4:4 + l])
                del rx_buf[:total_len]

                if cmd == 0x31 and len(p) >= 8:
                    now = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    pwr = bool(p[0] & 0x40)
                    ac = bool(p[0] & 0x01)
                    auto = bool(p[0] & 0x08)
                    dual = bool(p[0] & 0x04)
                    recirc = bool(p[1] & 0x10)
                    aqs = bool(p[1] & 0x08)
                    rear = bool(p[2] & 0x20)
                    front = bool(p[2] & 0x10)
                    b_int = p[3] & 0x03
                    dr_vent = p[4] & 0x0F
                    ps_vent = (p[4] >> 4) & 0x0F
                    fan = p[5]
                    lt = p[6]
                    rt = p[7]
                    lt_str = "LO" if lt == 0xFE else ("HI" if lt == 0xFF else f"{lt/2.0:.1f}C")
                    rt_str = "LO" if rt == 0xFE else ("HI" if rt == 0xFF else f"{rt/2.0:.1f}C")
                    print(f"[{now}] 🚗 [UART 0x31 CLIMA] Pwr={pwr} AC={ac} Auto={auto} Dual={dual} Fan={fan} L={lt_str} R={rt_str} Vent=(Dr:{dr_vent},Ps:{ps_vent}) Recirc={recirc} AQS={aqs} FrontDef={front} RearDef={rear} | {p.hex(' ')}")
                elif cmd in (0x11, 0x12, 0x21):
                    now = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    print(f"[{now}] 📡 [UART 0x{cmd:02X}] {p.hex(' ')}")

    t_ka = threading.Thread(target=keepalive_worker, daemon=True)
    t_hs = threading.Thread(target=handshake_worker, daemon=True)
    t_rx = threading.Thread(target=uart_rx_worker, daemon=True)

    t_ka.start()
    t_hs.start()
    t_rx.start()

    # Main loop: CAN delta sniffer on climate IDs
    CLIMA_IDS = {0x1D0, 0x1E3, 0x12D, 0x1E1, 0x217, 0x228, 0x366}
    last_can = {}

    try:
        while not stop_event.is_set():
            frame_raw = can_sock.recv(16)
            if not frame_raw:
                continue
            can_id, dlc, data = struct.unpack("=IB3x8s", frame_raw)
            if can_id not in CLIMA_IDS:
                continue
            payload = data[:dlc]
            if (can_id not in last_can) or (last_can[can_id] != payload):
                old = last_can.get(can_id, b'')
                now = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                diffs = []
                for i in range(min(len(old), len(payload))):
                    if old[i] != payload[i]:
                        diffs.append(f"B{i}: 0x{old[i]:02X}->0x{payload[i]:02X}")
                diff_str = f" ({', '.join(diffs)})" if diffs else " (NEW/DLC)"
                print(f"[{now}] ⚡ [CAN DELTA] 0x{can_id:03X} DLC={dlc}: {payload.hex(' ').upper()}{diff_str}")
                last_can[can_id] = payload

    except KeyboardInterrupt:
        print("\n[INFO] Stopping sniffer...")
    finally:
        stop_event.set()
        can_sock.close()
        ser.close()

if __name__ == "__main__":
    main()

