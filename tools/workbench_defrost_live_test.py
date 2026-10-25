#!/usr/bin/env python3
"""
Peugeot 407 Rear Defrost Live Workbench Test
============================================
Simulates Engine Running (900 RPM on 0x0B6) and Alternator/Compressor Authorized
on 0x12D so the physical BSI allows high-load heating consumers (Rear Defrost).
Sniffs CAN frames and Canbox UART telemetry in real-time.
"""

import sys
import time
import socket
import struct
import threading
import datetime
import serial

def calc_raise_cs(cmd: int, payload: bytes) -> int:
    s = (cmd + len(payload) + sum(payload)) & 0xFF
    return (~s) & 0xFF

def main():
    can_iface = "can0"
    try:
        from canbox_usb_ports import find_canbox_uart_device
        uart_port = find_canbox_uart_device()
    except Exception:
        uart_port = "/dev/ttyUSB0"
    baud_rate = 38400

    print("=" * 70)
    print("  PEUGEOT 407 REAR DEFROST LIVE WORKBENCH TEST")
    print(f"  Simulating Engine Running: 900 RPM on 0x0B6 (Alternator OK)")
    print(f"  CAN: {can_iface} | UART: {uart_port} @ {baud_rate} baud")
    print("=" * 70)

    stop_event = threading.Event()

    # Open CAN
    try:
        can_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        can_sock.bind((can_iface,))
    except Exception as e:
        print(f"[FATAL] Cannot open {can_iface}: {e}")
        return

    # Open UART
    try:
        ser = serial.Serial(uart_port, baud_rate, timeout=0.02)
    except Exception as e:
        ser = None
        print(f"[WARN] Cannot open UART {uart_port}: {e}")

    # Background CAN broadcaster: Engine Running + Compressor Authorized
    def engine_sim_worker():
        # 900 RPM: 900 * 8 = 7200 = 0x1C20
        frame_0b6 = struct.pack("=IB3x8s", 0x0B6, 8, bytes([0x1C, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0]))
        # Compressor loop authorized (0x80 in byte 7 and byte 0)
        frame_12d = struct.pack("=IB3x8s", 0x12D, 8, bytes([0x80, 0x32, 0x32, 0x00, 0x64, 0x00, 0x00, 0x80]))
        
        while not stop_event.is_set():
            try:
                can_sock.send(frame_0b6)
                can_sock.send(frame_12d)
            except OSError:
                pass
            time.sleep(0.04)

    t_engine = threading.Thread(target=engine_sim_worker, daemon=True)
    t_engine.start()

    # UART Listener
    def uart_worker():
        if not ser:
            return
        rx_buf = bytearray()
        while not stop_event.is_set():
            try:
                chunk = ser.read(64)
                if chunk:
                    rx_buf.extend(chunk)
                    while len(rx_buf) >= 4:
                        if rx_buf[0] == 0x2E:
                            cmd = rx_buf[1]
                            length = rx_buf[2]
                            if length > 32:
                                rx_buf.pop(0)
                                continue
                            total_len = 3 + length + 1
                            if len(rx_buf) < total_len:
                                break
                            payload = bytes(rx_buf[3:3 + length])
                            cs = rx_buf[3 + length]
                            if cs == calc_raise_cs(cmd, payload):
                                del rx_buf[:total_len]
                                if cmd == 0x21:
                                    pwr = bool(payload[0] & 0x80)
                                    rear = bool(payload[0] & 0x01)
                                    front = bool(payload[4] & 0x80) if len(payload) > 4 else False
                                    fan = payload[1] & 0x0F if len(payload) > 1 else 0
                                    raw = payload.hex(" ")
                                    tag = "🔥 [REAR DEFROST ON!]" if rear else "[CLIMA]"
                                    print(f"\n  >>> {tag} Raise 0x21: RearDefrost={rear} FrontDefrost={front} Fan={fan} Pwr={pwr} | {raw}")
                                else:
                                    pass
                                continue
                            else:
                                rx_buf.pop(0)
                                continue
                        else:
                            rx_buf.pop(0)
            except Exception:
                time.sleep(0.01)

    t_uart = threading.Thread(target=uart_worker, daemon=True)
    t_uart.start()

    print("[STATUS] Engine Running simulation is active on CAN bus.")
    print("[ACTION] Please press the Rear Defrost button on the fascia now!\n")

    last_can = {}
    CLIMA_IDS = {0x1D0, 0x1E3, 0x1E1, 0x217, 0x228, 0x336, 0x366, 0x0F6, 0x128, 0x168}

    try:
        while not stop_event.is_set():
            frame_raw = can_sock.recv(16)
            if not frame_raw:
                continue
            can_id, dlc, data = struct.unpack("=IB3x8s", frame_raw)
            payload = data[:dlc]

            now_str = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

            if (can_id not in last_can) or (last_can[can_id] != payload):
                old_p = last_can.get(can_id, b"")
                last_can[can_id] = payload
                if can_id in CLIMA_IDS:
                    print(f"[{now_str}] ⭐ [CLIMA CAN CHANGE] 0x{can_id:03X}: {old_p.hex(' ').upper()} -> {payload.hex(' ').upper()}")
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        can_sock.close()
        if ser:
            ser.close()

if __name__ == "__main__":
    main()

