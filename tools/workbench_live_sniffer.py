#!/usr/bin/env python3
"""
Dual CAN + UART Live Event Sniffer for Peugeot 407 Workbench
============================================================
Logs real-time CAN bus traffic on can0 and UART frames on /dev/ttyUSB0.
Performs delta-filtering on CAN frames: only prints when a CAN ID's data
changes, or when a new CAN ID appears, making button presses and state
transitions instantly obvious.
Simultaneously decodes Raise (0x2E) and Hiworld (0x5A) UART packets.
"""

import sys
import time
import socket
import struct
import threading
import datetime
import serial
try:
    from canbox_usb_ports import find_canbox_uart_device
except ImportError:
    import os
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from canbox_usb_ports import find_canbox_uart_device

def calc_raise_cs(cmd: int, payload: bytes) -> int:
    s = (cmd + len(payload) + sum(payload)) & 0xFF
    return (~s) & 0xFF

def calc_hiworld_cs(length: int, cmd: int, payload: bytes) -> int:
    return ((length + cmd + sum(payload)) - 1) & 0xFF

def main():
    can_iface = "can0"
    uart_port = find_canbox_uart_device()
    baud_rate = 38400

    log_filename = f"sniff_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    log_file = open(log_filename, "w", buffering=1)

    print("=" * 70)
    print("  PEUGEOT 407 WORKBENCH LIVE EVENT SNIFFER")
    print(f"  CAN Interface : {can_iface}")
    print(f"  UART Port     : {uart_port} @ {baud_rate} baud")
    print(f"  Log File      : {log_filename}")
    print("=" * 70)
    print("  [CAN DELTA FILTER ACTIVE] Identical periodic frames are suppressed.")
    print("  Press physical buttons (Rear Defrost, Climate, etc.) on the fascia now!\n")

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
        print(f"[WARN] Cannot open UART {uart_port}: {e} (CAN sniffing will continue)")
        ser = None

    # Thread 1: UART Listener & Decoder
    def uart_worker():
        if not ser:
            return
        rx_buf = bytearray()
        while not stop_event.is_set():
            try:
                chunk = ser.read(64)
                if chunk:
                    rx_buf.extend(chunk)
                    now_str = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

                    while len(rx_buf) >= 4:
                        # Try Raise: 0x2E <Cmd> <Len> <Payload...> <CS>
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
                                full_raw = f"2E {cmd:02X} {length:02X} {payload.hex(' ')} {cs:02X}"
                                log_line = f"[{now_str}] [UART RAISE 0x{cmd:02X}] {full_raw}"
                                log_file.write(log_line + "\n")

                                # Decode interesting Raise commands
                                if cmd == 0x21:
                                    pwr = bool(payload[0] & 0x80) if len(payload) > 0 else False
                                    rear = bool(payload[0] & 0x01) if len(payload) > 0 else False
                                    front = bool(payload[4] & 0x80) if len(payload) > 4 else False
                                    fan = payload[1] & 0x0F if len(payload) > 1 else 0
                                    ltemp = payload[2] if len(payload) > 2 else 0
                                    rtemp = payload[3] if len(payload) > 3 else 0
                                    print(f"\n  >>> [UART CLIMA] Raise 0x21 | Pwr={pwr} RearDefrost={rear} FrontDefrost={front} Fan={fan} LTemp={ltemp/2.0}C RTemp={rtemp/2.0}C | Raw: {full_raw}")
                                else:
                                    print(f"  [UART RAISE] Cmd 0x{cmd:02X} (len {length}): {payload.hex(' ')}")
                                continue
                            else:
                                rx_buf.pop(0)
                                continue

                        # Try Hiworld: 0x5A 0xA5 <Len> <Cmd> <Payload...> <CS>
                        elif len(rx_buf) >= 5 and rx_buf[0] == 0x5A and rx_buf[1] == 0xA5:
                            length = rx_buf[2]
                            if length > 32:
                                rx_buf.pop(0)
                                continue
                            total_len = 3 + length + 1
                            if len(rx_buf) < total_len:
                                break
                            cmd = rx_buf[3]
                            payload = bytes(rx_buf[4:3 + length])
                            cs = rx_buf[3 + length]
                            if cs == calc_hiworld_cs(length, cmd, payload):
                                del rx_buf[:total_len]
                                full_raw = f"5A A5 {length:02X} {cmd:02X} {payload.hex(' ')} {cs:02X}"
                                log_line = f"[{now_str}] [UART HIWORLD 0x{cmd:02X}] {full_raw}"
                                log_file.write(log_line + "\n")
                                if cmd == 0x31:
                                    rear = bool(payload[2] & 0x20) if len(payload) > 2 else False
                                    front = bool(payload[2] & 0x10) if len(payload) > 2 else False
                                    print(f"\n  >>> [UART CLIMA] Hiworld 0x31 | RearDefrost={rear} FrontDefrost={front} | Raw: {full_raw}")
                                else:
                                    print(f"  [UART HIWORLD] Cmd 0x{cmd:02X}: {payload.hex(' ')}")
                                continue
                            else:
                                rx_buf.pop(0)
                                continue
                        else:
                            rx_buf.pop(0)
            except Exception as e:
                time.sleep(0.01)

    t_uart = threading.Thread(target=uart_worker, daemon=True)
    t_uart.start()

    # Main thread: CAN Delta Sniffer
    last_can_frames = {}
    known_ids = set()

    CLIMA_IDS = {0x1D0, 0x1E3, 0x12D, 0x1E0, 0x1E1, 0x217, 0x228, 0x336, 0x366, 0x0F6, 0x128, 0x168}

    try:
        while not stop_event.is_set():
            frame_raw = can_sock.recv(16)
            if not frame_raw:
                continue
            can_id, dlc, data = struct.unpack("=IB3x8s", frame_raw)
            payload = data[:dlc]

            now_str = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            log_line = f"({now_str}) can0 {can_id:03X}#{payload.hex().upper()}"
            log_file.write(log_line + "\n")

            is_new = can_id not in known_ids
            has_changed = (can_id not in last_can_frames) or (last_can_frames[can_id] != payload)

            if is_new:
                known_ids.add(can_id)
                tag = "[NEW CAN ID]"
                if can_id in CLIMA_IDS:
                    tag += " ⭐ [CLIMATE/BODY]"
                print(f"[{now_str}] {tag} 0x{can_id:03X} DLC={dlc} -> {payload.hex(' ').upper()}")
            elif has_changed:
                old_payload = last_can_frames[can_id]
                tag = "[DELTA]"
                if can_id in CLIMA_IDS:
                    tag = "⭐ [CLIMA CHANGE]"
                print(f"[{now_str}] {tag} 0x{can_id:03X} DLC={dlc}: {old_payload.hex(' ').upper()} -> {payload.hex(' ').upper()}")

            last_can_frames[can_id] = payload

    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        can_sock.close()
        if ser:
            ser.close()
        log_file.close()
        print(f"\n[DONE] Sniffing stopped. Log saved to {log_filename}")

if __name__ == "__main__":
    main()

