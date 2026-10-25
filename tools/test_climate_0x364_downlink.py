#!/usr/bin/env python3
"""
Peugeot 407 Dual-Zone Climate Workbench - 0x364 & Hiworld 0x3B Verification
========================================================================
Tests:
1. Baseline sniffing of 0x1D0 / 0x1E3 (Current physical climate state).
2. CAN Injection of 0x364 frames on can0 (Driver Temp +, Driver Temp -, Passenger Temp +, etc.).
   Observes whether physical climate panel / BSI updates 0x1D0 / 0x1E3.
3. UART Injection of Hiworld Cmd 0x3B on /dev/ttyUSB0.
   Observes whether canbox transmits 0x364 on can0 and whether climate state updates.
"""

import sys
import os
import time
import socket
import struct
import threading
import serial

sys.path.insert(0, os.path.dirname(__file__))
from canbox_usb_ports import find_canbox_uart_device

CAN_FRAME_FMT = "=IB3x8s"

def make_can_frame(can_id: int, payload: bytes) -> bytes:
    dlc = len(payload)
    return struct.pack(CAN_FRAME_FMT, can_id, dlc, payload.ljust(8, b'\x00'))

def calc_hiworld_cs(length: int, cmd: int, payload: bytes) -> int:
    return ((length + cmd + sum(payload)) - 1) & 0xFF

def make_hiworld_pkt(cmd: int, payload: bytes) -> bytes:
    length = len(payload)
    cs = calc_hiworld_cs(length, cmd, payload)
    return bytes([0x5A, 0xA5, length, cmd]) + payload + bytes([cs])

def main():
    can_iface = "can0"
    uart_port = find_canbox_uart_device()
    baud_rate = 38400

    print("=" * 70)
    print("  PEUGEOT 407 WORKBENCH - 0x364 & HIWORLD 0x3B BENCH VERIFICATION")
    print(f"  CAN: {can_iface} | UART: {uart_port} @ {baud_rate} baud")
    print("=" * 70)

    # Open CAN socket
    can_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    can_sock.bind((can_iface,))

    # Open UART
    ser = serial.Serial(uart_port, baud_rate, timeout=0.01)

    # State tracking
    stop_event = threading.Event()
    cur_1d0 = None
    cur_1e3 = None
    seen_364_frames = []

    # Keepalive baseline frames (continuous 35ms)
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
        last_model_tx = 0
        while not stop_event.is_set():
            now = time.time()
            # Send keepalive CAN frames
            for cid, data in keepalive_frames.items():
                try:
                    can_sock.send(make_can_frame(cid, data))
                except Exception:
                    pass
            # Send Hiworld model 0x24 handshake every 2s
            if now - last_model_tx > 2.0:
                pkt_0x24 = make_hiworld_pkt(0x24, bytes([0x22, 0x00]))
                try:
                    ser.write(pkt_0x24)
                except Exception:
                    pass
                last_model_tx = now
            time.sleep(0.035)

    ka_thread = threading.Thread(target=keepalive_worker, daemon=True)
    ka_thread.start()

    # CAN listener worker
    def can_listener():
        nonlocal cur_1d0, cur_1e3
        while not stop_event.is_set():
            try:
                raw = can_sock.recv(16)
                cid, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
                cid &= 0x7FF
                payload = data[:dlc]
                if cid == 0x1D0:
                    cur_1d0 = payload
                elif cid == 0x1E3:
                    cur_1e3 = payload
                elif cid == 0x364:
                    seen_364_frames.append((time.time(), payload))
            except Exception:
                pass

    cl_thread = threading.Thread(target=can_listener, daemon=True)
    cl_thread.start()

    # UART listener worker
    uart_packets = []
    def uart_listener():
        rx_buf = bytearray()
        while not stop_event.is_set():
            try:
                b = ser.read(64)
                if b:
                    rx_buf.extend(b)
                    while len(rx_buf) >= 5:
                        if rx_buf[0] != 0x5A or rx_buf[1] != 0xA5:
                            rx_buf.pop(0)
                            continue
                        length = rx_buf[2]
                        total_len = length + 5
                        if len(rx_buf) < total_len:
                            break
                        cmd = rx_buf[3]
                        payload = bytes(rx_buf[4:4 + length])
                        cs = rx_buf[4 + length]
                        exp_cs = calc_hiworld_cs(length, cmd, payload)
                        if cs == exp_cs:
                            uart_packets.append((time.time(), cmd, payload))
                        rx_buf = rx_buf[total_len:]
            except Exception:
                pass

    ul_thread = threading.Thread(target=uart_listener, daemon=True)
    ul_thread.start()

    time.sleep(1.0) # Wait for initial state to settle

    def decode_temp_index(idx):
        if idx == 0:
            return "LO (Index 0)"
        elif idx == 22:
            return "HI (Index 22)"
        elif 1 <= idx <= 21:
            c = 14.0 + (idx - 1) * 0.5
            return f"{c:.1f}°C (Index {idx})"
        return f"Unknown ({idx})"

    def print_state(label="Current State"):
        if cur_1d0 and cur_1e3:
            l_1d0, r_1d0 = cur_1d0[5], cur_1d0[6]
            l_1e3, r_1e3 = cur_1e3[2], cur_1e3[3]
            fan = cur_1d0[2] & 0x0F
            print(f"[{label}]")
            print(f"  • 0x1D0: {cur_1d0.hex()} -> Left: {decode_temp_index(l_1d0)} | Right: {decode_temp_index(r_1d0)} | Fan: {fan}")
            print(f"  • 0x1E3: {cur_1e3.hex()} -> Left: {decode_temp_index(l_1e3)} | Right: {decode_temp_index(r_1e3)}")
        else:
            print(f"[{label}] No 0x1D0 / 0x1E3 received yet! (1D0={cur_1d0}, 1E3={cur_1e3})")

    print("\n--- PHASE 1: Baseline Climate State ---")
    print_state("Baseline")

    # Helper to send pulsed 0x364
    def pulse_364(payload_byte0, pulse_count=15, period=0.01):
        active_payload = bytes([payload_byte0, 0x00, 0x00, 0x00, 0x00, 0x00])
        idle_payload = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        for _ in range(pulse_count):
            can_sock.send(make_can_frame(0x364, active_payload))
            time.sleep(period)
        for _ in range(5):
            can_sock.send(make_can_frame(0x364, idle_payload))
            time.sleep(period)

    print("\n--- PHASE 2: Direct CAN Injection of 0x364 on can0 ---")
    
    # Test 1: Driver Temp UP (0x80)
    print("\n[CAN Test 1] Injecting 0x364#800000000000 (Driver Temp UP, 15 pulses)...")
    init_l = cur_1d0[5] if cur_1d0 else None
    pulse_364(0x80)
    time.sleep(0.3)
    print_state("After Driver Temp UP")
    post_l = cur_1d0[5] if cur_1d0 else None
    if init_l is not None and post_l is not None:
        print(f"  Result: Driver temp index changed: {init_l} -> {post_l} ({'+1' if post_l > init_l else 'NO CHANGE'})")

    time.sleep(0.5)

    # Test 2: Driver Temp DOWN (0x40)
    print("\n[CAN Test 2] Injecting 0x364#400000000000 (Driver Temp DOWN, 15 pulses)...")
    init_l = cur_1d0[5] if cur_1d0 else None
    pulse_364(0x40)
    time.sleep(0.3)
    print_state("After Driver Temp DOWN")
    post_l = cur_1d0[5] if cur_1d0 else None
    if init_l is not None and post_l is not None:
        print(f"  Result: Driver temp index changed: {init_l} -> {post_l} ({'-1' if post_l < init_l else 'NO CHANGE'})")

    time.sleep(0.5)

    # Test 3: Passenger Temp UP (0x20)
    print("\n[CAN Test 3] Injecting 0x364#200000000000 (Passenger Temp UP, 15 pulses)...")
    init_r = cur_1d0[6] if cur_1d0 else None
    pulse_364(0x20)
    time.sleep(0.3)
    print_state("After Passenger Temp UP")
    post_r = cur_1d0[6] if cur_1d0 else None
    if init_r is not None and post_r is not None:
        print(f"  Result: Passenger temp index changed: {init_r} -> {post_r} ({'+1' if post_r > init_r else 'NO CHANGE'})")

    time.sleep(0.5)

    # Test 4: Passenger Temp DOWN (0x10)
    print("\n[CAN Test 4] Injecting 0x364#100000000000 (Passenger Temp DOWN, 15 pulses)...")
    init_r = cur_1d0[6] if cur_1d0 else None
    pulse_364(0x10)
    time.sleep(0.3)
    print_state("After Passenger Temp DOWN")
    post_r = cur_1d0[6] if cur_1d0 else None
    if init_r is not None and post_r is not None:
        print(f"  Result: Passenger temp index changed: {init_r} -> {post_r} ({'-1' if post_r < init_r else 'NO CHANGE'})")

    time.sleep(0.5)

    # Test 5: Fan Speed UP (0x08) & DOWN (0x04)
    print("\n[CAN Test 5] Injecting 0x364#080000000000 (Fan Speed UP, 15 pulses)...")
    init_fan = (cur_1d0[2] & 0x0F) if cur_1d0 else None
    pulse_364(0x08)
    time.sleep(0.3)
    print_state("After Fan Speed UP")
    post_fan = (cur_1d0[2] & 0x0F) if cur_1d0 else None
    if init_fan is not None and post_fan is not None:
        print(f"  Result: Fan speed changed: {init_fan} -> {post_fan}")

    time.sleep(0.8)

    print("\n--- PHASE 3: Hiworld UART Downlink Injection (Cmd 0x3B) on /dev/ttyUSB0 ---")
    seen_364_frames.clear()

    # Subcmd 0x0C, Arg 0x01: Left Temp UP
    pkt = make_hiworld_pkt(0x3B, bytes([0x0C, 0x01]))
    print(f"\n[UART Test 1] Sending Hiworld Downlink to canbox: {pkt.hex()} (Left Temp UP)...")
    t_send = time.time()
    ser.write(pkt)
    time.sleep(0.5)
    
    # Check if canbox sent 0x364 on can0
    tx_364_on_can = [f for f in seen_364_frames if f[0] >= t_send]
    print(f"  Canbox transmitted {len(tx_364_on_can)} frame(s) on CAN 0x364:")
    for t, p in tx_364_on_can[:5]:
        print(f"    CAN ID 0x364: {p.hex()}")
    print_state("After UART Left Temp UP")

    # Subcmd 0x0C, Arg 0x02: Left Temp DOWN
    pkt = make_hiworld_pkt(0x3B, bytes([0x0C, 0x02]))
    print(f"\n[UART Test 2] Sending Hiworld Downlink to canbox: {pkt.hex()} (Left Temp DOWN)...")
    t_send = time.time()
    seen_364_frames.clear()
    ser.write(pkt)
    time.sleep(0.5)

    tx_364_on_can = [f for f in seen_364_frames if f[0] >= t_send]
    print(f"  Canbox transmitted {len(tx_364_on_can)} frame(s) on CAN 0x364:")
    for t, p in tx_364_on_can[:5]:
        print(f"    CAN ID 0x364: {p.hex()}")
    print_state("After UART Left Temp DOWN")

    # Check recent UART 0x31 packets from canbox
    recent_31 = [p for p in uart_packets if p[1] == 0x31]
    print(f"\nTotal Hiworld 0x31 climate packets received from canbox during test: {len(recent_31)}")
    if recent_31:
        latest = recent_31[-1]
        print(f"  Latest 0x31 payload: {latest[2].hex()}")

    stop_event.set()
    ka_thread.join(timeout=1.0)
    cl_thread.join(timeout=1.0)
    ul_thread.join(timeout=1.0)
    ser.close()
    can_sock.close()
    print("\n" + "=" * 70)
    print("  VERIFICATION COMPLETE")
    print("=" * 70)

if __name__ == "__main__":
    main()
