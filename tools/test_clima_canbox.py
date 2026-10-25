#!/usr/bin/env python3
"""
Test Climate (HVAC) reporting on connected Canbox.
Verifies whether the canbox reads CAN climate frames (0x1D0, 0x1E3, etc.)
and reports HVAC status (Raise Cmd 0x21 or Hiworld Cmd 0x31) over UART when
temperature or climate controls change.
"""

import sys
import time
import socket
import struct
import threading
import serial

def calc_raise_cs(cmd: int, payload: bytes) -> int:
    s = (cmd + len(payload) + sum(payload)) & 0xFF
    return (~s) & 0xFF

def parse_raise_frame(buf: bytearray):
    """Try to extract a Raise frame (0x2E <cmd> <len> <payload...> <cs>)."""
    while len(buf) >= 4:
        if buf[0] != 0x2E:
            buf.pop(0)
            continue
        cmd = buf[1]
        length = buf[2]
        if length > 32:
            buf.pop(0)
            continue
        total_len = 3 + length + 1
        if len(buf) < total_len:
            break
        payload = bytes(buf[3:3 + length])
        cs = buf[3 + length]
        expected_cs = calc_raise_cs(cmd, payload)
        if cs == expected_cs:
            del buf[:total_len]
            return cmd, payload
        else:
            buf.pop(0)
    return None, None

def parse_hiworld_frame(buf: bytearray):
    """Try to extract a Hiworld frame (0x5A 0xA5 <len> <cmd> <payload...> <cs>)."""
    while len(buf) >= 5:
        if buf[0] != 0x5A or buf[1] != 0xA5:
            buf.pop(0)
            continue
        length = buf[2]
        if length > 32:
            buf.pop(0)
            continue
        total_len = 3 + length + 1
        if len(buf) < total_len:
            break
        cmd = buf[3]
        payload = bytes(buf[4:3 + length])
        cs = buf[3 + length]
        expected_cs = ((length + cmd + sum(payload)) - 1) & 0xFF
        if cs == expected_cs:
            del buf[:total_len]
            return cmd, payload
        else:
            buf.pop(0)
    return None, None

def main():
    print("=" * 60)
    print("  Canbox Climate (HVAC) Telemetry Verification Tool")
    print("=" * 60)

    # 1. Open CAN socket
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind(('can0',))

    def send_can(cid, data):
        f = struct.pack('=IB3x8s', cid, len(data), data.ljust(8, b'\x00'))
        try:
            sock.send(f)
        except Exception as e:
            pass

    # 2. Open UART
    port = "/dev/ttyUSB0"
    baud = 38400
    try:
        ser = serial.Serial(port, baud, timeout=0.02)
        print(f"[OK] Opened UART {port} @ {baud} bps")
    except Exception as e:
        print(f"[FAIL] Cannot open {port}: {e}")
        return

    stop = threading.Event()

    # Dynamic CAN frames dictionary for background broadcaster
    active_can_frames = {
        0x036: bytes([0x0E, 0x00, 0x00, 0x2A, 0x01, 0x00, 0x00, 0xA0]), # Wakeup / Active
        0x0B6: bytes([0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
        0x0F6: bytes([0x88, 0x00, 0x00, 0x00, 0x00, 0x50, 0x00, 0x20]), # Ambient temp 24C
        0x128: bytes([0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]), # Ignition ON
        0x15B: bytes([0x01, 0x03, 0xB1, 0x40, 0x00, 0xD0, 0x00, 0x10]),
        0x168: bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
        0x261: bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
        0x2A1: bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
    }

    lock = threading.Lock()

    def bg_can_broadcaster():
        while not stop.is_set():
            with lock:
                items = list(active_can_frames.items())
            for cid, data in items:
                send_can(cid, data)
            time.sleep(0.04) # 40 ms cycle

    can_thread = threading.Thread(target=bg_can_broadcaster, daemon=True)
    can_thread.start()

    # UART receiver thread
    rx_buf = bytearray()
    hvac_packets = []
    all_packets = []

    def uart_listener():
        while not stop.is_set():
            chunk = ser.read(64)
            if chunk:
                rx_buf.extend(chunk)
                while True:
                    # Try Raise
                    cmd, payload = parse_raise_frame(rx_buf)
                    if cmd is not None:
                        all_packets.append(("RAISE", cmd, payload))
                        if cmd == 0x21:
                            hvac_packets.append(("RAISE", payload))
                            # Decode Raise 0x21
                            pwr = (payload[0] & 0x80) != 0 if len(payload) > 0 else 0
                            auto = (payload[0] & 0x08) != 0 if len(payload) > 0 else 0
                            fan = (payload[1] & 0x0F) if len(payload) > 1 else 0
                            ltemp = payload[2] if len(payload) > 2 else 0
                            rtemp = payload[3] if len(payload) > 3 else 0
                            defrost = (payload[0] & 0x01) != 0 if len(payload) > 0 else 0
                            print(f"\n>>> [RAISE 0x21 CLIMA RECEIVED] Pwr={pwr} Auto={auto} Fan={fan} LTemp=0x{ltemp:02X} ({ltemp/2.0}C) RTemp=0x{rtemp:02X} ({rtemp/2.0}C) RearDefrost={defrost} | Raw: {payload.hex(' ')}")
                        else:
                            print(f"[UART RAISE 0x{cmd:02X}] {payload.hex(' ')}")
                        continue

                    # Try Hiworld
                    cmd, payload = parse_hiworld_frame(rx_buf)
                    if cmd is not None:
                        all_packets.append(("HIWORLD", cmd, payload))
                        if cmd == 0x31:
                            hvac_packets.append(("HIWORLD", payload))
                            print(f"\n>>> [HIWORLD 0x31 CLIMA RECEIVED] Raw: {payload.hex(' ')}")
                        else:
                            print(f"[UART HIWORLD 0x{cmd:02X}] {payload.hex(' ')}")
                        continue

                    break

    uart_thread = threading.Thread(target=uart_listener, daemon=True)
    uart_thread.start()

    print("[INFO] CAN keepalive running, listening for UART frames...")
    time.sleep(1.5)

    # Scenarios to test:
    # 1. 0x1D0 standard 407 panel format:
    #    Byte 0: 0x08 (Auto / AC on)
    #    Byte 1: 0x80 (Flags)
    #    Byte 2: 0x07 (Fan speed)
    #    Byte 3: 0x00 (Distribution)
    #    Byte 4: 0x00 (Flaps)
    #    Byte 5: Left Temp index
    #    Byte 6: Right Temp index
    #
    # 2. 0x1E3 standard 407 EMF display format:
    #    Byte 0: 0x1C (Auto, Mono)
    #    Byte 1: 0x40 (Compressor on)
    #    Byte 2: Left Temp (0x0B = 21C, 0x0D = 22C, 0x11 = 24C)
    #    Byte 3: Right Temp
    #    Byte 4: 0x00
    #    Byte 5: 0x00
    #    Byte 6: 0x07 (Fan speed)
    #
    # 3. Both 0x1D0 and 0x1E3 simultaneously (authentic PSA 407 setup)
    #
    # 4. Alternative 0x1D0 format (Raise test vector format):
    #    [0xC8, 0x45, <LeftTempRaw>, <RightTempRaw>, 0x00, 0x00, 0x40, 0x00]

    test_vectors = [
        # (Description, dict of {can_id: data_bytes}, duration_sec)
        ("Baseline (No HVAC frames yet)", {}, 1.5),

        # Test A: 0x1D0 only (Standard 407 index 11 = 21.0 C)
        ("Test A1: 0x1D0 only (Temp 21.0 C / Index 0x0B)", {
            0x1D0: bytes([0x08, 0x80, 0x07, 0x00, 0x00, 0x0B, 0x0B])
        }, 2.0),

        # Test A2: 0x1D0 Temp Change -> Index 15 = 23.0 C
        ("Test A2: 0x1D0 Temp Change -> 23.0 C (Index 0x0F)", {
            0x1D0: bytes([0x08, 0x80, 0x07, 0x00, 0x00, 0x0F, 0x0F])
        }, 2.0),

        # Test A3: 0x1D0 Temp Change -> LO (Index 0x00)
        ("Test A3: 0x1D0 Temp Change -> LO (Index 0x00)", {
            0x1D0: bytes([0x08, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00])
        }, 2.0),

        # Test A4: 0x1D0 Temp Change -> HI (Index 0x16 = 22)
        ("Test A4: 0x1D0 Temp Change -> HI (Index 0x16)", {
            0x1D0: bytes([0x08, 0x80, 0x07, 0x00, 0x00, 0x16, 0x16])
        }, 2.0),

        # Test B: 0x1E3 only
        ("Test B1: 0x1E3 only (Temp 21.0 C / Index 0x0B)", {
            0x1E3: bytes([0x1C, 0x40, 0x0B, 0x0B, 0x00, 0x00, 0x07])
        }, 2.0),

        ("Test B2: 0x1E3 Temp Change -> 24.0 C (Index 0x11)", {
            0x1E3: bytes([0x1C, 0x40, 0x11, 0x11, 0x00, 0x00, 0x07])
        }, 2.0),

        # Test C: Authentic PSA 407 (0x1D0 + 0x1E3 + 0x12D combined!)
        ("Test C1: Authentic Combined 0x1D0 + 0x1E3 + 0x12D (Temp 21.0 C)", {
            0x1D0: bytes([0x08, 0x80, 0x07, 0x00, 0x00, 0x0B, 0x0B]),
            0x1E3: bytes([0x1C, 0x40, 0x0B, 0x0B, 0x00, 0x00, 0x07]),
            0x12D: bytes([0x80, 0x32, 0x32, 0x00, 0x64, 0x00, 0x00, 0x80])
        }, 2.5),

        ("Test C2: Authentic Combined 0x1D0 + 0x1E3 -> Temp 22.0 C (0x0D)", {
            0x1D0: bytes([0x08, 0x80, 0x07, 0x00, 0x00, 0x0D, 0x0D]),
            0x1E3: bytes([0x1C, 0x40, 0x0D, 0x0D, 0x00, 0x00, 0x07]),
            0x12D: bytes([0x80, 0x32, 0x32, 0x00, 0x64, 0x00, 0x00, 0x80])
        }, 2.5),

        ("Test C3: Authentic Combined 0x1D0 + 0x1E3 -> Temp 25.0 C (0x12)", {
            0x1D0: bytes([0x08, 0x80, 0x07, 0x00, 0x00, 0x12, 0x12]),
            0x1E3: bytes([0x1C, 0x40, 0x12, 0x12, 0x00, 0x00, 0x07]),
            0x12D: bytes([0x80, 0x32, 0x32, 0x00, 0x64, 0x00, 0x00, 0x80])
        }, 2.5),

        # Test D: Alternative/Synthetic Raise Vector (0x1D0 Byte 2/3 = Raw Temp: 0x2B = 21.5C, 0x2C = 22.0C)
        ("Test D1: Alternative 0x1D0 Vector (Byte 2=0x2B=21.5C, Byte 3=0x2C=22.0C)", {
            0x1D0: bytes([0xC8, 0x45, 0x2B, 0x2C, 0x00, 0x00, 0x40, 0x00])
        }, 2.5),

        ("Test D2: Alternative 0x1D0 Vector (Byte 2=0x30=24.0C, Byte 3=0x32=25.0C)", {
            0x1D0: bytes([0xC8, 0x45, 0x30, 0x32, 0x00, 0x00, 0x40, 0x00])
        }, 2.5),

        # Test E: Fan speed sweep on 0x1D0
        ("Test E: Fan Speed Changes (Speed 1 -> Speed 4 -> Speed 8)", {
            0x1D0: bytes([0x08, 0x80, 0x01, 0x00, 0x00, 0x0B, 0x0B]),
            0x1E3: bytes([0x1C, 0x40, 0x0B, 0x0B, 0x00, 0x00, 0x01]),
        }, 2.0),
    ]

    for desc, frames, duration in test_vectors:
        print(f"\n--- {desc} ---")
        with lock:
            # Clean previous test frames
            for k in [0x1D0, 0x1E3, 0x12D]:
                active_can_frames.pop(k, None)
            # Add new frames
            for k, v in frames.items():
                active_can_frames[k] = v
                print(f"  [CAN TX] 0x{k:03X} DLC={len(v)}: {v.hex(' ')}")
        time.sleep(duration)

    print("\n" + "=" * 60)
    print("  TEST SUMMARY")
    print("=" * 60)
    print(f"Total UART frames received: {len(all_packets)}")
    print(f"Total Clima frames received: {len(hvac_packets)}")
    if hvac_packets:
        print("\n[SUCCESS] Climate frames ARE reported by the canbox:")
        for proto, p in hvac_packets[:10]:
            print(f"  {proto}: {p.hex(' ')}")
    else:
        print("\n[FAIL] No Climate frames (Raise 0x21 or Hiworld 0x31) received.")
        print("Frames seen from canbox during test:")
        cmds = set(cmd for _, cmd, _ in all_packets)
        for c in cmds:
            sample = next(p for _, cmd, p in all_packets if cmd == c)
            print(f"  Cmd 0x{c:02X}: {sample.hex(' ')}")

    stop.set()
    ser.close()
    sock.close()

if __name__ == "__main__":
    main()

