#!/usr/bin/env python3
"""
HiWorld CAN Box Emulator & TPMS Generator with Handshake & Keep-Alive.
Compatible with Android Head Units (QF_Canbus, TS10, FYT, Topway, etc.)
"""

import argparse
import sys
import threading
import time

try:
    import serial
except ImportError:
    serial = None

# HiWorld Protocol Constants
SOF = bytes([0x5A, 0xA5])
CMD_BASE_INFO = 0x11        # Ignition, ACC, keys, steering
CMD_DOOR_STATUS = 0x12      # Door / Trunk / ACC status
CMD_TPMS_DISCRETE = 0x18    # Alarm states (OK / Low / Puncture)
CMD_TPMS_NUMERIC = 0x66     # Direct wheel pressures in 0.1 Bar
CMD_VERSION = 0xF0          # Firmware version & Model ID
CMD_CAR_SETTINGS = 0x31     # Vehicle customization page enable

# TPMS Alarm States
STATE_OK = 0x00
STATE_LOW_PRESSURE = 0x01
STATE_PUNCTURE = 0x02
STATE_OFFLINE = 0x03


def calculate_hiworld_checksum(length: int, cmd: int, payload: bytes) -> int:
    """HiWorld Checksum: ((length + cmd + sum(payload)) - 1) & 0xFF"""
    total = length + cmd + sum(payload)
    return (total - 1) & 0xFF


def build_hiworld_packet(cmd: int, payload: bytes) -> bytes:
    """Encapsulates payload into [0x5A, 0xA5, Len, Cmd, Payload..., Checksum]"""
    length = len(payload)
    checksum = calculate_hiworld_checksum(length, cmd, payload)
    return SOF + bytes([length, cmd]) + payload + bytes([checksum])


def build_version_packet(version_str: str = "V1.02.000HW") -> bytes:
    """Cmd 0xF0: Version identification packet"""
    payload = version_str.encode("ascii")
    return build_hiworld_packet(CMD_VERSION, payload)


def build_base_info_packet(acc_on: bool = True) -> bytes:
    """
    Cmd 0x11: Base vehicle state (Ignition/ACC active, steering centered).
    8 bytes payload.
    """
    # Byte 0: Power/ACC status (0x01 = ACC ON / Active)
    # Byte 1..5: Keys / Reserved
    # Byte 6..7: Steering angle (0 = center)
    p0 = 0x01 if acc_on else 0x00
    payload = bytes([p0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    return build_hiworld_packet(CMD_BASE_INFO, payload)


def build_numeric_tpms(fl_bar: float, fr_bar: float, rl_bar: float, rr_bar: float) -> bytes:
    """
    Cmd 0x66: 4-Wheel Pressures (0.1 Bar resolution).
    Payload: [Mode(0x01=Live), FL, FR, RL, RR, Unit(0x00=Bar)]
    """
    fl = int(round(fl_bar * 10))
    fr = int(round(fr_bar * 10))
    rl = int(round(rl_bar * 10))
    rr = int(round(rr_bar * 10))
    payload = bytes([0x01, fl, fr, rl, rr, 0x00])
    return build_hiworld_packet(CMD_NUMERIC_TPMS, payload)


def build_discrete_tpms(fl_state: int, fr_state: int, rl_state: int, rr_state: int) -> bytes:
    """
    Cmd 0x18: 4-Wheel Discrete Alarm Flags.
    Payload: [FL_state, FR_state, RL_state, RR_state]
    """
    payload = bytes([fl_state, fr_state, rl_state, rr_state])
    return build_hiworld_packet(CMD_DISCRETE_TPMS, payload)


def auto_alarm(pressure_bar: float, min_warn: float = 1.8) -> int:
    if pressure_bar <= 0.5:
        return STATE_PUNCTURE
    elif pressure_bar < min_warn:
        return STATE_LOW_PRESSURE
    return STATE_OK


def serial_listener(ser: "serial.Serial", version_pkt: bytes):
    """Listens for HU commands (e.g., Version requests) and responds."""
    rx_buf = bytearray()
    while ser.is_open:
        try:
            data = ser.read(ser.in_waiting or 1)
            if not data:
                continue
            rx_buf.extend(data)
            
            # Look for HiWorld SOF (0x5A 0xA5)
            while len(rx_buf) >= 4:
                if rx_buf[0] != 0x5A or rx_buf[1] != 0xA5:
                    rx_buf.pop(0)
                    continue
                
                pkt_len = rx_buf[2]
                total_len = pkt_len + 5  # SOF(2) + Len(1) + Cmd(1) + Payload(pkt_len) + CS(1)
                
                if len(rx_buf) < total_len:
                    break  # Wait for complete packet
                
                cmd = rx_buf[3]
                payload = rx_buf[4:4 + pkt_len]
                cs = rx_buf[total_len - 1]
                
                # Checksum verify
                expected_cs = calculate_hiworld_checksum(pkt_len, cmd, payload)
                if cs == expected_cs:
                    print(f"<- [HU RX] Cmd 0x{cmd:02X} (Len={pkt_len}): {rx_buf[:total_len].hex(' ').upper()}")
                    if cmd == CMD_VERSION or cmd == 0x7F:
                        # HU is asking for CAN box version
                        ser.write(version_pkt)
                        print(f"-> [TX Reply] Sent Version: {version_pkt.hex(' ').upper()}")
                
                rx_buf = rx_buf[total_len:]
        except Exception as e:
            break


def main():
    parser = argparse.ArgumentParser(description="HiWorld TPMS & Handshake Simulator")
    parser.add_argument("--fl", type=float, default=2.2, help="FL pressure in Bar (default: 2.2)")
    parser.add_argument("--fr", type=float, default=2.3, help="FR pressure in Bar (default: 2.3)")
    parser.add_argument("--rl", type=float, default=1.2, help="RL pressure in Bar (default: 1.2)")
    parser.add_argument("--rr", type=float, default=2.1, help="RR pressure in Bar (default: 2.1)")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB1", help="Serial port (e.g. /dev/ttyUSB0, COM3)")
    parser.add_argument("--baud", type=int, default=38400, help="Baud rate (default: 38400)")
    args = parser.parse_args()

    # Pre-build packets
    version_pkt = build_version_packet("V1.02.000HW")
    base_info_pkt = build_base_info_packet(acc_on=True)
    numeric_tpms_pkt = build_numeric_tpms(args.fl, args.fr, args.rl, args.rr)
    
    fl_st = auto_alarm(args.fl)
    fr_st = auto_alarm(args.fr)
    rl_st = auto_alarm(args.rl)
    rr_st = auto_alarm(args.rr)
    discrete_tpms_pkt = build_discrete_tpms(fl_st, fr_st, rl_st, rr_st)

    print("=" * 65)
    print("HiWorld TPMS Simulator with Handshake & Keep-Alive")
    print("=" * 65)
    print(f"Pressures: FL={args.fl:.1f} Bar, FR={args.fr:.1f} Bar, RL={args.rl:.1f} Bar, RR={args.rr:.1f} Bar")
    print(f"Alarms:    FL={fl_st}, FR={fr_st}, RL={rl_st} (LOW), RR={rr_st}")
    print("-" * 65)
    print(f"1. Version Handshake (0xF0): {version_pkt.hex(' ').upper()}")
    print(f"2. Base Power/ACC ON (0x11): {base_info_pkt.hex(' ').upper()}")
    print(f"3. Numeric TPMS      (0x66): {numeric_tpms_pkt.hex(' ').upper()}")
    print(f"4. Discrete Alarm    (0x18): {discrete_tpms_pkt.hex(' ').upper()}")
    print("=" * 65)

    if not args.port:
        print("\nNote: Provide --port <PORT> to transmit live to your Head Unit.")
        return

    if serial is None:
        print("Error: pyserial is required. Run: pip install pyserial")
        sys.exit(1)

    print(f"Opening {args.port} @ {args.baud} 8N1...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        print(f"Failed to open {args.port}: {e}")
        return

    # Start background listener thread to reply to HU version queries
    rx_thread = threading.Thread(target=serial_listener, args=(ser, version_pkt), daemon=True)
    rx_thread.start()

    print("Transmitting Handshake & Telemetry Stream (Press Ctrl+C to stop)...")
    
    # 1. Initial Handshake Burst
    for _ in range(3):
        ser.write(version_pkt)
        time.sleep(0.05)
        ser.write(base_info_pkt)
        time.sleep(0.05)

    try:
        count = 0
        while True:
            # Heartbeat: Base Info every 200ms
            ser.write(base_info_pkt)
            time.sleep(0.1)

            # TPMS Telemetry every 500ms
            if count % 2 == 0:
                ser.write(numeric_tpms_pkt)
                ser.write(discrete_tpms_pkt)
                # Re-announce version every 5 seconds
                if count % 20 == 0:
                    ser.write(version_pkt)

            count += 1
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()