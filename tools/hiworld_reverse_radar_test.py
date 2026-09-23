#!/usr/bin/env python3
"""
Hiworld Protocol Reverse & Radar Emulator
=========================================
Emulates Hiworld CAN box protocol packets sent to an Android Head Unit over
physical serial interface (/dev/ttyUSB1 @ 38400 baud by default).

Note on Reverse Screen Activation:
On Android Head Units (TS10, UIS7862, Topway, etc.), the reverse camera / radar
view is triggered by the hardware MCU BACK/REVERSE wire or by broadcasting
'com.qf.action.BACKCAR_START' over Android (KeyEvent 289). Use the --adb-trigger
flag to automatically send this broadcast via ADB if testing over USB/Wi-Fi.

Emulation targets:
1. Version handshake (0xF0) & Feature Enables (0x71, 0x72)
2. Radar sensor telemetry (0x41) with obstacle on BACK RIGHT (RR)
3. Steering angle guide lines (0x11)
4. Heartbeat keep-alive (0xFF)
"""

import argparse
import subprocess
import sys
import time
import signal

try:
    import serial
except ImportError:
    print("[ERROR] 'pyserial' package is required.")
    print("        Install it with: pip install pyserial")
    sys.exit(1)


# Hiworld Protocol Constants
HIWORLD_SOF1 = 0x5A
HIWORLD_SOF2 = 0xA5

CMD_CAR_BASE_INFO    = 0x11
CMD_DOOR_WINDOW      = 0x12
CMD_ECU_INFO_PAGE1   = 0x13
CMD_CAR_RADAR_STATE  = 0x41
CMD_FEATURE_ENABLE1  = 0x71
CMD_FEATURE_ENABLE2  = 0x72
CMD_VERSION_REPORT   = 0xF0
CMD_HEARTBEAT        = 0xFF


def build_hiworld_packet(cmd: int, payload: bytes or list or bytearray) -> bytes:
    """
    Builds a Hiworld framing packet:
    [0x5A, 0xA5, Len, Cmd, Payload..., Checksum]
    Checksum = (sum(Len + Cmd + Payload...) - 1) & 0xFF
    """
    payload_bytes = bytes(payload)
    length = len(payload_bytes)
    
    total_sum = length + cmd + sum(payload_bytes)
    checksum = (total_sum - 1) & 0xFF
    
    return bytes([HIWORLD_SOF1, HIWORLD_SOF2, length, cmd]) + payload_bytes + bytes([checksum])


def send_adb_broadcast(action: str):
    """Sends Android broadcast via ADB to toggle BackCar state."""
    cmd = ["adb", "shell", "am", "broadcast", "-a", action]
    try:
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    except Exception:
        pass


def main():
    parser = argparse.ArgumentParser(
        description="Hiworld Protocol Reverse & Back-Right Radar Serial Emulator"
    )
    parser.add_argument(
        "--port", "-p",
        default="/dev/ttyUSB1",
        help="Serial port path (default: /dev/ttyUSB1)"
    )
    parser.add_argument(
        "--baud", "-b",
        type=int,
        default=38400,
        help="Baud rate (default: 38400 for Hiworld protocol)"
    )
    parser.add_argument(
        "--rate", "-r",
        type=float,
        default=10.0,
        help="Broadcast frequency in Hz (default: 10.0 Hz / 100ms interval)"
    )
    parser.add_argument(
        "--radar-level", "-l",
        type=int,
        default=3,
        choices=[0, 1, 2, 3, 4, 5, 6],
        help="Obstacle distance level on Back Right: 0 (closest/red) to 6 (far/green) (default: 3)"
    )
    parser.add_argument(
        "--fw-version",
        default="HW_PSA_V2.04.01",
        help="Firmware version string to report (default: HW_PSA_V2.04.01)"
    )
    parser.add_argument(
        "--adb-trigger",
        action="store_true",
        help="Automatically trigger Android BACKCAR_START via ADB on start and BACKCAR_STOP on exit"
    )
    args = parser.parse_args()

    print("=" * 72)
    print("      HIWORLD PROTOCOL REVERSE & RADAR EMULATOR")
    print("=" * 72)
    print(f"Target Port      : {args.port}")
    print(f"Baud Rate        : {args.baud} 8N1")
    print(f"Broadcast Rate   : {args.rate} Hz ({1000/args.rate:.0f} ms interval)")
    print(f"Radar Obstacle   : BACK RIGHT (RR) -> Zone {args.radar_level}")
    print(f"ADB Auto-Trigger : {'Enabled' if args.adb_trigger else 'Disabled (trigger via BACK wire or adb)'}")
    print("=" * 72)

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )
        print(f"[OK] Opened serial device: {args.port}")
    except Exception as e:
        print(f"[ERROR] Failed to open {args.port}: {e}")
        print("Tip: Check device permissions (e.g., sudo chmod 666 /dev/ttyUSB1)")
        sys.exit(1)

    # 1. Prepare Initial Handshake Packets
    # Version packet (Cmd 0xF0)
    version_payload = args.fw_version.encode("ascii")
    pkt_version = build_hiworld_packet(CMD_VERSION_REPORT, version_payload)

    # Feature Enable 1 (Cmd 0x71): Lighting, Locks, Radar support enabled
    pkt_feat1 = build_hiworld_packet(CMD_FEATURE_ENABLE1, [0xFF, 0xFF])

    # Feature Enable 2 (Cmd 0x72): TPMS, Camera, Mirror features enabled
    pkt_feat2 = build_hiworld_packet(CMD_FEATURE_ENABLE2, [0xFB, 0xFF])

    # 2. Prepare Periodic Telemetry Packets
    # Heartbeat (Cmd 0xFF)
    pkt_heartbeat = build_hiworld_packet(CMD_HEARTBEAT, [0x01])

    # Steering Track Angle for Dynamic Camera Guide Lines (Cmd 0x11): 0 degrees center
    pkt_steering_track = build_hiworld_packet(CMD_CAR_BASE_INFO, [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    # Radar State (Cmd 0x41):
    # Sensor indices:
    #   Byte 0: Rear-Left (RL)
    #   Byte 1: Rear-Mid-Left (RML)
    #   Byte 2: Rear-Mid-Right (RMR)
    #   Byte 3: Rear-Right (RR)  <-- Obstacle here!
    #   Byte 4: Front-Left (FL)
    #   Byte 5: Front-Mid-Left (FML)
    #   Byte 6: Front-Mid-Right (FMR)
    #   Byte 7: Front-Right (FR)
    # Value semantics: 0x00..0x06 = Obstacle proximity zone; 0xFF = Clear / No obstacle
    radar_payload = [
        0xFF,              # RL: Clear
        0xFF,              # RML: Clear
        0xFF,              # RMR: Clear
        args.radar_level,  # RR: Obstacle on back right
        0xFF,              # FL: Clear
        0xFF,              # FML: Clear
        0xFF,              # FMR: Clear
        0xFF               # FR: Clear
    ]
    pkt_radar = build_hiworld_packet(CMD_CAR_RADAR_STATE, radar_payload)

    print("\n[INFO] Transmitting initial handshake sequence...")
    for name, pkt in [("Version Report (0xF0)", pkt_version),
                      ("Feature Enable 1 (0x71)", pkt_feat1),
                      ("Feature Enable 2 (0x72)", pkt_feat2)]:
        ser.write(pkt)
        ser.flush()
        print(f"  -> Sent {name:25s}: {pkt.hex(' ')}")
        time.sleep(0.05)

    if args.adb_trigger:
        print("[INFO] Sending ADB broadcast: com.qf.action.BACKCAR_START")
        send_adb_broadcast("com.qf.action.BACKCAR_START")
    else:
        print("[TIP] To open the reverse window via ADB, run in another terminal:")
        print("      adb shell am broadcast -a com.qf.action.BACKCAR_START")

    interval = 1.0 / args.rate
    print(f"\n[INFO] Starting periodic transmission loop (Interval: {interval*1000:.1f} ms)...")
    print("       Press Ctrl+C to exit safely.\n")

    running = True

    def sig_handler(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    cycle = 0
    try:
        while running:
            cycle += 1

            # Transmit radar obstacle status (Cmd 0x41)
            ser.write(pkt_radar)

            # Transmit steering angle guide line (Cmd 0x11)
            ser.write(pkt_steering_track)

            # Transmit periodic heartbeat every 5 cycles (~500ms)
            if cycle % 5 == 0:
                ser.write(pkt_heartbeat)

            ser.flush()

            # Read any response from HU if present
            if ser.in_waiting > 0:
                rx = ser.read(ser.in_waiting)
                print(f"\r\033[K[HU RX] Received {len(rx)} bytes: {rx.hex(' ')}")

            print(
                f"\r\033[K[TX Cycle #{cycle:05d}] Radar RR: Zone {args.radar_level} | "
                f"Radar Frame: {pkt_radar.hex(' ')}",
                end="",
                flush=True
            )

            time.sleep(interval)

    except serial.SerialException as e:
        print(f"\n[ERROR] Serial transmission failure: {e}")
    finally:
        print("\n\n[*] Clearing radar and closing serial on shutdown...")
        try:
            # Clear radar [0xFF] * 8
            pkt_radar_clear = build_hiworld_packet(CMD_CAR_RADAR_STATE, [0xFF] * 8)
            ser.write(pkt_radar_clear)
            ser.flush()
            ser.close()
            print("[OK] Serial device closed cleanly.")
        except Exception:
            pass

        if args.adb_trigger:
            print("[INFO] Sending ADB broadcast: com.qf.action.BACKCAR_STOP")
            send_adb_broadcast("com.qf.action.BACKCAR_STOP")


if __name__ == "__main__":
    main()
