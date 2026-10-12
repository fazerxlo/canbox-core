#!/usr/bin/env python3
"""
Hiworld CAN Box Serial Firmware Flasher / Updater (IAP Over UART)

Builds the Nuvoton NUC131 application firmware using PlatformIO,
packages it into a Hiworld .iap container, triggers bootloader mode
via the Hiworld 0xE0 command, unlocks flash, and streams the firmware records.

No ST-Link programmer required! Operates over USB-UART adapter connected to Head Unit RX/TX.
"""

import argparse
import glob
import os
import subprocess
import sys
import time

try:
    import serial
except ImportError:
    print("Error: 'pyserial' package is required. Install with: pip install pyserial")
    sys.exit(1)

# Protocol Constants
CMD_UPGRADE_TRIGGER = bytes([0x5A, 0xA5, 0x02, 0xE0, 0x00, 0x00, 0xE1])
UNLOCK_PASSWORD     = bytes([0x19, 0x78, 0x02, 0x17])
RECORD_SIZE         = 136
PAYLOAD_SIZE        = 128
MAGIC_HEADER        = bytes.fromhex("0109070800020107913247a4")
DEFAULT_KEY         = bytes([0x19, 0x84, 0x07, 0x42])


def find_serial_port() -> str:
    """Finds an available USB-to-serial port (/dev/ttyUSB* or COM*)."""
    if sys.platform.startswith("win"):
        ports = [f"COM{i}" for i in range(1, 32)]
    else:
        ports = sorted(glob.glob("/dev/ttyUSB*"))
        if not ports:
            ports = sorted(glob.glob("/dev/ttyACM*"))

    for p in ports:
        try:
            s = serial.Serial(p)
            s.close()
            return p
        except Exception:
            pass

    return "/dev/ttyUSB1" if os.path.exists("/dev/ttyUSB1") else "/dev/ttyUSB0"


def build_firmware(pio_env: str) -> str:
    """Invokes PlatformIO to compile the target application firmware."""
    print(f"\n[1/4] Building target firmware (Environment: '{pio_env}')...")
    pio_path = os.path.expanduser("~/.platformio/penv/bin/pio")
    if not os.path.exists(pio_path):
        pio_path = "pio"

    cmd = [pio_path, "run", "-e", pio_env]
    res = subprocess.run(cmd)
    if res.returncode != 0:
        print("\n[-] Build failed!")
        sys.exit(1)

    bin_path = os.path.join(".pio", "build", pio_env, "program.bin")
    if not os.path.exists(bin_path):
        print(f"[-] Binary not found at '{bin_path}'!")
        sys.exit(1)

    bin_size = os.path.getsize(bin_path)
    print(f"[+] Build succeeded: '{bin_path}' ({bin_size:,} bytes)")
    return bin_path


def pack_iap_data(bin_path: str, key: bytes = DEFAULT_KEY) -> bytes:
    """Packages raw binary into Hiworld 136-byte obfuscated IAP record stream."""
    with open(bin_path, "rb") as f:
        payload = bytearray(f.read())

    # Record 0 (Header)
    rec0 = bytearray(RECORD_SIZE)
    rec0[:12] = MAGIC_HEADER
    rec0[12:128] = b"\x00" * 116
    rec0[128:132] = b"P407"
    rec0[132:136] = key

    # Pad payload to 128 bytes
    pad_rem = len(payload) % PAYLOAD_SIZE
    if pad_rem != 0:
        payload += b"\xFF" * (PAYLOAD_SIZE - pad_rem)

    total_records = len(payload) // PAYLOAD_SIZE
    iap_stream = bytearray(rec0)

    for k in range(1, total_records + 1):
        chunk = payload[(k - 1) * PAYLOAD_SIZE : k * PAYLOAD_SIZE]
        # Hiworld additive byte obfuscation: cipher[i] = (plain[i] + i + key[i%4]) & 0xFF
        obf = bytes((chunk[i] + i + key[i & 3]) & 0xFF for i in range(len(chunk)))
        rec = bytearray(obf)
        rec += k.to_bytes(2, "little")               # Sequence number (1..N)
        rec.append((k * 100) // total_records)       # Progress percentage (0..100)
        rec.append(1 if k == total_records else 0)   # Last block flag (1 = EOF)

        # 32-bit additive checksum
        csum = sum(int.from_bytes(rec[j * 4 : (j + 1) * 4], "little") for j in range(33)) & 0xFFFFFFFF
        rec += csum.to_bytes(4, "little")
        iap_stream += rec

    return bytes(iap_stream)


def update_firmware(port: str, baudrate: int, iap_data: bytes):
    """Executes the serial handshake, unlock, erase, and record streaming."""
    total_records = len(iap_data) // RECORD_SIZE
    print(f"\n[2/4] Connecting to CAN Box on {port} at {baudrate} baud...")
    ser = serial.Serial(port, baudrate, timeout=0.2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    print("[3/4] Sending Hiworld upgrade trigger (0xE0)...")
    # Send upgrade trigger command (5A A5 02 E0 00 00 E1)
    ser.write(CMD_UPGRADE_TRIGGER)
    ser.flush()

    # Step A: Wait for 'U' (0x55) prompt
    print("      Waiting for Bootloader 'U' prompt...", end="", flush=True)
    start_time = time.time()
    got_u = False
    while time.time() - start_time < 3.0:
        b = ser.read(1)
        if b == b"U":
            got_u = True
            break
        elif b == b"":
            # Periodic retry if app was still busy
            ser.write(CMD_UPGRADE_TRIGGER)
            ser.flush()

    if not got_u:
        print(" TIMEOUT!")
        print("[-] Error: Bootloader did not reply with 'U' prompt.")
        ser.close()
        sys.exit(1)
    print(" [OK]")

    # Step B: Send unlock password (19 78 02 17)
    print("      Unlocking flash and erasing partition...", end="", flush=True)
    ser.write(UNLOCK_PASSWORD)
    ser.flush()

    # Step C: Wait for 'BS' acknowledgement
    start_time = time.time()
    ack = bytearray()
    while time.time() - start_time < 6.0:
        b = ser.read(1)
        if b:
            ack += b
            if b"BS" in ack:
                break

    if b"BS" not in ack:
        print(" FAILED!")
        print(f"[-] Error: Bootloader failed to acknowledge flash erase. Received: {ack.hex(' ')}")
        ser.close()
        sys.exit(1)
    print(" [READY]")

    # Step D: Stream records
    print(f"\n[4/4] Streaming {total_records} firmware records ({len(iap_data):,} bytes)...")
    start_stream = time.time()

    for rec_idx in range(total_records):
        chunk = iap_data[rec_idx * RECORD_SIZE : (rec_idx + 1) * RECORD_SIZE]
        ser.write(chunk)
        ser.flush()

        resp = ser.read(1)

        if rec_idx == total_records - 1:
            # Last record expects 'E' (0x45)
            if resp == b"E":
                pct = 100
                bar = "=" * 30
                print(f"\r[{bar}] 100% (Record {rec_idx + 1}/{total_records}) [SUCCESS]", flush=True)
            else:
                print(f"\n[-] Final EOF record error! Expected 'E', got {resp}")
                ser.close()
                sys.exit(1)
        else:
            # Intermediate record expects 'S' (0x53)
            if resp != b"S":
                print(f"\n[-] Record {rec_idx + 1}/{total_records} rejected! Resp: {resp}")
                ser.close()
                sys.exit(1)

            pct = int((rec_idx + 1) * 100 / total_records)
            bar_len = int(30 * (rec_idx + 1) / total_records)
            bar = "=" * bar_len + "-" * (30 - bar_len)
            print(f"\r[{bar}] {pct:3d}% (Record {rec_idx + 1}/{total_records})", end="", flush=True)

    elapsed = time.time() - start_stream
    speed = len(iap_data) / elapsed if elapsed > 0 else 0
    print(f"\n[+] Firmware upload complete in {elapsed:.2f}s ({speed:.0f} bytes/sec)!")

    # Step E: Verify application boot & telemetry
    print("\nVerifying application boot & Hiworld telemetry...")
    time.sleep(0.5)
    telemetry = bytearray()
    start_time = time.time()
    while time.time() - start_time < 3.0:
        b = ser.read(ser.in_waiting or 1)
        if b:
            telemetry += b

    ser.close()

    if b"\x5A\xA5" in telemetry:
        print("[+] SUCCESS: CAN Box application booted and live Hiworld telemetry is streaming!\n")
    else:
        print("[*] Note: Firmware flashed cleanly. Waiting for car telemetry stream.\n")


def main():
    parser = argparse.ArgumentParser(description="Build and flash NUC131 firmware over Hiworld serial IAP.")
    parser.add_argument("-p", "--port", default=None, help="Serial port device (default: auto-detect)")
    parser.add_argument("-b", "--baud", type=int, default=38400, help="UART baud rate (default: 38400)")
    parser.add_argument("-e", "--env", default="raise_nuc131", help="PlatformIO environment name (default: raise_nuc131)")
    parser.add_argument("--no-build", action="store_true", help="Skip compilation and upload existing binary")
    args = parser.parse_args()

    port = args.port or find_serial_port()

    # Step 1: Build
    if not args.no_build:
        bin_path = build_firmware(args.env)
    else:
        bin_path = os.path.join(".pio", "build", args.env, "program.bin")
        if not os.path.exists(bin_path):
            print(f"[-] Binary '{bin_path}' does not exist! Run without --no-build.")
            sys.exit(1)

    # Step 2: Pack
    iap_data = pack_iap_data(bin_path)

    # Step 3: Flash
    update_firmware(port, args.baud, iap_data)


if __name__ == "__main__":
    main()

