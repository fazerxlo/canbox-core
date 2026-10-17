#!/usr/bin/env python3
"""
Simple RD4 Steering Wheel Control (SWC) CAN Injector & Station Monitor
========================================================================
Connects to SocketCAN (default: can0 / slcan) and:
1. Emits SWC CAN frames on CAN ID 0x21F:
   - SOURCE button pulse (cycles RD4 source: Tuner -> CD -> CDC -> AUX)
   - Thumbwheel / Rotary Scroll Wheel steps (triggers & scrolls station list)
   - Seek Next / Previous Track buttons
2. Listens to CAN bus and decodes RD4 feedback in real time:
   - CAN 0x165: Audio Source (Tuner, CD, CDC, AUX 1, AUX 2, Standby)
   - CAN 0x2A5: RDS Station Name (e.g. 'RMF FM', 'JEDYNKA')
   - CAN 0x225: Frequency (e.g. 102.5 MHz), Band (FM1/FM2/AM), Preset Slot
   - CAN 0x125: ISO-TP Station List (reconstructs station list on scroll)

Usage:
  # Interactive mode:
  ./tools/test_rd4_swc.py
  ./tools/test_rd4_swc.py -i can0
  ./tools/test_rd4_swc.py -i vcan0

  # Automatic slcan setup:
  ./tools/test_rd4_swc.py --setup-slcan --slcan-dev /dev/ttyUSB0

  # One-shot CLI actions:
  ./tools/test_rd4_swc.py --source               # Pulse SOURCE once
  ./tools/test_rd4_swc.py --scroll-up 3          # Scroll wheel UP 3 steps
  ./tools/test_rd4_swc.py --scroll-down 2        # Scroll wheel DOWN 2 steps
  ./tools/test_rd4_swc.py --listen               # Monitor RD4 state changes
"""

import argparse
import os
import select
import socket
import struct
import subprocess
import sys
import termios
import threading
import time
import tty

# SocketCAN frame packing format: uint32 can_id, uint8 dlc, 3 bytes padding, 8 bytes data
CAN_FRAME_FMT = "=IB3x8s"

# PSA CAN2004 IDs
CAN_ID_SWC_STALK_21F   = 0x21F  # COM2000 Steering Wheel Controls
CAN_ID_RD4_SOURCE_165  = 0x165  # RD4 Audio Source & Power Status
CAN_ID_RD4_STATUS_225  = 0x225  # RD4 Radio Frequency, Band & Preset
CAN_ID_RD4_STATION_2A5 = 0x2A5  # RD4 RDS Station Name (8 ASCII bytes)
CAN_ID_RD4_LIST_125    = 0x125  # RD4 Station List (ISO-TP stream)
CAN_ID_RD4_TEXT_0A4    = 0x0A4  # RD4 RDS Radio Text (ISO-TP stream)


def parse_can_frame(raw_bytes: bytes):
    """Unpacks Linux struct can_frame (16 bytes)."""
    can_id_raw, can_dlc, data = struct.unpack(CAN_FRAME_FMT, raw_bytes)
    can_id = can_id_raw & 0x7FF
    dlc = min(can_dlc, 8)
    return can_id, dlc, data[:dlc]


def pack_can_frame(can_id: int, data: bytes) -> bytes:
    """Packs CAN frame into Linux struct can_frame (16 bytes)."""
    dlc = len(data)
    padded_data = data.ljust(8, b"\x00")
    return struct.pack(CAN_FRAME_FMT, can_id & 0x7FF, dlc, padded_data)


class IsoTpReassembler:
    """Simple ISO-TP multi-frame reassembler for CAN 0x125 station list."""
    def __init__(self):
        self.active = False
        self.total_length = 0
        self.received_length = 0
        self.next_sn = 1
        self.buffer = bytearray()

    def feed(self, data: bytes) -> bytes or None:
        if len(data) < 2:
            return None

        # Check for list close frame: 01 00
        if len(data) == 2 and data[0] == 0x01 and data[1] == 0x00:
            self.active = False
            return None

        pci_type = data[0] & 0xF0

        # Single Frame (0x0N)
        if pci_type == 0x00:
            length = data[0] & 0x0F
            self.active = False
            if 0 < length <= (len(data) - 1):
                return bytes(data[1:1 + length])
            return None

        # First Frame (0x1N)
        if pci_type == 0x10:
            self.total_length = ((data[0] & 0x0F) << 8) | data[1]
            self.buffer = bytearray(data[2:])
            self.received_length = len(self.buffer)
            self.next_sn = 1
            self.active = True
            return None

        # Consecutive Frame (0x2N)
        if pci_type == 0x20 and self.active:
            sn = data[0] & 0x0F
            if sn == (self.next_sn & 0x0F):
                chunk = data[1:]
                rem = self.total_length - self.received_length
                if len(chunk) > rem:
                    chunk = chunk[:rem]
                self.buffer.extend(chunk)
                self.received_length += len(chunk)
                self.next_sn = (self.next_sn + 1) & 0x0F

                if self.received_length >= self.total_length:
                    self.active = False
                    return bytes(self.buffer[:self.total_length])
            else:
                self.active = False

        return None


class Rd4SwcTester:
    def __init__(self, iface: str = "can0"):
        self.iface = iface
        self.sock = None
        self.running = False
        self.thumbwheel_counter = 0x00
        self.lock = threading.Lock()
        self.isotp_125 = IsoTpReassembler()

        # Cached RD4 state
        self.current_source_code = None
        self.current_source_str = "UNKNOWN"
        self.current_station_name = ""
        self.current_band = ""
        self.current_freq_mhz = 0.0
        self.current_preset = 0

        self._init_socket()

    def _init_socket(self):
        try:
            self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.sock.bind((self.iface,))
        except Exception as e:
            print(f"\n[ERROR] Failed to bind to SocketCAN interface '{self.iface}': {e}")
            print(f"[HINT]  Ensure interface is up: 'sudo ip link set dev {self.iface} up'")
            print(f"[HINT]  For SLCAN: run with '--setup-slcan --slcan-dev /dev/ttyUSB0'")
            print(f"[HINT]  For virtual CAN: 'sudo modprobe vcan && sudo ip link add dev {self.iface} type vcan && sudo ip link set dev {self.iface} up'\n")
            sys.exit(1)

    def send_can(self, can_id: int, data: bytes):
        raw = pack_can_frame(can_id, data)
        with self.lock:
            try:
                self.sock.send(raw)
            except OSError as e:
                print(f"[WARN] Failed to send CAN frame {can_id:03X}#: {e}")

    # --------------------------------------------------------------------------
    # SWC Button & Thumbwheel Actions (CAN 0x21F)
    # --------------------------------------------------------------------------
    def pulse_source(self):
        """
        Pulses SOURCE button on steering stalk (CAN 0x21F).
        Verified bit in PSA CAN2004: Byte 0 Bit 1 (0x02).
        Cycles RD4 audio: Tuner -> CD -> CDC -> AUX 1 -> AUX 2.
        """
        counter = self.thumbwheel_counter
        # Press frame: Byte 0 = 0x02 (SRC)
        self.send_can(CAN_ID_SWC_STALK_21F, bytes([0x02, counter, 0x00]))
        print(f"  [TX -> CAN 0x21F] Pressed SOURCE (02 {counter:02X} 00)")
        time.sleep(0.060)  # 60 ms pulse duration
        # Release frame: Byte 0 = 0x00
        self.send_can(CAN_ID_SWC_STALK_21F, bytes([0x00, counter, 0x00]))
        print(f"  [TX -> CAN 0x21F] Released SOURCE (00 {counter:02X} 00)")

    def scroll_up(self, steps: int = 1):
        """
        Rotates thumbwheel UP (increments counter).
        Triggers RD4 station list and selects next station.
        """
        for _ in range(steps):
            self.thumbwheel_counter = (self.thumbwheel_counter + 1) & 0xFF
            cnt = self.thumbwheel_counter
            self.send_can(CAN_ID_SWC_STALK_21F, bytes([0x00, cnt, 0x00]))
            print(f"  [TX -> CAN 0x21F] Scroll UP -> Counter: 0x{cnt:02X} ({cnt})")
            time.sleep(0.080)

    def scroll_down(self, steps: int = 1):
        """
        Rotates thumbwheel DOWN (decrements counter).
        Triggers RD4 station list and selects previous station.
        """
        for _ in range(steps):
            self.thumbwheel_counter = (self.thumbwheel_counter - 1) & 0xFF
            cnt = self.thumbwheel_counter
            self.send_can(CAN_ID_SWC_STALK_21F, bytes([0x00, cnt, 0x00]))
            print(f"  [TX -> CAN 0x21F] Scroll DOWN -> Counter: 0x{cnt:02X} ({cnt})")
            time.sleep(0.080)

    def seek_next(self):
        """Pulses Seek Next / Track Up button (Byte 0 Bit 7 = 0x80)."""
        counter = self.thumbwheel_counter
        self.send_can(CAN_ID_SWC_STALK_21F, bytes([0x80, counter, 0x00]))
        print(f"  [TX -> CAN 0x21F] Pressed SEEK NEXT (80 {counter:02X} 00)")
        time.sleep(0.060)
        self.send_can(CAN_ID_SWC_STALK_21F, bytes([0x00, counter, 0x00]))
        print(f"  [TX -> CAN 0x21F] Released SEEK NEXT (00 {counter:02X} 00)")

    def seek_prev(self):
        """Pulses Seek Prev / Track Down button (Byte 0 Bit 6 = 0x40)."""
        counter = self.thumbwheel_counter
        self.send_can(CAN_ID_SWC_STALK_21F, bytes([0x40, counter, 0x00]))
        print(f"  [TX -> CAN 0x21F] Pressed SEEK PREV (40 {counter:02X} 00)")
        time.sleep(0.060)
        self.send_can(CAN_ID_SWC_STALK_21F, bytes([0x00, counter, 0x00]))
        print(f"  [TX -> CAN 0x21F] Released SEEK PREV (00 {counter:02X} 00)")

    def pulse_volume_up(self):
        """Pulses Volume Up (Byte 0 Bit 3 = 0x08)."""
        counter = self.thumbwheel_counter
        self.send_can(CAN_ID_SWC_STALK_21F, bytes([0x08, counter, 0x00]))
        time.sleep(0.060)
        self.send_can(CAN_ID_SWC_STALK_21F, bytes([0x00, counter, 0x00]))
        print(f"  [TX -> CAN 0x21F] Volume +")

    def pulse_volume_down(self):
        """Pulses Volume Down (Byte 0 Bit 2 = 0x04)."""
        counter = self.thumbwheel_counter
        self.send_can(CAN_ID_SWC_STALK_21F, bytes([0x04, counter, 0x00]))
        time.sleep(0.060)
        self.send_can(CAN_ID_SWC_STALK_21F, bytes([0x00, counter, 0x00]))
        print(f"  [TX -> CAN 0x21F] Volume -")

    def pulse_mute(self):
        """Pulses Mute (Chorded Vol+ & Vol-: 0x08 | 0x04 = 0x0C)."""
        counter = self.thumbwheel_counter
        self.send_can(CAN_ID_SWC_STALK_21F, bytes([0x0C, counter, 0x00]))
        time.sleep(0.060)
        self.send_can(CAN_ID_SWC_STALK_21F, bytes([0x00, counter, 0x00]))
        print(f"  [TX -> CAN 0x21F] MUTE toggle")

    # --------------------------------------------------------------------------
    # RD4 CAN Receiver & Decoders
    # --------------------------------------------------------------------------
    def _decode_rd4_source_165(self, data: bytes):
        """Decodes CAN 0x165 (ETAT_AUTORADIO): source mode and power status."""
        if len(data) < 3:
            return
        src_raw = (data[2] >> 4) & 0x0F
        src_map = {
            0x1: "Tuner (FM/AM)",
            0x2: "Internal CD",
            0x3: "CD Changer (CDC)",
            0x4: "Line In (AUX 1)",
            0x5: "Line In (AUX 2)",
            0x7: "Bluetooth Audio",
        }
        source_str = src_map.get(src_raw, f"Off / Standby (0x{src_raw:X})")
        if self.current_source_code != src_raw:
            self.current_source_code = src_raw
            self.current_source_str = source_str
            ts = time.strftime("%H:%M:%S")
            print(f"\n[{ts}] >>> RD4 AUDIO SOURCE CHANGED: [ {source_str} ] <<<")

    def _decode_rd4_status_225(self, data: bytes):
        """Decodes CAN 0x225: Band, Frequency, and Preset slot matching peugeot_407.c."""
        if len(data) < 5:
            return

        # Preset slot (byte 1: 0=manual, 1..6)
        preset = (data[1] >> 4) & 0x0F if data[1] >= 0x10 else (data[1] & 0x0F)

        # Band decoding from byte 2
        raw_band = data[2]
        band_str = "FM1"
        is_fm = True
        if raw_band in (0x10, 0x90):
            band_str = "FM1"
        elif raw_band in (0x20, 0xA0):
            band_str = "FM2"
        elif raw_band in (0x40, 0xC0):
            band_str = "FM-AST"
        elif raw_band in (0x50, 0xD0):
            band_str = "AM"
            is_fm = False

        # Frequency (bytes 3 and 4)
        raw_freq = (data[3] << 8) | data[4]
        freq_mhz = 0.0
        if is_fm and raw_freq > 0:
            freq_0_1mhz = (raw_freq // 2) + 500
            freq_mhz = freq_0_1mhz / 10.0
        elif not is_fm and raw_freq > 0:
            freq_mhz = float(raw_freq)  # kHz direct for AM

        if band_str != self.current_band or preset != self.current_preset or abs(freq_mhz - self.current_freq_mhz) > 0.05:
            self.current_band = band_str
            self.current_preset = preset
            self.current_freq_mhz = freq_mhz
            ts = time.strftime("%H:%M:%S")
            preset_str = f"Preset P{preset}" if (1 <= preset <= 6) else "Manual"
            freq_str = f"{freq_mhz:.1f} MHz" if is_fm else f"{freq_mhz:.0f} kHz"
            print(f"[{ts}] RD4 Tuner: Band: {band_str} | {preset_str} | {freq_str}")

    def _decode_rd4_station_2a5(self, data: bytes):
        """Decodes CAN 0x2A5: 8-byte ASCII RDS Station Name."""
        try:
            # Clean non-printable characters
            name = "".join(chr(b) if 0x20 <= b <= 0x7E else " " for b in data).strip()
        except Exception:
            name = ""

        if name and name != self.current_station_name:
            self.current_station_name = name
            ts = time.strftime("%H:%M:%S")
            print(f"\n[{ts}] *** RD4 ACTIVE STATION: [ {name} ] ***\n")

    def _decode_rd4_station_list_125(self, data: bytes):
        """Decodes CAN 0x125: ISO-TP multi-frame radio station list."""
        reassembled = self.isotp_125.feed(data)
        if reassembled and len(reassembled) >= 13:
            raw_band = reassembled[0]
            band_name = "FM1" if raw_band in (0x10, 0x90) else ("FM2" if raw_band in (0x20, 0xA0) else "Radio")
            num_stations = (len(reassembled) - 4) // 9
            if num_stations > 12:
                num_stations = 12

            ts = time.strftime("%H:%M:%S")
            print(f"\n[{ts}] ===== RD4 STATION LIST RECEIVED ({band_name}, {num_stations} stations) =====")
            for i in range(num_stations):
                entry_offset = 4 + (i * 9)
                if entry_offset + 8 <= len(reassembled):
                    st_bytes = reassembled[entry_offset:entry_offset + 8]
                    st_name = "".join(chr(b) if 0x20 <= b <= 0x7E else " " for b in st_bytes).strip()
                    marker = " -> [CURRENT]" if st_name and st_name == self.current_station_name else ""
                    print(f"   [{i + 1:2d}] {st_name:<10s}{marker}")
            print("=================================================================\n")

    def process_incoming_frame(self, can_id: int, dlc: int, data: bytes):
        if can_id == CAN_ID_RD4_SOURCE_165:
            self._decode_rd4_source_165(data)
        elif can_id == CAN_ID_RD4_STATION_2A5:
            self._decode_rd4_station_2a5(data)
        elif can_id == CAN_ID_RD4_STATUS_225:
            self._decode_rd4_status_225(data)
        elif can_id == CAN_ID_RD4_LIST_125:
            self._decode_rd4_station_list_125(data)

    def listen_loop(self):
        """Background thread reading CAN frames."""
        while self.running:
            r, _, _ = select.select([self.sock], [], [], 0.05)
            if r:
                try:
                    raw = self.sock.recv(16)
                    if len(raw) == 16:
                        can_id, dlc, data = parse_can_frame(raw)
                        self.process_incoming_frame(can_id, dlc, data)
                except OSError:
                    break


def setup_slcan_interface(dev: str, iface: str = "can0", speed: str = "s6", uart_baud: int = 1000000):
    """Brings up SocketCAN interface over serial USB adapter (slcand)."""
    print(f"[*] Attaching SLCAN adapter on {dev} -> {iface} (speed: {speed}, baud: {uart_baud})...")
    try:
        subprocess.run(["sudo", "slcan_attach", "-f", f"-{speed.lstrip('-')}", "-o", dev], check=True)
        time.sleep(0.2)
        subprocess.run(["sudo", "slcand", "-S", str(uart_baud), dev, iface], check=True)
        time.sleep(0.3)
        subprocess.run(["sudo", "ip", "link", "set", "dev", iface, "up"], check=True)
        print(f"[OK] CAN interface '{iface}' is UP.")
        return True
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] SLCAN setup failed: {e}")
        return False


def get_key_nonblocking():
    """Reads a single keypress from stdin in raw mode."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], 0.05)
        if r:
            ch1 = sys.stdin.read(1)
            if ch1 == '\x1b':  # Escape sequence (e.g. arrow keys)
                r2, _, _ = select.select([sys.stdin], [], [], 0.02)
                if r2:
                    ch2 = sys.stdin.read(1)
                    if ch2 == '[':
                        ch3 = sys.stdin.read(1)
                        if ch3 == 'A': return 'UP'
                        if ch3 == 'B': return 'DOWN'
                        if ch3 == 'C': return 'RIGHT'
                        if ch3 == 'D': return 'LEFT'
            return ch1
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def print_banner(iface: str):
    print("=" * 72)
    print("    PEUGEOT RD4 STEERING STALK (SWC) CAN CONTROLLER & STATION CHECKER")
    print("=" * 72)
    print(f"Interface: {iface} (PSA CAN2004 Comfort Bus)")
    print("Commands:")
    print("  [s]          : Pulse SOURCE (Cycle: Tuner -> CD -> CDC -> AUX)")
    print("  [u] / [UP]   : Scroll Wheel UP   (Thumbwheel counter +1, triggers list)")
    print("  [d] / [DOWN] : Scroll Wheel DOWN (Thumbwheel counter -1, triggers list)")
    print("  [n] / [RIGHT]: Seek Next Station / Track")
    print("  [p] / [LEFT] : Seek Prev Station / Track")
    print("  [+] / [-]    : Volume Up / Down")
    print("  [m]          : Mute toggle")
    print("  [q] / [Ctrl+C]: Quit")
    print("=" * 72)
    print("[*] Listening for RD4 station changes (0x165, 0x2A5, 0x225, 0x125)...")
    print("-" * 72)


def main():
    parser = argparse.ArgumentParser(
        description="Peugeot RD4 SWC (CAN 0x21F) Stalk Injector & Station Change Checker"
    )
    parser.add_argument(
        "--iface", "-i",
        default="can0",
        help="SocketCAN interface name (default: can0)"
    )
    parser.add_argument(
        "--setup-slcan",
        action="store_true",
        help="Automatically configure and bring up SLCAN interface"
    )
    parser.add_argument(
        "--slcan-dev",
        default="/dev/ttyUSB0",
        help="Serial port for SLCAN adapter (default: /dev/ttyUSB0)"
    )
    parser.add_argument(
        "--source",
        action="store_true",
        help="Pulse SOURCE once and exit"
    )
    parser.add_argument(
        "--scroll-up",
        type=int,
        metavar="N",
        help="Scroll wheel UP N steps and exit"
    )
    parser.add_argument(
        "--scroll-down",
        type=int,
        metavar="N",
        help="Scroll wheel DOWN N steps and exit"
    )
    parser.add_argument(
        "--seek-next",
        action="store_true",
        help="Seek Next and exit"
    )
    parser.add_argument(
        "--seek-prev",
        action="store_true",
        help="Seek Prev and exit"
    )
    parser.add_argument(
        "--listen",
        action="store_true",
        help="Listen only mode: monitor and decode RD4 frames"
    )
    args = parser.parse_args()

    # Automatic SLCAN setup if requested
    if args.setup_slcan:
        setup_slcan_interface(args.slcan_dev, iface=args.iface)

    tester = Rd4SwcTester(iface=args.iface)
    tester.running = True

    # Start CAN listener thread
    listen_th = threading.Thread(target=tester.listen_loop, daemon=True)
    listen_th.start()

    # One-shot command execution
    if args.source:
        print("[*] Sending SOURCE pulse on CAN 0x21F...")
        tester.pulse_source()
        time.sleep(0.5)
        tester.running = False
        return

    if args.scroll_up is not None:
        steps = max(1, args.scroll_up)
        print(f"[*] Sending {steps} Scroll UP step(s) on CAN 0x21F...")
        tester.scroll_up(steps)
        time.sleep(0.5)
        tester.running = False
        return

    if args.scroll_down is not None:
        steps = max(1, args.scroll_down)
        print(f"[*] Sending {steps} Scroll DOWN step(s) on CAN 0x21F...")
        tester.scroll_down(steps)
        time.sleep(0.5)
        tester.running = False
        return

    if args.seek_next:
        print("[*] Sending SEEK NEXT on CAN 0x21F...")
        tester.seek_next()
        time.sleep(0.5)
        tester.running = False
        return

    if args.seek_prev:
        print("[*] Sending SEEK PREV on CAN 0x21F...")
        tester.seek_prev()
        time.sleep(0.5)
        tester.running = False
        return

    if args.listen:
        print(f"[*] Monitoring RD4 radio on '{args.iface}' (Press Ctrl+C to stop)...")
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n[*] Stopped.")
        finally:
            tester.running = False
        return

    # Interactive UI mode
    print_banner(args.iface)

    try:
        while True:
            key = get_key_nonblocking()
            if key is None:
                time.sleep(0.02)
                continue

            if key in ('q', 'Q', '\x03'):  # 'q' or Ctrl+C
                print("\n[*] Exiting...")
                break
            elif key in ('s', 'S'):
                tester.pulse_source()
            elif key in ('u', 'U', 'k', 'UP'):
                tester.scroll_up(1)
            elif key in ('d', 'D', 'j', 'DOWN'):
                tester.scroll_down(1)
            elif key in ('n', 'N', 'RIGHT'):
                tester.seek_next()
            elif key in ('p', 'P', 'LEFT'):
                tester.seek_prev()
            elif key == '+':
                tester.pulse_volume_up()
            elif key == '-':
                tester.pulse_volume_down()
            elif key in ('m', 'M'):
                tester.pulse_mute()

    except KeyboardInterrupt:
        print("\n[*] Interrupted.")
    finally:
        tester.running = False
        print("[*] Done.")


if __name__ == "__main__":
    main()
