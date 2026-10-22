#!/usr/bin/env python3
"""
OpenCanbox Core - Generic BSI & SLCAN Workbench Automated Test Runner
====================================================================
Facilitates automated and interactive testing of ANY OpenCanbox Core subsystem
against a physical vehicle bench comprising:
  - Peugeot 407 BSI (Body Systems Interface)
  - RD4 Radio / Climate / Comfort Bus ECUs
  - USB-to-CAN SLCAN adapter (can0 @ 125 kbps)
  - OpenCanbox Core firmware (native Linux daemon via PTY /tmp/ttyCanbox or physical MCU UART)

Architecture:
  This harness uses a modular Scenario Registry pattern. New subsystems
  (Climate, Trip Computer, Doors, Alerts, Radio, SWC) can be registered as
  WorkbenchScenario subclasses with minimal code.

Supported Scenarios:
  - config  : Car Personalization Options (0x260/0x361 <-> 0x76, 0x79, 0x7B, 0x7D)
  - climate : Dual-Zone HVAC Control (0x1E3/0x1D0 <-> 0x31, 0x3B)
  - trip    : Trip Computer & Resets (0x221 <-> 0x13, 0x14, 0x15, 0x1B)
  - doors   : Door & Trunk Status (0x220/0x036 <-> 0x12)
  - all     : Executes all registered scenarios in sequence

Usage:
  ./tools/test_bsi_workbench.py                     # Run default (config) test suite
  ./tools/test_bsi_workbench.py --scenario all      # Run all registered scenarios
  ./tools/test_bsi_workbench.py --scenario climate  # Run Climate/HVAC scenario
  ./tools/test_bsi_workbench.py --scenario trip     # Run Trip Computer scenario
  ./tools/test_bsi_workbench.py --sniff             # Sniff and decode live BSI CAN status
  ./tools/test_bsi_workbench.py --hu /dev/ttyUSB0   # Test physical hardware MCU UART
"""

import argparse
import glob
import os
import select
import serial
import socket
import struct
import subprocess
import sys
import time
from typing import Dict, List, Optional, Tuple

# SocketCAN constants
CAN_RAW = 1
CAN_FRAME_FMT = "=IB3x8s"

# Peugeot 407 / PSA CAN2004 Message IDs
CAN_ID_BSI_IGNITION_036 = 0x036
CAN_ID_BSI_SLOW_0F6     = 0x0F6
CAN_ID_ALERT_JOURNAL_120 = 0x120
CAN_ID_BSI_DOWNLINK_15B = 0x15B
CAN_ID_RD4_SOURCE_165   = 0x165
CAN_ID_CLIMATE_PANEL_1D0 = 0x1D0
CAN_ID_CLIMATE_EMF_1E3   = 0x1E3
CAN_ID_DOORS_BODY_220   = 0x220
CAN_ID_TRIP_INSTANT_221 = 0x221
CAN_ID_BSI_CONFIG_260   = 0x260
CAN_ID_BSI_PERSO_361    = 0x361

# Hiworld Framing Constants
HIWORLD_SYNC_1 = 0x5A
HIWORLD_SYNC_2 = 0xA5

HIWORLD_CMD_WHEEL_KEYS   = 0x11
HIWORLD_CMD_DOORS        = 0x12
HIWORLD_CMD_TRIP_INSTANT = 0x13
HIWORLD_CMD_TRIP1        = 0x14
HIWORLD_CMD_TRIP2        = 0x15
HIWORLD_CMD_TRIP_RESET   = 0x1B
HIWORLD_CMD_AC_INFO      = 0x31
HIWORLD_CMD_AC_SET       = 0x3B
HIWORLD_CMD_CENTRAL_STATE1 = 0x76
HIWORLD_CMD_CENTRAL_STATE2 = 0x79
HIWORLD_CMD_SETTING1       = 0x7B
HIWORLD_CMD_SETTING2       = 0x7D
HIWORLD_CMD_HEARTBEAT      = 0xFF

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
NATIVE_BIN_PATH = os.path.join(PROJECT_ROOT, ".pio", "build", "native_test", "program")


def calc_hiworld_cs(cmd: int, payload: bytes) -> int:
    """Calculates Hiworld checksum: ((len + cmd + sum(payload)) - 1) & 0xFF."""
    return ((len(payload) + cmd + sum(payload)) - 1) & 0xFF


def build_hiworld_packet(cmd: int, payload: bytes) -> bytes:
    """Encapsulates payload into 5A A5 <len> <cmd> <payload...> <cs>."""
    cs = calc_hiworld_cs(cmd, payload)
    return bytes([HIWORLD_SYNC_1, HIWORLD_SYNC_2, len(payload), cmd]) + bytes(payload) + bytes([cs])


def check_and_bringup_slcan(iface: str = "can0", dev: str = None) -> bool:
    """Ensures can0 is UP. If down, resets duplicate daemons and binds adapter."""
    try:
        res = subprocess.run(["ip", "link", "show", iface], capture_output=True, text=True)
        if res.returncode == 0 and "UP" in res.stdout:
            return True
    except Exception:
        pass

    print(f"[*] CAN interface '{iface}' is not UP. Attempting SLCAN bring-up...")

    if not dev:
        candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        for c in candidates:
            try:
                out = subprocess.run(["udevadm", "info", "-q", "property", "-n", c],
                                     capture_output=True, text=True).stdout
                if "1a86_USB2.0-Serial" in out or "CH340" in out:
                    dev = c
                    break
            except Exception:
                pass
        if not dev and candidates:
            dev = candidates[0]

    if not dev:
        print("[ERROR] No USB-to-serial adapter found for SLCAN!")
        return False

    print(f"[*] Binding SLCAN adapter {dev} -> {iface} (125 kbps)...")
    try:
        subprocess.run(["sudo", "-n", "killall", "slcand"], stderr=subprocess.DEVNULL)
        time.sleep(0.3)
        subprocess.run(["sudo", "-n", "slcan_attach", "-f", "-s6", "-o", dev], check=True)
        subprocess.run(["sudo", "-n", "slcand", "-S", "1000000", dev, iface], check=True)
        subprocess.run(["sudo", "-n", "ip", "link", "set", "dev", iface, "up"], check=True)
        time.sleep(0.3)

        res = subprocess.run(["ip", "link", "show", iface], capture_output=True, text=True)
        if "UP" in res.stdout:
            print(f"[OK] {iface} successfully initialized on {dev}.")
            return True
    except Exception as e:
        print(f"[ERROR] Failed to bring up SLCAN on {dev}: {e}")

    return False


class BsiWorkbenchHarness:
    """Core test harness coordinating SocketCAN and serial UART streams."""

    def __init__(self, can_iface: str = "can0", hu_port: str = None, use_native_daemon: bool = True):
        self.can_iface = can_iface
        self.hu_port = hu_port
        self.use_native_daemon = use_native_daemon
        self.proc = None
        self.can_sock = None
        self.uart = None

    def start(self) -> bool:
        try:
            self.can_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, CAN_RAW)
            self.can_sock.bind((self.can_iface,))
            self.can_sock.setblocking(False)
        except Exception as e:
            print(f"[ERROR] Cannot bind to SocketCAN '{self.can_iface}': {e}")
            return False

        if self.use_native_daemon:
            if not os.path.exists(NATIVE_BIN_PATH):
                print("[*] Compiling native_test OpenCanbox binary...")
                pio = os.path.expanduser("~/.platformio/penv/bin/pio")
                subprocess.run([pio, "run", "-s", "-e", "native_test"], cwd=PROJECT_ROOT, check=True)

            if os.path.exists("/tmp/ttyCanbox"):
                try:
                    os.unlink("/tmp/ttyCanbox")
                except Exception:
                    pass

            env = os.environ.copy()
            env["CANBOX_CAN_IFACE"] = self.can_iface
            env["CANBOX_HU_PROTOCOL"] = "hiworld"
            env["CANBOX_VEHICLE_PROFILE"] = "psa"

            self.proc = subprocess.Popen(
                [NATIVE_BIN_PATH],
                cwd=PROJECT_ROOT,
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            time.sleep(0.4)

            for _ in range(20):
                if os.path.exists("/tmp/ttyCanbox"):
                    break
                time.sleep(0.1)

            if not os.path.exists("/tmp/ttyCanbox"):
                print("[ERROR] canbox-core failed to create /tmp/ttyCanbox!")
                self.stop()
                return False

            self.hu_port = "/tmp/ttyCanbox"

        try:
            self.uart = serial.Serial(self.hu_port, 38400, timeout=0.05)
        except Exception as e:
            print(f"[ERROR] Cannot open HU serial port '{self.hu_port}': {e}")
            self.stop()
            return False

        return True

    def stop(self):
        if self.uart:
            try:
                self.uart.close()
            except Exception:
                pass
            self.uart = None
        if self.can_sock:
            try:
                self.can_sock.close()
            except Exception:
                pass
            self.can_sock = None
        if self.proc:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=1.0)
            except Exception:
                try:
                    self.proc.kill()
                except Exception:
                    pass
            self.proc = None

    def read_can_frame(self, target_id: int, timeout_sec: float = 0.8, predicate=None) -> Optional[List[int]]:
        """Waits for a specific CAN ID on the bus matching an optional predicate."""
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            r, _, _ = select.select([self.can_sock], [], [], 0.05)
            if r:
                raw = self.can_sock.recv(16)
                cid, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
                if cid == target_id:
                    payload = list(data[:dlc])
                    if predicate is None or predicate(payload):
                        return payload
        return None

    def send_can_frame(self, can_id: int, data: bytes):
        """Transmits a standard 11-bit CAN frame onto the bus."""
        raw = struct.pack(CAN_FRAME_FMT, can_id, len(data), bytes(data))
        self.can_sock.send(raw)

    def drain_can(self, duration: float = 0.05):
        """Discards pending CAN frames from the socket queue."""
        t0 = time.time()
        while time.time() - t0 < duration:
            r, _, _ = select.select([self.can_sock], [], [], 0.01)
            if r:
                self.can_sock.recv(16)

    def read_uart_packets(self, duration: float = 0.4) -> List[Tuple[int, List[int], bytes]]:
        """Reads and frames Hiworld packets (Cmd, Payload, Raw) from UART."""
        packets = []
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < duration:
            b = self.uart.read(100)
            if b:
                buf.extend(b)
                while len(buf) >= 4:
                    if buf[0] == HIWORLD_SYNC_1 and buf[1] == HIWORLD_SYNC_2:
                        length = buf[2]
                        total = length + 5
                        if len(buf) >= total:
                            pkt = buf[:total]
                            buf = buf[total:]
                            packets.append((pkt[3], list(pkt[4:-1]), pkt))
                            continue
                        else:
                            break
                    else:
                        buf.pop(0)
            time.sleep(0.01)
        return packets

    def send_hu_packet(self, cmd: int, payload: bytes) -> bytes:
        """Encapsulates and sends a Hiworld packet to canbox-core over UART."""
        pkt = build_hiworld_packet(cmd, payload)
        self.uart.reset_input_buffer()
        self.uart.write(pkt)
        return pkt


# ==============================================================================
# Base Scenario Interface
# ==============================================================================
class BaseWorkbenchScenario:
    """Base class for any workbench subsystem test scenario."""

    name: str = "base"
    description: str = "Base Scenario"

    def run(self, harness: BsiWorkbenchHarness, filter_key: str = None) -> bool:
        raise NotImplementedError


# ==============================================================================
# Scenario 1: Vehicle Personalization & Car Configuration Options
# ==============================================================================
class CarConfigScenario(BaseWorkbenchScenario):
    name = "config"
    description = "BSI Personalization Options (0x260/0x361 <-> 0x76, 0x79, 0x7B, 0x7D)"

    ITEMS = [
        {
            "key": "wiper", "name": "Rear Wiper in Reverse",
            "setting_id": 0x01, "cmd_id": HIWORLD_CMD_SETTING1,
            "can_byte": 5, "can_bit": 0x10, "inj_byte": 5, "inj_bit": 0x10, "hu_val_enable": 0x01,
            "chk_cmd": HIWORLD_CMD_CENTRAL_STATE1, "chk_byte": 1, "chk_bit": 0x80
        },
        {
            "key": "selective_unlock", "name": "Selective Door Unlocking",
            "setting_id": 0x04, "cmd_id": HIWORLD_CMD_SETTING1,
            "can_byte": 4, "can_bit": 0x04, "inj_byte": 4, "inj_bit": 0x04, "hu_val_enable": 0x01,
            "chk_cmd": HIWORLD_CMD_CENTRAL_STATE1, "chk_byte": 1, "chk_bit": 0x08
        },
        {
            "key": "drl", "name": "Daytime Running Lights (DRL)",
            "setting_id": 0x05, "cmd_id": HIWORLD_CMD_SETTING1,
            "can_byte": 3, "can_bit": 0x40, "inj_byte": 3, "inj_bit": 0x40, "hu_val_enable": 0x01,
            "chk_cmd": HIWORLD_CMD_CENTRAL_STATE1, "chk_byte": 1, "chk_bit": 0x04
        },
        {
            "key": "follow_me", "name": "Follow-Me-Home Delay (30s)",
            "setting_id": 0x06, "cmd_id": HIWORLD_CMD_SETTING1,
            "can_byte": 2, "can_bit": 0x20, "inj_byte": 2, "inj_bit": 0x20, "hu_val_enable": 0x02,
            "chk_cmd": HIWORLD_CMD_CENTRAL_STATE1, "chk_byte": 1, "chk_bit": 0x02
        },
        {
            "key": "radar", "name": "Parking Radar (AAS) Toggle",
            "setting_id": 0x0B, "cmd_id": HIWORLD_CMD_SETTING1,
            "can_byte": 5, "can_bit": 0x40, "inj_byte": 5, "inj_bit": 0x40, "hu_val_enable": 0x01,
            "chk_cmd": HIWORLD_CMD_CENTRAL_STATE1, "chk_byte": 0, "chk_bit": 0x40
        },
        {
            "key": "auto_lock", "name": "Auto Door Lock while Driving",
            "setting_id": 0x0C, "cmd_id": HIWORLD_CMD_SETTING1,
            "can_byte": 4, "can_bit": 0x08, "inj_byte": 4, "inj_bit": 0x08, "hu_val_enable": 0x01,
            "chk_cmd": HIWORLD_CMD_CENTRAL_STATE1, "chk_byte": 1, "chk_bit": 0x20
        },
        {
            "key": "boot", "name": "Motorized Tailgate / Remote Boot",
            "setting_id": 0x01, "cmd_id": HIWORLD_CMD_SETTING2,
            "can_byte": 5, "can_bit": 0x80, "inj_byte": 5, "inj_bit": 0x80, "hu_val_enable": 0x01,
            "chk_cmd": HIWORLD_CMD_CENTRAL_STATE2, "chk_byte": 0, "chk_bit": 0x80
        },
        {
            "key": "afs", "name": "Directional Headlights (AFS)",
            "setting_id": 0x02, "cmd_id": HIWORLD_CMD_SETTING2,
            "can_byte": 2, "can_bit": 0x80, "inj_byte": 2, "inj_bit": 0x80, "hu_val_enable": 0x01,
            "chk_cmd": HIWORLD_CMD_CENTRAL_STATE2, "chk_byte": 0, "chk_bit": 0x40
        },
    ]

    def run(self, harness: BsiWorkbenchHarness, filter_key: str = None) -> bool:
        print("\n" + "=" * 72)
        print("SCENARIO: BSI PERSONALIZATION & CAR CONFIGURATION OPTIONS")
        print("=" * 72)

        # Baseline: wait for live BSI 0x260 Profile 1 if physical bench is connected,
        # otherwise use synthetic Profile 1 baseline.
        base_can = harness.read_can_frame(
            CAN_ID_BSI_CONFIG_260,
            timeout_sec=2.0,
            predicate=lambda d: len(d) >= 8 and d[0] == 0x01
        )
        if base_can:
            print(f"[*] Live BSI 0x260 Baseline (Profile 1): {' '.join([f'{x:02X}' for x in base_can])}")
        else:
            print("[*] No live 0x260 on bus; using synthetic Profile 1 baseline")
            base_can = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        # Broadcast 0x361 equipment mask so menu items are enabled in state cache
        harness.send_can_frame(CAN_ID_BSI_PERSO_361, bytes([0x01, 0x01, 0x91, 0x40, 0x30, 0x10]))
        time.sleep(0.05)

        test_items = self.ITEMS
        if filter_key:
            test_items = [it for it in self.ITEMS if filter_key.lower() in it["key"] or filter_key.lower() in it["name"].lower()]
            if not test_items:
                print(f"[ERROR] Unknown setting filter '{filter_key}'")
                return False

        all_passed = True

        # Part A: CAN -> Core -> HU
        print("\n--- Part A: CAN Bus State Change (0x260) -> canbox-core -> HU Serial Telemetry ---")
        for item in test_items:
            name = item["name"]
            curr_can = harness.read_can_frame(
                CAN_ID_BSI_CONFIG_260,
                timeout_sec=0.6,
                predicate=lambda d: len(d) >= 8 and d[0] == 0x01
            ) or list(base_can)
            test_payload = list(curr_can)
            test_payload[0] = 0x01  # Profile 1

            if item["key"] == "follow_me":
                if test_payload[2] & 0x20:
                    test_payload[2] &= ~(0x20 | 0x0F)
                    target_on = False
                else:
                    test_payload[2] |= 0x22  # Follow-me ON (30s)
                    target_on = True
            else:
                test_payload[item["can_byte"]] ^= item["can_bit"]
                target_on = (test_payload[item["can_byte"]] & item["can_bit"]) != 0

            harness.send_can_frame(CAN_ID_BSI_CONFIG_260, bytes(test_payload))
            pkts = harness.read_uart_packets(duration=0.3)
            relevant = [p for p in pkts if p[0] == item["chk_cmd"]]

            uart_ok = False
            b_val = None
            if relevant:
                b_val = relevant[-1][1][item["chk_byte"]]
                uart_ok = ((b_val & item["chk_bit"]) != 0) == target_on

            flag = "[PASS]" if uart_ok else "[FAIL]"
            if not uart_ok:
                all_passed = False
            b_str = f"0x{b_val:02X}" if b_val is not None else "None"
            print(f"  {flag} CAN -> HU: {name:32s} (Byte[{item['chk_byte']}]={b_str} match: {uart_ok})")
            time.sleep(0.08)

        # Part B: HU -> Core -> CAN
        print("\n--- Part B: HU Downlink Command -> canbox-core -> CAN Injection (0x15B) & Uplink ---")
        for item in test_items:
            name = item["name"]
            val = item["hu_val_enable"]
            payload = bytes([item["setting_id"], val])
            harness.drain_can(0.05)
            harness.send_hu_packet(item["cmd_id"], payload)

            rx_pkts = harness.read_uart_packets(duration=0.25)
            ack_ok = any(p[0] == HIWORLD_CMD_HEARTBEAT and len(p[1]) >= 1 and p[1][0] == item["cmd_id"] for p in rx_pkts)

            injected_can = harness.read_can_frame(CAN_ID_BSI_DOWNLINK_15B, timeout_sec=0.25)
            can_ok = False
            if injected_can and len(injected_can) >= 8:
                inj_byte = item["inj_byte"]
                inj_bit = item["inj_bit"]
                can_ok = (injected_can[0] == 0x01) and ((injected_can[inj_byte] & inj_bit) != 0)

            tel_pkt = [p for p in rx_pkts if p[0] == item["chk_cmd"]]
            tel_ok = False
            if tel_pkt:
                b_val = tel_pkt[0][1][item["chk_byte"]]
                tel_ok = (b_val & item["chk_bit"]) != 0

            pass_all = ack_ok and can_ok and tel_ok
            flag = "[PASS]" if pass_all else "[FAIL]"
            if not pass_all:
                all_passed = False
            print(f"  {flag} HU -> CAN: {name:32s} (ACK={ack_ok}, CAN_Inj(0x15B)={can_ok}, HU_Echo={tel_ok})")
            time.sleep(0.08)

        return all_passed


# ==============================================================================
# Scenario 2: Dual-Zone Climate Control (HVAC)
# ==============================================================================
class ClimateScenario(BaseWorkbenchScenario):
    name = "climate"
    description = "Dual-Zone HVAC Status & Downlink Overrides (0x1E3/0x1D0 <-> 0x31, 0x3B)"

    def run(self, harness: BsiWorkbenchHarness, filter_key: str = None) -> bool:
        print("\n" + "=" * 72)
        print("SCENARIO: DUAL-ZONE CLIMATE CONTROL (HVAC)")
        print("=" * 72)

        all_passed = True

        # Test A1: CAN 0x1E3 (EMF Status) -> HU 0x31 (AC Telemetry)
        print("\n--- Test A1: CAN 0x1E3 Telemetry -> canbox-core -> HU 0x31 ---")
        # Frame: Byte 0: DUAL=1 (bit 0), AC_ON (bit 6=0), Byte 2: Left 21.5C (0x0F), Byte 3: Right 22.0C (0x10)
        can_1e3 = bytes([0x01, 0x00, 0x0F, 0x10, 0x20, 0x20, 0x00])
        harness.send_can_frame(CAN_ID_CLIMATE_EMF_1E3, can_1e3)

        pkts = harness.read_uart_packets(duration=0.3)
        ac_pkts = [p for p in pkts if p[0] == HIWORLD_CMD_AC_INFO]

        ac_uplink_ok = False
        if ac_pkts:
            payload = ac_pkts[-1][1]
            # Hiworld 0x31: Byte 0 Bit 2 (0x04) = Dual mode active
            dual_bit = (payload[0] & 0x04) != 0 if len(payload) > 0 else False
            ac_uplink_ok = dual_bit

        flag = "[PASS]" if ac_uplink_ok else "[FAIL]"
        if not ac_uplink_ok:
            all_passed = False
        print(f"  {flag} CAN 0x1E3 -> HU 0x31 Dual Zone mode decoding (Dual active: {ac_uplink_ok})")

        # Test B1: HU Downlink 0x3B (Dual/Mono Sync Toggle) -> CAN 0x1D0 Injection
        print("\n--- Test B1: HU Downlink 0x3B (Dual Sync) -> canbox-core -> CAN 0x1D0 ---")
        # Hiworld Cmd 0x3B Setting 0x0F = Toggle Dual Sync
        harness.drain_can(0.05)
        harness.send_hu_packet(HIWORLD_CMD_AC_SET, bytes([0x0F, 0x01]))

        # Check ACK
        rx_pkts = harness.read_uart_packets(duration=0.25)
        ack_ok = any(p[0] == HIWORLD_CMD_HEARTBEAT and len(p[1]) >= 1 and p[1][0] == HIWORLD_CMD_AC_SET for p in rx_pkts)

        # Check CAN 0x1D0 injected with DUAL_REQ
        injected_1d0 = harness.read_can_frame(CAN_ID_CLIMATE_PANEL_1D0, timeout_sec=0.2)
        can_inj_ok = (injected_1d0 is not None and len(injected_1d0) >= 1 and (injected_1d0[0] & 0x01) != 0)

        pass_all = ack_ok and can_inj_ok
        flag = "[PASS]" if pass_all else "[FAIL]"
        if not pass_all:
            all_passed = False
        can_str = ' '.join([f'{x:02X}' for x in injected_1d0]) if injected_1d0 else 'None'
        print(f"  {flag} HU 0x3B -> CAN 0x1D0 Climate injection (ACK={ack_ok}, 0x1D0={can_str})")

        return all_passed


# ==============================================================================
# Scenario 3: Trip Computer & Resets
# ==============================================================================
class TripScenario(BaseWorkbenchScenario):
    name = "trip"
    description = "Trip Computer Telemetry & Downlink Resets (0x221 <-> 0x13, 0x1B)"

    def run(self, harness: BsiWorkbenchHarness, filter_key: str = None) -> bool:
        print("\n" + "=" * 72)
        print("SCENARIO: TRIP COMPUTER & TELEMETRY")
        print("=" * 72)

        all_passed = True

        # Test A1: CAN 0x221 Instantaneous Trip -> HU 0x13
        print("\n--- Test A1: CAN 0x221 Instant Fuel & Autonomy -> HU 0x13 ---")
        # 0x221: Byte 0: 0x00 (info valid, no stalk press), Byte 1-2: Range 650 km (0x028A), Byte 3-4: Fuel 6.5 L/100km (0x0041)
        can_221 = bytes([0x00, 0x02, 0x8A, 0x00, 0x41, 0x00, 0x00])
        harness.send_can_frame(CAN_ID_TRIP_INSTANT_221, can_221)

        pkts = harness.read_uart_packets(duration=0.3)
        trip_pkts = [p for p in pkts if p[0] == HIWORLD_CMD_TRIP_INSTANT]

        trip_ok = len(trip_pkts) > 0
        flag = "[PASS]" if trip_ok else "[FAIL]"
        if not trip_ok:
            all_passed = False
        print(f"  {flag} CAN 0x221 -> HU 0x13 Instant Trip Telemetry (Received: {trip_ok})")

        # Test B1: HU Downlink 0x1B Trip 1 Reset -> CAN 0x221 Reset Pulse
        print("\n--- Test B1: HU Downlink 0x1B (Reset Trip 1) -> CAN 0x221 Pulse ---")
        harness.drain_can(0.05)
        # Cmd 0x1B: Target Page 2 (Trip 1), Reset Action 2
        harness.send_hu_packet(HIWORLD_CMD_TRIP_RESET, bytes([0x02, 0x02, 0x01]))

        rx_pkts = harness.read_uart_packets(duration=0.25)
        ack_ok = any(p[0] == HIWORLD_CMD_HEARTBEAT and len(p[1]) >= 1 and p[1][0] == HIWORLD_CMD_TRIP_RESET for p in rx_pkts)

        injected_221 = harness.read_can_frame(CAN_ID_TRIP_INSTANT_221, timeout_sec=0.2)
        # Trip 1 reset injects 0x221 with 0x80 or 0x40
        reset_pulse_ok = (injected_221 is not None and len(injected_221) >= 4)

        pass_all = ack_ok and reset_pulse_ok
        flag = "[PASS]" if pass_all else "[FAIL]"
        if not pass_all:
            all_passed = False
        print(f"  {flag} HU 0x1B -> CAN 0x221 Reset Injection (ACK={ack_ok}, Injected={reset_pulse_ok})")

        return all_passed


# ==============================================================================
# Scenario 4: Doors & Trunk Body Status
# ==============================================================================
class DoorsScenario(BaseWorkbenchScenario):
    name = "doors"
    description = "Door Switches & Trunk Opening (0x220/0x036 <-> 0x12)"

    def run(self, harness: BsiWorkbenchHarness, filter_key: str = None) -> bool:
        print("\n" + "=" * 72)
        print("SCENARIO: DOORS & BODY STATUS")
        print("=" * 72)

        # Test CAN 0x220 Door Open -> HU 0x12
        # 0x220: Driver door open
        can_doors = bytes([0x84, 0x00])
        harness.send_can_frame(CAN_ID_DOORS_BODY_220, can_doors)

        pkts = harness.read_uart_packets(duration=0.3)
        door_pkts = [p for p in pkts if p[0] == HIWORLD_CMD_DOORS]

        doors_ok = len(door_pkts) > 0
        flag = "[PASS]" if doors_ok else "[FAIL]"
        print(f"  {flag} CAN 0x220 -> HU 0x12 Doors Status (Received: {doors_ok})")
        return doors_ok


# ==============================================================================
# Scenario Registry & CLI Runner
# ==============================================================================
SCENARIO_REGISTRY: Dict[str, BaseWorkbenchScenario] = {
    "config": CarConfigScenario(),
    "climate": ClimateScenario(),
    "trip": TripScenario(),
    "doors": DoorsScenario(),
}


def sniff_bsi_status(can_iface: str):
    """Passively sniffs and decodes active BSI signals on the comfort bus."""
    print(f"[*] Sniffing PSA Comfort Bus on {can_iface} for 3 seconds...")
    try:
        s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, CAN_RAW)
        s.bind((can_iface,))
        s.setblocking(False)
    except Exception as e:
        print(f"[ERROR] Cannot bind to {can_iface}: {e}")
        return

    frames = {}
    t0 = time.time()
    while time.time() - t0 < 3.0:
        r, _, _ = select.select([s], [], [], 0.1)
        if r:
            raw = s.recv(16)
            cid, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
            frames[cid] = data[:dlc]

    s.close()

    print(f"\n[OK] Captured {len(frames)} active CAN IDs on bus:")
    for cid in sorted(frames.keys()):
        d = frames[cid]
        hex_data = " ".join([f"{x:02X}" for x in d])
        label = ""
        if cid == CAN_ID_BSI_IGNITION_036:
            label = f"[BSI Ignition: ACC={(d[4] == 1)} Dimmer={d[3] & 0x0F}]"
        elif cid == CAN_ID_BSI_SLOW_0F6:
            label = "[BSI Slow Telemetry / Economy Mode]"
        elif cid == CAN_ID_BSI_CONFIG_260:
            p_id = d[0] if len(d) > 0 else 0
            label = (f"[BSI Config Profile {p_id}: "
                     f"AFS={bool(d[2] & 0x80) if len(d) > 2 else False} "
                     f"FollowMe={bool(d[2] & 0x20) if len(d) > 2 else False} "
                     f"AutoLight={bool(d[2] & 0x10) if len(d) > 2 else False} "
                     f"DRL={bool(d[3] & 0x40) if len(d) > 3 else False} "
                     f"Lock={bool(d[4] & 0x08) if len(d) > 4 else False} "
                     f"Wiper={bool(d[5] & 0x10) if len(d) > 5 else False} "
                     f"Radar={bool(d[5] & 0x40) if len(d) > 5 else False}]")
        elif cid == CAN_ID_BSI_PERSO_361:
            label = (f"[BSI Equipment Mask (Menu Availability): "
                     f"FollowMeAvail={bool(d[2] & 0x10) if len(d) > 2 else False} "
                     f"DRLAvail={bool(d[3] & 0x40) if len(d) > 3 else False} "
                     f"AASAvail={bool(d[4] & 0x20) if len(d) > 4 else False}]")
        elif cid == CAN_ID_BSI_DOWNLINK_15B:
            label = f"[BSI Config Mutation Request (Downlink, Profile {d[0] if len(d) > 0 else 0})]"
        elif cid == CAN_ID_RD4_SOURCE_165:
            label = f"[RD4 Radio Source / Volume / Power: 0x{d[0]:02X}]"
        elif cid == CAN_ID_ALERT_JOURNAL_120:
            label = "[BSI Alert Journal / Diagnostics]"
        print(f"  ID 0x{cid:03X} [{len(d)}]: {hex_data:24s} {label}")


def main():
    parser = argparse.ArgumentParser(description="OpenCanbox Core Generic BSI & SLCAN Workbench Test Harness")
    parser.add_argument("--can", default="can0", help="SocketCAN interface name (default: can0)")
    parser.add_argument("--slcan-dev", default=None, help="SLCAN serial device (e.g. /dev/ttyUSB4)")
    parser.add_argument("--hu", default=None, help="Physical HU UART port (default: launches native simulator via PTY)")
    parser.add_argument("--scenario", default="config", help="Test scenario to run: config, climate, trip, doors, all (default: config)")
    parser.add_argument("--setting", default=None, help="Filter specific setting within scenario (e.g. drl, wiper, lock)")
    parser.add_argument("--sniff", action="store_true", help="Passively sniff and decode live BSI CAN traffic")
    args = parser.parse_args()

    if not check_and_bringup_slcan(args.can, args.slcan_dev):
        sys.exit(1)

    if args.sniff:
        sniff_bsi_status(args.can)
        sys.exit(0)

    use_native = (args.hu is None)
    harness = BsiWorkbenchHarness(can_iface=args.can, hu_port=args.hu, use_native_daemon=use_native)

    if not harness.start():
        sys.exit(1)

    try:
        scenarios_to_run = []
        if args.scenario == "all":
            scenarios_to_run = list(SCENARIO_REGISTRY.values())
        elif args.scenario in SCENARIO_REGISTRY:
            scenarios_to_run = [SCENARIO_REGISTRY[args.scenario]]
        else:
            print(f"[ERROR] Unknown scenario '{args.scenario}'. Available: {list(SCENARIO_REGISTRY.keys()) + ['all']}")
            sys.exit(1)

        overall_success = True
        for sc in scenarios_to_run:
            ok = sc.run(harness, filter_key=args.setting)
            if not ok:
                overall_success = False

        print("\n" + "=" * 72)
        print("OVERALL WORKBENCH VERDICT: {}".format("ALL SCENARIOS PASSED" if overall_success else "SOME SCENARIOS FAILED"))
        print("=" * 72 + "\n")
        sys.exit(0 if overall_success else 1)
    finally:
        harness.stop()


if __name__ == "__main__":
    main()
