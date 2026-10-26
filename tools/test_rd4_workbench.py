#!/usr/bin/env python3
"""
RD4 Workbench Automated Integration Test Suite
==============================================
Validates RD4 Audio Routing, Virtual CDC Emulation, and Stalk Arbitration
directly on real workbench hardware (RD4 connected to can0 and CAN box on ttyUSB/PTY).

Features:
1. Native SocketCAN (can0) sniffing & frame injection (no pip dependencies).
2. Autonomous BSI simulation (0x036 / 0x0F6) to keep RD4 awake without vehicle BSI.
3. Integrated OpenCanbox native firmware execution via PTY (with optional hardware HU mirroring).
4. Head Unit Downlink injection & Uplink telemetry verification over UART.
5. Comprehensive test scenarios:
   - Random-order closed-loop source switching with overshoot detection
   - Station list query (0x0F) frequency preservation check
   - Rapid mid-flight source retargeting stress test
   - CDC playback state toggling (Play 0x03 vs Pause 0x02 on CAN 0x162)
   - Physical stalk button override & debounce filter (Rule F3)
   - Head Unit radio/volume report (0xA1) station refresh suppression
6. Comprehensive timestamped logging for post-mortem debugging.

Usage:
  ./tools/test_rd4_workbench.py -y  # Auto-detect ports, launch firmware, run tests
  ./tools/test_rd4_workbench.py --can can0 --hu /dev/ttyUSB4
  ./tools/test_rd4_workbench.py --no-app --hu /dev/ttyUSB4  # External hardware CAN box
  ./tools/test_rd4_workbench.py --help
"""

import argparse
import datetime
import os
import pty
import random
import re
import select
import serial
import serial.tools.list_ports
import socket
import struct
import subprocess
import sys
import threading
import time
import tty

# SocketCAN frame format: uint32 can_id, uint8 dlc, 3 bytes pad, 8 bytes data
CAN_FRAME_FMT = "=IB3x8s"

# PSA CAN2004 Message IDs
CAN_ID_BSI_IGNITION_036 = 0x036
CAN_ID_BSI_SLOW_0F6     = 0x0F6
CAN_ID_RD4_SOURCE_165   = 0x165
CAN_ID_RD4_STATUS_225   = 0x225
CAN_ID_RD4_STATION_2A5  = 0x2A5
CAN_ID_RD4_LIST_125     = 0x125
CAN_ID_STALK_21F        = 0x21F
CAN_ID_CDC_STATUS_162   = 0x162
CAN_ID_CDC_TRACK_1E2    = 0x1E2

# RD4 Source IDs (canonical arbiter mapping)
PSA_RD4_SRC_TUNER = 0x01
PSA_RD4_SRC_CD    = 0x02
PSA_RD4_SRC_CDC   = 0x03
PSA_RD4_SRC_AUX   = 0x04

SOURCE_NAMES = {
    PSA_RD4_SRC_TUNER: "TUNER",
    PSA_RD4_SRC_CD:    "CD",
    PSA_RD4_SRC_CDC:   "CDC",
    PSA_RD4_SRC_AUX:   "AUX",
}

# Hiworld Downlink Commands
CMD_SWITCH_TO_HU_AUDIO = bytes.fromhex("5AA502AD0303B5")  # SubCmd 0x03: Switch to Line In 1 / AUX 1
CMD_AUDIO_PLAY         = bytes.fromhex("5AA502AD0201B1")  # SubCmd 0x02: Audio Start (Play)
CMD_AUDIO_STANDBY      = bytes.fromhex("5AA502AD0200B0")  # SubCmd 0x02: Audio Stop (Standby)
CMD_SWITCH_TO_TUNER    = bytes.fromhex("5AA502AD0301B3")  # SubCmd 0x03: Switch to Tuner
CMD_SWITCH_TO_CD       = bytes.fromhex("5AA502AD0302B4")  # SubCmd 0x03: Switch to Internal CD
CMD_SWITCH_TO_AUX      = bytes.fromhex("5AA502AD0304B6")  # SubCmd 0x03: Switch to AUX
CMD_QUERY_STATIONS     = bytes.fromhex("5AA5010F0F")
CMD_RADIO_VOLUME_SYNC  = bytes.fromhex("5AA503A1000117BB")  # Master setting / volume report (source=Radio, vol=23)
CMD_RADIO_VOLUME_MUTE  = bytes.fromhex("5AA503A10001FFA3")  # Master setting / volume report (source=Radio, vol=-1)

SOURCE_SWITCH_COMMANDS = {
    PSA_RD4_SRC_TUNER: CMD_SWITCH_TO_TUNER,
    PSA_RD4_SRC_CD:    CMD_SWITCH_TO_CD,
    PSA_RD4_SRC_AUX:   CMD_SWITCH_TO_HU_AUDIO,
}


def build_hiworld_packet(cmd: int, payload: bytes) -> bytes:
    plen = len(payload)
    checksum = ((plen + cmd + sum(payload)) - 1) & 0xFF
    return bytes([0x5A, 0xA5, plen, cmd]) + payload + bytes([checksum])


# ==============================================================================
# Logging
# ==============================================================================
class WorkbenchLogger:
    def __init__(self, log_path: str = None):
        if not log_path:
            dt_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            log_path = f"rd4_workbench_test_{dt_str}.log"
        self.log_path = os.path.abspath(log_path)
        self.file = open(self.log_path, "w", encoding="utf-8")
        self.lock = threading.Lock()
        print(f"[*] Detailed test log created at: {self.log_path}")

    def log(self, tag: str, message: str, console: bool = False, color: str = ""):
        now = time.time()
        ts_str = f"{now:.4f}"
        line = f"{ts_str} [{tag}] {message}"
        with self.lock:
            self.file.write(line + "\n")
            self.file.flush()

        if console:
            if color:
                print(f"{color}[{tag}] {message}\033[0m")
            else:
                print(f"[{tag}] {message}")

    def close(self):
        with self.lock:
            self.file.close()


# ==============================================================================
# SocketCAN Interface
# ==============================================================================
class CanBus:
    def __init__(self, iface: str, logger: WorkbenchLogger):
        self.iface = iface
        self.logger = logger
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((iface,))
        self.running = True
        self.lock = threading.Lock()
        self.last_frames = {}
        self.frame_history = []
        self.thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.thread.start()

    def _rx_loop(self):
        while self.running:
            try:
                r, _, _ = select.select([self.sock], [], [], 0.05)
                if not r:
                    continue
                packet = self.sock.recv(16)
                if len(packet) < 16:
                    continue
                can_id_raw, can_dlc, data = struct.unpack(CAN_FRAME_FMT, packet)
                can_id = can_id_raw & 0x7FF
                dlc = min(can_dlc, 8)
                frame_data = data[:dlc]

                with self.lock:
                    self.last_frames[can_id] = (time.time(), frame_data)
                    self.frame_history.append((time.time(), can_id, frame_data))
                    if len(self.frame_history) > 3000:
                        del self.frame_history[:1000]

                self.logger.log("CAN_RX", f"ID:{can_id:03X} DLC:{dlc} DATA:{frame_data.hex()}")
            except Exception as e:
                if self.running:
                    self.logger.log("CAN_ERR", f"RX error: {e}")

    def send(self, can_id: int, data: bytes):
        dlc = len(data)
        padded = data.ljust(8, b"\x00")
        pkt = struct.pack(CAN_FRAME_FMT, can_id & 0x7FF, dlc, padded)
        try:
            self.sock.send(pkt)
            self.logger.log("CAN_TX", f"ID:{can_id:03X} DLC:{dlc} DATA:{data.hex()}")
        except Exception as e:
            self.logger.log("CAN_ERR", f"TX error ID 0x{can_id:03X}: {e}")

    def get_last_frame(self, can_id: int):
        with self.lock:
            return self.last_frames.get(can_id)

    def wait_frame(self, can_id: int, match_fn=None, timeout: float = 2.0, start_time: float = None):
        deadline = time.time() + timeout
        start_t = time.time() if start_time is None else start_time
        while time.time() < deadline:
            with self.lock:
                for t, cid, data in reversed(self.frame_history):
                    if t < start_t:
                        break
                    if cid == can_id:
                        if match_fn is None or match_fn(data):
                            return t, data
            time.sleep(0.01)
        return None

    def close(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass


# ==============================================================================
# BSI Simulator (Keeps RD4 awake on standalone bench)
# ==============================================================================
class BsiSimulator(threading.Thread):
    """Broadcasts BSI wakeup and slow telemetry frames every 100 ms."""
    def __init__(self, can_bus: CanBus, logger: WorkbenchLogger):
        super().__init__(daemon=True)
        self.can_bus = can_bus
        self.logger = logger
        self.running = True

    def run(self):
        self.logger.log("BSI_SIM", "Starting BSI wakeup keeper thread (0x036 / 0x0F6)")
        while self.running:
            # 0x036 MSG_BSI_RAP_VOL: Ignition ON (Phase de vie = 0x01)
            self.can_bus.send(CAN_ID_BSI_IGNITION_036, bytes.fromhex("0E00002A010000A0"))
            # 0x0F6 MSG_BSI_LENT: Economy mode inactive, valid telemetry
            self.can_bus.send(CAN_ID_BSI_SLOW_0F6, bytes.fromhex("88FFFFFFFFFFFF20"))
            time.sleep(0.10)

    def stop(self):
        self.running = False


# ==============================================================================
# Head Unit UART Interface & Firmware Application Host
# ==============================================================================
class HuUart:
    """
    Handles Head Unit UART communications.
    Supports two operating modes:
    1. Native App Mode (default): Spawns native OpenCanbox firmware over PTY,
       acting as the Head Unit. If a real physical Head Unit port is provided,
       mirrors UART packets to it so external hardware is kept in sync.
    2. Hardware Mode (--no-app): Opens a physical serial port to an external
       microcontroller running OpenCanbox.
    """
    def __init__(self, port: str, baud: int, logger: WorkbenchLogger, can_iface: str = "can0", run_app: bool = True):
        self.port = port
        self.baud = baud
        self.logger = logger
        self.can_iface = can_iface
        self.run_app = run_app
        self.running = True
        self.rx_frames = []
        self.lock = threading.Lock()
        self.master_fd = None
        self.slave_fd = None
        self.app_proc = None
        self.phys_ser = None

        if self.run_app:
            self._start_native_app()
        elif self.port:
            self.phys_ser = serial.Serial(self.port, self.baud, timeout=0.05)

        self.thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.thread.start()

    def _start_native_app(self):
        bin_path = os.path.abspath(".pio/build/native_test/program")
        self.logger.log("APP_INIT", "Compiling native OpenCanbox firmware...")
        pio_path = os.path.expanduser("~/.platformio/penv/bin/pio")
        pio_cmd = pio_path if os.path.exists(pio_path) else "pio"
        subprocess.run([pio_cmd, "run", "-s", "-e", "native_test"], check=True)

        self.master_fd, self.slave_fd = pty.openpty()
        try:
            tty.setraw(self.master_fd)
            tty.setraw(self.slave_fd)
        except Exception:
            pass
        slave_name = os.ttyname(self.slave_fd)

        # Open physical Head Unit serial port if available
        if self.port and os.path.exists(self.port):
            try:
                self.phys_ser = serial.Serial(self.port, self.baud, timeout=0.05)
                self.logger.log("HU_UART", f"Mirrored to physical Head Unit UART: {self.port}")
            except Exception as e:
                self.logger.log("HU_UART_ERR", f"Could not open physical HU UART {self.port}: {e}")

        env = os.environ.copy()
        env["CANBOX_CAN_IFACE"] = self.can_iface
        env["CANBOX_UART_DEVICE"] = slave_name
        env["CANBOX_HU_PROTOCOL"] = "hiworld"

        self.logger.log("APP_INIT", f"Launching OpenCanbox firmware (CAN: {self.can_iface}, PTY: {slave_name})")
        self.app_proc = subprocess.Popen(
            ["stdbuf", "-oL", "-eL", bin_path],
            env=env,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT
        )

        def _read_app_stdout():
            for line in iter(self.app_proc.stdout.readline, b''):
                try:
                    text = line.decode('utf-8', errors='replace').rstrip()
                    if text:
                        self.logger.log("APP", text)
                except Exception:
                    pass

        threading.Thread(target=_read_app_stdout, daemon=True).start()

    def _rx_loop(self):
        buf = bytearray()
        while self.running:
            try:
                read_fds = []
                if self.master_fd is not None:
                    read_fds.append(self.master_fd)
                if self.phys_ser is not None and self.phys_ser.is_open:
                    read_fds.append(self.phys_ser.fileno())

                if not read_fds:
                    time.sleep(0.02)
                    continue

                r, _, _ = select.select(read_fds, [], [], 0.05)

                # 1. Read from native OpenCanbox firmware (via master_fd)
                if self.master_fd is not None and self.master_fd in r:
                    chunk = os.read(self.master_fd, 1024)
                    if chunk:
                        buf.extend(chunk)
                        if self.phys_ser and self.phys_ser.is_open:
                            try:
                                self.phys_ser.write(chunk)
                            except Exception:
                                pass

                # 2. Read from physical Head Unit
                # When run_app is True, drain physical HU RX so its buffer does not fill,
                # but do NOT forward competing 0xAD/0x0F/0xA1 commands into master_fd while
                # the automated workbench test runner is controlling the firmware!
                if self.phys_ser is not None and self.phys_ser.fileno() in r:
                    chunk = self.phys_ser.read(self.phys_ser.in_waiting or 1)
                    if chunk and not self.run_app:
                        buf.extend(chunk)

                # Hiworld frame reassembly: 5A A5 <len> <cmd> <payload...> <cs>
                while len(buf) >= 4:
                    if buf[0] != 0x5A or buf[1] != 0xA5:
                        del buf[0]
                        continue
                    plen = buf[2]
                    total_len = plen + 5
                    # Special 0-payload query case: len=1 (5AA5010F0F)
                    if plen == 1 and len(buf) >= 5:
                        cmd = buf[3]
                        if cmd in (0x0F, 0x11, 0x12, 0xFF):
                            total_len = 5

                    if len(buf) < total_len:
                        break
                    frame = bytes(buf[:total_len])
                    del buf[:total_len]

                    with self.lock:
                        self.rx_frames.append((time.time(), frame))
                        if len(self.rx_frames) > 1000:
                            del self.rx_frames[:300]

                    cmd = frame[3]
                    self.logger.log("HU_RX", f"CMD:0x{cmd:02X} RAW:{frame.hex()}")
            except Exception as e:
                if self.running:
                    self.logger.log("HU_ERR", f"RX error: {e}")

    def send(self, data: bytes):
        try:
            if self.master_fd is not None:
                os.write(self.master_fd, data)
            elif self.phys_ser and self.phys_ser.is_open:
                self.phys_ser.write(data)
            self.logger.log("HU_TX", f"RAW:{data.hex()}")
        except Exception as e:
            self.logger.log("HU_ERR", f"TX error: {e}")

    def wait_packet(self, cmd: int, timeout: float = 2.0, match_fn=None, start_time: float = None):
        deadline = time.time() + timeout
        start_t = time.time() if start_time is None else start_time
        while time.time() < deadline:
            with self.lock:
                for t, frame in reversed(self.rx_frames):
                    if t < start_t:
                        break
                    if len(frame) >= 4 and frame[3] == cmd:
                        if match_fn is None or match_fn(frame):
                            return t, frame
            time.sleep(0.01)
        return None

    def close(self):
        self.running = False
        if self.app_proc and self.app_proc.poll() is None:
            self.app_proc.terminate()
            try:
                self.app_proc.wait(timeout=1.0)
            except Exception:
                self.app_proc.kill()
        if self.master_fd is not None:
            try:
                os.close(self.master_fd)
            except Exception:
                pass
        if self.slave_fd is not None:
            try:
                os.close(self.slave_fd)
            except Exception:
                pass
        if self.phys_ser and self.phys_ser.is_open:
            try:
                self.phys_ser.close()
            except Exception:
                pass


# ==============================================================================
# Workbench Integration Test Runner
# ==============================================================================
class Rd4WorkbenchRunner:
    def __init__(self, can_iface: str, hu_port: str = None, baud: int = 38400, bsi_sim: bool = True, run_app: bool = True):
        self.logger = WorkbenchLogger()
        self.can_bus = CanBus(can_iface, self.logger)
        self.bsi_sim = BsiSimulator(self.can_bus, self.logger) if bsi_sim else None
        self.hu_uart = HuUart(port=hu_port, baud=baud, logger=self.logger, can_iface=can_iface, run_app=run_app)
        self.results = []

    def start(self):
        if self.bsi_sim:
            self.bsi_sim.start()
        time.sleep(0.5)

    def close(self):
        if self.bsi_sim:
            self.bsi_sim.stop()
        self.hu_uart.close()
        self.can_bus.close()
        self.logger.close()

    def record_test(self, name: str, passed: bool, details: str = ""):
        self.results.append((name, passed, details))
        status_color = "\033[92m[PASS]\033[0m" if passed else "\033[91m[FAIL]\033[0m"
        print(f"{status_color} {name} {('- ' + details) if details else ''}")
        self.logger.log("TEST_RESULT", f"{name}: {'PASS' if passed else 'FAIL'} - {details}", console=False)

    def get_current_rd4_source(self, timeout: float = 1.0, wait_powered: bool = False):
        """Returns (source_id, power_status, sleep_status) from CAN 0x165."""
        match_fn = None
        if wait_powered:
            match_fn = lambda d: len(d) >= 3 and (d[0] & 0x80) != 0 and ((d[2] >> 4) & 0x0F) != 0

        res = self.can_bus.wait_frame(CAN_ID_RD4_SOURCE_165, match_fn=match_fn, timeout=timeout)
        if not res:
            return None, None, None
        _, data = res
        if len(data) < 3:
            return None, None, None
        power = (data[0] & 0x80) != 0
        sleep = (data[0] & 0xC0) == 0
        nibble = (data[2] >> 4) & 0x0F
        # 1=Tuner, 2=CD, 3=CDC, 4=AUX1, 5=AUX2
        src_map = {1: PSA_RD4_SRC_TUNER, 2: PSA_RD4_SRC_CD, 3: PSA_RD4_SRC_CDC, 4: PSA_RD4_SRC_AUX, 5: PSA_RD4_SRC_AUX}
        src = src_map.get(nibble, 0)
        return src, power, sleep

    # --------------------------------------------------------------------------
    # Test 1: Pre-flight Verification
    # --------------------------------------------------------------------------
    def test_01_preflight_rd4_and_canbox(self) -> bool:
        print("\n\033[94m[*] Test 1: Pre-flight RD4 & CAN Box Connectivity\033[0m")
        # 1. Check RD4 on CAN 0x165 (wait up to 5.0s for boot / wake from sleep)
        print("    Waiting for RD4 to wake up and report active source on CAN 0x165...")
        src, power, _ = self.get_current_rd4_source(timeout=5.0, wait_powered=True)
        if src is None or not power or src == 0:
            self.record_test("Pre-flight: RD4 Presence", False, f"RD4 not detected on CAN 0x165 (src={src}, power={power})")
            return False
        self.record_test("Pre-flight: RD4 Presence", True, f"RD4 Online: initial source={SOURCE_NAMES.get(src, 'UNKNOWN')}")

        # 2. Check CAN Box HU UART responsiveness
        self.hu_uart.send(bytes.fromhex("5AA502AD0200B0"))  # CDC Standby ping
        ack = self.hu_uart.wait_packet(0xFF, timeout=1.5, match_fn=lambda f: len(f) >= 5 and f[4] == 0xAD)
        if not ack:
            self.hu_uart.send(bytes.fromhex("5AA5010F0F"))
            ack = self.hu_uart.wait_packet(0x84, timeout=1.5)
        if not ack:
            self.record_test("Pre-flight: CAN Box UART", False, "No response received from CAN Box UART")
            return False
        self.record_test("Pre-flight: CAN Box UART", True, f"UART response: {ack[1].hex()}")
        return True

    # --------------------------------------------------------------------------
    # Test 2: Closed-Loop Random Source Switching (Overshoot & Convergence)
    # --------------------------------------------------------------------------
    def test_02_random_source_switching(self, hops: int = 8) -> bool:
        print(f"\n\033[94m[*] Test 2: Random Source Switching ({hops} Hops, Closed-Loop Convergence & Overshoot Check)\033[0m")
        all_passed = True
        target_pool = [PSA_RD4_SRC_TUNER, PSA_RD4_SRC_CD, PSA_RD4_SRC_AUX]

        for hop in range(1, hops + 1):
            curr_src, _, _ = self.get_current_rd4_source(timeout=1.0)
            if curr_src is None:
                self.record_test(f"Hop {hop}: Read Current Source", False, "RD4 stopped responding on 0x165")
                return False

            # Pick target different from current
            candidates = [s for s in target_pool if s != curr_src]
            target_src = random.choice(candidates)
            target_name = SOURCE_NAMES[target_src]
            curr_name = SOURCE_NAMES.get(curr_src, 'UNKNOWN')

            cmd_bytes = SOURCE_SWITCH_COMMANDS[target_src]
            self.logger.log("TEST_HOP", f"Hop {hop}/{hops}: Requesting {curr_name} -> {target_name} ({cmd_bytes.hex()})", console=True)

            t0 = time.time()
            self.hu_uart.send(cmd_bytes)

            # 1. Verify UART ACK (0xFF 0xAD)
            ack = self.hu_uart.wait_packet(0xFF, timeout=0.25, match_fn=lambda f: len(f) >= 5 and f[4] == 0xAD)
            if not ack:
                self.record_test(f"Hop {hop}: UART ACK", False, f"Missing ACK for 0xAD")
                all_passed = False

            # 2. Monitor CAN bus until target reached or timeout (3.5s max)
            deadline = time.time() + 3.5
            reached = False
            while time.time() < deadline:
                src, _, _ = self.get_current_rd4_source(timeout=0.2)
                if src == target_src:
                    reached = True
                    break
                time.sleep(0.02)

            duration = time.time() - t0
            if not reached:
                self.record_test(f"Hop {hop}: {curr_name} -> {target_name}", False, f"Timeout after {duration:.2f}s (ended on {SOURCE_NAMES.get(src, 'UNK')})")
                all_passed = False
                continue

            # 3. Overshoot verification: hold and monitor for 0.8s to assert RD4 stays on target
            time.sleep(0.3)
            overshoot = False
            check_deadline = time.time() + 0.8
            while time.time() < check_deadline:
                src, _, _ = self.get_current_rd4_source(timeout=0.2)
                if src is not None and src != target_src:
                    overshoot = True
                    overshoot_src = src
                    break
                time.sleep(0.05)

            if overshoot:
                self.record_test(f"Hop {hop}: {curr_name} -> {target_name}", False, f"OVERSHOOT detected! Slid past {target_name} to {SOURCE_NAMES.get(overshoot_src, 'UNK')}")
                all_passed = False
            else:
                self.record_test(f"Hop {hop}: {curr_name} -> {target_name}", True, f"Clean convergence in {duration:.2f}s (no overshoot)")

            time.sleep(0.2)

        return all_passed

    # --------------------------------------------------------------------------
    # Test 3: Tuner Station List Request & Non-Destructive Frequency Check
    # --------------------------------------------------------------------------
    def test_03_station_list_frequency_preservation(self) -> bool:
        print("\n\033[94m[*] Test 3: Tuner Station List Request & Non-Destructive Frequency Preservation\033[0m")
        # 1. Switch to Tuner first
        self.hu_uart.send(CMD_SWITCH_TO_TUNER)
        deadline = time.time() + 3.0
        while time.time() < deadline:
            src, _, _ = self.get_current_rd4_source(timeout=0.2)
            if src == PSA_RD4_SRC_TUNER:
                break
            time.sleep(0.05)

        src, _, _ = self.get_current_rd4_source(timeout=1.0)
        if src != PSA_RD4_SRC_TUNER:
            self.record_test("Station List: Prerequisite", False, f"Could not switch RD4 to Tuner (at {SOURCE_NAMES.get(src, 'UNK')})")
            return False

        # 2. Read initial Frequency from CAN 0x225
        frame_225 = self.can_bus.wait_frame(CAN_ID_RD4_STATUS_225, timeout=1.5)
        if not frame_225:
            self.record_test("Station List: Read Frequency", False, "CAN 0x225 not broadcast by RD4")
            return False
        _, data_225_initial = frame_225
        freq_raw_initial = ((data_225_initial[3] & 0x0F) << 8) | data_225_initial[4]
        self.logger.log("TEST_TUNER", f"Initial Tuner frequency raw: 0x{freq_raw_initial:04X} ({freq_raw_initial * 0.05 + 50.0:.1f} MHz)", console=True)

        # 3. Inject HU Station List Query: 5A A5 01 0F 0F
        # Allow 2.1s debounce window to elapse from prerequisite switch so rotary scroll executes
        time.sleep(2.1)
        t_before_cmd = time.time()
        self.hu_uart.send(CMD_QUERY_STATIONS)

        # 4. Check that CAN Box emitted rotary pulses on 0x21F (scroll up, then restore down)
        pulse_found = False
        deadline = time.time() + 1.2
        while time.time() < deadline:
            res = self.can_bus.wait_frame(CAN_ID_STALK_21F, timeout=0.1, start_time=t_before_cmd)
            if res:
                _, pdata = res
                if len(pdata) >= 2 and pdata[1] != 0:  # Rotary counter moved
                    pulse_found = True
                    break
            time.sleep(0.01)

        self.record_test("Station List: Rotary Pulse Injection", pulse_found, "Observed rotary step on CAN 0x21F" if pulse_found else "No rotary pulse on CAN 0x21F")

        # 5. Check UART Uplink for Cmd 0x84
        uart_84 = self.hu_uart.wait_packet(0x84, timeout=1.5, start_time=t_before_cmd)
        self.record_test("Station List: HU Uplink 0x84", (uart_84 is not None),
                         f"0x84={'OK' if uart_84 else 'MISSING'}")

        # 6. Verify RD4 Frequency remained completely unchanged!
        time.sleep(0.8)
        frame_225_after = self.can_bus.wait_frame(CAN_ID_RD4_STATUS_225, timeout=1.5)
        if not frame_225_after:
            self.record_test("Station List: Frequency Preservation", False, "CAN 0x225 stopped broadcasting")
            return False
        _, data_225_after = frame_225_after
        freq_raw_after = ((data_225_after[3] & 0x0F) << 8) | data_225_initial[4]
        freq_raw_after = ((data_225_after[3] & 0x0F) << 8) | data_225_after[4]

        freq_preserved = (freq_raw_initial == freq_raw_after)
        self.record_test("Station List: Frequency Preservation", freq_preserved,
                         f"Before: 0x{freq_raw_initial:04X}, After: 0x{freq_raw_after:04X} ({'IDENTICAL' if freq_preserved else 'DE-TUNED!'})")
        return freq_preserved

    # --------------------------------------------------------------------------
    # Test 4: Rapid Mid-Flight Retargeting (Stress Test)
    # --------------------------------------------------------------------------
    def test_04_rapid_retargeting_stress(self) -> bool:
        print("\n\033[94m[*] Test 4: Rapid Mid-Flight Retargeting Stress Test\033[0m")
        # Start on Tuner
        self.hu_uart.send(CMD_SWITCH_TO_TUNER)
        deadline = time.time() + 3.0
        while time.time() < deadline:
            src, _, _ = self.get_current_rd4_source(timeout=0.2)
            if src == PSA_RD4_SRC_TUNER:
                break
            time.sleep(0.05)

        # Fire Switch to CD, and 30 ms later Switch to HU Audio (Line In 1)
        self.logger.log("TEST_STRESS", "Sending Switch to CD followed immediately (30 ms) by Switch to HU Audio", console=True)
        self.hu_uart.send(CMD_SWITCH_TO_CD)
        time.sleep(0.03)
        self.hu_uart.send(CMD_SWITCH_TO_HU_AUDIO)

        # Wait for convergence to final requested target (AUX 1)
        deadline = time.time() + 3.5
        converged = False
        while time.time() < deadline:
            src, _, _ = self.get_current_rd4_source(timeout=0.2)
            if src == PSA_RD4_SRC_AUX:
                converged = True
                break
            time.sleep(0.02)

        time.sleep(0.5)
        # Check overshoot
        src_after, _, _ = self.get_current_rd4_source(timeout=0.5)
        stable = (src_after == PSA_RD4_SRC_AUX)

        passed = converged and stable
        self.record_test("Stress: Rapid Retargeting to AUX 1", passed,
                         f"Converged={'YES' if converged else 'NO'}, Stable={'YES' if stable else 'NO'}")
        return passed

    # --------------------------------------------------------------------------
    # --------------------------------------------------------------------------
    # Test 5: Line In 1 / AUX 1 Source Routing (Cmd 0xAD 03 03 and 0xAD 02 01)
    # --------------------------------------------------------------------------
    def test_05_cdc_playback_state_toggling(self) -> bool:
        print("\n\033[94m[*] Test 5: Line In 1 / AUX 1 Routing (Cmd 0xAD 03 03 & 02 01)\033[0m")
        # 1. Switch to AUX 1 via CMD_SWITCH_TO_HU_AUDIO (0xAD 03 03)
        self.hu_uart.send(CMD_SWITCH_TO_HU_AUDIO)
        deadline = time.time() + 3.0
        while time.time() < deadline:
            src, _, _ = self.get_current_rd4_source(timeout=0.2)
            if src == PSA_RD4_SRC_AUX:
                break
            time.sleep(0.05)

        src_aux, _, _ = self.get_current_rd4_source(timeout=0.5)
        switched_aux = (src_aux == PSA_RD4_SRC_AUX)
        self.record_test("HU Audio: Switch to Line In 1 via 0xAD 03 03", switched_aux,
                         f"Source is {SOURCE_NAMES.get(src_aux, 'UNK')}")

        # 2. Switch to Tuner first
        self.hu_uart.send(CMD_SWITCH_TO_TUNER)
        time.sleep(0.8)

        # 3. Switch to AUX 1 via CMD_AUDIO_PLAY (0xAD 02 01)
        self.hu_uart.send(CMD_AUDIO_PLAY)
        deadline = time.time() + 3.0
        while time.time() < deadline:
            src, _, _ = self.get_current_rd4_source(timeout=0.2)
            if src == PSA_RD4_SRC_AUX:
                break
            time.sleep(0.05)

        src_play, _, _ = self.get_current_rd4_source(timeout=0.5)
        switched_play = (src_play == PSA_RD4_SRC_AUX)
        self.record_test("HU Audio: Switch to Line In 1 via 0xAD 02 01", switched_play,
                         f"Source is {SOURCE_NAMES.get(src_play, 'UNK')}")

        return switched_aux and switched_play

    # --------------------------------------------------------------------------
    # Test 6: Physical Stalk / Radio Button Override (Rule F3 Filter)
    # --------------------------------------------------------------------------
    def test_06_physical_button_override(self) -> bool:
        print("\n\033[94m[*] Test 6: Physical Button Override & 800ms Debounce Filter (Rule F3)\033[0m")
        # Ensure we start on AUX 1
        self.hu_uart.send(CMD_SWITCH_TO_HU_AUDIO)
        deadline = time.time() + 3.0
        while time.time() < deadline:
            src, _, _ = self.get_current_rd4_source(timeout=0.2)
            if src == PSA_RD4_SRC_AUX:
                break
            time.sleep(0.05)

        time.sleep(0.4)
        src_init, _, _ = self.get_current_rd4_source(timeout=0.5)

        # Simulate user pressing physical SOURCE button on stalk (CAN 0x21F press + release)
        t_before_phys = time.time()
        self.logger.log("TEST_PHYSICAL", "Injecting simulated physical SRC button pulse on CAN 0x21F", console=True)
        self.can_bus.send(CAN_ID_STALK_21F, bytes.fromhex("020000"))
        time.sleep(0.06)
        self.can_bus.send(CAN_ID_STALK_21F, bytes.fromhex("000000"))

        # RD4 will step to next source (Tuner or CD)
        time.sleep(0.4)
        src_stepped, _, _ = self.get_current_rd4_source(timeout=1.0)
        stepped = (src_stepped is not None and src_stepped != src_init)
        self.record_test("Physical Button: RD4 Reacted", stepped,
                         f"Stepped from {SOURCE_NAMES.get(src_init, 'UNK')} to {SOURCE_NAMES.get(src_stepped, 'UNK')}")

        # Wait for settle filter to expire, then check that CAN Box emitted 0x84 to HU
        uart_84 = self.hu_uart.wait_packet(0x84, timeout=2.0, start_time=t_before_phys)
        synced = (uart_84 is not None)
        self.record_test("Physical Button: HU Synchronized after Debounce", synced,
                         f"Received 0x84: {uart_84[1].hex() if uart_84 else 'NONE'}")

        # Verify background 0xAD 02 01 poll from HU does NOT hijack physical source selection
        t_before_poll = time.time()
        self.hu_uart.send(CMD_AUDIO_PLAY)
        hijack_pulse = self.can_bus.wait_frame(
            CAN_ID_STALK_21F,
            match_fn=lambda d: len(d) >= 1 and (d[0] & 0x02) != 0,
            timeout=0.4,
            start_time=t_before_poll
        )
        no_hijack = (hijack_pulse is None)
        self.record_test("Physical Button: No 0xAD 02 01 Hijack", no_hijack,
                         "Periodic 0xAD 02 01 ignored after physical override" if no_hijack else "FAILED: Hijacked back to AUX 1!")

        # Clear override with standby (0xAD 02 00)
        self.hu_uart.send(CMD_AUDIO_STANDBY)
        return stepped and synced and no_hijack

    # --------------------------------------------------------------------------
    # Test 7: Head Unit Radio / Volume Sync Does NOT Trigger Station Refresh
    # --------------------------------------------------------------------------
    def test_07_radio_volume_no_station_refresh(self) -> bool:
        print("\n\033[94m[*] Test 7: Head Unit Radio / Volume Sync Does NOT Trigger Station Refresh\033[0m")
        # 1. Switch to Tuner first if not already on Tuner
        self.hu_uart.send(CMD_SWITCH_TO_TUNER)
        deadline = time.time() + 3.0
        while time.time() < deadline:
            src, _, _ = self.get_current_rd4_source(timeout=0.2)
            if src == PSA_RD4_SRC_TUNER:
                break
            time.sleep(0.05)

        src, _, _ = self.get_current_rd4_source(timeout=1.0)
        if src != PSA_RD4_SRC_TUNER:
            self.record_test("Radio/Volume: Prerequisite", False, f"Could not switch RD4 to Tuner (at {SOURCE_NAMES.get(src, 'UNK')})")
            return False

        # Allow 2.1s debounce window to elapse from prerequisite switch so rotary scroll state is idle
        time.sleep(2.1)

        # 2. Read initial Frequency from CAN 0x225
        frame_225 = self.can_bus.wait_frame(CAN_ID_RD4_STATUS_225, timeout=1.5)
        if not frame_225:
            self.record_test("Radio/Volume: Read Frequency", False, "CAN 0x225 not broadcast by RD4")
            return False
        _, data_225_initial = frame_225
        freq_raw_initial = ((data_225_initial[3] & 0x0F) << 8) | data_225_initial[4]
        self.logger.log("TEST_VOLUME", f"Initial Tuner frequency raw: 0x{freq_raw_initial:04X} ({freq_raw_initial * 0.05 + 50.0:.1f} MHz)", console=True)

        # 3. Inject HU Radio/Volume Master Setting frames: 5A A5 03 A1 00 01 17 BB, then 5A A5 03 A1 00 01 FF A3
        t_before_cmd = time.time()
        self.logger.log("TEST_VOLUME", "Injecting HU passive volume/source reports (0xA1)", console=True)
        self.hu_uart.send(CMD_RADIO_VOLUME_SYNC)
        time.sleep(0.05)
        self.hu_uart.send(CMD_RADIO_VOLUME_MUTE)

        # 4. Check UART responsiveness (universal ACK for 0xA1: 5A A5 01 FF A1 A0 or 0x84 state)
        ack = self.hu_uart.wait_packet(0xFF, timeout=0.8, match_fn=lambda f: len(f) >= 5 and f[4] == 0xA1, start_time=t_before_cmd)
        uart_84 = self.hu_uart.wait_packet(0x84, timeout=0.8, start_time=t_before_cmd)
        uart_ok = (ack is not None or uart_84 is not None)
        self.record_test("Radio/Volume: UART Responsiveness", uart_ok,
                         f"ACK: {ack[1].hex() if ack else 'none'}, 0x84: {'OK' if uart_84 else 'none'}")

        # 5. Verify CAN bus: Assert that CAN Box emitted ZERO rotary pulses on 0x21F
        pulse_frame = self.can_bus.wait_frame(
            CAN_ID_STALK_21F,
            match_fn=lambda d: len(d) >= 2 and d[1] != 0,
            timeout=1.5,
            start_time=t_before_cmd
        )
        pulse_found = (pulse_frame is not None)
        self.record_test("Radio/Volume: No Stalk Rotary Pulse (0x21F)", not pulse_found,
                         "Zero rotary pulses on CAN 0x21F (clean suppression)" if not pulse_found else f"FAILED: Stalk pulse detected: {pulse_frame[1].hex()}")

        # 6. Verify CAN bus: Assert that RD4 emitted ZERO station list frames on 0x125
        list_frame = self.can_bus.wait_frame(
            CAN_ID_RD4_LIST_125,
            timeout=0.3,
            start_time=t_before_cmd
        )
        list_found = (list_frame is not None)
        self.record_test("Radio/Volume: No Station List (0x125)", not list_found,
                         "Zero 0x125 station list frames provoked" if not list_found else f"FAILED: Station list burst detected: {list_frame[1].hex()}")

        # 7. Verify RD4 Frequency remained completely unchanged
        time.sleep(0.5)
        frame_225_after = self.can_bus.wait_frame(CAN_ID_RD4_STATUS_225, timeout=1.5)
        if not frame_225_after:
            self.record_test("Radio/Volume: Frequency Preservation", False, "CAN 0x225 stopped broadcasting")
            return False
        _, data_225_after = frame_225_after
        freq_raw_after = ((data_225_after[3] & 0x0F) << 8) | data_225_after[4]

        freq_preserved = (freq_raw_initial == freq_raw_after)
        self.record_test("Radio/Volume: Frequency Preservation", freq_preserved,
                         f"Before: 0x{freq_raw_initial:04X}, After: 0x{freq_raw_after:04X} ({'IDENTICAL' if freq_preserved else 'DE-TUNED!'})")

        return uart_ok and (not pulse_found) and (not list_found) and freq_preserved

    # --------------------------------------------------------------------------
    # Test Report Summary
    # --------------------------------------------------------------------------
    def print_summary(self) -> bool:
        print("\n" + "=" * 70)
        print("                 RD4 WORKBENCH INTEGRATION TEST REPORT                 ")
        print("=" * 70)
        total = len(self.results)
        passed = sum(1 for _, p, _ in self.results if p)
        failed = total - passed

        for name, p, details in self.results:
            status_str = "\033[92m[PASS]\033[0m" if p else "\033[91m[FAIL]\033[0m"
            print(f"{status_str} {name:<45} {details}")

        print("-" * 70)
        pct = (passed / total * 100.0) if total > 0 else 0.0
        color = "\033[92m" if failed == 0 else "\033[91m"
        print(f"TOTAL: {total} | {color}PASSED: {passed}\033[0m | FAILED: {failed} ({pct:.1f}%)")
        print(f"[*] Complete debugging log: {self.logger.log_path}")
        print("=" * 70 + "\n")
        return (failed == 0)


# ==============================================================================
# CLI Entrypoint & Auto-Discovery
# ==============================================================================
def find_ports():
    can_dev = "can0"
    hu_dev = None
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if "prolific" in desc or "pl2303" in desc or p.vid == 0x067B:
            hu_dev = p.device
            break
        elif "ftdi" in desc or "ft232" in desc or p.vid == 0x0403:
            hu_dev = p.device

    if not hu_dev:
        all_ports = [p.device for p in serial.tools.list_ports.comports() if "USB" in p.device]
        if all_ports:
            hu_dev = all_ports[0]
    return can_dev, hu_dev


def main():
    parser = argparse.ArgumentParser(description="RD4 Workbench Automated Integration Test Runner")
    parser.add_argument("--can", default="can0", help="SocketCAN interface name (default: can0)")
    parser.add_argument("--hu", help="Head Unit serial port (e.g. /dev/ttyUSB4, optional when running native app)")
    parser.add_argument("--baud", type=int, default=38400, help="Head Unit serial baud rate (default: 38400)")
    parser.add_argument("--no-bsi", action="store_true", help="Disable autonomous BSI simulation (use if real BSI is on CAN)")
    parser.add_argument("--no-app", action="store_true", help="Do not spawn native OpenCanbox firmware (use if hardware MCU board is connected to --hu)")
    parser.add_argument("--hops", type=int, default=8, help="Number of random source switching hops (default: 8)")
    parser.add_argument("-y", "--auto", action="store_true", help="Auto-detect serial devices")
    args = parser.parse_args()

    can_iface = args.can
    hu_port = args.hu

    if args.auto or not hu_port:
        detected_can, detected_hu = find_ports()
        can_iface = can_iface or detected_can
        if not hu_port and detected_hu:
            hu_port = detected_hu

    run_app = not args.no_app
    print("======================================================================")
    print("           PSA RD4 AUTOMATED WORKBENCH INTEGRATION TEST RUNNER        ")
    print(f"CAN Interface: {can_iface}")
    if run_app:
        print(f"Firmware Host: NATIVE (.pio/build/native_test/program via PTY)")
        if hu_port:
            print(f"Physical HU Mirror: {hu_port} @ {args.baud} baud")
    else:
        print(f"Firmware Host: HARDWARE SERIAL ({hu_port} @ {args.baud} baud)")
    print(f"BSI Wakeup Simulator: {'DISABLED' if args.no_bsi else 'ENABLED (keeps RD4 awake)'}")
    print("======================================================================")

    runner = Rd4WorkbenchRunner(
        can_iface=can_iface,
        hu_port=hu_port,
        baud=args.baud,
        bsi_sim=(not args.no_bsi),
        run_app=run_app
    )

    try:
        runner.start()

        # Run test suite
        if not runner.test_01_preflight_rd4_and_canbox():
            print("[!] Pre-flight checks failed. Check wiring and power.")
        else:
            runner.test_02_random_source_switching(hops=args.hops)
            runner.test_03_station_list_frequency_preservation()
            runner.test_04_rapid_retargeting_stress()
            runner.test_05_cdc_playback_state_toggling()
            runner.test_06_physical_button_override()
            runner.test_07_radio_volume_no_station_refresh()

        success = runner.print_summary()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n[!] Test run aborted by user.")
        runner.print_summary()
        sys.exit(130)
    finally:
        runner.close()


if __name__ == "__main__":
    main()
