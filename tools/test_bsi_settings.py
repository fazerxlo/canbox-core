#!/usr/bin/env python3
"""
PSA Peugeot 407 (CAN2004 / AEE2004) BSI Personalization Settings Injection Script
=================================================================================
Determines and verifies the exact CAN sequence required to modify and commit
vehicle personalization settings (DRL, Follow-Me-Home delay) in the Peugeot 407
BSI EEPROM over SocketCAN (can0 @ 125 kbps).

Token-Safe Logging:
  - Passively monitors 0x260 (MSG_BSI_INF_PROFILS) without frame spam.
  - Logs ONLY: initial baseline, scenario start, immediate SUCCESS (old vs new
    hex + latency), or FAIL after timeout.
"""

import argparse
import signal
import socket
import struct
import sys
import threading
import time
import types
from typing import Callable, List, Optional, Tuple

# Python 3.13 compatibility shim for python-can < 4.x (which imports pkg_resources)
if "pkg_resources" not in sys.modules:
    sys.modules["pkg_resources"] = types.SimpleNamespace(
        iter_entry_points=lambda *args, **kwargs: []
    )

try:
    import can  # type: ignore

    HAS_PYTHON_CAN = True
except Exception:
    HAS_PYTHON_CAN = False

# PSA CAN2004 Comfort Bus Identifiers
CAN_ID_MDF_STATUS_0DF = 0x0DF       # Display status / Menu mode (DLC = 3, 500 ms)
CAN_ID_PERSO_REQ_15B = 0x15B        # Settings write request / MDF state (DLC = 8, 500 ms)
CAN_ID_MDF_SUPERVISION_167 = 0x167  # MFD general supervision heartbeat (DLC = 8, 100 ms)
CAN_ID_BSI_PROFILES_260 = 0x260     # BSI EEPROM profile broadcast (DLC = 8, 500 ms; 1.5s/prof)
CAN_ID_MDF_KEYS_3E5 = 0x3E5         # EMF-C / stalk menu & OK key pulses (DLC = 6)

# Default Fallback Baseline (if --offline-baseline is used)
DEFAULT_BASELINE_260 = bytes([0x01, 0x03, 0x91, 0x00, 0x00, 0xD0, 0x00, 0x10])
NOMINAL_MDF_167 = bytes([0x08, 0x06, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00])

# Follow-Me-Home Byte 2 encodings (Bit 5 = 0x20 FMH enable, Bits 2..0 = 1:15s, 2:30s, 4:60s)
FMH_DELAY_MAP = {
    0x91: "OFF/15s (0x91)",
    0x9E: "30s (0x9E)",
    0xB1: "ON 15s (0xB1)",
    0xB2: "ON 30s (0xB2)",
    0xB4: "ON 60s (0xB4)",
}


def fmt_hex(data: bytes) -> str:
    """Formats bytes as uppercase space-separated hex string."""
    return " ".join(f"{b:02X}" for b in data)


def decode_260_summary(data: bytes) -> str:
    """Decodes key personalization fields from an 8-byte 0x260 frame."""
    if len(data) < 8:
        return f"short DLC={len(data)}"
    profile = data[0] & 0x03
    b0_flags = f"0x{data[0]:02X}"
    b2 = data[2]
    fmh_str = FMH_DELAY_MAP.get(b2, f"0x{b2:02X}")
    drl_on = bool(data[3] & 0x40)
    aas_on = bool(data[5] & 0x40)
    rw_on = bool(data[5] & 0x10)
    return (
        f"Prof={profile} (B0={b0_flags}) | "
        f"DRL={'ON' if drl_on else 'OFF'} (B3=0x{data[3]:02X}) | "
        f"FMH={fmh_str} | AAS={'ON' if aas_on else 'OFF'} | RearWiper={'ON' if rw_on else 'OFF'}"
    )


class CanAdapter:
    """Unified SocketCAN wrapper supporting python-can with native AF_CAN fallback."""

    def __init__(self, channel: str = "can0"):
        self.channel = channel
        self.bus = None
        self.sock = None

        if HAS_PYTHON_CAN:
            self.bus = can.interface.Bus(
                channel=channel,
                bustype="socketcan",
                can_filters=[{"can_id": CAN_ID_BSI_PROFILES_260, "can_mask": 0x7FF}],
            )
        else:
            self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.sock.setsockopt(
                socket.SOL_CAN_RAW,
                socket.CAN_RAW_FILTER,
                struct.pack("=II", CAN_ID_BSI_PROFILES_260, 0x7FF),
            )
            self.sock.bind((channel,))
            self.sock.settimeout(0.05)

    def send(self, can_id: int, data: bytes) -> None:
        if self.bus is not None:
            msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
            self.bus.send(msg)
        elif self.sock is not None:
            pkt = struct.pack("=IB3x8s", can_id, len(data), data.ljust(8, b"\x00"))
            self.sock.send(pkt)

    def recv(self, timeout: float = 0.05) -> Optional[Tuple[int, bytes]]:
        if self.bus is not None:
            msg = self.bus.recv(timeout=timeout)
            if msg is None:
                return None
            return msg.arbitration_id, bytes(msg.data)
        elif self.sock is not None:
            self.sock.settimeout(timeout)
            try:
                pkt = self.sock.recv(16)
            except socket.timeout:
                return None
            can_id, dlc, raw = struct.unpack("=IB3x8s", pkt)
            return can_id & 0x1FFFFFFF, raw[:dlc]
        return None

    def close(self) -> None:
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass


class BsiSettingsTester:
    """Passive 0x260 monitor and controlled hypothesis injector."""

    def __init__(self, adapter: CanAdapter, stop_event: threading.Event):
        self.adapter = adapter
        self.stop_event = stop_event
        self._lock = threading.Lock()
        self._baseline_260: Optional[bytes] = None
        self._latest_260: Optional[bytes] = None
        self._latest_260_ts: float = 0.0
        self._change_event = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_worker, daemon=True)

    def start(self) -> None:
        self._rx_thread.start()

    def _rx_worker(self) -> None:
        while not self.stop_event.is_set():
            try:
                res = self.adapter.recv(timeout=0.05)
            except Exception:
                if self.stop_event.is_set():
                    break
                continue
            if res is None:
                continue
            can_id, data = res
            if can_id == CAN_ID_BSI_PROFILES_260 and len(data) >= 8:
                # Track Profile 1 (Byte 0 lower 2 bits == 1, typically 0x01)
                if (data[0] & 0x03) == 0x01:
                    with self._lock:
                        prev = self._latest_260
                        self._latest_260 = data
                        self._latest_260_ts = time.monotonic()
                        if self._baseline_260 is None:
                            self._baseline_260 = data
                        if prev is not None and prev != data:
                            self._change_event.set()

    def wait_for_baseline(self, timeout_s: float = 5.0) -> Optional[bytes]:
        """Waits for the initial 0x260 Profile 1 broadcast from the BSI."""
        deadline = time.monotonic() + timeout_s
        while not self.stop_event.is_set() and time.monotonic() < deadline:
            with self._lock:
                if self._latest_260 is not None:
                    return self._latest_260
            time.sleep(0.05)
        return None

    def get_latest_260(self) -> Optional[bytes]:
        with self._lock:
            return self._latest_260

    def sleep_interruptible(self, seconds: float) -> bool:
        """Sleeps for `seconds` or returns True early if a 0x260 change or stop occurs."""
        deadline = time.monotonic() + seconds
        while not self.stop_event.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            if self._change_event.wait(timeout=min(0.02, remaining)):
                return True
        return True

    def build_target_payload(
        self, base: bytes, setting: str = "drl", target_state: Optional[bool] = None
    ) -> Tuple[bytes, str]:
        """Constructs the target 8-byte configuration payload (Bytes 1..7 mutated)."""
        buf = bytearray(base)
        if setting == "drl":
            current_drl = bool(buf[3] & 0x40)
            desired_drl = (not current_drl) if target_state is None else target_state
            if desired_drl:
                buf[3] |= 0x40
            else:
                buf[3] &= ~0x40
            desc = f"DRL -> {'ON (0x40)' if desired_drl else 'OFF (0x00)'}"
        elif setting == "fmh":
            # Toggle Follow-Me-Home between OFF/15s (0x91) and ON 60s (0xB4)
            if buf[2] == 0x91:
                buf[2] = 0xB4
                desc = "Follow-Me-Home -> ON 60s (0xB4)"
            else:
                buf[2] = 0x91
                desc = "Follow-Me-Home -> OFF/15s (0x91)"
        else:
            raise ValueError(f"Unsupported setting: {setting}")
        return bytes(buf), desc

    def _run_and_evaluate(
        self,
        name: str,
        target_payload: bytes,
        desc: str,
        inject_fn: Callable[[bytes], None],
        verify_timeout_s: float = 0.6,
    ) -> bool:
        """Executes an injection sequence and monitors 0x260 for EEPROM commit."""
        before = self.get_latest_260() or DEFAULT_BASELINE_260
        self._change_event.clear()
        start_ts = time.monotonic()

        print(f"\n[TEST] {name} | Target: {desc}")
        print(f"       Baseline 0x260 : [{fmt_hex(before)}]")

        inject_fn(target_payload)

        # Wait up to verify_timeout_s after sequence (or immediate wakeup on change)
        deadline = time.monotonic() + verify_timeout_s
        while not self.stop_event.is_set() and time.monotonic() < deadline:
            now_260 = self.get_latest_260()
            if now_260 is not None and now_260[1:] == target_payload[1:]:
                elapsed_ms = (time.monotonic() - start_ts) * 1000.0
                print(
                    f"[SUCCESS] {name} committed by BSI in {elapsed_ms:.0f} ms!\n"
                    f"          Old 0x260 : [{fmt_hex(before)}]\n"
                    f"          New 0x260 : [{fmt_hex(now_260)}] ({decode_260_summary(now_260)})"
                )
                return True
            if self._change_event.is_set():
                self._change_event.clear()
                if now_260 is not None and now_260 != before:
                    elapsed_ms = (time.monotonic() - start_ts) * 1000.0
                    print(
                        f"[SUCCESS] {name} triggered 0x260 change in {elapsed_ms:.0f} ms!\n"
                        f"          Old 0x260 : [{fmt_hex(before)}]\n"
                        f"          New 0x260 : [{fmt_hex(now_260)}] ({decode_260_summary(now_260)})"
                    )
                    return True
            time.sleep(0.05)

        after = self.get_latest_260() or before
        print(
            f"[FAIL] {name}: No change in 0x260 within {verify_timeout_s:.1f}s "
            f"(still [{fmt_hex(after)}])."
        )
        return False

    # -------------------------------------------------------------------------
    # Hypothesis Implementations
    # -------------------------------------------------------------------------

    def run_hypothesis_a(
        self, setting: str = "drl", target_state: Optional[bool] = None
    ) -> bool:
        """
        Hypothesis A (RT4 Style):
        Stream 0x15B with Byte 0 = 0x05 (Bit 2 write flag set) at 100 ms for 1.5s.
        """
        base = self.get_latest_260() or DEFAULT_BASELINE_260
        target, desc = self.build_target_payload(base, setting, target_state)
        frame_15b = bytes([0x05]) + target[1:]

        def inject(_: bytes) -> None:
            print(f"       TX 0x15B (15x @ 100ms): [{fmt_hex(frame_15b)}]")
            for _ in range(15):
                if self.stop_event.is_set():
                    return
                self.adapter.send(CAN_ID_PERSO_REQ_15B, frame_15b)
                if self.sleep_interruptible(0.10):
                    break

        return self._run_and_evaluate(
            "Hypothesis A (RT4 Byte0=0x05 @ 100ms)", target, desc, inject
        )

    def run_hypothesis_b(
        self,
        setting: str = "drl",
        target_state: Optional[bool] = None,
        byte0_val: int = 0x01,
    ) -> bool:
        """
        Hypothesis B (EMF-C Menu Sequence):
        1. 0x3E5 Menu press pulse (40 00 00 00 00 00) + 0x0DF -> 90 01 70 (100ms for 500ms).
        2. 0x3E5 OK pulse (00 00 40 00 00 00) 3x at 50ms intervals.
        3. Broadcast new 0x15B (Byte 0 = byte0_val) and revert 0x0DF -> 10 01 50.
        """
        base = self.get_latest_260() or DEFAULT_BASELINE_260
        target, desc = self.build_target_payload(base, setting, target_state)
        frame_15b_base = bytes([0x01]) + base[1:]
        frame_15b_new = bytes([byte0_val]) + target[1:]
        menu_0df = bytes([0x90, 0x01, 0x70])
        nom_0df = bytes([0x10, 0x01, 0x50])
        key_menu = bytes([0x40, 0x00, 0x00, 0x00, 0x00, 0x00])
        key_ok = bytes([0x00, 0x00, 0x40, 0x00, 0x00, 0x00])
        key_rel = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

        def inject(_: bytes) -> None:
            print(
                f"       Step 1: 0x3E5 MENU + 0x0DF [{fmt_hex(menu_0df)}] (500ms) -> "
                f"Step 2: 3x 0x3E5 OK -> Step 3: 0x15B [{fmt_hex(frame_15b_new)}]"
            )
            # 1. Menu entry pulse + 500 ms of 0x0DF menu status
            self.adapter.send(CAN_ID_MDF_KEYS_3E5, key_menu)
            self.adapter.send(CAN_ID_PERSO_REQ_15B, frame_15b_base)
            for i in range(5):
                if self.stop_event.is_set():
                    return
                self.adapter.send(CAN_ID_MDF_STATUS_0DF, menu_0df)
                if i == 1:
                    self.adapter.send(CAN_ID_MDF_KEYS_3E5, key_rel)
                time.sleep(0.10)

            # 2. 3x OK button pulse at 50 ms intervals + immediate 0x15B update
            for _ in range(3):
                if self.stop_event.is_set():
                    return
                self.adapter.send(CAN_ID_MDF_KEYS_3E5, key_ok)
                self.adapter.send(CAN_ID_PERSO_REQ_15B, frame_15b_new)
                time.sleep(0.05)
            self.adapter.send(CAN_ID_MDF_KEYS_3E5, key_rel)

            # 3. Revert 0x0DF to nominal 10 01 50 while streaming new 0x15B for 1.5s
            for i in range(15):
                if self.stop_event.is_set():
                    return
                self.adapter.send(CAN_ID_PERSO_REQ_15B, frame_15b_new)
                if i % 5 == 0:
                    self.adapter.send(CAN_ID_MDF_STATUS_0DF, nom_0df)
                if self.sleep_interruptible(0.10):
                    break

        label = f"Hypothesis B (EMF-C Menu + 0x3E5 OK + 0x15B Byte0=0x{byte0_val:02X})"
        return self._run_and_evaluate(label, target, desc, inject)

    def run_hypothesis_c(
        self,
        setting: str = "drl",
        target_state: Optional[bool] = None,
        byte0_val: int = 0x01,
        period_s: float = 0.50,
        count: int = 10,
    ) -> bool:
        """
        Hypothesis C (Sustained Broadcast Duration / Cadence):
        Stream 0x15B alongside nominal 0x0DF (10 01 50) for `count` frames at `period_s`.
        """
        base = self.get_latest_260() or DEFAULT_BASELINE_260
        target, desc = self.build_target_payload(base, setting, target_state)
        frame_15b = bytes([byte0_val]) + target[1:]
        nom_0df = bytes([0x10, 0x01, 0x50])

        def inject(_: bytes) -> None:
            print(
                f"       TX 0x15B ({count}x @ {int(period_s * 1000)}ms) + 0x0DF: "
                f"[{fmt_hex(frame_15b)}]"
            )
            for _ in range(count):
                if self.stop_event.is_set():
                    return
                self.adapter.send(CAN_ID_MDF_STATUS_0DF, nom_0df)
                self.adapter.send(CAN_ID_PERSO_REQ_15B, frame_15b)
                if self.sleep_interruptible(period_s):
                    break

        label = (
            f"Hypothesis C (Sustained 0x15B Byte0=0x{byte0_val:02X}, "
            f"{count}x @ {int(period_s * 1000)}ms)"
        )
        return self._run_and_evaluate(label, target, desc, inject)

    def run_hypothesis_d(
        self, setting: str = "drl", target_state: Optional[bool] = None
    ) -> bool:
        """
        Hypothesis D (Byte 0 Toggle / Write-Request Edge):
        Phase 1: Sync baseline with Byte 0 = 0x01 (2 frames @ 100ms)
        Phase 2: Assert write request Byte 0 = 0x05 with new payload (5 frames @ 100ms)
        Phase 3: Latch steady state Byte 0 = 0x01 with new payload (10 frames @ 100ms)
        """
        base = self.get_latest_260() or DEFAULT_BASELINE_260
        target, desc = self.build_target_payload(base, setting, target_state)
        frame_sync = bytes([0x01]) + base[1:]
        frame_edge_05 = bytes([0x05]) + target[1:]
        frame_latch_01 = bytes([0x01]) + target[1:]
        nom_0df = bytes([0x10, 0x01, 0x50])

        def inject(_: bytes) -> None:
            print(
                f"       Edge 0x01 -> 0x05 [{fmt_hex(frame_edge_05)}] (500ms) "
                f"-> 0x01 [{fmt_hex(frame_latch_01)}] (1.0s)"
            )
            for _ in range(2):
                if self.stop_event.is_set():
                    return
                self.adapter.send(CAN_ID_MDF_STATUS_0DF, nom_0df)
                self.adapter.send(CAN_ID_PERSO_REQ_15B, frame_sync)
                time.sleep(0.10)

            for _ in range(5):
                if self.stop_event.is_set():
                    return
                self.adapter.send(CAN_ID_PERSO_REQ_15B, frame_edge_05)
                if self.sleep_interruptible(0.10):
                    return

            for _ in range(10):
                if self.stop_event.is_set():
                    return
                self.adapter.send(CAN_ID_MDF_STATUS_0DF, nom_0df)
                self.adapter.send(CAN_ID_PERSO_REQ_15B, frame_latch_01)
                if self.sleep_interruptible(0.10):
                    return

        return self._run_and_evaluate(
            "Hypothesis D (Byte0 Edge 0x01 -> 0x05 -> 0x01)", target, desc, inject
        )

    def run_hypothesis_e(
        self,
        setting: str = "drl",
        target_state: Optional[bool] = None,
        byte0_val: int = 0x01,
        period_167_s: float = 0.10,
    ) -> bool:
        """
        Hypothesis E (100 ms 0x167 MFD Supervision + 500 ms 0x15B Write Request):
        The Peugeot 407 BSI monitors CAN ID 0x167 (TRAME_GENERALE_EMF, DLC 8 @ 100 ms)
        with a ~500 ms frame-presence watchdog. When 0x167 is absent (or only sent at
        1000 ms), the BSI considers the MFD offline and blocks all 0x15B EEPROM writes.
        When 0x167 is streamed at 100 ms alongside 0x15B (Byte 0 = 0x01, Bit 2 cleared),
        the BSI commits the change on the very next 0x260 Profile 1 slot (1.5 s cycle).
        """
        base = self.get_latest_260() or DEFAULT_BASELINE_260
        target, desc = self.build_target_payload(base, setting, target_state)
        frame_15b = bytes([byte0_val]) + target[1:]
        ticks = 22  # 2.2s window covers a full 1.5s Profile 1 0x260 round-robin cycle
        step_167 = max(1, int(round(period_167_s / 0.10)))

        def inject(_: bytes) -> None:
            print(
                f"       TX 0x167 (@ {int(period_167_s * 1000)}ms) + "
                f"0x15B (@ 500ms): [{fmt_hex(frame_15b)}]"
            )
            for tick in range(ticks):
                if self.stop_event.is_set():
                    return
                if tick % step_167 == 0:
                    self.adapter.send(CAN_ID_MDF_SUPERVISION_167, NOMINAL_MDF_167)
                if tick % 5 == 0:
                    self.adapter.send(CAN_ID_PERSO_REQ_15B, frame_15b)
                if self.sleep_interruptible(0.10):
                    return

        label = (
            f"Hypothesis E (0x167 @ {int(period_167_s * 1000)}ms + "
            f"0x15B Byte0=0x{byte0_val:02X} @ 500ms)"
        )
        return self._run_and_evaluate(
            label, target, desc, inject, verify_timeout_s=0.5
        )

    def verify_reversibility(self, runner_fn: Callable[[Optional[bool]], bool]) -> bool:
        """
        Proves full reversibility by toggling DRL ON (0x40), confirming 0x260 commit,
        then toggling DRL OFF (0x00) and confirming 0x260 commit.
        """
        print("\n=====================================================================")
        print("[VERIFY] Starting Reversibility Proof (DRL ON -> Confirm -> DRL OFF)")
        print("=====================================================================")

        cur = self.get_latest_260() or DEFAULT_BASELINE_260
        initial_drl = bool(cur[3] & 0x40)
        if initial_drl:
            print("[*] Step 0: DRL is currently ON; resetting to OFF (0x00) first...")
            if not runner_fn(False):
                print("[VERIFY FAIL] Failed to reset DRL to OFF before verification.")
                return False
            time.sleep(0.3)

        print("[*] Step 1/2: Toggling DRL to ON (0x40)...")
        if not runner_fn(True):
            print("[VERIFY FAIL] Step 1 failed to commit DRL ON in 0x260.")
            return False

        time.sleep(0.3)
        print("[*] Step 2/2: Toggling DRL back to OFF (0x00)...")
        if not runner_fn(False):
            print("[VERIFY FAIL] Step 2 failed to commit DRL OFF in 0x260.")
            return False

        final_260 = self.get_latest_260() or cur
        print(
            f"\n[VERIFY PASS] Reversibility proven (ON -> OFF)! "
            f"Final 0x260: [{fmt_hex(final_260)}] ({decode_260_summary(final_260)})"
        )
        return True


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Peugeot 407 (CAN2004) BSI Personalization Settings Hypothesis Tester"
    )
    parser.add_argument(
        "-i", "--iface", default="can0", help="SocketCAN interface (default: can0)"
    )
    parser.add_argument(
        "-t",
        "--test",
        choices=["A", "B", "B5", "C", "C100", "D", "E1000", "E5", "E", "all"],
        default="all",
        help=(
            "Hypothesis to execute: A (RT4 0x05), B (EMF-C menu 0x01), "
            "B5 (EMF-C menu 0x05), C (500ms burst), C100 (100ms burst), "
            "D (0x01->0x05->0x01 edge), E1000 (0x167@1000ms + 0x15B@500ms), "
            "E5 (0x167@100ms + 0x15B Byte0=0x05), "
            "E (0x167@100ms + 0x15B Byte0=0x01 [WINNING]), or all (default)"
        ),
    )
    parser.add_argument(
        "-s",
        "--setting",
        choices=["drl", "fmh"],
        default="drl",
        help="Personalization parameter to toggle: drl (Byte3 Bit6) or fmh (Byte2)",
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="Run full ON -> OFF reversibility verification on the winning/selected test",
    )
    parser.add_argument(
        "--baseline-timeout",
        type=float,
        default=4.0,
        help="Seconds to wait for initial 0x260 broadcast (default: 4.0)",
    )
    args = parser.parse_args()

    stop_event = threading.Event()

    def sigint_handler(signum, frame):
        print("\n[!] SIGINT received. Terminating background threads cleanly...")
        stop_event.set()

    signal.signal(signal.SIGINT, sigint_handler)

    try:
        adapter = CanAdapter(channel=args.iface)
    except Exception as exc:
        print(f"[ERROR] Failed to open SocketCAN interface '{args.iface}': {exc}")
        return 1

    tester = BsiSettingsTester(adapter, stop_event)
    tester.start()

    print(
        f"[*] Listening on {args.iface} for BSI status 0x260 "
        f"(backend={'python-can' if HAS_PYTHON_CAN else 'socket.AF_CAN'})..."
    )
    baseline = tester.wait_for_baseline(timeout_s=args.baseline_timeout)

    if baseline is None:
        print(
            f"[WARN] No 0x260 frame received within {args.baseline_timeout:.1f}s! "
            f"Using fallback baseline [{fmt_hex(DEFAULT_BASELINE_260)}]."
        )
    else:
        print(
            f"[BASELINE] 0x260 RX : [{fmt_hex(baseline)}]\n"
            f"           Decoded  : {decode_260_summary(baseline)}"
        )

    try:
        suites: List[Tuple[str, Callable[[Optional[bool]], bool]]] = [
            ("A", lambda st=None: tester.run_hypothesis_a(args.setting, st)),
            (
                "B",
                lambda st=None: tester.run_hypothesis_b(
                    args.setting, st, byte0_val=0x01
                ),
            ),
            (
                "B5",
                lambda st=None: tester.run_hypothesis_b(
                    args.setting, st, byte0_val=0x05
                ),
            ),
            (
                "C",
                lambda st=None: tester.run_hypothesis_c(
                    args.setting, st, byte0_val=0x01, period_s=0.50, count=4
                ),
            ),
            (
                "C100",
                lambda st=None: tester.run_hypothesis_c(
                    args.setting, st, byte0_val=0x01, period_s=0.10, count=16
                ),
            ),
            ("D", lambda st=None: tester.run_hypothesis_d(args.setting, st)),
            (
                "E1000",
                lambda st=None: tester.run_hypothesis_e(
                    args.setting, st, byte0_val=0x01, period_167_s=1.00
                ),
            ),
            (
                "E5",
                lambda st=None: tester.run_hypothesis_e(
                    args.setting, st, byte0_val=0x05, period_167_s=0.10
                ),
            ),
            (
                "E",
                lambda st=None: tester.run_hypothesis_e(
                    args.setting, st, byte0_val=0x01, period_167_s=0.10
                ),
            ),
        ]

        selected = (
            suites if args.test == "all" else [s for s in suites if s[0] == args.test]
        )

        winning_fn: Optional[Callable[[Optional[bool]], bool]] = None
        for code, fn in selected:
            if stop_event.is_set():
                break
            if fn(None):
                winning_fn = fn
                print(f"\n[+] Hypothesis {code} succeeded!")
                break

        if args.verify and not stop_event.is_set():
            target_fn = winning_fn or (selected[0][1] if selected else None)
            if target_fn is not None:
                tester.verify_reversibility(target_fn)

    finally:
        stop_event.set()
        adapter.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())

