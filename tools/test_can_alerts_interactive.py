#!/usr/bin/env python3
"""
Interactive PSA CAN Alert Test Runner (SocketCAN)
=================================================
Iterates through verified PSA CAN2004 alert messages from doc/RT4_CAN_ALERTS_0x1A1.csv,
broadcasts frame 0x1A1 cyclically on SocketCAN (default: can0) at 200ms intervals,
and prompts the tester to confirm whether the Head Unit displays the expected alert text.

Controls:
  [SPACE] or [y]  -> OK (PASS) - advances to next alert
  [n]             -> NOT OK (FAIL) - prompt for observed text/note, advances to next
  [r]             -> Retransmit current alert frame
  [s]             -> Skip alert
  [q]             -> Quit and generate report immediately

At the end of the test session, a comprehensive Markdown and terminal report is generated.
"""

import sys
import os
import csv
import time
import socket
import struct
import select
import termios
import tty
import threading
import argparse
from datetime import datetime

CAN_FRAME_FMT = "=IB3x8s"
CAN_ALERT_FRAME_ID = 0x1A1
DEFAULT_CYCLE_INTERVAL = 0.200  # 200 ms (standard PSA BSI 0x1A1 cycle)


def find_repo_root():
    cur = os.path.dirname(os.path.abspath(__file__))
    while cur and cur != "/":
        if os.path.exists(os.path.join(cur, "doc", "RT4_CAN_ALERTS_0x1A1.csv")):
            return cur
        cur = os.path.dirname(cur)
    return os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class SocketCanTransmitter:
    def __init__(self, interface: str):
        self.interface = interface
        self.sock = None
        self._init_socket()

    def _init_socket(self):
        try:
            self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.sock.bind((self.interface,))
        except Exception as e:
            print(f"\n[ERROR] Failed to bind to SocketCAN interface '{self.interface}': {e}")
            print(f"[HINT]  Ensure interface is up: 'sudo ip link set {self.interface} up type can bitrate 125000'")
            print(f"[HINT]  Or for virtual CAN: 'sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up'\n")
            sys.exit(1)

    def send_frame(self, can_id: int, data: bytes):
        dlc = len(data)
        padded_data = data.ljust(8, b"\x00")
        frame = struct.pack(CAN_FRAME_FMT, can_id, dlc, padded_data)
        try:
            self.sock.send(frame)
        except OSError as e:
            print(f"\n[WARN] Failed to send CAN frame: {e}")

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass


class CyclicAlertSender:
    """Sends 0x1A1 alert frames continuously in the background to simulate living BSI."""
    def __init__(self, transmitter: SocketCanTransmitter, interval_sec: float = DEFAULT_CYCLE_INTERVAL):
        self.transmitter = transmitter
        self.interval = interval_sec
        self.current_data = None
        self._running = False
        self._thread = None
        self._lock = threading.Lock()

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def set_alert(self, data: bytes):
        with self._lock:
            self.current_data = data

    def clear_alert(self):
        with self._lock:
            self.current_data = bytes([0x00] * 8)
        # Send clearing frame immediately
        self.transmitter.send_frame(CAN_ALERT_FRAME_ID, bytes([0x00] * 8))

    def stop(self):
        self._running = False
        self.clear_alert()
        if self._thread:
            self._thread.join(timeout=1.0)

    def _run(self):
        while self._running:
            with self._lock:
                data = self.current_data
            if data is not None:
                self.transmitter.send_frame(CAN_ALERT_FRAME_ID, data)
            time.sleep(self.interval)


def parse_severity(sev_str: str) -> int:
    """Maps CSV severity label to PSA priority integer (0=INFO, 1=SERVICE, 2=STOP, 3=BELT)."""
    s = sev_str.upper()
    if "STOP" in s or "(2)" in s:
        return 2
    if "SERVICE" in s or "(1)" in s:
        return 1
    if "BELT" in s or "(3)" in s:
        return 3
    return 0


def build_alert_payload(can_id: int, severity: int, sound: int = 1, door_mask: int = 0, param: int = 0) -> bytes:
    """Builds standard 8-byte 0x1A1 CAN payload."""
    b0 = 0x80 | ((can_id >> 8) & 0x7F)
    b1 = can_id & 0xFF
    b2 = 0x80 | ((severity & 0x07) << 4) | (sound & 0x0F)
    b3 = door_mask & 0xFF
    b4 = param & 0xFF
    return bytes([b0, b1, b2, b3, b4, 0x00, 0x00, 0x00])


def load_alerts_from_csv(csv_path: str, include_inactive: bool = False):
    alerts = []
    if not os.path.exists(csv_path):
        print(f"[ERROR] CSV file not found: {csv_path}")
        sys.exit(1)

    with open(csv_path, mode="r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            can_id_hex = row.get("CAN_Alarm_ID_Hex", "").strip()
            if not can_id_hex or can_id_hex == "0xFFFF" or can_id_hex == "-":
                continue
            
            try:
                can_id = int(can_id_hex, 16)
            except ValueError:
                continue

            active_state = row.get("Active_State", "").strip()
            if not include_inactive and active_state.lower() != "active":
                continue

            alert_text = row.get("English_Alert_String", "").strip()
            sev_str = row.get("Severity", "").strip()
            category = row.get("Category_Description", "").strip()
            idx_hex = row.get("Alarm_Index_Hex", "").strip()
            hiworld_code = row.get("Hiworld_Wire_Code_Hex", "").strip()

            alerts.append({
                "can_id": can_id,
                "can_id_hex": f"0x{can_id:04X}",
                "alarm_index": idx_hex,
                "severity_str": sev_str,
                "severity": parse_severity(sev_str),
                "text": alert_text,
                "category": category,
                "hiworld_code": hiworld_code,
                "active_state": active_state,
            })
    return alerts


def get_single_key():
    """Reads a single keypress in raw terminal mode."""
    if not sys.stdin.isatty():
        ch = sys.stdin.read(1)
        return ch if ch else "q"

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        ch = sys.stdin.read(1)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def prompt_string(prompt_msg: str) -> str:
    """Reads a full string in normal cooked terminal mode."""
    sys.stdout.write(prompt_msg)
    sys.stdout.flush()
    try:
        return sys.stdin.readline().strip()
    except Exception:
        return ""


def main():
    parser = argparse.ArgumentParser(description="Interactive PSA CAN Alert Text Verification Tool")
    parser.add_argument("-i", "--interface", default="can0", help="SocketCAN interface name (default: can0)")
    parser.add_argument("-f", "--file", default=None, help="Path to RT4_CAN_ALERTS_0x1A1.csv")
    parser.add_argument("--all", action="store_true", help="Include Inactive alert entries from table")
    parser.add_argument("--id", default=None, help="Test only a single specific CAN ID (e.g. 0x0088 or 136)")
    parser.add_argument("--start", default=None, help="Start from specific CAN ID (e.g. 0x0088)")
    parser.add_argument("--interval", type=float, default=DEFAULT_CYCLE_INTERVAL, help="Broadcast cycle interval in seconds (default: 0.2)")
    args = parser.parse_args()

    repo_root = find_repo_root()
    csv_path = args.file or os.path.join(repo_root, "doc", "RT4_CAN_ALERTS_0x1A1.csv")

    print(f"Loading alerts from: {csv_path}")
    alerts = load_alerts_from_csv(csv_path, include_inactive=args.all)
    if not alerts:
        print("[ERROR] No alert entries loaded!")
        sys.exit(1)

    # Filter single ID if specified
    if args.id:
        target_id = int(args.id, 16) if args.id.startswith("0x") or args.id.startswith("0X") else int(args.id)
        alerts = [a for a in alerts if a["can_id"] == target_id]
        if not alerts:
            print(f"[ERROR] Alert ID {args.id} not found in database!")
            sys.exit(1)

    # Fast forward to start ID if specified
    if args.start:
        start_id = int(args.start, 16) if args.start.startswith("0x") or args.start.startswith("0X") else int(args.start)
        idx_found = next((i for i, a in enumerate(alerts) if a["can_id"] == start_id), None)
        if idx_found is not None:
            alerts = alerts[idx_found:]
        else:
            print(f"[WARN] Start ID {args.start} not found, starting from beginning.")

    total_alerts = len(alerts)
    print(f"Total alerts queued for test: {total_alerts}")
    print(f"Connecting to SocketCAN interface: {args.interface}")

    transmitter = SocketCanTransmitter(args.interface)
    cyclic_sender = CyclicAlertSender(transmitter, interval_sec=args.interval)
    cyclic_sender.start()

    results = []
    session_start_time = datetime.now()

    print("\n" + "=" * 78)
    print("  PSA CAN2004 ALERT VERIFICATION TEST STARTED")
    print("  Commands: [SPACE]/[y] = PASS | [n] = FAIL (with note) | [r] = RETRY | [s] = SKIP | [q] = QUIT")
    print("=" * 78 + "\n")

    try:
        for idx, alert in enumerate(alerts, start=1):
            can_id = alert["can_id"]
            can_id_hex = alert["can_id_hex"]
            expected_text = alert["text"]
            severity_str = alert["severity_str"]
            category = alert["category"]

            # Build 8-byte payload
            payload = build_alert_payload(can_id, alert["severity"], sound=1)
            payload_hex = "".join(f"{b:02X}" for b in payload)

            # Start cyclic broadcast
            cyclic_sender.set_alert(payload)

            while True:
                # Clear terminal line or print clean header
                print(f"\n--- [{idx}/{total_alerts}] Alert ID: {can_id_hex} ({can_id:d}) ---")
                print(f"  Frame:       1A1#{payload_hex} (Cyclic {int(args.interval*1000)}ms)")
                print(f"  Severity:    {severity_str}")
                print(f"  Expected:    \"{expected_text}\"")
                if category:
                    print(f"  Category:    {category}")
                sys.stdout.write("  Action [SPACE/y = OK, n = FAIL, r = Retransmit, s = Skip, q = Quit]: ")
                sys.stdout.flush()

                key = get_single_key()
                print(f" '{key}'" if key not in ("\r", "\n", " ") else (" [SPACE]" if key == " " else " [ENTER]"))

                if key in (" ", "y", "Y", "\r", "\n"):
                    results.append({
                        "can_id": can_id_hex,
                        "text": expected_text,
                        "status": "PASS",
                        "note": "",
                        "severity": severity_str
                    })
                    print("  -> RESULT: ✅ PASS")
                    break
                elif key in ("n", "N"):
                    note = prompt_string("  Enter observed text or failure reason (optional): ")
                    results.append({
                        "can_id": can_id_hex,
                        "text": expected_text,
                        "status": "FAIL",
                        "note": note if note else "Incorrect string or no display",
                        "severity": severity_str
                    })
                    print(f"  -> RESULT: ❌ FAIL (Note: {note or 'No text entered'})")
                    break
                elif key in ("r", "R"):
                    print("  -> Retransmitting frame...")
                    cyclic_sender.set_alert(payload)
                    continue
                elif key in ("s", "S"):
                    results.append({
                        "can_id": can_id_hex,
                        "text": expected_text,
                        "status": "SKIPPED",
                        "note": "Skipped by operator",
                        "severity": severity_str
                    })
                    print("  -> RESULT: ⏩ SKIPPED")
                    break
                elif key in ("q", "Q"):
                    print("\n[INFO] Aborting test session per user request.")
                    raise KeyboardInterrupt
                else:
                    print("  Unknown key. Press [SPACE] for OK, [n] for Fail, [q] to Quit.")

            # Clear alert on vehicle display before moving to next item
            cyclic_sender.clear_alert()
            time.sleep(0.4)

    except KeyboardInterrupt:
        print("\n\nTest session stopped.")
    finally:
        cyclic_sender.stop()
        transmitter.close()

    # Generate Report
    session_end_time = datetime.now()
    duration = str(session_end_time - session_start_time).split(".")[0]

    pass_count = sum(1 for r in results if r["status"] == "PASS")
    fail_count = sum(1 for r in results if r["status"] == "FAIL")
    skip_count = sum(1 for r in results if r["status"] == "SKIPPED")
    total_tested = len(results)

    print("\n" + "=" * 78)
    print("                       VERIFICATION SUMMARY REPORT")
    print("=" * 78)
    print(f" Interface:        {args.interface}")
    print(f" Total Tested:     {total_tested} / {total_alerts}")
    print(f" Passed (OK):      {pass_count} ({(pass_count/total_tested*100) if total_tested else 0:.1f}%)")
    print(f" Failed (FAIL):    {fail_count}")
    print(f" Skipped:          {skip_count}")
    print(f" Duration:         {duration}")
    print("=" * 78)

    if fail_count > 0:
        print("\nFAILED ALERTS DETAIL:")
        for r in results:
            if r["status"] == "FAIL":
                print(f"  * {r['can_id']}: Expected \"{r['text']}\" -> Note: {r['note']}")

    # Write Markdown Report File
    timestamp_str = session_start_time.strftime("%Y-%m-%d_%H-%M-%S")
    report_filename = f"ALERT_TEST_REPORT_{timestamp_str}.md"
    report_path = os.path.join(repo_root, "doc", report_filename)

    try:
        with open(report_path, "w", encoding="utf-8") as rf:
            rf.write(f"# PSA CAN Alert Verification Test Report\n\n")
            rf.write(f"- **Date / Time:** {session_start_time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            rf.write(f"- **Interface:** `{args.interface}`\n")
            rf.write(f"- **Duration:** {duration}\n")
            rf.write(f"- **Total Tested:** {total_tested} of {total_alerts}\n")
            rf.write(f"- **Passed (OK):** {pass_count}\n")
            rf.write(f"- **Failed:** {fail_count}\n")
            rf.write(f"- **Skipped:** {skip_count}\n\n")

            rf.write("## Test Results Table\n\n")
            rf.write("| CAN ID | Status | Severity | Expected Alert Text | Notes / Observed Text |\n")
            rf.write("|:---:|:---:|:---:|:---|:---|\n")
            for r in results:
                status_icon = "✅ PASS" if r["status"] == "PASS" else ("❌ FAIL" if r["status"] == "FAIL" else "⏩ SKIP")
                rf.write(f"| `{r['can_id']}` | **{status_icon}** | {r['severity']} | {r['text']} | {r['note']} |\n")

            if fail_count > 0:
                rf.write("\n## Action Items / Discrepancies\n\n")
                for r in results:
                    if r["status"] == "FAIL":
                        rf.write(f"- **`{r['can_id']}`**: Expected `{r['text']}` &mdash; *Observed:* `{r['note']}`\n")

        print(f"\n[SUCCESS] Detailed report saved to: doc/{report_filename}\n")
    except Exception as e:
        print(f"[ERROR] Failed to write report file: {e}")


if __name__ == "__main__":
    main()
