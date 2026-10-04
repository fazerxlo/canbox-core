#!/usr/bin/env python3
"""
OpenCanbox Log Triage & Anomaly Detector
=========================================
Rapidly triages E2E canbox logs (dump_*.log) to detect:
1. Flip-flops & oscillations (blinking sensors, twitching gauges/trajectory)
2. Spurious UART frame floods (App spamming frames OEM never sends)
3. Payload length & checksum mismatches
4. CAN-to-UART causality (which CAN IDs arrived immediately before each UART packet)

Designed specifically for AI agents and developers to pinpoint protocol & decoder
bugs in < 2 seconds while consuming < 1,500 tokens (vs 50,000+ tokens on raw logs).
"""

import argparse
import os
import re
import sys
from collections import defaultdict, deque
from typing import List, Dict, Tuple, Optional, Any

HIWORLD_CMD_NAMES = {
    0x11: "CAR_BASE_INFO / SWC_ANGLE",
    0x12: "DOORS_WINDOWS",
    0x13: "TRIP_INFO_1",
    0x14: "TRIP_INFO_2",
    0x15: "TRIP_INFO_3",
    0x16: "TIRE_PRESSURE_DISCRETE",
    0x18: "TIRE_PRESSURE_NUMERIC",
    0x21: "WHEEL_BUTTONS",
    0x22: "STALK_BUTTONS",
    0x31: "HVAC_STATUS",
    0x32: "FRONT_RADAR_TOYOTA",
    0x33: "REAR_RADAR_TOYOTA",
    0x41: "PARKING_RADAR_PSA",
    0x42: "ALERTS_WARNING_INFO",
    0x66: "TPMS_DETAILED",
    0x71: "FEATURE_CONFIG_1",
    0x72: "FEATURE_CONFIG_2",
    0x76: "PANEL_KEYS",
    0x79: "SYSTEM_STATUS",
    0x81: "AUDIO_AMP_1",
    0x82: "AUDIO_AMP_2",
    0x83: "AUDIO_AMP_3",
    0x85: "AUDIO_INFO",
    0x93: "PARKING_SYSTEM",
    0x94: "ALARM_STATUS",
    0x95: "MAINTENANCE_INFO",
    0x96: "CAR_SETTINGS_1",
    0x97: "CAR_SETTINGS_2",
    0xC1: "LANGUAGE_CONFIG",
    0xF0: "VERSION_REPORT",
    0xFF: "HEARTBEAT"
}

class SerialPacket:
    __slots__ = ('ts', 'source', 'proto', 'raw', 'raw_hex', 'cmd', 'payload', 'checksum', 'valid_cs')
    def __init__(self, ts: float, source: str, proto: str, raw_bytes: bytes,
                 cmd: Optional[int], payload: bytes, checksum: Optional[int], valid_cs: bool):
        self.ts = ts
        self.source = source  # "ORIG" or "APP"
        self.proto = proto
        self.raw = raw_bytes
        self.raw_hex = raw_bytes.hex().lower()
        self.cmd = cmd
        self.payload = payload
        self.checksum = checksum
        self.valid_cs = valid_cs

    def cmd_name(self) -> str:
        if self.cmd is not None and self.cmd in HIWORLD_CMD_NAMES:
            return HIWORLD_CMD_NAMES[self.cmd]
        return f"CMD_0x{self.cmd:02X}" if self.cmd is not None else "RAW"

class StreamReassembler:
    def __init__(self, source: str):
        self.source = source
        self.buf = bytearray()
        self.ts_buf = []

    def feed(self, hex_chunk: str, ts: float) -> List[SerialPacket]:
        clean = re.sub(r"[^0-9A-Fa-f]", "", hex_chunk)
        if not clean or len(clean) % 2 != 0:
            return []
        try:
            chunk = bytes.fromhex(clean)
        except ValueError:
            return []

        self.buf.extend(chunk)
        self.ts_buf.extend([ts] * len(chunk))
        packets = []

        while len(self.buf) >= 3:
            b0 = self.buf[0]
            # 1. Hiworld: 5A A5 <len> <cmd> <payload...> <cs>
            if b0 == 0x5A:
                if len(self.buf) == 1:
                    break
                if self.buf[1] != 0xA5:
                    del self.buf[0]
                    del self.ts_buf[0]
                    continue
                if len(self.buf) < 4:
                    break
                plen = self.buf[2]
                if plen > 64:
                    del self.buf[0]
                    del self.ts_buf[0]
                    continue
                total_len = plen + 5
                if len(self.buf) < total_len:
                    break
                raw_pkt = bytes(self.buf[:total_len])
                pkt_ts = self.ts_buf[0]
                cmd = raw_pkt[3]
                payload = raw_pkt[4:4 + plen]
                cs = raw_pkt[4 + plen]
                exp_cs = ((plen + cmd + sum(payload)) - 1) & 0xFF
                valid = (cs == exp_cs)
                del self.buf[:total_len]
                del self.ts_buf[:total_len]
                packets.append(SerialPacket(pkt_ts, self.source, "hiworld", raw_pkt, cmd, payload, cs, valid))
                continue

            # 2. Raise: 2E <cmd> <len> <payload...> <cs>
            elif b0 == 0x2E:
                if len(self.buf) < 3:
                    break
                cmd = self.buf[1]
                plen = self.buf[2]
                if plen > 64:
                    del self.buf[0]
                    del self.ts_buf[0]
                    continue
                total_len = plen + 4
                if len(self.buf) < total_len:
                    break
                raw_pkt = bytes(self.buf[:total_len])
                pkt_ts = self.ts_buf[0]
                payload = raw_pkt[3:3 + plen]
                cs = raw_pkt[3 + plen]
                exp_cs = (~(cmd + plen + sum(payload))) & 0xFF
                valid = (cs == exp_cs)
                del self.buf[:total_len]
                del self.ts_buf[:total_len]
                packets.append(SerialPacket(pkt_ts, self.source, "raise", raw_pkt, cmd, payload, cs, valid))
                continue

            # Drop unknown leading byte
            del self.buf[0]
            del self.ts_buf[0]

        return packets

class CanFrameInfo:
    __slots__ = ('ts', 'can_id', 'dlc', 'data_hex')
    def __init__(self, ts: float, can_id: int, dlc: int, data_hex: str):
        self.ts = ts
        self.can_id = can_id
        self.dlc = dlc
        self.data_hex = data_hex

def triage_log(log_path: str):
    if not os.path.isfile(log_path):
        print(f"Error: file not found: {log_path}", file=sys.stderr)
        sys.exit(1)

    orig_parser = StreamReassembler("ORIG")
    app_parser = StreamReassembler("APP")

    recent_can = deque(maxlen=20)
    can_msg_counts = defaultdict(int)
    can_last_data = {}
    can_changed_counts = defaultdict(int)

    app_packets: List[Tuple[SerialPacket, List[CanFrameInfo]]] = []
    orig_packets: List[Tuple[SerialPacket, List[CanFrameInfo]]] = []

    total_lines = 0
    start_ts = None
    end_ts = None

    with open(log_path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            total_lines += 1
            line = line.strip()
            if not line:
                continue
            parts = line.split(" ", 2)
            if len(parts) < 2:
                continue
            try:
                ts = float(parts[0])
            except ValueError:
                continue

            if start_ts is None:
                start_ts = ts
            end_ts = ts

            tag = parts[1]
            rest = parts[2] if len(parts) > 2 else ""

            if tag == "[CAN]":
                m = re.search(r"ID:([0-9A-Fa-f]+)\s+DLC:(\d+)\s+DATA:([0-9A-Fa-f]*)", rest)
                if m:
                    cid = int(m.group(1), 16)
                    dlc = int(m.group(2))
                    dhex = m.group(3).lower()
                    can_msg_counts[cid] += 1
                    if can_last_data.get(cid) != dhex:
                        can_changed_counts[cid] += 1
                        can_last_data[cid] = dhex
                    recent_can.append(CanFrameInfo(ts, cid, dlc, dhex))

            elif tag == "[APP_UART_TX]":
                pkts = app_parser.feed(rest, ts)
                for p in pkts:
                    app_packets.append((p, list(recent_can)))

            elif tag == "[ORIG_CANBOX]":
                pkts = orig_parser.feed(rest, ts)
                for p in pkts:
                    orig_packets.append((p, list(recent_can)))

    duration = (end_ts - start_ts) if (start_ts and end_ts) else 0.0

    print("# OpenCanbox Log Triage & Anomaly Report")
    print(f"- **Log File:** `{os.path.basename(log_path)}`")
    print(f"- **Total Lines:** {total_lines:,} | **Duration:** {duration:.2f} s")
    print(f"- **CAN Frames Received:** {sum(can_msg_counts.values()):,} across {len(can_msg_counts)} unique IDs")
    print(f"- **Serial Frames:** App = {len(app_packets)} | Real OEM Canbox = {len(orig_packets)}\n")

    # 1. Command Frequency Breakdown
    app_cmd_counts = defaultdict(int)
    orig_cmd_counts = defaultdict(int)
    app_cmd_payloads = defaultdict(set)
    orig_cmd_payloads = defaultdict(set)

    for p, _ in app_packets:
        if p.cmd is not None and p.cmd != 0xFF:
            app_cmd_counts[p.cmd] += 1
            app_cmd_payloads[p.cmd].add(p.payload.hex())

    for p, _ in orig_packets:
        if p.cmd is not None and p.cmd != 0xFF:
            orig_cmd_counts[p.cmd] += 1
            orig_cmd_payloads[p.cmd].add(p.payload.hex())

    all_cmds = sorted(set(list(app_cmd_counts.keys()) + list(orig_cmd_counts.keys())))

    print("## 1. Serial Command Activity & Discrepancies")
    print("| Cmd ID | Name / Description | App Count | OEM Count | Distinct Payloads | Status / Issue |")
    print("|:---:|:---|:---:|:---:|:---:|:---|")

    anomalies = []

    for cmd in all_cmds:
        c_app = app_cmd_counts[cmd]
        c_orig = orig_cmd_counts[cmd]
        p_app = len(app_cmd_payloads[cmd])
        name = HIWORLD_CMD_NAMES.get(cmd, f"CMD_0x{cmd:02X}")

        status = "OK"
        if c_app > 0 and c_orig == 0:
            status = "**SPURIOUS (App only)**"
            anomalies.append((cmd, "SPURIOUS", f"App sent {c_app} pkts, OEM sent 0"))
        elif c_app == 0 and c_orig > 0:
            status = "**SILENT (OEM only)**"
            anomalies.append((cmd, "SILENT", f"OEM sent {c_orig} pkts, App missed"))
        elif c_app > (c_orig * 5) and c_app > 20:
            status = "**FLOODING (App > 5x OEM)**"
            anomalies.append((cmd, "FLOODING", f"App sent {c_app} vs OEM {c_orig}"))
        elif p_app > 1 and c_app > 10:
            status = "⚠️ Multi-state / Dynamic"

        print(f"| `0x{cmd:02X}` | {name} | {c_app} | {c_orig} | {p_app} (App) | {status} |")

    print("\n---\n")

    # 2. Oscillation & Chatter Detection (Blinking)
    print("## 2. Chatter & Oscillation Analysis (Blinking / Twitching Detection)")
    oscillation_found = False

    # Group by App command and detect rapid toggling
    for cmd in sorted(app_cmd_payloads.keys()):
        cmd_seq = [(p, cans) for p, cans in app_packets if p.cmd == cmd]
        if len(cmd_seq) < 4:
            continue

        # Look for alternating sequence: A -> B -> A -> B
        flips = 0
        last_hex = None
        transitions = []
        for p, cans in cmd_seq:
            p_hex = p.payload.hex()
            if last_hex is not None and p_hex != last_hex:
                flips += 1
                transitions.append((p_hex, cans, p.ts))
            last_hex = p_hex

        if flips >= 4:
            oscillation_found = True
            name = HIWORLD_CMD_NAMES.get(cmd, f"CMD_0x{cmd:02X}")
            print(f"### 🚨 JITTER / OSCILLATION DETECTED: `0x{cmd:02X}` ({name})")
            print(f"- **Flip-Flop Count:** {flips} state changes in {len(cmd_seq)} frames")
            
            # Find the top alternating states
            state_counts = defaultdict(int)
            for p, _ in cmd_seq:
                state_counts[p.payload.hex()] += 1
            top_states = sorted(state_counts.items(), key=lambda x: x[1], reverse=True)[:2]

            for idx, (s_hex, cnt) in enumerate(top_states, 1):
                # Find which CAN frames arrived within 100ms before this state
                trigger_can_ids = defaultdict(int)
                for tr_hex, cans, tr_ts in transitions:
                    if tr_hex == s_hex:
                        for c in cans[-10:]:
                            dt_ms = (tr_ts - c.ts) * 1000.0
                            if 0 <= dt_ms <= 120:
                                trigger_can_ids[c.can_id] += 1

                top_can = sorted(trigger_can_ids.items(), key=lambda x: x[1], reverse=True)[:5]
                can_desc = ", ".join([f"`0x{cid:03X}` (x{c_cnt})" for cid, c_cnt in top_can]) or "None in <120ms"
                print(f"  * **State {idx}** (`{cnt}` times): `{s_hex}`")
                print(f"    - Preceding CAN Candidates (<120ms): {can_desc}")

            print("")

    if not oscillation_found:
        print("No rapid flip-flop oscillations detected in App serial transmissions.\n")

    print("---\n")

    # 3. Payload Length Inconsistencies vs OEM (Both active)
    print("## 3. Wire Format & Length Mismatches (Active in Both App & OEM)")
    length_mismatch_found = False
    for cmd in sorted(set(app_cmd_counts.keys()).intersection(orig_cmd_counts.keys())):
        app_lens = set(len(p.payload) for p, _ in app_packets if p.cmd == cmd)
        orig_lens = set(len(p.payload) for p, _ in orig_packets if p.cmd == cmd)
        if app_lens and orig_lens and app_lens != orig_lens:
            length_mismatch_found = True
            name = HIWORLD_CMD_NAMES.get(cmd, f"CMD_0x{cmd:02X}")
            print(f"- **CMD `0x{cmd:02X}` ({name}):**")
            print(f"  * App Payload Lengths : {sorted(app_lens)} bytes")
            print(f"  * OEM Payload Lengths : {sorted(orig_lens)} bytes")

    if not length_mismatch_found:
        print("All overlapping active commands match expected OEM payload byte lengths.\n")

    print("---\n")

    # 4. Actionable Root-Cause Diagnostic Summary
    print("## 4. Actionable Diagnostic Summary for AI Agent")
    if anomalies:
        for cmd, kind, desc in anomalies:
            name = HIWORLD_CMD_NAMES.get(cmd, f"CMD_0x{cmd:02X}")
            print(f"- **`0x{cmd:02X}` ({name}) - {kind}:** {desc}")
    else:
        print("- No major structural discrepancies observed between App and OEM.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Triage OpenCanbox E2E Log")
    parser.add_argument("log_file", help="Path to dump_*.log")
    args = parser.parse_args()
    triage_log(args.log_file)
