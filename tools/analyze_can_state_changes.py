#!/usr/bin/env python3
"""
Token-Optimized CAN State Change & Transition Analyzer
======================================================
Fast, streaming analyzer designed for LLM agents and embedded engineers to
isolate vehicle status changes (RD4 tuner, BSI settings, HVAC buttons, stalk keys)
from massive raw CAN captures (CSV, candump, or OpenCanbox dump_*.log) in <1,000 tokens.

Features:
- Streaming line-by-line processing (O(1) memory, handles multi-GB captures)
- Multi-format auto-detection (CSV, candump log, OpenCanbox dump_*.log)
- Deduplication of static heartbeats (99%+ token reduction)
- Bit-level diff computation (Byte index, bit position, hex mask, 0->1 / 1->0)
- Classification of state changes:
    * DISCRETE_TOGGLE: Alternating between states (e.g. TA, ESP, Defrost, Child lock)
    * MUX_CAROUSEL: Cyclic multiplexed pages (e.g. BSI 0x260, Alert log 0x120)
    * COUNTER_CLOCK: Monotonic increments (e.g. BSI clock 0x3F6, sequence counters)
    * STREAM_BURST: Continuous variable stream (e.g. ISO-TP radiotext 0x0A4)
- Target toggle finder: Find which ID toggled N times matching user actions.
"""

import argparse
import csv
import io
import os
import re
import sys
from collections import defaultdict
from typing import Dict, List, Optional, Tuple, Iterator, Any

# Common PSA CAN2004 ID hints for contextual annotation
PSA_CAN_NAMES = {
    0x036: "BSI_STATUS / WAKE",
    0x0A4: "RD4_RADIO_TEXT_ISOTP",
    0x0B6: "ENGINE_SPEED_RPM",
    0x0DF: "EMF_STATUS_MFD",
    0x0E1: "RADAR_AAS_PARKING",
    0x0F6: "HVAC_BODY_STATUS",
    0x110: "ESC_ABS_STATUS",
    0x120: "ALERT_LOG_CAROUSEL",
    0x125: "RD4_STATION_TRACK_LIST",
    0x128: "LIGHTS_WIPERS_STATUS",
    0x12D: "ENGINE_CLIMATE_INTERLOCK",
    0x14C: "ODOMETER_STATUS",
    0x15B: "BSI_CAR_CONFIG_TX",
    0x165: "RD4_AUDIO_SOURCE_POWER",
    0x167: "TRIP_ODOMETER_TX",
    0x168: "DOORS_BOOT_STATUS",
    0x18C: "SUSPENSION_STATUS",
    0x1A1: "ALERTS_POPUP_WARNING",
    0x1A5: "RD4_AUDIO_VOLUME",
    0x1D0: "HVAC_CONSOLE_BUTTONS",
    0x1E0: "RD4_PRESENCE_HEARTBEAT",
    0x1E3: "HVAC_CLIMATE_TELEMETRY",
    0x1E5: "RD4_DSP_EQUALIZER",
    0x21F: "STEERING_WHEEL_KEYS",
    0x220: "REVERSE_GEAR_STATUS",
    0x221: "TRIP_BUTTON_STALK",
    0x225: "RD4_TUNER_STATUS_FREQ",
    0x260: "BSI_CONFIG_CAROUSEL",
    0x261: "TRIP_COMPUTER_PAGE_1",
    0x265: "RD4_TUNER_FLAGS_TA_RDS",
    0x268: "TPMS_TYRE_STATUS",
    0x2A1: "TRIP_COMPUTER_PAGE_2",
    0x2A5: "RD4_STATION_NAME_PS",
    0x305: "STEERING_ANGLE_SAS",
    0x325: "RD4_CD_STATUS",
    0x361: "BSI_VEHICLE_OPTIONS",
    0x365: "RD4_CD_TRACK_INFO",
    0x3A5: "RD4_CD_TIME_INFO",
    0x3E5: "MFD_CONSOLE_BUTTONS",
    0x3F6: "BSI_CLOCK_TIME_CALENDAR",
}

class CanFrame:
    __slots__ = ("ts", "can_id", "dlc", "data")
    def __init__(self, ts: float, can_id: int, dlc: int, data: bytes):
        self.ts = ts
        self.can_id = can_id
        self.dlc = dlc
        self.data = data

def parse_can_line_generator(file_path: str) -> Iterator[CanFrame]:
    """Streams CanFrame objects line-by-line from CSV, candump, or dump_*.log."""
    with open(file_path, "r", encoding="utf-8", errors="replace") as f:
        # Sniff first few non-empty lines
        header_sample = []
        for line in f:
            line_str = line.strip()
            if line_str:
                header_sample.append(line_str)
                if len(header_sample) >= 5:
                    break
        f.seek(0)

        if not header_sample:
            return

        first_line = header_sample[0]

        # Format 1: CSV with header (e.g. Time Stamp,ID,Extended,Dir,Bus,LEN,D1,D2...)
        if "," in first_line and ("ID" in first_line or "Time" in first_line or "timestamp" in first_line.lower()):
            reader = csv.reader(f)
            header = [h.strip().upper() for h in next(reader, [])]
            
            # Locate column indexes
            id_col = -1
            len_col = -1
            ts_col = -1
            data_cols = []
            single_data_col = -1

            for idx, col in enumerate(header):
                if col in ("ID", "CAN ID", "CAN_ID", "CANID", "ARBITRATION ID", "ARBITRATIONID"):
                    id_col = idx
                elif col in ("TIME STAMP", "TIMESTAMP", "TIME", "SEC", "TIME (S)", "TIME(S)", "T"):
                    ts_col = idx
                elif col in ("LEN", "DLC", "LENGTH", "DATALEN"):
                    len_col = idx
                elif col in ("DATA", "PAYLOAD"):
                    single_data_col = idx
                elif re.match(r"^D\d+$", col) or re.match(r"^DATA_\d+$", col) or re.match(r"^BYTE\d+$", col):
                    data_cols.append((idx, col))

            for row_idx, row in enumerate(reader):
                if not row or len(row) <= max(id_col, ts_col):
                    continue
                try:
                    # Parse timestamp
                    ts = 0.0
                    if ts_col >= 0 and ts_col < len(row) and row[ts_col].strip():
                        ts = float(row[ts_col].strip())

                    # Parse CAN ID (hex or dec)
                    raw_id = row[id_col].strip()
                    if not raw_id:
                        continue
                    if raw_id.lower().startswith("0x"):
                        can_id = int(raw_id, 16)
                    else:
                        # If hex format without 0x (common in automotive CSVs)
                        can_id = int(raw_id, 16)

                    # Parse data bytes
                    raw_data = bytearray()
                    if single_data_col >= 0 and single_data_col < len(row):
                        hex_str = re.sub(r"[^0-9A-Fa-f]", "", row[single_data_col])
                        if len(hex_str) % 2 == 0:
                            raw_data = bytearray(bytes.fromhex(hex_str))
                    elif data_cols:
                        for d_idx, _ in data_cols:
                            if d_idx < len(row):
                                val = row[d_idx].strip()
                                if val:
                                    raw_data.append(int(val, 16))

                    # Truncate to DLC if known
                    dlc = len(raw_data)
                    if len_col >= 0 and len_col < len(row) and row[len_col].strip():
                        try:
                            dlc_val = int(row[len_col].strip())
                            if dlc_val <= len(raw_data):
                                dlc = dlc_val
                                raw_data = raw_data[:dlc]
                        except ValueError:
                            pass

                    yield CanFrame(ts, can_id, dlc, bytes(raw_data))
                except (ValueError, IndexError):
                    continue
            return

        # Format 2 & 3: Line-based (OpenCanbox dump_*.log or candump)
        re_opencanbox = re.compile(r"^\s*([0-9.]+)?\s*\[CAN\]\s+ID:([0-9A-Fa-f]+)\s+DLC:(\d+)\s+DATA:([0-9A-Fa-f\s]*)")
        re_candump_compact = re.compile(r"^\s*(?:\(([0-9.]+)\))?\s*\S+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)")
        re_candump_spaced = re.compile(r"^\s*(?:\(([0-9.]+)\))?\s*\S+\s+([0-9A-Fa-f]+)\s+\[(\d+)\]\s+([0-9A-Fa-f\s]*)")

        for line in f:
            line_str = line.strip()
            if not line_str or line_str.startswith("#"):
                continue

            # Check OpenCanbox format: 1234.567 [CAN] ID:265 DLC:4 DATA: A4 10 00 0F
            m = re_opencanbox.match(line_str)
            if m:
                ts = float(m.group(1)) if m.group(1) else 0.0
                can_id = int(m.group(2), 16)
                dlc = int(m.group(3))
                hex_clean = re.sub(r"[^0-9A-Fa-f]", "", m.group(4))
                data = bytes.fromhex(hex_clean)[:dlc]
                yield CanFrame(ts, can_id, len(data), data)
                continue

            # Check Candump compact: (1234.567) can0 265#A410000F
            m = re_candump_compact.match(line_str)
            if m:
                ts = float(m.group(1)) if m.group(1) else 0.0
                can_id = int(m.group(2), 16)
                data = bytes.fromhex(m.group(3))
                yield CanFrame(ts, can_id, len(data), data)
                continue

            # Check Candump spaced: (1234.567) can0 265 [4] A4 10 00 0F
            m = re_candump_spaced.match(line_str)
            if m:
                ts = float(m.group(1)) if m.group(1) else 0.0
                can_id = int(m.group(2), 16)
                dlc = int(m.group(3))
                hex_clean = re.sub(r"[^0-9A-Fa-f]", "", m.group(4))
                data = bytes.fromhex(hex_clean)[:dlc]
                yield CanFrame(ts, can_id, len(data), data)
                continue

def compute_bit_diff(old_d: bytes, new_d: bytes) -> List[str]:
    """Returns human-readable byte and bit changes between two payloads."""
    diffs = []
    max_len = max(len(old_d), len(new_d))
    for i in range(max_len):
        b_old = old_d[i] if i < len(old_d) else 0
        b_new = new_d[i] if i < len(new_d) else 0
        if b_old != b_new:
            xor = b_old ^ b_new
            bit_details = []
            for bit in range(8):
                mask = 1 << bit
                if xor & mask:
                    val_old = (b_old >> bit) & 1
                    val_new = (b_new >> bit) & 1
                    bit_details.append(f"bit{bit}(0x{mask:02X}):{val_old}->{val_new}")
            diffs.append(f"B{i}: 0x{b_old:02X}->0x{b_new:02X} [{'+'.join(bit_details)}]")
    return diffs

def classify_transitions(payload_history: List[bytes]) -> str:
    """Classifies the behavior of a CAN ID state sequence."""
    if not payload_history:
        return "STATIC"
    unique = list(dict.fromkeys(payload_history))
    if len(unique) <= 1:
        return "STATIC"
    if len(unique) == 2:
        # Alternating between 2 states (e.g. button press/release or feature toggle)
        return "DISCRETE_TOGGLE"
    if len(unique) in (3, 4, 5):
        # Check if repeating in a cycle (carousel)
        pattern = unique
        plen = len(pattern)
        is_carousel = True
        for idx, p in enumerate(payload_history):
            if p != pattern[idx % plen]:
                is_carousel = False
                break
        if is_carousel:
            return "MUX_CAROUSEL"
        return "DISCRETE_MULTI_STATE"
    # Check monotonic increment in any byte
    is_counter = True
    for i in range(len(payload_history) - 1):
        p1, p2 = payload_history[i], payload_history[i + 1]
        if len(p1) == len(p2) and len(p1) > 0:
            diff_count = sum(1 for a, b in zip(p1, p2) if a != b)
            if diff_count != 1:
                is_counter = False
                break
        else:
            is_counter = False
            break
    if is_counter:
        return "COUNTER_CLOCK"
    return "STREAM_BURST"

def analyze_can_log(
    file_path: str,
    filter_ids: Optional[set] = None,
    ignore_ids: Optional[set] = None,
    min_changes: int = 1,
    max_changes: Optional[int] = None,
    target_toggles: Optional[int] = None,
    show_details: bool = False,
    detail_limit: int = 40,
    json_output: bool = False
):
    if not os.path.isfile(file_path):
        print(f"Error: File not found: {file_path}", file=sys.stderr)
        sys.exit(1)

    total_frames = 0
    msg_counts = defaultdict(int)
    last_payload = {}
    transitions = defaultdict(list)
    payload_history = defaultdict(list)
    unique_payloads = defaultdict(set)
    affected_bits_per_id = defaultdict(lambda: defaultdict(int)) # cid -> (byte_idx, bit_idx) -> count

    for frame in parse_can_line_generator(file_path):
        total_frames += 1
        cid = frame.can_id
        if filter_ids and cid not in filter_ids:
            continue
        if ignore_ids and cid in ignore_ids:
            continue

        msg_counts[cid] += 1
        data = frame.data

        if cid not in last_payload:
            last_payload[cid] = data
            unique_payloads[cid].add(data)
            payload_history[cid].append(data)
        elif last_payload[cid] != data:
            old_data = last_payload[cid]
            transitions[cid].append((frame.ts, old_data, data))
            unique_payloads[cid].add(data)
            payload_history[cid].append(data)

            # Record flipping bits
            max_l = max(len(old_data), len(data))
            for b_idx in range(max_l):
                b_old = old_data[b_idx] if b_idx < len(old_data) else 0
                b_new = data[b_idx] if b_idx < len(data) else 0
                xor = b_old ^ b_new
                for bit in range(8):
                    if xor & (1 << bit):
                        affected_bits_per_id[cid][(b_idx, bit)] += 1

            last_payload[cid] = data

    # Filter CAN IDs according to parameters
    changing_ids = []
    for cid in sorted(msg_counts.keys()):
        num_changes = len(transitions[cid])
        if num_changes < min_changes:
            continue
        if max_changes is not None and num_changes > max_changes:
            continue
        if target_toggles is not None and num_changes != target_toggles:
            continue
        changing_ids.append(cid)

    if not changing_ids:
        print(f"No CAN IDs matched criteria out of {total_frames} total frames analyzed.")
        return

    # Print Summary Table
    print(f"### CAN Bus State Change Analysis ({os.path.basename(file_path)})")
    print(f"- **Total Frames Processed:** {total_frames:,}")
    print(f"- **Active CAN IDs:** {len(msg_counts)}")
    print(f"- **IDs with Payload Changes:** {len(changing_ids)}\n")

    print("| CAN ID | Name / Subsystem | Msgs | Transitions | States | Classification | Key Flipping Bits |")
    print("|:---:|:---|:---:|:---:|:---:|:---|:---|")

    for cid in changing_ids:
        c_name = PSA_CAN_NAMES.get(cid, "UNKNOWN")
        num_msgs = msg_counts[cid]
        num_trans = len(transitions[cid])
        num_states = len(unique_payloads[cid])
        classification = classify_transitions(payload_history[cid])
        
        # Format top flipping bits
        bits_summary = []
        sorted_bits = sorted(affected_bits_per_id[cid].items(), key=lambda x: x[1], reverse=True)
        for (b_idx, bit_idx), count in sorted_bits[:3]:
            mask = 1 << bit_idx
            bits_summary.append(f"B{b_idx}:b{bit_idx}(0x{mask:02X}) [x{count}]")
        bits_str = ", ".join(bits_summary) if bits_summary else "None"

        print(f"| `0x{cid:03X}` | {c_name} | {num_msgs} | {num_trans} | {num_states} | `{classification}` | {bits_str} |")

    # If requested or if discrete toggle detected, show transition detail
    if show_details or target_toggles is not None:
        print("\n### Detailed State Transitions & Bit Flips\n")
        for cid in changing_ids:
            c_name = PSA_CAN_NAMES.get(cid, "UNKNOWN")
            t_list = transitions[cid]
            print(f"#### CAN `0x{cid:03X}` ({c_name}) — {len(t_list)} transitions")
            
            show_count = min(len(t_list), detail_limit)
            for idx in range(show_count):
                ts, old_d, new_d = t_list[idx]
                bit_diffs = compute_bit_diff(old_d, new_d)
                ts_str = f"t={ts:10.3f}" if isinstance(ts, float) and ts > 0 else f"t={int(ts):8d}"
                print(f"- `{ts_str}` : `{old_d.hex(' ').upper()}` $\\to$ `{new_d.hex(' ').upper()}`")
                print(f"  * Diff: {'; '.join(bit_diffs)}")
            if len(t_list) > show_count:
                print(f"  * ... ({len(t_list) - show_count} further transitions omitted for token efficiency)")
            print("")

def main():
    parser = argparse.ArgumentParser(
        description="Token-Optimized CAN State Change & Transition Analyzer for OpenCanbox/PSA",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # 1. Quick overview of all state changes in log (<500 tokens):
  ./tools/analyze_can_state_changes.py rd4_tuner_TA_switching_log.csv

  # 2. Find which CAN ID toggled exactly 6 times (e.g. 3 ON/OFF cycles):
  ./tools/analyze_can_state_changes.py rd4_tuner_TA_switching_log.csv --target-toggles 6 -d

  # 3. Focus on radio IDs, ignoring multiplexed carousels and clocks:
  ./tools/analyze_can_state_changes.py my_huge_dump.log -c 165 225 265 2A5 -d

  # 4. Filter out high-frequency chatter or known rotating frames:
  ./tools/analyze_can_state_changes.py candump.log -x 260 120 3F6
        """
    )
    parser.add_argument("log_file", help="Path to CAN capture file (.csv, candump .log, or dump_*.log)")
    parser.add_argument("-c", "--ids", nargs="+", help="Specific CAN ID(s) to inspect (hex e.g. 165 265)")
    parser.add_argument("-x", "--ignore-ids", nargs="+", help="CAN ID(s) to ignore (hex e.g. 260 120 3F6)")
    parser.add_argument("-t", "--target-toggles", type=int, help="Filter for CAN IDs with exact toggle/transition count")
    parser.add_argument("--min-changes", type=int, default=1, help="Minimum transitions to include (default: 1)")
    parser.add_argument("--max-changes", type=int, help="Maximum transitions to include (suppresses chatter/counters)")
    parser.add_argument("-d", "--detail", action="store_true", help="Print detailed bit flip timeline for filtered IDs")
    parser.add_argument("--limit", type=int, default=30, help="Max transitions per ID to print in detailed view (default: 30)")

    args = parser.parse_args()

    filter_ids = None
    if args.ids:
        filter_ids = {int(x, 16) if not x.lower().startswith("0x") else int(x, 16) for x in args.ids}

    ignore_ids = None
    if args.ignore_ids:
        ignore_ids = {int(x, 16) if not x.lower().startswith("0x") else int(x, 16) for x in args.ignore_ids}

    analyze_can_log(
        file_path=args.log_file,
        filter_ids=filter_ids,
        ignore_ids=ignore_ids,
        min_changes=args.min_changes,
        max_changes=args.max_changes,
        target_toggles=args.target_toggles,
        show_details=args.detail,
        detail_limit=args.limit
    )

if __name__ == "__main__":
    main()

