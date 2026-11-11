#!/usr/bin/env python3
"""
Reconstructable CAN Delta Sniffer Log & Bit-Diff Tool
=====================================================
Reads a compact .csniff binary capture file, raw Hiworld UART stream,
or Android Head Unit CSV trace (# CANBOX_SNIFF_V1) and produces:
1. <file>_diff.txt: Chronological reverse-engineering bit-diff report
2. <file>_reconstructed.log: Deterministic full-rate standard dump_*.log file
"""

import argparse
import os
import sys
from collections import defaultdict
from typing import List, Dict, Tuple, Optional, Any

HIWORLD_SYNC_1 = 0x5A
HIWORLD_SYNC_2 = 0xA5
CMD_SNIFFER_CONTROL = 0xD0
CMD_SNIFFER_DATA = 0xD1

SUBTYPE_LIVE = 0x01
SUBTYPE_SUMMARY = 0x02
SUBTYPE_FINISHED = 0xFF

# Nominal PSA CAN2004 Comfort bus periods (ms) for trailing expansion
DEFAULT_PSA_PERIODS = {
    0x036: 100, 0x0B6: 50,  0x0DF: 200, 0x0E1: 100, 0x0E2: 1000, 0x0F6: 100,
    0x110: 100, 0x120: 1000, 0x126: 100, 0x128: 200, 0x12D: 500, 0x131: 100,
    0x14C: 100, 0x15B: 1000, 0x161: 500, 0x162: 100, 0x165: 100, 0x167: 1000,
    0x168: 200, 0x18C: 100, 0x190: 200, 0x1A0: 500, 0x1A1: 200, 0x1A2: 500,
    0x1A5: 500, 0x1A8: 200, 0x1CC: 200, 0x1D0: 500, 0x1DF: 200, 0x1E0: 500,
    0x1E1: 500, 0x1E2: 500, 0x1E3: 100, 0x1E5: 500, 0x1ED: 1000, 0x217: 100,
    0x21F: 100, 0x220: 200, 0x221: 1000, 0x225: 500, 0x227: 500, 0x24C: 100,
    0x260: 500, 0x261: 1000, 0x265: 500, 0x28C: 100, 0x2A0: 500, 0x2A1: 1000,
    0x2A5: 500, 0x2B6: 1000, 0x2E1: 1000, 0x315: 1000, 0x317: 1000, 0x325: 500,
    0x336: 1000, 0x355: 100, 0x361: 500, 0x365: 500, 0x3A5: 500, 0x3A7: 500,
    0x3B6: 1000, 0x3E5: 500, 0x3F6: 1000, 0x49F: 500, 0x50C: 1000, 0x512: 1000,
    0x51D: 1000, 0x51F: 1000, 0x520: 1000, 0x525: 1000, 0x52D: 1000, 0x531: 1000,
}


class SnifferRecord:
    __slots__ = ('dt_ms', 'can_id', 'dlc', 'is_tx', 'prev_repeat_count', 'period_10ms', 'data', 'subtype')
    def __init__(self, dt_ms: int, can_id: int, dlc: int, is_tx: bool,
                 prev_repeat_count: int, period_10ms: int, data: bytes, subtype: int):
        self.dt_ms = dt_ms
        self.can_id = can_id
        self.dlc = dlc
        self.is_tx = is_tx
        self.prev_repeat_count = prev_repeat_count
        self.period_10ms = period_10ms
        self.data = data
        self.subtype = subtype

    def __repr__(self) -> str:
        tx_str = "TX" if self.is_tx else "RX"
        return f"SnifferRecord(dt={self.dt_ms}ms, {tx_str} ID=0x{self.can_id:03X}, DLC={self.dlc}, repeats={self.prev_repeat_count}, period={self.period_10ms*10}ms, data={self.data.hex()})"


class SniffCapture:
    def __init__(self):
        self.records: List[SnifferRecord] = []
        self.total_unique_ids: int = 0
        self.dropped_events: int = 0
        self.uart_packets: List[Tuple[float, str, str]] = []  # (ts, tag, raw_hex)


def parse_csniff_text(text: str) -> SniffCapture:
    cap = SniffCapture()
    lines = [l.strip() for l in text.splitlines() if l.strip()]
    unique_ids = set()

    for line in lines:
        if line.startswith("#"):
            continue
        parts = line.split(",")
        if len(parts) < 7:
            continue
        try:
            dt_ms = int(parts[0])
            kind = parts[1].strip()
            id_or_cmd_str = parts[2].strip()
            dlc = int(parts[3])
            period_ms = int(parts[4])
            repeats = int(parts[5])
            raw_hex = parts[6].strip()
            note = parts[7].strip() if len(parts) > 7 else ""
            data = bytes.fromhex(raw_hex) if raw_hex else b""
        except Exception:
            continue

        if kind in ("CAN_RX", "CAN_TX"):
            can_id = int(id_or_cmd_str, 16)
            is_tx = (kind == "CAN_TX")
            unique_ids.add(can_id)
            period_10ms = max(0, period_ms // 10)
            subtype = SUBTYPE_SUMMARY if note == "SUMMARY" else SUBTYPE_LIVE
            cap.records.append(SnifferRecord(
                dt_ms=dt_ms,
                can_id=can_id,
                dlc=dlc,
                is_tx=is_tx,
                prev_repeat_count=repeats,
                period_10ms=period_10ms,
                data=data,
                subtype=subtype
            ))
        elif kind in ("UART_RX", "UART_TX"):
            cmd = int(id_or_cmd_str, 16)
            ts_sec = dt_ms / 1000.0
            plen = len(data)
            cs = ((plen + cmd + sum(data)) - 1) & 0xFF
            full_frame = bytes([HIWORLD_SYNC_1, HIWORLD_SYNC_2, plen, cmd]) + data + bytes([cs])
            tag = "[APP_UART_TX]" if kind == "UART_RX" else "[APP_UART_RX]"
            cap.uart_packets.append((ts_sec, tag, full_frame.hex().upper()))

    cap.total_unique_ids = len(unique_ids)
    cap.dropped_events = 0
    return cap


def parse_csniff_bytes(raw_data: bytes) -> SniffCapture:
    # First check if the file is in Android HU text / CSV format
    if raw_data.startswith(b"#") or b"# CANBOX_SNIFF" in raw_data[:128] or b"# FORMAT:" in raw_data[:128]:
        return parse_csniff_text(raw_data.decode("utf-8", errors="replace"))

    cap = SniffCapture()
    n = len(raw_data)
    idx = 0

    # Check if the file contains Hiworld framed packets (5A A5)
    has_sync = (raw_data.find(bytes([HIWORLD_SYNC_1, HIWORLD_SYNC_2])) != -1)

    if has_sync:
        while idx < n - 4:
            if raw_data[idx] == HIWORLD_SYNC_1 and raw_data[idx + 1] == HIWORLD_SYNC_2:
                plen = raw_data[idx + 2]
                cmd = raw_data[idx + 3]
                total_len = plen + 5
                if idx + total_len > n:
                    break

                payload = raw_data[idx + 4 : idx + 4 + plen]
                cs = raw_data[idx + 4 + plen]
                exp_cs = ((plen + cmd + sum(payload)) - 1) & 0xFF

                if cs == exp_cs:
                    frame_raw = raw_data[idx : idx + total_len]
                    if cmd == CMD_SNIFFER_DATA and len(payload) >= 1:
                        subtype = payload[0]
                        sub_data = payload[1:]

                        if subtype == SUBTYPE_FINISHED:
                            if len(sub_data) >= 3:
                                cap.total_unique_ids = sub_data[0]
                                cap.dropped_events = sub_data[1] | (sub_data[2] << 8)
                        elif subtype in (SUBTYPE_LIVE, SUBTYPE_SUMMARY):
                            _parse_records_payload(sub_data, subtype, cap)

                    idx += total_len
                    continue
            idx += 1
    else:
        # File is a concatenated raw payload stream
        while idx < n:
            subtype = raw_data[idx]
            idx += 1
            if subtype == SUBTYPE_FINISHED:
                if idx + 3 <= n:
                    cap.total_unique_ids = raw_data[idx]
                    cap.dropped_events = raw_data[idx + 1] | (raw_data[idx + 2] << 8)
                    idx += 3
                break
            elif subtype in (SUBTYPE_LIVE, SUBTYPE_SUMMARY):
                consumed = _parse_records_payload(raw_data[idx:], subtype, cap)
                idx += consumed
            else:
                idx += 1

    return cap


def _parse_records_payload(payload: bytes, subtype: int, cap: SniffCapture) -> int:
    idx = 0
    n = len(payload)
    while idx + 7 <= n:
        dt_ms = payload[idx] | (payload[idx + 1] << 8)
        id_flags_dlc = payload[idx + 2] | (payload[idx + 3] << 8)
        prev_repeat_count = payload[idx + 4] | (payload[idx + 5] << 8)
        period_10ms = payload[idx + 6]

        can_id = id_flags_dlc & 0x07FF
        dlc = (id_flags_dlc >> 11) & 0x0F
        is_tx = bool(id_flags_dlc & 0x8000)

        rec_len = 7 + dlc
        if idx + rec_len > n:
            break

        data = payload[idx + 7 : idx + rec_len]
        cap.records.append(SnifferRecord(dt_ms, can_id, dlc, is_tx, prev_repeat_count, period_10ms, data, subtype))
        idx += rec_len
    return idx


def generate_bit_diff_report(cap: SniffCapture, output_path: str):
    id_stats: Dict[int, Dict[str, Any]] = {}
    id_last_payload: Dict[int, bytes] = {}
    timeline_lines: List[str] = []

    for rec in cap.records:
        cid = rec.can_id
        if cid not in id_stats:
            id_stats[cid] = {
                'id': cid,
                'is_tx': rec.is_tx,
                'dlc': rec.dlc,
                'period_10ms': rec.period_10ms,
                'transitions': 0,
                'total_repeats': 0,
                'first_dt_ms': rec.dt_ms,
                'last_dt_ms': rec.dt_ms
            }

        st = id_stats[cid]
        st['last_dt_ms'] = rec.dt_ms
        if rec.period_10ms > 0:
            st['period_10ms'] = rec.period_10ms
        st['total_repeats'] += rec.prev_repeat_count

        ts_sec = rec.dt_ms / 1000.0

        if rec.subtype == SUBTYPE_LIVE:
            if cid not in id_last_payload:
                id_last_payload[cid] = rec.data
                dir_label = "CAN_TX" if rec.is_tx else "CAN"
                hex_data = " ".join(f"{b:02X}" for b in rec.data)
                timeline_lines.append(f"[+{ts_sec:6.3f}s] {dir_label} 0x{cid:03X} (DLC {rec.dlc}) [DISCOVERED]: {hex_data}")
            else:
                old_data = id_last_payload[cid]
                if rec.is_tx:
                    hex_data = " ".join(f"{b:02X}" for b in rec.data)
                    timeline_lines.append(f"[+{ts_sec:6.3f}s] CAN_TX 0x{cid:03X} (DLC {rec.dlc}): {hex_data}")
                elif old_data != rec.data or len(old_data) != rec.dlc:
                    st['transitions'] += 1
                    diffs = []
                    max_len = max(len(old_data), len(rec.data))
                    for b_idx in range(max_len):
                        b_old = old_data[b_idx] if b_idx < len(old_data) else None
                        b_new = rec.data[b_idx] if b_idx < len(rec.data) else None
                        if b_old != b_new:
                            if b_old is not None and b_new is not None:
                                bit_flips = []
                                for bit in range(8):
                                    bit_old = (b_old >> bit) & 1
                                    bit_new = (b_new >> bit) & 1
                                    if bit_old != bit_new:
                                        bit_flips.append(f"bit{bit}: {bit_old}->{bit_new}")
                                bit_str = " ".join(bit_flips)
                                diffs.append(f"Byte[{b_idx}]: 0x{b_old:02X} -> 0x{b_new:02X} [{bit_str}]")
                            elif b_old is None:
                                diffs.append(f"Byte[{b_idx}]: [added] -> 0x{b_new:02X}")
                            else:
                                diffs.append(f"Byte[{b_idx}]: 0x{b_old:02X} -> [removed]")

                    diff_summary = "; ".join(diffs)
                    old_hex = " ".join(f"{b:02X}" for b in old_data)
                    new_hex = " ".join(f"{b:02X}" for b in rec.data)
                    period_val = rec.period_10ms * 10 if rec.period_10ms > 0 else DEFAULT_PSA_PERIODS.get(cid, 0)
                    timeline_lines.append(
                        f"[+{ts_sec:6.3f}s] CAN 0x{cid:03X} (DLC {rec.dlc}) [CHANGED after {rec.prev_repeat_count} repeats / ~{period_val}ms period]:\n"
                        f"         Old: {old_hex}\n"
                        f"         New: {new_hex}\n"
                        f"         Diff: {diff_summary}"
                    )
                    id_last_payload[cid] = rec.data

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        f.write("================================================================================\n")
        f.write(" OpenCanbox Reconstructed Sniffer Report & Delta Analysis\n")
        f.write("================================================================================\n")
        f.write(f"Total Unique CAN IDs : {len(id_stats)} (reported by MCU: {cap.total_unique_ids})\n")
        f.write(f"Total Records Extracted : {len(cap.records)}\n")
        f.write(f"Dropped Events (Overflow) : {cap.dropped_events}\n\n")

        f.write("CAN IDs Summary Table:\n")
        f.write("--------------------------------------------------------------------------------\n")
        f.write(" CAN ID | Type | DLC | Cycle Period    | Transitions | Total Frames | Activity\n")
        f.write("--------------------------------------------------------------------------------\n")
        for cid in sorted(id_stats.keys()):
            st = id_stats[cid]
            p_ms = st['period_10ms'] * 10
            if p_ms <= 0 and cid in DEFAULT_PSA_PERIODS:
                p_ms = DEFAULT_PSA_PERIODS[cid]
            p_str = f"{p_ms:4d} ms" if p_ms > 0 else "Aperiodic"
            if p_ms > 0:
                hz = 1000.0 / p_ms
                p_str += f" ({hz:4.1f} Hz)"
            else:
                p_str += "          "
            type_str = "TX" if st['is_tx'] else "RX"
            total_frames = 1 + st['total_repeats'] + st['transitions']
            f.write(f" 0x{cid:03X}  | {type_str:4s} |  {st['dlc']}  | {p_str:15s} | {st['transitions']:11d} | {total_frames:12d} | [+{(st['first_dt_ms']/1000.0):.1f}s .. +{(st['last_dt_ms']/1000.0):.1f}s]\n")
        f.write("--------------------------------------------------------------------------------\n\n")

        f.write("Chronological Transitions & Bit Flipped Timeline:\n")
        f.write("================================================================================\n")
        for line in timeline_lines:
            f.write(line + "\n")
        f.write("================================================================================\n")
        f.write("End of sniffer capture report.\n")


def generate_reconstructed_log(cap: SniffCapture, output_path: str):
    expanded_frames: List[Tuple[float, str]] = []
    last_seen_state: Dict[int, Dict[str, Any]] = {}

    max_dt_ms = 0
    if cap.records:
        max_dt_ms = max(r.dt_ms for r in cap.records)
    if cap.uart_packets:
        max_uart_ms = int(max(p[0] for p in cap.uart_packets) * 1000)
        max_dt_ms = max(max_dt_ms, max_uart_ms)

    for rec in cap.records:
        cid = rec.can_id
        dt_ms = rec.dt_ms

        # 1. Expand prior repeated frames if repeat_count > 0 and we know previous state
        if rec.prev_repeat_count > 0 and cid in last_seen_state:
            prev = last_seen_state[cid]
            prev_dt = prev['dt_ms']
            count = rec.prev_repeat_count
            period_ms = rec.period_10ms * 10 if rec.period_10ms > 0 else prev.get('period_ms', 0)
            if period_ms <= 0:
                period_ms = DEFAULT_PSA_PERIODS.get(cid, 500)

            for r in range(1, count + 1):
                rep_dt = prev_dt + r * period_ms
                if rec.subtype == SUBTYPE_LIVE and rep_dt >= dt_ms:
                    rep_dt = prev_dt + ((dt_ms - prev_dt - 1) * r) // (count + 1)
                ts = rep_dt / 1000.0
                tag = "[CAN_TX]" if prev['is_tx'] else "[CAN]"
                d_hex = prev['data'].hex().upper()
                expanded_frames.append((ts, f"{ts:10.3f} {tag} ID:{cid:03X} DLC:{prev['dlc']} DATA:{d_hex}"))

        # 2. Add current event frame
        if rec.subtype == SUBTYPE_LIVE:
            ts = dt_ms / 1000.0
            tag = "[CAN_TX]" if rec.is_tx else "[CAN]"
            d_hex = rec.data.hex().upper()
            expanded_frames.append((ts, f"{ts:10.3f} {tag} ID:{cid:03X} DLC:{rec.dlc} DATA:{d_hex}"))
            
            eff_period = rec.period_10ms * 10 if rec.period_10ms > 0 else DEFAULT_PSA_PERIODS.get(cid, 500)
            last_seen_state[cid] = {
                'dt_ms': dt_ms,
                'data': rec.data,
                'dlc': rec.dlc,
                'is_tx': rec.is_tx,
                'period_ms': eff_period,
                'has_summary': False
            }
        elif rec.subtype == SUBTYPE_SUMMARY:
            if cid in last_seen_state:
                last_seen_state[cid]['dt_ms'] = dt_ms
                last_seen_state[cid]['has_summary'] = True

    # 3. Trailing expansion for all active IDs up to max_dt_ms
    for cid, prev in last_seen_state.items():
        if prev.get('has_summary'):
            continue
        p_ms = prev.get('period_ms') or DEFAULT_PSA_PERIODS.get(cid, 500)
        if p_ms <= 0:
            p_ms = 500
        prev_dt = prev['dt_ms']
        rem = max_dt_ms - prev_dt
        if rem >= p_ms:
            repeats = rem // p_ms
            for r in range(1, repeats + 1):
                rep_dt = prev_dt + r * p_ms
                ts = rep_dt / 1000.0
                tag = "[CAN_TX]" if prev['is_tx'] else "[CAN]"
                d_hex = prev['data'].hex().upper()
                expanded_frames.append((ts, f"{ts:10.3f} {tag} ID:{cid:03X} DLC:{prev['dlc']} DATA:{d_hex}"))

    # 4. Include UART packets
    for ts, tag, raw_hex in cap.uart_packets:
        expanded_frames.append((ts, f"{ts:10.3f} {tag} {raw_hex}"))

    # Sort all expanded frames chronologically
    expanded_frames.sort(key=lambda item: item[0])

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        for _, line in expanded_frames:
            f.write(line + "\n")


def main():
    parser = argparse.ArgumentParser(description="Reconstruct full-rate CAN log and bit-diff report from .csniff binary or CSV trace.")
    parser.add_argument("input_file", help="Path to input .csniff or binary capture file")
    parser.add_argument("-o", "--output-dir", default=None, help="Output directory (default: same directory as input file)")
    args = parser.parse_args()

    input_path = args.input_file
    if not os.path.isfile(input_path):
        print(f"Error: input file not found: {input_path}", file=sys.stderr)
        sys.exit(1)

    out_dir = args.output_dir or os.path.dirname(input_path) or "."
    os.makedirs(out_dir, exist_ok=True)
    base_name = os.path.splitext(os.path.basename(input_path))[0]
    diff_path = os.path.join(out_dir, f"{base_name}_diff.txt")
    recon_path = os.path.join(out_dir, f"{base_name}_reconstructed.log")

    with open(input_path, "rb") as f:
        raw_bytes = f.read()

    capture = parse_csniff_bytes(raw_bytes)
    print(f"Parsed {len(capture.records)} sniffer records and {len(capture.uart_packets)} UART packets from '{input_path}'.")
    print(f"Unique IDs reported: {capture.total_unique_ids}, Dropped: {capture.dropped_events}")

    generate_bit_diff_report(capture, diff_path)
    print(f"Generated bit-diff report: {diff_path}")

    generate_reconstructed_log(capture, recon_path)
    print(f"Generated reconstructed log: {recon_path}")


if __name__ == "__main__":
    main()
