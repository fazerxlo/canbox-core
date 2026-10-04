#!/usr/bin/env python3
"""
OpenCanbox Serial Frame Difference Analyzer
===========================================
Identifies differences in serial frames between the real OEM Canbox (ORIG_CANBOX)
and OpenCanbox app (APP_UART_TX) in relation to CAN actions.

Compresses / deduplicates repetitive CAN heartbeats and serial keep-alives to
save massive token usage and time during debugging and analysis.

Inputs:
  - log file (dump_*.log)
  - interesting CAN IDs (e.g. 1D0, 1E3, 12D)
  - interesting serial message prefixes or opcodes (e.g. 31, 5aa50c31, 11)

Outputs:
  - CAN action (which CAN ID and bytes changed)
  - Serial difference between Real Canbox and App (byte-by-byte diff, length diff, CS)
"""

import argparse
import os
import re
import sys
from typing import List, Dict, Tuple, Optional, Any


# Hiworld Command Descriptions
HIWORLD_CMD_NAMES = {
    0x11: "CAR_BASE_INFO",
    0x12: "DOORS_WINDOWS",
    0x13: "TRIP_INFO_1",
    0x14: "TRIP_INFO_2",
    0x15: "TRIP_INFO_3",
    0x16: "TIRE_PRESSURE",
    0x21: "WHEEL_BUTTONS",
    0x22: "STALK_BUTTONS",
    0x31: "HVAC_STATUS",
    0x32: "FRONT_RADAR",
    0x33: "REAR_RADAR",
    0x41: "PARKING_RADAR",
    0x42: "ALERTS_WARNINGS",
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

class Ansi:
    RESET   = "\033[0m"
    BOLD    = "\033[1m"
    RED     = "\033[31m"
    GREEN   = "\033[32m"
    YELLOW  = "\033[33m"
    BLUE    = "\033[34m"
    MAGENTA = "\033[35m"
    CYAN    = "\033[36m"
    GRAY    = "\033[90m"


class SerialPacket:
    def __init__(self, ts: float, source: str, proto: str, raw_bytes: bytes,
                 cmd: Optional[int], payload: bytes, checksum: Optional[int],
                 valid_cs: bool):
        self.ts = ts
        self.source = source  # "ORIG" or "APP"
        self.proto = proto
        self.raw = raw_bytes
        self.raw_hex = raw_bytes.hex().lower()
        self.cmd = cmd
        self.payload = payload
        self.checksum = checksum
        self.valid_cs = valid_cs
        self.repeat_count = 1

    def cmd_name(self) -> str:
        if self.cmd is not None and self.cmd in HIWORLD_CMD_NAMES:
            return HIWORLD_CMD_NAMES[self.cmd]
        return f"CMD_0x{self.cmd:02X}" if self.cmd is not None else "RAW"

    def __repr__(self):
        return f"<{self.source} {self.raw_hex} valid={self.valid_cs}>"


class CanEvent:
    def __init__(self, ts: float, can_id: int, dlc: int, data: bytes,
                 prev_data: Optional[bytes] = None):
        self.ts = ts
        self.can_id = can_id
        self.can_id_hex = f"{can_id:X}"
        self.dlc = dlc
        self.data = data
        self.data_hex = data.hex().lower()
        self.prev_data = prev_data
        self.changed_bytes = []
        if prev_data is not None:
            max_len = max(len(data), len(prev_data))
            for i in range(max_len):
                b_curr = data[i] if i < len(data) else None
                b_prev = prev_data[i] if i < len(prev_data) else None
                if b_curr != b_prev:
                    self.changed_bytes.append((i, b_prev, b_curr))


class StreamReassembler:
    """Reassembles fragmented byte streams from serial ports into discrete protocol packets."""
    def __init__(self, source: str, proto: str = "auto"):
        self.source = source
        self.proto = proto
        self.buf = bytearray()
        self.ts_buf = []

    def feed(self, hex_chunk: str, ts: float) -> List[SerialPacket]:
        clean_hex = re.sub(r"[^0-9a-fA-F]", "", hex_chunk)
        if not clean_hex:
            return []
        try:
            chunk_bytes = bytes.fromhex(clean_hex)
        except ValueError:
            return []

        for b in chunk_bytes:
            self.buf.append(b)
            self.ts_buf.append(ts)

        packets = []
        while len(self.buf) > 0:
            pkt = self._try_extract_packet()
            if pkt:
                packets.append(pkt)
            else:
                break
        return packets

    def _try_extract_packet(self) -> Optional[SerialPacket]:
        while len(self.buf) > 0:
            b0 = self.buf[0]

            # 1. Hiworld: 0x5A 0xA5 <len> <cmd> <payload...> <cs>
            if self.proto in ("auto", "hiworld") and b0 == 0x5A:
                if len(self.buf) == 1:
                    return None  # Wait for 0xA5
                if self.buf[1] != 0xA5:
                    del self.buf[0]
                    del self.ts_buf[0]
                    continue
                if len(self.buf) < 4:
                    return None  # Wait for len + cmd
                plen = self.buf[2]
                if plen > 64:
                    del self.buf[0]
                    del self.ts_buf[0]
                    continue
                total_len = plen + 5
                if len(self.buf) < total_len:
                    return None  # Wait for payload + checksum

                pkt_bytes = bytes(self.buf[:total_len])
                pkt_ts = self.ts_buf[0]
                cmd = pkt_bytes[3]
                payload = pkt_bytes[4:4 + plen]
                cs = pkt_bytes[4 + plen]
                exp_cs = ((plen + cmd + sum(payload)) - 1) & 0xFF
                valid = (cs == exp_cs)

                del self.buf[:total_len]
                del self.ts_buf[:total_len]

                return SerialPacket(
                    ts=pkt_ts,
                    source=self.source,
                    proto="hiworld",
                    raw_bytes=pkt_bytes,
                    cmd=cmd,
                    payload=payload,
                    checksum=cs,
                    valid_cs=valid
                )

            # 2. Raise: 0x2E <cmd> <len> <payload...> <cs>
            if self.proto in ("auto", "raise") and b0 == 0x2E:
                if len(self.buf) < 3:
                    return None
                cmd = self.buf[1]
                plen = self.buf[2]
                if plen > 64:
                    del self.buf[0]
                    del self.ts_buf[0]
                    continue
                total_len = plen + 4
                if len(self.buf) < total_len:
                    return None

                pkt_bytes = bytes(self.buf[:total_len])
                pkt_ts = self.ts_buf[0]
                payload = pkt_bytes[3:3 + plen]
                cs = pkt_bytes[3 + plen]
                exp_cs = (~(cmd + plen + sum(payload))) & 0xFF
                valid = (cs == exp_cs)

                del self.buf[:total_len]
                del self.ts_buf[:total_len]

                return SerialPacket(
                    ts=pkt_ts,
                    source=self.source,
                    proto="raise",
                    raw_bytes=pkt_bytes,
                    cmd=cmd,
                    payload=payload,
                    checksum=cs,
                    valid_cs=valid
                )

            # 3. Bagoo: 0xFD <total_len> <cmd> <payload...> <cs>
            if self.proto in ("auto", "bagoo") and b0 == 0xFD:
                if len(self.buf) < 3:
                    return None
                tlen = self.buf[1]
                if tlen < 3 or tlen > 67:
                    del self.buf[0]
                    del self.ts_buf[0]
                    continue
                total_len = tlen + 1
                if len(self.buf) < total_len:
                    return None

                pkt_bytes = bytes(self.buf[:total_len])
                pkt_ts = self.ts_buf[0]
                cmd = pkt_bytes[2]
                payload = pkt_bytes[3:total_len - 1]
                cs = pkt_bytes[total_len - 1]
                exp_cs = (tlen + cmd + sum(payload)) & 0xFF
                valid = (cs == exp_cs)

                del self.buf[:total_len]
                del self.ts_buf[:total_len]

                return SerialPacket(
                    ts=pkt_ts,
                    source=self.source,
                    proto="bagoo",
                    raw_bytes=pkt_bytes,
                    cmd=cmd,
                    payload=payload,
                    checksum=cs,
                    valid_cs=valid
                )

            # Drop unmatched leading byte
            del self.buf[0]
            del self.ts_buf[0]

        return None


def format_hex_spaced(data_bytes: bytes) -> str:
    return " ".join(f"{b:02x}" for b in data_bytes)


def highlight_diff(data_orig: bytes, data_app: bytes, color: bool = True) -> Tuple[str, str, List[Dict[str, Any]]]:
    """Generates formatted strings highlighting differing bytes and returns list of diff details."""
    diffs = []
    orig_parts = []
    app_parts = []

    max_len = max(len(data_orig), len(data_app))
    for i in range(max_len):
        b_orig = data_orig[i] if i < len(data_orig) else None
        b_app = data_app[i] if i < len(data_app) else None

        if b_orig is not None and b_app is not None:
            if b_orig != b_app:
                diffs.append({
                    "index": i,
                    "orig": b_orig,
                    "app": b_app,
                    "xor": b_orig ^ b_app
                })
                if color:
                    orig_parts.append(f"{Ansi.BOLD}{Ansi.RED}[{b_orig:02x}]{Ansi.RESET}")
                    app_parts.append(f"{Ansi.BOLD}{Ansi.GREEN}[{b_app:02x}]{Ansi.RESET}")
                else:
                    orig_parts.append(f"[{b_orig:02x}]")
                    app_parts.append(f"[{b_app:02x}]")
            else:
                orig_parts.append(f"{b_orig:02x}")
                app_parts.append(f"{b_app:02x}")
        elif b_orig is not None:
            diffs.append({
                "index": i,
                "orig": b_orig,
                "app": None,
                "xor": 0
            })
            if color:
                orig_parts.append(f"{Ansi.BOLD}{Ansi.RED}[{b_orig:02x}]{Ansi.RESET}")
            else:
                orig_parts.append(f"[{b_orig:02x}]")
        else:
            diffs.append({
                "index": i,
                "orig": None,
                "app": b_app,
                "xor": 0
            })
            if color:
                app_parts.append(f"{Ansi.BOLD}{Ansi.GREEN}[{b_app:02x}]{Ansi.RESET}")
            else:
                app_parts.append(f"[{b_app:02x}]")

    return " ".join(orig_parts), " ".join(app_parts), diffs


def match_prefix(packet: SerialPacket, prefixes: List[str]) -> bool:
    """Checks if a packet matches any of the user-provided prefixes or command opcodes."""
    if not prefixes:
        return True

    raw_hex = packet.raw_hex
    cmd_hex = f"{packet.cmd:02x}" if packet.cmd is not None else ""

    for p in prefixes:
        p_clean = p.lower().strip()
        if p_clean.startswith("0x"):
            p_clean = p_clean[2:]
        p_clean = re.sub(r"[^0-9a-f]", "", p_clean)
        if not p_clean:
            continue

        # If prefix is 1 byte (2 hex chars) and matches command opcode:
        if len(p_clean) == 2 and cmd_hex == p_clean:
            return True
        # If prefix matches start of raw frame:
        if raw_hex.startswith(p_clean):
            return True
        # If prefix matches command embedded in 5aa5..cmd
        if raw_hex.startswith("5aa5") and len(raw_hex) >= 8:
            if raw_hex[6:6 + len(p_clean)] == p_clean:
                return True
    return False


def parse_can_id_arg(can_ids_arg: Optional[List[str]]) -> Optional[set]:
    if not can_ids_arg:
        return None
    res = set()
    for item in can_ids_arg:
        for piece in re.split(r"[,;\s]+", item):
            piece = piece.strip()
            if piece:
                if piece.lower().startswith("0x"):
                    piece = piece[2:]
                try:
                    val = int(piece, 16)
                    res.add(val)
                except ValueError:
                    pass
    return res if res else None


def parse_prefix_arg(prefix_arg: Optional[List[str]]) -> List[str]:
    if not prefix_arg:
        return []
    res = []
    for item in prefix_arg:
        for piece in re.split(r"[,;\s]+", item):
            piece = piece.strip()
            if piece:
                res.append(piece)
    return res


def run_diff(
    log_path: str,
    can_ids_filter: Optional[set],
    prefix_filter: List[str],
    time_window: float = 1.2,
    dedup: bool = True,
    keep_heartbeat: bool = False,
    use_color: bool = True,
    summary_only: bool = False,
    export_compressed_path: Optional[str] = None
):
    if not os.path.isfile(log_path):
        print(f"[ERROR] File not found: {log_path}", file=sys.stderr)
        sys.exit(1)

    # Stream Reassemblers
    orig_parser = StreamReassembler(source="ORIG", proto="auto")
    app_parser = StreamReassembler(source="APP", proto="auto")

    # State tracking
    can_state: Dict[int, bytes] = {}
    recent_can_actions: List[CanEvent] = []
    
    # Packets
    orig_packets: List[SerialPacket] = []
    app_packets: List[SerialPacket] = []

    # Timeline of parsed items
    raw_lines_count = 0
    can_frames_count = 0
    can_actions_count = 0

    compressed_timeline = []

    # Read and parse log file
    with open(log_path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            raw_lines_count += 1
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

            tag = parts[1]
            rest = parts[2] if len(parts) > 2 else ""

            # 1. CAN stream
            if tag == "[CAN]":
                can_frames_count += 1
                m = re.search(r"ID:([0-9A-Fa-f]+)\s+DLC:(\d+)\s+DATA:([0-9A-Fa-f]*)", rest)
                if m:
                    cid = int(m.group(1), 16)
                    dlc = int(m.group(2))
                    data_hex = m.group(3)
                    data = bytes.fromhex(data_hex) if data_hex else b""

                    # Check if interesting CAN ID
                    if can_ids_filter is not None and cid not in can_ids_filter:
                        continue

                    prev = can_state.get(cid)
                    is_action = (prev is None or prev != data)
                    can_state[cid] = data

                    if is_action:
                        can_actions_count += 1
                        ev = CanEvent(ts=ts, can_id=cid, dlc=dlc, data=data, prev_data=prev)
                        recent_can_actions.append(ev)
                        compressed_timeline.append(("CAN", ev))

            # 2. APP serial output
            elif tag == "[APP_UART_TX]":
                new_pkts = app_parser.feed(rest, ts)
                for p in new_pkts:
                    if not keep_heartbeat and p.cmd == 0xFF:
                        continue
                    if match_prefix(p, prefix_filter):
                        app_packets.append(p)
                        compressed_timeline.append(("APP", p))

            # 3. ORIG reference serial output
            elif tag == "[ORIG_CANBOX]":
                if rest.startswith("RX: "):
                    hex_chunk = rest[4:]
                    new_pkts = orig_parser.feed(hex_chunk, ts)
                    for p in new_pkts:
                        if not keep_heartbeat and p.cmd == 0xFF:
                            continue
                        if match_prefix(p, prefix_filter):
                            orig_packets.append(p)
                            compressed_timeline.append(("ORIG", p))

    # Optional export of compressed log
    if export_compressed_path:
        with open(export_compressed_path, "w", encoding="utf-8") as exp_f:
            for item_type, item in compressed_timeline:
                if item_type == "CAN":
                    exp_f.write(f"{item.ts:.3f} [CAN] ID:{item.can_id_hex} DLC:{item.dlc} DATA:{item.data_hex}\n")
                elif item_type == "APP":
                    exp_f.write(f"{item.ts:.3f} [APP_UART_TX] {item.raw_hex}\n")
                elif item_type == "ORIG":
                    exp_f.write(f"{item.ts:.3f} [ORIG_CANBOX] RX: {item.raw_hex}\n")
        print(f"[*] Compressed log exported to: {export_compressed_path}")

    # Deduplicate consecutive identical serial frames if requested
    def dedup_packets(pkts: List[SerialPacket]) -> List[SerialPacket]:
        if not dedup or not pkts:
            return pkts
        deduped = []
        for p in pkts:
            if deduped and deduped[-1].raw_hex == p.raw_hex and (p.ts - deduped[-1].ts) < 2.0:
                deduped[-1].repeat_count += 1
            else:
                deduped.append(p)
        return deduped

    orig_deduped = dedup_packets(orig_packets)
    app_deduped = dedup_packets(app_packets)

    # Print Header
    header_color = Ansi.CYAN if use_color else ""
    reset_color = Ansi.RESET if use_color else ""
    bold_color = Ansi.BOLD if use_color else ""

    print(f"\n{bold_color}{header_color}================================================================================")
    print("                    OPENCANBOX SERIAL FRAME DIFFERENCE REPORT                   ")
    print(f"================================================================================{reset_color}")
    print(f"  Log File        : {log_path}")
    print(f"  Raw Lines       : {raw_lines_count:,}")
    print(f"  CAN Frames Seen : {can_frames_count:,} (Changed Actions: {can_actions_count:,})")
    print(f"  Target CAN IDs  : {', '.join(f'0x{i:X}' for i in sorted(can_ids_filter)) if can_ids_filter else 'ALL (Any CAN change)'}")
    print(f"  Target Prefixes : {', '.join(prefix_filter) if prefix_filter else 'ALL'}")
    print(f"  Real Canbox Pkts: {len(orig_packets)} (Deduped: {len(orig_deduped)})")
    print(f"  App Canbox Pkts : {len(app_packets)} (Deduped: {len(app_deduped)})")
    print(f"{bold_color}{header_color}--------------------------------------------------------------------------------{reset_color}\n")

    if not orig_deduped and not app_deduped:
        print("[!] No matching serial packets found for the given criteria.")
        return

    # Correlation & Pairing Engine
    # We correlate serial packets to the preceding CAN actions within `time_window`.
    # And we pair ORIG and APP packets that occur in proximity with matching command or sequence.
    used_app_indices = set()
    pairs = []

    for o_idx, o_pkt in enumerate(orig_deduped):
        # Find candidate APP packets within time_window
        best_a_idx = None
        min_dt = float("inf")

        for a_idx, a_pkt in enumerate(app_deduped):
            if a_idx in used_app_indices:
                continue
            # If both frames have a decoded command ID, they MUST match
            if o_pkt.cmd is not None and a_pkt.cmd is not None:
                if o_pkt.cmd != a_pkt.cmd:
                    continue
            elif o_pkt.proto != a_pkt.proto:
                continue

            dt = abs(a_pkt.ts - o_pkt.ts)
            if dt <= time_window:
                if dt < min_dt:
                    min_dt = dt
                    best_a_idx = a_idx

        if best_a_idx is not None:
            used_app_indices.add(best_a_idx)
            pairs.append((o_pkt, app_deduped[best_a_idx]))
        else:
            pairs.append((o_pkt, None))

    # Add remaining unpaired APP packets
    for a_idx, a_pkt in enumerate(app_deduped):
        if a_idx not in used_app_indices:
            pairs.append((None, a_pkt))

    # Sort pairs by timestamp
    pairs.sort(key=lambda p: (p[0].ts if p[0] else p[1].ts))

    diff_summary_by_cmd = {}
    event_counter = 0

    for o_pkt, a_pkt in pairs:
        event_counter += 1
        t_ref = o_pkt.ts if o_pkt else a_pkt.ts
        t_end = a_pkt.ts if a_pkt else o_pkt.ts
        t_min = min(t_ref, t_end)
        t_max = max(t_ref, t_end)

        # Find CAN actions that preceded this serial event (within window)
        can_actions_for_event = [
            ev for ev in recent_can_actions
            if (t_min - time_window) <= ev.ts <= (t_max + 0.05)
        ]

        # Group CAN actions by ID keeping latest
        unique_can_actions = []
        seen_cids = set()
        for ev in reversed(can_actions_for_event):
            if ev.can_id not in seen_cids:
                seen_cids.add(ev.can_id)
                unique_can_actions.append(ev)
        unique_can_actions.reverse()

        cmd_label = ""
        if o_pkt and o_pkt.cmd is not None:
            cmd_label = f"CMD: 0x{o_pkt.cmd:02X} ({o_pkt.cmd_name()})"
        elif a_pkt and a_pkt.cmd is not None:
            cmd_label = f"CMD: 0x{a_pkt.cmd:02X} ({a_pkt.cmd_name()})"
        else:
            cmd_label = "SERIAL FRAME"

        # Check difference
        has_diff = False
        diff_details = []
        orig_formatted = ""
        app_formatted = ""

        if o_pkt and a_pkt:
            if o_pkt.raw_hex != a_pkt.raw_hex:
                has_diff = True
                orig_formatted, app_formatted, diff_details = highlight_diff(
                    o_pkt.raw, a_pkt.raw, color=use_color
                )
            else:
                orig_formatted = format_hex_spaced(o_pkt.raw)
                app_formatted = format_hex_spaced(a_pkt.raw)
        elif o_pkt and not a_pkt:
            has_diff = True
            orig_formatted = format_hex_spaced(o_pkt.raw)
            app_formatted = f"{Ansi.YELLOW}[NO APP RESPONSE DETECTED]{Ansi.RESET}" if use_color else "[NO APP RESPONSE DETECTED]"
        elif a_pkt and not o_pkt:
            has_diff = True
            orig_formatted = f"{Ansi.YELLOW}[NO OEM CANBOX FRAME]{Ansi.RESET}" if use_color else "[NO OEM CANBOX FRAME]"
            app_formatted = format_hex_spaced(a_pkt.raw)

        # Update stats
        cmd_key = cmd_label
        if cmd_key not in diff_summary_by_cmd:
            diff_summary_by_cmd[cmd_key] = {
                "total": 0,
                "matched_diff": 0,
                "matched_same": 0,
                "missing_app": 0,
                "extra_app": 0,
                "byte_diffs": set()
            }
        diff_summary_by_cmd[cmd_key]["total"] += 1
        if o_pkt and a_pkt:
            if o_pkt.raw_hex != a_pkt.raw_hex:
                diff_summary_by_cmd[cmd_key]["matched_diff"] += 1
                for d in diff_details:
                    diff_summary_by_cmd[cmd_key]["byte_diffs"].add(d["index"])
            else:
                diff_summary_by_cmd[cmd_key]["matched_same"] += 1
        elif o_pkt and not a_pkt:
            diff_summary_by_cmd[cmd_key]["missing_app"] += 1
        elif a_pkt and not o_pkt:
            diff_summary_by_cmd[cmd_key]["extra_app"] += 1

        if summary_only:
            continue

        # Format Block Output
        border = "=" * 80
        sep = "-" * 80
        print(f"{border}")
        dt_str = f" (dt: {a_pkt.ts - o_pkt.ts:+.3f}s)" if (o_pkt and a_pkt) else ""
        print(f"EVENT #{event_counter} | Time: {t_min:.3f} - {t_max:.3f}{dt_str} | {cmd_label}")
        print(f"{sep}")

        # 1. CAN Action
        print(f"{bold_color}CAN ACTION:{reset_color}")
        if unique_can_actions:
            for ev in unique_can_actions:
                # Format changed bytes with brackets
                data_str_parts = []
                for b_idx in range(len(ev.data)):
                    b_val = ev.data[b_idx]
                    is_chg = any(c[0] == b_idx for c in ev.changed_bytes)
                    if is_chg:
                        if use_color:
                            data_str_parts.append(f"{Ansi.YELLOW}[{b_val:02x}]{Ansi.RESET}")
                        else:
                            data_str_parts.append(f"[{b_val:02x}]")
                    else:
                        data_str_parts.append(f"{b_val:02x}")
                data_repr = " ".join(data_str_parts)

                delta_desc = []
                for b_idx, b_old, b_new in ev.changed_bytes:
                    old_str = f"0x{b_old:02X}" if b_old is not None else "None"
                    new_str = f"0x{b_new:02X}" if b_new is not None else "None"
                    delta_desc.append(f"Byte {b_idx}: {old_str} -> {new_str}")

                delta_str = f" | Delta: {', '.join(delta_desc)}" if delta_desc else " (First occurrence)"
                print(f"  {ev.ts:.3f} | ID: 0x{ev.can_id_hex:<4} (DLC: {ev.dlc}) DATA: {data_repr}{delta_str}")
        else:
            print("  (No changed CAN frames observed within correlation window)")

        print()

        # 2. Serial Differences
        print(f"{bold_color}CANBOX APP DIFFERENCE:{reset_color}")
        if o_pkt:
            cs_status = "" if o_pkt.valid_cs else f" {Ansi.RED}[INVALID CS]{Ansi.RESET}"
            rep_str = f" (x{o_pkt.repeat_count})" if o_pkt.repeat_count > 1 else ""
            print(f"  ORIG ({o_pkt.ts:.3f}): {orig_formatted}{rep_str}{cs_status}")
        else:
            print(f"  ORIG: [No Packet Emitted by Real Canbox]")

        if a_pkt:
            cs_status = "" if a_pkt.valid_cs else f" {Ansi.RED}[INVALID CS]{Ansi.RESET}"
            rep_str = f" (x{a_pkt.repeat_count})" if a_pkt.repeat_count > 1 else ""
            print(f"  APP  ({a_pkt.ts:.3f}): {app_formatted}{rep_str}{cs_status}")
        else:
            print(f"  APP : [No Response Emitted by OpenCanbox App]")

        # Detailed breakdown of differences
        if has_diff:
            if o_pkt and a_pkt:
                print(f"\n  {Ansi.RED if use_color else ''}DIFFERENCE BREAKDOWN:{reset_color}")
                if len(o_pkt.payload) != len(a_pkt.payload):
                    print(f"    * Payload Length Mismatch: ORIG={len(o_pkt.payload)} bytes vs APP={len(a_pkt.payload)} bytes")
                for d in diff_details:
                    b_idx = d["index"]
                    o_v = d["orig"]
                    a_v = d["app"]
                    if o_v is not None and a_v is not None:
                        is_cs = (o_pkt.checksum is not None and b_idx == len(o_pkt.raw) - 1)
                        label = "Checksum Byte" if is_cs else f"Frame Byte {b_idx}"
                        print(f"    * {label:<16}: ORIG=0x{o_v:02X} ({o_v:>3})  !=  APP=0x{a_v:02X} ({a_v:>3})  [XOR: 0x{d['xor']:02X}]")
                    elif o_v is not None:
                        print(f"    * Frame Byte {b_idx:<16}: ORIG=0x{o_v:02X}  !=  APP=<missing>")
                    else:
                        print(f"    * Frame Byte {b_idx:<16}: ORIG=<missing>  !=  APP=0x{a_v:02X}")
            elif o_pkt and not a_pkt:
                print(f"\n  {Ansi.RED if use_color else ''}BUG DETECTED: Real canbox emitted {cmd_label}, but OpenCanbox App was silent!{reset_color}")
            elif a_pkt and not o_pkt:
                print(f"\n  {Ansi.YELLOW if use_color else ''}NOTE: OpenCanbox App emitted {cmd_label}, but Real canbox did not emit this frame.{reset_color}")
        else:
            print(f"\n  {Ansi.GREEN if use_color else ''}[IDENTICAL] Real Canbox and App produced identical serial frames.{reset_color}")

        print()

    # Summary Section
    print(f"{bold_color}{header_color}================================================================================")
    print("                               ANALYSIS SUMMARY                                 ")
    print(f"================================================================================{reset_color}")
    print(f"  Total Event Pairs      : {len(pairs)}")
    matched_diff = sum(stats["matched_diff"] for stats in diff_summary_by_cmd.values())
    matched_same = sum(stats["matched_same"] for stats in diff_summary_by_cmd.values())
    missing_app = sum(stats["missing_app"] for stats in diff_summary_by_cmd.values())
    extra_app = sum(stats["extra_app"] for stats in diff_summary_by_cmd.values())
    print(f"  Matched & Identical    : {matched_same}")
    print(f"  Matched with Diff      : {matched_diff}")
    print(f"  Missing in App (Silent): {missing_app}")
    print(f"  Extra/Spurious in App  : {extra_app}")
    print()
    print("  Command Breakdown:")
    for cmd_key, stats in sorted(diff_summary_by_cmd.items()):
        details = []
        if stats["matched_diff"] > 0:
            details.append(f"{stats['matched_diff']} diffs")
        if stats["matched_same"] > 0:
            details.append(f"{stats['matched_same']} identical")
        if stats["missing_app"] > 0:
            details.append(f"{stats['missing_app']} missing in app")
        if stats["extra_app"] > 0:
            details.append(f"{stats['extra_app']} extra in app")
        detail_str = ", ".join(details)
        b_diff_str = f"Differing Byte Indices: {sorted(list(stats['byte_diffs']))}" if stats['byte_diffs'] else ""
        sep_char = " | " if (detail_str and b_diff_str) else ""
        print(f"    - {cmd_key:<30}: {detail_str}{sep_char}{b_diff_str}")
    print(f"{bold_color}{header_color}================================================================================{reset_color}\n")


def main():
    parser = argparse.ArgumentParser(
        description="Identifies differences in serial frames between Real Canbox and App in relation to CAN actions."
    )
    parser.add_argument(
        "log",
        nargs="?",
        default=None,
        help="Path to dump log file (dump_*.log)"
    )
    parser.add_argument(
        "-l", "--log",
        dest="log_opt",
        help="Path to dump log file (alternative to positional)"
    )
    parser.add_argument(
        "-c", "--can-id", "--can-ids",
        dest="can_ids",
        nargs="+",
        help="Filter interesting CAN IDs in hex (e.g. -c 1D0 1E3 or -c 1d0,12d)"
    )
    parser.add_argument(
        "--preset",
        dest="preset",
        choices=["radar", "hvac", "doors", "swc", "trip", "tpms", "alerts", "bsi_config"],
        help="Quick PSA 407 preset for CAN IDs and serial prefixes (radar, hvac, doors, swc, trip, tpms, alerts, bsi_config)"
    )
    parser.add_argument(
        "-p", "--prefix", "--prefixes", "--canbox-prefix",
        dest="prefixes",
        nargs="+",
        help="Filter interesting serial frame prefixes or command opcodes (e.g. -p 31 or -p 5aa50c31)"
    )
    parser.add_argument(
        "-w", "--window",
        dest="window",
        type=float,
        default=1.2,
        help="Max time window in seconds to correlate CAN actions with serial responses (default: 1.2s)"
    )
    parser.add_argument(
        "--no-dedup",
        dest="dedup",
        action="store_false",
        default=True,
        help="Disable deduplication of repeated consecutive CAN and serial frames"
    )
    parser.add_argument(
        "--keep-heartbeat",
        dest="keep_heartbeat",
        action="store_true",
        default=False,
        help="Include periodic heartbeat frames (0xFF / 5aa501ff...)"
    )
    parser.add_argument(
        "--no-color",
        dest="no_color",
        action="store_true",
        default=False,
        help="Disable ANSI colors in output"
    )
    parser.add_argument(
        "-s", "--summary",
        dest="summary_only",
        action="store_true",
        default=False,
        help="Only display the summary statistics table"
    )
    parser.add_argument(
        "--export-compressed",
        dest="export_compressed",
        help="Save deduplicated/compressed event log to specified file"
    )

    args = parser.parse_args()

    log_file = args.log or args.log_opt
    if not log_file:
        parser.print_help()
        print("\n[ERROR] Log file is required. Provide path as first argument or via -l / --log.")
        sys.exit(1)

    PSA_PRESETS = {
        "radar": (["0E1", "260"], ["41"]),
        "hvac": (["1D0", "1E3", "0F6"], ["31"]),
        "doors": (["036", "0F6"], ["12"]),
        "swc": (["228", "128"], ["11", "21", "22"]),
        "trip": (["221", "261", "2A1", "0B6", "0F6"], ["13", "14", "15"]),
        "tpms": (["3A1", "348", "1E1"], ["18", "66"]),
        "alerts": (["1A1", "120", "168"], ["42"]),
        "bsi_config": (["39B", "2A8"], ["71", "72", "76", "79"]),
    }

    can_ids_arg = args.can_ids
    prefixes_arg = args.prefixes
    if args.preset and args.preset in PSA_PRESETS:
        preset_cids, preset_prefs = PSA_PRESETS[args.preset]
        if not can_ids_arg:
            can_ids_arg = preset_cids
        if not prefixes_arg:
            prefixes_arg = preset_prefs

    can_ids = parse_can_id_arg(can_ids_arg)
    prefixes = parse_prefix_arg(prefixes_arg)
    use_color = (not args.no_color) and sys.stdout.isatty()

    try:
        run_diff(
            log_path=log_file,
            can_ids_filter=can_ids,
            prefix_filter=prefixes,
            time_window=args.window,
            dedup=args.dedup,
            keep_heartbeat=args.keep_heartbeat,
            use_color=use_color,
            summary_only=args.summary_only,
            export_compressed_path=args.export_compressed
        )
    except BrokenPipeError:
        try:
            sys.stdout.close()
        except Exception:
            pass
        sys.exit(0)


if __name__ == "__main__":
    main()
