#!/usr/bin/env python3
"""
OpenCanbox Alert Log & Diagnostic Extraction Tool
=================================================
Extracts, decodes, and analyzes PSA CAN2004 vehicle alert requests and journals
from E2E dump logs (dump_*.log).

Key Extracted Data:
1. Alert Log Requests & Triggers:
   - Cockpit CHECK push-button pulses (CAN ID 0x221 bit 5)
   - Display target alerts (CAN ID 0x167 bytes 4..5)
   - Diagnostic requests to BSI (CAN ID 0x39B)
   - Head Unit Downlink queries (APP_UART_RX Cmd 0x2F, Cmd 0x24 handshake)
2. Alert Log Journal (CAN ID 0x120):
   - Reassembled 21-byte bitfield (168 bits) from multiplexed blocks 0x7C, 0xBC, 0xFC
   - Fully decoded active faults with CAN Alarm ID, Severity, Description, and Category
3. Real-Time Rolling Alerts (CAN ID 0x1A1):
   - Sequence toggle, display request, priority, chime, door masks, and parameters
4. Uplink Serial Alerts (APP_UART_TX vs ORIG_CANBOX):
   - Hiworld Cmd 0x42 (Warning Info) & Cmd 0xEA (Extended Warning)
   - Validation against CAN ground truth (detects phantom or missing alerts)
"""

import argparse
import csv
import os
import re
import sys
from collections import defaultdict
from typing import Dict, List, Optional, Tuple, Any

# Standard Hiworld Command Names
HIWORLD_CMD_NAMES = {
    0x11: "CAR_BASE_INFO / SWC",
    0x12: "DOORS_WINDOWS",
    0x13: "TRIP_INFO_1",
    0x14: "TRIP_INFO_2",
    0x15: "TRIP_INFO_3",
    0x16: "TIRE_PRESSURE_DISCRETE",
    0x18: "TIRE_PRESSURE_NUMERIC",
    0x21: "WHEEL_BUTTONS",
    0x22: "STALK_BUTTONS",
    0x24: "CAR_MODEL_HANDSHAKE",
    0x2F: "DIAGNOSTIC_ALERT_QUERY",
    0x31: "HVAC_STATUS",
    0x41: "PARKING_RADAR",
    0x42: "WARNING_INFO_ALERTS",
    0x71: "FEATURE_CONFIG_1",
    0x72: "FEATURE_CONFIG_2",
    0x76: "PANEL_KEYS",
    0x79: "SYSTEM_STATUS",
    0xEA: "CUSTOM_EXTENDED_ALERTS",
    0xF0: "VERSION_REPORT",
    0xFF: "HEARTBEAT"
}

SEVERITY_NAMES = {
    0: "ℹ️ INFO (0)",
    1: "⚠️ SERVICE (1)",
    2: "🛑 STOP (2)",
    3: "🛑 CRITICAL (3)"
}

CHIME_NAMES = {
    0: "No chime",
    1: "Single chime (Ding)",
    2: "Double chime",
    3: "Triple chime",
    4: "Warning chord (Gong)",
    6: "Rapid beep"
}

DOOR_NAMES = [
    (0x01, "Front Left"),
    (0x02, "Front Right"),
    (0x04, "Rear Left"),
    (0x08, "Rear Right"),
    (0x10, "Trunk / Boot"),
    (0x20, "Bonnet / Hood"),
    (0x40, "Rear Screen"),
    (0x80, "Roof / Reserved")
]


KNOWN_AUTOMOTIVE_ALERTS = {
    0x0000: ("Diagnosis OK", "ℹ️ INFO (0)", "BSI Diagnostics"),
    0x0001: ("Engine temperature fault: Stop the vehicle", "🛑 STOP (2)", "Powertrain / Cooling"),
    0x0002: ("Warning, engine oil pressure", "🛑 STOP (2)", "Engine Lubrication"),
    0x0003: ("Top up coolant level", "⚠️ SERVICE (1)", "Cooling System"),
    0x0004: ("Top up engine oil level", "⚠️ SERVICE (1)", "Engine Lubrication"),
    0x0005: ("Engine oil pressure fault: Stop the vehicle", "🛑 STOP (2)", "Engine Lubrication"),
    0x0008: ("Braking system faulty", "🛑 STOP (2)", "Braking System"),
    0x000A: ("Air suspension OK: vehicle leveled", "ℹ️ INFO (0)", "Active Suspension"),
    0x000B: ("Door(s) open", "ℹ️ INFO (0)", "Doors / Access"),
    0x000C: ("Handbrake on !", "ℹ️ INFO (0)", "Braking System"),
    0x000D: ("Puncture(s) in tyres detected", "🛑 STOP (2)", "TPMS"),
    0x000E: ("Power steering faulty", "⚠️ SERVICE (1)", "Steering"),
    0x000F: ("Risk of particle filter clogging: See handbook", "⚠️ SERVICE (1)", "Depollution / DPF"),
    0x0011: ("Suspension faulty: Max speed 90 km/h", "⚠️ SERVICE (1)", "Active Suspension"),
    0x0061: ("Service due", "ℹ️ INFO (0)", "Maintenance"),
    0x0064: ("Handbrake cable fault auto handbrake activated", "⚠️ SERVICE (1)", "Electric Parking Brake"),
    0x0066: ("Braking system faulty", "🛑 STOP (2)", "Braking System"),
    0x0067: ("Brake pads worn", "⚠️ SERVICE (1)", "Braking Wear"),
    0x0068: ("Handbrake faulty", "⚠️ SERVICE (1)", "Parking Brake"),
    0x0069: ("Mobile deflector faulty", "⚠️ SERVICE (1)", "Aerodynamics / Spoiler"),
    0x006A: ("ABS system faulty", "⚠️ SERVICE (1)", "Braking / Safety"),
    0x006B: ("ESP/ASR system faulty", "⚠️ SERVICE (1)", "Traction Control"),
    0x006C: ("Suspension faulty", "⚠️ SERVICE (1)", "Active Suspension"),
    0x006D: ("Power steering faulty", "🛑 STOP (2)", "Steering System"),
    0x006E: ("Gearbox fault: Repair needed", "⚠️ SERVICE (1)", "Automatic Gearbox"),
    0x006F: ("Speed control system faulty", "⚠️ SERVICE (1)", "Cruise Control"),
    0x0072: ("Suspension faulty: Max speed 90 km/h", "⚠️ SERVICE (1)", "Active Suspension"),
    0x0073: ("Ambient brightness sensor faulty", "ℹ️ INFO (0)", "Lighting Sensors"),
    0x0074: ("Bulb faulty", "⚠️ SERVICE (1)", "Lighting"),
    0x0075: ("Automatic headlamp adjustment faulty", "⚠️ SERVICE (1)", "Xenon / Lighting"),
    0x0076: ("Directional headlamps faulty", "⚠️ SERVICE (1)", "Lighting ECU"),
    0x0078: ("Airbag(s) or pretensioner seat belt(s) faulty", "⚠️ SERVICE (1)", "Safety / Airbags"),
    0x0079: ("Active bonnet faulty", "⚠️ SERVICE (1)", "Safety / Pyrotechnics"),
    0x007A: ("Automatic gearbox faulty", "⚠️ SERVICE (1)", "Transmission"),
    0x007D: ("Presence of water in diesel filter: Repair needed", "⚠️ SERVICE (1)", "Fuel System"),
    0x007E: ("Engine management system faulty", "⚠️ SERVICE (1)", "Engine Management"),
    0x007F: ("Depollution system faulty", "⚠️ SERVICE (1)", "Emissions / Engine"),
    0x0081: ("Particle filter additive level too low: Repair needed", "⚠️ SERVICE (1)", "Eolys / DPF"),
    0x0083: ("Immobiliser faulty", "⚠️ SERVICE (1)", "Security / Immobilizer"),
    0x0086: ("Right-hand sliding side door faulty", "⚠️ SERVICE (1)", "Body / Sliding Doors"),
    0x0087: ("Left-hand sliding side door faulty", "⚠️ SERVICE (1)", "Body / Sliding Doors"),
    0x0088: ("Parking assistance system faulty", "⚠️ SERVICE (1)", "Parking Radar / AAS"),
    0x0089: ("Space measuring system faulty", "⚠️ SERVICE (1)", "Parking Space Measurement"),
    0x008A: ("Battery charge faulty", "⚠️ SERVICE (1)", "Electrical / Alternator"),
    0x0097: ("Anti-wander lane-crossing warning device faulty", "⚠️ SERVICE (1)", "AFIL Lane Assist"),
    0x009A: ("Dipped headlamp bulb faulty", "⚠️ SERVICE (1)", "Lighting"),
    0x009B: ("Main beam headlamp bulb faulty", "⚠️ SERVICE (1)", "Lighting"),
    0x009C: ("Left hand brake light bulb faulty", "⚠️ SERVICE (1)", "Lighting"),
    0x009D: ("Foglamp bulb faulty", "⚠️ SERVICE (1)", "Lighting"),
    0x009E: ("Direction indicators faulty", "⚠️ SERVICE (1)", "Lighting"),
    0x009F: ("Left hand reversing light bulb faulty", "⚠️ SERVICE (1)", "Lighting"),
    0x00A0: ("Sidelamp bulb faulty", "⚠️ SERVICE (1)", "Lighting"),
    0x00CB: ("Automatic screen wipe activated", "ℹ️ INFO (0)", "Wipers"),
    0x00CD: ("No cruise control: speed low", "ℹ️ INFO (0)", "Speed Limiter / Cruise"),
    0x00CE: ("Cruise control activation impossible: enter speed", "ℹ️ INFO (0)", "Cruise Control"),
    0x00D1: ("Active bonnet deployed", "ℹ️ INFO (0)", "Pedestrian Protection"),
    0x00D3: ("None of rear passenger seat belts fastened", "⚠️ SERVICE (1)", "Safety / Seatbelt Monitor"),
    0x00D7: ("Put automatic gearbox in \"P\" position", "ℹ️ INFO (0)", "Gearbox Interlock"),
    0x00D8: ("Risk of black ice", "ℹ️ INFO (0)", "Weather / Safety"),
    0x00D9: ("Handbrake on !", "ℹ️ INFO (0)", "Parking Brake"),
    0x00DE: ("Front left hand door open", "⚠️ SERVICE (1)", "Doors / Access"),
    0x00DF: ("Screen washer fluid level low", "ℹ️ INFO (0)", "Driver Convenience"),
    0x00E0: ("Fuel level low", "ℹ️ INFO (0)", "Fuel Reserve"),
    0x00E1: ("Fuel circuit deactivated", "🛑 STOP (2)", "Crash Cutoff"),
    0x00E3: ("Remote control battery spent", "ℹ️ INFO (0)", "Key Battery"),
    0x00E4: ("Check and re-initialise tyre pressure", "ℹ️ INFO (0)", "TPMS System"),
    0x00E5: ("Tyre pressure(s) not monitored", "⚠️ SERVICE (1)", "TPMS"),
    0x00E8: ("Tyre pressure too low", "⚠️ SERVICE (1)", "Under-Inflation / TPMS"),
    0x00EA: ("Hands free starting system faulty", "⚠️ SERVICE (1)", "Keyless Access"),
    0x00EB: ("Starting phase has failed (consult handbook)", "⚠️ SERVICE (1)", "Powertrain / Starter"),
    0x00EC: ("Prolonged starting in progress", "ℹ️ INFO (0)", "Powertrain / Starter"),
    0x00ED: ("Starting impossible : unlock steering", "⚠️ SERVICE (1)", "Steering Lock"),
    0x00EF: ("Remote control not detected", "ℹ️ INFO (0)", "Keyless Access"),
    0x00F0: ("Diagnosis in progress...", "ℹ️ INFO (0)", "Cockpit CHECK / BSI"),
    0x00F1: ("Diagnosis completed", "ℹ️ INFO (0)", "Cockpit CHECK / BSI"),
    0x00F7: ("Rear LH seat belt not fastened", "⚠️ SERVICE (1)", "Safety / Seatbelt Monitor"),
    0x00F8: ("Rear center seat belt not fastened", "⚠️ SERVICE (1)", "Safety / Seatbelt Monitor"),
    0x00F9: ("Rear RH seat belt not fastened", "⚠️ SERVICE (1)", "Safety / Seatbelt Monitor"),
    0x012F: ("Automatic windscreen wiper activated", "ℹ️ INFO (0)", "Rain Sensor"),
    0x0130: ("Automatic windscreen wiper deactivated", "ℹ️ INFO (0)", "Rain Sensor"),
    0x0131: ("Automatic headlamp lighting activated", "ℹ️ INFO (0)", "Automatic Lighting"),
    0x0132: ("Automatic headlights deactivated", "ℹ️ INFO (0)", "Automatic Lighting"),
    0x0133: ("Self locking doors activated", "ℹ️ INFO (0)", "Central Locking"),
    0x0134: ("Self locking doors deactivated", "ℹ️ INFO (0)", "Central Locking"),
    0x0137: ("Child safety activated", "ℹ️ INFO (0)", "Child Lock"),
    0x0138: ("Child safety deactivated", "ℹ️ INFO (0)", "Child Lock"),
    0x013D: ("Parking difficult", "ℹ️ INFO (0)", "Parking Space Measurement"),
    0x0198: ("Stop & Start system faulty", "⚠️ SERVICE (1)", "Powertrain / Stop & Start"),
    0x01F6: ("Roof operation impossible: external temperature low", "ℹ️ INFO (0)", "Coupe-Cabriolet Roof"),
    0x01F7: ("Operation of roof impossible : speed too high", "ℹ️ INFO (0)", "Coupe-Cabriolet Roof"),
    0x01F8: ("Operation of roof impossible : boot open", "ℹ️ INFO (0)", "Coupe-Cabriolet Roof"),
    0x01FA: ("Impossible to move roof: screen not deployed", "ℹ️ INFO (0)", "Coupe-Cabriolet Roof"),
    0x01FB: ("Roof operation complete", "ℹ️ INFO (0)", "Coupe-Cabriolet Roof"),
    0x01FC: ("Roof operation incomplete", "ℹ️ INFO (0)", "Coupe-Cabriolet Roof"),
    0x01FD: ("Roof movement impossible: roof locked", "ℹ️ INFO (0)", "Coupe-Cabriolet Roof"),
    0x01FE: ("Folding roof mechanism faulty", "⚠️ SERVICE (1)", "Coupe-Cabriolet Roof"),
    0x01FF: ("Roof operation impossible: rear screen open", "ℹ️ INFO (0)", "Coupe-Cabriolet Roof"),
    0x0200: ("Roof movement impossible: luggage cover not locked", "ℹ️ INFO (0)", "Coupe-Cabriolet Roof"),
    0x0202: ("Right hand stop lamp bulb faulty", "⚠️ SERVICE (1)", "Lighting"),
}


class SerialPacket:
    __slots__ = ('ts', 'source', 'proto', 'raw', 'cmd', 'payload', 'checksum', 'valid_cs')
    def __init__(self, ts: float, source: str, proto: str, raw: bytes,
                 cmd: Optional[int], payload: bytes, checksum: Optional[int], valid_cs: bool):
        self.ts = ts
        self.source = source
        self.proto = proto
        self.raw = raw
        self.cmd = cmd
        self.payload = payload
        self.checksum = checksum
        self.valid_cs = valid_cs


class StreamReassembler:
    """Stitches fragmented UART chunks into valid Hiworld / Raise packets."""
    def __init__(self, source: str):
        self.source = source
        self.buf = bytearray()
        self.ts_buf: List[float] = []

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
            # Hiworld: 5A A5 <len> <cmd> <payload...> <cs>
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

            # Raise: 2E <cmd> <len> <payload...> <cs>
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
                exp_cs = sum(raw_pkt[1:3 + plen]) ^ 0xFF
                valid = (cs == exp_cs)
                del self.buf[:total_len]
                del self.ts_buf[:total_len]
                packets.append(SerialPacket(pkt_ts, self.source, "raise", raw_pkt, cmd, payload, cs, valid))
                continue
            else:
                del self.buf[0]
                del self.ts_buf[0]

        return packets


class AlertDatabase:
    """Manages alert metadata, lookup tables, and PSA-to-Hiworld mappings."""
    def __init__(self, csv_path: Optional[str] = None, c_profile_path: Optional[str] = None):
        self.bit_to_index: List[int] = [0xFFFF] * 168
        self.index_to_can_id: List[int] = [0xFFFF] * 208
        self.alerts_by_can_id: Dict[int, Dict[str, Any]] = {}
        self.alerts_by_index: Dict[int, Dict[str, Any]] = {}

        self._load_from_c_profile(c_profile_path)
        self._load_from_csv(csv_path)

    def _load_from_c_profile(self, c_path: Optional[str]):
        if not c_path or not os.path.exists(c_path):
            # Try default repo path
            c_path = os.path.join(os.path.dirname(__file__), "..", "src", "profiles", "peugeot_407.c")
            if not os.path.exists(c_path):
                c_path = "src/profiles/peugeot_407.c"

        if os.path.exists(c_path):
            try:
                with open(c_path, "r", encoding="utf-8") as f:
                    content = f.read()

                # Parse Alarm_BitToIndex_Tab
                m_bit = re.search(r'Alarm_BitToIndex_Tab\[PSA_JOURNAL_TOTAL_BITS\]\s*=\s*\{([^}]+)\};', content)
                if m_bit:
                    raw_vals = [v.strip() for v in m_bit.group(1).split(',') if v.strip()]
                    parsed = []
                    for v in raw_vals:
                        try:
                            parsed.append(int(v, 0))
                        except ValueError:
                            pass
                    if len(parsed) >= 168:
                        self.bit_to_index = parsed[:168]

                # Parse Alarm_IndexToPointer_Tab
                m_idx = re.search(r'Alarm_IndexToPointer_Tab\[TOTAL_ALARM_ENTRIES\]\s*=\s*\{([^}]+)\};', content)
                if m_idx:
                    entries = re.findall(r'\[(0x[0-9A-Fa-f]+|\d+)\]\s*=\s*(0x[0-9A-Fa-f]+|\d+)', m_idx.group(1))
                    for idx_str, val_str in entries:
                        idx = int(idx_str, 0)
                        val = int(val_str, 0)
                        if 0 <= idx < 208:
                            self.index_to_can_id[idx] = val
            except Exception as e:
                sys.stderr.write(f"Warning: Could not parse C profile ({e})\n")

    def _load_from_csv(self, csv_path: Optional[str]):
        if not csv_path or not os.path.exists(csv_path):
            csv_path = os.path.join(os.path.dirname(__file__), "..", "doc", "RT4_CAN_ALERTS_0x1A1.csv")
            if not os.path.exists(csv_path):
                csv_path = "doc/RT4_CAN_ALERTS_0x1A1.csv"

        if os.path.exists(csv_path):
            try:
                with open(csv_path, "r", encoding="utf-8") as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        try:
                            can_id = int(row["CAN_Alarm_ID_Hex"], 16)
                            a_idx = int(row["Alarm_Index_Hex"], 16)
                            severity_str = row["Severity"]
                            desc = row["English_Alert_String"]
                            category = row["Category_Description"]
                            hw_code_str = row.get("Hiworld_Wire_Code_Hex", "-")
                            hw_code = int(hw_code_str, 16) if hw_code_str and hw_code_str != "-" else can_id

                            # If known automotive alert, prefer the standard automotive description
                            if can_id in KNOWN_AUTOMOTIVE_ALERTS:
                                auto_desc, auto_sev, auto_cat = KNOWN_AUTOMOTIVE_ALERTS[can_id]
                                desc = auto_desc
                                severity_str = auto_sev
                                category = auto_cat

                            entry = {
                                "can_id": can_id,
                                "index": a_idx,
                                "severity_str": severity_str,
                                "description": desc,
                                "category": category,
                                "hiworld_code": hw_code,
                                "active_state": row["Active_State"]
                            }
                            self.alerts_by_can_id[can_id] = entry
                            self.alerts_by_index[a_idx] = entry
                        except Exception:
                            continue
            except Exception as e:
                sys.stderr.write(f"Warning: Could not parse CSV ({e})\n")

    def resolve_bit(self, bit: int) -> Dict[str, Any]:
        """Resolves a 0x120 journal bit index (0..167) into its alert metadata."""
        if bit < 0 or bit >= len(self.bit_to_index):
            return {"can_id": 0xFFFF, "description": "Out of bounds bit", "severity_str": "Unknown", "hiworld_code": 0}

        alarm_idx = self.bit_to_index[bit]
        if alarm_idx == 0xFFFF or alarm_idx >= len(self.index_to_can_id):
            return {"can_id": 0xFFFF, "alarm_idx": alarm_idx, "description": "(Unassigned / Reserved Bit)", "severity_str": "None", "hiworld_code": 0}

        can_id = self.index_to_can_id[alarm_idx]
        if can_id == 0xFFFF:
            return {"can_id": 0xFFFF, "alarm_idx": alarm_idx, "description": "(Unassigned Pointer)", "severity_str": "None", "hiworld_code": 0}

        if can_id in self.alerts_by_can_id:
            info = dict(self.alerts_by_can_id[can_id])
            info["alarm_idx"] = alarm_idx
            return info

        return {
            "can_id": can_id,
            "alarm_idx": alarm_idx,
            "description": f"CAN Alarm 0x{can_id:04X}",
            "severity_str": "Unknown",
            "category": "Unknown",
            "hiworld_code": can_id
        }

    def resolve_can_id(self, can_id: int) -> Dict[str, Any]:
        """Resolves a 15-bit CAN Alarm ID into metadata."""
        if can_id in self.alerts_by_can_id:
            return self.alerts_by_can_id[can_id]
        return {
            "can_id": can_id,
            "description": f"Unknown Alert 0x{can_id:04X}",
            "severity_str": "Unknown",
            "category": "Unknown",
            "hiworld_code": can_id
        }


class AlertLogExtractor:
    """Parses dump log and extracts alert requests, journal blocks, and popups."""
    def __init__(self, db: AlertDatabase):
        self.db = db
        self.app_tx_reassembler = StreamReassembler("APP_UART_TX")
        self.app_rx_reassembler = StreamReassembler("APP_UART_RX")
        self.orig_reassembler = StreamReassembler("ORIG_CANBOX")

        # Journal reassembly state (Mode A: 3 multiplexed blocks of 7 bytes = 21 bytes)
        self.journal_blocks: Dict[int, bytes] = {}
        self.journal_buffer = bytearray(21)
        self.journal_snapshots: List[Dict[str, Any]] = []

        # Extracted events
        self.request_events: List[Dict[str, Any]] = []
        self.popup_events: List[Dict[str, Any]] = []
        self.uart_alert_packets: List[Dict[str, Any]] = []
        self.observed_tx_0x42: set = set()
        self.observed_tx_0xEA: set = set()

        # Session metadata
        self.start_ts: Optional[float] = None
        self.end_ts: Optional[float] = None
        self.line_count = 0
        self.can_frame_count = 0
        self.detected_vehicle = "Unknown"

    def parse_log(self, log_path: str):
        with open(log_path, "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                self.line_count += 1
                line = line.strip()
                if not line:
                    continue

                m_ts = re.match(r"^(\d+\.\d+)\s+\[([^\]]+)\]\s+(.*)$", line)
                if not m_ts:
                    continue

                ts = float(m_ts.group(1))
                tag = m_ts.group(2)
                content = m_ts.group(3)

                if self.start_ts is None:
                    self.start_ts = ts
                self.end_ts = ts

                if tag == "CAN":
                    self.can_frame_count += 1
                    self._handle_can(ts, content)
                elif tag == "APP_UART_RX":
                    pkts = self.app_rx_reassembler.feed(content, ts)
                    for p in pkts:
                        self._handle_uart_rx(p)
                elif tag == "APP_UART_TX":
                    pkts = self.app_tx_reassembler.feed(content, ts)
                    for p in pkts:
                        self._handle_uart_tx(p)
                elif tag == "ORIG_CANBOX":
                    # OEM logger logs "RX: <hex>"
                    hex_match = re.search(r"(?:RX:\s*)?([0-9a-fA-F]+)", content)
                    if hex_match:
                        pkts = self.orig_reassembler.feed(hex_match.group(1), ts)
                        for p in pkts:
                            self._handle_orig_uart(p)

    def _handle_can(self, ts: float, content: str):
        # Format: ID:<HEX> DLC:<NUM> DATA:<HEX>
        m = re.match(r"ID:([0-9A-Fa-f]+)\s+DLC:(\d+)\s+DATA:([0-9A-Fa-f]+)", content)
        if not m:
            return

        can_id = int(m.group(1), 16)
        dlc = int(m.group(2))
        data_hex = m.group(3)
        try:
            data = bytes.fromhex(data_hex)
        except ValueError:
            return

        # 1. CAN ID 0x120: Alert Journal Multiplexing
        if can_id == 0x120 and dlc >= 8:
            b0 = data[0]
            block_id = (b0 >> 6) & 0x03  # 01b = Block 1 (0x7C), 10b = Block 2 (0xBC), 11b = Block 3 (0xFC)
            if 1 <= block_id <= 3:
                self.journal_blocks[block_id] = data[1:8]
                offset = (block_id - 1) * 7
                self.journal_buffer[offset:offset + 7] = data[1:8]

                # Check if we have complete 3 blocks
                if len(self.journal_blocks) == 3:
                    self._record_journal_snapshot(ts, bytes(self.journal_buffer))
                    self.journal_blocks.clear()

        # 2. CAN ID 0x1A1: Real-time Alert Popup
        elif can_id == 0x1A1 and dlc >= 4:
            toggle_bit = (data[0] & 0x80) != 0
            can_alarm_id = ((data[0] & 0x7F) << 8) | data[1]
            display_req = (data[2] & 0x80) != 0
            priority = (data[2] >> 4) & 0x07
            sound_id = data[2] & 0x0F
            door_mask = data[3] if dlc >= 4 else 0
            param = data[4] if dlc >= 5 else 0

            info = self.db.resolve_can_id(can_alarm_id)
            doors_str = self._decode_door_mask(door_mask)

            self.popup_events.append({
                "ts": ts,
                "can_id": can_id,
                "alarm_id": can_alarm_id,
                "raw_hex": data_hex,
                "toggle_bit": toggle_bit,
                "display_req": display_req,
                "priority": priority,
                "severity_str": SEVERITY_NAMES.get(priority, f"Level {priority}"),
                "sound_id": sound_id,
                "sound_str": CHIME_NAMES.get(sound_id, f"Sound {sound_id}"),
                "door_mask": door_mask,
                "doors_str": doors_str,
                "param": param,
                "description": info.get("description", "Unknown Alert"),
                "category": info.get("category", "-")
            })

        # 3. CAN ID 0x221: Cockpit Button / Trip / Diagnostic Check Request
        elif can_id == 0x221 and dlc >= 1:
            b0 = data[0]
            check_key = (b0 & 0x20) != 0  # Bit 5 = Cockpit CHECK button pressed
            trip1_reset = (b0 & 0x80) != 0
            trip2_reset = (b0 & 0x40) != 0

            if check_key or trip1_reset or trip2_reset:
                desc = []
                if check_key:
                    desc.append("Cockpit CHECK button pressed (BSI Diag Alert trigger)")
                if trip1_reset:
                    desc.append("Trip 1 Reset Pulse")
                if trip2_reset:
                    desc.append("Trip 2 Reset Pulse")

                self.request_events.append({
                    "ts": ts,
                    "source": "CAN (0x221)",
                    "type": "COCKPIT_TRIGGER",
                    "raw": data_hex,
                    "detail": ", ".join(desc)
                })

        # 4. CAN ID 0x167: Display Target Alert Command
        elif can_id == 0x167 and dlc >= 6:
            target_id = (data[4] << 8) | data[5]
            if target_id != 0x0000 and target_id != 0xFFFF:
                info = self.db.resolve_can_id(target_id & 0x7FFF)
                self.request_events.append({
                    "ts": ts,
                    "source": "CAN (0x167)",
                    "type": "DISPLAY_TARGET",
                    "raw": data_hex,
                    "detail": f"EMF Display alert target ID 0x{target_id:04X} ({info.get('description', 'Unknown')})"
                })

        # 5. CAN ID 0x39B: Diagnostic Request to BSI
        elif can_id == 0x39B and dlc >= 2:
            self.request_events.append({
                "ts": ts,
                "source": "CAN (0x39B)",
                "type": "DIAG_REQUEST",
                "raw": data_hex,
                "detail": f"Diagnostic command to BSI: Byte0=0x{data[0]:02X}, Byte1=0x{data[1]:02X}"
            })

    def _handle_uart_rx(self, p: SerialPacket):
        # Downlink packet from Head Unit
        if p.cmd == 0x2F:
            self.request_events.append({
                "ts": p.ts,
                "source": "APP_UART_RX",
                "type": "HU_DOWNLINK_QUERY",
                "raw": p.raw.hex(),
                "detail": "Head Unit alert journal query (Cmd 0x2F)"
            })
        elif p.cmd == 0x24:
            car_id = p.payload.hex() if p.payload else "00"
            car_label = "Peugeot 407" if p.payload and p.payload[:1] == b'\x22' else f"Car ID {car_id}"
            self.detected_vehicle = car_label
            self.request_events.append({
                "ts": p.ts,
                "source": "APP_UART_RX",
                "type": "VEHICLE_HANDSHAKE",
                "raw": p.raw.hex(),
                "detail": f"Head Unit vehicle handshake (Cmd 0x24): {car_label}"
            })

    def _handle_uart_tx(self, p: SerialPacket):
        if p.cmd in (0x42, 0xEA):
            self._decode_serial_alert(p, "APP_UART_TX (OpenCanbox)")

    def _handle_orig_uart(self, p: SerialPacket):
        if p.cmd in (0x42, 0xEA):
            self._decode_serial_alert(p, "ORIG_CANBOX (OEM)")

    def _decode_serial_alert(self, p: SerialPacket, source_label: str):
        payload = p.payload
        is_app_tx = "APP_UART_TX" in source_label
        if p.cmd == 0x42:
            # Hiworld Standard Alert
            if len(payload) == 2:
                # Single alert
                code = (payload[0] << 8) | payload[1]
                if is_app_tx:
                    self.observed_tx_0x42.add(code)
                info = self.db.resolve_can_id(code)
                self.uart_alert_packets.append({
                    "ts": p.ts,
                    "source": source_label,
                    "cmd": "0x42 (Single)",
                    "raw": p.raw.hex(),
                    "code": code,
                    "count": 1,
                    "items": [code],
                    "desc": info.get("description", "Unknown"),
                    "valid_cs": p.valid_cs
                })
            elif len(payload) >= 4:
                # Summary packet (Len 24: count in byte 3, up to 10 codes)
                count = payload[3] if len(payload) > 3 else 0
                codes = []
                for i in range(min(count, 10)):
                    offset = 4 + (i * 2)
                    if offset + 1 < len(payload):
                        c = (payload[offset] << 8) | payload[offset + 1]
                        if c != 0:
                            codes.append(c)
                if is_app_tx:
                    self.observed_tx_0x42.update(codes)
                self.uart_alert_packets.append({
                    "ts": p.ts,
                    "source": source_label,
                    "cmd": "0x42 (Summary)",
                    "raw": p.raw.hex(),
                    "count": count,
                    "items": codes,
                    "desc": f"Summary with {count} active alerts",
                    "valid_cs": p.valid_cs
                })
        elif p.cmd == 0xEA:
            # Custom Extended Alert
            if len(payload) == 5:
                can_id = (payload[0] << 8) | payload[1]
                flags = payload[2]
                doors = self._decode_door_mask(payload[3])
                param = payload[4]
                if is_app_tx:
                    self.observed_tx_0xEA.add(can_id)
                info = self.db.resolve_can_id(can_id)
                self.uart_alert_packets.append({
                    "ts": p.ts,
                    "source": source_label,
                    "cmd": "0xEA (Single Ext)",
                    "raw": p.raw.hex(),
                    "code": can_id,
                    "count": 1,
                    "items": [can_id],
                    "desc": f"{info.get('description', 'Unknown')} (Doors: {doors}, Param: 0x{param:02X})",
                    "valid_cs": p.valid_cs
                })
            elif len(payload) >= 4:
                # Extended Summary
                count = payload[3] if len(payload) > 3 else 0
                codes = []
                for i in range(count):
                    offset = 4 + (i * 5)
                    if offset + 1 < len(payload):
                        c = (payload[offset] << 8) | payload[offset + 1]
                        if c != 0:
                            codes.append(c)
                if is_app_tx:
                    self.observed_tx_0xEA.update(codes)
                self.uart_alert_packets.append({
                    "ts": p.ts,
                    "source": source_label,
                    "cmd": "0xEA (Summary Ext)",
                    "raw": p.raw.hex(),
                    "count": count,
                    "items": codes,
                    "desc": f"Extended Summary with {count} active alerts",
                    "valid_cs": p.valid_cs
                })

    def _record_journal_snapshot(self, ts: float, buffer: bytes):
        active_bits = []
        for bit in range(168):
            b_idx = bit >> 3
            mask = 1 << (7 - (bit & 7))
            if buffer[b_idx] & mask:
                active_bits.append(bit)

        # Decode active alerts
        decoded_alerts = []
        seen_can_ids = set()
        for bit in active_bits:
            info = self.db.resolve_bit(bit)
            can_id = info["can_id"]
            if can_id != 0xFFFF and can_id not in seen_can_ids:
                seen_can_ids.add(can_id)
                block_num = (bit // 56) + 1
                byte_in_block = (bit % 56) // 8
                bit_in_byte = 7 - (bit & 7)
                decoded_alerts.append({
                    "bit": bit,
                    "block": block_num,
                    "byte_in_block": byte_in_block,
                    "bit_in_byte": bit_in_byte,
                    "alarm_idx": info.get("alarm_idx", 0xFFFF),
                    "can_id": can_id,
                    "severity": info.get("severity_str", "Unknown"),
                    "hiworld_code": info.get("hiworld_code", can_id),
                    "description": info.get("description", f"Alert 0x{can_id:04X}"),
                    "category": info.get("category", "-")
                })

        # Deduplicate snapshot if identical to previous
        if self.journal_snapshots:
            last = self.journal_snapshots[-1]
            if last["raw_hex"] == buffer.hex() and len(last["alerts"]) == len(decoded_alerts):
                last["count_occurrences"] += 1
                last["last_ts"] = ts
                return

        self.journal_snapshots.append({
            "first_ts": ts,
            "last_ts": ts,
            "count_occurrences": 1,
            "raw_hex": buffer.hex(),
            "active_bits": active_bits,
            "alerts": decoded_alerts
        })

    def _decode_door_mask(self, mask: int) -> str:
        if mask == 0:
            return "None"
        if mask == 0xFF:
            return "All Doors"
        names = [label for bit, label in DOOR_NAMES if (mask & bit)]
        return ", ".join(names) if names else f"0x{mask:02X}"


def format_report_markdown(extractor: AlertLogExtractor, log_path: str, summary_only: bool = False) -> str:
    lines = []
    lines.append(f"# OpenCanbox Alert Log & Diagnostic Extraction Report")
    lines.append(f"**Log File:** `{os.path.basename(log_path)}`  ")
    duration = (extractor.end_ts - extractor.start_ts) if (extractor.start_ts and extractor.end_ts) else 0.0
    lines.append(f"**Duration:** {duration:.2f}s | **Total Lines:** {extractor.line_count:,} | **CAN Frames:** {extractor.can_frame_count:,}  ")
    lines.append(f"**Detected Vehicle Profile:** `{extractor.detected_vehicle}`  \n")

    # 1. Alert Log Requests & Trigger Events
    lines.append("## 1. Alert Log Requests & Cockpit Triggers")
    if extractor.request_events:
        lines.append("| Timestamp | Source | Event Type | Raw Data | Description |")
        lines.append("|---|---|---|---|---|")
        for ev in extractor.request_events:
            lines.append(f"| `{ev['ts']:.3f}` | **{ev['source']}** | `{ev['type']}` | `{ev['raw']}` | {ev['detail']} |")
    else:
        lines.append("*No explicit downlink alert queries (Cmd 0x2F) or cockpit CHECK pulses detected in this log.*")
    lines.append("")

    # 2. Persistent Alert Log / Journal (CAN ID 0x120)
    lines.append("## 2. Persistent Alert Log / Diagnostic Journal (CAN ID `0x120`)")
    if extractor.journal_snapshots:
        lines.append(f"Transmitted over 3 multiplexed blocks (`0x7C`, `0xBC`, `0xFC`) carrying a 21-byte (168-bit) bitfield.\n")
        for idx, snap in enumerate(extractor.journal_snapshots, 1):
            ts_str = f"`{snap['first_ts']:.3f}`" if snap['count_occurrences'] == 1 else f"`{snap['first_ts']:.3f}` – `{snap['last_ts']:.3f}` ({snap['count_occurrences']} cycles)"
            lines.append(f"### Journal Cycle #{idx} at {ts_str}")
            lines.append(f"**Raw 21-Byte Bitfield:** `{snap['raw_hex']}`  ")
            lines.append(f"**Total Active Vehicle Faults:** **{len(snap['alerts'])}**\n")

            if snap['alerts']:
                lines.append("| # | Bit Index | Block & Byte | Alarm Index | CAN Alarm ID | Severity | Sent in 0x42 (Std) | Sent in 0xEA (Ext) | Official Alert String | Category |")
                lines.append("|---|---|---|---|---|---|---|---|---|---|")
                for a_idx, a in enumerate(snap['alerts'], 1):
                    blk_str = f"Blk {a['block']}, Byte {a['byte_in_block']}.{a['bit_in_byte']}"
                    idx_str = f"`0x{a['alarm_idx']:02X}`" if a['alarm_idx'] != 0xFFFF else "-"
                    can_id_str = f"`0x{a['can_id']:04X}` ({a['can_id']})"
                    hw_str = f"`0x{a['hiworld_code']:04X}`" + (" ✅" if a['hiworld_code'] in extractor.observed_tx_0x42 else "")
                    ea_str = f"`0x{a['can_id']:04X}`" + (" ✅" if a['can_id'] in extractor.observed_tx_0xEA else "")
                    lines.append(f"| {a_idx} | Bit **{a['bit']}** | {blk_str} | {idx_str} | **{can_id_str}** | {a['severity']} | {hw_str} | {ea_str} | **{a['description']}** | {a['category']} |")
                lines.append("\n*(Note: ✅ indicates code was observed transmitted in this capture session)*")
            else:
                lines.append("*(Zero active faults recorded in journal bitfield)*")
            lines.append("")
    else:
        lines.append("*No multiplexed 0x120 journal frames received in this log.*")
    lines.append("")

    # 3. Real-Time Rolling Alerts (CAN ID 0x1A1)
    lines.append("## 3. Real-Time Warning Popups (CAN ID `0x1A1`)")
    if extractor.popup_events:
        lines.append(f"Total `0x1A1` Frames Received: **{len(extractor.popup_events)}**\n")
        lines.append("| Timestamp | CAN Alarm ID | State / Req | Severity | Acoustic Chime | Sent in 0x42 (Std) | Sent in 0xEA (Ext) | Doors Involved | Param | Alert Description |")
        lines.append("|---|---|---|---|---|---|---|---|---|---|")
        for p in extractor.popup_events:
            req_str = "Display Modal" if p["display_req"] else "Background Update"
            can_id_str = f"`0x{p['alarm_id']:04X}` ({p['alarm_id']})"
            p_info = extractor.db.resolve_can_id(p["alarm_id"])
            hw_code = p_info.get("hiworld_code", p["alarm_id"])
            hw_str = f"`0x{hw_code:04X}`" + (" ✅" if hw_code in extractor.observed_tx_0x42 else "")
            ea_str = f"`0x{p['alarm_id']:04X}`" + (" ✅" if p['alarm_id'] in extractor.observed_tx_0xEA else "")
            lines.append(f"| `{p['ts']:.3f}` | **{can_id_str}** | {req_str} | {p['severity_str']} | {p['sound_str']} | {hw_str} | {ea_str} | {p['doors_str']} | `0x{p['param']:02X}` | **{p['description']}** |")
    else:
        lines.append("*No real-time 0x1A1 warning frames received in this log.*")
    lines.append("")

    # 4. Uplink Serial Alerts (APP_UART_TX vs ORIG_CANBOX)
    lines.append("## 4. Head Unit Uplink Verification (Serial Output)")
    if extractor.uart_alert_packets:
        lines.append("| Timestamp | Emitter | Packet Type | Raw Frame Sent to HU | Active Alert Codes | Valid Checksum | Detail / Description |")
        lines.append("|---|---|---|---|---|---|---|")
        for u in extractor.uart_alert_packets:
            codes_str = ", ".join([f"0x{c:04X}" for c in u["items"]]) if u["items"] else "None (Cleared)"
            cs_str = "✅ Yes" if u["valid_cs"] else "❌ Error"
            raw_str = f"`{u['raw']}`"
            lines.append(f"| `{u['ts']:.3f}` | **{u['source']}** | `{u['cmd']}` | {raw_str} | `{codes_str}` | {cs_str} | {u['desc']} |")
    else:
        lines.append("*No serial alert packets (Cmd 0x42 / 0xEA) emitted in this log.*")
    lines.append("")

    # 5. Diagnostic Integrity Check & Cross-Validation
    lines.append("## 5. Diagnostic Cross-Check & Consistency Analysis")
    if extractor.journal_snapshots and extractor.uart_alert_packets:
        last_journal = extractor.journal_snapshots[-1]
        journal_can_ids = set([a["can_id"] for a in last_journal["alerts"]])
        journal_hw_codes = set([a["hiworld_code"] for a in last_journal["alerts"]])
        
        # Check summary packets
        summary_pkts = [u for u in extractor.uart_alert_packets if "Summary" in u["cmd"]]
        if summary_pkts:
            last_summary = summary_pkts[-1]
            summary_ids = set(last_summary["items"])
            
            # Check parity using either Hiworld wire code or native CAN ID
            unmatched_in_summary = set()
            for s_id in summary_ids:
                if s_id not in journal_hw_codes and s_id not in journal_can_ids:
                    unmatched_in_summary.add(s_id)
            
            unmatched_in_journal = set()
            for a in last_journal["alerts"]:
                if a["hiworld_code"] not in summary_ids and a["can_id"] not in summary_ids:
                    unmatched_in_journal.add(a["can_id"])

            if not unmatched_in_summary and not unmatched_in_journal:
                lines.append(f"✅ **Perfect Parity ({len(summary_ids)}/{len(last_journal['alerts'])} alerts):** Serial summary frame active list exactly matches all persistent faults in CAN `0x120` journal.")
            else:
                if unmatched_in_journal:
                    missing_str = ", ".join([f"0x{m:04X}" for m in unmatched_in_journal])
                    lines.append(f"⚠️ **Missing Alerts in Summary:** CAN journal has alerts not represented in serial summary: `{missing_str}`")
                if unmatched_in_summary:
                    phantom_str = ", ".join([f"0x{p:04X}" for p in unmatched_in_summary])
                    lines.append(f"❌ **Phantom Alerts in Summary:** Serial summary contains alerts NOT active in CAN journal: `{phantom_str}`")
        else:
            lines.append("ℹ️ *No serial summary packet was dispatched yet (single real-time alerts or idle).*")
    elif extractor.journal_snapshots:
        lines.append(f"ℹ️ *CAN Journal has {len(extractor.journal_snapshots[-1]['alerts'])} active faults; no serial summary packet was sent in this log segment.*")
    else:
        lines.append("ℹ️ *Insufficient telemetry to perform cross-check.*")

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description="Extract and decode Alert Log request data and active vehicle alerts from OpenCanbox dump logs."
    )
    parser.add_argument("log_file", help="Path to E2E dump log file (dump_*.log)")
    parser.add_argument("--csv", default=None, help="Path to RT4_CAN_ALERTS_0x1A1.csv")
    parser.add_argument("--c-profile", default=None, help="Path to peugeot_407.c")
    parser.add_argument("--summary-only", action="store_true", help="Print only summary tables")

    args = parser.parse_args()

    if not os.path.exists(args.log_file):
        sys.stderr.write(f"Error: Log file not found: {args.log_file}\n")
        sys.exit(1)

    db = AlertDatabase(args.csv, args.c_profile)
    extractor = AlertLogExtractor(db)
    extractor.parse_log(args.log_file)

    report = format_report_markdown(extractor, args.log_file, args.summary_only)
    print(report)


if __name__ == "__main__":
    main()
