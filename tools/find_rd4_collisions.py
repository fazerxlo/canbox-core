#!/usr/bin/env python3
"""
PSA RD4 / CDC Emulator CAN Bus Collision & Reset Analyzer
Usage: python find_rd4_collisions.py CD_can_log.csv
"""

import sys
import pandas as pd

# PSA Info-Div CAN IDs of interest
RD4_IDS = {'165', '1A5', '1E5', '225', '265', '2A5', '325', '365', '3A5', '3E5', '525', '131', '5E0'}
CDC_IDS = {'162', '1A2', '1E2', '122', '522'}
SWC_IDS = {'21F'}


def parse_hex(val):
    try:
        return int(str(val).strip(), 16)
    except Exception:
        return 0


def analyze_can_log(csv_path, contention_threshold_us=100):
    # index_col=False prevents pandas from shifting columns if rows have trailing commas
    df = pd.read_csv(csv_path, index_col=False)
    df.columns = [c.strip() for c in df.columns]

    # Normalize 3-hex-digit CAN ID
    df['CAN_ID'] = df['ID'].astype(str).str.strip().str.upper().str[-3:]

    # Build readable hex payload string
    def format_payload(row):
        length = int(row['LEN']) if pd.notna(row['LEN']) else 0
        return " ".join(
            str(row[f'D{i}']).strip().upper().zfill(2)
            for i in range(1, min(length, 8) + 1)
        )

    df['Payload'] = df.apply(format_payload, axis=1)

    print(f"======================================================================")
    print(f" Analyzing PSA CAN Log: {csv_path} ({len(df)} total frames)")
    print(f"======================================================================\n")

    state_165 = None
    state_131 = None
    state_162 = None

    timeline_events = []
    state_collisions = []
    id_bus_collisions = []

    # Track last timestamp per CAN ID to detect two nodes transmitting the same ID
    last_ts_by_id = {}

    for idx, row in df.iterrows():
        ts = int(row['Time Stamp'])
        cid = row['CAN_ID']
        payload = row['Payload']
        d1 = parse_hex(row.get('D1', 0))
        d2 = parse_hex(row.get('D2', 0))

        # Check for same-ID collision (two nodes transmitting on the same periodic ID < 15ms apart)
        if cid in (RD4_IDS | CDC_IDS | SWC_IDS):
            if cid in last_ts_by_id:
                dt = ts - last_ts_by_id[cid][1]
                # Ignore timestamp wrap-around at start of log
                if 0 <= dt < 15000:
                    id_bus_collisions.append((
                        idx, ts, cid, dt, last_ts_by_id[cid][2], payload
                    ))
            last_ts_by_id[cid] = (idx, ts, payload)

        # 1. Monitor RD4 Status (0x165)
        if cid == '165':
            if payload != state_165:
                desc = "RD4 Status Change"
                if d1 == 0xC8 and d2 == 0xC0:
                    desc = "RD4 Active (Unmuted, Source=C0)"
                elif d1 == 0x48 and d2 == 0xC0:
                    desc = "RD4 Muted / Source Switch Initiated (48 C0)"
                elif d1 == 0x48 and d2 == 0xD0:
                    desc = "RD4 Source Transition / Waiting Handshake (48 D0)"
                elif d1 == 0x08 and d2 == 0x90:
                    desc = "CRASH: RD4 Audio Controller Fault (08 90)"
                elif d1 == 0x00 and d2 == 0x00:
                    desc = "REBOOT: RD4 Watchdog Resetting (00 00 00 00)"
                timeline_events.append((idx, ts, cid, payload, desc))
                state_165 = payload

        # 2. Monitor RD4 Internal CD / Deck Command (0x131)
        elif cid == '131':
            if payload != state_131:
                if d1 == 0x01:
                    desc = "RD4 Internal CD / Deck -> ACTIVE (01)"
                elif d1 == 0x81:
                    desc = "RD4 Internal CD / Deck -> STANDBY (81)"
                else:
                    desc = f"RD4 0x131 Changed ({d1:02X})"
                timeline_events.append((idx, ts, cid, payload, desc))
                state_131 = payload

        # 3. Monitor CDC Emulator (0x162) & Check for State Collisions
        elif cid == '162':
            if payload != state_162:
                if d1 == 0x20:
                    desc = "CDC Emulator -> PLAYING (20)"
                elif d1 == 0xA0:
                    desc = "CDC Emulator -> STANDBY (A0)"
                else:
                    desc = f"CDC Emulator Status ({d1:02X})"
                timeline_events.append((idx, ts, cid, payload, desc))
                state_162 = payload

            # Detect State Collision:
            # Emulator claims CDC is Playing (0x20) while RD4 is muted/switching to Internal CD
            if d1 == 0x20 and state_165 is not None:
                if state_165.startswith(('48 C0', '48 D0', '08 90', '00 00')):
                    state_collisions.append({
                        'row': idx,
                        'ts': ts,
                        'cdc_payload': payload,
                        'rd4_165': state_165,
                        'rd4_131': state_131,
                        'reason': (
                            f"0x162 sends 0x20 (CDC Playing) while RD4 is exiting/switching "
                            f"(0x165={state_165}, 0x131={state_131})"
                        )
                    })

        # 4. Monitor RD4 Hardware Reset Diagnostic Frame (0x5E0)
        elif cid == '5E0':
            timeline_events.append((
                idx, ts, cid, payload,
                "RESET: RD4 Hardware Reset Diagnostic Frame (0x5E0)"
            ))

    # 5. Check for Close Bus Arbitration Contention (< contention_threshold_us)
    df['prev_TS'] = df['Time Stamp'].shift(1)
    df['prev_ID'] = df['CAN_ID'].shift(1)
    df['next_TS'] = df['Time Stamp'].shift(-1)
    df['next_ID'] = df['CAN_ID'].shift(-1)

    arbitration_contentions = []
    for idx, row in df[df['CAN_ID'].isin(CDC_IDS)].iterrows():
        ts = row['Time Stamp']
        cid = row['CAN_ID']
        if pd.notna(row['prev_TS']) and row['prev_ID'] in RD4_IDS:
            dt = abs(ts - row['prev_TS'])
            if dt <= contention_threshold_us:
                arbitration_contentions.append((idx, int(ts), cid, row['prev_ID'], int(dt), "after"))
        if pd.notna(row['next_TS']) and row['next_ID'] in RD4_IDS:
            dt = abs(row['next_TS'] - ts)
            if dt <= contention_threshold_us:
                arbitration_contentions.append((idx, int(ts), cid, row['next_ID'], int(dt), "before"))

    # --- PRINT REPORT ---
    print("1. STATE TRANSITION & CRASH TIMELINE")
    print("-" * 95)
    for idx, ts, cid, payload, desc in timeline_events:
        print(f"Row {idx:4d} | TS: {ts:8d} us | ID: 0x{cid} | {payload:<22} | {desc}")

    print(f"\n2. DETECTED CDC vs. RD4 STATE COLLISIONS ({len(state_collisions)} colliding frames)")
    print("-" * 95)
    if state_collisions:
        for c in state_collisions:
            print(f"Row {c['row']:4d} | TS: {c['ts']:8d} us | {c['reason']}")
    else:
        print("None detected.")

    print(f"\n3. SAME-ID / DOUBLE TRANSMISSION COLLISIONS (<15 ms apart)")
    print("-" * 95)
    if id_bus_collisions:
        for idx, ts, cid, dt, prev_p, curr_p in id_bus_collisions:
            print(f"Row {idx:4d} | TS: {ts:8d} us | ID: 0x{cid} | dt: {dt:5d} us | {prev_p} -> {curr_p}")
    else:
        print("None detected.")

    print(f"\n4. CLOSE BUS ARBITRATION CONTENTION (CDC within <= {contention_threshold_us} us of RD4)")
    print("-" * 95)
    if arbitration_contentions:
        for idx, ts, cdc_id, rd4_id, dt, pos in arbitration_contentions:
            print(f"Row {idx:4d} | TS: {ts:8d} us | CDC 0x{cdc_id} sent {dt} us {pos} RD4 0x{rd4_id}")
    else:
        print("None detected.")


if __name__ == '__main__':
    log_file = sys.argv[1] if len(sys.argv) > 1 else 'CD_can_log.csv'
    analyze_can_log(log_file)
