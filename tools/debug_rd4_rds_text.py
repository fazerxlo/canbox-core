#!/usr/bin/env python3
"""
Peugeot 407 RD4 RDS Radio Text (0x0A4) & MDF (EMF-C) Emulator / Debugger
========================================================================
Emulates the EMF-C Multifunction Display on SocketCAN (can0) and diagnoses
ISO-TP 0x0A4 -> 0x0A3 Flow Control & Consecutive Frame reassembly from RD4.

Usage examples:
  1. Passive listen + answer 0x0A4 with FC [30 00 00] (like trigger_rd4_station_list.py):
     python3 debug_rd4_rds_text.py

  2. Full MDF emulation (0x167, 0x525, 0x15B) + poke RD4 to force 0x0A4 re-transmission:
     python3 debug_rd4_rds_text.py --emulate-mdf --poke

  3. Test C firmware's exact FC parameters (STmin=0x0A, delay=2ms) to reproduce the bug:
     python3 debug_rd4_rds_text.py --stmin 0x0A --fc-delay-ms 2
"""

import argparse
import select
import socket
import struct
import sys
import time

CAN_FRAME_FMT = "=IB3x8s"

# PSA Comfort CAN IDs
CAN_ID_TEXTE_RADIO_0A4  = 0x0A4  # RD4 -> MDF: ISO-TP RDS Radio Text
CAN_ID_EMF_FC_09F       = 0x09F  # MDF -> RD4: ISO-TP Flow Control for 0x0A4
CAN_ID_STATION_LIST_125 = 0x125  # RD4 -> MDF: ISO-TP Station List
CAN_ID_EMF_FC_11F       = 0x11F  # MDF -> RD4: ISO-TP Flow Control for 0x125
CAN_ID_INFO_CONFIG_15B  = 0x15B  # MDF -> BSI/RD4: Config Ack
CAN_ID_ETAT_AUTORADIO   = 0x165  # RD4 Status
CAN_ID_ETAT_EMF_167     = 0x167  # MDF -> RD4/BSI: Screen Mode & RDS Text Request
CAN_ID_SWC_STALK_21F    = 0x21F  # COM2000 Stalk
CAN_ID_ETAT_TUNER_225   = 0x225  # RD4 Frequency & RDS flags
CAN_ID_NOM_STATION_2A5  = 0x2A5  # RD4 8-char PS Name
CAN_ID_MDF_DIAG_525     = 0x525  # MDF Presence / Keepalive


def pack_can(can_id: int, data: bytes) -> bytes:
    return struct.pack(CAN_FRAME_FMT, can_id & 0x7FF, len(data), data.ljust(8, b"\x00"))


def parse_can(raw: bytes):
    cid_raw, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
    return (cid_raw & 0x7FF), min(dlc, 8), data[:dlc]


def clean_ascii(raw: bytes) -> str:
    return "".join(chr(b) if 0x20 <= b <= 0x7E else " " for b in raw)


def decode_0a4_payload(buf: bytes):
    """Decodes reassembled 0x0A4 ISO-TP payload and prints header + ASCII text."""
    print("\n" + "=" * 65)
    print(f" [✓] REASSEMBLED CAN 0x0A4 RDS RADIO TEXT ({len(buf)} bytes)")
    print("=" * 65)
    print(f"  Raw Hex   : {buf.hex(' ')}")
    if len(buf) > 4:
        hdr = buf[:4]
        body = buf[4:]
        # Cut at first null byte (like C firmware line 461)
        null_idx = body.find(b"\x00")
        c_cropped = body[:null_idx] if null_idx != -1 else body
        print(f"  4B Header : {hdr.hex(' ')}  (c_filter_match={hdr[1:4] == b'\\x00\\x00\\x00'})")
        print(f"  Full Text : \"{clean_ascii(body).strip()}\"")
        print(f"  C-Cropped : \"{clean_ascii(c_cropped).strip()}\" (len={len(c_cropped)})")
    else:
        print(f"  Short Data: \"{clean_ascii(buf)}\"")
    print("=" * 65 + "\n")


def run_debugger(args):
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((args.interface,))

    fc_payload = bytes([0x30, args.bs & 0xFF, args.stmin & 0xFF])
    fc_can_id = args.fc_id
    print(f"[*] Bound to {args.interface}")
    print(f"[*] Flow Control target CAN ID : 0x{fc_can_id:03X}")
    print(f"[*] Flow Control payload       : [{fc_payload.hex(' ')}] (DLC={'8 (padded)' if args.pad_8 else len(fc_payload)}, delay={args.fc_delay_ms} ms)")
    print(f"[*] MDF Periodic Emulation     : {'ENABLED' if args.emulate_mdf else 'DISABLED (Passive/FC-only)'}")
    if args.no_fc:
        print("    -> Passive mode: NO Flow Control will be transmitted. Listening for real MDF response.")

    isotp_0a4_buf = bytearray()
    isotp_0a4_total = 0
    isotp_0a4_next_sn = 1
    ff_timestamp = 0.0

    seen_mdf_frames = {}

    t_start = time.time()
    while time.time() - t_start < args.timeout:
        now = time.time()

        # Check for ISO-TP timeout if First Frame was seen but CFs never arrived
        if isotp_0a4_buf and (now - ff_timestamp > 1.0):
            print(f"[!] ISO-TP 0x0A4 TIMEOUT! Received {len(isotp_0a4_buf)}/{isotp_0a4_total} bytes before stall.")
            print(f"    Partial buffer: {isotp_0a4_buf.hex(' ')} (\"{clean_ascii(isotp_0a4_buf)}\")")
            isotp_0a4_buf.clear()

        # 4. Receive CAN frames
        r, _, _ = select.select([sock], [], [], 0.02)
        if not r:
            continue

        pkt = sock.recv(16)
        if len(pkt) != 16:
            continue
        rx_time = time.time()
        cid, dlc, data = parse_can(pkt)

        # Track MDF periodic broadcast activity if physical MDF is on bus
        if cid in (CAN_ID_ETAT_EMF_167, CAN_ID_INFO_CONFIG_15B, CAN_ID_MDF_DIAG_525, 0x0DF, 0x1ED, 0x3F6):
            if seen_mdf_frames.get(cid) != data:
                seen_mdf_frames[cid] = data
                print(f"[{rx_time:.3f}] [MDF BROADCAST] ID:0x{cid:03X} DLC:{dlc} DATA:{data.hex(' ')}")

        # If waiting for ISO-TP response within 150 ms of FF, log any candidate frame on the bus!
        if ff_timestamp > 0 and (rx_time - ff_timestamp) <= 0.150 and cid != CAN_ID_TEXTE_RADIO_0A4:
            is_fc = (dlc >= 3 and (data[0] & 0xF0) == 0x30)
            prefix = ">>> [POSSIBLE FLOW CONTROL]" if is_fc else "    [CAN BUS POST-FF]"
            print(f"[{rx_time:.3f}] {prefix} (+{(rx_time - ff_timestamp)*1000.0:5.1f} ms) ID:0x{cid:03X} DLC:{dlc} DATA:{data.hex(' ')}")

        # Monitor Tuner status (0x225) & PS Name (0x2A5)
        if cid == CAN_ID_ETAT_TUNER_225 and dlc >= 5:
            raw_freq = (data[3] << 8) | data[4]
            freq_mhz = ((raw_freq / 2) + 500) / 10.0
            rds_on = bool(data[0] & 0x20)
            txt_bit = bool(data[0] & 0x02)
            if args.verbose:
                print(f"[{rx_time:.3f}] [0x225] Freq={freq_mhz:.1f} MHz | flags=0x{data[0]:02X} (RDS={rds_on}, bit1={txt_bit})")

        elif cid == CAN_ID_NOM_STATION_2A5 and dlc >= 8:
            if args.verbose:
                print(f"[{rx_time:.3f}] [0x2A5] PS Name: \"{clean_ascii(data)}\"")

        elif cid in (CAN_ID_EMF_FC_09F, 0x0A3):
            # Observe if another node (e.g., canbox-core) is also sending Flow Control!
            print(f"[{rx_time:.3f}] [BUS FC 0x{cid:03X}] Saw Flow Control on bus: {data.hex(' ')} (DLC={dlc})")

        elif cid == CAN_ID_TEXTE_RADIO_0A4 and dlc >= 2:
            pci = data[0] & 0xF0

            if pci == 0x00:  # Single Frame
                sf_len = data[0] & 0x0F
                payload = data[1:1 + sf_len]
                print(f"[{rx_time:.3f}] [0x0A4 SF] len={sf_len} data={payload.hex(' ')}")
                decode_0a4_payload(payload)

            elif pci == 0x10:  # First Frame
                isotp_0a4_total = ((data[0] & 0x0F) << 8) | data[1]
                isotp_0a4_buf = bytearray(data[2:dlc])
                isotp_0a4_next_sn = 1
                ff_timestamp = rx_time

                print(f"[{rx_time:.3f}] [0x0A4 FF] TotalLen={isotp_0a4_total} | FirstChunk={data[2:dlc].hex(' ')} (\"{clean_ascii(data[2:dlc])}\")")

                if not args.no_fc:
                    if args.fc_delay_ms > 0:
                        time.sleep(args.fc_delay_ms / 1000.0)
                    tx_fc = fc_payload.ljust(8, b"\x00") if args.pad_8 else fc_payload
                    sock.send(pack_can(fc_can_id, tx_fc))
                    print(f"[{time.time():.3f}] [TX FC 0x{fc_can_id:03X}] Sent FC: {tx_fc.hex(' ')} (DLC={len(tx_fc)})")

            elif pci == 0x20:  # Consecutive Frame
                sn = data[0] & 0x0F
                chunk = data[1:dlc]
                print(f"[{rx_time:.3f}] [0x0A4 CF] SN=0x{sn:X} (expected=0x{isotp_0a4_next_sn:X}) | {chunk.hex(' ')} (\"{clean_ascii(chunk)}\")")

                if isotp_0a4_buf:
                    isotp_0a4_buf.extend(chunk)
                    isotp_0a4_next_sn = (isotp_0a4_next_sn + 1) & 0x0F
                    if len(isotp_0a4_buf) >= isotp_0a4_total:
                        final_payload = bytes(isotp_0a4_buf[:isotp_0a4_total])
                        isotp_0a4_buf.clear()
                        decode_0a4_payload(final_payload)
                        if args.once:
                            return True

    return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Debug & Emulate MDF for RD4 RDS Text (CAN 0x0A4)")
    parser.add_argument("-i", "--interface", default="can0", help="SocketCAN interface (default: can0)")
    parser.add_argument("-t", "--timeout", type=float, default=30.0, help="Listen timeout in seconds (default: 30)")
    parser.add_argument("--fc-id", type=lambda x: int(x, 0), default=0x09F, help="Flow Control CAN ID (default: 0x09F)")
    parser.add_argument("--stmin", type=lambda x: int(x, 0), default=0x0A, help="ISO-TP FC STmin byte (default: 0x0A)")
    parser.add_argument("--bs", type=lambda x: int(x, 0), default=0x00, help="ISO-TP FC BlockSize byte (default: 0x00)")
    parser.add_argument("--fc-delay-ms", type=float, default=2.0, help="Delay in ms before sending FC (default: 2.0)")
    parser.add_argument("--pad-8", action="store_true", help="Pad Flow Control frame to 8 bytes (DLC=8)")
    parser.add_argument("--no-fc", action="store_true", help="Do not send Flow Control (use when physical MDF is connected)")
    parser.add_argument("--once", action="store_true", help="Exit immediately after decoding one full 0x0A4 message")
    parser.add_argument("-v", "--verbose", action="store_true", help="Print 0x225 frequency and 0x2A5 PS name updates")
    args = parser.parse_args()

    ok = run_debugger(args)
    sys.exit(0 if ok else 1)
