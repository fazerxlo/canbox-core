#!/usr/bin/env python3
"""
Hiworld CAN Box .bin to .iap Container Packer
Produces production-ready can_app.iap files for Head Unit updates.
"""
import argparse
import sys

REC_SIZE = 136
PAYLOAD_SIZE = 128
MAGIC_HEADER = bytes.fromhex("0109070800020107913247a4")
DEFAULT_KEY = bytes([0x19, 0x84, 0x07, 0x42])

def calc_record_csum(rec_bytes: bytes) -> int:
    """Computes the 32-bit additive sum across 33 uint32 words."""
    return sum(
        int.from_bytes(rec_bytes[j * 4 : j * 4 + 4], "little")
        for j in range(33)
    ) & 0xFFFFFFFF

def obfuscate_chunk(chunk: bytes, key: bytes) -> bytes:
    """Hiworld additive byte obfuscation: cipher[i] = (plain[i] + i + key[i%4]) & 0xFF"""
    return bytes((chunk[i] + i + key[i & 3]) & 0xFF for i in range(len(chunk)))

def pack_firmware(bin_path: str, out_iap_path: str, template_iap_path: str = None, key: bytes = DEFAULT_KEY):
    with open(bin_path, "rb") as f:
        payload = f.read()

    # Generate or extract Record 0
    if template_iap_path:
        with open(template_iap_path, "rb") as f:
            template = f.read()
        record_0 = bytearray(template[:REC_SIZE])
        record_0[132:136] = key
    else:
        record_0 = bytearray(REC_SIZE)
        record_0[:12] = MAGIC_HEADER
        record_0[12:128] = b"\x00" * 116  # Descriptor padding
        record_0[128:132] = b"P407"       # Peugeot 407 tag
        record_0[132:136] = key

    # Pad payload to 128-byte boundary
    pad_rem = len(payload) % PAYLOAD_SIZE
    if pad_rem != 0:
        payload += b"\xFF" * (PAYLOAD_SIZE - pad_rem)

    total_records = len(payload) // PAYLOAD_SIZE
    out_iap = bytearray(record_0)

    for k in range(1, total_records + 1):
        chunk = payload[(k - 1) * PAYLOAD_SIZE : k * PAYLOAD_SIZE]
        rec = bytearray(obfuscate_chunk(chunk, key))
        rec += k.to_bytes(2, "little")               # Sequence number (1..N)
        rec.append((k * 100) // total_records)       # Progress percentage (0..100)
        rec.append(1 if k == total_records else 0)   # Last block flag (1 = EOF)
        csum = calc_record_csum(rec)
        rec += csum.to_bytes(4, "little")            # 32-bit checksum
        out_iap += rec

    with open(out_iap_path, "wb") as f:
        f.write(out_iap)

    print(f"[+] Successfully generated '{out_iap_path}':")
    print(f"    - Input Binary : {len(payload)} bytes")
    print(f"    - Records      : {total_records + 1} (1 header + {total_records} data)")
    print(f"    - Output Size  : {len(out_iap)} bytes")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Pack NUC131 .bin into Hiworld can_app.iap")
    parser.add_argument("bin_file", help="Input compiled application .bin")
    parser.add_argument("-o", "--out", default="can_app.iap", help="Output .iap filename (default: can_app.iap)")
    parser.add_argument("-t", "--template", default=None, help="Optional existing Hiworld .iap template for Record 0")
    args = parser.parse_args()

    pack_firmware(args.bin_file, args.out, args.template)

