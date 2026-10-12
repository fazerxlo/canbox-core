#!/usr/bin/env python3
"""
NUC131 Direct TCL RPC Flash Programmer
Erase and programs flash page by page using native FMC ISP commands.
"""
import socket
import sys
import time

PAGE_SIZE = 512

class OpenOcdClient:
    def __init__(self, host="127.0.0.1", port=6666):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        
    def cmd(self, command: str) -> str:
        cmd_str = command + "\x1a"
        self.sock.sendall(cmd_str.encode("utf-8"))
        res = bytearray()
        while True:
            chunk = self.sock.recv(16384)
            if not chunk:
                break
            res += chunk
            if b"\x1a" in chunk:
                break
        return res.decode("utf-8", errors="ignore").rstrip("\x1a")
        
    def close(self):
        self.sock.close()

def flash_binary(client: OpenOcdClient, bin_path: str, base_addr: int):
    with open(bin_path, "rb") as f:
        data = bytearray(f.read())

    print(f"[*] Flashing {bin_path} ({len(data)} bytes) to 0x{base_addr:08X}...")

    # Pad data to 512-byte page boundary
    pad_rem = len(data) % PAGE_SIZE
    if pad_rem != 0:
        data += b"\xFF" * (PAGE_SIZE - pad_rem)

    total_pages = len(data) // PAGE_SIZE

    # Unlock FMC ISP
    client.cmd("reset halt")
    client.cmd("mww 0x50000100 0x59")
    client.cmd("mww 0x50000100 0x16")
    client.cmd("mww 0x50000100 0x88")
    client.cmd("mww 0x50000204 0x34") # Enable ISP clock
    client.cmd("mww 0x5000C000 0x79") # ISPEN | APUEN | CFGUEN | LDUEN

    start_time = time.time()
    for p in range(total_pages):
        page_addr = base_addr + p * PAGE_SIZE
        page_data = data[p * PAGE_SIZE : (p + 1) * PAGE_SIZE]

        # 1. Erase page
        client.cmd(f"mww 0x5000C004 0x{page_addr:08X}; mww 0x5000C00C 0x22; mww 0x5000C010 0x01")
        time.sleep(0.010)

        # 2. Program words in page
        cmds = []
        for w in range(0, PAGE_SIZE, 4):
            val = int.from_bytes(page_data[w : w + 4], "little")
            if val != 0xFFFFFFFF: # Skip blank words
                word_addr = page_addr + w
                cmds.append(f"mww 0x5000C004 0x{word_addr:08X}; mww 0x5000C008 0x{val:08X}; mww 0x5000C00C 0x21; mww 0x5000C010 0x01")

        if cmds:
            # Send batch of words in chunks of 4 with a short delay for FMC write cycle
            chunk_size = 4
            for i in range(0, len(cmds), chunk_size):
                client.cmd("; ".join(cmds[i : i + chunk_size]))
                time.sleep(0.003)

        print(f"    - Programmed page {p + 1}/{total_pages} ({(p + 1) * 100 // total_pages}%)...", end="\r")

    print()
    elapsed = time.time() - start_time
    print(f"[*] Flashing finished in {elapsed:.2f}s ({len(data)/elapsed/1024:.2f} KB/s)")

    # Verify
    print("[*] Verifying flash memory...")
    mismatches = 0
    words_total = len(data) // 4
    # Read in chunks of 32 words
    for w in range(0, words_total, 32):
        chunk_words = min(32, words_total - w)
        addr = base_addr + w * 4
        out = client.cmd(f"mdw 0x{addr:08X} {chunk_words}")
        # Parse hex words from lines
        read_vals = []
        for line in out.strip().splitlines():
            if ":" in line:
                parts = line.split(":")[1].strip().split()
                for p in parts:
                    try:
                        read_vals.append(int(p, 16))
                    except ValueError:
                        pass
        for i in range(min(len(read_vals), chunk_words)):
            expected = int.from_bytes(data[(w + i)*4 : (w + i + 1)*4], "little")
            if read_vals[i] != expected:
                print(f"[!] Mismatch at 0x{(addr + i*4):08X}: exp 0x{expected:08X}, got 0x{read_vals[i]:08X}")
                mismatches += 1
                if mismatches > 10:
                    break
        if mismatches > 10:
            break

    if mismatches == 0:
        print(f"[+] Flash at 0x{base_addr:08X} VERIFIED SUCCESSFULLY!")
    else:
        print(f"[!] Verification FAILED with {mismatches} errors!")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: nuc131_flasher.py <binary_file> <base_address_hex>")
        sys.exit(1)
        
    bin_file = sys.argv[1]
    base_address = int(sys.argv[2], 16)
    
    cli = OpenOcdClient()
    try:
        flash_binary(cli, bin_file, base_address)
    finally:
        cli.close()
