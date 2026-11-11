#!/usr/bin/env python3
"""
NUC131 Direct TCL RPC Flash Programmer
Erase and programs flash page by page using native FMC ISP commands.
"""
import socket
import subprocess
import sys
import time

PAGE_SIZE = 512


def ensure_openocd_running():
    res = subprocess.run(["pgrep", "-x", "openocd"], capture_output=True)
    if res.returncode != 0:
        subprocess.Popen(
            ["openocd", "-f", "openocd_nuc131.cfg", "-d0"],
            stdout=open("/tmp/openocd.log", "w"),
            stderr=subprocess.STDOUT,
        )
        time.sleep(0.5)

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

def read_words(client: OpenOcdClient, addr: int, count: int) -> list:
    out = client.cmd(f"mdw 0x{addr:08X} {count}")
    vals = []
    for line in out.strip().splitlines():
        if ":" in line:
            parts = line.split(":")[1].strip().split()
            for p in parts:
                try:
                    vals.append(int(p, 16))
                except ValueError:
                    pass
    return vals


def flash_binary(client: OpenOcdClient, bin_path: str, base_addr: int, force_full: bool = False):
    with open(bin_path, "rb") as f:
        data = bytearray(f.read())

    print(f"[*] Flashing {bin_path} ({len(data)} bytes) to 0x{base_addr:08X}...")

    # Pad data to 512-byte page boundary
    pad_rem = len(data) % PAGE_SIZE
    if pad_rem != 0:
        data += b"\xFF" * (PAGE_SIZE - pad_rem)

    total_pages = len(data) // PAGE_SIZE
    words_per_page = PAGE_SIZE // 4

    # 1. Reset halt target
    client.cmd("reset halt")

    # 2. Immediately enforce GPIO isolation during SWD halt:
    # - Clear GPD_MFP to disconnect CAN0 peripheral
    # - PC_DOUT = 0x00CF, PC_PMD = 0x0000F045 (PC3=1 Standby transceiver, silence buzzer)
    # - PD_DOUT = 0x00C0, PD_PMD = 0x00007000 (PD7=1 CAN_TX Recessive Push-Pull, PD6=Input)
    # - PB_DOUT = 0x0000113F, PB_PMD = 0x43030FF4 (Silence buzzer lines PB3/PB12)
    # - PA_DOUT = 0x00004DFF, PA_PMD = 0x75F5FF0F (ACC ON, other outputs off)
    client.cmd("mww 0x5000003C 0x00000000")
    client.cmd("mww 0x50004088 0x000000CF; mww 0x50004080 0x0000F045")
    client.cmd("mww 0x500040C8 0x000000C0; mww 0x500040C0 0x00007000")
    client.cmd("mww 0x50004048 0x0000113F; mww 0x50004040 0x43030FF4")
    client.cmd("mww 0x50004008 0x00004DFF; mww 0x50004000 0x75F5FF0F")

    # 3. Unlock FMC ISP
    client.cmd("mww 0x50000100 0x59")
    client.cmd("mww 0x50000100 0x16")
    client.cmd("mww 0x50000100 0x88")
    client.cmd("mww 0x50000204 0x34") # Enable ISP clock
    client.cmd("mww 0x5000C000 0x79") # ISPEN | APUEN | CFGUEN | LDUEN

    start_time = time.time()
    dirty_pages = []
    skipped_pages = 0

    for p in range(total_pages):
        page_addr = base_addr + p * PAGE_SIZE
        page_data = data[p * PAGE_SIZE : (p + 1) * PAGE_SIZE]
        expected_words = [
            int.from_bytes(page_data[w : w + 4], "little")
            for w in range(0, PAGE_SIZE, 4)
        ]

        if not force_full:
            current_words = read_words(client, page_addr, words_per_page)
            if len(current_words) == words_per_page and current_words == expected_words:
                skipped_pages += 1
                continue

        dirty_pages.append((p, page_addr, expected_words))

        # 1. Erase page
        client.cmd(f"mww 0x5000C004 0x{page_addr:08X}; mww 0x5000C00C 0x22; mww 0x5000C010 0x01")
        # Poll ISPTRG until page erase completes
        for _ in range(50):
            res = client.cmd("mdw 0x5000C010 1")
            if "00000000" in res:
                break
            time.sleep(0.002)

        # 2. Program words in page
        cmds = []
        for idx, val in enumerate(expected_words):
            if val != 0xFFFFFFFF: # Skip blank words
                word_addr = page_addr + idx * 4
                cmds.append(f"mww 0x5000C004 0x{word_addr:08X}; mww 0x5000C008 0x{val:08X}; mww 0x5000C00C 0x21; mww 0x5000C010 0x01")

        if cmds:
            # Send batch of words in chunks of 4 and poll completion
            chunk_size = 4
            for i in range(0, len(cmds), chunk_size):
                client.cmd("; ".join(cmds[i : i + chunk_size]))
                for _ in range(50):
                    res = client.cmd("mdw 0x5000C010 1")
                    if "00000000" in res:
                        break
                    time.sleep(0.001)

        print(f"    - Programmed page {p + 1}/{total_pages} ({len(dirty_pages)} modified, {skipped_pages} unchanged)...", end="\r")

    if dirty_pages:
        print()
    elapsed = time.time() - start_time
    print(f"[*] Flashing finished in {elapsed:.2f}s ({len(dirty_pages)}/{total_pages} pages programmed, {skipped_pages} unchanged skipped)")

    # Force bootloader mode with token, then reset halt to flush flash cache safely
    client.cmd("mww 0x20000000 0x55AA55AA")
    client.cmd("reset halt")

    # Verify modified pages (or full image if force_full)
    print("[*] Verifying flash memory...")
    mismatches = 0
    pages_to_verify = dirty_pages if (dirty_pages and not force_full) else [
        (p, base_addr + p * PAGE_SIZE, [
            int.from_bytes(data[p * PAGE_SIZE + w : p * PAGE_SIZE + w + 4], "little")
            for w in range(0, PAGE_SIZE, 4)
        ])
        for p in range(total_pages)
    ]

    for _, page_addr, expected_words in pages_to_verify:
        read_vals = read_words(client, page_addr, words_per_page)
        for i in range(min(len(read_vals), words_per_page)):
            if read_vals[i] != expected_words[i]:
                print(f"[!] Mismatch at 0x{(page_addr + i*4):08X}: exp 0x{expected_words[i]:08X}, got 0x{read_vals[i]:08X}")
                mismatches += 1
                if mismatches > 10:
                    break
        if mismatches > 10:
            break

    if mismatches == 0:
        print(f"[+] Flash at 0x{base_addr:08X} VERIFIED SUCCESSFULLY!")
    else:
        print(f"[!] Verification FAILED with {mismatches} errors!")

    # Clear token and run application
    client.cmd("mww 0x20000000 0x00000000")
    client.cmd("reset run")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: nuc131_flasher.py <binary_file> <base_address_hex> [--full]")
        sys.exit(1)
        
    bin_file = sys.argv[1]
    base_address = int(sys.argv[2], 16)
    force_full_flash = "--full" in sys.argv

    ensure_openocd_running()
    cli = OpenOcdClient()
    try:
        flash_binary(cli, bin_file, base_address, force_full=force_full_flash)
    finally:
        cli.close()
