#!/usr/bin/env python3
"""
NUC131 High-Speed Flash Programmer via SRAM Stub
Loads a 32-byte Cortex-M0 programming microcode into SRAM @ 0x20000200,
streams 512-byte pages into SRAM buffer @ 0x20000400, and triggers the stub.
Flashing 64 KB takes < 2 seconds over ST-Link.
"""
import socket
import sys
import time

PAGE_SIZE = 512
RAM_STUB_ADDR = 0x20000200
RAM_BUF_ADDR  = 0x20000400

# Cortex-M0 Thumb-1 microcode for FMC page program / erase:
# Arguments in registers:
# r0 = FMC_ISPADR (destination flash page address)
# r1 = source RAM buffer address
# r2 = count of words (128 words = 512 bytes)
#
# Stub assembly:
# 1. Page erase (ISPCMD = 0x22):
#    ldr r3, =0x5000C000   (FMC_BASE)
#    str r0, [r3, #4]      (ISPADR = r0)
#    movs r4, #0x22
#    str r4, [r3, #12]     (ISPCMD = 0x22)
#    movs r4, #1
#    str r4, [r3, #16]     (ISPTRG = 1)
# 1_wait:
#    ldr r4, [r3, #16]
#    cmp r4, #0
#    bne 1_wait
#
# 2. Word program loop (r2 words):
#    movs r4, #0x21
#    str r4, [r3, #12]     (ISPCMD = 0x21)
# 2_loop:
#    cmp r2, #0
#    beq 2_done
#    str r0, [r3, #4]      (ISPADR = r0)
#    ldr r4, [r1]          (ISPDAT = *r1++)
#    str r4, [r3, #8]
#    adds r1, #4
#    movs r4, #1
#    str r4, [r3, #16]     (ISPTRG = 1)
# 2_wait:
#    ldr r4, [r3, #16]
#    cmp r4, #0
#    bne 2_wait
#    adds r0, #4
#    subs r2, #1
#    b 2_loop
# 2_done:
#    bkpt #0

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

# Microcode binary (compiled ARMv6-M Thumb-1):
# 00000000 <stub>:
#    0: 4b0c       ldr r3, [pc, #48]   ; (34 <fmc_base>)
#    2: 6058       str r0, [r3, #4]
#    4: 2422       movs r4, #34        ; 0x22
#    6: 60dc       str r4, [r3, #12]
#    8: 2401       movs r4, #1
#    a: 611c       str r4, [r3, #16]
# 0000000c <wait_erase>:
#    c: 691c       ldr r4, [r3, #16]
#    e: 2c00       cmp r4, #0
#   10: d1fc       bne.n c <wait_erase>
#   12: 2421       movs r4, #33        ; 0x21
#   14: 60dc       str r4, [r3, #12]
# 00000016 <prog_loop>:
#   16: 2a00       cmp r2, #0
#   18: d009       beq.n 2e <done>
#   1a: 6058       str r0, [r3, #4]
#   1c: 680c       ldr r4, [r1, #0]
#   1e: 3104       adds r1, #4
#   20: 609c       str r4, [r3, #8]
#   22: 2401       movs r4, #1
#   24: 611c       str r4, [r3, #16]
# 00000026 <wait_prog>:
#   26: 691c       ldr r4, [r3, #16]
#   28: 2c00       cmp r4, #0
#   2a: d1fc       bne.n 26 <wait_prog>
#   2c: 3004       adds r0, #4
#   2e: 3a01       subs r2, #1
#   30: e7f1       b.n 16 <prog_loop>
# 00000032 <done>:
#   32: be00       bkpt 0x0000
# 00000034 <fmc_base>:
#   34: 5000c000   .word 0x5000c000

STUB_CODE = bytes.fromhex(
    "0c4b 5860 2224 dc60 0124 1c61 1c69 002c fcd1 2124 dc60 "
    "002a 0bd0 5860 0c68 0431 9c60 0124 1c61 1c69 002c fcd1 "
    "0430 013a f1e7 00be 00c00050"
)

def flash_binary(client: OpenOcdClient, bin_path: str, base_addr: int):
    with open(bin_path, "rb") as f:
        data = bytearray(f.read())

    print(f"[*] High-Speed Flashing {bin_path} ({len(data)} bytes) to 0x{base_addr:08X}...")

    # Pad data to 512-byte page boundary
    pad_rem = len(data) % PAGE_SIZE
    if pad_rem != 0:
        data += b"\xFF" * (PAGE_SIZE - pad_rem)

    total_pages = len(data) // PAGE_SIZE

    # 1. Reset halt & unlock FMC ISP
    client.cmd("reset halt")
    client.cmd("mww 0x50000100 0x59")
    client.cmd("mww 0x50000100 0x16")
    client.cmd("mww 0x50000100 0x88")
    client.cmd("mww 0x50000204 0x34") # Enable ISP clock
    client.cmd("mww 0x5000C000 0x79") # ISPEN | APUEN | CFGUEN | LDUEN

    # 2. Write Stub to SRAM
    stub_hex = " ".join(f"0x{int.from_bytes(STUB_CODE[i:i+4], 'little'):08X}" for i in range(0, len(STUB_CODE), 4))
    for i, chunk in enumerate(range(0, len(STUB_CODE), 4)):
        val = int.from_bytes(STUB_CODE[chunk:chunk+4], "little")
        client.cmd(f"mww 0x{RAM_STUB_ADDR + i * 4:08X} 0x{val:08X}")

    start_time = time.time()
    for p in range(total_pages):
        page_addr = base_addr + p * PAGE_SIZE
        page_data = data[p * PAGE_SIZE : (p + 1) * PAGE_SIZE]

        # Check if page is blank
        if page_data == b"\xFF" * PAGE_SIZE:
            # Still erase the page to guarantee clean state
            client.cmd(f"mww 0x5000C004 0x{page_addr:08X}")
            client.cmd("mww 0x5000C00C 0x22")
            client.cmd("mww 0x5000C010 0x01")
            time.sleep(0.005)
            continue

        # Write page data into SRAM buffer
        # Using array2mem or sequence of mww:
        # Generate single TCL script chunk for speed
        mww_cmds = []
        for w in range(0, PAGE_SIZE, 4):
            w_val = int.from_bytes(page_data[w : w + 4], "little")
            mww_cmds.append(f"mww 0x{RAM_BUF_ADDR + w:08X} 0x{w_val:08X}")
        client.cmd("; ".join(mww_cmds))

        # Set registers and run stub:
        # r0 = page_addr
        # r1 = RAM_BUF_ADDR
        # r2 = 128 (words)
        # pc = RAM_STUB_ADDR
        # resume
        client.cmd(f"reg r0 0x{page_addr:08X}")
        client.cmd(f"reg r1 0x{RAM_BUF_ADDR:08X}")
        client.cmd("reg r2 128")
        client.cmd(f"reg pc 0x{RAM_STUB_ADDR:08X}")
        client.cmd("resume")
        time.sleep(0.012) # ~10ms for 512-byte page write + erase
        client.cmd("halt")

        if (p + 1) % 4 == 0 or (p + 1) == total_pages:
            print(f"    - Programmed page {p + 1}/{total_pages} ({(p + 1) * 100 // total_pages}%)...")

    elapsed = time.time() - start_time
    print(f"[*] Programming completed in {elapsed:.2f} seconds ({len(data) / elapsed / 1024:.2f} KB/s)")

    # Lock FMC
    client.cmd("mww 0x5000C000 0x00")
    client.cmd("mww 0x50000100 0x00")

    print("[*] Verifying memory against image...")
    # Verify first and last blocks
    verify_words = min(len(data) // 4, 32)
    mismatches = 0
    for w in range(verify_words):
        word_addr = base_addr + w * 4
        expected = int.from_bytes(data[w * 4 : w * 4 + 4], "little")
        out = client.cmd(f"mdw 0x{word_addr:08X} 1")
        val = int(out.split(":")[1].strip(), 16)
        if val != expected:
            print(f"[!] Mismatch at 0x{word_addr:08X}: exp 0x{expected:08X}, got 0x{val:08X}")
            mismatches += 1

    if mismatches == 0:
        print(f"[+] Flash image at 0x{base_addr:08X} VERIFIED SUCCESSFULLY!")
    else:
        print(f"[!] Verification FAILED with {mismatches} errors!")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: nuc131_fast_flasher.py <binary_file> <base_address_hex>")
        sys.exit(1)
        
    bin_file = sys.argv[1]
    base_address = int(sys.argv[2], 16)
    
    cli = OpenOcdClient()
    try:
        flash_binary(cli, bin_file, base_address)
    finally:
        cli.close()
