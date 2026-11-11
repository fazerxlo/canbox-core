#!/usr/bin/env python3
"""
Non-destructive NUC131 Live Hardware Diagnostic & Sleep/Wake Qualifier Tool.
Auto-resolves ELF symbol addresses and keeps OpenOCD persistent across calls
so SWD re-initialization never resets the 3.0s CAN inactivity sleep timer.
"""
import argparse
import os
import socket
import subprocess
import sys
import time

DEFAULT_ELF = ".pio/build/raise_nuc131/program"
NM_BIN = os.path.expanduser(
    "~/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-nm"
)


class OpenOcdClient:
    def __init__(self, host="127.0.0.1", port=6666):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(3.0)
        self.sock.connect((host, port))

    def cmd(self, command: str) -> str:
        self.sock.sendall((command + "\x1a").encode("utf-8"))
        buf = bytearray()
        while b"\x1a" not in buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                break
            buf += chunk
        return buf.decode("utf-8", errors="ignore").rstrip("\x1a").strip()

    def mdw(self, addr: int) -> int:
        out = self.cmd(f"mdw 0x{addr:08X} 1")
        for line in out.splitlines():
            if ":" in line:
                return int(line.split(":")[1].strip(), 16)
        return 0

    def mdh(self, addr: int) -> int:
        out = self.cmd(f"mdh 0x{addr:08X} 1")
        for line in out.splitlines():
            if ":" in line:
                return int(line.split(":")[1].strip(), 16)
        return 0

    def mdb(self, addr: int) -> int:
        out = self.cmd(f"mdb 0x{addr:08X} 1")
        for line in out.splitlines():
            if ":" in line:
                return int(line.split(":")[1].strip(), 16)
        return 0

    def mww(self, addr: int, val: int):
        self.cmd(f"mww 0x{addr:08X} 0x{val:08X}")

    def close(self):
        self.sock.close()


def ensure_openocd_running():
    res = subprocess.run(["pgrep", "-x", "openocd"], capture_output=True)
    if res.returncode != 0:
        subprocess.Popen(
            ["openocd", "-f", "openocd_nuc131.cfg", "-d0"],
            stdout=open("/tmp/openocd.log", "w"),
            stderr=subprocess.STDOUT,
        )
        time.sleep(0.5)


def resolve_symbols(elf_path: str) -> dict:
    syms = {}
    if not os.path.exists(elf_path) or not os.path.exists(NM_BIN):
        return syms
    out = subprocess.check_output([NM_BIN, "-n", elf_path], text=True)
    wanted = {
        "s_can_bus_sleeping",
        "s_can_inactivity_ticks",
        "s_ticks_ms",
        "s_can_rx_rb",
        "s_irq_woke_by_can",
        "g_nuc131_can_woke_flag",
    }
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[2] in wanted:
            syms[parts[2]] = int(parts[0], 16)
    return syms


def read_snapshot(ocd: OpenOcdClient, syms: dict) -> dict:
    pa_dout = ocd.mdw(0x50004008)
    pc_dout = ocd.mdw(0x50004088)
    pd_pin = ocd.mdw(0x500040D0)
    pwrcon = ocd.mdw(0x50000200)
    can_sts = ocd.mdw(0x40180004)
    ispr = ocd.mdw(0xE000E200)
    snap = {
        "pa_dout": pa_dout,
        "acc": (pa_dout >> 8) & 1,
        "ill": (pa_dout >> 9) & 1,
        "pc_dout": pc_dout,
        "can_stb": (pc_dout >> 3) & 1,
        "can_rx_pin": (pd_pin >> 6) & 1,
        "pwrcon": pwrcon,
        "pwr_down_en": (pwrcon >> 7) & 1,
        "can_sts": can_sts,
        "ispr": ispr,
        "sleep": ocd.mdb(syms["s_can_bus_sleeping"]) if "s_can_bus_sleeping" in syms else -1,
        "inact": ocd.mdh(syms["s_can_inactivity_ticks"]) if "s_can_inactivity_ticks" in syms else -1,
        "ticks": ocd.mdw(syms["s_ticks_ms"]) if "s_ticks_ms" in syms else 0,
        "rx_head": ocd.mdw(syms["s_can_rx_rb"] + 16) if "s_can_rx_rb" in syms else 0,
    }
    return snap


def print_status(ocd: OpenOcdClient, syms: dict):
    s1 = read_snapshot(ocd, syms)
    time.sleep(0.25)
    s2 = read_snapshot(ocd, syms)
    dt = s2["ticks"] - s1["ticks"]
    drx = s2["rx_head"] - s1["rx_head"]
    print(
        f"[NUC131] sleep={s2['sleep']} | inact={s2['inact']}/30 | "
        f"ticks=0x{s2['ticks']:06X} (+{dt} ms/250ms) | CAN_RX=+{drx} frames ({drx*4} fps)"
    )
    print(
        f"         PA_DOUT=0x{s2['pa_dout']:04X} (ACC={s2['acc']}, ILL={s2['ill']}) | "
        f"PC_DOUT=0x{s2['pc_dout']:04X} (CAN_STB={s2['can_stb']}) | "
        f"PWRCON=0x{s2['pwrcon']:04X} (PWR_DOWN_EN={s2['pwr_down_en']}) | "
        f"CAN0_STS=0x{s2['can_sts']:04X}"
    )


def run_sleep_glitch_test(ocd: OpenOcdClient, syms: dict) -> bool:
    print("[*] Waiting up to 3.5s for NUC131 deep sleep entry (requires silent CAN bus)...")
    deadline = time.time() + 3.5
    s = read_snapshot(ocd, syms)
    while time.time() < deadline:
        if s["sleep"] == 1 and s["pwr_down_en"] == 1:
            break
        time.sleep(0.2)
        s = read_snapshot(ocd, syms)

    if s["sleep"] != 1 or s["pwr_down_en"] != 1 or s["can_stb"] != 1:
        print(
            f"[!] MCU did not enter deep sleep (sleep={s['sleep']}, CAN_STB={s['can_stb']}, "
            f"PWR_DOWN_EN={s['pwr_down_en']}, inact={s['inact']}). Is the BSI still transmitting on CAN?"
        )
        return False

    print(
        f"[+] Deep sleep confirmed: sleep={s['sleep']}, ticks=0x{s['ticks']:06X}, "
        f"ACC={s['acc']}, CAN_STB={s['can_stb']}, PWR_DOWN_EN={s['pwr_down_en']}"
    )
    print("[*] Injecting 10 transient CAN/GPIO wake glitches (GPCDEF_IRQn | CAN0_IRQn)...")
    for _ in range(10):
        ocd.mww(0xE000E200, (1 << 5) | (1 << 20))
        time.sleep(0.01)

    s_after = read_snapshot(ocd, syms)
    passed = (
        s_after["sleep"] == 1
        and s_after["ticks"] == s["ticks"]
        and s_after["acc"] == 0
        and s_after["can_stb"] == 1
        and s_after["pwr_down_en"] == 1
        and (s_after["ispr"] & ((1 << 5) | (1 << 20))) == 0
    )
    status_str = "PASS" if passed else "FAIL"
    print(
        f"[{status_str}] Post-glitch state: sleep={s_after['sleep']}, "
        f"ticks=0x{s_after['ticks']:06X} (unchanged={s_after['ticks'] == s['ticks']}), "
        f"ACC={s_after['acc']}, CAN_STB={s_after['can_stb']}, "
        f"PWR_DOWN_EN={s_after['pwr_down_en']}, ISPR=0x{s_after['ispr']:08X}"
    )
    return passed


def main():
    parser = argparse.ArgumentParser(description="NUC131 non-destructive live diagnostic tool")
    parser.add_argument("--elf", default=DEFAULT_ELF, help="Path to ELF file for symbol lookup")
    parser.add_argument("--test-sleep-glitch", action="store_true", help="Verify deep sleep & 10x glitch rejection")
    parser.add_argument("--stop-openocd", action="store_true", help="Stop OpenOCD daemon on exit")
    args = parser.parse_args()

    ensure_openocd_running()
    syms = resolve_symbols(args.elf)
    ocd = OpenOcdClient()
    try:
        if args.test_sleep_glitch:
            ok = run_sleep_glitch_test(ocd, syms)
            sys.exit(0 if ok else 1)
        else:
            print_status(ocd, syms)
    finally:
        ocd.close()
        if args.stop_openocd:
            subprocess.run(["pkill", "-x", "openocd"], check=False)


if __name__ == "__main__":
    main()
