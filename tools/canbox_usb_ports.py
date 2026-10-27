#!/usr/bin/env python3
"""
Workbench USB Hardware Port Auto-Discovery
=========================================
Resolves serial devices by persistent USB VID:PID hardware signatures
(matching tools/canbox_e2e_logger.py) so port renumbering (/dev/ttyUSB0,
/dev/ttyUSB1, /dev/ttyUSB3, etc.) on reboots or reconnects never breaks
automation.

Hardware Profile Mapping:
- CAN Adapter   : QinHeng CH340   (VID 0x1A86, PID 0x7523) -> slcand / can0
- Canbox UART   : FTDI FT232R     (VID 0x0403, PID 0x6001) -> Canbox UART
- Head Unit UART: Prolific PL2303 (VID 0x067B, PID 0x2303) -> Android HU UART
"""

import os
import re
import glob
import subprocess
from typing import Optional, List, Dict, Any

try:
    import serial.tools.list_ports
except ImportError:
    pass

DEVICE_PROFILES = {
    "can": {
        "vid": 0x1A86,
        "pid": 0x7523,
        "name": "QinHeng CH340 CAN Adapter",
        "role": "CAN (slcand can0)",
    },
    "canbox": {
        "vid": 0x0403,
        "pid": 0x6001,
        "name": "FTDI FT232R Serial UART IC",
        "role": "Canbox UART",
    },
    "hu": {
        "vid": 0x067B,
        "pid": 0x2303,
        "name": "Prolific PL2303 Serial Port",
        "role": "Head Unit UART",
    },
}

def get_sysfs_vid_pid(dev_path: str) -> tuple[Optional[int], Optional[int]]:
    """Inspects Linux sysfs directly to extract USB VID:PID for a tty device."""
    try:
        name = os.path.basename(dev_path)
        syspath = os.path.realpath(f"/sys/class/tty/{name}/device")
        p = syspath
        while p and p != "/":
            vid_path = os.path.join(p, "idVendor")
            pid_path = os.path.join(p, "idProduct")
            if os.path.exists(vid_path) and os.path.exists(pid_path):
                with open(vid_path, "r") as f:
                    vid = int(f.read().strip(), 16)
                with open(pid_path, "r") as f:
                    pid = int(f.read().strip(), 16)
                return vid, pid
            p = os.path.dirname(p)
    except Exception:
        pass
    return None, None

def find_slcan_device() -> Optional[str]:
    """Finds which ttyUSB device is currently held by slcand or is the CH340 adapter."""
    # 1. Check running slcand process arguments
    try:
        res = subprocess.run(["ps", "-eo", "args"], capture_output=True, text=True)
        for line in res.stdout.splitlines():
            if "slcand" in line:
                m = re.search(r"(ttyUSB\d+|ttyACM\d+)", line)
                if m:
                    return f"/dev/{m.group(1)}"
    except Exception:
        pass

    # 2. Match CH340 VID:PID
    try:
        for p in serial.tools.list_ports.comports():
            if p.vid == DEVICE_PROFILES["can"]["vid"] and p.pid == DEVICE_PROFILES["can"]["pid"]:
                return p.device
    except Exception:
        pass

    for dev in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
        vid, pid = get_sysfs_vid_pid(dev)
        if vid == DEVICE_PROFILES["can"]["vid"] and pid == DEVICE_PROFILES["can"]["pid"]:
            return dev

    return None

def find_canbox_uart_device() -> str:
    """
    Finds the Canbox UART serial port by VID:PID (FTDI 0403:6001).
    Automatically skips the active SLCAN CAN port.
    Falls back gracefully to the first non-CAN serial port.
    """
    slcan_dev = find_slcan_device()

    # 1. Exact VID:PID match (FTDI FT232R 0403:6001)
    try:
        for p in serial.tools.list_ports.comports():
            if p.vid == DEVICE_PROFILES["canbox"]["vid"] and p.pid == DEVICE_PROFILES["canbox"]["pid"]:
                return p.device
    except Exception:
        pass

    # 2. Check sysfs for FTDI VID:PID
    candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    for dev in candidates:
        if dev == slcan_dev:
            continue
        vid, pid = get_sysfs_vid_pid(dev)
        if vid == DEVICE_PROFILES["canbox"]["vid"] and pid == DEVICE_PROFILES["canbox"]["pid"]:
            return dev

    # 3. Fallback: Any serial port that is NOT the CH340 CAN adapter
    for dev in candidates:
        if dev == slcan_dev:
            continue
        vid, pid = get_sysfs_vid_pid(dev)
        if vid != DEVICE_PROFILES["can"]["vid"] or pid != DEVICE_PROFILES["can"]["pid"]:
            return dev

    return "/dev/ttyUSB0"

def find_hu_uart_device() -> Optional[str]:
    """Finds the Head Unit UART port (PL2303 067B:2303)."""
    try:
        for p in serial.tools.list_ports.comports():
            if p.vid == DEVICE_PROFILES["hu"]["vid"] and p.pid == DEVICE_PROFILES["hu"]["pid"]:
                return p.device
    except Exception:
        pass
    for dev in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
        vid, pid = get_sysfs_vid_pid(dev)
        if vid == DEVICE_PROFILES["hu"]["vid"] and pid == DEVICE_PROFILES["hu"]["pid"]:
            return dev
    return None

if __name__ == "__main__":
    print("=" * 60)
    print("  Workbench USB Port Hardware Discovery")
    print("=" * 60)
    slcan = find_slcan_device()
    canbox = find_canbox_uart_device()
    hu = find_hu_uart_device()
    print(f"  • CAN (slcand/can0) : {slcan or 'NOT CONNECTED'} [CH340 1A86:7523]")
    print(f"  • Canbox UART       : {canbox or 'NOT CONNECTED'} [FTDI  0403:6001]")
    print(f"  • Head Unit UART    : {hu or 'NOT CONNECTED'} [PL2303 067B:2303]")
    print("=" * 60)
