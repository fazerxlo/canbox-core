#!/usr/bin/env python3
import argparse
import glob
import os
import subprocess
import sys
import threading
import time
import tempfile
import serial
import serial.tools.list_ports
import select
import tty
import termios
import signal

import socket
import struct
import pty
import re

CAN_LINE_RE = re.compile(r"^(?:ID:)?(?:0x)?([0-9A-Fa-f]+)\s+(.+)$")

DEVICE_PROFILES = {
    "can": {
        "vid": 0x1A86,
        "pid": 0x7523,
        "name": "QinHeng Electronics CH340 serial converter",
        "role": "CAN (slcan)",
        "keywords": ["ch340", "qinheng", "1a86"],
    },
    "hu": {
        "vid": 0x067B,
        "pid": 0x2303,
        "name": "Prolific Technology, Inc. PL2303 Serial Port / Mobile Phone Data Cable",
        "role": "Head Unit (default hiworld)",
        "keywords": ["pl2303", "prolific", "067b"],
    },
    "orig": {
        "vid": 0x0403,
        "pid": 0x6001,
        "name": "Future Technology Devices International, Ltd FT232 Serial (UART) IC",
        "role": "Original Canbox (Optional)",
        "keywords": ["ft232", "ftdi", "0403"],
    },
}

def get_device_info(dev_path):
    vid = None
    pid = None
    desc = ""
    try:
        for p in serial.tools.list_ports.comports():
            if p.device == dev_path:
                vid = p.vid
                pid = p.pid
                parts = [p.description or "", p.product or "", p.manufacturer or ""]
                desc = " - ".join([x for x in parts if x])
                break
    except Exception:
        pass

    if vid is None or pid is None:
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
                    break
                p = os.path.dirname(p)
        except Exception:
            pass

    return vid, pid, desc

def format_port_label(dev_path):
    vid, pid, desc = get_device_info(dev_path)
    if vid is not None and pid is not None:
        vid_pid_hex = f"{vid:04x}:{pid:04x}"
        for prof in DEVICE_PROFILES.values():
            if prof["vid"] == vid and prof["pid"] == pid:
                return f"{dev_path} [ID {vid_pid_hex} {prof['name']}]"
        extra = f" {desc}" if desc else ""
        return f"{dev_path} [ID {vid_pid_hex}{extra}]"
    elif desc:
        return f"{dev_path} [{desc}]"
    return dev_path

def find_matching_device(role_key, available_devices):
    prof = DEVICE_PROFILES.get(role_key)
    if not prof:
        return None
    # 1. Exact VID:PID match
    for dev in available_devices:
        vid, pid, _ = get_device_info(dev)
        if vid == prof["vid"] and pid == prof["pid"]:
            return dev
    # 2. Keyword fallback in description
    for dev in available_devices:
        _, _, desc = get_device_info(dev)
        desc_lower = desc.lower()
        if any(kw in desc_lower for kw in prof["keywords"]):
            return dev
    return None

def get_serial_ports():
    return sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))

def prompt_choice(prompt_text, choices, default_choice=None, allow_skip=False):
    if not choices:
        print("No unassigned serial devices found.")
    else:
        for i, choice in enumerate(choices):
            print(f"{i+1}) {format_port_label(choice)}")
            
    if allow_skip:
        print("0) Skip")
    print("c) Custom path")
        
    default_str = None
    if default_choice and default_choice in choices:
        idx = choices.index(default_choice) + 1
        default_str = f"{idx} ({default_choice})"
    elif not default_choice and allow_skip:
        default_str = "Skip (0)"
    elif choices and not allow_skip:
        default_choice = choices[0]
        default_str = f"1 ({choices[0]})"
    
    if default_str:
        full_prompt = f"{prompt_text} [Default: {default_str}]: "
    else:
        full_prompt = f"{prompt_text}: "
        
    while True:
        ans = input(full_prompt).strip()
        
        if ans == "":
            if default_choice:
                return default_choice
            elif allow_skip:
                return None
                
        if allow_skip and ans == "0":
            return None
            
        if ans.lower() == "c":
            custom = input("Enter custom device path (e.g. /dev/ttyS0): ").strip()
            if custom:
                return custom
            continue
            
        try:
            idx = int(ans) - 1
            if 0 <= idx < len(choices):
                return choices[idx]
        except ValueError:
            if ans.startswith("/dev/"):
                return ans
                
        print("Invalid choice. Try again.")

def setup_slcan(dev):
    print(f"[*] Setting up slcan on {dev} to can0...")
    # These may require sudo password or nopasswd sudoers
    subprocess.run(["sudo", "slcan_attach", "-f", "-s6", "-o", dev], check=False)
    subprocess.run(["sudo", "slcand", "-S", "1000000", dev, "can0"], check=False)
    subprocess.run(["sudo", "ip", "link", "set", "dev", "can0", "up"], check=False)
    print("[*] slcan setup complete.")

def build_app():
    pio_path = os.path.expanduser("~/.platformio/penv/bin/pio")
    pio_cmd = pio_path if os.path.exists(pio_path) else "pio"
    print("[*] Building native app...")
    subprocess.run([pio_cmd, "run", "-e", "native_test"], check=True)
    bin_path = os.path.abspath(".pio/build/native_test/program")
    return bin_path



class Logger:
    def __init__(self):
        self.fd, self.temp_path = tempfile.mkstemp(prefix="canbox_e2e_", suffix=".log")
        self.file = os.fdopen(self.fd, "w")
        self.lock = threading.Lock()
        print(f"[*] Temporary log file created at {self.temp_path}")
        
    def log(self, source, message):
        ts = time.time()
        with self.lock:
            self.file.write(f"{ts:.3f} [{source}] {message}\n")
            self.file.flush()

    def dump_last_ns(self, ns = 5, compact = True):
        with self.lock:
            self.file.flush()
        
        import datetime
        dt_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        out_filename = f"dump_{dt_str}.log"
        print(f"\n[*] Dumping last {ns} seconds of logs to {out_filename}...\r")
        
        current_time = time.time()
        cutoff = current_time - float(ns)
        
        last_can_frames = {}
        total_count = 0
        written_count = 0
        compacted_count = 0

        try:
            with open(self.temp_path, "r") as f_in, open(out_filename, "w") as f_out:
                for line in f_in:
                    parts = line.split(" ", 2)
                    if len(parts) >= 2:
                        try:
                            ts = float(parts[0])
                            if ts < cutoff:
                                continue
                        except ValueError:
                            continue

                        total_count += 1
                        tag = parts[1]
                        if compact and tag == "[CAN]" and len(parts) > 2:
                            msg = parts[2].strip()
                            m = CAN_LINE_RE.match(msg)
                            if m:
                                try:
                                    can_id = int(m.group(1), 16)
                                    payload = m.group(2).strip()
                                    if last_can_frames.get(can_id) == payload:
                                        compacted_count += 1
                                        continue
                                    last_can_frames[can_id] = payload
                                except ValueError:
                                    pass

                        f_out.write(line)
                        written_count += 1

            if compact and compacted_count > 0:
                print(f"[*] Dump saved to {out_filename} ({written_count}/{total_count} lines, {compacted_count} repeated CAN frames compacted)\r")
            else:
                print(f"[*] Dump saved to {out_filename} ({written_count} lines)\r")
        except Exception as e:
            print(f"[!] Failed to dump logs: {e}\r")
            
    def cleanup(self):
        self.file.close()
        try:
            os.remove(self.temp_path)
            print(f"\n[*] Cleaned up temporary log file {self.temp_path}")
        except Exception:
            pass

def read_stream(stream, logger, source, stats):
    try:
        for line in iter(stream.readline, b''):
            if line:
                try:
                    text = line.decode('utf-8', errors='replace').rstrip()
                except:
                    text = repr(line)
                if text:
                    logger.log(source, text)
                    stats["APP_LINES"] += 1
    except Exception:
        pass

class UartProxy(threading.Thread):
    def __init__(self, real_port, baud, logger, stats):
        super().__init__(daemon=True)
        self.real_port = real_port
        self.baud = baud
        self.logger = logger
        self.stats = stats
        self.master_fd, self.slave_fd = pty.openpty()
        self.slave_name = os.ttyname(self.slave_fd)
        try:
            tty.setraw(self.master_fd)
            tty.setraw(self.slave_fd)
        except:
            pass

    def run(self):
        try:
            ser = serial.Serial(self.real_port, self.baud, timeout=0.1)
        except Exception as e:
            self.logger.log("HU_UART_ERR", str(e))
            return
            
        while True:
            try:
                r, _, _ = select.select([self.master_fd, ser.fileno()], [], [], 0.1)
                if self.master_fd in r:
                    data = os.read(self.master_fd, 1024)
                    if data:
                        self.logger.log("APP_UART_TX", f"{data.hex()}")
                        self.stats["HU_TX_BYTES"] += len(data)
                        ser.write(data)
                if ser.fileno() in r:
                    data = ser.read(ser.in_waiting or 1)
                    if data:
                        self.logger.log("APP_UART_RX", f"{data.hex()}")
                        self.stats["HU_RX_BYTES"] += len(data)
                        os.write(self.master_fd, data)
            except Exception:
                pass

class CanSniffer(threading.Thread):
    def __init__(self, iface, logger, stats):
        super().__init__(daemon=True)
        self.iface = iface
        self.logger = logger
        self.stats = stats
        self.sock = None

    def run(self):
        try:
            self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, 1) # CAN_RAW=1
            self.sock.bind((self.iface,))
            self.sock.setblocking(True)
            while True:
                pkt = self.sock.recv(16)
                if pkt and len(pkt) == 16:
                    can_id_raw, can_dlc, data = struct.unpack("=IB3x8s", pkt)
                    is_extended = (can_id_raw & 0x80000000) != 0
                    can_id = can_id_raw & (0x1FFFFFFF if is_extended else 0x000007FF)
                    dlc = min(can_dlc, 8)
                    data_hex = data[:dlc].hex()
                    self.logger.log("CAN", f"ID:{can_id:X} DLC:{dlc} DATA:{data_hex}")
                    self.stats["CAN_FRAMES"] += 1
        except Exception as e:
            self.logger.log("CAN_ERR", str(e))

def read_serial(port, baud, logger, source, stats):
    try:
        ser = serial.Serial(port, baud, timeout=1)
        while True:
            data = ser.read(ser.in_waiting or 1)
            if data:
                hex_data = data.hex()
                logger.log(source, f"RX: {hex_data}")
                stats["ORIG_BYTES"] += len(data)
    except Exception as e:
        logger.log(source, f"Serial error: {e}")

def main():
    parser = argparse.ArgumentParser(description="OpenCanbox E2E Test & Logger")
    parser.add_argument("--can", help="CAN device path (e.g. /dev/ttyUSB0 or socketcan interface like can0/vcan0)")
    parser.add_argument("--hu", "--serial", dest="hu", help="Head Unit serial device (e.g. /dev/ttyUSB1)")
    parser.add_argument("--orig", help="Original Canbox serial device (e.g. /dev/ttyUSB2)")
    parser.add_argument("--baud", type=int, default=38400, help="Head Unit serial baud rate (default: 38400)")
    parser.add_argument("--protocol", default="hiworld", help="Head Unit protocol (default: hiworld)")
    parser.add_argument("-y", "--auto", action="store_true", help="Auto-assign devices by detected USB signatures without prompting")
    args = parser.parse_args()

    print("=== OpenCanbox E2E Test & Logger ===")
    ports = get_serial_ports()

    if args.auto:
        print("[*] Auto-assigning devices by USB signatures...")
        can_dev = args.can or find_matching_device("can", ports) or (ports[0] if ports else None)
        if can_dev and can_dev in ports:
            ports.remove(can_dev)

        hu_dev = args.hu or find_matching_device("hu", ports) or (ports[0] if ports else None)
        if hu_dev and hu_dev in ports:
            ports.remove(hu_dev)

        orig_dev = args.orig or find_matching_device("orig", ports)
        if orig_dev and orig_dev in ports:
            ports.remove(orig_dev)

        print(f"[*] CAN device:      {format_port_label(can_dev) if can_dev else 'None'}")
        print(f"[*] Head Unit:       {format_port_label(hu_dev) if hu_dev else 'None'}")
        print(f"[*] Original Canbox: {format_port_label(orig_dev) if orig_dev else 'None (Skipped)'}")
    else:
        # 1. CAN device
        if args.can:
            can_dev = args.can
            if can_dev in ports:
                ports.remove(can_dev)
        else:
            can_default = find_matching_device("can", ports) or (ports[0] if ports else None)
            can_dev = prompt_choice("1) Select ttyUSB/ttyACM device for CAN (slcan)", ports, default_choice=can_default)
            if can_dev in ports:
                ports.remove(can_dev)

        # 2. HU device
        if args.hu:
            hu_dev = args.hu
            if hu_dev in ports:
                ports.remove(hu_dev)
        else:
            hu_default = find_matching_device("hu", ports) or (ports[0] if ports else None)
            hu_dev = prompt_choice("2) Select ttyUSB/ttyACM device for Head Unit (default hiworld)", ports, default_choice=hu_default)
            if hu_dev in ports:
                ports.remove(hu_dev)

        # 3. Original Canbox
        if args.orig is not None:
            orig_dev = args.orig
            if orig_dev in ports:
                ports.remove(orig_dev)
        else:
            orig_default = find_matching_device("orig", ports)
            orig_dev = prompt_choice("3) Select ttyUSB/ttyACM device for Original Canbox (Optional)", ports, default_choice=orig_default, allow_skip=True)
            if orig_dev and orig_dev in ports:
                ports.remove(orig_dev)

    if not can_dev:
        print("[!] Error: CAN device is required.")
        sys.exit(1)

    if not hu_dev:
        print("[!] Error: Head Unit device is required.")
        sys.exit(1)

    if can_dev.startswith("/dev/"):
        setup_slcan(can_dev)
        can_iface = "can0"
    else:
        can_iface = can_dev

    app_bin = build_app()

    logger = Logger()
    stats = {"APP_LINES": 0, "ORIG_BYTES": 0, "CAN_FRAMES": 0, "HU_TX_BYTES": 0, "HU_RX_BYTES": 0}

    can_sniffer = CanSniffer(can_iface, logger, stats)
    can_sniffer.start()

    # Start UART proxy for Head Unit connection
    uart_proxy = UartProxy(hu_dev, args.baud, logger, stats)
    uart_proxy.start()

    # Start original canbox logger if selected
    if orig_dev:
        t_orig = threading.Thread(target=read_serial, args=(orig_dev, args.baud, logger, "ORIG_CANBOX", stats), daemon=True)
        t_orig.start()

    # Start the app
    env = os.environ.copy()
    env["CANBOX_CAN_IFACE"] = can_iface
    env["CANBOX_UART_DEVICE"] = uart_proxy.slave_name
    env["CANBOX_HU_PROTOCOL"] = args.protocol
    
    print("[*] Starting the application...")
    
    # Use stdbuf to force line buffering for stdout/stderr
    app_proc = subprocess.Popen(
        ["stdbuf", "-oL", "-eL", app_bin],
        env=env,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT
    )
    
    t_app = threading.Thread(target=read_stream, args=(app_proc.stdout, logger, "APP", stats), daemon=True)
    t_app.start()
    
    print("[*] App started. Press 'L' to dump last 5s of logs. Press 'S' to dump last 30s of logs. Press 'q' or Esc to quit.\r")
    
    # Terminal setup to read single characters
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    
    def cleanup(signum=None, frame=None):
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        if app_proc.poll() is None:
            app_proc.terminate()
        logger.cleanup()
        print("\n[*] Exiting...")
        sys.exit(0)
        
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)
    
    try:
        tty.setcbreak(fd)
        last_stats_time = 0
        while app_proc.poll() is None:
            now = time.time()
            if now - last_stats_time > 0.5:
                # Print stats
                sys.stdout.write(f"\r[STATS] CAN: {stats['CAN_FRAMES']} | APP UART TX: {stats['HU_TX_BYTES']} RX: {stats['HU_RX_BYTES']} | ORIG RX: {stats['ORIG_BYTES']}   ")
                sys.stdout.flush()
                last_stats_time = now
                
            if select.select([sys.stdin], [], [], 0.1)[0]:
                ch = sys.stdin.read(1)
                if ch.lower() == 'l':
                    logger.dump_last_ns(5)
                elif ch.lower() == 's':
                    logger.dump_last_ns(30)
                elif ch.lower() == 'q' or ch == '\x1b':
                    break
                else:
                    if app_proc.stdin:
                        app_proc.stdin.write(ch.encode())
                        app_proc.stdin.flush()
    except Exception as e:
        print(f"\n[!] Error reading input: {e}")
    finally:
        cleanup()

if __name__ == "__main__":
    main()

