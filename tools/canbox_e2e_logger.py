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
import select
import tty
import termios
import signal

import socket
import struct
import pty

def get_serial_ports():
    return sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))

def prompt_choice(prompt_text, choices, allow_skip=False):
    if not choices:
        print("No unassigned serial devices found.")
    else:
        for i, choice in enumerate(choices):
            print(f"{i+1}) {choice}")
            
    if allow_skip:
        print("0) Skip")
    print("c) Custom path")
        
    default_option = choices[0] if choices else ("0" if allow_skip else None)
    
    if default_option and default_option != "0":
        full_prompt = f"{prompt_text} [Default: 1]: "
    elif default_option == "0":
        full_prompt = f"{prompt_text} [Default: Skip]: "
    else:
        full_prompt = f"{prompt_text}: "
        
    while True:
        ans = input(full_prompt).strip()
        
        if ans == "":
            if default_option == "0":
                return None
            elif default_option:
                return default_option
                
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

    def dump_last_30s(self):
        with self.lock:
            self.file.flush()
        
        import datetime
        dt_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        out_filename = f"dump_{dt_str}.log"
        print(f"\n[*] Dumping last 30 seconds of logs to {out_filename}...\r")
        
        current_time = time.time()
        cutoff = current_time - 30.0
        
        try:
            with open(self.temp_path, "r") as f_in, open(out_filename, "w") as f_out:
                for line in f_in:
                    parts = line.split(" ", 1)
                    if len(parts) == 2:
                        try:
                            ts = float(parts[0])
                            if ts >= cutoff:
                                f_out.write(line)
                        except ValueError:
                            pass
            print(f"[*] Dump saved to {out_filename}\r")
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
    print("=== OpenCanbox E2E Test & Logger ===")
    ports = get_serial_ports()
    
    # 1. Ask CAN device
    can_dev = prompt_choice("1) Select ttyUSB/ttyACM device for CAN (slcan)", ports)
    if can_dev in ports:
        ports.remove(can_dev)
    
    # 2. Ask HU device
    hu_dev = prompt_choice("2) Select ttyUSB/ttyACM device for Head Unit (default hiworld)", ports)
    if hu_dev in ports:
        ports.remove(hu_dev)
    
    # 4. Ask Original Canbox
    orig_dev = prompt_choice("3) Select ttyUSB/ttyACM device for Original Canbox (Optional)", ports, allow_skip=True)
    if orig_dev and orig_dev in ports:
        ports.remove(orig_dev)
    
    setup_slcan(can_dev)
    app_bin = build_app()
    
    logger = Logger()
    stats = {"APP_LINES": 0, "ORIG_BYTES": 0, "CAN_FRAMES": 0, "HU_TX_BYTES": 0, "HU_RX_BYTES": 0}
    
    can_sniffer = CanSniffer("can0", logger, stats)
    can_sniffer.start()
    
    # Start UART proxy for Head Unit connection
    uart_proxy = UartProxy(hu_dev, 38400, logger, stats)
    uart_proxy.start()
    
    # Start original canbox logger if selected
    if orig_dev:
        # Assuming 38400 baud for original canbox too
        t_orig = threading.Thread(target=read_serial, args=(orig_dev, 38400, logger, "ORIG_CANBOX", stats), daemon=True)
        t_orig.start()
        
    # 3. Start the app
    env = os.environ.copy()
    env["CANBOX_CAN_IFACE"] = "can0"
    env["CANBOX_UART_DEVICE"] = uart_proxy.slave_name
    env["CANBOX_HU_PROTOCOL"] = "hiworld"
    
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
    
    print("[*] App started. Press 'L' to dump last 30s of logs. Press 'q' or Esc to quit.\r")
    
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
                    logger.dump_last_30s()
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

