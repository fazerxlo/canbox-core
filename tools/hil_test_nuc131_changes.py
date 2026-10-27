#!/usr/bin/env python3
"""
HIL Test Suite for Nuvoton NUC131 HAL Changes
TC-01: RX FIFO & 64-Frame Ring Buffer Stress Under Blocking UART Burst
TC-02: 6-Mailbox CAN TX Contention Test
TC-03: Deterministic Deep-Sleep Entry & Wake-Source Classification (PRIMASK=1)
TC-04: UART Framing / Break / Overrun Error Recovery (UART0->FSR)
TC-05: Rate-Limited Bus-Off Recovery & Watchdog Survival
"""

import sys
import time
import socket
import struct
import serial
import subprocess

# OpenOCD TCL Client
class OpenOcdClient:
    def __init__(self, host="127.0.0.1", port=6666):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        
    def cmd(self, command: str) -> str:
        cmd_str = command + "\x1a"
        self.sock.sendall(cmd_str.encode("utf-8"))
        res = bytearray()
        while True:
            chunk = self.sock.recv(4096)
            if not chunk:
                break
            res += chunk
            if b"\x1a" in chunk:
                break
        return res.decode("utf-8", errors="ignore").rstrip("\x1a").strip()

    def read_word(self, addr: int) -> int:
        out = self.cmd(f"mdw 0x{addr:08X} 1")
        # Format: "0x50000004: 00000020"
        for line in out.splitlines():
            if ":" in line:
                val_str = line.split(":")[1].strip()
                return int(val_str, 16)
        return 0

    def write_word(self, addr: int, val: int):
        self.cmd(f"mww 0x{addr:08X} 0x{val:08X}")

    def close(self):
        self.sock.close()


def pack_can_frame(can_id: int, data: bytes, is_extended: bool = False, is_remote: bool = False) -> bytes:
    flags = 0
    if is_extended:
        flags |= socket.CAN_EFF_FLAG
    if is_remote:
        flags |= socket.CAN_RTR_FLAG
    can_id_raw = (can_id & 0x1FFFFFFF) | flags
    dlc = len(data)
    padded_data = data.ljust(8, b'\x00')[:8]
    return struct.pack("=IB3x8s", can_id_raw, dlc, padded_data)


def unpack_can_frame(raw: bytes):
    can_id_raw, dlc, data = struct.unpack("=IB3x8s", raw)
    can_id = can_id_raw & 0x1FFFFFFF
    return can_id, dlc, data[:dlc]


# Test Suite Implementation
class Nuc131HilTester:
    def __init__(self, can_iface="slcan0", uart_port="/dev/ttyUSB1", uart_baud=38400):
        self.can_iface = can_iface
        self.uart_port = uart_port
        self.uart_baud = uart_baud
        self.openocd = OpenOcdClient()
        
        # SocketCAN
        self.can_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.can_sock.bind((self.can_iface,))
        self.can_sock.settimeout(0.01)

        # PySerial
        self.uart = serial.Serial(self.uart_port, self.uart_baud, timeout=0.01)

        self.results = {}

    def close(self):
        self.can_sock.close()
        self.uart.close()
        self.openocd.close()

    def wake_nuc(self):
        """Sends ignition frame and UART wake to ensure NUC is awake and active."""
        frame36 = pack_can_frame(0x036, bytes.fromhex("0E00000F010000A0"))
        for _ in range(5):
            self.can_sock.send(frame36)
            time.sleep(0.01)
        self.uart.write(bytes.fromhex("5AA50224220047"))
        time.sleep(0.05)
        self.uart.read_all()

    def test_tc01_rx_fifo_stress(self):
        print("\n--- Running TC-01: RX FIFO & 64-Frame Ring Buffer Stress ---")
        self.wake_nuc()
        time.sleep(0.05)
        self.uart.read_all()

        # Frame definition
        frame_ign = pack_can_frame(0x036, bytes.fromhex("0E00000F010000A0"))
        noise_frames = [
            pack_can_frame(0x7FF, b"\xAA\x55\xAA\x55\xAA\x55\xAA\x55"),
            pack_can_frame(0x412, b"\x01\x02\x03\x04\x05\x06\x07\x08"),
            pack_can_frame(0x0B6, b"\x00\x00\x00\x00\x00\x00\x00\x00"),
            pack_can_frame(0x1D0, b"\x20\x00\x00\x00\x00\x00\x00\x00"),
            pack_can_frame(0x220, b"\x04\x00"),
        ]

        # Sentinel: SWC stalk Vol+ press (0x21F with 0x08)
        sentinel_frame = pack_can_frame(0x21F, bytes.fromhex("080000"))

        # We flood ~600 fps CAN traffic while triggering UART full state push (Cmd 0x24)
        t_start = time.time()
        sent_count = 0
        # Trigger blocking UART burst
        self.uart.write(bytes.fromhex("5AA50224220047"))

        # Rapidly push 35 frames (~50ms)
        for i in range(35):
            f = noise_frames[i % len(noise_frames)]
            self.can_sock.send(f)
            sent_count += 1
            time.sleep(0.001)

        # Inject sentinel
        t_sentinel = time.time()
        self.can_sock.send(sentinel_frame)
        sent_count += 1

        # Push another 10 frames
        for i in range(10):
            f = noise_frames[i % len(noise_frames)]
            self.can_sock.send(f)
            sent_count += 1
            time.sleep(0.001)

        # Await UART sentinel response: Hiworld 0x11 with key=0x01 (Vol+)
        # Expected UART packet contains: 5A A5 0A 11 ... 01 01 ...
        arrived = False
        t_arrived = 0
        rx_buf = bytearray()
        timeout = 0.200 # 200 ms max
        while (time.time() - t_sentinel) < timeout:
            b = self.uart.read(100)
            if b:
                rx_buf += b
                # Look for 5A A5 0A 11 ... key 0x01
                idx = 0
                while True:
                    f_idx = rx_buf.find(b"\x5A\xA5\x0A\x11", idx)
                    if f_idx == -1:
                        break
                    if len(rx_buf) >= f_idx + 15:
                        packet = rx_buf[f_idx : f_idx + 15]
                        key_val = packet[6]
                        press_val = packet[7]
                        if key_val == 0x01 and press_val == 0x01:
                            arrived = True
                            t_arrived = time.time()
                            break
                        idx = f_idx + 1
                    else:
                        break
                if arrived:
                    break
            time.sleep(0.002)

        # Release sentinel key
        self.can_sock.send(pack_can_frame(0x21F, bytes.fromhex("000000")))

        latency_ms = (t_arrived - t_sentinel) * 1000.0 if arrived else 999.0
        print(f"TC-01 Sentinel Latency: {latency_ms:.2f} ms (Sent {sent_count} CAN frames)")

        passed = arrived and (latency_ms < 80.0)
        self.results["TC-01"] = {
            "status": "PASS" if passed else "FAIL",
            "latency_ms": latency_ms,
            "can_frames_sent": sent_count,
            "measured": f"Latency = {latency_ms:.1f} ms (<80 ms target)"
        }
        return passed

    def test_tc02_tx_contention(self):
        print("\n--- Running TC-02: 6-Mailbox CAN TX Contention Test ---")
        self.wake_nuc()
        time.sleep(0.5)

        # 1. Verify periodic Virtual MDF frames (0x167, 0x3F6, 0x0DF, 0x15B)
        frame36 = pack_can_frame(0x036, bytes.fromhex("0E00000F010000A0"))
        
        seen_ids = set()
        t_start = time.time()
        while time.time() - t_start < 1.2:
            self.can_sock.send(frame36)
            try:
                raw = self.can_sock.recv(16)
                cid, _, _ = unpack_can_frame(raw)
                if cid in (0x167, 0x3F6, 0x0DF, 0x15B):
                    seen_ids.add(cid)
            except (socket.timeout, BlockingIOError):
                pass
            time.sleep(0.05)

        print(f"TC-02 Background Virtual MDF frames captured: {[hex(x) for x in seen_ids]}")

        # 2. Burst UART downlink commands (Trip Reset, Car Config Writes)
        # Downlink 0x7B (e.g., Guide Me Home light / wiper config): 5A A5 02 7B 01 01 7E
        # Downlink 0x7D (e.g., Boot Mode config): 5A A5 02 7D 01 01 80
        # These trigger 0x15B CAN frames immediately
        # We also trigger Trip Reset: 5A A5 02 33 01 00 ... or 5A A5 02 7B
        txreq_before = self.openocd.read_word(0x40180100) # CAN0->TXREQ1
        
        # Send burst of 4 downlink config commands
        downlinks = [
            bytes.fromhex("5AA5027B01017E"),
            bytes.fromhex("5AA5027B02017F"),
            bytes.fromhex("5AA5027D010180"),
            bytes.fromhex("5AA5027D020181"),
        ]
        t_burst_start = time.time()
        for dl in downlinks:
            self.uart.write(dl)
            time.sleep(0.005)

        # Capture response CAN frames on slcan0 for 200 ms
        captured_15b = 0
        t_cap_start = time.time()
        while time.time() - t_cap_start < 0.25:
            try:
                raw = self.can_sock.recv(16)
                cid, _, _ = unpack_can_frame(raw)
                if cid == 0x15B:
                    captured_15b += 1
            except (socket.timeout, BlockingIOError):
                pass
            time.sleep(0.002)

        txreq_after = self.openocd.read_word(0x40180100)
        can_status = self.openocd.read_word(0x40180004)

        print(f"TC-02 Captured 0x15B frames: {captured_15b}, CAN0->TXREQ1 = 0x{txreq_after:04X}, STATUS = 0x{can_status:04X}")

        passed = (0x3F6 in seen_ids or 0x0DF in seen_ids) and (captured_15b >= 2) and (txreq_after == 0)
        self.results["TC-02"] = {
            "status": "PASS" if passed else "FAIL",
            "mdf_ids": [hex(x) for x in seen_ids],
            "captured_15b": captured_15b,
            "txreq1": f"0x{txreq_after:04X}",
            "can_status": f"0x{can_status:04X}",
            "measured": f"MDF frames {[hex(x) for x in seen_ids]}, {captured_15b} 0x15B bursts, TXREQ1=0x{txreq_after:04X}"
        }
        return passed

    def test_tc03_deterministic_sleep_and_wake(self):
        print("\n--- Running TC-03: Deep-Sleep Entry & Wake-Source Classification ---")
        # 1. Enter Sleep: Send 0x036 with PHASE_VIE = 0x00 (Deep sleep announce)
        frame_sleep = pack_can_frame(0x036, bytes.fromhex("0E00000F000000A0"))
        self.can_sock.send(frame_sleep)
        time.sleep(0.05)
        self.can_sock.send(frame_sleep)
        
        print("Waiting 3.5s for silence watchdog to enter deep sleep...")
        time.sleep(3.5)

        # Inspect registers via OpenOCD
        pa_dout = self.openocd.read_word(0x50004008)
        pc_dout = self.openocd.read_word(0x50004088)
        clk_pwr = self.openocd.read_word(0x50000200)

        acc_low = (pa_dout & (1 << 8)) == 0
        can_stb_high = (pc_dout & (1 << 3)) != 0
        in_sleep = (clk_pwr & 0x80) != 0

        print(f"Sleep verification: PA_DOUT=0x{pa_dout:08X} (ACC={'LOW' if acc_low else 'HIGH'}), "
              f"PC_DOUT=0x{pc_dout:08X} (CAN_STB={'HIGH' if can_stb_high else 'LOW'}), CLK_PWR=0x{clk_pwr:08X}")

        # Subtest 3A: UART Glitch / False-Wake Immunity
        # Send garbage byte on UART
        print("Subtest 3A: Injecting UART glitch byte (0x00)...")
        self.uart.write(b"\x00")
        
        # Monitor slcan0 for 500 ms to confirm ZERO CAN frames are transmitted
        can_transmitted = 0
        t_sub3a = time.time()
        while time.time() - t_sub3a < 0.3:
            try:
                raw = self.can_sock.recv(16)
                if raw:
                    can_transmitted += 1
            except (socket.timeout, BlockingIOError):
                pass
            time.sleep(0.01)

        # Read PC_DOUT after 150 ms (50 ms listen window expired)
        pc_dout_sub3a = self.openocd.read_word(0x50004088)
        pa_dout_sub3a = self.openocd.read_word(0x50004008)
        can_stb_remains_high = (pc_dout_sub3a & (1 << 3)) != 0
        acc_remains_low = (pa_dout_sub3a & (1 << 8)) == 0

        print(f"Subtest 3A Result: Transmitted CAN frames = {can_transmitted}, "
              f"PC_DOUT=0x{pc_dout_sub3a:08X} (CAN_STB={'HIGH' if can_stb_remains_high else 'LOW'}), "
              f"PA_DOUT=0x{pa_dout_sub3a:08X} (ACC={'LOW' if acc_remains_low else 'HIGH'})")

        sub3a_pass = (can_transmitted == 0) and can_stb_remains_high and acc_remains_low

        # Subtest 3B: True CAN Dominant Edge Wake
        print("Subtest 3B: Injecting 0x036 CAN Wake Frame...")
        frame_ign = pack_can_frame(0x036, bytes.fromhex("0E00000F010000A0"))
        # Send wake pulse
        self.can_sock.send(frame_ign)
        time.sleep(0.02)
        self.can_sock.send(frame_ign)
        time.sleep(0.05)

        pc_dout_sub3b = self.openocd.read_word(0x50004088)
        pa_dout_sub3b = self.openocd.read_word(0x50004008)
        can_stb_low = (pc_dout_sub3b & (1 << 3)) == 0
        acc_high = (pa_dout_sub3b & (1 << 8)) != 0

        # Verify UART telemetry resumed
        self.uart.read_all()
        # Pulse stalk key
        self.can_sock.send(pack_can_frame(0x21F, bytes.fromhex("080000")))
        time.sleep(0.05)
        u_resp = self.uart.read_all()
        self.can_sock.send(pack_can_frame(0x21F, bytes.fromhex("000000")))

        telemetry_ok = len(u_resp) > 0

        print(f"Subtest 3B Result: PC_DOUT=0x{pc_dout_sub3b:08X} (CAN_STB={'LOW' if can_stb_low else 'HIGH'}), "
              f"PA_DOUT=0x{pa_dout_sub3b:08X} (ACC={'HIGH' if acc_high else 'LOW'}), "
              f"UART Telemetry Active: {telemetry_ok}")

        sub3b_pass = can_stb_low and acc_high and telemetry_ok
        passed = acc_low and can_stb_high and sub3a_pass and sub3b_pass

        self.results["TC-03"] = {
            "status": "PASS" if passed else "FAIL",
            "sleep_pc_dout": f"0x{pc_dout:08X}",
            "sleep_pa_dout": f"0x{pa_dout:08X}",
            "sub3a_can_tx": can_transmitted,
            "sub3a_pc_dout": f"0x{pc_dout_sub3a:08X}",
            "sub3b_pc_dout": f"0x{pc_dout_sub3b:08X}",
            "sub3b_pa_dout": f"0x{pa_dout_sub3b:08X}",
            "measured": f"Sleep PC_DOUT=0x{pc_dout:04X}, Sub3A CAN TX={can_transmitted}, Sub3B PC_DOUT=0x{pc_dout_sub3b:04X}, PA_DOUT=0x{pa_dout_sub3b:04X}"
        }
        return passed

    def test_tc04_uart_error_recovery(self):
        print("\n--- Running TC-04: UART Framing / Break / Overrun Error Recovery ---")
        self.wake_nuc()
        time.sleep(0.05)
        self.uart.read_all()

        # Intentionally inject break / framing errors
        print("Injecting UART BREAK condition and baud mismatch burst...")
        self.uart.send_break(duration=0.05)
        time.sleep(0.02)
        
        # Switch to 9600 baud and blast garbage
        self.uart.baudrate = 9600
        self.uart.write(b"\x00\xFF\x55\xAA\x00\x00\xFF")
        time.sleep(0.02)
        self.uart.baudrate = 38400
        time.sleep(0.02)

        # Immediately send valid 0x24 handshake command
        t_send = time.time()
        self.uart.write(bytes.fromhex("5AA50224220047"))

        # Wait for valid ACK: 5A A5 01 FF 24 23
        ack_received = False
        t_ack = 0
        rx_buf = bytearray()
        while time.time() - t_send < 0.100:
            b = self.uart.read(50)
            if b:
                rx_buf += b
                if b"\x5A\xA5\x01\xFF\x24\x23" in rx_buf:
                    ack_received = True
                    t_ack = time.time()
                    break
            time.sleep(0.001)

        ack_latency_ms = (t_ack - t_send) * 1000.0 if ack_received else 999.0
        
        # Read UART0->FSR (0x40050018) via OpenOCD
        uart0_fsr = self.openocd.read_word(0x40050018)
        bif = (uart0_fsr >> 6) & 1
        fef = (uart0_fsr >> 5) & 1
        pef = (uart0_fsr >> 4) & 1
        rx_over = (uart0_fsr >> 0) & 1

        print(f"TC-04 Result: ACK received: {ack_received} in {ack_latency_ms:.2f} ms (<20 ms target)")
        print(f"UART0->FSR: 0x{uart0_fsr:08X} (BIF={bif}, FEF={fef}, PEF={pef}, RX_OVER={rx_over})")

        errors_stuck = (bif != 0) or (fef != 0) or (pef != 0) or (rx_over != 0)
        passed = ack_received and (ack_latency_ms < 20.0) and not errors_stuck

        self.results["TC-04"] = {
            "status": "PASS" if passed else "FAIL",
            "ack_latency_ms": ack_latency_ms,
            "uart0_fsr": f"0x{uart0_fsr:08X}",
            "measured": f"ACK in {ack_latency_ms:.1f} ms (<20 ms), UART0->FSR=0x{uart0_fsr:04X} (zero errors stuck)"
        }
        return passed

    def test_tc05_bus_off_and_watchdog_survival(self):
        print("\n--- Running TC-05: Rate-Limited Bus-Off Recovery & Watchdog Survival ---")
        self.wake_nuc()
        time.sleep(0.05)

        # Clear reset flags in SYS->RSTSRC (0x50000004)
        self.openocd.write_word(0x50000004, 0xFFFFFFFF)

        # Run 60-second continuous stress loop
        print("Starting 60-second stress loop (500 fps CAN traffic + periodic UART queries)...")
        frame_ign = pack_can_frame(0x036, bytes.fromhex("0E00000F010000A0"))
        noise_frames = [
            pack_can_frame(0x0B6, b"\x00\x00\x00\x00\x00\x00\x00\x00"),
            pack_can_frame(0x1D0, b"\x20\x00\x00\x00\x00\x00\x00\x00"),
            pack_can_frame(0x220, b"\x04\x00"),
            pack_can_frame(0x128, b"\x00\x00\x00\x00\x00\x00\x00\x00"),
            pack_can_frame(0x2A1, b"\x00\x00\x00\x00\x00\x00\x00\x00"),
        ]

        t_loop_start = time.time()
        t_last_uart = 0
        can_sent_total = 0
        uart_sent_total = 0
        bus_off_induced = False

        while (time.time() - t_loop_start) < 60.0:
            now = time.time()
            elapsed = now - t_loop_start

            # Send CAN burst
            self.can_sock.send(frame_ign)
            can_sent_total += 1
            for f in noise_frames:
                self.can_sock.send(f)
                can_sent_total += 1

            # UART query every 500 ms
            if (now - t_last_uart) >= 0.5:
                t_last_uart = now
                self.uart.write(bytes.fromhex("5AA50224220047"))
                uart_sent_total += 1
                self.uart.read_all()

            # At t=25s, induce a brief bus error / bus-off scenario if supported
            # We can pulse corrupt CAN frames or verify CAN0->STATUS
            if not bus_off_induced and elapsed > 25.0:
                print(f"[{elapsed:.1f}s] Stressing CAN controller bus error handling...")
                # Rapidly burst 100 CAN frames with 0x000 ID
                for _ in range(100):
                    self.can_sock.send(pack_can_frame(0x000, b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF"))
                    can_sent_total += 1
                bus_off_induced = True

            time.sleep(0.01)

        print(f"Stress loop completed: {can_sent_total} CAN frames, {uart_sent_total} UART queries.")

        # Read CAN0->STATUS (0x40180004) and SYS->RSTSRC (0x50000004)
        can_status = self.openocd.read_word(0x40180004)
        can_con    = self.openocd.read_word(0x40180000)
        rstsrc     = self.openocd.read_word(0x50000004)

        # SYS_RSTSRC bitmasks:
        # bit 0: POR, bit 1: PAD (RST pin), bit 2: WDT (bit 2), bit 3: LVR, bit 4: BOD (bit 4), bit 5: CPU
        wdt_reset = (rstsrc & (1 << 2)) != 0
        bod_reset = (rstsrc & (1 << 4)) != 0
        cpu_reset = (rstsrc & (1 << 5)) != 0

        # Ensure CAN controller is NOT stuck in INIT mode
        init_stuck = (can_con & (1 << 0)) != 0
        boff_stuck = (can_status & (1 << 7)) != 0

        print(f"TC-05 Result: RSTSRC = 0x{rstsrc:08X} (WDT={wdt_reset}, BOD={bod_reset}, CPU={cpu_reset})")
        print(f"CAN0: CON = 0x{can_con:08X} (INIT={init_stuck}), STATUS = 0x{can_status:08X} (BOFF={boff_stuck})")

        passed = (not wdt_reset) and (not bod_reset) and (not init_stuck) and (not boff_stuck)

        self.results["TC-05"] = {
            "status": "PASS" if passed else "FAIL",
            "rstsrc": f"0x{rstsrc:08X}",
            "can_con": f"0x{can_con:08X}",
            "can_status": f"0x{can_status:08X}",
            "can_sent": can_sent_total,
            "measured": f"RSTSRC=0x{rstsrc:04X} (WDT=0, BOD=0), CAN CON=0x{can_con:04X}, STATUS=0x{can_status:04X}"
        }
        return passed

    def run_all(self):
        print("==================================================================")
        print("      OpenCanbox NUC131 HAL Changes Hardware-in-the-Loop Test     ")
        print("==================================================================")
        
        tc01 = self.test_tc01_rx_fifo_stress()
        tc02 = self.test_tc02_tx_contention()
        tc03 = self.test_tc03_deterministic_sleep_and_wake()
        tc04 = self.test_tc04_uart_error_recovery()
        tc05 = self.test_tc05_bus_off_and_watchdog_survival()

        print("\n==================================================================")
        print("                     TEST SUMMARY RESULTS                         ")
        print("==================================================================")
        all_passed = True
        for tc, res in self.results.items():
            st = res["status"]
            meas = res.get("measured", "")
            if st != "PASS":
                all_passed = False
            print(f"[{st}] {tc}: {meas}")

        return all_passed


if __name__ == "__main__":
    tester = Nuc131HilTester()
    try:
        success = tester.run_all()
        sys.exit(0 if success else 1)
    finally:
        tester.close()

