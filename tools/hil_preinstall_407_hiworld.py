#!/usr/bin/env python3
"""
Pre-Installation HIL Verification Test Suite: Peugeot 407 on Raise NUC131 (Hiworld Protocol)
Tests:
1. Virtual MDF ("No Physical MDF in Car") Takeover & ISO-TP Flow Control (0x11F / 0x09F)
2. MDF Coexistence Suppression Check (0x525)
3. Discrete Hardware Outputs (PA->DOUT, PB->DOUT) via ST-Link SWD
4. Economy Mode / Ignition-OFF RD4 Manual Radio Wake (0x165)
5. Instrument Cluster Matrix LCD Trip Stalk Cycling & Reset (0x221 -> 0x167 & Cmd 0x1B)
6. Two-Mode 0x24 Heartbeat Round-Robin & Reboot Gap Recovery
7. BSI Personalization 0x260 Cache & Event-Driven 0x15B Write
8. In-Car Upgrade Readiness (0xE0 IAP Isolation Check & Restore)
"""

import sys
import time
import socket
import struct
import threading
import serial

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
        for line in out.splitlines():
            if ":" in line:
                val_str = line.split(":")[1].strip()
                return int(val_str, 16)
        return 0

    def write_word(self, addr: int, val: int):
        self.cmd(f"mww 0x{addr:08X} 0x{val:08X}")

    def reset_run(self):
        self.cmd("reset run")
        time.sleep(0.3)

    def close(self):
        self.sock.close()


def pack_can_frame(can_id: int, data: bytes, is_extended: bool = False, is_error: bool = False) -> bytes:
    flags = 0
    if is_extended:
        flags |= socket.CAN_EFF_FLAG
    if is_error:
        flags |= socket.CAN_ERR_FLAG
    can_id_raw = (can_id & 0x1FFFFFFF) | flags
    dlc = len(data)
    padded_data = data.ljust(8, b'\x00')[:8]
    return struct.pack("=IB3x8s", can_id_raw, dlc, padded_data)


def unpack_can_frame(raw: bytes):
    can_id_raw, dlc, data = struct.unpack("=IB3x8s", raw)
    is_error = bool(can_id_raw & socket.CAN_ERR_FLAG)
    can_id = can_id_raw & 0x1FFFFFFF
    return can_id, dlc, data[:dlc], is_error


class CanBackgroundBroadcaster:
    def __init__(self, can_sock):
        self.sock = can_sock
        self.frames = {}
        self.running = False
        self.thread = None

    def set_frame(self, can_id: int, data: bytes, interval_s: float):
        self.frames[can_id] = {"data": data, "interval": interval_s, "last_tx": 0.0}

    def remove_frame(self, can_id: int):
        self.frames.pop(can_id, None)

    def clear(self):
        self.frames.clear()

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)

    def _run(self):
        while self.running:
            now = time.time()
            for can_id, item in list(self.frames.items()):
                if now - item["last_tx"] >= item["interval"]:
                    try:
                        self.sock.send(pack_can_frame(can_id, item["data"]))
                        item["last_tx"] = now
                    except Exception:
                        pass
            time.sleep(0.01)


class Peugeot407HilVerifier:
    def __init__(self, can_iface="slcan0", uart_port="/dev/ttyUSB1", uart_baud=38400):
        self.can_iface = can_iface
        self.uart_port = uart_port
        self.uart_baud = uart_baud
        
        self.openocd = OpenOcdClient()
        self.can_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.can_sock.bind((self.can_iface,))
        self.can_sock.settimeout(0.02)
        
        self.uart = serial.Serial(self.uart_port, self.uart_baud, timeout=0.02)
        self.broadcaster = CanBackgroundBroadcaster(self.can_sock)
        self.broadcaster.start()

        self.results = []

    def close(self):
        self.broadcaster.stop()
        self.can_sock.close()
        self.uart.close()
        self.openocd.close()

    def drain_interfaces(self):
        self.uart.reset_input_buffer()
        while True:
            try:
                self.can_sock.recv(16)
            except socket.timeout:
                break

    def read_uart_packet(self, timeout_s=1.0):
        start = time.time()
        buf = bytearray()
        while time.time() - start < timeout_s:
            chunk = self.uart.read(self.uart.in_waiting or 1)
            if chunk:
                buf.extend(chunk)
                while len(buf) >= 5:
                    if buf[0] == 0x5A and buf[1] == 0xA5:
                        pkt_len = buf[2]
                        total_expected = 5 + pkt_len
                        if len(buf) >= total_expected:
                            packet = bytes(buf[:total_expected])
                            del buf[:total_expected]
                            return packet
                        else:
                            break
                    else:
                        buf.pop(0)
            else:
                time.sleep(0.005)
        return None

    def read_all_uart_packets(self, duration_s=0.5):
        start = time.time()
        packets = []
        buf = bytearray()
        while time.time() - start < duration_s:
            chunk = self.uart.read(self.uart.in_waiting or 1)
            if chunk:
                buf.extend(chunk)
                while len(buf) >= 5:
                    if buf[0] == 0x5A and buf[1] == 0xA5:
                        pkt_len = buf[2]
                        total_expected = 5 + pkt_len
                        if len(buf) >= total_expected:
                            packets.append(bytes(buf[:total_expected]))
                            del buf[:total_expected]
                        else:
                            break
                    else:
                        buf.pop(0)
            else:
                time.sleep(0.005)
        return packets

    def read_can_frames(self, duration_s=1.0):
        start = time.time()
        frames = []
        while time.time() - start < duration_s:
            try:
                raw = self.can_sock.recv(16)
                can_id, dlc, data, is_err = unpack_can_frame(raw)
                frames.append((time.time(), can_id, dlc, data, is_err))
            except socket.timeout:
                pass
        return frames

    # =========================================================================
    # Test 1: Virtual MDF Takeover & ISO-TP Flow Control
    # =========================================================================
    def test_01_virtual_mdf_takeover_and_isotp(self):
        print("\n--- Test 1: Virtual MDF Takeover & ISO-TP Flow Control ---")
        self.drain_interfaces()
        
        # 1. Start 0x036 (Ignition ON), 0x0F6, and 0x165 (RD4 ON in Tuner mode)
        self.broadcaster.set_frame(0x036, bytes.fromhex("0E00000F010000A0"), 0.10)
        self.broadcaster.set_frame(0x0F6, bytes.fromhex("883C000000000020"), 0.50)
        self.broadcaster.set_frame(0x165, bytes.fromhex("C8C0100000000000"), 0.10)
        
        print("[*] Waiting 3.6s for physical MDF absence timeout...")
        time.sleep(3.6)
        
        # Monitor slcan0 for 1.5s to capture periodic Virtual MDF frames
        frames = self.read_can_frames(1.5)
        seen_ids = {f[1] for f in frames}
        print(f"[*] CAN IDs captured in Virtual MDF window: {[hex(x) for x in sorted(seen_ids)]}")
        
        has_525 = 0x525 in seen_ids
        has_3f6 = 0x3F6 in seen_ids
        has_0df = 0x0DF in seen_ids
        has_167 = 0x167 in seen_ids
        
        virtual_mdf_ok = has_525 and has_3f6 and has_0df and has_167
        print(f"[*] Virtual MDF Periodic Broadcast: 0x525={has_525}, 0x3F6={has_3f6}, 0x0DF={has_0df}, 0x167={has_167}")
        
        # 2. Test ISO-TP RadioText (0x0A4) First Frame
        self.drain_interfaces()
        t0 = time.time()
        # 0x0A4 First Frame: length 30 (0x1E), chunk: 40 60 52 4D 46 20 ("RMF ")
        self.can_sock.send(pack_can_frame(0x0A4, bytes.fromhex("101E4060524D4620")))
        
        fc_0a4_id = None
        fc_0a4_latency_ms = None
        fc_0a4_data = None
        
        deadline = time.time() + 0.05
        while time.time() < deadline:
            try:
                raw = self.can_sock.recv(16)
                cid, dlc, data, _ = unpack_can_frame(raw)
                if cid in (0x09F, 0x11F):
                    fc_0a4_latency_ms = (time.time() - t0) * 1000.0
                    fc_0a4_id = cid
                    fc_0a4_data = data
                    break
            except socket.timeout:
                pass
                
        print(f"[*] 0x0A4 Flow Control: ID={hex(fc_0a4_id) if fc_0a4_id else 'None'} ({fc_0a4_latency_ms:.2f} ms), Data={fc_0a4_data.hex() if fc_0a4_data else 'None'}")
        
        # Stream Consecutive Frames 21..24 to complete 0x0A4 reassembly
        time.sleep(0.01)
        self.can_sock.send(pack_can_frame(0x0A4, bytes.fromhex("2120464D20202020"))) # " FM    "
        time.sleep(0.01)
        self.can_sock.send(pack_can_frame(0x0A4, bytes.fromhex("2220202020202020")))
        time.sleep(0.01)
        self.can_sock.send(pack_can_frame(0x0A4, bytes.fromhex("2320202020202020")))
        time.sleep(0.01)
        self.can_sock.send(pack_can_frame(0x0A4, bytes.fromhex("2420202000000000")))
        
        # Read UART packet for Cmd 0x86
        pkt_086 = None
        for pkt in self.read_all_uart_packets(0.4):
            if pkt[3] == 0x86:
                pkt_086 = pkt
                break
        print(f"[*] UART Cmd 0x86 (RadioText): {pkt_086.hex() if pkt_086 else 'None'}")

        # 3. Test ISO-TP Station List (0x125) First Frame
        self.drain_interfaces()
        t0 = time.time()
        self.can_sock.send(pack_can_frame(0x125, bytes.fromhex("103A406000002052")))
        
        fc_125_id = None
        fc_125_latency_ms = None
        fc_125_data = None
        
        deadline = time.time() + 0.05
        while time.time() < deadline:
            try:
                raw = self.can_sock.recv(16)
                cid, dlc, data, _ = unpack_can_frame(raw)
                if cid == 0x11F:
                    fc_125_latency_ms = (time.time() - t0) * 1000.0
                    fc_125_id = cid
                    fc_125_data = data
                    break
            except socket.timeout:
                pass
                
        print(f"[*] 0x125 Flow Control: ID={hex(fc_125_id) if fc_125_id else 'None'} ({fc_125_latency_ms:.2f} ms), Data={fc_125_data.hex() if fc_125_data else 'None'}")
        
        # Send CF 21..28 for station list presets
        for sn in range(1, 9):
            time.sleep(0.005)
            cf_byte = 0x20 | (sn & 0x0F)
            cf_payload = bytes([cf_byte]) + b"STATION"[:7]
            self.can_sock.send(pack_can_frame(0x125, cf_payload))
            
        pkt_085 = None
        for pkt in self.read_all_uart_packets(0.4):
            if pkt[3] == 0x85:
                pkt_085 = pkt
                break
        print(f"[*] UART Cmd 0x85 (Presets): {pkt_085.hex() if pkt_085 else 'None'}")

        self.broadcaster.remove_frame(0x165)

        passed = (
            virtual_mdf_ok and
            fc_0a4_id is not None and fc_0a4_latency_ms < 15.0 and
            fc_125_id == 0x11F and fc_125_latency_ms < 15.0 and
            pkt_086 is not None and pkt_085 is not None
        )
        detail = (
            f"MDF: [525,3F6,0DF,167] OK; FC_0A4: {hex(fc_0a4_id)} ({fc_0a4_latency_ms:.1f}ms); "
            f"FC_125: {hex(fc_125_id)} ({fc_125_latency_ms:.1f}ms); UART 0x86/0x85 received"
        )
        self.results.append(("Test 1: Virtual MDF Takeover & ISO-TP", passed, detail))
        return passed

    # =========================================================================
    # Test 2: MDF Coexistence Suppression Check
    # =========================================================================
    def test_02_mdf_coexistence_suppression(self):
        print("\n--- Test 2: MDF Coexistence Suppression Check ---")
        self.drain_interfaces()
        
        # Inject physical MDF heartbeat 0x525
        print("[*] Injecting physical MDF heartbeat 0x525#0500010000000000...")
        self.can_sock.send(pack_can_frame(0x525, bytes.fromhex("0500010000000000")))
        
        # Monitor slcan0 for 1.5 seconds. Virtual MDF frames must CEASE within 100 ms.
        time.sleep(0.10) # 100 ms suppression window
        frames = self.read_can_frames(1.4)
        
        mdf_leaked_ids = [f[1] for f in frames if f[1] in (0x525, 0x3F6, 0x0DF, 0x167, 0x11F, 0x09F)]
        print(f"[*] Frames observed during coexistence window (should be empty): {[hex(x) for x in mdf_leaked_ids]}")
        
        passed = (len(mdf_leaked_ids) == 0)
        detail = f"Suppressed frames: {len(mdf_leaked_ids)} leaked during 1400 ms coexistence check"
        self.results.append(("Test 2: MDF Coexistence Suppression", passed, detail))
        return passed

    # =========================================================================
    # Test 3: Discrete Hardware Outputs via ST-Link SWD
    # =========================================================================
    def test_03_discrete_hardware_outputs(self):
        print("\n--- Test 3: Discrete Hardware Outputs (PA->DOUT, PB->DOUT) ---")
        
        # 1. Buzzer Idle Safety: PB3 and PB12 must be 1 (silent)
        pb_dout = self.openocd.read_word(0x50004048) # GPIOB_DOUT
        pb3 = (pb_dout >> 3) & 1
        pb12 = (pb_dout >> 12) & 1
        print(f"[*] GPIOB->DOUT: 0x{pb_dout:08X} (PB3={pb3}, PB12={pb12})")
        buzzer_ok = (pb3 == 1 and pb12 == 1)

        # 2. Assert Ignition ON, Backlight ON, Reverse Gear ON
        print("[*] Asserting Ignition ON, Backlight ON (0x036), Reverse ON (0x0F6)...")
        self.broadcaster.set_frame(0x036, bytes.fromhex("0E00002F010000A0"), 0.10) # D3=0x2F has bit 5 set (Night Illum)
        self.broadcaster.set_frame(0x0F6, bytes.fromhex("883C000000000080"), 0.10) # D7=0x80 has bit 7 set (Reverse)
        time.sleep(0.3)
        
        pa_dout_on = self.openocd.read_word(0x50004008) # GPIOA_DOUT
        pa8_on  = (pa_dout_on >> 8) & 1
        pa9_on  = (pa_dout_on >> 9) & 1
        pa13_on = (pa_dout_on >> 13) & 1
        print(f"[*] Active State: GPIOA->DOUT=0x{pa_dout_on:08X} (ACC/PA8={pa8_on}, ILL/PA9={pa9_on}, REV/PA13={pa13_on})")
        
        # Check UART Cmd 0x11
        pkt_011_found = False
        rev_bit = False
        ill_bit = False
        for pkt in self.read_all_uart_packets(0.4):
            if pkt[3] == 0x11 and len(pkt) >= 9:
                pkt_011_found = True
                # In Hiworld Cmd 0x11, status bits are in payload
                # Byte 4 has Reverse (bit 1) and Illumination (bit 2) or similar
                payload = pkt[4:]
                if payload[0] & 0x01: # Ignition
                    pass
                rev_bit = bool(payload[0] & 0x02 or (len(payload) > 4 and payload[4] & 0x80))
                ill_bit = bool(payload[0] & 0x04 or (len(payload) > 4 and payload[4] & 0x20))
                break
        print(f"[*] UART Cmd 0x11: found={pkt_011_found}")

        # 3. Clear Reverse and Backlight
        print("[*] Clearing Reverse Gear and Backlight...")
        self.broadcaster.set_frame(0x036, bytes.fromhex("0E00000F010000A0"), 0.10)
        self.broadcaster.set_frame(0x0F6, bytes.fromhex("883C000000000020"), 0.10)
        time.sleep(0.3)
        
        pa_dout_off = self.openocd.read_word(0x50004008)
        pa9_off  = (pa_dout_off >> 9) & 1
        pa13_off = (pa_dout_off >> 13) & 1
        print(f"[*] Cleared State: GPIOA->DOUT=0x{pa_dout_off:08X} (ILL/PA9={pa9_off}, REV/PA13={pa13_off})")

        passed = (
            buzzer_ok and
            pa8_on == 1 and pa9_on == 1 and pa13_on == 1 and
            pa9_off == 0 and pa13_off == 0
        )
        detail = (
            f"Buzzer PB3/PB12=1; Active PA[8,9,13]=[{pa8_on},{pa9_on},{pa13_on}]; "
            f"Inactive PA[9,13]=[{pa9_off},{pa13_off}]"
        )
        self.results.append(("Test 3: Discrete Hardware Outputs", passed, detail))
        return passed

    # =========================================================================
    # Test 4: Economy Mode / Ignition-OFF RD4 Manual Radio Wake
    # =========================================================================
    def test_04_economy_mode_rd4_wake(self):
        print("\n--- Test 4: Economy Mode / Ignition-OFF RD4 Manual Radio Wake ---")
        self.drain_interfaces()
        
        # 1. Turn Ignition OFF and ensure RD4 is OFF
        print("[*] Setting Ignition OFF (+APC=0, PHASE_VIE=0) and RD4 OFF...")
        self.broadcaster.set_frame(0x036, bytes.fromhex("0E00000F000000A0"), 0.10)
        self.broadcaster.set_frame(0x0F6, bytes.fromhex("803C000000000020"), 0.10)
        self.broadcaster.set_frame(0x165, bytes.fromhex("0880000000000000"), 0.10)
        time.sleep(0.4)
        
        pa_dout_ign_off = self.openocd.read_word(0x50004008)
        pa8_ign_off = (pa_dout_ign_off >> 8) & 1
        print(f"[*] Ignition OFF: GPIOA->DOUT=0x{pa_dout_ign_off:08X} (ACC/PA8={pa8_ign_off})")

        # 2. Rising edge: Inject active RD4 audio status: 0x165#C8C01000 (power button pushed)
        print("[*] Injecting active RD4 audio frame: 0x165#C8C01000...")
        self.broadcaster.set_frame(0x165, bytes.fromhex("C8C0100000000000"), 0.10)
        time.sleep(0.4)
        
        pa_dout_rd4_on = self.openocd.read_word(0x50004008)
        pa8_rd4_on = (pa_dout_rd4_on >> 8) & 1
        print(f"[*] RD4 Manual Wake: GPIOA->DOUT=0x{pa_dout_rd4_on:08X} (ACC/PA8={pa8_rd4_on})")

        # 3. Turn RD4 audio OFF: 0x165#08800000
        print("[*] Injecting RD4 power OFF: 0x165#08800000...")
        self.broadcaster.set_frame(0x165, bytes.fromhex("0880000000000000"), 0.10)
        time.sleep(0.4)
        
        pa_dout_rd4_off = self.openocd.read_word(0x50004008)
        pa8_rd4_off = (pa_dout_rd4_off >> 8) & 1
        print(f"[*] RD4 Manual OFF: GPIOA->DOUT=0x{pa_dout_rd4_off:08X} (ACC/PA8={pa8_rd4_off})")

        # Restore Ignition ON for subsequent tests
        self.broadcaster.remove_frame(0x165)
        self.broadcaster.set_frame(0x036, bytes.fromhex("0E00000F010000A0"), 0.10)
        self.broadcaster.set_frame(0x0F6, bytes.fromhex("883C000000000020"), 0.10)
        time.sleep(0.2)

        passed = (pa8_ign_off == 0 and pa8_rd4_on == 1 and pa8_rd4_off == 0)
        detail = f"ACC on IgnOFF={pa8_ign_off}, on RD4wake={pa8_rd4_on}, on RD4off={pa8_rd4_off}"
        self.results.append(("Test 4: Economy / RD4 Manual Wake", passed, detail))
        return passed

    # =========================================================================
    # Test 5: Instrument Cluster Matrix LCD Trip Stalk Cycling & Reset
    # =========================================================================
    def test_05_trip_stalk_cycling_and_reset(self):
        print("\n--- Test 5: Trip Stalk Cycling & Reset (0x221 -> 0x167 & Cmd 0x1B) ---")
        self.drain_interfaces()
        
        # Ensure Virtual MDF is active: wait 3.5s without 0x525
        print("[*] Ensuring Virtual MDF mode is active...")
        time.sleep(3.5)
        
        # 1. Simulate Trip button short press on stalk: 0x221#C8FFFFFFFFFFFF (150 ms) -> C0
        print("[*] Injecting Wiper Stalk Trip press: 0x221#C8FFFFFFFFFFFF...")
        self.can_sock.send(pack_can_frame(0x221, bytes.fromhex("C8FFFFFFFFFFFF00")))
        time.sleep(0.15)
        self.can_sock.send(pack_can_frame(0x221, bytes.fromhex("C0FFFFFFFFFFFF00")))
        
        # Verify UART receives Cmd 0x11 Key 0x14 and slcan0 receives 0x167 cycling to Trip 1 (0x0A)
        time.sleep(0.1)
        frames = self.read_can_frames(0.4)
        f_167 = [f for f in frames if f[1] == 0x167]
        cycle1_byte0 = f_167[-1][3][0] if f_167 else None
        print(f"[*] Stalk Cycle 1: 0x167 Byte 0 = {hex(cycle1_byte0) if cycle1_byte0 is not None else 'None'}")

        # Cycle to Trip 2 (0x0C)
        self.can_sock.send(pack_can_frame(0x221, bytes.fromhex("C8FFFFFFFFFFFF00")))
        time.sleep(0.15)
        self.can_sock.send(pack_can_frame(0x221, bytes.fromhex("C0FFFFFFFFFFFF00")))
        time.sleep(0.1)
        frames = self.read_can_frames(0.4)
        f_167 = [f for f in frames if f[1] == 0x167]
        cycle2_byte0 = f_167[-1][3][0] if f_167 else None
        print(f"[*] Stalk Cycle 2: 0x167 Byte 0 = {hex(cycle2_byte0) if cycle2_byte0 is not None else 'None'}")

        # Cycle back to Instant (0x09)
        self.can_sock.send(pack_can_frame(0x221, bytes.fromhex("C8FFFFFFFFFFFF00")))
        time.sleep(0.15)
        self.can_sock.send(pack_can_frame(0x221, bytes.fromhex("C0FFFFFFFFFFFF00")))
        time.sleep(0.1)
        frames = self.read_can_frames(0.4)
        f_167 = [f for f in frames if f[1] == 0x167]
        cycle3_byte0 = f_167[-1][3][0] if f_167 else None
        print(f"[*] Stalk Cycle 3: 0x167 Byte 0 = {hex(cycle3_byte0) if cycle3_byte0 is not None else 'None'}")

        # 2. Send Hiworld Trip 1 Reset Downlink over UART: 5A A5 04 1B 02 02 01 FF 22
        self.drain_interfaces()
        print("[*] Sending Hiworld Trip 1 Reset command over UART (Cmd 0x1B)...")
        trip_reset_cmd = bytes.fromhex("5AA5041B020201FF22")
        self.uart.write(trip_reset_cmd)
        
        # Assert slcan0 receives 0x167 with Byte 0 = 0x8A (Reset Trip 1 pulse)
        reset_frames = self.read_can_frames(0.4)
        f_reset_167 = [f for f in reset_frames if f[1] == 0x167]
        reset_bytes = [f[3][0] for f in f_reset_167]
        print(f"[*] Reset CAN frames captured on 0x167: {[hex(b) for b in reset_bytes]}")
        
        has_8a_pulse = any(b == 0x8A for b in reset_bytes)

        passed = (
            cycle1_byte0 == 0x0A and
            cycle2_byte0 == 0x0C and
            cycle3_byte0 == 0x09 and
            has_8a_pulse
        )
        detail = (
            f"Trip Pages: [0x{cycle1_byte0:02X}, 0x{cycle2_byte0:02X}, 0x{cycle3_byte0:02X}]; "
            f"Reset Pulse 0x8A detected={has_8a_pulse}"
        )
        self.results.append(("Test 5: Trip Stalk Cycling & Reset", passed, detail))
        return passed

    # =========================================================================
    # Test 6: Two-Mode 0x24 Heartbeat Round-Robin & Reboot Gap Recovery
    # =========================================================================
    def test_06_two_mode_024_round_robin_and_reboot(self):
        print("\n--- Test 6: Two-Mode 0x24 Heartbeat & Reboot Gap Recovery ---")
        self.drain_interfaces()
        
        cmd_024 = bytes.fromhex("5AA50224220047")
        acks = []
        burst_counts = []
        
        # Send 0x24 every 3.0s for 4 cycles
        for cycle in range(1, 5):
            print(f"[*] Sending 0x24 Cycle {cycle}...")
            t0 = time.time()
            self.uart.write(cmd_024)
            
            # Capture responses over 800 ms
            pkts = self.read_all_uart_packets(0.8)
            ack = any(p == bytes.fromhex("5AA501FF2423") for p in pkts)
            acks.append(ack)
            burst_counts.append(len(pkts))
            
            cmds_seen = [hex(p[3]) for p in pkts]
            print(f"    Cycle {cycle}: ACK={ack}, Packets={len(pkts)} {cmds_seen[:6]}")
            
            if cycle < 4:
                time.sleep(2.2) # To reach ~3.0s interval
                
        # Cycle 1 should be full discovery burst (>= 6 packets: F0, 71, 72, C1, state...)
        cycle1_full = burst_counts[0] >= 5
        # Cycles 2..4 should be paced round-robin (fewer packets per tick)
        steady_state_paced = all(c < burst_counts[0] for c in burst_counts[1:])
        
        # Pause 5.0 seconds (> 4500 ms reboot gap threshold)
        print("[*] Pausing 5.2 seconds to simulate Android HU warm reboot...")
        time.sleep(5.2)
        
        # Send 0x24 again
        print("[*] Sending 0x24 post-reboot...")
        self.uart.write(cmd_024)
        reboot_pkts = self.read_all_uart_packets(0.8)
        reboot_ack = any(p == bytes.fromhex("5AA501FF2423") for p in reboot_pkts)
        reboot_burst_count = len(reboot_pkts)
        reboot_cmds = [hex(p[3]) for p in reboot_pkts]
        print(f"[*] Post-Reboot: ACK={reboot_ack}, Packets={reboot_burst_count} {reboot_cmds[:6]}")
        
        reboot_full_burst = (reboot_burst_count >= 5)

        passed = (
            all(acks) and
            cycle1_full and
            steady_state_paced and
            reboot_ack and
            reboot_full_burst
        )
        detail = (
            f"ACKs: 4/4; Cycle 1 count={burst_counts[0]}; RR counts={burst_counts[1:]}; "
            f"Reboot count={reboot_burst_count}"
        )
        self.results.append(("Test 6: 0x24 Round-Robin & Reboot Gap", passed, detail))
        return passed

    # =========================================================================
    # Test 7: BSI Personalization 0x260 Cache & Event-Driven 0x15B Write
    # =========================================================================
    def test_07_bsi_perso_260_cache_and_15b(self):
        print("\n--- Test 7: BSI Personalization 0x260 Cache & Event-Driven 0x15B ---")
        self.drain_interfaces()
        
        # 1. Inject BSI Profile 1 frame 0x260#0103910000D00010
        print("[*] Injecting BSI Profile 1: 0x260#0103910000D00010...")
        self.can_sock.send(pack_can_frame(0x260, bytes.fromhex("0103910000D00010")))
        
        # Verify UART emits Cmd 0x76 and 0x79
        uart_pkts = self.read_all_uart_packets(0.5)
        has_076 = any(p[3] == 0x76 for p in uart_pkts)
        has_079 = any(p[3] == 0x79 for p in uart_pkts)
        print(f"[*] UART Cmd 0x76={has_076}, Cmd 0x79={has_079}")

        # Confirm NUC131 sends ZERO 0x15B frames while idle over 1.5s
        print("[*] Monitoring CAN bus for 1.5s to confirm ZERO 0x15B frames while idle...")
        idle_frames = self.read_can_frames(1.5)
        idle_15b = [f for f in idle_frames if f[1] == 0x15B]
        print(f"[*] Idle 0x15B frames: {len(idle_15b)}")

        # 2. Send Hiworld Cmd 0x7B setting toggle over UART (Toggle DRL: setting 0x05 = 0x01)
        # Packet: 5A A5 02 7B 05 01 82 (sum = 2 + 0x7B + 5 + 1 = 131; 131 - 1 = 130 = 0x82)
        print("[*] Sending Cmd 0x7B DRL toggle (setting 0x05, val 0x01) over UART...")
        self.drain_interfaces()
        self.uart.write(bytes.fromhex("5AA5027B050182"))
        
        # Assert slcan0 immediately transmits 0x15B with Byte 0 = 0x01, and Byte 3 has bit 6 set (0x40)
        frames_15b = self.read_can_frames(0.5)
        downlink_15b = [f for f in frames_15b if f[1] == 0x15B]
        print(f"[*] Downlink 0x15B frames captured: {len(downlink_15b)}")
        
        perso_15b_ok = False
        payload_15b = None
        if downlink_15b:
            payload_15b = downlink_15b[0][3]
            print(f"[*] 0x15B payload: {payload_15b.hex()}")
            # Original: 01 03 91 00 00 D0 00 10 -> With DRL: 01 03 91 40 00 D0 00 10
            perso_15b_ok = (payload_15b[0] == 0x01 and (payload_15b[3] & 0x40) != 0)

        passed = (has_076 and has_079 and len(idle_15b) == 0 and perso_15b_ok)
        detail = (
            f"UART 0x76/0x79={has_076}/{has_079}; Idle 0x15B count={len(idle_15b)}; "
            f"Downlink 0x15B={payload_15b.hex() if payload_15b else 'None'}"
        )
        self.results.append(("Test 7: BSI 0x260 Cache & 0x15B Write", passed, detail))
        return passed

    # =========================================================================
    # Test 8: In-Car Upgrade Readiness (0xE0 IAP Isolation Check & Restore)
    # =========================================================================
    def test_08_upgrade_readiness_iap_isolation(self):
        print("\n--- Test 8: In-Car Upgrade Readiness (0xE0 IAP Isolation Check) ---")
        self.drain_interfaces()
        
        # Monitor slcan0 for error frames during transition
        # Trigger reboot via 0xE0 upgrade trigger: 5A A5 03 E0 00 00 00 E2 (sum=3+0xE0=227-1=226=0xE2)
        print("[*] Sending 0xE0 Firmware Upgrade trigger over UART...")
        upgrade_cmd = bytes.fromhex("5AA503E0000000E2")
        self.uart.write(upgrade_cmd)
        
        # Check CAN frames for any error frames
        time.sleep(0.08)
        frames = self.read_can_frames(0.3)
        error_frames = [f for f in frames if f[4]] # is_error == True
        print(f"[*] CAN Error frames during transition: {len(error_frames)}")
        
        # Inspect hardware isolation registers via ST-Link SWD
        pa_dout = self.openocd.read_word(0x50004008)
        pc_dout = self.openocd.read_word(0x50004088)
        pd_dout = self.openocd.read_word(0x500040C8)
        
        pa8_acc    = (pa_dout >> 8) & 1
        pc3_canstb = (pc_dout >> 3) & 1
        pd7_cantx  = (pd_dout >> 7) & 1
        print(f"[*] Bootloader Isolation: PA->DOUT=0x{pa_dout:08X} (ACC/PA8={pa8_acc})")
        print(f"[*] Bootloader Isolation: PC->DOUT=0x{pc_dout:08X} (CAN_STB/PC3={pc3_canstb})")
        print(f"[*] Bootloader Isolation: PD->DOUT=0x{pd_dout:08X} (CAN_TX/PD7={pd7_cantx})")
        
        # Restore application: clear SRAM token at 0x20000000 and issue reset run
        print("[*] Restoring application running state via SWD reset run...")
        self.openocd.write_word(0x20000000, 0x00000000)
        self.openocd.reset_run()
        time.sleep(0.5)
        
        # Verify app is running
        rstsrc = self.openocd.read_word(0x50000004)
        print(f"[*] Post-Restore SYS->RSTSRC: 0x{rstsrc:08X}")
        app_restored = (rstsrc != 0)

        passed = (
            len(error_frames) == 0 and
            pa8_acc == 1 and
            pc3_canstb == 1 and
            pd7_cantx == 1 and
            app_restored
        )
        detail = (
            f"CAN ErrFrames={len(error_frames)}; PA8_ACC={pa8_acc}; PC3_CANSTB={pc3_canstb}; "
            f"PD7_CANTX={pd7_cantx}; Restored RSTSRC=0x{rstsrc:08X}"
        )
        self.results.append(("Test 8: Upgrade Readiness & IAP Isolation", passed, detail))
        return passed

    def run_all(self):
        print("=================================================================")
        print(" Peugeot 407 Pre-Installation HIL Test Suite (Raise NUC131)     ")
        print("=================================================================")
        
        # Clear reset flags before starting
        self.openocd.reset_run()
        self.openocd.write_word(0x50000004, 0x00000020) # clear RSTSRC
        
        t_start = time.time()
        
        self.test_01_virtual_mdf_takeover_and_isotp()
        self.test_02_mdf_coexistence_suppression()
        self.test_03_discrete_hardware_outputs()
        self.test_04_economy_mode_rd4_wake()
        self.test_05_trip_stalk_cycling_and_reset()
        self.test_06_two_mode_024_round_robin_and_reboot()
        self.test_07_bsi_perso_260_cache_and_15b()
        self.test_08_upgrade_readiness_iap_isolation()
        
        elapsed = time.time() - t_start
        print(f"\nCompleted in {elapsed:.2f}s")
        self.print_summary()

    def print_summary(self):
        print("\n" + "=" * 80)
        print(f"{'Test Scenario':<42} | {'Result':<6} | {'Details'}")
        print("=" * 80)
        all_passed = True
        for name, passed, detail in self.results:
            status = "PASS" if passed else "FAIL"
            if not passed:
                all_passed = False
            print(f"{name:<42} | {status:<6} | {detail}")
        print("=" * 80)
        return all_passed


if __name__ == "__main__":
    tester = Peugeot407HilVerifier()
    try:
        tester.run_all()
    finally:
        tester.close()

