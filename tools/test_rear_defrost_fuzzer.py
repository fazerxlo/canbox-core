#!/usr/bin/env python3
import sys
import time
import struct
import socket
import threading
import serial

UART_PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB1"
CAN_INTERFACE = "can0"
UART_BAUD = 38400

CAN_FRAME_FMT = "<IB3x8s"
stop_event = threading.Event()

current_test_info = "Inicjalizacja..."

def send_can_frame(sock: socket.socket, can_id: int, data: bytes):
    dlc = len(data)
    frame = struct.pack(CAN_FRAME_FMT, can_id, dlc, data.ljust(8, b"\x00"))
    try:
        sock.send(frame)
    except OSError:
        pass
    time.sleep(0.002)

def verify_hiworld_cs(frame: bytes) -> bool:
    if len(frame) < 5:
        return False
    s = sum(frame[2:-1]) & 0xFF
    return ((s - 1) & 0xFF) == frame[-1]

def uart_receiver_thread(ser: serial.Serial):
    global current_test_info
    print(f"[UART] Nasłuchiwanie na porcie {ser.name} ({UART_BAUD} 8N1)...")
    buf = bytearray()
    last_payload_31 = None

    while not stop_event.is_set():
        try:
            chunk = ser.read(64)
            if not chunk:
                continue
            buf.extend(chunk)

            while len(buf) >= 5:
                if buf[0] == 0x5A and buf[1] == 0xA5:
                    length = buf[2]
                    total_len = length + 5
                    if len(buf) < total_len:
                        break

                    frame = bytes(buf[:total_len])
                    cmd = frame[3]
                    payload = frame[4:-1]

                    # ACK do Hiworlda
                    ack_cs = (0x01 + 0xFF + cmd - 1) & 0xFF
                    try:
                        ser.write(bytes([0x5A, 0xA5, 0x01, 0xFF, cmd, ack_cs]))
                    except serial.SerialException:
                        pass

                    # Sprawdzamy 0x31 (CarAcState), ale ignorujemy samą zmianę B11 (Temp zewn. z 0x0F6 B5)
                    if cmd == 0x31 and verify_hiworld_cs(frame):
                        if payload != last_payload_31:
                            hex_data = " ".join(f"{b:02X}" for b in payload)
                            
                            diffs = []
                            if last_payload_31 and len(last_payload_31) == len(payload):
                                diffs = [
                                    f"B{i}:0x{last_payload_31[i]:02X}->0x{payload[i]:02X}"
                                    for i in range(len(payload))
                                    if payload[i] != last_payload_31[i]
                                ]
                            diff_str = " | Zmiana UART: " + ", ".join(diffs) if diffs else ""

                            byte_2 = payload[2] if len(payload) > 2 else 0
                            rear_def = "ON" if (byte_2 & 0x20) else "OFF"
                            front_def = "ON" if (byte_2 & 0x10) else "OFF"

                            print(f"\n🎯 [ZMIANA UART 0x31] podczas: {current_test_info}")
                            print(f"   Raw:[{hex_data}] (B2 Rear:{rear_def}, Front:{front_def}){diff_str}")
                            last_payload_31 = payload

                    del buf[:total_len]
                else:
                    del buf[0]
        except serial.SerialException as e:
            print(f"\r[UART BŁĄD] {e}")
            break

def can_fuzzer_thread(sock: socket.socket):
    global current_test_info
    print(f"[CAN] Start nadawania tła BSI/ECU oraz fuzzingu na {CAN_INTERFACE}...")

    payload_036 = bytes([0x0E, 0x00, 0x00, 0x20, 0x01, 0x00, 0x00, 0xA0])
    payload_0b6 = bytes([0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    # Stała baza 0x0F6 (Zapłon + Silnik, Bajt 5 = 0x50 stała temp, żeby nie śmiecić B11)
    base_0f6    = bytes([0x88, 0x00, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00])

    # Rozszerzona lista ID związanych z HVAC / BSI w Peugeot 407 CAN2004
    targets = [
        (0x1E1, 8), # Panel klimatyzacji 407 (Główny)
        (0x0F6, 8), # BSI Status (pomijamy Bajt 5 = temp zewnętrzna)
        (0x1D0, 7), # Klimatyzacja CAN2004 (DLC=7)
        (0x1D0, 8), # Klimatyzacja CAN2004 (na wypadek gdyby Hiworld wymagał DLC=8)
        (0x217, 8), # Statusy BSI / Komfort / Ogrzewanie szyby
        (0x228, 8), # BSI / Panel
        (0x316, 8), # Klimatyzacja / Nawiewy
        (0x336, 8), # Statusy komfortu
        (0x346, 8), # Klimatyzacja (dodatkowa)
        (0x366, 8), # BSI klimatyzacja
        (0x168, 8), # Kontrolki zegarów (CMB)
    ]

    # Testujemy pojedyncze bity ORAZ kombinacje wielobitowe (gdyby bit "AC ON" musiał być ustawiony razem z bitem funkcji)
    test_values = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0xFF]

    while not stop_event.is_set():
        for can_id, dlc in targets:
            for byte_idx in range(dlc):
                # Pomijamy Bajt 5 w 0x0F6, bo już wiemy, że to temperatura zewnętrzna (B11 w 0x31)
                if can_id == 0x0F6 and byte_idx == 5:
                    continue

                for val in test_values:
                    if stop_event.is_set():
                        return

                    current_test_info = f"CAN ID: 0x{can_id:03X} (DLC:{dlc}) | Bajt: {byte_idx} | Wartość: 0x{val:02X}"
                    sys.stdout.write(f"\r🔍 Testuję -> {current_test_info}   ")
                    sys.stdout.flush()

                    # Tworzymy ramkę z aktywnym tłem (w 0x1E1 trzymamy 0x00 lub 0x3C na B0/B1)
                    data = bytearray(dlc)
                    if can_id == 0x1E1:
                        # Z pierwszego logu wiemy, że 0x3C na B0/B1 aktywowało wysyłanie 0x31!
                        data[0] = 0x3C
                        data[1] = 0x3C
                    elif can_id == 0x0F6:
                        data[:] = base_0f6

                    # Jeśli testujemy Bajt 0 lub 1 w 0x1E1, nadpisujemy go bezpośrednio badaną wartością
                    if can_id == 0x1E1 and byte_idx in (0, 1):
                        data[byte_idx] = val
                    else:
                        data[byte_idx] ^= val

                    for _ in range(4):
                        send_can_frame(sock, 0x036, payload_036)
                        send_can_frame(sock, 0x0B6, payload_0b6)
                        if can_id != 0x0F6:
                            send_can_frame(sock, 0x0F6, base_0f6)
                        if can_id != 0x1E1:
                            # Trzymamy wybudzony 0x1E1 w tle podczas testowania innych ID!
                            send_can_frame(sock, 0x1E1, bytes([0x3C, 0x3C, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00]))
                        send_can_frame(sock, can_id, bytes(data))
                        time.sleep(0.035)

                    # Krótki reset badanego bajtu dla wywołania delty
                    reset_data = bytearray(dlc)
                    if can_id == 0x1E1:
                        reset_data[0] = 0x3C
                        reset_data[1] = 0x3C
                    elif can_id == 0x0F6:
                        reset_data[:] = base_0f6
                    send_can_frame(sock, can_id, bytes(reset_data))
                    time.sleep(0.035)

def main():
    try:
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((CAN_INTERFACE,))
    except Exception as e:
        print(f"[BŁĄD] SocketCAN '{CAN_INTERFACE}': {e}")
        return

    try:
        ser = serial.Serial(UART_PORT, UART_BAUD, timeout=0.05)
    except Exception as e:
        print(f"[BŁĄD] Nie można otworzyć UART '{UART_PORT}': {e}")
        sock.close()
        return

    t_uart = threading.Thread(target=uart_receiver_thread, args=(ser,), daemon=True)
    t_can = threading.Thread(target=can_fuzzer_thread, args=(sock,), daemon=True)

    t_uart.start()
    t_can.start()

    print(f"\n=== FUZZER MAPOWANIA 0x31 HIWORLD v3 (Port: {UART_PORT}) ===")
    print("Naciśnij Ctrl+C, aby zakończyć.\n")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[INFO] Zatrzymywanie...")
        stop_event.set()
        ser.close()
        sock.close()

if __name__ == "__main__":
    main()
