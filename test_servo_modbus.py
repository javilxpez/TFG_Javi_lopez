#!/usr/bin/env python3
"""
Test directo servo Modbus RTU via USB-RS485
Prueba baud rates 9600/19200/38400/115200 e IDs 1-10
"""
import sys
import time
import struct
import serial

PORT  = "/dev/ttyUSB0"
IDS   = list(range(1, 11))
BAUDS = [9600, 19200, 38400, 115200]

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def build_read(slave_id: int, reg: int, count: int = 1) -> bytes:
    frame = struct.pack(">BBHH", slave_id, 0x03, reg, count)
    crc   = crc16(frame)
    return frame + struct.pack("<H", crc)

def try_read(ser: serial.Serial, slave_id: int, reg: int) -> bytes | None:
    ser.reset_input_buffer()
    pkt = build_read(slave_id, reg)
    ser.write(pkt)
    ser.flush()
    time.sleep(0.1)
    resp = ser.read(ser.in_waiting or 1)
    if not resp:
        return None
    # leer hasta timeout
    deadline = time.monotonic() + 0.15
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            resp += chunk
    return resp if resp else None

print(f"Puerto: {PORT}")
print("=" * 55)

found = False
for baud in BAUDS:
    print(f"\n── {baud} baud ─────────────────────────────")
    try:
        ser = serial.Serial(PORT, baud, timeout=0.15)
    except serial.SerialException as e:
        print(f"  ERROR abriendo puerto: {e}")
        sys.exit(1)
    time.sleep(0.1)

    for sid in IDS:
        resp = try_read(ser, sid, 0)  # registro 0 = control mode
        if resp:
            crc_ok = len(resp) >= 5 and crc16(resp[:-2]) == struct.unpack("<H", resp[-2:])[0]
            hex_str = " ".join(f"{b:02X}" for b in resp)
            status  = "✓ CRC OK  ← SERVO ENCONTRADO!" if crc_ok else "✗ CRC MAL (ruido?)"
            print(f"  ID={sid:3d}  RX {len(resp)} bytes: {hex_str}  {status}")
            if crc_ok:
                found = True
        else:
            print(f"  ID={sid:3d}  sin respuesta")

    ser.close()

print("\n" + "=" * 55)
if not found:
    print("NINGUNA RESPUESTA en ningún baud rate ni ID.")
    print()
    print("Posibles causas:")
    print("  1. Servo sin alimentación (verificar LEDs del variador)")
    print("  2. Cables A/B invertidos entre adaptador y servo")
    print("  3. GND no compartido entre adaptador y servo")
    print("  4. El servo no tiene habilitada la comunicación Modbus")
else:
    print("Servo encontrado. Ver resultado ✓ arriba.")
