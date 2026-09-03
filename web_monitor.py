#!/usr/bin/env python3
"""
Web Monitor — Pico 2 + ZSC31014 + Servo
Dashboard en tiempo real vía WebSocket.

Instalar:  pip install fastapi "uvicorn[standard]" pyserial
Ejecutar:  python web_monitor.py [puerto]
Abrir:     http://localhost:8080
"""

import asyncio
import glob
import json
import logging
import os
import struct
import sys
import threading
import time
from collections import deque
from contextlib import asynccontextmanager
from pathlib import Path

import serial
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse

# ── Logging (console + file) ──────────────────────────────
LOG_FILE = str(Path(__file__).parent / "web_monitor.log")
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d %(message)s",
    datefmt="%H:%M:%S",
    handlers=[logging.FileHandler(LOG_FILE), logging.StreamHandler(sys.stdout)],
)
log = logging.getLogger("monitor")

BAUD = 115200
SYNC = bytes([0xAA, 0x55])

# Batched frame: header (14 B) + N × sample (12 B)
HEADER_FMT  = "<BIHBBBBBBB"   # id, base_t, bridge, lc_status, flags, error, servo_id, rpm_st, trq_st, n
HEADER_SIZE = struct.calcsize(HEADER_FMT)   # 14
SAMPLE_FMT  = "<HhhhhBB"      # dt_ms, rpm, current_x10, ref_cmd, base_read, io, state
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)   # 12

CMD_INIT = 0x01; CMD_STOP = 0x02; CMD_SET_PARAM = 0x03
CMD_MOVE_A = 0x04; CMD_MOVE_B = 0x05; CMD_SHUTDOWN = 0x06

PARAM_SPEED = 0x01; PARAM_FORCE_LIMIT = 0x02; PARAM_ZERO_OFFSET = 0x03
PARAM_ACCEL = 0x04; PARAM_DECEL = 0x05; PARAM_STIFFNESS = 0x06; PARAM_FORCE_CALIB = 0x07

STATE_NAMES = {0:"IDLE",1:"SCANNING",2:"CONFIGURING",3:"STOPPED",4:"MOVING_A",5:"MOVING_B",6:"ERROR"}
MODE_NAMES  = {0:"PARADO",1:"MOV_A",2:"MOV_B"}
ERROR_NAMES = {0:"—",1:"Célula de carga no encontrada",2:"Fallo escaneo servo (RS485?)",
               3:"Fallo configuración servo",4:"Límite de fuerza"}
MB_STATUS   = {0x00:"OK",0x01:"IllegalFn",0x02:"IllegalAddr",0x03:"IllegalVal",0x04:"SlaveFail",
               0xE0:"BadSlaveID",0xE1:"BadFn",0xE2:"TIMEOUT",0xE3:"badCRC"}
def mbst(s): return MB_STATUS.get(s, f"0x{s:02X}")

# ── Estado compartido (serial thread → async broadcaster) ──
msg_deque: deque = deque(maxlen=100)   # thread-safe single-producer single-consumer
clients: set[WebSocket] = set()

shared = {
    "port": None,
    "pos_rev": 0.0,
    "last_t": None,
    "force_calib": 0,
    "count": 0,
    "rate": 0.0,
}
shared_lock = threading.Lock()
ser_ref: list = [None]   # contenedor mutable para la referencia al puerto serie

# ── Protocolo ─────────────────────────────────────────────
def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc

def build_packet(payload: bytes) -> bytes:
    return SYNC + bytes([len(payload)]) + payload + bytes([crc8(payload)])

def decode_packet(payload: bytes) -> list | None:
    """Batched frame → list of per-sample dicts (each looks like the old single packet)."""
    if len(payload) < HEADER_SIZE or payload[0] != 0x02:
        return None
    (_pid, base_t, bridge, lc_status, lc_flags, error, servo_id,
     rpm_status, trq_status, n) = struct.unpack(HEADER_FMT, payload[:HEADER_SIZE])
    out = []
    off = HEADER_SIZE
    for _ in range(n):
        if off + SAMPLE_SIZE > len(payload):
            break
        dt, rpm, cur, ref, base, io, state = struct.unpack(SAMPLE_FMT, payload[off:off + SAMPLE_SIZE])
        off += SAMPLE_SIZE
        out.append({
            "t_ms": (base_t + dt) & 0xFFFFFFFF,
            "rpm": rpm, "current_x10": cur, "ref_cmd": ref, "base_read": base,
            "io": io, "servo_state": state,
            "mode": 1 if state == 4 else 2 if state == 5 else 0,
            "bridge": bridge, "lc_status": lc_status, "lc_flags": lc_flags,
            "error": error, "servo_id": servo_id,
            "rpm_status": rpm_status, "trq_status": trq_status,
        })
    return out

def read_packet(ser) -> bytes | None:
    while True:
        b = ser.read(1)
        if not b: return None
        if b[0] == SYNC[0]:
            b2 = ser.read(1)
            if b2 and b2[0] == SYNC[1]: break
    lb = ser.read(1)
    if not lb: return None
    n = lb[0]
    if not n or n > 254: return None
    raw = ser.read(n + 1)
    if len(raw) < n + 1: return None
    payload = raw[:n]
    return payload if raw[n] == crc8(payload) else b"CRC_ERR"

def find_port() -> str | None:
    for pat in ["/dev/ttyACM*", "/dev/ttyUSB*", "/dev/cu.usbmodem*", "COM*"]:
        ports = sorted(glob.glob(pat))
        if ports: return ports[0]
    return None

# ── Hilo serie ────────────────────────────────────────────
def serial_thread(preferred: str | None):
    rate_window: list[float] = []
    while True:
        port = preferred if (preferred and os.path.exists(preferred)) else find_port()
        if not port:
            time.sleep(1.0)
            continue

        ser = None
        try:
            ser = serial.Serial(port, BAUD, timeout=0.1)
            ser_ref[0] = ser
            with shared_lock:
                shared["port"] = port
            time.sleep(0.5)
            ser.reset_input_buffer()
            print(f"# Serial OK: {port}")

            while True:
                payload = read_packet(ser)
                if payload is None or payload == b"CRC_ERR":
                    continue
                samples = decode_packet(payload)
                if not samples:
                    continue

                last = samples[-1]
                log.info("FRAME n=%d  st=%-11s rpm=%5d  trq=%6.1f  ref=%5d  id=%d  RPMrd=%-7s TRQrd=%-7s",
                         len(samples), STATE_NAMES.get(last["servo_state"], "?"), last["rpm"],
                         last["current_x10"] / 10.0, last["ref_cmd"], last["servo_id"],
                         mbst(last["rpm_status"]), mbst(last["trq_status"]))

                for data in samples:                       # one loop iteration per sample
                    t_ms = data["t_ms"]
                    rate_window.append(t_ms)               # rate on DEVICE time → true loop Hz
                    while rate_window and t_ms - rate_window[0] > 1000:
                        rate_window.pop(0)
                    with shared_lock:
                        rpm = data["rpm"]
                        if shared["last_t"] is not None:   # integrate on DEVICE time
                            dt = (t_ms - shared["last_t"]) / 1000.0
                            if 0.0 < dt < 5.0:
                                shared["pos_rev"] += rpm * dt / 60.0
                        shared["last_t"] = t_ms
                        shared["count"] += 1
                        pos_rev = shared["pos_rev"]; fc = shared["force_calib"]; cnt = shared["count"]

                    io = data["io"]; br = data["base_read"]
                    msg = dict(data)
                    msg["pos_rev"]    = round(pos_rev, 5)
                    msg["force_n"]    = round(br / fc, 3) if fc else None
                    msg["state_name"] = STATE_NAMES.get(data["servo_state"], "?")
                    msg["mode_name"]  = MODE_NAMES.get(data["mode"], "?")
                    msg["error_name"] = ERROR_NAMES.get(data["error"], f"cod {data['error']}")
                    msg["limit_a"]    = bool(io & 0x01)
                    msg["limit_b"]    = bool(io & 0x02)
                    msg["rate"]       = len(rate_window)
                    msg["count"]      = cnt
                    msg["port"]       = port
                    msg_deque.append(msg)

                with shared_lock:
                    shared["rate"] = len(rate_window)

        except Exception as e:
            print(f"# Serial error: {e}  (reintentando en 1 s...)")
        finally:
            ser_ref[0] = None
            if ser is not None:
                try: ser.close()
                except Exception: pass
            with shared_lock:
                shared["last_t"] = None   # avoid a huge dt jump across reconnect

        time.sleep(1.0)

# ── Broadcaster WebSocket ─────────────────────────────────
async def broadcaster():
    while True:
        if msg_deque and clients:
            msg = msg_deque.popleft()
            text = json.dumps(msg)
            dead: set[WebSocket] = set()
            for ws in list(clients):
                try:
                    await ws.send_text(text)
                except Exception:
                    dead.add(ws)
            clients.difference_update(dead)
        else:
            await asyncio.sleep(0.02)

# ── FastAPI ───────────────────────────────────────────────
@asynccontextmanager
async def lifespan(app: FastAPI):
    preferred = sys.argv[1] if len(sys.argv) > 1 else None
    threading.Thread(target=serial_thread, args=(preferred,), daemon=True).start()
    asyncio.create_task(broadcaster())
    yield

app = FastAPI(lifespan=lifespan)

@app.get("/")
async def index():
    return HTMLResponse((Path(__file__).parent / "static" / "index.html").read_text())

@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    clients.add(ws)
    try:
        while True:
            text = await ws.receive_text()
            _handle_command(json.loads(text))
    except WebSocketDisconnect:
        pass
    finally:
        clients.discard(ws)

def _handle_command(cmd: dict):
    action = cmd.get("cmd", "")
    extra = " ".join(f"{k}={v}" for k, v in cmd.items() if k != "cmd")
    log.info("CMD   %-9s %s", action, extra)

    ser = ser_ref[0]
    if not ser:
        log.warning("CMD   %-9s ignored — no serial port", action)
        return

    try:
        if action == "init":
            addr = int(cmd.get("addr", 0))
            payload = bytes([CMD_INIT, addr]) if 0 < addr <= 247 else bytes([CMD_INIT])
            ser.write(build_packet(payload))
            with shared_lock:
                shared["pos_rev"] = 0.0
                shared["last_t"] = None
        elif action == "stop":
            ser.write(build_packet(bytes([CMD_STOP])))
        elif action == "move_a":
            ser.write(build_packet(bytes([CMD_MOVE_A])))
        elif action == "move_b":
            ser.write(build_packet(bytes([CMD_MOVE_B])))
        elif action == "shutdown":
            ser.write(build_packet(bytes([CMD_SHUTDOWN])))
        elif action == "reset_pos":
            with shared_lock:
                shared["pos_rev"] = 0.0
                shared["last_t"] = None
        elif action == "set_param":
            pid = int(cmd.get("param", 0))
            val = int(cmd.get("value", 0))
            if pid == PARAM_FORCE_CALIB:
                with shared_lock:
                    shared["force_calib"] = val
            ser.write(build_packet(bytes([CMD_SET_PARAM, pid]) + struct.pack("<h", val)))
    except (serial.SerialException, OSError) as e:
        # Puerto caído (p.ej. reflasheo/replug): soltar el handle; el hilo serie reconecta.
        print(f"# Serial write error: {e}  (marcando desconectado)")
        ser_ref[0] = None
        try: ser.close()
        except Exception: pass

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)
