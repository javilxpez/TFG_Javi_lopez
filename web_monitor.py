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

# Batched frame: header (14 B) + N × sample (18 B)
HEADER_FMT  = "<BIHBBBBBBB"   # id, base_t, bridge, lc_status, flags, error, servo_id, rpm_st, trq_st, n
HEADER_SIZE = struct.calcsize(HEADER_FMT)   # 14
# lc2 = célula 2 (NAU7802) en crudo, 24 bit con signo extendido a int32
SAMPLE_FMT  = "<HhhhhBBHi"    # dt_ms, rpm, current_x10, ref_cmd, base_read, io, state, work_us, lc2
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)   # 18

# lc_flags: b0 = LC1 válida, b1 = servo conectado, b2 = config LC1 aplicada,
#           b3 = LC2 válida, b4 = LC2 inicializada

CMD_INIT = 0x01; CMD_STOP = 0x02; CMD_SET_PARAM = 0x03
CMD_MOVE_A = 0x04; CMD_MOVE_B = 0x05; CMD_SHUTDOWN = 0x06; CMD_HOME = 0x08
CMD_LC_PROGRAM = 0x09; CMD_LC_QUERY = 0x0A; CMD_LC_HOLD = 0x0B

PARAM_SPEED = 0x01; PARAM_FORCE_LIMIT = 0x02; PARAM_ZERO_OFFSET = 0x03
PARAM_ACCEL = 0x04; PARAM_DECEL = 0x05; PARAM_STIFFNESS = 0x06; PARAM_FORCE_CALIB = 0x07
PARAM_HOME_OFFSET = 0x08; PARAM_HOMING_SPEED = 0x09

# v2: frame propio de estado de homing
PACKET_ID_HOME = 0x03
HOME_FMT  = "<BiiBB"   # id, pos_mrev, range_mrev, phase, flags
HOME_SIZE = struct.calcsize(HOME_FMT)

# Programación de la célula: modelo pregunta-respuesta. Cada CMD_LC_PROGRAM /
# CMD_LC_QUERY devuelve exactamente un frame de estos, también cuando se rechaza.
PACKET_ID_LC = 0x04
LC_FMT  = "<BBBBBHHBBH" # id, req, res, ganancia, offset, cfg_antes, cfg_después, ack, flags, cfg1
LC_SIZE = struct.calcsize(LC_FMT)
LC_RESULT_NAMES = {
    0: "sin programar",
    1: "grabado y verificado",
    2: "ya estaba así (no se ha escrito)",
    3: "rechazado: el sistema no está parado",
    4: "rechazado: bus ocupado",
    5: "el chip se reinicia pero no da el 0x5A a tiempo",
    6: "fallo: la relectura no coincide",
    7: "lectura de la configuración actual",
    8: "sigue alimentada con el reset abajo y el bus parqueado: hay otro camino",
    9: "la célula no contesta: sin tensión o sin bus",
    10: "reset mantenido para medir — mide ahora el rail de la Click",
}
LC_RESULT_OK = (1, 2, 7, 10)

STATE_NAMES = {0:"IDLE",1:"SCANNING",2:"CONFIGURING",3:"STOPPED",4:"MOVING_A",5:"MOVING_B",6:"ERROR",
               7:"HOMING",8:"PROGRAMANDO"}
MODE_NAMES  = {0:"PARADO",1:"MOV_A",2:"MOV_B"}
HOME_PHASE_NAMES = {0:"—",1:"BUSCANDO A",2:"BUSCANDO B",3:"YENDO A HOME",4:"COMPLETADO",5:"FALLO"}
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
    "home_range_rev": 0.0,
    "home_phase": 0,
    "home_range_valid": False,
    "lc_gain": None,       # lo que el firmware dice que hay grabado en la célula
    "lc_offset": None,
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

def decode_home(payload: bytes) -> dict | None:
    """v2: frame de estado de homing (PACKET_ID_HOME)."""
    if len(payload) < HOME_SIZE or payload[0] != PACKET_ID_HOME:
        return None
    _pid, pos_mrev, range_mrev, phase, flags = struct.unpack(HOME_FMT, payload[:HOME_SIZE])
    return {"home_pos_rev": pos_mrev / 1000.0, "home_range_rev": range_mrev / 1000.0,
            "home_phase": phase, "home_range_valid": bool(flags & 0x01)}

def decode_lc(payload: bytes) -> dict | None:
    """Respuesta a CMD_LC_PROGRAM / CMD_LC_QUERY (PACKET_ID_LC)."""
    if len(payload) < LC_SIZE or payload[0] != PACKET_ID_LC:
        return None
    (_pid, req, result, gain, offset,
     cfg_before, cfg_after, ack, flags, cfg1) = struct.unpack(LC_FMT, payload[:LC_SIZE])
    return {"lc_req": req, "lc_result": result,
            "lc_result_name": LC_RESULT_NAMES.get(result, f"?{result}"),
            "lc_ok": result in LC_RESULT_OK,
            "lc_gain": gain, "lc_offset": offset,
            "lc_cfg_before": cfg_before, "lc_cfg_after": cfg_after,
            "lc_ack": ack,
            # Si el pin de reset no corta, la EEPROM se graba igual pero el valor nuevo
            # no se carga hasta que el chip pase por un apagado de verdad.
            "lc_en_cuts": bool(flags & 0x01),
            # Retardo del barrido con el que se logró entrar en modo comando, en µs:
            # es el dato que dice cuánto tarda de verdad en arrancar esta Click.
            "lc_entry_us": ((flags >> 4) & 0x0F) * 150,
            "lc_cfg1": cfg1}


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
        (dt, rpm, cur, ref, base, io, state,
         work_us, lc2) = struct.unpack(SAMPLE_FMT, payload[off:off + SAMPLE_SIZE])
        off += SAMPLE_SIZE
        out.append({
            "t_ms": (base_t + dt) & 0xFFFFFFFF, "work_us": work_us,
            "rpm": rpm, "current_x10": cur, "ref_cmd": ref, "base_read": base,
            "io": io, "servo_state": state,
            "mode": 1 if state == 4 else 2 if state == 5 else 0,
            "bridge": bridge, "lc_status": lc_status, "lc_flags": lc_flags,
            "lc2": lc2, "lc2_valid": bool(lc_flags & 0x08), "lc2_ok": bool(lc_flags & 0x10),
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

                if payload and payload[0] == PACKET_ID_LC:     # respuesta de programación
                    lc = decode_lc(payload)
                    if lc:
                        with shared_lock:
                            shared["lc_gain"]   = lc["lc_gain"]
                            shared["lc_offset"] = lc["lc_offset"]
                        msg = dict(lc)
                        msg["type"] = "lc_config"
                        msg["port"] = port
                        msg_deque.append(msg)
                    continue

                if payload and payload[0] == PACKET_ID_HOME:   # v2: estado de homing
                    hp = decode_home(payload)
                    if hp:
                        with shared_lock:
                            shared["pos_rev"]          = hp["home_pos_rev"]  # firmware manda la posición
                            shared["home_range_rev"]   = hp["home_range_rev"]
                            shared["home_phase"]        = hp["home_phase"]
                            shared["home_range_valid"]  = hp["home_range_valid"]
                        msg = dict(hp)
                        msg["type"]            = "home"
                        msg["pos_rev"]         = round(hp["home_pos_rev"], 5)
                        msg["home_range_rev"]  = round(hp["home_range_rev"], 5)
                        msg["home_phase_name"] = HOME_PHASE_NAMES.get(hp["home_phase"], "?")
                        msg["port"]            = port
                        msg_deque.append(msg)
                    continue

                samples = decode_packet(payload)
                if not samples:
                    continue

                last = samples[-1]
                maxw = max(s["work_us"] for s in samples) / 1000.0
                log.info("FRAME n=%d  st=%-11s rpm=%5d  trq=%6.1f  ref=%5d  id=%d  RPMrd=%-7s TRQrd=%-7s  loop_max=%.1fms",
                         len(samples), STATE_NAMES.get(last["servo_state"], "?"), last["rpm"],
                         last["current_x10"] / 10.0, last["ref_cmd"], last["servo_id"],
                         mbst(last["rpm_status"]), mbst(last["trq_status"]), maxw)

                for data in samples:                       # one loop iteration per sample
                    t_ms = data["t_ms"]
                    # El Pico reinicia su millis() a 0 cada vez que se reflashea o se
                    # resetea. Sin esta comprobación, t_ms da un salto hacia atrás, la
                    # resta de abajo sale negativa, la ventana no se poda NUNCA y crece
                    # sin límite: "3813 Hz" no era el loop, era el número de muestras
                    # acumuladas desde el último reflasheo.
                    if rate_window and not (0 <= t_ms - rate_window[-1] < 5000):
                        rate_window.clear()                # reloj del equipo reiniciado
                        shared["last_t"] = None            # y la integración, rebasada
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
                        h_range = shared["home_range_rev"]; h_phase = shared["home_phase"]
                        h_valid = shared["home_range_valid"]

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
                    msg["home_range_rev"]   = round(h_range, 5)
                    msg["home_phase"]       = h_phase
                    msg["home_phase_name"]  = HOME_PHASE_NAMES.get(h_phase, "?")
                    msg["home_range_valid"] = h_valid
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

def _ack(action: str, ok: bool, detail: str):
    """Todo comando contesta. Sin esto, un comando que no llega a salir por el puerto
    —o que no lo reconoce nadie— se pierde en silencio y desde la interfaz es
    indistinguible de uno que sí ha funcionado."""
    msg_deque.append({"type": "cmd_ack", "cmd": action, "ok": bool(ok), "detail": detail})
    log.info("ACK   %-9s %-4s %s", action, "OK" if ok else "FALLO", detail)


def _handle_command(cmd: dict):
    action = cmd.get("cmd", "")
    extra = " ".join(f"{k}={v}" for k, v in cmd.items() if k != "cmd")
    log.info("CMD   %-9s %s", action, extra)

    ser = ser_ref[0]
    if not ser:
        _ack(action, False, "sin puerto serie: el Pico no está conectado")
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
        elif action == "home":
            ser.write(build_packet(bytes([CMD_HOME])))
        elif action == "lc_query":
            ser.write(build_packet(bytes([CMD_LC_QUERY])))
        elif action == "lc_hold":
            # park: 0 = líneas en alta impedancia, 1 = forzadas a masa
            park = 1 if int(cmd.get("park", 0)) else 0
            ser.write(build_packet(bytes([CMD_LC_HOLD, park])))
        elif action == "lc_program":
            # Las dos palabras enteras. El firmware sanea lo intocable (I2C/SPI, bits
            # reservados, polaridades de temperatura) antes de escribir nada.
            bcfg = int(cmd.get("bcfg", 0)) & 0xFFFF
            cfg1 = int(cmd.get("cfg1", 0)) & 0xFFFF
            ser.write(build_packet(bytes([CMD_LC_PROGRAM]) + struct.pack("<HH", bcfg, cfg1)))
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
        else:
            # Sin este caso, un nombre mal escrito en la interfaz no hacía nada y no
            # se notaba: el botón parecía roto sin decir por qué.
            _ack(action, False, f"acción desconocida: {action!r}")
            return
    except (serial.SerialException, OSError) as e:
        # Puerto caído (p.ej. reflasheo/replug): soltar el handle; el hilo serie reconecta.
        print(f"# Serial write error: {e}  (marcando desconectado)")
        ser_ref[0] = None
        try: ser.close()
        except Exception: pass
        _ack(action, False, f"error de puerto: {e}")
        return
    except Exception as e:
        _ack(action, False, f"{type(e).__name__}: {e}")
        return
    _ack(action, True, "enviado")

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)
