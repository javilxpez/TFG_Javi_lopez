#!/usr/bin/env python3
"""
Monitor Posición A→B — Pico 2 + ZSC31014 + Servo
Binary protocol 19 bytes, RPM integration for position.

Usage:
    python monitor_posicion.py [port]

Controls:
    i  Init (scan + configure servo) + reset position
    r  Reset position counter
    b  Move B  (+speed)
    a  Move A  (-speed)
    x/s  Stop
    q  Shutdown + quit
"""

import sys, os, struct, time, signal, glob, threading, serial

BAUD = 115200
SYNC = bytes([0xAA, 0x55])

# Batched frame: header (14 B) + N × sample (12 B)
HEADER_FMT  = "<BIHBBBBBBB"   # id, base_t, bridge, lc_status, flags, error, servo_id, rpm_st, trq_st, n
HEADER_SIZE = struct.calcsize(HEADER_FMT)
SAMPLE_FMT  = "<HhhhhBB"      # dt_ms, rpm, current_x10, ref_cmd, base_read, io, state
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)

MOVE_SPEED = 10  # RPM default

CMD_INIT      = 0x01
CMD_STOP      = 0x02
CMD_SET_PARAM = 0x03
CMD_MOVE_A    = 0x04
CMD_MOVE_B    = 0x05
CMD_SHUTDOWN  = 0x06

PARAM_SPEED       = 0x01
PARAM_FORCE_LIMIT = 0x02
PARAM_ZERO_OFFSET = 0x03
PARAM_ACCEL       = 0x04
PARAM_DECEL       = 0x05
PARAM_STIFFNESS   = 0x06
PARAM_FORCE_CALIB = 0x07

STATE_NAMES = {
    0: "IDLE", 1: "SCANNING", 2: "CONFIGURING",
    3: "STOPPED", 4: "MOVING_A", 5: "MOVING_B", 6: "ERROR",
}
MODE_NAMES = {0: "PARADO", 1: "MOV_A", 2: "MOV_B"}

CURSOR_HOME = "\033[H"; CLEAR_DOWN = "\033[J"
BOLD = "\033[1m"; DIM = "\033[2m"
GREEN = "\033[32m"; YELLOW = "\033[33m"; RED = "\033[31m"; CYAN = "\033[36m"
RESET = "\033[0m"; HIDE_CURSOR = "\033[?25l"; SHOW_CURSOR = "\033[?25h"


def find_port():
    for pat in ["/dev/ttyACM*","/dev/ttyUSB*","/dev/cu.usbmodem*","COM*"]:
        ports = sorted(glob.glob(pat))
        if ports:
            return ports[0]
    return None


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_packet(payload: bytes) -> bytes:
    return SYNC + bytes([len(payload)]) + payload + bytes([crc8(payload)])

def send_init(ser):     ser.write(build_packet(bytes([CMD_INIT])))
def send_stop(ser):     ser.write(build_packet(bytes([CMD_STOP])))
def send_move_a(ser):   ser.write(build_packet(bytes([CMD_MOVE_A])))
def send_move_b(ser):   ser.write(build_packet(bytes([CMD_MOVE_B])))
def send_shutdown(ser): ser.write(build_packet(bytes([CMD_SHUTDOWN])))

def send_param(ser, pid, value):
    ser.write(build_packet(bytes([CMD_SET_PARAM, pid]) + struct.pack("<h", value)))


def bar(value, width=28, lo=0.0, hi=1.0):
    frac = max(0.0, min(1.0, (value - lo) / (hi - lo))) if hi != lo else 0
    n = int(frac * width)
    return f"[{'█'*n}{'░'*(width-n)}]"


def read_packet(ser) -> bytes | None:
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] == SYNC[0]:
            b2 = ser.read(1)
            if b2 and b2[0] == SYNC[1]:
                break
    lb = ser.read(1)
    if not lb: return None
    length = lb[0]
    if not length or length > 254: return None
    raw = ser.read(length + 1)
    if len(raw) < length + 1: return None
    payload = raw[:length]
    if raw[length] != crc8(payload): return b"CRC_ERR"
    return payload


def decode_packet(payload: bytes) -> dict | None:
    # Batched frame → return the LAST sample as a flat dict (terminal shows current state)
    if len(payload) < HEADER_SIZE or payload[0] != 0x02: return None
    (_pid, base_t, bridge, lc_status, lc_flags, error, servo_id,
     rpm_status, trq_status, n) = struct.unpack(HEADER_FMT, payload[:HEADER_SIZE])
    if n == 0: return None
    off = HEADER_SIZE + (n - 1) * SAMPLE_SIZE
    if off + SAMPLE_SIZE > len(payload): return None
    dt, rpm, cur, ref, base, io, state = struct.unpack(SAMPLE_FMT, payload[off:off + SAMPLE_SIZE])
    return {"packet_id": _pid, "t_ms": (base_t + dt) & 0xFFFFFFFF, "bridge": bridge,
            "lc_status": lc_status, "lc_flags": lc_flags, "rpm": rpm, "current_x10": cur,
            "servo_state": state, "mode": 1 if state == 4 else 2 if state == 5 else 0,
            "ref_cmd": ref, "base_read": base, "io": io, "error": error,
            "servo_id": servo_id, "rpm_status": rpm_status, "trq_status": trq_status}


def render(data: dict, meta: dict):
    bridge      = data.get("bridge", 0)
    lc_status   = data.get("lc_status", 0)
    lc_flags    = data.get("lc_flags", 0)
    t_ms        = data.get("t_ms", 0)
    rpm          = data.get("rpm", 0)
    current_x10  = data.get("current_x10", 0)
    state        = data.get("servo_state", 0)
    mode        = data.get("mode", 0)
    ref_cmd     = data.get("ref_cmd", 0)
    base_read   = data.get("base_read", 0)

    lc_valid    = bool(lc_flags & 0x01)
    srv_conn    = bool(lc_flags & 0x02)
    lc_cfg      = bool(lc_flags & 0x04)

    lc_status_s = {0:f"{GREEN}VALID{RESET}",1:f"{YELLOW}STALE{RESET}",2:f"{RED}CMD{RESET}"}.get(lc_status,f"{RED}ERR{RESET}")
    state_s = STATE_NAMES.get(state,"???")
    mode_s  = MODE_NAMES.get(mode,"???")
    sc = {0:DIM,1:YELLOW,2:YELLOW,3:GREEN,4:CYAN,5:CYAN,6:RED}.get(state,RESET)

    fc = meta.get("force_calib", 0)
    pos_rev = meta.get("pos_rev", 0.0)

    lines = []
    lines.append(f"{BOLD}{'═'*58}")
    lines.append(f"  Monitor Posición A→B — Pico 2{RESET}")
    lines.append(f"{'═'*58}")
    lines.append(f"{DIM}  Puerto: {meta['port']}  @{BAUD}{RESET}")
    lines.append(f"  Paquetes: {meta['count']}  │  {meta['rate']:.0f} Hz  │  CRC err: {meta['crc_err']}  │  Dec err: {meta['decode_err']}")
    lines.append("─"*58)
    lines.append(f"  Estado       │  {sc}{state_s}{RESET}")
    lines.append(f"  Modo         │  {CYAN}{mode_s}{RESET}")
    lines.append(f"  Servo        │  {GREEN+'Conectado'+RESET if srv_conn else RED+'Desconectado'+RESET}")
    lines.append(f"  Tiempo       │  {t_ms/1000:.1f} s")
    lines.append("─"*58)
    lines.append(f"  Bridge       │  {bridge:>6d} / 16383")
    lines.append(f"  LC Estado    │  {lc_status_s}  válido: {'Sí' if lc_valid else 'No'}")
    lines.append(f"  LC Config    │  {'OK' if lc_cfg else '---'}")
    lines.append(f"  Fuerza rel.  │  {base_read:>7d} cnt  {bar(base_read, lo=-2000, hi=2000)}")
    if fc and fc != 0:
        force_n = base_read / fc
        lines.append(f"  Fuerza (N)   │  {force_n:>7.2f} N  {bar(force_n, lo=-50, hi=50)}")
    lines.append("─"*58)
    current_a = current_x10 / 10.0
    lines.append(f"  RPM          │  {rpm:>6d}")
    lines.append(f"  Corriente    │  {current_a:>6.1f} A  {bar(abs(current_a), lo=0.0, hi=5.0)}  {'<--' if current_x10 < 0 else '-->'}")
    lines.append(f"  Cmd velocidad│  {ref_cmd:>6d} RPM")
    lines.append(f"  Posición     │  {pos_rev:>8.4f} rev")
    lines.append("─"*58)
    lines.append(f"{DIM}  [i] Init  [b] Mov_B  [a] Mov_A  [x] Stop  [r] ResetPos  [q] Salir{RESET}")

    sys.stdout.write(CURSOR_HOME + "\033[K\n".join(lines) + "\033[K" + CLEAR_DOWN + "\n")
    sys.stdout.flush()


def input_thread(ser, running, meta):
    try:
        if os.name == "nt":
            import msvcrt
            getch = lambda: msvcrt.getch().decode(errors="replace")
        else:
            import tty, termios
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            tty.setcbreak(fd)
            getch = lambda: sys.stdin.read(1)

        while running.is_set():
            ch = getch()
            if ch == "i":
                send_init(ser)
                meta["pos_rev"] = 0.0
                meta["last_packet_t"] = None
            elif ch == "r":
                meta["pos_rev"] = 0.0
                meta["last_packet_t"] = None
            elif ch == "b":
                send_move_b(ser)
            elif ch == "a":
                send_move_a(ser)
            elif ch in ("x","s","X","S"):
                send_stop(ser)
            elif ch in ("q","Q"):
                send_shutdown(ser)
                time.sleep(0.2)
                running.clear()
                break
    except Exception:
        pass
    finally:
        if os.name != "nt":
            try: termios.tcsetattr(fd, termios.TCSANOW, old)
            except Exception: pass


def drain_text(ser):
    ser.timeout = 0.1
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        line = ser.readline()
        if not line: break
        text = line.decode(errors="replace").strip()
        if text.startswith("#"): print(text)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if not port:
        print("ERROR: No se encontró puerto serie."); sys.exit(1)

    print(f"Conectando a {port} @ {BAUD}...")
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
    except serial.SerialException as e:
        print(f"ERROR: {e}"); sys.exit(1)

    time.sleep(0.5)
    drain_text(ser)
    ser.reset_input_buffer()

    sys.stdout.write(HIDE_CURSOR)

    running = threading.Event()
    running.set()

    meta = {
        "port": port, "count": 0, "crc_err": 0, "decode_err": 0,
        "rate": 0.0, "pos_rev": 0.0, "last_packet_t": None,
        "force_calib": 0,
    }

    kb = threading.Thread(target=input_thread, args=(ser, running, meta), daemon=True)
    kb.start()

    signal.signal(signal.SIGINT,  lambda *_: running.clear())
    signal.signal(signal.SIGTERM, lambda *_: running.clear())

    rate_window = []
    last_data = {}

    try:
        while running.is_set():
            payload = read_packet(ser)
            if payload is None:
                if last_data: render(last_data, meta)
                continue
            if payload == b"CRC_ERR":
                meta["crc_err"] += 1
                continue

            data = decode_packet(payload)
            if data is None:
                meta["decode_err"] += 1
                continue

            now = time.monotonic()
            rate_window.append(now)
            rate_window = [t for t in rate_window if now - t < 1.0]
            meta["rate"] = len(rate_window)
            meta["count"] += 1

            # RPM integration for position
            rpm = data.get("rpm", 0)
            if meta["last_packet_t"] is not None:
                dt = now - meta["last_packet_t"]
                meta["pos_rev"] += rpm * dt / 60.0
            meta["last_packet_t"] = now

            last_data = data
            render(data, meta)

    except serial.SerialException:
        sys.stdout.write(SHOW_CURSOR)
        print(f"\n{RED}Serial desconectado.{RESET}")
    finally:
        running.clear()
        sys.stdout.write(SHOW_CURSOR + "\n")
        ser.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        sys.stdout.write(SHOW_CURSOR)
        print(f"ERROR: {e}")
        sys.exit(1)
