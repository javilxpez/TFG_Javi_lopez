#!/usr/bin/env python3
"""
ZSC31014 + Servo Monitor — Binary Protocol v2
Bidirectional: telemetry (ESP→Python) + commands (Python→ESP)

Usage:
    python monitor.py              # auto-detect port
    python monitor.py /dev/ttyACM0 # explicit port

Controls (keyboard):
    1  Start in TORQUE mode
    2  Start in SPEED mode
    s  Stop (servo OFF, back to idle)
    t  Set torque reference (prompts for value)
    v  Set speed reference (prompts for value)
    q  Quit

Dependencies:
    pip install pyserial
"""

import sys
import os
import struct
import time
import signal
import glob
import threading
import serial

# ── Config ────────────────────────────────────────
BAUD = 115200
SYNC = bytes([0xAA, 0x55])

# Command IDs
CMD_START     = 0x01
CMD_STOP      = 0x02
CMD_SET_PARAM = 0x03

# Param IDs
PARAM_TORQUE_REF      = 0x01
PARAM_SPEED_REF       = 0x02
PARAM_TORQUE_LIM_POS  = 0x03
PARAM_TORQUE_LIM_NEG  = 0x04

# State names (match firmware enum)
STATE_NAMES = {
    0: "IDLE",
    1: "SCANNING",
    2: "CONFIGURING",
    3: "RUNNING",
    4: "ERROR",
}

MODE_NAMES = {0: "SPEED", 1: "TORQUE"}

# Error codes (match firmware #defines)
ERROR_CODES = {
    0x00: "NONE",
    0x01: "LC_STALE_TIMEOUT — load cell stuck returning stale data >50ms",
    0x02: "LC_I2C_FAIL — I2C read returned no bytes",
    0x03: "LC_NOT_FOUND — load cell not detected at startup",
    0x04: "SERVO_CONFIG_FAIL — servo configuration write failed",
    0x05: "SERVO_SCAN_FAIL — no servo found on bus",
}

# ── Packet decoders by packet_id ──────────────────
# v1 kept for backwards compat, v2 is current
DECODERS = {
    0x01: ("<BIHBB", ("packet_id", "t_ms", "bridge", "lc_status", "lc_flags")),
    0x02: ("<BIHBBhhBB", ("packet_id", "t_ms", "bridge", "lc_status", "lc_flags",
                           "rpm", "torque_x10", "servo_state", "mode")),
}

# ── ANSI ──────────────────────────────────────────
CURSOR_HOME = "\033[H"
CLEAR_DOWN  = "\033[J"
BOLD        = "\033[1m"
DIM         = "\033[2m"
GREEN       = "\033[32m"
YELLOW      = "\033[33m"
RED         = "\033[31m"
CYAN        = "\033[36m"
RESET       = "\033[0m"
HIDE_CURSOR = "\033[?25l"
SHOW_CURSOR = "\033[?25h"


def find_port():
    patterns = [
        "/dev/ttyACM*", "/dev/ttyUSB*",
        "/dev/cu.usbmodem*", "/dev/cu.usbserial*", "/dev/cu.SLAB*",
        "COM*",
    ]
    for pat in patterns:
        ports = sorted(glob.glob(pat))
        if ports:
            return ports[0]
    return None


def crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_packet(payload: bytes) -> bytes:
    """Wrap payload in sync + length + crc frame."""
    return SYNC + bytes([len(payload)]) + payload + bytes([crc8(payload)])


def send_start(ser: serial.Serial, torque: bool):
    payload = bytes([CMD_START, 0x01 if torque else 0x00])
    ser.write(build_packet(payload))


def send_stop(ser: serial.Serial):
    ser.write(build_packet(bytes([CMD_STOP])))


def send_set_param(ser: serial.Serial, param_id: int, value: int):
    v = struct.pack("<h", value)
    payload = bytes([CMD_SET_PARAM, param_id]) + v
    ser.write(build_packet(payload))


def bar(value, width=30, lo=0.0, hi=1.0):
    frac = max(0.0, min(1.0, (value - lo) / (hi - lo))) if hi != lo else 0
    filled = int(frac * width)
    return f"[{'█' * filled}{'░' * (width - filled)}]"


def decode_packet(payload: bytes) -> dict | None:
    if not payload:
        return None
    packet_id = payload[0]
    dec = DECODERS.get(packet_id)
    if not dec:
        return None
    fmt, fields = dec
    expected = struct.calcsize(fmt)
    if len(payload) < expected:
        return None
    values = struct.unpack(fmt, payload[:expected])
    return dict(zip(fields, values))


def render(data: dict, meta: dict):
    bridge     = data.get("bridge", 0)
    norm       = bridge / 16383.0
    lc_status  = data.get("lc_status", -1)
    lc_flags   = data.get("lc_flags", 0)
    t_ms       = data.get("t_ms", 0)
    rpm        = data.get("rpm", 0)
    torque_x10 = data.get("torque_x10", 0)
    torque     = torque_x10 / 10.0
    state      = data.get("servo_state", 0)
    mode       = data.get("mode", 0)

    state_name = STATE_NAMES.get(state, "???")
    mode_name  = MODE_NAMES.get(mode, "???")

    lc_status_str = {
        0: f"{GREEN}VALID{RESET}",
        1: f"{YELLOW}STALE{RESET}",
        2: f"{RED}CMD{RESET}",
    }.get(lc_status, f"{RED}ERR{RESET}")

    state_color = {
        0: DIM,       # IDLE
        1: YELLOW,    # SCANNING
        2: YELLOW,    # CONFIGURING
        3: GREEN,     # RUNNING
        4: RED,       # ERROR
    }.get(state, RESET)

    rows = []

    # System
    # In ERROR state, lc_flags carries the error code; otherwise bit0 = valid
    if state == 4:  # STATE_ERROR
        err_code = lc_flags
        lc_valid = False
        servo_connected = False
    else:
        err_code = 0
        lc_valid = bool(lc_flags & 0x01)
        servo_connected = bool(lc_flags & 0x02)

    rows.append(("State",       f"{state_color}{state_name}{RESET}"))
    if state == 4:
        err_desc = ERROR_CODES.get(err_code, f"UNKNOWN (0x{err_code:02X})")
        rows.append(("Error",       f"{RED}E{err_code:02X}: {err_desc}{RESET}"))
    rows.append(("Mode",        f"{CYAN}{mode_name}{RESET}"))
    servo_str = f"{GREEN}Connected{RESET}" if servo_connected else f"{RED}Disconnected{RESET}"
    rows.append(("Servo",       servo_str))
    rows.append(("Device Time", f"{t_ms / 1000:.1f}s"))
    rows.append(("", ""))  # separator

    # Load Cell
    rows.append(("Bridge Raw",  f"{bridge:>6d} / 16383"))
    rows.append(("Normalized",  f"{norm:>9.4f}  {bar(norm)}"))
    rows.append(("LC Status",   lc_status_str))
    lc_valid_str = f"{GREEN}Yes{RESET}" if lc_valid else f"{DIM}No{RESET}"
    rows.append(("LC Valid",    lc_valid_str))
    rows.append(("", ""))

    # Motor
    rows.append(("RPM",         f"{rpm:>6d}"))
    rows.append(("Torque",      f"{torque:>6.1f}%  {bar(torque, lo=-100, hi=100)}"))

    # ── Expand here ──────────────────────────────
    # rows.append(("New Field", f"{data.get('new_field', 0)}"))
    # ─────────────────────────────────────────────

    label_w = max((len(r[0]) for r in rows if r[0]), default=12)

    lines = []
    lines.append(f"{BOLD}{'═' * 56}")
    lines.append(f"  ZSC31014 + Servo Monitor{RESET}")
    lines.append(f"{'═' * 56}")
    lines.append(f"{DIM}  Port: {meta['port']}  @{BAUD}  │  Ctrl+C / q to quit{RESET}")
    lines.append(f"  Reads: {meta['count']}  │  {meta['rate']:.0f} Hz  │  CRC err: {meta['crc_err']}  │  Decode err: {meta['decode_err']}")
    lines.append("─" * 56)

    for label, value in rows:
        if not label:
            lines.append("")
        else:
            lines.append(f"  {label:<{label_w}}  │  {value}")

    lines.append("─" * 56)
    lines.append(f"{DIM}  [1] Start Torque  [2] Start Speed  [s] Stop  [q] Quit{RESET}")
    lines.append(f"{DIM}  [t] Set Torque Ref  [v] Set Speed Ref{RESET}")

    sys.stdout.write(CURSOR_HOME + "\n".join(lines) + CLEAR_DOWN + "\n")
    sys.stdout.flush()


def read_packet(ser: serial.Serial) -> bytes | None:
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] == SYNC[0]:
            b2 = ser.read(1)
            if not b2:
                return None
            if b2[0] == SYNC[1]:
                break

    lb = ser.read(1)
    if not lb:
        return None
    length = lb[0]
    if length == 0 or length > 60:
        return None

    raw = ser.read(length + 1)
    if len(raw) < length + 1:
        return None

    payload = raw[:length]
    crc_rx = raw[length]
    if crc_rx != crc8(payload):
        return b"CRC_ERR"

    return payload


def drain_text(ser: serial.Serial):
    ser.timeout = 0.5
    while True:
        line = ser.readline()
        if not line:
            break
        text = line.decode(errors="replace").strip()
        if text.startswith("#"):
            print(text)
        elif text == "":
            continue
        else:
            break
    ser.timeout = 0.1  # Fast for interleaved read + keyboard


def input_thread(ser: serial.Serial, running: threading.Event):
    """Handle keyboard input in a separate thread."""
    try:
        # Platform-specific raw keyboard input
        if os.name == "nt":
            import msvcrt
            def getch():
                return msvcrt.getch().decode(errors="replace")
        else:
            import tty
            import termios
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            tty.setcbreak(fd)
            def getch():
                return sys.stdin.read(1)

        while running.is_set():
            ch = getch()

            if ch == "1":
                send_start(ser, torque=True)
            elif ch == "2":
                send_start(ser, torque=False)
            elif ch in ("s", "S"):
                send_stop(ser)
            elif ch in ("t", "T"):
                # Quick inline prompt — print below dashboard
                sys.stdout.write(f"\033[25;1H\033[K  Torque ref (x10, e.g. 40=4.0%): ")
                sys.stdout.flush()
                # Restore cooked mode briefly for line input
                if os.name != "nt":
                    termios.tcsetattr(fd, termios.TCSANOW, old)
                try:
                    val = int(input())
                    send_set_param(ser, PARAM_TORQUE_REF, val)
                except (ValueError, EOFError):
                    pass
                if os.name != "nt":
                    tty.setcbreak(fd)
            elif ch in ("v", "V"):
                sys.stdout.write(f"\033[25;1H\033[K  Speed RPM: ")
                sys.stdout.flush()
                if os.name != "nt":
                    termios.tcsetattr(fd, termios.TCSANOW, old)
                try:
                    val = int(input())
                    send_set_param(ser, PARAM_SPEED_REF, val)
                except (ValueError, EOFError):
                    pass
                if os.name != "nt":
                    tty.setcbreak(fd)
            elif ch in ("q", "Q"):
                running.clear()
                break

    except Exception:
        pass
    finally:
        if os.name != "nt":
            try:
                termios.tcsetattr(fd, termios.TCSANOW, old)
            except Exception:
                pass


def die(msg: str):
    """Print error and wait briefly so the message is visible even if the
    terminal window auto-closes (e.g. double-click launch)."""
    print(f"\n{RED}ERROR: {msg}{RESET}")
    print(f"{DIM}(exiting in 5 s — press Ctrl+C to close now){RESET}")
    try:
        time.sleep(5)
    except KeyboardInterrupt:
        pass
    sys.exit(1)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if not port:
        die("No serial port found.\n  Pass port as argument, e.g.:  python monitor.py /dev/ttyACM0")

    print(f"Connecting to {port} @ {BAUD}...")

    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
    except serial.SerialException as e:
        die(f"Cannot open {port}: {e}")

    # Drain startup text
    time.sleep(0.5)
    drain_text(ser)
    ser.reset_input_buffer()

    sys.stdout.write(HIDE_CURSOR)

    running = threading.Event()
    running.set()

    # Start keyboard thread
    kb = threading.Thread(target=input_thread, args=(ser, running), daemon=True)
    kb.start()

    def cleanup(*_):
        running.clear()

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    meta = {"port": port, "count": 0, "crc_err": 0, "decode_err": 0, "rate": 0.0}
    rate_window = []
    last_data = {}

    try:
        while running.is_set():
            payload = read_packet(ser)
            if payload is None:
                # No data — render last known state anyway
                if last_data:
                    render(last_data, meta)
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
            last_data = data

            render(data, meta)

    except serial.SerialException:
        sys.stdout.write(SHOW_CURSOR)
        print(f"\n{RED}Serial disconnected.{RESET}")
    finally:
        running.clear()
        sys.stdout.write(SHOW_CURSOR + "\n")
        ser.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        sys.stdout.write(SHOW_CURSOR)
        die(f"Unhandled exception: {e}")