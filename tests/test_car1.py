#!/usr/bin/env python3
from __future__ import annotations

import json
import socket
import time

HOST = "192.168.4.1"
PORT = 9000
CAR_ID = 1

HZ = 30.0
DT = 1.0 / HZ

# speed settings (0~1)
V_FWD = 0.40
V_TURN = 0.35


def make_line(vl: float, vr: float) -> bytes:
    msg = {"actions": {str(CAR_ID): [float(vl), float(vr)]}}
    return (json.dumps(msg, separators=(",", ":")) + "\n").encode("utf-8")


def hold(sock: socket.socket, vl: float, vr: float, seconds: float) -> None:
    """Send (vl, vr) repeatedly for `seconds`."""
    t_end = time.time() + seconds
    line = make_line(vl, vr)
    while time.time() < t_end:
        sock.sendall(line)
        time.sleep(DT)


def stop(sock: socket.socket, seconds: float = 0.30) -> None:
    """Send stop repeatedly to be safe."""
    hold(sock, 0.0, 0.0, seconds)


def auto_pattern(sock: socket.socket) -> None:
    seq = [
        ("FORWARD",  +V_FWD, +V_FWD, 1.5),
        ("STOP",      0.0,    0.0,  0.6),
        ("BACKWARD", -V_FWD, -V_FWD, 1.5),
        ("STOP",      0.0,    0.0,  0.6),
        ("TURN_L",   -V_TURN, +V_TURN, 1.2),
        ("STOP",      0.0,    0.0,  0.6),
        ("TURN_R",   +V_TURN, -V_TURN, 1.2),
        ("STOP",      0.0,    0.0,  0.6),
    ]
    for name, vl, vr, dur in seq:
        print(f"[auto] {name:8s} vl={vl:+.2f} vr={vr:+.2f}  {dur:.1f}s")
        hold(sock, vl, vr, dur)
    stop(sock)
    print("[auto] done.")


def interactive(sock: socket.socket) -> None:
    print("\n=== CAR1 TEST REMOTE ===")
    print("Enter one of: w/a/s/d/x")
    print("  w: forward")
    print("  s: backward")
    print("  a: left (spin)")
    print("  d: right (spin)")
    print("  x: stop")
    print("  p: auto pattern")
    print("  q: quit\n")

    stop(sock)

    while True:
        cmd = input("> ").strip().lower()
        if not cmd:
            continue

        if cmd == "q":
            stop(sock)
            break
        if cmd == "p":
            auto_pattern(sock)
            continue

        if cmd == "w":
            vl, vr = +V_FWD, +V_FWD
        elif cmd == "s":
            vl, vr = -V_FWD, -V_FWD
        elif cmd == "a":
            vl, vr = -V_TURN, +V_TURN
        elif cmd == "d":
            vl, vr = +V_TURN, -V_TURN
        elif cmd == "x":
            vl, vr = 0.0, 0.0
        else:
            print("Unknown. Use w/a/s/d/x, p, q")
            continue

        # Hold for a short burst so you can see it move
        # (you can type the same command again to keep moving)
        print(f"[send] vl={vl:+.2f} vr={vr:+.2f} (0.6s)")
        hold(sock, vl, vr, 0.6)
        stop(sock, 0.15)


def main() -> int:
    print(f"[pc] connecting to AP0 TCP {HOST}:{PORT} ...")
    with socket.create_connection((HOST, PORT), timeout=3.0) as sock:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        # read hello if any (optional)
        sock.settimeout(0.2)
        try:
            data = sock.recv(4096)
            if data:
                print("[pc] recv:", data.decode("utf-8", errors="replace").strip())
        except Exception:
            pass
        finally:
            sock.settimeout(None)

        interactive(sock)

    print("[pc] bye.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
