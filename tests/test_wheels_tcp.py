#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import socket
import time
from typing import Tuple


def send_actions(sock: socket.socket, car_id: int, vl: float, vr: float) -> None:
    """Send one action line in the format expected by your AP0 firmware."""
    msg = {
        "actions": {
            str(car_id): [float(vl), float(vr)]
        }
    }
    line = json.dumps(msg, separators=(",", ":")) + "\n"
    sock.sendall(line.encode("utf-8"))


def stop(sock: socket.socket, car_id: int, hold_s: float = 0.15, hz: float = 30.0) -> None:
    """Send a brief stop command repeatedly (helps overcome packet loss)."""
    dt = 1.0 / hz
    t_end = time.time() + hold_s
    while time.time() < t_end:
        send_actions(sock, car_id, 0.0, 0.0)
        time.sleep(dt)


def run_step(sock: socket.socket, car_id: int, vl: float, vr: float, duration_s: float, hz: float) -> None:
    """Hold (vl, vr) for duration by sending at hz."""
    dt = 1.0 / hz
    t_end = time.time() + duration_s
    while time.time() < t_end:
        send_actions(sock, car_id, vl, vr)
        time.sleep(dt)


def pattern(sock: socket.socket, car_id: int, speed: float, hz: float) -> None:
    """
    A simple sequence to test directions:
      1) Forward
      2) Backward
      3) Spin left
      4) Spin right
      5) Left wheel only
      6) Right wheel only
    """
    seq: Tuple[Tuple[str, float, float, float], ...] = (
        ("FORWARD",  +speed, +speed, 1.5),
        ("STOP",      0.0,    0.0,   0.6),
        ("BACKWARD", -speed, -speed, 1.5),
        ("STOP",      0.0,    0.0,   0.6),
        ("SPIN_L",   -speed, +speed, 1.2),
        ("STOP",      0.0,    0.0,   0.6),
        ("SPIN_R",   +speed, -speed, 1.2),
        ("STOP",      0.0,    0.0,   0.6),
        ("LEFT_ONLY", +speed,  0.0,  1.2),
        ("STOP",      0.0,    0.0,   0.6),
        ("RIGHT_ONLY", 0.0,  +speed, 1.2),
        ("STOP",      0.0,    0.0,   0.6),
    )

    for name, vl, vr, dur in seq:
        print(f"[test] {name:10s}  vl={vl:+.2f}  vr={vr:+.2f}  dur={dur:.1f}s")
        run_step(sock, car_id, vl, vr, dur, hz)

    # final stop
    stop(sock, car_id)


def main() -> int:
    ap = argparse.ArgumentParser(description="microMVP wheel direction test (TCP -> AP0)")
    ap.add_argument("--host", default="192.168.4.1")
    ap.add_argument("--port", type=int, default=9000)
    ap.add_argument("--car-id", type=int, default=0, help="CAR_ID on the firmware (AP0 uses 0)")
    ap.add_argument("--speed", type=float, default=0.35, help="wheel command magnitude in [0,1]")
    ap.add_argument("--hz", type=float, default=30.0, help="send rate (Hz). 20~50 recommended")
    ap.add_argument("--mode", choices=["pattern", "forward", "backward", "spinL", "spinR", "left", "right", "stop"],
                    default="pattern")
    ap.add_argument("--duration", type=float, default=2.0, help="duration for single-mode actions (seconds)")
    args = ap.parse_args()

    if not (0.0 <= abs(args.speed) <= 1.0):
        raise SystemExit("--speed must be within [0,1]")

    print(f"[test] connecting to {args.host}:{args.port} ...")
    with socket.create_connection((args.host, args.port), timeout=3.0) as sock:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        # Optional: read the hello line if present (non-blocking-ish)
        sock.settimeout(0.2)
        try:
            hello = sock.recv(4096)
            if hello:
                print("[test] recv:", hello.decode("utf-8", errors="replace").strip())
        except Exception:
            pass
        finally:
            sock.settimeout(None)

        if args.mode == "pattern":
            pattern(sock, args.car_id, args.speed, args.hz)
            print("[test] done.")
            return 0

        # Single-action modes
        if args.mode == "forward":
            vl, vr = +args.speed, +args.speed
        elif args.mode == "backward":
            vl, vr = -args.speed, -args.speed
        elif args.mode == "spinL":
            vl, vr = -args.speed, +args.speed
        elif args.mode == "spinR":
            vl, vr = +args.speed, -args.speed
        elif args.mode == "left":
            vl, vr = +args.speed, 0.0
        elif args.mode == "right":
            vl, vr = 0.0, +args.speed
        elif args.mode == "stop":
            vl, vr = 0.0, 0.0
        else:
            vl, vr = 0.0, 0.0

        print(f"[test] mode={args.mode}  vl={vl:+.2f} vr={vr:+.2f}  for {args.duration:.2f}s")
        run_step(sock, args.car_id, vl, vr, args.duration, args.hz)
        stop(sock, args.car_id)
        print("[test] done.")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
