#!/usr/bin/env python3
#它从相机读图 → 用 OpenCV ArUco 只检测角点 → 把每个 tag 的 4 个角点像素坐标通过 ZMQ PUB 发出去。
from __future__ import annotations

import argparse
import time
from typing import Optional, Tuple, List

import cv2
import zmq


DICT_MAP = {
    "4X4_50": cv2.aruco.DICT_4X4_50,
    "4X4_100": cv2.aruco.DICT_4X4_100,
    "4X4_250": cv2.aruco.DICT_4X4_250,
    "4X4_1000": cv2.aruco.DICT_4X4_1000,
    "5X5_50": cv2.aruco.DICT_5X5_50,
    "5X5_100": cv2.aruco.DICT_5X5_100,
    "5X5_250": cv2.aruco.DICT_5X5_250,
    "5X5_1000": cv2.aruco.DICT_5X5_1000,
    "6X6_50": cv2.aruco.DICT_6X6_50,
    "6X6_100": cv2.aruco.DICT_6X6_100,
    "6X6_250": cv2.aruco.DICT_6X6_250,
    "6X6_1000": cv2.aruco.DICT_6X6_1000,
    "7X7_50": cv2.aruco.DICT_7X7_50,
    "7X7_100": cv2.aruco.DICT_7X7_100,
    "7X7_250": cv2.aruco.DICT_7X7_250,
    "7X7_1000": cv2.aruco.DICT_7X7_1000,
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="ArUco -> ZMQ publisher (corners only)")
    p.add_argument("--bind", default="tcp://*:5556", help="ZMQ PUB bind endpoint, e.g. tcp://*:5556")
    p.add_argument("--camera", default="0",
                   help="Camera index (e.g. 0/1) or device path (e.g. /dev/video1 or /dev/v4l/by-id/xxx)")
    p.add_argument("--dict", default="4X4_50", choices=sorted(DICT_MAP.keys()),
                   help="Aruco dictionary type (must match printed tags)")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=float, default=30.0, help="Target publish FPS (best-effort)")
    p.add_argument("--show", action="store_true", help="Show debug window with detected markers")
    p.add_argument("--no-draw", action="store_true", help="Do not draw overlays when --show")
    p.add_argument("--min-perimeter-rate", type=float, default=0.03,
                   help="DetectorParameters.minMarkerPerimeterRate")
    p.add_argument("--max-perimeter-rate", type=float, default=4.0,
                   help="DetectorParameters.maxMarkerPerimeterRate")
    return p.parse_args()


def open_camera(camera_arg: str) -> cv2.VideoCapture:
    # camera_arg can be "0" / "1" or "/dev/video1" etc.
    cap: cv2.VideoCapture
    if camera_arg.isdigit():
        cap = cv2.VideoCapture(int(camera_arg))
    else:
        cap = cv2.VideoCapture(camera_arg)

    if not cap.isOpened():
        raise RuntimeError(f"Failed to open camera: {camera_arg}")

    return cap


def main() -> int:
    args = parse_args()

    # ZMQ PUB
    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.bind(args.bind)

    # Camera
    cap = open_camera(args.camera)
    # Best-effort settings (some cameras ignore)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(args.width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(args.height))
    cap.set(cv2.CAP_PROP_FPS, float(args.fps))

    # ArUco detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_MAP[args.dict])
    params = cv2.aruco.DetectorParameters()
    params.minMarkerPerimeterRate = float(args.min_perimeter_rate)
    params.maxMarkerPerimeterRate = float(args.max_perimeter_rate)
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    print(f"[vision] PUB bind: {args.bind}")
    print(f"[vision] Camera: {args.camera}  dict={args.dict}  show={args.show}")
    print("[vision] Message format: tag_id x0 y0 x1 y1 x2 y2 x3 y3")

    target_dt = 1.0 / max(args.fps, 1e-6)
    next_t = time.time()

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue

            corners, ids, _rejected = detector.detectMarkers(frame)

            # Publish each detected marker as one message
            if ids is not None and len(ids) > 0:
                ids_list = ids.flatten().tolist()

                # corners: list of (1,4,2) float arrays
                for tag_id, c in zip(ids_list, corners):
                    pts = c.reshape(4, 2)  # order: [tl, tr, br, bl] in OpenCV ArUco
                    # Your ArucoObserver expects 4 corners (x0,y0 ... x3,y3)
                    msg = (
                        f"{int(tag_id)} "
                        f"{pts[0,0]:.3f} {pts[0,1]:.3f} "
                        f"{pts[1,0]:.3f} {pts[1,1]:.3f} "
                        f"{pts[2,0]:.3f} {pts[2,1]:.3f} "
                        f"{pts[3,0]:.3f} {pts[3,1]:.3f}"
                    )
                    pub.send_string(msg)

                if args.show and not args.no_draw:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            if args.show:
                cv2.imshow("vision_aruco_zmq_publisher", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break

            # Best-effort pacing
            now = time.time()
            if now < next_t:
                time.sleep(max(0.0, next_t - now))
            next_t += target_dt

    finally:
        cap.release()
        if args.show:
            cv2.destroyAllWindows()
        pub.close(0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
