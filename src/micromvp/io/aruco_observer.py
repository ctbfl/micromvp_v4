#从publisher接收角点并变成 2D Pose
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import zmq

from micromvp.core.transport import Pose


Point = Tuple[float, float]


@dataclass(slots=True)
class ObserverConfig:
    endpoint: str = "tcp://localhost:5556"
    stale_timeout_s: float = 0.5
    alpha: float = 0.3
    img_height: int = 720 
    theta_offset: float = 0.0
    homography: Optional[Tuple[Tuple[float, float, float],
                               Tuple[float, float, float],
                               Tuple[float, float, float]]] = None



class ArucoObserver:
    """
    Subscribe ZMQ messages from aruco_publisher.py:
      "<id> x0 y0 x1 y1 x2 y2 x3 y3"

    Compute:
      center = (p0 + p2)/2
      theta  = atan2( (mid_top - mid_bottom) ) or a stable edge-based heading
    Output:
      Dict[tag_id, Pose(x,y,theta)]
    """

    def __init__(self, endpoint: str, cfg: ObserverConfig | None = None) -> None:
        self._cfg = cfg or ObserverConfig(endpoint=endpoint)
        self._cfg.endpoint = endpoint

        self._lock = threading.Lock()
        self._poses: Dict[int, Pose] = {}
        self._last_seen: Dict[int, float] = {}

        self._running = False
        self._thread: Optional[threading.Thread] = None

        # ZMQ
        self._ctx = zmq.Context.instance()
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.setsockopt(zmq.SUBSCRIBE, b"")
        # small buffer helps reduce lag
        self._sub.setsockopt(zmq.RCVHWM, 5)

    def start(self) -> None:
        if self._thread is not None:
            return
        self._sub.connect(self._cfg.endpoint)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        try:
            self._sub.close(0)
        except Exception:
            pass

    def get_poses(self) -> Dict[int, Pose]:
        now = time.time()
        with self._lock:
            # drop stale
            if self._cfg.stale_timeout_s > 0:
                stale = [k for k, t in self._last_seen.items() if (now - t) > self._cfg.stale_timeout_s]
                for k in stale:
                    self._poses.pop(k, None)
                    self._last_seen.pop(k, None)
            return dict(self._poses)

    # ---------------- internal ----------------
    def _loop(self) -> None:
        poller = zmq.Poller()
        poller.register(self._sub, zmq.POLLIN)

        while self._running:
            events = dict(poller.poll(timeout=50))
            if self._sub not in events:
                continue

            try:
                msg = self._sub.recv_string(flags=zmq.NOBLOCK)
            except Exception:
                continue

            parsed = self._parse_msg(msg)
            if parsed is None:
                continue
            tag_id, corners = parsed

            pose = self._corners_to_pose(corners)

            # smoothing
            with self._lock:
                old = self._poses.get(tag_id)
                if old is not None and self._cfg.alpha > 0:
                    a = float(self._cfg.alpha)
                    x = (1 - a) * old.x + a * pose.x
                    y = (1 - a) * old.y + a * pose.y
                    # wrap angle carefully
                    theta = self._blend_angle(old.theta, pose.theta, a)
                    pose = Pose(x=x, y=y, theta=theta)

                self._poses[tag_id] = pose
                self._last_seen[tag_id] = time.time()

    @staticmethod
    def _parse_msg(msg: str) -> Optional[Tuple[int, Tuple[Point, Point, Point, Point]]]:
        parts = msg.strip().split()
        if len(parts) != 9:
            return None
        try:
            tag_id = int(parts[0])
            nums = [float(x) for x in parts[1:]]
        except ValueError:
            return None

        p0 = (nums[0], nums[1])
        p1 = (nums[2], nums[3])
        p2 = (nums[4], nums[5])
        p3 = (nums[6], nums[7])
        return tag_id, (p0, p1, p2, p3)

    def _apply_h(self, u: float, v: float) -> Tuple[float, float]:
        H = self._cfg.homography
        if H is None:
            return u, v
        h00, h01, h02 = H[0]
        h10, h11, h12 = H[1]
        h20, h21, h22 = H[2]
        x = h00 * u + h01 * v + h02
        y = h10 * u + h11 * v + h12
        w = h20 * u + h21 * v + h22
        if abs(w) < 1e-9:
            return u, v
        return x / w, y / w

    def _corners_to_pose(self, corners: Tuple[Point, Point, Point, Point]) -> Pose:
        """
        We assume corners are in OpenCV ArUco order. We'll derive:
          center: average of all corners
          heading: vector from mid-bottom edge to mid-top edge (or similar)
        Because corner ordering can be confusing, we use a robust method:
          - compute center
          - compute two edge midpoints: m01, m12, m23, m30
          - choose the longest opposite-midpoint axis as forward direction
        """
        (p0, p1, p2, p3) = corners

        # center (pixel)
        cu = (p0[0] + p1[0] + p2[0] + p3[0]) / 4.0
        cv = (p0[1] + p1[1] + p2[1] + p3[1]) / 4.0

        # 关键：像素坐标 y-down -> y-up
        # 你用的相机高度，和 publisher --height 一致
        cv = self._cfg.img_height - cv


        # map to world if homography provided
        x, y = self._apply_h(cu, cv)

        # edge midpoints (pixel)
        # top/bottom midpoints
        m01 = ((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0)
        m23 = ((p2[0] + p3[0]) / 2.0, (p2[1] + p3[1]) / 2.0)

        vx = m01[0] - m23[0]
        vy = m01[1] - m23[1]
        vy = -vy  # y-up

        import math
        theta = math.atan2(vy, vx)

        theta += float(self._cfg.theta_offset)
        theta = (theta + math.pi) % (2 * math.pi) - math.pi

        return Pose(x=float(x), y=float(y), theta=float(theta))


    @staticmethod
    def _blend_angle(a0: float, a1: float, alpha: float) -> float:
        import math
        # shortest angle difference
        d = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
        return a0 + alpha * d