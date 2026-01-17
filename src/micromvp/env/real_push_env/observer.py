"""
ArUco-based camera observer for real robot pose detection.

Fix:
- Background thread NEVER calls any cv2 HighGUI (imshow/waitKey/namedWindow/destroyWindow).
- Preview window (if enabled) is rendered ONLY from the GUI main thread via render().
- Warmup is silent (no image display).
"""

from __future__ import annotations

import platform
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

# -----------------------------
# Workspace marker layout (cm)
# -----------------------------
WORKSPACE_W_CM = 40.0
WORKSPACE_H_CM = 60.0
MARKER_SIZE_CM = 4.0

ID_2_COORDINATE = {
    8: (0, 0),
    10: (36, 0),
    7: (18, 7),
    0: (10, 20),
    2: (26, 20),
    4: (0, 28),
    6: (36, 28),
    1: (10, 36),
    3: (26, 36),
    5: (18, 49),
    9: (0, 56),
    11: (36, 56),
}


@dataclass
class ObserverConfig:
    # Camera
    camera_device: int = 0
    resolution: str = "720p"
    fps: int = 60
    undistort: bool = False

    marker_center_to_wheel_center_offset_cm: list = (0.0, 0.0)  # (x_cm, y_cm), we want observer to give back wheel center, not marker position

    # Calibration
    calibration_file: str = "/home/omen/junshan/micromvp_push/camera/config/camera.yaml"

    # ArUco dicts
    workspace_dict: str = "DICT_5X5_50"
    car_dict: str = "DICT_4X4_50"

    # Marker size
    car_marker_size_mm: float = 36.0

    # Workspace pose estimation
    ws_reproj_err_px: float = 5.0
    min_ws_inliers: int = 12

    # Warmup (silent)
    warmup_frames: int = 30

    # Preview control
    no_preview: bool = False
    preview_window_name: str = "ArucoObserver"
    draw_axis: bool = True
    axis_length_m: float = 0.03



@dataclass
class CarObservation:
    car_id: int
    x_cm: float
    y_cm: float
    yaw_deg: float
    timestamp: float


class ArucoObserver:
    """
    Threading contract:
    - start(): safe to call from GUI main thread
    - _run_loop(): background thread compute only
    - render(): MUST be called from GUI main thread if preview enabled
    - stop(): ideally from GUI main thread (so destroyWindow is safe)
    """

    def __init__(self, config: ObserverConfig) -> None:
        self._config = config
        self._running = False

        # Camera and calibration
        self._cap: Optional[cv2.VideoCapture] = None
        self._K: Optional[np.ndarray] = None
        self._D: Optional[np.ndarray] = None

        # ArUco detectors
        self._ws_detector: Optional[cv2.aruco.ArucoDetector] = None
        self._car_detector: Optional[cv2.aruco.ArucoDetector] = None
        self._car_marker_len_m: float = config.car_marker_size_mm / 1000.0
        self._marker_pts_m: Optional[np.ndarray] = None

        # Workspace pose cache
        self._rvec_ws: Optional[np.ndarray] = None
        self._tvec_ws: Optional[np.ndarray] = None
        self._R_ws_cam: Optional[np.ndarray] = None
        self._t_ws_cam: Optional[np.ndarray] = None
        self._ws_fixed = False

        # Observations
        self._observations: Dict[int, CarObservation] = {}
        self._obs_lock = threading.Lock()

        # Thread
        self._thread: Optional[threading.Thread] = None

        # Preview frame buffer (produced by bg thread, consumed by main thread)
        self._vis_frame: Optional[np.ndarray] = None
        self._vis_lock = threading.Lock()
        self._window_inited = False  # only touched in main thread

        # Optional callback (IMPORTANT: do not touch Qt here!)
        self._frame_callback: Optional[Callable[[np.ndarray], None]] = None

    # -------------------------
    # Public APIs
    # -------------------------
    def start(self) -> bool:
        if self._running:
            return True

        try:
            self._K, self._D, _ = self._load_calibration(self._config.calibration_file)
        except Exception as e:
            print(f"[ArucoObserver] Failed to load calibration: {e}")
            return False

        self._cap = self._open_camera()
        if self._cap is None or not self._cap.isOpened():
            print("[ArucoObserver] Failed to open camera")
            return False

        self._setup_detectors()
        self._marker_pts_m = self._marker_corners_in_marker_frame(self._car_marker_len_m)

        # Warmup is silent: only compute best workspace pose
        if self._config.warmup_frames > 0:
            ok = self._run_warmup_silent()
            self._ws_fixed = bool(ok)
            if not ok:
                print("[ArucoObserver] Warmup failed -> switch to dynamic mode")
        else:
            self._ws_fixed = False

        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

        mode = "WARMUP-FIXED" if self._ws_fixed else "DYNAMIC"
        print(f"[ArucoObserver] Started in {mode} mode (no_preview={self._config.no_preview})")
        return True

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

        # HighGUI cleanup MUST be main thread safe: only do it if preview is enabled.
        if not self._config.no_preview and self._window_inited:
            try:
                cv2.destroyWindow(self._config.preview_window_name)
            except Exception:
                pass
            self._window_inited = False

    def get_observations(self) -> Dict[int, CarObservation]:
        with self._obs_lock:
            return dict(self._observations)

    def set_frame_callback(self, callback: Callable[[np.ndarray], None]) -> None:
        self._frame_callback = callback

    def is_workspace_ready(self) -> bool:
        return (self._rvec_ws is not None) and (self._tvec_ws is not None)

    def render(self) -> None:
        """
        Preview rendering: MUST be called from GUI main thread (Qt main thread).
        If no_preview=True, does nothing.
        """
        if self._config.no_preview:
            return
        # Init window once (in main thread)
        if not self._window_inited:
            try:
                cv2.namedWindow(self._config.preview_window_name, cv2.WINDOW_NORMAL)
            except Exception:
                # If HighGUI is not available, just disable preview silently
                print("[ArucoObserver] Warning: cv2.namedWindow failed, disabling preview.")
                self._config.no_preview = True
                return
            self._window_inited = True

        frame = None
        with self._vis_lock:
            if self._vis_frame is not None:
                frame = self._vis_frame.copy()

        if frame is None:
            return

        cv2.imshow(self._config.preview_window_name, frame)
        # waitKey MUST also be main thread
        cv2.waitKey(1)

    # -------------------------
    # Camera / calibration
    # -------------------------
    def _open_camera(self) -> Optional[cv2.VideoCapture]:
        resolutions = {"480p": (640, 480), "720p": (1280, 720), "1080p": (1920, 1080)}
        width, height = resolutions.get(self._config.resolution, (1280, 720))
        system = platform.system()
        if system == "Linux":
            cap = cv2.VideoCapture(self._config.camera_device, cv2.CAP_V4L2)
        elif system == "Windows":
            cap = cv2.VideoCapture(self._config.camera_device, cv2.CAP_DSHOW)
        else:
            cap = cv2.VideoCapture(self._config.camera_device)

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, self._config.fps)
        # NOTE: exposure is camera-dependent; keep your old setting if you want
        cap.set(cv2.CAP_PROP_EXPOSURE, -6)
        return cap

    def _load_calibration(self, file_path: str) -> Tuple[np.ndarray, np.ndarray, Tuple[int, int]]:
        with open(file_path, "r") as f:
            calib = yaml.safe_load(f)
        K = np.array(calib["camera_matrix"], dtype=np.float32)
        D = np.array(calib["distortion_coefficients"], dtype=np.float32)
        size = tuple(calib["image_size"][:2])
        return K, D, size

    def _setup_detectors(self) -> None:
        aruco = cv2.aruco
        dicts = {
            "DICT_4X4_50": aruco.DICT_4X4_50,
            "DICT_5X5_50": aruco.DICT_5X5_50,
        }
        ws_dict = aruco.getPredefinedDictionary(dicts.get(self._config.workspace_dict, aruco.DICT_5X5_50))
        car_dict = aruco.getPredefinedDictionary(dicts.get(self._config.car_dict, aruco.DICT_4X4_50))
        params = aruco.DetectorParameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self._ws_detector = aruco.ArucoDetector(ws_dict, params)
        self._car_detector = aruco.ArucoDetector(car_dict, params)

    def _marker_corners_in_marker_frame(self, marker_len_m: float) -> np.ndarray:
        h = marker_len_m / 2.0
        return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float32)

    # -------------------------
    # Workspace pose estimation
    # -------------------------
    def _marker_workspace_corners_cm(self, marker_id: int) -> Optional[np.ndarray]:
        if marker_id not in ID_2_COORDINATE:
            return None
        x, y = ID_2_COORDINATE[marker_id]
        s = MARKER_SIZE_CM
        return np.array([[x, y], [x, y + s], [x + s, y + s], [x + s, y]], dtype=np.float32)

    def _build_workspace_correspondences(
        self, corners: List, ids: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, List[int]]:
        obj_pts, img_pts, used_ids = [], [], []
        if ids is None:
            return np.zeros((0, 3), np.float32), np.zeros((0, 2), np.float32), used_ids

        for i, mid in enumerate(ids.flatten().tolist()):
            ws_corners_cm = self._marker_workspace_corners_cm(mid)
            if ws_corners_cm is None:
                continue
            ws_corners_m = np.hstack([ws_corners_cm / 100.0, np.zeros((4, 1), np.float32)])
            img_corners = corners[i].reshape(4, 2).astype(np.float32)
            obj_pts.append(ws_corners_m)
            img_pts.append(img_corners)
            used_ids.append(mid)

        if len(obj_pts) == 0:
            return np.zeros((0, 3), np.float32), np.zeros((0, 2), np.float32), used_ids

        return np.vstack(obj_pts).astype(np.float32), np.vstack(img_pts).astype(np.float32), used_ids

    def _solve_workspace_pose(
        self, obj_pts: np.ndarray, img_pts: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], int]:
        if obj_pts.shape[0] < 8:
            return None, None, 0
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            objectPoints=obj_pts,
            imagePoints=img_pts,
            cameraMatrix=self._K,
            distCoeffs=self._D,
            reprojectionError=self._config.ws_reproj_err_px,
            confidence=0.999,
            iterationsCount=200,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if (not ok) or (rvec is None) or (inliers is None):
            return None, None, 0
        return rvec.astype(np.float32), tvec.astype(np.float32), int(inliers.shape[0])

    def _invert_rt(self, rvec: np.ndarray, tvec: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        R, _ = cv2.Rodrigues(rvec)
        R_inv = R.T
        t_inv = -R_inv @ tvec.reshape(3, 1)
        return R_inv.astype(np.float32), t_inv.astype(np.float32)

    def _estimate_workspace_pose(self, gray: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], int]:
        ws_corners, ws_ids, _ = self._ws_detector.detectMarkers(gray)
        if ws_ids is None or len(ws_ids) == 0:
            return None, None, 0
        obj_pts, img_pts, _ = self._build_workspace_correspondences(ws_corners, ws_ids)
        return self._solve_workspace_pose(obj_pts, img_pts)

    def _run_warmup_silent(self) -> bool:
        N = int(self._config.warmup_frames)
        best_inliers = -1
        best_rvec = None
        best_tvec = None

        print(f"[ArucoObserver] Warmup (silent) for {N} frames...")
        for _ in range(N):
            ret, frame = self._cap.read()
            if not ret:
                continue
            if self._config.undistort:
                frame = cv2.undistort(frame, self._K, self._D)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            rvec, tvec, inliers = self._estimate_workspace_pose(gray)
            if rvec is not None and inliers > best_inliers:
                best_inliers = inliers
                best_rvec = rvec
                best_tvec = tvec

        if best_rvec is None:
            return False

        self._rvec_ws, self._tvec_ws = best_rvec, best_tvec
        self._R_ws_cam, self._t_ws_cam = self._invert_rt(self._rvec_ws, self._tvec_ws)
        print(f"[ArucoObserver] Warmup OK (best inliers={best_inliers})")
        return True

    # -------------------------
    # Car detection
    # -------------------------
    def _convert_car_info(self, corners_cm: np.ndarray) -> Tuple[np.ndarray, float]:
        center = corners_cm.mean(axis=0)
        v = 0.5 * (corners_cm[0] + corners_cm[1]) - 0.5 * (corners_cm[2] + corners_cm[3])
        yaw = float(np.degrees(np.arctan2(v[1], v[0])))
        return center, (yaw + 360.0) % 360.0

    def _detect_cars(self, gray: np.ndarray, frame_for_vis: Optional[np.ndarray]) -> Dict[int, CarObservation]:
        observations: Dict[int, CarObservation] = {}
        timestamp = time.time()

        car_corners, car_ids, _ = self._car_detector.detectMarkers(gray)
        if car_ids is None or len(car_ids) == 0:
            return observations

        # Pose for each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            car_corners, self._car_marker_len_m, self._K, self._D
        )

        for i, mid in enumerate(car_ids.flatten().tolist()):
            car_id = int(mid)
            rvec_car = rvecs[i, 0, :]
            tvec_car = tvecs[i, 0, :].reshape(3, 1)

            R_car, _ = cv2.Rodrigues(rvec_car)
            corners_cam = (R_car @ self._marker_pts_m.T + tvec_car).T  # (4,3) in cam
            corners_ws_m = (self._R_ws_cam @ corners_cam.T + self._t_ws_cam).T
            corners_cm = (corners_ws_m * 100.0)[:, :2]
            center_cm, yaw_deg = self._convert_car_info(corners_cm)

            observations[car_id] = CarObservation(
                car_id=car_id,
                x_cm=float(center_cm[0])+self._config.marker_center_to_wheel_center_offset_cm[0],
                y_cm=float(center_cm[1])+self._config.marker_center_to_wheel_center_offset_cm[1],
                yaw_deg=float(yaw_deg),
                timestamp=timestamp,
            )

            # Debug drawing only on local vis frame (still in bg thread, but only drawing ops, no HighGUI!)
            if frame_for_vis is not None:
                if self._config.draw_axis:
                    cv2.drawFrameAxes(frame_for_vis, self._K, self._D, rvec_car, tvec_car, self._config.axis_length_m)
                pts_px = car_corners[i].reshape(4, 2).mean(axis=0)
                self._put_text(frame_for_vis, f"{car_id}: {center_cm[0]:.0f},{center_cm[1]:.0f}",
                               (int(pts_px[0]), int(pts_px[1]) + 20))

        # Also draw detected markers on vis frame if enabled
        if frame_for_vis is not None:
            cv2.aruco.drawDetectedMarkers(frame_for_vis, car_corners, car_ids)

        return observations

    # -------------------------
    # Background loop (NO HighGUI)
    # -------------------------
    def _run_loop(self) -> None:
        while self._running:
            ret, frame = self._cap.read()
            if not ret:
                time.sleep(0.005)
                continue

            if self._config.undistort:
                frame = cv2.undistort(frame, self._K, self._D)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # If not fixed, update workspace pose dynamically
            if not self._ws_fixed:
                rvec, tvec, inliers = self._estimate_workspace_pose(gray)
                if rvec is not None and inliers >= self._config.min_ws_inliers:
                    self._rvec_ws, self._tvec_ws = rvec, tvec
                    self._R_ws_cam, self._t_ws_cam = self._invert_rt(rvec, tvec)

            ws_ok = (self._rvec_ws is not None) and (self._tvec_ws is not None)

            # Prepare a visualization frame (draw only; still no HighGUI here)
            vis = None
            if not self._config.no_preview:
                vis = frame.copy()

            if ws_ok:
                if vis is not None:
                    self._draw_workspace_boundary(vis, self._rvec_ws, self._tvec_ws)
                observations = self._detect_cars(gray, vis)
                with self._obs_lock:
                    self._observations = observations
            else:
                with self._obs_lock:
                    self._observations = {}

            if vis is not None:
                msg = f"Cars: {len(self._observations)}" if ws_ok else "WS Not Ready"
                self._put_text(vis, msg, (10, 30))
                with self._vis_lock:
                    self._vis_frame = vis  # already a copy

            # Optional callback (never touch Qt UI inside it!)
            if self._frame_callback is not None:
                try:
                    self._frame_callback(frame)
                except Exception:
                    pass

    # -------------------------
    # Drawing utils (no HighGUI)
    # -------------------------
    def _draw_workspace_boundary(self, frame: np.ndarray, rvec: np.ndarray, tvec: np.ndarray) -> None:
        boundary_cm = np.array(
            [[0, 0, 0],
             [0, WORKSPACE_H_CM, 0],
             [WORKSPACE_W_CM, WORKSPACE_H_CM, 0],
             [WORKSPACE_W_CM, 0, 0]],
            dtype=np.float32,
        )
        img_pts, _ = cv2.projectPoints(boundary_cm / 100.0, rvec, tvec, self._K, self._D)
        cv2.polylines(frame, [img_pts.astype(np.int32)], True, (0, 255, 255), 3)

    def _put_text(self, img: np.ndarray, text: str, org: Tuple[int, int], scale: float = 0.6) -> None:
        cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 255, 0), 2)
