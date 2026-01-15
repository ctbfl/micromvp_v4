#检测摄像头中的 ArUco 标签并显示其 ID

import cv2
import numpy as np

def main():
    cap = cv2.VideoCapture(2)  # 0 通常是默认摄像头
    if not cap.isOpened():
        raise RuntimeError("摄像头打开失败：请检查 /dev/video* 或权限")

    # 选择字典：你贴在车上的标签必须和这里一致
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        corners, ids, rejected = detector.detectMarkers(frame)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            # 打印检测到的所有 id
            print("Detected IDs:", ids.flatten().tolist())

        cv2.imshow("aruco", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):  # ESC/q退出
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
