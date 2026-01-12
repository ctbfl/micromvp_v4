import cv2
import numpy as np

# 1. 设置字典
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

marker_id = 2
marker_size_px = 800

# 2. 生成标记图像
marker_img = cv2.aruco.generateImageMarker(
    aruco_dict,
    marker_id,
    marker_size_px
)

# 3. 保存到指定目录
# 注意：在 Linux 中路径使用正斜杠 "/"
save_path = "/home/rain/Downloads/aruco_4x4_50_id2.png"

# 执行保存
success = cv2.imwrite(save_path, marker_img)

if success:
    print(f"成功！文件已保存到: {save_path}")
else:
    print("保存失败，请检查文件夹权限或路径是否存在。")