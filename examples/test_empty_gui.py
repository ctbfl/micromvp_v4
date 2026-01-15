"""
Only include GUI.


MicroMVP GUI 功能测试脚本
该脚本用于在没有真实环境和控制器的情况下，独立测试 GUI 模块的所有功能。
测试覆盖：
1. 动态控制面板控件生成与回调。
2. 小车状态渲染（包含 ID 标签与方向）。
3. UUID 绘图系统（创建、修改、删除）。
4. 坐标系转换（Workspace -> Pixels）。
"""

import sys
import os
import time
import math
import random
import threading
from typing import Dict, List

# 将 src 目录添加到路径，确保能导入 micromvp
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))

from micromvp.core.models import WorkspaceConfig, CarState, Point
from micromvp.gui import MVPWindow

def run_test():
    # --- 1. 初始化工作空间配置 ---
    # 定义一个 800x600 的空间，小车尺寸 40x60，轴心偏移为中心点
    ws_config = WorkspaceConfig(
        width=800.0,
        height=600.0,
        car_width=40.0,
        car_height=40.0,
        offset_w=20.0,
        offset_h=20.0,
        wheel_base=35.0,
        max_wheel_speed=100.0,
        frequency=30.0, # 30Hz 更新率
        car_id_list=[1, 2, 7] # 测试不连续的 ID
    )

    # --- 2. 定义 GUI 控件配置 ---
    gui_config = {
        "canvas": {
            "draw_curve_callback": True,
            "click_canvas_callback": False
        },
        "control_panel": [
            {"type": "label", "text": "=== 模拟测试控制 ==="},
            {"type": "toggle", "label": "小车移动", "default": True, "callback_name": "toggle_movement"},
            {"type": "continuous_slider", "label": "旋转速度", "range": [0.1, 5.0], "default": 1.0, "callback_name": "set_rot_speed"},
            {"type": "discrete_slider", "label": "绘图数量", "tiers": [0, 5, 10, 20], "default_idx": 1, "callback_name": "set_draw_count"},
            {"type": "options", "label": "绘图颜色", "options": ["red", "green", "blue", "yellow"], "default": "red", "callback_name": "change_color"},
            {"type": "input", "label": "控制台指令", "placeholder": "在此输入...", "callback_name": "console_cmd"}
        ]
    }

    # --- 3. 模拟系统状态变量 ---
    state_lock = threading.Lock()
    sim_vars = {
        "running": True,
        "rot_speed": 1.0,
        "draw_count": 5,
        "draw_color": "red",
        "t": 0.0
    }

    # 初始化模拟小车状态
    car_states: Dict[int, CarState] = {}
    for cid in ws_config.car_id_list:
        car_states[cid] = CarState(car_id=cid, x=400, y=300, theta=0)

    # --- 4. 定义回调函数 ---
    def handle_toggle(val: bool):
        sim_vars["running"] = val
        print(f"[GUI Callback] Movement toggled: {val}")

    def handle_rot(val: float):
        sim_vars["rot_speed"] = val
        print(f"[GUI Callback] Rotation speed set to: {val}")

    def handle_draw_count(val: int):
        sim_vars["draw_count"] = val
        print(f"[GUI Callback] Drawing count: {val}")

    def handle_color(val: str):
        sim_vars["draw_color"] = val
        print(f"[GUI Callback] Color changed: {val}")

    def handle_click(cid: int):
        print(f"[GUI Callback] Car {cid} clicked! Updating Inspector.")
        # 这里逻辑会自动触发 Inspector 更新，因为 MVPWindow 持有引用

    def handle_canvas_click(x, y):
        print(f"[GUI Event] Canvas clicked at Workspace coordinates: ({x:.1f}, {y:.1f})")

    def handle_curve(points: List[Point]):
        print(f"[GUI Event] User drew a curve with {len(points)} points.")

    def handle_cmd(string_cmd: str):
        print(f"[GUI Command] Received console command: {string_cmd}")

    # --- 5. 实例化窗口并注册回调 ---
    # 注意：此时还没开始逻辑循环
    gui = MVPWindow(gui_config, ws_config)
    
    gui.register_callback("toggle_movement", handle_toggle)
    gui.register_callback("set_rot_speed", handle_rot)
    gui.register_callback("set_draw_count", handle_draw_count)
    gui.register_callback("change_color", handle_color)
    gui.register_callback("on_car_click", handle_click)
    gui.register_callback("on_canvas_click", handle_canvas_click)
    gui.register_callback("on_curve_drawn", handle_curve)
    gui.register_callback("console_cmd", handle_cmd)

    # --- 6. 核心逻辑模拟循环 (仿真逻辑线程) ---
    def logic_thread():
        dt = ws_config.dt
        print("Starting mock logic loop...")
        
        while True:
            start_time = time.time()
            
            with state_lock:
                if sim_vars["running"]:
                    sim_vars["t"] += dt
                
                t = sim_vars["t"]
                
                # 更新小车位置（让它们绕圆圈跑）
                for i, cid in enumerate(ws_config.car_id_list):
                    radius = 100 + i * 50
                    cx, cy = 400, 300
                    car_states[cid].x = cx + radius * math.cos(t * sim_vars["rot_speed"] + i)
                    car_states[cid].y = cy + radius * math.sin(t * sim_vars["rot_speed"] + i)
                    dx = car_states[cid].x - cx
                    dy = car_states[cid].y - cy
                    car_states[cid].theta = ((t * sim_vars["rot_speed"] + i + math.pi/2) % (2 * math.pi)) * 180 / math.pi
                    
                    # 模拟一些 Metadata
                    car_states[cid].metadata = {
                        "Target": f"WP_{int(t) % 10}",
                        "Battery": f"{max(0, 100 - t/10):.1f}%",
                        "Path_Length": len(ws_config.car_id_list)
                    }

                # 生成额外的 UUID 绘图（模拟动态变化的数据）
                drawings = []
                
                # 1. 绘制一个固定的中心圆（测试持久 UUID）
                drawings.append({
                    "uuid": "center_marker",
                    "type": "circle",
                    "center": (400, 300),
                    "radius": 10,
                    "color": "orange"
                })

                # 2. 动态生成一些点（测试 UUID 创建与销毁）
                for i in range(sim_vars["draw_count"]):
                    angle = (t * 2 + i * (2 * math.pi / 20)) 
                    dist = 200 + 20 * math.sin(t * 5)
                    px = 400 + dist * math.cos(angle)
                    py = 300 + dist * math.sin(angle)
                    drawings.append({
                        "uuid": f"dynamic_dot_{i}",
                        "type": "point",
                        "position": (px, py),
                        "radius": 4,
                        "color": sim_vars["draw_color"]
                    })

                # 3. 绘制连接小车的线段
                if len(ws_config.car_id_list) >= 2:
                    c1 = car_states[ws_config.car_id_list[0]]
                    c2 = car_states[ws_config.car_id_list[1]]
                    drawings.append({
                        "uuid": "car_linker",
                        "type": "line",
                        "start": (c1.x, c1.y),
                        "end": (c2.x, c2.y),
                        "color": "gray",
                        "width": 1
                    })

            # 推送快照给 GUI
            # 注意：gui.update 是线程安全的，内部使用信号处理
            gui.update(car_states, drawings)

            # 维持频率
            elapsed = time.time() - start_time
            time.sleep(max(0, dt - elapsed))

    # 启动后台逻辑线程
    t = threading.Thread(target=logic_thread, daemon=True)
    t.start()

    # --- 7. 启动 GUI (阻塞) ---
    print("Launching GUI...")
    gui.run()

if __name__ == "__main__":
    run_test()