import sys
import os
import time
import math
import random
import threading
from typing import Dict, List


from micromvp.core.models import WorkspaceConfig, CarState, Point, Action, RobotObservation
from micromvp.gui import MVPWindow
from micromvp.env import SimEnv, SimConfig



# setup env
sim_config = SimConfig(
    width=400, height=600,
    initial_poses=[(1, 100, 100, 0), (2, 200, 200, 90)]
)
env = SimEnv(sim_config)
ws_config = env.workspace_config

# setup gui
gui_config = {
    "canvas": {
        "draw_curve_callback": True,
        "click_canvas_callback": False
    },
    "control_panel": [
        {"type": "label", "text": "Simulation Env Test"},
        # {"type": "toggle", "label": "小车移动", "default": True, "callback_name": "toggle_movement"},
        # {"type": "continuous_slider", "label": "旋转速度", "range": [0.1, 5.0], "default": 1.0, "callback_name": "set_rot_speed"},
        # {"type": "discrete_slider", "label": "绘图数量", "tiers": [0, 5, 10, 20], "default_idx": 1, "callback_name": "set_draw_count"},
        # {"type": "options", "label": "绘图颜色", "options": ["red", "green", "blue", "yellow"], "default": "red", "callback_name": "change_color"},
        {"type": "input", "label": "控制台指令", "placeholder": "在此输入...", "callback_name": "console_cmd"}
    ]
}
gui = MVPWindow(gui_config, ws_config)

# main loop
def logic_loop():
    while True:
        observation = env.step({0: Action.stop(), 1: Action.stop()})
        car_states = {obs.robot_id: CarState(
            car_id=obs.robot_id,
            x=obs.x, 
            y=obs.y,
            theta=obs.theta,
            status_label="SIM_IDLE",
            metadata = {"timestamp": obs.timestamp}
        ) for obs in observation.values()}
        gui.update(car_states, additional_drawings=None)
        time.sleep(1/ws_config.frequency)

threading.Thread(target=logic_loop, daemon=True).start()

# run gui
gui.run()