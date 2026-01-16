Coordinator is a very flexible element to conduct as a bridge between environment, car controllers, GUI(User), and other high level code logic. 

It will register callback in GUI, so that it can handle the curve and point drawing in GUI.
It will also provide the 
It will  receive environmetn observation from env and distribute them to car controllers, then collect action from car controllers and give the action back to the environment to execute.

Some coordinator might also need to take care of the collision checking to make sure these cars will not collide into each other.



A main code sample would looks like:
```
def main():
    # --- 1. 环境初始化 ---
    # Env 是物理世界的唯一来源，它决定了所有的度量单位（如：1个单位 = 1像素）
    env = RealEnv() 
    ws_config: WorkspaceConfig = env.workspace_config

    # --- 2. 控制器初始化 ---
    # 为每辆小车创建一个独立的控制器实例
    ctrl_config = CarControllerConfig(lookahead_base_ratio=1.2)
    controllers: Dict[int, SomeController] = {}
    
    for robot_id in ws_config.car_ids:
        controllers[robot_id] = SomeController(robot_id, ctrl_config, ws_config)

    # --- 3. 协调器初始化 ---
    # Coordinator 负责全局调度，比如分配任务、避障逻辑
    coordinator = SomeCoordinator(ws_config, controllers)

    # --- 4. GUI 配置与初始化 (GUI 程序员重点关注) ---
    # GUI_config 决定了左侧面板生成哪些控件。
    # 每一个 'callback_name' 必须在后面通过 gui.register 进行绑定。
    gui_config = {
        "canvas": {
            "draw_curve_callback": True,  # 允许用户在画布上画线
            "click_car_callback": True    # 允许用户点击小车event sent back to coordinator. (by default user can always click a car to inspect its info on inspector)
        },
        "control_panel": [
            {"type": "label", "text": "=== 任务调度 ==="},
            {
                "type": "options", 
                "label": "选择阵型", 
                "options": ["Circle", "8-Shape", "Square"], 
                "default": "Circle",
                "callback_name": "cmd_change_pattern"
            },
            {
                "type": "continuous_slider", 
                "label": "全局巡航速度", 
                "range": [0.1, 1.0], 
                "default": 0.5,
                "callback_name": "cmd_set_speed"
            },
            {"type": "toggle", "label": "紧急避障", "default": True, "callback_name": "cmd_toggle_safety"},
            {"type": "input", "label": "发送指令", "placeholder": "输入调试代码...", "callback_name": "cmd_shell"}
        ]
    }

    # 实例化 GUI
    # GUI 需要 ws_config 来设置画布比例, gui_config to setup control panel ui
    gui = MVPWindow(gui_config, ws_config)

    # --- 5. 绑定回调接口 (桥接 GUI 与业务逻辑) ---
    # 当用户操作左侧面板控件时，GUI 会自动调用对应的 handler。
    # GUI 程序员应确保：UI 触发时传递的参数类型与 handler 接收的类型一致。
    gui.register_callback("cmd_change_pattern", coordinator.set_pattern)
    gui.register_callback("cmd_set_speed", coordinator.update_global_speed)
    gui.register_callback("cmd_toggle_safety", coordinator.toggle_safety)
    
    # 画布交互回调
    gui.register_callback("on_canvas_click", coordinator.handle_map_click)
    gui.register_callback("on_curve_drawn", coordinator.handle_user_path)

    # --- 6. 核心逻辑循环 (Logic Thread) ---
    def logic_loop():
        """
        这个线程以 WorkspaceConfig.frequency (如 20Hz) 运行。
        它是系统的心跳，负责 观测 -> 决策 -> 执行。
        """
        rate = ws_config.dt
        print(f"逻辑线程已启动，频率: {ws_config.frequency}Hz")
        
        while True:
            start_time = time.time()

            # A. 从环境获取原始观测
            obs_dict: Dict[int, RobotObservation] = env.get_observations()

            # B. 协调器处理观测，更新内部状态，并计算所有车的动作
            # 在 process 内部，各个控制器的 CarState 会被更新
            actions: Dict[int, Action] = coordinator.process(obs_dict)

            # C. 获取Car state and 额外的绘图指令 (UUID 机制)
            world_state = coordinator.gather_car_state() # GUI 程序员注意：这里返回的 list[dict] 将驱动右侧画布的动态显示
            additional_drawings = coordinator. get_additional_drawings() # here refer to src/micromvp/gui/readme.md to get the actual data format to communicate with GUI.

            # D. 推送数据快照给 GUI 进行渲染
            # 注意：此处 gui.update 是非阻塞的，它只是更新 GUI 的内部缓存
            gui.update(world_state, additional_drawings)

            # E. 执行动作
            env.apply_actions(actions)

            # 控制频率
            elapsed = time.time() - start_time
            time.sleep(max(0, rate - elapsed))

    # 启动逻辑子线程
    logic_thread = threading.Thread(target=logic_loop, daemon=True)
    logic_thread.start()

    # --- 7. 启动 GUI 主循环 (Main Thread) ---
    # 在 PyQt6 中，GUI 必须在主线程运行。
    # 它会阻塞在这里，直到用户关闭窗口。
    gui.run()

```

The `process()` function, and the `gather_car_state()` function must be accomplished. For the  `get_additional_drawings() ` is optional, only implemnet when the coordinator need. But that definitely need to be write in the abstract class. Let the implementation pass the function if no use.
