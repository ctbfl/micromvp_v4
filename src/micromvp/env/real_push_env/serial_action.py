"""
Serial action sender for real robot control (Aggregated Protocol).

Sends wheel speed commands to multiple robots via a single Serial Port.

Protocol Structure (Aggregated):
-----------------------------------------------------------------------------------------
| Header (2B) | Seq (2B) | Count (1B) | Body (N * 5B)             | Checksum (1B)       |
-----------------------------------------------------------------------------------------
| 0xAA 0x55   | uint16   | uint8      | [ID(1B) L(2B) R(2B)] ...  | Sum(Seq..Body) & FF |
-----------------------------------------------------------------------------------------

- Speed values are in range [-1000, 1000] representing normalized [-1.0, 1.0].
- Robot ID is uint8 (0-255).
"""
from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple, List, Union

import serial  # pip install pyserial
from micromvp.core.models import Action


@dataclass
class SerialActionConfig:
    """Configuration for Serial action sender."""
    # 串口配置
    port: str = "/dev/ttyACM0"
    baudrate: int = 115200
    
    # 发送频率 (Hz)
    send_hz: float = 30.0

    # 是否反转右轮 (部分电机接线相反)
    invert_right_wheel: bool = False

    # 预设的活跃 Robot ID 列表 (用于初始化)
    # 串口是广播总线，如果你只控制 ID 1 和 2，可以在这里指定
    initial_robot_ids: List[int] = field(default_factory=list)


class SerialActionSender:
    """
    Serial-based action sender.
    Replaces the original UDPActionSender while keeping the API compatible.
    """

    def __init__(self, config: SerialActionConfig) -> None:
        self._config = config
        self._running = False
        
        # 串口对象
        self._ser: Optional[serial.Serial] = None
        self._serial_lock = threading.Lock()  # 保护串口写入，防止多线程冲突

        # 状态管理
        # robot_id -> Action
        self._actions: Dict[int, Action] = {}
        # 追踪活跃的 Robot ID (替代原来的 Endpoint 列表)
        self._active_robot_ids: set = set(config.initial_robot_ids)
        
        self._action_lock = threading.Lock() # 保护 _actions 字典

        # 全局序列号 (所有机器人共用一个时间轴)
        self._global_seq: int = 0

        # 后台线程
        self._thread: Optional[threading.Thread] = None

    def start(self) -> bool:
        """Start the serial connection and sender thread."""
        if self._running:
            return True

        try:
            self._ser = serial.Serial(
                port=self._config.port,
                baudrate=self._config.baudrate,
                timeout=0.1,
                write_timeout=0.1
            )
            print(f"[SerialSender] Port {self._config.port} opened at {self._config.baudrate}")
        except serial.SerialException as e:
            print(f"[SerialSender] Error opening port: {e}")
            return False

        self._running = True
        self._thread = threading.Thread(target=self._send_loop, daemon=True)
        self._thread.start()
        
        print(f"[SerialSender] Started at {self._config.send_hz} Hz")
        return True

    def stop(self) -> None:
        """Stop the sender and close serial port."""
        if not self._running:
            return

        self._running = False

        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

        # 发送全停指令
        self.stop_all()

        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        
        print("[SerialSender] Stopped")

    def set_action(self, robot_id: int, action: Action) -> None:
        """Set action for a robot (API Compatible)."""
        with self._action_lock:
            self._actions[robot_id] = action
            self._active_robot_ids.add(robot_id)

    def set_actions(self, actions: Dict[int, Action]) -> None:
        """Set actions for multiple robots (API Compatible)."""
        with self._action_lock:
            for robot_id, action in actions.items():
                self._actions[robot_id] = action
                self._active_robot_ids.add(robot_id)

    def send_immediate(self, robot_id: int, action: Action) -> bool:
        """
        Send action immediately (bypassing the loop rate).
        Note: In serial bus, this inserts a packet for this specific robot immediately.
        """
        # 更新缓存状态
        self.set_action(robot_id, action)
        
        # 立即构建并发送单机包
        return self._send_packet({robot_id: action})

    def stop_all(self) -> None:
        """Send stop command to all known robots immediately."""
        stop_actions = {}
        with self._action_lock:
            for robot_id in self._active_robot_ids:
                stop_action = Action.stop()
                self._actions[robot_id] = stop_action
                stop_actions[robot_id] = stop_action
        
        # 立即发送停止帧
        self._send_packet(stop_actions)

    def add_robot(self, robot_id: int, ip: str = "", port: int = 0) -> None:
        """
        Register a new robot.
        Arguments `ip` and `port` are ignored in Serial mode but kept for API compatibility.
        """
        with self._action_lock:
            self._active_robot_ids.add(robot_id)
            if robot_id not in self._actions:
                self._actions[robot_id] = Action.stop()

    def remove_robot(self, robot_id: int) -> None:
        """Unregister a robot."""
        with self._action_lock:
            self._active_robot_ids.discard(robot_id)
            self._actions.pop(robot_id, None)

    # -------------------------------------------------------------------------
    # Internal - Protocol Implementation
    # -------------------------------------------------------------------------

    def _send_loop(self) -> None:
        """Background loop."""
        period = 1.0 / max(1.0, self._config.send_hz)

        while self._running:
            loop_start = time.perf_counter()

            # 1. 获取当前所有动作快照
            with self._action_lock:
                # 过滤掉不在 active_list 里的，虽然理论上 set_action 会同步更新
                current_actions = {
                    rid: act for rid, act in self._actions.items() 
                    if rid in self._active_robot_ids
                }

            # 2. 发送聚合包
            if current_actions:
                self._send_packet(current_actions)

            # 3. 控频
            elapsed = time.perf_counter() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _send_packet(self, actions_map: Dict[int, Action]) -> bool:
        """
        Construct and write the aggregated packet to serial.
        Thread-safe wrapper around serial write.
        """
        if self._ser is None or not self._ser.is_open:
            return False

        payload_body = bytearray()
        valid_count = 0

        # 构建 Body
        for robot_id, action in actions_map.items():
            # 限制 RobotID 范围 (0-255)
            if not (0 <= robot_id <= 255):
                continue

            # 速度处理
            left = max(-1.0, min(1.0, action.left_speed))
            right = max(-1.0, min(1.0, action.right_speed))
            
            if self._config.invert_right_wheel:
                right = -right

            l_int = int(left * 1000)
            r_int = int(right * 1000)

            # Pack: [ID (1B)] [Left (2B)] [Right (2B)]
            # <Bhh = Little Endian: uchar, short, short
            payload_body.extend(struct.pack("<Bhh", robot_id, l_int, r_int))
            valid_count += 1

        if valid_count == 0:
            return False

        with self._serial_lock:
            try:
                # 1. 更新序列号
                seq = self._global_seq
                self._global_seq = (seq + 1) & 0xFFFF

                # 2. 构建 Header 部分: [Seq (2B)] [Count (1B)]
                header_info = struct.pack("<HB", seq, valid_count)

                # 3. 计算 Checksum
                # Checksum = Sum(HeaderInfo + Body) & 0xFF
                check_data = header_info + payload_body
                checksum = sum(check_data) & 0xFF

                # 4. 组装完整帧: [Head AA 55] + [CheckData] + [Checksum]
                full_packet = b"\xAA\x55" + check_data + struct.pack("B", checksum)

                # 5. 写入串口
                self._ser.write(full_packet)
                return True
            except serial.SerialException as e:
                print(f"Serial write error: {e}")
                return False
            except Exception as e:
                print(f"Unexpected serial error: {e}")
                return False