import argparse
import csv
import os
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import numpy as np

from i2rt.motor_drivers.dm_driver import DMChainCanInterface, ReceiveMode

# Import required robot libraries
from i2rt.robots.motor_chain_robot import MotorChainRobot
from i2rt.robots.robot import Robot
from i2rt.robots.utils import GripperType


@dataclass
class Args:
    channel: str = "can0"
    use_gravity_comp: bool = True
    temp_record_flag: bool = False


def create_yam_robot(args: Args) -> Robot:
    """Create YAM robot instance directly from code without config file"""
    # Configure motor chain
    motor_list = [
        [0x01, "DM4340"],
        [0x02, "DM4340"],
        [0x03, "DM4340"],
        [0x04, "DM4310"],
        [0x05, "DM4310"],
        [0x06, "DM4310"],
        [0x07, "DM4310"],
    ]

    motor_chain = DMChainCanInterface(
        motor_list=motor_list,
        motor_offset=[0, 0, 0, 0, 0, 0, 0],
        motor_direction=[1, 1, 1, 1, 1, 1, 1],
        channel=args.channel,
        motor_chain_name="yam",
        receive_mode=ReceiveMode.p16,
    )

    xml = "robot_models/yam/mjmodel.xml" if args.use_gravity_comp else None
    # Create the robot
    robot = MotorChainRobot(
        motor_chain=motor_chain,
        xml_path=xml,
        use_gravity_comp=args.use_gravity_comp,
        gravity_comp_factor=1.3,
        gripper_index=6,
        kp=[80, 80, 80, 10, 10, 10, 20],
        kd=[5, 5, 5, 1.5, 1.5, 1.5, 0.5],
        joint_limits=[[-2.09, 3.14], [0, 3.14], [0.05, 3.14], [-1.35, 1.35], [-1.50, 1.50], [-2.00, 2.00]],
        gripper_limits=[0.0, -2.4],  # with deadzone
        limit_gripper_force=50.0,
        gripper_type=GripperType.CRANK_4310,
        temp_record_flag=args.temp_record_flag,
    )

    return robot


# 与 aging_motion_control 主循环一致：每周期 command_joint_pos 一次
AGING_COMMAND_SLEEP_S = 0.01
AGING_COMMAND_HZ = 1.0 / AGING_COMMAND_SLEEP_S

# Predefined arm motions
YAM_ARM_MOTIONS = [
    np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
    np.array([2.93, 0.05, 0.60, -0.08, 0.15, 2.07, 0.4]),
    np.array([3.06, 1.68, 3.14, 1.58, 1.57, -2.07, 0]),
    np.array([0.06, 0.68, 1.14, 0.58, 0.57, 0.07, 0.75]),
    np.array([-2.60, 0.0, 0.0, 1.58, -1.57, 1.40, 1.0]),
    np.array([-2.61, 0.0, 3.14, -1.59, -1.57, -2.06, 0.6]),
    np.array([0.12, 0.0, 0.74, 1.58, -1.57, 2.07, 0]),
]


def _ensure_robot_control_healthy(robot: Robot) -> None:
    """电机链内部线程若因超时/断连退出，Robot.update 仍可能只读旧 state 而不抛错，此处主动终止老化。"""
    tex = getattr(robot, "thread_exception", None)
    if tex is not None:
        raise RuntimeError(f"机器人控制线程异常: {tex}")
    mc = getattr(robot, "motor_chain", None)
    th = getattr(mc, "_control_thread", None)
    if th is not None and not th.is_alive():
        raise RuntimeError(
            "电机链 CAN 控制线程已停止（常见：某台电机无响应或通信断开），已中止老化。"
        )


def _simultaneous_zero_all_joints(
    robot: Robot,
    motion_duration: float,
    sleep_time: float,
    progress: Optional[Dict[str, Any]],
    t0: float,
    stop_event: Optional[threading.Event] = None,
    done_message: str = "老化停止：各关节同时插值回零。",
) -> bool:
    """从当前关节角在同一时段内同步线性插值到全 0（与逐关节顺序回零相对）。"""
    num_steps = max(1, int(motion_duration / sleep_time))
    _ensure_robot_control_healthy(robot)
    start = np.asarray(robot.get_joint_pos(), dtype=float).copy()
    n = len(start)
    target = np.zeros(n, dtype=float)
    for i in range(num_steps):
        if stop_event is not None and stop_event.is_set():
            return False
        if (i & 0xF) == 0:
            _ensure_robot_control_healthy(robot)
        alpha = (i + 1) / float(num_steps)
        q = start + alpha * (target - start)
        robot.command_joint_pos(q)
        if progress is not None and (i & 0xF) == 0:
            progress["elapsed_s"] = time.time() - t0
        time.sleep(sleep_time)
    print(done_message)
    return True


def _sequential_zero_by_motor_id(
    robot: Robot,
    motion_duration: float,
    sleep_time: float,
    progress: Optional[Dict[str, Any]],
    t0: float,
    stop_event: Optional[threading.Event] = None,
    done_message: str = "老化停止：已按 ID 顺序逐关节置零位。",
    motor_ids: Optional[List[int]] = None,
) -> bool:
    """按电机 ID（1-based）顺序依次将该关节目标置 0；motor_ids 为 None 时全部关节。返回是否全程完成。"""
    num_steps = max(1, int(motion_duration / sleep_time))
    n = len(robot.get_joint_pos())
    if motor_ids is None:
        joint_iter = list(range(n))
    else:
        joint_iter = sorted({int(mid) - 1 for mid in motor_ids if 1 <= int(mid) <= n})
    for j in joint_iter:
        if stop_event is not None and stop_event.is_set():
            return False
        _ensure_robot_control_healthy(robot)
        start = np.asarray(robot.get_joint_pos(), dtype=float).copy()
        target = start.copy()
        target[j] = 0.0
        for i in range(num_steps):
            if stop_event is not None and stop_event.is_set():
                return False
            if (i & 0xF) == 0:
                _ensure_robot_control_healthy(robot)
            q = start + (i / float(num_steps)) * (target - start)
            robot.command_joint_pos(q)
            if progress is not None and (i & 0xF) == 0:
                progress["elapsed_s"] = time.time() - t0
            time.sleep(sleep_time)
    print(done_message)
    return True


def run_sequential_zero(
    channel: str,
    motion_duration: float = 1.5,
    stop_event: Optional[threading.Event] = None,
    motor_ids: Optional[List[int]] = None,
) -> bool:
    """独立顺序置零位（与老化停止时相同插值逻辑），创建 YAM 机器人并逐关节到 0。motor_ids 为 None 表示全部。"""
    robot = create_yam_robot(Args(channel=channel, use_gravity_comp=False, temp_record_flag=False))
    try:
        t0 = time.time()
        if motor_ids is None:
            done_msg = "顺序置零位完成（全部电机，按 ID 逐关节到 0）。"
        else:
            ids_s = ",".join(str(int(m)) for m in sorted(set(motor_ids)))
            done_msg = f"顺序置零位完成（电机 ID: {ids_s}）。"
        ok = _sequential_zero_by_motor_id(
            robot,
            motion_duration,
            AGING_COMMAND_SLEEP_S,
            None,
            t0,
            stop_event=stop_event,
            done_message=done_msg,
            motor_ids=motor_ids,
        )
        if not ok and stop_event is not None and stop_event.is_set():
            print("顺序置零位已中止。")
        return ok
    finally:
        robot.close()


JOINT_VELOCITY_LIMITS = {
    0: 0.5,  # joint 1
    1: 0.8,  # joint 2
    2: 0.45,  # joint 3
    3: 0.45,  # joint 4
    4: 0.7,  # joint 5
    5: 0.45,  # joint 6
    6: 1.2,  # joint 7
}


def aging_motion_control(
    robot: Robot,
    motion_duration: float = 1.5,  #每个动作（从当前位姿到目标位姿）的持续时间（秒），默认 1.5 秒
    motion_list: Optional[List[np.ndarray]] = None, #一组目标关节角度（弧度），机器人会依次运动到这些位置
    save_log: bool = True,   #是否记录循环次数和时间戳到日志文件
    log_trajectories: Optional[str] = None,  #轨迹日志保存路径。如果提供，会将关节位置、速度、力矩、温度等数据保存为 CSV 文件
    stop_event: Optional[threading.Event] = None,
    progress: Optional[Dict[str, Any]] = None,
):
    """
    Perform aging motion control on a robot.

    Args:
        robot (Robot): The robot to control.
        motion_duration (float): Duration of each motion in seconds.
        motion_list (List[np.ndarray]): List of target joint positions.
        save_log (bool): If True, save a log file with cycle counts and timestamps.
        log_trajectories (str): Directory path to save trajectory data. If None, no trajectory data will be saved.
                               If provided, trajectory data (position, velocity, effort) will be logged to CSV files.

    This function will:
    1. Execute a series of motions on the robot continuously
    2. Log cycle counts and timestamps
    3. Optionally log trajectory data (joint positions, velocities, efforts) to CSV files
       when log_trajectories is provided. Data is saved every 100 cycles to reduce overhead.
    """
    
    if motion_list is None:   #确保必须传入动作列表，否则报错。
        raise ValueError("motion_list must be provided")
    
    # Create log directory if specified and doesn't exist 如果要保存轨迹数据，先创建目录（如 logs/trajectories/）。
    if log_trajectories and not os.path.exists(log_trajectories):
        os.makedirs(log_trajectories)

    start_timestamp = time.strftime("%Y%m%d_%H%M%S")  #日志文件名包含启动时间，避免覆盖。
    if log_trajectories:
        save_log_path = os.path.join(log_trajectories, f"aging_motion_log_{start_timestamp}.txt")
    else:
        save_log_path = f"aging_motion_log_{start_timestamp}.txt"

    cycle_count = 0    #每个动作被分解为 num_steps 步，实现平滑插值运动。
    sleep_time = AGING_COMMAND_SLEEP_S
    num_steps = int(motion_duration / sleep_time)   # 例如 1.5 / 0.01 = 150 步
    t0 = time.time()
    if progress is not None:
        progress["t0"] = t0
        progress["cycles"] = 0
        progress["elapsed_s"] = 0.0

    def _stopped() -> bool:
        return stop_event is not None and stop_event.is_set()

    def _exit_after_stop_homing() -> None:
        _simultaneous_zero_all_joints(robot, motion_duration, sleep_time, progress, t0)

    while True:
        if _stopped():
            _exit_after_stop_homing()
            return
        _ensure_robot_control_healthy(robot)
        if progress is not None:
            progress["elapsed_s"] = time.time() - t0
        cycle_start_time = time.strftime("%Y%m%d_%H%M%S")  # 记录每个循环开始时间
      
        for motion in motion_list:  #执行一个完整的动作序列
            if _stopped():
                _exit_after_stop_homing()
                return
            _ensure_robot_control_healthy(robot)
            current_joint_pos = robot.get_joint_pos()   ## 获取当前关节角

            for i in range(num_steps):   # 线性插值：从当前位姿逐步移动到目标 motion
                if _stopped():
                    _exit_after_stop_homing()
                    return
                if (i & 0xF) == 0:
                    _ensure_robot_control_healthy(robot)
                interpolated_motion = current_joint_pos + i / num_steps * (motion - current_joint_pos)
                robot.command_joint_pos(interpolated_motion)   # 发送控制指令
                if progress is not None and (i & 0xF) == 0:
                    progress["elapsed_s"] = time.time() - t0

                # Record observations 只有在 log_trajectories 且每 100 个循环时才记录轨迹
                if log_trajectories and (cycle_count % 100 == 0):
                    observations = robot.get_observations()  # 获取传感器数据
                    csv_filename = os.path.join(log_trajectories, f"cycle_{cycle_count}_{cycle_start_time}.csv")
                    timestamp = time.time()  # 格式化时间（精确到毫秒）
                    milliseconds = int((timestamp % 1) * 1000)
                    formatted_time = (
                        time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(timestamp)) + f".{milliseconds:03d}"
                    )
                    # CSV 文件写入逻辑（带 header）
                    file_exists = os.path.isfile(csv_filename)
                    with open(csv_filename, "a", newline="") as csvfile:
                        csv_writer = csv.writer(csvfile)

                        # Write header if file doesn't exist    第一次写入时写 header
                        if not file_exists:
                            joint_count = len(observations["joint_pos"])
                            header = ["timestamp"]

                            header += [f"pos_joint_{j}" for j in range(joint_count)]
                            if "gripper_pos" in observations:
                                header += ["pos_gripper"]

                            if "joint_vel" in observations:
                                header += [f"vel_joint_{j}" for j in range(len(observations["joint_vel"]))]

                            if "joint_eff" in observations:
                                header += [f"eff_joint_{j}" for j in range(len(observations["joint_eff"]))]

                            if "motor_mos_temp" in observations:
                                header += [f"motor_mos_temp{j}" for j in range(len(observations["motor_mos_temp"]))]

                            if "motor_rotor_temp" in observations:
                                header += [
                                    f"motor_rotor_temp{j}" for j in range(len(observations["motor_rotor_temp"]))
                                ]

                            csv_writer.writerow(header)

                        # Prepare row data  写入数据行
                        row_data = [formatted_time]

                        row_data += observations["joint_pos"].tolist()
                        if "gripper_pos" in observations:
                            row_data += observations["gripper_pos"].tolist()

                        if "joint_vel" in observations:
                            row_data += observations["joint_vel"].tolist()

                        if "joint_eff" in observations:
                            row_data += observations["joint_eff"].tolist()

                        if "motor_mos_temp" in observations:
                            row_data += observations["motor_mos_temp"].tolist()

                        if "motor_rotor_temp" in observations:
                            row_data += observations["motor_rotor_temp"].tolist()

                        csv_writer.writerow(row_data)

                time.sleep(sleep_time)  # 控制频率 ~100Hz

        # Increment cycle count
        cycle_count += 1   #每完成一次 motion_list 中所有动作，cycle_count 加 1
        if progress is not None:
            progress["cycles"] = cycle_count
            progress["elapsed_s"] = time.time() - t0
        timestamp = time.ctime()

        # Print the cycle count 打印并记录循环次数和时间
        print(f"Cycle: {cycle_count}, Timestamp: {timestamp}")

        if save_log:
            with open(save_log_path, "a") as f:
                f.write(f"Cycle: {cycle_count}, Timestamp: {timestamp}\n")


def main() -> None:
    """Main function to run the aging test"""
    try:
        parser = argparse.ArgumentParser(description="YAM robot aging test")
        parser.add_argument("--channel", type=str, default="can0", help="CAN channel to use")
        args = parser.parse_args()
        # make sure all can interfaces are up.
        #subprocess.run(["bash", "scripts/reset_all_can.sh"], check=True)
        time.sleep(0.5)

        # Create robot instance directly
        robot = create_yam_robot(Args(channel=args.channel))
        print("Successfully created YAM robot instance")

        # Run aging test with predefined motions
        aging_motion_control(robot=robot, motion_list=YAM_ARM_MOTIONS, motion_duration=1.5, save_log=True, log_trajectories="./data")
    except Exception as e:
        print(f"Error during aging test: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
