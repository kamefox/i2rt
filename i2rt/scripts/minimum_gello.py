import time
from dataclasses import dataclass
from typing import Any, Dict, Literal

import numpy as np
import portal
import tyro

from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.motor_chain_robot import MotorChainRobot
from i2rt.robots.robot import Robot

DEFAULT_ROBOT_PORT = 11333


class ServerRobot:
    """A simple server for a leader robot."""

    def __init__(self, robot: Robot, port: str):
        self._robot = robot
        self._server = portal.Server(port)
        print(f"Robot Sever Binding to {port}, Robot: {robot}")

        self._server.bind("num_dofs", self._robot.num_dofs)
        self._server.bind("get_joint_pos", self._robot.get_joint_pos)
        self._server.bind("command_joint_pos", self._robot.command_joint_pos)
        self._server.bind("command_joint_state", self._robot.command_joint_state)
        self._server.bind("get_observations", self._robot.get_observations)

    def serve(self) -> None:
        """Serve the leader robot."""
        self._server.start()


class ClientRobot(Robot):
    """A simple client for a leader robot."""

    def __init__(self, port: int = DEFAULT_ROBOT_PORT, host: str = "127.0.0.1"):
        self._client = portal.Client(f"{host}:{port}")

    def num_dofs(self) -> int:
        """Get the number of joints in the robot.

        Returns:
            int: The number of joints in the robot.
        """
        return self._client.num_dofs().result()

    def get_joint_pos(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        return self._client.get_joint_pos().result()

    def command_joint_pos(self, joint_pos: np.ndarray) -> None:
        """Command the leader robot to the given state.

        Args:
            joint_pos (T): The state to command the leader robot to.
        """
        self._client.command_joint_pos(joint_pos)

    def command_joint_state(self, joint_state: Dict[str, np.ndarray]) -> None:
        """Command the leader robot to the given state.

        Args:
            joint_state (Dict[str, np.ndarray]): The state to command the leader robot to.
        """
        self._client.command_joint_state(joint_state)

    def get_observations(self) -> Dict[str, np.ndarray]:
        """Get the current observations of the leader robot.

        Returns:
            Dict[str, np.ndarray]: The current observations of the leader robot.
        """
        return self._client.get_observations().result()


class YAMLeaderRobot:
    def __init__(self, robot: MotorChainRobot):
        self._robot = robot
        self._motor_chain = robot.motor_chain

    def get_info(self) -> np.ndarray:
        qpos = self._robot.get_observations()["joint_pos"]
        encoder_obs = self._motor_chain.get_same_bus_device_states()
        time.sleep(0.01)
        gripper_cmd = 1 - encoder_obs[0].position
        qpos_with_gripper = np.concatenate([qpos, [gripper_cmd]])
        return qpos_with_gripper, encoder_obs[0].io_inputs

    def get_handle_ui_dict(self) -> Dict[str, Any]:
        """与 can_selector /api/teaching-handle-status 卡片字段对齐，供 Leader 遥测 portal 调用。"""
        _ = self._robot.get_observations()["joint_pos"]
        encoder_obs = self._motor_chain.get_same_bus_device_states()
        time.sleep(0.01)
        enc = encoder_obs[0]
        gripper_cmd = 1.0 - float(enc.position)
        io = enc.io_inputs
        btn = list(io) if io else []
        b1 = bool(float(btn[0]) > 0.5) if len(btn) > 0 else None
        b2 = bool(float(btn[1]) > 0.5) if len(btn) > 1 else None
        gn = max(0.0, min(1.0, float(gripper_cmd)))
        return {
            "button1": b1,
            "button2": b2,
            "trigger_normalized": gn,
            "trigger_velocity_rad_s": float(enc.velocity),
            "report_frequency_hz": None,
        }

    def command_joint_pos(self, joint_pos: np.ndarray) -> None:
        assert joint_pos.shape[0] == 6
        self._robot.command_joint_pos(joint_pos)

    def update_kp_kd(self, kp: np.ndarray, kd: np.ndarray) -> None:
        self._robot.update_kp_kd(kp, kd)


@dataclass
class Args:
    gripper: Literal["crank_4310", "linear_3507", "linear_4310", "yam_teaching_handle", "no_gripper"] = "yam_teaching_handle"
    mode: Literal["follower", "leader", "visualizer_local", "visualizer_remote"] = "follower"
    server_host: str = "localhost"
    server_port: int = DEFAULT_ROBOT_PORT
    can_channel: str = "can0"
    bilateral_kp: float = 0.0
    arm_type: Literal["yam", "yam_pro", "yam_ultra", "yam_ultra_26", "big_yam", "big_yamZY", "bigger_yam"] = "yam"


def main(args: Args) -> None:
    from i2rt.robots.utils import ArmType, GripperType

    gripper_type = GripperType.from_string_name(args.gripper)
    arm_t = ArmType.from_string_name(args.arm_type)

    if "remote" not in args.mode:
        robot = get_yam_robot(
            channel=args.can_channel,
            arm_type=arm_t,
            gripper_type=gripper_type,
            zero_gravity_mode=args.mode != "follower",
        )

    if args.mode == "follower":
        server_robot = ServerRobot(robot, args.server_port)
        server_robot.serve()
    elif args.mode == "leader":
        robot = YAMLeaderRobot(robot)
        robot_current_kp = np.array(robot._robot._kp, copy=True)
        client_robot = ClientRobot(args.server_port, host=args.server_host)
        telem_port = int(args.server_port) + 1
        if telem_port <= 65535:
            telem_srv = portal.Server(telem_port)
            telem_srv.bind("get_joint_pos", robot._robot.get_joint_pos)
            telem_srv.bind("get_observations", robot._robot.get_observations)
            telem_srv.bind("get_teleop_handle_snapshot", robot.get_handle_ui_dict)
            telem_srv.start(block=False)
            print(f"Leader telemetry portal (for can_selector) on {telem_port}")
        else:
            print("Warning: server_port too high; leader telemetry portal disabled")

        # sync the robot state
        current_joint_pos, current_button = robot.get_info()
        current_follower_joint_pos = client_robot.get_joint_pos()
        print(f"Current leader joint pos: {current_joint_pos}")
        print(f"Current follower joint pos: {current_follower_joint_pos}")

        def slow_move(joint_pos: np.ndarray, duration: float = 1.0) -> None:
            for i in range(100):
                current_joint_pos = joint_pos
                follower_command_joint_pos = current_joint_pos * i / 100 + current_follower_joint_pos * (1 - i / 100)
                client_robot.command_joint_pos(follower_command_joint_pos)
                time.sleep(0.03)

        # 示教柄按键1：第1次进入同步/随动 → 第2次仅退出同步（Leader 仍零重力工作）→ 第3次再同步，循环
        sync_active = False
        engaged = False  # 至少进过一次同步后，退出同步才开最小跟随，避免上电即拉 Follower
        prev_btn1 = False  # 按键1 上升沿触发，避免电平抖动/慢释放导致数秒后误再次同步
        while True:
            current_joint_pos, current_button = robot.get_info()
            btn1_high = float(current_button[0]) > 0.5
            if btn1_high and not prev_btn1:
                if not sync_active:
                    robot.update_kp_kd(kp=robot_current_kp * args.bilateral_kp, kd=np.ones(6) * 0.0)
                    robot.command_joint_pos(current_joint_pos[:6])
                    slow_move(current_joint_pos)
                    engaged = True
                else:
                    print("exit sync only: leader kp/kd=0 (zero-g); min follow Leader->Follower stays on")
                    robot.update_kp_kd(kp=np.ones(6) * 0.0, kd=np.ones(6) * 0.0)
                    robot.command_joint_pos(current_follower_joint_pos[:6])
                sync_active = not sync_active
                while float(current_button[0]) > 0.5:
                    time.sleep(0.03)
                    current_joint_pos, current_button = robot.get_info()
                prev_btn1 = False
            else:
                prev_btn1 = btn1_high

            current_follower_joint_pos = client_robot.get_joint_pos()

            if sync_active:
                client_robot.command_joint_pos(current_joint_pos)
                # this will set the bilateral force in joint space proportional to the bilateral kp
                robot.command_joint_pos(current_follower_joint_pos[:6])
            elif engaged:
                # 同步已关：仍最小位置跟随（Follower 跟踪 Leader），无 Leader 侧双边力
                client_robot.command_joint_pos(current_joint_pos)

            time.sleep(0.01)
    elif "visualizer" in args.mode:
        import mujoco
        import mujoco.viewer
        if args.mode == "visualizer_remote":
            robot = ClientRobot(args.server_port, host=args.server_host)
        xml_path = gripper_type.get_xml_path()
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)

        dt: float = 0.01
        with mujoco.viewer.launch_passive(
            model=model,
            data=data,
            show_left_ui=False,
            show_right_ui=False,
        ) as viewer:
            mujoco.mjv_defaultFreeCamera(model, viewer.cam)
            viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE

            while viewer.is_running():
                step_start = time.time()
                joint_pos = robot.get_joint_pos()
                data.qpos[:] = joint_pos[: model.nq]

                # sync the model state
                mujoco.mj_kinematics(model, data)
                viewer.sync()
                time_until_next_step = dt - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
    else:
        raise ValueError(f"Invalid mode: {args.mode}")


if __name__ == "__main__":
    main(tyro.cli(Args))
