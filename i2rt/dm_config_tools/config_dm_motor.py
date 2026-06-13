"""
DM Motor Configuration Tool
============================
Interactive CLI for configuring Damiao (DM) series motors over CAN.

Run directly to open the interactive menu::

    python dm_config_tools/config_dm_motor.py --channel can0

Supported operations:
- Set motor CAN ID and master ID
- Set control mode (MIT / speed)
- Configure velocity PID gains (KP / KI)
- Read all motor parameters
- Find a motor ID by scanning the bus
- Set motor zero position
"""

import sys
from pathlib import Path

# Add parent directory to path for direct execution
sys.path.insert(0, str(Path(__file__).parent.parent))

import time
from dataclasses import dataclass
from typing import Any

import tyro

from dm_config_tools.config_utils import (
    RawCanInterface,
    get_special_message_response,
    save_to_memory,
    write_special_message,
)
from i2rt.motor_drivers.dm_driver import ControlMode, DMSingleMotorCanInterface, MotorType

TIMEOUT = 4000  # Default motor communication timeout in ms


@dataclass
class Args:
    """CLI arguments for the motor configuration tool.

    Attributes:
        channel: CAN interface name (default ``"can0"``).
        bustype: python-can bus type (default ``"socketcan"``).
    """

    channel: str = "can0"
    bustype: str = "socketcan"


def config_4310V(can_interface: RawCanInterface, old_id: int, new_id: int, vel_ki: float = 0.004, vel_kp: float = 0.006) -> None:
    """Configure a DM4310V motor for speed mode.

    Sets ID, master ID, control mode (speed=3), p_max, and timeout,
    then saves all registers to flash.

    Args:
        can_interface: Open CAN bus to use for communication.
        old_id: Current motor CAN ID on the bus.
        new_id: Target CAN ID to assign.
        vel_ki: Velocity integral gain (informational; not written here).
        vel_kp: Velocity proportional gain (informational; not written here).
    """
    new_master_id = new_id + 16
    write_special_message(can_interface, old_id, "timeout", TIMEOUT)
    write_special_message(can_interface, old_id, "master_id", new_master_id)
    write_special_message(can_interface, old_id, "control_mode", 3)
    write_special_message(can_interface, old_id, "p_max", 3.1415926)
    write_special_message(can_interface, old_id, "id", new_id)
    for reg_name in ["timeout", "master_id", "id", "control_mode", "p_max"]:
        print(save_to_memory(can_interface, new_id, reg_name))


def config_6215(can_interface: RawCanInterface, old_id: int, new_id: int) -> None:
    """Configure a DMH6215 motor for speed mode (p_max=12.5, control_mode=3).

    Args:
        can_interface: Open CAN bus to use for communication.
        old_id: Current motor CAN ID on the bus.
        new_id: Target CAN ID to assign.
    """
    new_master_id = new_id + 16
    write_special_message(can_interface, old_id, "timeout", TIMEOUT)
    write_special_message(can_interface, old_id, "master_id", new_master_id)
    write_special_message(can_interface, old_id, "p_max", 12.5)
    write_special_message(can_interface, old_id, "control_mode", 3)
    write_special_message(can_interface, old_id, "id", new_id)
    for reg_name in ["timeout", "master_id", "id", "control_mode", "p_max"]:
        print(save_to_memory(can_interface, new_id, reg_name))


def config_6215_mit(can_interface: RawCanInterface, old_id: int, new_id: int) -> None:
    """Configure a DMH6215 motor for MIT mode (p_max=12.5, control_mode=1).

    Args:
        can_interface: Open CAN bus to use for communication.
        old_id: Current motor CAN ID on the bus.
        new_id: Target CAN ID to assign.
    """
    new_master_id = new_id + 16
    write_special_message(can_interface, old_id, "timeout", TIMEOUT)
    write_special_message(can_interface, old_id, "master_id", new_master_id)
    write_special_message(can_interface, old_id, "p_max", 12.5)
    write_special_message(can_interface, old_id, "control_mode", 1)
    write_special_message(can_interface, old_id, "id", new_id)
    for reg_name in ["timeout", "master_id", "id", "control_mode", "p_max"]:
        print(save_to_memory(can_interface, new_id, reg_name))


def config_4310(can_interface: RawCanInterface, old_id: int, new_id: int, vel_ki: float = 0.004, vel_kp: float = 0.006) -> None:
    """Configure a DM4310/4340/3507 motor for MIT mode (p_max=12.5, control_mode=1).

    Also used for DM3507 and DM4340 which share the same register layout.

    Args:
        can_interface: Open CAN bus to use for communication.
        old_id: Current motor CAN ID on the bus.
        new_id: Target CAN ID to assign.
        vel_ki: Velocity integral gain (informational; not written here).
        vel_kp: Velocity proportional gain (informational; not written here).
    """
    new_master_id = new_id + 16
    write_special_message(can_interface, old_id, "timeout", TIMEOUT)
    write_special_message(can_interface, old_id, "master_id", new_master_id)
    write_special_message(can_interface, old_id, "p_max", 12.5)
    # mit mode
    write_special_message(can_interface, old_id, "control_mode", 1)
    write_special_message(can_interface, old_id, "id", new_id)
    for reg_name in ["timeout", "master_id", "id", "control_mode", "p_max"]:
        print(save_to_memory(can_interface, new_id, reg_name))


def change_ki(can_interface: RawCanInterface, id: int, ki: float) -> None:
    """Write and save the velocity integral gain (vel_ki) for a motor.

    Args:
        can_interface: Open CAN bus to use for communication.
        id: Motor CAN ID.
        ki: New vel_ki value to write.
    """
    write_special_message(can_interface, id, "vel_ki", ki)
    for reg_name in ["vel_ki"]:
        print(save_to_memory(can_interface, id, reg_name))


def change_kp(can_interface: RawCanInterface, id: int, kp: float) -> None:
    """Write and save the velocity proportional gain (vel_kp) for a motor.

    Args:
        can_interface: Open CAN bus to use for communication.
        id: Motor CAN ID.
        kp: New vel_kp value to write.
    """
    write_special_message(can_interface, id, "vel_kp", kp)
    for reg_name in ["vel_kp"]:
        print(save_to_memory(can_interface, id, reg_name))


def read_parameter(can_interface: RawCanInterface, id: int) -> dict[str, Any]:
    """Read and print all parameters for a motor. Returns the values as a dict."""
    params: dict[str, Any] = {}
    for reg_name in [
        "sw_ver",
        "timeout",
        "master_id",
        "id",
        "control_mode",
        "gear_ratio",
        "vel_kp",
        "vel_ki",
        "p_max",
        "v_max",
        "torque_max",
        "over_voltage",
    ]:
        result = get_special_message_response(can_interface, id, reg_name)
        print(f"{reg_name}: {result}")
        params[reg_name] = result
    return params


def find_id(can_interface: RawCanInterface) -> int:
    """Scan the CAN bus for the first responding motor ID (0–254).

    Args:
        can_interface: Open CAN bus to scan.

    Returns:
        The first motor ID that responds.

    Raises:
        Exception: If no motor responds on IDs 0–254.
    """
    for i in range(0, 255):
        try:
            print(get_special_message_response(can_interface, i, "timeout"))
            return i
        except Exception:
            pass
    raise Exception("No motor found, please check the connection.")


def motor_config_interface(args: Args, can_interface: RawCanInterface) -> None:
    print("Select motor type(default config) or select what you would like to perform :")
    print("1) Configure speed mode motor(4310V/6215)")
    print("2) Configure MIT mode motor(3507/4310/4340/6215)")
    print("3) Find ID")
    print("4) Read parameters")
    print("5) Set zero position")

    action1 = input("Enter your choice (1/2/3/4/5): ")
    motor_control_interface = DMSingleMotorCanInterface(
        channel=args.channel, bustype="socketcan", control_mode=ControlMode.MIT
    )

    if action1 not in ["1", "2", "3", "4", "5"]:
        print("Invalid choice, please try again.")
        return

    if action1 == "3":
        id = find_id(can_interface)
        if id is not None:
            print(f"Motor ID found: {id}, in hex: {hex(id)}")
        return

    if action1 == "4":
        motor_id = int(input("Enter the motor ID to check parameters: "))
        params = read_parameter(can_interface, motor_id)
        print(params)
        return

    if action1 == "5":
        motor_id = int(input("Enter the motor ID to set zero position: "))
        motor_control_interface.save_zero_position(motor_id)
        return

    if action1 == "1":
        print("Select motor type for Speed mode:")
        print("1) 4310V")
        print("2) 6215")
        motor_choice = input("Enter motor choice (1/2): ")
        if motor_choice == "1":
            motor_type = MotorType.DM4310V
        elif motor_choice == "2":
            motor_type = MotorType.DMH6215
        else:
            print("Invalid motor choice")
            return

    elif action1 == "2":
        print("Select motor type for MIT mode:")
        print("1) 3507")
        print("2) 4310")
        print("3) 4340")
        print("4) 6215")
        motor_choice = input("Enter motor choice (1/2/3/4): ")
        if motor_choice == "1":
            motor_type = MotorType.DM3507
        elif motor_choice == "2":
            motor_type = MotorType.DM4310
        elif motor_choice == "3":
            motor_type = MotorType.DM4340
        elif motor_choice == "4":
            motor_type = MotorType.DMH6215MIT
        else:
            print("Invalid motor choice")
            return

    print("What would you like to do?")
    print("1) Change ID (master ID using rule p16)")
    print("2) Configure KP and KI")
    print("3) Control motor")
    action2 = input("Enter your choice (1/2/3): ")

    motor_control_interface = DMSingleMotorCanInterface(
        channel=args.channel,
        bustype="socketcan",
        control_mode=ControlMode.MIT if action1 == "2" else ControlMode.VEL,
    )

    if action2 == "1":
        old_id = int(input("Enter the old motor ID: "))
        new_id = int(input("Enter the new motor ID: "))

        if motor_type == MotorType.DM4310V:
            config_4310V(can_interface, old_id, new_id)
        elif motor_type == MotorType.DM4310:
            config_4310(can_interface, old_id, new_id)
        elif motor_type == MotorType.DMH6215:
            config_6215(can_interface, old_id, new_id)
        elif motor_type == MotorType.DMH6215MIT:
            config_6215_mit(can_interface, old_id, new_id)
        elif motor_type == MotorType.DM3507:
            config_4310(can_interface, old_id, new_id)  # 3507 uses same config as 4310
        elif motor_type == MotorType.DM4340:
            config_4310(can_interface, old_id, new_id)  # 4340 uses same config as 4310

        print(f"Motor ID has been changed from {old_id} to {new_id}.")

    elif action2 == "2":
        motor_id = int(input("Enter the motor ID: "))
        kp = float(input("Enter the vel_KP value: "))
        ki = float(input("Enter the vel_KI value: "))

        change_ki(can_interface, motor_id, ki)
        change_kp(can_interface, motor_id, kp)

        print(f"Motor KP and KI values have been set to KP: {kp}, KI: {ki}.")

    elif action2 == "3":
        motor_id = int(input("Enter the motor ID: "))
        print("For mit mode current kp kd is 10, 1, please enter target pos ane vel")
        pos = float(input("Enter the target position: "))
        vel = float(input("Enter the target velocity: "))
        motor_control_interface.clean_error(motor_id)
        assert motor_type is not None
        motor_control_interface.motor_on(motor_id, motor_type)
        while True:
            feedback = motor_control_interface.set_control(
                motor_id,
                motor_type,
                pos=pos,
                vel=vel,
                kp=10,
                kd=1.0,
                torque=0.0,
            )
            print(feedback)
            time.sleep(0.01)
    else:
        print("Invalid choice, please try again.")


def main() -> None:
    args = tyro.cli(Args)
    can_interface = RawCanInterface(channel=args.channel, bustype=args.bustype)
    motor_config_interface(args, can_interface)


if __name__ == "__main__":
    main()
