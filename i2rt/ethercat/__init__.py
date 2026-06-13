"""EtherCAT master helpers and motor command packing for EtherCAT-CAN gateways."""

from i2rt.ethercat.master import (
    EtherCATMaster,
    list_ethernet_interface_details,
    list_ethernet_interfaces,
    resolve_soem_master_bin,
)
from i2rt.ethercat.ping import run_ping_motors_via_ethercat
from i2rt.ethercat.runtime import EtherCATRuntime
from i2rt.ethercat.motor_control import (
    get_motor_parameter,
    motor_comm_mode_reading,
    motor_id_reading,
    motor_id_reset,
    motor_id_setting,
    motor_setzero,
    motor_setting,
    rv_can_data_repack,
    send_motor_ctrl_cmd,
    set_motor_acceleration,
    set_motor_cur_tor,
    set_motor_feedback_kp_kd,
    set_motor_linkage_speed_ki,
    set_motor_position,
    set_motor_speed,
)
from i2rt.ethercat.types import EtherCATMsg, MotorCommFeedback, MotorMsg, ODMotorMsg

__all__ = [
    "EtherCATMaster",
    "EtherCATRuntime",
    "EtherCATMsg",
    "MotorCommFeedback",
    "MotorMsg",
    "ODMotorMsg",
    "get_motor_parameter",
    "run_ping_motors_via_ethercat",
    "resolve_soem_master_bin",
    "list_ethernet_interface_details",
    "list_ethernet_interfaces",
    "motor_comm_mode_reading",
    "motor_id_reading",
    "motor_id_reset",
    "motor_id_setting",
    "motor_setzero",
    "motor_setting",
    "rv_can_data_repack",
    "send_motor_ctrl_cmd",
    "set_motor_acceleration",
    "set_motor_cur_tor",
    "set_motor_feedback_kp_kd",
    "set_motor_linkage_speed_ki",
    "set_motor_position",
    "set_motor_speed",
]
