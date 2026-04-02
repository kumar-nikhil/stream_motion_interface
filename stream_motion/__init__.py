"""
stream_motion – Python client for FANUC Stream Motion (J519)
============================================================
Manual: B-84904EN/01  R-30iB/R-30iB Plus Controller

Quick start:
    from stream_motion import StreamMotionClient

    with StreamMotionClient(robot_ip="127.0.0.1") as client:
        client.start_status_output()
        client.wait_for_ready()
        client.stream_joint_trajectory(my_trajectory)
        client.stop_status_output()
"""

from .client import StreamMotionClient
from .packets import (
    StatusPacket,
    CommandPositionResponse,
    build_status_start_packet,
    build_status_stop_packet,
    build_command_packet,
    parse_status_packet,
)
from .constants import (
    ROBOT_DEFAULT_IP,
    ROBOT_UDP_PORT,
    COMM_CYCLE_MS,
    DATA_FORMAT_JOINT,
    DATA_FORMAT_CARTESIAN,
    STATUS_WAITING_FOR_CMD,
    STATUS_ROBOT_MOVING,
)
from .trajectory import (
    minimum_jerk_trajectory,
    minimum_jerk_cartesian_trajectory,
    trapezoidal_joint_trajectory,
    linear_joint_interpolation,
    smooth_trajectory,
    check_limits,
    pad_to_9,
)

__version__ = "0.1.0"
__all__ = [
    "StreamMotionClient",
    "StatusPacket",
    "CommandPositionResponse",
    "build_status_start_packet",
    "build_status_stop_packet",
    "build_command_packet",
    "parse_status_packet",
    "minimum_jerk_trajectory",
    "minimum_jerk_cartesian_trajectory",
    "trapezoidal_joint_trajectory",
    "linear_joint_interpolation",
    "check_limits",
    "pad_to_9",
    "ROBOT_DEFAULT_IP",
    "ROBOT_UDP_PORT",
    "COMM_CYCLE_MS",
    "DATA_FORMAT_JOINT",
    "DATA_FORMAT_CARTESIAN",
]
