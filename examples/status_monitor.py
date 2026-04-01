"""
Example: Live Status Monitor
=============================
Connects to the robot and continuously prints its position.
Useful for verifying connectivity with ROBOGUIDE before doing any motion.

Run this BEFORE running the robot TP program to verify the UDP connection.
"""

import logging
import signal
import time

from stream_motion import StreamMotionClient, StatusPacket

logging.basicConfig(level=logging.INFO, format="%(asctime)s  %(message)s")

ROBOT_IP   = "127.0.0.1"
ROBOT_PORT = 60015

running = True


def on_status(status: StatusPacket) -> None:
    """Called every ~8ms when a status packet arrives."""
    j = status.joints
    c = status.cart
    flags = (
        ("READY" if status.is_waiting_for_command else "     ")
        + (" CMD_RX" if status.is_command_received else "      ")
        + (" MOVING" if status.is_moving else "      ")
        + (" SYSRDY" if status.is_sysrdy else "      ")
    )
    print(
        f"\rSeq:{status.sequence_no:6d} | {flags} | "
        f"J1:{j[0]:7.2f} J2:{j[1]:7.2f} J3:{j[2]:7.2f} "
        f"J4:{j[3]:7.2f} J5:{j[4]:7.2f} J6:{j[5]:7.2f} | "
        f"X:{c[0]:8.2f} Y:{c[1]:8.2f} Z:{c[2]:8.2f}",
        end="",
        flush=True,
    )


def main() -> None:
    global running

    def _stop(sig, frame):
        global running
        running = False

    signal.signal(signal.SIGINT, _stop)

    print(f"Connecting to ROBOGUIDE/robot at {ROBOT_IP}:{ROBOT_PORT}")
    print("Press Ctrl+C to stop\n")

    with StreamMotionClient(robot_ip=ROBOT_IP, robot_port=ROBOT_PORT) as client:
        client.set_status_callback(on_status)
        client.start_status_output()
        while running:
            time.sleep(0.1)
        client.stop_status_output()

    print("\nDisconnected.")


if __name__ == "__main__":
    main()
