"""
Example: Basic Joint-Space Move with Stream Motion
===================================================
Manual: B-84904EN/01  Section 2 PREPARATION FOR USE

Prerequisites
-------------
1. ROBOGUIDE (or real robot) running with Stream Motion option (J519).
2. A TP program loaded on the controller that contains:
       IBGN start[1]
       IBGN end[1]
3. The TP program is running in AUTO mode at 100% override.
4. The robot's joints are already near the start_joints position below.
   (Move the robot there manually before running this script.)

When using ROBOGUIDE on the same PC, the IP is 127.0.0.1 (default).

SAFETY WARNING (Section 2.4 of manual)
---------------------------------------
Stream Motion can cause unintended motion. Always test in simulation
(ROBOGUIDE) before running on a real robot. Ensure the surrounding area
is clear.
"""

import logging
import time

from stream_motion import (
    StreamMotionClient,
    trapezoidal_joint_trajectory,
    smooth_trajectory,
    check_limits,
)

# ── Configure logging ─────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(name)s – %(message)s",
)
log = logging.getLogger(__name__)

# ── Robot / Communication Settings ───────────────────────────────────────────
ROBOT_IP   = "192.168.56.1"  # ROBOGUIDE VM IP
ROBOT_PORT = 60015

# ── Motion Parameters ─────────────────────────────────────────────────────────
# Current joint position (read from the robot before running, or use get_current_joints())
START_JOINTS = [33.67, 8.94, 2.90, 14.04, -53.70, 89.03]  # current position after last move

# Target joint position – small move from current position
END_JOINTS   = [38.67, 8.94, 2.90, 14.04, -53.70, 89.03]  # J1 +5 degrees

# Per-joint velocity limits [deg/s]  – read from $STMO_GRP.$JNT_VEL_LIM
# These are approximate defaults; query the robot for exact values.
VEL_LIMITS = [150.0, 130.0, 200.0, 270.0, 270.0, 360.0]

# Per-joint acceleration limits [deg/s²] – read from $STMO_GRP.$JNT_ACC_LIM
ACC_LIMITS = [500.0, 400.0, 700.0, 900.0, 900.0, 1200.0]


def main() -> None:
    log.info("=" * 60)
    log.info("Stream Motion – Basic Joint Move")
    log.info("Robot: %s:%d", ROBOT_IP, ROBOT_PORT)
    log.info("=" * 60)

    # 1. Plan the trajectory BEFORE connecting
    log.info("Planning trapezoidal trajectory: %s → %s", START_JOINTS, END_JOINTS)
    trajectory = trapezoidal_joint_trajectory(
        start_joints = START_JOINTS,
        end_joints   = END_JOINTS,
        vel_limits   = VEL_LIMITS,
        acc_limits   = ACC_LIMITS,
        scale        = 0.3,          # 30% of limits — conservative for first live test
    )
    # Smooth the trajectory to eliminate jerk spikes at trapezoidal ramp transitions.
    # window=10 reduces effective jerk by ~10x at the cost of slight path rounding.
    # 2. Validate the raw trapezoidal trajectory BEFORE smoothing.
    # Smoothing is applied after — it only reduces jerk, never increases vel/acc.
    violations = check_limits(trajectory, VEL_LIMITS, ACC_LIMITS)
    if violations:
        log.error("Trajectory limit violations detected:")
        for v in violations:
            log.error("  %s", v)
        log.error("Aborting – fix trajectory before sending to robot")
        return
    log.info("Trajectory limit check: PASSED")
    trajectory = smooth_trajectory(trajectory, window=10)
    log.info("Trajectory: %d waypoints (smoothed, window=10)", len(trajectory))

    # 3. Connect and stream
    with StreamMotionClient(robot_ip=ROBOT_IP, robot_port=ROBOT_PORT) as client:

        # Start receiving status packets
        client.start_status_output()
        log.info("Waiting for robot TP program to reach IBGN start[1]...")

        # Block until bit 0 of the status byte goes HIGH
        # (requires the TP program to be running)
        if not client.wait_for_ready(timeout=60.0):
            log.error(
                "Robot did not become ready. Check:\n"
                "  1. TP program is running in AUTO mode at 100%% override\n"
                "  2. IBGN start[1] is in the program\n"
                "  3. ROBOGUIDE/robot IP is correct\n"
                "  4. Stream Motion option (J519) is loaded"
            )
            client.stop_status_output()
            return

        # Print current position
        status = client.last_status
        log.info("Current joints: %s", [f"{j:.2f}" for j in status.joints[:6]])
        log.info("Current cart:   %s", [f"{c:.2f}" for c in status.cart[:6]])

        # 4. Stream the trajectory
        log.info("Streaming %d waypoints...", len(trajectory))
        success = client.stream_joint_trajectory(trajectory)

        if success:
            log.info("Motion complete! Waiting for robot to stop...")
            client.wait_for_stop(timeout=5.0)
        else:
            log.warning("Trajectory stream returned False – check robot alarms")

        # Stop status output
        client.stop_status_output()

    log.info("Done.")


if __name__ == "__main__":
    main()
