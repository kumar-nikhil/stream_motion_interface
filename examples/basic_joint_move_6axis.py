"""
Example: 6-Axis Coordinated Joint Move via Stream Motion
=========================================================
Manual: B-84904EN/01

Moves all 6 joints simultaneously from the robot's current position by the
specified DELTA_JOINTS offsets.  All axes are time-synchronised — the slowest
axis (largest displacement relative to its limit) sets the total duration T,
and every other joint scales proportionally so they all finish together.

Verified working in ROBOGUIDE (STREAM_MOTION_LEARNINGS.md, Test 2).
Default delta: J1+2  J2+1  J3-1  J4+2  J5-2  J6+2  (all degrees)

Prerequisites
-------------
Same as basic_joint_move.py:
  - TP program running with IBGN start[1] / IBGN end[1]
  - AUTO mode, 100% override, no active faults
"""

import logging

from stream_motion import (
    StreamMotionClient,
    minimum_jerk_trajectory,
    check_limits,
)
from stream_motion.constants import CRX_VEL_LIMITS, CRX_ACC_LIMITS, CRX_JRK_LIMITS

# ── Configure logging ─────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(name)s – %(message)s",
)
log = logging.getLogger(__name__)

# ── Robot / Communication Settings ───────────────────────────────────────────
ROBOT_IP   = "192.168.56.1"
ROBOT_PORT = 60015

# ── Motion Parameters ─────────────────────────────────────────────────────────
# Relative move applied to the robot's live start position.
# Replicate the GPT project's confirmed working test:
#   --delta-joints 8 5 -5 8 -8 8  (from STREAM_MOTION_LEARNINGS.md, Test 2)
# Using smaller values here as a conservative first test; scale up once confirmed.
DELTA_JOINTS = [2.0, 1.0, -1.0, 2.0, -2.0, 2.0]  # [deg] per joint

# Conservative scale — all peaks at ~5% of limits.
# Speed ladder (increase after each successful test):
#   0.05  →  0.10  →  0.20  →  0.40  →  0.80
# Watch for: MOTN-609 (velocity), MOTN-610 (accel), MOTN-611 (jerk)
SCALE = 0.05


def main() -> None:
    log.info("=" * 60)
    log.info("Stream Motion – 6-Axis Coordinated Move")
    log.info("Robot: %s:%d", ROBOT_IP, ROBOT_PORT)
    log.info("Delta: %s deg", DELTA_JOINTS)
    log.info("Scale: %.2f  (%.0f%% of all limits)", SCALE, SCALE * 100)
    log.info("=" * 60)

    with StreamMotionClient(robot_ip=ROBOT_IP, robot_port=ROBOT_PORT) as client:

        client.start_status_output()
        log.info("Waiting for TP program to reach IBGN start[1]...")

        if not client.wait_for_ready(timeout=60.0):
            log.error("Robot not ready — check TP program, mode, and override")
            client.stop_status_output()
            return

        start_joints = client.get_current_joints()
        if start_joints is None:
            log.error("Could not read current joint positions — aborting")
            client.stop_status_output()
            return

        end_joints = [s + d for s, d in zip(start_joints, DELTA_JOINTS)]

        log.info("Start: %s", [f"{j:.3f}" for j in start_joints])
        log.info("End:   %s", [f"{j:.3f}" for j in end_joints])

        # Plan: minimum-jerk profile, time-synchronised across all 6 axes
        trajectory = minimum_jerk_trajectory(
            start_joints = start_joints,
            end_joints   = end_joints,
            vel_limits   = CRX_VEL_LIMITS,
            acc_limits   = CRX_ACC_LIMITS,
            jrk_limits   = CRX_JRK_LIMITS,
            scale        = SCALE,
        )

        violations = check_limits(trajectory, CRX_VEL_LIMITS, CRX_ACC_LIMITS, CRX_JRK_LIMITS)
        if violations:
            log.error("Trajectory limit violations (aborting):")
            for v in violations:
                log.error("  %s", v)
            client.stop_status_output()
            return

        T = (len(trajectory) - 1) * 0.008
        log.info("Trajectory: %d waypoints  T=%.2f s — limit check PASSED",
                 len(trajectory), T)

        # Log first 4 waypoints to verify cubic ramp-up on each axis
        log.info("First 4 waypoints:")
        for i in range(min(4, len(trajectory))):
            log.info("  [%d] %s", i,
                     [f"{v:.5f}" for v in trajectory[i]])

        log.info("Streaming %d waypoints to robot...", len(trajectory))
        success = client.stream_joint_trajectory(trajectory)

        if success:
            log.info("Motion complete! Waiting for robot to settle...")
            client.wait_for_stop(timeout=15.0)
        else:
            log.warning("Trajectory stream returned False — check robot alarms")

        client.stop_status_output()

    log.info("Done.")


if __name__ == "__main__":
    main()
