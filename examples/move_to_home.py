"""
Example: Move to Robot Home Position via Stream Motion
======================================================
Manual: B-84904EN/01

Moves the CRX-10iA/L from its current position to the canonical home pose:
    J1=0  J2=0  J3=0  J4=0  J5=-90  J6=0

This is a large multi-axis move — all six joints typically move simultaneously.
A conservative SCALE is used to stay well within velocity/acceleration/jerk
limits regardless of where the robot starts.

Verified working in ROBOGUIDE (STREAM_MOTION_LEARNINGS.md, Test 4).

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
from stream_motion.constants import (
    CRX_VEL_LIMITS,
    CRX_ACC_LIMITS,
    CRX_JRK_LIMITS,
    HOME_JOINTS,
)

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
# Scale applied to vel, acc, and jerk limits when computing T.
# Confirmed fault-free at 0.05 (2026-04-02).  Now at 0.50 = 50% of all limits.
# Speed ladder: 0.05 ✓ → 0.50 (current) → 0.80 → 1.00 (watch MOTN-609/610/611)
SCALE = 0.50


def main() -> None:
    log.info("=" * 60)
    log.info("Stream Motion – Move to Home")
    log.info("Robot: %s:%d", ROBOT_IP, ROBOT_PORT)
    log.info("Home:  J1=%.1f  J2=%.1f  J3=%.1f  J4=%.1f  J5=%.1f  J6=%.1f",
             *HOME_JOINTS)
    log.info("=" * 60)

    with StreamMotionClient(robot_ip=ROBOT_IP, robot_port=ROBOT_PORT) as client:

        client.start_status_output()
        log.info("Waiting for TP program to reach IBGN start[1]...")

        if not client.wait_for_ready(timeout=60.0):
            log.error("Robot not ready — check TP program, mode, and override")
            client.stop_status_output()
            return

        # Read live position so trajectory starts exactly here
        start_joints = client.get_current_joints()
        if start_joints is None:
            log.error("Could not read current joint positions — aborting")
            client.stop_status_output()
            return

        log.info("Start: %s", [f"{j:.3f}" for j in start_joints])
        log.info("Home:  %s", [f"{j:.3f}" for j in HOME_JOINTS])

        # Compute per-joint displacements for logging
        deltas = [HOME_JOINTS[i] - start_joints[i] for i in range(6)]
        log.info("Delta: %s", [f"{d:+.3f}" for d in deltas])

        if all(abs(d) < 0.05 for d in deltas):
            log.info("Already at home — nothing to do.")
            client.stop_status_output()
            return

        # Plan: minimum-jerk profile, time-synchronised across all 6 axes.
        # T is set by the joint that needs the longest time (largest displacement
        # relative to its limit).  All other joints finish in the same T.
        trajectory = minimum_jerk_trajectory(
            start_joints = start_joints,
            end_joints   = HOME_JOINTS,
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
