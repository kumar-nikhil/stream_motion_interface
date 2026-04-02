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
4. The robot can be at any nearby joint position; this script reads
   the actual live position before planning the trajectory.

When using ROBOGUIDE on the same PC, the IP is 127.0.0.1 (default).

SAFETY WARNING (Section 2.4 of manual)
---------------------------------------
Stream Motion can cause unintended motion. Always test in simulation
(ROBOGUIDE) before running on a real robot. Ensure the surrounding area
is clear.
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
ROBOT_IP   = "192.168.56.1"  # ROBOGUIDE VM IP
ROBOT_PORT = 60015

# ── Motion Parameters ─────────────────────────────────────────────────────────
# Relative move applied to the robot's live start position.
# Only J1 moves (+5 degrees); all other joints stay at current position.
DELTA_JOINTS = [5.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Per-joint limits from $STMO_GRP[1] on the pendant (CRX-10iA/L).
# Actual: VEL=[120,120,180,180,180,180], ACC=[279.9,...], JRK=[1240,...,1860,...]
VEL_LIMITS = CRX_VEL_LIMITS   # [deg/s]
ACC_LIMITS = CRX_ACC_LIMITS   # [deg/s²]
JRK_LIMITS = CRX_JRK_LIMITS   # [deg/s³]

# Safety scale applied to vel, acc, AND jerk limits when computing T.
# The working reference project (ChatGPT version) used a hardcoded 2.8 s
# duration for a 3° move, which puts peak jerk at ~14 deg/s³ (≈1 % of limit).
# SCALE = 0.05 reproduces that conservatism: T ≈ 1.7 s for a 5° J1 move,
# peak jerk ≈ 62 deg/s³ (5 % of limit).  Increase toward 0.8 once confirmed working.
SCALE = 0.05


def main() -> None:
    log.info("=" * 60)
    log.info("Stream Motion – Basic Joint Move")
    log.info("Robot: %s:%d", ROBOT_IP, ROBOT_PORT)
    log.info("=" * 60)

    with StreamMotionClient(robot_ip=ROBOT_IP, robot_port=ROBOT_PORT) as client:

        # ── 1. Start receiving status packets ─────────────────────────────────
        client.start_status_output()
        log.info("Waiting for robot TP program to reach IBGN start[1]...")

        if not client.wait_for_ready(timeout=60.0):
            log.error(
                "Robot did not become ready. Check:\n"
                "  1. TP program is running in AUTO mode at 100%% override\n"
                "  2. IBGN start[1] is in the program\n"
                "  3. ROBOGUIDE/robot IP is correct (%s)\n"
                "  4. Stream Motion option (J519) is loaded",
                ROBOT_IP,
            )
            client.stop_status_output()
            return

        # ── 2. Read the robot's ACTUAL current joint position ─────────────────
        # Critical: trajectory must start here so smoothed[0] ≈ current servo
        # position. A mismatch creates an implicit velocity spike at packet 0→1
        # that the controller detects as excessive acceleration (MOTN-721).
        start_joints = client.get_current_joints()
        if start_joints is None:
            log.error("Could not read current joint positions – aborting")
            client.stop_status_output()
            return

        end_joints = [s + d for s, d in zip(start_joints, DELTA_JOINTS)]

        log.info("Start joints: %s", [f"{j:.3f}" for j in start_joints])
        log.info("End   joints: %s", [f"{j:.3f}" for j in end_joints])
        log.info("Current cart: %s", [f"{c:.2f}" for c in client.get_current_cart() or []])

        # ── 3. Plan trajectory from actual position ───────────────────────────
        log.info(
            "Planning trajectory  scale=%.2f  VEL_lim=%s  ACC_lim=%s  JRK_lim=%s",
            SCALE, VEL_LIMITS, ACC_LIMITS, JRK_LIMITS,
        )
        # minimum_jerk_trajectory uses a 5th-order polynomial: p = D*(10s³-15s⁴+6s⁵)
        # - Zero velocity AND acceleration at start/end → no filter needed
        # - First position step ≈ 10*D/N³ (cubic, tiny) → no jerk spike at packet 1
        # - T is computed to satisfy vel, acc, AND jerk limits simultaneously
        trajectory = minimum_jerk_trajectory(
            start_joints = start_joints,
            end_joints   = end_joints,
            vel_limits   = VEL_LIMITS,
            acc_limits   = ACC_LIMITS,
            jrk_limits   = JRK_LIMITS,
            scale        = SCALE,
        )

        # Validate the trajectory against all three limits
        violations = check_limits(trajectory, VEL_LIMITS, ACC_LIMITS, JRK_LIMITS)
        if violations:
            log.error("Trajectory limit violations (aborting):")
            for v in violations:
                log.error("  %s", v)
            client.stop_status_output()
            return
        log.info("Trajectory: %d waypoints – limit check PASSED", len(trajectory))

        # Debug: show first 6 waypoints and step sizes to verify cubic ramp-up
        log.info("First 6 waypoints (J1):  Δ = position change per 8ms step")
        prev = trajectory[0][0]
        for i in range(min(6, len(trajectory))):
            cur = trajectory[i][0]
            log.info("  [%d] J1=%.5f°  Δ=%.5f°", i, cur, cur - prev)
            prev = cur

        # ── 4. Stream the trajectory ──────────────────────────────────────────
        log.info("Streaming %d waypoints to robot...", len(trajectory))
        success = client.stream_joint_trajectory(trajectory)

        if success:
            log.info("Motion complete! Waiting for robot to settle...")
            client.wait_for_stop(timeout=10.0)
        else:
            log.warning("Trajectory stream returned False – check robot alarms")

        client.stop_status_output()

    log.info("Done.")


if __name__ == "__main__":
    main()
