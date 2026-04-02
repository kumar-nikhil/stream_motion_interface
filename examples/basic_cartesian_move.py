"""
Example: Cartesian TCP Move via Stream Motion
=============================================
Manual: B-84904EN/01 – Section 4.1 (Cartesian format), Table 3.3c

Moves the TCP by a small delta in Cartesian space [X, Y, Z, W, P, R]:
  - XYZ in millimetres
  - WPR (orientation) in degrees

The trajectory is a straight Cartesian line using the same minimum-jerk
quintic polynomial as the joint-space examples.  Duration is computed from
the desired linear and angular TCP speed limits so you never need to tune
a fixed duration manually.

⚠  IMPORTANT — READ BEFORE RUNNING  ⚠
--------------------------------------
Cartesian streaming (DATA_FORMAT_CARTESIAN = 0) has additional constraints
compared to joint streaming:

1. The robot configuration (wrist/elbow flip) MUST NOT change during the move.
   If the straight-line path passes through a singularity or forces a config
   change, the controller raises MOTN-156 and stops.  Keep deltas small
   (≤ 50 mm / ≤ 20 deg) until you know your workspace is safe.

2. Start near the home position [0,0,0,0,-90,0] for predictable behaviour —
   run move_to_home.py first if needed.

3. If MOTN-156 fires: reduce the delta, or switch to joint-space streaming
   for large moves and use Cartesian only for fine positioning.

Verified working in ROBOGUIDE (CRX-10iA/L, SCALE≡speed-limited at 500 mm/s).

Prerequisites
-------------
Same as all other examples:
  - TP program running with IBGN start[1] / IBGN end[1]
  - AUTO mode, 100% override, no active faults
"""

import logging

from stream_motion import (
    StreamMotionClient,
    minimum_jerk_cartesian_trajectory,
)
from stream_motion.constants import (
    CRX_CART_LINEAR_MMS,
    CRX_CART_ANGULAR_DEGS,
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
# Cartesian delta applied to the robot's live TCP pose at IBGN.
# [X mm, Y mm, Z mm, W deg, P deg, R deg]
#
# Default: +20 mm in X only — conservative first test.
# The move stays within a small workspace region near wherever the robot is.
# Raise delta values gradually once confirmed fault-free.
DELTA_CART = [20.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# TCP speed limits for trajectory planning.
# T is computed so peak speed never exceeds these values.
# These are imported from constants.py:
#   CRX_CART_LINEAR_MMS   = 500.0 mm/s  (67% of 750 mm/s collaborative limit)
#   CRX_CART_ANGULAR_DEGS =  90.0 deg/s
# Override here if you want to test a different speed:
MAX_LINEAR_MMS   = CRX_CART_LINEAR_MMS
MAX_ANGULAR_DEGS = CRX_CART_ANGULAR_DEGS


def main() -> None:
    log.info("=" * 60)
    log.info("Stream Motion – Cartesian TCP Move")
    log.info("Robot: %s:%d", ROBOT_IP, ROBOT_PORT)
    log.info("Delta: X=%.1f  Y=%.1f  Z=%.1f  W=%.1f  P=%.1f  R=%.1f  [mm/deg]",
             *DELTA_CART)
    log.info("Speed limits: linear=%.0f mm/s  angular=%.0f deg/s",
             MAX_LINEAR_MMS, MAX_ANGULAR_DEGS)
    log.info("=" * 60)

    with StreamMotionClient(robot_ip=ROBOT_IP, robot_port=ROBOT_PORT) as client:

        client.start_status_output()
        log.info("Waiting for TP program to reach IBGN start[1]...")

        if not client.wait_for_ready(timeout=60.0):
            log.error("Robot not ready — check TP program, mode, and override")
            client.stop_status_output()
            return

        # Read live Cartesian pose so trajectory starts exactly here.
        # cart = [X, Y, Z, W, P, R] from the status packet (Section 3.3b).
        start_pose = client.get_current_cart()
        if start_pose is None:
            log.error("Could not read current Cartesian pose — aborting")
            client.stop_status_output()
            return

        end_pose = [start_pose[i] + DELTA_CART[i] for i in range(6)]

        log.info("Start pose: X=%.3f  Y=%.3f  Z=%.3f  W=%.3f  P=%.3f  R=%.3f",
                 *start_pose[:6])
        log.info("End pose:   X=%.3f  Y=%.3f  Z=%.3f  W=%.3f  P=%.3f  R=%.3f",
                 *end_pose[:6])

        # Check that the delta is non-trivial
        if all(abs(DELTA_CART[i]) < 0.01 for i in range(6)):
            log.info("Delta is zero — nothing to do.")
            client.stop_status_output()
            return

        # Plan: minimum-jerk Cartesian trajectory.
        # T is set by whichever constraint (linear or angular speed) is binding.
        trajectory = minimum_jerk_cartesian_trajectory(
            start_pose           = start_pose,
            end_pose             = end_pose,
            max_tcp_linear_mms   = MAX_LINEAR_MMS,
            max_tcp_angular_degs = MAX_ANGULAR_DEGS,
        )

        T = (len(trajectory) - 1) * 0.008
        log.info("Trajectory: %d waypoints  T=%.2f s", len(trajectory), T)

        # Log first few waypoints to verify smooth ramp-up
        log.info("First 4 waypoints [X, Y, Z]:")
        for i in range(min(4, len(trajectory))):
            p = trajectory[i]
            log.info("  [%d]  X=%.4f  Y=%.4f  Z=%.4f  W=%.4f  P=%.4f  R=%.4f",
                     i, p[0], p[1], p[2], p[3], p[4], p[5])

        log.info("Streaming %d Cartesian waypoints...", len(trajectory))
        success = client.stream_cartesian_trajectory(trajectory)

        if success:
            log.info("Motion complete! Waiting for robot to settle...")
            client.wait_for_stop(timeout=15.0)
        else:
            log.warning("Trajectory stream returned False — check robot alarms")

        client.stop_status_output()

    log.info("Done.")


if __name__ == "__main__":
    main()
