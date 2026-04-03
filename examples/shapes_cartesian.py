"""
Example: Cartesian Shape Tracing via Stream Motion
===================================================
Manual: B-84904EN/01 – Section 4.1 (Cartesian format)

Traces basic 2D shapes in Cartesian space using the robot's live TCP pose
as the centre.  All shapes are drawn in the XY plane (Z constant) by default.

Supported shapes
----------------
  circle     — constant-speed, closed loop
  square     — 4-sided polygon (regular)
  rectangle  — width × height, stops at each corner
  triangle   — 3-sided polygon
  pentagon   — 5-sided polygon
  hexagon    — 6-sided polygon
  polygon    — any N-sided regular polygon (set N_SIDES below)

Change SHAPE and SIZE_MM below, then run.

⚠  IMPORTANT — READ BEFORE RUNNING  ⚠
--------------------------------------
1. Run move_to_home.py FIRST so the robot is in a known, safe configuration.
2. Keep SIZE_MM ≤ 50 mm for initial tests.
3. The shape is drawn relative to the live TCP pose at IBGN — the robot
   draws the shape wherever it currently is.
4. MOTN-156 (config change) fires if the path crosses a singularity.
   Reduce SIZE_MM or choose a different start pose if this happens.
5. For polygons: the robot stops at each corner (zero velocity).
   For circles:  the robot moves at constant speed without stopping.

Prerequisites
-------------
  - TP program running with IBGN start[1] / IBGN end[1]
  - AUTO mode, 100% override, no active faults
"""

import logging

from stream_motion import (
    StreamMotionClient,
    circle_cartesian_trajectory,
    polygon_cartesian_trajectory,
    rectangle_cartesian_trajectory,
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

# ── Shape Selection ───────────────────────────────────────────────────────────
# Choose one of: "circle", "square", "rectangle", "triangle",
#                "pentagon", "hexagon", "polygon"
SHAPE = "circle"

# Size:
#   circle    → radius in mm
#   square    → circumradius (centre to vertex) in mm
#   rectangle → (width, height) defined by RECT_WIDTH_MM / RECT_HEIGHT_MM
#   polygon   → circumradius in mm
SIZE_MM = 50.0          # Start at 50 mm and raise once confirmed fault-free

# Rectangle dimensions (only used when SHAPE = "rectangle")
RECT_WIDTH_MM  = 80.0   # mm
RECT_HEIGHT_MM = 50.0   # mm

# For "polygon" shape: how many sides?
N_SIDES = 7             # 3=triangle, 4=square, 5=pentagon, 6=hexagon, 7=heptagon …

# Which plane to draw in
PLANE = "XY"            # "XY" (Z constant), "XZ" (Y constant), "YZ" (X constant)

# Clockwise or counter-clockwise
CLOCKWISE = False

# Speed — defaults to the library's confirmed-safe values
MAX_LINEAR_MMS   = CRX_CART_LINEAR_MMS    # 150 mm/s
MAX_ANGULAR_DEGS = CRX_CART_ANGULAR_DEGS  # 45 deg/s


# ── Shape name → polygon sides mapping ───────────────────────────────────────
SHAPE_SIDES = {
    "triangle": 3,
    "square":   4,
    "pentagon": 5,
    "hexagon":  6,
    "polygon":  N_SIDES,
}


def build_trajectory(center_pose):
    """Return the full waypoint list for the selected shape."""

    shape = SHAPE.lower()

    if shape == "circle":
        traj = circle_cartesian_trajectory(
            center_pose        = center_pose,
            radius_mm          = SIZE_MM,
            plane              = PLANE,
            max_tcp_linear_mms = MAX_LINEAR_MMS,
            clockwise          = CLOCKWISE,
        )
        log.info("Shape: CIRCLE  radius=%.1f mm  plane=%s", SIZE_MM, PLANE)

    elif shape == "rectangle":
        traj = rectangle_cartesian_trajectory(
            center_pose          = center_pose,
            width_mm             = RECT_WIDTH_MM,
            height_mm            = RECT_HEIGHT_MM,
            plane                = PLANE,
            max_tcp_linear_mms   = MAX_LINEAR_MMS,
            max_tcp_angular_degs = MAX_ANGULAR_DEGS,
            clockwise            = CLOCKWISE,
        )
        log.info("Shape: RECTANGLE  %.1f × %.1f mm  plane=%s",
                 RECT_WIDTH_MM, RECT_HEIGHT_MM, PLANE)

    elif shape in SHAPE_SIDES:
        n = SHAPE_SIDES[shape]
        traj = polygon_cartesian_trajectory(
            center_pose          = center_pose,
            radius_mm            = SIZE_MM,
            n_sides              = n,
            plane                = PLANE,
            max_tcp_linear_mms   = MAX_LINEAR_MMS,
            max_tcp_angular_degs = MAX_ANGULAR_DEGS,
            clockwise            = CLOCKWISE,
        )
        log.info("Shape: %s (%d sides)  radius=%.1f mm  plane=%s",
                 shape.upper(), n, SIZE_MM, PLANE)

    else:
        raise ValueError(
            f"Unknown shape {SHAPE!r}. Choose: circle, square, rectangle, "
            f"triangle, pentagon, hexagon, polygon"
        )

    return traj


def main() -> None:
    log.info("=" * 60)
    log.info("Stream Motion – Cartesian Shape Tracing")
    log.info("Robot: %s:%d", ROBOT_IP, ROBOT_PORT)
    log.info("Shape: %s  |  Plane: %s  |  Speed: %.0f mm/s",
             SHAPE.upper(), PLANE, MAX_LINEAR_MMS)
    log.info("=" * 60)

    with StreamMotionClient(robot_ip=ROBOT_IP, robot_port=ROBOT_PORT) as client:

        client.start_status_output()
        log.info("Waiting for TP program to reach IBGN start[1]...")

        if not client.wait_for_ready(timeout=60.0):
            log.error("Robot not ready — check TP program, mode, and override")
            client.stop_status_output()
            return

        # Read live Cartesian pose — this becomes the centre of the shape
        center_pose = client.get_current_cart()
        if center_pose is None:
            log.error("Could not read Cartesian pose — aborting")
            client.stop_status_output()
            return

        log.info("Centre pose: X=%.3f  Y=%.3f  Z=%.3f  W=%.3f  P=%.3f  R=%.3f",
                 *center_pose[:6])

        # Build trajectory for selected shape
        try:
            trajectory = build_trajectory(center_pose)
        except ValueError as exc:
            log.error("%s", exc)
            client.stop_status_output()
            return

        T = (len(trajectory) - 1) * 0.008
        log.info("Trajectory: %d waypoints  T=%.2f s", len(trajectory), T)

        # Log first vertex (start point) and a midpoint for sanity check
        if trajectory:
            p0 = trajectory[0]
            pm = trajectory[len(trajectory) // 4]
            log.info("Start:  X=%.3f  Y=%.3f  Z=%.3f", p0[0], p0[1], p0[2])
            log.info("1/4pt:  X=%.3f  Y=%.3f  Z=%.3f", pm[0], pm[1], pm[2])

        log.info("Streaming %d waypoints...", len(trajectory))
        success = client.stream_cartesian_trajectory(trajectory)

        if success:
            log.info("Shape complete! Waiting for robot to settle...")
            client.wait_for_stop(timeout=30.0)
        else:
            log.warning("Trajectory stream returned False — check robot alarms")

        client.stop_status_output()

    log.info("Done.")


if __name__ == "__main__":
    main()
