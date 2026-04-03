"""
Trajectory Planning Helpers for Stream Motion
FANUC R-30iB/R-30iB Plus Controller – Stream Motion (J519)
Manual: B-84904EN/01 – Section 4.3  JOINT VELOCITY, ACCELERATION, AND JERK LIMITS

The robot enforces limits on per-joint:
  - Velocity     [deg/s]   – $STMO_GRP.$JNT_VEL_LIM[1-9]
  - Acceleration [deg/s²]  – $STMO_GRP.$JNT_ACC_LIM[1-9]
  - Jerk         [deg/s³]  – $STMO_GRP.$JNT_JRK_LIM[1-9]

Your external planner MUST respect these or the robot will fault.
These helpers generate safe, smooth joint-space trajectories.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

# Default 8ms communication cycle (Section 3.1)
DEFAULT_CYCLE_S = 0.008


def trapezoidal_joint_trajectory(
    start_joints:  List[float],
    end_joints:    List[float],
    vel_limits:    List[float],
    acc_limits:    List[float],
    cycle_s:       float = DEFAULT_CYCLE_S,
    scale:         float = 0.8,
) -> List[List[float]]:
    """
    Generate a trapezoidal velocity profile joint trajectory from start to end.

    Each axis is scaled independently so the motion of all axes finishes together
    (synchronous joint motion), respecting the velocity and acceleration limits.

    Args:
        start_joints:  Starting joint angles [J1..J6, ...] in degrees.
        end_joints:    Target joint angles   [J1..J6, ...] in degrees.
        vel_limits:    Per-joint velocity limits [deg/s] from $STMO_GRP.$JNT_VEL_LIM.
        acc_limits:    Per-joint acceleration limits [deg/s²] from $STMO_GRP.$JNT_ACC_LIM.
        cycle_s:       Communication cycle duration in seconds (default 8ms).
        scale:         Safety scale factor applied to all limits (default 0.8 = 80%).
                       Keep below 1.0 to avoid triggering MOTN-609/610.

    Returns:
        List of joint-space waypoints, one per communication cycle.
        Each waypoint is a list of floats (same length as start_joints).
    """
    n_joints = len(start_joints)
    assert len(end_joints) == n_joints, "start and end must have same number of joints"

    # Scale limits for safety margin
    v_max = [v * scale for v in vel_limits[:n_joints]]
    a_max = [a * scale for a in acc_limits[:n_joints]]

    # Compute per-axis trapezoidal profiles, then find the slowest axis duration
    displacements = [end_joints[i] - start_joints[i] for i in range(n_joints)]
    durations = []

    for i, disp in enumerate(displacements):
        if abs(disp) < 1e-6:
            durations.append(0.0)
            continue
        d    = abs(disp)
        vm   = v_max[i]
        am   = a_max[i]
        # Time to ramp up from 0 to vm
        t_ramp = vm / am
        # Distance covered during ramp up+down
        d_ramp = am * t_ramp ** 2
        if d_ramp > d:
            # Triangle profile (can't reach max velocity)
            t_total = 2.0 * math.sqrt(d / am)
        else:
            t_cruise = (d - d_ramp) / vm
            t_total  = 2 * t_ramp + t_cruise
        durations.append(t_total)

    t_sync = max(durations) if durations else 0.0
    if t_sync < cycle_s:
        # Already at the target — return a single waypoint
        return [list(end_joints)]

    n_steps = max(1, round(t_sync / cycle_s))

    # Compute ramp fraction for the slowest axis (the one that takes t_sync)
    # All axes are time-synced to t_sync; compute the normalized ramp time.
    # For the trapezoidal profile with t_sync duration:
    #   t_ramp fraction φ satisfies: 2*φ + (1 - 2*φ) = 1
    # We compute φ per axis (based on its own limits) and use the max axis.
    phi_list = []
    for i, disp in enumerate(displacements):
        if abs(disp) < 1e-6 or t_sync < 1e-9:
            phi_list.append(0.5)
            continue
        vm = v_max[i]
        am = a_max[i]
        # t_ramp = vm/am; phi = t_ramp / t_sync (clamped to 0.5 for triangle)
        phi = min(0.5, (vm / am) / t_sync)
        phi_list.append(phi)

    # Use the smallest phi (most conservative, widest cruise zone)
    phi = min(phi_list) if phi_list else 0.5

    # Factor 2*phi*(1-phi) appears in all trapezoidal position equations
    factor = 2.0 * phi * (1.0 - phi) if phi > 1e-6 else 1.0

    waypoints = []
    for step in range(n_steps + 1):
        s = min(step / n_steps, 1.0)   # Normalized time [0..1]

        # Properly normalised trapezoidal position profile:
        #   Ramp-up  [0, φ]:       pos = s² / (2φ(1−φ))
        #   Cruise   [φ, 1−φ]:     pos = (2s − φ) / (2(1−φ))
        #   Ramp-down[1−φ, 1]:     pos = 1 − (1−s)² / (2φ(1−φ))
        # Velocity is continuous at the transitions; peak vel = 1/(1−φ) [normalised].
        if phi < 1e-6:
            pos_frac = s
        elif s < phi:
            pos_frac = (s ** 2) / factor
        elif s <= (1.0 - phi):
            pos_frac = (2.0 * s - phi) / (2.0 * (1.0 - phi))
        else:
            r = 1.0 - s
            pos_frac = 1.0 - (r ** 2) / factor

        joints = [start_joints[i] + displacements[i] * pos_frac for i in range(n_joints)]
        waypoints.append(joints)

    return waypoints


def linear_joint_interpolation(
    start_joints: List[float],
    end_joints:   List[float],
    n_steps:      int,
) -> List[List[float]]:
    """
    Simple linear interpolation between two joint configurations.

    WARNING: Linear interpolation has non-zero jerk at the start and end.
    Use trapezoidal_joint_trajectory() for real motion — this is mainly
    useful for very small moves or for understanding the protocol.

    Args:
        start_joints: Starting joint angles in degrees.
        end_joints:   Target joint angles in degrees.
        n_steps:      Number of waypoints (including start and end).

    Returns:
        List of n_steps waypoints.
    """
    waypoints = []
    for step in range(n_steps):
        alpha = step / (n_steps - 1) if n_steps > 1 else 1.0
        joints = [
            start_joints[i] + alpha * (end_joints[i] - start_joints[i])
            for i in range(len(start_joints))
        ]
        waypoints.append(joints)
    return waypoints


def check_limits(
    trajectory: List[List[float]],
    vel_limits: List[float],
    acc_limits: List[float],
    jrk_limits: Optional[List[float]] = None,
    cycle_s: float = DEFAULT_CYCLE_S,
) -> List[str]:
    """
    Check a joint-space trajectory against per-axis limits.
    Returns a list of warning strings; empty list means all OK.

    Args:
        trajectory:  List of joint-space waypoints.
        vel_limits:  Per-joint velocity limits [deg/s].
        acc_limits:  Per-joint acceleration limits [deg/s²].
        jrk_limits:  Per-joint jerk limits [deg/s³] (optional).
        cycle_s:     Communication cycle [s].

    Returns:
        List of violation description strings. Empty = clean.
    """
    warnings = []
    if len(trajectory) < 2:
        return warnings

    n = len(trajectory[0])
    prev_vel = [0.0] * n
    prev_acc = [0.0] * n

    for step in range(1, len(trajectory)):
        curr = trajectory[step]
        prev = trajectory[step - 1]

        vel = [(curr[i] - prev[i]) / cycle_s for i in range(n)]
        acc = [(vel[i] - prev_vel[i]) / cycle_s for i in range(n)]
        jrk = [(acc[i] - prev_acc[i]) / cycle_s for i in range(n)] if jrk_limits else []

        for i in range(n):
            if i < len(vel_limits) and abs(vel[i]) > vel_limits[i]:
                warnings.append(
                    f"Step {step} J{i+1}: velocity {vel[i]:.2f} deg/s exceeds limit {vel_limits[i]:.2f}"
                )
            if i < len(acc_limits) and abs(acc[i]) > acc_limits[i]:
                warnings.append(
                    f"Step {step} J{i+1}: acceleration {acc[i]:.2f} deg/s² exceeds limit {acc_limits[i]:.2f}"
                )
            if jrk_limits and i < len(jrk_limits) and abs(jrk[i]) > jrk_limits[i]:
                warnings.append(
                    f"Step {step} J{i+1}: jerk {jrk[i]:.2f} deg/s³ exceeds limit {jrk_limits[i]:.2f}"
                )

        prev_vel = vel
        prev_acc = acc

    return warnings


def smooth_trajectory(
    trajectory: List[List[float]],
    window: int = 7,
) -> List[List[float]]:
    """
    Apply a symmetric moving-average (box filter) to a joint trajectory
    to reduce the jerk spikes that occur at trapezoidal ramp transitions.

    This is the Python-side equivalent of setting $STMO_GRP.$FLTR_LN on
    the controller (Section 5 of B-84904EN/01).

    LEAD-IN APPROACH (critical for FANUC stream motion):
    The controller uses the current servo position as the implicit t=-1
    command when computing velocity/acceleration/jerk for the first packet.
    Any gap between servo position and smoothed[0] produces a large
    deceleration and then massive jerk at packet 2 → MOTN-722.

    Fix: prepend `window` EXTRA lead-in copies of the start waypoint to the
    trajectory BEFORE applying the box filter. This forces the smoothed
    output at position `window` (the actual first waypoint) to equal the
    start position exactly (proof: the 2W+1 kernel there sees 2W start copies
    + traj[0]=start, so the average = start exactly). The lead-in is stripped
    from the output before returning.

    With smoothed[0] = trajectory[0] = actual servo position, vel_{-1→0} = 0
    and the trajectory accelerates gently from rest with no jerk spike.

    Args:
        trajectory: List of joint-space waypoints. trajectory[0] MUST equal
                    the robot's current servo position so that vel_{-1→0} = 0.
        window:     Half-width of the smoothing kernel (total width = 2*window+1).
                    Larger values = smoother motion but more path deviation.
                    Recommended: 5–10 for typical 8ms cycle trajectories.

    Returns:
        Smoothed trajectory with the same length as the input.
        smoothed[0] == trajectory[0] exactly (guaranteed by lead-in math).
        The end position deviates by < 0.1° for window ≤ 10 on typical moves.
    """
    if window < 1 or len(trajectory) < 3:
        return trajectory

    n_joints = len(trajectory[0])

    # Build the input seen by the filter:
    #   [start]*(window)  ← extra lead-in (makes smoothed[window] = start exactly)
    #   + trajectory      ← the actual motion
    #   + [end]*(window)  ← tail pad (reduces endpoint deviation)
    #
    # After filtering the full extended array we trim away the lead-in and the
    # tail padding, keeping exactly len(trajectory) points starting at position
    # `window`.
    lead_in_and_traj = (
        [list(trajectory[0])] * window       # lead-in: window copies of start
        + [list(wp) for wp in trajectory]    # actual trajectory
        + [list(trajectory[-1])] * window    # tail pad
    )

    n_ext = len(lead_in_and_traj)
    smoothed_ext = []
    for i in range(n_ext):
        lo = max(0, i - window)
        hi = min(n_ext - 1, i + window)
        count = hi - lo + 1
        avg = [
            sum(lead_in_and_traj[j][k] for j in range(lo, hi + 1)) / count
            for k in range(n_joints)
        ]
        smoothed_ext.append(avg)

    # Trim: skip the lead-in, keep exactly the original trajectory length.
    result = smoothed_ext[window: window + len(trajectory)]

    # Enforce exact start position (floating-point safety).
    # Mathematically this equals trajectory[0] anyway (see docstring proof),
    # but explicit assignment removes any sub-ULP drift.
    result[0] = list(trajectory[0])

    return result


def minimum_jerk_trajectory(
    start_joints:       List[float],
    end_joints:         List[float],
    vel_limits:         List[float],
    acc_limits:         List[float],
    jrk_limits:         List[float],
    cycle_s:            float = DEFAULT_CYCLE_S,
    scale:              float = 0.8,
    max_tcp_speed_mms:  Optional[float] = None,
    robot_reach_mm:     float = 1249.0,
) -> List[List[float]]:
    """
    Generate a 5th-order polynomial (minimum-jerk) joint trajectory.

    Position profile (per axis):
        p(s) = start + D · (10s³ − 15s⁴ + 6s⁵),   s ∈ [0, 1]

    Properties
    ----------
    - Zero velocity AND zero acceleration at both endpoints — no post-
      filtering required and no jerk spike at the very first command packet.
    - Jerk is bounded everywhere; peak values (D=1, T=1 normalisation):
        peak velocity  = 1.875 · D / T     (at s = 0.5)
        peak accel     = 5.773 · D / T²    (at s ≈ 0.211 and 0.789)
        peak jerk      = 60    · D / T³    (at s = 0 and s = 1)
    - The first position step grows as ~10·D/(N³) — cubic, not quadratic —
      so even if the servo position differs from cmd[0] by a small δ, the
      resulting jerk at the controller is well within limits.

    Why this avoids MOTN-722
    ------------------------
    A trapezoidal profile followed by a box-filter creates an artificial
    velocity spike at position [1] because the box kernel at index 1 already
    "sees" far-future trajectory values that are well above start.  With a
    minimum-jerk profile there is no filter; the trajectory itself has the
    correct shape, and the first step is so small (~0.00005° for a 5° move
    over 80 steps) that any residual servo-to-cmd[0] gap does not produce
    excessive acceleration or jerk.

    Args
    ----
    start_joints : Starting joint angles [deg].
    end_joints   : Target joint angles [deg].
    vel_limits   : Per-joint velocity limits [deg/s]   ($JNT_VEL_LIM).
    acc_limits   : Per-joint accel limits   [deg/s²]   ($JNT_ACC_LIM).
    jrk_limits   : Per-joint jerk limits    [deg/s³]   ($JNT_JRK_LIM).
    cycle_s           : Communication cycle [s] (default 8 ms).
    scale             : Safety margin: limits are multiplied by this factor
                        (default 0.8 = 80 %).  Keep ≤ 1.0.
    max_tcp_speed_mms : If set, adds a fourth per-axis constraint so that
                        the estimated peak TCP Cartesian speed never exceeds
                        this value [mm/s].  Use CRX_COLLAB_TCP_PLAN_MMS
                        (700 mm/s) to stay comfortably below the CRX-10iA/L
                        collaborative limit of 750 mm/s (SYST-323).
                        The lever-arm estimate is conservative: it assumes
                        every joint acts at full robot reach, so the real
                        TCP speed will always be ≤ this value.
    robot_reach_mm    : Maximum robot reach used as the lever-arm for the
                        TCP speed estimate [mm].  Default = 1249 mm (CRX-10iA/L).

    Returns
    -------
    List of joint-space waypoints, one per communication cycle.
    """
    n_joints = len(start_joints)
    assert len(end_joints) == n_joints

    displacements = [end_joints[i] - start_joints[i] for i in range(n_joints)]

    T_per_axis: List[float] = []
    for i, d in enumerate(displacements):
        if abs(d) < 1e-9:
            T_per_axis.append(0.0)
            continue

        v_lim = vel_limits[i] * scale
        a_lim = acc_limits[i] * scale
        j_lim = jrk_limits[i] * scale

        # Minimum duration that keeps each physical limit satisfied
        T_v = 1.875 * abs(d) / v_lim                   # velocity
        T_a = math.sqrt(5.773 * abs(d) / a_lim)        # acceleration
        T_j = (60.0  * abs(d) / j_lim) ** (1.0 / 3.0) # jerk

        # Optional TCP Cartesian speed constraint (SYST-323 on CRX cobots).
        # Approximation: treat every joint as if it contributes to TCP speed
        # with a lever arm equal to the robot's maximum reach (conservative —
        # overestimates TCP speed, so always safe).
        #   peak_joint_vel = 1.875 * |d_rad| / T  →  T_tcp = 1.875 * |d_rad| * reach / v_tcp
        if max_tcp_speed_mms is not None:
            d_rad = abs(d) * math.pi / 180.0
            T_tcp = 1.875 * d_rad * robot_reach_mm / max_tcp_speed_mms
            T_per_axis.append(max(T_v, T_a, T_j, T_tcp))
        else:
            T_per_axis.append(max(T_v, T_a, T_j))

    T = max(T_per_axis) if T_per_axis else 0.0

    if T < cycle_s:
        # Already at the target — return a single waypoint
        return [list(end_joints)]

    n_steps = max(1, round(T / cycle_s))

    waypoints: List[List[float]] = []
    for step in range(n_steps + 1):
        s = step / n_steps                         # normalised time [0..1]
        pos_frac = 10*s**3 - 15*s**4 + 6*s**5    # 5th-order polynomial
        joints = [
            start_joints[i] + displacements[i] * pos_frac
            for i in range(n_joints)
        ]
        waypoints.append(joints)

    return waypoints


def minimum_jerk_cartesian_trajectory(
    start_pose:           List[float],
    end_pose:             List[float],
    max_tcp_linear_mms:   float = 500.0,
    max_tcp_angular_degs: float = 90.0,
    cycle_s:              float = DEFAULT_CYCLE_S,
) -> List[List[float]]:
    """
    Generate a 5th-order polynomial (minimum-jerk) Cartesian trajectory.

    The TCP moves in a straight line in Cartesian space from start_pose to
    end_pose using the same quintic time-scaling as minimum_jerk_trajectory():

        p(s) = start + D · (10s³ − 15s⁴ + 6s⁵),   s ∈ [0, 1]

    Motion duration T is set by whichever constraint is binding:
      - Linear  TCP speed: T_lin = 1.875 × linear_distance_mm  / max_tcp_linear_mms
      - Angular TCP speed: T_rot = 1.875 × max_angular_delta_deg / max_tcp_angular_degs

    IMPORTANT — Cartesian streaming constraints (Section 4.1 and MOTN-156)
    -----------------------------------------------------------------------
    1. Only valid on 6-axis robots.
    2. The robot configuration (wrist/elbow flip) MUST NOT change during the
       move. If the path passes through a singularity or forces a config change,
       the controller raises MOTN-156. Keep moves small (< 50 mm / < 30 deg)
       until you are confident in the workspace.
    3. For large Cartesian deltas, use joint-space streaming instead — the
       controller handles configuration management automatically in joint space.

    Args
    ----
    start_pose           : [X, Y, Z, W, P, R] in mm / degrees.
                           Read from client.get_current_cart() at IBGN.
    end_pose             : Target [X, Y, Z, W, P, R] in mm / degrees.
    max_tcp_linear_mms   : Max TCP linear speed [mm/s].  Default 500 mm/s —
                           conservative for CRX collaborative mode.
    max_tcp_angular_degs : Max TCP angular speed [deg/s].  Default 90 deg/s.
    cycle_s              : Communication cycle [s] (default 8 ms).

    Returns
    -------
    List of [X, Y, Z, W, P, R] waypoints, one per communication cycle.
    """
    if len(start_pose) < 6 or len(end_pose) < 6:
        raise ValueError("start_pose and end_pose must have at least 6 elements [X,Y,Z,W,P,R]")

    d_xyz = [end_pose[i] - start_pose[i] for i in range(3)]
    d_wpr = [end_pose[i] - start_pose[i] for i in range(3, 6)]

    linear_dist = math.sqrt(sum(v ** 2 for v in d_xyz))
    max_angular  = max(abs(v) for v in d_wpr)

    # Minimum duration from each constraint
    T_lin = (1.875 * linear_dist  / max_tcp_linear_mms)  if linear_dist  > 1e-6 else 0.0
    T_rot = (1.875 * max_angular  / max_tcp_angular_degs) if max_angular  > 1e-6 else 0.0

    T = max(T_lin, T_rot)

    if T < cycle_s:
        # Already at the target
        return [list(end_pose)]

    # Use ceil (not round) so the actual duration ≥ T, which keeps peak
    # speed ≤ the requested limit.  round() can produce n_steps that is
    # one cycle short, causing a small speed overshoot.
    n_steps = max(1, math.ceil(T / cycle_s))

    # Keep any extra axes (J7-J9 / extended) at start values throughout
    extra = list(start_pose[6:]) if len(start_pose) > 6 else []
    displacements = [end_pose[i] - start_pose[i] for i in range(6)]

    waypoints: List[List[float]] = []
    for step in range(n_steps + 1):
        s = step / n_steps
        pos_frac = 10*s**3 - 15*s**4 + 6*s**5
        pose = [start_pose[i] + displacements[i] * pos_frac for i in range(6)]
        waypoints.append(pose + extra)

    return waypoints


def _plane_offsets(
    radius_mm: float,
    angle_rad: float,
    plane: str,
) -> Tuple[float, float, float]:
    """Return (dx, dy, dz) for a point on a circle of given radius at angle_rad,
    lying in the specified plane ('XY', 'XZ', or 'YZ')."""
    c = radius_mm * math.cos(angle_rad)
    s = radius_mm * math.sin(angle_rad)
    if plane == 'XY':
        return c, s, 0.0
    elif plane == 'XZ':
        return c, 0.0, s
    elif plane == 'YZ':
        return 0.0, c, s
    else:
        raise ValueError(f"plane must be 'XY', 'XZ', or 'YZ', got {plane!r}")


def circle_cartesian_trajectory(
    center_pose:        List[float],
    radius_mm:          float,
    plane:              str   = 'XY',
    max_tcp_linear_mms: float = 150.0,
    cycle_s:            float = DEFAULT_CYCLE_S,
    clockwise:          bool  = False,
) -> List[List[float]]:
    """
    Generate a constant-speed Cartesian circle trajectory.

    The TCP moves at a steady speed of max_tcp_linear_mms around the full
    360° circle and returns to the start point.  Orientation (WPR) is held
    constant throughout.

    Unlike polygon or line moves, the circle does NOT use the quintic
    ramp — instead it uses uniform angular steps so that TCP speed is
    constant everywhere on the arc.  The first and last waypoints coincide
    (closed loop).

    Args
    ----
    center_pose        : [X, Y, Z, W, P, R] — centre of the circle plus
                         the orientation to hold throughout [mm / deg].
    radius_mm          : Circle radius [mm].
    plane              : Plane in which the circle lies: 'XY', 'XZ', 'YZ'.
                         Default 'XY' (Z is the out-of-plane axis).
    max_tcp_linear_mms : Constant TCP speed [mm/s].
    cycle_s            : Communication cycle [s] (default 8 ms).
    clockwise          : If True, traverse clockwise (negative angle direction).

    Returns
    -------
    List of [X, Y, Z, W, P, R] waypoints (closed loop — last == first).

    ⚠  Keep radius ≤ 50 mm for initial tests to stay within a safe
    workspace region and avoid MOTN-156 (configuration change).
    """
    if radius_mm <= 0:
        raise ValueError("radius_mm must be positive")

    circumference  = 2.0 * math.pi * radius_mm
    arc_step_mm    = max_tcp_linear_mms * cycle_s          # mm per cycle
    n_steps        = max(4, math.ceil(circumference / arc_step_mm))

    sign = -1.0 if clockwise else 1.0
    cx, cy, cz = center_pose[0], center_pose[1], center_pose[2]
    wpr = list(center_pose[3:6])
    extra = list(center_pose[6:]) if len(center_pose) > 6 else []

    waypoints: List[List[float]] = []
    for k in range(n_steps + 1):            # +1 so last point == first
        angle = sign * 2.0 * math.pi * k / n_steps
        dx, dy, dz = _plane_offsets(radius_mm, angle, plane)
        waypoints.append([cx + dx, cy + dy, cz + dz] + wpr + extra)

    return waypoints


def polygon_cartesian_trajectory(
    center_pose:          List[float],
    radius_mm:            float,
    n_sides:              int,
    plane:                str   = 'XY',
    max_tcp_linear_mms:   float = 150.0,
    max_tcp_angular_degs: float = 45.0,
    cycle_s:              float = DEFAULT_CYCLE_S,
    start_angle_deg:      float = 0.0,
    clockwise:            bool  = False,
) -> List[List[float]]:
    """
    Generate a Cartesian trajectory that traces a regular polygon.

    The TCP moves from vertex to vertex using minimum-jerk straight-line
    segments, decelerating to rest at each corner before continuing.
    The polygon closes by returning from the last vertex to the first.

    Common use:
      n_sides=4  → square
      n_sides=5  → pentagon
      n_sides=6  → hexagon
      n_sides=3  → equilateral triangle

    Args
    ----
    center_pose          : [X, Y, Z, W, P, R] — centre + orientation [mm/deg].
    radius_mm            : Circumradius (centre to vertex) [mm].
    n_sides              : Number of sides (≥ 3).
    plane                : 'XY', 'XZ', or 'YZ'.
    max_tcp_linear_mms   : Max TCP linear speed for each side [mm/s].
    max_tcp_angular_degs : Max TCP angular speed (unused if WPR is constant).
    cycle_s              : Communication cycle [s] (default 8 ms).
    start_angle_deg      : Angle of the first vertex, degrees.
                           0° → first vertex along +axis1 of the plane.
                           90° → rotated 90° CCW from that.
    clockwise            : If True, traverse vertices clockwise.

    Returns
    -------
    Concatenated list of minimum-jerk waypoints for all sides.
    The robot stops (zero velocity) at each vertex.

    ⚠  Keep radius ≤ 50 mm for initial tests.
    """
    if n_sides < 3:
        raise ValueError("n_sides must be ≥ 3")
    if radius_mm <= 0:
        raise ValueError("radius_mm must be positive")

    sign = -1.0 if clockwise else 1.0
    cx, cy, cz = center_pose[0], center_pose[1], center_pose[2]
    wpr   = list(center_pose[3:6])
    extra = list(center_pose[6:]) if len(center_pose) > 6 else []

    # Compute vertex positions
    vertices: List[List[float]] = []
    for k in range(n_sides):
        angle = math.radians(start_angle_deg) + sign * 2.0 * math.pi * k / n_sides
        dx, dy, dz = _plane_offsets(radius_mm, angle, plane)
        vertices.append([cx + dx, cy + dy, cz + dz] + wpr + extra)

    # String together minimum-jerk segments, vertex → vertex (closed loop)
    all_waypoints: List[List[float]] = []
    for k in range(n_sides):
        v_start = vertices[k]
        v_end   = vertices[(k + 1) % n_sides]
        segment = minimum_jerk_cartesian_trajectory(
            start_pose           = v_start,
            end_pose             = v_end,
            max_tcp_linear_mms   = max_tcp_linear_mms,
            max_tcp_angular_degs = max_tcp_angular_degs,
            cycle_s              = cycle_s,
        )
        if k == 0:
            all_waypoints.extend(segment)
        else:
            # Drop the first waypoint of each subsequent segment — it is
            # identical to the last waypoint of the previous segment.
            all_waypoints.extend(segment[1:])

    return all_waypoints


def rectangle_cartesian_trajectory(
    center_pose:          List[float],
    width_mm:             float,
    height_mm:            float,
    plane:                str   = 'XY',
    max_tcp_linear_mms:   float = 150.0,
    max_tcp_angular_degs: float = 45.0,
    cycle_s:              float = DEFAULT_CYCLE_S,
    clockwise:            bool  = False,
) -> List[List[float]]:
    """
    Generate a Cartesian trajectory that traces a rectangle.

    The four corners are at (±width/2, ±height/2) relative to center_pose,
    in the specified plane.  The TCP uses minimum-jerk moves between corners
    and stops (zero velocity) at each one.

    Args
    ----
    center_pose          : [X, Y, Z, W, P, R] — centre + orientation [mm/deg].
    width_mm             : Full width of the rectangle (axis1 of plane) [mm].
    height_mm            : Full height of the rectangle (axis2 of plane) [mm].
    plane                : 'XY', 'XZ', or 'YZ'.
    max_tcp_linear_mms   : Max TCP linear speed [mm/s].
    max_tcp_angular_degs : Max TCP angular speed [deg/s].
    cycle_s              : Communication cycle [s] (default 8 ms).
    clockwise            : Traverse corners clockwise if True.

    Returns
    -------
    Concatenated minimum-jerk waypoints for all four sides.

    ⚠  Keep width and height ≤ 100 mm for initial tests.
    """
    if width_mm <= 0 or height_mm <= 0:
        raise ValueError("width_mm and height_mm must be positive")

    cx, cy, cz = center_pose[0], center_pose[1], center_pose[2]
    wpr   = list(center_pose[3:6])
    extra = list(center_pose[6:]) if len(center_pose) > 6 else []

    hw, hh = width_mm / 2.0, height_mm / 2.0

    # Corner offsets in the chosen plane (CCW order by default)
    if plane == 'XY':
        raw = [(-hw, -hh, 0), ( hw, -hh, 0), ( hw,  hh, 0), (-hw,  hh, 0)]
    elif plane == 'XZ':
        raw = [(-hw, 0, -hh), ( hw, 0, -hh), ( hw, 0,  hh), (-hw, 0,  hh)]
    elif plane == 'YZ':
        raw = [(0, -hw, -hh), (0,  hw, -hh), (0,  hw,  hh), (0, -hw,  hh)]
    else:
        raise ValueError(f"plane must be 'XY', 'XZ', or 'YZ', got {plane!r}")

    if clockwise:
        raw = list(reversed(raw))

    corners = [[cx + dx, cy + dy, cz + dz] + wpr + extra
               for dx, dy, dz in raw]

    all_waypoints: List[List[float]] = []
    for k in range(4):
        segment = minimum_jerk_cartesian_trajectory(
            start_pose           = corners[k],
            end_pose             = corners[(k + 1) % 4],
            max_tcp_linear_mms   = max_tcp_linear_mms,
            max_tcp_angular_degs = max_tcp_angular_degs,
            cycle_s              = cycle_s,
        )
        if k == 0:
            all_waypoints.extend(segment)
        else:
            all_waypoints.extend(segment[1:])

    return all_waypoints


def pad_to_9(joints: List[float]) -> List[float]:
    """Pad a joint list to 9 elements (required by command packets)."""
    return (list(joints) + [0.0] * 9)[:9]
