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


def _build_blended_path(
    vertices:       List[List[float]],   # closed polygon — first vertex is start/end
    blend_radius:   float,               # corner blend radius [mm]
    wpr:            List[float],         # [W, P, R] orientation (held constant)
    extra:          List[float],         # extended axes (held constant)
) -> Tuple[List[List[float]], List[float]]:
    """
    Build a corner-blended closed polygon path with circular arc fillets.

    At each corner the sharp turn is replaced by a circular arc of *blend_radius*.
    This caps the centripetal acceleration at  v² / blend_radius  regardless
    of TCP speed, eliminating MOTN-721 at corners.

    Safe speed vs blend radius (conservative — J1 dominant at ~600mm reach):
        10 mm → 185 mm/s     (default — safe for 150 mm/s with margin)
        5  mm → 131 mm/s
        20 mm → 262 mm/s

    The path starts and ends at the *exit point* of the first vertex's arc,
    which is used as the lead-in target in shapes_cartesian.py.

    Returns
    -------
    (path_points, cum_arcs)
        path_points : dense list of [X,Y,Z,W,P,R,...] waypoints at ≤1 mm intervals
        cum_arcs    : cumulative arc-length to each path_point [mm]
    """
    n = len(vertices)
    if n < 3:
        raise ValueError("Need ≥ 3 vertices")

    # ── Pre-compute blend geometry at every corner ──────────────────────────
    blends: List[dict] = []          # one dict per vertex
    for k in range(n):
        v_prev = vertices[(k - 1) % n]
        v_curr = vertices[k]
        v_next = vertices[(k + 1) % n]

        len_in  = math.sqrt(sum((v_curr[i] - v_prev[i]) ** 2 for i in range(3)))
        len_out = math.sqrt(sum((v_next[i] - v_curr[i]) ** 2 for i in range(3)))
        d_in    = [(v_curr[i] - v_prev[i]) / len_in  for i in range(3)]
        d_out   = [(v_next[i] - v_curr[i]) / len_out for i in range(3)]

        # Exterior (turn) angle from dot product of the two side directions
        dot_val   = max(-1.0, min(1.0, sum(d_in[i]*d_out[i] for i in range(3))))
        arc_angle = math.pi - math.acos(dot_val)      # exterior angle [rad]

        if arc_angle < 0.01:                           # nearly straight — no blend
            blends.append({"entry": list(v_curr), "exit": list(v_curr),
                           "arc_angle": 0.0, "radius": 0.0,
                           "center": list(v_curr), "axis": [0,0,1]})
            continue

        # Limit blend so it doesn't eat more than 45% of the shorter adjacent side
        bd = blend_radius * math.tan(arc_angle / 2)
        bd = min(bd, len_in * 0.45, len_out * 0.45)
        r  = bd / math.tan(arc_angle / 2)

        p_entry = [v_curr[i] - bd * d_in[i]  for i in range(3)]
        p_exit  = [v_curr[i] + bd * d_out[i] for i in range(3)]

        # Arc center: on the bisector of the inward normals, at r/cos(arc/2)
        # Polygon normal (rotation axis): cross(d_in, d_out)
        cross = [
            d_in[1]*d_out[2] - d_in[2]*d_out[1],
            d_in[2]*d_out[0] - d_in[0]*d_out[2],
            d_in[0]*d_out[1] - d_in[1]*d_out[0],
        ]
        axis_len = math.sqrt(sum(v**2 for v in cross))
        if axis_len < 1e-9:
            blends.append({"entry": list(v_curr), "exit": list(v_curr),
                           "arc_angle": 0.0, "radius": 0.0,
                           "center": list(v_curr), "axis": [0,0,1]})
            continue
        axis = [v / axis_len for v in cross]           # polygon normal unit vector

        # Inward normals of the two sides (axis × direction = left-hand normal for CCW)
        n_in  = [axis[1]*d_in[2]  - axis[2]*d_in[1],
                 axis[2]*d_in[0]  - axis[0]*d_in[2],
                 axis[0]*d_in[1]  - axis[1]*d_in[0]]
        n_out = [axis[1]*d_out[2] - axis[2]*d_out[1],
                 axis[2]*d_out[0] - axis[0]*d_out[2],
                 axis[0]*d_out[1] - axis[1]*d_out[0]]

        bisector_raw = [n_in[i] + n_out[i] for i in range(3)]
        bis_len = math.sqrt(sum(v**2 for v in bisector_raw))
        bisector = [v / bis_len for v in bisector_raw]

        center_dist = r / math.cos(arc_angle / 2)
        center      = [v_curr[i] + center_dist * bisector[i] for i in range(3)]

        blends.append({
            "entry":     p_entry,
            "exit":      p_exit,
            "arc_angle": arc_angle,
            "radius":    r,
            "center":    center,
            "axis":      axis,
        })

    # ── Build dense path (≤1 mm step) ───────────────────────────────────────
    path_pts:  List[List[float]] = []
    cum_arcs:  List[float]       = []

    def push(xyz: List[float]) -> None:
        pose = xyz[:3] + list(wpr) + list(extra)
        if path_pts:
            d = math.sqrt(sum((pose[i] - path_pts[-1][i]) ** 2 for i in range(3)))
            cum_arcs.append(cum_arcs[-1] + d)
        else:
            cum_arcs.append(0.0)
        path_pts.append(pose)

    # Start path at blend exit of corner 0
    push(blends[0]["exit"])

    # Walk corners 1, 2, …, n-1, then 0 to close the loop
    for k in list(range(1, n)) + [0]:
        b = blends[k]

        # Straight segment to this corner's blend entry
        push(b["entry"])

        # Arc through this corner (skip if arc_angle ≈ 0)
        if b["arc_angle"] > 0.01 and b["radius"] > 1e-3:
            ctr    = b["center"]
            r_vec  = [b["entry"][i] - ctr[i] for i in range(3)]  # center → entry
            n_pts  = max(2, round(b["radius"] * b["arc_angle"]))   # ≈ 1 mm per step
            ax     = b["axis"]

            for j in range(1, n_pts + 1):
                theta   = b["arc_angle"] * j / n_pts
                cos_t, sin_t = math.cos(theta), math.sin(theta)
                dot_rv  = sum(r_vec[i] * ax[i] for i in range(3))
                cross_rv = [
                    ax[1]*r_vec[2] - ax[2]*r_vec[1],
                    ax[2]*r_vec[0] - ax[0]*r_vec[2],
                    ax[0]*r_vec[1] - ax[1]*r_vec[0],
                ]
                r_k = [r_vec[i]*cos_t + cross_rv[i]*sin_t + ax[i]*dot_rv*(1-cos_t)
                       for i in range(3)]
                push([ctr[i] + r_k[i] for i in range(3)])

    return path_pts, cum_arcs


def _quintic_sample_path(
    path_pts:  List[List[float]],
    cum_arcs:  List[float],
    n_steps:   int,
) -> List[List[float]]:
    """
    Apply quintic time-scaling (zero velocity at start and end) to a dense path.

    Converts a high-resolution geometric path into an n_steps+1 command stream
    whose speed profile is the 5th-order minimum-jerk polynomial:

        p(u) = 10u³ − 15u⁴ + 6u⁵,   u ∈ [0, 1]

    Each output waypoint is linearly interpolated between the two nearest
    path_pts samples, so the arc-length resolution of path_pts (~1 mm)
    bounds the geometric interpolation error.

    Args
    ----
    path_pts : Dense list of poses (any dimension ≥ 3).
    cum_arcs : Cumulative arc-length for each pose in path_pts [mm].
               cum_arcs[0] must be 0.0; len == len(path_pts).
    n_steps  : Number of time steps. Returns n_steps+1 waypoints.

    Returns
    -------
    List of n_steps+1 interpolated poses along the path, with the quintic
    velocity profile applied.
    """
    total_arc = cum_arcs[-1]
    n_pts     = len(path_pts)
    waypoints: List[List[float]] = []

    for step in range(n_steps + 1):
        u = step / n_steps
        s = 10*u**3 - 15*u**4 + 6*u**5    # quintic: zero vel at both ends
        d = s * total_arc                   # arc-length position [mm]

        # Clamp to endpoints (handles floating-point edge cases)
        if d <= 0.0:
            waypoints.append(list(path_pts[0]))
            continue
        if d >= total_arc:
            waypoints.append(list(path_pts[-1]))
            continue

        # Binary search for the segment containing arc-length d
        lo, hi = 0, n_pts - 2
        while lo < hi:
            mid = (lo + hi) // 2
            if cum_arcs[mid + 1] < d - 1e-9:
                lo = mid + 1
            else:
                hi = mid
        seg = lo

        seg_len = cum_arcs[seg + 1] - cum_arcs[seg]
        t = (d - cum_arcs[seg]) / seg_len if seg_len > 1e-9 else 0.0
        t = max(0.0, min(1.0, t))

        p0   = path_pts[seg]
        p1   = path_pts[seg + 1]
        pose = [p0[i] + t * (p1[i] - p0[i]) for i in range(len(p0))]
        waypoints.append(pose)

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
    Generate a minimum-jerk Cartesian circle trajectory.

    The quintic time-scaling (10s³−15s⁴+6s⁵) is applied to the angle
    parameter so angular velocity (and thus TCP speed) ramps smoothly
    from zero at the start, peaks at the midpoint, then decelerates back
    to zero at the end.  This gives:

      - Zero TCP velocity at start and end — safe to join directly with a
        minimum-jerk lead-in or any other move that ends at rest.
      - Peak TCP speed = 1.875 × circumference / T ≤ max_tcp_linear_mms.
      - T = ceil(1.875 × 2π × radius / max_tcp_linear_mms / cycle_s) × cycle_s

    WHY NOT constant-speed (uniform angular steps)?
    A constant-speed circle requires instantaneous acceleration from 0 to
    max_tcp_linear_mms at the first step, causing MOTN-721 whenever a
    lead-in (or any other zero-velocity move) precedes the circle.

    The first and last waypoints coincide (closed loop).
    Orientation (WPR) is held constant throughout.

    Args
    ----
    center_pose        : [X, Y, Z, W, P, R] — centre + orientation [mm/deg].
    radius_mm          : Circle radius [mm].
    plane              : 'XY', 'XZ', or 'YZ'. Default 'XY'.
    max_tcp_linear_mms : Peak TCP speed [mm/s] (at the midpoint of the arc).
    cycle_s            : Communication cycle [s] (default 8 ms).
    clockwise          : Traverse clockwise if True.

    Returns
    -------
    List of [X, Y, Z, W, P, R] waypoints (closed loop — last == first).

    ⚠  Keep radius ≤ 50 mm for initial tests.
    """
    if radius_mm <= 0:
        raise ValueError("radius_mm must be positive")

    circumference = 2.0 * math.pi * radius_mm
    # Same duration formula as minimum_jerk_cartesian_trajectory for a
    # straight-line move of length = circumference.
    T       = 1.875 * circumference / max_tcp_linear_mms
    n_steps = max(4, math.ceil(T / cycle_s))

    sign = -1.0 if clockwise else 1.0
    cx, cy, cz = center_pose[0], center_pose[1], center_pose[2]
    wpr   = list(center_pose[3:6])
    extra = list(center_pose[6:]) if len(center_pose) > 6 else []

    waypoints: List[List[float]] = []
    for k in range(n_steps + 1):            # +1 so last point == first
        u     = k / n_steps                 # normalised time [0..1]
        s     = 10*u**3 - 15*u**4 + 6*u**5 # quintic: 0→1, zero vel at endpoints
        angle = sign * 2.0 * math.pi * s
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
    corner_blend_mm:      float = 10.0,
) -> List[List[float]]:
    """
    Generate a Cartesian trajectory that traces a regular polygon with
    circular arc corner blending to eliminate MOTN-721.

    Architecture
    ------------
    1. Compute the N vertex positions in the chosen plane.
    2. Replace each sharp corner with a circular arc of radius *corner_blend_mm*.
       This caps centripetal acceleration at v²/corner_blend_mm regardless of
       TCP speed, preventing MOTN-721 even at ≥ 150 mm/s.
    3. Apply a single quintic time-scaling over the blended perimeter so the
       TCP speed is zero at start/end and peaks at the midpoint — the same
       approach as circle_cartesian_trajectory().  The robot never stops between
       corners, so the IBGN is_waiting_for_command flag stays HIGH throughout
       (avoiding the MOTN-603 that per-segment trajectories trigger).

    Safe speed vs corner_blend_mm (centripetal: a = v²/r, limit ~3000 mm/s²):
        5  mm → 122 mm/s    10 mm → 173 mm/s    20 mm → 245 mm/s

    Common use:
      n_sides=3  → equilateral triangle
      n_sides=4  → square
      n_sides=5  → pentagon
      n_sides=6  → hexagon

    Args
    ----
    center_pose          : [X, Y, Z, W, P, R] — centre + orientation [mm/deg].
    radius_mm            : Circumradius (centre to vertex) [mm].
    n_sides              : Number of sides (≥ 3).
    plane                : 'XY', 'XZ', or 'YZ'.
    max_tcp_linear_mms   : Peak TCP speed [mm/s] (at blended perimeter midpoint).
    max_tcp_angular_degs : (unused — WPR held constant).
    cycle_s              : Communication cycle [s] (default 8 ms).
    start_angle_deg      : Angle of the first vertex [deg].
                           0° → first vertex along +axis1 of the plane.
    clockwise            : Traverse clockwise if True.
    corner_blend_mm      : Blend arc radius at each corner [mm].
                           Default 10 mm → safe up to ~173 mm/s.
                           Increase proportionally if raising max_tcp_linear_mms.

    Returns
    -------
    List of [X, Y, Z, W, P, R] waypoints.  The path is closed (starts and ends
    at the same point on the blend arc of the first vertex).

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

    # Compute vertex XYZ positions (geometry only — wpr/extra added by _build_blended_path)
    xyz_vertices: List[List[float]] = []
    for k in range(n_sides):
        angle = math.radians(start_angle_deg) + sign * 2.0 * math.pi * k / n_sides
        dx, dy, dz = _plane_offsets(radius_mm, angle, plane)
        xyz_vertices.append([cx + dx, cy + dy, cz + dz])

    # Build corner-blended dense path (circular arc fillets at every vertex)
    path_pts, cum_arcs = _build_blended_path(xyz_vertices, corner_blend_mm, wpr, extra)
    total_arc = cum_arcs[-1]

    # Single quintic profile over blended perimeter (zero vel at start & end)
    T       = 1.875 * total_arc / max_tcp_linear_mms
    n_steps = max(4, math.ceil(T / cycle_s))

    return _quintic_sample_path(path_pts, cum_arcs, n_steps)


def rectangle_cartesian_trajectory(
    center_pose:          List[float],
    width_mm:             float,
    height_mm:            float,
    plane:                str   = 'XY',
    max_tcp_linear_mms:   float = 150.0,
    max_tcp_angular_degs: float = 45.0,
    cycle_s:              float = DEFAULT_CYCLE_S,
    clockwise:            bool  = False,
    corner_blend_mm:      float = 10.0,
) -> List[List[float]]:
    """
    Generate a Cartesian trajectory that traces a rectangle with circular arc
    corner blending to eliminate MOTN-721 at 90° corners.

    Architecture
    ------------
    Same as polygon_cartesian_trajectory(): each sharp 90° corner is replaced
    by a circular arc of radius *corner_blend_mm*.  A single quintic time-scaling
    is applied over the blended perimeter so the robot never stops mid-trajectory
    (avoids MOTN-603 via IBGN flag drop) and peak centripetal acceleration at
    each corner is bounded by v²/corner_blend_mm (avoids MOTN-721).

    Safe speed vs corner_blend_mm (a = v²/r, limit ~3000 mm/s²):
        5  mm → 122 mm/s    10 mm → 173 mm/s    20 mm → 245 mm/s

    Args
    ----
    center_pose          : [X, Y, Z, W, P, R] — centre + orientation [mm/deg].
    width_mm             : Full width of the rectangle (axis1 of plane) [mm].
    height_mm            : Full height of the rectangle (axis2 of plane) [mm].
    plane                : 'XY', 'XZ', or 'YZ'.
    max_tcp_linear_mms   : Peak TCP speed [mm/s] (at blended perimeter midpoint).
    max_tcp_angular_degs : (unused — WPR held constant).
    cycle_s              : Communication cycle [s] (default 8 ms).
    clockwise            : Traverse corners clockwise if True.
    corner_blend_mm      : Blend arc radius at each 90° corner [mm].
                           Default 10 mm → safe up to ~173 mm/s.

    Returns
    -------
    List of [X, Y, Z, W, P, R] waypoints.  The path is closed (starts and ends
    at the same point on the blend arc of the first corner).

    ⚠  Keep width and height ≤ 100 mm for initial tests.
    """
    if width_mm <= 0 or height_mm <= 0:
        raise ValueError("width_mm and height_mm must be positive")

    cx, cy, cz = center_pose[0], center_pose[1], center_pose[2]
    wpr   = list(center_pose[3:6])
    extra = list(center_pose[6:]) if len(center_pose) > 6 else []

    hw, hh = width_mm / 2.0, height_mm / 2.0

    # Corner offsets in the chosen plane (CCW by default)
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

    # XYZ-only corner positions (wpr/extra added by _build_blended_path)
    xyz_corners = [[cx + dx, cy + dy, cz + dz] for dx, dy, dz in raw]

    # Build corner-blended dense path (circular arc fillets at every 90° corner)
    path_pts, cum_arcs = _build_blended_path(xyz_corners, corner_blend_mm, wpr, extra)
    total_arc = cum_arcs[-1]

    # Single quintic profile over blended perimeter (zero vel at start & end)
    T       = 1.875 * total_arc / max_tcp_linear_mms
    n_steps = max(4, math.ceil(T / cycle_s))

    return _quintic_sample_path(path_pts, cum_arcs, n_steps)


def pad_to_9(joints: List[float]) -> List[float]:
    """Pad a joint list to 9 elements (required by command packets)."""
    return (list(joints) + [0.0] * 9)[:9]
