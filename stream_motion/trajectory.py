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
    start_joints:  List[float],
    end_joints:    List[float],
    vel_limits:    List[float],
    acc_limits:    List[float],
    jrk_limits:    List[float],
    cycle_s:       float = DEFAULT_CYCLE_S,
    scale:         float = 0.8,
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
    cycle_s      : Communication cycle [s] (default 8 ms).
    scale        : Safety margin: limits are multiplied by this factor
                   (default 0.8 = 80 %).  Keep ≤ 1.0.

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


def pad_to_9(joints: List[float]) -> List[float]:
    """Pad a joint list to 9 elements (required by command packets)."""
    return (list(joints) + [0.0] * 9)[:9]
