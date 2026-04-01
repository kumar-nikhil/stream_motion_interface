"""
Unit Tests – Trajectory Planning
Run with:  python -m pytest tests/ -v
"""

import pytest
from stream_motion.trajectory import (
    trapezoidal_joint_trajectory,
    linear_joint_interpolation,
    check_limits,
    pad_to_9,
    DEFAULT_CYCLE_S,
)

VEL_LIMITS = [150.0, 130.0, 200.0, 270.0, 270.0, 360.0]
ACC_LIMITS = [500.0, 400.0, 700.0, 900.0, 900.0, 1200.0]
JRK_LIMITS = [2000.0, 1500.0, 3000.0, 4000.0, 4000.0, 5000.0]


class TestTrapezoidalTrajectory:
    def test_returns_list(self):
        traj = trapezoidal_joint_trajectory(
            [0]*6, [10]*6, VEL_LIMITS, ACC_LIMITS
        )
        assert isinstance(traj, list)
        assert len(traj) > 0

    def test_starts_near_start(self):
        start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        end   = [30.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        traj = trapezoidal_joint_trajectory(start, end, VEL_LIMITS, ACC_LIMITS)
        first = traj[0]
        assert all(abs(first[i] - start[i]) < 0.5 for i in range(6))

    def test_ends_at_target(self):
        start = [0.0] * 6
        end   = [20.0, -10.0, 5.0, 0.0, 0.0, 0.0]
        traj = trapezoidal_joint_trajectory(start, end, VEL_LIMITS, ACC_LIMITS)
        last = traj[-1]
        assert all(abs(last[i] - end[i]) < 0.5 for i in range(6))

    def test_no_limit_violations(self):
        start = [0.0] * 6
        end   = [15.0, -5.0, 8.0, 0.0, 3.0, 0.0]
        traj = trapezoidal_joint_trajectory(
            start, end, VEL_LIMITS, ACC_LIMITS, scale=0.5
        )
        violations = check_limits(traj, VEL_LIMITS, ACC_LIMITS, JRK_LIMITS)
        assert violations == [], f"Unexpected violations: {violations}"

    def test_same_start_and_end_returns_one_waypoint(self):
        pos = [0.0] * 6
        traj = trapezoidal_joint_trajectory(pos, pos, VEL_LIMITS, ACC_LIMITS)
        assert len(traj) == 1
        assert traj[0] == pos


class TestLinearInterpolation:
    def test_n_steps(self):
        traj = linear_joint_interpolation([0]*6, [10]*6, n_steps=10)
        assert len(traj) == 10

    def test_first_waypoint_is_start(self):
        start = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        traj = linear_joint_interpolation(start, [0]*6, n_steps=5)
        assert traj[0] == start

    def test_last_waypoint_is_end(self):
        end = [10.0, 20.0, 30.0, 0.0, 0.0, 0.0]
        traj = linear_joint_interpolation([0]*6, end, n_steps=5)
        assert all(abs(traj[-1][i] - end[i]) < 1e-9 for i in range(6))


class TestCheckLimits:
    def test_clean_trajectory_no_warnings(self):
        traj = linear_joint_interpolation([0]*6, [1]*6, n_steps=50)
        warnings = check_limits(traj, VEL_LIMITS, ACC_LIMITS)
        assert warnings == []

    def test_too_fast_trajectory_gives_warnings(self):
        # 1000 degrees in one 8ms step = 125000 deg/s – way over limit
        traj = [[0.0]*6, [1000.0]*6]
        warnings = check_limits(traj, VEL_LIMITS, ACC_LIMITS)
        assert len(warnings) > 0
        assert any("velocity" in w for w in warnings)

    def test_single_point_no_warnings(self):
        warnings = check_limits([[0]*6], VEL_LIMITS, ACC_LIMITS)
        assert warnings == []


class TestPadToNine:
    def test_pads_short_list(self):
        result = pad_to_9([1.0, 2.0, 3.0])
        assert len(result) == 9
        assert result[3:] == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def test_truncates_long_list(self):
        result = pad_to_9([1.0] * 12)
        assert len(result) == 9

    def test_preserves_values(self):
        vals = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        result = pad_to_9(vals)
        assert result[:6] == vals
