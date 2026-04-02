# stream_motion_interface

A Python UDP client for the **FANUC Stream Motion (J519)** interface, targeting the **CRX-10iA/L** collaborative robot. Built for high-resolution joint-space and Cartesian motion streaming, with a path toward ROS integration.

> **Reference manual**: B-84904EN/01 — Stream Motion for R-30iB/R-30iB Plus
> **Tested on**: FANUC CRX-10iA/L (ROBOGUIDE simulation at `192.168.56.1`)

---

## What this is

Stream Motion (option J519) lets an external PC send joint-space or Cartesian position commands to the robot at the controller's native 8 ms servo cycle — one command every 8 ms instead of every 12 ms or slower.

This library handles:
- UDP handshake and status-paced command loop
- **Joint streaming**: 5th-order polynomial (minimum-jerk) trajectory with full vel/acc/jerk limits and TCP speed capping (SYST-323 prevention)
- **Cartesian streaming**: straight-line TCP moves with configurable linear and angular speed limits
- Pre-flight limit checking before any joint motion
- Live position read at IBGN (joint and Cartesian) for accurate start position

---

## Requirements

- Python 3.9+
- FANUC controller with option J519 (Stream Motion) installed
- Network connection to the controller (UDP port 60015)
- A TP program on the controller with `IBGN start[*]` / `IBGN end[*]`

No external Python packages required — only the standard library.

---

## Installation

```bash
git clone https://github.com/kumar-nikhil/stream_motion_interface.git
cd stream_motion_interface
pip install -e .
```

---

## Controller Setup

### 1. TP program

Create and run this program on the teach pendant before running any Python script:

```fanuc-tp
1:  UFRAME_NUM=0
2:  UTOOL_NUM=1
3:  PAYLOAD[1]
4:  $STMO_GRP[1].$FLTR_LN=1
5:  IBGN start[1]
6:  IBGN end[1]
7:  END
```

The robot stops at line 5 (`IBGN start[1]`) and waits. When the Python client sends the final command packet with `last=1`, it proceeds through `IBGN end[1]` and finishes.

### 2. Robot mode

- **AUTO** mode, **100% speed override**
- No active faults (RESET if needed)
- The TP program must be **running** before the Python script connects

### 3. System variables (verify on pendant)

```
$STMO.$PHYS_PORT      = 2     (CD38B — default)
$STMO_GRP[1].$FLTR_LN = 1    (set in TP program above)
$STMO.$PKT_STACK      = 10   (command buffer depth)
```

---

## Quick Start

### Joint-space move

```python
from stream_motion import StreamMotionClient, minimum_jerk_trajectory
from stream_motion.constants import (
    CRX_VEL_LIMITS, CRX_ACC_LIMITS, CRX_JRK_LIMITS,
    CRX_COLLAB_TCP_PLAN_MMS, CRX_REACH_MM,
)

with StreamMotionClient(robot_ip="192.168.56.1") as client:
    client.start_status_output()
    client.wait_for_ready(timeout=30.0)

    start = client.get_current_joints()     # live servo position at IBGN
    end   = [s + d for s, d in zip(start, [5, 0, 0, 0, 0, 0])]

    trajectory = minimum_jerk_trajectory(
        start_joints      = start,
        end_joints        = end,
        vel_limits        = CRX_VEL_LIMITS,
        acc_limits        = CRX_ACC_LIMITS,
        jrk_limits        = CRX_JRK_LIMITS,
        scale             = 0.50,
        max_tcp_speed_mms = CRX_COLLAB_TCP_PLAN_MMS,
        robot_reach_mm    = CRX_REACH_MM,
    )

    client.stream_joint_trajectory(trajectory)
    client.stop_status_output()
```

### Cartesian move

```python
from stream_motion import StreamMotionClient, minimum_jerk_cartesian_trajectory
from stream_motion.constants import CRX_CART_LINEAR_MMS, CRX_CART_ANGULAR_DEGS

with StreamMotionClient(robot_ip="192.168.56.1") as client:
    client.start_status_output()
    client.wait_for_ready(timeout=30.0)

    start = client.get_current_cart()       # live [X,Y,Z,W,P,R] at IBGN
    end   = [start[i] + d for i, d in enumerate([20, 0, 0, 0, 0, 0])]

    trajectory = minimum_jerk_cartesian_trajectory(
        start_pose           = start,
        end_pose             = end,
        max_tcp_linear_mms   = CRX_CART_LINEAR_MMS,
        max_tcp_angular_degs = CRX_CART_ANGULAR_DEGS,
    )

    client.stream_cartesian_trajectory(trajectory)
    client.stop_status_output()
```

---

## Example Scripts

All examples are in `examples/`. Set `ROBOT_IP` at the top of each file.

### `basic_joint_move.py`
Moves J1 by +5° from the robot's current position. Good first test — small, single-axis.

### `basic_joint_move_6axis.py`
Moves all 6 joints simultaneously by `[+2, +1, -1, +2, -2, +2]` degrees. All axes time-synchronised — every joint starts and finishes together.

### `move_to_home.py`
Moves from any current position to `[0, 0, 0, 0, -90, 0]`. The hardest joint test — large multi-axis displacement exercises the TCP speed constraint.

### `basic_cartesian_move.py`
Moves the TCP by a Cartesian delta `[X, Y, Z, W, P, R]`. Default: `+200mm X`. Reads the live TCP pose at IBGN via `get_current_cart()`.

### `status_monitor.py`
Prints live status packets without sending any motion. Use to verify the connection and read the robot's current state.

---

## Trajectory Generation

### Joint space — `minimum_jerk_trajectory()`

5th-order polynomial trajectory:

```
p(s) = start + D · (10s³ − 15s⁴ + 6s⁵),   s ∈ [0, 1]
```

Zero velocity and zero acceleration at both endpoints — no post-filtering needed. The first position step grows as `~10·D/N³` (cubic), so any residual servo-to-command gap at the first packet produces negligible jerk.

**Per-axis duration constraints** — the binding one sets T:
```
T_vel  = 1.875 × |d| / (v_lim × scale)
T_acc  = √(5.773 × |d| / (a_lim × scale))
T_jerk = ∛(60.0 × |d| / (j_lim × scale))
T_tcp  = 1.875 × |d_rad| × reach_mm / max_tcp_mms   ← CRX cobot TCP limit
```

### Cartesian space — `minimum_jerk_cartesian_trajectory()`

Same quintic time-scaling applied to `[X, Y, Z, W, P, R]` directly. The TCP moves in a straight Cartesian line.

**Duration constraints:**
```
T_lin = 1.875 × linear_distance_mm  / max_tcp_linear_mms
T_rot = 1.875 × max_angular_delta_deg / max_tcp_angular_degs
T     = max(T_lin, T_rot)
n_steps = ceil(T / 0.008)   ← ceil guarantees speed ≤ limit
```

**Why Cartesian needs lower speeds than joint:** in joint streaming our planner controls joint accelerations directly. In Cartesian streaming the controller runs its own IK — the Jacobian can amplify Cartesian acceleration into large joint accelerations at certain configurations. 500 mm/s → 11 waypoints for 20 mm → MOTN-721. 150 mm/s → 33 waypoints → smooth motion.

### Waypoint counts at current speed settings

| Move | Joint (SCALE=0.50) | Cartesian (150 mm/s) |
|------|-------------------|----------------------|
| 20 mm / 5° | 99 pts / 0.79s | 33 pts / 0.26s |
| 50 mm | 247 pts / 1.97s | 80 pts / 0.63s |
| 200 mm | — | 314 pts / 2.50s |
| J2 86° home | 629 pts / 5.03s | — |

---

## CRX-10iA/L Constants

```python
# Joint limits (from $STMO_GRP[1], verified on pendant)
CRX_VEL_LIMITS  = [120, 120, 180, 180, 180, 180]       # deg/s
CRX_ACC_LIMITS  = [265, 265, 399, 399, 399, 399]        # deg/s²
CRX_JRK_LIMITS  = [1240, 1240, 1860, 1860, 1860, 1860] # deg/s³

# Collaborative TCP limit
CRX_COLLAB_TCP_LIMIT_MMS = 750.0   # SYST-323 hard limit
CRX_COLLAB_TCP_PLAN_MMS  = 700.0   # planning target (7% margin)
CRX_REACH_MM             = 1249.0  # lever-arm for TCP speed estimate

# Cartesian streaming (speed-ladder starting point)
CRX_CART_LINEAR_MMS   = 150.0     # confirmed safe — raise: 200 → 250 → 300 mm/s
CRX_CART_ANGULAR_DEGS =  45.0     # confirmed safe — raise proportionally

HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, -90.0, 0.0]
```

---

## Protocol Overview

FANUC Stream Motion uses UDP on port 60015.

### Startup sequence
1. Python sends **StatusOutputStart** (version, physical port)
2. Robot streams **StatusPackets** every 8 ms (joints, Cartesian pose, status bits)
3. Python waits for status bit 0 = 1 (`IBGN start[*]` reached)
4. Python reads live position via `get_current_joints()` or `get_current_cart()`

### Command loop
```
wait for status packet  →  send CommandPacket  →  repeat
```

One command per 8 ms cycle. Final packet has `last=1`. Python sends **StatusOutputStop**.

### Protocol versions
| Version | Feature |
|---------|---------|
| 1 | Base — joint-space streaming |
| 2 | Adds double-precision Cartesian commands |
| 3 | Adds comm timing adjustment, adaptive deceleration |

Default: **version 3** (confirmed supported on CRX-10iA/L).

---

## Fault Reference

| Fault | Meaning | Common cause |
|-------|---------|--------------|
| MOTN-609 | Joint velocity exceeded | `scale` too high |
| MOTN-610 | Joint acceleration exceeded | `scale` too high |
| MOTN-611 | Joint jerk exceeded | `scale` too high |
| MOTN-721 | Vel/acc limit (stream motion) | Limits in constants.py don't match pendant; or Cartesian speed too high |
| MOTN-722 | Jerk limit (stream motion) | Trajectory shape problem at start — use `minimum_jerk_trajectory` |
| MOTN-603 | Next command not received | Python too slow; increase `$PKT_STACK` |
| MOTN-156 | Cartesian config change | Path crosses singularity or forces wrist flip — reduce delta |
| SYST-323 | Collaborative TCP speed | TCP > 750 mm/s — pass `max_tcp_speed_mms` to joint planner |

---

## Repository Structure

```
stream_motion_interface/
├── stream_motion/
│   ├── __init__.py        # Public API exports
│   ├── client.py          # StreamMotionClient (UDP, status loop, streaming)
│   ├── constants.py       # Protocol constants, CRX limits
│   ├── packets.py         # Packet builders and parsers
│   └── trajectory.py      # minimum_jerk_trajectory(), minimum_jerk_cartesian_trajectory(),
│                          # check_limits(), smooth_trajectory(), helpers
├── examples/
│   ├── basic_joint_move.py        # Single-axis J1 move
│   ├── basic_joint_move_6axis.py  # Coordinated 6-axis move
│   ├── move_to_home.py            # Move to [0,0,0,0,-90,0] from anywhere
│   ├── basic_cartesian_move.py    # Cartesian TCP move [X,Y,Z,W,P,R]
│   └── status_monitor.py          # Print live status packets (no motion)
├── SESSION_NOTES.md       # Development log, fault history, speed ladders
└── README.md
```

---

## Roadmap

- **Live limit table query** — read `$JNT_VEL_LIM` etc. directly from controller via `PKT_TYPE_LIMIT_REQUEST`
- **I/O in command packets** — write DO/RO bits synchronized to waypoints (gripper, tooling)
- **Multi-segment streaming** — chain moves A→B→C without stopping between segments
- **ROS integration** — joint trajectory action server backed by this client

---

## License

MIT
