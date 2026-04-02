# stream_motion_interface

A Python UDP client for the **FANUC Stream Motion (J519)** interface, targeting the **CRX-10iA/L** collaborative robot. Built for high-resolution joint-space motion streaming, with a path toward ROS integration.

> **Reference manual**: B-84904EN/01 — Stream Motion for R-30iB/R-30iB Plus
> **Tested on**: FANUC CRX-10iA/L (ROBOGUIDE simulation at `192.168.56.1`)

---

## What this is

Stream Motion (option J519) lets an external PC send joint-space (or Cartesian) position commands to the robot at the controller's native 8 ms servo cycle. This replaces FANUC's Remote Motion Interface with a much higher-resolution UDP protocol — one command packet every 8 ms instead of every 12 ms or slower.

This library handles:
- UDP handshake and status-paced command loop
- 5th-order polynomial (minimum-jerk) trajectory generation with full vel/acc/jerk limits
- TCP Cartesian speed capping for CRX cobot collaborative mode (SYST-323 prevention)
- Pre-flight limit checking before any motion
- Live joint position read at IBGN for accurate start position

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
$STMO.$PHYS_PORT   = 2       (CD38B — default)
$STMO_GRP[1].$FLTR_LN = 1   (set in TP program)
$STMO.$PKT_STACK   = 10      (command buffer depth)
```

---

## Quick Start

```python
from stream_motion import StreamMotionClient, minimum_jerk_trajectory
from stream_motion.constants import (
    CRX_VEL_LIMITS, CRX_ACC_LIMITS, CRX_JRK_LIMITS,
    CRX_COLLAB_TCP_PLAN_MMS, CRX_REACH_MM,
)

with StreamMotionClient(robot_ip="192.168.56.1") as client:
    client.start_status_output()
    client.wait_for_ready(timeout=30.0)     # blocks until IBGN start[1]

    start = client.get_current_joints()     # live servo position
    end   = [s + d for s, d in zip(start, [5, 0, 0, 0, 0, 0])]

    trajectory = minimum_jerk_trajectory(
        start_joints      = start,
        end_joints        = end,
        vel_limits        = CRX_VEL_LIMITS,
        acc_limits        = CRX_ACC_LIMITS,
        jrk_limits        = CRX_JRK_LIMITS,
        scale             = 0.50,
        max_tcp_speed_mms = CRX_COLLAB_TCP_PLAN_MMS,  # SYST-323 guard
        robot_reach_mm    = CRX_REACH_MM,
    )

    client.stream_joint_trajectory(trajectory)
    client.stop_status_output()
```

---

## Example Scripts

All examples are in `examples/`. Set `ROBOT_IP` at the top of each file.

### `basic_joint_move.py`
Moves J1 by +5° from the robot's current position. Good first test — small, single-axis, easy to verify visually.

```bash
python examples/basic_joint_move.py
```

### `basic_joint_move_6axis.py`
Moves all 6 joints simultaneously by a small delta (`[+2, +1, -1, +2, -2, +2]` degrees). All axes are time-synchronised — every joint starts and finishes at the same time.

```bash
python examples/basic_joint_move_6axis.py
```

### `move_to_home.py`
Moves from any current position to the canonical home pose `[0, 0, 0, 0, -90, 0]`. This is the hardest test — large multi-axis displacement requires the TCP speed constraint to engage.

```bash
python examples/move_to_home.py
```

### `status_monitor.py`
Prints live status packets (joint positions, Cartesian pose, status bits) without sending any motion. Useful for verifying the connection and reading the robot's current state.

```bash
python examples/status_monitor.py
```

---

## Trajectory Generation

### `minimum_jerk_trajectory()`

Generates a 5th-order polynomial trajectory:

```
p(s) = start + D · (10s³ − 15s⁴ + 6s⁵),   s ∈ [0, 1]
```

**Why this profile?**
The FANUC controller uses the robot's current servo position as an implicit `t = −1` data point when computing velocity/acceleration/jerk for the very first command packet. Any gap between the servo position and `cmd[0]`, or any velocity spike in the trajectory shape, produces fault-triggering jerk.

The minimum-jerk profile has zero velocity and zero acceleration at both endpoints by construction. The first position step grows as `~10·D/N³` (cubic, not quadratic) — so small that any residual servo-to-command gap is irrelevant.

This is why earlier trapezoidal + box-filter approaches produced MOTN-722: the box filter's warm-up distortion at the trajectory start created a 0.030° jump at step[1], causing 101,875 deg/s³ of jerk (82× the limit).

### Duration calculation

For each axis, four constraints bound the minimum motion time. The longest wins:

```
T_vel  = 1.875 × |d| / (v_lim × scale)
T_acc  = √(5.773 × |d| / (a_lim × scale))
T_jerk = ∛(60.0 × |d| / (j_lim × scale))
T_tcp  = 1.875 × |d_rad| × reach_mm / max_tcp_mms   ← cobot TCP limit
```

All axes are then time-synchronised to the longest `T`, producing coordinated motion where every joint starts and finishes simultaneously.

### CRX-10iA/L limits

```python
CRX_VEL_LIMITS = [120, 120, 180, 180, 180, 180]   # deg/s
CRX_ACC_LIMITS = [265, 265, 399, 399, 399, 399]    # deg/s²
CRX_JRK_LIMITS = [1240, 1240, 1860, 1860, 1860, 1860]  # deg/s³

CRX_COLLAB_TCP_LIMIT_MMS = 750.0   # SYST-323 hard limit
CRX_COLLAB_TCP_PLAN_MMS  = 700.0   # planning target (7% margin)
CRX_REACH_MM             = 1249.0  # maximum reach, used as lever-arm
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, -90.0, 0.0]
```

---

## Protocol Overview

FANUC Stream Motion uses a UDP request/response pattern on port 60015.

### Startup sequence
1. Python sends **StatusOutputStart** packet (version, physical port)
2. Robot sends **StatusPackets** every 8 ms (joint pos, Cartesian pos, status bits)
3. Python reads status bits, waits for bit 0 = 1 (`IBGN start[*]` reached)
4. Python reads live joint positions via `get_current_joints()`

### Command loop (one per 8 ms cycle)
```
wait for status packet  →  read status  →  send CommandPacket  →  repeat
```
Each command packet carries 9 joint positions (J1–J9), `data_format` (joint or Cartesian), and a `last` flag (set on the final waypoint).

### Shutdown
- Final command packet: `last=1`
- Python sends **StatusOutputStop**
- Robot exits `IBGN end[*]` and finishes the TP program

### Protocol versions
| Version | Feature |
|---------|---------|
| 1 | Base — joint-space streaming |
| 2 | Adds double-precision Cartesian commands |
| 3 | Adds comm timing adjustment, adaptive deceleration |

This library defaults to **version 3** (`PROTOCOL_VERSION_DEFAULT = 3`), confirmed supported on CRX-10iA/L.

---

## Fault Reference

| Fault | Meaning | Common cause |
|-------|---------|--------------|
| MOTN-609 | Joint velocity limit exceeded | `scale` too high, or limits misconfigured |
| MOTN-610 | Joint acceleration limit exceeded | Same as above |
| MOTN-611 | Joint jerk limit exceeded | Same as above |
| MOTN-721 | Velocity/accel limit exceeded (stream motion specific) | Limits in `constants.py` don't match `$STMO_GRP` pendant values |
| MOTN-722 | Jerk limit exceeded (stream motion specific) | Trajectory shape problem at start (see trajectory section) |
| MOTN-603 | Next command not received in time | Python too slow — check CPU load, increase `PKT_STACK` |
| MOTN-625 | Abnormal position jump | Gap > `$THRS_ABNPOS` (100,000 mm/deg) between packets |
| SYST-323 | Collaborative TCP speed limit | TCP Cartesian speed > 750 mm/s — pass `max_tcp_speed_mms` to trajectory |

---

## Repository Structure

```
stream_motion_interface/
├── stream_motion/
│   ├── __init__.py        # Public API exports
│   ├── client.py          # StreamMotionClient (UDP connection, status loop, streaming)
│   ├── constants.py       # Protocol constants, CRX limits, home position
│   ├── packets.py         # Packet builders and parsers (struct packing)
│   └── trajectory.py      # minimum_jerk_trajectory(), check_limits(), helpers
├── examples/
│   ├── basic_joint_move.py        # Single-axis J1 move
│   ├── basic_joint_move_6axis.py  # Coordinated 6-axis move
│   ├── move_to_home.py            # Move to [0,0,0,0,-90,0] from anywhere
│   └── status_monitor.py          # Print live status packets (no motion)
├── SESSION_NOTES.md       # Development log and fault history
└── README.md
```

---

## Roadmap

- **Live limit table query** — call `PKT_TYPE_LIMIT_REQUEST` to read `$JNT_VEL_LIM` etc. directly from the controller instead of hardcoding
- **I/O in command packets** — write DO/RO bits synchronized to trajectory waypoints (gripper, tooling)
- **Multi-segment streaming** — chain moves A→B→C without stopping the robot between segments
- **Cartesian streaming** — `stream_cartesian_trajectory()` example with small XYZ moves
- **ROS integration** — joint trajectory action server backed by this client

---

## License

MIT
