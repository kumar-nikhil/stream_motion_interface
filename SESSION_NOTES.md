# Stream Motion Interface — Session Notes
_Last updated: 2026-04-03 (session 3 — arc-blending complete, laptop restart pending)_

## Project
- **Repo**: `C:\Users\NIKHIL\PycharmProjects\stream_motion_interface`
- **GitHub**: https://github.com/kumar-nikhil/stream_motion_interface
- **Robot**: FANUC CRX-10iA/L — simulated in ROBOGUIDE at `192.168.56.1`
- **Protocol**: FANUC Stream Motion (J519), UDP port 60015, big-endian, 8 ms cycle
- **Manual**: B-84904EN/01
- **Reference (working GPT project)**: `C:\Users\NIKHIL\PycharmProjects\stream_motion_gpt`

---

## Current Status (2026-04-03)

Circle confirmed fault-free. All polygons rewritten with arc-blending — pending test.

### Confirmed working
| Script | Setting | Result |
|--------|---------|--------|
| `basic_joint_move.py` — J1 +5° | SCALE=0.50 | ✓ Smooth, no alarms |
| `basic_joint_move_6axis.py` — 6-axis delta | SCALE=0.50 | ✓ Smooth, stable |
| `move_to_home.py` — any → [0,0,0,0,-90,0] | SCALE=0.50 | ✓ Smooth, repeatable |
| `basic_cartesian_move.py` — +200mm X | 150 mm/s | ✓ Smooth, 314 waypoints, 2.5s |
| `shapes_cartesian.py` — circle 50mm R | 150 mm/s | ✓ Full circle, no fault (trail fix) |

### Pending test (after arc-blend rewrite)
| Script | Setting | Status |
|--------|---------|--------|
| `shapes_cartesian.py` — pentagon 50mm R | 150 mm/s, blend=10mm | Arc-blended rewrite — needs test |
| `shapes_cartesian.py` — square/hexagon/rectangle | 150 mm/s, blend=10mm | Arc-blended rewrite — needs test |
| `shapes_cartesian.py` — triangle | 150 mm/s, blend=10mm | Arc-blended rewrite — needs test |

### Speed ladder status
**Joint space**: `0.05 ✓ → 0.50 ✓ → 0.80 (next on real hardware) → 1.00`
**Cartesian**:  `50 mm/s ✓ → 150 mm/s ✓ → 200 → 250 → 300 mm/s`

---

## Confirmed Working Architecture

### Send pattern (critical)
```
recv_status()  →  send one CommandPacket  →  repeat
```
Status packet arrives every 8 ms. Client must send exactly one command per
status. The background listener thread fires `_status_event` on each receipt;
`_stream_trajectory()` blocks on this event before each send.

### Trajectory shape: joint space
- **5th-order polynomial (minimum-jerk)**: `p(s) = D·(10s³ − 15s⁴ + 6s⁵)`
- Zero velocity AND zero acceleration at both endpoints — no filter needed
- First step: `~10·D/N³` (cubic) — vanishingly small, eliminates servo-to-cmd[0] jerk
- T = max(T_vel, T_acc, T_jerk, T_tcp) across all axes — slowest axis sets pace

### Trajectory shape: Cartesian space
- Same quintic polynomial applied to [X, Y, Z, W, P, R] directly
- T = max(T_lin, T_rot):
  - `T_lin = 1.875 × dist_mm / max_tcp_linear_mms`
  - `T_rot = 1.875 × max_angular_deg / max_tcp_angular_degs`
- Uses `math.ceil` for n_steps to guarantee peak speed ≤ limit
- WHY slower than joint: controller does IK internally — joint accelerations
  from the Jacobian are unbounded from our side. 500 mm/s → 11 waypoints →
  MOTN-721 on J2 at waypoint 3. 150 mm/s → 314 waypoints → smooth.

### TCP speed constraint (CRX cobot joint streaming)
- SYST-323 fires when TCP Cartesian speed > 750 mm/s (independent of joint limits)
- Fix: `T_tcp = 1.875 × |d_rad| × reach_mm / max_tcp_mms` (conservative lever arm)
- Planning target: 700 mm/s (7% margin below 750 mm/s limit)

---

## Full Fault History & Root Causes

| Fault | Root Cause | Fix Applied |
|-------|-----------|-------------|
| MOTN-721 | Python limits 1.5–2× above actual robot values | Read `$STMO_GRP` from pendant; `CRX_VEL/ACC/JRK_LIMITS` |
| MOTN-721 | Hardcoded start position didn't match servo | Read live `get_current_joints()` at IBGN |
| MOTN-722 | Box filter on quadratic profile → step[1] jumped 0.030°, jerk=101,875 deg/s³ (82× limit) | Replaced trapezoidal+boxfilter with `minimum_jerk_trajectory` |
| SYST-323 | J2=86° at SCALE=0.50 → peak TCP ~1307 mm/s >> 750 mm/s | Added `max_tcp_speed_mms` lever-arm constraint to joint planner |
| MOTN-721 (Cartesian) | 500 mm/s → only 11 waypoints for 20mm → IK produced excessive J2 acceleration at waypoint 3 | Reduced to 50 mm/s initially, then confirmed 150 mm/s safe |
| MOTN-720 (shapes, first run) | Shape's first waypoint is offset from centre by radius → 50mm jump in one 8ms cycle = 6250 mm/s | Added lead-in move in `shapes_cartesian.py`: current_pose → shape_traj[0] |
| MOTN-721 (circle, second run) | Uniform angular steps in circle → 0→150 mm/s in 8ms = 18,750 mm/s² at lead-in join | Replaced uniform steps with quintic angle profile (zero vel at both ends) |
| MOTN-603 (circle, third run) | Windows thread wakeup latency (1–15ms) accumulated over 4.5s/571-waypoint circle drains $PKT_STACK=10 buffer | Added 20 trail dwell waypoints (160ms) at end; `last=1` lands on final trail point |
| MOTN-603 (pentagon, first run) | Per-segment min-jerk polygon stops at each vertex (zero vel). FANUC IBGN drops `is_waiting_for_command` at zero velocity → Python sees `status=0x04`, stops sending → robot fires MOTN-603. Hit at waypoint 345/560 (mid-trajectory, 3rd side). Trail fix can't help mid-trajectory faults. | Rewrote `polygon_cartesian_trajectory` and `rectangle_cartesian_trajectory` to use quintic-perimeter (no stopping at corners), same approach as circle |
| MOTN-721 (pentagon, R=200mm) | User set SIZE_MM=200mm (should be ≤50mm). Sharp polygon corners cause instantaneous direction change in one 8ms step: a = 2×v×sin(θ/2)/0.008. At 72° turn (pentagon) at ~143mm/s: a ≈ 21,025 mm/s² vs limit ~3426 mm/s² (J1 at 600mm reach). | Added corner arc-blending (`_build_blended_path` + `_quintic_sample_path`). 10mm blend radius caps centripetal acc at v²/r = 2250 mm/s² at 150mm/s. Reset SIZE_MM=50mm. |

---

## Actual Robot Limits (`$STMO_GRP[1]` from pendant, verified in ROBOGUIDE)

```
$JNT_VEL_LIM[1..6] = [120, 120, 180, 180, 180, 180] deg/s
$JNT_ACC_LIM[1..6] = [265, 265, 399, 399, 399, 399]  deg/s²
$JNT_JRK_LIM[1..6] = [1240, 1240, 1860, 1860, 1860, 1860] deg/s³
$LMT_MODE = 1    (limits recalculated every cycle based on TCP speed)
$FLTR_LN  = 1
$MAX_SPD  = 2000 mm/s
Protocol max version reported by robot = 3  (PROTOCOL_VERSION_3 used as default)
```

---

## Git Commit Log (this session)

| Commit | Message |
|--------|---------|
| *pending* | Fix MOTN-721 polygon corners: arc-blending with _build_blended_path + _quintic_sample_path |
| `ad41a2f` | Fix MOTN-603 polygon: rewrite polygon+rectangle to quintic-perimeter (no vertex stops) |
| `295c50c` | Fix MOTN-603 circle: add 20 trail dwell waypoints at end of shape trajectory |
| `eb737d9` | Fix MOTN-721: use quintic profile for circle (was uniform steps) |
| `1d1aa5b` | Add shapes_cartesian.py: circle, square, polygon, rectangle, pentagon, hexagon |
| `39624da` | Raise Cartesian speed to 150 mm/s / 45 deg/s (confirmed safe) |
| `e6109f9` | Lower Cartesian speed defaults to fix MOTN-721 |
| `cdc77c1` | Add Cartesian streaming: minimum_jerk_cartesian_trajectory + example |
| `d3932d3` | Add README and update session notes |
| `774ccb3` | Add TCP Cartesian speed constraint to minimum_jerk_trajectory |
| `773935d` | Raise SCALE from 0.05 to 0.50 across all examples (confirmed working) |
| `a10ebe3` | Port learnings from stream_motion_gpt: home move, 6-axis, protocol v3 |

**Current HEAD**: `3665d4f` on `main` (5 commits ahead of origin — `git push` after restart).

---

## Current Constants Summary

### Joint space
```python
CRX_VEL_LIMITS  = [120, 120, 180, 180, 180, 180]      # deg/s
CRX_ACC_LIMITS  = [265, 265, 399, 399, 399, 399]       # deg/s²
CRX_JRK_LIMITS  = [1240, 1240, 1860, 1860, 1860, 1860] # deg/s³
CRX_COLLAB_TCP_LIMIT_MMS = 750.0   # SYST-323 hard limit
CRX_COLLAB_TCP_PLAN_MMS  = 700.0   # planning target (7% margin)
CRX_REACH_MM             = 1249.0  # lever-arm for TCP speed estimate
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, -90.0, 0.0]
```

### Cartesian space
```python
CRX_CART_LINEAR_MMS   = 150.0   # confirmed safe [mm/s] — speed ladder: → 200 → 250 → 300
CRX_CART_ANGULAR_DEGS =  45.0   # confirmed safe [deg/s]
```

---

## minimum_jerk_trajectory — Key Numbers

Peak coefficients (normalised D=1, T=1):
- Peak velocity = 1.875 D/T    at s=0.5
- Peak accel    = 5.773 D/T²   at s≈0.211 and 0.789
- Peak jerk     = 60.0  D/T³   at s=0 and s=1

Per-axis time budget (joint space):
```
T_v   = 1.875 × |d| / (v_lim × σ)
T_a   = √(5.773 × |d| / (a_lim × σ))
T_j   = ∛(60 × |d| / (j_lim × σ))
T_tcp = 1.875 × |d_rad| × reach_mm / max_tcp_mms   ← CRX cobot only
T     = max(T_v, T_a, T_j, T_tcp)
```

Cartesian time budget:
```
T_lin = 1.875 × dist_mm / max_tcp_linear_mms
T_rot = 1.875 × max_angular_deg / max_tcp_angular_degs
T     = max(T_lin, T_rot)
n_steps = ceil(T / cycle_s)   ← ceil, not round, to guarantee speed ≤ limit
```

---

## MOTN-603 Root Cause and Fix (2026-04-03)

**Fault**: MOTN-603 "Receiving interval over" fired near end of circle trajectory (robot stopped 0.7mm short of closing).

**Root cause**: Python running on Windows. Windows thread scheduler wakeup latency is 1–15ms. The main send loop wakes on `_status_event` but Python's GIL + OS scheduler can delay wakeup by a full 8ms cycle. Over a long trajectory (circle = 571 waypoints, 4.57s) these jitter events accumulate and occasionally drain the robot's $PKT_STACK=10 command buffer. The shorter basic_cartesian_move (314 waypoints, 2.51s) never triggered this because there wasn't enough time for drift to accumulate.

**Evidence**: Python logged "Trajectory complete" (all 571 packets sent) while robot already stopped at Y=-149.344 (0.7mm before target Y=-150.0). This means MOTN-603 fired mid-trajectory; robot halted; Python continued sending to a faulted robot (status bits lagged clearing); Python eventually finished and returned True.

**Fix applied** (`shapes_cartesian.py`):
```python
TRAIL_CYCLES = 20   # 20 × 8ms = 160ms dwell
trail_wp = list(trajectory[-1])
trajectory = trajectory + [trail_wp] * TRAIL_CYCLES
```
`last=1` now lands on the 20th trail waypoint, 160ms after real motion ends. The robot holds its final position during the trail. Even if Python falls 2 cycles behind at the very end, it has 20 slots of slack before `last=1` is reached.

**Why 20 cycles**: $PKT_STACK=10 cycles minimum; Windows jitter can stack 2 missed cycles → 20 gives 2× safety margin.

---

## MOTN-603 Pentagon Root Cause (2026-04-03)

**Fault**: MOTN-603 fired at waypoint 345/560 (`status=0x04 moving=False`) during pentagon shape.  Robot stopped mid-way through 3rd side (V2→V3).  Python's new improved warning correctly detected the status change and stopped sending.

**Root cause**: The old `polygon_cartesian_trajectory` used **independent minimum-jerk segments per side**, each decelerating to zero velocity at the endpoint vertex.  The FANUC IBGN controller drops the `is_waiting_for_command` bit (bit 0 of status byte) when velocity reaches zero, because it considers the current IBGN streaming "segment" complete.  Python's `_stream_trajectory()` checks:

```python
if not s.is_waiting_for_command and not s.is_command_received:
    return False   # ← triggered here with status=0x04
```

When this fired, Python stopped sending.  The robot's buffer drained and MOTN-603 fired.  This happened mid-trajectory — the trail-dwell fix at the *end* of the trajectory is irrelevant here.

**Fix**: Rewrote both `polygon_cartesian_trajectory` and `rectangle_cartesian_trajectory` to use **quintic time-scaling over total perimeter** (same as `circle_cartesian_trajectory`).  The robot never reaches zero velocity between corners, so IBGN stays active throughout.

- Speed magnitude: smooth quintic (0 → peak → 0 over full perimeter)
- Velocity direction: changes over one 8 ms step at each corner (sharp corner in Cartesian space)
- Risk: MOTN-721 at corners if speed too high → reduce `MAX_LINEAR_MMS` if needed
- 150 mm/s tested on circle (smooth corners) — should be fine for 90° and 72° corners at this speed

**Waypoint counts after rewrite** (50 mm radius, 150 mm/s):
```
Pentagon (5×R=50): perimeter = 5 × 2×50×sin(36°) = 294 mm → T=3.68s → 461 waypoints
Square   (4×R=50): perimeter = 4 × 50√2 = 283 mm         → T=3.54s → 443 waypoints
Hexagon  (6×R=50): perimeter = 6 × 50   = 300 mm          → T=3.75s → 469 waypoints
```

---

---

## MOTN-721 Pentagon Corner Root Cause (2026-04-03)

**Fault**: MOTN-721 "Exceeded Jnt Acc/Dec Limit" at waypoint 971/2171 during pentagon with SIZE_MM=200mm (user had raised it from 50mm).

**Root cause**: The quintic-perimeter approach applied a smooth speed *magnitude* profile but the velocity *direction* changed instantaneously at each sharp polygon corner (over one 8ms step). The acceleration at a sharp corner:

```
a = 2 × v × sin(θ/2) / cycle_s
```

For a pentagon (72° exterior turn) at ~143 mm/s:

```
a = 2 × 143 × sin(36°) / 0.008 ≈ 21,025 mm/s²
```

J2 at ~600mm reach contributes roughly `a_joint ≈ a_cart / reach_rad`. The estimated joint accel ≈ 21,025/10.5 ≈ 2000 deg/s², which exceeds J1/J2 limit of 265 deg/s² → MOTN-721.

**Fix**: Replaced sharp corners with circular arc fillets (`_build_blended_path`).

- Each corner: detect exterior angle, compute blend distance and actual arc radius
- Blend radius capped at 45% of the shorter adjacent side so arcs never overlap
- Arc waypoints via Rodrigues rotation: `r_k = r·cos(θ) + (axis×r)·sin(θ) + axis(axis·r)(1−cos(θ))`
- Returns dense path (~1mm steps) + cumulative arc-length array
- `_quintic_sample_path` re-applies quintic time-scaling over the blended path

**Centripetal acceleration at corners** (the binding constraint):

```
a_centripetal = v² / blend_radius
```

With CORNER_BLEND_MM=10mm at 150mm/s: `150²/10 = 2250 mm/s²`
Estimated joint accel: `2250/10.5 ≈ 214 deg/s²` — below J1/J2 limit of 265 deg/s² ✓

**Safe speed vs blend radius**:
```
blend = 5mm  → safe up to ~122 mm/s
blend = 10mm → safe up to ~173 mm/s   ← default
blend = 20mm → safe up to ~245 mm/s
```
Scale `CORNER_BLEND_MM` proportionally when raising `MAX_LINEAR_MMS`.

**Measured (Python simulation)**:
```
pentagon (50mm R, 150mm/s, 10mm blend):  max_step=1.20mm  max_speed=149.7mm/s  max_acc_tangential=258mm/s²
square   (50mm R, 150mm/s, 10mm blend):  max_step=1.20mm  max_speed=149.7mm/s  max_acc_tangential=145mm/s²
triangle (50mm R, 150mm/s, 10mm blend):  max_step=1.20mm  max_speed=149.9mm/s  max_acc_tangential=816mm/s²  ← from quintic ramp, not corners
hexagon  (50mm R, 150mm/s, 10mm blend):  max_step=1.20mm  max_speed=149.8mm/s  max_acc_tangential=1438mm/s²
```
Note: `max_acc_tangential` is the magnitude of speed-change rate, not the full acceleration vector. The centripetal component at corners is the same for all: v²/r ≈ 2250 mm/s².

---

## Next Steps (real hardware)

1. **Commit + push latest** (from PowerShell):
   ```powershell
   cd C:\Users\NIKHIL\PycharmProjects\stream_motion_interface
   git add stream_motion/trajectory.py examples/shapes_cartesian.py SESSION_NOTES.md
   git commit -m "Fix MOTN-721 polygon corners: arc-blending with _build_blended_path + _quintic_sample_path"
   git push
   ```

2. **Test pentagon** in ROBOGUIDE with SIZE_MM=50, CORNER_BLEND_MM=10 — confirm MOTN-721 is gone

3. **If MOTN-721 persists**: increase CORNER_BLEND_MM (20mm → safe up to ~245 mm/s) or reduce MAX_LINEAR_MMS

4. **Test all shapes**: square, rectangle, hexagon, triangle

5. **Real-robot tests** at current settings (SCALE=0.50 joint, 150 mm/s Cartesian)

6. **Speed ladders** once confirmed fault-free on real hardware:
   - Joint: SCALE 0.80 → 1.00
   - Cartesian: 200 → 250 → 300 mm/s
   - Faults to watch: MOTN-609/610/611 (joint), SYST-323 (TCP), MOTN-721 (Cartesian IK)

6. **Future features**:
   - Live limit table query via `PKT_TYPE_LIMIT_REQUEST` (auto-configure limits)
   - I/O in command packets (gripper synchronisation)
   - Multi-segment streaming (A→B→C without stopping)

---

## TP Program (required on controller)

```fanuc-tp
1:  UFRAME_NUM=0
2:  UTOOL_NUM=1
3:  PAYLOAD[1]
4:  $STMO_GRP[1].$FLTR_LN=1
5:  IBGN start[1]
6:  IBGN end[1]
7:  END
```

Robot must be in AUTO mode, 100% speed override, no active faults.

---

## Git Lock Workaround (if PyCharm holds lock files)

```powershell
Remove-Item ".git\index.lock" -Force -ErrorAction SilentlyContinue
Remove-Item ".git\HEAD.lock"  -Force -ErrorAction SilentlyContinue
```
