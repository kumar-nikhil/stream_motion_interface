# Stream Motion Interface — Session Notes
_Last updated: 2026-04-02_

## Project
- **Repo**: `C:\Users\NIKHIL\PycharmProjects\stream_motion_interface`
- **GitHub**: https://github.com/kumar-nikhil/stream_motion_interface
- **Robot**: FANUC CRX-10iA/L — simulated in ROBOGUIDE at `192.168.56.1`
- **Protocol**: FANUC Stream Motion (J519), UDP port 60015, big-endian, 8 ms cycle
- **Manual**: B-84904EN/01
- **Reference (working GPT project)**: `C:\Users\NIKHIL\PycharmProjects\stream_motion_gpt`

---

## Current Status (2026-04-02)

All four example scripts run fault-free in ROBOGUIDE at current speed settings.
Ready to test on real CRX-10iA/L hardware.

### Confirmed working
| Script | Setting | Result |
|--------|---------|--------|
| `basic_joint_move.py` — J1 +5° | SCALE=0.50 | Smooth, no alarms |
| `basic_joint_move_6axis.py` — 6-axis delta | SCALE=0.50 | Smooth, stable |
| `move_to_home.py` — any → [0,0,0,0,-90,0] | SCALE=0.50 | Smooth, repeatable |
| `basic_cartesian_move.py` — +200mm X | 150 mm/s | Smooth, 314 waypoints, 2.5s |

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
| `39624da` | Raise Cartesian speed to 150 mm/s / 45 deg/s (confirmed safe) |
| `e6109f9` | Lower Cartesian speed defaults to fix MOTN-721 |
| `cdc77c1` | Add Cartesian streaming: minimum_jerk_cartesian_trajectory + example |
| `d3932d3` | Add README and update session notes |
| `774ccb3` | Add TCP Cartesian speed constraint to minimum_jerk_trajectory |
| `773935d` | Raise SCALE from 0.05 to 0.50 across all examples (confirmed working) |
| `a10ebe3` | Port learnings from stream_motion_gpt: home move, 6-axis, protocol v3 |

**Current HEAD**: `39624da` on `main`.

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

## Next Steps (real hardware)

1. **Push latest** (from PowerShell):
   ```powershell
   cd C:\Users\NIKHIL\PycharmProjects\stream_motion_interface
   git push
   ```

2. **Real-robot tests** at current settings (SCALE=0.50 joint, 150 mm/s Cartesian)

3. **Speed ladders** once confirmed fault-free on real hardware:
   - Joint: SCALE 0.80 → 1.00
   - Cartesian: 200 → 250 → 300 mm/s
   - Faults to watch: MOTN-609/610/611 (joint), SYST-323 (TCP), MOTN-721 (Cartesian IK)

4. **Future features**:
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
