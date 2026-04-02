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

All three example scripts run fault-free in ROBOGUIDE at SCALE=0.50.
Ready to test on real CRX-10iA/L hardware.

### Confirmed working
| Script | SCALE | Result |
|--------|-------|--------|
| `basic_joint_move.py` — J1 +5° | 0.50 | Smooth, no alarms |
| `basic_joint_move_6axis.py` — coordinated 6-axis | 0.50 | Smooth, stable |
| `move_to_home.py` — any → [0,0,0,0,-90,0] | 0.50 | Smooth, repeatable |

### Speed ladder status
`0.05 ✓ → 0.50 ✓ → 0.80 (next on real hardware) → 1.00`

---

## Confirmed Working Architecture

### Send pattern (critical)
```
recv_status()  →  send one CommandPacket  →  repeat
```
Status packet arrives every 8 ms. Client must send exactly one command per
status. The background listener thread fires `_status_event` on each receipt;
`_stream_trajectory()` blocks on this event before each send.

### Trajectory shape (critical)
- **5th-order polynomial (minimum-jerk)**: `p(s) = D·(10s³ − 15s⁴ + 6s⁵)`
- Zero velocity AND zero acceleration at both endpoints — no filter needed
- First step size: `~10·D/N³` (cubic) — tiny enough that any servo-to-cmd[0]
  gap does not produce excessive jerk at the controller
- The controller uses servo position as implicit t=−1 to compute vel/acc/jerk
  for the very first packet; the cubic ramp-up makes this gap irrelevant

### TCP speed constraint (CRX cobot-specific)
- SYST-323 fires when TCP Cartesian speed > 750 mm/s (independent of joint limits)
- Fix: add `T_tcp = 1.875 × |d_rad| × reach_mm / max_tcp_mms` as a fourth
  per-axis constraint; conservative lever-arm = full robot reach for all joints
- Planning target: 700 mm/s (7% margin below hard 750 mm/s limit)
- Worst case (J2=86° at SCALE=0.50): T_tcp=5.04s, 629 waypoints, peak TCP=700 mm/s ✓

---

## Full Fault History & Root Causes

| Fault | Root Cause | Fix Applied |
|-------|-----------|-------------|
| MOTN-721 | Python limits were 1.5–2× above actual robot values | Read `$STMO_GRP` from pendant; added `CRX_VEL/ACC/JRK_LIMITS` |
| MOTN-721 | Hardcoded start position didn't match servo position | Read live `get_current_joints()` at IBGN |
| MOTN-722 | Box filter on quadratic profile → step[1] jumped 0.030°, jerk=101,875 deg/s³ (82× limit) | Replaced trapezoidal+boxfilter entirely with `minimum_jerk_trajectory` |
| SYST-323 | J2=86° at SCALE=0.50 → peak TCP ~1307 mm/s >> 750 mm/s cobot limit | Added `max_tcp_speed_mms` parameter with lever-arm T_tcp constraint |

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

Note: `$LMT_MODE=1` means the controller interpolates limits based on TCP speed.
We always use the tightest (high-speed) column — safe but conservative for short moves.

---

## Git Commit Log (this session)

| Commit | Message |
|--------|---------|
| `7342acd` | Add TCP Cartesian speed constraint to minimum_jerk_trajectory |
| `0ba94dd` | Add move_to_home, 6-axis example, HOME_JOINTS, PROTOCOL_VERSION_3 default |
| `7d3d1bb` | Add minimum_jerk_trajectory (5th-order polynomial) |
| `39be2b4` | Lead-in fix (superseded by minimum_jerk_trajectory) |

**Current HEAD**: `7342acd` on `main`.

---

## minimum_jerk_trajectory — Key Numbers

Peak coefficients (normalised D=1, T=1):
- Peak velocity = 1.875 D/T    at s=0.5
- Peak accel    = 5.773 D/T²   at s≈0.211 and 0.789
- Peak jerk     = 60.0  D/T³   at s=0 and s=1

Per-axis time budget:
```
T_v   = 1.875 × |d| / (v_lim × σ)
T_a   = √(5.773 × |d| / (a_lim × σ))
T_j   = ∛(60 × |d| / (j_lim × σ))
T_tcp = 1.875 × |d_rad| × reach_mm / max_tcp_mms   ← CRX cobot only
T     = max(T_v, T_a, T_j, T_tcp)
```

Example — `move_to_home.py`, J2=86.246°, SCALE=0.50:
```
T_vel  = 2.70s  (was binding before TCP constraint)
T_tcp  = 5.04s  (binding with TCP constraint)
→ 629 waypoints, peak TCP = 700 mm/s  ✓  (limit = 750 mm/s)
```

---

## Next Steps (real hardware)

1. **Push to GitHub** (from PowerShell — sandbox proxy blocks push):
   ```powershell
   cd C:\Users\NIKHIL\PycharmProjects\stream_motion_interface
   git push
   ```

2. **First real-robot tests** at SCALE=0.50:
   - `basic_joint_move.py` — small 5° J1 move
   - `basic_joint_move_6axis.py` — small 6-axis delta
   - `move_to_home.py` — large multi-axis move

3. **Speed ladder** — once all three pass fault-free on real hardware:
   - SCALE 0.80 → all three scripts
   - SCALE 1.00 → all three scripts
   - Faults to watch: MOTN-609/610/611 (vel/acc/jerk), SYST-323 (TCP)

4. **Future features** (see README.md):
   - Live limit table query via `PKT_TYPE_LIMIT_REQUEST`
   - I/O in command packets (gripper synchronisation)
   - Multi-segment streaming (A→B→C without stopping)
   - Cartesian streaming example
   - Pre-flight `check_limits` abort guard in client

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
