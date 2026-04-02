# Stream Motion Interface — Session Notes
_Last updated: 2026-04-02_

## Project
- **Repo**: `C:\Users\NIKHIL\PycharmProjects\stream_motion_interface`
- **GitHub**: https://github.com/kumar-nikhil/stream_motion_interface
- **Robot**: FANUC CRX-10iA/L simulated in ROBOGUIDE at `192.168.56.1`
- **Protocol**: FANUC Stream Motion (J519), UDP port 60015, big-endian, ~8 ms cycle
- **Reference (working)**: `C:\Users\NIKHIL\PycharmProjects\stream_motion_gpt`

---

## Confirmed Working Pattern (from stream_motion_gpt + LEARNINGS doc)

1. **Quintic trajectory** — `10u³ − 15u⁴ + 6u⁵` — zero vel + zero acc at endpoints
2. **Box filter** on top (window=10) — barely distorts quintic; OK to apply
3. **Force `smooth_path[0] = start_joints`** — must exactly match servo position
4. **Status-paced send** — `recv_status()` → send one command → repeat
5. **Conservative duration** — GPT used 2.8 s for 3° move (~1% of limits)

### Tests confirmed working in stream_motion_gpt
| Test | Script | Result |
|------|--------|--------|
| Single-axis J1 move | `basic_joint_move.py` | Smooth, no alarms |
| 6-axis coordinated | `basic_joint_move_6axis.py` | Smooth, stable |
| Large multi-axis | `--delta-joints 8 5 -5 8 -8 8` | No issues |
| Move to HOME | `move_to_home.py` | Smooth, repeatable |

---

## Full Fault History & Root Causes

| Fault | Root Cause | Fix Applied |
|-------|-----------|-------------|
| MOTN-721 | Wrong Python limits (1.5–2× above robot values) | Added `CRX_VEL/ACC_LIMITS` from pendant |
| MOTN-721 | Hardcoded start position didn't match robot | Read live `get_current_joints()` at IBGN |
| MOTN-722 (Sq.1641) | Lead-in override: smoothed[0]=start but smoothed[1] jumped 0.030° due to box filter on quadratic trajectory | Lead-in fix (partial) |
| MOTN-722 (Sq.2) | Box filter on quadratic (s²) profile → step[0→1]=0.030°, vel=3.80 deg/s, jerk=101,875 deg/s³ | **Replaced with minimum_jerk_trajectory** |

---

## Current Code State

| File | Commit | Change |
|------|--------|--------|
| `stream_motion/trajectory.py` | 7d3d1bb | `minimum_jerk_trajectory()` added |
| `stream_motion/constants.py` | 0ba94dd | `HOME_JOINTS`, `PROTOCOL_VERSION_DEFAULT=3` |
| `stream_motion/client.py` | 0ba94dd | Default version → 3 |
| `examples/basic_joint_move.py` | 7c0caec | Uses `minimum_jerk_trajectory`, `SCALE=0.05` |
| `examples/basic_joint_move_6axis.py` | 0ba94dd | NEW — 6-axis coordinated move |
| `examples/move_to_home.py` | 0ba94dd | NEW — moves to `[0,0,0,0,-90,0]` |

**Git:** local `main` at `0ba94dd` — **push needed from PowerShell**.

---

## Actual Robot Limits ($STMO_GRP[1] from pendant)

```
$JNT_VEL_LIM[1..6] = [120, 120, 180, 180, 180, 180] deg/s
$JNT_ACC_LIM[1..6] = [279.9, 279.9, 419.9, 419.9, 419.9, 419.9] deg/s²
$JNT_JRK_LIM[1..6] = [1240, 1240, 1860, 1860, 1860, 1860] deg/s³
$LMT_MODE = 1  (limits updated every cycle)
$FLTR_LN = 1
$MAX_SPD = 2000 mm/s
Protocol max version = 3
```

---

## minimum_jerk_trajectory — Reference Numbers

Profile: `p(s) = D·(10s³−15s⁴+6s⁵)`, zero vel+acc at endpoints, no filter needed.

| SCALE | T (5° J1) | Waypoints | Peak jerk | % of limit |
|-------|-----------|-----------|-----------|-----------|
| 0.05  | 1.69 s    | 212       | 62 deg/s³ | 5% ✓ |
| 0.20  | 0.97 s    | 122       | 330 deg/s³ | 27% ✓ |
| 0.50  | 0.79 s    | 99        | 600 deg/s³ | 48% ✓ |
| 0.80  | 0.67 s    | 85        | 984 deg/s³ | 79% ✓ |

Speed ladder (raise after each confirmed fault-free test): `0.05 → 0.10 → 0.20 → 0.40 → 0.80`

Faults to watch when increasing speed:
- `MOTN-609` — velocity limit exceeded
- `MOTN-610` — acceleration limit exceeded
- `MOTN-611` — jerk limit exceeded
- `MOTN-603` — next command not received in time

---

## Next Steps

1. **Push to GitHub** from PowerShell:
   ```powershell
   cd C:\Users\NIKHIL\PycharmProjects\stream_motion_interface
   git push
   ```

2. **Test basic_joint_move.py** (SCALE=0.05, 5° J1 move):
   - RESET fault → run TP → run script
   - Expected: 212 waypoints, ~1.7 s, no MOTN fault

3. **Test move_to_home.py** (large multi-axis, same SCALE=0.05)

4. **Test basic_joint_move_6axis.py** (small 6-axis delta)

5. **Speed ladder**: once all three pass, raise SCALE in each script step by step

---

## HOME Position
`[0.0, 0.0, 0.0, 0.0, -90.0, 0.0]` — defined in `constants.py` as `HOME_JOINTS`

## Git Lock Workaround
```powershell
Remove-Item "C:\Users\NIKHIL\PycharmProjects\stream_motion_interface\.git\HEAD.lock" -Force
Remove-Item "C:\Users\NIKHIL\PycharmProjects\stream_motion_interface\.git\index.lock" -Force
```
