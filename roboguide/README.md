# ROBOGUIDE Setup for Stream Motion Testing

## Steps

1. Open ROBOGUIDE and load or create a workcell with an R-30iB Plus controller.
2. Ensure the **Stream Motion (J519)** option is loaded (check under System → Config).
3. Import `STREAM_MOTION_TEST.ls` into the TP program list.
4. Set `$STMO.$PHYS_PORT = 2` (or 1 for CD38A).
5. Set the controller IP to `127.0.0.1` (loopback, since Python runs on same PC).
6. Set the program to run in **AUTO mode** at **100% override**.
7. Run `STREAM_MOTION_TEST` — the cursor will stop at `IBGN start[1]` waiting for packets.
8. Now run `python examples/basic_joint_move.py`.
