"""
Stream Motion Protocol Constants
FANUC R-30iB/R-30iB Plus Controller – Stream Motion (J519)
Manual: B-84904EN/01
"""

# ── Network ──────────────────────────────────────────────────────────────────
ROBOT_DEFAULT_IP = "127.0.0.1"          # Use localhost when testing with ROBOGUIDE on same PC
ROBOT_UDP_PORT   = 60015                 # Fixed UDP port (Section 3.1)
COMM_CYCLE_MS    = 8                     # Default communication cycle in milliseconds
SOCKET_TIMEOUT_S = 1.0                   # Socket receive timeout in seconds

# ── Packet Types (Table 3.3) ──────────────────────────────────────────────────
PKT_TYPE_STATUS_START    = 0             # External Device → Robot  (Table 3.3a)
PKT_TYPE_STATUS          = 0             # Robot → External Device  (Table 3.3b)
PKT_TYPE_COMMAND         = 1             # External Device → Robot  (Table 3.3c)
PKT_TYPE_STATUS_STOP     = 2             # External Device → Robot  (Table 3.3d)
PKT_TYPE_LIMIT_REQUEST   = 3             # External Device → Robot  (Table B.3a)
PKT_TYPE_LIMIT_RESPONSE  = 3             # Robot → External Device  (Table B.3b)
PKT_TYPE_CMD_POS_REQUEST = 4             # External Device → Robot  (Table 4.4a)
PKT_TYPE_CMD_POS_RESPONSE= 4            # Robot → External Device  (Table 4.4b)

# ── Protocol Versions ─────────────────────────────────────────────────────────
PROTOCOL_VERSION_1 = 1                   # Base version (joint only)
PROTOCOL_VERSION_2 = 2                   # Adds double-precision Cartesian (Appendix E.1)
PROTOCOL_VERSION_3 = 3                   # Adds comm timing adjustment (Appendix E.2)
#
# CRX-10iA/L in ROBOGUIDE confirmed: robot reports max_version = 3
# Version 3 enables adaptive deceleration and dynamic status rate — safer for
# high-speed streaming.  Use this as the default for all new scripts.
PROTOCOL_VERSION_DEFAULT = PROTOCOL_VERSION_3

# ── Position Data Formats (Command Packet – data_format field) ────────────────
DATA_FORMAT_CARTESIAN = 0               # XYZ + WPR + 3 extended axes (6-axis robots only)
DATA_FORMAT_JOINT     = 1               # J1–J6 (+ up to J9 for extended axes)

# ── Status Packet Bit Masks (status byte) ─────────────────────────────────────
STATUS_WAITING_FOR_CMD  = 0x01          # Bit 0: 1 = IBGN start[*] active, waiting for command
STATUS_CMD_RECEIVED     = 0x02          # Bit 1: 1 = command packet has been received
STATUS_SYSRDY           = 0x04          # Bit 2: 1 = SYSRDY ON
STATUS_ROBOT_MOVING     = 0x08          # Bit 3: 1 = robot is moving

# ── I/O Types (Reading/Writing I/O fields in Command Packet) ──────────────────
IO_TYPE_NONE = 0
IO_TYPE_DI   = 1
IO_TYPE_DO   = 2
IO_TYPE_RI   = 8
IO_TYPE_RO   = 9
IO_TYPE_SI   = 11
IO_TYPE_SO   = 12
IO_TYPE_WI   = 16
IO_TYPE_WO   = 17
IO_TYPE_UI   = 20
IO_TYPE_UO   = 21
IO_TYPE_WSI  = 26
IO_TYPE_WSO  = 27
IO_TYPE_F    = 35
IO_TYPE_M    = 36

# ── Physical Port (system variable $STMO.$PHYS_PORT) ─────────────────────────
PHYS_PORT_CD38A = 1
PHYS_PORT_CD38B = 2                     # Default

# ── Packet Buffer (system variable $STMO.$PKT_STACK) ─────────────────────────
PKT_STACK_MIN     = 2
PKT_STACK_MAX     = 10
PKT_STACK_DEFAULT = 10

# ── Limit Types for Limit Table Request (Table B.3a) ─────────────────────────
LIMIT_TYPE_VELOCITY     = 0             # deg/s
LIMIT_TYPE_ACCELERATION = 1             # deg/s²
LIMIT_TYPE_JERK         = 2             # deg/s³

# ── Packet Sizes (bytes) ───────────────────────────────────────────────────────
# These match the struct sizes defined in packets.py
SIZE_STATUS_START_PKT  = 8
SIZE_STATUS_PKT        = 164            # 4+4+4+1+1+2+2+2+4 + 9*4 + 9*4 + 9*4 + padding
SIZE_COMMAND_PKT       = 68             # 4+4+4+1+1+2+2+1+1+2+2+2+2 + 9*4
SIZE_STATUS_STOP_PKT   = 8
SIZE_LIMIT_REQUEST_PKT = 16
SIZE_CMD_POS_REQ_PKT   = 8
SIZE_CMD_POS_RESP_PKT  = 88            # 4+4+4 + 9*4 + 9*4

# ── Sequence Number Rollover ───────────────────────────────────────────────────
SEQ_NO_MAX = 0xFFFFFFFF                 # Rolls over to 0 after this value

# ── Abnormal Position Threshold (system variable $STMO.$THRS_ABNPOS) ─────────
THRS_ABNPOS_DEFAULT = 100_000           # mm or degrees — raise MOTN-625 if exceeded

# ── CRX-10iA/L Actual Robot Limits (read from $STMO_GRP[1] on pendant) ────────
# These are READ-ONLY reference values at full payload + max Cartesian speed.
# ($LMT_MODE=1, so actual enforced limits may be higher at low speeds.)
# Source: ROBOGUIDE system variables, verified 2026-04-01.
#
#   $STMO_GRP[1].$JNT_VEL_LIM[1..6]  [deg/s]
CRX_VEL_LIMITS  = [120.0, 120.0, 180.0, 180.0, 180.0, 180.0]
#
#   $STMO_GRP[1].$JNT_ACC_LIM[1..6]  [deg/s²]  – use 5% margin
CRX_ACC_LIMITS  = [265.0, 265.0, 399.0, 399.0, 399.0, 399.0]
#
#   $STMO_GRP[1].$JNT_JRK_LIM[1..6]  [deg/s³]  – use 5% margin
CRX_JRK_LIMITS  = [1240.0, 1240.0, 1860.0, 1860.0, 1860.0, 1860.0]
#
#   $STMO_GRP[1].$MAX_SPD = 2000 mm/s  (Vmax for limit table interpolation)
CRX_MAX_SPD_MMS = 2000

# ── CRX-10iA/L Collaborative Safety Limits ───────────────────────────────────
# SYST-323 "Collaborative speed limit (TCP)" is triggered when the TCP
# Cartesian speed exceeds the robot's collaborative limit.
# For CRX-10iA/L this is 750 mm/s (enforced by the robot safety system,
# independent of the stream motion joint limits).
CRX_COLLAB_TCP_LIMIT_MMS = 750.0        # hard limit — triggers SYST-323
CRX_COLLAB_TCP_PLAN_MMS  = 700.0        # planning target (7% margin below hard limit)

# CRX-10iA/L maximum reach [mm] — used as conservative upper-bound lever arm
# for all joints when estimating worst-case TCP Cartesian speed.
# The true lever arm per joint depends on robot configuration; using the full
# reach for all joints is safe (overestimates TCP speed → gives more margin).
# Actual reach from FANUC datasheet: 1249 mm.
CRX_REACH_MM = 1249.0

# ── CRX-10iA/L Home Position [deg] ───────────────────────────────────────────
# Verified in ROBOGUIDE: safe upright pose used for large-range return moves.
#   J1=0  J2=0  J3=0  J4=0  J5=-90  J6=0
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, -90.0, 0.0]
