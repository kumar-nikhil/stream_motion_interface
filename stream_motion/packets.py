"""
Stream Motion Packet Definitions
FANUC R-30iB/R-30iB Plus Controller – Stream Motion (J519)
Manual: B-84904EN/01 – Chapter 3 COMMUNICATION PROTOCOL

All packets use big-endian byte order (network byte order).
Protocol uses UDP on port 60015.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Tuple, List

from .constants import (
    PKT_TYPE_STATUS_START,
    PKT_TYPE_COMMAND,
    PKT_TYPE_STATUS_STOP,
    PKT_TYPE_LIMIT_REQUEST,
    PKT_TYPE_CMD_POS_REQUEST,
    PROTOCOL_VERSION_1,
    DATA_FORMAT_JOINT,
    IO_TYPE_NONE,
)


# ─────────────────────────────────────────────────────────────────────────────
# Status Output Start Packet  (External Device → Robot)
# Table 3.3(a): Tells the robot to begin sending Status packets every cycle.
# ─────────────────────────────────────────────────────────────────────────────

# Format:  >II  =  big-endian, 2× unsigned int (4 bytes each)  → 8 bytes total
_STATUS_START_FMT = ">II"

def build_status_start_packet(version: int = PROTOCOL_VERSION_1) -> bytes:
    """Build a Status Output Start Packet (8 bytes)."""
    return struct.pack(_STATUS_START_FMT, PKT_TYPE_STATUS_START, version)


# ─────────────────────────────────────────────────────────────────────────────
# Status Output Stop Packet  (External Device → Robot)
# Table 3.3(d): Tells the robot to stop sending Status packets.
# ─────────────────────────────────────────────────────────────────────────────

_STATUS_STOP_FMT = ">II"

def build_status_stop_packet(version: int = PROTOCOL_VERSION_1) -> bytes:
    """Build a Status Output Stop Packet (8 bytes)."""
    return struct.pack(_STATUS_STOP_FMT, 2, version)  # packet_type = 2


# ─────────────────────────────────────────────────────────────────────────────
# Status Packet  (Robot → External Device)
# Table 3.3(b): Sent every communication cycle (~8 ms) by the robot.
# ─────────────────────────────────────────────────────────────────────────────

# Format breakdown:
#   >      = big-endian
#   I      = pkt_type          (4 bytes)
#   I      = version           (4 bytes)
#   I      = sequence_no       (4 bytes)
#   B      = status            (1 byte)
#   B      = io_r_type         (1 byte)
#   H      = io_r_idx          (2 bytes)
#   H      = io_r_mask         (2 bytes)
#   H      = r_io_val          (2 bytes)
#   I      = time_stamp        (4 bytes)
#   9f     = cart[9]           (36 bytes)  X,Y,Z,W,P,R,E1,E2,E3 [mm / deg]
#   9f     = ang[9]            (36 bytes)  J1..J9               [deg or mm]
#   9f     = q_current[9]      (36 bytes)  motor currents        [A]
# Total = 4+4+4+1+1+2+2+2+4+36+36+36 = 132 bytes (no padding issues in Python)

_STATUS_PKT_FMT = ">IIIBBBHHHHI9f9f9f"
# Note: We have BBB for status + io_r_type + pad? No – let's be precise:
# B status(1) + B io_r_type(1) + H io_r_idx(2) + H io_r_mask(2) + H r_io_val(2) + I time_stamp(4)
_STATUS_PKT_FMT = ">IIIB B H H H I 9f 9f 9f".replace(" ", "")

STATUS_PKT_SIZE = struct.calcsize(_STATUS_PKT_FMT)


@dataclass
class StatusPacket:
    """Parsed Status Packet received from the robot."""
    pkt_type:    int
    version:     int
    sequence_no: int
    status:      int        # Bitmask – see constants.STATUS_* flags
    io_r_type:   int
    io_r_idx:    int
    io_r_mask:   int
    r_io_val:    int
    time_stamp:  int        # ms, resolution 2 ms
    cart:        Tuple[float, ...]   # X,Y,Z,W,P,R,E1,E2,E3 [mm/deg]
    joints:      Tuple[float, ...]   # J1..J9               [deg or mm]
    currents:    Tuple[float, ...]   # Motor currents        [A]

    # ── Convenience Properties ────────────────────────────────────────────────
    @property
    def is_waiting_for_command(self) -> bool:
        """True when IBGN start[*] is active and robot is ready for commands."""
        return bool(self.status & 0x01)

    @property
    def is_command_received(self) -> bool:
        """True after the robot has received at least one command packet."""
        return bool(self.status & 0x02)

    @property
    def is_sysrdy(self) -> bool:
        """True when SYSRDY is ON."""
        return bool(self.status & 0x04)

    @property
    def is_moving(self) -> bool:
        """True when the robot is in motion."""
        return bool(self.status & 0x08)

    @property
    def xyz(self) -> Tuple[float, float, float]:
        return self.cart[0], self.cart[1], self.cart[2]

    @property
    def wpr(self) -> Tuple[float, float, float]:
        return self.cart[3], self.cart[4], self.cart[5]

    def __repr__(self) -> str:
        xyz = f"({self.cart[0]:.2f}, {self.cart[1]:.2f}, {self.cart[2]:.2f})"
        j = ", ".join(f"{j:.2f}" for j in self.joints[:6])
        return (
            f"StatusPacket(seq={self.sequence_no}, "
            f"status=0x{self.status:02X}, "
            f"XYZ={xyz}mm, "
            f"joints=[{j}]°, "
            f"t={self.time_stamp}ms)"
        )


def parse_status_packet(data: bytes) -> StatusPacket:
    """
    Parse raw bytes into a StatusPacket.
    Raises struct.error if the data is too short or malformed.
    """
    if len(data) < STATUS_PKT_SIZE:
        raise ValueError(
            f"Status packet too short: got {len(data)} bytes, expected {STATUS_PKT_SIZE}"
        )
    fields = struct.unpack_from(_STATUS_PKT_FMT, data)
    return StatusPacket(
        pkt_type    = fields[0],
        version     = fields[1],
        sequence_no = fields[2],
        status      = fields[3],
        io_r_type   = fields[4],
        io_r_idx    = fields[5],
        io_r_mask   = fields[6],
        r_io_val    = fields[7],
        time_stamp  = fields[8],
        cart        = fields[9:18],
        joints      = fields[18:27],
        currents    = fields[27:36],
    )


# ─────────────────────────────────────────────────────────────────────────────
# Command Packet  (External Device → Robot)
# Table 3.3(c): Sent each cycle to tell the robot its next destination position.
# ─────────────────────────────────────────────────────────────────────────────

# Format:
#   I  pkt_type       (4)
#   I  version        (4)
#   I  sequence_no    (4)
#   B  last           (1)   – set to 1 to terminate
#   B  io_r_type      (1)
#   H  io_r_idx       (2)
#   H  io_r_mask      (2)
#   B  data_format    (1)   – 0=Cartesian, 1=Joint
#   B  io_w_type      (1)
#   H  io_w_idx       (2)
#   H  io_w_mask      (2)
#   H  io_w_val       (2)
#   H  unused         (2)
#   9f cmd[9]         (36)  – destination positions
# Total = 4+4+4+1+1+2+2+1+1+2+2+2+2+36 = 64 bytes

_CMD_PKT_FMT = ">IIIB B HH B B HHH H 9f".replace(" ", "")
CMD_PKT_SIZE = struct.calcsize(_CMD_PKT_FMT)


def build_command_packet(
    sequence_no:   int,
    positions:     List[float],
    data_format:   int   = DATA_FORMAT_JOINT,
    last:          int   = 0,
    io_r_type:     int   = IO_TYPE_NONE,
    io_r_idx:      int   = 0,
    io_r_mask:     int   = 0,
    io_w_type:     int   = IO_TYPE_NONE,
    io_w_idx:      int   = 0,
    io_w_mask:     int   = 0,
    io_w_val:      int   = 0,
    version:       int   = PROTOCOL_VERSION_1,
) -> bytes:
    """
    Build a Command Packet.

    Args:
        sequence_no:  Match the sequence number from the last received Status Packet,
                      then increment for subsequent packets.
        positions:    List of 9 floats.
                      Joint format:     [J1, J2, J3, J4, J5, J6, E1, E2, E3] in degrees/mm.
                      Cartesian format: [X, Y, Z, W, P, R, E1, E2, E3] in mm/degrees.
        data_format:  DATA_FORMAT_JOINT (1) or DATA_FORMAT_CARTESIAN (0).
        last:         Set to 1 to send the final packet and end Stream Motion control.
        io_*:         Optional I/O read/write fields (see constants for types).
        version:      Protocol version number (default 1).

    Returns:
        64-byte packed binary packet ready for UDP transmission.
    """
    # Pad positions list to exactly 9 elements
    cmd = list(positions) + [0.0] * (9 - len(positions))
    cmd = cmd[:9]

    return struct.pack(
        _CMD_PKT_FMT,
        PKT_TYPE_COMMAND,   # pkt_type = 1
        version,
        sequence_no,
        last,
        io_r_type,
        io_r_idx,
        io_r_mask,
        data_format,
        io_w_type,
        io_w_idx,
        io_w_mask,
        io_w_val,
        0,                  # unused
        *cmd,
    )


# ─────────────────────────────────────────────────────────────────────────────
# Command Position Request Packet  (External Device → Robot)
# Table 4.4(a): Request the robot's current *command* position (not servo pos).
# Can only be sent once every 100 ms, and only after status output is started.
# ─────────────────────────────────────────────────────────────────────────────

_CMD_POS_REQ_FMT = ">II"

def build_cmd_pos_request_packet(version: int = PROTOCOL_VERSION_1) -> bytes:
    """Build a Command Position Request Packet (8 bytes)."""
    return struct.pack(_CMD_POS_REQ_FMT, PKT_TYPE_CMD_POS_REQUEST, version)


# ─────────────────────────────────────────────────────────────────────────────
# Command Position Response Packet  (Robot → External Device)
# Table 4.4(b): Contains both Cartesian and Joint command positions.
# ─────────────────────────────────────────────────────────────────────────────

# Format: I I I  9f  9f  = 4+4+4+36+36 = 84 bytes
_CMD_POS_RESP_FMT = ">III 9f 9f".replace(" ", "")
CMD_POS_RESP_SIZE = struct.calcsize(_CMD_POS_RESP_FMT)


@dataclass
class CommandPositionResponse:
    """Parsed Command Position Response from the robot."""
    pkt_type:    int
    version:     int
    time_stamp:  int
    cart:        Tuple[float, ...]   # X,Y,Z,W,P,R,E1,E2,E3
    joints:      Tuple[float, ...]   # J1..J9


def parse_cmd_pos_response(data: bytes) -> CommandPositionResponse:
    """Parse a Command Position Response Packet."""
    fields = struct.unpack_from(_CMD_POS_RESP_FMT, data)
    return CommandPositionResponse(
        pkt_type   = fields[0],
        version    = fields[1],
        time_stamp = fields[2],
        cart       = fields[3:12],
        joints     = fields[12:21],
    )


# ─────────────────────────────────────────────────────────────────────────────
# Allowable Limit Table Request Packet  (External Device → Robot)
# Table B.3(a): Request velocity / acceleration / jerk limits for one axis.
# ─────────────────────────────────────────────────────────────────────────────

_LIMIT_REQ_FMT = ">IIII"

def build_limit_request_packet(
    axis_number: int,        # 1–9
    limit_type:  int,        # 0=velocity, 1=acceleration, 2=jerk
    version:     int = PROTOCOL_VERSION_1,
) -> bytes:
    """Build an Allowable Limit Table Request Packet (16 bytes)."""
    return struct.pack(_LIMIT_REQ_FMT, PKT_TYPE_LIMIT_REQUEST, version, axis_number, limit_type)
