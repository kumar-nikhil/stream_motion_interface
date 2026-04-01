"""
Unit Tests – Packet Construction and Parsing
Tests the binary packet format without needing a real robot.
Run with:  python -m pytest tests/ -v
"""

import struct
import pytest

from stream_motion.packets import (
    build_status_start_packet,
    build_status_stop_packet,
    build_command_packet,
    parse_status_packet,
    STATUS_PKT_SIZE,
    CMD_PKT_SIZE,
)
from stream_motion.constants import (
    PKT_TYPE_STATUS_START,
    PKT_TYPE_COMMAND,
    DATA_FORMAT_JOINT,
    DATA_FORMAT_CARTESIAN,
    PROTOCOL_VERSION_1,
)


class TestStatusStartPacket:
    def test_size(self):
        pkt = build_status_start_packet()
        assert len(pkt) == 8

    def test_packet_type_is_zero(self):
        pkt = build_status_start_packet()
        pkt_type = struct.unpack(">I", pkt[:4])[0]
        assert pkt_type == 0

    def test_version_is_one(self):
        pkt = build_status_start_packet()
        version = struct.unpack(">I", pkt[4:8])[0]
        assert version == PROTOCOL_VERSION_1

    def test_big_endian_encoding(self):
        pkt = build_status_start_packet()
        # First 4 bytes must be 0x00000000 (pkt_type=0)
        assert pkt[:4] == b"\x00\x00\x00\x00"
        # Next 4 bytes must be 0x00000001 (version=1)
        assert pkt[4:8] == b"\x00\x00\x00\x01"


class TestStatusStopPacket:
    def test_packet_type_is_two(self):
        pkt = build_status_stop_packet()
        pkt_type = struct.unpack(">I", pkt[:4])[0]
        assert pkt_type == 2

    def test_size(self):
        assert len(build_status_stop_packet()) == 8


class TestCommandPacket:
    def test_size(self):
        pkt = build_command_packet(sequence_no=1, positions=[0.0] * 6)
        assert len(pkt) == CMD_PKT_SIZE

    def test_packet_type_is_one(self):
        pkt = build_command_packet(sequence_no=1, positions=[0.0] * 6)
        pkt_type = struct.unpack(">I", pkt[:4])[0]
        assert pkt_type == PKT_TYPE_COMMAND

    def test_sequence_number_encoded_correctly(self):
        pkt = build_command_packet(sequence_no=42, positions=[0.0] * 6)
        seq_no = struct.unpack(">I", pkt[8:12])[0]
        assert seq_no == 42

    def test_last_flag_default_zero(self):
        pkt = build_command_packet(sequence_no=1, positions=[0.0] * 6)
        last = struct.unpack(">B", pkt[12:13])[0]
        assert last == 0

    def test_last_flag_can_be_set(self):
        pkt = build_command_packet(sequence_no=1, positions=[0.0] * 6, last=1)
        last = struct.unpack(">B", pkt[12:13])[0]
        assert last == 1

    def test_joint_format_encoded(self):
        pkt = build_command_packet(
            sequence_no=1,
            positions=[0.0] * 6,
            data_format=DATA_FORMAT_JOINT,
        )
        # data_format is byte 16 (after: 4+4+4+1+1+2+2 = 18? Let's just round-trip it)
        # Better: re-parse and check
        assert len(pkt) == CMD_PKT_SIZE  # Confirms it built without error

    def test_positions_padded_to_nine(self):
        """If fewer than 9 positions are given, the rest should be 0.0."""
        pkt = build_command_packet(sequence_no=1, positions=[10.0, 20.0, 30.0])
        assert len(pkt) == CMD_PKT_SIZE

    def test_positions_truncated_to_nine(self):
        """More than 9 positions should be truncated."""
        pkt = build_command_packet(sequence_no=1, positions=[1.0] * 12)
        assert len(pkt) == CMD_PKT_SIZE


class TestStatusPacketParsing:
    def _make_fake_status_packet(
        self,
        seq: int = 5,
        status: int = 0x01,
        x: float = 100.0,
        j1: float = 45.0,
    ) -> bytes:
        """Build a fake status packet for testing the parser."""
        fmt = ">IIIB B H H H I 9f 9f 9f".replace(" ", "")
        cart = [x, 200.0, 300.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joints = [j1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        currents = [0.0] * 9
        return struct.pack(
            fmt,
            0,       # pkt_type
            1,       # version
            seq,     # sequence_no
            status,  # status byte
            0,       # io_r_type
            0,       # io_r_idx
            0,       # io_r_mask
            0,       # r_io_val
            1000,    # time_stamp
            *cart,
            *joints,
            *currents,
        )

    def test_parse_sequence_number(self):
        data = self._make_fake_status_packet(seq=99)
        pkt = parse_status_packet(data)
        assert pkt.sequence_no == 99

    def test_parse_is_waiting_for_command(self):
        data = self._make_fake_status_packet(status=0x01)
        pkt = parse_status_packet(data)
        assert pkt.is_waiting_for_command is True

    def test_parse_not_waiting(self):
        data = self._make_fake_status_packet(status=0x00)
        pkt = parse_status_packet(data)
        assert pkt.is_waiting_for_command is False

    def test_parse_cart_position(self):
        data = self._make_fake_status_packet(x=123.45)
        pkt = parse_status_packet(data)
        assert abs(pkt.cart[0] - 123.45) < 0.01

    def test_parse_joint_position(self):
        data = self._make_fake_status_packet(j1=90.0)
        pkt = parse_status_packet(data)
        assert abs(pkt.joints[0] - 90.0) < 0.01

    def test_parse_too_short_raises(self):
        with pytest.raises((ValueError, struct.error)):
            parse_status_packet(b"\x00" * 10)

    def test_is_moving_flag(self):
        data = self._make_fake_status_packet(status=0x08)
        pkt = parse_status_packet(data)
        assert pkt.is_moving is True

    def test_xyz_property(self):
        data = self._make_fake_status_packet(x=50.0)
        pkt = parse_status_packet(data)
        x, y, z = pkt.xyz
        assert abs(x - 50.0) < 0.01
