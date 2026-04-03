"""
Stream Motion Client
FANUC R-30iB/R-30iB Plus Controller – Stream Motion (J519)
Manual: B-84904EN/01

Usage flow (Section 2 & 3):
  1. Robot TP program runs IBGN start[*] → robot waits
  2. Client sends StatusOutputStart → robot streams StatusPackets every ~8ms
  3. Client waits for status.is_waiting_for_command == True
  4. Client sends CommandPackets in a tight loop (one per status packet received)
  5. When finished, client sends CommandPacket with last=1
  6. Client sends StatusOutputStop
"""

from __future__ import annotations

import logging
import socket
import threading
import time
from typing import Callable, List, Optional, Tuple

from .constants import (
    ROBOT_DEFAULT_IP,
    ROBOT_UDP_PORT,
    COMM_CYCLE_MS,
    SOCKET_TIMEOUT_S,
    STATUS_WAITING_FOR_CMD,
    DATA_FORMAT_JOINT,
    DATA_FORMAT_CARTESIAN,
    IO_TYPE_NONE,
    PROTOCOL_VERSION_1,
    PROTOCOL_VERSION_DEFAULT,
    LIMIT_TYPE_VELOCITY,
    LIMIT_TYPE_ACCELERATION,
    LIMIT_TYPE_JERK,
)
from .packets import (
    StatusPacket,
    CommandPositionResponse,
    build_status_start_packet,
    build_status_stop_packet,
    build_command_packet,
    build_cmd_pos_request_packet,
    build_limit_request_packet,
    parse_status_packet,
    parse_cmd_pos_response,
    STATUS_PKT_SIZE,
    CMD_POS_RESP_SIZE,
)

logger = logging.getLogger(__name__)


class StreamMotionClient:
    """
    UDP client for the FANUC Stream Motion interface.

    Example (simple joint-space move):
        client = StreamMotionClient(robot_ip="127.0.0.1")
        client.connect()
        client.start_status_output()

        # Wait until robot TP program reaches IBGN start[*]
        client.wait_for_ready(timeout=30.0)

        # Stream a pre-planned joint trajectory
        trajectory = [[0, 0, 0, 0, 0, 0], [10, 5, -5, 0, 0, 0], ...]
        client.stream_joint_trajectory(trajectory)

        client.stop_status_output()
        client.disconnect()
    """

    def __init__(
        self,
        robot_ip:   str   = ROBOT_DEFAULT_IP,
        robot_port: int   = ROBOT_UDP_PORT,
        timeout:    float = SOCKET_TIMEOUT_S,
        version:    int   = PROTOCOL_VERSION_DEFAULT,
    ):
        self.robot_ip   = robot_ip
        self.robot_port = robot_port
        self.timeout    = timeout
        self.version    = version

        self._sock: Optional[socket.socket] = None
        self._robot_addr: Tuple[str, int] = (robot_ip, robot_port)

        # Latest received status packet (thread-safe via lock)
        self._last_status: Optional[StatusPacket] = None
        self._status_lock  = threading.Lock()
        self._status_event = threading.Event()  # Fires on each new status packet

        # Background listener thread
        self._listener_thread:  Optional[threading.Thread] = None
        self._listener_running: bool = False

        # Callbacks
        self._on_status_callback: Optional[Callable[[StatusPacket], None]] = None

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(self) -> None:
        """Open the UDP socket."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(self.timeout)
        logger.info("UDP socket opened → %s:%d", self.robot_ip, self.robot_port)

    def disconnect(self) -> None:
        """Stop the listener and close the socket."""
        self._stop_listener()
        if self._sock:
            self._sock.close()
            self._sock = None
        logger.info("Disconnected from robot")

    def __enter__(self) -> "StreamMotionClient":
        self.connect()
        return self

    def __exit__(self, *args) -> None:
        self.disconnect()

    # ── Status Streaming ──────────────────────────────────────────────────────

    def start_status_output(self) -> None:
        """
        Send Status Output Start Packet → robot will stream StatusPackets every cycle.
        Automatically starts a background listener thread.
        """
        pkt = build_status_start_packet(self.version)
        self._send(pkt)
        self._start_listener()
        logger.info("Status output started")

    def stop_status_output(self) -> None:
        """Send Status Output Stop Packet → robot stops streaming."""
        pkt = build_status_stop_packet(self.version)
        self._send(pkt)
        self._stop_listener()
        logger.info("Status output stopped")

    def set_status_callback(self, callback: Callable[[StatusPacket], None]) -> None:
        """Register a function to be called every time a status packet is received."""
        self._on_status_callback = callback

    # ── Waiting ───────────────────────────────────────────────────────────────

    def wait_for_ready(self, timeout: float = 30.0) -> bool:
        """
        Block until the robot's TP program has reached IBGN start[*] and is
        ready to receive command packets (status bit 0 goes HIGH).

        Returns True if ready within timeout, False otherwise.
        """
        deadline = time.monotonic() + timeout
        logger.info("Waiting for robot to reach IBGN start[*] (timeout=%.1fs)...", timeout)
        while time.monotonic() < deadline:
            self._status_event.wait(timeout=0.1)
            self._status_event.clear()
            with self._status_lock:
                s = self._last_status
            if s and s.is_waiting_for_command:
                logger.info("Robot is ready (seq=%d)", s.sequence_no)
                return True
        logger.warning("Timed out waiting for robot ready signal")
        return False

    def wait_for_stop(self, timeout: float = 10.0) -> bool:
        """Block until the robot stops moving (status bit 3 goes LOW)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._status_event.wait(timeout=0.1)
            self._status_event.clear()
            with self._status_lock:
                s = self._last_status
            if s and not s.is_moving:
                return True
        return False

    # ── Status Inspection ─────────────────────────────────────────────────────

    @property
    def last_status(self) -> Optional[StatusPacket]:
        """Returns the most recently received StatusPacket (thread-safe)."""
        with self._status_lock:
            return self._last_status

    def get_current_joints(self) -> Optional[List[float]]:
        """Returns [J1..J6] in degrees, or None if no status received yet."""
        with self._status_lock:
            s = self._last_status
        return list(s.joints[:6]) if s else None

    def get_current_cart(self) -> Optional[List[float]]:
        """Returns [X, Y, Z, W, P, R] in mm/degrees, or None if no status received yet."""
        with self._status_lock:
            s = self._last_status
        return list(s.cart[:6]) if s else None

    # ── Command Position (Section 4.4) ────────────────────────────────────────

    def get_command_position(self, timeout: float = 0.5) -> Optional[CommandPositionResponse]:
        """
        Request the robot's current *command* position (not servo feedback position).
        May be sent at most once per 100 ms.
        Returns a CommandPositionResponse or None on timeout.
        """
        if not self._sock:
            raise RuntimeError("Not connected – call connect() first")
        pkt = build_cmd_pos_request_packet(self.version)
        self._send(pkt)
        old_timeout = self._sock.gettimeout()
        self._sock.settimeout(timeout)
        try:
            data, _ = self._sock.recvfrom(4096)
            return parse_cmd_pos_response(data)
        except socket.timeout:
            logger.warning("Command position request timed out")
            return None
        finally:
            self._sock.settimeout(old_timeout)

    # ── Trajectory Streaming ──────────────────────────────────────────────────

    def stream_joint_trajectory(
        self,
        trajectory: List[List[float]],
        cycle_s: float = COMM_CYCLE_MS / 1000.0,
    ) -> bool:
        """
        Stream a pre-planned joint-space trajectory to the robot.

        Args:
            trajectory: List of waypoints, each a list of up to 9 joint values [deg/mm].
                        The sequence is sent at ~one packet per received status cycle.
            cycle_s:    Fallback send rate if background listener is not active.

        Returns:
            True if all packets sent successfully, False if the robot stopped early.

        NOTE: The robot's velocity, acceleration, and jerk limits MUST be respected.
              See Section 4.3 and Appendix B for details on limits.
        """
        return self._stream_trajectory(
            trajectory=trajectory,
            data_format=DATA_FORMAT_JOINT,
            cycle_s=cycle_s,
        )

    def stream_cartesian_trajectory(
        self,
        trajectory: List[List[float]],
        cycle_s: float = COMM_CYCLE_MS / 1000.0,
    ) -> bool:
        """
        Stream a pre-planned Cartesian trajectory [X,Y,Z,W,P,R,...] to the robot.

        NOTE: Cartesian format is only supported on 6-axis robots.
              The robot configuration must not change during the trajectory.
              (See Section 4.1 and alarm MOTN-156)
        """
        return self._stream_trajectory(
            trajectory=trajectory,
            data_format=DATA_FORMAT_CARTESIAN,
            cycle_s=cycle_s,
        )

    def _stream_trajectory(
        self,
        trajectory: List[List[float]],
        data_format: int,
        cycle_s: float,
    ) -> bool:
        """Internal: stream a trajectory and send the final 'last=1' packet."""
        if not self._sock:
            raise RuntimeError("Not connected – call connect() first")

        total = len(trajectory)
        logger.info(
            "Streaming %d waypoints (format=%s)",
            total,
            "Cartesian" if data_format == DATA_FORMAT_CARTESIAN else "Joint",
        )

        # Per the manual (Table 3.3c): the first command packet must use the same
        # sequence number as the last received status packet. Each subsequent command
        # packet increments its own counter independently (not the status seq).
        cmd_seq_no: Optional[int] = None   # initialised from first status packet

        for idx, waypoint in enumerate(trajectory):
            is_last = 1 if idx == total - 1 else 0

            # Wait for the next status packet to sync to the robot's cycle
            self._status_event.wait(timeout=cycle_s * 3)
            self._status_event.clear()

            with self._status_lock:
                s = self._last_status

            if s is None:
                logger.error("No status packet received – aborting trajectory")
                return False

            if not s.is_waiting_for_command and not s.is_command_received:
                logger.warning(
                    "Robot stopped waiting for commands at waypoint %d/%d "
                    "(status=0x%02X moving=%s) — check pendant for faults",
                    idx, total, s.status, s.is_moving,
                )
                return False

            # First packet: mirror the status seq; subsequent packets: increment own counter
            if cmd_seq_no is None:
                cmd_seq_no = s.sequence_no
            else:
                cmd_seq_no = (cmd_seq_no + 1) & 0xFFFFFFFF

            pkt = build_command_packet(
                sequence_no = cmd_seq_no,
                positions   = waypoint,
                data_format = data_format,
                last        = is_last,
                version     = self.version,
            )
            self._send(pkt)

            if idx % 50 == 0 or is_last:
                logger.debug("Sent waypoint %d/%d (cmd_seq=%d, last=%d)", idx + 1, total, cmd_seq_no, is_last)

        logger.info("Trajectory complete")
        return True

    # ── Limit Table (Appendix B) ──────────────────────────────────────────────

    def request_limit_table(
        self,
        axis_number: int,
        limit_type:  int = LIMIT_TYPE_VELOCITY,
        timeout:     float = 1.0,
    ) -> Optional[bytes]:
        """
        Request the allowable limit table for a given axis and limit type.
        Returns raw response bytes (caller is responsible for parsing Table B.3b).

        Args:
            axis_number: 1–9
            limit_type:  LIMIT_TYPE_VELOCITY / ACCELERATION / JERK  (0/1/2)
        """
        pkt = build_limit_request_packet(axis_number, limit_type, self.version)
        self._send(pkt)
        old_timeout = self._sock.gettimeout()
        self._sock.settimeout(timeout)
        try:
            data, _ = self._sock.recvfrom(4096)
            return data
        except socket.timeout:
            logger.warning("Limit table request timed out")
            return None
        finally:
            self._sock.settimeout(old_timeout)

    # ── Background Listener Thread ────────────────────────────────────────────

    def _start_listener(self) -> None:
        """Start the background status-packet listener thread."""
        if self._listener_running:
            return
        self._listener_running = True
        self._listener_thread = threading.Thread(
            target=self._listener_loop,
            name="StreamMotionListener",
            daemon=True,
        )
        self._listener_thread.start()
        logger.debug("Listener thread started")

    def _stop_listener(self) -> None:
        """Signal the listener thread to stop and wait for it."""
        self._listener_running = False
        self._status_event.set()  # Unblock any waiting code
        if self._listener_thread and self._listener_thread.is_alive():
            self._listener_thread.join(timeout=2.0)
        self._listener_thread = None
        logger.debug("Listener thread stopped")

    def _listener_loop(self) -> None:
        """Background thread: receive status packets from the robot."""
        logger.debug("Listener loop running")
        while self._listener_running:
            try:
                data, _ = self._sock.recvfrom(4096)
                if len(data) >= STATUS_PKT_SIZE:
                    status = parse_status_packet(data)
                    with self._status_lock:
                        self._last_status = status
                    self._status_event.set()
                    if self._on_status_callback:
                        try:
                            self._on_status_callback(status)
                        except Exception as exc:
                            logger.error("Status callback error: %s", exc)
            except socket.timeout:
                pass  # Normal – robot may not have started streaming yet
            except Exception as exc:
                if self._listener_running:
                    logger.error("Listener error: %s", exc)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _send(self, data: bytes) -> None:
        """Send raw bytes to the robot."""
        if not self._sock:
            raise RuntimeError("Not connected – call connect() first")
        self._sock.sendto(data, self._robot_addr)
