"""Tests for bounded off-thread camera report conversion."""

import threading

import pytest

import mc_bridge.camera_worker as camera_worker
from mc_bridge.camera_worker import (
    CameraFrame,
    CameraReport,
    CameraReportWorker,
    CameraStreamFrame,
)
from mc_bridge.packet import JsonObject


def _frame(data: bytes = b'jpeg') -> CameraFrame:
    return CameraFrame(
        session_id=10,
        request_sequence=2,
        camera='mast',
        jpeg_data=data,
        latitude_deg=47.0,
        longitude_deg=-122.0,
        altitude_m=30.0,
        orientation_x=0.0,
        orientation_y=0.0,
        orientation_z=0.0,
        orientation_w=1.0,
    )


def test_camera_frame_is_base64_encoded_off_thread() -> None:
    """JPEG bytes become the unchanged Mission Control packet shape."""
    delivered = threading.Event()
    packets: list[JsonObject] = []

    def handle(_: CameraReport, packet: JsonObject) -> None:
        packets.append(packet)
        delivered.set()

    worker = CameraReportWorker(handle, capacity=1)
    try:
        assert worker.offer(_frame(b'jpeg'))
        assert delivered.wait(timeout=1.0)
    finally:
        worker.close()

    assert packets[0]['data'] == 'anBlZw=='
    assert packets[0]['orientW'] == 1.0


def test_stream_units_are_reconstructed_for_mission_control() -> None:
    """Flattened ROS bytes recover the legacy number-array framing."""
    delivered = threading.Event()
    packets: list[JsonObject] = []

    def handle(_: CameraReport, packet: JsonObject) -> None:
        packets.append(packet)
        delivered.set()

    worker = CameraReportWorker(handle, capacity=1)
    try:
        assert worker.offer(
            CameraStreamFrame(
                session_id=10,
                request_sequence=3,
                camera='mast',
                available=True,
                data=b'\x00\x00\x01\x65\x88',
                unit_lengths=(3, 2),
            ),
        )
        assert delivered.wait(timeout=1.0)
    finally:
        worker.close()

    assert packets[0]['data'] == [[0, 0, 1], [101, 136]]


def test_unavailable_stream_frame_becomes_null() -> None:
    """An unavailable camera remains distinct from an empty encoded frame."""
    delivered = threading.Event()
    packets: list[JsonObject] = []

    def handle(_: CameraReport, packet: JsonObject) -> None:
        packets.append(packet)
        delivered.set()

    worker = CameraReportWorker(handle, capacity=1)
    try:
        assert worker.offer(
            CameraStreamFrame(
                session_id=10,
                request_sequence=3,
                camera='mast',
                available=False,
                data=b'',
                unit_lengths=(),
            ),
        )
        assert delivered.wait(timeout=1.0)
    finally:
        worker.close()

    assert packets[0]['data'] is None


def test_latest_camera_value_replaces_pending_work() -> None:
    """A slow encoder cannot create an unbounded stale-frame backlog."""
    handler_started = threading.Event()
    release_handler = threading.Event()
    delivered = threading.Event()
    payloads: list[str] = []

    def handle(_: CameraReport, packet: JsonObject) -> None:
        payloads.append(str(packet['data']))
        if len(payloads) == 1:
            handler_started.set()
            assert release_handler.wait(timeout=1.0)
        else:
            delivered.set()

    worker = CameraReportWorker(handle, capacity=1)
    try:
        assert worker.offer(_frame(b'first'))
        assert handler_started.wait(timeout=1.0)
        assert worker.offer(_frame(b'second'))
        assert worker.offer(_frame(b'third'))
        release_handler.set()
        assert delivered.wait(timeout=1.0)
    finally:
        release_handler.set()
        worker.close()

    assert worker.dropped == 1
    assert payloads == ['Zmlyc3Q=', 'dGhpcmQ=']


def test_newer_frame_supersedes_one_still_encoding(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Obsolete conversion work is dropped before publication."""
    encoding_started = threading.Event()
    release_encoding = threading.Event()
    delivered = threading.Event()
    packets: list[JsonObject] = []
    original = camera_worker._to_packet

    def slow_conversion(report: CameraReport) -> JsonObject:
        if isinstance(report, CameraFrame) and report.jpeg_data == b'first':
            encoding_started.set()
            assert release_encoding.wait(timeout=1.0)
        return original(report)

    def handle(_: CameraReport, packet: JsonObject) -> None:
        packets.append(packet)
        delivered.set()

    monkeypatch.setattr(camera_worker, '_to_packet', slow_conversion)
    worker = CameraReportWorker(handle, capacity=1)
    try:
        assert worker.offer(_frame(b'first'))
        assert encoding_started.wait(timeout=1.0)
        assert worker.offer(_frame(b'second'))
        release_encoding.set()
        assert delivered.wait(timeout=1.0)
    finally:
        release_encoding.set()
        worker.close()

    assert worker.dropped == 1
    assert packets[0]['data'] == 'c2Vjb25k'
