"""Bounded off-thread conversion of camera reports to JSON packets."""

from __future__ import annotations

import base64
from collections import OrderedDict
from collections.abc import Callable
from dataclasses import dataclass
import threading
from typing import TypeAlias

from mc_bridge.packet import JsonObject


@dataclass(frozen=True, slots=True)
class CameraFrame:
    """A JPEG and pose captured for one connection-scoped request."""

    session_id: int
    request_sequence: int
    camera: str
    jpeg_data: bytes
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    orientation_x: float
    orientation_y: float
    orientation_z: float
    orientation_w: float


@dataclass(frozen=True, slots=True)
class CameraStreamFrame:
    """One H.264 frame split into Annex-B encoded units."""

    session_id: int
    request_sequence: int
    camera: str
    available: bool
    data: bytes
    unit_lengths: tuple[int, ...]


CameraReport: TypeAlias = CameraFrame | CameraStreamFrame
PacketHandler: TypeAlias = Callable[[CameraReport, JsonObject], None]
ErrorHandler: TypeAlias = Callable[[Exception], None]
CameraKey: TypeAlias = tuple[str, str]
PendingReport: TypeAlias = tuple[int, CameraReport]


class CameraReportWorker:
    """Convert only the newest pending report for each camera and kind."""

    def __init__(
        self,
        handler: PacketHandler,
        *,
        capacity: int,
        error_handler: ErrorHandler | None = None,
    ) -> None:
        """Start a worker with a small, bounded latest-value mailbox."""
        if capacity < 1:
            raise ValueError('Camera worker capacity must be positive')
        self._handler = handler
        self._capacity = capacity
        self._error_handler = error_handler or (lambda _: None)
        self._condition = threading.Condition()
        self._items: OrderedDict[CameraKey, PendingReport] = OrderedDict()
        self._versions: dict[CameraKey, int] = {}
        self._next_version = 0
        self._closed = False
        self._dropped = 0
        self._thread = threading.Thread(
            target=self._run,
            name='mc-bridge-camera-reports',
            daemon=False,
        )
        self._thread.start()

    @property
    def dropped(self) -> int:
        """Return the number of replaced or evicted reports."""
        with self._condition:
            return self._dropped

    def offer(self, report: CameraReport) -> bool:
        """Retain a report without blocking the ROS executor."""
        kind = 'frame' if isinstance(report, CameraFrame) else 'stream'
        key = kind, report.camera
        with self._condition:
            if self._closed:
                return False
            self._next_version += 1
            version = self._next_version
            if key in self._items:
                del self._items[key]
                self._dropped += 1
            elif len(self._items) == self._capacity:
                evicted_key, _ = self._items.popitem(last=False)
                self._versions.pop(evicted_key, None)
                self._dropped += 1
            self._versions[key] = version
            self._items[key] = version, report
            self._condition.notify()
            return True

    def clear(self) -> None:
        """Discard all pending reports, normally at session end."""
        with self._condition:
            self._dropped += len(self._items)
            self._items.clear()
            self._versions.clear()

    def close(self) -> None:
        """Discard pending work and stop the worker."""
        with self._condition:
            if self._closed:
                return
            self._closed = True
            self._items.clear()
            self._versions.clear()
            self._condition.notify()
        self._thread.join(timeout=5.0)
        if self._thread.is_alive():
            raise RuntimeError(
                'Camera worker did not stop within five seconds'
            )

    def _run(self) -> None:
        while True:
            with self._condition:
                while not self._closed and not self._items:
                    self._condition.wait()
                if self._closed:
                    return
                key, pending = self._items.popitem(last=False)
                version, report = pending
            try:
                packet = _to_packet(report)
                with self._condition:
                    if self._versions.get(key) != version:
                        self._dropped += 1
                        continue
                self._handler(report, packet)
            except Exception as error:
                self._error_handler(error)
            finally:
                with self._condition:
                    if self._versions.get(key) == version:
                        del self._versions[key]


def _to_packet(report: CameraReport) -> JsonObject:
    if isinstance(report, CameraFrame):
        return {
            'type': 'cameraFrameReport',
            'camera': report.camera,
            'data': base64.b64encode(report.jpeg_data).decode('ascii'),
            'lat': report.latitude_deg,
            'lon': report.longitude_deg,
            'alt': report.altitude_m,
            'orientX': report.orientation_x,
            'orientY': report.orientation_y,
            'orientZ': report.orientation_z,
            'orientW': report.orientation_w,
        }

    if not report.available:
        data: list[list[int]] | None = None
    else:
        data = []
        offset = 0
        for length in report.unit_lengths:
            end = offset + length
            data.append(list(report.data[offset:end]))
            offset = end
        if offset != len(report.data):
            raise ValueError('Camera stream unit lengths do not match data')
    return {
        'type': 'cameraStreamReport',
        'camera': report.camera,
        'data': data,
    }
