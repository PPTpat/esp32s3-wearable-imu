"""Wire-format decoder for the wearable IMU BLE protocol (no BLE dependencies).

Mirrors components/protocol/ in the firmware. See docs/ble_protocol.md.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

PROTOCOL_VERSION = 1
TYPE_SAMPLES = 0x01
HEADER_BYTES = 10
SAMPLE_BYTES = 12
CRC_BYTES = 2
MAX_SAMPLES_PER_FRAME = 16
INFO_BYTES = 12

SERVICE_UUID = "458f0001-54a4-46d8-a8d8-c8f5f76846d3"
DATA_UUID = "458f0002-54a4-46d8-a8d8-c8f5f76846d3"
CONTROL_UUID = "458f0003-54a4-46d8-a8d8-c8f5f76846d3"
INFO_UUID = "458f0004-54a4-46d8-a8d8-c8f5f76846d3"

CMD_START = b"\x01"
CMD_STOP = b"\x02"


class FrameError(Exception):
    """Raised when a frame fails validation. `reason` is a short machine-readable tag."""

    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


@dataclass(frozen=True)
class Sample:
    seq: int
    accel: tuple[int, int, int]  # raw LSB
    gyro: tuple[int, int, int]  # raw LSB


@dataclass(frozen=True)
class Frame:
    frame_seq: int
    first_sample_seq: int
    samples: list[Sample]


@dataclass(frozen=True)
class DeviceInfo:
    odr_hz: int
    accel_mg_per_lsb: float
    gyro_mdps_per_lsb: float


def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF). Check: b'123456789' -> 0x29B1."""
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def decode_frame(data: bytes) -> Frame:
    """Validate and parse one sample frame. Raises FrameError on any problem."""
    if len(data) < HEADER_BYTES + CRC_BYTES:
        raise FrameError("too_short")
    (crc_rx,) = struct.unpack_from("<H", data, len(data) - CRC_BYTES)
    if crc16_ccitt(data[:-CRC_BYTES]) != crc_rx:
        raise FrameError("bad_crc")
    version, ftype, frame_seq, first_seq, count, _reserved = struct.unpack_from("<BBHIBB", data, 0)
    if version != PROTOCOL_VERSION:
        raise FrameError("bad_version")
    if ftype != TYPE_SAMPLES:
        raise FrameError("bad_type")
    if count == 0 or count > MAX_SAMPLES_PER_FRAME or len(data) != HEADER_BYTES + count * SAMPLE_BYTES + CRC_BYTES:
        raise FrameError("bad_length")
    samples = []
    for i in range(count):
        values = struct.unpack_from("<6h", data, HEADER_BYTES + i * SAMPLE_BYTES)
        samples.append(Sample((first_seq + i) & 0xFFFFFFFF, values[:3], values[3:]))
    return Frame(frame_seq, first_seq, samples)


def decode_info(data: bytes) -> DeviceInfo:
    """Parse the Info characteristic."""
    if len(data) != INFO_BYTES:
        raise FrameError("bad_length")
    version, _reserved, odr, accel, gyro = struct.unpack("<BBHff", data)
    if version != PROTOCOL_VERSION:
        raise FrameError("bad_version")
    return DeviceInfo(odr, accel, gyro)