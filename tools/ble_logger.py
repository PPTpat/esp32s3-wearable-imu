"""Connect to the wearable IMU, stream samples, report loss and optionally save a CSV.

Usage:
    pip install -r tools/requirements.txt
    python tools/ble_logger.py --duration 30 --csv data/session.csv
"""
from __future__ import annotations

import argparse
import asyncio
import csv
import sys
import time

from bleak import BleakClient, BleakScanner

import imu_protocol as p


class Session:
    """Collects frames and keeps loss counters."""

    def __init__(self, info: p.DeviceInfo, writer: csv.writer | None):
        self.info = info
        self.writer = writer
        self.frames = 0
        self.samples = 0
        self.frame_gaps = 0
        self.sample_gaps = 0
        self.errors: dict[str, int] = {}
        self._last_frame_seq: int | None = None
        self._next_sample_seq: int | None = None

    def on_notify(self, _sender, data: bytearray) -> None:
        try:
            frame = p.decode_frame(bytes(data))
        except p.FrameError as err:
            self.errors[err.reason] = self.errors.get(err.reason, 0) + 1
            return
        if self._last_frame_seq is not None:
            self.frame_gaps += (frame.frame_seq - self._last_frame_seq - 1) & 0xFFFF
        self._last_frame_seq = frame.frame_seq
        if self._next_sample_seq is not None and frame.first_sample_seq != self._next_sample_seq:
            self.sample_gaps += (frame.first_sample_seq - self._next_sample_seq) & 0xFFFFFFFF
        self._next_sample_seq = (frame.first_sample_seq + len(frame.samples)) & 0xFFFFFFFF

        self.frames += 1
        self.samples += len(frame.samples)
        if self.writer is not None:
            for s in frame.samples:
                t = s.seq / self.info.odr_hz
                a = [v * self.info.accel_mg_per_lsb / 1000.0 for v in s.accel]
                g = [v * self.info.gyro_mdps_per_lsb / 1000.0 for v in s.gyro]
                self.writer.writerow([s.seq, f"{t:.4f}", *map(lambda x: f"{x:.5f}", a), *map(lambda x: f"{x:.4f}", g)])


async def run(args: argparse.Namespace) -> int:
    print(f"Scanning for '{args.name}'...")
    device = await BleakScanner.find_device_by_name(args.name, timeout=10.0)
    if device is None:
        print("Device not found", file=sys.stderr)
        return 1

    csv_file = open(args.csv, "w", newline="") if args.csv else None
    try:
        async with BleakClient(device) as client:
            info = p.decode_info(bytes(await client.read_gatt_char(p.INFO_UUID)))
            print(f"Connected: {info.odr_hz} Hz, accel {info.accel_mg_per_lsb} mg/LSB, "
                  f"gyro {info.gyro_mdps_per_lsb} mdps/LSB")
            writer = None
            if csv_file is not None:
                writer = csv.writer(csv_file)
                writer.writerow(["seq", "t_s", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"])
            session = Session(info, writer)

            await client.start_notify(p.DATA_UUID, session.on_notify)
            await client.write_gatt_char(p.CONTROL_UUID, p.CMD_START, response=True)
            started = time.monotonic()
            await asyncio.sleep(args.duration)
            await client.write_gatt_char(p.CONTROL_UUID, p.CMD_STOP, response=True)
            await client.stop_notify(p.DATA_UUID)
            elapsed = time.monotonic() - started
    finally:
        if csv_file is not None:
            csv_file.close()

    rate = session.samples / elapsed if elapsed > 0 else 0.0
    print(f"{session.samples} samples in {elapsed:.1f}s ({rate:.1f} Hz, expected {info.odr_hz})")
    print(f"frames={session.frames} frame_gaps={session.frame_gaps} sample_gaps={session.sample_gaps} "
          f"errors={session.errors or 0}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default="ESP32S3-IMU", help="advertised device name")
    parser.add_argument("--duration", type=float, default=10.0, help="seconds to record")
    parser.add_argument("--csv", help="write converted samples (g, dps) to this file")
    return asyncio.run(run(parser.parse_args()))


if __name__ == "__main__":
    sys.exit(main())