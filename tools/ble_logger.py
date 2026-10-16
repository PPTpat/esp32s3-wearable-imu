"""Unit tests for imu_protocol (run: python -m unittest discover -s tools)."""
import struct
import unittest

import imu_protocol as p

# Same bytes as kGolden in test/host/test_frame.cpp, built independently of the firmware.
GOLDEN = bytes.fromhex("010102016400000002000100feff0300fcff0500faff000100ffff7f008000000700f710")


class ProtocolTests(unittest.TestCase):
    def test_crc_check_value(self):
        self.assertEqual(p.crc16_ccitt(b"123456789"), 0x29B1)

    def test_decode_golden_frame(self):
        frame = p.decode_frame(GOLDEN)
        self.assertEqual(frame.frame_seq, 0x0102)
        self.assertEqual(frame.first_sample_seq, 100)
        self.assertEqual(len(frame.samples), 2)
        self.assertEqual(frame.samples[0].accel, (1, -2, 3))
        self.assertEqual(frame.samples[0].gyro, (-4, 5, -6))
        self.assertEqual(frame.samples[1].seq, 101)
        self.assertEqual(frame.samples[1].gyro[0], -32768)

    def test_corruption_is_detected(self):
        bad = bytearray(GOLDEN)
        bad[14] ^= 0x01
        with self.assertRaises(p.FrameError) as ctx:
            p.decode_frame(bytes(bad))
        self.assertEqual(ctx.exception.reason, "bad_crc")

    def test_too_short(self):
        with self.assertRaises(p.FrameError) as ctx:
            p.decode_frame(GOLDEN[:5])
        self.assertEqual(ctx.exception.reason, "too_short")

    def test_wrong_version_with_valid_crc(self):
        body = bytearray(GOLDEN[:-2])
        body[0] = 2
        frame = bytes(body) + struct.pack("<H", p.crc16_ccitt(bytes(body)))
        with self.assertRaises(p.FrameError) as ctx:
            p.decode_frame(frame)
        self.assertEqual(ctx.exception.reason, "bad_version")

    def test_decode_info(self):
        data = struct.pack("<BBHff", 1, 0, 104, 0.122, 17.5)
        info = p.decode_info(data)
        self.assertEqual(info.odr_hz, 104)
        self.assertAlmostEqual(info.accel_mg_per_lsb, 0.122, places=5)
        self.assertAlmostEqual(info.gyro_mdps_per_lsb, 17.5, places=5)


if __name__ == "__main__":
    unittest.main()