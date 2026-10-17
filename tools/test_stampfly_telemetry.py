"""Tests for stampfly_telemetry.py (run: py -m unittest discover -s tools -v).

Golden vectors are shared with drone/test/test_telemetry_frame/test_main.cpp.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))

import stampfly_telemetry as t  # noqa: E402

GOLDEN_DATA = bytes([0xA5, 0x5A, 0x02, 0x0E, 0xE8, 0x03, 0x00, 0x00, 0x07, 0x00,
                     0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x20, 0xC0, 0xA3, 0x5D])
GOLDEN_SCHEMA = bytes([0xA5, 0x5A, 0x01, 0x08, 0x01, 0x02, 0x00, 0x02,
                       0x61, 0x2C, 0x62, 0x63, 0x61, 0xA4])


class FrameTests(unittest.TestCase):
    def test_fletcher16_reference(self):
        self.assertEqual(t.fletcher16(b"abcde"), 0xC8F0)

    def test_encoders_match_golden(self):
        self.assertEqual(t.encode_data(1000, 7, [1.0, -2.5]), GOLDEN_DATA)
        self.assertEqual(t.encode_schema(["a", "bc"]), [GOLDEN_SCHEMA])

    def test_parser_skips_noise_and_split_chunks(self):
        stream = b"Start StampFly!\r\n" + GOLDEN_SCHEMA + b"\xA5garbage" + GOLDEN_DATA
        p = t.FrameParser()
        frames = []
        for i in range(0, len(stream), 3):  # feed in small chunks
            frames.extend(p.feed(stream[i : i + 3]))
        self.assertEqual(len(frames), 2)
        self.assertIsInstance(frames[0], t.SchemaChunk)
        self.assertEqual(frames[0].names, ["a", "bc"])
        self.assertEqual(frames[1].time_us, 1000)
        self.assertEqual(frames[1].seq, 7)
        self.assertEqual(frames[1].values, [1.0, -2.5])

    def test_parser_rejects_bad_checksum(self):
        bad = bytearray(GOLDEN_DATA)
        bad[12] ^= 0xFF
        p = t.FrameParser()
        self.assertEqual(list(p.feed(bytes(bad))), [])
        self.assertEqual(p.bad_checksums, 1)

    def test_decoder_rows_and_lost_frames(self):
        d = t.TelemetryDecoder()
        data = b"".join(t.encode_schema(["x", "y"])) + t.encode_data(10, 1, [1, 2]) + t.encode_data(20, 4, [3, 4])
        rows = list(d.feed(data))
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[1]["x"], 3.0)
        self.assertAlmostEqual(rows[1]["time_s"], 20e-6)
        self.assertEqual(d.lost, 2)

    def test_decoder_assembles_chunked_schema(self):
        names = ["field_name_%04d" % i for i in range(40)]
        chunks = t.encode_schema(names)
        self.assertEqual(len(chunks), 3)
        d = t.TelemetryDecoder()
        data = b"noise" + b"".join(chunks) + t.encode_data(5, 0, [float(i) for i in range(40)])
        rows = list(d.feed(data))
        self.assertEqual(d.schema.names, names)
        self.assertEqual(rows[0]["field_name_0039"], 39.0)

    def test_decoder_waits_for_schema(self):
        d = t.TelemetryDecoder()
        self.assertEqual(list(d.feed(t.encode_data(10, 1, [1, 2]))), [])


if __name__ == "__main__":
    unittest.main()
