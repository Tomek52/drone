"""Tests for flow_calibration.py (run: py -m unittest discover -s tools -v)."""

import csv
import os
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))


def write_carry_log(path, rpc=0.025, h=0.4):
    """Synthetic hand-carry log: 1.0 m forward (t=3..5.5 s), 1.0 m right (t=9..11 s).

    The simulated sensor is mounted rotated: sensor_y = -body_x, sensor_x = body_y.
    """
    sx = sy = 0.0
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "flow_sum_x", "flow_sum_y", "height_m"])
        for k in range(800):
            t = k * 0.02
            v_fwd = 0.4 if 3 <= t < 5.5 else 0.0
            v_right = 0.5 if 9 <= t < 11 else 0.0
            bx = v_fwd / h * 0.02 / rpc
            by = v_right / h * 0.02 / rpc
            sx += by
            sy += -bx
            w.writerow([100 + t, round(sx), round(sy), h])


class FlowCalibrationTests(unittest.TestCase):
    def test_detects_swap_sign_and_scale(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "carry.csv")
            write_carry_log(path)
            out = subprocess.run(
                [sys.executable, os.path.join(HERE, "flow_calibration.py"), path,
                 "--forward", "2.5", "6", "1.0", "--right", "8.5", "11.5", "1.0"],
                capture_output=True, text=True, check=True).stdout
        self.assertIn("kFlowSwapXY       = true", out)
        self.assertIn("kFlowSignX       = -1.0f", out)
        self.assertIn("kFlowSignY       = 1.0f", out)
        self.assertIn("kFlowRadPerCount = 0.02500f", out)


if __name__ == "__main__":
    unittest.main()
