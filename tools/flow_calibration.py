#!/usr/bin/env python3
"""Optical flow axis mapping and scale calibration from a hand-carry log.

Procedure (docs/08-testy.md, "Kalibracja optical flow"):
  1. Propellers OFF. Drone connected to the PC by USB, telemetry_logger.py running.
  2. Hold the drone level at a constant height (~0.3-0.5 m) above a textured floor.
  3. Move it straight FORWARD by a measured distance (e.g. 1.00 m, use a tape),
     pause, then straight to the RIGHT by a measured distance.
  4. Note the start/end times of both moves on the plot (plot_log.py --groups flow).

Example:
    py tools/flow_calibration.py logs/carry.csv --forward 3.0 6.5 1.0 --right 9.0 12.0 1.0

The script takes the raw sensor counts over each segment (cumulative
counters flow_sum_x/flow_sum_y) and
prints the values for drone/lib/poshold/src/poshold_config.hpp:
kFlowSwapXY, kFlowSignX, kFlowSignY and kFlowRadPerCount.

Rotation is NOT compensated here: keep the drone level and do not rotate it
during the moves (the estimator compensates rotation in flight).
"""

from __future__ import annotations

import argparse
import csv
import sys
from typing import Dict, List, Tuple


def load(path: str) -> Dict[str, List[float]]:
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        header = next(r)
        cols: Dict[str, List[float]] = {h: [] for h in header}
        for row in r:
            for h, v in zip(header, row):
                cols[h].append(float(v))
    return cols


def segment_sums(cols: Dict[str, List[float]], t0: float, t1: float) -> Tuple[float, float, float, int]:
    """Returns (counts x, counts y, mean height, rows) for t0 <= t <= t1 (time relative to log start).

    Uses the cumulative counters flow_sum_x/flow_sum_y, so the result is exact
    even though telemetry (50 Hz) is slower than the flow reads (100 Hz).
    """
    t = cols["time_s"]
    start = t[0]
    idx = [i for i, ti in enumerate(t) if t0 <= ti - start <= t1]
    if not idx:
        return 0.0, 0.0, 0.0, 0
    first, last = idx[0], idx[-1]
    sx = cols["flow_sum_x"][last] - cols["flow_sum_x"][first]
    sy = cols["flow_sum_y"][last] - cols["flow_sum_y"][first]
    h = sum(cols["height_m"][i] for i in idx) / len(idx)
    return sx, sy, h, len(idx)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--forward", nargs=3, type=float, metavar=("T0", "T1", "DIST_M"), required=True,
                    help="time window [s, relative to log start] and distance of the forward move")
    ap.add_argument("--right", nargs=3, type=float, metavar=("T0", "T1", "DIST_M"), required=True,
                    help="time window and distance of the move to the right")
    args = ap.parse_args()

    cols = load(args.csv)
    for need in ("time_s", "flow_sum_x", "flow_sum_y", "height_m"):
        if need not in cols:
            print("Missing column %s - log from firmware without the position estimator?" % need, file=sys.stderr)
            return 1

    fx, fy, fh, fn = segment_sums(cols, args.forward[0], args.forward[1])
    rx, ry, rh, rn = segment_sums(cols, args.right[0], args.right[1])
    print("Forward move: sum dx=%.0f dy=%.0f, mean height %.3f m (%d rows)" % (fx, fy, fh, fn))
    print("Right move:   sum dx=%.0f dy=%.0f, mean height %.3f m (%d rows)" % (rx, ry, rh, rn))
    if fn == 0 or rn == 0 or fh <= 0 or rh <= 0:
        print("Empty segment or no height - check the time windows.", file=sys.stderr)
        return 1

    # Forward motion should show up mostly on one sensor axis -> that is body X.
    swap = abs(fy) > abs(fx)
    fwd_counts = fy if swap else fx
    right_counts = rx if swap else ry
    sign_x = 1.0 if fwd_counts > 0 else -1.0
    sign_y = 1.0 if right_counts > 0 else -1.0

    cross_fwd = abs(fx if swap else fy) / max(abs(fwd_counts), 1.0)
    cross_right = abs(ry if swap else rx) / max(abs(right_counts), 1.0)

    # Translation d at height h: angle = d / h  ->  rad per count = d / (h * counts)
    rpc_x = args.forward[2] / (fh * abs(fwd_counts)) if fwd_counts else float("nan")
    rpc_y = args.right[2] / (rh * abs(right_counts)) if right_counts else float("nan")

    print()
    print("Suggested values for drone/lib/poshold/src/poshold_config.hpp:")
    print("  constexpr bool kFlowSwapXY       = %s;" % ("true" if swap else "false"))
    print("  constexpr float kFlowSignX       = %.1ff;" % sign_x)
    print("  constexpr float kFlowSignY       = %.1ff;" % sign_y)
    print("  constexpr float kFlowRadPerCount = %.5ff;  // X: %.5f, Y: %.5f" % ((rpc_x + rpc_y) / 2, rpc_x, rpc_y))
    print()
    print("Cross-axis leakage: forward %.0f%%, right %.0f%% (should be < 20%%)" % (cross_fwd * 100, cross_right * 100))
    if cross_fwd > 0.2 or cross_right > 0.2:
        print("WARNING: large cross-axis leakage - the moves were not straight or the drone rotated.")
    if abs(rpc_x - rpc_y) > 0.2 * max(rpc_x, rpc_y):
        print("WARNING: X and Y scales differ by more than 20% - repeat the test.")
    print("After verifying, set kFlowAxesVerified = true.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
