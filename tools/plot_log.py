#!/usr/bin/env python3
"""Plots a StampFly telemetry CSV for controller tuning.

Examples (Windows 11):
    py tools/plot_log.py logs/20261014_120000.csv            # saves PNG next to the CSV
    py tools/plot_log.py logs/20261014_120000.csv --show     # also opens a window
    py tools/plot_log.py logs/x.csv --groups attitude altitude --t0 5 --t1 25

Each panel has a single y axis; the measured value is solid blue and its
reference/setpoint is dashed orange (raw sensor data: thin aqua), so the same pairing reads the same way in
every panel. Panels whose columns are missing in the log are skipped, so the
script works with logs from any firmware branch.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from typing import Dict, List, Tuple

# Fixed categorical order (validated palette): blue, orange, aqua, yellow.
SERIES_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
TEXT_SECONDARY = "#5f5e5a"
GRID = "#e4e3dd"

# group -> list of panels; panel = (title, unit, [(column, label, style_index, dashed)])
Panel = Tuple[str, str, List[Tuple[str, str, int, bool]]]
GROUPS: Dict[str, List[Panel]] = {
    "attitude": [
        ("Roll", "deg", [("roll_deg", "measured", 0, False), ("roll_ref_deg", "reference", 1, True)]),
        ("Pitch", "deg", [("pitch_deg", "measured", 0, False), ("pitch_ref_deg", "reference", 1, True)]),
        ("Yaw rate", "deg/s", [("r_dps", "measured", 0, False), ("r_ref_dps", "reference", 1, True)]),
    ],
    "rates": [
        ("Roll rate p", "deg/s", [("p_dps", "measured", 0, False), ("p_ref_dps", "reference", 1, True)]),
        ("Pitch rate q", "deg/s", [("q_dps", "measured", 0, False), ("q_ref_dps", "reference", 1, True)]),
    ],
    "altitude": [
        ("Altitude", "m", [("alt_est_m", "estimate", 0, False), ("alt_ref_m", "reference", 1, True),
                           ("alt_tof_m", "ToF (filtered)", 2, False)]),
        ("Vertical speed", "m/s", [("vz_mps", "estimate", 0, False), ("vz_ref_mps", "reference", 1, True)]),
        ("Thrust", "duty", [("thrust_duty", "thrust", 0, False)]),
    ],
    "position": [
        ("Position X (forward)", "m", [("pos_x_m", "estimate", 0, False), ("pos_sp_x_m", "setpoint", 1, True)]),
        ("Position Y (right)", "m", [("pos_y_m", "estimate", 0, False), ("pos_sp_y_m", "setpoint", 1, True)]),
    ],
    "velocity": [
        ("Velocity X", "m/s", [("vel_x_mps", "estimate", 0, False), ("vel_sp_x_mps", "setpoint", 1, True),
                               ("flow_vx_mps", "optical flow", 2, False)]),
        ("Velocity Y", "m/s", [("vel_y_mps", "estimate", 0, False), ("vel_sp_y_mps", "setpoint", 1, True),
                               ("flow_vy_mps", "optical flow", 2, False)]),
    ],
    "flow": [
        ("Optical flow quality (SQUAL)", "-", [("flow_squal", "SQUAL", 0, False)]),
        ("Flow valid", "0/1", [("flow_valid", "valid", 0, False)]),
        ("Flow reject reason (0 = ok)", "code", [("flow_reject", "reason", 0, False)]),
        ("Height above floor (tilt compensated)", "m", [("height_m", "height", 0, False)]),
        ("Flow raw counts", "counts", [("flow_dx", "dx", 0, False), ("flow_dy", "dy", 1, False)]),
    ],
    "battery": [
        ("Battery voltage", "V", [("vbat_v", "voltage", 0, False)]),
        ("Battery current", "A", [("ibat_a", "current", 0, False)]),
    ],
    "sticks": [
        ("Sticks", "-", [("stick_thr", "throttle", 0, False), ("stick_roll", "roll", 1, False),
                         ("stick_pitch", "pitch", 2, False), ("stick_yaw", "yaw", 3, False)]),
    ],
    "timing": [
        ("Loop period", "s", [("loop_dt_s", "loop dt", 0, False)]),
        ("Radio packet age", "loops", [("rc_age_loops", "age", 0, False)]),
    ],
}


def load_csv(path: str) -> Dict[str, List[float]]:
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = next(reader)
        cols: Dict[str, List[float]] = {h: [] for h in header}
        for row in reader:
            for h, v in zip(header, row):
                try:
                    cols[h].append(float(v))
                except ValueError:
                    cols[h].append(float("nan"))
    return cols


def select_panels(cols: Dict[str, List[float]], groups: List[str]) -> List[Panel]:
    panels = []
    for g in groups:
        for title, unit, series in GROUPS[g]:
            present = [s for s in series if s[0] in cols]
            if present:
                panels.append((title, unit, present))
    return panels


def plot(cols: Dict[str, List[float]], panels: List[Panel], title: str, t0: float, t1: float):
    import matplotlib.pyplot as plt

    t = cols["time_s"]
    start = t[0] if t else 0.0
    rel = [x - start for x in t]
    idx = [i for i, x in enumerate(rel) if x >= t0 and (t1 <= 0 or x <= t1)]
    fig, axes = plt.subplots(len(panels), 1, sharex=True, figsize=(11, 2.4 * len(panels) + 0.6), squeeze=False)
    for ax, (ptitle, unit, series) in zip(axes[:, 0], panels):
        for col, label, ci, dashed in series:
            raw = ci == 2  # raw sensor series: thinner and drawn under the estimate
            ax.plot([rel[i] for i in idx], [cols[col][i] for i in idx], color=SERIES_COLORS[ci],
                    linewidth=1.0 if raw else (1.4 if dashed else 1.6), linestyle="--" if dashed else "-",
                    label=label, zorder=1 if raw else 2)
        ax.set_title(ptitle, loc="left", fontsize=10)
        ax.set_ylabel(unit, color=TEXT_SECONDARY)
        ax.grid(True, color=GRID, linewidth=0.8)
        ax.tick_params(colors=TEXT_SECONDARY)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        # Legend above the plot area, so it never covers data.
        ax.legend(loc="lower right", bbox_to_anchor=(1.0, 1.0), fontsize=8, frameon=False,
                  ncol=len(series), borderaxespad=0.2)
    axes[-1, 0].set_xlabel("time since start of log [s]", color=TEXT_SECONDARY)
    fig.suptitle(title, x=0.01, ha="left", fontsize=11)
    fig.tight_layout()
    return fig


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="telemetry CSV from telemetry_logger.py")
    ap.add_argument("--groups", nargs="+", choices=sorted(GROUPS), default=list(GROUPS),
                    help="panel groups to plot (default: all available)")
    ap.add_argument("--t0", type=float, default=0.0, help="start time [s] relative to the log start")
    ap.add_argument("--t1", type=float, default=0.0, help="end time [s] (0 = end of log)")
    ap.add_argument("--out", help="PNG path (default: next to the CSV)")
    ap.add_argument("--show", action="store_true", help="open an interactive window")
    args = ap.parse_args()

    if not args.show:
        import matplotlib

        matplotlib.use("Agg")
    cols = load_csv(args.csv)
    if "time_s" not in cols or not cols["time_s"]:
        print("CSV has no time_s column or no rows.", file=sys.stderr)
        return 1
    panels = select_panels(cols, args.groups)
    if not panels:
        print("None of the requested groups has columns in this log.", file=sys.stderr)
        return 1
    fig = plot(cols, panels, os.path.basename(args.csv), args.t0, args.t1)
    out = args.out or os.path.splitext(args.csv)[0] + ".png"
    fig.savefig(out, dpi=110)
    print("Saved %s (%d panels)" % (out, len(panels)))
    if args.show:
        import matplotlib.pyplot as plt

        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
