#!/usr/bin/env python3
"""Records StampFly telemetry to CSV.

Works with the drone connected over USB (bench / tether) and with the joystick
connected over USB (real flights: the joystick forwards ESP-NOW telemetry).

Examples (Windows 11):
    py tools/telemetry_logger.py --list
    py tools/telemetry_logger.py --port COM5
    py tools/telemetry_logger.py --port COM5 --duration 60 --raw
    py tools/telemetry_logger.py --from-raw logs/20261014_120000.bin

Output: logs/YYYYmmdd_HHMMSS.csv (+ .bin with raw bytes when --raw is given).
Stop with Ctrl+C.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import os
import sys
import time
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from stampfly_telemetry import TelemetryDecoder  # noqa: E402

ESPRESSIF_USB_VID = 0x303A  # ESP32-S3 native USB (StampS3, AtomS3)


class CsvSink:
    """Writes rows to CSV; starts a new file if the schema (column set) changes."""

    def __init__(self, base_path: str) -> None:
        self.base_path = base_path
        self.part = 0
        self.columns: Optional[list] = None
        self.file = None
        self.writer = None
        self.paths: list = []

    def write(self, row: dict) -> None:
        cols = list(row.keys())
        if cols != self.columns:
            self._open(cols)
        self.writer.writerow([row[c] for c in cols])

    def _open(self, cols: list) -> None:
        self.close()
        suffix = "" if self.part == 0 else "_part%d" % self.part
        path = self.base_path + suffix + ".csv"
        self.part += 1
        self.file = open(path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(cols)
        self.columns = cols
        self.paths.append(path)

    def close(self) -> None:
        if self.file:
            self.file.close()
            self.file = None


def list_ports() -> None:
    from serial.tools import list_ports as lp

    for p in lp.comports():
        tag = "  <- ESP32-S3 (StampS3/AtomS3)" if p.vid == ESPRESSIF_USB_VID else ""
        print("%-10s %s%s" % (p.device, p.description, tag))


def auto_port() -> Optional[str]:
    from serial.tools import list_ports as lp

    found = [p.device for p in lp.comports() if p.vid == ESPRESSIF_USB_VID]
    return found[0] if len(found) == 1 else None


def status_line(dec: TelemetryDecoder, row: Optional[dict], elapsed: float) -> str:
    parts = ["t=%6.1fs rows=%d lost=%d bad=%d" % (elapsed, dec.rows, dec.lost, dec.parser.bad_checksums)]
    if row:
        for key in ("mode", "alt_est_m", "vbat_v", "pos_x_m", "pos_y_m", "flow_squal"):
            if key in row:
                parts.append("%s=%.2f" % (key, row[key]))
    return "  ".join(parts)


def run_serial(args: argparse.Namespace, base: str) -> int:
    import serial

    port = args.port or auto_port()
    if not port:
        print("No port given and auto-detection failed. Use --list and --port COMx.", file=sys.stderr)
        return 2
    dec = TelemetryDecoder()
    sink = CsvSink(base)
    raw = open(base + ".bin", "wb") if args.raw else None
    print("Logging %s -> %s.csv (Ctrl+C to stop)" % (port, base))
    start = time.monotonic()
    last_print = 0.0
    last_row = None
    try:
        with serial.Serial(port, args.baud, timeout=0.1) as ser:
            while True:
                chunk = ser.read(4096)
                if raw and chunk:
                    raw.write(chunk)
                for row in dec.feed(chunk):
                    sink.write(row)
                    last_row = row
                now = time.monotonic() - start
                if now - last_print >= 1.0:
                    print(status_line(dec, last_row, now))
                    last_print = now
                if args.duration and now >= args.duration:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        sink.close()
        if raw:
            raw.close()
    print("Saved: %s" % ", ".join(sink.paths) if sink.paths else "No telemetry rows received.")
    return 0 if sink.paths else 1


def run_from_raw(args: argparse.Namespace) -> int:
    dec = TelemetryDecoder()
    base = os.path.splitext(args.from_raw)[0]
    sink = CsvSink(base)
    with open(args.from_raw, "rb") as f:
        for row in dec.feed(f.read()):
            sink.write(row)
    sink.close()
    print("Decoded %d rows (lost %d, bad checksums %d) -> %s"
          % (dec.rows, dec.lost, dec.parser.bad_checksums, ", ".join(sink.paths) or "-"))
    return 0 if sink.paths else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port, e.g. COM5 (default: auto-detect ESP32-S3)")
    ap.add_argument("--baud", type=int, default=115200, help="ignored by USB CDC, kept for completeness")
    ap.add_argument("--out", default="logs", help="output directory (default: logs)")
    ap.add_argument("--duration", type=float, default=0, help="stop after N seconds (0 = until Ctrl+C)")
    ap.add_argument("--raw", action="store_true", help="also save raw bytes (.bin) for later re-decoding")
    ap.add_argument("--from-raw", help="decode a previously saved .bin file instead of a serial port")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    args = ap.parse_args()

    if args.list:
        list_ports()
        return 0
    if args.from_raw:
        return run_from_raw(args)
    os.makedirs(args.out, exist_ok=True)
    base = os.path.join(args.out, dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
    return run_serial(args, base)


if __name__ == "__main__":
    sys.exit(main())
