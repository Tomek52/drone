"""Tests for the phone telemetry page (joystick/web/telemetry.html).

- the embedded C++ header must match the page (run tools/gen_phone_page.py after editing the page);
- the page's JavaScript decoder must give the same rows as tools/stampfly_telemetry.py (needs a headless
  Chrome/Edge; skipped when none is installed).

Run: py -m unittest discover -s tools -v
"""

import html
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(__file__))

import gen_phone_page as gen  # noqa: E402
import stampfly_telemetry as t  # noqa: E402


def find_browser():
    for name in ("google-chrome", "google-chrome-stable", "chromium", "chromium-browser", "chrome", "msedge"):
        path = shutil.which(name)
        if path:
            return path
    for path in (
        r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
        r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
        r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    ):
        if os.path.exists(path):
            return path
    return None


class HeaderTests(unittest.TestCase):
    def test_header_matches_page(self):
        with open(gen.DST, "r", encoding="utf-8", newline="") as f:
            current = f.read().replace("\r\n", "\n")
        self.assertEqual(current, gen.render_header(gen.read_page()), "run py tools/gen_phone_page.py")


class JsDecoderTests(unittest.TestCase):
    def test_js_decoder_matches_python(self):
        browser = find_browser()
        if browser is None:
            self.skipTest("no Chrome/Edge found")
        names = ["mode", "alt_est_m", "vbat_filt_v", "alt_ref_m", "alt_tof_m", "x_nan"]
        frames = t.encode_schema(names, max_payload=24)  # several chunks
        self.assertGreater(len(frames), 1)
        rows = [
            (1000, 7, [1.0, 0.5, 3.75, 0.0, 0.0, 1.5]),
            (21000, 8, [2.0, 0.55, 3.74, 1.0, 0.25, float("nan")]),
            (61000, 10, [2.0, 0.6, 3.7, 1.0, 0.5, -2.5]),  # seq 9 lost
            (4294967295, 65535, [3.0, 1.0, 3.6, 1.0, 1.0, 0.125]),
            (5, 0, [3.0, 1.0, 3.6, 1.0, 1.0, 0.125]),  # wrap of seq (no loss)
        ]
        stream = list(frames)
        for time_us, seq, values in rows:
            stream.append(t.encode_data(time_us, seq, values))
        bad = bytearray(t.encode_data(9, 9, [0.0] * 6))
        bad[-1] ^= 0xFF  # broken checksum: must be counted and skipped
        stream.append(bytes(bad))

        # reference: the PC decoder
        dec = t.TelemetryDecoder()
        ref = []
        for fr in stream:
            ref.extend(dec.feed(fr))
        self.assertEqual(len(ref), len(rows))

        with open(gen.SRC, "r", encoding="utf-8") as f:
            page = f.read()
        inject = "<script>window.SELFTEST_FRAMES=%s;</script>\n" % json.dumps([f.hex() for f in stream])
        page = page.replace("<script>\n\"use strict\";", inject + "<script>\n\"use strict\";", 1)
        self.assertIn("SELFTEST_FRAMES=", page)
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "page.html")
            with open(src, "w", encoding="utf-8") as f:
                f.write(page)
            cmd = [browser, "--headless=new", "--disable-gpu", "--no-sandbox", "--user-data-dir=" + os.path.join(tmp, "p"),
                   "--virtual-time-budget=3000", "--dump-dom", "file:///" + src.replace("\\", "/")]
            out = subprocess.run(cmd, capture_output=True, timeout=60).stdout.decode("utf-8", "replace")
        start = out.find('<pre id="selftest">')
        self.assertGreaterEqual(start, 0, out[-500:])
        body = out[start + len('<pre id="selftest">'):out.find("</pre>", start)]
        res = json.loads(html.unescape(body))

        self.assertEqual(res["rows"], len(rows))
        self.assertEqual(res["lost"], dec.lost)
        self.assertEqual(res["bad"], 1)
        self.assertEqual(len(res["parts"]), 1)
        part = res["parts"][0]
        self.assertEqual(part["head"], "time_s,seq," + ",".join(names))
        self.assertEqual(len(part["lines"]), len(ref))
        for line, row in zip(part["lines"], ref):
            cells = line.split(",")
            self.assertEqual(len(cells), 2 + len(names))
            self.assertAlmostEqual(float(cells[0]), row["time_s"], places=9)
            self.assertEqual(int(cells[1]), row["seq"])
            for name, cell in zip(names, cells[2:]):
                a, b = float(cell), row[name]
                self.assertTrue((math.isnan(a) and math.isnan(b)) or a == b, "%s: %s != %s" % (name, cell, b))


if __name__ == "__main__":
    unittest.main()
