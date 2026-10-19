#!/usr/bin/env python3
"""Offline simulation of the soft barometric altitude hold (docs/12-lagodny-regulator-wysokosci.md).

The plant and the sensor are modelled on numbers measured in the flight logs:
  - vertical gain 12-16 m/s^2 per unit of thrust duty (logs/20261019_222945.csv),
  - barometer: single samples ~0.12 m noisy, 0.3 s lag, a prop wash shift of ~1.4 m per unit of thrust change,
    slow drift (thermal, ~1 cm/s worst case) and an offset error after learning (+-0.3 m),
  - the hover trim from the battery voltage is off by a few percent of duty.

Run:  py tools/soft_altitude_sim.py            (prints the scenario table for the chosen gains)
      py tools/soft_altitude_sim.py --search   (grid search of the gains)
"""
import argparse
import itertools
import math
import random
import statistics as st

DT = 0.01            # simulation step [s]
BARO_DT = 0.04       # barometer update period [s] (25 Hz)


class Params:
    """Controller parameters."""

    def __init__(self, kp=0.020, ki=0.0030, kd=0.060, deadband=0.5, umax=0.08, tau_f=1.5, tau_v=2.0, vmax_cmd=0.0):
        self.kp = kp              # duty per metre of error (outside the dead zone)
        self.ki = ki              # duty per metre*second
        self.kd = kd              # duty per m/s
        self.deadband = deadband  # [m]
        self.umax = umax          # authority around the trim [duty]
        self.tau_f = tau_f        # low-pass of the barometer height [s]
        self.tau_v = tau_v        # low-pass of the derivative [s]


class Plant:
    def __init__(self, gain=14.0, drag=0.3, lag=0.10, trim_error=0.0, seed=1):
        self.k = gain
        self.c = drag
        self.lag = lag
        self.trim_error = trim_error  # true hover duty minus the assumed trim
        self.h = 1.0
        self.v = 0.0
        self.thr_act = 0.0            # actual thrust offset after the motor lag
        self.rng = random.Random(seed)

    def step(self, u, wind_acc):
        self.thr_act += (u - self.thr_act) * DT / (self.lag + DT)
        acc = self.k * (self.thr_act - self.trim_error) - self.c * self.v + wind_acc
        self.v += acc * DT
        self.h += self.v * DT


class Baro:
    def __init__(self, noise=0.12, lag=0.3, wash_gain=1.4, drift=0.0, offset=0.0, seed=2):
        self.noise = noise
        self.lag = lag
        self.wash = wash_gain
        self.drift = drift
        self.offset = offset
        self.bias = 0.0
        self.rng = random.Random(seed)
        self.filt = None
        self.t = 0.0
        self.last = 0.0

    def read(self, h_true, thr_offset):
        # slow random walk (weather, thermal) plus a constant drift
        self.bias += self.drift * DT + self.rng.gauss(0, 0.004) * math.sqrt(DT / 0.01) * 0.1
        raw = h_true + self.wash * thr_offset + self.offset + self.bias + self.rng.gauss(0, self.noise)
        a = DT / (self.lag + DT)
        self.filt = raw if self.filt is None else self.filt + (raw - self.filt) * a
        self.t += DT
        if self.t >= BARO_DT:
            self.t = 0.0
            self.last = self.filt
        return self.last


def deadzone(e, db):
    if e > db:
        return e - db
    if e < -db:
        return e + db
    return 0.0


def simulate(p: Params, ref_fn, T=160.0, plant=None, baro=None, wind_fn=None):
    plant = plant or Plant()
    baro = baro or Baro()
    wind_fn = wind_fn or (lambda t: 0.0)
    n = int(T / DT)
    hf = None
    hprev = None
    vf = 0.0
    integ = 0.0
    u = 0.0
    hs, refs, us = [], [], []
    for i in range(n):
        t = i * DT
        ref = ref_fn(t)
        z = baro.read(plant.h, plant.thr_act)
        hf = z if hf is None else hf + (z - hf) * DT / (p.tau_f + DT)
        if hprev is None:
            hprev = hf
        d = (hf - hprev) / DT
        hprev = hf
        vf += (d - vf) * DT / (p.tau_v + DT)
        e = ref - hf
        ed = deadzone(e, p.deadband)
        integ += ed * DT
        integ = max(-p.umax / max(p.ki, 1e-9), min(p.umax / max(p.ki, 1e-9), integ))
        u = p.kp * ed + p.ki * integ - p.kd * vf
        u = max(-p.umax, min(p.umax, u))
        plant.step(u, wind_fn(t))
        hs.append(plant.h)
        refs.append(ref)
        us.append(u)
    return hs, refs, us


SCENARIOS = {
    "hold 5 m, start at 3 m": dict(ref=lambda t: 5.0, h0=3.0),
    "hold 5 m, trim +3%": dict(ref=lambda t: 5.0, h0=5.0, trim_error=0.03),
    "hold 5 m, trim -3%": dict(ref=lambda t: 5.0, h0=5.0, trim_error=-0.03),
    "hold 5 m, baro offset +0.5": dict(ref=lambda t: 5.0, h0=5.0, offset=0.5),
    "hold 5 m, baro drift 1 cm/s": dict(ref=lambda t: 5.0, h0=5.0, drift=0.01),
    "hold 5 m, gusts +-0.3 m/s2": dict(ref=lambda t: 5.0, h0=5.0, gust=0.3),
    "step 5 -> 8 m at 20 s": dict(ref=lambda t: 5.0 if t < 20 else 8.0, h0=5.0),
    "plant gain 10": dict(ref=lambda t: 5.0, h0=5.0, gain=10.0),
    "plant gain 18, lag 0.2": dict(ref=lambda t: 5.0, h0=5.0, gain=18.0, lag=0.2),
}


def run_scenario(p, spec, seed=1):
    plant = Plant(gain=spec.get("gain", 14.0), lag=spec.get("lag", 0.10), trim_error=spec.get("trim_error", 0.0), seed=seed)
    plant.h = spec["h0"]
    baro = Baro(drift=spec.get("drift", 0.0), offset=spec.get("offset", 0.0), seed=seed + 10)
    g = spec.get("gust", 0.0)
    rng = random.Random(seed + 20)
    gust_state = {"v": 0.0, "t": -1.0}

    def wind(t):
        if g == 0.0:
            return 0.0
        k = int(t / 3.0)
        if k != gust_state["t"]:
            gust_state["t"] = k
            gust_state["v"] = rng.uniform(-g, g)
        return gust_state["v"]

    hs, refs, us = simulate(p, spec["ref"], plant=plant, baro=baro, wind_fn=wind)
    # metrics over the last 50 s
    start = int(110 / DT)
    err = [h - r for h, r in zip(hs[start:], refs[start:])]
    peak = max(abs(h - r) for h, r in zip(hs, refs)) if spec["h0"] == spec["ref"](0) else None
    vs = [(hs[i + 1] - hs[i]) / DT for i in range(start, len(hs) - 1)]
    return dict(rms=math.sqrt(sum(e * e for e in err) / len(err)), mx=max(abs(e) for e in err), vrms=math.sqrt(sum(v * v for v in vs) / len(vs)),
                umax=max(abs(x) for x in us[start:]), peak=peak)


def table(p):
    print(f"{'scenario':32s} {'rms err':>8s} {'max err':>8s} {'v rms':>7s} {'max u':>6s}")
    worst = 0.0
    for name, spec in SCENARIOS.items():
        m = run_scenario(p, spec)
        worst = max(worst, m["mx"])
        print(f"{name:32s} {m['rms']:8.2f} {m['mx']:8.2f} {m['vrms']:7.2f} {m['umax']:6.3f}")
    print(f"worst |error| after settling: {worst:.2f} m")
    return worst


def score(p):
    worst = 0.0
    vr = 0.0
    for spec in SCENARIOS.values():
        m = run_scenario(p, spec)
        worst = max(worst, m["mx"])
        vr = max(vr, m["vrms"])
    return worst, vr


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--search", action="store_true")
    for name, default in (("kp", 0.020), ("ki", 0.003), ("kd", 0.060), ("deadband", 0.5), ("umax", 0.08), ("tau_f", 1.5), ("tau_v", 2.0)):
        ap.add_argument("--" + name, type=float, default=default)
    a = ap.parse_args()
    if a.search:
        best = []
        for kp, ki, kd, tf in itertools.product((0.002, 0.004, 0.008, 0.016), (0.0003, 0.001, 0.003), (0.04, 0.08, 0.12), (0.3, 0.5, 1.0)):
            p = Params(kp=kp, ki=ki, kd=kd, deadband=a.deadband, umax=a.umax, tau_f=tf, tau_v=a.tau_v)
            w, v = score(p)
            best.append((w + 2.0 * v, w, v, kp, ki, kd, tf))
        best.sort()
        print("score  worst  vrms   kp    ki     kd    tau_f")
        for s, w, v, kp, ki, kd, tf in best[:10]:
            print(f"{s:5.2f} {w:6.2f} {v:5.2f} {kp:5.3f} {ki:6.4f} {kd:5.3f} {tf:4.1f}")
        return
    table(Params(a.kp, a.ki, a.kd, a.deadband, a.umax, a.tau_f, a.tau_v))


if __name__ == "__main__":
    main()
