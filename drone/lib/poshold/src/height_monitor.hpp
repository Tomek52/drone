// SPDX-License-Identifier: MIT
//
// [POSHOLD] Plausibility check of the bottom ToF height against the barometer.
//
// The ToF is the only height sensor the flight controller trusts. Direct sunlight or a
// torch makes it return wrong values that still carry status "valid" (measured 2026-10-19),
// and upstream keeps the last reading when the sensor returns nothing. This monitor keeps
// an offset between the barometer height and the ToF height while both agree (slow, so the
// barometer's thermal drift of ~1.2 Pa/K is absorbed) and declares the ToF untrusted when
// they disagree for longer than a short hold time.
//
// Hardware independent, unit tested (test/test_height_monitor).

#ifndef POSHOLD_HEIGHT_MONITOR_HPP
#define POSHOLD_HEIGHT_MONITOR_HPP

#include "poshold_config.hpp"

namespace poshold {

struct HeightMonitorParams {
    float gate_m        = config::kHmGate;         // |ToF - predicted| above this is a disagreement
    float hold_s        = config::kHmHold;         // disagreement for this long -> untrusted
    float regain_gate_m = config::kHmRegainGate;   // agreement needed to trust the ToF again ...
    float regain_s      = config::kHmRegain;       // ... for this long
    float tau_offset_s  = config::kHmTauOffset;    // time constant of the offset while trusted
};

struct HeightMonitorOutput {
    bool trusted           = true;   // ToF may be used
    float predicted_m      = 0.0f;   // barometer height + offset (valid once initialised)
    float innovation_m     = 0.0f;   // ToF - predicted (0 without a valid ToF sample)
    bool initialised       = false;
};

class HeightMonitor {
public:
    explicit HeightMonitor(const HeightMonitorParams& p = HeightMonitorParams()) : p_(p) {}

    // Forget everything (e.g. on the ground before take-off).
    void reset();

    // One ToF sample (~30 Hz). tof_valid: status 0 and a plausible range.
    // baro_m: barometer height relative to the ground reference (filtered by the caller or not).
    // baro_ok: the barometer is present and delivers data.
    HeightMonitorOutput update(float dt, bool tof_valid, float tof_m, bool baro_ok, float baro_m);

    bool trusted() const { return trusted_; }

private:
    HeightMonitorParams p_;
    bool initialised_ = false;
    bool trusted_     = true;
    float offset_     = 0.0f;
    float bad_time_   = 0.0f;
    float good_time_  = 0.0f;
};

}  // namespace poshold

#endif
