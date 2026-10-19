// SPDX-License-Identifier: MIT
//
// [POSHOLD] Height above the take-off point from the barometer, for the Stable ABS mode.
//
// abs = low-passed barometer height - prop wash correction (quadratic in the thrust duty)
//       + a slowly learned offset. The offset follows the bottom ToF only while the two agree
//       within kAbsLearnGate, so obstacles under the drone (bed, table) and a blinded ToF do
//       not move the height; the barometer's thermal drift (~1.2 Pa/K, ~10 cm/K) and weather
//       are tracked. On the ground (parked) the height is taken from the ToF.
//
// Hardware independent, unit tested (test/test_abs_height).

#ifndef POSHOLD_ABS_HEIGHT_HPP
#define POSHOLD_ABS_HEIGHT_HPP

#include "poshold_config.hpp"

namespace poshold {

struct AbsHeightParams {
    float pa_per_m    = config::kAbsPaPerMeter;
    float wash_pa     = config::kAbsWashPa;
    float tau_baro_s  = config::kAbsTauBaro;
    float tau_drift_s = config::kAbsTauDrift;
    float tau_fast_s  = config::kAbsTauDriftFast;
    float learn_gate  = config::kAbsLearnGate;
    float learn_max   = config::kAbsLearnMaxToF;
};

class AbsHeight {
public:
    explicit AbsHeight(const AbsHeightParams& p = AbsHeightParams()) : p_(p) {}

    // baro_h_m: barometer height relative to the ground reference (raw); thrust: duty 0..1.
    // parked: the drone stands on the ground. Returns the height [m].
    // fast_learn: the ToF is the height source at the moment, so the offset follows it quickly.
    float update(float dt, bool baro_ok, float baro_h_m, float thrust, bool tof_valid, float tof_m, bool parked,
                 bool fast_learn = false);

    float height() const { return abs_; }
    float offset() const { return off_; }

private:
    AbsHeightParams p_;
    bool init_    = false;
    float baro_f_ = 0.0f;
    float off_    = 0.0f;
    float abs_    = 0.0f;
};

}  // namespace poshold

#endif
