// SPDX-License-Identifier: MIT
//
// See abs_height.hpp.

#include "abs_height.hpp"

#include <math.h>

namespace poshold {

float AbsHeight::update(float dt, bool baro_ok, float baro_h_m, float thrust, bool tof_valid, float tof_m,
                        bool parked) {
    if (!baro_ok) {
        abs_ = tof_valid ? tof_m : abs_;
        return abs_;
    }
    if (!init_) {
        baro_f_ = baro_h_m;
        init_   = true;
    } else {
        baro_f_ += (baro_h_m - baro_f_) * (dt / (p_.tau_baro_s + dt));
    }
    const float t     = thrust < 0.0f ? 0.0f : (thrust > 1.0f ? 1.0f : thrust);
    const float wash  = p_.wash_pa * t * t / p_.pa_per_m;  // apparent height added by the prop wash [m]
    const float basis = baro_f_ - wash;

    if (parked) {
        // On the ground: the ToF is the reference.
        if (tof_valid) off_ = tof_m - basis;
    } else if (tof_valid && tof_m < p_.learn_max) {
        const float innov = tof_m - (basis + off_);
        if (fabsf(innov) < p_.learn_gate) off_ += innov * (dt / (p_.tau_drift_s + dt));
    }
    abs_ = basis + off_;
    return abs_;
}

}  // namespace poshold
