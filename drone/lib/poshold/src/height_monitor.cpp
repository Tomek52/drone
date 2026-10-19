// SPDX-License-Identifier: MIT
//
// See height_monitor.hpp.

#include "height_monitor.hpp"

#include <math.h>

namespace poshold {

void HeightMonitor::reset() {
    initialised_ = false;
    trusted_     = true;
    offset_      = 0.0f;
    baro_f_init_ = false;
    bad_time_    = 0.0f;
    good_time_   = 0.0f;
}

HeightMonitorOutput HeightMonitor::update(float dt, bool tof_valid, float tof_m, bool baro_ok, float baro_m) {
    HeightMonitorOutput out;
    if (!baro_ok) {
        // Without a barometer there is nothing to compare with: do not distrust the ToF.
        trusted_ = true;
        out.trusted = true;
        return out;
    }
    if (!baro_f_init_) {
        baro_f_      = baro_m;
        baro_f_init_ = true;
    } else {
        baro_f_ += (baro_m - baro_f_) * (dt / (p_.tau_baro_s + dt));
    }
    baro_m = baro_f_;

    if (!initialised_) {
        if (tof_valid) {
            offset_      = tof_m - baro_m;
            initialised_ = true;
        }
        out.initialised  = initialised_;
        out.predicted_m  = baro_m + offset_;
        out.trusted      = trusted_;
        return out;
    }

    const float predicted = baro_m + offset_;
    const float innov     = tof_valid ? tof_m - predicted : 0.0f;
    const bool agree      = tof_valid && fabsf(innov) < p_.gate_m;

    if (trusted_) {
        if (agree) {
            bad_time_ = 0.0f;
            offset_ += innov * (dt / (p_.tau_offset_s + dt));
        } else {
            bad_time_ += dt;
            if (bad_time_ > p_.hold_s) {
                trusted_   = false;
                good_time_ = 0.0f;
            }
        }
    } else {
        // The offset stays frozen while untrusted (the barometer carries on from there).
        if (tof_valid && fabsf(innov) < p_.regain_gate_m) {
            good_time_ += dt;
            if (good_time_ > p_.regain_s) {
                trusted_  = true;
                bad_time_ = 0.0f;
            }
        } else {
            good_time_ = 0.0f;
        }
    }

    out.initialised  = true;
    out.trusted      = trusted_;
    out.predicted_m  = predicted;
    out.innovation_m = innov;
    return out;
}

}  // namespace poshold
