// SPDX-License-Identifier: MIT
//
// See tof_step.hpp.

#include "tof_step.hpp"

#include <math.h>

namespace poshold {

float TofStepComp::update(bool active, bool tof_valid, float tof_m, bool detect) {
    if (!active) {
        offset_     = 0.0f;
        prev_d_     = 0.0f;
        prev_valid_ = false;
        saturated_  = false;
        return 0.0f;
    }
    if (!tof_valid) return offset_;  // keep the offset, wait for a real sample
    if (!detect) {
        prev_m_     = tof_m;
        prev_valid_ = true;
        prev_d_     = 0.0f;
        return check_height(tof_m);
    }
    if (prev_valid_) {
        const float d = tof_m - prev_m_;
        float edge    = 0.0f;
        if (fabsf(d) > p_.step_m) {
            edge    = d;
            prev_d_ = 0.0f;
        } else if (fabsf(d) > p_.small_m && fabsf(prev_d_) > p_.small_m && (d > 0.0f) == (prev_d_ > 0.0f) &&
                   fabsf(d + prev_d_) > p_.pair_m) {
            // the beam slides over an edge in two steps: the first one went through unnoticed
            edge    = d + prev_d_;
            prev_d_ = 0.0f;
        } else {
            prev_d_ = d;
        }
        if (edge != 0.0f) {
            offset_ -= edge;
            if (offset_ > p_.max_offset_m) {
                offset_    = p_.max_offset_m;
                saturated_ = true;
            } else if (offset_ < -p_.max_offset_m) {
                offset_    = -p_.max_offset_m;
                saturated_ = true;
            } else {
                saturated_ = false;
            }
        }
    }
    prev_m_     = tof_m;
    prev_valid_ = true;
    return check_height(tof_m);
}

// The height above the take-off point cannot be negative: a compensated height below the minimum means that
// noisy edges (a hand-held card pulled out under the sensor, logs/20261019_233504.csv) have piled up a wrong
// offset. Fall back to the plain ToF height.
float TofStepComp::check_height(float tof_m) {
    if (tof_m + offset_ < p_.min_height_m) {
        offset_    = 0.0f;
        saturated_ = true;
    }
    return offset_;
}

}  // namespace poshold
