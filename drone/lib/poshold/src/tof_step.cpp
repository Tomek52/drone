// SPDX-License-Identifier: MIT
//
// See tof_step.hpp.

#include "tof_step.hpp"

#include <math.h>

namespace poshold {

float TofStepComp::update(bool active, bool tof_valid, float tof_m) {
    if (!active) {
        offset_     = 0.0f;
        prev_valid_ = false;
        saturated_  = false;
        return 0.0f;
    }
    if (!tof_valid) return offset_;  // keep the offset, wait for a real sample
    if (prev_valid_) {
        const float d = tof_m - prev_m_;
        if (fabsf(d) > p_.step_m) {
            offset_ -= d;
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
    return offset_;
}

}  // namespace poshold
