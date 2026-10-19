// SPDX-License-Identifier: MIT
//
// [POSHOLD] Obstacle edge compensation for the bottom ToF: the height stays "absolute" (above the
// take-off point) when the drone flies over a bed or a table.
//
// The ToF delivers a sample every ~33 ms and the drone cannot change its height by more than a few
// centimetres in that time, so a jump larger than kTofStepM between two consecutive samples is an
// obstacle edge, not a climb. On such a jump the offset absorbs the step, which keeps the height
// continuous; flying back off the obstacle brings the offset back. A single wrong sample (jump out
// and back) cancels itself. Hardware independent, unit tested (test/test_tof_step).

#ifndef POSHOLD_TOF_STEP_HPP
#define POSHOLD_TOF_STEP_HPP

#include "poshold_config.hpp"

namespace poshold {

struct TofStepParams {
    float step_m       = config::kTofStepM;
    float pair_m       = config::kTofStepPairM;
    float small_m      = config::kTofStepSmallM;
    float max_offset_m = config::kTofStepMaxOffset;
};

class TofStepComp {
public:
    explicit TofStepComp(const TofStepParams& p = TofStepParams()) : p_(p) {}

    // One new ToF sample [m] (after the upstream outlier filter). active = false (on the ground, landing)
    // clears the offset. Returns the offset to add to the ToF height [m].
    // detect = false (e.g. the drone is tilted: the slant range jumps without an edge) keeps the offset and
    // follows the samples without looking for edges.
    float update(bool active, bool tof_valid, float tof_m, bool detect = true);

    float offset() const { return offset_; }
    bool saturated() const { return saturated_; }  // the offset hit its limit: the ToF is not trustworthy

private:
    TofStepParams p_;
    bool prev_valid_ = false;
    float prev_m_    = 0.0f;
    float prev_d_    = 0.0f;  // last jump that was not taken for an edge (pair rule)
    float offset_    = 0.0f;
    bool saturated_  = false;
};

}  // namespace poshold

#endif
