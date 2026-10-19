// SPDX-License-Identifier: MIT
//
// [POSHOLD] Soft altitude hold on the barometer height, for flights above the ToF range
// (docs/12-lagodny-regulator-wysokosci.md).
//
// The thrust is the hover trim (from the battery voltage, computed by the upstream code) plus a small, slow
// correction u: a dead zone around the setpoint (the barometer is only good to ~0.7 m), a weak PI action
// (the trim is a few percent off) and damping from the filtered barometer derivative. The authority is
// limited to +-umax of the thrust duty. No fast velocity loop: the sensor sits in the prop wash and lags by
// 0.3 s, so a fast loop oscillates (docs/11-stable-abs.md).
//
// Hardware independent, unit tested (test/test_soft_alt); gains from tools/soft_altitude_sim.py.

#ifndef POSHOLD_SOFT_ALT_HPP
#define POSHOLD_SOFT_ALT_HPP

#include "poshold_config.hpp"

namespace poshold {

struct SoftAltParams {
    float kp       = config::kSoftKp;        // duty per metre outside the dead zone
    float ki       = config::kSoftKi;        // duty per metre and second
    float kd       = config::kSoftKd;        // duty per m/s
    float deadband = config::kSoftDeadband;  // [m]
    float umax     = config::kSoftUMax;      // [duty]
    float tau_v    = config::kSoftTauV;      // [s] low-pass of the derivative
};

class SoftAlt {
public:
    explicit SoftAlt(const SoftAltParams& p = SoftAltParams()) : p_(p) {}

    // Bumpless entry: u_init is the offset from the trim that the previous controller was commanding.
    void reset(float h, float u_init);

    // h: height from the barometer [m], ref: setpoint [m]. Returns u, the thrust duty offset from the trim.
    float update(float dt, float h, float ref);

    float u() const { return u_; }

private:
    SoftAltParams p_;
    bool init_   = false;
    float h_prev_ = 0.0f;
    float vf_     = 0.0f;
    float i_u_    = 0.0f;
    float u_      = 0.0f;
};

}  // namespace poshold

#endif
