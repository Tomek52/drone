// SPDX-License-Identifier: MIT
//
// See soft_alt.hpp.

#include "soft_alt.hpp"

namespace poshold {

namespace {

float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

float deadzone(float e, float db) {
    if (e > db) return e - db;
    if (e < -db) return e + db;
    return 0.0f;
}

}  // namespace

void SoftAlt::reset(float h, float u_init) {
    (void)h;
    init_ = true;
    i_u_  = clampf(u_init, -p_.umax, p_.umax);
    u_    = i_u_;
}

float SoftAlt::update(float dt, float h, float ref, float v) {
    if (!init_) reset(h, 0.0f);
    const float ed = deadzone(ref - h, p_.deadband);
    i_u_           = clampf(i_u_ + p_.ki * ed * dt, -p_.umax, p_.umax);
    u_             = clampf(p_.kp * ed + i_u_ - p_.kd * v, -p_.umax, p_.umax);
    return u_;
}

}  // namespace poshold
