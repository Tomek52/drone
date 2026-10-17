// SPDX-License-Identifier: MIT
//
// See position_estimator.hpp for the model and frame conventions.

#include "position_estimator.hpp"

#include <math.h>

#include "math_utils.hpp"

namespace poshold {

void PositionEstimator::reset() {
    s_            = EstimatorState();
    p_sum_        = 0.0f;
    q_sum_        = 0.0f;
    rate_samples_ = 0;
    good_count_   = 0;
    bad_count_    = 0;
}

void PositionEstimator::reset_position() {
    s_.px = 0.0f;
    s_.py = 0.0f;
}

void PositionEstimator::predict(float fx, float fy, float fz, float p, float q, float roll, float pitch, float yaw,
                                float dt) {
    if (!(dt > 0.0f)) return;
    roll_  = roll;
    pitch_ = pitch;

    // Accumulate body rates for the gyro compensation of the next flow sample.
    p_sum_ += p;
    q_sum_ += q;
    rate_samples_++;
    last_p_ = p;
    last_q_ = q;

    // Specific force rotated into the heading frame (roll and pitch removed).
    // Gravity has no horizontal component, so this is the horizontal acceleration.
    const float cr = cosf(roll), sr = sinf(roll);
    const float cp = cosf(pitch), sp = sinf(pitch);
    const float g  = p_.gravity;
    s_.acc_x       = g * (cp * fx + sp * sr * fy + sp * cr * fz);
    s_.acc_y       = g * (cr * fy - sr * fz);

    s_.vx += (s_.acc_x - s_.bias_x) * dt;
    s_.vy += (s_.acc_y - s_.bias_y) * dt;

    float wx, wy;
    rotate2d(s_.vx, s_.vy, yaw, &wx, &wy);
    s_.px += wx * dt;
    s_.py += wy * dt;
}

FlowReject PositionEstimator::check_sample(const FlowSample& s, float height, float p_avg, float q_avg) const {
    if (!s.read_ok || !(height > 0.0f)) return FlowReject::kNoData;
    if (height < p_.min_height) return FlowReject::kHeightLow;
    if (height > p_.max_height) return FlowReject::kHeightHigh;
    if (s.squal < p_.min_squal) return FlowReject::kLowQuality;
    if (fabsf(p_avg) > p_.max_body_rate || fabsf(q_avg) > p_.max_body_rate) return FlowReject::kBodyRate;
    if (fabsf(roll_) > p_.max_tilt || fabsf(pitch_) > p_.max_tilt) return FlowReject::kTilt;
    return FlowReject::kNone;
}

void PositionEstimator::update_flow(const FlowSample& s, float range_m, float dt) {
    // Mean body rates over the flow integration interval.
    const float p_avg = rate_samples_ ? p_sum_ / rate_samples_ : last_p_;
    const float q_avg = rate_samples_ ? q_sum_ / rate_samples_ : last_q_;
    p_sum_        = 0.0f;
    q_sum_        = 0.0f;
    rate_samples_ = 0;

    s_.height = range_m > 0.0f ? range_m * cosf(roll_) * cosf(pitch_) : 0.0f;
    s_.squal  = s.squal;

    FlowReject reject = (dt > 0.0f) ? check_sample(s, s_.height, p_avg, q_avg) : FlowReject::kNoData;

    if (reject == FlowReject::kNone) {
        // Map sensor counts to the body convention (x fwd, y right).
        float cx = p_.flow_swap_xy ? s.dy : s.dx;
        float cy = p_.flow_swap_xy ? s.dx : s.dy;
        cx *= p_.flow_sign_x;
        cy *= p_.flow_sign_y;
        const float wx = cx * p_.flow_rad_per_count / dt;
        const float wy = cy * p_.flow_rad_per_count / dt;
        s_.flow_vx     = s_.height * (wx - q_avg);
        s_.flow_vy     = s_.height * (wy + p_avg);

        // Complementary correction of velocity and accelerometer bias.
        const float ex = s_.flow_vx - s_.vx;
        const float ey = s_.flow_vy - s_.vy;
        const float kv = dt / (p_.vel_tau + dt);
        const float kb = dt / (p_.bias_tau * p_.vel_tau);
        s_.vx += kv * ex;
        s_.vy += kv * ey;
        s_.bias_x = clampf(s_.bias_x - kb * ex, -p_.bias_max, p_.bias_max);
        s_.bias_y = clampf(s_.bias_y - kb * ey, -p_.bias_max, p_.bias_max);

        bad_count_ = 0;
        if (good_count_ < 255) good_count_++;
        if (good_count_ >= p_.good_to_valid) s_.flow_valid = true;
    } else {
        good_count_ = 0;
        if (bad_count_ < 255) bad_count_++;
        if (bad_count_ >= p_.bad_to_invalid) s_.flow_valid = false;
    }
    s_.reject = reject;
}

}  // namespace poshold
