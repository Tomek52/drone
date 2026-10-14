// SPDX-License-Identifier: MIT
//
// Horizontal velocity / position estimator for Position Hold.
//
// Inputs:
//   - IMU at the control rate (400 Hz): body specific force, body rates,
//     attitude from the existing Madgwick filter,
//   - optical flow (PMW3901) at ~100 Hz plus the slant range from the bottom
//     ToF sensor.
//
// Frames:
//   body    : x forward, y right, z down (upstream convention)
//   heading : body yawed frame with zero roll/pitch (horizontal plane)
//   world   : heading frame rotated by the yaw angle at the time of motion;
//             origin = point where reset()/reset_position() was called.
//
// Optical flow model (flat floor, small angles):
//   flow rate wx [rad/s] = vx/h + q,  wy = vy/h - p
//   => vx = h * (wx - q),  vy = h * (wy + p),  h = range * cos(roll) * cos(pitch)
// The flow velocity corrects the accelerometer-integrated velocity in a
// complementary filter that also estimates the horizontal accelerometer bias.
// Position is the integral of the velocity (drift is accepted in this version).
//
// Hardware independent: compiled for ESP32 and for host unit tests.

#ifndef POSHOLD_POSITION_ESTIMATOR_HPP
#define POSHOLD_POSITION_ESTIMATOR_HPP

#include <stdint.h>

#include "poshold_config.hpp"

namespace poshold {

struct FlowSample {
    int16_t dx    = 0;  // raw sensor counts since the previous read
    int16_t dy    = 0;
    uint8_t squal = 0;  // surface quality reported by the sensor
    bool read_ok  = false;
};

// Why the last flow sample was rejected (0 = accepted).
enum class FlowReject : uint8_t {
    kNone       = 0,
    kNoData     = 1,  // sensor read failed or no range measurement
    kHeightLow  = 2,
    kHeightHigh = 3,
    kLowQuality = 4,  // SQUAL below threshold
    kBodyRate   = 5,  // rotating too fast for reliable compensation
    kTilt       = 6,
};

struct EstimatorParams {
    bool flow_swap_xy        = config::kFlowSwapXY;
    float flow_sign_x        = config::kFlowSignX;
    float flow_sign_y        = config::kFlowSignY;
    float flow_rad_per_count = config::kFlowRadPerCount;
    uint8_t min_squal        = config::kFlowMinSqual;
    float min_height         = config::kFlowMinHeight;
    float max_height         = config::kFlowMaxHeight;
    float max_body_rate      = config::kFlowMaxBodyRate;
    float max_tilt           = config::kFlowMaxTilt;
    uint8_t good_to_valid    = config::kFlowGoodSamplesToValid;
    uint8_t bad_to_invalid   = config::kFlowBadSamplesToInvalid;
    float vel_tau            = config::kVelCorrectionTau;
    float bias_tau           = config::kAccelBiasTau;
    float bias_max           = config::kAccelBiasMax;
    float gravity            = config::kGravity;
};

struct EstimatorState {
    float vx = 0.0f, vy = 0.0f;            // fused velocity, heading frame [m/s]
    float px = 0.0f, py = 0.0f;            // position, world frame [m]
    float flow_vx = 0.0f, flow_vy = 0.0f;  // velocity from the last flow sample [m/s]
    float height       = 0.0f;             // tilt-compensated height [m]
    float acc_x = 0.0f, acc_y = 0.0f;      // horizontal acceleration, heading frame [m/s^2]
    float bias_x = 0.0f, bias_y = 0.0f;    // estimated accelerometer bias [m/s^2]
    uint8_t squal      = 0;
    bool flow_valid    = false;            // flow trusted (with hysteresis)
    FlowReject reject  = FlowReject::kNoData;
};

class PositionEstimator {
   public:
    explicit PositionEstimator(const EstimatorParams& params = EstimatorParams()) : p_(params) {}

    // Clears everything (velocity, position, bias, validity).
    void reset();
    // Sets the current position as the origin; velocity and bias are kept.
    void reset_position();

    // Called at the control rate.
    //   fx, fy, fz : body specific force [g] (level and still: 0, 0, -1)
    //   p, q       : body roll and pitch rates [rad/s]
    //   roll, pitch, yaw : attitude [rad]
    void predict(float fx, float fy, float fz, float p, float q, float roll, float pitch, float yaw, float dt);

    // Called for every optical flow read.
    //   range_m : slant distance from the bottom ToF [m], <= 0 when unavailable
    //   dt      : time since the previous flow read [s]
    void update_flow(const FlowSample& s, float range_m, float dt);

    const EstimatorState& state() const { return s_; }
    const EstimatorParams& params() const { return p_; }

   private:
    FlowReject check_sample(const FlowSample& s, float height, float p_avg, float q_avg) const;

    EstimatorParams p_;
    EstimatorState s_;
    float roll_ = 0.0f, pitch_ = 0.0f;
    float p_sum_ = 0.0f, q_sum_ = 0.0f;
    uint16_t rate_samples_ = 0;
    float last_p_ = 0.0f, last_q_ = 0.0f;
    uint8_t good_count_ = 0, bad_count_ = 0;
};

}  // namespace poshold

#endif  // POSHOLD_POSITION_ESTIMATOR_HPP
