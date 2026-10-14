// SPDX-License-Identifier: MIT
//
// Position Hold configuration: ALL limits, gains and thresholds of the
// Position Hold feature live in this one file.
//
// Units: SI (m, s, rad) unless the name says otherwise.
// Values marked "CALIBRATE" must be checked on the real drone; see
// docs/06-position-hold.md and docs/07-strojenie.md.

#ifndef POSHOLD_CONFIG_HPP
#define POSHOLD_CONFIG_HPP

#include <stdint.h>

namespace poshold {
namespace config {

// ---------------------------------------------------------------------------
// Optical flow sensor (PMW3901MB) -> body frame mapping
// ---------------------------------------------------------------------------
// Convention after mapping: flow_x > 0 when the drone moves FORWARD over the
// ground, flow_y > 0 when it moves to the RIGHT (body frame: x fwd, y right,
// z down). The sensor's mounting orientation on StampFly is not documented, so
// the mapping below must be verified with the hand-carry test (docs/08-testy.md).
// CALIBRATE: set kFlowAxesVerified = true only after that test passed. Position
// Hold refuses to engage while it is false.
constexpr bool kFlowAxesVerified = false;
constexpr bool kFlowSwapXY       = false;  // use sensor Y as body X (and vice versa)
constexpr float kFlowSignX       = 1.0f;   // +1 or -1, applied after the swap
constexpr float kFlowSignY       = 1.0f;

// Angle per count. Derived from the datasheet field of view (42 deg) over
// the 35-pixel array: 42 deg / 35 = 0.733 rad / 35 = 0.0209 rad.
// CALIBRATE with the known-distance test (docs/07-strojenie.md).
constexpr float kFlowRadPerCount = 0.0209f;

// Flow is read every N-th 400 Hz control loop (4 -> 100 Hz).
constexpr uint8_t kFlowReadDivider = 4;

// ---------------------------------------------------------------------------
// Optical flow quality gates
// ---------------------------------------------------------------------------
// SQUAL (surface quality) below this value -> sample rejected.
// CALIBRATE: check typical SQUAL over your floor in the telemetry (flow_squal).
constexpr uint8_t kFlowMinSqual = 30;
// Height window in which optical flow is trusted [m]. Lower bound: sensor
// focus range starts at ~80 mm (datasheet); upper bound: ToF reliability
// and flow resolution.
constexpr float kFlowMinHeight = 0.10f;
constexpr float kFlowMaxHeight = 1.50f;
// Reject samples during fast rotations or large tilt (compensation errors grow).
constexpr float kFlowMaxBodyRate = 3.0f;   // [rad/s], |p| and |q|
constexpr float kFlowMaxTilt     = 0.45f;  // [rad] (~26 deg), |roll| and |pitch|
// Hysteresis: number of consecutive good/bad flow samples to switch validity.
constexpr uint8_t kFlowGoodSamplesToValid = 5;  // 50 ms at 100 Hz
constexpr uint8_t kFlowBadSamplesToInvalid = 3;  // 30 ms at 100 Hz

// ---------------------------------------------------------------------------
// Velocity / position estimator (complementary filter)
// ---------------------------------------------------------------------------
// Time constant of the correction of the accelerometer-integrated velocity
// towards the optical-flow velocity [s]. Smaller = trust flow more.
constexpr float kVelCorrectionTau = 0.15f;
// Time constant of the horizontal accelerometer bias estimate [s].
constexpr float kAccelBiasTau = 4.0f;
// Clamp of the estimated accelerometer bias [m/s^2].
constexpr float kAccelBiasMax = 1.5f;
// Gravity [m/s^2].
constexpr float kGravity = 9.80665f;

}  // namespace config
}  // namespace poshold

#endif  // POSHOLD_CONFIG_HPP
