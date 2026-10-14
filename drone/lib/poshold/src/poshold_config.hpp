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


// ---------------------------------------------------------------------------
// Pilot input in Position Hold
// ---------------------------------------------------------------------------
// Stick dead band around the centre (sticks are normalised to -1..1).
constexpr float kStickDeadband = 0.10f;
// false: stick deflection moves the setpoint with a speed proportional to it
//        (full deflection = kMaxSpeed); on release the drone stops and holds.
// true:  "step" mode: every deflection beyond kStepThreshold moves the
//        setpoint once by kStepXY (horizontal) or kStepZ (vertical).
constexpr bool kStepMode        = false;
constexpr float kStepXY         = 0.30f;  // [m]
constexpr float kStepZ          = 0.20f;  // [m]
constexpr float kStepThreshold  = 0.60f;  // deflection that triggers a step
constexpr float kStepRearm      = 0.30f;  // deflection below which the next step is armed

// ---------------------------------------------------------------------------
// Position Hold limits
// ---------------------------------------------------------------------------
constexpr float kMaxSpeed       = 0.50f;  // [m/s] horizontal speed at full stick / max velocity setpoint
constexpr float kMaxClimbRate   = 0.30f;  // [m/s] vertical speed at full stick
constexpr float kMaxYawRate     = 1.57f;  // [rad/s] (90 deg/s) at full stick
constexpr float kMaxTilt        = 0.1745f;  // [rad] (10 deg) hard limit of roll/pitch references
constexpr float kMinAltitude    = 0.20f;  // [m] altitude setpoint range in Position Hold
constexpr float kMaxAltitude    = 1.20f;  //     (inside the optical flow height window)
constexpr float kMaxLead        = 0.50f;  // [m] max distance between setpoint and drone while flying with the stick
constexpr float kBrakeTau       = 0.30f;  // [s] on stick release the setpoint = position + velocity * kBrakeTau
// Geofence around the take-off point: the setpoint never leaves kFenceRadius;
// beyond kFenceRadius * kFenceLandFactor the drone lands (estimate or control failure).
constexpr float kFenceRadius     = 2.00f;  // [m]
constexpr float kFenceLandFactor = 1.25f;

// ---------------------------------------------------------------------------
// Position Hold controller gains (CALIBRATE with logs, docs/07-strojenie.md)
// ---------------------------------------------------------------------------
// Position error [m] -> velocity setpoint [m/s].
constexpr float kPosKp = 1.0f;
// Velocity error [m/s] -> horizontal acceleration [m/s^2] (PI).
constexpr float kVelKp   = 2.0f;
constexpr float kVelKi   = 0.5f;
constexpr float kVelIMax = 1.0f;  // [m/s^2] integrator limit (anti wind-up)

// ---------------------------------------------------------------------------
// Supervisor / safety
// ---------------------------------------------------------------------------
// Engage only above this height and after the automatic take-off ramp.
constexpr float kEngageMinHeight = 0.15f;  // [m]
// Give up engaging (fallback to Stable) if not possible within this time.
constexpr float kEngageTimeout   = 3.0f;   // [s]
// Flow invalid / height out of window for longer than this -> fallback to Stable.
constexpr float kFlowLossGrace   = 0.20f;  // [s]
// Control link: the upstream timeout (40 loops = 0.1 s) defines "link lost".
// With Auto Altitude the drone first hovers (sticks neutral, Position Hold
// keeps holding if active) and only then starts the upstream auto landing.
constexpr float kLinkLostAfter     = 0.10f;  // [s]
constexpr float kLinkLossHoverTime = 3.0f;   // [s]
// Battery (INA3221 voltage, low-pass filtered). CALIBRATE with logs: the
// voltage sags under load. Never discharge below 3.0 V per cell under load.
constexpr float kBatteryFilterTau  = 1.0f;   // [s]
constexpr float kBatteryWarnVolt   = 3.50f;  // [V] warning beeps
constexpr float kBatteryLandVolt   = 3.35f;  // [V] automatic landing ...
constexpr float kBatteryLandTime   = 2.0f;   // [s] ... when below for this long

}  // namespace config
}  // namespace poshold

#endif  // POSHOLD_CONFIG_HPP
