// SPDX-License-Identifier: MIT
//
// Position Hold controller (cascade):
//
//   position setpoint (world) --P--> velocity setpoint (heading, + stick feed-forward)
//   --PI--> horizontal acceleration --atan--> roll/pitch references (limit kMaxTilt)
//   --> existing upstream angle controller
//
// Altitude: the controller produces an altitude setpoint for the existing
// upstream Auto Altitude controller (Alt_ref); yaw: a yaw-rate reference.
//
// Sticks (normalised -1..1, dead band applied here):
//   forward/right move the horizontal setpoint, climb moves the altitude
//   setpoint, yaw rotates. Continuous mode: speed proportional to deflection;
//   step mode: one fixed step per deflection.
//
// Frames: see position_estimator.hpp. Angles follow the upstream convention:
// roll > 0 = right side down (accelerates to the right), pitch > 0 = nose up
// (accelerates backwards).

#ifndef POSHOLD_POSITION_CONTROLLER_HPP
#define POSHOLD_POSITION_CONTROLLER_HPP

#include "poshold_config.hpp"

namespace poshold {

struct ControllerParams {
    float stick_deadband = config::kStickDeadband;
    bool step_mode       = config::kStepMode;
    float step_xy        = config::kStepXY;
    float step_z         = config::kStepZ;
    float step_threshold = config::kStepThreshold;
    float step_rearm     = config::kStepRearm;
    float max_speed      = config::kMaxSpeed;
    float max_climb_rate = config::kMaxClimbRate;
    float max_yaw_rate   = config::kMaxYawRate;
    float max_tilt       = config::kMaxTilt;
    float min_altitude   = config::kMinAltitude;
    float max_altitude   = config::kMaxAltitude;
    float max_lead       = config::kMaxLead;
    float brake_tau      = config::kBrakeTau;
    float fence_radius   = config::kFenceRadius;
    float pos_kp         = config::kPosKp;
    float vel_kp         = config::kVelKp;
    float vel_ki         = config::kVelKi;
    float vel_i_max      = config::kVelIMax;
    float gravity        = config::kGravity;
};

struct ControllerInput {
    float px = 0, py = 0;    // estimated position, world frame [m]
    float vx = 0, vy = 0;    // estimated velocity, heading frame [m/s]
    float yaw = 0;           // [rad]
    float stick_fwd   = 0;   // +1 = forward
    float stick_right = 0;   // +1 = right
    float stick_climb = 0;   // +1 = up
    float stick_yaw   = 0;   // +1 = turn right
    float dt = 0;            // [s]
};

struct ControllerOutput {
    float roll_ref     = 0;  // [rad]
    float pitch_ref    = 0;  // [rad]
    float yaw_rate_ref = 0;  // [rad/s]
    float alt_sp       = 0;  // [m]
    float sp_x = 0, sp_y = 0;    // position setpoint, world [m]
    float vsp_x = 0, vsp_y = 0;  // velocity setpoint, heading [m/s]
    float acc_x = 0, acc_y = 0;  // commanded acceleration, heading [m/s^2]
};

class PositionController {
   public:
    explicit PositionController(const ControllerParams& p = ControllerParams()) : p_(p) {}

    // Starts holding the given position and altitude; resets the integrators.
    void engage(float px, float py, float altitude);

    ControllerOutput update(const ControllerInput& in);

    const ControllerParams& params() const { return p_; }
    float sp_x() const { return sp_x_; }
    float sp_y() const { return sp_y_; }
    float alt_sp() const { return alt_sp_; }

   private:
    void update_setpoint_continuous(const ControllerInput& in, float fwd, float right, float* ff_x, float* ff_y);
    void update_setpoint_step(const ControllerInput& in, float fwd, float right);
    void clamp_setpoint_to_fence();

    ControllerParams p_;
    float sp_x_ = 0, sp_y_ = 0, alt_sp_ = 0;
    float int_x_ = 0, int_y_ = 0;
    bool stick_was_active_ = false;
    bool step_armed_fwd_ = true, step_armed_right_ = true, step_armed_climb_ = true;
};

}  // namespace poshold

#endif  // POSHOLD_POSITION_CONTROLLER_HPP
