// SPDX-License-Identifier: MIT
//
// See position_controller.hpp.

#include "position_controller.hpp"

#include <math.h>

#include "math_utils.hpp"

namespace poshold {

void PositionController::engage(float px, float py, float altitude) {
    sp_x_   = px;
    sp_y_   = py;
    alt_sp_ = clampf(altitude, p_.min_altitude, p_.max_altitude);
    int_x_  = 0.0f;
    int_y_  = 0.0f;
    stick_was_active_ = false;
    step_armed_fwd_ = step_armed_right_ = step_armed_climb_ = true;
    clamp_setpoint_to_fence();
}

void PositionController::clamp_setpoint_to_fence() {
    limit_norm2d(&sp_x_, &sp_y_, p_.fence_radius);
}

void PositionController::update_setpoint_continuous(const ControllerInput& in, float fwd, float right, float* ff_x,
                                                    float* ff_y) {
    // Commanded velocity in the heading frame.
    float cx = fwd * p_.max_speed;
    float cy = right * p_.max_speed;
    limit_norm2d(&cx, &cy, p_.max_speed);
    const bool active = (cx != 0.0f || cy != 0.0f);

    if (active) {
        float wx, wy;
        rotate2d(cx, cy, in.yaw, &wx, &wy);
        sp_x_ += wx * in.dt;
        sp_y_ += wy * in.dt;
        // Do not let the setpoint run away from the drone.
        float lx = sp_x_ - in.px, ly = sp_y_ - in.py;
        limit_norm2d(&lx, &ly, p_.max_lead);
        sp_x_ = in.px + lx;
        sp_y_ = in.py + ly;
    } else if (stick_was_active_) {
        // Stick released: hold at the predicted stopping point instead of
        // pulling back to a setpoint that is already behind the drone.
        float wx, wy;
        rotate2d(in.vx, in.vy, in.yaw, &wx, &wy);
        sp_x_ = in.px + wx * p_.brake_tau;
        sp_y_ = in.py + wy * p_.brake_tau;
    }
    stick_was_active_ = active;
    *ff_x = cx;
    *ff_y = cy;
}

void PositionController::update_setpoint_step(const ControllerInput& in, float fwd, float right) {
    float step_fwd = 0.0f, step_right = 0.0f;
    if (step_armed_fwd_ && fabsf(fwd) >= p_.step_threshold) {
        step_fwd        = fwd > 0 ? p_.step_xy : -p_.step_xy;
        step_armed_fwd_ = false;
    } else if (fabsf(fwd) < p_.step_rearm) {
        step_armed_fwd_ = true;
    }
    if (step_armed_right_ && fabsf(right) >= p_.step_threshold) {
        step_right        = right > 0 ? p_.step_xy : -p_.step_xy;
        step_armed_right_ = false;
    } else if (fabsf(right) < p_.step_rearm) {
        step_armed_right_ = true;
    }
    if (step_fwd != 0.0f || step_right != 0.0f) {
        float wx, wy;
        rotate2d(step_fwd, step_right, in.yaw, &wx, &wy);
        sp_x_ += wx;
        sp_y_ += wy;
    }
}

ControllerOutput PositionController::update(const ControllerInput& in) {
    ControllerOutput out;
    const float fwd   = deadband(clampf(in.stick_fwd, -1, 1), p_.stick_deadband);
    const float right = deadband(clampf(in.stick_right, -1, 1), p_.stick_deadband);
    const float climb = deadband(clampf(in.stick_climb, -1, 1), p_.stick_deadband);
    const float yaw   = deadband(clampf(in.stick_yaw, -1, 1), p_.stick_deadband);

    // --- Setpoints -------------------------------------------------------------
    float ff_x = 0.0f, ff_y = 0.0f;
    if (p_.step_mode) {
        update_setpoint_step(in, fwd, right);
        if (step_armed_climb_ && fabsf(climb) >= p_.step_threshold) {
            alt_sp_ += climb > 0 ? p_.step_z : -p_.step_z;
            step_armed_climb_ = false;
        } else if (fabsf(climb) < p_.step_rearm) {
            step_armed_climb_ = true;
        }
    } else {
        update_setpoint_continuous(in, fwd, right, &ff_x, &ff_y);
        alt_sp_ += climb * p_.max_climb_rate * in.dt;
    }
    clamp_setpoint_to_fence();
    alt_sp_ = clampf(alt_sp_, p_.min_altitude, p_.max_altitude);

    // --- Position loop (P) + feed-forward ---------------------------------------
    float ex, ey;  // position error in the heading frame
    rotate2d(sp_x_ - in.px, sp_y_ - in.py, -in.yaw, &ex, &ey);
    float vsx = p_.pos_kp * ex + ff_x;
    float vsy = p_.pos_kp * ey + ff_y;
    limit_norm2d(&vsx, &vsy, p_.max_speed);

    // --- Velocity loop (PI with anti wind-up) -------------------------------------
    const float evx = vsx - in.vx;
    const float evy = vsy - in.vy;
    int_x_          = clampf(int_x_ + p_.vel_ki * evx * in.dt, -p_.vel_i_max, p_.vel_i_max);
    int_y_          = clampf(int_y_ + p_.vel_ki * evy * in.dt, -p_.vel_i_max, p_.vel_i_max);
    float ax        = p_.vel_kp * evx + int_x_;
    float ay        = p_.vel_kp * evy + int_y_;
    limit_norm2d(&ax, &ay, p_.gravity * tanf(p_.max_tilt));

    // --- Acceleration -> attitude references ------------------------------------
    // Forward acceleration needs nose down (negative pitch); acceleration to the
    // right needs right side down (positive roll).
    out.pitch_ref    = clampf(-atanf(ax / p_.gravity), -p_.max_tilt, p_.max_tilt);
    out.roll_ref     = clampf(atanf(ay / p_.gravity), -p_.max_tilt, p_.max_tilt);
    out.yaw_rate_ref = yaw * p_.max_yaw_rate;
    out.alt_sp       = alt_sp_;
    out.sp_x         = sp_x_;
    out.sp_y         = sp_y_;
    out.vsp_x        = vsx;
    out.vsp_y        = vsy;
    out.acc_x        = ax;
    out.acc_y        = ay;
    return out;
}

}  // namespace poshold
