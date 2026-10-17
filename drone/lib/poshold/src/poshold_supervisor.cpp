// SPDX-License-Identifier: MIT
//
// See poshold_supervisor.hpp.

#include "poshold_supervisor.hpp"

namespace poshold {

void PosHoldSupervisor::to_fallback(Reason r, SupervisorOutput* out) {
    state_           = PhState::kFallback;
    fallback_reason_ = r;
    out->fallback_event = true;
}

SupervisorOutput PosHoldSupervisor::update(const SupervisorInput& in) {
    SupervisorOutput out;
    const bool airborne = in.flying || in.landing;

    // --- Battery (all modes) ----------------------------------------------------------
    if (vbat_f_ <= 0.0f) vbat_f_ = in.vbat;  // first sample
    if (in.dt > 0.0f) vbat_f_ += (in.vbat - vbat_f_) * (in.dt / (p_.batt_tau + in.dt));
    out.vbat_filtered       = vbat_f_;
    out.low_battery_warning = airborne && vbat_f_ > 0.0f && vbat_f_ < p_.batt_warn;
    if (in.flying && vbat_f_ > 0.0f && vbat_f_ < p_.batt_land) {
        low_batt_time_ += in.dt;
    } else {
        low_batt_time_ = 0.0f;
    }

    // --- Not in the air: reset everything ----------------------------------------------
    if (!airborne) {
        state_           = PhState::kIdle;
        fallback_reason_ = Reason::kNone;
        wait_time_ = bad_time_ = 0.0f;
        landing_latched_ = false;
        landing_reason_  = Reason::kNone;
        out.state        = state_;
        return out;
    }

    // --- Link loss (all modes) ---------------------------------------------------------
    out.link_lost = in.link_age > p_.link_lost_after;
    // The upstream timeout (40 loops = 0.1 s) trips exactly when link_age == link_lost_after, one
    // loop before link_lost becomes true, so with Auto Altitude the landing must be blocked
    // from the first missed packet on, not only once link_lost is set.
    if (in.auto_alt && in.flying) {
        out.allow_link_landing = in.link_age > p_.link_lost_after + p_.link_hover_time;
        if (out.link_lost) out.neutral_sticks = true;
    }

    // --- Landing requests (latched until touchdown) ------------------------------------
    if (!landing_latched_ && low_batt_time_ >= p_.batt_land_time) {
        landing_latched_ = true;
        landing_reason_  = Reason::kLowBattery;
    }

    // --- Position Hold state machine ---------------------------------------------------
    const bool requested     = in.poshold_requested;
    const bool height_ok     = in.height >= p_.min_height && in.height <= p_.max_height;
    const bool flow_ok       = in.flow_valid && height_ok;
    const bool sensor_usable = in.flow_sensor_ok && in.flow_calibrated;

    switch (state_) {
        case PhState::kIdle:
            if (requested && in.flying && in.auto_alt) {
                state_     = PhState::kWaitEngage;
                wait_time_ = 0.0f;
            }
            break;

        case PhState::kWaitEngage:
            if (!requested) {
                state_ = PhState::kIdle;
            } else if (!in.flow_sensor_ok) {
                to_fallback(Reason::kFlowSensorMissing, &out);
            } else if (!in.flow_calibrated) {
                to_fallback(Reason::kFlowNotCalibrated, &out);
            } else if (in.takeoff_done) {
                if (flow_ok && in.height >= p_.engage_min_height) {
                    state_         = PhState::kActive;
                    bad_time_      = 0.0f;
                    out.engage_now = true;
                } else {
                    wait_time_ += in.dt;
                    if (wait_time_ >= p_.engage_timeout)
                        to_fallback(height_ok ? Reason::kFlowInvalid : Reason::kHeightOutOfRange, &out);
                }
            }
            break;

        case PhState::kActive:
            if (!requested && !out.link_lost) {
                state_ = PhState::kIdle;  // pilot switched to Stable
            } else if (in.landing && in.height < p_.min_height) {
                state_ = PhState::kIdle;  // close to the ground while landing: level attitude, no alarm
            } else if (!sensor_usable) {
                to_fallback(Reason::kFlowSensorMissing, &out);
            } else if (!flow_ok) {
                bad_time_ += in.dt;
                if (bad_time_ >= p_.flow_loss_grace)
                    to_fallback(height_ok ? Reason::kFlowInvalid : Reason::kHeightOutOfRange, &out);
            } else {
                bad_time_ = 0.0f;
            }
            if (state_ == PhState::kActive && in.distance > p_.fence_land_dist) {
                to_fallback(Reason::kGeofence, &out);
                if (!landing_latched_) {
                    landing_latched_ = true;
                    landing_reason_  = Reason::kGeofence;
                }
            }
            break;

        case PhState::kFallback:
            break;  // latched until touchdown
    }

    out.state           = state_;
    out.poshold_active  = state_ == PhState::kActive;
    out.request_landing = landing_latched_ && in.flying;

    // Reason shown on the joystick, most important first.
    if (landing_latched_) {
        out.reason = landing_reason_;
    } else if (out.link_lost) {
        out.reason = Reason::kLinkLost;
    } else if (state_ == PhState::kFallback) {
        out.reason = fallback_reason_;
    } else {
        out.reason = Reason::kNone;
    }
    return out;
}

}  // namespace poshold
