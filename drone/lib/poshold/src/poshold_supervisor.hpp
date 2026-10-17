// SPDX-License-Identifier: MIT
//
// Position Hold supervisor: decides when the position controller may fly the
// drone and runs the safety logic of this project.
//
//   Idle --(PosHold requested, flying, Auto Alt)--> WaitEngage
//   WaitEngage --(take-off done, flow valid, height ok)--> Active
//   WaitEngage --(no sensor / not calibrated / timeout)--> Fallback
//   Active --(flow invalid or height out of window > grace)--> Fallback
//   Active --(pilot selects Stable)--> Idle
//   Fallback: latched until the drone lands (no automatic re-engage in the air)
//
// Fallback = Stable + Auto Altitude flown with the sticks; it is signalled
// (buzzer + LED + reason on the joystick).
//
// Independent of the mode:
//   - link loss: with Auto Altitude the drone hovers (sticks neutral) for
//     kLinkLossHoverTime, then the upstream auto landing is allowed;
//   - low battery: landing request after kBatteryLandTime below kBatteryLandVolt;
//   - geofence: landing request when the estimated distance from the take-off
//     point exceeds kFenceRadius * kFenceLandFactor.

#ifndef POSHOLD_SUPERVISOR_HPP
#define POSHOLD_SUPERVISOR_HPP

#include <rc_protocol.hpp>

#include "poshold_config.hpp"

namespace poshold {

using stampfly_link::Reason;

struct SupervisorParams {
    float min_height        = config::kFlowMinHeight;
    float max_height        = config::kFlowMaxHeight;
    float engage_min_height = config::kEngageMinHeight;
    float engage_timeout    = config::kEngageTimeout;
    float flow_loss_grace   = config::kFlowLossGrace;
    float link_lost_after   = config::kLinkLostAfter;
    float link_hover_time   = config::kLinkLossHoverTime;
    float batt_tau          = config::kBatteryFilterTau;
    float batt_warn         = config::kBatteryWarnVolt;
    float batt_land         = config::kBatteryLandVolt;
    float batt_land_time    = config::kBatteryLandTime;
    float fence_land_dist   = config::kFenceRadius * config::kFenceLandFactor;
};

struct SupervisorInput {
    bool poshold_requested = false;  // pilot selected Position Hold
    bool flying            = false;  // upstream FLIGHT_MODE
    bool landing           = false;  // upstream AUTO_LANDING_MODE
    bool auto_alt          = false;  // upstream altitude controller active (Alt_flag)
    bool takeoff_done      = false;  // upstream automatic take-off ramp finished
    bool flow_sensor_ok    = false;  // PMW3901 detected at boot
    bool flow_calibrated   = false;  // config::kFlowAxesVerified
    bool flow_valid        = false;  // estimator flow validity
    float height           = 0;      // tilt-compensated height [m] (0 = unknown)
    float distance         = 0;      // estimated distance from the take-off point [m]
    float link_age         = 0;      // time since the last valid control packet [s]
    float vbat             = 0;      // battery voltage [V]
    float dt               = 0;      // [s]
};

enum class PhState : uint8_t { kIdle = 0, kWaitEngage = 1, kActive = 2, kFallback = 3 };

struct SupervisorOutput {
    PhState state            = PhState::kIdle;
    bool poshold_active      = false;  // controller drives the attitude references
    bool engage_now          = false;  // one-shot: initialise the controller setpoints
    bool fallback_event      = false;  // one-shot: play the fallback alert
    bool link_lost           = false;
    bool neutral_sticks      = false;  // link lost: treat the sticks as centred
    bool allow_link_landing  = true;   // upstream may switch to auto landing on link loss
    bool request_landing     = false;  // low battery / geofence: start auto landing
    bool low_battery_warning = false;
    Reason reason            = Reason::kNone;
    float vbat_filtered      = 0;
};

class PosHoldSupervisor {
   public:
    explicit PosHoldSupervisor(const SupervisorParams& p = SupervisorParams()) : p_(p) {}
    SupervisorOutput update(const SupervisorInput& in);
    PhState state() const { return state_; }

   private:
    void to_fallback(Reason r, SupervisorOutput* out);

    SupervisorParams p_;
    PhState state_         = PhState::kIdle;
    Reason fallback_reason_ = Reason::kNone;
    float wait_time_       = 0;
    float bad_time_        = 0;
    float vbat_f_          = 0;
    float low_batt_time_   = 0;
    bool landing_latched_  = false;
    Reason landing_reason_ = Reason::kNone;
};

}  // namespace poshold

#endif  // POSHOLD_SUPERVISOR_HPP
