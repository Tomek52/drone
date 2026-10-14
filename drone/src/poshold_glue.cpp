// SPDX-License-Identifier: MIT
//
// [POSHOLD] See poshold_glue.hpp.

#include "poshold_glue.hpp"

#ifndef POSHOLD_PMW_DEBUG
#define POSHOLD_PMW_DEBUG 0
#endif

#include <Arduino.h>

#include <math.h>
#include <math_utils.hpp>

#include "alerts.hpp"
#include "telemetry_stream.hpp"
#include "flight_control.hpp"
#include "led.hpp"
#include "rc.hpp"
#include "rc_link.hpp"
#include "sensor.hpp"

extern uint16_t Auto_takeoff_counter;  // flight_control.cpp: automatic take-off ramp (done at 1000)

using poshold::PhState;
using stampfly_link::FlightMode;

namespace {

poshold::PositionEstimator g_estimator;
Pmw3901Motion g_last_flow;
uint32_t g_loop_counter    = 0;
uint32_t g_last_flow_us    = 0;
uint8_t g_last_mode        = INIT_MODE;
int32_t g_flow_sum_x       = 0;  // cumulative raw counts (exact segment sums for calibration)
int32_t g_flow_sum_y       = 0;
// [POSHOLD] diagnostics: reject reasons of EVERY flow sample (100 Hz), because the
// 50 Hz telemetry only sees one phase of them.
uint8_t g_reject_mask       = 0;  // bit n set: reason n occurred since the last telemetry read
uint32_t g_reject_total     = 0;  // cumulative number of rejected samples

// Minimum valid bottom ToF reading [mm]; upstream ignores readings <= 20 mm.
constexpr int16_t kMinRawRangeMm = 20;

constexpr uint16_t kTakeoffDoneCount = 1000;  // upstream get_command(): ramp length in loops
constexpr float kLoopPeriod          = 0.0025f;

poshold::PosHoldSupervisor g_supervisor;
poshold::PositionController g_controller;
poshold::SupervisorOutput g_sup;
poshold::ControllerOutput g_ctrl;

}  // namespace

void poshold_init(void) {
    pmw3901_init();
    g_estimator.reset();
    g_last_flow_us = micros();
}

void poshold_update_estimation(void) {
    // Sensor values are only meaningful after the gyro offset calibration.
    if (Mode <= AVERAGE_MODE) return;

    // The take-off point is the origin (geofence, logs): reset on take-off only,
    // not on FLIGHT -> AUTO_LANDING / FLIP transitions.
    if (Mode == FLIGHT_MODE && g_last_mode == PARKING_MODE) g_estimator.reset_position();
    g_last_mode = Mode;

    g_estimator.predict(Accel_x, Accel_y, Accel_z, Roll_rate, Pitch_rate, Roll_angle - Roll_angle_offset,
                        Pitch_angle - Pitch_angle_offset, Yaw_angle - Yaw_angle_offset, Interval_time);

    if (++g_loop_counter % poshold::config::kFlowReadDivider != 0) return;

    const uint32_t now = micros();
    const float dt     = (now - g_last_flow_us) * 1.0e-6f;
    g_last_flow_us     = now;

    g_last_flow = pmw3901_read_motion();
#if POSHOLD_PMW_DEBUG
    // [POSHOLD] bench diagnostics only (build with -DPOSHOLD_PMW_DEBUG=1), ~2 Hz.
    if (g_loop_counter % 200 == 0) pmw3901_debug_dump();
#endif
    if (g_last_flow.ok) {
        g_flow_sum_x += g_last_flow.dx;
        g_flow_sum_y += g_last_flow.dy;
    }
    poshold::FlowSample s;
    s.read_ok = g_last_flow.ok;
    s.dx      = g_last_flow.dx;
    s.dy      = g_last_flow.dy;
    s.squal   = g_last_flow.squal;

    // Altitude2: Kalman-filtered bottom ToF distance (slant range, not tilt compensated).
    const float range = (RawRange > kMinRawRangeMm) ? (float)Altitude2 : -1.0f;
    g_estimator.update_flow(s, range, dt);

    const uint8_t reason = (uint8_t)g_estimator.state().reject;
    g_reject_mask |= (uint8_t)(1u << reason);
    if (reason != 0) g_reject_total++;
}

const poshold::EstimatorState& poshold_estimator_state(void) {
    return g_estimator.state();
}

const Pmw3901Motion& poshold_last_flow(void) {
    return g_last_flow;
}

int32_t poshold_flow_sum_x(void) {
    return g_flow_sum_x;
}

int32_t poshold_flow_sum_y(void) {
    return g_flow_sum_y;
}

uint8_t poshold_flow_reject_mask_take(void) {
    const uint8_t m = g_reject_mask;
    g_reject_mask   = 0;
    return m;
}

uint32_t poshold_flow_reject_total(void) {
    return g_reject_total;
}

// ---------------------------------------------------------------------------
// Position Hold control and safety
// ---------------------------------------------------------------------------

namespace {

void set_neutral_sticks(void) {
    Stick[AILERON]     = 0.0f;
    Stick[ELEVATOR]    = 0.0f;
    Stick[THROTTLE]    = 0.0f;  // Auto Altitude: 0 = keep the altitude setpoint
    Stick[RUDDER]      = Rudder_center;
    Stick[BUTTON_ARM]  = 0.0f;
    Stick[BUTTON_FLIP] = 0.0f;
}

void update_alerts_and_led(const poshold::SupervisorOutput& o, bool airborne) {
    if (o.fallback_event) alerts_play(Alert::kFallback);
    if (o.engage_now) alerts_play(Alert::kModeOn);
    if (airborne && o.link_lost) {
        alerts_play(Alert::kLinkLost);
    } else if (airborne && o.low_battery_warning) {
        alerts_stop(Alert::kLinkLost);
        alerts_play(Alert::kLowBattery);
    } else {
        alerts_stop(Alert::kLinkLost);
        alerts_stop(Alert::kLowBattery);
    }

    if (o.state == PhState::kActive) {
        alerts_set_led_override(LedOwner::kPosHold, BLUE, BLUE, 0);  // steady blue: Position Hold
    } else if (o.state == PhState::kFallback) {
        alerts_set_led_override(LedOwner::kPosHold, BLUE, 0x000000, 200);  // blinking blue: Position Hold lost
    } else {
        alerts_clear_led_override(LedOwner::kPosHold);
    }
}

void report_status(const poshold::SupervisorOutput& o) {
    using namespace stampfly_link;
    const FlightMode requested = rc_link_requested_mode();
    const bool available       = pmw3901_present() && poshold::config::kFlowAxesVerified;
    FlightMode active;
    if (o.poshold_active) {
        active = FlightMode::kPosHold;
    } else {
        active = requested == FlightMode::kSport ? FlightMode::kSport : FlightMode::kStable;
    }
    uint8_t flags = 0;
    if (available) flags |= kStatusFlowAvailable;
    if (g_estimator.state().flow_valid) flags |= kStatusFlowValid;
    if (o.poshold_active) flags |= kStatusPosHoldActive;
    if (o.state == PhState::kFallback || o.link_lost || o.request_landing) flags |= kStatusFailsafe;
    if (o.low_battery_warning) flags |= kStatusLowBattery;
    Reason reason = o.reason;
    // On the ground, explain why Position Hold would not be available.
    if (reason == Reason::kNone && requested == FlightMode::kPosHold && !o.poshold_active) {
        if (!pmw3901_present()) reason = Reason::kFlowSensorMissing;
        else if (!poshold::config::kFlowAxesVerified) reason = Reason::kFlowNotCalibrated;
    }
    rc_link_set_status(active, flags, reason);
}

}  // namespace

void poshold_safety_update(void) {
    if (Mode <= AVERAGE_MODE) return;
    const poshold::EstimatorState& est = g_estimator.state();

    poshold::SupervisorInput in;
    in.poshold_requested = rc_link_requested_mode() == FlightMode::kPosHold;
    in.flying            = Mode == FLIGHT_MODE || Mode == FLIP_MODE;  // a flip (Stable/Sport) is still flight
    in.landing           = Mode == AUTO_LANDING_MODE;
    in.auto_alt          = Alt_flag >= 1;
    in.takeoff_done      = Auto_takeoff_counter >= kTakeoffDoneCount;
    in.flow_sensor_ok    = pmw3901_present();
    in.flow_calibrated   = poshold::config::kFlowAxesVerified;
    in.flow_valid        = est.flow_valid;
    in.height            = est.height;
    in.distance          = sqrtf(est.px * est.px + est.py * est.py);
    in.link_age          = Connect_flag * kLoopPeriod;  // loops since the last packet (counted in FLIGHT_MODE)
    in.vbat              = Voltage;
    in.dt                = Interval_time;

    g_sup = g_supervisor.update(in);

    if (g_sup.neutral_sticks) set_neutral_sticks();
    if (g_sup.engage_now) g_controller.engage(est.px, est.py, Alt_ref);
    if (g_sup.request_landing && Mode == FLIGHT_MODE) {
        Mode = AUTO_LANDING_MODE;
        telemetry_text("[POSHOLD] auto landing, reason %u\r\n", (unsigned)g_sup.reason);
    }

    if (g_sup.poshold_active) {
        poshold::ControllerInput ci;
        ci.px          = est.px;
        ci.py          = est.py;
        ci.vx          = est.vx;
        ci.vy          = est.vy;
        ci.yaw         = Yaw_angle - Yaw_angle_offset;
        // Upstream stick conventions: elevator < 0 = forward (nose down),
        // aileron > 0 = right, throttle > 0 = up, rudder > 0 = yaw right.
        ci.stick_fwd   = -Stick[ELEVATOR];
        ci.stick_right = Stick[AILERON];
        ci.stick_climb = (Mode == FLIGHT_MODE) ? (float)Stick[THROTTLE] : 0.0f;
        ci.stick_yaw   = Stick[RUDDER] - Rudder_center;
        ci.dt          = Interval_time;
        g_ctrl         = g_controller.update(ci);
    }

    const bool airborne = Mode == FLIGHT_MODE || Mode == AUTO_LANDING_MODE || Mode == FLIP_MODE;
    update_alerts_and_led(g_sup, airborne);
    report_status(g_sup);
}

void poshold_after_get_command(void) {
    if (!g_sup.poshold_active) return;
    Alt_ref            = g_ctrl.alt_sp;
    Yaw_rate_reference = g_ctrl.yaw_rate_ref;
    // No flips in Position Hold.
    if (Mode == FLIP_MODE) {
        Mode      = FLIGHT_MODE;
        Flip_flag = 0;
    }
}

void poshold_override_attitude(volatile float* roll_ref, volatile float* pitch_ref) {
    if (!g_sup.poshold_active) return;
    *roll_ref  = g_ctrl.roll_ref;
    *pitch_ref = g_ctrl.pitch_ref;
}

bool poshold_link_loss_landing_allowed(void) {
    return g_sup.allow_link_landing;
}

const poshold::SupervisorOutput& poshold_supervisor_output(void) {
    return g_sup;
}

const poshold::ControllerOutput& poshold_controller_output(void) {
    return g_ctrl;
}
