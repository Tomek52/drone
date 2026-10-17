// SPDX-License-Identifier: MIT
//
// [POSHOLD] See poshold_glue.hpp.

#include "poshold_glue.hpp"

#include <Arduino.h>

#include "flight_control.hpp"
#include "sensor.hpp"

namespace {

poshold::PositionEstimator g_estimator;
Pmw3901Motion g_last_flow;
uint32_t g_loop_counter    = 0;
uint32_t g_last_flow_us    = 0;
uint8_t g_last_mode        = INIT_MODE;
int32_t g_flow_sum_x       = 0;  // cumulative raw counts (exact segment sums for calibration)
int32_t g_flow_sum_y       = 0;

// Minimum valid bottom ToF reading [mm]; upstream ignores readings <= 20 mm.
constexpr int16_t kMinRawRangeMm = 20;

}  // namespace

void poshold_init(void) {
    pmw3901_init();
    g_estimator.reset();
    g_last_flow_us = micros();
}

void poshold_update_estimation(void) {
    // Sensor values are only meaningful after the gyro offset calibration.
    if (Mode <= AVERAGE_MODE) return;

    // New origin whenever the flight mode changes (e.g. take-off, landing).
    if (Mode != g_last_mode) {
        g_estimator.reset_position();
        g_last_mode = Mode;
    }

    g_estimator.predict(Accel_x, Accel_y, Accel_z, Roll_rate, Pitch_rate, Roll_angle - Roll_angle_offset,
                        Pitch_angle - Pitch_angle_offset, Yaw_angle - Yaw_angle_offset, Interval_time);

    if (++g_loop_counter % poshold::config::kFlowReadDivider != 0) return;

    const uint32_t now = micros();
    const float dt     = (now - g_last_flow_us) * 1.0e-6f;
    g_last_flow_us     = now;

    g_last_flow = pmw3901_read_motion();
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
