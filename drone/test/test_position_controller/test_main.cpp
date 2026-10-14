// Unit tests for lib/poshold/src/position_controller.* (pio test -e native).
// Includes a closed-loop simulation with a simple drone model.

#include <math.h>
#include <unity.h>

#include "position_controller.hpp"

using namespace poshold;

void setUp() {}
void tearDown() {}

namespace {

constexpr float kDt = 0.0025f;
constexpr float kG  = 9.80665f;
constexpr float kDeg = 0.0174533f;

ControllerParams continuous_params() {
    ControllerParams p;
    p.step_mode = false;
    return p;
}

ControllerParams step_params() {
    ControllerParams p;
    p.step_mode = true;
    return p;
}

// Simple horizontal drone model: the attitude follows its reference with a
// first-order lag, the attitude produces horizontal acceleration.
struct Sim {
    float px = 0, py = 0;  // world
    float vx = 0, vy = 0;  // world (yaw is fixed in the sim)
    float roll = 0, pitch = 0;
    float yaw = 0;
    float att_tau = 0.08f;

    void step(float roll_ref, float pitch_ref, float dt) {
        roll += (roll_ref - roll) * dt / att_tau;
        pitch += (pitch_ref - pitch) * dt / att_tau;
        // heading-frame acceleration
        const float ahx = -kG * tanf(pitch);
        const float ahy = kG * tanf(roll);
        const float c = cosf(yaw), s = sinf(yaw);
        vx += (c * ahx - s * ahy) * dt;
        vy += (s * ahx + c * ahy) * dt;
        px += vx * dt;
        py += vy * dt;
    }

    ControllerInput input(float fwd = 0, float right = 0, float climb = 0) const {
        ControllerInput in;
        in.px = px;
        in.py = py;
        // heading-frame velocity
        const float c = cosf(yaw), s = sinf(yaw);
        in.vx          = c * vx + s * vy;
        in.vy          = -s * vx + c * vy;
        in.yaw         = yaw;
        in.stick_fwd   = fwd;
        in.stick_right = right;
        in.stick_climb = climb;
        in.dt          = kDt;
        return in;
    }
};

}  // namespace

void test_at_setpoint_commands_level() {
    PositionController c(continuous_params());
    c.engage(0.3f, -0.2f, 0.5f);
    ControllerInput in;
    in.px = 0.3f;
    in.py = -0.2f;
    in.dt = kDt;
    const ControllerOutput o = c.update(in);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, o.roll_ref);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, o.pitch_ref);
}

void test_setpoint_ahead_pitches_nose_down() {
    PositionController c(continuous_params());
    c.engage(0.2f, 0.0f, 0.5f);  // setpoint 0.2 m in front of the drone (yaw 0)
    ControllerInput in;
    in.dt = kDt;
    const ControllerOutput o = c.update(in);
    TEST_ASSERT_TRUE(o.pitch_ref < 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, o.roll_ref);
}

void test_setpoint_right_rolls_right() {
    PositionController c(continuous_params());
    c.engage(0.0f, 0.2f, 0.5f);
    ControllerInput in;
    in.dt = kDt;
    const ControllerOutput o = c.update(in);
    TEST_ASSERT_TRUE(o.roll_ref > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, o.pitch_ref);
}

void test_yaw_rotates_the_error_into_the_heading_frame() {
    PositionController c(continuous_params());
    c.engage(0.0f, 0.3f, 0.5f);  // setpoint at world +y
    ControllerInput in;
    in.yaw = 1.5707963f;  // drone faces world +y: setpoint is straight ahead
    in.dt  = kDt;
    const ControllerOutput o = c.update(in);
    TEST_ASSERT_TRUE(o.pitch_ref < 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, o.roll_ref);
}

void test_tilt_is_limited() {
    PositionController c(continuous_params());
    c.engage(1.5f, 1.5f, 0.5f);
    ControllerInput in;
    in.vx = -2.0f;  // large error in every loop
    in.dt = kDt;
    ControllerOutput o;
    for (int i = 0; i < 4000; i++) o = c.update(in);
    const float max_tilt = c.params().max_tilt;
    TEST_ASSERT_TRUE(fabsf(o.pitch_ref) <= max_tilt + 1e-6f);
    TEST_ASSERT_TRUE(fabsf(o.roll_ref) <= max_tilt + 1e-6f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f * kDeg, 10.0f * kDeg, fabsf(o.pitch_ref));
}

void test_closed_loop_converges_to_setpoint() {
    PositionController c(continuous_params());
    Sim sim;
    c.engage(0.5f, -0.3f, 0.5f);
    float max_x = 0.0f;
    for (int i = 0; i < (int)(6.0f / kDt); i++) {
        const ControllerOutput o = c.update(sim.input());
        sim.step(o.roll_ref, o.pitch_ref, kDt);
        if (sim.px > max_x) max_x = sim.px;
    }
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.5f, sim.px);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, -0.3f, sim.py);
    TEST_ASSERT_TRUE(max_x < 0.5f * 1.3f);  // overshoot below 30 %
}

void test_closed_loop_rejects_constant_disturbance() {
    // A constant push (e.g. tilt offset / wind) is removed by the integrator.
    PositionController c(continuous_params());
    Sim sim;
    c.engage(0.0f, 0.0f, 0.5f);
    for (int i = 0; i < (int)(10.0f / kDt); i++) {
        const ControllerOutput o = c.update(sim.input());
        sim.step(o.roll_ref + 2.0f * kDeg, o.pitch_ref, kDt);  // 2 deg roll bias
    }
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, sim.py);
}

void test_stick_moves_setpoint_and_stops_on_release() {
    PositionController c(continuous_params());
    Sim sim;
    c.engage(0.0f, 0.0f, 0.5f);
    for (int i = 0; i < (int)(2.0f / kDt); i++) {  // full forward stick for 2 s
        const ControllerOutput o = c.update(sim.input(1.0f, 0.0f));
        sim.step(o.roll_ref, o.pitch_ref, kDt);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.1f, c.params().max_speed, sim.vx);  // ~0.5 m/s
    const float x_release = sim.px;
    for (int i = 0; i < (int)(4.0f / kDt); i++) {
        const ControllerOutput o = c.update(sim.input());
        sim.step(o.roll_ref, o.pitch_ref, kDt);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, sim.vx);  // stopped
    TEST_ASSERT_TRUE(sim.px > x_release);           // no pull-back behind the release point
    TEST_ASSERT_TRUE(sim.px < x_release + 0.4f);    // short braking distance
}

void test_setpoint_lead_is_limited() {
    PositionController c(continuous_params());
    c.engage(0.0f, 0.0f, 0.5f);
    ControllerInput in;  // drone does not move (e.g. blocked)
    in.stick_fwd = 1.0f;
    in.dt        = kDt;
    for (int i = 0; i < (int)(5.0f / kDt); i++) c.update(in);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, c.params().max_lead, c.sp_x());
}

void test_setpoint_stays_inside_fence() {
    PositionController c(continuous_params());
    ControllerInput in;
    in.stick_right = 1.0f;
    in.dt          = kDt;
    c.engage(0.0f, 0.0f, 0.5f);
    for (int i = 0; i < 4000; i++) {
        in.py = c.sp_y();  // drone follows perfectly
        c.update(in);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, c.params().fence_radius, c.sp_y());
}

void test_altitude_setpoint_rate_and_limits() {
    PositionController c(continuous_params());
    c.engage(0, 0, 0.5f);
    ControllerInput in;
    in.stick_climb = 1.0f;
    in.dt          = kDt;
    for (int i = 0; i < 400; i++) c.update(in);  // 1 s full up
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f + c.params().max_climb_rate, c.alt_sp());
    for (int i = 0; i < 4000; i++) c.update(in);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, c.params().max_altitude, c.alt_sp());
    in.stick_climb = -1.0f;
    for (int i = 0; i < 8000; i++) c.update(in);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, c.params().min_altitude, c.alt_sp());
}

void test_deadband_ignores_small_stick_noise() {
    PositionController c(continuous_params());
    c.engage(0, 0, 0.5f);
    ControllerInput in;
    in.stick_fwd   = 0.05f;
    in.stick_climb = -0.08f;
    in.dt          = kDt;
    for (int i = 0; i < 4000; i++) c.update(in);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.sp_x());
    TEST_ASSERT_EQUAL_FLOAT(0.5f, c.alt_sp());
}

void test_yaw_rate_from_stick() {
    PositionController c(continuous_params());
    c.engage(0, 0, 0.5f);
    ControllerInput in;
    in.stick_yaw = 1.0f;
    in.dt        = kDt;
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, c.params().max_yaw_rate, c.update(in).yaw_rate_ref);
}

void test_step_mode_one_step_per_deflection() {
    PositionController c(step_params());
    c.engage(0, 0, 0.5f);
    ControllerInput in;
    in.dt        = kDt;
    in.stick_fwd = 1.0f;
    for (int i = 0; i < 400; i++) c.update(in);  // held for 1 s: only one step
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, c.params().step_xy, c.sp_x());
    in.stick_fwd = 0.0f;
    c.update(in);
    in.stick_fwd = -0.8f;
    c.update(in);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, c.sp_x());
    in.stick_fwd   = 0.0f;
    in.stick_climb = 0.9f;
    c.update(in);
    c.update(in);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f + c.params().step_z, c.alt_sp());
}

void test_step_mode_needs_rearm() {
    PositionController c(step_params());
    c.engage(0, 0, 0.5f);
    ControllerInput in;
    in.dt          = kDt;
    in.stick_right = 0.7f;
    c.update(in);
    in.stick_right = 0.45f;  // between rearm and threshold: still disarmed
    c.update(in);
    in.stick_right = 0.7f;
    c.update(in);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, c.params().step_xy, c.sp_y());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_at_setpoint_commands_level);
    RUN_TEST(test_setpoint_ahead_pitches_nose_down);
    RUN_TEST(test_setpoint_right_rolls_right);
    RUN_TEST(test_yaw_rotates_the_error_into_the_heading_frame);
    RUN_TEST(test_tilt_is_limited);
    RUN_TEST(test_closed_loop_converges_to_setpoint);
    RUN_TEST(test_closed_loop_rejects_constant_disturbance);
    RUN_TEST(test_stick_moves_setpoint_and_stops_on_release);
    RUN_TEST(test_setpoint_lead_is_limited);
    RUN_TEST(test_setpoint_stays_inside_fence);
    RUN_TEST(test_altitude_setpoint_rate_and_limits);
    RUN_TEST(test_deadband_ignores_small_stick_noise);
    RUN_TEST(test_yaw_rate_from_stick);
    RUN_TEST(test_step_mode_one_step_per_deflection);
    RUN_TEST(test_step_mode_needs_rearm);
    return UNITY_END();
}
