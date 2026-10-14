// Unit tests for lib/poshold/src/position_estimator.* (pio test -e native).
//
// A tiny simulator generates consistent IMU and optical-flow inputs from a
// known "true" motion, so the tests check the physics conventions end to end:
// flow sign/scale, gyro compensation, tilt-compensated height, quality gates,
// accelerometer fusion and position integration with yaw.

#include <math.h>
#include <unity.h>

#include "position_estimator.hpp"

using namespace poshold;

void setUp();
void tearDown() {}

namespace {

constexpr float kDt     = 0.0025f;  // 400 Hz
constexpr int kFlowDiv  = 4;        // 100 Hz
constexpr float kG      = 9.80665f;

EstimatorParams test_params() {
    EstimatorParams p;  // defaults from poshold_config.hpp, identity axis mapping
    p.flow_swap_xy = false;
    p.flow_sign_x  = 1.0f;
    p.flow_sign_y  = 1.0f;
    return p;
}

struct Truth {
    float vx = 0, vy = 0;          // heading-frame velocity [m/s]
    float ax = 0, ay = 0;          // heading-frame acceleration [m/s^2]
    float p = 0, q = 0;            // body rates [rad/s]
    float roll = 0, pitch = 0, yaw = 0;
    float height = 0.5f;           // vertical height above the floor [m]
    uint8_t squal = 100;
    float accel_bias_x = 0.0f;     // bias added to measured specific force [m/s^2]
};

// Body specific force [g] for a given heading-frame acceleration and attitude
// (inverse of the rotation used by the estimator), plus gravity.
void specific_force(const Truth& t, float* fx, float* fy, float* fz) {
    const float cr = cosf(t.roll), sr = sinf(t.roll), cp = cosf(t.pitch), sp = sinf(t.pitch);
    // heading-frame specific force: (ax, ay, -g)
    const float hx = t.ax, hy = t.ay, hz = -kG;
    // body = Rx(roll)^T * Ry(pitch)^T * heading
    const float x1 = cp * hx - sp * hz;
    const float z1 = sp * hx + cp * hz;
    const float bx = x1;
    const float by = cr * hy + sr * z1;
    const float bz = -sr * hy + cr * z1;
    *fx = (bx + t.accel_bias_x) / kG;
    *fy = by / kG;
    *fz = bz / kG;
}

// Flow counts for one flow period. Like the real sensor, the fractional part
// of the motion is carried over to the next read (counts accumulate in the
// sensor's registers until they are read).
struct FlowSim {
    float carry_x = 0.0f, carry_y = 0.0f;
    FlowSample sample(const Truth& t, float dt_flow, float rad_per_count) {
        const float wx = t.vx / t.height + t.q;
        const float wy = t.vy / t.height - t.p;
        carry_x += wx * dt_flow / rad_per_count;
        carry_y += wy * dt_flow / rad_per_count;
        FlowSample s;
        s.dx = (int16_t)truncf(carry_x);
        s.dy = (int16_t)truncf(carry_y);
        carry_x -= s.dx;
        carry_y -= s.dy;
        s.squal   = t.squal;
        s.read_ok = true;
        return s;
    }
};
FlowSim g_flow;

float slant_range(const Truth& t) {
    return t.height / (cosf(t.roll) * cosf(t.pitch));
}

// Runs the estimator for `seconds` with constant truth.
void run(PositionEstimator& est, const Truth& t, float seconds) {
    const int steps = (int)(seconds / kDt + 0.5f);
    for (int i = 0; i < steps; i++) {
        float fx, fy, fz;
        specific_force(t, &fx, &fy, &fz);
        est.predict(fx, fy, fz, t.p, t.q, t.roll, t.pitch, t.yaw, kDt);
        if ((i + 1) % kFlowDiv == 0) {
            const float dtf = kDt * kFlowDiv;
            est.update_flow(g_flow.sample(t, dtf, est.params().flow_rad_per_count), slant_range(t), dtf);
        }
    }
}

}  // namespace

void setUp() { g_flow = FlowSim(); }

void test_hover_still_gives_zero_velocity() {
    PositionEstimator est(test_params());
    Truth t;
    run(est, t, 2.0f);
    TEST_ASSERT_TRUE(est.state().flow_valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, est.state().vx);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, est.state().vy);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, est.state().px);
}

void test_constant_forward_velocity() {
    PositionEstimator est(test_params());
    Truth t;
    t.vx = 0.3f;
    run(est, t, 2.0f);
    // At 0.5 m one count per 10 ms sample is ~1 m/s, so single samples are
    // coarse; the complementary filter averages them.
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.3f, est.state().vx);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.0f, est.state().vy);
}

void test_constant_right_velocity() {
    PositionEstimator est(test_params());
    Truth t;
    t.vy = -0.4f;
    t.height = 0.8f;
    run(est, t, 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, -0.4f, est.state().vy);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.0f, est.state().vx);
}

void test_pure_rotation_is_compensated() {
    PositionEstimator est(test_params());
    Truth t;
    t.q = 1.0f;   // pitching at 1 rad/s, no translation
    t.p = -0.7f;  // and rolling
    // Single flow samples are quantised (~1 m/s per count at 0.5 m and 100 Hz),
    // so check the drift of the fused estimate: it must stay near zero.
    float max_abs = 0.0f;
    for (int k = 0; k < 20; k++) {
        run(est, t, 0.1f);
        max_abs = fmaxf(max_abs, fmaxf(fabsf(est.state().vx), fabsf(est.state().vy)));
    }
    TEST_ASSERT_TRUE(est.state().flow_valid);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, max_abs);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, est.state().px);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, est.state().py);
}

void test_height_is_tilt_compensated() {
    PositionEstimator est(test_params());
    Truth t;
    t.roll  = 0.3f;
    t.pitch = -0.2f;
    t.height = 0.6f;
    run(est, t, 0.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.6f, est.state().height);
}

void test_stationary_tilt_gives_no_horizontal_acceleration() {
    PositionEstimator est(test_params());
    Truth t;
    t.roll  = 0.2f;
    t.pitch = 0.25f;
    float fx, fy, fz;
    specific_force(t, &fx, &fy, &fz);
    est.predict(fx, fy, fz, 0, 0, t.roll, t.pitch, 0, kDt);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, est.state().acc_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, est.state().acc_y);
}

void test_nose_up_reads_positive_fx() {
    // Upstream convention check: nose up (pitch > 0) at rest -> +x specific force.
    Truth t;
    t.pitch = 0.3f;
    float fx, fy, fz;
    specific_force(t, &fx, &fy, &fz);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, sinf(0.3f), fx);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -cosf(0.3f), fz);
}

void test_low_squal_invalidates_flow_with_hysteresis() {
    PositionEstimator est(test_params());
    Truth t;
    run(est, t, 0.5f);
    TEST_ASSERT_TRUE(est.state().flow_valid);
    t.squal = 5;
    run(est, t, 0.02f);  // 2 bad samples: still valid (needs 3)
    TEST_ASSERT_TRUE(est.state().flow_valid);
    run(est, t, 0.02f);
    TEST_ASSERT_FALSE(est.state().flow_valid);
    TEST_ASSERT_EQUAL((int)FlowReject::kLowQuality, (int)est.state().reject);
    t.squal = 100;
    run(est, t, 0.03f);  // 3 good samples: not yet valid (needs 5)
    TEST_ASSERT_FALSE(est.state().flow_valid);
    run(est, t, 0.03f);
    TEST_ASSERT_TRUE(est.state().flow_valid);
}

void test_height_window() {
    PositionEstimator est(test_params());
    Truth t;
    t.height = 0.05f;
    run(est, t, 0.2f);
    TEST_ASSERT_FALSE(est.state().flow_valid);
    TEST_ASSERT_EQUAL((int)FlowReject::kHeightLow, (int)est.state().reject);
    t.height = 2.5f;
    run(est, t, 0.2f);
    TEST_ASSERT_EQUAL((int)FlowReject::kHeightHigh, (int)est.state().reject);
}

void test_no_range_is_rejected() {
    PositionEstimator est(test_params());
    FlowSample s;
    s.read_ok = true;
    s.squal   = 100;
    est.update_flow(s, -1.0f, 0.01f);
    TEST_ASSERT_EQUAL((int)FlowReject::kNoData, (int)est.state().reject);
}

void test_fast_rotation_is_rejected() {
    PositionEstimator est(test_params());
    Truth t;
    t.p = 5.0f;
    run(est, t, 0.1f);
    TEST_ASSERT_EQUAL((int)FlowReject::kBodyRate, (int)est.state().reject);
}

void test_accelerometer_bias_is_estimated() {
    PositionEstimator est(test_params());
    Truth t;
    t.accel_bias_x = 0.3f;  // m/s^2 bias on the body x accelerometer
    run(est, t, 30.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.3f, est.state().bias_x);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, est.state().vx);
}

void test_velocity_coasts_on_accelerometer_during_flow_dropout() {
    PositionEstimator est(test_params());
    Truth t;
    t.vx = 0.2f;
    run(est, t, 1.0f);
    const float before = est.state().vx;
    t.squal = 0;  // flow lost, still flying at constant velocity (no acceleration)
    run(est, t, 0.3f);
    TEST_ASSERT_FALSE(est.state().flow_valid);
    // Without flow corrections the accelerometer integration keeps the value
    // (small drift from the bias estimate learnt from quantisation noise).
    TEST_ASSERT_FLOAT_WITHIN(0.03f, before, est.state().vx);
    TEST_ASSERT_FLOAT_WITHIN(0.08f, 0.2f, est.state().vx);
}

void test_position_integrates_in_world_frame_with_yaw() {
    PositionEstimator est(test_params());
    Truth t;
    t.yaw = 1.5707963f;  // facing +90 deg: forward motion = world +y
    t.vx  = 0.25f;
    run(est, t, 1.0f);  // let the velocity converge
    est.reset_position();
    run(est, t, 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, est.state().px);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.5f, est.state().py);
}

void test_axis_mapping_swap_and_sign() {
    EstimatorParams p = test_params();
    p.flow_swap_xy = true;
    p.flow_sign_x  = -1.0f;
    PositionEstimator est(p);
    // Sensor reports forward motion on its Y axis with inverted sign.
    FlowSample s;
    s.read_ok = true;
    s.squal   = 100;
    s.dx      = 0;
    s.dy      = -10;
    for (int i = 0; i < 4; i++) est.predict(0, 0, -1, 0, 0, 0, 0, 0, kDt);
    est.update_flow(s, 0.5f, 0.01f);
    TEST_ASSERT_TRUE(est.state().flow_vx > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, est.state().flow_vy);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_hover_still_gives_zero_velocity);
    RUN_TEST(test_constant_forward_velocity);
    RUN_TEST(test_constant_right_velocity);
    RUN_TEST(test_pure_rotation_is_compensated);
    RUN_TEST(test_height_is_tilt_compensated);
    RUN_TEST(test_stationary_tilt_gives_no_horizontal_acceleration);
    RUN_TEST(test_nose_up_reads_positive_fx);
    RUN_TEST(test_low_squal_invalidates_flow_with_hysteresis);
    RUN_TEST(test_height_window);
    RUN_TEST(test_no_range_is_rejected);
    RUN_TEST(test_fast_rotation_is_rejected);
    RUN_TEST(test_accelerometer_bias_is_estimated);
    RUN_TEST(test_velocity_coasts_on_accelerometer_during_flow_dropout);
    RUN_TEST(test_position_integrates_in_world_frame_with_yaw);
    RUN_TEST(test_axis_mapping_swap_and_sign);
    return UNITY_END();
}
