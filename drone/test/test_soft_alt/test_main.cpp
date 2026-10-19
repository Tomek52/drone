// Unit tests for lib/poshold/src/soft_alt.* (pio test -e native).

#include <unity.h>

#include <math.h>

#include "soft_alt.hpp"

using namespace poshold;

void setUp() {}
void tearDown() {}

namespace {

constexpr float kDt = 0.01f;

struct Plant {
    float h = 5.0f, v = 0.0f, thr = 0.0f;
    float gain = 14.0f, drag = 0.3f, lag = 0.1f, trim_error = 0.0f;
    void step(float u, float wind) {
        thr += (u - thr) * kDt / (lag + kDt);
        v += (gain * (thr - trim_error) - drag * v + wind) * kDt;
        h += v * kDt;
    }
};

}  // namespace

void test_inside_the_dead_zone_the_output_stays_zero() {
    SoftAlt c;
    float u = 1.0f;
    for (int i = 0; i < 1000; i++) u = c.update(kDt, 5.3f, 5.0f);  // 0.3 m below, dead zone is 0.5 m
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, u);
}

void test_the_sign_follows_the_error_and_is_limited() {
    SoftAlt c;
    float u = 0.0f;
    for (int i = 0; i < 3000; i++) u = c.update(kDt, 2.0f, 8.0f);  // far below the setpoint
    TEST_ASSERT_TRUE(u > 0.0f);
    TEST_ASSERT_TRUE(u <= config::kSoftUMax + 1e-6f);
    SoftAlt d;
    for (int i = 0; i < 3000; i++) u = d.update(kDt, 9.0f, 3.0f);
    TEST_ASSERT_TRUE(u < 0.0f);
    TEST_ASSERT_TRUE(u >= -config::kSoftUMax - 1e-6f);
}

void test_bumpless_entry_starts_at_the_previous_command() {
    SoftAlt c;
    c.reset(5.0f, 0.03f);
    const float u = c.update(kDt, 5.0f, 5.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.03f, u);
    SoftAlt d;
    d.reset(5.0f, 0.5f);  // an out of range request is limited
    TEST_ASSERT_TRUE(d.update(kDt, 5.0f, 5.0f) <= config::kSoftUMax + 1e-6f);
}

void test_closed_loop_holds_the_setpoint_with_a_trim_error() {
    // double integrator with a 3% duty trim error: the integral action has to take it out
    Plant p;
    p.h = 5.0f;
    p.trim_error = 0.03f;
    SoftAlt c;
    c.reset(p.h, 0.0f);
    float worst = 0.0f;
    for (int i = 0; i < 16000; i++) {
        const float u = c.update(kDt, p.h, 5.0f);
        p.step(u, 0.0f);
        if (i > 12000) worst = fmaxf(worst, fabsf(p.h - 5.0f));
    }
    TEST_ASSERT_TRUE(worst < 1.0f);
    TEST_ASSERT_TRUE(fabsf(p.v) < 0.2f);
}

void test_closed_loop_follows_a_step_without_oscillating() {
    Plant p;
    p.h = 5.0f;
    SoftAlt c;
    c.reset(p.h, 0.0f);
    float ref = 5.0f;
    float peak = 0.0f;
    for (int i = 0; i < 20000; i++) {
        if (i == 2000) ref = 8.0f;
        const float u = c.update(kDt, p.h, ref);
        p.step(u, 0.0f);
        if (i > 14000) peak = fmaxf(peak, fabsf(p.h - ref));
    }
    TEST_ASSERT_TRUE(peak < 1.0f);
    TEST_ASSERT_TRUE(p.h < 9.5f);  // no large overshoot
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_inside_the_dead_zone_the_output_stays_zero);
    RUN_TEST(test_the_sign_follows_the_error_and_is_limited);
    RUN_TEST(test_bumpless_entry_starts_at_the_previous_command);
    RUN_TEST(test_closed_loop_holds_the_setpoint_with_a_trim_error);
    RUN_TEST(test_closed_loop_follows_a_step_without_oscillating);
    return UNITY_END();
}
