// Unit tests for lib/poshold/src/abs_height.* (pio test -e native).

#include <unity.h>

#include "abs_height.hpp"

using namespace poshold;

void setUp() {}
void tearDown() {}

namespace {

constexpr float kDt    = 0.0025f;
constexpr float kWashM = 13.0f * 0.73f * 0.73f / 11.5f;  // apparent height added at hover thrust 0.73

float run(AbsHeight& a, float seconds, float baro, float thrust, bool tof_valid, float tof, bool parked) {
    float h = 0.0f;
    for (float t = 0.0f; t < seconds; t += kDt) h = a.update(kDt, true, baro, thrust, tof_valid, tof, parked);
    return h;
}

}  // namespace

void test_parked_height_follows_the_tof() {
    AbsHeight a;
    const float h = run(a, 1.0f, 0.0f, 0.0f, true, 0.03f, true);
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.03f, h);
}

void test_prop_wash_is_removed_at_hover_thrust() {
    AbsHeight a;
    run(a, 1.0f, 0.0f, 0.0f, true, 0.03f, true);
    const float h = run(a, 3.0f, 0.5f + kWashM, 0.73f, true, 0.53f, false);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.53f, h);
}

void test_obstacle_under_the_drone_does_not_move_the_height() {
    // 0.53 m over the floor; a bed 0.5 m high passes below: the ToF reads 0.03 m, the barometer is unchanged
    AbsHeight a;
    run(a, 1.0f, 0.0f, 0.0f, true, 0.03f, true);
    run(a, 3.0f, 0.5f + kWashM, 0.73f, true, 0.53f, false);
    const float h = run(a, 20.0f, 0.5f + kWashM, 0.73f, true, 0.03f, false);
    TEST_ASSERT_FLOAT_WITHIN(0.08f, 0.53f, h);
}

void test_blinded_tof_is_ignored() {
    AbsHeight a;
    run(a, 1.0f, 0.0f, 0.0f, true, 0.03f, true);
    run(a, 3.0f, 0.5f + kWashM, 0.73f, true, 0.53f, false);
    // the ToF is stuck at 0.5 m while the drone climbs 5 m by the barometer
    float baro = 0.5f + kWashM;
    float h    = 0.0f;
    for (float t = 0.0f; t < 10.0f; t += kDt) {
        baro += 0.5f * kDt;
        h = a.update(kDt, true, baro, 0.73f, true, 0.50f, false);
    }
    TEST_ASSERT_TRUE(h > 4.0f);
}

void test_slow_drift_is_learned_from_the_tof() {
    AbsHeight a;
    run(a, 1.0f, 0.0f, 0.0f, true, 0.03f, true);
    float baro = 0.5f + kWashM;
    float h    = 0.0f;
    for (float t = 0.0f; t < 120.0f; t += kDt) {
        baro += 0.01f * kDt;  // 1 cm/s thermal drift, the drone really stays at 0.53 m
        h = a.update(kDt, true, baro, 0.73f, true, 0.53f, false);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.30f, 0.53f, h);
}

void test_no_barometer_falls_back_to_the_tof() {
    AbsHeight a;
    float h = 0.0f;
    for (int i = 0; i < 10; i++) h = a.update(kDt, false, 0.0f, 0.5f, true, 0.42f, false);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.42f, h);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_parked_height_follows_the_tof);
    RUN_TEST(test_prop_wash_is_removed_at_hover_thrust);
    RUN_TEST(test_obstacle_under_the_drone_does_not_move_the_height);
    RUN_TEST(test_blinded_tof_is_ignored);
    RUN_TEST(test_slow_drift_is_learned_from_the_tof);
    RUN_TEST(test_no_barometer_falls_back_to_the_tof);
    return UNITY_END();
}
