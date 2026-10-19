// Unit tests for lib/poshold/src/tof_step.* (pio test -e native).

#include <unity.h>

#include "tof_step.hpp"

using namespace poshold;

void setUp() {}
void tearDown() {}

void test_steady_height_gives_no_offset() {
    TofStepComp c;
    float off = 1.0f;
    for (int i = 0; i < 100; i++) off = c.update(true, true, 1.00f + 0.005f * (i % 3));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, off);
}

void test_flying_over_an_obstacle_keeps_the_height_continuous() {
    // 1.0 m over the floor, a 0.5 m bed passes below: the ToF reads 0.5 m for a while, then 1.0 m again
    TofStepComp c;
    float h = 0.0f;
    for (int i = 0; i < 30; i++) h = 1.00f + c.update(true, true, 1.00f);
    for (int i = 0; i < 60; i++) {
        h = 0.50f + c.update(true, true, 0.50f);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.00f, h);
    }
    for (int i = 0; i < 30; i++) {
        h = 1.00f + c.update(true, true, 1.00f);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.00f, h);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, c.offset());
}

void test_a_single_wrong_sample_cancels_itself() {
    TofStepComp c;
    for (int i = 0; i < 20; i++) c.update(true, true, 1.00f);
    c.update(true, true, 2.00f);  // spike
    const float off = c.update(true, true, 1.00f);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, off);
}

void test_an_edge_seen_in_two_steps_is_compensated() {
    // 1.0 m, the beam slides over a 0.45 m chair edge: -0.12 then -0.06 then -0.05 (as in logs/20261019_205655.csv)
    TofStepComp c;
    for (int i = 0; i < 20; i++) c.update(true, true, 1.00f);
    c.update(true, true, 0.88f);  // -0.12 is already a step
    c.update(true, true, 0.82f);  // -0.06 and
    const float off = c.update(true, true, 0.77f);  // -0.05: two small steps of 0.11 in a row
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.12f, off);  // only the first jump (> 0.07) was taken, the pair is below 0.12
    const float off2 = c.update(true, true, 0.71f);  // -0.06: the pair -0.05 + -0.06 = 0.11 is not > 0.12
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.12f, off2);
    const float off3 = c.update(true, true, 0.60f);  // -0.11 is a step
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.23f, off3);
}

void test_two_medium_jumps_in_a_row_are_one_edge() {
    TofStepComp c;
    for (int i = 0; i < 20; i++) c.update(true, true, 1.00f);
    c.update(true, true, 0.94f);  // -0.06, below the single step limit
    const float off = c.update(true, true, 0.875f);  // -0.065 (sum 0.125 > 0.12): one edge of 0.125
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.125f, off);
}

void test_a_real_climb_is_not_taken_for_an_edge() {
    // 1 m/s at 30 Hz is 3 cm per sample
    TofStepComp c;
    float off = 0.0f;
    for (int i = 0; i < 90; i++) off = c.update(true, true, 0.5f + 0.033f * i);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, off);
}

void test_invalid_samples_keep_the_offset() {
    TofStepComp c;
    for (int i = 0; i < 20; i++) c.update(true, true, 1.00f);
    c.update(true, true, 0.50f);  // bed
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.50f, c.offset());
    const float off = c.update(true, false, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.50f, off);
}

void test_inactive_clears_the_offset() {
    TofStepComp c;
    for (int i = 0; i < 20; i++) c.update(true, true, 1.00f);
    c.update(true, true, 0.50f);
    const float off = c.update(false, true, 0.50f);  // landing / on the ground
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, off);
}

void test_offset_saturates_and_reports_it() {
    TofStepComp c;
    c.update(true, true, 3.0f);
    c.update(true, true, 2.5f);
    c.update(true, true, 2.0f);
    c.update(true, true, 1.5f);
    c.update(true, true, 1.0f);
    c.update(true, true, 0.5f);  // a staircase of 0.5 m steps: 2.5 m of offset
    TEST_ASSERT_TRUE(c.saturated());
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.5f, c.offset());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_steady_height_gives_no_offset);
    RUN_TEST(test_flying_over_an_obstacle_keeps_the_height_continuous);
    RUN_TEST(test_a_single_wrong_sample_cancels_itself);
    RUN_TEST(test_an_edge_seen_in_two_steps_is_compensated);
    RUN_TEST(test_two_medium_jumps_in_a_row_are_one_edge);
    RUN_TEST(test_a_real_climb_is_not_taken_for_an_edge);
    RUN_TEST(test_invalid_samples_keep_the_offset);
    RUN_TEST(test_inactive_clears_the_offset);
    RUN_TEST(test_offset_saturates_and_reports_it);
    return UNITY_END();
}
