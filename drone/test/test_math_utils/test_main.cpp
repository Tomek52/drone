// Unit tests for lib/poshold/src/math_utils.hpp (run with: pio test -e native)

#include <unity.h>
#include "math_utils.hpp"

using namespace poshold;

void setUp() {}
void tearDown() {}

void test_clampf() {
    TEST_ASSERT_EQUAL_FLOAT(1.0f, clampf(5.0f, -1.0f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, clampf(-5.0f, -1.0f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.25f, clampf(0.25f, -1.0f, 1.0f));
}

void test_deadband_zero_inside_band() {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, deadband(0.05f, 0.1f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, deadband(-0.1f, 0.1f));
}

void test_deadband_rescales_to_full_range() {
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, deadband(1.0f, 0.2f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, deadband(-1.0f, 0.2f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, deadband(0.6f, 0.2f));
}

void test_wrap_pi() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, wrap_pi(2.0f * kPi));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -kPi / 2.0f, wrap_pi(3.0f * kPi / 2.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, kPi, wrap_pi(-kPi));
}

void test_rotate2d_quarter_turn() {
    float x, y;
    rotate2d(1.0f, 0.0f, kPi / 2.0f, &x, &y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, y);
}

void test_limit_norm2d() {
    float x = 3.0f, y = 4.0f;
    limit_norm2d(&x, &y, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.6f, x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.8f, y);
    float a = 0.1f, b = 0.1f;
    limit_norm2d(&a, &b, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.1f, a);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_clampf);
    RUN_TEST(test_deadband_zero_inside_band);
    RUN_TEST(test_deadband_rescales_to_full_range);
    RUN_TEST(test_wrap_pi);
    RUN_TEST(test_rotate2d_quarter_turn);
    RUN_TEST(test_limit_norm2d);
    return UNITY_END();
}
