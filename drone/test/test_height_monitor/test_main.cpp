// Unit tests for lib/poshold/src/height_monitor.* (pio test -e native).

#include <unity.h>

#include "height_monitor.hpp"

using namespace poshold;

void setUp() {}
void tearDown() {}

namespace {

constexpr float kDt = 1.0f / 30.0f;  // ToF sample period

// Runs the monitor for `seconds`; the barometer reads `baro`, the ToF `tof` (valid or not).
HeightMonitorOutput run(HeightMonitor& m, float seconds, bool tof_valid, float tof, float baro) {
    HeightMonitorOutput o;
    for (float t = 0.0f; t < seconds; t += kDt) o = m.update(kDt, tof_valid, tof, true, baro);
    return o;
}

}  // namespace

void test_stays_trusted_when_sensors_agree() {
    HeightMonitor m;
    const HeightMonitorOutput o = run(m, 5.0f, true, 0.50f, 0.48f);
    TEST_ASSERT_TRUE(o.trusted);
    TEST_ASSERT_TRUE(o.initialised);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.50f, o.predicted_m);
}

void test_noisy_barometer_does_not_cause_false_alarms() {
    // single barometer samples are ~12 cm noisy; the ToF is steady at 0.5 m
    HeightMonitor m;
    HeightMonitorOutput o;
    unsigned seed = 12345u;
    for (float t = 0.0f; t < 30.0f; t += kDt) {
        seed          = seed * 1664525u + 1013904223u;
        const float n = ((int)((seed >> 16) & 0xFFFF) - 32768) / 32768.0f * 0.30f;  // +-0.30 m uniform
        o             = m.update(kDt, true, 0.50f, true, 0.50f + n);
        TEST_ASSERT_TRUE(o.trusted);
    }
}

void test_slow_barometer_drift_is_absorbed() {
    // thermal drift of 1.2 cm/s for 60 s, the ToF stays at 0.5 m
    HeightMonitor m;
    run(m, 1.0f, true, 0.50f, 0.50f);
    HeightMonitorOutput o;
    float baro = 0.50f;
    for (float t = 0.0f; t < 60.0f; t += kDt) {
        baro += 0.012f * kDt;
        o = m.update(kDt, true, 0.50f, true, baro);
    }
    TEST_ASSERT_TRUE(o.trusted);
}

void test_tof_stuck_while_drone_climbs_becomes_untrusted() {
    // upstream situation: the ToF keeps reporting 0.5 m (status valid) but the barometer shows the climb
    HeightMonitor m;
    run(m, 2.0f, true, 0.50f, 0.50f);
    HeightMonitorOutput o;
    float baro = 0.50f;
    bool untrusted_seen = false;
    for (float t = 0.0f; t < 5.0f; t += kDt) {
        baro += 0.5f * kDt;  // climbing at 0.5 m/s
        o = m.update(kDt, true, 0.50f, true, baro);
        if (!o.trusted) untrusted_seen = true;
    }
    TEST_ASSERT_TRUE(untrusted_seen);
    TEST_ASSERT_FALSE(o.trusted);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 3.0f, o.predicted_m);  // the barometer still tells the real height
}

void test_missing_tof_samples_become_untrusted_after_hold_time() {
    HeightMonitor m;
    run(m, 2.0f, true, 0.50f, 0.50f);
    HeightMonitorOutput o = run(m, 0.2f, false, 0.0f, 0.50f);
    TEST_ASSERT_TRUE(o.trusted);  // shorter than the hold time
    o = run(m, 0.5f, false, 0.0f, 0.50f);
    TEST_ASSERT_FALSE(o.trusted);
}

void test_regains_trust_only_after_a_sustained_agreement() {
    HeightMonitor m;
    run(m, 2.0f, true, 0.50f, 0.50f);
    run(m, 1.0f, false, 0.0f, 0.50f);
    TEST_ASSERT_FALSE(m.trusted());
    HeightMonitorOutput o = run(m, 0.5f, true, 0.50f, 0.50f);
    TEST_ASSERT_FALSE(o.trusted);  // not long enough yet
    o = run(m, 1.0f, true, 0.50f, 0.50f);
    TEST_ASSERT_TRUE(o.trusted);
}

void test_a_glitch_resets_the_regain_timer() {
    HeightMonitor m;
    run(m, 2.0f, true, 0.50f, 0.50f);
    run(m, 1.0f, false, 0.0f, 0.50f);
    run(m, 0.8f, true, 0.50f, 0.50f);
    run(m, 0.1f, true, 1.50f, 0.50f);  // disagrees again
    HeightMonitorOutput o = run(m, 0.5f, true, 0.50f, 0.50f);
    TEST_ASSERT_FALSE(o.trusted);
}

void test_no_barometer_never_distrusts_the_tof() {
    HeightMonitor m;
    HeightMonitorOutput o;
    for (float t = 0.0f; t < 5.0f; t += kDt) o = m.update(kDt, false, 0.0f, false, 0.0f);
    TEST_ASSERT_TRUE(o.trusted);
}

void test_not_initialised_until_a_valid_tof_sample() {
    HeightMonitor m;
    HeightMonitorOutput o = run(m, 1.0f, false, 0.0f, 0.1f);
    TEST_ASSERT_FALSE(o.initialised);
    TEST_ASSERT_TRUE(o.trusted);
    o = run(m, kDt, true, 0.5f, 0.1f);
    TEST_ASSERT_TRUE(o.initialised);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_stays_trusted_when_sensors_agree);
    RUN_TEST(test_noisy_barometer_does_not_cause_false_alarms);
    RUN_TEST(test_slow_barometer_drift_is_absorbed);
    RUN_TEST(test_tof_stuck_while_drone_climbs_becomes_untrusted);
    RUN_TEST(test_missing_tof_samples_become_untrusted_after_hold_time);
    RUN_TEST(test_regains_trust_only_after_a_sustained_agreement);
    RUN_TEST(test_a_glitch_resets_the_regain_timer);
    RUN_TEST(test_no_barometer_never_distrusts_the_tof);
    RUN_TEST(test_not_initialised_until_a_valid_tof_sample);
    return UNITY_END();
}
