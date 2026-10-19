// Unit tests for lib/poshold/src/poshold_supervisor.* (pio test -e native).

#include <unity.h>

#include "poshold_supervisor.hpp"

using namespace poshold;

void setUp() {}
void tearDown() {}

namespace {

constexpr float kDt = 0.0025f;

SupervisorInput flying_ok() {
    SupervisorInput in;
    in.poshold_requested = true;
    in.flying            = true;
    in.auto_alt          = true;
    in.takeoff_done      = true;
    in.flow_sensor_ok    = true;
    in.flow_calibrated   = true;
    in.flow_valid        = true;
    in.height            = 0.5f;
    in.vbat              = 3.9f;
    in.dt                = kDt;
    return in;
}

SupervisorOutput run(PosHoldSupervisor& s, const SupervisorInput& in, float seconds) {
    SupervisorOutput o;
    const int n = (int)(seconds / kDt + 0.5f);
    for (int i = 0; i < n; i++) o = s.update(in);
    return o;
}

}  // namespace

void test_engages_when_ready() {
    PosHoldSupervisor s;
    SupervisorOutput o = s.update(flying_ok());  // Idle -> WaitEngage
    TEST_ASSERT_FALSE(o.poshold_active);
    o = s.update(flying_ok());
    TEST_ASSERT_TRUE(o.engage_now);
    TEST_ASSERT_TRUE(o.poshold_active);
    o = s.update(flying_ok());
    TEST_ASSERT_FALSE(o.engage_now);  // one-shot
    TEST_ASSERT_EQUAL((int)Reason::kNone, (int)o.reason);
}

void test_waits_for_takeoff_and_height() {
    PosHoldSupervisor s;
    SupervisorInput in = flying_ok();
    in.takeoff_done    = false;
    in.height          = 0.05f;
    SupervisorOutput o = run(s, in, 5.0f);  // no timeout before take-off is done
    TEST_ASSERT_EQUAL((int)PhState::kWaitEngage, (int)o.state);
    in.takeoff_done = true;
    in.height       = 0.4f;
    o               = run(s, in, 0.01f);
    TEST_ASSERT_TRUE(o.poshold_active);
}

void test_no_sensor_or_calibration_falls_back_with_reason() {
    PosHoldSupervisor s;
    SupervisorInput in  = flying_ok();
    in.flow_calibrated  = false;
    s.update(in);
    SupervisorOutput o = s.update(in);
    TEST_ASSERT_TRUE(o.fallback_event);
    TEST_ASSERT_EQUAL((int)PhState::kFallback, (int)o.state);
    TEST_ASSERT_EQUAL((int)Reason::kFlowNotCalibrated, (int)o.reason);

    PosHoldSupervisor s2;
    in                = flying_ok();
    in.flow_sensor_ok = false;
    s2.update(in);
    o = s2.update(in);
    TEST_ASSERT_EQUAL((int)Reason::kFlowSensorMissing, (int)o.reason);
}

void test_engage_timeout() {
    PosHoldSupervisor s;
    SupervisorInput in = flying_ok();
    in.flow_valid      = false;
    SupervisorOutput o = run(s, in, 2.9f);
    TEST_ASSERT_EQUAL((int)PhState::kWaitEngage, (int)o.state);
    o = run(s, in, 0.2f);
    TEST_ASSERT_EQUAL((int)PhState::kFallback, (int)o.state);
    TEST_ASSERT_EQUAL((int)Reason::kFlowInvalid, (int)o.reason);
}

void test_flow_loss_falls_back_after_grace_and_latches() {
    PosHoldSupervisor s;
    run(s, flying_ok(), 0.1f);
    SupervisorInput in = flying_ok();
    in.flow_valid      = false;
    SupervisorOutput o = run(s, in, 0.15f);
    TEST_ASSERT_TRUE(o.poshold_active);  // grace 0.2 s
    o = run(s, in, 0.1f);
    TEST_ASSERT_FALSE(o.poshold_active);
    TEST_ASSERT_EQUAL((int)Reason::kFlowInvalid, (int)o.reason);
    // flow recovers: no automatic re-engage in the air
    o = run(s, flying_ok(), 2.0f);
    TEST_ASSERT_FALSE(o.poshold_active);
    TEST_ASSERT_EQUAL((int)PhState::kFallback, (int)o.state);
    // after landing everything is reset
    SupervisorInput ground = flying_ok();
    ground.flying          = false;
    o                      = s.update(ground);
    TEST_ASSERT_EQUAL((int)PhState::kIdle, (int)o.state);
}

void test_brief_flow_glitch_is_tolerated() {
    PosHoldSupervisor s;
    run(s, flying_ok(), 0.1f);
    SupervisorInput bad = flying_ok();
    bad.flow_valid      = false;
    for (int k = 0; k < 10; k++) {
        run(s, bad, 0.1f);
        run(s, flying_ok(), 0.1f);
    }
    TEST_ASSERT_EQUAL((int)PhState::kActive, (int)s.state());
}

void test_height_out_of_window_falls_back() {
    PosHoldSupervisor s;
    run(s, flying_ok(), 0.1f);
    SupervisorInput in = flying_ok();
    in.height          = 1.8f;
    SupervisorOutput o = run(s, in, 0.3f);
    TEST_ASSERT_EQUAL((int)Reason::kHeightOutOfRange, (int)o.reason);
}

void test_pilot_switch_to_stable_deactivates_without_alarm() {
    PosHoldSupervisor s;
    run(s, flying_ok(), 0.1f);
    SupervisorInput in    = flying_ok();
    in.poshold_requested  = false;
    SupervisorOutput o    = s.update(in);
    TEST_ASSERT_FALSE(o.poshold_active);
    TEST_ASSERT_FALSE(o.fallback_event);
    TEST_ASSERT_EQUAL((int)PhState::kIdle, (int)o.state);
}

void test_link_loss_hover_then_landing_with_auto_alt() {
    PosHoldSupervisor s;
    run(s, flying_ok(), 0.1f);
    SupervisorInput in = flying_ok();
    in.link_age        = 0.5f;
    SupervisorOutput o = s.update(in);
    TEST_ASSERT_TRUE(o.link_lost);
    TEST_ASSERT_TRUE(o.neutral_sticks);
    TEST_ASSERT_FALSE(o.allow_link_landing);
    TEST_ASSERT_TRUE(o.poshold_active);  // keeps holding position while hovering
    TEST_ASSERT_EQUAL((int)Reason::kLinkLost, (int)o.reason);
    in.link_age = 3.2f;
    o           = s.update(in);
    TEST_ASSERT_TRUE(o.allow_link_landing);
    in.link_age = 0.0f;  // link back
    o           = s.update(in);
    TEST_ASSERT_FALSE(o.link_lost);
    TEST_ASSERT_FALSE(o.neutral_sticks);
}

void test_link_loss_landing_blocked_at_upstream_timeout() {
    PosHoldSupervisor s;
    run(s, flying_ok(), 0.1f);
    SupervisorInput in = flying_ok();
    in.link_age        = 40 * 0.0025f;  // upstream rc_isconnected() trips here
    SupervisorOutput o = s.update(in);
    TEST_ASSERT_FALSE(o.allow_link_landing);
    in.link_age = 3.0f;
    o           = s.update(in);
    TEST_ASSERT_FALSE(o.allow_link_landing);
    in.link_age = 3.2f;
    o           = s.update(in);
    TEST_ASSERT_TRUE(o.allow_link_landing);
}

void test_link_loss_manual_alt_lands_immediately() {
    PosHoldSupervisor s;
    SupervisorInput in   = flying_ok();
    in.poshold_requested = false;
    in.auto_alt          = false;
    in.link_age          = 0.2f;
    const SupervisorOutput o = s.update(in);
    TEST_ASSERT_TRUE(o.link_lost);
    TEST_ASSERT_FALSE(o.neutral_sticks);
    TEST_ASSERT_TRUE(o.allow_link_landing);
}

void test_low_battery_requests_landing_in_any_mode() {
    PosHoldSupervisor s;
    SupervisorInput in   = flying_ok();
    in.poshold_requested = false;  // Stable
    run(s, in, 1.0f);
    in.vbat            = 3.2f;
    SupervisorOutput o = run(s, in, 3.0f);  // filter (1 s) + 2 s below threshold
    TEST_ASSERT_FALSE(o.request_landing);
    o = run(s, in, 2.0f);
    TEST_ASSERT_TRUE(o.request_landing);
    TEST_ASSERT_TRUE(o.low_battery_warning);
    TEST_ASSERT_EQUAL((int)Reason::kLowBattery, (int)o.reason);
    in.vbat = 3.9f;  // voltage recovers after load drops: landing stays requested
    o       = run(s, in, 2.0f);
    TEST_ASSERT_TRUE(o.request_landing);
}

void test_untrusted_tof_requests_landing_and_latches() {
    PosHoldSupervisor s;
    SupervisorInput in = flying_ok();
    run(s, in, 1.0f);
    in.tof_untrusted     = true;
    SupervisorOutput o   = s.update(in);
    TEST_ASSERT_TRUE(o.request_landing);
    TEST_ASSERT_EQUAL((int)Reason::kToFUnreliable, (int)o.reason);
    in.tof_untrusted = false;  // the ToF seems fine again: the landing stays requested
    o                = run(s, in, 1.0f);
    TEST_ASSERT_TRUE(o.request_landing);
}

void test_short_voltage_dip_does_not_land() {
    PosHoldSupervisor s;
    SupervisorInput in = flying_ok();
    run(s, in, 1.0f);
    in.vbat = 3.0f;
    run(s, in, 0.3f);  // short dip (e.g. punch-out)
    in.vbat            = 3.9f;
    SupervisorOutput o = run(s, in, 5.0f);
    TEST_ASSERT_FALSE(o.request_landing);
}

void test_geofence_lands() {
    PosHoldSupervisor s;
    run(s, flying_ok(), 0.1f);
    SupervisorInput in = flying_ok();
    in.distance        = 2.6f;
    SupervisorOutput o = s.update(in);
    TEST_ASSERT_TRUE(o.request_landing);
    TEST_ASSERT_FALSE(o.poshold_active);
    TEST_ASSERT_EQUAL((int)Reason::kGeofence, (int)o.reason);
}

void test_landing_near_ground_deactivates_silently() {
    PosHoldSupervisor s;
    run(s, flying_ok(), 0.1f);
    SupervisorInput in = flying_ok();
    in.flying          = false;
    in.landing         = true;
    in.height          = 0.3f;
    SupervisorOutput o = s.update(in);
    TEST_ASSERT_TRUE(o.poshold_active);  // keeps holding XY while descending
    in.height = 0.08f;
    o         = s.update(in);
    TEST_ASSERT_FALSE(o.poshold_active);
    TEST_ASSERT_FALSE(o.fallback_event);
    TEST_ASSERT_EQUAL((int)Reason::kNone, (int)o.reason);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_engages_when_ready);
    RUN_TEST(test_waits_for_takeoff_and_height);
    RUN_TEST(test_no_sensor_or_calibration_falls_back_with_reason);
    RUN_TEST(test_engage_timeout);
    RUN_TEST(test_flow_loss_falls_back_after_grace_and_latches);
    RUN_TEST(test_brief_flow_glitch_is_tolerated);
    RUN_TEST(test_height_out_of_window_falls_back);
    RUN_TEST(test_pilot_switch_to_stable_deactivates_without_alarm);
    RUN_TEST(test_link_loss_hover_then_landing_with_auto_alt);
    RUN_TEST(test_link_loss_landing_blocked_at_upstream_timeout);
    RUN_TEST(test_link_loss_manual_alt_lands_immediately);
    RUN_TEST(test_low_battery_requests_landing_in_any_mode);
    RUN_TEST(test_untrusted_tof_requests_landing_and_latches);
    RUN_TEST(test_short_voltage_dip_does_not_land);
    RUN_TEST(test_geofence_lands);
    RUN_TEST(test_landing_near_ground_deactivates_silently);
    return UNITY_END();
}
