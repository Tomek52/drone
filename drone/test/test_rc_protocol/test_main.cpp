// Unit tests for the shared control protocol and joystick logic
// (common/stampfly_link/src/rc_protocol.hpp, joystick_logic.hpp).

#include <unity.h>
#include <joystick_logic.hpp>
#include <rc_protocol.hpp>

using namespace stampfly_link;

void setUp() {}
void tearDown() {}

static void fill_v1(uint8_t* b) {
    for (int i = 0; i < 24; i++) b[i] = (uint8_t)(i * 7 + 3);
}

void test_v2_roundtrip() {
    uint8_t v1[24], pkt[kRcPacketLen];
    fill_v1(v1);
    TEST_ASSERT_EQUAL(kRcPacketLen, encode_rc_v2(v1, FlightMode::kPosHold, kRcFlagKill, pkt));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(v1, pkt, 24);  // v1 layout preserved
    RcExtension ext;
    TEST_ASSERT_EQUAL((int)RcDecode::kOk, (int)decode_rc(pkt, sizeof(pkt), &ext));
    TEST_ASSERT_EQUAL(kRcProtocolVersion, ext.version);
    TEST_ASSERT_EQUAL((int)FlightMode::kPosHold, (int)ext.flight_mode);
    TEST_ASSERT_EQUAL(kRcFlagKill, ext.flags);
}

void test_v1_packet_is_version_mismatch() {
    uint8_t v1pkt[kRcPacketV1Len];
    fill_v1(v1pkt);
    v1pkt[24] = sum8(v1pkt, 24);  // valid v1 checksum
    RcExtension ext;
    TEST_ASSERT_EQUAL((int)RcDecode::kVersionMismatch, (int)decode_rc(v1pkt, sizeof(v1pkt), &ext));
    TEST_ASSERT_EQUAL(1, ext.version);
}

void test_other_version_is_mismatch() {
    uint8_t v1[24], pkt[kRcPacketLen];
    fill_v1(v1);
    encode_rc_v2(v1, FlightMode::kStable, 0, pkt);
    pkt[kRcOffVersion] = 3;
    pkt[kRcOffChecksum] = sum8(pkt, kRcOffChecksum);
    RcExtension ext;
    TEST_ASSERT_EQUAL((int)RcDecode::kVersionMismatch, (int)decode_rc(pkt, sizeof(pkt), &ext));
    TEST_ASSERT_EQUAL(3, ext.version);
}

void test_bad_checksum_and_length() {
    uint8_t v1[24], pkt[kRcPacketLen];
    fill_v1(v1);
    encode_rc_v2(v1, FlightMode::kStable, 0, pkt);
    pkt[5] ^= 0x10;
    RcExtension ext;
    TEST_ASSERT_EQUAL((int)RcDecode::kBadChecksum, (int)decode_rc(pkt, sizeof(pkt), &ext));
    TEST_ASSERT_EQUAL((int)RcDecode::kBadLength, (int)decode_rc(pkt, 11, &ext));
}

void test_unknown_flight_mode_falls_back_to_stable() {
    uint8_t v1[24], pkt[kRcPacketLen];
    fill_v1(v1);
    encode_rc_v2(v1, FlightMode::kStable, 0, pkt);
    pkt[kRcOffFlightMode] = 9;
    pkt[kRcOffChecksum]   = sum8(pkt, kRcOffChecksum);
    RcExtension ext;
    TEST_ASSERT_EQUAL((int)RcDecode::kOk, (int)decode_rc(pkt, sizeof(pkt), &ext));
    TEST_ASSERT_EQUAL((int)FlightMode::kStable, (int)ext.flight_mode);
}

void test_status_roundtrip() {
    DroneStatus s;
    s.version     = kRcProtocolVersion;
    s.flags       = kStatusRcOk | kStatusPosHoldActive;
    s.active_mode = FlightMode::kPosHold;
    s.reason      = 4;
    uint8_t buf[kDroneStatusLen];
    TEST_ASSERT_EQUAL(kDroneStatusLen, encode_status(s, buf));
    DroneStatus d;
    TEST_ASSERT_TRUE(decode_status(buf, sizeof(buf), &d));
    TEST_ASSERT_EQUAL(s.flags, d.flags);
    TEST_ASSERT_EQUAL((int)FlightMode::kPosHold, (int)d.active_mode);
    TEST_ASSERT_EQUAL(4, d.reason);
    buf[3] ^= 1;
    TEST_ASSERT_FALSE(decode_status(buf, sizeof(buf), &d));
}

void test_status_does_not_collide_with_other_packets() {
    // Upstream telemetry starts with 88 88 / 99 99, binary telemetry with A5 5A.
    const uint8_t telem[kDroneStatusLen] = {88, 88, 0, 0, 0, 0, 0, 0};
    TEST_ASSERT_FALSE(is_status_packet(telem, sizeof(telem)));
}

void test_mode_cycle_on_ground() {
    FlightModeSelector m;
    TEST_ASSERT_EQUAL((int)FlightMode::kStable, (int)m.mode());
    m.cycle_mode_on_ground();
    TEST_ASSERT_EQUAL((int)FlightMode::kSport, (int)m.mode());
    TEST_ASSERT_EQUAL(kCtrlRate, m.control_mode_byte());
    m.cycle_mode_on_ground();
    TEST_ASSERT_EQUAL((int)FlightMode::kPosHold, (int)m.mode());
    TEST_ASSERT_EQUAL(kCtrlAngle, m.control_mode_byte());  // old drones see Stable
    m.cycle_mode_on_ground();
    TEST_ASSERT_EQUAL((int)FlightMode::kStable, (int)m.mode());
}

void test_altitude_cycles_auto_manual_abs() {
    FlightModeSelector m;
    TEST_ASSERT_TRUE(m.alt_auto());
    TEST_ASSERT_FALSE(m.alt_abs());
    m.toggle_alt_on_ground();  // manual
    TEST_ASSERT_FALSE(m.alt_auto());
    TEST_ASSERT_FALSE(m.alt_abs());
    m.toggle_alt_on_ground();  // ABS: upstream altitude control stays on
    TEST_ASSERT_TRUE(m.alt_auto());
    TEST_ASSERT_TRUE(m.alt_abs());
    m.toggle_alt_on_ground();  // back to auto
    TEST_ASSERT_TRUE(m.alt_auto());
    TEST_ASSERT_FALSE(m.alt_abs());
}

void test_poshold_and_abs_are_exclusive() {
    FlightModeSelector m;
    m.toggle_alt_on_ground();
    m.toggle_alt_on_ground();  // ABS
    m.cycle_mode_on_ground();
    m.cycle_mode_on_ground();  // -> Position Hold forces Auto altitude
    TEST_ASSERT_EQUAL((int)FlightMode::kPosHold, (int)m.mode());
    TEST_ASSERT_FALSE(m.alt_abs());
    m.toggle_alt_on_ground();  // Manual requested -> Stable
    TEST_ASSERT_EQUAL((int)FlightMode::kStable, (int)m.mode());
    m.set(FlightMode::kSport, true);  // Manual -> Auto
    m.toggle_alt_on_ground();         // Manual
    m.toggle_alt_on_ground();         // Abs
    TEST_ASSERT_TRUE(m.alt_abs());
    m.set(FlightMode::kSport, true);  // keeps ABS
    TEST_ASSERT_TRUE(m.alt_abs());
}

void test_poshold_forces_auto_altitude() {
    FlightModeSelector m;
    m.toggle_alt_on_ground();  // manual altitude
    TEST_ASSERT_FALSE(m.alt_auto());
    m.cycle_mode_on_ground();
    m.cycle_mode_on_ground();  // -> Position Hold
    TEST_ASSERT_TRUE(m.alt_auto());
    m.toggle_alt_on_ground();  // manual altitude requested -> back to Stable
    TEST_ASSERT_FALSE(m.alt_auto());
    TEST_ASSERT_EQUAL((int)FlightMode::kStable, (int)m.mode());
}

void test_in_flight_only_poshold_to_stable() {
    FlightModeSelector m;
    TEST_ASSERT_FALSE(m.left_button_in_flight());  // Stable: not consumed (upstream action)
    m.set(FlightMode::kPosHold, true);
    TEST_ASSERT_TRUE(m.left_button_in_flight());
    TEST_ASSERT_EQUAL((int)FlightMode::kStable, (int)m.mode());
    m.set(FlightMode::kSport, true);
    TEST_ASSERT_FALSE(m.left_button_in_flight());
    TEST_ASSERT_EQUAL((int)FlightMode::kSport, (int)m.mode());
}

void test_kill_switch_needs_hold_and_lingers() {
    KillSwitch k(0.5f, 0.5f);
    const float dt = 0.01f;
    for (int i = 0; i < 40; i++) TEST_ASSERT_FALSE(k.update(true, dt));  // 0.4 s
    for (int i = 0; i < 11; i++) k.update(true, dt);                      // > 0.5 s
    TEST_ASSERT_TRUE(k.active());
    TEST_ASSERT_TRUE(k.just_activated());
    TEST_ASSERT_FALSE(k.just_activated());
    for (int i = 0; i < 40; i++) TEST_ASSERT_TRUE(k.update(false, dt));  // lingers
    for (int i = 0; i < 20; i++) k.update(false, dt);
    TEST_ASSERT_FALSE(k.active());
}

void test_kill_switch_short_presses_do_nothing() {
    KillSwitch k(0.5f, 0.5f);
    for (int r = 0; r < 10; r++) {
        for (int i = 0; i < 30; i++) TEST_ASSERT_FALSE(k.update(true, 0.01f));
        for (int i = 0; i < 5; i++) TEST_ASSERT_FALSE(k.update(false, 0.01f));
    }
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_v2_roundtrip);
    RUN_TEST(test_v1_packet_is_version_mismatch);
    RUN_TEST(test_other_version_is_mismatch);
    RUN_TEST(test_bad_checksum_and_length);
    RUN_TEST(test_unknown_flight_mode_falls_back_to_stable);
    RUN_TEST(test_status_roundtrip);
    RUN_TEST(test_status_does_not_collide_with_other_packets);
    RUN_TEST(test_mode_cycle_on_ground);
    RUN_TEST(test_altitude_cycles_auto_manual_abs);
    RUN_TEST(test_poshold_and_abs_are_exclusive);
    RUN_TEST(test_poshold_forces_auto_altitude);
    RUN_TEST(test_in_flight_only_poshold_to_stable);
    RUN_TEST(test_kill_switch_needs_hold_and_lingers);
    RUN_TEST(test_kill_switch_short_presses_do_nothing);
    return UNITY_END();
}
