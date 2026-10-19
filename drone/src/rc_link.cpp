// SPDX-License-Identifier: MIT
//
// [POSHOLD] See rc_link.hpp.

#include "rc_link.hpp"

#include <Arduino.h>
#include <esp_now.h>

#include "alerts.hpp"
#include "telemetry_stream.hpp"
#include "flight_control.hpp"
#include "led.hpp"

using namespace stampfly_link;

extern uint8_t TelemAddr[6];  // rc.cpp: MAC of the joystick (set on first packet)
void motor_stop(void);        // flight_control.cpp

namespace {

constexpr uint16_t kStatusDivider = 80;  // 400 Hz / 80 = 5 Hz
constexpr uint32_t kMismatchColorA = RED;
constexpr uint32_t kMismatchColorB = BLUE;

// Written by the ESP-NOW callback (Wi-Fi task), read by the control loop.
volatile bool g_proto_ok       = false;
volatile bool g_mismatch_seen  = false;
volatile uint8_t g_peer_version = 0;
volatile uint8_t g_mode         = (uint8_t)FlightMode::kStable;
volatile uint8_t g_flags        = 0;

// Status reported to the joystick.
bool g_status_override   = false;
FlightMode g_active_mode = FlightMode::kStable;
uint8_t g_extra_flags    = 0;
Reason g_reason          = Reason::kNone;

uint32_t g_counter        = 0;
bool g_mismatch_signalled = false;
Reason g_kill_reason      = Reason::kNone;
uint32_t g_kill_until_ms  = 0;

bool telem_addr_known(void) {
    for (int i = 0; i < 6; i++)
        if (TelemAddr[i]) return true;
    return false;
}

void send_status(void) {
    if (!telem_addr_known()) return;
    DroneStatus s;
    s.version = kRcProtocolVersion;
    FlightMode active;
    uint8_t flags;
    Reason reason;
    if (g_status_override) {
        active = g_active_mode;
        flags  = g_extra_flags;
        reason = g_reason;
    } else {
        // Firmware without the Position Hold controller: Position Hold
        // requests are flown as Stable (byte 21 of the packet is "angle").
        const FlightMode req = rc_link_requested_mode();
        active               = req == FlightMode::kPosHold ? FlightMode::kStable : req;
        flags                = 0;
        reason               = req == FlightMode::kPosHold ? Reason::kNotImplemented : Reason::kNone;
    }
    if (g_proto_ok) flags |= kStatusRcOk;
    if (!g_proto_ok && g_mismatch_seen) reason = Reason::kProtocolMismatch;
    if (millis() < g_kill_until_ms) reason = g_kill_reason;
    s.flags       = flags;
    s.active_mode = active;
    s.reason      = (uint8_t)reason;
    uint8_t buf[kDroneStatusLen];
    encode_status(s, buf);
    esp_now_send(TelemAddr, buf, sizeof(buf));
}

}  // namespace

bool rc_link_accept(const uint8_t* data, int len) {
    RcExtension ext;
    const RcDecode r = decode_rc(data, (size_t)len, &ext);
    if (r == RcDecode::kOk) {
        g_peer_version = ext.version;
        g_mode         = (uint8_t)ext.flight_mode;
        g_flags        = ext.flags;
        g_proto_ok     = true;
        return true;
    }
    if (r == RcDecode::kVersionMismatch) {
        g_peer_version  = ext.version;
        g_proto_ok      = false;
        g_mismatch_seen = true;
        g_flags         = 0;
    }
    return false;
}

void rc_link_update(void) {
    // Kill switch: stop the motors immediately in every flying state.
    if (rc_link_kill_active() && (Mode == FLIGHT_MODE || Mode == FLIP_MODE || Mode == AUTO_LANDING_MODE)) {
        motor_stop();
        Mode = PARKING_MODE;  // the PARKING branch keeps the motors off and resets the controllers
        alerts_play(Alert::kKill);
        g_kill_reason   = Reason::kKill;
        g_kill_until_ms = millis() + 3000;
        telemetry_text("[POSHOLD] KILL from joystick\r\n");
    }

    // Protocol mismatch: refuse to arm (sticks are ignored in rc.cpp) and signal it.
    const bool mismatch = !g_proto_ok && g_mismatch_seen;
    if (mismatch && Mode == PARKING_MODE) {
        if (!g_mismatch_signalled) {
            alerts_play(Alert::kProtocolMismatch);
            alerts_set_led_override(LedOwner::kProtocol, kMismatchColorA, kMismatchColorB, 250);
            telemetry_text("[POSHOLD] protocol mismatch: joystick v%u, drone v%u\r\n", g_peer_version,
                             kRcProtocolVersion);
            g_mismatch_signalled = true;
        }
    } else if (g_mismatch_signalled) {
        alerts_stop(Alert::kProtocolMismatch);
        alerts_clear_led_override(LedOwner::kProtocol);
        g_mismatch_signalled = false;
    }

    if (++g_counter % kStatusDivider == 0) send_status();
}

bool rc_link_protocol_ok(void) {
    return g_proto_ok;
}

uint8_t rc_link_peer_version(void) {
    return g_peer_version;
}

FlightMode rc_link_requested_mode(void) {
    return g_proto_ok ? (FlightMode)g_mode : FlightMode::kStable;
}

bool rc_link_abs_alt_requested(void) {
    return g_proto_ok && (g_flags & kRcFlagAbsAlt);
}

bool rc_link_kill_active(void) {
    return g_proto_ok && (g_flags & kRcFlagKill);
}

void rc_link_set_status(FlightMode active_mode, uint8_t extra_flags, Reason reason) {
    g_status_override = true;
    g_active_mode     = active_mode;
    g_extra_flags     = extra_flags;
    g_reason          = reason;
}
