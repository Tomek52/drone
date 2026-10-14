// SPDX-License-Identifier: MIT
//
// [POSHOLD] Control protocol v2 on the drone side (common/stampfly_link/
// src/rc_protocol.hpp): version check, flight mode and kill flag from the
// joystick, status packet back to the joystick.

#ifndef RC_LINK_HPP
#define RC_LINK_HPP

#include <stdint.h>
#include <rc_protocol.hpp>

// Called from the ESP-NOW receive callback (rc.cpp) after the MAC filter.
// Returns false if the packet must be ignored (wrong version, bad checksum).
bool rc_link_accept(const uint8_t* data, int len);

// Call once per control loop iteration, before the mode state machine:
// applies the kill switch, signals a protocol mismatch, sends the status.
void rc_link_update(void);

bool rc_link_protocol_ok(void);
uint8_t rc_link_peer_version(void);
stampfly_link::FlightMode rc_link_requested_mode(void);
bool rc_link_kill_active(void);

// Set by the Position Hold code: what the drone actually flies and why.
void rc_link_set_status(stampfly_link::FlightMode active_mode, uint8_t extra_flags, stampfly_link::Reason reason);

#endif
