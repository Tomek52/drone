// SPDX-License-Identifier: MIT
//
// [POSHOLD] Forwards binary telemetry frames received from the drone over
// ESP-NOW to the joystick's USB serial port, so a PC connected to the
// joystick can log flights (tools/telemetry_logger.py).

#ifndef TELEMETRY_FORWARD_H
#define TELEMETRY_FORWARD_H

#include <stddef.h>
#include <stdint.h>

// Returns true if the packet was a telemetry frame (and was forwarded or
// dropped); the caller should then stop processing it.
bool telemetry_forward(const uint8_t *data, int len);

#endif
