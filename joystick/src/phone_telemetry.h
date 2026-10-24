// SPDX-License-Identifier: MIT
//
// [POSHOLD] Live telemetry on a phone: the joystick opens its own Wi-Fi access point and serves a page
// (joystick/web/telemetry.html) that receives the drone telemetry frames over a WebSocket and records them
// to CSV (same columns as tools/telemetry_logger.py). Off by default: nothing is started until the screen
// button is held on the setup screen, so normal flights are unchanged. Switching it off restarts the
// joystick. Control stays on the joystick; the page cannot send anything to the drone.
// See docs/13-telemetria-telefon.md.

#ifndef PHONE_TELEMETRY_H
#define PHONE_TELEMETRY_H

#include <stddef.h>
#include <stdint.h>

// Starts the access point on the given ESP-NOW channel plus the web/WebSocket task. Returns false on failure.
bool phone_telemetry_start(uint8_t channel);
bool phone_telemetry_active(void);

// Called from the Wi-Fi task with a valid telemetry frame; never blocks, drops the frame if the queue is full.
void phone_telemetry_push(const uint8_t *data, size_t len);

#endif
