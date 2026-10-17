// SPDX-License-Identifier: MIT
//
// [POSHOLD] Binary telemetry stream for tuning and analysis.
//
// Data frames (see common/stampfly_link/src/telemetry_frame.hpp) are sent:
//   - over USB serial (bench / tether tests),
//   - over ESP-NOW to the paired joystick, which forwards them to its own USB
//     port, so a PC connected to the joystick can log real flights.
// Both outputs are non-blocking: a frame is dropped if there is no room.
// tools/telemetry_logger.py decodes the stream into CSV.

#ifndef TELEMETRY_STREAM_HPP
#define TELEMETRY_STREAM_HPP

#include <stdint.h>

// Call once per 400 Hz control loop iteration (after the control step).
void telemetry_stream_update(void);

// Number of frames dropped because the USB TX buffer was full.
uint32_t telemetry_stream_usb_dropped(void);

#endif
