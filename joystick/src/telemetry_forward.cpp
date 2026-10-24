// SPDX-License-Identifier: MIT
//
// [POSHOLD] See telemetry_forward.h.

#include "telemetry_forward.h"

#include <Arduino.h>
#include <telemetry_frame.hpp>

#include "phone_telemetry.h"

bool telemetry_forward(const uint8_t *data, int len) {
    if (len < 2 || data[0] != stampfly_link::kSync0 || data[1] != stampfly_link::kSync1) return false;
    if (!stampfly_link::is_valid_frame(data, (size_t)len)) return true;
    phone_telemetry_push(data, (size_t)len);  // no-op until the phone Wi-Fi is switched on
    // Called from the Wi-Fi task: never block, drop the frame if USB is busy.
    if (USBSerial.availableForWrite() >= len) USBSerial.write(data, (size_t)len);
    return true;
}
