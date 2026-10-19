// SPDX-License-Identifier: MIT
//
// [POSHOLD] Binary telemetry stream, see telemetry_stream.hpp.

#include "telemetry_stream.hpp"

#include <Arduino.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <esp_now.h>
#include <telemetry_frame.hpp>

#include "flight_control.hpp"
#include "poshold_glue.hpp"
#include "rc_link.hpp"
#include "rc.hpp"
#include "sensor.hpp"
#include "baro.hpp"
#include "tof.hpp"

using stampfly_link::encode_data;
using stampfly_link::encode_schema_chunk;
using stampfly_link::kMaxFrameSize;

extern uint8_t TelemAddr[6];  // rc.cpp: MAC of the joystick (set on first packet)

namespace {

// Output rates as dividers of the 400 Hz control loop.
constexpr uint16_t kUsbDivider    = 8;    // 50 Hz
constexpr uint16_t kEspNowDivider = 16;   // 25 Hz (keeps radio airtime low)
constexpr uint16_t kSchemaDivider = 800;  // schema burst every 2 s
constexpr float kRadToDeg         = 57.29577951f;

struct Field {
    const char* name;
    float (*get)(void);
};

// Telemetry fields. Units are part of the name (deg, dps, m, mps, v, ...).
// New modules append their fields here; tools pick them up automatically
// from the schema frame.
const Field kFields[] = {
    {"mode", []() -> float { return (float)Mode; }},
    {"alt_mode", []() -> float { return (float)Throttle_control_mode; }},
    {"roll_deg", []() -> float { return (Roll_angle - Roll_angle_offset) * kRadToDeg; }},
    {"pitch_deg", []() -> float { return (Pitch_angle - Pitch_angle_offset) * kRadToDeg; }},
    {"yaw_deg", []() -> float { return (Yaw_angle - Yaw_angle_offset) * kRadToDeg; }},
    {"p_dps", []() -> float { return Roll_rate * kRadToDeg; }},
    {"q_dps", []() -> float { return Pitch_rate * kRadToDeg; }},
    {"r_dps", []() -> float { return Yaw_rate * kRadToDeg; }},
    {"roll_ref_deg", []() -> float { return Roll_angle_reference * kRadToDeg; }},
    {"pitch_ref_deg", []() -> float { return Pitch_angle_reference * kRadToDeg; }},
    {"p_ref_dps", []() -> float { return Roll_rate_reference * kRadToDeg; }},
    {"q_ref_dps", []() -> float { return Pitch_rate_reference * kRadToDeg; }},
    {"r_ref_dps", []() -> float { return Yaw_rate_reference * kRadToDeg; }},
    {"thrust_duty", []() -> float { return Thrust_command / BATTERY_VOLTAGE; }},
    {"alt_ref_m", []() -> float { return (float)Alt_ref; }},
    {"alt_est_m", []() -> float { return (float)Altitude2; }},
    {"alt_tof_m", []() -> float { return (float)Altitude; }},
    {"tof_raw_mm", []() -> float { return (float)RawRange; }},
    {"vz_mps", []() -> float { return (float)Alt_velocity; }},
    {"vz_ref_mps", []() -> float { return Z_dot_ref; }},
    {"vbat_v", []() -> float { return (float)Voltage; }},
    {"stick_thr", []() -> float { return (float)Stick[THROTTLE]; }},
    {"stick_roll", []() -> float { return (float)Stick[AILERON]; }},
    {"stick_pitch", []() -> float { return (float)Stick[ELEVATOR]; }},
    {"stick_yaw", []() -> float { return (float)Stick[RUDDER]; }},
    {"rc_age_loops", []() -> float { return (float)Connect_flag; }},
    {"loop_dt_s", []() -> float { return (float)Interval_time; }},
    // Position estimator (feat/position-estimator)
    {"flow_dx", []() -> float { return (float)poshold_last_flow().dx; }},
    {"flow_dy", []() -> float { return (float)poshold_last_flow().dy; }},
    {"flow_sum_x", []() -> float { return (float)poshold_flow_sum_x(); }},
    {"flow_sum_y", []() -> float { return (float)poshold_flow_sum_y(); }},
    {"flow_squal", []() -> float { return (float)poshold_last_flow().squal; }},
    {"flow_shutter", []() -> float { return (float)poshold_last_flow().shutter; }},
    {"flow_valid", []() -> float { return poshold_estimator_state().flow_valid ? 1.0f : 0.0f; }},
    {"flow_reject", []() -> float { return (float)poshold_estimator_state().reject; }},
    {"flow_rej_mask", []() -> float { return (float)poshold_flow_reject_mask_take(); }},
    {"flow_vx_mps", []() -> float { return poshold_estimator_state().flow_vx; }},
    {"flow_vy_mps", []() -> float { return poshold_estimator_state().flow_vy; }},
    {"vel_x_mps", []() -> float { return poshold_estimator_state().vx; }},
    {"vel_y_mps", []() -> float { return poshold_estimator_state().vy; }},
    {"pos_x_m", []() -> float { return poshold_estimator_state().px; }},
    {"pos_y_m", []() -> float { return poshold_estimator_state().py; }},
    {"height_m", []() -> float { return poshold_estimator_state().height; }},
    {"thrust0", []() -> float { return (float)Thrust0; }},
    // Control link v2 (feat/joystick-mode)
    {"rc_proto_ok", []() -> float { return rc_link_protocol_ok() ? 1.0f : 0.0f; }},
    {"rc_flight_mode", []() -> float { return (float)rc_link_requested_mode(); }},
    // Position Hold controller and supervisor (feat/position-controller)
    {"ph_state", []() -> float { return (float)poshold_supervisor_output().state; }},
    {"ph_reason", []() -> float { return (float)poshold_supervisor_output().reason; }},
    {"pos_sp_x_m", []() -> float { return poshold_controller_output().sp_x; }},
    {"pos_sp_y_m", []() -> float { return poshold_controller_output().sp_y; }},
    {"vel_sp_x_mps", []() -> float { return poshold_controller_output().vsp_x; }},
    {"vel_sp_y_mps", []() -> float { return poshold_controller_output().vsp_y; }},
    {"dist_m", []() -> float {
         const auto& e = poshold_estimator_state();
         return sqrtf(e.px * e.px + e.py * e.py);
     }},
    {"vbat_filt_v", []() -> float { return poshold_supervisor_output().vbat_filtered; }},
    // Barometer and bottom ToF diagnostics (feat/baro-abs); replace ctrl_mode, alt_flag,
    // flow_rej_total and acc_bias_x to stay within the frame size
    {"baro_pa", []() -> float { return baro_pressure_pa(); }},
    {"baro_temp_c", []() -> float { return baro_temperature_c(); }},
    {"tof_status", []() -> float { return (float)tof_bottom_diag().status; }},
    {"tof_sig_mcps", []() -> float { return tof_bottom_diag().signal_mcps; }},
};
constexpr size_t kFieldCount = sizeof(kFields) / sizeof(kFields[0]);
static_assert(kFieldCount <= stampfly_link::kMaxDataFields, "too many telemetry fields for one frame");

uint32_t g_counter     = 0;
uint16_t g_seq_usb     = 0;  // separate sequences: lost frames are counted per link
uint16_t g_seq_espnow  = 0;
uint32_t g_usb_dropped = 0;
size_t g_schema_next   = 0;  // next field index of the running schema burst
bool g_schema_active   = false;

bool telem_addr_known(void) {
    for (int i = 0; i < 6; i++)
        if (TelemAddr[i]) return true;
    return false;
}

// Non-blocking USB write: only writes when the whole frame fits in the TX buffer.
void usb_write(const uint8_t* buf, size_t len) {
    if ((size_t)USBSerial.availableForWrite() >= len) {
        USBSerial.write(buf, len);
    } else {
        g_usb_dropped++;
    }
}

void espnow_write(const uint8_t* buf, size_t len) {
    if (telem_addr_known()) esp_now_send(TelemAddr, buf, len);
}

// Builds the next schema chunk and advances the burst state.
size_t build_schema_chunk(uint8_t* out) {
    const char* names[kFieldCount];
    for (size_t i = 0; i < kFieldCount; i++) names[i] = kFields[i].name;
    size_t consumed = 0;
    const size_t len = encode_schema_chunk(names, kFieldCount, g_schema_next, &consumed, out, kMaxFrameSize);
    g_schema_next += consumed;
    if (len == 0 || g_schema_next >= kFieldCount) g_schema_active = false;
    return len;
}

size_t build_data(uint16_t seq, uint8_t* out) {
    float values[kFieldCount];
    for (size_t i = 0; i < kFieldCount; i++) values[i] = kFields[i].get();
    return encode_data(micros(), seq, values, kFieldCount, out, kMaxFrameSize);
}

}  // namespace

void telemetry_stream_update(void) {
    uint8_t frame[kMaxFrameSize];
    const uint32_t n = g_counter++;

    if (n % kSchemaDivider == 0) {
        g_schema_active = true;
        g_schema_next   = 0;
    }
    // The schema does not fit into one frame: send one chunk every 4th
    // iteration (10 ms, so the 256-byte USB TX buffer can drain) and skip data
    // frames until the burst is complete (~30 ms every 2 s).
    if (g_schema_active) {
        if (n % 4 == 0) {
            const size_t len = build_schema_chunk(frame);
            if (len) {
                usb_write(frame, len);
                espnow_write(frame, len);
            }
        }
        return;
    }

    // The ESP-NOW slot is offset from the USB slot, so at most one data frame
    // is built per loop iteration.
    if ((n % kUsbDivider) == 0) {
        const size_t len = build_data(g_seq_usb++, frame);
        if (len) usb_write(frame, len);
    } else if ((n % kEspNowDivider) == 4) {
        const size_t len = build_data(g_seq_espnow++, frame);
        if (len) espnow_write(frame, len);
    }
}

uint32_t telemetry_stream_usb_dropped(void) {
    return g_usb_dropped;
}

void telemetry_text(const char* fmt, ...) {
    char buf[96];
    va_list args;
    va_start(args, fmt);
    const int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (n <= 0) return;
    usb_write((const uint8_t*)buf, (size_t)(n < (int)sizeof(buf) ? n : (int)sizeof(buf) - 1));
}
