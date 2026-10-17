// SPDX-License-Identifier: MIT
//
// [POSHOLD] See flight_mode.h.

#include "flight_mode.h"

#include <Arduino.h>
#include <joystick_logic.hpp>
#include <rc_protocol.hpp>

#include "app_lvgl.h"
#include "buzzer.h"
#include "main.h"

using namespace stampfly_link;

namespace {

constexpr uint32_t kStatusTimeoutMs    = 1500;  // status packets arrive at ~5 Hz
constexpr uint32_t kOldFirmwareAfterMs = 3000;  // upstream telemetry but no status -> old drone firmware

// Colours (RGB hex) used on the screen.
constexpr uint32_t kColPosHold = 0x2A78D6;  // blue, same hue as the drone LED in Position Hold
constexpr uint32_t kColWarn    = 0xEB6834;  // orange: fallback / not available
constexpr uint32_t kColError   = 0xC1001D;  // red: protocol error, kill
constexpr uint32_t kColWhite   = 0xFFFFFF;
constexpr uint32_t kColBlack   = 0x000000;

FlightModeSelector g_sel;
KillSwitch g_kill;
volatile bool g_kill_flag = false;
volatile bool g_kill_beep = false;

// Last drone status (written in the Wi-Fi task).
volatile uint32_t g_status_ms = 0;
volatile bool g_status_seen   = false;
DroneStatus g_status;
uint32_t g_first_telemetry_ms = 0;

// Left button interception state (LVGL task).
bool g_btn_prev     = false;
bool g_btn_swallow  = false;
volatile bool g_mode_beep = false;

lv_obj_t *g_setup_badge = nullptr;

void sync_upstream_modes() {
    Mode    = g_sel.control_mode_byte();
    AltMode = g_sel.alt_auto() ? ALT_CONTROL_MODE : NOT_ALT_CONTROL_MODE;
}

bool status_fresh() {
    return g_status_seen && (millis() - g_status_ms) < kStatusTimeoutMs;
}

// Drone answers with upstream telemetry but never sent a v2 status packet.
bool drone_firmware_old() {
    if (!is_fly_flag) {
        g_first_telemetry_ms = 0;
        return false;
    }
    if (g_first_telemetry_ms == 0) g_first_telemetry_ms = millis();
    return !g_status_seen && (millis() - g_first_telemetry_ms) > kOldFirmwareAfterMs;
}

bool protocol_error() {
    if (drone_firmware_old()) return true;
    return status_fresh() && (g_status.version != kRcProtocolVersion ||
                              g_status.reason == (uint8_t)Reason::kProtocolMismatch);
}

const char *reason_short(uint8_t r) {
    switch ((Reason)r) {
        case Reason::kNotImplemented: return "NO FW";
        case Reason::kFlowSensorMissing: return "NO SENS";
        case Reason::kFlowNotCalibrated: return "NO CAL";
        case Reason::kFlowInvalid: return "FLOW";
        case Reason::kHeightOutOfRange: return "HEIGHT";
        case Reason::kGeofence: return "FENCE";
        case Reason::kLinkLost: return "LINK";
        case Reason::kLowBattery: return "BATT";
        case Reason::kKill: return "KILL";
        default: return "";
    }
}

void style_badge(lv_obj_t *o, uint32_t bg, uint32_t fg) {
    lv_obj_set_style_bg_opa(o, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(o, lv_color_hex(bg), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(o, lv_color_hex(fg), LV_PART_MAIN | LV_STATE_DEFAULT);
}

}  // namespace

// --- Main loop ---------------------------------------------------------------

void flight_mode_update(float dt) {
    const bool kill = g_kill.update(getArmButton() == 1, dt);
    if (g_kill.just_activated()) g_kill_beep = true;  // beep from the LVGL task, never block this loop
    g_kill_flag = kill;
}

size_t flight_mode_build_packet(const uint8_t *v1_bytes, uint8_t *out) {
    return encode_rc_v2(v1_bytes, g_sel.mode(), g_kill_flag ? kRcFlagKill : 0, out);
}

// --- ESP-NOW -------------------------------------------------------------------

bool flight_mode_on_packet(const uint8_t *data, int len) {
    if (!is_status_packet(data, (size_t)len)) return false;
    DroneStatus s;
    if (decode_status(data, (size_t)len, &s)) {
        g_status      = s;
        g_status_ms   = millis();
        g_status_seen = true;
    }
    return true;
}

// --- Buttons -------------------------------------------------------------------

void flight_mode_on_mode_button(void) {
    g_sel.cycle_mode_on_ground();
    sync_upstream_modes();
}

void flight_mode_on_alt_button(void) {
    g_sel.toggle_alt_on_ground();
    sync_upstream_modes();
}

bool flight_mode_intercept_left_button(bool pressed) {
    const bool edge = pressed && !g_btn_prev;
    g_btn_prev      = pressed;
    if (!pressed) {
        g_btn_swallow = false;
        return false;
    }
    if (edge && page_nums == PAGE_RUNNING && fly_mode == FLIGHT_MODE &&
        g_sel.mode() == FlightMode::kPosHold) {
        g_sel.left_button_in_flight();  // Position Hold -> Stable
        sync_upstream_modes();
        g_btn_swallow = true;
        g_mode_beep   = true;
    }
    return g_btn_swallow;
}

// --- Screens -------------------------------------------------------------------

void flight_mode_ui_attach_setup(lv_obj_t *screen) {
    // Badge over the Stable/Sport picture (top-left 64x75 area).
    g_setup_badge = lv_label_create(screen);
    lv_obj_set_pos(g_setup_badge, 0, 0);
    lv_obj_set_size(g_setup_badge, 64, 75);
    lv_label_set_long_mode(g_setup_badge, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(g_setup_badge, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(g_setup_badge, &lv_font_montserratMedium_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(g_setup_badge, 18, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_flag(g_setup_badge, LV_OBJ_FLAG_HIDDEN);
}

void flight_mode_ui_update_setup(void) {
    if (g_setup_badge == nullptr) return;
    if (protocol_error()) {
        style_badge(g_setup_badge, kColError, kColWhite);
        lv_label_set_text(g_setup_badge, "DRONE\nFW?");
        lv_obj_clear_flag(g_setup_badge, LV_OBJ_FLAG_HIDDEN);
    } else if (g_sel.mode() == FlightMode::kPosHold) {
        const bool unavailable = status_fresh() && !(g_status.flags & kStatusFlowAvailable);
        style_badge(g_setup_badge, unavailable ? kColWarn : kColPosHold, kColWhite);
        lv_label_set_text(g_setup_badge, unavailable ? "POS\nN/A" : "POS\nHOLD");
        lv_obj_set_style_bg_color(guider_ui.screen_cont_2, lv_color_hex(kColPosHold), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_clear_flag(g_setup_badge, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(g_setup_badge, LV_OBJ_FLAG_HIDDEN);
    }
}

void flight_mode_ui_update_running(void) {
    if (g_kill_beep) {
        g_kill_beep = false;
        buzzer_sound(1000, 400);
    }
    if (g_mode_beep) {
        g_mode_beep = false;
        beep();
    }

    // The drone buzzer is drowned out by the motors in flight, so the low-battery warning
    // (status flag from the drone, set below 3.50 V filtered) also sounds here.
    const bool low_batt = status_fresh() && (g_status.flags & kStatusLowBattery);
    static uint32_t last_batt_beep = 0;
    if (low_batt && !g_kill_flag && millis() - last_batt_beep > 1500) {
        last_batt_beep = millis();
        buzzer_sound(3000, 150);
    }

    // The first line of the top-left panel shows the mode instead of the front ToF distance.
    lv_obj_t *label = guider_ui.screen_running_label_1;
    lv_obj_t *panel = guider_ui.screen_running_cont_2;
    char text[24];
    uint32_t bg = kColWhite, fg = kColBlack;

    if (g_kill_flag) {
        snprintf(text, sizeof(text), "KILL");
        bg = kColError;
        fg = kColWhite;
    } else if (protocol_error()) {
        snprintf(text, sizeof(text), "PROTO ERR");
        bg = kColError;
        fg = kColWhite;
    } else if (low_batt) {
        snprintf(text, sizeof(text), "LOW BATT");
        bg = kColError;
        fg = kColWhite;
    } else if (g_sel.mode() == FlightMode::kPosHold) {
        const bool fresh = status_fresh();
        if (fresh && g_status.active_mode != FlightMode::kPosHold &&
            g_status.reason != (uint8_t)Reason::kNone) {
            snprintf(text, sizeof(text), "STB %s", reason_short(g_status.reason));
            bg = kColWarn;
            fg = kColWhite;
        } else {
            snprintf(text, sizeof(text), "POS HOLD");
            bg = kColPosHold;
            fg = kColWhite;
        }
    } else if (g_sel.mode() == FlightMode::kSport) {
        snprintf(text, sizeof(text), "SPORT");
    } else {
        snprintf(text, sizeof(text), "STABLE");
    }

    lv_label_set_text(label, text);
    lv_obj_set_style_bg_color(panel, lv_color_hex(bg), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(label, lv_color_hex(fg), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(guider_ui.screen_running_label_2, lv_color_hex(fg), LV_PART_MAIN | LV_STATE_DEFAULT);
}
