// SPDX-License-Identifier: MIT
//
// [POSHOLD] Flight mode selection (Stable / Sport / Position Hold), kill
// switch, control protocol v2 and the related screen elements.
// Logic: common/stampfly_link/src/joystick_logic.hpp (unit tested).
// Protocol: common/stampfly_link/src/rc_protocol.hpp, docs/04-protokol-espnow.md.

#ifndef FLIGHT_MODE_H
#define FLIGHT_MODE_H

#include <stddef.h>
#include <stdint.h>

#include "lvgl.h"

// --- Main loop (100 Hz) ------------------------------------------------------
// Updates the kill switch from the stick button that upstream does not use.
void flight_mode_update(float dt);
// Builds the v2 control packet from the v1 bytes 0..23. Returns its length.
size_t flight_mode_build_packet(const uint8_t *v1_bytes, uint8_t *out);

// --- ESP-NOW receive callback ------------------------------------------------
// Returns true if the packet was a drone status packet.
bool flight_mode_on_packet(const uint8_t *data, int len);

// --- Buttons (LVGL task) -----------------------------------------------------
void flight_mode_on_mode_button(void);  // left front button, setup screen
void flight_mode_on_alt_button(void);   // right front button, setup screen
// Left front button state as read for LVGL. Returns true if the press is
// consumed here (Position Hold -> Stable in flight) and must be hidden from LVGL.
bool flight_mode_intercept_left_button(bool pressed);

// --- Screens (LVGL task) -----------------------------------------------------
void flight_mode_ui_attach_setup(lv_obj_t *screen);
void flight_mode_ui_update_setup(void);
void flight_mode_ui_update_running(void);

#endif
