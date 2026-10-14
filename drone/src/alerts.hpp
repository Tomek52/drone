// SPDX-License-Identifier: MIT
//
// [POSHOLD] Non-blocking buzzer patterns and LED overrides.
//
// The upstream buzzer_sound() blocks the control loop for its whole duration,
// so it must not be used in flight. alerts_update() advances the patterns
// from the 400 Hz loop without blocking.

#ifndef ALERTS_HPP
#define ALERTS_HPP

#include <stdint.h>

enum class Alert : uint8_t {
    kNone = 0,
    kProtocolMismatch,  // repeated: two low beeps every 2 s
    kKill,              // one long high beep
    kFallback,          // three short beeps (Position Hold -> Stable)
    kLowBattery,        // repeated: long low beeps
    kLinkLost,          // repeated: fast beeps
    kModeOn,            // short rising chirp (Position Hold engaged)
};

// Starts a pattern. A repeated pattern runs until alerts_stop() or another
// alert is started; one-shot patterns end by themselves.
void alerts_play(Alert a);
void alerts_stop(Alert a);  // stops `a` if it is the current pattern
Alert alerts_current(void);

// Call once per control loop iteration (after led_drive()).
void alerts_update(void);

// Applies the active LED override to the onboard LEDs without showing them. Called by
// led_drive() right before led_show(), so one frame per loop is sent (showing the upstream
// color and the override color alternately at 400 Hz looked white).
void alerts_apply_led_override(void);

// LED overrides applied after the upstream led_drive(). Each owner has its
// own slot; the owner with the higher value wins.
enum class LedOwner : uint8_t {
    kPosHold  = 0,  // Position Hold active / fallback (in flight)
    kProtocol = 1,  // protocol mismatch (on the ground)
    kCount    = 2,
};
// blink_ms = 0: steady color_a; otherwise alternate color_a / color_b.
void alerts_set_led_override(LedOwner owner, uint32_t color_a, uint32_t color_b, uint16_t blink_ms);
void alerts_clear_led_override(LedOwner owner);

#endif
