// SPDX-License-Identifier: MIT
//
// [POSHOLD] See alerts.hpp.

#include "alerts.hpp"

#include <Arduino.h>

#include "led.hpp"

namespace {

// Same LEDC channel as the upstream buzzer (buzzer.cpp, G40).
constexpr uint8_t kBuzzerChannel = 5;

struct Step {
    uint16_t freq;  // 0 = silence
    uint16_t ms;
};

struct Pattern {
    const Step* steps;
    uint8_t count;
    bool repeat;
};

const Step kMismatch[]  = {{800, 120}, {0, 120}, {800, 120}, {0, 1640}};
const Step kKill[]      = {{2500, 600}, {0, 10}};
const Step kFallback[]  = {{3000, 80}, {0, 80}, {3000, 80}, {0, 80}, {3000, 80}, {0, 10}};
// [POSHOLD] 2500 Hz, not 600 Hz: the buzzer is a piezo and was inaudible over the motors at 600 Hz
// (low-battery warning not heard in the 2026-10-17 discharge flight).
const Step kLowBatt[]   = {{2500, 300}, {0, 700}};
const Step kLinkLost[]  = {{2000, 60}, {0, 140}};
const Step kModeOn[]    = {{2000, 60}, {3000, 60}, {4000, 80}, {0, 10}};

Pattern pattern_for(Alert a) {
    switch (a) {
        case Alert::kProtocolMismatch: return {kMismatch, 4, true};
        case Alert::kKill: return {kKill, 2, false};
        case Alert::kFallback: return {kFallback, 6, false};
        case Alert::kLowBattery: return {kLowBatt, 2, true};
        case Alert::kLinkLost: return {kLinkLost, 2, true};
        case Alert::kModeOn: return {kModeOn, 4, false};
        default: return {nullptr, 0, false};
    }
}

Alert g_alert        = Alert::kNone;
uint8_t g_step       = 0;
uint32_t g_step_end  = 0;
uint16_t g_tone      = 0xFFFF;

struct LedSlot {
    uint32_t a     = 0;
    uint32_t b     = 0;
    uint16_t blink = 0;
    bool active    = false;
};
LedSlot g_led[(int)LedOwner::kCount];

void set_tone(uint16_t f) {
    if (f == g_tone) return;
    g_tone = f;
    if (f) {
        ledcWriteTone(kBuzzerChannel, f);
        ledcWrite(kBuzzerChannel, 127);
    } else {
        ledcWriteTone(kBuzzerChannel, 0);
    }
}

void start_step(const Pattern& p, uint32_t now) {
    set_tone(p.steps[g_step].freq);
    g_step_end = now + p.steps[g_step].ms;
}

}  // namespace

void alerts_play(Alert a) {
    if (a == g_alert) return;  // keep a running pattern running
    g_alert = a;
    g_step  = 0;
    const Pattern p = pattern_for(a);
    if (p.count) {
        start_step(p, millis());
    } else {
        set_tone(0);
    }
}

void alerts_stop(Alert a) {
    if (g_alert != a) return;
    g_alert = Alert::kNone;
    set_tone(0);
}

Alert alerts_current(void) {
    return g_alert;
}

void alerts_update(void) {
    const uint32_t now = millis();
    const Pattern p    = pattern_for(g_alert);
    if (p.count && (int32_t)(now - g_step_end) >= 0) {
        g_step++;
        if (g_step >= p.count) {
            if (p.repeat) {
                g_step = 0;
            } else {
                g_alert = Alert::kNone;
                set_tone(0);
            }
        }
        if (g_alert != Alert::kNone) start_step(p, now);
    }

}

void alerts_apply_led_override(void) {
    const uint32_t now = millis();
    for (int i = (int)LedOwner::kCount - 1; i >= 0; i--) {
        const LedSlot& slot = g_led[i];
        if (!slot.active) continue;
        const bool phase = slot.blink ? ((now / slot.blink) & 1u) : false;
        const uint32_t c = phase ? slot.b : slot.a;
        onboard_led1(c, 1);
        onboard_led2(c, 1);
        break;
    }
}

void alerts_set_led_override(LedOwner owner, uint32_t color_a, uint32_t color_b, uint16_t blink_ms) {
    LedSlot& slot = g_led[(int)owner];
    slot.a        = color_a;
    slot.b        = color_b;
    slot.blink    = blink_ms;
    slot.active   = true;
}

void alerts_clear_led_override(LedOwner owner) {
    g_led[(int)owner].active = false;
}
