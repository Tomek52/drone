// SPDX-License-Identifier: MIT
//
// Hardware-independent joystick logic (flight mode selection, kill switch).
// Used by the joystick firmware and by host unit tests.

#ifndef STAMPFLY_LINK_JOYSTICK_LOGIC_HPP
#define STAMPFLY_LINK_JOYSTICK_LOGIC_HPP

#include <stdint.h>

#include "rc_protocol.hpp"

namespace stampfly_link {

// Flight mode selection.
//   On the ground (setup screen): the left front button cycles
//   Stable -> Sport -> Position Hold -> Stable; the right front button toggles
//   Auto / Manual altitude.
//   In flight: the left front button only leaves Position Hold (-> Stable);
//   entering Sport or Position Hold in the air is not possible.
// Position Hold always uses Auto altitude: selecting it forces Auto altitude,
// and switching to Manual altitude while it is selected falls back to Stable.
enum class AltSelect : uint8_t { kAuto = 0, kManual = 1, kAbs = 2 };

class FlightModeSelector {
   public:
    // abs_available = false: the right button only cycles Auto <-> Manual (Stable ABS is experimental).
    explicit FlightModeSelector(bool abs_available = true) : abs_available_(abs_available) {}

    FlightMode mode() const { return mode_; }
    // Upstream altitude control is on for Auto and Abs (the barometer only replaces the height source).
    bool alt_auto() const { return alt_ != AltSelect::kManual; }
    bool alt_abs() const { return alt_ == AltSelect::kAbs; }

    // Value for byte 21 of the control packet (v1 meaning, understood by any drone).
    uint8_t control_mode_byte() const { return mode_ == FlightMode::kSport ? kCtrlRate : kCtrlAngle; }

    void cycle_mode_on_ground() {
        mode_ = (FlightMode)(((uint8_t)mode_ + 1) % kFlightModeCount);
        if (mode_ == FlightMode::kPosHold) alt_ = AltSelect::kAuto;
    }

    // Auto -> Manual -> ABS -> Auto. Position Hold needs the ToF height, so it falls back to Stable.
    void toggle_alt_on_ground() {
        alt_ = (AltSelect)(((uint8_t)alt_ + 1) % (abs_available_ ? 3 : 2));
        if (alt_ != AltSelect::kAuto && mode_ == FlightMode::kPosHold) mode_ = FlightMode::kStable;
    }

    // Left front button pressed while flying. Returns true if the press was
    // consumed (mode changed) and must not trigger the upstream action.
    bool left_button_in_flight() {
        if (mode_ != FlightMode::kPosHold) return false;
        mode_ = FlightMode::kStable;
        return true;
    }

    // Lets the UI restore a mode (e.g. after reading the drone status). Keeps ABS if Auto is requested.
    void set(FlightMode m, bool alt_auto) {
        mode_ = m;
        if (m == FlightMode::kPosHold) alt_ = AltSelect::kAuto;
        else if (!alt_auto) alt_ = AltSelect::kManual;
        else if (alt_ == AltSelect::kManual) alt_ = AltSelect::kAuto;
    }

   private:
    bool abs_available_ = true;
    FlightMode mode_ = FlightMode::kStable;
    AltSelect alt_   = AltSelect::kAuto;  // upstream default: Auto altitude
};

// Kill switch: holding the button for `hold_s` activates the kill flag. The
// flag stays active while the button is held and for `linger_s` after release,
// so the drone receives it even if a few packets are lost.
class KillSwitch {
   public:
    explicit KillSwitch(float hold_s = 0.5f, float linger_s = 0.5f) : hold_s_(hold_s), linger_s_(linger_s) {}

    // Returns true while the kill flag must be sent.
    bool update(bool pressed, float dt) {
        if (pressed) {
            held_ += dt;
            if (held_ >= hold_s_) {
                active_ = true;
                linger_ = linger_s_;
            }
        } else {
            held_ = 0.0f;
            if (active_) {
                linger_ -= dt;
                if (linger_ <= 0.0f) active_ = false;
            }
        }
        return active_;
    }

    // True exactly once per activation (for a one-shot beep).
    bool just_activated() {
        const bool r = active_ && !reported_;
        if (active_) reported_ = true;
        if (!active_) reported_ = false;
        return r;
    }

    bool active() const { return active_; }

   private:
    float hold_s_, linger_s_;
    float held_    = 0.0f;
    float linger_  = 0.0f;
    bool active_   = false;
    bool reported_ = false;
};

}  // namespace stampfly_link

#endif  // STAMPFLY_LINK_JOYSTICK_LOGIC_HPP
