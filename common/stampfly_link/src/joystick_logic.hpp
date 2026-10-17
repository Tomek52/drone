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
class FlightModeSelector {
   public:
    FlightMode mode() const { return mode_; }
    bool alt_auto() const { return alt_auto_; }

    // Value for byte 21 of the control packet (v1 meaning, understood by any drone).
    uint8_t control_mode_byte() const { return mode_ == FlightMode::kSport ? kCtrlRate : kCtrlAngle; }

    void cycle_mode_on_ground() {
        mode_ = (FlightMode)(((uint8_t)mode_ + 1) % kFlightModeCount);
        if (mode_ == FlightMode::kPosHold) alt_auto_ = true;
    }

    void toggle_alt_on_ground() {
        alt_auto_ = !alt_auto_;
        if (!alt_auto_ && mode_ == FlightMode::kPosHold) mode_ = FlightMode::kStable;
    }

    // Left front button pressed while flying. Returns true if the press was
    // consumed (mode changed) and must not trigger the upstream action.
    bool left_button_in_flight() {
        if (mode_ != FlightMode::kPosHold) return false;
        mode_ = FlightMode::kStable;
        return true;
    }

    // Lets the UI restore a mode (e.g. after reading the drone status).
    void set(FlightMode m, bool alt_auto) {
        mode_     = m;
        alt_auto_ = alt_auto || m == FlightMode::kPosHold;
    }

   private:
    FlightMode mode_ = FlightMode::kStable;
    bool alt_auto_   = true;  // upstream default: Auto altitude
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
