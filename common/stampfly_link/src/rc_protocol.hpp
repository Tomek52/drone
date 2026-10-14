// SPDX-License-Identifier: MIT
//
// ESP-NOW control protocol between the Atom JoyStick and the StampFly.
// Documented in docs/04-protokol-espnow.md.
//
// Version 1 (upstream): 25-byte packet, no version field, checksum at byte 24.
// Version 2 (this project): bytes 0..23 keep the v1 meaning, followed by
//
//   offset  field
//   24      protocol version (= kRcProtocolVersion)
//   25      flight mode (FlightMode)
//   26      flags (kRcFlagKill, ...)
//   27      reserved (0)
//   28      checksum = sum of bytes 0..27 (mod 256)
//
// Compatibility:
//   - new drone + old joystick (25 bytes)  -> decode() returns kVersionMismatch;
//     the drone ignores the sticks and refuses to arm.
//   - old drone + new joystick (29 bytes)  -> the old firmware checks byte 24
//     against sum(0..23); it only matches by chance, and even then byte 21
//     (control mode) is 0 = Stable, never an unknown value.
//
// Drone -> joystick status packet (kDroneStatusLen bytes, ~5 Hz):
//   0..1 magic 'S' 'F', 2 protocol version, 3 status flags, 4 active flight
//   mode (what the drone really flies), 5 failsafe/fallback reason,
//   6 reserved, 7 checksum = sum of bytes 0..6.

#ifndef STAMPFLY_LINK_RC_PROTOCOL_HPP
#define STAMPFLY_LINK_RC_PROTOCOL_HPP

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace stampfly_link {

constexpr uint8_t kRcProtocolVersion = 2;
constexpr size_t kRcPacketV1Len      = 25;
constexpr size_t kRcPacketLen        = 29;

constexpr size_t kRcOffVersion    = 24;
constexpr size_t kRcOffFlightMode = 25;
constexpr size_t kRcOffFlags      = 26;
constexpr size_t kRcOffChecksum   = 28;

// Byte 21 (control mode) values, unchanged from v1.
constexpr uint8_t kCtrlAngle = 0;
constexpr uint8_t kCtrlRate  = 1;

enum class FlightMode : uint8_t {
    kStable  = 0,
    kSport   = 1,
    kPosHold = 2,
};
constexpr uint8_t kFlightModeCount = 3;

// Flags (byte 26).
constexpr uint8_t kRcFlagKill = 0x01;  // stop the motors immediately

enum class RcDecode : uint8_t {
    kOk              = 0,
    kBadLength       = 1,
    kVersionMismatch = 2,
    kBadChecksum     = 3,
};

struct RcExtension {
    uint8_t version        = 0;  // protocol version of the sender (1 for 25-byte packets)
    FlightMode flight_mode = FlightMode::kStable;
    uint8_t flags          = 0;
};

inline uint8_t sum8(const uint8_t* d, size_t n) {
    uint8_t s = 0;
    for (size_t i = 0; i < n; i++) s = (uint8_t)(s + d[i]);
    return s;
}

// Builds a v2 packet from the v1 bytes 0..23 prepared by the upstream
// joystick code. `out` must hold kRcPacketLen bytes. Returns kRcPacketLen.
inline size_t encode_rc_v2(const uint8_t* v1_bytes, FlightMode mode, uint8_t flags, uint8_t* out) {
    memcpy(out, v1_bytes, 24);
    out[kRcOffVersion]    = kRcProtocolVersion;
    out[kRcOffFlightMode] = (uint8_t)mode;
    out[kRcOffFlags]      = flags;
    out[27]               = 0;
    out[kRcOffChecksum]   = sum8(out, kRcOffChecksum);
    return kRcPacketLen;
}

// Validates a received control packet. On kOk, `ext` holds the v2 fields and
// bytes 0..23 can be parsed with the v1 layout.
inline RcDecode decode_rc(const uint8_t* buf, size_t len, RcExtension* ext) {
    *ext = RcExtension();
    if (len == kRcPacketV1Len) {
        ext->version = 1;
        return RcDecode::kVersionMismatch;
    }
    if (len != kRcPacketLen) return RcDecode::kBadLength;
    ext->version = buf[kRcOffVersion];
    if (ext->version != kRcProtocolVersion) return RcDecode::kVersionMismatch;
    if (buf[kRcOffChecksum] != sum8(buf, kRcOffChecksum)) return RcDecode::kBadChecksum;
    const uint8_t m  = buf[kRcOffFlightMode];
    ext->flight_mode = m < kFlightModeCount ? (FlightMode)m : FlightMode::kStable;
    ext->flags       = buf[kRcOffFlags];
    return RcDecode::kOk;
}

// --- Drone -> joystick status ------------------------------------------------

constexpr size_t kDroneStatusLen = 8;
constexpr uint8_t kStatusMagic0  = 'S';
constexpr uint8_t kStatusMagic1  = 'F';

// Status flags (byte 3).
constexpr uint8_t kStatusRcOk          = 0x01;  // last control packet had a matching version
constexpr uint8_t kStatusFlowAvailable = 0x02;  // flow sensor detected and calibrated
constexpr uint8_t kStatusFlowValid     = 0x04;  // flow currently trusted
constexpr uint8_t kStatusPosHoldActive = 0x08;  // Position Hold controller in control
constexpr uint8_t kStatusFailsafe      = 0x10;  // a failsafe / fallback is active
constexpr uint8_t kStatusLowBattery    = 0x20;

// Why the drone does not fly the requested mode, or why a failsafe is active
// (status byte 5). Shown on the joystick screen and in telemetry.
enum class Reason : uint8_t {
    kNone               = 0,
    kNotImplemented     = 1,  // firmware without the Position Hold controller
    kFlowSensorMissing  = 2,  // PMW3901 not detected at boot
    kFlowNotCalibrated  = 3,  // kFlowAxesVerified == false
    kFlowInvalid        = 4,  // flow quality lost in flight
    kHeightOutOfRange   = 5,
    kGeofence           = 6,  // too far from the take-off point
    kLinkLost           = 7,  // no control packets
    kLowBattery         = 8,
    kKill               = 9,  // motors stopped from the joystick
    kProtocolMismatch   = 10,
};

struct DroneStatus {
    uint8_t version        = 0;
    uint8_t flags          = 0;
    FlightMode active_mode = FlightMode::kStable;
    uint8_t reason         = 0;  // Reason
};

inline size_t encode_status(const DroneStatus& s, uint8_t* out) {
    out[0] = kStatusMagic0;
    out[1] = kStatusMagic1;
    out[2] = s.version;
    out[3] = s.flags;
    out[4] = (uint8_t)s.active_mode;
    out[5] = s.reason;
    out[6] = 0;
    out[7] = sum8(out, 7);
    return kDroneStatusLen;
}

inline bool is_status_packet(const uint8_t* buf, size_t len) {
    return len == kDroneStatusLen && buf[0] == kStatusMagic0 && buf[1] == kStatusMagic1;
}

inline bool decode_status(const uint8_t* buf, size_t len, DroneStatus* out) {
    if (!is_status_packet(buf, len) || buf[7] != sum8(buf, 7)) return false;
    out->version     = buf[2];
    out->flags       = buf[3];
    out->active_mode = buf[4] < kFlightModeCount ? (FlightMode)buf[4] : FlightMode::kStable;
    out->reason      = buf[5];
    return true;
}

}  // namespace stampfly_link

#endif  // STAMPFLY_LINK_RC_PROTOCOL_HPP
