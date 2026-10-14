// SPDX-License-Identifier: MIT
//
// Binary telemetry frames shared by the drone (producer), the joystick
// (ESP-NOW -> USB forwarder) and the PC tools (tools/stampfly_telemetry.py).
//
// Frame layout (all multi-byte values little-endian):
//
//   offset  size  field
//   0       1     sync0 = 0xA5
//   1       1     sync1 = 0x5A
//   2       1     type  (FrameType)
//   3       1     payload length N (0..kMaxPayload)
//   4       N     payload
//   4+N     2     Fletcher-16 checksum over bytes [2 .. 4+N)  (type, length, payload)
//
// Payloads:
//   kSchema: u8 format version, u8 total field count, u8 index of the first
//            field in this chunk, u8 number of fields in this chunk, then ASCII
//            field names separated by ',' (no terminator). The full list does
//            not fit into one frame, so it is split into chunks. Sent
//            periodically so that a logger attached mid-stream can decode data.
//   kData:   u32 time [us], u16 sequence number, then field_count x float32.
//
// The frame fits in a single ESP-NOW packet (max 250 bytes).

#ifndef STAMPFLY_LINK_TELEMETRY_FRAME_HPP
#define STAMPFLY_LINK_TELEMETRY_FRAME_HPP

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace stampfly_link {

constexpr uint8_t kSync0               = 0xA5;
constexpr uint8_t kSync1               = 0x5A;
constexpr uint8_t kTelemetryFormatVersion = 1;
constexpr size_t kSchemaPrefixSize    = 4;
constexpr size_t kHeaderSize           = 4;
constexpr size_t kChecksumSize         = 2;
constexpr size_t kMaxPayload           = 240;
constexpr size_t kMaxFrameSize         = kHeaderSize + kMaxPayload + kChecksumSize;
constexpr size_t kDataPrefixSize       = 6;  // u32 time_us + u16 seq
constexpr size_t kMaxDataFields        = (kMaxPayload - kDataPrefixSize) / 4;

enum class FrameType : uint8_t {
    kSchema = 0x01,
    kData   = 0x02,
};

inline uint16_t fletcher16(const uint8_t* data, size_t len) {
    uint16_t sum1 = 0, sum2 = 0;
    for (size_t i = 0; i < len; i++) {
        sum1 = (uint16_t)((sum1 + data[i]) % 255);
        sum2 = (uint16_t)((sum2 + sum1) % 255);
    }
    return (uint16_t)((sum2 << 8) | sum1);
}

// Wraps a payload into a frame. Returns the frame size, or 0 if it does not fit.
inline size_t encode_frame(FrameType type, const uint8_t* payload, size_t payload_len, uint8_t* out,
                           size_t out_size) {
    if (payload_len > kMaxPayload) return 0;
    const size_t total = kHeaderSize + payload_len + kChecksumSize;
    if (out_size < total) return 0;
    out[0] = kSync0;
    out[1] = kSync1;
    out[2] = (uint8_t)type;
    out[3] = (uint8_t)payload_len;
    if (payload_len) memcpy(out + kHeaderSize, payload, payload_len);
    const uint16_t c = fletcher16(out + 2, 2 + payload_len);
    out[kHeaderSize + payload_len]     = (uint8_t)(c & 0xFF);
    out[kHeaderSize + payload_len + 1] = (uint8_t)(c >> 8);
    return total;
}

// Builds one schema chunk frame with as many names as fit, starting at
// names[first]. *consumed receives the number of names written. Returns the
// frame size, or 0 on error (e.g. a single name longer than the payload).
inline size_t encode_schema_chunk(const char* const* names, size_t total, size_t first, size_t* consumed,
                                  uint8_t* out, size_t out_size) {
    uint8_t payload[kMaxPayload];
    *consumed = 0;
    if (total > kMaxDataFields || first >= total) return 0;
    size_t n = kSchemaPrefixSize;
    size_t i = first;
    for (; i < total; i++) {
        const size_t len = strlen(names[i]);
        const size_t sep = (i > first) ? 1 : 0;
        if (n + sep + len > kMaxPayload) break;
        if (sep) payload[n++] = ',';
        memcpy(payload + n, names[i], len);
        n += len;
    }
    if (i == first) return 0;
    payload[0] = kTelemetryFormatVersion;
    payload[1] = (uint8_t)total;
    payload[2] = (uint8_t)first;
    payload[3] = (uint8_t)(i - first);
    *consumed  = i - first;
    return encode_frame(FrameType::kSchema, payload, n, out, out_size);
}

// Builds a data frame. Returns frame size or 0.
inline size_t encode_data(uint32_t time_us, uint16_t seq, const float* values, size_t count, uint8_t* out,
                          size_t out_size) {
    uint8_t payload[kMaxPayload];
    if (count > kMaxDataFields) return 0;
    memcpy(payload, &time_us, 4);
    memcpy(payload + 4, &seq, 2);
    memcpy(payload + kDataPrefixSize, values, count * 4);
    return encode_frame(FrameType::kData, payload, kDataPrefixSize + count * 4, out, out_size);
}

// Quick check used by the joystick forwarder: does this buffer hold exactly one
// complete, valid frame?
inline bool is_valid_frame(const uint8_t* buf, size_t len) {
    if (len < kHeaderSize + kChecksumSize) return false;
    if (buf[0] != kSync0 || buf[1] != kSync1) return false;
    const size_t payload_len = buf[3];
    if (len != kHeaderSize + payload_len + kChecksumSize) return false;
    const uint16_t c = fletcher16(buf + 2, 2 + payload_len);
    return buf[kHeaderSize + payload_len] == (uint8_t)(c & 0xFF) &&
           buf[kHeaderSize + payload_len + 1] == (uint8_t)(c >> 8);
}

}  // namespace stampfly_link

#endif  // STAMPFLY_LINK_TELEMETRY_FRAME_HPP
