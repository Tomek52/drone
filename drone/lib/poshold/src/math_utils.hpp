// SPDX-License-Identifier: MIT
//
// Small, dependency-free math helpers shared by the Position Hold modules.
// Everything here must compile both for ESP32 (Arduino) and for the host
// (PlatformIO `native` test environment), so no Arduino headers are used.

#ifndef POSHOLD_MATH_UTILS_HPP
#define POSHOLD_MATH_UTILS_HPP

#include <math.h>

namespace poshold {

constexpr float kPi = 3.14159265358979323846f;

inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

// Symmetric dead band around zero. Output is rescaled so that it is
// continuous at the edge of the dead band and still reaches +-1 at |x| = 1.
inline float deadband(float x, float band) {
    if (band <= 0.0f) return x;
    if (band >= 1.0f) return 0.0f;
    if (x > band) return (x - band) / (1.0f - band);
    if (x < -band) return (x + band) / (1.0f - band);
    return 0.0f;
}

// Wraps an angle to the range (-pi, pi].
inline float wrap_pi(float a) {
    while (a > kPi) a -= 2.0f * kPi;
    while (a <= -kPi) a += 2.0f * kPi;
    return a;
}

// Rotates the 2D vector (x, y) by angle psi (counter-clockwise for a right-handed
// x/y frame). Used to convert between the body frame and the yaw-aligned
// world frame.
inline void rotate2d(float x, float y, float psi, float* out_x, float* out_y) {
    const float c = cosf(psi);
    const float s = sinf(psi);
    *out_x = c * x - s * y;
    *out_y = s * x + c * y;
}

// Limits the length of the 2D vector (x, y) to max_norm, keeping its direction.
inline void limit_norm2d(float* x, float* y, float max_norm) {
    const float n = sqrtf((*x) * (*x) + (*y) * (*y));
    if (n > max_norm && n > 0.0f) {
        const float k = max_norm / n;
        *x *= k;
        *y *= k;
    }
}

}  // namespace poshold

#endif  // POSHOLD_MATH_UTILS_HPP
