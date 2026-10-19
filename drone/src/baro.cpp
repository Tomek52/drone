// SPDX-License-Identifier: MIT
//
// [POSHOLD] See baro.hpp. Compensation: Bosch BMP280 datasheet, 32-bit integer formulas.

#include "baro.hpp"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

namespace {

constexpr uint8_t kAddr        = 0x76;
constexpr uint8_t kRegId       = 0xD0;
constexpr uint8_t kRegCalib    = 0x88;
constexpr uint8_t kRegCtrlMeas = 0xF4;
constexpr uint8_t kRegConfig   = 0xF5;
constexpr uint8_t kRegData     = 0xF7;
constexpr uint32_t kPeriodMs   = 40;  // 25 Hz, the sensor converts at ~23 Hz

struct Calib {
    uint16_t t1;
    int16_t t2, t3;
    uint16_t p1;
    int16_t p2, p3, p4, p5, p6, p7, p8, p9;
};

Calib g_c;
bool g_present      = false;
float g_pressure    = 0.0f;
float g_temperature = 0.0f;
float g_ref_pa      = 0.0f;  // pressure at ground level
bool g_parked       = false;
int32_t g_t_fine    = 0;
uint32_t g_last_ms  = 0;

bool read_regs(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire1.beginTransmission(kAddr);
    Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom((int)kAddr, (int)len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire1.read();
    return true;
}

bool write_reg(uint8_t reg, uint8_t value) {
    Wire1.beginTransmission(kAddr);
    Wire1.write(reg);
    Wire1.write(value);
    return Wire1.endTransmission() == 0;
}

int32_t compensate_t(int32_t adc_t) {
    int32_t v1 = ((((adc_t >> 3) - ((int32_t)g_c.t1 << 1))) * ((int32_t)g_c.t2)) >> 11;
    int32_t v2 = (((((adc_t >> 4) - ((int32_t)g_c.t1)) * ((adc_t >> 4) - ((int32_t)g_c.t1))) >> 12) *
                  ((int32_t)g_c.t3)) >> 14;
    g_t_fine = v1 + v2;
    return (g_t_fine * 5 + 128) >> 8;  // 0.01 deg C
}

uint32_t compensate_p(int32_t adc_p) {
    int64_t v1 = (int64_t)g_t_fine - 128000;
    int64_t v2 = v1 * v1 * (int64_t)g_c.p6;
    v2         = v2 + ((v1 * (int64_t)g_c.p5) << 17);
    v2         = v2 + (((int64_t)g_c.p4) << 35);
    v1         = ((v1 * v1 * (int64_t)g_c.p3) >> 8) + ((v1 * (int64_t)g_c.p2) << 12);
    v1         = (((((int64_t)1) << 47) + v1)) * ((int64_t)g_c.p1) >> 33;
    if (v1 == 0) return 0;
    int64_t p = 1048576 - adc_p;
    p         = (((p << 31) - v2) * 3125) / v1;
    v1        = (((int64_t)g_c.p9) * (p >> 13) * (p >> 13)) >> 25;
    v2        = (((int64_t)g_c.p8) * p) >> 19;
    p         = ((p + v1 + v2) >> 8) + (((int64_t)g_c.p7) << 4);
    return (uint32_t)p;  // Q24.8, Pa * 256
}

}  // namespace

bool baro_init(void) {
    uint8_t id = 0;
    if (!read_regs(kRegId, &id, 1) || (id != 0x58 && id != 0x60)) {
        USBSerial.printf("[POSHOLD] BMP280 not detected (id=0x%02X)\r\n", id);
        return false;
    }
    uint8_t b[24];
    if (!read_regs(kRegCalib, b, 24)) return false;
    g_c.t1 = (uint16_t)(b[1] << 8 | b[0]);
    g_c.t2 = (int16_t)(b[3] << 8 | b[2]);
    g_c.t3 = (int16_t)(b[5] << 8 | b[4]);
    g_c.p1 = (uint16_t)(b[7] << 8 | b[6]);
    g_c.p2 = (int16_t)(b[9] << 8 | b[8]);
    g_c.p3 = (int16_t)(b[11] << 8 | b[10]);
    g_c.p4 = (int16_t)(b[13] << 8 | b[12]);
    g_c.p5 = (int16_t)(b[15] << 8 | b[14]);
    g_c.p6 = (int16_t)(b[17] << 8 | b[16]);
    g_c.p7 = (int16_t)(b[19] << 8 | b[18]);
    g_c.p8 = (int16_t)(b[21] << 8 | b[20]);
    g_c.p9 = (int16_t)(b[23] << 8 | b[22]);
    // standby 0.5 ms, IIR x4; temperature x2, pressure x16, normal mode
    if (!write_reg(kRegConfig, 0x08) || !write_reg(kRegCtrlMeas, 0x57)) return false;
    g_present = true;
    USBSerial.printf("[POSHOLD] BMP280 ready (id=0x%02X)\r\n", id);
    return true;
}

bool baro_present(void) {
    return g_present;
}

void baro_update(void) {
    if (!g_present) return;
    const uint32_t now = millis();
    if (now - g_last_ms < kPeriodMs) return;
    g_last_ms = now;
    uint8_t d[6];
    if (!read_regs(kRegData, d, 6)) return;
    const int32_t adc_p = (int32_t)((uint32_t)d[0] << 12 | (uint32_t)d[1] << 4 | d[2] >> 4);
    const int32_t adc_t = (int32_t)((uint32_t)d[3] << 12 | (uint32_t)d[4] << 4 | d[5] >> 4);
    if (adc_p == 0x80000 || adc_t == 0x80000) return;  // conversion not finished yet
    g_temperature = compensate_t(adc_t) / 100.0f;
    const uint32_t p = compensate_p(adc_p);
    if (p == 0) return;
    g_pressure = p / 256.0f;
    if (g_parked) {
        // exponential mean (time constant ~0.8 s) as the ground reference
        g_ref_pa = g_ref_pa <= 0.0f ? g_pressure : g_ref_pa + (g_pressure - g_ref_pa) * 0.05f;
    }
}

float baro_pressure_pa(void) {
    return g_pressure;
}

float baro_temperature_c(void) {
    return g_temperature;
}

float baro_height_m(void) {
    if (g_pressure <= 0.0f || g_ref_pa <= 0.0f) return 0.0f;
    return 44330.0f * (1.0f - powf(g_pressure / g_ref_pa, 0.1903f));
}

void baro_set_parked(bool parked) {
    g_parked = parked;
}
