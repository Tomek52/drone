// SPDX-License-Identifier: MIT
//
// [POSHOLD] PMW3901MB driver, see pmw3901.hpp.

#include "pmw3901.hpp"

#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <string.h>

namespace {

constexpr gpio_num_t kPinCs      = GPIO_NUM_12;  // "CS2" in the StampFly pin map
constexpr int kClockHz           = 2000000;      // datasheet maximum SCLK: 2 MHz
constexpr uint8_t kRegProductId  = 0x00;
constexpr uint8_t kRegMotion     = 0x02;
constexpr uint8_t kRegMotionBurst = 0x16;
constexpr uint8_t kRegPowerUpReset = 0x3A;
constexpr uint8_t kRegInvProductId = 0x5F;
constexpr uint8_t kProductId     = 0x49;
constexpr uint8_t kInvProductId  = 0xB6;

spi_device_handle_t g_dev = nullptr;
bool g_present            = false;
uint8_t g_product_id      = 0;

void cs(bool active) {
    gpio_set_level(kPinCs, active ? 0 : 1);
}

// Full-duplex transfer of `len` bytes while CS is held by the caller.
bool xfer(const uint8_t* tx, uint8_t* rx, size_t len) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length    = len * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    return spi_device_polling_transmit(g_dev, &t) == ESP_OK;
}

bool reg_write(uint8_t reg, uint8_t value) {
    WORD_ALIGNED_ATTR uint8_t tx[4] = {(uint8_t)(reg | 0x80u), value, 0, 0};
    if (spi_device_acquire_bus(g_dev, portMAX_DELAY) != ESP_OK) return false;
    cs(true);
    delayMicroseconds(50);
    const bool ok = xfer(tx, nullptr, 2);
    delayMicroseconds(50);
    cs(false);
    spi_device_release_bus(g_dev);
    delayMicroseconds(200);
    return ok;
}

bool reg_read(uint8_t reg, uint8_t* value) {
    WORD_ALIGNED_ATTR uint8_t tx[4] = {(uint8_t)(reg & 0x7Fu), 0, 0, 0};
    WORD_ALIGNED_ATTR uint8_t rx[4] = {0};
    if (spi_device_acquire_bus(g_dev, portMAX_DELAY) != ESP_OK) return false;
    cs(true);
    delayMicroseconds(50);
    bool ok = xfer(tx, nullptr, 1);
    delayMicroseconds(50);
    ok = ok && xfer(tx + 1, rx, 1);
    delayMicroseconds(100);
    cs(false);
    spi_device_release_bus(g_dev);
    *value = rx[0];
    return ok;
}

// Performance optimisation registers (Bitcraze PMW3901 driver, MIT).
void init_registers() {
    static const uint8_t kSeq1[][2] = {
        {0x7F, 0x00}, {0x61, 0xAD}, {0x7F, 0x03}, {0x40, 0x00}, {0x7F, 0x05}, {0x41, 0xB3}, {0x43, 0xF1},
        {0x45, 0x14}, {0x5B, 0x32}, {0x5F, 0x34}, {0x7B, 0x08}, {0x7F, 0x06}, {0x44, 0x1B}, {0x40, 0xBF},
        {0x4E, 0x3F}, {0x7F, 0x08}, {0x65, 0x20}, {0x6A, 0x18}, {0x7F, 0x09}, {0x4F, 0xAF}, {0x5F, 0x40},
        {0x48, 0x80}, {0x49, 0x80}, {0x57, 0x77}, {0x60, 0x78}, {0x61, 0x78}, {0x62, 0x08}, {0x63, 0x50},
        {0x7F, 0x0A}, {0x45, 0x60}, {0x7F, 0x00}, {0x4D, 0x11}, {0x55, 0x80}, {0x74, 0x1F}, {0x75, 0x1F},
        {0x4A, 0x78}, {0x4B, 0x78}, {0x44, 0x08}, {0x45, 0x50}, {0x64, 0xFF}, {0x65, 0x1F}, {0x7F, 0x14},
        {0x65, 0x60}, {0x66, 0x08}, {0x63, 0x78}, {0x7F, 0x15}, {0x48, 0x58}, {0x7F, 0x07}, {0x41, 0x0D},
        {0x43, 0x14}, {0x4B, 0x0E}, {0x45, 0x0F}, {0x44, 0x42}, {0x4C, 0x80}, {0x7F, 0x10}, {0x5B, 0x02},
        {0x7F, 0x07}, {0x40, 0x41}, {0x70, 0x00},
    };
    static const uint8_t kSeq2[][2] = {
        {0x32, 0x44}, {0x7F, 0x07}, {0x40, 0x40}, {0x7F, 0x06}, {0x62, 0xF0}, {0x63, 0x00}, {0x7F, 0x0D},
        {0x48, 0xC0}, {0x6F, 0xD5}, {0x7F, 0x00}, {0x5B, 0xA0}, {0x4E, 0xA8}, {0x5A, 0x50}, {0x40, 0x80},
    };
    for (const auto& w : kSeq1) reg_write(w[0], w[1]);
    delay(100);
    for (const auto& w : kSeq2) reg_write(w[0], w[1]);
}

}  // namespace

bool pmw3901_init(void) {
    g_present = false;

    gpio_reset_pin(kPinCs);
    gpio_set_direction(kPinCs, GPIO_MODE_OUTPUT);
    cs(false);

    if (g_dev == nullptr) {
        spi_device_interface_config_t cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.mode           = 3;  // CPOL = 1, CPHA = 1
        cfg.clock_speed_hz = kClockHz;
        cfg.spics_io_num   = -1;  // CS driven manually: the sensor needs delays inside a transfer
        cfg.queue_size     = 1;
        if (spi_bus_add_device(SPI2_HOST, &cfg, &g_dev) != ESP_OK) {
            USBSerial.printf("[POSHOLD] PMW3901: spi_bus_add_device failed\r\n");
            g_dev = nullptr;
            return false;
        }
    }

    // Reset the sensor's SPI port, then power-up reset.
    cs(true);
    delay(1);
    cs(false);
    delay(1);
    reg_write(kRegPowerUpReset, 0x5A);
    delay(5);

    uint8_t id = 0, inv = 0;
    reg_read(kRegProductId, &id);
    reg_read(kRegInvProductId, &inv);
    g_product_id = id;
    if (id != kProductId || inv != kInvProductId) {
        USBSerial.printf("[POSHOLD] PMW3901 not found (id=0x%02X inv=0x%02X)\r\n", id, inv);
        return false;
    }

    // Clear the motion registers once, as recommended by the reference drivers.
    uint8_t dummy;
    for (uint8_t r = kRegMotion; r <= 0x06; r++) reg_read(r, &dummy);
    delay(1);

    init_registers();
    g_present = true;
    USBSerial.printf("[POSHOLD] PMW3901 ready (id=0x%02X)\r\n", id);
    return true;
}

bool pmw3901_present(void) {
    return g_present;
}

uint8_t pmw3901_product_id(void) {
    return g_product_id;
}

Pmw3901Motion pmw3901_read_motion(void) {
    Pmw3901Motion m;
    if (!g_present) return m;

    // Motion burst: Motion, Observation, Delta_X_L, Delta_X_H, Delta_Y_L,
    // Delta_Y_H, SQUAL, RawData_Sum, Maximum_RawData, Minimum_RawData,
    // Shutter_Upper, Shutter_Lower.
    WORD_ALIGNED_ATTR uint8_t tx[4] = {kRegMotionBurst, 0, 0, 0};
    WORD_ALIGNED_ATTR uint8_t zeros[12] = {0};
    WORD_ALIGNED_ATTR uint8_t rx[12]    = {0};
    if (spi_device_acquire_bus(g_dev, portMAX_DELAY) != ESP_OK) return m;
    cs(true);
    bool ok = xfer(tx, nullptr, 1);
    delayMicroseconds(50);
    ok = ok && xfer(zeros, rx, sizeof(rx));
    cs(false);
    spi_device_release_bus(g_dev);
    if (!ok) return m;

    m.ok      = true;
    m.motion  = (rx[0] & 0x80u) != 0;
    m.dx      = (int16_t)((uint16_t)rx[3] << 8 | rx[2]);
    m.dy      = (int16_t)((uint16_t)rx[5] << 8 | rx[4]);
    m.squal   = rx[6];
    m.shutter = (uint16_t)((uint16_t)rx[10] << 8 | rx[11]);
    return m;
}

void pmw3901_debug_dump(void) {
    if (!g_present) return;
    WORD_ALIGNED_ATTR uint8_t tx[4]     = {kRegMotionBurst, 0, 0, 0};
    WORD_ALIGNED_ATTR uint8_t zeros[12] = {0};
    WORD_ALIGNED_ATTR uint8_t rx[12]    = {0};
    if (spi_device_acquire_bus(g_dev, portMAX_DELAY) != ESP_OK) return;
    cs(true);
    bool ok = xfer(tx, nullptr, 1);
    delayMicroseconds(50);
    ok = ok && xfer(zeros, rx, sizeof(rx));
    cs(false);
    spi_device_release_bus(g_dev);

    // Single register reads: Motion, DeltaX_L/H, DeltaY_L/H, SQUAL, Shutter_U/L.
    static const uint8_t kRegs[] = {0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x0C, 0x0B};
    uint8_t v[sizeof(kRegs)]     = {0};
    for (size_t i = 0; i < sizeof(kRegs); i++) reg_read(kRegs[i], &v[i]);

    USBSerial.printf(
        "\r\n[PMWDBG] ok=%d burst=%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X"
        " regs mot=%02X dxl=%02X dxh=%02X dyl=%02X dyh=%02X sq=%02X shU=%02X shL=%02X\r\n",
        ok, rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7], rx[8], rx[9], rx[10], rx[11], v[0], v[1], v[2],
        v[3], v[4], v[5], v[6], v[7]);
}
