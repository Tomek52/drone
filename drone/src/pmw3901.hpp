// SPDX-License-Identifier: MIT
//
// [POSHOLD] Driver for the PMW3901MB optical flow sensor on StampFly.
//
// Wiring (docs.m5stack.com StampFly pin map, docs/01-sprzet.md):
//   shared SPI bus with the BMI270 (SPI2_HOST: MOSI G14, SCK G44, MISO G43),
//   chip select "CS2" = G12.
// The bus is initialised by the upstream BMI270 code (lib/bmi270/common.c);
// pmw3901_init() must be called after imu_init().
//
// The register-level init sequence and access timing follow the MIT-licensed
// Bitcraze PMW3901 Arduino driver (Copyright (c) 2017 Bitcraze AB); the motion
// burst layout follows the MIT-licensed Pimoroni pmw3901-python driver.

#ifndef PMW3901_HPP
#define PMW3901_HPP

#include <stdint.h>

struct Pmw3901Motion {
    bool ok             = false;  // SPI transfer done and sensor initialised
    bool motion         = false;  // motion bit set (new movement since last read)
    int16_t dx          = 0;      // counts since the previous read
    int16_t dy          = 0;
    uint8_t squal       = 0;      // surface quality
    uint16_t shutter    = 0;      // exposure; saturates in the dark
};

// Adds the sensor to the SPI bus, resets it, checks the product ID and loads
// the performance register set. Returns false if the sensor is not detected
// (e.g. wrong chip select); Position Hold is then unavailable.
bool pmw3901_init(void);

// True after a successful pmw3901_init().
bool pmw3901_present(void);

// Product ID read during init (0x49 expected), for diagnostics.
uint8_t pmw3901_product_id(void);

// Reads one motion burst (~0.15 ms). Call at ~100 Hz from the control loop.
Pmw3901Motion pmw3901_read_motion(void);

#endif
