// SPDX-License-Identifier: MIT
//
// [POSHOLD] Driver for the BMP280 barometer on StampFly (I2C 0x76, same bus as the ToF sensors).
//
// Read-only for now: pressure, temperature and the height above the pressure measured
// on the ground (docs/10-plany.md, "wysokosc bezwzgledna"). Nothing in the control loop
// uses it yet.

#ifndef BARO_HPP
#define BARO_HPP

#include <stdint.h>

// Detects the sensor (chip id 0x58 / 0x60) and starts continuous conversion
// (pressure x16, temperature x2, IIR x4, ~23 Hz). Returns false if it is absent.
bool baro_init(void);
bool baro_present(void);

// Reads the latest conversion; rate-limited to ~25 Hz internally, cheap to call every loop.
void baro_update(void);

// Pressure [Pa] and temperature [deg C] of the last conversion; 0 if the sensor is absent.
float baro_pressure_pa(void);
float baro_temperature_c(void);

// Height [m] above the reference pressure. The reference is the mean pressure while the
// drone is parked: call with parked = true every loop in PARKING_MODE.
float baro_height_m(void);
void baro_set_parked(bool parked);

#endif
