// SPDX-License-Identifier: MIT
//
// [POSHOLD] Glue between the upstream firmware (sensor globals, control loop)
// and the hardware-independent Position Hold code in lib/poshold.

#ifndef POSHOLD_GLUE_HPP
#define POSHOLD_GLUE_HPP

#include <position_estimator.hpp>

#include "pmw3901.hpp"

// Call once after sensor_init() (the SPI bus must already be initialised).
void poshold_init(void);

// Call once per 400 Hz loop iteration, right after sensor_read().
void poshold_update_estimation(void);

const poshold::EstimatorState& poshold_estimator_state(void);
const Pmw3901Motion& poshold_last_flow(void);

// Cumulative raw flow counts since boot (for the hand-carry calibration).
int32_t poshold_flow_sum_x(void);
int32_t poshold_flow_sum_y(void);

#endif
