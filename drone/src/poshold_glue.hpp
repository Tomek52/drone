// SPDX-License-Identifier: MIT
//
// [POSHOLD] Glue between the upstream firmware (sensor globals, control loop)
// and the hardware-independent Position Hold code in lib/poshold.

#ifndef POSHOLD_GLUE_HPP
#define POSHOLD_GLUE_HPP

#include <position_controller.hpp>
#include <position_estimator.hpp>
#include <poshold_supervisor.hpp>

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

// --- Position Hold control and safety (feat/position-controller) -------------
// Before the mode state machine (after rc_link_update()): supervisor,
// failsafes (link loss hover, low battery and geofence landing), controller.
void poshold_safety_update(void);
// Right after get_command() in FLIGHT_MODE: altitude setpoint, yaw rate, no flips.
void poshold_after_get_command(void);
// In angle_control(), after the upstream references are computed.
void poshold_override_attitude(volatile float* roll_ref, volatile float* pitch_ref);
// Upstream link-loss check: may the drone switch to auto landing now?
bool poshold_link_loss_landing_allowed(void);

const poshold::SupervisorOutput& poshold_supervisor_output(void);
const poshold::ControllerOutput& poshold_controller_output(void);

#endif
