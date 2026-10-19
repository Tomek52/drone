// SPDX-License-Identifier: MIT
//
// [POSHOLD] Glue between the upstream firmware (sensor globals, control loop)
// and the hardware-independent Position Hold code in lib/poshold.

#ifndef POSHOLD_GLUE_HPP
#define POSHOLD_GLUE_HPP

#include <position_controller.hpp>
#include <position_estimator.hpp>
#include <poshold_supervisor.hpp>
#include <height_monitor.hpp>
#include <abs_height.hpp>
#include <tof_step.hpp>

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
// Diagnostics: bit n = flow reject reason n occurred in any 100 Hz sample since the
// previous call (clears the mask); cumulative count of rejected samples.
uint8_t poshold_flow_reject_mask_take(void);
uint32_t poshold_flow_reject_total(void);

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

// Stable ABS: height from the barometer replaces the ToF range in the altitude estimator.
bool poshold_abs_active(void);
int16_t poshold_abs_range_mm(void);
float poshold_abs_height(void);
float poshold_alt_ref_max(void);  // altitude setpoint limit: 1.8 m with the ToF, higher in Stable ABS
float poshold_alt_limit(void);    // height above which the upstream code forces a descent
float poshold_alt_meas_std(void); // measurement noise for the altitude estimator

// ToF obstacle edge compensation (absolute height over obstacles).
void poshold_tof_step_sample(int16_t range_mm);  // once per new ToF sample, after the outlier filter
int16_t poshold_tof_offset_mm(void);             // added to the range fed to the altitude estimator
float poshold_tof_offset_m(void);
const poshold::HeightMonitorOutput& poshold_height_monitor_output(void);  // ToF vs barometer, observe-only
const poshold::ControllerOutput& poshold_controller_output(void);

#endif
