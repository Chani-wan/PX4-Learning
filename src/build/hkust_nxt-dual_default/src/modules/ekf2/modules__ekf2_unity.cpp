/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file control.cpp
 * Control functions for ekf attitude and position estimator.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */


#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlFusionModes(const imuSample &imu_delayed)
{
	// Store the status to enable change detection
	_control_status_prev.value = _control_status.value;
	_state_reset_count_prev = _state_reset_status.reset_count;

	if (_system_flag_buffer) {
		systemFlagUpdate system_flags_delayed;

		if (_system_flag_buffer->pop_first_older_than(imu_delayed.time_us, &system_flags_delayed)) {

			set_vehicle_at_rest(system_flags_delayed.at_rest);
			set_in_air_status(system_flags_delayed.in_air);

			set_is_fixed_wing(system_flags_delayed.is_fixed_wing);

			if (system_flags_delayed.gnd_effect) {
				set_gnd_effect();
			}

			set_constant_pos(system_flags_delayed.constant_pos);
		}
	}

	// monitor the tilt alignment
	if (!_control_status.flags.tilt_align) {
		// whilst we are aligning the tilt, monitor the variances
		// Once the tilt variances have reduced to equivalent of 3 deg uncertainty
		// and declare the tilt alignment complete
		if (getTiltVariance() < sq(math::radians(3.f))) {
			_control_status.flags.tilt_align = true;

			// send alignment status message to the console
			const char *height_source = "unknown";

			if (_control_status.flags.baro_hgt) {
				height_source = "baro";

			} else if (_control_status.flags.ev_hgt) {
				height_source = "ev";

			} else if (_control_status.flags.gps_hgt) {
				height_source = "gps";

			} else if (_control_status.flags.rng_hgt) {
				height_source = "rng";
			}

			ECL_INFO("%llu: EKF aligned, (%s hgt, IMU buf: %i, OBS buf: %i)",
				 (unsigned long long)imu_delayed.time_us, height_source, (int)_imu_buffer_length, (int)_obs_buffer_length);

			ECL_DEBUG("tilt aligned, roll: %.3f, pitch %.3f, yaw: %.3f",
				  (double)matrix::Eulerf(_state.quat_nominal).phi(),
				  (double)matrix::Eulerf(_state.quat_nominal).theta(),
				  (double)matrix::Eulerf(_state.quat_nominal).psi()
				 );
		}
	}

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// control use of observations for aiding
	controlMagFusion(imu_delayed);
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	controlOpticalFlowFusion(imu_delayed);
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_GNSS)
	controlGpsFusion(imu_delayed);
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)
	_aux_global_position.update(*this, imu_delayed);
#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION

#if defined(CONFIG_EKF2_AIRSPEED)
	controlAirDataFusion(imu_delayed);
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	controlBetaFusion(imu_delayed);
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_DRAG_FUSION)
	controlDragFusion(imu_delayed);
#endif // CONFIG_EKF2_DRAG_FUSION

	controlHeightFusion(imu_delayed);

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	controlGravityFusion(imu_delayed);
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// Additional data odometry data from an external estimator can be fused.
	controlExternalVisionFusion(imu_delayed);
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AUXVEL)
	// Additional horizontal velocity data from an auxiliary sensor can be fused
	controlAuxVelFusion(imu_delayed);
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_TERRAIN)
	controlTerrainFakeFusion();
	updateTerrainValidity();
#endif // CONFIG_EKF2_TERRAIN

	controlZeroInnovationHeadingUpdate();

	_zero_velocity_update.update(*this, imu_delayed);

	if (_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::GyroBias)) {
		_zero_gyro_update.update(*this, imu_delayed);
	}

	// Fake position measurement for constraining drift when no other velocity or position measurements
	controlFakePosFusion();
	controlFakeHgtFusion();

	// check if we are no longer fusing measurements that directly constrain velocity drift
	updateDeadReckoningStatus();
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file covariance.cpp
 * Contains functions for initialising, predicting and updating the state
 * covariance matrix
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Roman Bast <bastroman@gmail.com>
 *
 */

#include "ekf.h"
#include <ekf_derivation/generated/predict_covariance.h>

#include <math.h>
#include <mathlib/mathlib.h>

// Sets initial values for the covariance matrix
// Do not call before quaternion states have been initialised
void Ekf::initialiseCovariance()
{
	P.zero();

	resetQuatCov(0.f); // Start with no initial uncertainty to improve fine leveling through zero vel/pos fusion

	// velocity
#if defined(CONFIG_EKF2_GNSS)
	const float vel_var = sq(fmaxf(_params.gps_vel_noise, 0.01f));
#else
	const float vel_var = sq(0.5f);
#endif
	P.uncorrelateCovarianceSetVariance<State::vel.dof>(State::vel.idx, Vector3f(vel_var, vel_var, sq(1.5f) * vel_var));

	// position
#if defined(CONFIG_EKF2_BAROMETER)
	float z_pos_var = sq(fmaxf(_params.baro_noise, 0.01f));
#else
	float z_pos_var = sq(1.f);
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_GNSS)
	const float xy_pos_var = sq(fmaxf(_params.gps_pos_noise, 0.01f));

	if (_control_status.flags.gps_hgt) {
		z_pos_var = sq(fmaxf(1.5f * _params.gps_pos_noise, 0.01f));
	}

#else
	const float xy_pos_var = sq(fmaxf(_params.pos_noaid_noise, 0.01f));
#endif

#if defined(CONFIG_EKF2_RANGE_FINDER)

	if (_control_status.flags.rng_hgt) {
		z_pos_var = sq(fmaxf(_params.range_noise, 0.01f));
	}

#endif // CONFIG_EKF2_RANGE_FINDER

	P.uncorrelateCovarianceSetVariance<State::pos.dof>(State::pos.idx, Vector3f(xy_pos_var, xy_pos_var, z_pos_var));

	resetGyroBiasCov();

	resetAccelBiasCov();

#if defined(CONFIG_EKF2_MAGNETOMETER)
	resetMagEarthCov();
	resetMagBiasCov();
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	resetWindCov();
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_TERRAIN)
	// use the ground clearance value as our uncertainty
	P.uncorrelateCovarianceSetVariance<State::terrain.dof>(State::terrain.idx, sq(_params.rng_gnd_clearance));
#endif // CONFIG_EKF2_TERRAIN
}

void Ekf::predictCovariance(const imuSample &imu_delayed)
{
	// predict the covariance
	const float dt = 0.5f * (imu_delayed.delta_vel_dt + imu_delayed.delta_ang_dt);

	// gyro noise variance
	float gyro_noise = _params.gyro_noise;
	const float gyro_var = sq(gyro_noise);

	// accel noise variance
	float accel_noise = _params.accel_noise;
	Vector3f accel_var;

	for (unsigned i = 0; i < 3; i++) {
		if (_fault_status.flags.bad_acc_vertical || imu_delayed.delta_vel_clipping[i]) {
			// Increase accelerometer process noise if bad accel data is detected
			accel_var(i) = sq(BADACC_BIAS_PNOISE);

		} else {
			accel_var(i) = sq(accel_noise);
		}
	}

	// calculate variances and upper diagonal covariances for quaternion, velocity, position and gyro bias states
	P = sym::PredictCovariance(_state.vector(), P,
				   imu_delayed.delta_vel / imu_delayed.delta_vel_dt, accel_var,
				   imu_delayed.delta_ang / imu_delayed.delta_ang_dt, gyro_var,
				   dt);

	// Construct the process noise variance diagonal for those states with a stationary process model
	// These are kinematic states and their error growth is controlled separately by the IMU noise variances

	// gyro bias: add process noise
	{
		const float gyro_bias_sig = dt * _params.gyro_bias_p_noise;
		const float gyro_bias_process_noise = sq(gyro_bias_sig);

		for (unsigned index = 0; index < State::gyro_bias.dof; index++) {
			const unsigned i = State::gyro_bias.idx + index;

			if (P(i, i) < gyro_var) {
				P(i, i) += gyro_bias_process_noise;
			}
		}
	}

	// accel bias: add process noise
	{
		const float accel_bias_sig = dt * _params.accel_bias_p_noise;
		const float accel_bias_process_noise = sq(accel_bias_sig);

		for (unsigned index = 0; index < State::accel_bias.dof; index++) {
			const unsigned i = State::accel_bias.idx + index;

			if (P(i, i) < accel_var(index)) {
				P(i, i) += accel_bias_process_noise;
			}
		}
	}


#if defined(CONFIG_EKF2_MAGNETOMETER)
	// mag_I: add process noise
	float mag_I_sig = dt * _params.mage_p_noise;
	float mag_I_process_noise = sq(mag_I_sig);

	for (unsigned index = 0; index < State::mag_I.dof; index++) {
		const unsigned i = State::mag_I.idx + index;

		if (P(i, i) < sq(_params.mag_noise)) {
			P(i, i) += mag_I_process_noise;
		}
	}

	// mag_B: add process noise
	float mag_B_sig = dt * _params.magb_p_noise;
	float mag_B_process_noise = sq(mag_B_sig);

	for (unsigned index = 0; index < State::mag_B.dof; index++) {
		const unsigned i = State::mag_B.idx + index;

		if (P(i, i) < sq(_params.mag_noise)) {
			P(i, i) += mag_B_process_noise;
		}
	}

#endif // CONFIG_EKF2_MAGNETOMETER


#if defined(CONFIG_EKF2_WIND)

	// wind vel: add process noise
	const float height_rate = _height_rate_lpf.update(_state.vel(2), imu_delayed.delta_vel_dt);
	const float wind_vel_nsd_scaled = _params.wind_vel_nsd * (1.f + _params.wind_vel_nsd_scaler * fabsf(height_rate));
	const float wind_vel_process_noise = sq(wind_vel_nsd_scaled) * dt;

	for (unsigned index = 0; index < State::wind_vel.dof; index++) {
		const unsigned i = State::wind_vel.idx + index;

		if (P(i, i) < sq(_params.initial_wind_uncertainty)) {
			P(i, i) += wind_vel_process_noise;
		}
	}

#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_TERRAIN)

	if (_height_sensor_ref != HeightSensor::RANGE) {
		// predict the state variance growth where the state is the vertical position of the terrain underneath the vehicle
		// process noise due to errors in vehicle height estimate
		float terrain_process_noise = sq(imu_delayed.delta_vel_dt * _params.terrain_p_noise);

		// process noise due to terrain gradient
		terrain_process_noise += sq(imu_delayed.delta_vel_dt * _params.terrain_gradient) * (sq(_state.vel(0)) + sq(_state.vel(
						 1)));
		P(State::terrain.idx, State::terrain.idx) += terrain_process_noise;
	}

#endif // CONFIG_EKF2_TERRAIN

	// covariance matrix is symmetrical, so copy upper half to lower half
	for (unsigned row = 0; row < State::size; row++) {
		for (unsigned column = 0; column < row; column++) {
			P(row, column) = P(column, row);
		}
	}

	constrainStateVariances();
}

void Ekf::constrainStateVariances()
{
	// NOTE: This limiting is a last resort and should not be relied on
	// TODO: Split covariance prediction into separate F*P*transpose(F) and Q contributions
	// and set corresponding entries in Q to zero when states exceed 50% of the limit
	// Covariance diagonal limits. Use same values for states which
	// belong to the same group (e.g. vel_x, vel_y, vel_z)

	constrainStateVar(State::quat_nominal, 1e-9f, 1.f);
	constrainStateVar(State::vel, 1e-6f, 1e6f);
	constrainStateVar(State::pos, 1e-6f, 1e6f);
	constrainStateVarLimitRatio(State::gyro_bias, kGyroBiasVarianceMin, 1.f);
	constrainStateVarLimitRatio(State::accel_bias, kAccelBiasVarianceMin, 1.f);

#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (_control_status.flags.mag) {
		constrainStateVarLimitRatio(State::mag_I, kMagVarianceMin, 1.f);
		constrainStateVarLimitRatio(State::mag_B, kMagVarianceMin, 1.f);
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)

	if (_control_status.flags.wind) {
		constrainStateVarLimitRatio(State::wind_vel, 1e-6f, 1e6f);
	}

#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_TERRAIN)
	constrainStateVarLimitRatio(State::terrain, 0.f, 1e4f);
#endif // CONFIG_EKF2_TERRAIN
}

void Ekf::constrainStateVar(const IdxDof &state, float min, float max)
{
	for (unsigned i = state.idx; i < (state.idx + state.dof); i++) {
		if (P(i, i) < min) {
			P(i, i) = min;

		} else if (P(i, i) > max) {
			// Constrain the variance growth by fusing zero innovation as clipping the variance
			// would artifically increase the correlation between states and destabilize the filter.
			const float innov = 0.f;
			const float R = 10.f * P(i, i); // This reduces the variance by ~10% as K = P / (P + R)
			const float innov_var = P(i, i) + R;
			fuseDirectStateMeasurement(innov, innov_var, R, i);
		}
	}
}

void Ekf::constrainStateVarLimitRatio(const IdxDof &state, float min, float max, float max_ratio)
{
	// the ratio of a max and min variance must not exceed max_ratio
	float state_var_max = 0.f;

	for (unsigned i = state.idx; i < (state.idx + state.dof); i++) {
		if (P(i, i) > state_var_max) {
			state_var_max = P(i, i);
		}
	}

	float limited_max = math::constrain(state_var_max, min, max);
	float limited_min = math::constrain(limited_max / max_ratio, min, max);

	constrainStateVar(state, limited_min, limited_max);
}

void Ekf::resetQuatCov(const float yaw_noise)
{
	const float tilt_var = sq(math::max(_params.initial_tilt_err, 0.01f));
	float yaw_var = sq(0.01f);

	// update the yaw angle variance using the variance of the measurement
	if (PX4_ISFINITE(yaw_noise)) {
		// using magnetic heading tuning parameter
		yaw_var = sq(yaw_noise);
	}

	resetQuatCov(Vector3f(tilt_var, tilt_var, yaw_var));
}

void Ekf::resetQuatCov(const Vector3f &rot_var_ned)
{
	P.uncorrelateCovarianceSetVariance<State::quat_nominal.dof>(State::quat_nominal.idx, rot_var_ned);
}

void Ekf::resetGyroBiasCov()
{
	// Zero the corresponding covariances and set
	// variances to the values use for initial alignment
	P.uncorrelateCovarianceSetVariance<State::gyro_bias.dof>(State::gyro_bias.idx, sq(_params.switch_on_gyro_bias));
}

void Ekf::resetGyroBiasZCov()
{
	P.uncorrelateCovarianceSetVariance<1>(State::gyro_bias.idx + 2, sq(_params.switch_on_gyro_bias));
}

void Ekf::resetAccelBiasCov()
{
	// Zero the corresponding covariances and set
	// variances to the values use for initial alignment
	P.uncorrelateCovarianceSetVariance<State::accel_bias.dof>(State::accel_bias.idx, sq(_params.switch_on_accel_bias));
}

#if defined(CONFIG_EKF2_MAGNETOMETER)
void Ekf::resetMagEarthCov()
{
	ECL_INFO("reset mag earth covariance");

	P.uncorrelateCovarianceSetVariance<State::mag_I.dof>(State::mag_I.idx, sq(_params.mag_noise));
}

void Ekf::resetMagBiasCov()
{
	ECL_INFO("reset mag bias covariance");

	P.uncorrelateCovarianceSetVariance<State::mag_B.dof>(State::mag_B.idx, sq(_params.mag_noise));
}
#endif // CONFIG_EKF2_MAGNETOMETER
/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ekf.cpp
 * Core functions for ekf attitude and position estimator.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 */

#include "ekf.h"

#include <mathlib/mathlib.h>

bool Ekf::init(uint64_t timestamp)
{
	if (!_initialised) {
		_initialised = initialise_interface(timestamp);
		reset();
	}

	return _initialised;
}

void Ekf::reset()
{
	ECL_INFO("reset");

	_state.quat_nominal.setIdentity();
	_state.vel.setZero();
	_state.pos.setZero();
	_state.gyro_bias.setZero();
	_state.accel_bias.setZero();

#if defined(CONFIG_EKF2_MAGNETOMETER)
	_state.mag_I.setZero();
	_state.mag_B.setZero();
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	_state.wind_vel.setZero();
#endif // CONFIG_EKF2_WIND
	//
#if defined(CONFIG_EKF2_TERRAIN)
	// assume a ground clearance
	_state.terrain = -_gpos.altitude() + _params.rng_gnd_clearance;
#endif // CONFIG_EKF2_TERRAIN

#if defined(CONFIG_EKF2_RANGE_FINDER)
	_range_sensor.setPitchOffset(_params.rng_sens_pitch);
	_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
	_range_sensor.setQualityHysteresis(_params.range_valid_quality_s);
	_range_sensor.setMaxFogDistance(_params.rng_fog);
#endif // CONFIG_EKF2_RANGE_FINDER

	_control_status.value = 0;
	_control_status_prev.value = 0;

	_control_status.flags.in_air = true;
	_control_status_prev.flags.in_air = true;

	_fault_status.value = 0;
	_innov_check_fail_status.value = 0;

#if defined(CONFIG_EKF2_GNSS)
	resetGpsDriftCheckFilters();
	_gps_checks_passed = false;
#endif // CONFIG_EKF2_GNSS
	_local_origin_alt = NAN;

	_output_predictor.reset();

	// Ekf private fields
	_time_last_horizontal_aiding = 0;
	_time_last_v_pos_aiding = 0;
	_time_last_v_vel_aiding = 0;

	_time_last_hor_pos_fuse = 0;
	_time_last_hgt_fuse = 0;
	_time_last_hor_vel_fuse = 0;
	_time_last_ver_vel_fuse = 0;
	_time_last_heading_fuse = 0;
	_time_last_terrain_fuse = 0;

	_last_known_gpos.setZero();

#if defined(CONFIG_EKF2_BAROMETER)
	_baro_counter = 0;
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_MAGNETOMETER)
	_mag_counter = 0;
#endif // CONFIG_EKF2_MAGNETOMETER

	_time_bad_vert_accel = 0;
	_time_good_vert_accel = 0;

	for (auto &clip_count : _clip_counter) {
		clip_count = 0;
	}

	_zero_velocity_update.reset();

	updateParameters();
}

bool Ekf::update()
{
	if (!_filter_initialised) {
		_filter_initialised = initialiseFilter();

		if (!_filter_initialised) {
			return false;
		}
	}

	// Only run the filter if IMU data in the buffer has been updated
	if (_imu_updated) {
		_imu_updated = false;

		// get the oldest IMU data from the buffer
		// TODO: explicitly pop at desired time horizon
		const imuSample imu_sample_delayed = _imu_buffer.get_oldest();

		// calculate an average filter update time
		//  filter and limit input between -50% and +100% of nominal value
		float input = 0.5f * (imu_sample_delayed.delta_vel_dt + imu_sample_delayed.delta_ang_dt);
		float filter_update_s = 1e-6f * _params.filter_update_interval_us;
		_dt_ekf_avg = 0.99f * _dt_ekf_avg + 0.01f * math::constrain(input, 0.5f * filter_update_s, 2.f * filter_update_s);

		updateIMUBiasInhibit(imu_sample_delayed);

		// perform state and covariance prediction for the main filter
		predictCovariance(imu_sample_delayed);
		predictState(imu_sample_delayed);

		// control fusion of observation data
		controlFusionModes(imu_sample_delayed);

		_output_predictor.correctOutputStates(imu_sample_delayed.time_us, _state.quat_nominal, _state.vel, _gpos,
						      _state.gyro_bias, _state.accel_bias);

		return true;
	}

	return false;
}

bool Ekf::initialiseFilter()
{
	// Filter accel for tilt initialization
	const imuSample &imu_init = _imu_buffer.get_newest();

	// protect against zero data
	if (imu_init.delta_vel_dt < 1e-4f || imu_init.delta_ang_dt < 1e-4f) {
		return false;
	}

	if (_is_first_imu_sample) {
		_accel_lpf.reset(imu_init.delta_vel / imu_init.delta_vel_dt);
		_gyro_lpf.reset(imu_init.delta_ang / imu_init.delta_ang_dt);
		_is_first_imu_sample = false;

	} else {
		_accel_lpf.update(imu_init.delta_vel / imu_init.delta_vel_dt);
		_gyro_lpf.update(imu_init.delta_ang / imu_init.delta_ang_dt);
	}

	if (!initialiseTilt()) {
		return false;
	}

	// initialise the state covariance matrix now we have starting values for all the states
	initialiseCovariance();

	// reset the output predictor state history to match the EKF initial values
	_output_predictor.alignOutputFilter(_state.quat_nominal, _state.vel, _gpos);

	return true;
}

bool Ekf::initialiseTilt()
{
	const float accel_norm = _accel_lpf.getState().norm();
	const float gyro_norm = _gyro_lpf.getState().norm();

	if (accel_norm < 0.8f * CONSTANTS_ONE_G ||
	    accel_norm > 1.2f * CONSTANTS_ONE_G ||
	    gyro_norm > math::radians(15.0f)) {
		return false;
	}

	// get initial tilt estimate from delta velocity vector, assuming vehicle is static
	_state.quat_nominal = Quatf(_accel_lpf.getState(), Vector3f(0.f, 0.f, -1.f));
	_R_to_earth = Dcmf(_state.quat_nominal);

	return true;
}

void Ekf::predictState(const imuSample &imu_delayed)
{
	if (std::fabs(_gpos.latitude_rad() - _earth_rate_lat_ref_rad) > math::radians(1.0)) {
		_earth_rate_lat_ref_rad = _gpos.latitude_rad();
		_earth_rate_NED = calcEarthRateNED((float)_earth_rate_lat_ref_rad);
	}

	// apply imu bias corrections
	const Vector3f delta_ang_bias_scaled = getGyroBias() * imu_delayed.delta_ang_dt;
	Vector3f corrected_delta_ang = imu_delayed.delta_ang - delta_ang_bias_scaled;

	// subtract component of angular rate due to earth rotation
	corrected_delta_ang -= _R_to_earth.transpose() * _earth_rate_NED * imu_delayed.delta_ang_dt;

	const Quatf dq(AxisAnglef{corrected_delta_ang});

	// rotate the previous quaternion by the delta quaternion using a quaternion multiplication
	_state.quat_nominal = (_state.quat_nominal * dq).normalized();
	_R_to_earth = Dcmf(_state.quat_nominal);

	// Calculate an earth frame delta velocity
	const Vector3f delta_vel_bias_scaled = getAccelBias() * imu_delayed.delta_vel_dt;
	const Vector3f corrected_delta_vel = imu_delayed.delta_vel - delta_vel_bias_scaled;
	const Vector3f corrected_delta_vel_ef = _R_to_earth * corrected_delta_vel;

	// save the previous value of velocity so we can use trapzoidal integration
	const Vector3f vel_last = _state.vel;

	// calculate the increment in velocity using the current orientation
	_state.vel += corrected_delta_vel_ef;

	// compensate for acceleration due to gravity, Coriolis and transport rate
	const Vector3f gravity_acceleration(0.f, 0.f, CONSTANTS_ONE_G); // simplistic model
	const Vector3f coriolis_acceleration = -2.f * _earth_rate_NED.cross(vel_last);
	const Vector3f transport_rate = -_gpos.computeAngularRateNavFrame(vel_last).cross(vel_last);
	_state.vel += (gravity_acceleration + coriolis_acceleration + transport_rate) * imu_delayed.delta_vel_dt;

	// predict position states via trapezoidal integration of velocity
	_gpos += (vel_last + _state.vel) * imu_delayed.delta_vel_dt * 0.5f;
	_state.pos(2) = -_gpos.altitude();

	// constrain states
	_state.vel = matrix::constrain(_state.vel, -_params.velocity_limit, _params.velocity_limit);

	// calculate a filtered horizontal acceleration this are used for manoeuvre detection elsewhere
	_accel_horiz_lpf.update(corrected_delta_vel_ef.xy() / imu_delayed.delta_vel_dt, imu_delayed.delta_vel_dt);
}

bool Ekf::resetGlobalPosToExternalObservation(const double latitude, const double longitude, const float altitude,
		const float eph,
		const float epv, uint64_t timestamp_observation)
{
	if (!checkLatLonValidity(latitude, longitude)) {
		return false;
	}

	if (!_local_origin_lat_lon.isInitialized()) {
		if (!resetLatLonTo(latitude, longitude, sq(eph))) {
			return false;
		}

		initialiseAltitudeTo(altitude, sq(epv));

		return true;
	}

	Vector3f pos_correction;

	// apply a first order correction using velocity at the delayed time horizon and the delta time
	if ((timestamp_observation > 0) && isLocalHorizontalPositionValid()) {

		timestamp_observation = math::min(_time_latest_us, timestamp_observation);

		float dt_us;

		if (_time_delayed_us >= timestamp_observation) {
			dt_us = static_cast<float>(_time_delayed_us - timestamp_observation);

		} else {
			dt_us = -static_cast<float>(timestamp_observation - _time_delayed_us);
		}

		const float dt_s = dt_us * 1e-6f;
		pos_correction = _state.vel * dt_s;
	}

	LatLonAlt gpos(latitude, longitude, altitude);
	bool alt_valid = true;

	if (!checkAltitudeValidity(gpos.altitude())) {
		gpos.setAltitude(_gpos.altitude());
		alt_valid = false;
	}

	const LatLonAlt gpos_corrected = gpos + pos_correction;

	{
		const float obs_var = math::max(sq(eph), sq(0.01f));

		const Vector2f innov = (_gpos - gpos_corrected).xy();
		const Vector2f innov_var = Vector2f(getStateVariance<State::pos>()) + obs_var;

		const float sq_gate = sq(5.f); // magic hardcoded gate
		const float test_ratio = sq(innov(0)) / (sq_gate * innov_var(0)) + sq(innov(1)) / (sq_gate * innov_var(1));

		const bool innov_rejected = (test_ratio > 1.f);

		if (!_control_status.flags.in_air || (eph > 0.f && eph < 1.f) || innov_rejected) {
			// when on ground or accuracy chosen to be very low, we hard reset position
			// this allows the user to still send hard resets at any time
			ECL_INFO("reset position to external observation");
			_information_events.flags.reset_pos_to_ext_obs = true;

			resetHorizontalPositionTo(gpos_corrected.latitude_deg(), gpos_corrected.longitude_deg(), obs_var);
			_last_known_gpos.setLatLon(gpos_corrected);

		} else {
			ECL_INFO("fuse external observation as position measurement");
			fuseDirectStateMeasurement(innov(0), innov_var(0), obs_var, State::pos.idx + 0);
			fuseDirectStateMeasurement(innov(1), innov_var(1), obs_var, State::pos.idx + 1);

			// Use the reset counters to inform the controllers about a potential large position jump
			// TODO: compute the actual position change
			_state_reset_status.reset_count.posNE++;
			_state_reset_status.posNE_change.zero();

			_time_last_hor_pos_fuse = _time_delayed_us;
			_last_known_gpos.setLatLon(gpos_corrected);
		}
	}

	if (alt_valid) {
		const float obs_var = math::max(sq(epv), sq(0.01f));

		ECL_INFO("reset height to external observation");
		initialiseAltitudeTo(gpos_corrected.altitude(), obs_var);
		_last_known_gpos.setAltitude(gpos_corrected.altitude());
	}

	return true;
}

void Ekf::updateParameters()
{
	_params.gyro_noise = math::constrain(_params.gyro_noise, 0.f, 1.f);
	_params.accel_noise = math::constrain(_params.accel_noise, 0.f, 1.f);

	_params.gyro_bias_p_noise = math::constrain(_params.gyro_bias_p_noise, 0.f, 1.f);
	_params.accel_bias_p_noise = math::constrain(_params.accel_bias_p_noise, 0.f, 1.f);

#if defined(CONFIG_EKF2_MAGNETOMETER)
	_params.mage_p_noise = math::constrain(_params.mage_p_noise, 0.f, 1.f);
	_params.magb_p_noise = math::constrain(_params.magb_p_noise, 0.f, 1.f);
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	_params.wind_vel_nsd = math::constrain(_params.wind_vel_nsd, 0.f, 1.f);
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)
	_aux_global_position.updateParameters();
#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION
}

template<typename T>
static void printRingBuffer(const char *name, RingBuffer<T> *rb)
{
	if (rb) {
		printf("%s: %d/%d entries (%d/%d Bytes) (%zu Bytes per entry)\n",
		       name,
		       rb->entries(), rb->get_length(), rb->get_used_size(), rb->get_total_size(),
		       sizeof(T));
	}
}

void Ekf::print_status()
{
	printf("\nStates: (%.4f seconds ago)\n", (_time_latest_us - _time_delayed_us) * 1e-6);
	printf("Orientation (%d-%d): [%.3f, %.3f, %.3f, %.3f] (Euler [%.1f, %.1f, %.1f] deg) var: [%.1e, %.1e, %.1e]\n",
	       State::quat_nominal.idx, State::quat_nominal.idx + State::quat_nominal.dof - 1,
	       (double)_state.quat_nominal(0), (double)_state.quat_nominal(1), (double)_state.quat_nominal(2),
	       (double)_state.quat_nominal(3),
	       (double)math::degrees(matrix::Eulerf(_state.quat_nominal).phi()),
	       (double)math::degrees(matrix::Eulerf(_state.quat_nominal).theta()),
	       (double)math::degrees(matrix::Eulerf(_state.quat_nominal).psi()),
	       (double)getStateVariance<State::quat_nominal>()(0), (double)getStateVariance<State::quat_nominal>()(1),
	       (double)getStateVariance<State::quat_nominal>()(2)
	      );

	printf("Velocity (%d-%d): [%.3f, %.3f, %.3f] var: [%.1e, %.1e, %.1e]\n",
	       State::vel.idx, State::vel.idx + State::vel.dof - 1,
	       (double)_state.vel(0), (double)_state.vel(1), (double)_state.vel(2),
	       (double)getStateVariance<State::vel>()(0), (double)getStateVariance<State::vel>()(1),
	       (double)getStateVariance<State::vel>()(2)
	      );

	const Vector3f position = getPosition();
	printf("Position (%d-%d): [%.3f, %.3f, %.3f] var: [%.1e, %.1e, %.1e]\n",
	       State::pos.idx, State::pos.idx + State::pos.dof - 1,
	       (double)position(0), (double)position(1), (double) position(2),
	       (double)getStateVariance<State::pos>()(0), (double)getStateVariance<State::pos>()(1),
	       (double)getStateVariance<State::pos>()(2)
	      );

	printf("Gyro Bias (%d-%d): [%.6f, %.6f, %.6f] var: [%.1e, %.1e, %.1e]\n",
	       State::gyro_bias.idx, State::gyro_bias.idx + State::gyro_bias.dof - 1,
	       (double)_state.gyro_bias(0), (double)_state.gyro_bias(1), (double)_state.gyro_bias(2),
	       (double)getStateVariance<State::gyro_bias>()(0), (double)getStateVariance<State::gyro_bias>()(1),
	       (double)getStateVariance<State::gyro_bias>()(2)
	      );

	printf("Accel Bias (%d-%d): [%.6f, %.6f, %.6f] var: [%.1e, %.1e, %.1e]\n",
	       State::accel_bias.idx, State::accel_bias.idx + State::accel_bias.dof - 1,
	       (double)_state.accel_bias(0), (double)_state.accel_bias(1), (double)_state.accel_bias(2),
	       (double)getStateVariance<State::accel_bias>()(0), (double)getStateVariance<State::accel_bias>()(1),
	       (double)getStateVariance<State::accel_bias>()(2)
	      );

#if defined(CONFIG_EKF2_MAGNETOMETER)
	printf("Magnetic Field (%d-%d): [%.3f, %.3f, %.3f] var: [%.1e, %.1e, %.1e]\n",
	       State::mag_I.idx, State::mag_I.idx + State::mag_I.dof - 1,
	       (double)_state.mag_I(0), (double)_state.mag_I(1), (double)_state.mag_I(2),
	       (double)getStateVariance<State::mag_I>()(0), (double)getStateVariance<State::mag_I>()(1),
	       (double)getStateVariance<State::mag_I>()(2)
	      );

	printf("Magnetic Bias (%d-%d): [%.3f, %.3f, %.3f] var: [%.1e, %.1e, %.1e]\n",
	       State::mag_B.idx, State::mag_B.idx + State::mag_B.dof - 1,
	       (double)_state.mag_B(0), (double)_state.mag_B(1), (double)_state.mag_B(2),
	       (double)getStateVariance<State::mag_B>()(0), (double)getStateVariance<State::mag_B>()(1),
	       (double)getStateVariance<State::mag_B>()(2)
	      );
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	printf("Wind velocity (%d-%d): [%.3f, %.3f] var: [%.1e, %.1e]\n",
	       State::wind_vel.idx, State::wind_vel.idx + State::wind_vel.dof - 1,
	       (double)_state.wind_vel(0), (double)_state.wind_vel(1),
	       (double)getStateVariance<State::wind_vel>()(0), (double)getStateVariance<State::wind_vel>()(1)
	      );
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_TERRAIN)
	printf("Terrain position (%d): %.3f var: %.1e\n",
	       State::terrain.idx,
	       (double)_state.terrain,
	       (double)getStateVariance<State::terrain>()(0)
	      );
#endif // CONFIG_EKF2_TERRAIN

	printf("\nP:\n");
	P.print();

	printf("EKF average dt: %.6f seconds\n", (double)_dt_ekf_avg);
	printf("minimum observation interval %d us\n", _min_obs_interval_us);

	printRingBuffer("IMU buffer", &_imu_buffer);
	printRingBuffer("system flag buffer", _system_flag_buffer);

#if defined(CONFIG_EKF2_AIRSPEED)
	printRingBuffer("airspeed buffer", _airspeed_buffer);
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_AUXVEL)
	printRingBuffer("aux vel buffer", _auxvel_buffer);
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_BAROMETER)
	printRingBuffer("baro buffer", _baro_buffer);
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_DRAG_FUSION)
	printRingBuffer("drag buffer", _drag_buffer);
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	printRingBuffer("ext vision buffer", _ext_vision_buffer);
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)
	printRingBuffer("gps buffer", _gps_buffer);
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_MAGNETOMETER)
	printRingBuffer("mag buffer", _mag_buffer);
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	printRingBuffer("flow buffer", _flow_buffer);
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_RANGE_FINDER)
	printRingBuffer("range buffer", _range_buffer);
#endif // CONFIG_EKF2_RANGE_FINDER


	_output_predictor.print_status();
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ekf_helper.cpp
 * Definition of ekf helper functions.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>
#include <lib/world_magnetic_model/geo_mag_declination.h>
#include <cstdlib>

bool Ekf::isHeightResetRequired() const
{
	// check if height is continuously failing because of accel errors
	const bool continuous_bad_accel_hgt = isTimedOut(_time_good_vert_accel, (uint64_t)_params.bad_acc_reset_delay_us);

	// check if height has been inertial deadreckoning for too long
	const bool hgt_fusion_timeout = isTimedOut(_time_last_hgt_fuse, _params.hgt_fusion_timeout_max);

	return (continuous_bad_accel_hgt || hgt_fusion_timeout);
}

Vector3f Ekf::calcEarthRateNED(float lat_rad) const
{
	return Vector3f(CONSTANTS_EARTH_SPIN_RATE * cosf(lat_rad),
			0.0f,
			-CONSTANTS_EARTH_SPIN_RATE * sinf(lat_rad));
}

void Ekf::getEkfGlobalOrigin(uint64_t &origin_time, double &latitude, double &longitude, float &origin_alt) const
{
	origin_time = _local_origin_lat_lon.getProjectionReferenceTimestamp();
	latitude = _local_origin_lat_lon.getProjectionReferenceLat();
	longitude = _local_origin_lat_lon.getProjectionReferenceLon();
	origin_alt  = getEkfGlobalOriginAltitude();
}

bool Ekf::checkLatLonValidity(const double latitude, const double longitude)
{
	const bool lat_valid = (PX4_ISFINITE(latitude) && (abs(latitude) <= 90));
	const bool lon_valid = (PX4_ISFINITE(longitude) && (abs(longitude) <= 180));

	return (lat_valid && lon_valid);
}

bool Ekf::checkAltitudeValidity(const float altitude)
{
	// sanity check valid altitude anywhere between the Mariana Trench and edge of Space
	return (PX4_ISFINITE(altitude) && ((altitude > -12'000.f) && (altitude < 100'000.f)));
}

bool Ekf::setEkfGlobalOrigin(const double latitude, const double longitude, const float altitude, const float hpos_var,
			     const float vpos_var)
{
	if (!setLatLonOrigin(latitude, longitude, hpos_var)) {
		return false;
	}

	// altitude is optional
	setAltOrigin(altitude, vpos_var);

	return true;
}

bool Ekf::setLatLonOrigin(const double latitude, const double longitude, const float hpos_var)
{
	if (!checkLatLonValidity(latitude, longitude)) {
		return false;
	}

	if (!_local_origin_lat_lon.isInitialized() && isLocalHorizontalPositionValid()) {
		// Already navigating in a local frame, use the origin to initialize global position
		const Vector2f pos_prev = getLocalHorizontalPosition();
		_local_origin_lat_lon.initReference(latitude, longitude, _time_delayed_us);
		double new_latitude;
		double new_longitude;
		_local_origin_lat_lon.reproject(pos_prev(0), pos_prev(1), new_latitude, new_longitude);
		resetHorizontalPositionTo(new_latitude, new_longitude, hpos_var);

	} else {
		// Simply move the origin and compute the change in local position
		const Vector2f pos_prev = getLocalHorizontalPosition();
		_local_origin_lat_lon.initReference(latitude, longitude, _time_delayed_us);
		const Vector2f pos_new = getLocalHorizontalPosition();
		const Vector2f delta_pos = pos_new - pos_prev;
		updateHorizontalPositionResetStatus(delta_pos);
	}

	return true;
}

bool Ekf::setAltOrigin(const float altitude, const float vpos_var)
{
	if (!checkAltitudeValidity(altitude)) {
		return false;
	}

	ECL_INFO("EKF origin altitude %.1fm -> %.1fm", (double)_local_origin_alt,
		 (double)altitude);

	if (!PX4_ISFINITE(_local_origin_alt) && isLocalVerticalPositionValid()) {
		const float local_alt_prev = _gpos.altitude();
		_local_origin_alt = altitude;
		resetAltitudeTo(local_alt_prev + _local_origin_alt);

	} else {
		const float delta_origin_alt = altitude - _local_origin_alt;
		_local_origin_alt = altitude;
		updateVerticalPositionResetStatus(-delta_origin_alt);

#if defined(CONFIG_EKF2_TERRAIN)
		updateTerrainResetStatus(-delta_origin_alt);
#endif // CONFIG_EKF2_TERRAIN
	}

	return true;
}

bool Ekf::resetGlobalPositionTo(const double latitude, const double longitude, const float altitude,
				const float hpos_var, const float vpos_var)
{
	if (!resetLatLonTo(latitude, longitude, hpos_var)) {
		return false;
	}

	// altitude is optional
	initialiseAltitudeTo(altitude, vpos_var);

	return true;
}

bool Ekf::resetLatLonTo(const double latitude, const double longitude, const float hpos_var)
{
	if (!checkLatLonValidity(latitude, longitude)) {
		return false;
	}

	Vector2f pos_prev;

	if (!_local_origin_lat_lon.isInitialized()) {
		MapProjection zero_ref;
		zero_ref.initReference(0.0, 0.0);
		pos_prev = zero_ref.project(_gpos.latitude_deg(), _gpos.longitude_deg());

		_local_origin_lat_lon.initReference(latitude, longitude, _time_delayed_us);

		// if we are already doing aiding, correct for the change in position since the EKF started navigating
		if (isLocalHorizontalPositionValid()) {
			double est_lat;
			double est_lon;
			_local_origin_lat_lon.reproject(-pos_prev(0), -pos_prev(1), est_lat, est_lon);
			_local_origin_lat_lon.initReference(est_lat, est_lon, _time_delayed_us);
		}

		ECL_INFO("Origin set to lat=%.6f, lon=%.6f",
			 _local_origin_lat_lon.getProjectionReferenceLat(), _local_origin_lat_lon.getProjectionReferenceLon());

	} else {
		pos_prev = _local_origin_lat_lon.project(_gpos.latitude_deg(), _gpos.longitude_deg());
	}

	_gpos.setLatLonDeg(latitude, longitude);
	_output_predictor.resetLatLonTo(latitude, longitude);

	const Vector2f delta_horz_pos = getLocalHorizontalPosition() - pos_prev;

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	_ev_pos_b_est.setBias(_ev_pos_b_est.getBias() - delta_horz_pos);
#endif // CONFIG_EKF2_EXTERNAL_VISION

	updateHorizontalPositionResetStatus(delta_horz_pos);

	if (PX4_ISFINITE(hpos_var)) {
		P.uncorrelateCovarianceSetVariance<2>(State::pos.idx, math::max(sq(0.01f), hpos_var));
	}

	// Reset the timout timer
	_time_last_hor_pos_fuse = _time_delayed_us;

	return true;
}

bool Ekf::initialiseAltitudeTo(const float altitude, const float vpos_var)
{
	if (!checkAltitudeValidity(altitude)) {
		return false;
	}

	if (!PX4_ISFINITE(_local_origin_alt)) {
		const float local_alt_prev = _gpos.altitude();

		if (isLocalVerticalPositionValid()) {
			_local_origin_alt = altitude - local_alt_prev;

		} else {
			_local_origin_alt = altitude;
		}

		ECL_INFO("Origin alt=%.3f", (double)_local_origin_alt);
	}

	resetAltitudeTo(altitude, vpos_var);

	return true;
}

void Ekf::get_ekf_gpos_accuracy(float *ekf_eph, float *ekf_epv) const
{
	if (global_origin_valid()) {
		get_ekf_lpos_accuracy(ekf_eph, ekf_epv);

	} else {
		*ekf_eph = INFINITY;
		*ekf_epv = INFINITY;
	}
}

void Ekf::get_ekf_lpos_accuracy(float *ekf_eph, float *ekf_epv) const
{
	// TODO - allow for baro drift in vertical position error
	float hpos_err = sqrtf(P.trace<2>(State::pos.idx));

	// If we are dead-reckoning for too long, use the innovations as a conservative alternate measure of the horizontal position error
	// The reason is that complete rejection of measurements is often caused by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	if (_horizontal_deadreckon_time_exceeded) {
#if defined(CONFIG_EKF2_GNSS)

		if (_control_status.flags.gnss_pos) {
			hpos_err = math::max(hpos_err, Vector2f(_aid_src_gnss_pos.innovation).norm());
		}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

		if (_control_status.flags.ev_pos) {
			hpos_err = math::max(hpos_err, Vector2f(_aid_src_ev_pos.innovation).norm());
		}

#endif // CONFIG_EKF2_EXTERNAL_VISION
	}

	*ekf_eph = hpos_err;
	*ekf_epv = sqrtf(P(State::pos.idx + 2, State::pos.idx + 2));
}

void Ekf::get_ekf_vel_accuracy(float *ekf_evh, float *ekf_evv) const
{
	float hvel_err = sqrtf(P.trace<2>(State::vel.idx));

	// If we are dead-reckoning for too long, use the innovations as a conservative alternate measure of the horizontal velocity error
	// The reason is that complete rejection of measurements is often caused by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	if (_horizontal_deadreckon_time_exceeded) {
		float vel_err_conservative = 0.0f;

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

		if (_control_status.flags.opt_flow) {
			float gndclearance = math::max(_params.rng_gnd_clearance, 0.1f);
			vel_err_conservative = math::max(getHagl(), gndclearance) * Vector2f(_aid_src_optical_flow.innovation).norm();
		}

#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_GNSS)

		if (_control_status.flags.gnss_pos) {
			vel_err_conservative = math::max(vel_err_conservative, Vector2f(_aid_src_gnss_pos.innovation).norm());
		}

		if (_control_status.flags.gnss_vel) {
			vel_err_conservative = math::max(vel_err_conservative, Vector2f(_aid_src_gnss_vel.innovation).norm());
		}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

		if (_control_status.flags.ev_pos) {
			vel_err_conservative = math::max(vel_err_conservative, Vector2f(_aid_src_ev_pos.innovation).norm());
		}

		if (_control_status.flags.ev_vel) {
			vel_err_conservative = math::max(vel_err_conservative, Vector2f(_aid_src_ev_vel.innovation).norm());
		}

#endif // CONFIG_EKF2_EXTERNAL_VISION

		hvel_err = math::max(hvel_err, vel_err_conservative);
	}

	*ekf_evh = hvel_err;
	*ekf_evv = sqrtf(P(State::vel.idx + 2, State::vel.idx + 2));
}

void Ekf::get_ekf_ctrl_limits(float *vxy_max, float *vz_max, float *hagl_min, float *hagl_max_z,
			      float *hagl_max_xy) const
{
	// Do not require limiting by default
	*vxy_max = NAN;
	*vz_max = NAN;
	*hagl_min = NAN;
	*hagl_max_z = NAN;
	*hagl_max_xy = NAN;

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// Calculate range finder limits
	const float rangefinder_hagl_min = _range_sensor.getValidMinVal();

	// Allow use of 90% of rangefinder maximum range to allow for angular motion
	const float rangefinder_hagl_max = 0.9f * _range_sensor.getValidMaxVal();

	// TODO : calculate visual odometry limits
	const bool relying_on_rangefinder = isOnlyActiveSourceOfVerticalPositionAiding(_control_status.flags.rng_hgt);

	if (relying_on_rangefinder) {
		*hagl_min = rangefinder_hagl_min;
		*hagl_max_z = rangefinder_hagl_max;
	}

# if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// Keep within flow AND range sensor limits when exclusively using optical flow
	const bool relying_on_optical_flow = isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow);

	if (relying_on_optical_flow) {
		// Calculate optical flow limits
		float flow_hagl_min = _flow_min_distance;
		float flow_hagl_max = _flow_max_distance;

		// only limit optical flow height is dependent on range finder or terrain estimate invalid (precaution)
		if ((!_control_status.flags.opt_flow_terrain && _control_status.flags.rng_terrain)
		    || !isTerrainEstimateValid()
		   ) {
			flow_hagl_min = math::max(flow_hagl_min, rangefinder_hagl_min);
			flow_hagl_max = math::min(flow_hagl_max, rangefinder_hagl_max);
		}

		const float flow_constrained_height = math::constrain(getHagl(), flow_hagl_min, flow_hagl_max);

		// Allow ground relative velocity to use 50% of available flow sensor range to allow for angular motion
		float flow_vxy_max = 0.5f * _flow_max_rate * flow_constrained_height;
		flow_hagl_max = math::max(flow_hagl_max * 0.9f, flow_hagl_max - 1.0f);

		*vxy_max = flow_vxy_max;
		*hagl_min = flow_hagl_min;
		*hagl_max_xy = flow_hagl_max;
	}

# endif // CONFIG_EKF2_OPTICAL_FLOW

#endif // CONFIG_EKF2_RANGE_FINDER
}

void Ekf::resetGyroBias()
{
	// Zero the gyro bias states
	_state.gyro_bias.zero();

	resetGyroBiasCov();
}

void Ekf::resetAccelBias()
{
	// Zero the accel bias states
	_state.accel_bias.zero();

	resetAccelBiasCov();
}

float Ekf::getHeadingInnovationTestRatio() const
{
	// return the largest heading innovation test ratio
	float test_ratio = -1.f;

#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
		for (auto &test_ratio_filtered : _aid_src_mag.test_ratio_filtered) {
			test_ratio = math::max(test_ratio, fabsf(test_ratio_filtered));
		}
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS_YAW)

	if (_control_status.flags.gnss_yaw) {
		test_ratio = math::max(test_ratio, fabsf(_aid_src_gnss_yaw.test_ratio_filtered));
	}

#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_yaw) {
		test_ratio = math::max(test_ratio, fabsf(_aid_src_ev_yaw.test_ratio_filtered));
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	if (PX4_ISFINITE(test_ratio) && (test_ratio >= 0.f)) {
		return sqrtf(test_ratio);
	}

	return NAN;
}

float Ekf::getHorizontalVelocityInnovationTestRatio() const
{
	// return the largest velocity innovation test ratio
	float test_ratio = -1.f;

#if defined(CONFIG_EKF2_GNSS)

	if (_control_status.flags.gnss_vel) {
		for (int i = 0; i < 2; i++) { // only xy
			test_ratio = math::max(test_ratio, fabsf(_aid_src_gnss_vel.test_ratio_filtered[i]));
		}
	}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_vel) {
		for (int i = 0; i < 2; i++) { // only xy
			test_ratio = math::max(test_ratio, fabsf(_aid_src_ev_vel.test_ratio_filtered[i]));
		}
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

	if (isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow)) {
		for (auto &test_ratio_filtered : _aid_src_optical_flow.test_ratio_filtered) {
			test_ratio = math::max(test_ratio, fabsf(test_ratio_filtered));
		}
	}

#endif // CONFIG_EKF2_OPTICAL_FLOW

	if (PX4_ISFINITE(test_ratio) && (test_ratio >= 0.f)) {
		return sqrtf(test_ratio);
	}

	return NAN;
}

float Ekf::getVerticalVelocityInnovationTestRatio() const
{
	// return the largest velocity innovation test ratio
	float test_ratio = -1.f;

#if defined(CONFIG_EKF2_GNSS)

	if (_control_status.flags.gnss_vel) {
		test_ratio = math::max(test_ratio, fabsf(_aid_src_gnss_vel.test_ratio_filtered[2]));
	}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_vel) {
		test_ratio = math::max(test_ratio, fabsf(_aid_src_ev_vel.test_ratio_filtered[2]));
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	if (PX4_ISFINITE(test_ratio) && (test_ratio >= 0.f)) {
		return sqrtf(test_ratio);
	}

	return NAN;
}

float Ekf::getHorizontalPositionInnovationTestRatio() const
{
	// return the largest position innovation test ratio
	float test_ratio = -1.f;

#if defined(CONFIG_EKF2_GNSS)

	if (_control_status.flags.gnss_pos) {
		for (auto &test_ratio_filtered : _aid_src_gnss_pos.test_ratio_filtered) {
			test_ratio = math::max(test_ratio, fabsf(test_ratio_filtered));
		}
	}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_pos) {
		for (auto &test_ratio_filtered : _aid_src_ev_pos.test_ratio_filtered) {
			test_ratio = math::max(test_ratio, fabsf(test_ratio_filtered));
		}
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)

	if (_control_status.flags.aux_gpos) {
		test_ratio = math::max(test_ratio, fabsf(_aux_global_position.test_ratio_filtered()));
	}

#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION

	if (PX4_ISFINITE(test_ratio) && (test_ratio >= 0.f)) {
		return sqrtf(test_ratio);
	}

	return NAN;
}

float Ekf::getVerticalPositionInnovationTestRatio() const
{
	// return the combined vertical position innovation test ratio
	float hgt_sum = 0.f;
	int n_hgt_sources = 0;

#if defined(CONFIG_EKF2_BAROMETER)

	if (_control_status.flags.baro_hgt) {
		hgt_sum += sqrtf(fabsf(_aid_src_baro_hgt.test_ratio_filtered));
		n_hgt_sources++;
	}

#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_GNSS)

	if (_control_status.flags.gps_hgt) {
		hgt_sum += sqrtf(fabsf(_aid_src_gnss_hgt.test_ratio_filtered));
		n_hgt_sources++;
	}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_RANGE_FINDER)

	if (_control_status.flags.rng_hgt) {
		hgt_sum += sqrtf(fabsf(_aid_src_rng_hgt.test_ratio_filtered));
		n_hgt_sources++;
	}

#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_hgt) {
		hgt_sum += sqrtf(fabsf(_aid_src_ev_hgt.test_ratio_filtered));
		n_hgt_sources++;
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	if (n_hgt_sources > 0) {
		return math::max(hgt_sum / static_cast<float>(n_hgt_sources), FLT_MIN);
	}

	return NAN;
}

float Ekf::getAirspeedInnovationTestRatio() const
{
#if defined(CONFIG_EKF2_AIRSPEED)

	if (_control_status.flags.fuse_aspd) {
		// return the airspeed fusion innovation test ratio
		return sqrtf(fabsf(_aid_src_airspeed.test_ratio_filtered));
	}

#endif // CONFIG_EKF2_AIRSPEED

	return NAN;
}

float Ekf::getSyntheticSideslipInnovationTestRatio() const
{
#if defined(CONFIG_EKF2_SIDESLIP)

	if (_control_status.flags.fuse_beta) {
		// return the synthetic sideslip innovation test ratio
		return sqrtf(fabsf(_aid_src_sideslip.test_ratio_filtered));
	}

#endif // CONFIG_EKF2_SIDESLIP

	return NAN;
}

float Ekf::getHeightAboveGroundInnovationTestRatio() const
{
	// return the combined HAGL innovation test ratio
	float hagl_sum = 0.f;
	int n_hagl_sources = 0;

#if defined(CONFIG_EKF2_TERRAIN)

# if defined(CONFIG_EKF2_OPTICAL_FLOW)

	if (_control_status.flags.opt_flow_terrain) {
		hagl_sum += sqrtf(math::max(fabsf(_aid_src_optical_flow.test_ratio_filtered[0]),
					    _aid_src_optical_flow.test_ratio_filtered[1]));
		n_hagl_sources++;
	}

# endif // CONFIG_EKF2_OPTICAL_FLOW

# if defined(CONFIG_EKF2_RANGE_FINDER)

	if (_control_status.flags.rng_terrain) {
		hagl_sum += sqrtf(fabsf(_aid_src_rng_hgt.test_ratio_filtered));
		n_hagl_sources++;
	}

# endif // CONFIG_EKF2_RANGE_FINDER

#endif // CONFIG_EKF2_TERRAIN

	if (n_hagl_sources > 0) {
		return math::max(hagl_sum / static_cast<float>(n_hagl_sources), FLT_MIN);
	}

	return NAN;
}

uint16_t Ekf::get_ekf_soln_status() const
{
	// LEGACY Mavlink bitmask containing state of estimator solution (see Mavlink ESTIMATOR_STATUS_FLAGS)
	union ekf_solution_status_u {
		struct {
			uint16_t attitude           : 1;
			uint16_t velocity_horiz     : 1;
			uint16_t velocity_vert      : 1;
			uint16_t pos_horiz_rel      : 1;
			uint16_t pos_horiz_abs      : 1;
			uint16_t pos_vert_abs       : 1;
			uint16_t pos_vert_agl       : 1;
			uint16_t const_pos_mode     : 1;
			uint16_t pred_pos_horiz_rel : 1;
			uint16_t pred_pos_horiz_abs : 1;
			uint16_t gps_glitch         : 1;
			uint16_t accel_error        : 1;
		} flags;
		uint16_t value;
	} soln_status{};

	// 1	ESTIMATOR_ATTITUDE	True if the attitude estimate is good
	soln_status.flags.attitude = attitude_valid();

	// 2	ESTIMATOR_VELOCITY_HORIZ	True if the horizontal velocity estimate is good
	soln_status.flags.velocity_horiz = isLocalHorizontalPositionValid();

	// 4	ESTIMATOR_VELOCITY_VERT	True if the vertical velocity estimate is good
	soln_status.flags.velocity_vert = isLocalVerticalVelocityValid() || isLocalVerticalPositionValid();

	// 8	ESTIMATOR_POS_HORIZ_REL	True if the horizontal position (relative) estimate is good
	soln_status.flags.pos_horiz_rel = isLocalHorizontalPositionValid();

	// 16	ESTIMATOR_POS_HORIZ_ABS	True if the horizontal position (absolute) estimate is good
	soln_status.flags.pos_horiz_abs = isGlobalHorizontalPositionValid();

	// 32	ESTIMATOR_POS_VERT_ABS	True if the vertical position (absolute) estimate is good
	soln_status.flags.pos_vert_abs = isVerticalAidingActive();

	// 64	ESTIMATOR_POS_VERT_AGL	True if the vertical position (above ground) estimate is good
#if defined(CONFIG_EKF2_TERRAIN)
	soln_status.flags.pos_vert_agl = isTerrainEstimateValid();
#endif // CONFIG_EKF2_TERRAIN

	// 128	ESTIMATOR_CONST_POS_MODE	True if the EKF is in a constant position mode and is not using external measurements (eg GNSS or optical flow)
	soln_status.flags.const_pos_mode = _control_status.flags.fake_pos || _control_status.flags.valid_fake_pos
					   || _control_status.flags.vehicle_at_rest;

	// 256	ESTIMATOR_PRED_POS_HORIZ_REL	True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate
	soln_status.flags.pred_pos_horiz_rel = isHorizontalAidingActive();

	// 512	ESTIMATOR_PRED_POS_HORIZ_ABS	True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate
	soln_status.flags.pred_pos_horiz_abs = _control_status.flags.gnss_pos || _control_status.flags.aux_gpos;

	// 1024	ESTIMATOR_GPS_GLITCH	True if the EKF has detected a GNSS glitch
#if defined(CONFIG_EKF2_GNSS)
	const bool gnss_vel_innov_bad = Vector3f(_aid_src_gnss_vel.test_ratio).max() > 1.f;
	const bool gnss_pos_innov_bad = Vector2f(_aid_src_gnss_pos.test_ratio).max() > 1.f;
	soln_status.flags.gps_glitch = (gnss_vel_innov_bad || gnss_pos_innov_bad);
#endif // CONFIG_EKF2_GNSS

	// 2048	ESTIMATOR_ACCEL_ERROR	True if the EKF has detected bad accelerometer data
	soln_status.flags.accel_error = _fault_status.flags.bad_acc_vertical || _fault_status.flags.bad_acc_clipping;

	return soln_status.value;
}

void Ekf::fuse(const VectorState &K, float innovation)
{
	// quat_nominal
	Quatf delta_quat(matrix::AxisAnglef(K.slice<State::quat_nominal.dof, 1>(State::quat_nominal.idx,
					    0) * (-1.f * innovation)));
	_state.quat_nominal = delta_quat * _state.quat_nominal;
	_state.quat_nominal.normalize();
	_R_to_earth = Dcmf(_state.quat_nominal);

	// vel
	_state.vel = matrix::constrain(_state.vel - K.slice<State::vel.dof, 1>(State::vel.idx, 0) * innovation, -1.e3f, 1.e3f);

	// pos
	const Vector3f pos_correction = K.slice<State::pos.dof, 1>(State::pos.idx, 0) * (-innovation);

	// Accumulate position in global coordinates
	_gpos += pos_correction;
	_state.pos.zero();
	// Also store altitude in the state vector as this is used for optical flow fusion
	_state.pos(2) = -_gpos.altitude();

	// gyro_bias
	_state.gyro_bias = matrix::constrain(_state.gyro_bias - K.slice<State::gyro_bias.dof, 1>(State::gyro_bias.idx,
					     0) * innovation,
					     -getGyroBiasLimit(), getGyroBiasLimit());

	// accel_bias
	_state.accel_bias = matrix::constrain(_state.accel_bias - K.slice<State::accel_bias.dof, 1>(State::accel_bias.idx,
					      0) * innovation,
					      -getAccelBiasLimit(), getAccelBiasLimit());

#if defined(CONFIG_EKF2_MAGNETOMETER)

	// mag_I, mag_B
	if (_control_status.flags.mag) {
		_state.mag_I = matrix::constrain(_state.mag_I - K.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0) * innovation, -1.f,
						 1.f);
		_state.mag_B = matrix::constrain(_state.mag_B - K.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0) * innovation,
						 -getMagBiasLimit(), getMagBiasLimit());
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)

	// wind_vel
	if (_control_status.flags.wind) {
		_state.wind_vel = matrix::constrain(_state.wind_vel - K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx,
						    0) * innovation, -1.e2f, 1.e2f);
	}

#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_TERRAIN)
	_state.terrain = math::constrain(_state.terrain - K(State::terrain.idx) * innovation, -1e4f, 1e4f);
#endif // CONFIG_EKF2_TERRAIN
}

void Ekf::updateDeadReckoningStatus()
{
	updateHorizontalDeadReckoningstatus();
	updateVerticalDeadReckoningStatus();
}

void Ekf::updateHorizontalDeadReckoningstatus()
{
	bool inertial_dead_reckoning = true;
	bool aiding_expected_in_air = false;

	// velocity aiding active
	if ((_control_status.flags.gnss_vel || _control_status.flags.ev_vel)
	    && isRecent(_time_last_hor_vel_fuse, _params.no_aid_timeout_max)
	   ) {
		inertial_dead_reckoning = false;
	}

	// position aiding active
	if ((_control_status.flags.gnss_pos || _control_status.flags.ev_pos || _control_status.flags.aux_gpos)
	    && isRecent(_time_last_hor_pos_fuse, _params.no_aid_timeout_max)
	   ) {
		inertial_dead_reckoning = false;
	}

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

	// optical flow active
	if (_control_status.flags.opt_flow
	    && isRecent(_aid_src_optical_flow.time_last_fuse, _params.no_aid_timeout_max)
	   ) {
		inertial_dead_reckoning = false;

	} else {
		if (!_control_status.flags.in_air && (_params.flow_ctrl == 1)
		    && isRecent(_aid_src_optical_flow.timestamp_sample, _params.no_aid_timeout_max)
		   ) {
			// currently landed, but optical flow aiding should be possible once in air
			aiding_expected_in_air = true;
		}
	}

#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_AIRSPEED)

	// air data aiding active
	if ((_control_status.flags.fuse_aspd && isRecent(_aid_src_airspeed.time_last_fuse, _params.no_aid_timeout_max))
	    && (_control_status.flags.fuse_beta && isRecent(_aid_src_sideslip.time_last_fuse, _params.no_aid_timeout_max))
	   ) {
		// wind_dead_reckoning: no other aiding but air data
		_control_status.flags.wind_dead_reckoning = inertial_dead_reckoning;

		// air data aiding is active, we're not inertial dead reckoning
		inertial_dead_reckoning = false;

	} else {
		_control_status.flags.wind_dead_reckoning = false;

		if (!_control_status.flags.in_air && _control_status.flags.fixed_wing
		    && (_params.beta_fusion_enabled == 1)
		    && (_params.arsp_thr > 0.f) && isRecent(_aid_src_airspeed.timestamp_sample, _params.no_aid_timeout_max)
		   ) {
			// currently landed, but air data aiding should be possible once in air
			aiding_expected_in_air = true;
		}
	}

#endif // CONFIG_EKF2_AIRSPEED

	// zero velocity update
	if (isRecent(_zero_velocity_update.time_last_fuse(), _params.no_aid_timeout_max)) {
		// only respect as a valid aiding source now if we expect to have another valid source once in air
		if (aiding_expected_in_air) {
			inertial_dead_reckoning = false;
		}
	}

	if (_control_status.flags.valid_fake_pos && isRecent(_aid_src_fake_pos.time_last_fuse, _params.no_aid_timeout_max)) {
		// only respect as a valid aiding source now if we expect to have another valid source once in air
		if (aiding_expected_in_air) {
			inertial_dead_reckoning = false;
		}
	}

	if (inertial_dead_reckoning) {
		if (isTimedOut(_time_last_horizontal_aiding, (uint64_t)_params.valid_timeout_max)) {
			// deadreckon time exceeded
			if (!_horizontal_deadreckon_time_exceeded) {
				ECL_WARN("horizontal dead reckon time exceeded");
				_horizontal_deadreckon_time_exceeded = true;
			}
		}

	} else {
		if (_time_delayed_us > _params.no_aid_timeout_max) {
			_time_last_horizontal_aiding = _time_delayed_us - _params.no_aid_timeout_max;
		}

		_horizontal_deadreckon_time_exceeded = false;

	}

	_control_status.flags.inertial_dead_reckoning = inertial_dead_reckoning;
}

void Ekf::updateVerticalDeadReckoningStatus()
{
	if (isVerticalPositionAidingActive()) {
		_time_last_v_pos_aiding = _time_last_hgt_fuse;
		_vertical_position_deadreckon_time_exceeded = false;

	} else if (isTimedOut(_time_last_v_pos_aiding, (uint64_t)_params.valid_timeout_max)) {
		_vertical_position_deadreckon_time_exceeded = true;
	}

	if (isVerticalVelocityAidingActive()) {
		_time_last_v_vel_aiding = _time_last_ver_vel_fuse;
		_vertical_velocity_deadreckon_time_exceeded = false;

	} else if (isTimedOut(_time_last_v_vel_aiding, (uint64_t)_params.valid_timeout_max)
		   && _vertical_position_deadreckon_time_exceeded) {

		_vertical_velocity_deadreckon_time_exceeded = true;
	}
}

Vector3f Ekf::getRotVarBody() const
{
	const matrix::SquareMatrix3f rot_cov_body = getStateCovariance<State::quat_nominal>();
	return matrix::SquareMatrix3f(_R_to_earth.T() * rot_cov_body * _R_to_earth).diag();
}

Vector3f Ekf::getRotVarNed() const
{
	const matrix::SquareMatrix3f rot_cov_ned = getStateCovariance<State::quat_nominal>();
	return rot_cov_ned.diag();
}

float Ekf::getYawVar() const
{
	return getRotVarNed()(2);
}

float Ekf::getTiltVariance() const
{
	const Vector3f rot_var_ned = getRotVarNed();
	return rot_var_ned(0) + rot_var_ned(1);
}

#if defined(CONFIG_EKF2_BAROMETER)
void Ekf::updateGroundEffect()
{
	if (_control_status.flags.in_air && !_control_status.flags.fixed_wing) {
#if defined(CONFIG_EKF2_TERRAIN)

		if (isTerrainEstimateValid()) {
			// automatically set ground effect if terrain is valid
			float height = getHagl();
			_control_status.flags.gnd_effect = (height < _params.gnd_effect_max_hgt);

		} else
#endif // CONFIG_EKF2_TERRAIN
			if (_control_status.flags.gnd_effect) {
				// Turn off ground effect compensation if it times out
				if (isTimedOut(_time_last_gnd_effect_on, GNDEFFECT_TIMEOUT)) {
					_control_status.flags.gnd_effect = false;
				}
			}

	} else {
		_control_status.flags.gnd_effect = false;
	}
}
#endif // CONFIG_EKF2_BAROMETER


void Ekf::updateIMUBiasInhibit(const imuSample &imu_delayed)
{
	// inhibit learning of imu accel bias if the manoeuvre levels are too high to protect against the effect of sensor nonlinearities or bad accel data is detected
	// xy accel bias learning is also disabled on ground as those states are poorly observable when perpendicular to the gravity vector
	{
		const Vector3f gyro_corrected = imu_delayed.delta_ang / imu_delayed.delta_ang_dt - _state.gyro_bias;

		const float alpha = math::constrain((imu_delayed.delta_ang_dt / _params.acc_bias_learn_tc), 0.f, 1.f);
		const float beta = 1.f - alpha;

		_ang_rate_magnitude_filt = fmaxf(gyro_corrected.norm(), beta * _ang_rate_magnitude_filt);
	}

	{
		const Vector3f accel_corrected = imu_delayed.delta_vel / imu_delayed.delta_vel_dt - _state.accel_bias;

		const float alpha = math::constrain((imu_delayed.delta_vel_dt / _params.acc_bias_learn_tc), 0.f, 1.f);
		const float beta = 1.f - alpha;

		_accel_magnitude_filt = fmaxf(accel_corrected.norm(), beta * _accel_magnitude_filt);
	}


	const bool is_manoeuvre_level_high = (_ang_rate_magnitude_filt > _params.acc_bias_learn_gyr_lim)
					     || (_accel_magnitude_filt > _params.acc_bias_learn_acc_lim);


	// gyro bias inhibit
	const bool do_inhibit_all_gyro_axes = !(_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::GyroBias));

	for (unsigned index = 0; index < State::gyro_bias.dof; index++) {
		bool is_bias_observable = true; // TODO: gyro bias conditions
		_gyro_bias_inhibit[index] = do_inhibit_all_gyro_axes || !is_bias_observable;
	}

	// accel bias inhibit
	const bool do_inhibit_all_accel_axes = !(_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::AccelBias))
					       || is_manoeuvre_level_high
					       || _fault_status.flags.bad_acc_vertical;

	for (unsigned index = 0; index < State::accel_bias.dof; index++) {
		bool is_bias_observable = true;

		if (_control_status.flags.vehicle_at_rest) {
			is_bias_observable = true;

		} else if (_control_status.flags.fake_hgt) {
			is_bias_observable = false;

		} else if (_control_status.flags.fake_pos) {
			// when using fake position (but not fake height) only consider an accel bias observable if aligned with the gravity vector
			is_bias_observable = (fabsf(_R_to_earth(2, index)) > 0.966f); // cos 15 degrees ~= 0.966
		}

		_accel_bias_inhibit[index] = do_inhibit_all_accel_axes || imu_delayed.delta_vel_clipping[index] || !is_bias_observable;
	}
}

void Ekf::fuseDirectStateMeasurement(const float innov, const float innov_var, const float R, const int state_index)
{
	VectorState K;  // Kalman gain vector for any single observation - sequential fusion is used.

	// calculate kalman gain K = PHS, where S = 1/innovation variance
	for (int row = 0; row < State::size; row++) {
		K(row) = P(row, state_index) / innov_var;
	}

	clearInhibitedStateKalmanGains(K);

#if false
	// Matrix implementation of the Joseph stabilized covariance update
	// This is extremely expensive to compute. Use for debugging purposes only.
	auto A = matrix::eye<float, State::size>();
	VectorState H;
	H(state_index) = 1.f;
	A -= K.multiplyByTranspose(H);
	P = A * P;
	P = P.multiplyByTranspose(A);

	const VectorState KR = K * R;
	P += KR.multiplyByTranspose(K);
#else
	// Efficient implementation of the Joseph stabilized covariance update
	// Based on "G. J. Bierman. Factorization Methods for Discrete Sequential Estimation. Academic Press, Dover Publications, New York, 1977, 2006"
	// P = (I - K * H) * P * (I - K * H).T   + K * R * K.T
	//   =      P_temp     * (I - H.T * K.T) + K * R * K.T
	//   =      P_temp - P_temp * H.T * K.T  + K * R * K.T

	// Step 1: conventional update
	// Compute P_temp and store it in P to avoid allocating more memory
	// P is symmetric, so PH == H.T * P.T == H.T * P. Taking the row is faster as matrices are row-major
	VectorState PH = P.row(state_index);

	for (unsigned i = 0; i < State::size; i++) {
		for (unsigned j = 0; j < State::size; j++) {
			P(i, j) -= K(i) * PH(j); // P is now not symmetric if K is not optimal (e.g.: some gains have been zeroed)
		}
	}

	// Step 2: stabilized update
	// P (or "P_temp") is not symmetric so we must take the column
	PH = P.col(state_index);

	for (unsigned i = 0; i < State::size; i++) {
		for (unsigned j = 0; j <= i; j++) {
			P(i, j) = P(i, j) - PH(i) * K(j) + K(i) * R * K(j);
			P(j, i) = P(i, j);
		}
	}

#endif

	constrainStateVariances();

	// apply the state corrections
	fuse(K, innov);
}

bool Ekf::measurementUpdate(VectorState &K, const VectorState &H, const float R, const float innovation)
{
	clearInhibitedStateKalmanGains(K);

#if false
	// Matrix implementation of the Joseph stabilized covariance update
	// This is extremely expensive to compute. Use for debugging purposes only.
	auto A = matrix::eye<float, State::size>();
	A -= K.multiplyByTranspose(H);
	P = A * P;
	P = P.multiplyByTranspose(A);

	const VectorState KR = K * R;
	P += KR.multiplyByTranspose(K);
#else
	// Efficient implementation of the Joseph stabilized covariance update
	// Based on "G. J. Bierman. Factorization Methods for Discrete Sequential Estimation. Academic Press, Dover Publications, New York, 1977, 2006"
	// P = (I - K * H) * P * (I - K * H).T   + K * R * K.T
	//   =      P_temp     * (I - H.T * K.T) + K * R * K.T
	//   =      P_temp - P_temp * H.T * K.T  + K * R * K.T

	// Step 1: conventional update
	// Compute P_temp and store it in P to avoid allocating more memory
	// P is symmetric, so PH == H.T * P.T == H.T * P. Taking the row is faster as matrices are row-major
	VectorState PH = P * H; // H is stored as a column vector. H is in fact H.T

	for (unsigned i = 0; i < State::size; i++) {
		for (unsigned j = 0; j < State::size; j++) {
			P(i, j) -= K(i) * PH(j); // P is now not symmetrical if K is not optimal (e.g.: some gains have been zeroed)
		}
	}

	// Step 2: stabilized update
	PH = P * H; // H is stored as a column vector. H is in fact H.T

	for (unsigned i = 0; i < State::size; i++) {
		for (unsigned j = 0; j <= i; j++) {
			P(i, j) = P(i, j) - PH(i) * K(j) + K(i) * R * K(j);
			P(j, i) = P(i, j);
		}
	}

#endif

	constrainStateVariances();

	// apply the state corrections
	fuse(K, innovation);
	return true;
}

void Ekf::resetAidSourceStatusZeroInnovation(estimator_aid_source1d_s &status) const
{
	status.time_last_fuse = _time_delayed_us;

	status.innovation = 0.f;
	status.innovation_filtered = 0.f;
	status.innovation_variance = status.observation_variance;

	status.test_ratio = 0.f;
	status.test_ratio_filtered = 0.f;

	status.innovation_rejected = false;
	status.fused = true;
}

void Ekf::updateAidSourceStatus(estimator_aid_source1d_s &status, const uint64_t &timestamp_sample,
				const float &observation, const float &observation_variance,
				const float &innovation, const float &innovation_variance,
				float innovation_gate) const
{
	bool innovation_rejected = false;

	const float test_ratio = sq(innovation) / (sq(innovation_gate) * innovation_variance);

	if ((status.timestamp_sample > 0) && (timestamp_sample > status.timestamp_sample)) {

		const float dt_s = math::constrain((timestamp_sample - status.timestamp_sample) * 1e-6f, 0.001f, 1.f);

		static constexpr float tau = 0.5f;
		const float alpha = math::constrain(dt_s / (dt_s + tau), 0.f, 1.f);

		// test_ratio_filtered
		if (PX4_ISFINITE(status.test_ratio_filtered)) {
			status.test_ratio_filtered += alpha * (matrix::sign(innovation) * test_ratio - status.test_ratio_filtered);

		} else {
			// otherwise, init the filtered test ratio
			status.test_ratio_filtered = test_ratio;
		}

		// innovation_filtered
		if (PX4_ISFINITE(status.innovation_filtered)) {
			status.innovation_filtered += alpha * (innovation - status.innovation_filtered);

		} else {
			// otherwise, init the filtered innovation
			status.innovation_filtered = innovation;
		}


		// limit extremes in filtered values
		static constexpr float kNormalizedInnovationLimit = 2.f;
		static constexpr float kTestRatioLimit = sq(kNormalizedInnovationLimit);

		if (test_ratio > kTestRatioLimit) {

			status.test_ratio_filtered = math::constrain(status.test_ratio_filtered, -kTestRatioLimit, kTestRatioLimit);

			const float innov_limit = kNormalizedInnovationLimit * innovation_gate * sqrtf(innovation_variance);
			status.innovation_filtered = math::constrain(status.innovation_filtered, -innov_limit, innov_limit);
		}

	} else {
		// invalid timestamp_sample, reset
		status.test_ratio_filtered = test_ratio;
		status.innovation_filtered = innovation;
	}

	status.test_ratio = test_ratio;

	status.observation = observation;
	status.observation_variance = observation_variance;

	status.innovation = innovation;
	status.innovation_variance = innovation_variance;

	if ((test_ratio > 1.f)
	    || !PX4_ISFINITE(test_ratio)
	    || !PX4_ISFINITE(status.innovation)
	    || !PX4_ISFINITE(status.innovation_variance)
	   ) {
		innovation_rejected = true;
	}

	status.timestamp_sample = timestamp_sample;

	// if any of the innovations are rejected, then the overall innovation is rejected
	status.innovation_rejected = innovation_rejected;

	// reset
	status.fused = false;
}

void Ekf::clearInhibitedStateKalmanGains(VectorState &K) const
{
	for (unsigned i = 0; i < State::gyro_bias.dof; i++) {
		if (_gyro_bias_inhibit[i]) {
			K(State::gyro_bias.idx + i) = 0.f;
		}
	}

	for (unsigned i = 0; i < State::accel_bias.dof; i++) {
		if (_accel_bias_inhibit[i]) {
			K(State::accel_bias.idx + i) = 0.f;
		}
	}

#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (!_control_status.flags.mag) {
		for (unsigned i = 0; i < State::mag_I.dof; i++) {
			K(State::mag_I.idx + i) = 0.f;
		}
	}

	if (!_control_status.flags.mag) {
		for (unsigned i = 0; i < State::mag_B.dof; i++) {
			K(State::mag_B.idx + i) = 0.f;
		}
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)

	if (!_control_status.flags.wind) {
		for (unsigned i = 0; i < State::wind_vel.dof; i++) {
			K(State::wind_vel.idx + i) = 0.f;
		}
	}

#endif // CONFIG_EKF2_WIND
}

float Ekf::getHeadingInnov() const
{
#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
		return Vector3f(_aid_src_mag.innovation).max();
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS_YAW)

	if (_control_status.flags.gnss_yaw) {
		return _aid_src_gnss_yaw.innovation;
	}

#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_yaw) {
		return _aid_src_ev_yaw.innovation;
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	return 0.f;
}

float Ekf::getHeadingInnovVar() const
{
#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
		return Vector3f(_aid_src_mag.innovation_variance).max();
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS_YAW)

	if (_control_status.flags.gnss_yaw) {
		return _aid_src_gnss_yaw.innovation_variance;
	}

#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_yaw) {
		return _aid_src_ev_yaw.innovation_variance;
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	return 0.f;
}

float Ekf::getHeadingInnovRatio() const
{
#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
		return Vector3f(_aid_src_mag.test_ratio).max();
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS_YAW)

	if (_control_status.flags.gnss_yaw) {
		return _aid_src_gnss_yaw.test_ratio;
	}

#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_yaw) {
		return _aid_src_ev_yaw.test_ratio;
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	return 0.f;
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file estimator_interface.cpp
 * Definition of base class for attitude estimators
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Siddharth B Purohit <siddharthbharatpurohit@gmail.com>
 */

#include "estimator_interface.h"

#include <mathlib/mathlib.h>

EstimatorInterface::~EstimatorInterface()
{
#if defined(CONFIG_EKF2_GNSS)
	delete _gps_buffer;
#endif // CONFIG_EKF2_GNSS
#if defined(CONFIG_EKF2_MAGNETOMETER)
	delete _mag_buffer;
#endif // CONFIG_EKF2_MAGNETOMETER
#if defined(CONFIG_EKF2_BAROMETER)
	delete _baro_buffer;
#endif // CONFIG_EKF2_BAROMETER
#if defined(CONFIG_EKF2_RANGE_FINDER)
	delete _range_buffer;
#endif // CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_AIRSPEED)
	delete _airspeed_buffer;
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	delete _flow_buffer;
#endif // CONFIG_EKF2_OPTICAL_FLOW
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	delete _ext_vision_buffer;
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_DRAG_FUSION)
	delete _drag_buffer;
#endif // CONFIG_EKF2_DRAG_FUSION
#if defined(CONFIG_EKF2_AUXVEL)
	delete _auxvel_buffer;
#endif // CONFIG_EKF2_AUXVEL
}

// Accumulate imu data and store to buffer at desired rate
void EstimatorInterface::setIMUData(const imuSample &imu_sample)
{
	// TODO: resolve misplaced responsibility
	if (!_initialised) {
		_initialised = init(imu_sample.time_us);
	}

	_time_latest_us = imu_sample.time_us;

	// the output observer always runs
	_output_predictor.calculateOutputStates(imu_sample.time_us, imu_sample.delta_ang, imu_sample.delta_ang_dt,
						imu_sample.delta_vel, imu_sample.delta_vel_dt);

	// accumulate and down-sample imu data and push to the buffer when new downsampled data becomes available
	if (_imu_down_sampler.update(imu_sample)) {

		_imu_updated = true;

		imuSample imu_downsampled = _imu_down_sampler.getDownSampledImuAndTriggerReset();

		// as a precaution constrain the integration delta time to prevent numerical problems
		const float filter_update_period_s = _params.filter_update_interval_us * 1e-6f;
		const float imu_min_dt = 0.5f * filter_update_period_s;
		const float imu_max_dt = 2.0f * filter_update_period_s;

		imu_downsampled.delta_ang_dt = math::constrain(imu_downsampled.delta_ang_dt, imu_min_dt, imu_max_dt);
		imu_downsampled.delta_vel_dt = math::constrain(imu_downsampled.delta_vel_dt, imu_min_dt, imu_max_dt);

		_imu_buffer.push(imu_downsampled);

		// get the oldest data from the buffer
		_time_delayed_us = _imu_buffer.get_oldest().time_us;

		// calculate the minimum interval between observations required to guarantee no loss of data
		// this will occur if data is overwritten before its time stamp falls behind the fusion time horizon
		_min_obs_interval_us = (imu_sample.time_us - _time_delayed_us) / (_obs_buffer_length - 1);
	}

#if defined(CONFIG_EKF2_DRAG_FUSION)
	setDragData(imu_sample);
#endif // CONFIG_EKF2_DRAG_FUSION
}

#if defined(CONFIG_EKF2_MAGNETOMETER)
void EstimatorInterface::setMagData(const magSample &mag_sample)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_mag_buffer == nullptr) {
		_mag_buffer = new RingBuffer<magSample>(_obs_buffer_length);

		if (_mag_buffer == nullptr || !_mag_buffer->valid()) {
			delete _mag_buffer;
			_mag_buffer = nullptr;
			printBufferAllocationFailed("mag");
			return;
		}
	}

	const int64_t time_us = mag_sample.time_us
				- static_cast<int64_t>(_params.mag_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_mag_buffer->get_newest().time_us + _min_obs_interval_us)) {

		magSample mag_sample_new{mag_sample};
		mag_sample_new.time_us = time_us;

		_mag_buffer->push(mag_sample_new);
		_time_last_mag_buffer_push = _time_latest_us;

	} else {
		ECL_WARN("mag data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _mag_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS)
void EstimatorInterface::setGpsData(const gnssSample &gnss_sample)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_gps_buffer == nullptr) {
		_gps_buffer = new RingBuffer<gnssSample>(_obs_buffer_length);

		if (_gps_buffer == nullptr || !_gps_buffer->valid()) {
			delete _gps_buffer;
			_gps_buffer = nullptr;
			printBufferAllocationFailed("GPS");
			return;
		}
	}

	const int64_t time_us = gnss_sample.time_us
				- static_cast<int64_t>(_params.gps_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	if (time_us >= static_cast<int64_t>(_gps_buffer->get_newest().time_us + _min_obs_interval_us)) {

		gnssSample gnss_sample_new(gnss_sample);

		gnss_sample_new.time_us = time_us;

		_gps_buffer->push(gnss_sample_new);
		_time_last_gps_buffer_push = _time_latest_us;

#if defined(CONFIG_EKF2_GNSS_YAW)

		if (PX4_ISFINITE(gnss_sample.yaw)) {
			_time_last_gnss_yaw_buffer_push = _time_latest_us;
		}

#endif // CONFIG_EKF2_GNSS_YAW

	} else {
		ECL_WARN("GPS data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _gps_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_BAROMETER)
void EstimatorInterface::setBaroData(const baroSample &baro_sample)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_baro_buffer == nullptr) {
		_baro_buffer = new RingBuffer<baroSample>(_obs_buffer_length);

		if (_baro_buffer == nullptr || !_baro_buffer->valid()) {
			delete _baro_buffer;
			_baro_buffer = nullptr;
			printBufferAllocationFailed("baro");
			return;
		}
	}

	const int64_t time_us = baro_sample.time_us
				- static_cast<int64_t>(_params.baro_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_baro_buffer->get_newest().time_us + _min_obs_interval_us)) {

		baroSample baro_sample_new{baro_sample};
		baro_sample_new.time_us = time_us;

		_baro_buffer->push(baro_sample_new);
		_time_last_baro_buffer_push = _time_latest_us;

	} else {
		ECL_WARN("baro data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _baro_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_AIRSPEED)
void EstimatorInterface::setAirspeedData(const airspeedSample &airspeed_sample)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_airspeed_buffer == nullptr) {
		_airspeed_buffer = new RingBuffer<airspeedSample>(_obs_buffer_length);

		if (_airspeed_buffer == nullptr || !_airspeed_buffer->valid()) {
			delete _airspeed_buffer;
			_airspeed_buffer = nullptr;
			printBufferAllocationFailed("airspeed");
			return;
		}
	}

	const int64_t time_us = airspeed_sample.time_us
				- static_cast<int64_t>(_params.airspeed_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_airspeed_buffer->get_newest().time_us + _min_obs_interval_us)) {

		airspeedSample airspeed_sample_new{airspeed_sample};
		airspeed_sample_new.time_us = time_us;

		_airspeed_buffer->push(airspeed_sample_new);

	} else {
		ECL_WARN("airspeed data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _airspeed_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_RANGE_FINDER)
void EstimatorInterface::setRangeData(const sensor::rangeSample &range_sample)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_range_buffer == nullptr) {
		_range_buffer = new RingBuffer<sensor::rangeSample>(_obs_buffer_length);

		if (_range_buffer == nullptr || !_range_buffer->valid()) {
			delete _range_buffer;
			_range_buffer = nullptr;
			printBufferAllocationFailed("range");
			return;
		}
	}

	const int64_t time_us = range_sample.time_us
				- static_cast<int64_t>(_params.range_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_range_buffer->get_newest().time_us + _min_obs_interval_us)) {

		sensor::rangeSample range_sample_new{range_sample};
		range_sample_new.time_us = time_us;

		_range_buffer->push(range_sample_new);
		_time_last_range_buffer_push = _time_latest_us;

	} else {
		ECL_WARN("range data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _range_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
void EstimatorInterface::setOpticalFlowData(const flowSample &flow)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_flow_buffer == nullptr) {
		_flow_buffer = new RingBuffer<flowSample>(_imu_buffer_length);

		if (_flow_buffer == nullptr || !_flow_buffer->valid()) {
			delete _flow_buffer;
			_flow_buffer = nullptr;
			printBufferAllocationFailed("flow");
			return;
		}
	}

	const int64_t time_us = flow.time_us
				- static_cast<int64_t>(_params.flow_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_flow_buffer->get_newest().time_us + _min_obs_interval_us)) {

		flowSample optflow_sample_new{flow};
		optflow_sample_new.time_us = time_us;

		_flow_buffer->push(optflow_sample_new);

	} else {
		ECL_WARN("optical flow data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _flow_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
void EstimatorInterface::setExtVisionData(const extVisionSample &evdata)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_ext_vision_buffer == nullptr) {
		_ext_vision_buffer = new RingBuffer<extVisionSample>(_obs_buffer_length);

		if (_ext_vision_buffer == nullptr || !_ext_vision_buffer->valid()) {
			delete _ext_vision_buffer;
			_ext_vision_buffer = nullptr;
			printBufferAllocationFailed("vision");
			return;
		}
	}

	// calculate the system time-stamp for the mid point of the integration period
	const int64_t time_us = evdata.time_us
				- static_cast<int64_t>(_params.ev_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_ext_vision_buffer->get_newest().time_us + _min_obs_interval_us)) {

		extVisionSample ev_sample_new{evdata};
		ev_sample_new.time_us = time_us;

		_ext_vision_buffer->push(ev_sample_new);
		_time_last_ext_vision_buffer_push = _time_latest_us;

	} else {
		ECL_WARN("EV data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _ext_vision_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AUXVEL)
void EstimatorInterface::setAuxVelData(const auxVelSample &auxvel_sample)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_auxvel_buffer == nullptr) {
		_auxvel_buffer = new RingBuffer<auxVelSample>(_obs_buffer_length);

		if (_auxvel_buffer == nullptr || !_auxvel_buffer->valid()) {
			delete _auxvel_buffer;
			_auxvel_buffer = nullptr;
			printBufferAllocationFailed("aux vel");
			return;
		}
	}

	const int64_t time_us = auxvel_sample.time_us
				- static_cast<int64_t>(_params.auxvel_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_auxvel_buffer->get_newest().time_us + _min_obs_interval_us)) {

		auxVelSample auxvel_sample_new{auxvel_sample};
		auxvel_sample_new.time_us = time_us;

		_auxvel_buffer->push(auxvel_sample_new);

	} else {
		ECL_WARN("aux velocity data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _auxvel_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_AUXVEL

void EstimatorInterface::setSystemFlagData(const systemFlagUpdate &system_flags)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_system_flag_buffer == nullptr) {
		_system_flag_buffer = new RingBuffer<systemFlagUpdate>(_obs_buffer_length);

		if (_system_flag_buffer == nullptr || !_system_flag_buffer->valid()) {
			delete _system_flag_buffer;
			_system_flag_buffer = nullptr;
			printBufferAllocationFailed("system flag");
			return;
		}
	}

	const int64_t time_us = system_flags.time_us
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_system_flag_buffer->get_newest().time_us + _min_obs_interval_us)) {

		systemFlagUpdate system_flags_new{system_flags};
		system_flags_new.time_us = time_us;

		_system_flag_buffer->push(system_flags_new);

	} else {
		ECL_DEBUG("system flag update too fast %" PRIi64 " < %" PRIu64 " + %d", time_us,
			  _system_flag_buffer->get_newest().time_us, _min_obs_interval_us);
	}
}

#if defined(CONFIG_EKF2_DRAG_FUSION)
void EstimatorInterface::setDragData(const imuSample &imu)
{
	// down-sample the drag specific force data by accumulating and calculating the mean when
	// sufficient samples have been collected
	if (_params.drag_ctrl > 0) {

		// Allocate the required buffer size if not previously done
		if (_drag_buffer == nullptr) {
			_drag_buffer = new RingBuffer<dragSample>(_obs_buffer_length);

			if (_drag_buffer == nullptr || !_drag_buffer->valid()) {
				delete _drag_buffer;
				_drag_buffer = nullptr;
				printBufferAllocationFailed("drag");
				return;
			}
		}

		// don't use any accel samples that are clipping
		if (imu.delta_vel_clipping[0] || imu.delta_vel_clipping[1] || imu.delta_vel_clipping[2]) {
			// reset accumulators
			_drag_sample_count = 0;
			_drag_down_sampled.accelXY.zero();
			_drag_down_sampled.time_us = 0;
			_drag_sample_time_dt = 0.0f;

			return;
		}

		_drag_sample_count++;
		// note acceleration is accumulated as a delta velocity
		_drag_down_sampled.accelXY(0) += imu.delta_vel(0);
		_drag_down_sampled.accelXY(1) += imu.delta_vel(1);
		_drag_down_sampled.time_us += imu.time_us;
		_drag_sample_time_dt += imu.delta_vel_dt;

		// calculate the downsample ratio for drag specific force data
		uint8_t min_sample_ratio = (uint8_t) ceilf((float)_imu_buffer_length / _obs_buffer_length);

		if (min_sample_ratio < 5) {
			min_sample_ratio = 5;
		}

		// calculate and store means from accumulated values
		if (_drag_sample_count >= min_sample_ratio) {
			// note conversion from accumulated delta velocity to acceleration
			_drag_down_sampled.accelXY(0) /= _drag_sample_time_dt;
			_drag_down_sampled.accelXY(1) /= _drag_sample_time_dt;
			_drag_down_sampled.time_us /= _drag_sample_count;

			// write to buffer
			_drag_buffer->push(_drag_down_sampled);

			// reset accumulators
			_drag_sample_count = 0;
			_drag_down_sampled.accelXY.zero();
			_drag_down_sampled.time_us = 0;
			_drag_sample_time_dt = 0.0f;
		}
	}
}
#endif // CONFIG_EKF2_DRAG_FUSION

bool EstimatorInterface::initialise_interface(uint64_t timestamp)
{
	const float filter_update_period_ms = _params.filter_update_interval_us / 1000.f;

	// calculate the IMU buffer length required to accomodate the maximum delay with some allowance for jitter
	_imu_buffer_length = math::max(2, (int)ceilf(_params.delay_max_ms / filter_update_period_ms));

	// set the observation buffer length to handle the minimum time of arrival between observations in combination
	// with the worst case delay from current time to ekf fusion time
	// allow for worst case 50% extension of the ekf fusion time horizon delay due to timing jitter
	const float ekf_delay_ms = _params.delay_max_ms * 1.5f;
	_obs_buffer_length = roundf(ekf_delay_ms / filter_update_period_ms);

	// limit to be no longer than the IMU buffer (we can't process data faster than the EKF prediction rate)
	_obs_buffer_length = math::min(_obs_buffer_length, _imu_buffer_length);

	ECL_DEBUG("EKF max time delay %.1f ms, OBS length %d\n", (double)ekf_delay_ms, _obs_buffer_length);

	if (!_imu_buffer.allocate(_imu_buffer_length) || !_output_predictor.allocate(_imu_buffer_length)) {

		printBufferAllocationFailed("IMU and output");
		return false;
	}

	_time_delayed_us = timestamp;
	_time_latest_us = timestamp;

	_fault_status.value = 0;

	return true;
}

Vector3f EstimatorInterface::getPosition() const
{
	LatLonAlt lla = _output_predictor.getLatLonAlt();
	float x;
	float y;

	if (_local_origin_lat_lon.isInitialized()) {
		_local_origin_lat_lon.project(lla.latitude_deg(), lla.longitude_deg(), x, y);

	} else {
		MapProjection zero_ref;
		zero_ref.initReference(0.0, 0.0);
		zero_ref.project(lla.latitude_deg(), lla.longitude_deg(), x, y);
	}

	const float z = -(lla.altitude() - getEkfGlobalOriginAltitude());

	return Vector3f(x, y, z);
}

bool EstimatorInterface::isOnlyActiveSourceOfHorizontalAiding(const bool aiding_flag) const
{
	return aiding_flag && !isOtherSourceOfHorizontalAidingThan(aiding_flag);
}

bool EstimatorInterface::isOtherSourceOfHorizontalAidingThan(const bool aiding_flag) const
{
	const int nb_sources = getNumberOfActiveHorizontalAidingSources();
	return aiding_flag ? nb_sources > 1 : nb_sources > 0;
}

int EstimatorInterface::getNumberOfActiveHorizontalAidingSources() const
{
	return getNumberOfActiveHorizontalPositionAidingSources() + getNumberOfActiveHorizontalVelocityAidingSources();
}

bool EstimatorInterface::isOnlyActiveSourceOfHorizontalPositionAiding(const bool aiding_flag) const
{
	return aiding_flag && !isOtherSourceOfHorizontalPositionAidingThan(aiding_flag);
}

bool EstimatorInterface::isOtherSourceOfHorizontalPositionAidingThan(const bool aiding_flag) const
{
	const int nb_sources = getNumberOfActiveHorizontalPositionAidingSources();
	return aiding_flag ? nb_sources > 1 : nb_sources > 0;
}

int EstimatorInterface::getNumberOfActiveHorizontalPositionAidingSources() const
{
	return int(_control_status.flags.gnss_pos)
	       + int(_control_status.flags.ev_pos)
	       + int(_control_status.flags.aux_gpos);
}

int EstimatorInterface::getNumberOfActiveHorizontalVelocityAidingSources() const
{
	return int(_control_status.flags.gnss_vel)
	       + int(_control_status.flags.opt_flow)
	       + int(_control_status.flags.ev_vel)
	       // Combined airspeed and sideslip fusion allows sustained wind relative dead reckoning
	       // and so is treated as a single aiding source.
	       + int(_control_status.flags.fuse_aspd && _control_status.flags.fuse_beta);
}

bool EstimatorInterface::isHorizontalAidingActive() const
{
	return getNumberOfActiveHorizontalAidingSources() > 0;
}

bool EstimatorInterface::isOtherSourceOfVerticalPositionAidingThan(const bool aiding_flag) const
{
	const int nb_sources = getNumberOfActiveVerticalPositionAidingSources();
	return aiding_flag ? nb_sources > 1 : nb_sources > 0;
}

bool EstimatorInterface::isVerticalPositionAidingActive() const
{
	return getNumberOfActiveVerticalPositionAidingSources() > 0;
}

bool EstimatorInterface::isOnlyActiveSourceOfVerticalPositionAiding(const bool aiding_flag) const
{
	return aiding_flag && !isOtherSourceOfVerticalPositionAidingThan(aiding_flag);
}

int EstimatorInterface::getNumberOfActiveVerticalPositionAidingSources() const
{
	return int(_control_status.flags.gps_hgt)
	       + int(_control_status.flags.baro_hgt)
	       + int(_control_status.flags.rng_hgt)
	       + int(_control_status.flags.ev_hgt);
}

bool EstimatorInterface::isVerticalAidingActive() const
{
	return isVerticalPositionAidingActive() || isVerticalVelocityAidingActive();
}

bool EstimatorInterface::isVerticalVelocityAidingActive() const
{
	return getNumberOfActiveVerticalVelocityAidingSources() > 0;
}

int EstimatorInterface::getNumberOfActiveVerticalVelocityAidingSources() const
{
	return int(_control_status.flags.gnss_vel)
	       + int(_control_status.flags.ev_vel);
}

bool EstimatorInterface::isNorthEastAidingActive() const
{
	return _control_status.flags.gnss_pos
	       || _control_status.flags.gnss_vel
	       || _control_status.flags.aux_gpos;
}

void EstimatorInterface::printBufferAllocationFailed(const char *buffer_name)
{
	if (buffer_name) {
		ECL_ERR("%s buffer allocation failed", buffer_name);
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file height_control.cpp
 */

#include "ekf.h"

void Ekf::controlHeightFusion(const imuSample &imu_delayed)
{
	checkVerticalAccelerationHealth(imu_delayed);

#if defined(CONFIG_EKF2_BAROMETER)
	updateGroundEffect();

	controlBaroHeightFusion(imu_delayed);
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_GNSS)
	controlGnssHeightFusion(_gps_sample_delayed);
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_RANGE_FINDER)
	controlRangeHaglFusion(imu_delayed);
#endif // CONFIG_EKF2_RANGE_FINDER

	checkHeightSensorRefFallback();
}

void Ekf::checkHeightSensorRefFallback()
{
	if (_height_sensor_ref != HeightSensor::UNKNOWN) {
		// The reference sensor is running, all good
		return;
	}

	HeightSensor fallback_list[4];

	switch (static_cast<HeightSensor>(_params.height_sensor_ref)) {
	default:

	/* FALLTHROUGH */
	case HeightSensor::UNKNOWN:
		fallback_list[0] = HeightSensor::GNSS;
		fallback_list[1] = HeightSensor::BARO;
		fallback_list[2] = HeightSensor::EV;
		fallback_list[3] = HeightSensor::RANGE;
		break;

	case HeightSensor::BARO:
		fallback_list[0] = HeightSensor::BARO;
		fallback_list[1] = HeightSensor::GNSS;
		fallback_list[2] = HeightSensor::EV;
		fallback_list[3] = HeightSensor::RANGE;
		break;

	case HeightSensor::GNSS:
		fallback_list[0] = HeightSensor::GNSS;
		fallback_list[1] = HeightSensor::BARO;
		fallback_list[2] = HeightSensor::EV;
		fallback_list[3] = HeightSensor::RANGE;
		break;

	case HeightSensor::RANGE:
		fallback_list[0] = HeightSensor::RANGE;
		fallback_list[1] = HeightSensor::EV;
		fallback_list[2] = HeightSensor::BARO;
		fallback_list[3] = HeightSensor::GNSS;
		break;

	case HeightSensor::EV:
		fallback_list[0] = HeightSensor::EV;
		fallback_list[1] = HeightSensor::RANGE;
		fallback_list[2] = HeightSensor::BARO;
		fallback_list[3] = HeightSensor::GNSS;
		break;
	}

	for (unsigned i = 0; i < 4; i++) {
		if (((fallback_list[i] == HeightSensor::BARO) && _control_status.flags.baro_hgt)
		    || ((fallback_list[i] == HeightSensor::GNSS) && _control_status.flags.gps_hgt)
		    || ((fallback_list[i] == HeightSensor::RANGE) && _control_status.flags.rng_hgt)
		    || ((fallback_list[i] == HeightSensor::EV) && _control_status.flags.ev_hgt)) {
			ECL_INFO("fallback to secondary height reference");
			_height_sensor_ref = fallback_list[i];
			break;
		}
	}
}

void Ekf::checkVerticalAccelerationHealth(const imuSample &imu_delayed)
{
	// Check for IMU accelerometer vibration induced clipping as evidenced by the vertical
	// innovations being positive and not stale.
	// Clipping usually causes the average accel reading to move towards zero which makes the INS
	// think it is falling and produces positive vertical innovations.

	Likelihood inertial_nav_falling_likelihood = estimateInertialNavFallingLikelihood();

	const uint16_t kClipCountLimit = 1.f / _dt_ekf_avg;

	bool acc_clip_warning[3] {};
	bool acc_clip_critical[3] {};

	for (int axis = 0; axis < 3; axis++) {
		if (imu_delayed.delta_vel_clipping[axis] && (_clip_counter[axis] < kClipCountLimit)) {
			_clip_counter[axis]++;

		} else if (_clip_counter[axis] > 0) {
			_clip_counter[axis]--;
		}

		// warning if more than 50% clipping affected IMU samples within the past 1 second
		acc_clip_warning[axis] = _clip_counter[axis] >= kClipCountLimit / 2;
		acc_clip_critical[axis] = _clip_counter[axis] >= kClipCountLimit;
	}

	// bad_acc_clipping if ALL axes are reporting warning or if ANY axis is critical
	const bool all_axis_warning = (acc_clip_warning[0] && acc_clip_warning[1] && acc_clip_warning[2]);
	const bool any_axis_critical = (acc_clip_critical[0] || acc_clip_critical[1] || acc_clip_critical[2]);

	_fault_status.flags.bad_acc_clipping = all_axis_warning || any_axis_critical;

	// if Z axis is warning or any other axis critical
	const bool is_clipping_frequently = acc_clip_warning[2] || _fault_status.flags.bad_acc_clipping;

	// Do not require evidence of clipping if the likelihood of having the INS falling is high
	const bool bad_vert_accel = (is_clipping_frequently && (inertial_nav_falling_likelihood == Likelihood::MEDIUM))
				    || (inertial_nav_falling_likelihood == Likelihood::HIGH);

	if (bad_vert_accel) {
		_time_bad_vert_accel = imu_delayed.time_us;

	} else {
		_time_good_vert_accel = imu_delayed.time_us;
	}

	// declare a bad vertical acceleration measurement and make the declaration persist
	// for a minimum of BADACC_PROBATION seconds
	if (_fault_status.flags.bad_acc_vertical) {
		_fault_status.flags.bad_acc_vertical = isRecent(_time_bad_vert_accel, BADACC_PROBATION);

	} else {
		_fault_status.flags.bad_acc_vertical = bad_vert_accel;
	}
}

Likelihood Ekf::estimateInertialNavFallingLikelihood() const
{
	bool likelihood_high = false;
	bool likelihood_medium = false;

	enum class ReferenceType { PRESSURE, GNSS, GROUND };

	struct {
		ReferenceType ref_type{};
		float innov{0.f};
		float innov_var{0.f};
		bool failed_min{false};
		bool failed_lim{false};
	} checks[6] {};

#if defined(CONFIG_EKF2_BAROMETER)

	if (_control_status.flags.baro_hgt) {
		checks[0] = {ReferenceType::PRESSURE, _aid_src_baro_hgt.innovation, _aid_src_baro_hgt.innovation_variance};
	}

#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_GNSS)

	if (_control_status.flags.gps_hgt) {
		checks[1] = {ReferenceType::GNSS, _aid_src_gnss_hgt.innovation, _aid_src_gnss_hgt.innovation_variance};
	}

	if (_control_status.flags.gnss_vel) {
		checks[2] = {ReferenceType::GNSS, _aid_src_gnss_vel.innovation[2], _aid_src_gnss_vel.innovation_variance[2]};
	}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_RANGE_FINDER)

	if (_control_status.flags.rng_hgt) {
		// Range is a distance to ground measurement, not a direct height observation and has an opposite sign
		checks[3] = {ReferenceType::GROUND, -_aid_src_rng_hgt.innovation, _aid_src_rng_hgt.innovation_variance};
	}

#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_hgt) {
		checks[4] = {ReferenceType::GROUND, _aid_src_ev_hgt.innovation, _aid_src_ev_hgt.innovation_variance};
	}

	if (_control_status.flags.ev_vel) {
		checks[5] = {ReferenceType::GROUND, _aid_src_ev_vel.innovation[2], _aid_src_ev_vel.innovation_variance[2]};
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	// Compute the check based on innovation ratio for all the sources
	for (unsigned i = 0; i < 6; i++) {
		if (checks[i].innov_var < FLT_EPSILON) {
			continue;
		}

		const float innov_ratio = checks[i].innov / sqrtf(checks[i].innov_var);
		checks[i].failed_min = innov_ratio > _params.vert_innov_test_min;
		checks[i].failed_lim = innov_ratio > _params.vert_innov_test_lim;
	}

	// Check all the sources agains each other
	for (unsigned i = 0; i < 6; i++) {
		if (checks[i].failed_lim) {
			// There is a chance that the inertial nav is falling if one source is failing the test
			likelihood_medium = true;
		}

		for (unsigned j = 0; j < 6; j++) {

			if ((checks[i].ref_type != checks[j].ref_type) && checks[i].failed_lim && checks[j].failed_min) {
				// There is a high chance that the inertial nav is failing if two sources are failing the test
				likelihood_high = true;
			}
		}
	}

	if (likelihood_high) {
		return Likelihood::HIGH;

	} else if (likelihood_medium) {
		return Likelihood::MEDIUM;
	}

	return Likelihood::LOW;
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ekf.h"

bool Ekf::fuseHorizontalVelocity(estimator_aid_source2d_s &aid_src)
{
	// vx, vy
	if (!aid_src.innovation_rejected) {
		for (unsigned i = 0; i < 2; i++) {
			fuseDirectStateMeasurement(aid_src.innovation[i], aid_src.innovation_variance[i], aid_src.observation_variance[i],
						   State::vel.idx + i);
		}

		aid_src.fused = true;
		aid_src.time_last_fuse = _time_delayed_us;

		_time_last_hor_vel_fuse = _time_delayed_us;

	} else {
		aid_src.fused = false;
	}

	return aid_src.fused;
}

bool Ekf::fuseVelocity(estimator_aid_source3d_s &aid_src)
{
	// vx, vy, vz
	if (!aid_src.innovation_rejected) {
		for (unsigned i = 0; i < 3; i++) {
			fuseDirectStateMeasurement(aid_src.innovation[i], aid_src.innovation_variance[i], aid_src.observation_variance[i],
						   State::vel.idx + i);
		}

		aid_src.fused = true;
		aid_src.time_last_fuse = _time_delayed_us;

		_time_last_hor_vel_fuse = _time_delayed_us;
		_time_last_ver_vel_fuse = _time_delayed_us;

	} else {
		aid_src.fused = false;
	}

	return aid_src.fused;
}

void Ekf::resetHorizontalVelocityTo(const Vector2f &new_horz_vel, const Vector2f &new_horz_vel_var)
{
	const Vector2f delta_horz_vel = new_horz_vel - Vector2f(_state.vel);
	_state.vel.xy() = new_horz_vel;

	if (PX4_ISFINITE(new_horz_vel_var(0))) {
		P.uncorrelateCovarianceSetVariance<1>(State::vel.idx, math::max(sq(0.01f), new_horz_vel_var(0)));
	}

	if (PX4_ISFINITE(new_horz_vel_var(1))) {
		P.uncorrelateCovarianceSetVariance<1>(State::vel.idx + 1, math::max(sq(0.01f), new_horz_vel_var(1)));
	}

	P.uncorrelateCovarianceSetVariance<1>(State::pos.idx, P(State::pos.idx, State::pos.idx));
	P.uncorrelateCovarianceSetVariance<1>(State::pos.idx + 1, P(State::pos.idx + 1, State::pos.idx + 1));

	_output_predictor.resetHorizontalVelocityTo(delta_horz_vel);

	// record the state change
	if (_state_reset_status.reset_count.velNE == _state_reset_count_prev.velNE) {
		_state_reset_status.velNE_change = delta_horz_vel;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.velNE_change += delta_horz_vel;
	}

	_state_reset_status.reset_count.velNE++;

	// Reset the timout timer
	_time_last_hor_vel_fuse = _time_delayed_us;
}

void Ekf::resetVerticalVelocityTo(float new_vert_vel, float new_vert_vel_var)
{
	const float delta_vert_vel = new_vert_vel - _state.vel(2);
	_state.vel(2) = new_vert_vel;

	if (PX4_ISFINITE(new_vert_vel_var)) {
		P.uncorrelateCovarianceSetVariance<1>(State::vel.idx + 2, math::max(sq(0.01f), new_vert_vel_var));
	}

	P.uncorrelateCovarianceSetVariance<1>(State::pos.idx + 2, P(State::pos.idx + 2, State::pos.idx + 2));

	_output_predictor.resetVerticalVelocityTo(delta_vert_vel);

	// record the state change
	if (_state_reset_status.reset_count.velD == _state_reset_count_prev.velD) {
		_state_reset_status.velD_change = delta_vert_vel;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.velD_change += delta_vert_vel;
	}

	_state_reset_status.reset_count.velD++;

	// Reset the timout timer
	_time_last_ver_vel_fuse = _time_delayed_us;
}

void Ekf::resetHorizontalVelocityToZero()
{
	ECL_INFO("reset velocity to zero");
	_information_events.flags.reset_vel_to_zero = true;

	// Used when falling back to non-aiding mode of operation
	resetHorizontalVelocityTo(Vector2f{0.f, 0.f}, 25.f);
}

void Ekf::resetVerticalVelocityToZero()
{
	// we don't know what the vertical velocity is, so set it to zero
	// Set the variance to a value large enough to allow the state to converge quickly
	// that does not destabilise the filter
	resetVerticalVelocityTo(0.0f, 10.f);
}

void Ekf::resetVelocityTo(const Vector3f &new_vel, const Vector3f &new_vel_var)
{
	resetHorizontalVelocityTo(Vector2f(new_vel), Vector2f(new_vel_var(0), new_vel_var(1)));
	resetVerticalVelocityTo(new_vel(2), new_vel_var(2));
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ekf.h"

void Ekf::updateVerticalPositionAidStatus(estimator_aid_source1d_s &aid_src, const uint64_t &time_us,
		const float observation, const float observation_variance, const float innovation_gate) const
{
	float innovation = -_gpos.altitude() - observation;
	float innovation_variance = getStateVariance<State::pos>()(2) + observation_variance;

	updateAidSourceStatus(aid_src, time_us,
			      observation, observation_variance,
			      innovation, innovation_variance,
			      innovation_gate);

	// z special case if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	if (_fault_status.flags.bad_acc_vertical && aid_src.innovation_rejected) {
		const float innov_limit = innovation_gate * sqrtf(aid_src.innovation_variance);
		aid_src.innovation = math::constrain(aid_src.innovation, -innov_limit, innov_limit);
		aid_src.innovation_rejected = false;
	}
}

bool Ekf::fuseHorizontalPosition(estimator_aid_source2d_s &aid_src)
{
	// x & y
	if (!aid_src.innovation_rejected) {
		for (unsigned i = 0; i < 2; i++) {
			fuseDirectStateMeasurement(aid_src.innovation[i], aid_src.innovation_variance[i], aid_src.observation_variance[i],
						   State::pos.idx + i);
		}

		aid_src.fused = true;
		aid_src.time_last_fuse = _time_delayed_us;

		_time_last_hor_pos_fuse = _time_delayed_us;

	} else {
		aid_src.fused = false;
	}

	return aid_src.fused;
}

bool Ekf::fuseVerticalPosition(estimator_aid_source1d_s &aid_src)
{
	// z
	if (!aid_src.innovation_rejected) {
		fuseDirectStateMeasurement(aid_src.innovation, aid_src.innovation_variance, aid_src.observation_variance,
					   State::pos.idx + 2);

		aid_src.fused = true;
		aid_src.time_last_fuse = _time_delayed_us;

		_time_last_hgt_fuse = _time_delayed_us;

	} else {
		aid_src.fused = false;
	}

	return aid_src.fused;
}

void Ekf::resetHorizontalPositionTo(const double &new_latitude, const double &new_longitude,
				    const Vector2f &new_horz_pos_var)
{
	const Vector2f delta_horz_pos = computeDeltaHorizontalPosition(new_latitude, new_longitude);

	updateHorizontalPositionResetStatus(delta_horz_pos);

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	_ev_pos_b_est.setBias(_ev_pos_b_est.getBias() - delta_horz_pos);
#endif // CONFIG_EKF2_EXTERNAL_VISION
	//_gps_pos_b_est.setBias(_gps_pos_b_est.getBias() + _state_reset_status.posNE_change);

	_gpos.setLatLonDeg(new_latitude, new_longitude);
	_output_predictor.resetLatLonTo(new_latitude, new_longitude);

	if (PX4_ISFINITE(new_horz_pos_var(0))) {
		P.uncorrelateCovarianceSetVariance<1>(State::pos.idx, math::max(sq(0.01f), new_horz_pos_var(0)));
	}

	if (PX4_ISFINITE(new_horz_pos_var(1))) {
		P.uncorrelateCovarianceSetVariance<1>(State::pos.idx + 1, math::max(sq(0.01f), new_horz_pos_var(1)));
	}

	// Reset the timout timer
	_time_last_hor_pos_fuse = _time_delayed_us;
}

Vector2f Ekf::computeDeltaHorizontalPosition(const double &new_latitude, const double &new_longitude) const
{
	Vector2f pos;
	Vector2f pos_new;

	if (_local_origin_lat_lon.isInitialized()) {
		_local_origin_lat_lon.project(_gpos.latitude_deg(), _gpos.longitude_deg(), pos(0), pos(1));
		_local_origin_lat_lon.project(new_latitude, new_longitude, pos_new(0), pos_new(1));

	} else {
		MapProjection zero_ref;
		zero_ref.initReference(0.0, 0.0);
		zero_ref.project(_gpos.latitude_deg(), _gpos.longitude_deg(), pos(0), pos(1));
		zero_ref.project(new_latitude, new_longitude, pos_new(0), pos_new(1));
	}

	return pos_new - pos;
}

Vector2f Ekf::getLocalHorizontalPosition() const
{
	if (_local_origin_lat_lon.isInitialized()) {
		return _local_origin_lat_lon.project(_gpos.latitude_deg(), _gpos.longitude_deg());

	} else {
		MapProjection zero_ref;
		zero_ref.initReference(0.0, 0.0);
		return zero_ref.project(_gpos.latitude_deg(), _gpos.longitude_deg());
	}
}

void Ekf::updateHorizontalPositionResetStatus(const Vector2f &delta)
{
	if (_state_reset_status.reset_count.posNE == _state_reset_count_prev.posNE) {
		_state_reset_status.posNE_change = delta;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.posNE_change += delta;
	}

	_state_reset_status.reset_count.posNE++;
}

void Ekf::resetHorizontalPositionTo(const Vector2f &new_pos,
				    const Vector2f &new_horz_pos_var)
{
	double new_latitude;
	double new_longitude;

	if (_local_origin_lat_lon.isInitialized()) {
		_local_origin_lat_lon.reproject(new_pos(0), new_pos(1), new_latitude, new_longitude);

	} else {
		MapProjection zero_ref;
		zero_ref.initReference(0.0, 0.0);
		zero_ref.reproject(new_pos(0), new_pos(1), new_latitude, new_longitude);
	}

	resetHorizontalPositionTo(new_latitude, new_longitude, new_horz_pos_var);
}

void Ekf::resetAltitudeTo(const float new_altitude, float new_vert_pos_var)
{
	const float old_altitude = _gpos.altitude();
	_gpos.setAltitude(new_altitude);

	if (PX4_ISFINITE(new_vert_pos_var)) {
		// the state variance is the same as the observation
		P.uncorrelateCovarianceSetVariance<1>(State::pos.idx + 2, math::max(sq(0.01f), new_vert_pos_var));
	}

	const float delta_z = -(new_altitude - old_altitude);

	// apply the change in height / height rate to our newest height / height rate estimate
	// which have already been taken out from the output buffer
	_output_predictor.resetAltitudeTo(new_altitude, delta_z);

	updateVerticalPositionResetStatus(delta_z);

#if defined(CONFIG_EKF2_BAROMETER)
	_baro_b_est.setBias(_baro_b_est.getBias() + delta_z);
#endif // CONFIG_EKF2_BAROMETER
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	_ev_hgt_b_est.setBias(_ev_hgt_b_est.getBias() - delta_z);
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_GNSS)
	_gps_hgt_b_est.setBias(_gps_hgt_b_est.getBias() + delta_z);
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_TERRAIN)
	updateTerrainResetStatus(delta_z);
	_state.terrain += delta_z;
#endif // CONFIG_EKF2_TERRAIN

	// Reset the timout timer
	_time_last_hgt_fuse = _time_delayed_us;
}

void Ekf::updateVerticalPositionResetStatus(const float delta_z)
{
	if (_state_reset_status.reset_count.posD == _state_reset_count_prev.posD) {
		_state_reset_status.posD_change = delta_z;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.posD_change += delta_z;
	}

	_state_reset_status.reset_count.posD++;
}

void Ekf::updateTerrainResetStatus(const float delta_z)
{
	if (_state_reset_status.reset_count.hagl == _state_reset_count_prev.hagl) {
		_state_reset_status.hagl_change = delta_z;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.hagl_change += delta_z;
	}

	_state_reset_status.reset_count.hagl++;
}

void Ekf::resetHorizontalPositionToLastKnown()
{
	ECL_INFO("reset position to last known (%.3f, %.3f)", (double)_last_known_gpos.latitude_deg(),
		 (double)_last_known_gpos.longitude_deg());
	_information_events.flags.reset_pos_to_last_known = true;

	// Used when falling back to non-aiding mode of operation
	resetHorizontalPositionTo(_last_known_gpos.latitude_deg(), _last_known_gpos.longitude_deg(),
				  sq(_params.pos_noaid_noise));
}
/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ekf.h"

#include <ekf_derivation/generated/compute_yaw_innov_var_and_h.h>

#include <mathlib/mathlib.h>

bool Ekf::fuseYaw(estimator_aid_source1d_s &aid_src_status, const VectorState &H_YAW)
{
	// check if the innovation variance calculation is badly conditioned
	if (aid_src_status.innovation_variance >= aid_src_status.observation_variance) {
		// the innovation variance contribution from the state covariances is not negative, no fault
		_fault_status.flags.bad_hdg = false;

	} else {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_hdg = true;

		// we reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		ECL_ERR("yaw fusion numerical error - covariance reset");

		return false;
	}

	// calculate the Kalman gains
	// only calculate gains for states we are using
	VectorState Kfusion;
	const float heading_innov_var_inv = 1.f / aid_src_status.innovation_variance;

	for (uint8_t row = 0; row < State::size; row++) {
		for (uint8_t col = 0; col <= 3; col++) {
			Kfusion(row) += P(row, col) * H_YAW(col);
		}

		Kfusion(row) *= heading_innov_var_inv;
	}

	// set the heading unhealthy if the test fails
	if (aid_src_status.innovation_rejected) {
		_innov_check_fail_status.flags.reject_yaw = true;

		// if we are in air we don't want to fuse the measurement
		// we allow to use it when on the ground because the large innovation could be caused
		// by interference or a large initial gyro bias
		if (!_control_status.flags.in_air
		    && isTimedOut(_time_last_in_air, (uint64_t)5e6)
		    && isTimedOut(aid_src_status.time_last_fuse, (uint64_t)1e6)
		   ) {
			// constrain the innovation to the maximum set by the gate
			// we need to delay this forced fusion to avoid starting it
			// immediately after touchdown, when the drone is still armed
			const float gate_sigma = math::max(_params.heading_innov_gate, 1.f);
			const float gate_limit = sqrtf((sq(gate_sigma) * aid_src_status.innovation_variance));
			aid_src_status.innovation = math::constrain(aid_src_status.innovation, -gate_limit, gate_limit);

			// also reset the yaw gyro variance to converge faster and avoid
			// being stuck on a previous bad estimate
			resetGyroBiasZCov();

		} else {
			return false;
		}

	} else {
		_innov_check_fail_status.flags.reject_yaw = false;
	}

	measurementUpdate(Kfusion, H_YAW, aid_src_status.observation_variance, aid_src_status.innovation);

	_time_last_heading_fuse = _time_delayed_us;

	aid_src_status.time_last_fuse = _time_delayed_us;
	aid_src_status.fused = true;

	_fault_status.flags.bad_hdg = false;

	return true;
}

void Ekf::computeYawInnovVarAndH(float variance, float &innovation_variance, VectorState &H_YAW) const
{
	sym::ComputeYawInnovVarAndH(_state.vector(), P, variance, &innovation_variance, &H_YAW);
}

void Ekf::resetQuatStateYaw(float yaw, float yaw_variance)
{
	// save a copy of the quaternion state for later use in calculating the amount of reset change
	const Quatf quat_before_reset = _state.quat_nominal;

	// update the yaw angle variance
	if (PX4_ISFINITE(yaw_variance) && (yaw_variance > FLT_EPSILON)) {
		P.uncorrelateCovarianceSetVariance<1>(2, yaw_variance);
	}

	// update transformation matrix from body to world frame using the current estimate
	// update the rotation matrix using the new yaw value
	_R_to_earth = updateYawInRotMat(yaw, Dcmf(_state.quat_nominal));

	// calculate the amount that the quaternion has changed by
	const Quatf quat_after_reset(_R_to_earth);
	const Quatf q_error((quat_after_reset * quat_before_reset.inversed()).normalized());

	// update quaternion states
	_state.quat_nominal = quat_after_reset;

	// add the reset amount to the output observer buffered data
	_output_predictor.resetQuaternion(q_error);

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	// update EV attitude error filter
	if (_ev_q_error_initialized) {
		const Quatf ev_q_error_updated = (q_error * _ev_q_error_filt.getState()).normalized();
		_ev_q_error_filt.reset(ev_q_error_updated);
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	// record the state change
	if (_state_reset_status.reset_count.quat == _state_reset_count_prev.quat) {
		_state_reset_status.quat_change = q_error;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.quat_change = q_error * _state_reset_status.quat_change;
		_state_reset_status.quat_change.normalize();
	}

	_state_reset_status.reset_count.quat++;

	_time_last_heading_fuse = _time_delayed_us;
}
#include "imu_down_sampler/imu_down_sampler.hpp"

#include <lib/mathlib/mathlib.h>

ImuDownSampler::ImuDownSampler(int32_t &target_dt_us) : _target_dt_us(target_dt_us)
{
	reset();
}

// integrate imu samples until target dt reached
// assumes that dt of the gyroscope is close to the dt of the accelerometer
// returns true if target dt is reached
bool ImuDownSampler::update(const imuSample &imu_sample_new)
{
	_delta_ang_dt_avg = 0.9f * _delta_ang_dt_avg + 0.1f * imu_sample_new.delta_ang_dt;

	// accumulate time deltas
	_imu_down_sampled.time_us = imu_sample_new.time_us;
	_imu_down_sampled.delta_ang_dt += imu_sample_new.delta_ang_dt;
	_imu_down_sampled.delta_vel_dt += imu_sample_new.delta_vel_dt;
	_imu_down_sampled.delta_vel_clipping[0] |= imu_sample_new.delta_vel_clipping[0];
	_imu_down_sampled.delta_vel_clipping[1] |= imu_sample_new.delta_vel_clipping[1];
	_imu_down_sampled.delta_vel_clipping[2] |= imu_sample_new.delta_vel_clipping[2];

	// use a quaternion to accumulate delta angle data
	// this quaternion represents the rotation from the start to end of the accumulation period
	const Quatf delta_q(AxisAnglef(imu_sample_new.delta_ang));
	_delta_angle_accumulated = _delta_angle_accumulated * delta_q;
	_delta_angle_accumulated.normalize();

	// rotate the accumulated delta velocity data forward each time so it is always in the updated rotation frame
	const Dcmf delta_R(delta_q.inversed());
	_imu_down_sampled.delta_vel = delta_R * _imu_down_sampled.delta_vel;

	// accumulate the most recent delta velocity data at the updated rotation frame
	// assume effective sample time is halfway between the previous and current rotation frame
	_imu_down_sampled.delta_vel += (imu_sample_new.delta_vel + delta_R * imu_sample_new.delta_vel) * 0.5f;

	_accumulated_samples++;


	// required number of samples accumulated and the total time is at least half of the target
	//  OR total time already exceeds the target
	if ((_accumulated_samples >= _required_samples && _imu_down_sampled.delta_ang_dt > _min_dt_s)
	    || (_imu_down_sampled.delta_ang_dt > _target_dt_s)) {

		_imu_down_sampled.delta_ang = AxisAnglef(_delta_angle_accumulated);
		return true;
	}

	return false;
}

void ImuDownSampler::reset()
{
	_imu_down_sampled = {};
	_delta_angle_accumulated.setIdentity();
	_accumulated_samples = 0;

	// target dt in seconds safely constrained
	float target_dt_s = math::constrain(_target_dt_us, (int32_t)1000, (int32_t)100000) * 1e-6f;

	_required_samples = math::max((int)roundf(target_dt_s / _delta_ang_dt_avg), 1);

	_target_dt_s = _required_samples * _delta_ang_dt_avg;

	// minimum delta angle dt (in addition to number of samples)
	_min_dt_s = math::max(_delta_ang_dt_avg * (_required_samples - 1.f), _delta_ang_dt_avg * 0.5f);
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file fake_height_control.cpp
 * Control functions for ekf fake height fusion
 */

#include "ekf.h"

void Ekf::controlFakeHgtFusion()
{
	auto &aid_src = _aid_src_fake_hgt;

	// If we aren't doing any aiding, fake position measurements at the last known vertical position to constrain drift
	const bool fake_hgt_data_ready = !isVerticalAidingActive()
					 && isTimedOut(aid_src.time_last_fuse, (uint64_t)2e5); // Fuse fake height at a limited rate

	if (fake_hgt_data_ready) {

		const float obs_var = sq(_params.pos_noaid_noise);
		const float innov_gate = 3.f;

		updateVerticalPositionAidStatus(aid_src, _time_delayed_us, -_last_known_gpos.altitude(), obs_var, innov_gate);

		const bool continuing_conditions_passing = !isVerticalAidingActive();
		const bool starting_conditions_passing = continuing_conditions_passing
				&& _vertical_velocity_deadreckon_time_exceeded
				&& _vertical_position_deadreckon_time_exceeded;

		if (_control_status.flags.fake_hgt) {
			if (continuing_conditions_passing) {

				// always protect against extreme values that could result in a NaN
				if (aid_src.test_ratio < sq(100.0f / innov_gate)) {
					if (!aid_src.innovation_rejected) {
						fuseDirectStateMeasurement(aid_src.innovation, aid_src.innovation_variance, aid_src.observation_variance,
									   State::pos.idx + 2);

						aid_src.fused = true;
						aid_src.time_last_fuse = _time_delayed_us;
					}
				}

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, (uint64_t)4e5);

				if (is_fusion_failing) {
					resetFakeHgtFusion();
				}

			} else {
				stopFakeHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				ECL_INFO("start fake height fusion");
				_control_status.flags.fake_hgt = true;
				resetFakeHgtFusion();
			}
		}

	} else if (_control_status.flags.fake_hgt && isVerticalAidingActive()) {
		stopFakeHgtFusion();
	}
}

void Ekf::resetFakeHgtFusion()
{
	ECL_INFO("reset fake height fusion");
	_last_known_gpos.setAltitude(_gpos.altitude());

	resetVerticalVelocityToZero();
	resetHeightToLastKnown();

	_aid_src_fake_hgt.time_last_fuse = _time_delayed_us;
}

void Ekf::resetHeightToLastKnown()
{
	_information_events.flags.reset_pos_to_last_known = true;
	ECL_INFO("reset height to last known (%.3f)", (double)_last_known_gpos.altitude());
	resetAltitudeTo(_last_known_gpos.altitude(), sq(_params.pos_noaid_noise));
}

void Ekf::stopFakeHgtFusion()
{
	if (_control_status.flags.fake_hgt) {
		ECL_INFO("stop fake height fusion");
		_control_status.flags.fake_hgt = false;
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file fake_pos_control.cpp
 * Control functions for ekf fake position fusion
 */

#include "ekf.h"

void Ekf::controlFakePosFusion()
{
	auto &aid_src = _aid_src_fake_pos;

	// If we aren't doing any aiding, fake position measurements at the last known position to constrain drift
	// During initial tilt alignment, fake position is used to perform a "quasi-stationary" leveling of the EKF
	const bool fake_pos_data_ready = !isHorizontalAidingActive()
					 && isTimedOut(aid_src.time_last_fuse, (uint64_t)2e5); // Fuse fake position at a limited rate

	if (fake_pos_data_ready) {

		Vector2f obs_var;

		if (_control_status.flags.in_air && _control_status.flags.tilt_align) {
			obs_var(0) = obs_var(1) = sq(fmaxf(_params.pos_noaid_noise, 1.f));

		} else if (!_control_status.flags.in_air && _control_status.flags.vehicle_at_rest) {
			// Accelerate tilt fine alignment by fusing more
			// aggressively when the vehicle is at rest
			obs_var(0) = obs_var(1) = sq(0.01f);

		} else {
			obs_var(0) = obs_var(1) = sq(0.5f);
		}

		const Vector2f innovation = (_gpos - _last_known_gpos).xy();

		const float innov_gate = 3.f;

		updateAidSourceStatus(aid_src,
				      _time_delayed_us,
				      Vector2f(_gpos.latitude_deg(), _gpos.longitude_deg()), // observation
				      obs_var,                                               // observation variance
				      innovation,                       // innovation
				      Vector2f(getStateVariance<State::pos>()) + obs_var,    // innovation variance
				      innov_gate);                                           // innovation gate

		const bool enable_valid_fake_pos = _control_status.flags.constant_pos || _control_status.flags.vehicle_at_rest;
		const bool enable_fake_pos = !enable_valid_fake_pos
					     && (getTiltVariance() > sq(math::radians(3.f)))
					     && !(_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::GravityVector))
					     && _horizontal_deadreckon_time_exceeded;

		_control_status.flags.fake_pos = runFakePosStateMachine(enable_fake_pos, _control_status.flags.fake_pos, aid_src);
		_control_status.flags.valid_fake_pos = runFakePosStateMachine(enable_valid_fake_pos,
						       _control_status.flags.valid_fake_pos, aid_src);

	} else if ((_control_status.flags.fake_pos || _control_status.flags.valid_fake_pos) && isHorizontalAidingActive()) {
		ECL_INFO("stop fake position fusion");
		_control_status.flags.fake_pos = false;
		_control_status.flags.valid_fake_pos = false;
	}
}

void Ekf::resetFakePosFusion()
{
	ECL_INFO("reset fake position fusion");
	_last_known_gpos.setLatLon(_gpos);

	resetHorizontalPositionToLastKnown();
	resetHorizontalVelocityToZero();

	_aid_src_fake_pos.time_last_fuse = _time_delayed_us;
}

bool Ekf::runFakePosStateMachine(const bool enable_conditions_passing, bool status_flag,
				 estimator_aid_source2d_s &aid_src)
{
	if (status_flag) {
		if (enable_conditions_passing) {
			if (!aid_src.innovation_rejected) {
				for (unsigned i = 0; i < 2; i++) {
					fuseDirectStateMeasurement(aid_src.innovation[i], aid_src.innovation_variance[i], aid_src.observation_variance[i],
								   State::pos.idx + i);
				}

				aid_src.fused = true;
				aid_src.time_last_fuse = _time_delayed_us;
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, (uint64_t)4e5);

			if (is_fusion_failing) {
				ECL_WARN("fake position fusion failing, resetting");
				resetFakePosFusion();
			}

		} else {
			ECL_INFO("stop fake position fusion");
			status_flag = false;
		}

	} else {
		if (enable_conditions_passing) {
			ECL_INFO("start fake position fusion");
			status_flag = true;

			resetFakePosFusion();
		}
	}

	return status_flag;
}
/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ZeroGyroUpdate.hpp"

#include "../ekf.h"

ZeroGyroUpdate::ZeroGyroUpdate()
{
	reset();
}

void ZeroGyroUpdate::reset()
{
	_zgup_delta_ang.setZero();
	_zgup_delta_ang_dt = 0.f;
}

bool ZeroGyroUpdate::update(Ekf &ekf, const estimator::imuSample &imu_delayed)
{
	// When at rest, fuse the gyro data as a direct observation of the gyro bias
	if (ekf.control_status_flags().vehicle_at_rest) {
		// Downsample gyro data to run the fusion at a lower rate
		_zgup_delta_ang += imu_delayed.delta_ang;
		_zgup_delta_ang_dt += imu_delayed.delta_ang_dt;

		static constexpr float zgup_dt = 0.2f;
		const bool zero_gyro_update_data_ready = _zgup_delta_ang_dt >= zgup_dt;

		if (zero_gyro_update_data_ready) {

			Vector3f gyro_bias = _zgup_delta_ang / _zgup_delta_ang_dt;

			const float obs_var = sq(math::constrain(ekf.getGyroNoise(), 0.f, 1.f));

			for (unsigned i = 0; i < 3; i++) {
				const float innovation = ekf.state().gyro_bias(i) - gyro_bias(i);
				const float innov_var = ekf.getGyroBiasVariance()(i) + obs_var;
				ekf.fuseDirectStateMeasurement(innovation, innov_var, obs_var, State::gyro_bias.idx + i);
			}

			// Reset the integrators
			_zgup_delta_ang.setZero();
			_zgup_delta_ang_dt = 0.f;

			return true;
		}

	} else if (ekf.control_status_prev_flags().vehicle_at_rest) {
		// Reset the integrators
		_zgup_delta_ang.setZero();
		_zgup_delta_ang_dt = 0.f;
	}

	return false;
}
/****************************************************************************
 *
 *   Copyright (c) 2022-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ZeroVelocityUpdate.hpp"

#include "../ekf.h"

ZeroVelocityUpdate::ZeroVelocityUpdate()
{
	reset();
}

void ZeroVelocityUpdate::reset()
{
	_time_last_fuse = 0;
}

bool ZeroVelocityUpdate::update(Ekf &ekf, const estimator::imuSample &imu_delayed)
{
	// Fuse zero velocity at a limited rate (every 200 milliseconds)
	const bool zero_velocity_update_data_ready = (_time_last_fuse + 200'000 < imu_delayed.time_us);

	if (zero_velocity_update_data_ready) {
		const bool continuing_conditions_passing = ekf.control_status_flags().vehicle_at_rest
				&& ekf.control_status_prev_flags().vehicle_at_rest
				&& (!ekf.isVerticalVelocityAidingActive()
				    || !ekf.control_status_flags().tilt_align); // otherwise the filter is "too rigid" to follow a position drift

		if (continuing_conditions_passing) {
			Vector3f vel_obs{0.f, 0.f, 0.f};

			// Set a low variance initially for faster leveling and higher
			// later to let the states follow the measurements
			const float obs_var = ekf.control_status_flags().tilt_align ? sq(0.2f) : sq(0.001f);
			Vector3f innov_var = ekf.getVelocityVariance() + obs_var;

			for (unsigned i = 0; i < 3; i++) {
				const float innovation = ekf.state().vel(i) - vel_obs(i);
				ekf.fuseDirectStateMeasurement(innovation, innov_var(i), obs_var, State::vel.idx + i);
			}

			_time_last_fuse = imu_delayed.time_us;

			return true;
		}
	}

	return false;
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file zero_innovation_heading_update.cpp
 * Control function for ekf heading update when at rest or no other heading source available
 */

#include "ekf.h"

void Ekf::controlZeroInnovationHeadingUpdate()
{
	const bool yaw_aiding = _control_status.flags.mag_hdg || _control_status.flags.mag_3D
				|| _control_status.flags.ev_yaw || _control_status.flags.gnss_yaw;

	// fuse zero innovation at a limited rate if the yaw variance is too large
	if (!yaw_aiding
	    && isTimedOut(_time_last_heading_fuse, (uint64_t)200'000)) {

		// Use an observation variance larger than usual but small enough
		// to constrain the yaw variance just below the threshold
		const float obs_var = _control_status.flags.tilt_align ? 0.25f : 0.001f;

		estimator_aid_source1d_s aid_src_status{};
		aid_src_status.observation = getEulerYaw(_state.quat_nominal);
		aid_src_status.observation_variance = obs_var;
		aid_src_status.innovation = 0.f;

		VectorState H_YAW;

		computeYawInnovVarAndH(obs_var, aid_src_status.innovation_variance, H_YAW);

		if (!_control_status.flags.tilt_align
		    || (aid_src_status.innovation_variance - obs_var) > sq(_params.mag_heading_noise)) {
			// The yaw variance is too large, fuse fake measurement
			fuseYaw(aid_src_status, H_YAW);
		}
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file airspeed_fusion.cpp
 * airspeed fusion methods.
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Carl Olsson <carlolsson.co@gmail.com>
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <ekf_derivation/generated/compute_airspeed_h.h>
#include <ekf_derivation/generated/compute_airspeed_innov_and_innov_var.h>
#include <ekf_derivation/generated/compute_wind_init_and_cov_from_airspeed.h>

#include <mathlib/mathlib.h>

void Ekf::controlAirDataFusion(const imuSample &imu_delayed)
{
	// control activation and initialisation/reset of wind states required for airspeed fusion

	// If both airspeed and sideslip fusion have timed out and we are not using a drag observation model then we no longer have valid wind estimates
	const bool airspeed_timed_out = isTimedOut(_aid_src_airspeed.time_last_fuse, (uint64_t)10e6);
	const bool sideslip_timed_out = isTimedOut(_aid_src_sideslip.time_last_fuse, (uint64_t)10e6);

	if (_control_status.flags.fake_pos || (airspeed_timed_out && sideslip_timed_out && (_params.drag_ctrl == 0))) {
		_control_status.flags.wind = false;
	}

	if (_control_status.flags.wind && _external_wind_init) {
		_external_wind_init = false;
	}

#if defined(CONFIG_EKF2_GNSS)

	// clear yaw estimator airspeed (updated later with true airspeed if airspeed fusion is active)
	if (_control_status.flags.fixed_wing) {
		if (_control_status.flags.in_air && !_control_status.flags.vehicle_at_rest) {
			if (!_control_status.flags.fuse_aspd) {
				_yawEstimator.setTrueAirspeed(_params.EKFGSF_tas_default);
			}

		} else {
			_yawEstimator.setTrueAirspeed(0.f);
		}
	}

#endif // CONFIG_EKF2_GNSS

	if (_params.arsp_thr <= 0.f) {
		stopAirspeedFusion();
		return;
	}

	if (_airspeed_buffer && _airspeed_buffer->pop_first_older_than(imu_delayed.time_us, &_airspeed_sample_delayed)) {

		const airspeedSample &airspeed_sample = _airspeed_sample_delayed;

		updateAirspeed(airspeed_sample, _aid_src_airspeed);

		_innov_check_fail_status.flags.reject_airspeed =
			_aid_src_airspeed.innovation_rejected; // TODO: remove this redundant flag

		const bool continuing_conditions_passing = _control_status.flags.in_air
				&& !_control_status.flags.fake_pos;

		const bool is_airspeed_significant = airspeed_sample.true_airspeed > _params.arsp_thr;
		const bool is_airspeed_consistent = (_aid_src_airspeed.test_ratio > 0.f && _aid_src_airspeed.test_ratio < 1.f);
		const bool starting_conditions_passing = continuing_conditions_passing
				&& is_airspeed_significant
				&& (is_airspeed_consistent || !_control_status.flags.wind || _control_status.flags.inertial_dead_reckoning);

		if (_control_status.flags.fuse_aspd) {
			if (continuing_conditions_passing) {
				if (is_airspeed_significant) {
					fuseAirspeed(airspeed_sample, _aid_src_airspeed);
				}

#if defined(CONFIG_EKF2_GNSS)
				_yawEstimator.setTrueAirspeed(airspeed_sample.true_airspeed);
#endif // CONFIG_EKF2_GNSS

				const bool is_fusion_failing = isTimedOut(_aid_src_airspeed.time_last_fuse, (uint64_t)10e6);

				if (is_fusion_failing) {
					stopAirspeedFusion();
				}

			} else {
				stopAirspeedFusion();
			}

		} else if (starting_conditions_passing) {
			ECL_INFO("starting airspeed fusion");

			if (_control_status.flags.inertial_dead_reckoning && !is_airspeed_consistent) {
				resetVelUsingAirspeed(airspeed_sample);

			} else if (!_external_wind_init
				   && (!_control_status.flags.wind
				       || getWindVelocityVariance().longerThan(sq(_params.initial_wind_uncertainty)))) {
				resetWindUsingAirspeed(airspeed_sample);
				_aid_src_airspeed.time_last_fuse = _time_delayed_us;
			}

			_control_status.flags.wind = true;
			_control_status.flags.fuse_aspd = true;
		}

	} else if (_control_status.flags.fuse_aspd && !isRecent(_airspeed_sample_delayed.time_us, (uint64_t)1e6)) {
		ECL_WARN("Airspeed data stopped");
		stopAirspeedFusion();
	}
}

void Ekf::updateAirspeed(const airspeedSample &airspeed_sample, estimator_aid_source1d_s &aid_src) const
{
	// Variance for true airspeed measurement - (m/sec)^2
	const float R = sq(math::constrain(_params.eas_noise, 0.5f, 5.0f) *
			   math::constrain(airspeed_sample.eas2tas, 0.9f, 10.0f));

	float innov = 0.f;
	float innov_var = 0.f;
	sym::ComputeAirspeedInnovAndInnovVar(_state.vector(), P, airspeed_sample.true_airspeed, R, FLT_EPSILON,
					     &innov, &innov_var);

	updateAidSourceStatus(aid_src,
			      airspeed_sample.time_us,                 // sample timestamp
			      airspeed_sample.true_airspeed,           // observation
			      R,                                       // observation variance
			      innov,                                   // innovation
			      innov_var,                               // innovation variance
			      math::max(_params.tas_innov_gate, 1.f)); // innovation gate
}

void Ekf::fuseAirspeed(const airspeedSample &airspeed_sample, estimator_aid_source1d_s &aid_src)
{
	if (aid_src.innovation_rejected) {
		return;
	}

	// determine if we need the airspeed fusion to correct states other than wind
	const bool update_wind_only = !_control_status.flags.wind_dead_reckoning;

	const float innov_var = aid_src.innovation_variance;

	if (innov_var < aid_src.observation_variance || innov_var < FLT_EPSILON) {
		// Reset the estimator covariance matrix
		// if we are getting aiding from other sources, warn and reset the wind states and covariances only
		const char *action_string = nullptr;

		if (update_wind_only) {
			resetWindUsingAirspeed(airspeed_sample);
			action_string = "wind";

		} else {
			initialiseCovariance();
			_state.wind_vel.setZero();
			action_string = "full";
		}

		ECL_ERR("airspeed badly conditioned - %s covariance reset", action_string);

		_fault_status.flags.bad_airspeed = true;

		return;
	}

	_fault_status.flags.bad_airspeed = false;

	const VectorState H = sym::ComputeAirspeedH(_state.vector(), FLT_EPSILON);
	VectorState K = P * H / aid_src.innovation_variance;

	if (update_wind_only) {
		const Vector2f K_wind = K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0);
		K.setZero();
		K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0) = K_wind;
	}

	measurementUpdate(K, H, aid_src.observation_variance, aid_src.innovation);

	aid_src.fused = true;
	aid_src.time_last_fuse = _time_delayed_us;

	if (!update_wind_only) {
		_time_last_hor_vel_fuse = _time_delayed_us;
	}
}

void Ekf::stopAirspeedFusion()
{
	if (_control_status.flags.fuse_aspd) {
		ECL_INFO("stopping airspeed fusion");
		_control_status.flags.fuse_aspd = false;

#if defined(CONFIG_EKF2_GNSS)
		_yawEstimator.setTrueAirspeed(NAN);
#endif // CONFIG_EKF2_GNSS
	}
}

void Ekf::resetWindUsingAirspeed(const airspeedSample &airspeed_sample)
{
	constexpr float sideslip_var = sq(math::radians(15.0f));

	const float euler_yaw = getEulerYaw(_R_to_earth);
	const float airspeed_var = sq(math::constrain(_params.eas_noise, 0.5f, 5.0f)
				      * math::constrain(airspeed_sample.eas2tas, 0.9f, 10.0f));

	matrix::SquareMatrix<float, State::wind_vel.dof> P_wind;
	sym::ComputeWindInitAndCovFromAirspeed(_state.vel, euler_yaw, airspeed_sample.true_airspeed, getVelocityVariance(),
					       getYawVar(), sideslip_var, airspeed_var, &_state.wind_vel, &P_wind);

	resetStateCovariance<State::wind_vel>(P_wind);

	ECL_INFO("reset wind using airspeed to (%.3f, %.3f)", (double)_state.wind_vel(0), (double)_state.wind_vel(1));

	resetAidSourceStatusZeroInnovation(_aid_src_airspeed);
}

void Ekf::resetVelUsingAirspeed(const airspeedSample &airspeed_sample)
{
	const float euler_yaw = getEulerYaw(_R_to_earth);

	// Estimate velocity using zero sideslip assumption and airspeed measurement
	Vector2f horizontal_velocity;
	horizontal_velocity(0) = _state.wind_vel(0) + airspeed_sample.true_airspeed * cosf(euler_yaw);
	horizontal_velocity(1) = _state.wind_vel(1) + airspeed_sample.true_airspeed * sinf(euler_yaw);

	float vel_var = NAN; // Do not reset the velocity variance as wind variance estimate is most likely not correct
	resetHorizontalVelocityTo(horizontal_velocity, vel_var);

	_aid_src_airspeed.time_last_fuse = _time_delayed_us;
}
/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ekf.h"

#include "aid_sources/aux_global_position/aux_global_position.hpp"

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)

void AuxGlobalPosition::update(Ekf &ekf, const estimator::imuSample &imu_delayed)
{

#if defined(MODULE_NAME)

	if (_aux_global_position_sub.updated()) {

		vehicle_global_position_s aux_global_position{};
		_aux_global_position_sub.copy(&aux_global_position);

		const int64_t time_us = aux_global_position.timestamp_sample - static_cast<int64_t>(_param_ekf2_agp_delay.get() * 1000);

		AuxGlobalPositionSample sample{};
		sample.time_us = time_us;
		sample.latitude = aux_global_position.lat;
		sample.longitude = aux_global_position.lon;
		sample.altitude_amsl = aux_global_position.alt;
		sample.eph = aux_global_position.eph;
		sample.epv = aux_global_position.epv;
		sample.lat_lon_reset_counter = aux_global_position.lat_lon_reset_counter;

		_aux_global_position_buffer.push(sample);

		_time_last_buffer_push = imu_delayed.time_us;
	}

#endif // MODULE_NAME

	AuxGlobalPositionSample sample;

	if (_aux_global_position_buffer.pop_first_older_than(imu_delayed.time_us, &sample)) {

		if (!(_param_ekf2_agp_ctrl.get() & static_cast<int32_t>(Ctrl::HPOS))) {
			return;
		}

		estimator_aid_source2d_s &aid_src = _aid_src_aux_global_position;
		const LatLonAlt position(sample.latitude, sample.longitude, sample.altitude_amsl);
		const Vector2f innovation = (ekf.getLatLonAlt() - position).xy(); // altitude measurements are not used

		// relax the upper observation noise limit which prevents bad measurements perturbing the position estimate
		float pos_noise = math::max(sample.eph, _param_ekf2_agp_noise.get(), 0.01f);
		const float pos_var = sq(pos_noise);
		const Vector2f pos_obs_var(pos_var, pos_var);

		ekf.updateAidSourceStatus(aid_src,
					  sample.time_us,                                      // sample timestamp
					  matrix::Vector2d(sample.latitude, sample.longitude), // observation
					  pos_obs_var,                                         // observation variance
					  innovation,                                          // innovation
					  Vector2f(ekf.getPositionVariance()) + pos_obs_var,   // innovation variance
					  math::max(_param_ekf2_agp_gate.get(), 1.f));         // innovation gate

		const bool starting_conditions = PX4_ISFINITE(sample.latitude) && PX4_ISFINITE(sample.longitude)
						 && ekf.control_status_flags().yaw_align;
		const bool continuing_conditions = starting_conditions
						   && ekf.global_origin_valid();

		switch (_state) {
		case State::stopped:

		/* FALLTHROUGH */
		case State::starting:
			if (starting_conditions) {
				_state = State::starting;

				if (ekf.global_origin_valid()) {
					const bool fused = ekf.fuseHorizontalPosition(aid_src);
					bool reset = false;

					if (!fused && !ekf.isOtherSourceOfHorizontalPositionAidingThan(ekf.control_status_flags().aux_gpos)) {
						ekf.resetHorizontalPositionTo(sample.latitude, sample.longitude, Vector2f(aid_src.observation_variance));
						ekf.resetAidSourceStatusZeroInnovation(aid_src);
						reset = true;
					}

					if (fused || reset) {
						ekf.enableControlStatusAuxGpos();
						_reset_counters.lat_lon = sample.lat_lon_reset_counter;
						_state = State::active;
					}

				} else {
					// Try to initialize using measurement
					if (ekf.resetGlobalPositionTo(sample.latitude, sample.longitude, sample.altitude_amsl, pos_var,
								      sq(sample.epv))) {
						ekf.resetAidSourceStatusZeroInnovation(aid_src);
						ekf.enableControlStatusAuxGpos();
						_reset_counters.lat_lon = sample.lat_lon_reset_counter;
						_state = State::active;
					}
				}
			}

			break;

		case State::active:
			if (continuing_conditions) {
				ekf.fuseHorizontalPosition(aid_src);

				if (isTimedOut(aid_src.time_last_fuse, imu_delayed.time_us, ekf._params.no_aid_timeout_max)
				    || (_reset_counters.lat_lon != sample.lat_lon_reset_counter)) {
					if (ekf.isOnlyActiveSourceOfHorizontalPositionAiding(ekf.control_status_flags().aux_gpos)) {

						ekf.resetHorizontalPositionTo(sample.latitude, sample.longitude, Vector2f(aid_src.observation_variance));

						ekf.resetAidSourceStatusZeroInnovation(aid_src);

						_reset_counters.lat_lon = sample.lat_lon_reset_counter;

					} else {
						ekf.disableControlStatusAuxGpos();
						_state = State::stopped;
					}
				}

			} else {
				ekf.disableControlStatusAuxGpos();
				_state = State::stopped;
			}

			break;

		default:
			break;
		}

#if defined(MODULE_NAME)
		aid_src.timestamp = hrt_absolute_time();
		_estimator_aid_src_aux_global_position_pub.publish(aid_src);

		_test_ratio_filtered = math::max(fabsf(aid_src.test_ratio_filtered[0]), fabsf(aid_src.test_ratio_filtered[1]));
#endif // MODULE_NAME

	} else if ((_state != State::stopped) && isTimedOut(_time_last_buffer_push, imu_delayed.time_us, (uint64_t)5e6)) {
		ekf.disableControlStatusAuxGpos();
		_state = State::stopped;
		ECL_WARN("Aux global position data stopped");
	}
}

#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION
/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ekf.h"

void Ekf::controlAuxVelFusion(const imuSample &imu_sample)
{
	if (_auxvel_buffer) {
		auxVelSample sample;

		if (_auxvel_buffer->pop_first_older_than(imu_sample.time_us, &sample)) {

			updateAidSourceStatus(_aid_src_aux_vel,
					      sample.time_us,                                           // sample timestamp
					      sample.vel,                                               // observation
					      sample.velVar,                                            // observation variance
					      Vector2f(_state.vel.xy()) - sample.vel,                   // innovation
					      Vector2f(getStateVariance<State::vel>()) + sample.velVar, // innovation variance
					      math::max(_params.auxvel_gate, 1.f));                     // innovation gate

			if (isHorizontalAidingActive()) {
				fuseHorizontalVelocity(_aid_src_aux_vel);
			}
		}
	}
}

void Ekf::stopAuxVelFusion()
{
	ECL_INFO("stopping aux vel fusion");
	//_control_status.flags.aux_vel = false;
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file baro_height_control.cpp
 * Control functions for ekf barometric height fusion
 */

#include "ekf.h"

void Ekf::controlBaroHeightFusion(const imuSample &imu_sample)
{
	static constexpr const char *HGT_SRC_NAME = "baro";

	auto &aid_src = _aid_src_baro_hgt;
	HeightBiasEstimator &bias_est = _baro_b_est;

	bias_est.predict(_dt_ekf_avg);

	baroSample baro_sample;

	if (_baro_buffer && _baro_buffer->pop_first_older_than(imu_sample.time_us, &baro_sample)) {

#if defined(CONFIG_EKF2_BARO_COMPENSATION)
		const float measurement = compensateBaroForDynamicPressure(imu_sample, baro_sample.hgt);
#else
		const float measurement = baro_sample.hgt;
#endif

		const float measurement_var = sq(_params.baro_noise);

		const bool measurement_valid = PX4_ISFINITE(measurement) && PX4_ISFINITE(measurement_var);

		if (measurement_valid) {
			if ((_baro_counter == 0) || baro_sample.reset) {
				_baro_lpf.reset(measurement);
				_baro_counter = 1;
				_control_status.flags.baro_fault = false;

			} else {
				_baro_lpf.update(measurement);
				_baro_counter++;
			}

			if (_baro_counter <= _obs_buffer_length) {
				// Initialize the pressure offset (included in the baro bias)
				bias_est.setBias(-_gpos.altitude() + _baro_lpf.getState());
			}
		}

		// vertical position innovation - baro measurement has opposite sign to earth z axis
		updateVerticalPositionAidStatus(aid_src,
						baro_sample.time_us,
						-(measurement - bias_est.getBias()),      // observation
						measurement_var + bias_est.getBiasVar(),  // observation variance
						math::max(_params.baro_innov_gate, 1.f)); // innovation gate

		// Compensate for positive static pressure transients (negative vertical position innovations)
		// caused by rotor wash ground interaction by applying a temporary deadzone to baro innovations.
		if (_control_status.flags.gnd_effect && (_params.gnd_effect_deadzone > 0.f)) {

			const float deadzone_start = 0.0f;
			const float deadzone_end = deadzone_start + _params.gnd_effect_deadzone;

			if (aid_src.innovation < -deadzone_start) {
				if (aid_src.innovation <= -deadzone_end) {
					aid_src.innovation += deadzone_end;

				} else {
					aid_src.innovation = -deadzone_start;
				}
			}
		}

		// update the bias estimator before updating the main filter but after
		// using its current state to compute the vertical position innovation
		if (measurement_valid) {
			bias_est.setMaxStateNoise(sqrtf(measurement_var));
			bias_est.setProcessNoiseSpectralDensity(_params.baro_bias_nsd);
			bias_est.fuseBias(measurement - _gpos.altitude(), measurement_var + P(State::pos.idx + 2, State::pos.idx + 2));
		}

		// determine if we should use height aiding
		const bool continuing_conditions_passing = (_params.baro_ctrl == 1)
				&& measurement_valid
				&& (_baro_counter > _obs_buffer_length)
				&& !_control_status.flags.baro_fault;

		const bool starting_conditions_passing = continuing_conditions_passing
				&& isNewestSampleRecent(_time_last_baro_buffer_push, 2 * BARO_MAX_INTERVAL);

		if (_control_status.flags.baro_hgt) {

			if (continuing_conditions_passing) {

				fuseVerticalPosition(aid_src);

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired() && (_height_sensor_ref == HeightSensor::BARO)) {
					// All height sources are failing
					ECL_WARN("%s height fusion reset required, all height sources failing", HGT_SRC_NAME);

					_information_events.flags.reset_hgt_to_baro = true;
					resetAltitudeTo(_baro_lpf.getState() - bias_est.getBias(), measurement_var);
					bias_est.setBias(-_gpos.altitude() + _baro_lpf.getState());
					resetAidSourceStatusZeroInnovation(aid_src);

					// reset vertical velocity if no valid sources available
					if (!isVerticalVelocityAidingActive()) {
						resetVerticalVelocityToZero();
					}

					aid_src.time_last_fuse = imu_sample.time_us;

				} else if (is_fusion_failing) {
					ECL_WARN("stopping %s height fusion, fusion failing", HGT_SRC_NAME);
					stopBaroHgtFusion();

					if (isRecent(_time_last_hgt_fuse, _params.hgt_fusion_timeout_max)) {
						// Some other height source is still working
						_control_status.flags.baro_fault = true;
					}
				}

			} else {
				ECL_WARN("stopping %s height fusion, continuing conditions failing", HGT_SRC_NAME);
				stopBaroHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				if (_params.height_sensor_ref == static_cast<int32_t>(HeightSensor::BARO)) {
					ECL_INFO("starting %s height fusion, resetting height", HGT_SRC_NAME);
					_height_sensor_ref = HeightSensor::BARO;

					_information_events.flags.reset_hgt_to_baro = true;
					initialiseAltitudeTo(measurement, measurement_var);
					bias_est.reset();
					resetAidSourceStatusZeroInnovation(aid_src);

				} else {
					ECL_INFO("starting %s height fusion", HGT_SRC_NAME);
					bias_est.setBias(-_gpos.altitude() + _baro_lpf.getState());
				}

				aid_src.time_last_fuse = imu_sample.time_us;
				bias_est.setFusionActive();
				_control_status.flags.baro_hgt = true;
			}
		}

	} else if (_control_status.flags.baro_hgt
		   && !isNewestSampleRecent(_time_last_baro_buffer_push, 2 * BARO_MAX_INTERVAL)) {
		// No data anymore. Stop until it comes back.
		ECL_WARN("stopping %s height fusion, no data", HGT_SRC_NAME);
		stopBaroHgtFusion();
	}
}

void Ekf::stopBaroHgtFusion()
{
	if (_control_status.flags.baro_hgt) {

		if (_height_sensor_ref == HeightSensor::BARO) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_baro_b_est.setFusionInactive();

		_control_status.flags.baro_hgt = false;
	}
}

#if defined(CONFIG_EKF2_BARO_COMPENSATION)
float Ekf::compensateBaroForDynamicPressure(const imuSample &imu_sample, const float baro_alt_uncompensated) const
{
	if (_control_status.flags.wind && isLocalHorizontalPositionValid()) {
		// calculate static pressure error = Pmeas - Ptruth
		// model position error sensitivity as a body fixed ellipse with a different scale in the positive and
		// negative X and Y directions. Used to correct baro data for positional errors

		// Calculate airspeed in body frame
		const Vector3f angular_velocity = (imu_sample.delta_ang / imu_sample.delta_ang_dt) - _state.gyro_bias;
		const Vector3f vel_imu_rel_body_ned = _R_to_earth * (angular_velocity % _params.imu_pos_body);
		const Vector3f velocity_earth = _state.vel - vel_imu_rel_body_ned;

		const Vector3f wind_velocity_earth(_state.wind_vel(0), _state.wind_vel(1), 0.0f);

		const Vector3f airspeed_earth = velocity_earth - wind_velocity_earth;

		const Vector3f airspeed_body = _state.quat_nominal.rotateVectorInverse(airspeed_earth);

		const Vector3f K_pstatic_coef(
			airspeed_body(0) >= 0.f ? _params.static_pressure_coef_xp : _params.static_pressure_coef_xn,
			airspeed_body(1) >= 0.f ? _params.static_pressure_coef_yp : _params.static_pressure_coef_yn,
			_params.static_pressure_coef_z);

		const Vector3f airspeed_squared = matrix::min(airspeed_body.emult(airspeed_body), sq(_params.max_correction_airspeed));

		const float pstatic_err = 0.5f * _air_density * (airspeed_squared.dot(K_pstatic_coef));

		// correct baro measurement using pressure error estimate and assuming sea level gravity
		return baro_alt_uncompensated + pstatic_err / (_air_density * CONSTANTS_ONE_G);
	}

	// otherwise return the uncorrected baro measurement
	return baro_alt_uncompensated;
}
#endif // CONFIG_EKF2_BARO_COMPENSATION
/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file drag_fusion.cpp
 * Body frame drag fusion methods used for multi-rotor wind estimation.
 */

#include "ekf.h"
#include <ekf_derivation/generated/compute_drag_x_innov_var_and_h.h>
#include <ekf_derivation/generated/compute_drag_y_innov_var_and_h.h>

#include <mathlib/mathlib.h>
#include <lib/atmosphere/atmosphere.h>

void Ekf::controlDragFusion(const imuSample &imu_delayed)
{
	if ((_params.drag_ctrl > 0) && _drag_buffer) {

		if (!_control_status.flags.wind && !_control_status.flags.fake_pos && _control_status.flags.in_air) {
			_control_status.flags.wind = true;

			if (!_external_wind_init) {
				resetWindCov();
			}
		}

		dragSample drag_sample;

		if (_drag_buffer->pop_first_older_than(imu_delayed.time_us, &drag_sample)) {
			fuseDrag(drag_sample);
		}
	}
}

void Ekf::fuseDrag(const dragSample &drag_sample)
{
	const float R_ACC = fmaxf(_params.drag_noise, 0.5f); // observation noise variance in specific force drag (m/sec**2)**2
	const float rho = fmaxf(_air_density, 0.1f); // air density (kg/m**3)

	// correct rotor momentum drag for increase in required rotor mass flow with altitude
	// obtained from momentum disc theory
	const float mcoef_corrrected = fmaxf(_params.mcoef * sqrtf(rho / atmosphere::kAirDensitySeaLevelStandardAtmos), 0.f);

	// drag model parameters
	const bool using_bcoef_x = _params.bcoef_x > 1.0f;
	const bool using_bcoef_y = _params.bcoef_y > 1.0f;
	const bool using_mcoef   = _params.mcoef   > 0.001f;

	if (!using_bcoef_x && !using_bcoef_y && !using_mcoef) {
		return;
	}

	// calculate relative wind velocity in earth frame and rotate into body frame
	const Vector3f rel_wind_earth(_state.vel(0) - _state.wind_vel(0),
				      _state.vel(1) - _state.wind_vel(1),
				      _state.vel(2));
	const Vector3f rel_wind_body = _state.quat_nominal.rotateVectorInverse(rel_wind_earth);
	const float rel_wind_speed = rel_wind_body.norm();
	const auto state_vector_prev = _state.vector();

	Vector2f bcoef_inv{0.f, 0.f};

	if (using_bcoef_x) {
		bcoef_inv(0) = 1.f / _params.bcoef_x;
	}

	if (using_bcoef_y) {
		bcoef_inv(1) = 1.f / _params.bcoef_y;
	}

	if (using_bcoef_x && using_bcoef_y) {

		// Interpolate between the X and Y bluff body drag coefficients using current relative velocity
		// This creates an elliptic drag distribution around the XY plane
		bcoef_inv(0) = Vector2f(bcoef_inv.emult(rel_wind_body.xy()) / rel_wind_body.xy().norm()).norm();
		bcoef_inv(1) = bcoef_inv(0);
	}

	Vector2f observation{};
	Vector2f observation_variance{R_ACC, R_ACC};
	Vector2f innovation{};
	Vector2f innovation_variance{};

	// Apply an innovation consistency check with a 5 Sigma threshold
	const float innov_gate = 5.f;

	VectorState H;

	// perform sequential fusion of XY specific forces
	for (uint8_t axis_index = 0; axis_index < 2; axis_index++) {
		// measured drag acceleration corrected for sensor bias
		const float mea_acc = drag_sample.accelXY(axis_index) - _state.accel_bias(axis_index);

		// Drag is modelled as an arbitrary combination of bluff body drag that proportional to
		// equivalent airspeed squared, and rotor momentum drag that is proportional to true airspeed
		// parallel to the rotor disc and mass flow through the rotor disc.
		const float pred_acc = -0.5f * bcoef_inv(axis_index) * rho * rel_wind_body(axis_index) * rel_wind_speed
				       - rel_wind_body(axis_index) * mcoef_corrrected;

		observation(axis_index) = mea_acc;
		innovation(axis_index) = pred_acc - mea_acc;

		if (axis_index == 0) {
			sym::ComputeDragXInnovVarAndH(state_vector_prev, P, rho, bcoef_inv(axis_index), mcoef_corrrected, R_ACC, FLT_EPSILON,
						      &innovation_variance(axis_index), &H);

			if (!using_bcoef_x && !using_mcoef) {
				continue;
			}

		} else if (axis_index == 1) {
			sym::ComputeDragYInnovVarAndH(state_vector_prev, P, rho, bcoef_inv(axis_index), mcoef_corrrected, R_ACC, FLT_EPSILON,
						      &innovation_variance(axis_index), &H);

			if (!using_bcoef_y && !using_mcoef) {
				continue;
			}
		}

		if (innovation_variance(axis_index) < R_ACC) {
			// calculation is badly conditioned
			return;
		}

		const float test_ratio = sq(innovation(axis_index)) / (sq(innov_gate) * innovation_variance(axis_index));

		if (_control_status.flags.in_air && _control_status.flags.wind && !_control_status.flags.fake_pos
		    && PX4_ISFINITE(innovation_variance(axis_index)) && PX4_ISFINITE(innovation(axis_index))
		    && (test_ratio < 1.f)
		   ) {

			VectorState K = P * H / innovation_variance(axis_index);

			measurementUpdate(K, H, R_ACC, innovation(axis_index));
		}
	}

	updateAidSourceStatus(_aid_src_drag,
			      drag_sample.time_us,  // sample timestamp
			      observation,          // observation
			      observation_variance, // observation variance
			      innovation,           // innovation
			      innovation_variance,  // innovation variance
			      innov_gate);          // innovation gate

	_aid_src_drag.fused = true;
	_aid_src_drag.time_last_fuse = _time_delayed_us;
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ev_control.cpp
 * Control functions for ekf external vision control
 */

#include "ekf.h"
#include "aid_sources/external_vision/ev_vel.h"

void Ekf::controlExternalVisionFusion(const imuSample &imu_sample)
{
	_ev_pos_b_est.predict(_dt_ekf_avg);
	_ev_hgt_b_est.predict(_dt_ekf_avg);

	// Check for new external vision data
	extVisionSample ev_sample;

	if (_ext_vision_buffer && _ext_vision_buffer->pop_first_older_than(imu_sample.time_us, &ev_sample)) {

		bool ev_reset = (ev_sample.reset_counter != _ev_sample_prev.reset_counter);

		// determine if we should use the horizontal position observations
		bool quality_sufficient = (_params.ev_quality_minimum <= 0) || (ev_sample.quality >= _params.ev_quality_minimum);

		const bool starting_conditions_passing = quality_sufficient
				&& ((ev_sample.time_us - _ev_sample_prev.time_us) < EV_MAX_INTERVAL)
				&& ((_params.ev_quality_minimum <= 0)
				    || (_ev_sample_prev.quality >= _params.ev_quality_minimum)) // previous quality sufficient
				&& ((_params.ev_quality_minimum <= 0)
				    || (_ext_vision_buffer->get_newest().quality >= _params.ev_quality_minimum)) // newest quality sufficient
				&& isNewestSampleRecent(_time_last_ext_vision_buffer_push, EV_MAX_INTERVAL);

		updateEvAttitudeErrorFilter(ev_sample, ev_reset);

		controlEvYawFusion(imu_sample, ev_sample, starting_conditions_passing, ev_reset, quality_sufficient, _aid_src_ev_yaw);

		switch (ev_sample.vel_frame) {
		case VelocityFrame::BODY_FRAME_FRD: {
				EvVelBodyFrameFrd ev_vel_body(*this, ev_sample, _params.ev_vel_noise, imu_sample);
				controlEvVelFusion(ev_vel_body, starting_conditions_passing, ev_reset, quality_sufficient, _aid_src_ev_vel);
				break;
			}

		case VelocityFrame::LOCAL_FRAME_NED: {
				EvVelLocalFrameNed ev_vel_ned(*this, ev_sample, _params.ev_vel_noise, imu_sample);
				controlEvVelFusion(ev_vel_ned, starting_conditions_passing, ev_reset, quality_sufficient, _aid_src_ev_vel);
				break;
			}

		case VelocityFrame::LOCAL_FRAME_FRD: {
				EvVelLocalFrameFrd ev_vel_frd(*this, ev_sample, _params.ev_vel_noise, imu_sample);
				controlEvVelFusion(ev_vel_frd, starting_conditions_passing, ev_reset, quality_sufficient, _aid_src_ev_vel);
				break;
			}

		default:
			return;
		}

		controlEvPosFusion(imu_sample, ev_sample, starting_conditions_passing, ev_reset, quality_sufficient, _aid_src_ev_pos);
		controlEvHeightFusion(imu_sample, ev_sample, starting_conditions_passing, ev_reset, quality_sufficient,
				      _aid_src_ev_hgt);

		if (quality_sufficient) {
			_ev_sample_prev = ev_sample;
		}

	} else if ((_control_status.flags.ev_pos || _control_status.flags.ev_vel || _control_status.flags.ev_yaw
		    || _control_status.flags.ev_hgt)
		   && isTimedOut(_ev_sample_prev.time_us, 2 * EV_MAX_INTERVAL)) {

		// Turn off EV fusion mode if no data has been received
		stopEvPosFusion();
		stopEvVelFusion();
		stopEvYawFusion();
		stopEvHgtFusion();

		_ev_q_error_initialized = false;

		ECL_WARN("vision data stopped");
	}
}

void Ekf::updateEvAttitudeErrorFilter(extVisionSample &ev_sample, bool ev_reset)
{
	const Quatf q_error((_state.quat_nominal * ev_sample.quat.inversed()).normalized());

	if (!q_error.isAllFinite()) {
		return;
	}

	if (!_ev_q_error_initialized || ev_reset) {
		_ev_q_error_filt.reset(q_error);
		_ev_q_error_initialized = true;

	} else {
		_ev_q_error_filt.update(q_error);
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ev_height_control.cpp
 * Control functions for ekf external vision height fusion
 */

#include "ekf.h"

void Ekf::controlEvHeightFusion(const imuSample &imu_sample, const extVisionSample &ev_sample,
				const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient,
				estimator_aid_source1d_s &aid_src)
{
	static constexpr const char *AID_SRC_NAME = "EV height";

	HeightBiasEstimator &bias_est = _ev_hgt_b_est;

	// bias_est.predict(_dt_ekf_avg) called by controlExternalVisionFusion()

	// correct position for offset relative to IMU
	const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

	// rotate measurement into correct earth frame if required
	Vector3f pos{ev_sample.pos};
	Matrix3f pos_cov{matrix::diag(ev_sample.position_var)};

	// rotate EV to the EKF reference frame unless we're operating entirely in vision frame
	if (!(_control_status.flags.ev_yaw && _control_status.flags.ev_pos)) {

		const Quatf q_error(_ev_q_error_filt.getState());

		if (q_error.isAllFinite()) {
			const Dcmf R_ev_to_ekf(q_error);

			pos = R_ev_to_ekf * ev_sample.pos;
			pos_cov = R_ev_to_ekf * matrix::diag(ev_sample.position_var) * R_ev_to_ekf.transpose();

			// increase minimum variance to include EV orientation variance
			// TODO: do this properly
			const float orientation_var_max = math::max(ev_sample.orientation_var(0), ev_sample.orientation_var(1));
			pos_cov(2, 2) = math::max(pos_cov(2, 2), orientation_var_max);
		}
	}

	const float measurement = pos(2) - pos_offset_earth(2);
	float measurement_var = math::max(pos_cov(2, 2), sq(_params.ev_pos_noise), sq(0.01f));

#if defined(CONFIG_EKF2_GNSS)

	// increase minimum variance if GPS active
	if (_control_status.flags.gps_hgt) {
		measurement_var = math::max(measurement_var, sq(_params.gps_pos_noise));
	}

#endif // CONFIG_EKF2_GNSS

	const bool measurement_valid = PX4_ISFINITE(measurement) && PX4_ISFINITE(measurement_var);

	updateVerticalPositionAidStatus(aid_src,
					ev_sample.time_us,
					measurement - bias_est.getBias(),           // observation
					measurement_var + bias_est.getBiasVar(),    // observation variance
					math::max(_params.ev_pos_innov_gate, 1.f)); // innovation gate

	// update the bias estimator before updating the main filter but after
	// using its current state to compute the vertical position innovation
	if (measurement_valid && quality_sufficient) {
		bias_est.setMaxStateNoise(sqrtf(measurement_var));
		bias_est.setProcessNoiseSpectralDensity(_params.ev_hgt_bias_nsd);
		bias_est.fuseBias(measurement + _gpos.altitude(), measurement_var + P(State::pos.idx + 2, State::pos.idx + 2));
	}

	const bool continuing_conditions_passing = (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::VPOS))
			&& measurement_valid;

	const bool starting_conditions_passing = common_starting_conditions_passing
			&& continuing_conditions_passing;

	if (_control_status.flags.ev_hgt) {
		if (continuing_conditions_passing) {
			if (ev_reset) {

				if (quality_sufficient) {
					ECL_INFO("reset to %s", AID_SRC_NAME);

					if (_height_sensor_ref == HeightSensor::EV) {
						_information_events.flags.reset_hgt_to_ev = true;
						resetAltitudeTo(-measurement, measurement_var);
						bias_est.reset();

					} else {
						bias_est.setBias(_gpos.altitude() + measurement);
					}

					aid_src.time_last_fuse = _time_delayed_us;

				} else {
					// EV has reset, but quality isn't sufficient
					// we have no choice but to stop EV and try to resume once quality is acceptable
					stopEvHgtFusion();
					return;
				}

			} else if (quality_sufficient) {
				fuseVerticalPosition(aid_src);

			} else {
				aid_src.innovation_rejected = true;
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

			if (isHeightResetRequired() && quality_sufficient && (_height_sensor_ref == HeightSensor::EV)) {
				// All height sources are failing
				ECL_WARN("%s fusion reset required, all height sources failing", AID_SRC_NAME);
				_information_events.flags.reset_hgt_to_ev = true;
				resetAltitudeTo(-measurement - bias_est.getBias(), measurement_var);
				bias_est.setBias(_gpos.altitude() + measurement);

				aid_src.time_last_fuse = _time_delayed_us;

			} else if (is_fusion_failing) {
				// A reset did not fix the issue but all the starting checks are not passing
				// This could be a temporary issue, stop the fusion without declaring the sensor faulty
				ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);
				stopEvHgtFusion();
			}

		} else {
			// Stop fusion but do not declare it faulty
			ECL_WARN("stopping %s fusion, continuing conditions failing", AID_SRC_NAME);
			stopEvHgtFusion();
		}

	} else {
		if (starting_conditions_passing) {
			// activate fusion, only reset if necessary
			if (_params.height_sensor_ref == static_cast<int32_t>(HeightSensor::EV)) {
				ECL_INFO("starting %s fusion, resetting state", AID_SRC_NAME);
				_information_events.flags.reset_hgt_to_ev = true;
				resetAltitudeTo(-measurement, measurement_var);

				_height_sensor_ref = HeightSensor::EV;
				bias_est.reset();

			} else {
				ECL_INFO("starting %s fusion", AID_SRC_NAME);
				bias_est.setBias(_gpos.altitude() + measurement);
			}

			aid_src.time_last_fuse = _time_delayed_us;
			bias_est.setFusionActive();
			_control_status.flags.ev_hgt = true;
		}
	}
}

void Ekf::stopEvHgtFusion()
{
	if (_control_status.flags.ev_hgt) {

		if (_height_sensor_ref == HeightSensor::EV) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_ev_hgt_b_est.setFusionInactive();

		_control_status.flags.ev_hgt = false;
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ev_pos_control.cpp
 * Control functions for ekf external vision position fusion
 */

#include "ekf.h"

static constexpr const char *EV_AID_SRC_NAME = "EV position";


void Ekf::controlEvPosFusion(const imuSample &imu_sample, const extVisionSample &ev_sample,
			     const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient,
			     estimator_aid_source2d_s &aid_src)
{
	const bool yaw_alignment_changed = (!_control_status_prev.flags.ev_yaw && _control_status.flags.ev_yaw)
					   || (_control_status_prev.flags.yaw_align != _control_status.flags.yaw_align);

	// determine if we should use EV position aiding
	bool continuing_conditions_passing = (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::HPOS))
					     && _control_status.flags.tilt_align
					     && PX4_ISFINITE(ev_sample.pos(0))
					     && PX4_ISFINITE(ev_sample.pos(1));

	// correct position for offset relative to IMU
	const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

	const bool bias_fusion_was_active = _ev_pos_b_est.fusionActive();

	// rotate measurement into correct earth frame if required
	Vector3f pos{NAN, NAN, NAN};
	Matrix3f pos_cov{};

	switch (ev_sample.pos_frame) {
	case PositionFrame::LOCAL_FRAME_NED:
		if (_control_status.flags.yaw_align) {
			pos = ev_sample.pos - pos_offset_earth;
			pos_cov = matrix::diag(ev_sample.position_var);

			if (_control_status.flags.gnss_pos) {
				_ev_pos_b_est.setFusionActive();

			} else {
				_ev_pos_b_est.setFusionInactive();
			}

		} else {
			continuing_conditions_passing = false;
			_ev_pos_b_est.setFusionInactive();
			_ev_pos_b_est.reset();
		}

		break;

	case PositionFrame::LOCAL_FRAME_FRD:
		if (_control_status.flags.ev_yaw) {
			// using EV frame
			pos = ev_sample.pos - pos_offset_earth;
			pos_cov = matrix::diag(ev_sample.position_var);

			_ev_pos_b_est.setFusionInactive();
			_ev_pos_b_est.reset();

		} else {
			// rotate EV to the EKF reference frame
			const Dcmf R_ev_to_ekf = Dcmf(_ev_q_error_filt.getState());

			pos = R_ev_to_ekf * ev_sample.pos - pos_offset_earth;
			pos_cov = R_ev_to_ekf * matrix::diag(ev_sample.position_var) * R_ev_to_ekf.transpose();

			// increase minimum variance to include EV orientation variance
			// TODO: do this properly
			const float orientation_var_max = ev_sample.orientation_var.max();

			for (int i = 0; i < 2; i++) {
				pos_cov(i, i) = math::max(pos_cov(i, i), orientation_var_max);
			}

			if (_control_status.flags.gnss_pos) {
				_ev_pos_b_est.setFusionActive();

			} else {
				_ev_pos_b_est.setFusionInactive();
			}
		}

		break;

	default:
		continuing_conditions_passing = false;
		_ev_pos_b_est.setFusionInactive();
		_ev_pos_b_est.reset();
		break;
	}

#if defined(CONFIG_EKF2_GNSS)

	// increase minimum variance if GNSS is active (position reference)
	if (_control_status.flags.gnss_pos) {
		for (int i = 0; i < 2; i++) {
			pos_cov(i, i) = math::max(pos_cov(i, i), sq(_params.gps_pos_noise));
		}
	}

#endif // CONFIG_EKF2_GNSS

	const Vector2f position_estimate = getLocalHorizontalPosition();

	const Vector2f measurement{pos(0), pos(1)};

	const Vector2f measurement_var{
		math::max(pos_cov(0, 0), sq(_params.ev_pos_noise), sq(0.01f)),
		math::max(pos_cov(1, 1), sq(_params.ev_pos_noise), sq(0.01f))
	};

	const bool measurement_valid = measurement.isAllFinite() && measurement_var.isAllFinite();

	// bias fusion activated (GNSS position activated)
	if (!bias_fusion_was_active && _ev_pos_b_est.fusionActive()) {
		if (quality_sufficient) {
			// reset the bias estimator
			_ev_pos_b_est.setBias(-position_estimate + measurement);

		} else if (isOtherSourceOfHorizontalAidingThan(_control_status.flags.ev_pos)) {
			// otherwise stop EV position, when quality is good again it will restart with reset bias
			stopEvPosFusion();
		}
	}

	const Vector2f position = measurement - _ev_pos_b_est.getBias();
	const Vector2f pos_obs_var = measurement_var + _ev_pos_b_est.getBiasVar();

	updateAidSourceStatus(aid_src,
			      ev_sample.time_us,                                      // sample timestamp
			      position,                                               // observation
			      pos_obs_var,                                            // observation variance
			      position_estimate - position,                           // innovation
			      Vector2f(getStateVariance<State::pos>()) + pos_obs_var, // innovation variance
			      math::max(_params.ev_pos_innov_gate, 1.f));             // innovation gate

	// update the bias estimator before updating the main filter but after
	// using its current state to compute the vertical position innovation
	if (measurement_valid && quality_sufficient) {
		_ev_pos_b_est.setMaxStateNoise(Vector2f(sqrtf(measurement_var(0)), sqrtf(measurement_var(1))));
		_ev_pos_b_est.setProcessNoiseSpectralDensity(_params.ev_hgt_bias_nsd); // TODO
		_ev_pos_b_est.fuseBias(measurement - position_estimate,
				       measurement_var + Vector2f(getStateVariance<State::pos>()));
	}

	if (!measurement_valid) {
		continuing_conditions_passing = false;
	}

	const bool starting_conditions_passing = common_starting_conditions_passing
			&& continuing_conditions_passing;

	if (_control_status.flags.ev_pos) {

		if (continuing_conditions_passing) {
			const bool bias_estimator_change = (bias_fusion_was_active != _ev_pos_b_est.fusionActive());
			const bool reset = ev_reset || yaw_alignment_changed || bias_estimator_change;

			updateEvPosFusion(measurement, measurement_var, quality_sufficient, reset, aid_src);

		} else {
			// Stop fusion but do not declare it faulty
			ECL_WARN("stopping %s fusion, continuing conditions failing", EV_AID_SRC_NAME);
			stopEvPosFusion();
		}

	} else {
		if (starting_conditions_passing) {
			startEvPosFusion(measurement, measurement_var, aid_src);
		}
	}
}

void Ekf::startEvPosFusion(const Vector2f &measurement, const Vector2f &measurement_var,
			   estimator_aid_source2d_s &aid_src)
{
	// activate fusion
	// TODO:  (_params.position_sensor_ref == PositionSensor::EV)
	if (_control_status.flags.gnss_pos) {
		ECL_INFO("starting %s fusion", EV_AID_SRC_NAME);
		_ev_pos_b_est.setBias(-getLocalHorizontalPosition() + measurement);
		_ev_pos_b_est.setFusionActive();

	} else {
		ECL_INFO("starting %s fusion, resetting state", EV_AID_SRC_NAME);
		//_position_sensor_ref = PositionSensor::EV;
		_information_events.flags.reset_pos_to_vision = true;
		resetHorizontalPositionTo(measurement, measurement_var);
		_ev_pos_b_est.reset();
	}

	aid_src.time_last_fuse = _time_delayed_us;

	_nb_ev_pos_reset_available = 5;
	_information_events.flags.starting_vision_pos_fusion = true;
	_control_status.flags.ev_pos = true;
}

void Ekf::updateEvPosFusion(const Vector2f &measurement, const Vector2f &measurement_var, bool quality_sufficient,
			    bool reset, estimator_aid_source2d_s &aid_src)
{
	if (reset) {

		if (quality_sufficient) {

			if (!_control_status.flags.gnss_pos) {
				ECL_INFO("reset to %s", EV_AID_SRC_NAME);
				_information_events.flags.reset_pos_to_vision = true;
				resetHorizontalPositionTo(measurement, measurement_var);
				_ev_pos_b_est.reset();

			} else {
				_ev_pos_b_est.setBias(-getLocalHorizontalPosition() + measurement);
			}

			aid_src.time_last_fuse = _time_delayed_us;

		} else {
			// EV has reset, but quality isn't sufficient
			// we have no choice but to stop EV and try to resume once quality is acceptable
			stopEvPosFusion();
			return;
		}

	} else if (quality_sufficient) {
		fuseHorizontalPosition(aid_src);

	} else {
		aid_src.innovation_rejected = true;
	}

	const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.no_aid_timeout_max); // 1 second

	if (is_fusion_failing) {
		bool pos_xy_fusion_failing = isTimedOut(_time_last_hor_pos_fuse, _params.no_aid_timeout_max);

		if ((_nb_ev_pos_reset_available > 0) && quality_sufficient) {
			// Data seems good, attempt a reset
			ECL_WARN("%s fusion failing, resetting", EV_AID_SRC_NAME);

			if (_control_status.flags.gnss_pos && !pos_xy_fusion_failing) {
				// reset EV position bias
				_ev_pos_b_est.setBias(-Vector2f(getLocalHorizontalPosition()) + measurement);

			} else {
				_information_events.flags.reset_pos_to_vision = true;

				if (_control_status.flags.gnss_pos) {
					resetHorizontalPositionTo(measurement - _ev_pos_b_est.getBias(), measurement_var + _ev_pos_b_est.getBiasVar());
					_ev_pos_b_est.setBias(-getLocalHorizontalPosition() + measurement);

				} else {
					resetHorizontalPositionTo(measurement, measurement_var);
					_ev_pos_b_est.reset();
				}
			}

			aid_src.time_last_fuse = _time_delayed_us;

			if (_control_status.flags.in_air) {
				_nb_ev_pos_reset_available--;
			}

		} else {
			// A reset did not fix the issue but all the starting checks are not passing
			// This could be a temporary issue, stop the fusion without declaring the sensor faulty
			ECL_WARN("stopping %s, fusion failing", EV_AID_SRC_NAME);
			stopEvPosFusion();
		}
	}
}

void Ekf::stopEvPosFusion()
{
	if (_control_status.flags.ev_pos) {
		_control_status.flags.ev_pos = false;
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ev_vel_control.cpp
 * Control functions for ekf external vision velocity fusion
 */

#include "ekf.h"
#include "aid_sources/external_vision/ev_vel.h"
#include "ekf_derivation/generated/compute_body_vel_innov_var_h.h"
#include "ekf_derivation/generated/compute_body_vel_y_innov_var.h"
#include "ekf_derivation/generated/compute_body_vel_z_innov_var.h"

void Ekf::controlEvVelFusion(ExternalVisionVel &ev, const bool common_starting_conditions_passing, const bool ev_reset,
			     const bool quality_sufficient, estimator_aid_source3d_s &aid_src)
{
	static constexpr const char *AID_SRC_NAME = "EV velocity";

	const bool yaw_alignment_changed = (!_control_status_prev.flags.ev_yaw && _control_status.flags.ev_yaw)
					   || (_control_status_prev.flags.yaw_align != _control_status.flags.yaw_align);

	// determine if we should use EV velocity aiding
	bool continuing_conditions_passing = (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::VEL))
					     && _control_status.flags.tilt_align
					     && ev._sample.vel.isAllFinite()
					     && !ev._sample.vel.longerThan(_params.velocity_limit);


	continuing_conditions_passing &= ev._measurement.isAllFinite() && ev._measurement_var.isAllFinite();

	float gate = math::max(_params.ev_vel_innov_gate, 1.f);

	if (_control_status.flags.ev_vel) {
		if (continuing_conditions_passing) {
			if ((ev_reset && isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.ev_vel)) || yaw_alignment_changed) {
				if (quality_sufficient) {
					ECL_INFO("reset to %s", AID_SRC_NAME);
					_information_events.flags.reset_vel_to_vision = true;
					ev.resetVelocity();
					resetAidSourceStatusZeroInnovation(aid_src);

				} else {
					// EV has reset, but quality isn't sufficient
					// we have no choice but to stop EV and try to resume once quality is acceptable
					stopEvVelFusion();
					return;
				}

			} else if (quality_sufficient) {
				ev.fuseVelocity(aid_src, gate);

			} else {
				aid_src.innovation_rejected = true;
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.no_aid_timeout_max); // 1 second

			if (is_fusion_failing) {

				if ((_nb_ev_vel_reset_available > 0) && quality_sufficient) {
					_information_events.flags.reset_vel_to_vision = true;
					ECL_WARN("%s fusion failing, resetting", AID_SRC_NAME);
					ev.resetVelocity();
					resetAidSourceStatusZeroInnovation(aid_src);

					if (_control_status.flags.in_air) {
						_nb_ev_vel_reset_available--;
					}

				} else {
					ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);


					stopEvVelFusion();
				}

			} else if (isHeightResetRequired()) {
				ev.resetVerticalVelocity();
			}

		} else {
			// Stop fusion but do not declare it faulty
			ECL_WARN("stopping %s fusion, continuing conditions failing", AID_SRC_NAME);
			stopEvVelFusion();
		}

	} else {

		if (common_starting_conditions_passing && continuing_conditions_passing) {
			// make starting condition more sensitive if horizontal aiding is active
			gate = isHorizontalAidingActive() ? (float)sqrt(10.f) * gate : gate;

			// activate fusion, only reset if necessary
			if (!isHorizontalAidingActive() || yaw_alignment_changed) {
				ECL_INFO("starting %s fusion, resetting velocity to (%.3f, %.3f, %.3f)", AID_SRC_NAME,
					 (double)ev._measurement(0), (double)ev._measurement(1),
					 (double)ev._measurement(2));
				_information_events.flags.reset_vel_to_vision = true;
				ev.resetVelocity();
				resetAidSourceStatusZeroInnovation(aid_src);

				_control_status.flags.ev_vel = true;

			} else if (ev.fuseVelocity(aid_src, gate)) {
				ECL_INFO("starting %s fusion", AID_SRC_NAME);
				_control_status.flags.ev_vel = true;
			}

			if (_control_status.flags.ev_vel) {
				_nb_ev_vel_reset_available = 5;
				_information_events.flags.starting_vision_vel_fusion = true;
			}
		}
	}
}

void Ekf::stopEvVelFusion()
{
	if (_control_status.flags.ev_vel) {
		_control_status.flags.ev_vel = false;
	}
}

void Ekf::fuseLocalFrameVelocity(estimator_aid_source3d_s &aid_src, const uint64_t &timestamp,
				 const Vector3f &measurement, const Vector3f &measurement_var, const float &innovation_gate)
{
	updateAidSourceStatus(aid_src,
			      timestamp,				// sample timestamp
			      measurement,				// observation
			      measurement_var,				// observation variance
			      _state.vel - measurement,			// innovation
			      getVelocityVariance() + measurement_var,	// innovation variance
			      innovation_gate);				// innovation gate
	fuseVelocity(aid_src);
}

void Ekf::fuseBodyFrameVelocity(estimator_aid_source3d_s &aid_src, const uint64_t &timestamp,
				const Vector3f &measurement, const Vector3f &measurement_var, const float &innovation_gate)
{
	VectorState H[3];
	Vector3f innov_var;
	Vector3f innov = _R_to_earth.transpose() * _state.vel - measurement;
	const auto state_vector = _state.vector();
	sym::ComputeBodyVelInnovVarH(state_vector, P, measurement_var, &innov_var, &H[0], &H[1], &H[2]);

	updateAidSourceStatus(aid_src,
			      timestamp,				// sample timestamp
			      measurement,				// observation
			      measurement_var,				// observation variance
			      innov,					// innovation
			      innov_var,				// innovation variance
			      innovation_gate);				// innovation gate

	if (!aid_src.innovation_rejected) {
		for (uint8_t index = 0; index <= 2; index++) {
			if (index == 1) {
				sym::ComputeBodyVelYInnovVar(state_vector, P, measurement_var(index), &aid_src.innovation_variance[index]);

			} else if (index == 2) {
				sym::ComputeBodyVelZInnovVar(state_vector, P, measurement_var(index), &aid_src.innovation_variance[index]);
			}

			aid_src.innovation[index] = Vector3f(_R_to_earth.transpose().row(index)) * _state.vel - measurement(index);

			VectorState Kfusion = P * H[index] / aid_src.innovation_variance[index];
			measurementUpdate(Kfusion, H[index], aid_src.observation_variance[index], aid_src.innovation[index]);
		}

		aid_src.fused = true;
		aid_src.time_last_fuse = _time_delayed_us;

		_time_last_hor_vel_fuse = _time_delayed_us;
		_time_last_ver_vel_fuse = _time_delayed_us;
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ev_yaw_control.cpp
 * Control functions for ekf external vision yaw fusion
 */

#include "ekf.h"

void Ekf::controlEvYawFusion(const imuSample &imu_sample, const extVisionSample &ev_sample,
			     const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient,
			     estimator_aid_source1d_s &aid_src)
{
	static constexpr const char *AID_SRC_NAME = "EV yaw";

	float obs = getEulerYaw(ev_sample.quat);
	float obs_var = math::max(ev_sample.orientation_var(2), _params.ev_att_noise, sq(0.01f));

	float innov = wrap_pi(getEulerYaw(_R_to_earth) - obs);
	float innov_var = 0.f;

	VectorState H_YAW;
	computeYawInnovVarAndH(obs_var, innov_var, H_YAW);

	updateAidSourceStatus(aid_src,
			      ev_sample.time_us,                           // sample timestamp
			      obs,                                         // observation
			      obs_var,                                     // observation variance
			      innov,                                       // innovation
			      innov_var,                                   // innovation variance
			      math::max(_params.heading_innov_gate, 1.f)); // innovation gate

	if (ev_reset) {
		_control_status.flags.ev_yaw_fault = false;
	}

	// determine if we should use EV yaw aiding
	bool continuing_conditions_passing = (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::YAW))
					     && _control_status.flags.tilt_align
					     && !_control_status.flags.ev_yaw_fault
					     && PX4_ISFINITE(aid_src.observation)
					     && PX4_ISFINITE(aid_src.observation_variance);

	// if GNSS is enabled don't allow EV yaw if EV isn't NED
	if ((_control_status.flags.gnss_vel || _control_status.flags.gnss_pos) && _control_status.flags.yaw_align
	    && (ev_sample.pos_frame != PositionFrame::LOCAL_FRAME_NED)
	   ) {
		continuing_conditions_passing = false;
	}

	const bool starting_conditions_passing = common_starting_conditions_passing
			&& continuing_conditions_passing
			&& isTimedOut(aid_src.time_last_fuse, (uint32_t)1e6);

	if (_control_status.flags.ev_yaw) {
		if (continuing_conditions_passing) {

			if (ev_reset) {

				if (quality_sufficient) {
					ECL_INFO("reset to %s", AID_SRC_NAME);
					//_information_events.flags.reset_yaw_to_vision = true; // TODO
					resetQuatStateYaw(aid_src.observation, aid_src.observation_variance);
					aid_src.time_last_fuse = _time_delayed_us;

				} else {
					// EV has reset, but quality isn't sufficient
					// we have no choice but to stop EV and try to resume once quality is acceptable
					stopEvYawFusion();
					return;
				}

			} else if (quality_sufficient) {
				fuseYaw(aid_src, H_YAW);

			} else {
				aid_src.innovation_rejected = true;
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.no_aid_timeout_max);

			if (is_fusion_failing) {
				if ((_nb_ev_yaw_reset_available > 0) && quality_sufficient) {
					// Data seems good, attempt a reset
					//_information_events.flags.reset_yaw_to_vision = true; // TODO
					ECL_WARN("%s fusion failing, resetting", AID_SRC_NAME);
					resetQuatStateYaw(aid_src.observation, aid_src.observation_variance);
					aid_src.time_last_fuse = _time_delayed_us;

					if (_control_status.flags.in_air) {
						_nb_ev_yaw_reset_available--;
					}

				} else if (starting_conditions_passing) {
					// Data seems good, but previous reset did not fix the issue
					// something else must be wrong, declare the sensor faulty and stop the fusion
					//_control_status.flags.ev_yaw_fault = true;
					ECL_WARN("stopping %s fusion, starting conditions failing", AID_SRC_NAME);
					stopEvYawFusion();

				} else {
					// A reset did not fix the issue but all the starting checks are not passing
					// This could be a temporary issue, stop the fusion without declaring the sensor faulty
					ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);
					stopEvYawFusion();
				}
			}

		} else {
			// Stop fusion but do not declare it faulty
			ECL_WARN("stopping %s fusion, continuing conditions failing", AID_SRC_NAME);
			stopEvYawFusion();
		}

	} else {
		if (starting_conditions_passing) {
			// activate fusion
			if (ev_sample.pos_frame == PositionFrame::LOCAL_FRAME_NED) {

				if (_control_status.flags.yaw_align) {

					if (fuseYaw(aid_src, H_YAW)) {
						ECL_INFO("starting %s fusion", AID_SRC_NAME);
						_information_events.flags.starting_vision_yaw_fusion = true;

						_control_status.flags.ev_yaw = true;
					}

				} else {
					// reset yaw to EV and set yaw_align
					ECL_INFO("starting %s fusion, resetting state", AID_SRC_NAME);
					_information_events.flags.starting_vision_yaw_fusion = true;

					resetQuatStateYaw(aid_src.observation, aid_src.observation_variance);
					_control_status.flags.yaw_align = true;
					_control_status.flags.ev_yaw = true;

					aid_src.time_last_fuse = _time_delayed_us;
				}

			} else if (ev_sample.pos_frame == PositionFrame::LOCAL_FRAME_FRD) {
				// turn on fusion of external vision yaw measurements
				ECL_INFO("starting %s fusion, resetting state", AID_SRC_NAME);

				// reset yaw to EV
				resetQuatStateYaw(aid_src.observation, aid_src.observation_variance);
				aid_src.time_last_fuse = _time_delayed_us;

				_information_events.flags.starting_vision_yaw_fusion = true;
				_control_status.flags.yaw_align = false;
				_control_status.flags.ev_yaw = true;
			}

			if (_control_status.flags.ev_yaw) {
				_nb_ev_yaw_reset_available = 5;
			}
		}
	}
}

void Ekf::stopEvYawFusion()
{
#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_yaw) {

		_control_status.flags.ev_yaw = false;
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file gnss_height_control.cpp
 * Control functions for ekf GNSS height fusion
 */

#include "ekf.h"

void Ekf::controlGnssHeightFusion(const gnssSample &gps_sample)
{
	static constexpr const char *HGT_SRC_NAME = "GNSS";

	auto &aid_src = _aid_src_gnss_hgt;
	HeightBiasEstimator &bias_est = _gps_hgt_b_est;

	bias_est.predict(_dt_ekf_avg);

	if (_gps_data_ready) {

		// relax the upper observation noise limit which prevents bad GPS perturbing the position estimate
		float noise = math::max(gps_sample.vacc, 1.5f * _params.gps_pos_noise); // use 1.5 as a typical ratio of vacc/hacc

		if (!isOnlyActiveSourceOfVerticalPositionAiding(_control_status.flags.gps_hgt)) {
			// if we are not using another source of aiding, then we are reliant on the GPS
			// observations to constrain attitude errors and must limit the observation noise value.
			if (noise > _params.pos_noaid_noise) {
				noise = _params.pos_noaid_noise;
			}
		}

		const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;
		const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
		const float gnss_alt = gps_sample.alt + pos_offset_earth(2);

		const float measurement = gnss_alt;
		const float measurement_var = sq(noise);

		const bool measurement_valid = PX4_ISFINITE(measurement) && PX4_ISFINITE(measurement_var);

		// GNSS position, vertical position GNSS measurement has opposite sign to earth z axis
		updateVerticalPositionAidStatus(aid_src,
						gps_sample.time_us,
						-(measurement - bias_est.getBias()),
						measurement_var + bias_est.getBiasVar(),
						math::max(_params.gps_pos_innov_gate, 1.f));

		// update the bias estimator before updating the main filter but after
		// using its current state to compute the vertical position innovation
		if (measurement_valid) {
			bias_est.setMaxStateNoise(sqrtf(measurement_var));
			bias_est.setProcessNoiseSpectralDensity(_params.gps_hgt_bias_nsd);
			bias_est.fuseBias(measurement - _gpos.altitude(), measurement_var + P(State::pos.idx + 2, State::pos.idx + 2));
		}

		// determine if we should use height aiding
		const bool common_conditions_passing = measurement_valid
						       && _local_origin_lat_lon.isInitialized()
						       && _gps_checks_passed;

		const bool continuing_conditions_passing = (_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::VPOS))
				&& common_conditions_passing;

		const bool starting_conditions_passing = continuing_conditions_passing
				&& isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL);

		const bool altitude_initialisation_conditions_passing = common_conditions_passing
				&& !PX4_ISFINITE(_local_origin_alt)
				&& _params.height_sensor_ref == static_cast<int32_t>(HeightSensor::GNSS)
				&& isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL);

		if (_control_status.flags.gps_hgt) {
			if (continuing_conditions_passing) {

				fuseVerticalPosition(aid_src);

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired() && (_height_sensor_ref == HeightSensor::GNSS)) {
					// All height sources are failing
					ECL_WARN("%s height fusion reset required, all height sources failing", HGT_SRC_NAME);

					_information_events.flags.reset_hgt_to_gps = true;
					resetAltitudeTo(measurement, measurement_var);
					bias_est.setBias(-_gpos.altitude() + measurement);
					resetAidSourceStatusZeroInnovation(aid_src);

					aid_src.time_last_fuse = _time_delayed_us;

				} else if (is_fusion_failing) {
					// Some other height source is still working
					ECL_WARN("stopping %s height fusion, fusion failing", HGT_SRC_NAME);
					stopGpsHgtFusion();
				}

			} else {
				ECL_WARN("stopping %s height fusion, continuing conditions failing", HGT_SRC_NAME);
				stopGpsHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				if (_params.height_sensor_ref == static_cast<int32_t>(HeightSensor::GNSS)) {
					ECL_INFO("starting %s height fusion, resetting height", HGT_SRC_NAME);
					_height_sensor_ref = HeightSensor::GNSS;

					_information_events.flags.reset_hgt_to_gps = true;

					initialiseAltitudeTo(measurement, measurement_var);
					bias_est.reset();
					resetAidSourceStatusZeroInnovation(aid_src);

				} else {
					ECL_INFO("starting %s height fusion", HGT_SRC_NAME);
					bias_est.setBias(-_gpos.altitude() + measurement);
				}

				aid_src.time_last_fuse = _time_delayed_us;
				bias_est.setFusionActive();
				_control_status.flags.gps_hgt = true;

			} if (altitude_initialisation_conditions_passing) {

				// Do not start GNSS altitude aiding, but use measurement
				// to initialize altitude and bias of other height sensors
				_information_events.flags.reset_hgt_to_gps = true;

				initialiseAltitudeTo(measurement, measurement_var);
				bias_est.reset();
			}
		}

	} else if (_control_status.flags.gps_hgt
		   && !isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL)) {
		// No data anymore. Stop until it comes back.
		ECL_WARN("stopping %s height fusion, no data", HGT_SRC_NAME);
		stopGpsHgtFusion();
	}
}

void Ekf::stopGpsHgtFusion()
{
	if (_control_status.flags.gps_hgt) {

		if (_height_sensor_ref == HeightSensor::GNSS) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_gps_hgt_b_est.setFusionInactive();

		_control_status.flags.gps_hgt = false;
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file gps_checks.cpp
 * Perform pre-flight and in-flight GPS quality checks
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>

// GPS pre-flight check bit locations
#define MASK_GPS_NSATS   (1<<0)
#define MASK_GPS_PDOP    (1<<1)
#define MASK_GPS_HACC    (1<<2)
#define MASK_GPS_VACC    (1<<3)
#define MASK_GPS_SACC    (1<<4)
#define MASK_GPS_HDRIFT  (1<<5)
#define MASK_GPS_VDRIFT  (1<<6)
#define MASK_GPS_HSPD    (1<<7)
#define MASK_GPS_VSPD    (1<<8)
#define MASK_GPS_SPOOFED (1<<9)

bool Ekf::runGnssChecks(const gnssSample &gps)
{
	_gps_check_fail_status.flags.spoofed = gps.spoofed;

	// Check the fix type
	_gps_check_fail_status.flags.fix = (gps.fix_type < 3);

	// Check the number of satellites
	_gps_check_fail_status.flags.nsats = (gps.nsats < _params.req_nsats);

	// Check the position dilution of precision
	_gps_check_fail_status.flags.pdop = (gps.pdop > _params.req_pdop);

	// Check the reported horizontal and vertical position accuracy
	_gps_check_fail_status.flags.hacc = (gps.hacc > _params.req_hacc);
	_gps_check_fail_status.flags.vacc = (gps.vacc > _params.req_vacc);

	// Check the reported speed accuracy
	_gps_check_fail_status.flags.sacc = (gps.sacc > _params.req_sacc);

	// Calculate time lapsed since last update, limit to prevent numerical errors and calculate a lowpass filter coefficient
	constexpr float filt_time_const = 10.0f;
	const float dt = math::constrain(float(int64_t(gps.time_us) - int64_t(_gps_pos_prev.getProjectionReferenceTimestamp()))
					 * 1e-6f, 0.001f, filt_time_const);
	const float filter_coef = dt / filt_time_const;

	// The following checks are only valid when the vehicle is at rest
	const double lat = gps.lat;
	const double lon = gps.lon;

	if (!_control_status.flags.in_air && _control_status.flags.vehicle_at_rest) {
		// Calculate position movement since last measurement
		float delta_pos_n = 0.0f;
		float delta_pos_e = 0.0f;

		// calculate position movement since last GPS fix
		if (_gps_pos_prev.getProjectionReferenceTimestamp() > 0) {
			_gps_pos_prev.project(lat, lon, delta_pos_n, delta_pos_e);

		} else {
			// no previous position has been set
			_gps_pos_prev.initReference(lat, lon, gps.time_us);
			_gps_alt_prev = gps.alt;
		}

		// Calculate the horizontal and vertical drift velocity components and limit to 10x the threshold
		const Vector3f vel_limit(_params.req_hdrift, _params.req_hdrift, _params.req_vdrift);
		Vector3f delta_pos(delta_pos_n, delta_pos_e, (_gps_alt_prev - gps.alt));

		// Apply a low pass filter
		_gps_pos_deriv_filt = delta_pos / dt * filter_coef + _gps_pos_deriv_filt * (1.0f - filter_coef);

		// Apply anti-windup to the state instead of the input to avoid generating a bias on asymmetric signals
		_gps_pos_deriv_filt = matrix::constrain(_gps_pos_deriv_filt, -10.0f * vel_limit, 10.0f * vel_limit);

		// hdrift: calculate the horizontal drift speed and fail if too high
		_gps_horizontal_position_drift_rate_m_s = Vector2f(_gps_pos_deriv_filt.xy()).norm();
		_gps_check_fail_status.flags.hdrift = (_gps_horizontal_position_drift_rate_m_s > _params.req_hdrift);

		// vdrift: fail if the vertical drift speed is too high
		_gps_vertical_position_drift_rate_m_s = fabsf(_gps_pos_deriv_filt(2));
		_gps_check_fail_status.flags.vdrift = (_gps_vertical_position_drift_rate_m_s > _params.req_vdrift);

		// hspeed: check the magnitude of the filtered horizontal GNSS velocity
		const Vector2f gps_velNE = matrix::constrain(Vector2f(gps.vel.xy()),
					   -10.0f * _params.req_hdrift,
					   10.0f * _params.req_hdrift);
		_gps_velNE_filt = gps_velNE * filter_coef + _gps_velNE_filt * (1.0f - filter_coef);
		_gps_filtered_horizontal_velocity_m_s = _gps_velNE_filt.norm();
		_gps_check_fail_status.flags.hspeed = (_gps_filtered_horizontal_velocity_m_s > _params.req_hdrift);

		// vspeed: check the magnitude of the filtered vertical GNSS velocity
		const float gnss_vz_limit = 10.f * _params.req_vdrift;
		const float gnss_vz = math::constrain(gps.vel(2), -gnss_vz_limit, gnss_vz_limit);
		_gps_vel_d_filt = gnss_vz * filter_coef + _gps_vel_d_filt * (1.f - filter_coef);

		_gps_check_fail_status.flags.vspeed = (fabsf(_gps_vel_d_filt) > _params.req_vdrift);

	} else if (_control_status.flags.in_air) {
		// These checks are always declared as passed when flying
		// If on ground and moving, the last result before movement commenced is kept
		_gps_check_fail_status.flags.hdrift = false;
		_gps_check_fail_status.flags.vdrift = false;
		_gps_check_fail_status.flags.hspeed = false;
		_gps_check_fail_status.flags.vspeed = false;

		resetGpsDriftCheckFilters();

	} else {
		// This is the case where the vehicle is on ground and IMU movement is blocking the drift calculation
		resetGpsDriftCheckFilters();
	}

	// force horizontal speed failure if above the limit
	if (gps.vel.xy().longerThan(_params.velocity_limit)) {
		_gps_check_fail_status.flags.hspeed = true;
	}

	// force vertical speed failure if above the limit
	if (fabsf(gps.vel(2)) > _params.velocity_limit) {
		_gps_check_fail_status.flags.vspeed = true;
	}

	// save GPS fix for next time
	_gps_pos_prev.initReference(lat, lon, gps.time_us);
	_gps_alt_prev = gps.alt;

	// assume failed first time through
	if (_last_gps_fail_us == 0) {
		_last_gps_fail_us = _time_delayed_us;
	}

	// if any user selected checks have failed, record the fail time
	if (
		_gps_check_fail_status.flags.fix ||
		(_gps_check_fail_status.flags.nsats   && (_params.gps_check_mask & MASK_GPS_NSATS)) ||
		(_gps_check_fail_status.flags.pdop    && (_params.gps_check_mask & MASK_GPS_PDOP)) ||
		(_gps_check_fail_status.flags.hacc    && (_params.gps_check_mask & MASK_GPS_HACC)) ||
		(_gps_check_fail_status.flags.vacc    && (_params.gps_check_mask & MASK_GPS_VACC)) ||
		(_gps_check_fail_status.flags.sacc    && (_params.gps_check_mask & MASK_GPS_SACC)) ||
		(_gps_check_fail_status.flags.hdrift  && (_params.gps_check_mask & MASK_GPS_HDRIFT)) ||
		(_gps_check_fail_status.flags.vdrift  && (_params.gps_check_mask & MASK_GPS_VDRIFT)) ||
		(_gps_check_fail_status.flags.hspeed  && (_params.gps_check_mask & MASK_GPS_HSPD)) ||
		(_gps_check_fail_status.flags.vspeed  && (_params.gps_check_mask & MASK_GPS_VSPD)) ||
		(_gps_check_fail_status.flags.spoofed && (_params.gps_check_mask & MASK_GPS_SPOOFED))
	) {
		_last_gps_fail_us = _time_delayed_us;
		return false;

	} else {
		_last_gps_pass_us = _time_delayed_us;
		return true;
	}
}

void Ekf::resetGpsDriftCheckFilters()
{
	_gps_velNE_filt.setZero();
	_gps_vel_d_filt = 0.f;

	_gps_pos_deriv_filt.setZero();

	_gps_horizontal_position_drift_rate_m_s = NAN;
	_gps_vertical_position_drift_rate_m_s = NAN;
	_gps_filtered_horizontal_velocity_m_s = NAN;
}
/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file gps_control.cpp
 * Control functions for ekf GNSS fusion
 */

#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlGpsFusion(const imuSample &imu_delayed)
{
	if (!_gps_buffer || (_params.gnss_ctrl == 0)) {
		stopGnssFusion();
		return;
	}

	if (!gyro_bias_inhibited()) {
		_yawEstimator.setGyroBias(getGyroBias(), _control_status.flags.vehicle_at_rest);
	}

	// run EKF-GSF yaw estimator once per imu_delayed update
	_yawEstimator.predict(imu_delayed.delta_ang, imu_delayed.delta_ang_dt,
			      imu_delayed.delta_vel, imu_delayed.delta_vel_dt,
			      (_control_status.flags.in_air && !_control_status.flags.vehicle_at_rest));

	_gps_intermittent = !isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL);

	// check for arrival of new sensor data at the fusion time horizon
	_gps_data_ready = _gps_buffer->pop_first_older_than(imu_delayed.time_us, &_gps_sample_delayed);

	if (_gps_data_ready) {
		const gnssSample &gnss_sample = _gps_sample_delayed;

		if (runGnssChecks(gnss_sample)
		    && isTimedOut(_last_gps_fail_us, max((uint64_t)1e6, (uint64_t)_min_gps_health_time_us / 10))) {
			if (isTimedOut(_last_gps_fail_us, (uint64_t)_min_gps_health_time_us)) {
				// First time checks are passing, latching.
				if (!_gps_checks_passed) {
					_information_events.flags.gps_checks_passed = true;
				}

				_gps_checks_passed = true;
			}

		} else {
			// Skip this sample
			_gps_data_ready = false;

			if ((_control_status.flags.gnss_vel || _control_status.flags.gnss_pos)
			    && isTimedOut(_last_gps_pass_us, _params.reset_timeout_max)) {
				stopGnssFusion();
				ECL_WARN("GNSS quality poor - stopping use");
			}
		}

		updateGnssPos(gnss_sample, _aid_src_gnss_pos);
		updateGnssVel(imu_delayed, gnss_sample, _aid_src_gnss_vel);

	} else if (_control_status.flags.gnss_vel || _control_status.flags.gnss_pos) {
		if (!isNewestSampleRecent(_time_last_gps_buffer_push, _params.reset_timeout_max)) {
			stopGnssFusion();
			ECL_WARN("GNSS data stopped");
		}
	}

	if (_gps_data_ready) {
#if defined(CONFIG_EKF2_GNSS_YAW)
		const gnssSample &gnss_sample = _gps_sample_delayed;
		controlGnssYawFusion(gnss_sample);
#endif // CONFIG_EKF2_GNSS_YAW

		controlGnssYawEstimator(_aid_src_gnss_vel);

		bool do_vel_pos_reset = false;

		if (_control_status.flags.gnss_vel || _control_status.flags.gnss_pos) {
			// Reset checks need to run together, before each control function runs because the result would change
			// after the first one runs
			do_vel_pos_reset = shouldResetGpsFusion();

			if (_control_status.flags.in_air
			    && isYawFailure()
			    && isTimedOut(_time_last_hor_vel_fuse, _params.EKFGSF_reset_delay)
			    && (_time_last_hor_vel_fuse > _time_last_on_ground_us)) {
				do_vel_pos_reset = tryYawEmergencyReset();
			}
		}

		controlGnssVelFusion(_aid_src_gnss_vel, do_vel_pos_reset);
		controlGnssPosFusion(_aid_src_gnss_pos, do_vel_pos_reset);
	}
}

void Ekf::controlGnssVelFusion(estimator_aid_source3d_s &aid_src, const bool force_reset)
{
	const bool continuing_conditions_passing = (_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::VEL))
			&& _control_status.flags.tilt_align
			&& _control_status.flags.yaw_align;
	const bool starting_conditions_passing = continuing_conditions_passing && _gps_checks_passed;

	if (_control_status.flags.gnss_vel) {
		if (continuing_conditions_passing) {
			if (force_reset) {
				ECL_WARN("GNSS fusion timeout, resetting");
				resetVelocityToGnss(aid_src);

			} else {
				fuseVelocity(aid_src);

				if (isHeightResetRequired()) {
					// reset vertical velocity if height is failing
					resetVerticalVelocityTo(aid_src.observation[2], aid_src.observation_variance[2]);
				}
			}

		} else {
			stopGnssVelFusion();
		}

	} else {
		if (starting_conditions_passing) {
			ECL_INFO("starting GNSS velocity fusion");
			_information_events.flags.starting_gps_fusion = true;

			// when already using another velocity source velocity reset is not necessary
			if (!isHorizontalAidingActive()
			    || isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
			    || !_control_status_prev.flags.yaw_align
			   ) {
				// reset velocity
				resetVelocityToGnss(aid_src);
			}

			_control_status.flags.gnss_vel = true;
		}
	}
}

void Ekf::controlGnssPosFusion(estimator_aid_source2d_s &aid_src, const bool force_reset)
{
	const bool gnss_pos_enabled = (_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::HPOS));

	const bool continuing_conditions_passing = gnss_pos_enabled
			&& _control_status.flags.tilt_align
			&& _control_status.flags.yaw_align;
	const bool starting_conditions_passing = continuing_conditions_passing && _gps_checks_passed;
	const bool gpos_init_conditions_passing = gnss_pos_enabled && _gps_checks_passed;

	if (_control_status.flags.gnss_pos) {
		if (continuing_conditions_passing) {
			if (force_reset) {
				ECL_WARN("GNSS fusion timeout, resetting");
				resetHorizontalPositionToGnss(aid_src);

			} else {
				fuseHorizontalPosition(aid_src);
			}

		} else {
			stopGnssPosFusion();
		}

	} else {
		if (starting_conditions_passing) {
			ECL_INFO("starting GNSS position fusion");
			_information_events.flags.starting_gps_fusion = true;

			resetHorizontalPositionToGnss(aid_src);
			_control_status.flags.gnss_pos = true;

		} else if (gpos_init_conditions_passing && !_local_origin_lat_lon.isInitialized()) {
			resetHorizontalPositionToGnss(aid_src);
		}
	}
}

void Ekf::updateGnssVel(const imuSample &imu_sample, const gnssSample &gnss_sample, estimator_aid_source3d_s &aid_src)
{
	// correct velocity for offset relative to IMU
	const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;

	const Vector3f angular_velocity = imu_sample.delta_ang / imu_sample.delta_ang_dt - _state.gyro_bias;
	const Vector3f vel_offset_body = angular_velocity % pos_offset_body;
	const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;
	const Vector3f velocity = gnss_sample.vel - vel_offset_earth;

	const float vel_var = sq(math::max(gnss_sample.sacc, _params.gps_vel_noise, 0.01f));
	const Vector3f vel_obs_var(vel_var, vel_var, vel_var * sq(1.5f));

	const float innovation_gate = math::max(_params.gps_vel_innov_gate, 1.f);

	updateAidSourceStatus(aid_src,
			      gnss_sample.time_us,                  // sample timestamp
			      velocity,                             // observation
			      vel_obs_var,                          // observation variance
			      _state.vel - velocity,                // innovation
			      getVelocityVariance() + vel_obs_var,  // innovation variance
			      innovation_gate);                     // innovation gate

	// vz special case if there is bad vertical acceleration data, then don't reject measurement if GNSS reports velocity accuracy is acceptable,
	// but limit innovation to prevent spikes that could destabilise the filter
	bool bad_acc_vz_rejected = _fault_status.flags.bad_acc_vertical
				   && (aid_src.test_ratio[2] > 1.f)                                   // vz rejected
				   && (aid_src.test_ratio[0] < 1.f) && (aid_src.test_ratio[1] < 1.f); // vx & vy accepted

	if (bad_acc_vz_rejected
	    && (gnss_sample.sacc < _params.req_sacc)
	   ) {
		const float innov_limit = innovation_gate * sqrtf(aid_src.innovation_variance[2]);
		aid_src.innovation[2] = math::constrain(aid_src.innovation[2], -innov_limit, innov_limit);
		aid_src.innovation_rejected = false;
	}
}

void Ekf::updateGnssPos(const gnssSample &gnss_sample, estimator_aid_source2d_s &aid_src)
{
	// correct position and height for offset relative to IMU
	const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;
	const Vector3f pos_offset_earth = Vector3f(_R_to_earth * pos_offset_body);
	const LatLonAlt measurement(gnss_sample.lat, gnss_sample.lon, gnss_sample.alt);
	const LatLonAlt measurement_corrected = measurement + (-pos_offset_earth);
	const Vector2f innovation = (_gpos - measurement_corrected).xy();

	// relax the upper observation noise limit which prevents bad GPS perturbing the position estimate
	float pos_noise = math::max(gnss_sample.hacc, _params.gps_pos_noise);

	if (!isOtherSourceOfHorizontalAidingThan(_control_status.flags.gnss_pos)) {
		// if we are not using another source of aiding, then we are reliant on the GNSS
		// observations to constrain attitude errors and must limit the observation noise value.
		if (pos_noise > _params.pos_noaid_noise) {
			pos_noise = _params.pos_noaid_noise;
		}
	}

	const float pos_var = math::max(sq(pos_noise), sq(0.01f));
	const Vector2f pos_obs_var(pos_var, pos_var);
	const matrix::Vector2d observation(measurement_corrected.latitude_deg(), measurement_corrected.longitude_deg());

	updateAidSourceStatus(aid_src,
			      gnss_sample.time_us,                                    // sample timestamp
			      observation,                                            // observation
			      pos_obs_var,                                            // observation variance
			      innovation,                                             // innovation
			      Vector2f(getStateVariance<State::pos>()) + pos_obs_var, // innovation variance
			      math::max(_params.gps_pos_innov_gate, 1.f));            // innovation gate
}

void Ekf::controlGnssYawEstimator(estimator_aid_source3d_s &aid_src_vel)
{
	// update yaw estimator velocity (basic sanity check on GNSS velocity data)
	const float vel_var = aid_src_vel.observation_variance[0];
	const Vector2f vel_xy(aid_src_vel.observation);

	if ((vel_var > 0.f)
	    && (vel_var < _params.req_sacc)
	    && vel_xy.isAllFinite()) {

		_yawEstimator.fuseVelocity(vel_xy, vel_var, _control_status.flags.in_air);

		// Try to align yaw using estimate if available
		if (((_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::VEL))
		     || (_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::HPOS)))
		    && !_control_status.flags.yaw_align
		    && _control_status.flags.tilt_align) {
			if (resetYawToEKFGSF()) {
				ECL_INFO("GPS yaw aligned using IMU");
			}
		}
	}
}

bool Ekf::tryYawEmergencyReset()
{
	bool success = false;

	/* A rapid reset to the yaw emergency estimate is performed if horizontal velocity innovation checks continuously
	 * fails while the difference between the yaw emergency estimator and the yaw estimate is large.
	 * This enables recovery from a bad yaw estimate. A reset is not performed if the fault condition was
	 * present before flight to prevent triggering due to GPS glitches or other sensor errors.
	 */
	if (resetYawToEKFGSF()) {
		ECL_WARN("GPS emergency yaw reset");

		if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
			// stop using the magnetometer in the main EKF otherwise its fusion could drag the yaw around
			// and cause another navigation failure
			_control_status.flags.mag_fault = true;
		}

#if defined(CONFIG_EKF2_GNSS_YAW)

		if (_control_status.flags.gnss_yaw) {
			_control_status.flags.gnss_yaw_fault = true;
		}

#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

		if (_control_status.flags.ev_yaw) {
			_control_status.flags.ev_yaw_fault = true;
		}

#endif // CONFIG_EKF2_EXTERNAL_VISION

		success = true;
	}

	return success;
}

void Ekf::resetVelocityToGnss(estimator_aid_source3d_s &aid_src)
{
	_information_events.flags.reset_vel_to_gps = true;
	resetVelocityTo(Vector3f(aid_src.observation), Vector3f(aid_src.observation_variance));

	resetAidSourceStatusZeroInnovation(aid_src);
}

void Ekf::resetHorizontalPositionToGnss(estimator_aid_source2d_s &aid_src)
{
	_information_events.flags.reset_pos_to_gps = true;
	resetLatLonTo(aid_src.observation[0], aid_src.observation[1],
		      aid_src.observation_variance[0] +
		      aid_src.observation_variance[1]);

	resetAidSourceStatusZeroInnovation(aid_src);
}

bool Ekf::shouldResetGpsFusion() const
{
	/* We are relying on aiding to constrain drift so after a specified time
	 * with no aiding we need to do something
	 */
	bool has_horizontal_aiding_timed_out = isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
					       && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max);

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

	if (has_horizontal_aiding_timed_out) {
		// horizontal aiding hasn't timed out if optical flow still active
		if (_control_status.flags.opt_flow && isRecent(_aid_src_optical_flow.time_last_fuse, _params.reset_timeout_max)) {
			has_horizontal_aiding_timed_out = false;
		}
	}

#endif // CONFIG_EKF2_OPTICAL_FLOW

	const bool is_reset_required = has_horizontal_aiding_timed_out
				       || (isTimedOut(_time_last_hor_pos_fuse, 2 * _params.reset_timeout_max)
					   && (_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::HPOS)));

	const bool is_inflight_nav_failure = _control_status.flags.in_air
					     && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
					     && isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
					     && (_time_last_hor_vel_fuse > _time_last_on_ground_us)
					     && (_time_last_hor_pos_fuse > _time_last_on_ground_us);

	return (is_reset_required || is_inflight_nav_failure);
}

void Ekf::stopGnssFusion()
{
	if (_control_status.flags.gnss_vel || _control_status.flags.gnss_pos) {
		_last_gps_fail_us = 0;
		_last_gps_pass_us = 0;
	}

	stopGnssVelFusion();
	stopGnssPosFusion();
	stopGpsHgtFusion();
#if defined(CONFIG_EKF2_GNSS_YAW)
	stopGnssYawFusion();
#endif // CONFIG_EKF2_GNSS_YAW

	_yawEstimator.reset();
}

void Ekf::stopGnssVelFusion()
{
	if (_control_status.flags.gnss_vel) {
		ECL_INFO("stopping GNSS velocity fusion");
		_control_status.flags.gnss_vel = false;

		//TODO: what if gnss yaw or height is used?
		if (!_control_status.flags.gnss_pos) {
			_last_gps_fail_us = 0;
			_last_gps_pass_us = 0;
		}
	}
}

void Ekf::stopGnssPosFusion()
{
	if (_control_status.flags.gnss_pos) {
		ECL_INFO("stopping GNSS position fusion");
		_control_status.flags.gnss_pos = false;

		//TODO: what if gnss yaw or height is used?
		if (!_control_status.flags.gnss_vel) {
			_last_gps_fail_us = 0;
			_last_gps_pass_us = 0;
		}
	}
}

bool Ekf::isYawEmergencyEstimateAvailable() const
{
	// don't allow reet using the EKF-GSF estimate until the filter has started fusing velocity
	// data and the yaw estimate has converged
	if (!_yawEstimator.isActive()) {
		return false;
	}

	const float yaw_var = _yawEstimator.getYawVar();

	return (yaw_var > 0.f)
	       && (yaw_var < sq(_params.EKFGSF_yaw_err_max))
	       && PX4_ISFINITE(yaw_var);
}

bool Ekf::isYawFailure() const
{
	if (!isYawEmergencyEstimateAvailable()) {
		return false;
	}

	const float euler_yaw = getEulerYaw(_R_to_earth);
	const float yaw_error = wrap_pi(euler_yaw - _yawEstimator.getYaw());

	return fabsf(yaw_error) > math::radians(25.f);
}

bool Ekf::resetYawToEKFGSF()
{
	if (!isYawEmergencyEstimateAvailable()) {
		return false;
	}

	// don't allow reset if there's just been a yaw reset
	const bool yaw_alignment_changed = (_control_status_prev.flags.yaw_align != _control_status.flags.yaw_align);
	const bool quat_reset = (_state_reset_status.reset_count.quat != _state_reset_count_prev.quat);

	if (yaw_alignment_changed || quat_reset) {
		return false;
	}

	ECL_INFO("yaw estimator reset heading %.3f -> %.3f rad",
		 (double)getEulerYaw(_R_to_earth), (double)_yawEstimator.getYaw());

	resetQuatStateYaw(_yawEstimator.getYaw(), _yawEstimator.getYawVar());

	_control_status.flags.yaw_align = true;
	_information_events.flags.yaw_aligned_to_imu_gps = true;

	return true;
}

bool Ekf::getDataEKFGSF(float *yaw_composite, float *yaw_variance, float yaw[N_MODELS_EKFGSF],
			float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF])
{
	return _yawEstimator.getLogData(yaw_composite, yaw_variance, yaw, innov_VN, innov_VE, weight);
}
/****************************************************************************
 *
 *   Copyright (c) 2018-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file gnss_yaw_control.cpp
 * Definition of functions required to use yaw obtained from GNSS dual antenna measurements.
 * Equations generated using src/modules/ekf2/EKF/python/ekf_derivation/derivation.py
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>
#include <cstdlib>

#include <ekf_derivation/generated/compute_gnss_yaw_pred_innov_var_and_h.h>

void Ekf::controlGnssYawFusion(const gnssSample &gnss_sample)
{
	if (!(_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::YAW))
	    || _control_status.flags.gnss_yaw_fault) {

		stopGnssYawFusion();
		return;
	}

	const bool is_new_data_available = PX4_ISFINITE(gnss_sample.yaw);

	if (is_new_data_available) {

		updateGnssYaw(gnss_sample);

		const bool continuing_conditions_passing = _control_status.flags.tilt_align;

		const bool is_gnss_yaw_data_intermittent = !isNewestSampleRecent(_time_last_gnss_yaw_buffer_push,
				2 * GNSS_YAW_MAX_INTERVAL);

		const bool starting_conditions_passing = continuing_conditions_passing
				&& _gps_checks_passed
				&& !is_gnss_yaw_data_intermittent
				&& !_gps_intermittent;

		if (_control_status.flags.gnss_yaw) {
			if (continuing_conditions_passing) {

				fuseGnssYaw(gnss_sample.yaw_offset);

				const bool is_fusion_failing = isTimedOut(_aid_src_gnss_yaw.time_last_fuse, _params.reset_timeout_max);

				if (is_fusion_failing) {
					stopGnssYawFusion();

					// Before takeoff, we do not want to continue to rely on the current heading
					// if we had to stop the fusion
					if (!_control_status.flags.in_air) {
						ECL_INFO("clearing yaw alignment");
						_control_status.flags.yaw_align = false;
					}
				}

			} else {
				// Stop GNSS yaw fusion but do not declare it faulty
				stopGnssYawFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate GNSS yaw fusion

				if (!_control_status.flags.in_air
				    || !_control_status.flags.yaw_align
				    || !isNorthEastAidingActive()) {

					// Reset before starting the fusion
					if (resetYawToGnss(gnss_sample.yaw, gnss_sample.yaw_offset)) {

						resetAidSourceStatusZeroInnovation(_aid_src_gnss_yaw);

						_control_status.flags.gnss_yaw = true;
						_control_status.flags.yaw_align = true;
					}

				} else if (!_aid_src_gnss_yaw.innovation_rejected) {
					// Do not force a reset but wait for the consistency check to pass
					_control_status.flags.gnss_yaw = true;
					fuseGnssYaw(gnss_sample.yaw_offset);
				}

				if (_control_status.flags.gnss_yaw) {
					ECL_INFO("starting GNSS yaw fusion");
				}
			}
		}

	} else if (_control_status.flags.gnss_yaw
		   && !isNewestSampleRecent(_time_last_gnss_yaw_buffer_push, _params.reset_timeout_max)) {

		// No yaw data in the message anymore. Stop until it comes back.
		stopGnssYawFusion();
	}
}

void Ekf::updateGnssYaw(const gnssSample &gnss_sample)
{
	// calculate the observed yaw angle of antenna array, converting a from body to antenna yaw measurement
	const float measured_hdg = wrap_pi(gnss_sample.yaw + gnss_sample.yaw_offset);

	const float yaw_acc = PX4_ISFINITE(gnss_sample.yaw_acc) ? gnss_sample.yaw_acc : 0.f;
	const float R_YAW = sq(fmaxf(yaw_acc, _params.gnss_heading_noise));

	float heading_pred;
	float heading_innov_var;

	VectorState H;
	sym::ComputeGnssYawPredInnovVarAndH(_state.vector(), P, gnss_sample.yaw_offset, R_YAW, FLT_EPSILON,
					    &heading_pred, &heading_innov_var, &H);

	updateAidSourceStatus(_aid_src_gnss_yaw,
			      gnss_sample.time_us,                          // sample timestamp
			      measured_hdg,                                // observation
			      R_YAW,                                       // observation variance
			      wrap_pi(heading_pred - measured_hdg),        // innovation
			      heading_innov_var,                           // innovation variance
			      math::max(_params.heading_innov_gate, 1.f)); // innovation gate
}

void Ekf::fuseGnssYaw(float antenna_yaw_offset)
{
	auto &aid_src = _aid_src_gnss_yaw;

	if (aid_src.innovation_rejected) {
		_innov_check_fail_status.flags.reject_yaw = true;
		return;
	}

	if (!PX4_ISFINITE(antenna_yaw_offset)) {
		antenna_yaw_offset = 0.f;
	}

	float heading_pred;
	float heading_innov_var;
	VectorState H;

	// Note: we recompute innov and innov_var because it doesn't cost much more than just computing H
	// making a separate function just for H uses more flash space without reducing CPU load significantly
	sym::ComputeGnssYawPredInnovVarAndH(_state.vector(), P, antenna_yaw_offset, aid_src.observation_variance, FLT_EPSILON,
					    &heading_pred, &heading_innov_var, &H);

	// check if the innovation variance calculation is badly conditioned
	if (aid_src.innovation_variance < aid_src.observation_variance) {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_hdg = true;

		// we reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		ECL_ERR("GNSS yaw numerical error - covariance reset");
		stopGnssYawFusion();
		return;
	}

	_fault_status.flags.bad_hdg = false;
	_innov_check_fail_status.flags.reject_yaw = false;

	if ((fabsf(aid_src.test_ratio_filtered) > 0.2f)
	    && !_control_status.flags.in_air && isTimedOut(aid_src.time_last_fuse, (uint64_t)1e6)
	   ) {
		// A constant large signed test ratio is a sign of wrong gyro bias
		// Reset the yaw gyro variance to converge faster and avoid
		// being stuck on a previous bad estimate
		resetGyroBiasZCov();
	}

	// calculate the Kalman gains
	// only calculate gains for states we are using
	VectorState Kfusion = P * H / aid_src.innovation_variance;

	measurementUpdate(Kfusion, H, aid_src.observation_variance, aid_src.innovation);

	_fault_status.flags.bad_hdg = false;
	aid_src.fused = true;
	aid_src.time_last_fuse = _time_delayed_us;

	_time_last_heading_fuse = _time_delayed_us;
}

bool Ekf::resetYawToGnss(const float gnss_yaw, const float gnss_yaw_offset)
{
	// define the predicted antenna array vector and rotate into earth frame
	const Vector3f ant_vec_bf = {cosf(gnss_yaw_offset), sinf(gnss_yaw_offset), 0.0f};
	const Vector3f ant_vec_ef = _R_to_earth * ant_vec_bf;

	// check if antenna array vector is within 30 degrees of vertical and therefore unable to provide a reliable heading
	if (fabsf(ant_vec_ef(2)) > cosf(math::radians(30.0f)))  {
		return false;
	}

	// GNSS yaw measurement is already compensated for antenna offset in the driver
	const float measured_yaw = gnss_yaw;

	const float yaw_variance = sq(fmaxf(_params.gnss_heading_noise, 1.e-2f));
	resetQuatStateYaw(measured_yaw, yaw_variance);

	return true;
}

void Ekf::stopGnssYawFusion()
{
	if (_control_status.flags.gnss_yaw) {

		_control_status.flags.gnss_yaw = false;

		ECL_INFO("stopping GNSS yaw fusion");
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file gravity_fusion.cpp
 * Fuse observations from the gravity vector to constrain roll
 * and pitch (a la complementary filter).
 *
 * @author Daniel M. Sahu <danielmohansahu@gmail.com>
 */

#include "ekf.h"
#include <ekf_derivation/generated/compute_gravity_xyz_innov_var_and_hx.h>
#include <ekf_derivation/generated/compute_gravity_y_innov_var_and_h.h>
#include <ekf_derivation/generated/compute_gravity_z_innov_var_and_h.h>

#include <mathlib/mathlib.h>

void Ekf::controlGravityFusion(const imuSample &imu)
{
	// get raw accelerometer reading at delayed horizon and expected measurement noise (gaussian)
	const Vector3f measurement = Vector3f(imu.delta_vel / imu.delta_vel_dt - _state.accel_bias).unit();
	const float measurement_var = math::max(sq(_params.gravity_noise), sq(0.01f));

	const float upper_accel_limit = CONSTANTS_ONE_G * 1.1f;
	const float lower_accel_limit = CONSTANTS_ONE_G * 0.9f;
	const bool accel_lpf_norm_good = (_accel_magnitude_filt > lower_accel_limit)
					 && (_accel_magnitude_filt < upper_accel_limit);

	// fuse gravity observation if our overall acceleration isn't too big
	_control_status.flags.gravity_vector = (_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::GravityVector))
					       && (accel_lpf_norm_good || _control_status.flags.vehicle_at_rest)
					       && !isHorizontalAidingActive();

	// calculate kalman gains and innovation variances
	Vector3f innovation = _state.quat_nominal.rotateVectorInverse(Vector3f(0.f, 0.f, -1.f)) - measurement;
	Vector3f innovation_variance;
	const auto state_vector = _state.vector();
	VectorState H;
	sym::ComputeGravityXyzInnovVarAndHx(state_vector, P, measurement_var, &innovation_variance, &H);

	// fill estimator aid source status
	updateAidSourceStatus(_aid_src_gravity,
			      imu.time_us,                                                 // sample timestamp
			      measurement,                                                 // observation
			      Vector3f{measurement_var, measurement_var, measurement_var}, // observation variance
			      innovation,                                                  // innovation
			      innovation_variance,                                         // innovation variance
			      0.25f);                                                      // innovation gate

	// update the states and covariance using sequential fusion
	for (uint8_t index = 0; index <= 2; index++) {
		// Calculate Kalman gains and observation jacobians
		if (index == 0) {
			// everything was already computed above

		} else if (index == 1) {
			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			sym::ComputeGravityYInnovVarAndH(state_vector, P, measurement_var, &_aid_src_gravity.innovation_variance[index], &H);

			// recalculate innovation using the updated state
			_aid_src_gravity.innovation[index] = _state.quat_nominal.rotateVectorInverse(Vector3f(0.f, 0.f,
							     -1.f))(index) - measurement(index);

		} else if (index == 2) {
			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			sym::ComputeGravityZInnovVarAndH(state_vector, P, measurement_var, &_aid_src_gravity.innovation_variance[index], &H);

			// recalculate innovation using the updated state
			_aid_src_gravity.innovation[index] = _state.quat_nominal.rotateVectorInverse(Vector3f(0.f, 0.f,
							     -1.f))(index) - measurement(index);
		}

		VectorState K = P * H / _aid_src_gravity.innovation_variance[index];

		const bool accel_clipping = imu.delta_vel_clipping[0] || imu.delta_vel_clipping[1] || imu.delta_vel_clipping[2];

		if (_control_status.flags.gravity_vector && !_aid_src_gravity.innovation_rejected && !accel_clipping) {
			measurementUpdate(K, H, _aid_src_gravity.observation_variance[index], _aid_src_gravity.innovation[index]);
		}
	}

	_aid_src_gravity.fused = true;
	_aid_src_gravity.time_last_fuse = imu.time_us;
}
/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mag_control.cpp
 * Control functions for ekf magnetic field fusion
 */

#include "ekf.h"
#include <mathlib/mathlib.h>

#include <lib/world_magnetic_model/geo_mag_declination.h>

#include <ekf_derivation/generated/compute_mag_innov_innov_var_and_hx.h>

void Ekf::controlMagFusion(const imuSample &imu_sample)
{
	static constexpr const char *AID_SRC_NAME = "mag";
	estimator_aid_source3d_s &aid_src = _aid_src_mag;

	// reset the flight alignment flag so that the mag fields will be
	//  re-initialised next time we achieve flight altitude
	if (!_control_status_prev.flags.in_air && _control_status.flags.in_air) {
		_control_status.flags.mag_aligned_in_flight = false;
	}

	if (_params.mag_fusion_type == MagFuseType::NONE) {
		stopMagFusion();
		return;
	}

	magSample mag_sample;

	if (_mag_buffer && _mag_buffer->pop_first_older_than(imu_sample.time_us, &mag_sample)) {

		if (mag_sample.reset || (_mag_counter == 0)) {
			// sensor or calibration has changed, reset low pass filter
			_control_status.flags.mag_fault = false;

			_state.mag_B.zero();
			resetMagBiasCov();

			stopMagFusion();

			_mag_lpf.reset(mag_sample.mag);
			_mag_counter = 1;

			if (!_control_status.flags.in_air) {
				// Assume that a reset on the ground is caused by a change in mag calibration
				// Clear alignment to force a clean reset
				_control_status.flags.yaw_align = false;
			}

		} else {
			_mag_lpf.update(mag_sample.mag);
			_mag_counter++;
		}

		// check for WMM update periodically or if global origin has changed
		bool wmm_updated = false;

		if (global_origin().isInitialized()) {

			bool origin_newer_than_last_mag = (global_origin().getProjectionReferenceTimestamp() > aid_src.time_last_fuse);

			if (global_origin_valid()
			    && (origin_newer_than_last_mag || (isLocalHorizontalPositionValid() && isTimedOut(_wmm_mag_time_last_checked, 10e6)))
			   ) {
				if (updateWorldMagneticModel(_gpos.latitude_deg(), _gpos.longitude_deg())) {
					wmm_updated = true;
				}

				_wmm_mag_time_last_checked = _time_delayed_us;

			} else if (origin_newer_than_last_mag) {
				// use global origin to update WMM
				if (updateWorldMagneticModel(global_origin().getProjectionReferenceLat(),
							     global_origin().getProjectionReferenceLon())
				   ) {
					wmm_updated = true;
				}
			}
		}

		// if enabled, use knowledge of theoretical magnetic field vector to calculate a synthetic magnetomter Z component value.
		// this is useful if there is a lot of interference on the sensor measurement.
		if (_params.synthesize_mag_z && (_params.mag_declination_source & GeoDeclinationMask::USE_GEO_DECL)
		    && (_wmm_earth_field_gauss.isAllFinite() && _wmm_earth_field_gauss.longerThan(0.f))
		   ) {
			mag_sample.mag(2) = calculate_synthetic_mag_z_measurement(mag_sample.mag, _wmm_earth_field_gauss);

			_control_status.flags.synthetic_mag_z = true;

		} else {
			_control_status.flags.synthetic_mag_z = false;
		}

		// reset flags
		_fault_status.flags.bad_mag_x = false;
		_fault_status.flags.bad_mag_y = false;
		_fault_status.flags.bad_mag_z = false;

		// XYZ Measurement uncertainty. Need to consider timing errors for fast rotations
		const float R_MAG = math::max(sq(_params.mag_noise), sq(0.01f));

		// calculate intermediate variables used for X axis innovation variance, observation Jacobians and Kalman gains
		Vector3f mag_innov;
		Vector3f innov_var;

		// Observation jacobian and Kalman gain vectors
		VectorState H;
		sym::ComputeMagInnovInnovVarAndHx(_state.vector(), P, mag_sample.mag, R_MAG, FLT_EPSILON, &mag_innov, &innov_var, &H);

		updateAidSourceStatus(aid_src,
				      mag_sample.time_us,                      // sample timestamp
				      mag_sample.mag,                          // observation
				      Vector3f(R_MAG, R_MAG, R_MAG),           // observation variance
				      mag_innov,                               // innovation
				      innov_var,                               // innovation variance
				      math::max(_params.mag_innov_gate, 1.f)); // innovation gate

		// Perform an innovation consistency check and report the result
		_innov_check_fail_status.flags.reject_mag_x = (aid_src.test_ratio[0] > 1.f);
		_innov_check_fail_status.flags.reject_mag_y = (aid_src.test_ratio[1] > 1.f);
		_innov_check_fail_status.flags.reject_mag_z = (aid_src.test_ratio[2] > 1.f);

		// determine if we should use mag fusion
		bool continuing_conditions_passing = ((_params.mag_fusion_type == MagFuseType::INIT)
						      || (_params.mag_fusion_type == MagFuseType::AUTO)
						      || (_params.mag_fusion_type == MagFuseType::HEADING))
						     && _control_status.flags.tilt_align
						     && (_control_status.flags.yaw_align || (!_control_status.flags.ev_yaw && !_control_status.flags.yaw_align))
						     && mag_sample.mag.longerThan(0.f)
						     && mag_sample.mag.isAllFinite();

		const bool starting_conditions_passing = continuing_conditions_passing
				&& checkMagField(mag_sample.mag)
				&& (_mag_counter > 3) // wait until we have more than a few samples through the filter
				&& (_control_status.flags.yaw_align == _control_status_prev.flags.yaw_align) // no yaw alignment change this frame
				&& (_state_reset_status.reset_count.quat ==
				    _state_reset_count_prev.quat) // don't allow starting on same frame as yaw reset
				&& isNewestSampleRecent(_time_last_mag_buffer_push, MAG_MAX_INTERVAL);

		checkMagHeadingConsistency(mag_sample);

		{
			const bool mag_consistent_or_no_ne_aiding = _control_status.flags.mag_heading_consistent || !isNorthEastAidingActive();
			const bool common_conditions_passing = _control_status.flags.mag
							       && ((_control_status.flags.yaw_align && mag_consistent_or_no_ne_aiding)
									       || (!_control_status.flags.ev_yaw && !_control_status.flags.yaw_align))
							       && !_control_status.flags.mag_fault
							       && !_control_status.flags.mag_field_disturbed
							       && !_control_status.flags.ev_yaw
							       && !_control_status.flags.gnss_yaw;

			_control_status.flags.mag_3D = common_conditions_passing
						       && (_params.mag_fusion_type == MagFuseType::AUTO)
						       && _control_status.flags.mag_aligned_in_flight;

			_control_status.flags.mag_hdg = common_conditions_passing
							&& ((_params.mag_fusion_type == MagFuseType::HEADING)
							    || (_params.mag_fusion_type == MagFuseType::AUTO && !_control_status.flags.mag_3D));
		}

		// TODO: allow clearing mag_fault if mag_3d is good?

		if (_control_status.flags.mag_3D && !_control_status_prev.flags.mag_3D) {
			ECL_INFO("starting mag 3D fusion");

		} else if (!_control_status.flags.mag_3D && _control_status_prev.flags.mag_3D) {
			ECL_INFO("stopping mag 3D fusion");
		}

		// if we are using 3-axis magnetometer fusion, but without external NE aiding,
		// then the declination must be fused as an observation to prevent long term heading drift
		const bool no_ne_aiding_or_not_moving = !isNorthEastAidingActive() || _control_status.flags.vehicle_at_rest;
		_control_status.flags.mag_dec = _control_status.flags.mag && no_ne_aiding_or_not_moving;

		if (_control_status.flags.mag) {

			if (continuing_conditions_passing && _control_status.flags.yaw_align) {

				if (checkHaglYawResetReq() || (wmm_updated && no_ne_aiding_or_not_moving)) {
					ECL_INFO("reset to %s", AID_SRC_NAME);
					resetMagStates(_mag_lpf.getState(), _control_status.flags.mag_hdg || _control_status.flags.mag_3D);
					aid_src.time_last_fuse = imu_sample.time_us;

				} else {
					// The normal sequence is to fuse the magnetometer data first before fusing
					// declination angle at a higher uncertainty to allow some learning of
					// declination angle over time.
					const bool update_all_states = _control_status.flags.mag_3D || _control_status.flags.mag_hdg;
					const bool update_tilt = _control_status.flags.mag_3D;
					fuseMag(mag_sample.mag, R_MAG, H, aid_src, update_all_states, update_tilt);

					// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
					if (update_all_states && update_tilt) {
						_fault_status.flags.bad_mag_x = (aid_src.innovation_variance[0] < aid_src.observation_variance[0]);
						_fault_status.flags.bad_mag_y = (aid_src.innovation_variance[1] < aid_src.observation_variance[1]);
						_fault_status.flags.bad_mag_z = (aid_src.innovation_variance[2] < aid_src.observation_variance[2]);
					}

					if (_control_status.flags.mag_dec) {

						// observation variance (rad**2)
						const float R_DECL = sq(0.5f);

						if ((_params.mag_declination_source & GeoDeclinationMask::USE_GEO_DECL)
						    && PX4_ISFINITE(_wmm_declination_rad)
						   ) {
							// using declination from the world magnetic model
							fuseDeclination(_wmm_declination_rad, 0.5f, update_all_states);

						} else if ((_params.mag_declination_source & GeoDeclinationMask::SAVE_GEO_DECL)
							   && PX4_ISFINITE(_params.mag_declination_deg) && (fabsf(_params.mag_declination_deg) > 0.f)
							  ) {
							// using previously saved declination
							fuseDeclination(math::radians(_params.mag_declination_deg), R_DECL, update_all_states);

						} else {
							// if there is no aiding coming from an inertial frame we need to fuse some declination
							// even if we don't know the value, it's better to fuse 0 than nothing
							float declination_rad = 0.f;
							fuseDeclination(declination_rad, R_DECL);
						}
					}
				}

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.reset_timeout_max);

				if (is_fusion_failing) {
					if (no_ne_aiding_or_not_moving) {
						ECL_WARN("%s fusion failing, resetting", AID_SRC_NAME);
						resetMagStates(_mag_lpf.getState(), _control_status.flags.mag_hdg || _control_status.flags.mag_3D);
						aid_src.time_last_fuse = imu_sample.time_us;

					} else {
						ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);
						stopMagFusion();
					}
				}

			} else {
				// Stop fusion but do not declare it faulty
				ECL_DEBUG("stopping %s fusion, continuing conditions no longer passing", AID_SRC_NAME);
				stopMagFusion();
			}

		} else {
			if (starting_conditions_passing) {

				// activate fusion, reset mag states and initialize variance if first init or in flight reset
				if (!_control_status.flags.yaw_align
				    || wmm_updated
				    || !_state.mag_I.longerThan(0.f)
				    || (getStateVariance<State::mag_I>().min() < kMagVarianceMin)
				    || (getStateVariance<State::mag_B>().min() < kMagVarianceMin)
				   ) {
					ECL_INFO("starting %s fusion, resetting states", AID_SRC_NAME);

					bool reset_heading = !_control_status.flags.yaw_align;

					resetMagStates(_mag_lpf.getState(), reset_heading);
					aid_src.time_last_fuse = imu_sample.time_us;

					if (reset_heading) {
						_control_status.flags.yaw_align = true;
						resetAidSourceStatusZeroInnovation(aid_src);
					}

					_control_status.flags.mag = true;

				} else {
					if (fuseMag(mag_sample.mag, R_MAG, H, aid_src)) {
						ECL_INFO("starting %s fusion", AID_SRC_NAME);
						_control_status.flags.mag = true;
					}
				}
			}
		}

	} else if (!isNewestSampleRecent(_time_last_mag_buffer_push, 2 * MAG_MAX_INTERVAL)) {
		// No data anymore. Stop until it comes back.
		stopMagFusion();
	}
}

void Ekf::stopMagFusion()
{
	if (_control_status.flags.mag) {
		ECL_INFO("stopping mag fusion");

		resetMagEarthCov();
		resetMagBiasCov();

		if (_control_status.flags.yaw_align && (_control_status.flags.mag_3D || _control_status.flags.mag_hdg)) {
			// reset yaw alignment from mag unless yaw is observable through North-East aiding

			if (!isNorthEastAidingActive()) {
				_control_status.flags.yaw_align = false;
			}
		}

		_control_status.flags.mag = false;
		_control_status.flags.mag_dec = false;

		if (_control_status.flags.mag_3D) {
			ECL_INFO("stopping mag 3D fusion");
			_control_status.flags.mag_3D = false;
		}

		if (_control_status.flags.mag_hdg) {
			ECL_INFO("stopping mag heading fusion");
			_control_status.flags.mag_hdg = false;
			_fault_status.flags.bad_hdg = false;
		}

		_control_status.flags.mag_aligned_in_flight = false;

		_fault_status.flags.bad_mag_x = false;
		_fault_status.flags.bad_mag_y = false;
		_fault_status.flags.bad_mag_z = false;

		_fault_status.flags.bad_mag_decl = false;
	}
}

bool Ekf::checkHaglYawResetReq() const
{
#if defined(CONFIG_EKF2_TERRAIN)

	// We need to reset the yaw angle after climbing away from the ground to enable
	// recovery from ground level magnetic interference.
	if (_control_status.flags.in_air && _control_status.flags.yaw_align && !_control_status.flags.mag_aligned_in_flight) {
		// Check if height has increased sufficiently to be away from ground magnetic anomalies
		// and request a yaw reset if not already requested.
		static constexpr float mag_anomalies_max_hagl = 1.5f;
		const bool above_mag_anomalies = (getTerrainVPos() + _gpos.altitude()) > mag_anomalies_max_hagl;
		return above_mag_anomalies;
	}

#endif // CONFIG_EKF2_TERRAIN

	return false;
}

void Ekf::resetMagStates(const Vector3f &mag, bool reset_heading)
{
	// reinit mag states
	const Vector3f mag_I_before_reset = _state.mag_I;
	const Vector3f mag_B_before_reset = _state.mag_B;

	static constexpr float kMagEarthMinGauss = 0.01f; // minimum difference in mag earth field strength for reset (Gauss)

	// if world magnetic model (inclination, declination, strength) available then use it to reset mag states
	if (_wmm_earth_field_gauss.longerThan(0.f) && _wmm_earth_field_gauss.isAllFinite()) {
		// use expected earth field to reset states

		// mag_I: reset, skipped if negligible change in state
		const Vector3f mag_I = _wmm_earth_field_gauss;
		bool mag_I_reset = false;

		if ((_state.mag_I - mag_I).longerThan(kMagEarthMinGauss)) {
			_state.mag_I = mag_I;
			resetMagEarthCov();
			mag_I_reset = true;
		}

		// mag_B: reset, skipped if mag_I didn't change
		if (!reset_heading && _control_status.flags.yaw_align) {
			if (mag_I_reset) {
				// mag_B: reset using WMM
				const Dcmf R_to_body = quatToInverseRotMat(_state.quat_nominal);
				_state.mag_B = mag - (R_to_body * _wmm_earth_field_gauss);
				resetMagBiasCov();
			} // otherwise keep existing mag_B state (!mag_I_reset)

		} else {
			_state.mag_B.zero();
			resetMagBiasCov();
		}

		if (reset_heading) {
			resetMagHeading(mag);
		}

	} else {
		// mag_B: reset
		_state.mag_B.zero();
		resetMagBiasCov();

		// Use the magnetometer measurement to reset the heading
		if (reset_heading) {
			resetMagHeading(mag);
		}

		// mag_I: use the last magnetometer measurement to reset the field states
		const Vector3f mag_I = _R_to_earth * mag;

		if ((_state.mag_I - mag_I).longerThan(kMagEarthMinGauss)) {
			_state.mag_I = mag_I;
			resetMagEarthCov();
		}
	}

	if ((_state.mag_I - mag_I_before_reset).longerThan(0.f)) {
		ECL_INFO("resetting mag I [%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f]",
			 (double)mag_I_before_reset(0), (double)mag_I_before_reset(1), (double)mag_I_before_reset(2),
			 (double)_state.mag_I(0), (double)_state.mag_I(1), (double)_state.mag_I(2));
	}

	if ((_state.mag_B - mag_B_before_reset).longerThan(0.f)) {
		ECL_INFO("resetting mag B [%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f]",
			 (double)mag_B_before_reset(0), (double)mag_B_before_reset(1), (double)mag_B_before_reset(2),
			 (double)_state.mag_B(0), (double)_state.mag_B(1), (double)_state.mag_B(2));
	}

	// record the start time for the magnetic field alignment
	if (_control_status.flags.in_air) {
		_control_status.flags.mag_aligned_in_flight = true;
		_flt_mag_align_start_time = _time_delayed_us;
	}
}

void Ekf::checkMagHeadingConsistency(const magSample &mag_sample)
{
	// use mag bias if variance good
	Vector3f mag_bias{0.f, 0.f, 0.f};
	const Vector3f mag_bias_var = getMagBiasVariance();

	if ((mag_bias_var.min() > 0.f) && (mag_bias_var.max() <= sq(_params.mag_noise))) {
		mag_bias = _state.mag_B;
	}

	// calculate mag heading
	// Rotate the measurements into earth frame using the zero yaw angle
	const Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);

	// the angle of the projection onto the horizontal gives the yaw angle
	// calculate the yaw innovation and wrap to the interval between +-pi
	const Vector3f mag_earth_pred = R_to_earth * (mag_sample.mag - mag_bias);
	const float declination = getMagDeclination();
	const float measured_hdg = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + declination;

	if (_control_status.flags.yaw_align) {
		const float innovation = wrap_pi(getEulerYaw(_R_to_earth) - measured_hdg);
		_mag_heading_innov_lpf.update(innovation);

	} else {
		_mag_heading_innov_lpf.reset(0.f);
	}

	if (fabsf(_mag_heading_innov_lpf.getState()) < _params.mag_heading_noise) {
		// Check if there has been enough change in horizontal velocity to make yaw observable

		if (isNorthEastAidingActive() && (_accel_horiz_lpf.getState().longerThan(_params.mag_acc_gate))) {
			// yaw angle must be observable to consider consistency
			_control_status.flags.mag_heading_consistent = true;
		}

	} else {
		_control_status.flags.mag_heading_consistent = false;
	}
}

bool Ekf::checkMagField(const Vector3f &mag_sample)
{
	_control_status.flags.mag_field_disturbed = false;

	if (_params.mag_check == 0) {
		// skip all checks
		return true;
	}

	bool is_check_failing = false;
	_mag_strength = mag_sample.length();

	if (_params.mag_check & static_cast<int32_t>(MagCheckMask::STRENGTH)) {
		if (PX4_ISFINITE(_wmm_field_strength_gauss)) {
			if (!isMeasuredMatchingExpected(_mag_strength, _wmm_field_strength_gauss, _params.mag_check_strength_tolerance_gs)) {
				_control_status.flags.mag_field_disturbed = true;
				is_check_failing = true;
			}

		} else if (_params.mag_check & static_cast<int32_t>(MagCheckMask::FORCE_WMM)) {
			is_check_failing = true;

		} else {
			constexpr float average_earth_mag_field_strength = 0.45f; // Gauss
			constexpr float average_earth_mag_gate_size = 0.40f; // +/- Gauss

			if (!isMeasuredMatchingExpected(mag_sample.length(), average_earth_mag_field_strength, average_earth_mag_gate_size)) {
				_control_status.flags.mag_field_disturbed = true;
				is_check_failing = true;
			}
		}
	}

	const Vector3f mag_earth = _R_to_earth * mag_sample;
	_mag_inclination = asinf(mag_earth(2) / fmaxf(mag_earth.norm(), 1e-4f));

	if (_params.mag_check & static_cast<int32_t>(MagCheckMask::INCLINATION)) {
		if (PX4_ISFINITE(_wmm_inclination_rad)) {
			const float inc_tol_rad = radians(_params.mag_check_inclination_tolerance_deg);
			const float inc_error_rad = wrap_pi(_mag_inclination - _wmm_inclination_rad);

			if (fabsf(inc_error_rad) > inc_tol_rad) {
				_control_status.flags.mag_field_disturbed = true;
				is_check_failing = true;
			}

		} else if (_params.mag_check & static_cast<int32_t>(MagCheckMask::FORCE_WMM)) {
			is_check_failing = true;

		} else {
			// No check possible when the global position is unknown
			// TODO: add parameter to remember the inclination between boots
		}
	}

	if (is_check_failing || (_time_last_mag_check_failing == 0)) {
		_time_last_mag_check_failing = _time_delayed_us;
	}

	return ((_time_delayed_us - _time_last_mag_check_failing) > (uint64_t)_min_mag_health_time_us);
}

bool Ekf::isMeasuredMatchingExpected(const float measured, const float expected, const float gate)
{
	return (measured >= expected - gate)
	       && (measured <= expected + gate);
}

void Ekf::resetMagHeading(const Vector3f &mag)
{
	// use mag bias if variance good (unless configured for HEADING only)
	Vector3f mag_bias{0.f, 0.f, 0.f};
	const Vector3f mag_bias_var = getMagBiasVariance();

	if ((mag_bias_var.min() > 0.f) && (mag_bias_var.max() <= sq(_params.mag_noise))) {
		mag_bias = _state.mag_B;
	}

	// calculate mag heading
	// rotate the magnetometer measurements into earth frame using a zero yaw angle
	const Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);

	// the angle of the projection onto the horizontal gives the yaw angle
	const Vector3f mag_earth_pred = R_to_earth * (mag - mag_bias);

	// calculate the observed yaw angle and yaw variance
	const float declination = getMagDeclination();
	float yaw_new = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + declination;
	float yaw_new_variance = math::max(sq(_params.mag_heading_noise), sq(0.01f));

	ECL_INFO("reset mag heading %.3f -> %.3f rad (bias:[%.3f, %.3f, %.3f], declination:%.1f)",
		 (double)getEulerYaw(_R_to_earth), (double)yaw_new,
		 (double)mag_bias(0), (double)mag_bias(1), (double)mag_bias(2),
		 (double)declination);

	// update quaternion states and corresponding covarainces
	resetQuatStateYaw(yaw_new, yaw_new_variance);

	_time_last_heading_fuse = _time_delayed_us;

	_mag_heading_innov_lpf.reset(0.f);
	_control_status.flags.mag_heading_consistent = true;
}

float Ekf::getMagDeclination()
{
	// set source of magnetic declination for internal use
	if (_control_status.flags.mag_aligned_in_flight) {
		// Use value consistent with earth field state
		return atan2f(_state.mag_I(1), _state.mag_I(0));

	} else if ((_params.mag_declination_source & GeoDeclinationMask::USE_GEO_DECL)
		   && PX4_ISFINITE(_wmm_declination_rad)
		  ) {
		// if available use value returned by geo library
		return _wmm_declination_rad;

	} else if ((_params.mag_declination_source & GeoDeclinationMask::SAVE_GEO_DECL)
		   && PX4_ISFINITE(_params.mag_declination_deg) && (fabsf(_params.mag_declination_deg) > 0.f)
		  ) {
		// using saved mag declination
		return math::radians(_params.mag_declination_deg);
	}

	// otherwise unavailable
	return 0.f;
}

bool Ekf::updateWorldMagneticModel(const double latitude_deg, const double longitude_deg)
{
	// set the magnetic field data returned by the geo library using the current GPS position
	const float declination_rad = math::radians(get_mag_declination_degrees(latitude_deg, longitude_deg));
	const float inclination_rad = math::radians(get_mag_inclination_degrees(latitude_deg, longitude_deg));
	const float strength_gauss = get_mag_strength_gauss(latitude_deg, longitude_deg);

	if (PX4_ISFINITE(declination_rad) && PX4_ISFINITE(inclination_rad) && PX4_ISFINITE(strength_gauss)) {

		const bool declination_changed = (fabsf(declination_rad - _wmm_declination_rad) > math::radians(1.f));
		const bool inclination_changed = (fabsf(inclination_rad - _wmm_inclination_rad) > math::radians(1.f));
		const bool strength_changed = (fabsf(strength_gauss - _wmm_field_strength_gauss) > 0.01f);

		if (!PX4_ISFINITE(_wmm_declination_rad)
		    || !PX4_ISFINITE(_wmm_inclination_rad)
		    || !PX4_ISFINITE(_wmm_field_strength_gauss)
		    || !_wmm_earth_field_gauss.longerThan(0.f)
		    || !_wmm_earth_field_gauss.isAllFinite()
		    || declination_changed
		    || inclination_changed
		    || strength_changed
		   ) {

			ECL_DEBUG("WMM declination updated %.3f -> %.3f deg (lat=%.6f, lon=%.6f)",
				  (double)math::degrees(_wmm_declination_rad), (double)math::degrees(declination_rad),
				  (double)latitude_deg, (double)longitude_deg
				 );

			_wmm_declination_rad = declination_rad;
			_wmm_inclination_rad = inclination_rad;
			_wmm_field_strength_gauss = strength_gauss;

			_wmm_earth_field_gauss = Dcmf(Eulerf(0, -inclination_rad, declination_rad)) * Vector3f(strength_gauss, 0, 0);

			return true;
		}
	}

	return false;
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mag_fusion.cpp
 * Magnetometer fusion methods.
 * Equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <ekf_derivation/generated/compute_mag_y_innov_var_and_h.h>
#include <ekf_derivation/generated/compute_mag_z_innov_var_and_h.h>

#include <ekf_derivation/generated/compute_mag_declination_pred_innov_var_and_h.h>

#include <mathlib/mathlib.h>

bool Ekf::fuseMag(const Vector3f &mag, const float R_MAG, VectorState &H, estimator_aid_source3d_s &aid_src,
		  bool update_all_states, bool update_tilt)
{
	// if any axis failed, abort the mag fusion
	if (aid_src.innovation_rejected) {
		return false;
	}

	const auto state_vector = _state.vector();

	// update the states and covariance using sequential fusion of the magnetometer components
	for (uint8_t index = 0; index <= 2; index++) {
		// Calculate Kalman gains and observation jacobians
		if (index == 0) {
			// everything was already computed

		} else if (index == 1) {
			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			sym::ComputeMagYInnovVarAndH(state_vector, P, R_MAG, FLT_EPSILON, &aid_src.innovation_variance[index], &H);

			// recalculate innovation using the updated state
			aid_src.innovation[index] = _state.quat_nominal.rotateVectorInverse(_state.mag_I)(index) + _state.mag_B(index) - mag(
							    index);

		} else if (index == 2) {
			// we do not fuse synthesized magnetomter measurements when doing 3D fusion
			if (_control_status.flags.synthetic_mag_z) {
				continue;
			}

			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			sym::ComputeMagZInnovVarAndH(state_vector, P, R_MAG, FLT_EPSILON, &aid_src.innovation_variance[index], &H);

			// recalculate innovation using the updated state
			aid_src.innovation[index] = _state.quat_nominal.rotateVectorInverse(_state.mag_I)(index) + _state.mag_B(index) - mag(
							    index);
		}

		if (aid_src.innovation_variance[index] < R_MAG) {
			ECL_ERR("mag numerical error covariance reset");

			// we need to re-initialise covariances and abort this fusion step
			if (update_all_states) {
				resetQuatCov(_params.mag_heading_noise);
			}

			resetMagEarthCov();
			resetMagBiasCov();

			return false;
		}

		VectorState Kfusion = P * H / aid_src.innovation_variance[index];

		if (update_all_states) {
			if (!update_tilt) {
				Kfusion(State::quat_nominal.idx + 0) = 0.f;
				Kfusion(State::quat_nominal.idx + 1) = 0.f;
			}

		} else {
			// zero non-mag Kalman gains if not updating all states

			// copy mag_I and mag_B Kalman gains
			const Vector3f K_mag_I = Kfusion.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0);
			const Vector3f K_mag_B = Kfusion.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0);

			// zero all Kalman gains, then restore mag
			Kfusion.setZero();
			Kfusion.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0) = K_mag_I;
			Kfusion.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0) = K_mag_B;
		}

		measurementUpdate(Kfusion, H, aid_src.observation_variance[index], aid_src.innovation[index]);
	}

	_fault_status.flags.bad_mag_x = false;
	_fault_status.flags.bad_mag_y = false;
	_fault_status.flags.bad_mag_z = false;

	aid_src.fused = true;
	aid_src.time_last_fuse = _time_delayed_us;

	if (update_all_states) {
		_time_last_heading_fuse = _time_delayed_us;
	}

	return true;
}

bool Ekf::fuseDeclination(float decl_measurement_rad, float R, bool update_all_states)
{
	VectorState H;
	float decl_pred;
	float innovation_variance;

	sym::ComputeMagDeclinationPredInnovVarAndH(_state.vector(), P, R, FLT_EPSILON,
			&decl_pred, &innovation_variance, &H);

	const float innovation = wrap_pi(decl_pred - decl_measurement_rad);

	if (innovation_variance < R) {
		// variance calculation is badly conditioned
		_fault_status.flags.bad_mag_decl = true;
		return false;
	}

	// Calculate the Kalman gains
	VectorState Kfusion = P * H / innovation_variance;

	if (!update_all_states) {
		// zero non-mag Kalman gains if not updating all states

		// copy mag_I and mag_B Kalman gains
		const Vector3f K_mag_I = Kfusion.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0);
		const Vector3f K_mag_B = Kfusion.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0);

		// zero all Kalman gains, then restore mag
		Kfusion.setZero();
		Kfusion.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0) = K_mag_I;
		Kfusion.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0) = K_mag_B;
	}

	measurementUpdate(Kfusion, H, R, innovation);

	_fault_status.flags.bad_mag_decl = false;

	return true;
}

float Ekf::calculate_synthetic_mag_z_measurement(const Vector3f &mag_meas, const Vector3f &mag_earth_predicted)
{
	// theoretical magnitude of the magnetometer Z component value given X and Y sensor measurement and our knowledge
	// of the earth magnetic field vector at the current location
	const float mag_z_abs = sqrtf(math::max(sq(mag_earth_predicted.length()) - sq(mag_meas(0)) - sq(mag_meas(1)), 0.0f));

	// calculate sign of synthetic magnetomter Z component based on the sign of the predicted magnetometer Z component
	const float mag_z_body_pred = mag_earth_predicted.dot(_R_to_earth.col(2));

	return (mag_z_body_pred < 0) ? -mag_z_abs : mag_z_abs;
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file optical_flow_control.cpp
 * Control functions for optical flow fusion
 */

#include "ekf.h"

#include <ekf_derivation/generated/compute_flow_xy_innov_var_and_hx.h>

void Ekf::controlOpticalFlowFusion(const imuSample &imu_delayed)
{
	if (!_flow_buffer || (_params.flow_ctrl != 1)) {
		stopFlowFusion();
		return;
	}

	VectorState H;

	// New optical flow data is available and is ready to be fused when the midpoint of the sample falls behind the fusion time horizon
	if (_flow_buffer->pop_first_older_than(imu_delayed.time_us, &_flow_sample_delayed)) {

		// flow gyro has opposite sign convention
		_ref_body_rate = -(imu_delayed.delta_ang / imu_delayed.delta_ang_dt - getGyroBias());

		// ensure valid flow sample gyro rate before proceeding
		switch (static_cast<FlowGyroSource>(_params.flow_gyro_src)) {
		default:

		/* FALLTHROUGH */
		case FlowGyroSource::Auto:
			if (!PX4_ISFINITE(_flow_sample_delayed.gyro_rate(0)) || !PX4_ISFINITE(_flow_sample_delayed.gyro_rate(1))) {
				_flow_sample_delayed.gyro_rate = _ref_body_rate;
			}

			if (!PX4_ISFINITE(_flow_sample_delayed.gyro_rate(2))) {
				// Some flow modules only provide X ind Y angular rates. If this is the case, complete the vector with our own Z gyro
				_flow_sample_delayed.gyro_rate(2) = _ref_body_rate(2);
			}

			break;

		case FlowGyroSource::Internal:
			_flow_sample_delayed.gyro_rate = _ref_body_rate;
			break;
		}

		const flowSample &flow_sample = _flow_sample_delayed;

		const int32_t min_quality = _control_status.flags.in_air
					    ? _params.flow_qual_min
					    : _params.flow_qual_min_gnd;

		const bool is_quality_good = (flow_sample.quality >= min_quality);

		bool is_tilt_good = true;

#if defined(CONFIG_EKF2_RANGE_FINDER)
		is_tilt_good = (_R_to_earth(2, 2) > _params.range_cos_max_tilt);
#endif // CONFIG_EKF2_RANGE_FINDER

		calcOptFlowBodyRateComp(flow_sample);

		// calculate optical LOS rates using optical flow rates that have had the body angular rate contribution removed
		// correct for gyro bias errors in the data used to do the motion compensation
		// Note the sign convention used: A positive LOS rate is a RH rotation of the scene about that axis.
		const Vector3f flow_gyro_corrected = flow_sample.gyro_rate - _flow_gyro_bias;
		const Vector2f flow_compensated = flow_sample.flow_rate - flow_gyro_corrected.xy();

		// calculate the optical flow observation variance
		const float R_LOS = calcOptFlowMeasVar(flow_sample);

		const float epsilon = 1e-3f;
		Vector2f innov_var;
		sym::ComputeFlowXyInnovVarAndHx(_state.vector(), P, R_LOS, epsilon, &innov_var, &H);

		// run the innovation consistency check and record result
		updateAidSourceStatus(_aid_src_optical_flow,
				      flow_sample.time_us,                                 // sample timestamp
				      flow_compensated,                                    // observation
				      Vector2f{R_LOS, R_LOS},                              // observation variance
				      predictFlow(flow_gyro_corrected) - flow_compensated, // innovation
				      innov_var,                                           // innovation variance
				      math::max(_params.flow_innov_gate, 1.f));            // innovation gate

		// logging
		_flow_rate_compensated = flow_compensated;

		// compute the velocities in body and local frames from corrected optical flow measurement for logging only
		const float range = predictFlowRange();
		_flow_vel_body(0) = -flow_compensated(1) * range;
		_flow_vel_body(1) =  flow_compensated(0) * range;

		if (_flow_counter == 0) {
			_flow_vel_body_lpf.reset(_flow_vel_body);
			_flow_counter = 1;

		} else {

			_flow_vel_body_lpf.update(_flow_vel_body);
			_flow_counter++;
		}

		// Check if we are in-air and require optical flow to control position drift
		bool is_flow_required = _control_status.flags.in_air
					&& (_control_status.flags.inertial_dead_reckoning // is doing inertial dead-reckoning so must constrain drift urgently
					    || isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow));

		const bool is_within_sensor_dist = (getHagl() >= _flow_min_distance) && (getHagl() <= _flow_max_distance);

		const bool is_magnitude_good = flow_sample.flow_rate.isAllFinite()
					       && !flow_sample.flow_rate.longerThan(_flow_max_rate)
					       && !flow_compensated.longerThan(_flow_max_rate);

		const bool continuing_conditions_passing = (_params.flow_ctrl == 1)
				&& _control_status.flags.tilt_align
				&& is_within_sensor_dist;

		const bool starting_conditions_passing = continuing_conditions_passing
				&& is_quality_good
				&& is_magnitude_good
				&& is_tilt_good
				&& (_flow_counter > 10)
				&& (isTerrainEstimateValid() || isHorizontalAidingActive())
				&& isTimedOut(_aid_src_optical_flow.time_last_fuse, (uint64_t)2e6); // Prevent rapid switching

		// If the height is relative to the ground, terrain height cannot be observed.
		_control_status.flags.opt_flow_terrain = _control_status.flags.opt_flow && !(_height_sensor_ref == HeightSensor::RANGE);

		if (_control_status.flags.opt_flow) {
			if (continuing_conditions_passing) {

				if (is_quality_good && is_magnitude_good && is_tilt_good) {
					fuseOptFlow(H, _control_status.flags.opt_flow_terrain);
				}

				// handle the case when we have optical flow, are reliant on it, but have not been using it for an extended period
				if (isTimedOut(_aid_src_optical_flow.time_last_fuse, _params.no_aid_timeout_max)) {
					if (is_flow_required && is_quality_good && is_magnitude_good) {
						resetFlowFusion(flow_sample);

						if (_control_status.flags.opt_flow_terrain && !isTerrainEstimateValid()) {
							resetTerrainToFlow();
						}

					} else {
						stopFlowFusion();
					}
				}

			} else {
				stopFlowFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// If the height is relative to the ground, terrain height cannot be observed.
				_control_status.flags.opt_flow_terrain = (_height_sensor_ref != HeightSensor::RANGE);

				if (isHorizontalAidingActive()) {
					if (fuseOptFlow(H, _control_status.flags.opt_flow_terrain)) {
						ECL_INFO("starting optical flow");
						_control_status.flags.opt_flow = true;

					} else if (_control_status.flags.opt_flow_terrain && !_control_status.flags.rng_terrain) {
						ECL_INFO("starting optical flow, resetting terrain");
						resetTerrainToFlow();
						_control_status.flags.opt_flow = true;
					}

				} else {
					if (isTerrainEstimateValid() || (_height_sensor_ref == HeightSensor::RANGE)) {
						ECL_INFO("starting optical flow, resetting");
						resetFlowFusion(flow_sample);
						_control_status.flags.opt_flow = true;

					} else if (_control_status.flags.opt_flow_terrain) {
						ECL_INFO("starting optical flow, resetting terrain");
						resetTerrainToFlow();
						_control_status.flags.opt_flow = true;
					}
				}

				_control_status.flags.opt_flow_terrain = _control_status.flags.opt_flow && !(_height_sensor_ref == HeightSensor::RANGE);
			}
		}

	} else if (_control_status.flags.opt_flow && isTimedOut(_flow_sample_delayed.time_us, _params.reset_timeout_max)) {
		stopFlowFusion();
	}
}

void Ekf::resetFlowFusion(const flowSample &flow_sample)
{
	ECL_INFO("reset velocity to flow");
	_information_events.flags.reset_vel_to_flow = true;

	const float flow_vel_var = sq(predictFlowRange()) * calcOptFlowMeasVar(flow_sample);
	resetHorizontalVelocityTo(getFilteredFlowVelNE(), flow_vel_var);

	resetAidSourceStatusZeroInnovation(_aid_src_optical_flow);

	_innov_check_fail_status.flags.reject_optflow_X = false;
	_innov_check_fail_status.flags.reject_optflow_Y = false;
}

void Ekf::resetTerrainToFlow()
{
	ECL_INFO("reset hagl to flow");

	// TODO: use the flow data
	const float new_terrain = -_gpos.altitude() + _params.rng_gnd_clearance;
	const float delta_terrain = new_terrain - _state.terrain;
	_state.terrain = new_terrain;
	P.uncorrelateCovarianceSetVariance<State::terrain.dof>(State::terrain.idx, 100.f);

	resetAidSourceStatusZeroInnovation(_aid_src_optical_flow);

	_innov_check_fail_status.flags.reject_optflow_X = false;
	_innov_check_fail_status.flags.reject_optflow_Y = false;


	// record the state change
	if (_state_reset_status.reset_count.hagl == _state_reset_count_prev.hagl) {
		_state_reset_status.hagl_change = delta_terrain;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.hagl_change += delta_terrain;
	}

	_state_reset_status.reset_count.hagl++;
}

void Ekf::stopFlowFusion()
{
	if (_control_status.flags.opt_flow) {
		ECL_INFO("stopping optical flow fusion");
		_control_status.flags.opt_flow = false;
		_control_status.flags.opt_flow_terrain = false;

		_fault_status.flags.bad_optflow_X = false;
		_fault_status.flags.bad_optflow_Y = false;

		_innov_check_fail_status.flags.reject_optflow_X = false;
		_innov_check_fail_status.flags.reject_optflow_Y = false;

		_flow_counter = 0;
	}
}

void Ekf::calcOptFlowBodyRateComp(const flowSample &flow_sample)
{
	// calculate the bias estimate using a combined LPF and spike filter
	_flow_gyro_bias = 0.99f * _flow_gyro_bias
			  + 0.01f * matrix::constrain(flow_sample.gyro_rate - _ref_body_rate, -0.1f, 0.1f);
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file optical_flow_fusion.cpp
 */

#include "ekf.h"

#include <mathlib/mathlib.h>
#include <float.h>
#include <ekf_derivation/generated/compute_flow_xy_innov_var_and_hx.h>
#include <ekf_derivation/generated/compute_flow_y_innov_var_and_h.h>

bool Ekf::fuseOptFlow(VectorState &H, const bool update_terrain)
{
	const auto state_vector = _state.vector();

	// if either axis fails we abort the fusion
	if (_aid_src_optical_flow.innovation_rejected) {
		_innov_check_fail_status.flags.reject_optflow_X = true;
		_innov_check_fail_status.flags.reject_optflow_Y = true;
		return false;
	}

	// fuse observation axes sequentially
	for (uint8_t index = 0; index <= 1; index++) {
		if (index == 0) {
			// everything was already computed before

		} else if (index == 1) {
			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			const float R_LOS = _aid_src_optical_flow.observation_variance[1];
			const float epsilon = 1e-3f;
			sym::ComputeFlowYInnovVarAndH(state_vector, P, R_LOS, epsilon, &_aid_src_optical_flow.innovation_variance[1], &H);

			// recalculate the innovation using the updated state
			const Vector3f flow_gyro_corrected = _flow_sample_delayed.gyro_rate - _flow_gyro_bias;
			_aid_src_optical_flow.innovation[1] = predictFlow(flow_gyro_corrected)(1) - static_cast<float>
							      (_aid_src_optical_flow.observation[1]);

			// recalculate the test ratio as the measurement jacobian is highly non linear
			// when close to the ground (singularity at 0) and the innovation can suddenly become really
			// large and destabilize the filter
			_aid_src_optical_flow.test_ratio[1] = sq(_aid_src_optical_flow.innovation[1]) / (sq(
					_params.flow_innov_gate) * _aid_src_optical_flow.innovation_variance[1]);

			if (_aid_src_optical_flow.test_ratio[1] > 1.f) {
				continue;
			}
		}

		if (_aid_src_optical_flow.innovation_variance[index] < _aid_src_optical_flow.observation_variance[index]) {
			// we need to reinitialise the covariance matrix and abort this fusion step
			ECL_ERR("Opt flow error - covariance reset");
			initialiseCovariance();
			return false;
		}

		VectorState Kfusion = P * H / _aid_src_optical_flow.innovation_variance[index];

		if (!update_terrain) {
			Kfusion(State::terrain.idx) = 0.f;
		}

		measurementUpdate(Kfusion, H, _aid_src_optical_flow.observation_variance[index],
				  _aid_src_optical_flow.innovation[index]);
	}

	_fault_status.flags.bad_optflow_X = false;
	_fault_status.flags.bad_optflow_Y = false;

	_innov_check_fail_status.flags.reject_optflow_X = (_aid_src_optical_flow.test_ratio[0] > 1.f);
	_innov_check_fail_status.flags.reject_optflow_Y = (_aid_src_optical_flow.test_ratio[1] > 1.f);

	_aid_src_optical_flow.time_last_fuse = _time_delayed_us;
	_aid_src_optical_flow.fused = true;

	_time_last_hor_vel_fuse = _time_delayed_us;

	if (update_terrain) {
		_time_last_terrain_fuse = _time_delayed_us;
	}

	return true;
}

float Ekf::predictFlowHagl() const
{
	// calculate the sensor position relative to the IMU
	const Vector3f pos_offset_body = _params.flow_pos_body - _params.imu_pos_body;

	// calculate the sensor position relative to the IMU in earth frame
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

	// calculate the height above the ground of the optical flow camera. Since earth frame is NED
	// a positive offset in earth frame leads to a smaller height above the ground.
	const float height_above_gnd_est = fabsf(getHagl() - pos_offset_earth(2));

	// Never return a really small value to avoid generating insanely large flow innovations
	// that could destabilize the filter
	constexpr float min_hagl = 1e-2f;

	return fmaxf(height_above_gnd_est, min_hagl);
}
float Ekf::predictFlowRange() const
{
	// calculate range from focal point to centre of image
	// absolute distance to the frame region in view
	return predictFlowHagl() / _R_to_earth(2, 2);
}

Vector2f Ekf::predictFlow(const Vector3f &flow_gyro) const
{
	// calculate the sensor position relative to the IMU
	const Vector3f pos_offset_body = _params.flow_pos_body - _params.imu_pos_body;

	// calculate the velocity of the sensor relative to the imu in body frame
	// Note: flow gyro is the negative of the body angular velocity, thus use minus sign
	const Vector3f vel_rel_imu_body = -flow_gyro % pos_offset_body;

	// calculate the velocity of the sensor in the earth frame
	const Vector3f vel_rel_earth = _state.vel + _R_to_earth * vel_rel_imu_body;

	// rotate into body frame
	const Vector2f vel_body = _state.quat_nominal.rotateVectorInverse(vel_rel_earth).xy();

	// calculate range from focal point to centre of image
	const float scale = _R_to_earth(2, 2) / predictFlowHagl();

	return Vector2f(vel_body(1) * scale, -vel_body(0) * scale);
}

float Ekf::calcOptFlowMeasVar(const flowSample &flow_sample) const
{
	// calculate the observation noise variance - scaling noise linearly across flow quality range
	const float R_LOS_best = fmaxf(_params.flow_noise, 0.05f);
	const float R_LOS_worst = fmaxf(_params.flow_noise_qual_min, 0.05f);

	// calculate a weighting that varies between 1 when flow quality is best and 0 when flow quality is worst
	float weighting = (255.f - (float)_params.flow_qual_min);

	if (weighting >= 1.f) {
		weighting = math::constrain((float)(flow_sample.quality - _params.flow_qual_min) / weighting, 0.f, 1.f);

	} else {
		weighting = 0.0f;
	}

	// take the weighted average of the observation noise for the best and wort flow quality
	const float R_LOS = sq(R_LOS_best * weighting + R_LOS_worst * (1.f - weighting));

	return R_LOS;
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file range_finder_consistency_check.cpp
 */

#include <aid_sources/range_finder/range_finder_consistency_check.hpp>

void RangeFinderConsistencyCheck::update(float dist_bottom, float dist_bottom_var, float vz, float vz_var,
		bool horizontal_motion, uint64_t time_us)
{
	if (horizontal_motion) {
		_time_last_horizontal_motion = time_us;
	}

	const float dt = static_cast<float>(time_us - _time_last_update_us) * 1e-6f;

	if ((_time_last_update_us == 0)
	    || (dt < 0.001f) || (dt > 0.5f)) {
		_time_last_update_us = time_us;
		_dist_bottom_prev = dist_bottom;
		return;
	}

	const float vel_bottom = (dist_bottom - _dist_bottom_prev) / dt;
	_innov = -vel_bottom - vz; // vel_bottom is +up while vz is +down

	// Variance of the time derivative of a random variable: var(dz/dt) = 2*var(z) / dt^2
	const float var = 2.f * dist_bottom_var / (dt * dt);
	_innov_var = var + vz_var;

	const float normalized_innov_sq = (_innov * _innov) / _innov_var;
	_test_ratio = normalized_innov_sq / (_gate * _gate);
	_signed_test_ratio_lpf.setParameters(dt, _signed_test_ratio_tau);
	const float signed_test_ratio = matrix::sign(_innov) * _test_ratio;
	_signed_test_ratio_lpf.update(signed_test_ratio);

	updateConsistency(vz, time_us);

	_time_last_update_us = time_us;
	_dist_bottom_prev = dist_bottom;
}

void RangeFinderConsistencyCheck::updateConsistency(float vz, uint64_t time_us)
{
	if (fabsf(_signed_test_ratio_lpf.getState()) >= 1.f) {
		if ((time_us - _time_last_horizontal_motion) > _signed_test_ratio_tau) {
			_is_kinematically_consistent = false;
			_time_last_inconsistent_us = time_us;
		}

	} else {
		if ((fabsf(vz) > _min_vz_for_valid_consistency)
		    && (_test_ratio < 1.f)
		    && ((time_us - _time_last_inconsistent_us) > _consistency_hyst_time_us)
		   ) {
			_is_kinematically_consistent = true;
		}
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file range_height_control.cpp
 * Control functions for ekf range finder height fusion
 */

#include "ekf.h"
#include "ekf_derivation/generated/compute_hagl_h.h"
#include "ekf_derivation/generated/compute_hagl_innov_var.h"

void Ekf::controlRangeHaglFusion(const imuSample &imu_sample)
{
	static constexpr const char *HGT_SRC_NAME = "RNG";

	bool rng_data_ready = false;

	if (_range_buffer) {
		// Get range data from buffer and check validity
		rng_data_ready = _range_buffer->pop_first_older_than(imu_sample.time_us, _range_sensor.getSampleAddress());
		_range_sensor.setDataReadiness(rng_data_ready);

		// update range sensor angle parameters in case they have changed
		_range_sensor.setPitchOffset(_params.rng_sens_pitch);
		_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
		_range_sensor.setQualityHysteresis(_params.range_valid_quality_s);
		_range_sensor.setMaxFogDistance(_params.rng_fog);

		_range_sensor.runChecks(imu_sample.time_us, _R_to_earth);

		if (_range_sensor.isDataHealthy()) {
			// correct the range data for position offset relative to the IMU
			const Vector3f pos_offset_body = _params.rng_pos_body - _params.imu_pos_body;
			const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
			_range_sensor.setRange(_range_sensor.getRange() + pos_offset_earth(2) / _range_sensor.getCosTilt());

			if (_control_status.flags.in_air) {
				const bool horizontal_motion = _control_status.flags.fixed_wing
							       || (sq(_state.vel(0)) + sq(_state.vel(1)) > fmaxf(P.trace<2>(State::vel.idx), 0.1f));

				const float dist_dependant_var = sq(_params.range_noise_scaler * _range_sensor.getDistBottom());
				const float var = sq(_params.range_noise) + dist_dependant_var;

				_rng_consistency_check.setGate(_params.range_kin_consistency_gate);
				_rng_consistency_check.update(_range_sensor.getDistBottom(), math::max(var, 0.001f), _state.vel(2),
							      P(State::vel.idx + 2, State::vel.idx + 2), horizontal_motion, imu_sample.time_us);
			}

		} else {
			// If we are supposed to be using range finder data but have bad range measurements
			// and are on the ground, then synthesise a measurement at the expected on ground value
			if (!_control_status.flags.in_air
			    && _range_sensor.isRegularlySendingData()
			    && _range_sensor.isDataReady()) {

				_range_sensor.setRange(_params.rng_gnd_clearance);
				_range_sensor.setValidity(true); // bypass the checks
			}
		}

		_control_status.flags.rng_kin_consistent = _rng_consistency_check.isKinematicallyConsistent();

	} else {
		return;
	}

	auto &aid_src = _aid_src_rng_hgt;

	if (rng_data_ready && _range_sensor.getSampleAddress()) {

		updateRangeHagl(aid_src);
		const bool measurement_valid = PX4_ISFINITE(aid_src.observation) && PX4_ISFINITE(aid_src.observation_variance);

		const bool continuing_conditions_passing = ((_params.rng_ctrl == static_cast<int32_t>(RngCtrl::ENABLED))
				|| (_params.rng_ctrl == static_cast<int32_t>(RngCtrl::CONDITIONAL)))
				&& _control_status.flags.tilt_align
				&& measurement_valid
				&& _range_sensor.isDataHealthy()
				&& _rng_consistency_check.isKinematicallyConsistent();

		const bool starting_conditions_passing = continuing_conditions_passing
				&& isNewestSampleRecent(_time_last_range_buffer_push, 2 * estimator::sensor::RNG_MAX_INTERVAL)
				&& _range_sensor.isRegularlySendingData();


		const bool do_conditional_range_aid = (_control_status.flags.rng_terrain || _control_status.flags.rng_hgt)
						      && (_params.rng_ctrl == static_cast<int32_t>(RngCtrl::CONDITIONAL))
						      && isConditionalRangeAidSuitable();

		const bool do_range_aid = (_control_status.flags.rng_terrain || _control_status.flags.rng_hgt)
					  && (_params.rng_ctrl == static_cast<int32_t>(RngCtrl::ENABLED));

		if (_control_status.flags.rng_hgt) {
			if (!(do_conditional_range_aid || do_range_aid)) {
				ECL_INFO("stopping %s fusion", HGT_SRC_NAME);
				stopRngHgtFusion();
			}

		} else {
			if (_params.height_sensor_ref == static_cast<int32_t>(HeightSensor::RANGE)) {
				if (do_conditional_range_aid) {
					// Range finder is used while hovering to stabilize the height estimate. Don't reset but use it as height reference.
					ECL_INFO("starting conditional %s height fusion", HGT_SRC_NAME);
					_height_sensor_ref = HeightSensor::RANGE;

					_control_status.flags.rng_hgt = true;
					stopRngTerrFusion();

					if (!_control_status.flags.opt_flow_terrain && aid_src.innovation_rejected) {
						resetTerrainToRng(aid_src);
						resetAidSourceStatusZeroInnovation(aid_src);
					}

				} else if (do_range_aid) {
					// Range finder is the primary height source, the ground is now the datum used
					// to compute the local vertical position
					ECL_INFO("starting %s height fusion, resetting height", HGT_SRC_NAME);
					_height_sensor_ref = HeightSensor::RANGE;

					_information_events.flags.reset_hgt_to_rng = true;
					resetAltitudeTo(aid_src.observation, aid_src.observation_variance);
					_state.terrain = 0.f;
					resetAidSourceStatusZeroInnovation(aid_src);
					_control_status.flags.rng_hgt = true;
					stopRngTerrFusion();

					aid_src.time_last_fuse = imu_sample.time_us;
				}

			} else {
				if (do_conditional_range_aid || do_range_aid) {
					ECL_INFO("starting %s height fusion", HGT_SRC_NAME);
					_control_status.flags.rng_hgt = true;

					if (!_control_status.flags.opt_flow_terrain && aid_src.innovation_rejected) {
						resetTerrainToRng(aid_src);
						resetAidSourceStatusZeroInnovation(aid_src);
					}
				}
			}
		}

		if (_control_status.flags.rng_hgt || _control_status.flags.rng_terrain) {
			if (continuing_conditions_passing) {

				fuseHaglRng(aid_src, _control_status.flags.rng_hgt, _control_status.flags.rng_terrain);

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired() && _control_status.flags.rng_hgt && (_height_sensor_ref == HeightSensor::RANGE)) {
					// All height sources are failing
					ECL_WARN("%s height fusion reset required, all height sources failing", HGT_SRC_NAME);

					_information_events.flags.reset_hgt_to_rng = true;
					resetAltitudeTo(aid_src.observation - _state.terrain);
					resetAidSourceStatusZeroInnovation(aid_src);

					// reset vertical velocity if no valid sources available
					if (!isVerticalVelocityAidingActive()) {
						resetVerticalVelocityToZero();
					}

					aid_src.time_last_fuse = imu_sample.time_us;

				} else if (is_fusion_failing) {
					// Some other height source is still working
					if (_control_status.flags.opt_flow_terrain && isTerrainEstimateValid()) {
						ECL_WARN("stopping %s fusion, fusion failing", HGT_SRC_NAME);
						stopRngHgtFusion();
						stopRngTerrFusion();

					} else {
						resetTerrainToRng(aid_src);
						resetAidSourceStatusZeroInnovation(aid_src);
					}
				}

			} else {
				ECL_WARN("stopping %s fusion, continuing conditions failing", HGT_SRC_NAME);
				stopRngHgtFusion();
				stopRngTerrFusion();
			}

		} else {
			if (starting_conditions_passing) {
				if (_control_status.flags.opt_flow_terrain) {
					if (!aid_src.innovation_rejected) {
						_control_status.flags.rng_terrain = true;
						fuseHaglRng(aid_src, _control_status.flags.rng_hgt, _control_status.flags.rng_terrain);
					}

				} else {
					if (aid_src.innovation_rejected) {
						resetTerrainToRng(aid_src);
						resetAidSourceStatusZeroInnovation(aid_src);
					}

					_control_status.flags.rng_terrain = true;
				}
			}
		}

	} else if ((_control_status.flags.rng_hgt || _control_status.flags.rng_terrain)
		   && !isNewestSampleRecent(_time_last_range_buffer_push, 2 * estimator::sensor::RNG_MAX_INTERVAL)) {
		// No data anymore. Stop until it comes back.
		ECL_WARN("stopping %s fusion, no data", HGT_SRC_NAME);
		stopRngHgtFusion();
		stopRngTerrFusion();
	}
}

void Ekf::updateRangeHagl(estimator_aid_source1d_s &aid_src)
{
	const float measurement = math::max(_range_sensor.getDistBottom(), _params.rng_gnd_clearance);
	const float measurement_variance = getRngVar();

	float innovation_variance;
	sym::ComputeHaglInnovVar(P, measurement_variance, &innovation_variance);

	const float innov_gate = math::max(_params.range_innov_gate, 1.f);
	updateAidSourceStatus(aid_src,
			      _range_sensor.getSampleAddress()->time_us, // sample timestamp
			      measurement,                               // observation
			      measurement_variance,                      // observation variance
			      getHagl() - measurement,                   // innovation
			      innovation_variance,                       // innovation variance
			      innov_gate);                               // innovation gate

	// z special case if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	if (_fault_status.flags.bad_acc_vertical && aid_src.innovation_rejected) {
		const float innov_limit = innov_gate * sqrtf(aid_src.innovation_variance);
		aid_src.innovation = math::constrain(aid_src.innovation, -innov_limit, innov_limit);
		aid_src.innovation_rejected = false;
	}
}

float Ekf::getRngVar() const
{
	return fmaxf(
		       P(State::pos.idx + 2, State::pos.idx + 2)
		       + sq(_params.range_noise)
		       + sq(_params.range_noise_scaler * _range_sensor.getRange()),
		       0.f);
}

void Ekf::resetTerrainToRng(estimator_aid_source1d_s &aid_src)
{
	// Since the distance is not a direct observation of the terrain state but is based
	// on the height state, a reset should consider the height uncertainty. This can be
	// done by manipulating the Kalman gain to inject all the innovation in the terrain state
	// and create the correct correlation with the terrain state with a covariance update.
	P.uncorrelateCovarianceSetVariance<State::terrain.dof>(State::terrain.idx, 0.f);

	const float old_terrain = _state.terrain;

	VectorState H;
	sym::ComputeHaglH(&H);

	VectorState K;
	K(State::terrain.idx) = 1.f; // innovation is forced into the terrain state to create a "reset"

	measurementUpdate(K, H, aid_src.observation_variance, aid_src.innovation);

	// record the state change
	const float delta_terrain = _state.terrain - old_terrain;

	if (_state_reset_status.reset_count.hagl == _state_reset_count_prev.hagl) {
		_state_reset_status.hagl_change = delta_terrain;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.hagl_change += delta_terrain;
	}

	_state_reset_status.reset_count.hagl++;

	aid_src.time_last_fuse = _time_delayed_us;
}

bool Ekf::isConditionalRangeAidSuitable()
{
	// check if we can use range finder measurements to estimate height, use hysteresis to avoid rapid switching
	// Note that the 0.7 coefficients and the innovation check are arbitrary values but work well in practice
	float range_hagl_max = _params.max_hagl_for_range_aid;
	float max_vel_xy = _params.max_vel_for_range_aid;

	const float hagl_test_ratio = _aid_src_rng_hgt.test_ratio;

	bool is_hagl_stable = (hagl_test_ratio < 1.f);

	if (!_control_status.flags.rng_hgt) {
		range_hagl_max = 0.7f * _params.max_hagl_for_range_aid;
		max_vel_xy = 0.7f * _params.max_vel_for_range_aid;
		is_hagl_stable = (hagl_test_ratio < 0.01f);
	}

	const bool is_in_range = (getHagl() < range_hagl_max);

	bool is_below_max_speed = true;

	if (isHorizontalAidingActive()) {
		is_below_max_speed = !_state.vel.xy().longerThan(max_vel_xy);
	}

	return is_in_range && is_hagl_stable && is_below_max_speed;
}

void Ekf::stopRngHgtFusion()
{
	if (_control_status.flags.rng_hgt) {

		if (_height_sensor_ref == HeightSensor::RANGE) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_control_status.flags.rng_hgt = false;
	}
}

void Ekf::stopRngTerrFusion()
{
	_control_status.flags.rng_terrain = false;
}
/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ekf.h"
#include "ekf_derivation/generated/compute_hagl_h.h"

bool Ekf::fuseHaglRng(estimator_aid_source1d_s &aid_src, bool update_height, bool update_terrain)
{
	if (aid_src.innovation_rejected) {
		_innov_check_fail_status.flags.reject_hagl = true;
		return false;
	}

	VectorState H;

	sym::ComputeHaglH(&H);

	// calculate the Kalman gain
	VectorState K = P * H / aid_src.innovation_variance;

	if (!update_terrain) {
		K(State::terrain.idx) = 0.f;
	}

	if (!update_height) {
		const float k_terrain = K(State::terrain.idx);
		K.zero();
		K(State::terrain.idx) = k_terrain;
	}

	measurementUpdate(K, H, aid_src.observation_variance, aid_src.innovation);

	// record last successful fusion event
	_innov_check_fail_status.flags.reject_hagl = false;

	aid_src.time_last_fuse = _time_delayed_us;
	aid_src.fused = true;

	if (update_terrain) {
		_time_last_terrain_fuse = _time_delayed_us;
	}

	return true;
}
/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file sensor_range_finder.cpp
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 *
 */

#include <aid_sources/range_finder/sensor_range_finder.hpp>

#include <lib/matrix/matrix/math.hpp>

namespace estimator
{
namespace sensor
{

void SensorRangeFinder::runChecks(const uint64_t current_time_us, const matrix::Dcmf &R_to_earth)
{
	updateSensorToEarthRotation(R_to_earth);
	updateValidity(current_time_us);
}

void SensorRangeFinder::updateSensorToEarthRotation(const matrix::Dcmf &R_to_earth)
{
	// calculate 2,2 element of rotation matrix from sensor frame to earth frame
	// this is required for use of range finder and flow data
	_cos_tilt_rng_to_earth = R_to_earth(2, 0) * _sin_pitch_offset + R_to_earth(2, 2) * _cos_pitch_offset;
}

void SensorRangeFinder::updateValidity(uint64_t current_time_us)
{
	updateDtDataLpf(current_time_us);

	if (_is_faulty || isSampleOutOfDate(current_time_us) || !isDataContinuous()) {
		_is_sample_valid = false;
		_is_regularly_sending_data = false;
		return;
	}

	_is_regularly_sending_data = true;

	// Don't run the checks unless we have retrieved new data from the buffer
	if (_is_sample_ready) {
		_is_sample_valid = false;

		_time_bad_quality_us = _sample.quality == 0 ? current_time_us : _time_bad_quality_us;

		if (!isQualityOk(current_time_us) || !isTiltOk() || !isDataInRange()) {
			return;
		}

		updateStuckCheck();
		updateFogCheck(getDistBottom(), _sample.time_us);

		if (!_is_stuck && !_is_blocked) {
			_is_sample_valid = true;
			_time_last_valid_us = _sample.time_us;
		}
	}
}

bool SensorRangeFinder::isQualityOk(uint64_t current_time_us) const
{
	return current_time_us - _time_bad_quality_us > _quality_hyst_us;
}

void SensorRangeFinder::updateDtDataLpf(uint64_t current_time_us)
{
	// Calculate a first order IIR low-pass filtered time of arrival between samples using a 2 second time constant.
	float alpha = 0.5f * _dt_update;
	_dt_data_lpf = _dt_data_lpf * (1.0f - alpha) + alpha * (current_time_us - _sample.time_us);

	// Apply spike protection to the filter state.
	_dt_data_lpf = fminf(_dt_data_lpf, 4e6f);
}

inline bool SensorRangeFinder::isSampleOutOfDate(uint64_t current_time_us) const
{
	return (current_time_us - _sample.time_us) > 2 * RNG_MAX_INTERVAL;
}

inline bool SensorRangeFinder::isDataInRange() const
{
	return (_sample.rng >= _rng_valid_min_val) && (_sample.rng <= _rng_valid_max_val);
}

void SensorRangeFinder::updateStuckCheck()
{
	if (!isStuckDetectorEnabled()) {
		// Stuck detector disabled
		_is_stuck = false;
		return;
	}

	// Check for "stuck" range finder measurements when range was not valid for certain period
	// This handles a failure mode observed with some lidar sensors
	if (((_sample.time_us - _time_last_valid_us) > (uint64_t)10e6)) {

		// require a variance of rangefinder values to check for "stuck" measurements
		if (_stuck_max_val - _stuck_min_val > _stuck_threshold) {
			_stuck_min_val = 0.0f;
			_stuck_max_val = 0.0f;
			_is_stuck = false;

		} else {
			if (_sample.rng > _stuck_max_val) {
				_stuck_max_val = _sample.rng;
			}

			if (_stuck_min_val < 0.1f || _sample.rng < _stuck_min_val) {
				_stuck_min_val = _sample.rng;
			}

			_is_stuck = true;
		}
	}
}

void SensorRangeFinder::updateFogCheck(const float dist_bottom, const uint64_t time_us)
{
	if (_max_fog_dist > 0.f) {

		const float median_dist = _median_dist.apply(dist_bottom);
		const float factor = 2.f; // magic hardcoded factor

		if (!_is_blocked && median_dist < _max_fog_dist && _prev_median_dist - median_dist > factor * _max_fog_dist) {
			_is_blocked = true;

		} else if (_is_blocked && median_dist > factor * _max_fog_dist) {
			_is_blocked = false;
		}

		_prev_median_dist = median_dist;
	}
}

} // namespace sensor
} // namespace estimator
/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file sideslip_fusion.cpp
 * sideslip fusion methods.
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Carl Olsson <carlolsson.co@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"
#include <ekf_derivation/generated/compute_sideslip_innov_and_innov_var.h>
#include <ekf_derivation/generated/compute_sideslip_h.h>

#include <mathlib/mathlib.h>

void Ekf::controlBetaFusion(const imuSample &imu_delayed)
{
	_control_status.flags.fuse_beta = _params.beta_fusion_enabled
					  && (_control_status.flags.fixed_wing || _control_status.flags.fuse_aspd)
					  && _control_status.flags.in_air
					  && !_control_status.flags.fake_pos;

	if (_control_status.flags.fuse_beta) {

		// Perform synthetic sideslip fusion at regular intervals when in-air and sideslip fusion had been enabled externally:
		const bool beta_fusion_time_triggered = isTimedOut(_aid_src_sideslip.time_last_fuse, _params.beta_avg_ft_us);

		if (beta_fusion_time_triggered) {

			updateSideslip(_aid_src_sideslip);
			_innov_check_fail_status.flags.reject_sideslip = _aid_src_sideslip.innovation_rejected;

			if (fuseSideslip(_aid_src_sideslip)) {
				_control_status.flags.wind = true;

			} else if (!_external_wind_init && !_control_status.flags.wind) {
				resetWindCov();
			}
		}
	}
}

void Ekf::updateSideslip(estimator_aid_source1d_s &aid_src) const
{
	float observation = 0.f;
	const float R = math::max(sq(_params.beta_noise), sq(0.01f)); // observation noise variance
	const float epsilon = 1e-3f;
	float innov;
	float innov_var;
	sym::ComputeSideslipInnovAndInnovVar(_state.vector(), P, R, epsilon, &innov, &innov_var);

	updateAidSourceStatus(aid_src,
			      _time_delayed_us,                         // sample timestamp
			      observation,                              // observation
			      R,                                        // observation variance
			      innov,                                    // innovation
			      innov_var,                                // innovation variance
			      math::max(_params.beta_innov_gate, 1.f)); // innovation gate
}

bool Ekf::fuseSideslip(estimator_aid_source1d_s &sideslip)
{
	if (sideslip.innovation_rejected) {
		return false;
	}

	// determine if we need the sideslip fusion to correct states other than wind
	bool update_wind_only = !_control_status.flags.wind_dead_reckoning;

	// Reset covariance and states if the calculation is badly conditioned
	if ((sideslip.innovation_variance < sideslip.observation_variance)
	    || (sideslip.innovation_variance < FLT_EPSILON)) {
		_fault_status.flags.bad_sideslip = true;

		// if we are getting aiding from other sources, warn and reset the wind states and covariances only
		const char *action_string = nullptr;

		if (update_wind_only) {
			resetWindCov();
			action_string = "wind";

		} else {
			initialiseCovariance();
			_state.wind_vel.setZero();
			action_string = "full";
		}

		ECL_ERR("sideslip badly conditioned - %s covariance reset", action_string);

		return false;
	}

	_fault_status.flags.bad_sideslip = false;

	const float epsilon = 1e-3f;

	const VectorState H = sym::ComputeSideslipH(_state.vector(), epsilon);
	VectorState K = P * H / sideslip.innovation_variance;

	if (update_wind_only) {
		const Vector2f K_wind = K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0);
		K.setZero();
		K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0) = K_wind;
	}

	measurementUpdate(K, H, sideslip.observation_variance, sideslip.innovation);

	sideslip.fused = true;
	sideslip.time_last_fuse = _time_delayed_us;

	return true;
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file terrain_control.cpp
 */

#include "ekf.h"
#include "ekf_derivation/generated/compute_hagl_innov_var.h"

#include <mathlib/mathlib.h>

void Ekf::initTerrain()
{
	// assume a ground clearance
	_state.terrain = -_gpos.altitude() + _params.rng_gnd_clearance;

	// use the ground clearance value as our uncertainty
	P.uncorrelateCovarianceSetVariance<State::terrain.dof>(State::terrain.idx, sq(_params.rng_gnd_clearance));
}

void Ekf::controlTerrainFakeFusion()
{
	// If we are on ground, store the local position and time to use as a reference
	if (!_control_status.flags.in_air) {
		_last_on_ground_posD = -_gpos.altitude();
		_control_status.flags.rng_fault = false;

	} else if (!_control_status_prev.flags.in_air) {
		// Let the estimator run freely before arming for bench testing purposes, but reset on takeoff
		// because when using optical flow measurements, it is safer to start with a small distance to ground
		// as an overestimated distance leads to an overestimated velocity, causing a dangerous behavior.
		initTerrain();
	}

	if (!_control_status.flags.in_air) {
		bool no_terrain_aiding = !_control_status.flags.rng_terrain
					 && !_control_status.flags.opt_flow_terrain
					 && isTimedOut(_time_last_terrain_fuse, (uint64_t)1e6);

		if (no_terrain_aiding && (_height_sensor_ref != HeightSensor::RANGE)) {
			initTerrain();
		}
	}
}

void Ekf::updateTerrainValidity()
{
	bool valid_opt_flow_terrain = false;
	bool valid_rng_terrain = false;
	bool positive_hagl_var = false;
	bool small_relative_hagl_var = false;

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

	if (_control_status.flags.opt_flow_terrain
	    && isRecent(_aid_src_optical_flow.time_last_fuse, _params.hgt_fusion_timeout_max)
	   ) {
		valid_opt_flow_terrain = true;
	}

#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_RANGE_FINDER)

	if (_control_status.flags.rng_terrain
	    && isRecent(_aid_src_rng_hgt.time_last_fuse, _params.hgt_fusion_timeout_max)
	   ) {
		valid_rng_terrain = true;
	}

#endif // CONFIG_EKF2_RANGE_FINDER

	if (_time_last_terrain_fuse != 0) {
		// Assume being valid when the uncertainty is small compared to the height above ground
		float hagl_var = INFINITY;
		sym::ComputeHaglInnovVar(P, 0.f, &hagl_var);

		positive_hagl_var = hagl_var > 0.f;

		if (positive_hagl_var
		    && (hagl_var < sq(fmaxf(0.1f * getHagl(), 0.5f)))
		   ) {
			small_relative_hagl_var = true;
		}
	}

	const bool positive_hagl = getHagl() >= 0.f;

	if (!_terrain_valid) {
		// require valid RNG or optical flow (+valid variance) to initially consider terrain valid
		if (positive_hagl
		    && positive_hagl_var
		    && (valid_rng_terrain
			|| (valid_opt_flow_terrain && small_relative_hagl_var))
		   ) {
			_terrain_valid = true;
		}

	} else {
		// terrain was previously valid, continue considering valid if variance is good
		_terrain_valid = positive_hagl && positive_hagl_var && small_relative_hagl_var;
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file wind.cpp
 * Helper functions for wind states
 */

#include "ekf.h"
#include <ekf_derivation/generated/compute_wind_init_and_cov_from_wind_speed_and_direction.h>

void Ekf::resetWindToExternalObservation(float wind_speed, float wind_direction, float wind_speed_accuracy,
		float wind_direction_accuracy)
{
	if (!_control_status.flags.in_air) {

		const float wind_speed_constrained = math::max(wind_speed, 0.0f);
		const float wind_direction_var = sq(wind_direction_accuracy);
		const float wind_speed_var = sq(wind_speed_accuracy);

		Vector2f wind;
		Vector2f wind_var;

		sym::ComputeWindInitAndCovFromWindSpeedAndDirection(wind_speed_constrained, wind_direction, wind_speed_var,
				wind_direction_var, &wind, &wind_var);

		ECL_INFO("reset wind states to external observation");
		_information_events.flags.reset_wind_to_ext_obs = true;
		_external_wind_init = true;

		resetWindTo(wind, wind_var);

	}
}

void Ekf::resetWindTo(const Vector2f &wind, const Vector2f &wind_var)
{
	_state.wind_vel = wind;

	if (PX4_ISFINITE(wind_var(0))) {
		P.uncorrelateCovarianceSetVariance<1>(State::wind_vel.idx,
						      math::min(sq(_params.initial_wind_uncertainty), wind_var(0)));
	}

	if (PX4_ISFINITE(wind_var(1))) {
		P.uncorrelateCovarianceSetVariance<1>(State::wind_vel.idx + 1,
						      math::min(sq(_params.initial_wind_uncertainty), wind_var(1)));
	}
}

void Ekf::resetWindCov()
{
	// start with a small initial uncertainty to improve the initial estimate
	P.uncorrelateCovarianceSetVariance<State::wind_vel.dof>(State::wind_vel.idx, sq(_params.initial_wind_uncertainty));
}

void Ekf::resetWindToZero()
{
	ECL_INFO("reset wind to zero");
	_state.wind_vel.setZero();
	resetWindCov();
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <px4_platform_common/events.h>
#include "EKF2.hpp"

using namespace time_literals;
using math::constrain;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

static constexpr float kDefaultExternalPosAccuracy = 50.0f; // [m]
static constexpr float kMaxDelaySecondsExternalPosMeasurement = 15.0f; // [s]

pthread_mutex_t ekf2_module_mutex = PTHREAD_MUTEX_INITIALIZER;
static px4::atomic<EKF2 *> _objects[EKF2_MAX_INSTANCES] {};
#if defined(CONFIG_EKF2_MULTI_INSTANCE)
static px4::atomic<EKF2Selector *> _ekf2_selector {nullptr};
#endif // CONFIG_EKF2_MULTI_INSTANCE

EKF2::EKF2(bool multi_mode, const px4::wq_config_t &config, bool replay_mode):
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, config),
	_replay_mode(replay_mode && !multi_mode),
	_multi_mode(multi_mode),
	_instance(multi_mode ? -1 : 0),
	_attitude_pub(multi_mode ? ORB_ID(estimator_attitude) : ORB_ID(vehicle_attitude)),
	_local_position_pub(multi_mode ? ORB_ID(estimator_local_position) : ORB_ID(vehicle_local_position)),
	_global_position_pub(multi_mode ? ORB_ID(estimator_global_position) : ORB_ID(vehicle_global_position)),
	_odometry_pub(multi_mode ? ORB_ID(estimator_odometry) : ORB_ID(vehicle_odometry)),
#if defined(CONFIG_EKF2_WIND)
	_wind_pub(multi_mode ? ORB_ID(estimator_wind) : ORB_ID(wind)),
#endif // CONFIG_EKF2_WIND
	_params(_ekf.getParamHandle()),
	_param_ekf2_predict_us(_params->filter_update_interval_us),
	_param_ekf2_delay_max(_params->delay_max_ms),
	_param_ekf2_imu_ctrl(_params->imu_ctrl),
	_param_ekf2_vel_lim(_params->velocity_limit),
#if defined(CONFIG_EKF2_AUXVEL)
	_param_ekf2_avel_delay(_params->auxvel_delay_ms),
#endif // CONFIG_EKF2_AUXVEL
	_param_ekf2_gyr_noise(_params->gyro_noise),
	_param_ekf2_acc_noise(_params->accel_noise),
	_param_ekf2_gyr_b_noise(_params->gyro_bias_p_noise),
	_param_ekf2_acc_b_noise(_params->accel_bias_p_noise),
#if defined(CONFIG_EKF2_WIND)
	_param_ekf2_wind_nsd(_params->wind_vel_nsd),
#endif // CONFIG_EKF2_WIND
	_param_ekf2_noaid_noise(_params->pos_noaid_noise),
#if defined(CONFIG_EKF2_GNSS)
	_param_ekf2_gps_ctrl(_params->gnss_ctrl),
	_param_ekf2_gps_delay(_params->gps_delay_ms),
	_param_ekf2_gps_pos_x(_params->gps_pos_body(0)),
	_param_ekf2_gps_pos_y(_params->gps_pos_body(1)),
	_param_ekf2_gps_pos_z(_params->gps_pos_body(2)),
	_param_ekf2_gps_v_noise(_params->gps_vel_noise),
	_param_ekf2_gps_p_noise(_params->gps_pos_noise),
	_param_ekf2_gps_p_gate(_params->gps_pos_innov_gate),
	_param_ekf2_gps_v_gate(_params->gps_vel_innov_gate),
	_param_ekf2_gps_check(_params->gps_check_mask),
	_param_ekf2_req_eph(_params->req_hacc),
	_param_ekf2_req_epv(_params->req_vacc),
	_param_ekf2_req_sacc(_params->req_sacc),
	_param_ekf2_req_nsats(_params->req_nsats),
	_param_ekf2_req_pdop(_params->req_pdop),
	_param_ekf2_req_hdrift(_params->req_hdrift),
	_param_ekf2_req_vdrift(_params->req_vdrift),
	_param_ekf2_gsf_tas_default(_params->EKFGSF_tas_default),
#endif // CONFIG_EKF2_GNSS
#if defined(CONFIG_EKF2_BAROMETER)
	_param_ekf2_baro_ctrl(_params->baro_ctrl),
	_param_ekf2_baro_delay(_params->baro_delay_ms),
	_param_ekf2_baro_noise(_params->baro_noise),
	_param_ekf2_baro_gate(_params->baro_innov_gate),
	_param_ekf2_gnd_eff_dz(_params->gnd_effect_deadzone),
	_param_ekf2_gnd_max_hgt(_params->gnd_effect_max_hgt),
# if defined(CONFIG_EKF2_BARO_COMPENSATION)
	_param_ekf2_aspd_max(_params->max_correction_airspeed),
	_param_ekf2_pcoef_xp(_params->static_pressure_coef_xp),
	_param_ekf2_pcoef_xn(_params->static_pressure_coef_xn),
	_param_ekf2_pcoef_yp(_params->static_pressure_coef_yp),
	_param_ekf2_pcoef_yn(_params->static_pressure_coef_yn),
	_param_ekf2_pcoef_z(_params->static_pressure_coef_z),
# endif // CONFIG_EKF2_BARO_COMPENSATION
#endif // CONFIG_EKF2_BAROMETER
#if defined(CONFIG_EKF2_AIRSPEED)
	_param_ekf2_asp_delay(_params->airspeed_delay_ms),
	_param_ekf2_tas_gate(_params->tas_innov_gate),
	_param_ekf2_eas_noise(_params->eas_noise),
	_param_ekf2_arsp_thr(_params->arsp_thr),
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_SIDESLIP)
	_param_ekf2_beta_gate(_params->beta_innov_gate),
	_param_ekf2_beta_noise(_params->beta_noise),
	_param_ekf2_fuse_beta(_params->beta_fusion_enabled),
#endif // CONFIG_EKF2_SIDESLIP
#if defined(CONFIG_EKF2_MAGNETOMETER)
	_param_ekf2_mag_delay(_params->mag_delay_ms),
	_param_ekf2_mag_e_noise(_params->mage_p_noise),
	_param_ekf2_mag_b_noise(_params->magb_p_noise),
	_param_ekf2_head_noise(_params->mag_heading_noise),
	_param_ekf2_mag_noise(_params->mag_noise),
	_param_ekf2_mag_decl(_params->mag_declination_deg),
	_param_ekf2_hdg_gate(_params->heading_innov_gate),
	_param_ekf2_mag_gate(_params->mag_innov_gate),
	_param_ekf2_decl_type(_params->mag_declination_source),
	_param_ekf2_mag_type(_params->mag_fusion_type),
	_param_ekf2_mag_acclim(_params->mag_acc_gate),
	_param_ekf2_mag_check(_params->mag_check),
	_param_ekf2_mag_chk_str(_params->mag_check_strength_tolerance_gs),
	_param_ekf2_mag_chk_inc(_params->mag_check_inclination_tolerance_deg),
	_param_ekf2_synthetic_mag_z(_params->synthesize_mag_z),
#endif // CONFIG_EKF2_MAGNETOMETER
	_param_ekf2_hgt_ref(_params->height_sensor_ref),
	_param_ekf2_noaid_tout(_params->valid_timeout_max),
#if defined(CONFIG_EKF2_TERRAIN) || defined(CONFIG_EKF2_OPTICAL_FLOW) || defined(CONFIG_EKF2_RANGE_FINDER)
	_param_ekf2_min_rng(_params->rng_gnd_clearance),
#endif // CONFIG_EKF2_TERRAIN || CONFIG_EKF2_OPTICAL_FLOW || CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_TERRAIN)
	_param_ekf2_terr_noise(_params->terrain_p_noise),
	_param_ekf2_terr_grad(_params->terrain_gradient),
#endif // CONFIG_EKF2_TERRAIN
#if defined(CONFIG_EKF2_RANGE_FINDER)
	_param_ekf2_rng_ctrl(_params->rng_ctrl),
	_param_ekf2_rng_delay(_params->range_delay_ms),
	_param_ekf2_rng_noise(_params->range_noise),
	_param_ekf2_rng_sfe(_params->range_noise_scaler),
	_param_ekf2_rng_gate(_params->range_innov_gate),
	_param_ekf2_rng_pitch(_params->rng_sens_pitch),
	_param_ekf2_rng_a_vmax(_params->max_vel_for_range_aid),
	_param_ekf2_rng_a_hmax(_params->max_hagl_for_range_aid),
	_param_ekf2_rng_a_igate(_params->range_aid_innov_gate),
	_param_ekf2_rng_qlty_t(_params->range_valid_quality_s),
	_param_ekf2_rng_k_gate(_params->range_kin_consistency_gate),
	_param_ekf2_rng_fog(_params->rng_fog),
	_param_ekf2_rng_pos_x(_params->rng_pos_body(0)),
	_param_ekf2_rng_pos_y(_params->rng_pos_body(1)),
	_param_ekf2_rng_pos_z(_params->rng_pos_body(2)),
#endif // CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	_param_ekf2_ev_delay(_params->ev_delay_ms),
	_param_ekf2_ev_ctrl(_params->ev_ctrl),
	_param_ekf2_ev_qmin(_params->ev_quality_minimum),
	_param_ekf2_evp_noise(_params->ev_pos_noise),
	_param_ekf2_evv_noise(_params->ev_vel_noise),
	_param_ekf2_eva_noise(_params->ev_att_noise),
	_param_ekf2_evv_gate(_params->ev_vel_innov_gate),
	_param_ekf2_evp_gate(_params->ev_pos_innov_gate),
	_param_ekf2_ev_pos_x(_params->ev_pos_body(0)),
	_param_ekf2_ev_pos_y(_params->ev_pos_body(1)),
	_param_ekf2_ev_pos_z(_params->ev_pos_body(2)),
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	_param_ekf2_of_ctrl(_params->flow_ctrl),
	_param_ekf2_of_gyr_src(_params->flow_gyro_src),
	_param_ekf2_of_delay(_params->flow_delay_ms),
	_param_ekf2_of_n_min(_params->flow_noise),
	_param_ekf2_of_n_max(_params->flow_noise_qual_min),
	_param_ekf2_of_qmin(_params->flow_qual_min),
	_param_ekf2_of_qmin_gnd(_params->flow_qual_min_gnd),
	_param_ekf2_of_gate(_params->flow_innov_gate),
	_param_ekf2_of_pos_x(_params->flow_pos_body(0)),
	_param_ekf2_of_pos_y(_params->flow_pos_body(1)),
	_param_ekf2_of_pos_z(_params->flow_pos_body(2)),
#endif // CONFIG_EKF2_OPTICAL_FLOW
#if defined(CONFIG_EKF2_DRAG_FUSION)
	_param_ekf2_drag_ctrl(_params->drag_ctrl),
	_param_ekf2_drag_noise(_params->drag_noise),
	_param_ekf2_bcoef_x(_params->bcoef_x),
	_param_ekf2_bcoef_y(_params->bcoef_y),
	_param_ekf2_mcoef(_params->mcoef),
#endif // CONFIG_EKF2_DRAG_FUSION
#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	_param_ekf2_grav_noise(_params->gravity_noise),
#endif // CONFIG_EKF2_GRAVITY_FUSION
	_param_ekf2_imu_pos_x(_params->imu_pos_body(0)),
	_param_ekf2_imu_pos_y(_params->imu_pos_body(1)),
	_param_ekf2_imu_pos_z(_params->imu_pos_body(2)),
	_param_ekf2_gbias_init(_params->switch_on_gyro_bias),
	_param_ekf2_abias_init(_params->switch_on_accel_bias),
	_param_ekf2_angerr_init(_params->initial_tilt_err),
	_param_ekf2_abl_lim(_params->acc_bias_lim),
	_param_ekf2_abl_acclim(_params->acc_bias_learn_acc_lim),
	_param_ekf2_abl_gyrlim(_params->acc_bias_learn_gyr_lim),
	_param_ekf2_abl_tau(_params->acc_bias_learn_tc),
	_param_ekf2_gyr_b_lim(_params->gyro_bias_lim)
{
	AdvertiseTopics();
}

EKF2::~EKF2()
{
	perf_free(_ekf_update_perf);
	perf_free(_msg_missed_imu_perf);
}

void EKF2::AdvertiseTopics()
{
	// advertise expected minimal topic set immediately for logging
	_attitude_pub.advertise();
	_local_position_pub.advertise();
	_estimator_event_flags_pub.advertise();
	_estimator_sensor_bias_pub.advertise();
	_estimator_status_pub.advertise();
	_estimator_status_flags_pub.advertise();

	if (_multi_mode) {
		// only force advertise these in multi mode to ensure consistent uORB instance numbering
		_global_position_pub.advertise();
		_odometry_pub.advertise();

#if defined(CONFIG_EKF2_WIND)
		_wind_pub.advertise();
#endif // CONFIG_EKF2_WIND
	}

#if defined(CONFIG_EKF2_GNSS)

	if (_param_ekf2_gps_ctrl.get()) {
		_estimator_gps_status_pub.advertise();
		_yaw_est_pub.advertise();
	}

#endif // CONFIG_EKF2_GNSS

	// verbose logging
	if (_param_ekf2_log_verbose.get()) {
		_estimator_innovation_test_ratios_pub.advertise();
		_estimator_innovation_variances_pub.advertise();
		_estimator_innovations_pub.advertise();
		_estimator_states_pub.advertise();

#if defined(CONFIG_EKF2_AIRSPEED)

		if (_param_ekf2_arsp_thr.get() > 0.f) {
			_estimator_aid_src_airspeed_pub.advertise();
		}

#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_BAROMETER)

		if (_param_ekf2_baro_ctrl.get()) {
			_estimator_aid_src_baro_hgt_pub.advertise();
			_estimator_baro_bias_pub.advertise();
		}

#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_DRAG_FUSION)

		if (_param_ekf2_drag_ctrl.get()) {
			_estimator_aid_src_drag_pub.advertise();
		}

#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

		if (_param_ekf2_ev_ctrl.get() & static_cast<int32_t>(EvCtrl::VPOS)) {
			_estimator_aid_src_ev_hgt_pub.advertise();
			_estimator_ev_pos_bias_pub.advertise();
		}

		if (_param_ekf2_ev_ctrl.get() & static_cast<int32_t>(EvCtrl::HPOS)) {
			_estimator_aid_src_ev_pos_pub.advertise();
			_estimator_ev_pos_bias_pub.advertise();
		}

		if (_param_ekf2_ev_ctrl.get() & static_cast<int32_t>(EvCtrl::VEL)) {
			_estimator_aid_src_ev_vel_pub.advertise();
		}

		if (_param_ekf2_ev_ctrl.get() & static_cast<int32_t>(EvCtrl::YAW)) {
			_estimator_aid_src_ev_yaw_pub.advertise();
		}

#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)

		if (_param_ekf2_gps_ctrl.get()) {
			if (_param_ekf2_gps_ctrl.get() & static_cast<int32_t>(GnssCtrl::VPOS)) {
				_estimator_aid_src_gnss_hgt_pub.advertise();
				_estimator_gnss_hgt_bias_pub.advertise();
			}

			if (_param_ekf2_gps_ctrl.get() & static_cast<int32_t>(GnssCtrl::HPOS)) {
				_estimator_aid_src_gnss_pos_pub.advertise();
			}

			if (_param_ekf2_gps_ctrl.get() & static_cast<int32_t>(GnssCtrl::VEL)) {
				_estimator_aid_src_gnss_vel_pub.advertise();
			}

# if defined(CONFIG_EKF2_GNSS_YAW)

			if (_param_ekf2_gps_ctrl.get() & static_cast<int32_t>(GnssCtrl::YAW)) {
				_estimator_aid_src_gnss_yaw_pub.advertise();
			}

# endif // CONFIG_EKF2_GNSS_YAW
		}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_GRAVITY_FUSION)

		if (_param_ekf2_imu_ctrl.get() & static_cast<int32_t>(ImuCtrl::GravityVector)) {
			_estimator_aid_src_gravity_pub.advertise();
		}

#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_MAGNETOMETER)

		if (_param_ekf2_mag_type.get() != MagFuseType::NONE) {
			_estimator_aid_src_mag_pub.advertise();
		}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

		if (_param_ekf2_of_ctrl.get()) {
			_estimator_optical_flow_vel_pub.advertise();
			_estimator_aid_src_optical_flow_pub.advertise();
		}

#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_RANGE_FINDER)

		// RNG advertise
		if (_param_ekf2_rng_ctrl.get()) {
			_estimator_aid_src_rng_hgt_pub.advertise();
		}

#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_SIDESLIP)

		if (_param_ekf2_fuse_beta.get()) {
			_estimator_aid_src_sideslip_pub.advertise();
		}

#endif // CONFIG_EKF2_SIDESLIP

	} // end verbose logging
}

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
bool EKF2::multi_init(int imu, int mag)
{
	bool changed_instance = _vehicle_imu_sub.ChangeInstance(imu);

#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (!_magnetometer_sub.ChangeInstance(mag)) {
		changed_instance = false;
	}

#endif // CONFIG_EKF2_MAGNETOMETER

	const int status_instance = _estimator_states_pub.get_instance();

	if ((status_instance >= 0) && changed_instance
	    && (_attitude_pub.get_instance() == status_instance)
	    && (_local_position_pub.get_instance() == status_instance)
	    && (_global_position_pub.get_instance() == status_instance)) {

		_instance = status_instance;

		ScheduleNow();
		return true;
	}

	PX4_ERR("publication instance problem: %d att: %d lpos: %d gpos: %d", status_instance,
		_attitude_pub.get_instance(), _local_position_pub.get_instance(), _global_position_pub.get_instance());

	return false;
}
#endif // CONFIG_EKF2_MULTI_INSTANCE

int EKF2::print_status(bool verbose)
{
	PX4_INFO_RAW("ekf2:%d EKF dt: %.4fs, attitude: %d, local position: %d, global position: %d\n",
		     _instance, (double)_ekf.get_dt_ekf_avg(), _ekf.attitude_valid(),
		     _ekf.isLocalHorizontalPositionValid(), _ekf.isGlobalHorizontalPositionValid());

	perf_print_counter(_ekf_update_perf);
	perf_print_counter(_msg_missed_imu_perf);

	if (verbose) {
#if defined(CONFIG_EKF2_VERBOSE_STATUS)
		_ekf.print_status();
#endif // CONFIG_EKF2_VERBOSE_STATUS
	}

	return 0;
}

void EKF2::Run()
{
	if (should_exit()) {
		_sensor_combined_sub.unregisterCallback();
		_vehicle_imu_sub.unregisterCallback();

		return;
	}

	// check for parameter updates
	if (_parameter_update_sub.updated() || !_callback_registered) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		VerifyParams();

		// force advertise topics immediately for logging (EKF2_LOG_VERBOSE, per aid source control)
		AdvertiseTopics();

#if defined(CONFIG_EKF2_GNSS)
		_ekf.set_min_required_gps_health_time(_param_ekf2_req_gps_h.get() * 1_s);
#endif // CONFIG_EKF2_GNSS

		const matrix::Vector3f imu_pos_body(_param_ekf2_imu_pos_x.get(),
						    _param_ekf2_imu_pos_y.get(),
						    _param_ekf2_imu_pos_z.get());
		_ekf.output_predictor().set_imu_offset(imu_pos_body);
		_ekf.output_predictor().set_pos_correction_tc(_param_ekf2_tau_pos.get());
		_ekf.output_predictor().set_vel_correction_tc(_param_ekf2_tau_vel.get());

#if defined(CONFIG_EKF2_AIRSPEED)
		// The airspeed scale factor correcton is only available via parameter as used by the airspeed module
		param_t param_aspd_scale = param_find("ASPD_SCALE_1");

		if (param_aspd_scale != PARAM_INVALID) {
			param_get(param_aspd_scale, &_airspeed_scale_factor);
		}

#endif // CONFIG_EKF2_AIRSPEED

		_ekf.updateParameters();
	}

	if (!_callback_registered) {
#if defined(CONFIG_EKF2_MULTI_INSTANCE)

		if (_multi_mode) {
			_callback_registered = _vehicle_imu_sub.registerCallback();

		} else
#endif // CONFIG_EKF2_MULTI_INSTANCE
		{
			_callback_registered = _sensor_combined_sub.registerCallback();
		}

		if (!_callback_registered) {
			ScheduleDelayed(10_ms);
			return;
		}
	}

	if (_vehicle_command_sub.updated()) {
		vehicle_command_s vehicle_command;

		if (_vehicle_command_sub.update(&vehicle_command)) {

			vehicle_command_ack_s command_ack{};
			command_ack.command = vehicle_command.command;
			command_ack.target_system = vehicle_command.source_system;
			command_ack.target_component = vehicle_command.source_component;

			if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN) {
				double latitude = vehicle_command.param5;
				double longitude = vehicle_command.param6;
				float altitude = vehicle_command.param7;

				if (_ekf.setEkfGlobalOrigin(latitude, longitude, altitude)) {
					// Validate the ekf origin status.
					uint64_t origin_time {};
					_ekf.getEkfGlobalOrigin(origin_time, latitude, longitude, altitude);
					PX4_INFO("%d - New NED origin (LLA): %3.10f, %3.10f, %4.3f\n",
						 _instance, latitude, longitude, static_cast<double>(altitude));

					command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					PX4_ERR("%d - Failed to set new NED origin (LLA): %3.10f, %3.10f, %4.3f\n",
						_instance, latitude, longitude, static_cast<double>(altitude));

					command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
				}

				command_ack.timestamp = hrt_absolute_time();
				_vehicle_command_ack_pub.publish(command_ack);

			} else if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_EXTERNAL_POSITION_ESTIMATE) {

				if ((_ekf.control_status_flags().wind_dead_reckoning || _ekf.control_status_flags().inertial_dead_reckoning
				     || (!_ekf.control_status_flags().in_air && !_ekf.control_status_flags().gnss_pos))
				    && PX4_ISFINITE(vehicle_command.param2)
				    && PX4_ISFINITE(vehicle_command.param5) && PX4_ISFINITE(vehicle_command.param6)
				   ) {

					const float measurement_delay_seconds = math::constrain(vehicle_command.param2, 0.0f,
										kMaxDelaySecondsExternalPosMeasurement);
					const uint64_t timestamp_observation = vehicle_command.timestamp - measurement_delay_seconds * 1_s;

					float accuracy = kDefaultExternalPosAccuracy;

					if (PX4_ISFINITE(vehicle_command.param3) && vehicle_command.param3 > FLT_EPSILON) {
						accuracy = vehicle_command.param3;
					}

					if (_ekf.resetGlobalPosToExternalObservation(vehicle_command.param5, vehicle_command.param6, vehicle_command.param7,
							accuracy, accuracy, timestamp_observation)
					   ) {
						command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

					} else {
						command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
					}

				} else {
					command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED; // TODO: expand
				}

				command_ack.timestamp = hrt_absolute_time();
				_vehicle_command_ack_pub.publish(command_ack);
			}

			if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_EXTERNAL_WIND_ESTIMATE) {
#if defined(CONFIG_EKF2_WIND)
				// wind direction is given as azimuth where wind blows FROM
				// PX4 backend expects direction where wind blows TO
				const float wind_direction_rad = wrap_pi(math::radians(vehicle_command.param3) + M_PI_F);
				const float wind_direction_accuracy_rad = math::radians(vehicle_command.param4);
				_ekf.resetWindToExternalObservation(vehicle_command.param1, wind_direction_rad, vehicle_command.param2,
								    wind_direction_accuracy_rad);
				command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
#else
				command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
#endif // CONFIG_EKF2_WIND
				command_ack.timestamp = hrt_absolute_time();
				_vehicle_command_ack_pub.publish(command_ack);
			}
		}
	}

	bool imu_updated = false;
	imuSample imu_sample_new {};

	hrt_abstime imu_dt = 0; // for tracking time slip later

#if defined(CONFIG_EKF2_MULTI_INSTANCE)

	if (_multi_mode) {
		const unsigned last_generation = _vehicle_imu_sub.get_last_generation();
		vehicle_imu_s imu;
		imu_updated = _vehicle_imu_sub.update(&imu);

		if (imu_updated && (_vehicle_imu_sub.get_last_generation() != last_generation + 1)) {
			perf_count(_msg_missed_imu_perf);
		}

		if (imu_updated) {
			imu_sample_new.time_us = imu.timestamp_sample;
			imu_sample_new.delta_ang_dt = imu.delta_angle_dt * 1.e-6f;
			imu_sample_new.delta_ang = Vector3f{imu.delta_angle};
			imu_sample_new.delta_vel_dt = imu.delta_velocity_dt * 1.e-6f;
			imu_sample_new.delta_vel = Vector3f{imu.delta_velocity};

			if (imu.delta_velocity_clipping > 0) {
				imu_sample_new.delta_vel_clipping[0] = imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_X;
				imu_sample_new.delta_vel_clipping[1] = imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_Y;
				imu_sample_new.delta_vel_clipping[2] = imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_Z;
			}

			imu_dt = imu.delta_angle_dt;

			if ((_device_id_accel == 0) || (_device_id_gyro == 0)) {
				_device_id_accel = imu.accel_device_id;
				_device_id_gyro = imu.gyro_device_id;
				_accel_calibration_count = imu.accel_calibration_count;
				_gyro_calibration_count = imu.gyro_calibration_count;

			} else {
				if ((imu.accel_calibration_count != _accel_calibration_count)
				    || (imu.accel_device_id != _device_id_accel)) {

					PX4_DEBUG("%d - resetting accelerometer bias", _instance);
					_device_id_accel = imu.accel_device_id;

					_ekf.resetAccelBias();
					_accel_calibration_count = imu.accel_calibration_count;

					// reset bias learning
					_accel_cal = {};
				}

				if ((imu.gyro_calibration_count != _gyro_calibration_count)
				    || (imu.gyro_device_id != _device_id_gyro)) {

					PX4_DEBUG("%d - resetting rate gyro bias", _instance);
					_device_id_gyro = imu.gyro_device_id;

					_ekf.resetGyroBias();
					_gyro_calibration_count = imu.gyro_calibration_count;

					// reset bias learning
					_gyro_cal = {};
				}
			}
		}

	} else
#endif // CONFIG_EKF2_MULTI_INSTANCE
	{
		const unsigned last_generation = _sensor_combined_sub.get_last_generation();
		sensor_combined_s sensor_combined;
		imu_updated = _sensor_combined_sub.update(&sensor_combined);

		if (imu_updated && (_sensor_combined_sub.get_last_generation() != last_generation + 1)) {
			perf_count(_msg_missed_imu_perf);
		}

		if (imu_updated) {
			imu_sample_new.time_us = sensor_combined.timestamp;
			imu_sample_new.delta_ang_dt = sensor_combined.gyro_integral_dt * 1.e-6f;
			imu_sample_new.delta_ang = Vector3f{sensor_combined.gyro_rad} * imu_sample_new.delta_ang_dt;
			imu_sample_new.delta_vel_dt = sensor_combined.accelerometer_integral_dt * 1.e-6f;
			imu_sample_new.delta_vel = Vector3f{sensor_combined.accelerometer_m_s2} * imu_sample_new.delta_vel_dt;

			if (sensor_combined.accelerometer_clipping > 0) {
				imu_sample_new.delta_vel_clipping[0] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_X;
				imu_sample_new.delta_vel_clipping[1] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_Y;
				imu_sample_new.delta_vel_clipping[2] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_Z;
			}

			imu_dt = sensor_combined.gyro_integral_dt;

			if (sensor_combined.accel_calibration_count != _accel_calibration_count) {

				PX4_DEBUG("%d - resetting accelerometer bias", _instance);

				_ekf.resetAccelBias();
				_accel_calibration_count = sensor_combined.accel_calibration_count;

				// reset bias learning
				_accel_cal = {};
			}

			if (sensor_combined.gyro_calibration_count != _gyro_calibration_count) {

				PX4_DEBUG("%d - resetting rate gyro bias", _instance);

				_ekf.resetGyroBias();
				_gyro_calibration_count = sensor_combined.gyro_calibration_count;

				// reset bias learning
				_gyro_cal = {};
			}
		}

		if (_sensor_selection_sub.updated() || (_device_id_accel == 0 || _device_id_gyro == 0)) {
			sensor_selection_s sensor_selection;

			if (_sensor_selection_sub.copy(&sensor_selection)) {
				if (_device_id_accel != sensor_selection.accel_device_id) {

					_device_id_accel = sensor_selection.accel_device_id;

					_ekf.resetAccelBias();

					// reset bias learning
					_accel_cal = {};
				}

				if (_device_id_gyro != sensor_selection.gyro_device_id) {

					_device_id_gyro = sensor_selection.gyro_device_id;

					_ekf.resetGyroBias();

					// reset bias learning
					_gyro_cal = {};
				}
			}
		}
	}

	if (imu_updated) {
		const hrt_abstime now = imu_sample_new.time_us;

		// push imu data into estimator
		_ekf.setIMUData(imu_sample_new);
		PublishAttitude(now); // publish attitude immediately (uses quaternion from output predictor)

		// integrate time to monitor time slippage
		if (_start_time_us > 0) {
			_integrated_time_us += imu_dt;
			_last_time_slip_us = (imu_sample_new.time_us - _start_time_us) - _integrated_time_us;

		} else {
			_start_time_us = imu_sample_new.time_us;
			_last_time_slip_us = 0;
		}

		// ekf2_timestamps (using 0.1 ms relative timestamps)
		ekf2_timestamps_s ekf2_timestamps {
			.timestamp = now,
			.airspeed_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.airspeed_validated_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.distance_sensor_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.optical_flow_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.vehicle_air_data_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.vehicle_magnetometer_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.visual_odometry_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
		};

#if defined(CONFIG_EKF2_AIRSPEED)
		UpdateAirspeedSample(ekf2_timestamps);
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_AUXVEL)
		UpdateAuxVelSample(ekf2_timestamps);
#endif // CONFIG_EKF2_AUXVEL
#if defined(CONFIG_EKF2_BAROMETER)
		UpdateBaroSample(ekf2_timestamps);
#endif // CONFIG_EKF2_BAROMETER
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		UpdateExtVisionSample(ekf2_timestamps);
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
		UpdateFlowSample(ekf2_timestamps);
#endif // CONFIG_EKF2_OPTICAL_FLOW
#if defined(CONFIG_EKF2_GNSS)
		UpdateGpsSample(ekf2_timestamps);
#endif // CONFIG_EKF2_GNSS
#if defined(CONFIG_EKF2_MAGNETOMETER)
		UpdateMagSample(ekf2_timestamps);
#endif // CONFIG_EKF2_MAGNETOMETER
#if defined(CONFIG_EKF2_RANGE_FINDER)
		UpdateRangeSample(ekf2_timestamps);
#endif // CONFIG_EKF2_RANGE_FINDER
		UpdateSystemFlagsSample(ekf2_timestamps);

		// run the EKF update and output
		const hrt_abstime ekf_update_start = hrt_absolute_time();

		if (_ekf.update()) {
			perf_set_elapsed(_ekf_update_perf, hrt_elapsed_time(&ekf_update_start));

			PublishLocalPosition(now);
			PublishOdometry(now, imu_sample_new);
			PublishGlobalPosition(now);
			PublishSensorBias(now);

#if defined(CONFIG_EKF2_WIND)
			PublishWindEstimate(now);
#endif // CONFIG_EKF2_WIND

			// publish status/logging messages
			PublishEventFlags(now);
			PublishStatus(now);
			PublishStatusFlags(now);

			if (_param_ekf2_log_verbose.get()) {
				PublishAidSourceStatus(now);
				PublishInnovations(now);
				PublishInnovationTestRatios(now);
				PublishInnovationVariances(now);
				PublishStates(now);

#if defined(CONFIG_EKF2_BAROMETER)
				PublishBaroBias(now);
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
				PublishEvPosBias(now);
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)
				PublishGnssHgtBias(now);
#endif // CONFIG_EKF2_GNSS

			}

#if defined(CONFIG_EKF2_GNSS)
			PublishGpsStatus(now);
			PublishYawEstimatorStatus(now);
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
			PublishOpticalFlowVel(now);
#endif // CONFIG_EKF2_OPTICAL_FLOW

			UpdateAccelCalibration(now);
			UpdateGyroCalibration(now);
#if defined(CONFIG_EKF2_MAGNETOMETER)
			UpdateMagCalibration(now);
#endif // CONFIG_EKF2_MAGNETOMETER
		}

		// publish ekf2_timestamps
		_ekf2_timestamps_pub.publish(ekf2_timestamps);
	}

	// re-schedule as backup timeout
	ScheduleDelayed(100_ms);
}

void EKF2::VerifyParams()
{
#if defined(CONFIG_EKF2_MAGNETOMETER)

	// EKF2_MAG_TYPE obsolete options
	if ((_param_ekf2_mag_type.get() != MagFuseType::AUTO)
	    && (_param_ekf2_mag_type.get() != MagFuseType::HEADING)
	    && (_param_ekf2_mag_type.get() != MagFuseType::NONE)
	    && (_param_ekf2_mag_type.get() != MagFuseType::INIT)
	   ) {

		mavlink_log_critical(&_mavlink_log_pub, "EKF2_MAG_TYPE invalid, resetting to default");
		/* EVENT
		 * @description <param>EKF2_MAG_TYPE</param> is set to {1:.0}.
		 */
		events::send<float>(events::ID("ekf2_mag_type_invalid"), events::Log::Warning,
				    "EKF2_MAG_TYPE invalid, resetting to default", _param_ekf2_mag_type.get());

		_param_ekf2_mag_type.set(0);
		_param_ekf2_mag_type.commit();
	}

#endif // CONFIG_EKF2_MAGNETOMETER

	float delay_max = _param_ekf2_delay_max.get();

#if defined(CONFIG_EKF2_AUXVEL)

	if (_param_ekf2_avel_delay.get() > delay_max) {
		delay_max = _param_ekf2_avel_delay.get();
	}

#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_BAROMETER)

	if (_param_ekf2_baro_delay.get() > delay_max) {
		delay_max = _param_ekf2_baro_delay.get();
	}

#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_AIRSPEED)

	if (_param_ekf2_asp_delay.get() > delay_max) {
		delay_max = _param_ekf2_asp_delay.get();
	}

#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (_param_ekf2_mag_delay.get() > delay_max) {
		delay_max = _param_ekf2_mag_delay.get();
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_RANGE_FINDER)

	if (_param_ekf2_rng_delay.get() > delay_max) {
		delay_max = _param_ekf2_rng_delay.get();
	}

#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_GNSS)

	if (_param_ekf2_gps_delay.get() > delay_max) {
		delay_max = _param_ekf2_gps_delay.get();
	}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

	if (_param_ekf2_of_delay.get() > delay_max) {
		delay_max = _param_ekf2_of_delay.get();
	}

#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_param_ekf2_ev_delay.get() > delay_max) {
		delay_max = _param_ekf2_ev_delay.get();
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	if (delay_max > _param_ekf2_delay_max.get()) {
		/* EVENT
		 * @description EKF2_DELAY_MAX({1}ms) is too small compared to the maximum sensor delay ({2})
		 */
		events::send<float, float>(events::ID("nf_delay_max_too_small"), events::Log::Warning,
					   "EKF2_DELAY_MAX increased to {2}ms, please reboot", _param_ekf2_delay_max.get(),
					   delay_max);
		_param_ekf2_delay_max.commit_no_notification(delay_max);
	}
}

void EKF2::PublishAidSourceStatus(const hrt_abstime &timestamp)
{
#if defined(CONFIG_EKF2_AIRSPEED)
	// airspeed
	PublishAidSourceStatus(_ekf.aid_src_airspeed(), _status_airspeed_pub_last, _estimator_aid_src_airspeed_pub);
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_SIDESLIP)
	// sideslip
	PublishAidSourceStatus(_ekf.aid_src_sideslip(), _status_sideslip_pub_last, _estimator_aid_src_sideslip_pub);
#endif // CONFIG_EKF2_SIDESLIP
#if defined(CONFIG_EKF2_BAROMETER)
	// baro height
	PublishAidSourceStatus(_ekf.aid_src_baro_hgt(), _status_baro_hgt_pub_last, _estimator_aid_src_baro_hgt_pub);
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_DRAG_FUSION)
	// drag
	PublishAidSourceStatus(_ekf.aid_src_drag(), _status_drag_pub_last, _estimator_aid_src_drag_pub);
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// RNG height
	PublishAidSourceStatus(_ekf.aid_src_rng_hgt(), _status_rng_hgt_pub_last, _estimator_aid_src_rng_hgt_pub);
#endif // CONFIG_EKF2_RANGE_FINDER

	// fake position
	PublishAidSourceStatus(_ekf.aid_src_fake_pos(), _status_fake_pos_pub_last, _estimator_aid_src_fake_pos_pub);
	PublishAidSourceStatus(_ekf.aid_src_fake_hgt(), _status_fake_hgt_pub_last, _estimator_aid_src_fake_hgt_pub);

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// external vision (EV) hgt/pos/vel/yaw
	PublishAidSourceStatus(_ekf.aid_src_ev_hgt(), _status_ev_hgt_pub_last, _estimator_aid_src_ev_hgt_pub);
	PublishAidSourceStatus(_ekf.aid_src_ev_pos(), _status_ev_pos_pub_last, _estimator_aid_src_ev_pos_pub);
	PublishAidSourceStatus(_ekf.aid_src_ev_vel(), _status_ev_vel_pub_last, _estimator_aid_src_ev_vel_pub);
	PublishAidSourceStatus(_ekf.aid_src_ev_yaw(), _status_ev_yaw_pub_last, _estimator_aid_src_ev_yaw_pub);
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)
	// GNSS hgt/pos/vel/yaw
	PublishAidSourceStatus(_ekf.aid_src_gnss_hgt(), _status_gnss_hgt_pub_last, _estimator_aid_src_gnss_hgt_pub);
	PublishAidSourceStatus(_ekf.aid_src_gnss_pos(), _status_gnss_pos_pub_last, _estimator_aid_src_gnss_pos_pub);
	PublishAidSourceStatus(_ekf.aid_src_gnss_vel(), _status_gnss_vel_pub_last, _estimator_aid_src_gnss_vel_pub);
# if defined(CONFIG_EKF2_GNSS_YAW)
	PublishAidSourceStatus(_ekf.aid_src_gnss_yaw(), _status_gnss_yaw_pub_last, _estimator_aid_src_gnss_yaw_pub);
# endif // CONFIG_EKF2_GNSS_YAW
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// mag 3d
	PublishAidSourceStatus(_ekf.aid_src_mag(), _status_mag_pub_last, _estimator_aid_src_mag_pub);
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	// gravity
	PublishAidSourceStatus(_ekf.aid_src_gravity(), _status_gravity_pub_last, _estimator_aid_src_gravity_pub);
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_AUXVEL)
	// aux velocity
	PublishAidSourceStatus(_ekf.aid_src_aux_vel(), _status_aux_vel_pub_last, _estimator_aid_src_aux_vel_pub);
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// optical flow
	PublishAidSourceStatus(_ekf.aid_src_optical_flow(), _status_optical_flow_pub_last, _estimator_aid_src_optical_flow_pub);
#endif // CONFIG_EKF2_OPTICAL_FLOW
}

void EKF2::PublishAttitude(const hrt_abstime &timestamp)
{
	if (_ekf.attitude_valid()) {
		// generate vehicle attitude quaternion data
		vehicle_attitude_s att;
		att.timestamp_sample = timestamp;
		_ekf.getQuaternion().copyTo(att.q);

		_ekf.get_quat_reset(&att.delta_q_reset[0], &att.quat_reset_counter);
		att.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_attitude_pub.publish(att);

	}  else if (_replay_mode) {
		// in replay mode we have to tell the replay module not to wait for an update
		// we do this by publishing an attitude with zero timestamp
		vehicle_attitude_s att{};
		_attitude_pub.publish(att);
	}
}

#if defined(CONFIG_EKF2_BAROMETER)
void EKF2::PublishBaroBias(const hrt_abstime &timestamp)
{
	if (_ekf.aid_src_baro_hgt().timestamp_sample != 0) {
		const BiasEstimator::status &status = _ekf.getBaroBiasEstimatorStatus();

		if (fabsf(status.bias - _last_baro_bias_published) > 0.001f) {
			_estimator_baro_bias_pub.publish(fillEstimatorBiasMsg(status, _ekf.aid_src_baro_hgt().timestamp_sample, timestamp,
							 _device_id_baro));

			_last_baro_bias_published = status.bias;
		}
	}
}
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_GNSS)
void EKF2::PublishGnssHgtBias(const hrt_abstime &timestamp)
{
	if (_ekf.get_gps_sample_delayed().time_us != 0) {
		const BiasEstimator::status &status = _ekf.getGpsHgtBiasEstimatorStatus();

		if (fabsf(status.bias - _last_gnss_hgt_bias_published) > 0.001f) {
			_estimator_gnss_hgt_bias_pub.publish(fillEstimatorBiasMsg(status, _ekf.get_gps_sample_delayed().time_us, timestamp));

			_last_gnss_hgt_bias_published = status.bias;
		}
	}
}
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
void EKF2::PublishEvPosBias(const hrt_abstime &timestamp)
{
	if (_ekf.aid_src_ev_hgt().timestamp_sample) {

		estimator_bias3d_s bias{};

		// height
		BiasEstimator::status bias_est_status[3];
		bias_est_status[0] = _ekf.getEvPosBiasEstimatorStatus(0);
		bias_est_status[1] = _ekf.getEvPosBiasEstimatorStatus(1);
		bias_est_status[2] = _ekf.getEvHgtBiasEstimatorStatus();

		for (int i = 0; i < 3; i++) {
			bias.bias[i] = bias_est_status[i].bias;
			bias.bias_var[i] = bias_est_status[i].bias_var;

			bias.innov[i] = bias_est_status[i].innov;
			bias.innov_var[i] = bias_est_status[i].innov_var;
			bias.innov_test_ratio[i] = bias_est_status[i].innov_test_ratio;
		}

		const Vector3f bias_vec{bias.bias};

		if ((bias_vec - _last_ev_bias_published).longerThan(0.01f)) {
			bias.timestamp_sample = _ekf.aid_src_ev_hgt().timestamp_sample;
			bias.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
			_estimator_ev_pos_bias_pub.publish(bias);

			_last_ev_bias_published = Vector3f(bias.bias);
		}
	}
}
#endif // CONFIG_EKF2_EXTERNAL_VISION

estimator_bias_s EKF2::fillEstimatorBiasMsg(const BiasEstimator::status &status, uint64_t timestamp_sample_us,
		uint64_t timestamp, uint32_t device_id)
{
	estimator_bias_s bias{};
	bias.timestamp_sample = timestamp_sample_us;
	bias.device_id = device_id;
	bias.bias = status.bias;
	bias.bias_var = status.bias_var;
	bias.innov = status.innov;
	bias.innov_var = status.innov_var;
	bias.innov_test_ratio = status.innov_test_ratio;
	bias.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

	return bias;
}

void EKF2::PublishEventFlags(const hrt_abstime &timestamp)
{
	// information events
	uint32_t information_events = _ekf.information_event_status().value;
	bool information_event_updated = false;

	if (information_events != 0) {
		information_event_updated = true;
		_filter_information_event_changes++;
	}

	if (information_event_updated) {
		estimator_event_flags_s event_flags{};
		event_flags.timestamp_sample = _ekf.time_delayed_us();

		event_flags.information_event_changes           = _filter_information_event_changes;
		event_flags.gps_checks_passed                   = _ekf.information_event_flags().gps_checks_passed;
		event_flags.reset_vel_to_gps                    = _ekf.information_event_flags().reset_vel_to_gps;
		event_flags.reset_vel_to_flow                   = _ekf.information_event_flags().reset_vel_to_flow;
		event_flags.reset_vel_to_vision                 = _ekf.information_event_flags().reset_vel_to_vision;
		event_flags.reset_vel_to_zero                   = _ekf.information_event_flags().reset_vel_to_zero;
		event_flags.reset_pos_to_last_known             = _ekf.information_event_flags().reset_pos_to_last_known;
		event_flags.reset_pos_to_gps                    = _ekf.information_event_flags().reset_pos_to_gps;
		event_flags.reset_pos_to_vision                 = _ekf.information_event_flags().reset_pos_to_vision;
		event_flags.starting_gps_fusion                 = _ekf.information_event_flags().starting_gps_fusion;
		event_flags.starting_vision_pos_fusion          = _ekf.information_event_flags().starting_vision_pos_fusion;
		event_flags.starting_vision_vel_fusion          = _ekf.information_event_flags().starting_vision_vel_fusion;
		event_flags.starting_vision_yaw_fusion          = _ekf.information_event_flags().starting_vision_yaw_fusion;
		event_flags.yaw_aligned_to_imu_gps              = _ekf.information_event_flags().yaw_aligned_to_imu_gps;
		event_flags.reset_hgt_to_baro                   = _ekf.information_event_flags().reset_hgt_to_baro;
		event_flags.reset_hgt_to_gps                    = _ekf.information_event_flags().reset_hgt_to_gps;
		event_flags.reset_hgt_to_rng                    = _ekf.information_event_flags().reset_hgt_to_rng;
		event_flags.reset_hgt_to_ev                     = _ekf.information_event_flags().reset_hgt_to_ev;

		event_flags.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_estimator_event_flags_pub.update(event_flags);

		_last_event_flags_publish = event_flags.timestamp;

		_ekf.clear_information_events();

	} else if ((_last_event_flags_publish != 0) && (timestamp >= _last_event_flags_publish + 1_s)) {
		// continue publishing periodically
		_estimator_event_flags_pub.get().timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_estimator_event_flags_pub.update();
		_last_event_flags_publish = _estimator_event_flags_pub.get().timestamp;
	}
}

void EKF2::PublishGlobalPosition(const hrt_abstime &timestamp)
{
	if (_ekf.global_origin_valid() && _ekf.control_status().flags.yaw_align) {
		// generate and publish global position data
		vehicle_global_position_s global_pos{};
		global_pos.timestamp_sample = timestamp;

		// Position GPS / WGS84 frame
		const LatLonAlt lla = _ekf.getLatLonAlt();
		global_pos.lat = lla.latitude_deg();
		global_pos.lon = lla.longitude_deg();
		global_pos.lat_lon_valid = _ekf.isGlobalHorizontalPositionValid();

		global_pos.alt = lla.altitude();
		global_pos.alt_valid = _ekf.isGlobalVerticalPositionValid();

#if defined(CONFIG_EKF2_GNSS)
		global_pos.alt_ellipsoid = altAmslToEllipsoid(global_pos.alt);
#endif

		// global altitude has opposite sign of local down position
		float delta_z = 0.f;
		uint8_t z_reset_counter = 0;
		_ekf.get_posD_reset(&delta_z, &z_reset_counter);
		global_pos.delta_alt = -delta_z;
		global_pos.alt_reset_counter = z_reset_counter;

		float delta_xy[2] {};
		uint8_t xy_reset_counter = 0;
		_ekf.get_posNE_reset(delta_xy, &xy_reset_counter);
		global_pos.lat_lon_reset_counter = xy_reset_counter;

		_ekf.get_ekf_gpos_accuracy(&global_pos.eph, &global_pos.epv);

#if defined(CONFIG_EKF2_TERRAIN)

		// Terrain altitude in m, WGS84
		global_pos.terrain_alt = _ekf.getEkfGlobalOriginAltitude() - _ekf.getTerrainVertPos();
		global_pos.terrain_alt_valid = _ekf.isTerrainEstimateValid();

		float delta_hagl = 0.f;
		_ekf.get_hagl_reset(&delta_hagl, &global_pos.terrain_reset_counter);
		global_pos.delta_terrain = -delta_z;
#endif // CONFIG_EKF2_TERRAIN

		global_pos.dead_reckoning = _ekf.control_status_flags().inertial_dead_reckoning
					    || _ekf.control_status_flags().wind_dead_reckoning;

		global_pos.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_global_position_pub.publish(global_pos);
	}
}

#if defined(CONFIG_EKF2_GNSS)
void EKF2::PublishGpsStatus(const hrt_abstime &timestamp)
{
	const hrt_abstime timestamp_sample = _ekf.get_gps_sample_delayed().time_us;

	if (timestamp_sample == _last_gps_status_published) {
		return;
	}

	estimator_gps_status_s estimator_gps_status{};
	estimator_gps_status.timestamp_sample = timestamp_sample;

	estimator_gps_status.position_drift_rate_horizontal_m_s = _ekf.gps_horizontal_position_drift_rate_m_s();
	estimator_gps_status.position_drift_rate_vertical_m_s   = _ekf.gps_vertical_position_drift_rate_m_s();
	estimator_gps_status.filtered_horizontal_speed_m_s      = _ekf.gps_filtered_horizontal_velocity_m_s();

	estimator_gps_status.checks_passed = _ekf.gps_checks_passed();

	estimator_gps_status.check_fail_gps_fix          = _ekf.gps_check_fail_status_flags().fix;
	estimator_gps_status.check_fail_min_sat_count    = _ekf.gps_check_fail_status_flags().nsats;
	estimator_gps_status.check_fail_max_pdop         = _ekf.gps_check_fail_status_flags().pdop;
	estimator_gps_status.check_fail_max_horz_err     = _ekf.gps_check_fail_status_flags().hacc;
	estimator_gps_status.check_fail_max_vert_err     = _ekf.gps_check_fail_status_flags().vacc;
	estimator_gps_status.check_fail_max_spd_err      = _ekf.gps_check_fail_status_flags().sacc;
	estimator_gps_status.check_fail_max_horz_drift   = _ekf.gps_check_fail_status_flags().hdrift;
	estimator_gps_status.check_fail_max_vert_drift   = _ekf.gps_check_fail_status_flags().vdrift;
	estimator_gps_status.check_fail_max_horz_spd_err = _ekf.gps_check_fail_status_flags().hspeed;
	estimator_gps_status.check_fail_max_vert_spd_err = _ekf.gps_check_fail_status_flags().vspeed;
	estimator_gps_status.check_fail_spoofed_gps      = _ekf.gps_check_fail_status_flags().spoofed;

	estimator_gps_status.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_gps_status_pub.publish(estimator_gps_status);


	_last_gps_status_published = timestamp_sample;
}
#endif // CONFIG_EKF2_GNSS

void EKF2::PublishInnovations(const hrt_abstime &timestamp)
{
	// publish estimator innovation data
	estimator_innovations_s innovations{};
	innovations.timestamp_sample = _ekf.time_delayed_us();

#if defined(CONFIG_EKF2_GNSS)
	// GPS
	innovations.gps_hvel[0] = _ekf.aid_src_gnss_vel().innovation[0];
	innovations.gps_hvel[1] = _ekf.aid_src_gnss_vel().innovation[1];
	innovations.gps_vvel    = _ekf.aid_src_gnss_vel().innovation[2];
	innovations.gps_hpos[0] = _ekf.aid_src_gnss_pos().innovation[0];
	innovations.gps_hpos[1] = _ekf.aid_src_gnss_pos().innovation[1];
	innovations.gps_vpos    = _ekf.aid_src_gnss_hgt().innovation;
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// External Vision
	innovations.ev_hvel[0] = _ekf.aid_src_ev_vel().innovation[0];
	innovations.ev_hvel[1] = _ekf.aid_src_ev_vel().innovation[1];
	innovations.ev_vvel    = _ekf.aid_src_ev_vel().innovation[2];
	innovations.ev_hpos[0] = _ekf.aid_src_ev_pos().innovation[0];
	innovations.ev_hpos[1] = _ekf.aid_src_ev_pos().innovation[1];
	innovations.ev_vpos    = _ekf.aid_src_ev_hgt().innovation;
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// Height sensors
#if defined(CONFIG_EKF2_RANGE_FINDER)
	innovations.rng_vpos = _ekf.aid_src_rng_hgt().innovation;
#endif // CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_BAROMETER)
	innovations.baro_vpos = _ekf.aid_src_baro_hgt().innovation;
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_AUXVEL)
	// Auxiliary velocity
	innovations.aux_hvel[0] = _ekf.aid_src_aux_vel().innovation[0];
	innovations.aux_hvel[1] = _ekf.aid_src_aux_vel().innovation[1];
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// Optical flow
	innovations.flow[0] = _ekf.aid_src_optical_flow().innovation[0];
	innovations.flow[1] = _ekf.aid_src_optical_flow().innovation[1];
#endif // CONFIG_EKF2_OPTICAL_FLOW

	// heading
	innovations.heading = _ekf.getHeadingInnov();

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// mag_field
	innovations.mag_field[0] = _ekf.aid_src_mag().innovation[0];
	innovations.mag_field[1] = _ekf.aid_src_mag().innovation[1];
	innovations.mag_field[2] = _ekf.aid_src_mag().innovation[2];
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	// gravity
	innovations.gravity[0] = _ekf.aid_src_gravity().innovation[0];
	innovations.gravity[1] = _ekf.aid_src_gravity().innovation[1];
	innovations.gravity[2] = _ekf.aid_src_gravity().innovation[2];
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_DRAG_FUSION)
	// drag
	innovations.drag[0] = _ekf.aid_src_drag().innovation[0];
	innovations.drag[1] = _ekf.aid_src_drag().innovation[1];
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_AIRSPEED)
	// airspeed
	innovations.airspeed = _ekf.aid_src_airspeed().innovation;
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	// beta
	innovations.beta = _ekf.aid_src_sideslip().innovation;
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_TERRAIN) && defined(CONFIG_EKF2_RANGE_FINDER)
	// hagl
	innovations.hagl = _ekf.aid_src_rng_hgt().innovation;
#endif // CONFIG_EKF2_TERRAIN && CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// hagl_rate
	innovations.hagl_rate = _ekf.getHaglRateInnov();
#endif // CONFIG_EKF2_RANGE_FINDER

	innovations.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_innovations_pub.publish(innovations);
}

void EKF2::PublishInnovationTestRatios(const hrt_abstime &timestamp)
{
	// publish estimator innovation test ratio data
	estimator_innovations_s test_ratios{};
	test_ratios.timestamp_sample = _ekf.time_delayed_us();

#if defined(CONFIG_EKF2_GNSS)
	// GPS
	test_ratios.gps_hvel[0] = _ekf.aid_src_gnss_vel().test_ratio[0];
	test_ratios.gps_hvel[1] = _ekf.aid_src_gnss_vel().test_ratio[1];
	test_ratios.gps_vvel    = _ekf.aid_src_gnss_vel().test_ratio[2];
	test_ratios.gps_hpos[0] = _ekf.aid_src_gnss_pos().test_ratio[0];
	test_ratios.gps_hpos[1] = _ekf.aid_src_gnss_pos().test_ratio[1];
	test_ratios.gps_vpos    = _ekf.aid_src_gnss_hgt().test_ratio;
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// External Vision
	test_ratios.ev_hvel[0] = _ekf.aid_src_ev_vel().test_ratio[0];
	test_ratios.ev_hvel[1] = _ekf.aid_src_ev_vel().test_ratio[1];
	test_ratios.ev_vvel    = _ekf.aid_src_ev_vel().test_ratio[2];
	test_ratios.ev_hpos[0] = _ekf.aid_src_ev_pos().test_ratio[0];
	test_ratios.ev_hpos[1] = _ekf.aid_src_ev_pos().test_ratio[1];
	test_ratios.ev_vpos    = _ekf.aid_src_ev_hgt().test_ratio;
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// Height sensors
#if defined(CONFIG_EKF2_RANGE_FINDER)
	test_ratios.rng_vpos = _ekf.aid_src_rng_hgt().test_ratio;
#endif // CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_BAROMETER)
	test_ratios.baro_vpos = _ekf.aid_src_baro_hgt().test_ratio;
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_AUXVEL)
	// Auxiliary velocity
	test_ratios.aux_hvel[0] = _ekf.aid_src_aux_vel().test_ratio[0];
	test_ratios.aux_hvel[1] = _ekf.aid_src_aux_vel().test_ratio[1];
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// Optical flow
	test_ratios.flow[0] = _ekf.aid_src_optical_flow().test_ratio[0];
	test_ratios.flow[1] = _ekf.aid_src_optical_flow().test_ratio[1];
#endif // CONFIG_EKF2_OPTICAL_FLOW

	// heading
	test_ratios.heading = _ekf.getHeadingInnovRatio();

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// mag_field
	test_ratios.mag_field[0] = _ekf.aid_src_mag().test_ratio[0];
	test_ratios.mag_field[1] = _ekf.aid_src_mag().test_ratio[1];
	test_ratios.mag_field[2] = _ekf.aid_src_mag().test_ratio[2];
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	// gravity
	test_ratios.gravity[0] = _ekf.aid_src_gravity().test_ratio[0];
	test_ratios.gravity[1] = _ekf.aid_src_gravity().test_ratio[1];
	test_ratios.gravity[2] = _ekf.aid_src_gravity().test_ratio[2];
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_DRAG_FUSION)
	// drag
	test_ratios.drag[0] = _ekf.aid_src_drag().test_ratio[0];
	test_ratios.drag[1] = _ekf.aid_src_drag().test_ratio[1];
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_AIRSPEED)
	// airspeed
	test_ratios.airspeed = _ekf.aid_src_airspeed().test_ratio;
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	// beta
	test_ratios.beta = _ekf.aid_src_sideslip().test_ratio;
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_TERRAIN) && defined(CONFIG_EKF2_RANGE_FINDER)
	// hagl
	test_ratios.hagl = _ekf.aid_src_rng_hgt().test_ratio;
#endif // CONFIG_EKF2_TERRAIN && CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// hagl_rate
	test_ratios.hagl_rate = _ekf.getHaglRateInnovRatio();
#endif // CONFIG_EKF2_RANGE_FINDER

	test_ratios.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_innovation_test_ratios_pub.publish(test_ratios);
}

void EKF2::PublishInnovationVariances(const hrt_abstime &timestamp)
{
	// publish estimator innovation variance data
	estimator_innovations_s variances{};
	variances.timestamp_sample = _ekf.time_delayed_us();

#if defined(CONFIG_EKF2_GNSS)
	// GPS
	variances.gps_hvel[0] = _ekf.aid_src_gnss_vel().innovation_variance[0];
	variances.gps_hvel[1] = _ekf.aid_src_gnss_vel().innovation_variance[1];
	variances.gps_vvel    = _ekf.aid_src_gnss_vel().innovation_variance[2];
	variances.gps_hpos[0] = _ekf.aid_src_gnss_pos().innovation_variance[0];
	variances.gps_hpos[1] = _ekf.aid_src_gnss_pos().innovation_variance[1];
	variances.gps_vpos    = _ekf.aid_src_gnss_hgt().innovation_variance;
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// External Vision
	variances.ev_hvel[0] = _ekf.aid_src_ev_vel().innovation_variance[0];
	variances.ev_hvel[1] = _ekf.aid_src_ev_vel().innovation_variance[1];
	variances.ev_vvel    = _ekf.aid_src_ev_vel().innovation_variance[2];
	variances.ev_hpos[0] = _ekf.aid_src_ev_pos().innovation_variance[0];
	variances.ev_hpos[1] = _ekf.aid_src_ev_pos().innovation_variance[1];
	variances.ev_vpos    = _ekf.aid_src_ev_hgt().innovation_variance;
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// Height sensors
#if defined(CONFIG_EKF2_RANGE_FINDER)
	variances.rng_vpos = _ekf.aid_src_rng_hgt().innovation_variance;
#endif // CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_BAROMETER)
	variances.baro_vpos = _ekf.aid_src_baro_hgt().innovation_variance;
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_AUXVEL)
	// Auxiliary velocity
	variances.aux_hvel[0] = _ekf.aid_src_aux_vel().innovation_variance[0];
	variances.aux_hvel[1] = _ekf.aid_src_aux_vel().innovation_variance[1];
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// Optical flow
	variances.flow[0] = _ekf.aid_src_optical_flow().innovation_variance[0];
	variances.flow[1] = _ekf.aid_src_optical_flow().innovation_variance[1];
#endif // CONFIG_EKF2_OPTICAL_FLOW

	// heading
	variances.heading = _ekf.getHeadingInnovVar();

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// mag_field
	variances.mag_field[0] = _ekf.aid_src_mag().innovation_variance[0];
	variances.mag_field[1] = _ekf.aid_src_mag().innovation_variance[1];
	variances.mag_field[2] = _ekf.aid_src_mag().innovation_variance[2];
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	// gravity
	variances.gravity[0] = _ekf.aid_src_gravity().innovation_variance[0];
	variances.gravity[1] = _ekf.aid_src_gravity().innovation_variance[1];
	variances.gravity[2] = _ekf.aid_src_gravity().innovation_variance[2];
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_DRAG_FUSION)
	// drag
	variances.drag[0] = _ekf.aid_src_drag().innovation_variance[0];
	variances.drag[1] = _ekf.aid_src_drag().innovation_variance[1];
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_AIRSPEED)
	// airspeed
	variances.airspeed = _ekf.aid_src_airspeed().innovation_variance;
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	// beta
	variances.beta = _ekf.aid_src_sideslip().innovation_variance;
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_TERRAIN) && defined(CONFIG_EKF2_RANGE_FINDER)
	// hagl
	variances.hagl = _ekf.aid_src_rng_hgt().innovation_variance;
#endif // CONFIG_EKF2_TERRAIN && CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// hagl_rate
	variances.hagl_rate = _ekf.getHaglRateInnovVar();
#endif // CONFIG_EKF2_RANGE_FINDER

	variances.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_innovation_variances_pub.publish(variances);
}

void EKF2::PublishLocalPosition(const hrt_abstime &timestamp)
{
	vehicle_local_position_s lpos{};
	// generate vehicle local position data
	lpos.timestamp_sample = timestamp;

	// Position of body origin in local NED frame
	const Vector3f position{_ekf.getPosition()};
	lpos.x = position(0);
	lpos.y = position(1);
	lpos.z = position(2);

	// Velocity of body origin in local NED frame (m/s)
	const Vector3f velocity{_ekf.getVelocity()};
	lpos.vx = velocity(0);
	lpos.vy = velocity(1);
	lpos.vz = velocity(2);

	// vertical position time derivative (m/s)
	lpos.z_deriv = _ekf.getVerticalPositionDerivative();

	// Acceleration of body origin in local frame
	const Vector3f vel_deriv{_ekf.getVelocityDerivative()};
	_ekf.resetVelocityDerivativeAccumulation();
	lpos.ax = vel_deriv(0);
	lpos.ay = vel_deriv(1);
	lpos.az = vel_deriv(2);

	lpos.xy_valid = _ekf.isLocalHorizontalPositionValid();
	lpos.v_xy_valid = _ekf.isLocalHorizontalPositionValid();

	// TODO: some modules (e.g.: mc_pos_control) don't handle v_z_valid != z_valid properly
	lpos.z_valid = _ekf.isLocalVerticalPositionValid() || _ekf.isLocalVerticalVelocityValid();
	lpos.v_z_valid = _ekf.isLocalVerticalVelocityValid() || _ekf.isLocalVerticalPositionValid();

	// Position of local NED origin in GPS / WGS84 frame
	if (_ekf.global_origin_valid()) {
		lpos.ref_timestamp = _ekf.global_origin().getProjectionReferenceTimestamp();
		lpos.ref_lat = _ekf.global_origin().getProjectionReferenceLat(); // Reference point latitude in degrees
		lpos.ref_lon = _ekf.global_origin().getProjectionReferenceLon(); // Reference point longitude in degrees
		lpos.ref_alt = _ekf.getEkfGlobalOriginAltitude();           // Reference point in MSL altitude meters
		lpos.xy_global = true;
		lpos.z_global = true;

	} else {
		lpos.ref_timestamp = 0;
		lpos.ref_lat = static_cast<double>(NAN);
		lpos.ref_lon = static_cast<double>(NAN);
		lpos.ref_alt = NAN;
		lpos.xy_global = false;
		lpos.z_global = false;
	}

	Quatf delta_q_reset;
	_ekf.get_quat_reset(&delta_q_reset(0), &lpos.heading_reset_counter);

	lpos.heading = Eulerf(_ekf.getQuaternion()).psi();
	lpos.unaided_heading = _ekf.getUnaidedYaw();
	lpos.heading_var = _ekf.getYawVar();
	lpos.delta_heading = Eulerf(delta_q_reset).psi();
	lpos.heading_good_for_control = _ekf.isYawFinalAlignComplete();
	lpos.tilt_var = _ekf.getTiltVariance();

#if defined(CONFIG_EKF2_TERRAIN)
	// Distance to bottom surface (ground) in meters, must be positive
	lpos.dist_bottom_valid = _ekf.isTerrainEstimateValid();
	lpos.dist_bottom = math::max(_ekf.getHagl(), 0.f);
	lpos.dist_bottom_var = _ekf.getTerrainVariance();
	_ekf.get_hagl_reset(&lpos.delta_dist_bottom, &lpos.dist_bottom_reset_counter);

	lpos.dist_bottom_sensor_bitfield = vehicle_local_position_s::DIST_BOTTOM_SENSOR_NONE;

	if (_ekf.control_status_flags().rng_terrain) {
		lpos.dist_bottom_sensor_bitfield |= vehicle_local_position_s::DIST_BOTTOM_SENSOR_RANGE;
	}

	if (_ekf.control_status_flags().opt_flow_terrain) {
		lpos.dist_bottom_sensor_bitfield |= vehicle_local_position_s::DIST_BOTTOM_SENSOR_FLOW;
	}

#endif // CONFIG_EKF2_TERRAIN

	_ekf.get_ekf_lpos_accuracy(&lpos.eph, &lpos.epv);
	_ekf.get_ekf_vel_accuracy(&lpos.evh, &lpos.evv);

	// get state reset information of position and velocity
	_ekf.get_posD_reset(&lpos.delta_z, &lpos.z_reset_counter);
	_ekf.get_velD_reset(&lpos.delta_vz, &lpos.vz_reset_counter);
	_ekf.get_posNE_reset(&lpos.delta_xy[0], &lpos.xy_reset_counter);
	_ekf.get_velNE_reset(&lpos.delta_vxy[0], &lpos.vxy_reset_counter);

	lpos.dead_reckoning = _ekf.control_status_flags().inertial_dead_reckoning
			      || _ekf.control_status_flags().wind_dead_reckoning;

	// get control limit information
	_ekf.get_ekf_ctrl_limits(&lpos.vxy_max, &lpos.vz_max, &lpos.hagl_min, &lpos.hagl_max_z, &lpos.hagl_max_xy);

	// convert NaN to INFINITY
	if (!PX4_ISFINITE(lpos.vxy_max)) {
		lpos.vxy_max = INFINITY;
	}

	if (!PX4_ISFINITE(lpos.vz_max)) {
		lpos.vz_max = INFINITY;
	}

	if (!PX4_ISFINITE(lpos.hagl_min)) {
		lpos.hagl_min = INFINITY;
	}

	if (!PX4_ISFINITE(lpos.hagl_max_z)) {
		lpos.hagl_max_z = INFINITY;
	}

	if (!PX4_ISFINITE(lpos.hagl_max_xy)) {
		lpos.hagl_max_xy = INFINITY;
	}

	// publish vehicle local position data
	lpos.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_local_position_pub.publish(lpos);
}

void EKF2::PublishOdometry(const hrt_abstime &timestamp, const imuSample &imu_sample)
{
	// generate vehicle odometry data
	vehicle_odometry_s odom;
	odom.timestamp_sample = imu_sample.time_us;

	// position
	odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
	_ekf.getPosition().copyTo(odom.position);

	// orientation quaternion
	_ekf.getQuaternion().copyTo(odom.q);

	// velocity
	odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;
	_ekf.getVelocity().copyTo(odom.velocity);

	// angular_velocity
	const Vector3f rates{imu_sample.delta_ang / imu_sample.delta_ang_dt};
	const Vector3f angular_velocity = rates - _ekf.getGyroBias();
	angular_velocity.copyTo(odom.angular_velocity);

	// velocity covariances
	_ekf.getVelocityVariance().copyTo(odom.velocity_variance);

	// position covariances
	_ekf.getPositionVariance().copyTo(odom.position_variance);

	// orientation covariance
	_ekf.getRotVarBody().copyTo(odom.orientation_variance);

	odom.reset_counter = _ekf.get_quat_reset_count()
			     + _ekf.get_velNE_reset_count() + _ekf.get_velD_reset_count()
			     + _ekf.get_posNE_reset_count() + _ekf.get_posD_reset_count();

	odom.quality = 0;

	// publish vehicle odometry data
	odom.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_odometry_pub.publish(odom);
}

void EKF2::PublishSensorBias(const hrt_abstime &timestamp)
{
	// estimator_sensor_bias
	const Vector3f gyro_bias{_ekf.getGyroBias()};
	const Vector3f accel_bias{_ekf.getAccelBias()};

#if defined(CONFIG_EKF2_MAGNETOMETER)
	const Vector3f mag_bias {_ekf.getMagBias()};
#endif // CONFIG_EKF2_MAGNETOMETER

	// publish at ~1 Hz, or sooner if there's a change
	if ((gyro_bias - _last_gyro_bias_published).longerThan(0.001f)
	    || (accel_bias - _last_accel_bias_published).longerThan(0.001f)
#if defined(CONFIG_EKF2_MAGNETOMETER)
	    || (mag_bias - _last_mag_bias_published).longerThan(0.001f)
#endif // CONFIG_EKF2_MAGNETOMETER
	    || (timestamp >= _last_sensor_bias_published + 1_s)) {

		estimator_sensor_bias_s bias{};
		bias.timestamp_sample = _ekf.time_delayed_us();

		// take device ids from sensor_selection_s if not using specific vehicle_imu_s
		if ((_device_id_gyro != 0) && (_param_ekf2_imu_ctrl.get() & static_cast<int32_t>(ImuCtrl::GyroBias))) {
			const Vector3f bias_var{_ekf.getGyroBiasVariance()};

			bias.gyro_device_id = _device_id_gyro;
			gyro_bias.copyTo(bias.gyro_bias);
			bias.gyro_bias_limit = _ekf.getGyroBiasLimit();
			bias_var.copyTo(bias.gyro_bias_variance);
			bias.gyro_bias_valid = bias_var.longerThan(0.f) && !bias_var.longerThan(0.1f);
			bias.gyro_bias_stable = _gyro_cal.cal_available;
			_last_gyro_bias_published = gyro_bias;
		}

		if ((_device_id_accel != 0) && (_param_ekf2_imu_ctrl.get() & static_cast<int32_t>(ImuCtrl::AccelBias))) {
			const Vector3f bias_var{_ekf.getAccelBiasVariance()};

			bias.accel_device_id = _device_id_accel;
			accel_bias.copyTo(bias.accel_bias);
			bias.accel_bias_limit = _ekf.getAccelBiasLimit();
			bias_var.copyTo(bias.accel_bias_variance);
			bias.accel_bias_valid = bias_var.longerThan(0.f) && !bias_var.longerThan(0.1f);
			bias.accel_bias_stable = _accel_cal.cal_available;
			_last_accel_bias_published = accel_bias;
		}

#if defined(CONFIG_EKF2_MAGNETOMETER)

		if (_device_id_mag != 0) {
			const Vector3f bias_var{_ekf.getMagBiasVariance()};

			bias.mag_device_id = _device_id_mag;
			mag_bias.copyTo(bias.mag_bias);
			bias.mag_bias_limit = _ekf.getMagBiasLimit();
			bias_var.copyTo(bias.mag_bias_variance);
			bias.mag_bias_valid = bias_var.longerThan(0.f) && !bias_var.longerThan(0.1f);
			bias.mag_bias_stable = _mag_cal.cal_available;
			_last_mag_bias_published = mag_bias;
		}

#endif // CONFIG_EKF2_MAGNETOMETER

		bias.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_estimator_sensor_bias_pub.publish(bias);

		_last_sensor_bias_published = bias.timestamp;
	}
}

void EKF2::PublishStates(const hrt_abstime &timestamp)
{
	// publish estimator states
	estimator_states_s states;
	states.timestamp_sample = _ekf.time_delayed_us();
	const auto state_vector = _ekf.state().vector();
	state_vector.copyTo(states.states);
	states.n_states = state_vector.size();
	_ekf.covariances_diagonal().copyTo(states.covariances);
	states.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_states_pub.publish(states);
}

void EKF2::PublishStatus(const hrt_abstime &timestamp)
{
	estimator_status_s status{};
	status.timestamp_sample = _ekf.time_delayed_us();

	_ekf.getOutputTrackingError().copyTo(status.output_tracking_error);

#if defined(CONFIG_EKF2_GNSS)
	// only report enabled GPS check failures (the param indexes are shifted by 1 bit, because they don't include
	// the GPS Fix bit, which is always checked)
	status.gps_check_fail_flags = _ekf.gps_check_fail_status().value & (((uint16_t)_params->gps_check_mask << 1) | 1);
#endif // CONFIG_EKF2_GNSS

	status.control_mode_flags = _ekf.control_status().value;
	status.filter_fault_flags = _ekf.fault_status().value;

	// vel_test_ratio
	float vel_xy_test_ratio = _ekf.getHorizontalVelocityInnovationTestRatio();
	float vel_z_test_ratio = _ekf.getVerticalVelocityInnovationTestRatio();

	if (PX4_ISFINITE(vel_xy_test_ratio) && PX4_ISFINITE(vel_z_test_ratio)) {
		status.vel_test_ratio = math::max(vel_xy_test_ratio, vel_z_test_ratio);

	} else if (PX4_ISFINITE(vel_xy_test_ratio)) {
		status.vel_test_ratio = vel_xy_test_ratio;

	} else if (PX4_ISFINITE(vel_z_test_ratio)) {
		status.vel_test_ratio = vel_z_test_ratio;

	} else {
		status.vel_test_ratio = NAN;
	}

	status.hdg_test_ratio = _ekf.getHeadingInnovationTestRatio();
	status.pos_test_ratio = _ekf.getHorizontalPositionInnovationTestRatio();
	status.hgt_test_ratio = _ekf.getVerticalPositionInnovationTestRatio();
	status.tas_test_ratio = _ekf.getAirspeedInnovationTestRatio();
	status.hagl_test_ratio = _ekf.getHeightAboveGroundInnovationTestRatio();
	status.beta_test_ratio = _ekf.getSyntheticSideslipInnovationTestRatio();

	_ekf.get_ekf_lpos_accuracy(&status.pos_horiz_accuracy, &status.pos_vert_accuracy);
	status.solution_status_flags = _ekf.get_ekf_soln_status();

	// reset counters
	status.reset_count_vel_ne = _ekf.state_reset_status().reset_count.velNE;
	status.reset_count_vel_d = _ekf.state_reset_status().reset_count.velD;
	status.reset_count_pos_ne = _ekf.state_reset_status().reset_count.posNE;
	status.reset_count_pod_d = _ekf.state_reset_status().reset_count.posD;
	status.reset_count_quat = _ekf.state_reset_status().reset_count.quat;

	status.time_slip = _last_time_slip_us * 1e-6f;

	static constexpr float kMinTestRatioPreflight = 0.5f;
	status.pre_flt_fail_innov_heading   = (kMinTestRatioPreflight < status.hdg_test_ratio);
	status.pre_flt_fail_innov_height    = (kMinTestRatioPreflight < status.hgt_test_ratio);
	status.pre_flt_fail_innov_pos_horiz = (kMinTestRatioPreflight < status.pos_test_ratio);
	status.pre_flt_fail_innov_vel_horiz = (kMinTestRatioPreflight < vel_xy_test_ratio);
	status.pre_flt_fail_innov_vel_vert  = (kMinTestRatioPreflight < vel_z_test_ratio);

	status.pre_flt_fail_mag_field_disturbed = _ekf.control_status_flags().mag_field_disturbed;

	status.accel_device_id = _device_id_accel;
#if defined(CONFIG_EKF2_BAROMETER)
	status.baro_device_id = _device_id_baro;
#endif // CONFIG_EKF2_BAROMETER
	status.gyro_device_id = _device_id_gyro;

#if defined(CONFIG_EKF2_MAGNETOMETER)
	status.mag_device_id = _device_id_mag;

	_ekf.get_mag_checks(status.mag_inclination_deg, status.mag_inclination_ref_deg, status.mag_strength_gs,
			    status.mag_strength_ref_gs);
#endif // CONFIG_EKF2_MAGNETOMETER

	status.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_status_pub.publish(status);
}

void EKF2::PublishStatusFlags(const hrt_abstime &timestamp)
{
	// publish at ~ 1 Hz (or immediately if filter control status or fault status changes)
	bool update = (timestamp >= _last_status_flags_publish + 1_s);

	// filter control status
	if (_ekf.control_status().value != _filter_control_status) {
		update = true;
		_filter_control_status = _ekf.control_status().value;
		_filter_control_status_changes++;
	}

	// filter fault status
	if (_ekf.fault_status().value != _filter_fault_status) {
		update = true;
		_filter_fault_status = _ekf.fault_status().value;
		_filter_fault_status_changes++;
	}

	// innovation check fail status
	if (_ekf.innov_check_fail_status().value != _innov_check_fail_status) {
		update = true;
		_innov_check_fail_status = _ekf.innov_check_fail_status().value;
		_innov_check_fail_status_changes++;
	}

	if (update) {
		estimator_status_flags_s status_flags{};
		status_flags.timestamp_sample = _ekf.time_delayed_us();

		status_flags.control_status_changes   = _filter_control_status_changes;
		status_flags.cs_tilt_align            = _ekf.control_status_flags().tilt_align;
		status_flags.cs_yaw_align             = _ekf.control_status_flags().yaw_align;
		status_flags.cs_gnss_pos              = _ekf.control_status_flags().gnss_pos;
		status_flags.cs_opt_flow              = _ekf.control_status_flags().opt_flow;
		status_flags.cs_mag_hdg               = _ekf.control_status_flags().mag_hdg;
		status_flags.cs_mag_3d                = _ekf.control_status_flags().mag_3D;
		status_flags.cs_mag_dec               = _ekf.control_status_flags().mag_dec;
		status_flags.cs_in_air                = _ekf.control_status_flags().in_air;
		status_flags.cs_wind                  = _ekf.control_status_flags().wind;
		status_flags.cs_baro_hgt              = _ekf.control_status_flags().baro_hgt;
		status_flags.cs_rng_hgt               = _ekf.control_status_flags().rng_hgt;
		status_flags.cs_gps_hgt               = _ekf.control_status_flags().gps_hgt;
		status_flags.cs_ev_pos                = _ekf.control_status_flags().ev_pos;
		status_flags.cs_ev_yaw                = _ekf.control_status_flags().ev_yaw;
		status_flags.cs_ev_hgt                = _ekf.control_status_flags().ev_hgt;
		status_flags.cs_fuse_beta             = _ekf.control_status_flags().fuse_beta;
		status_flags.cs_mag_field_disturbed   = _ekf.control_status_flags().mag_field_disturbed;
		status_flags.cs_fixed_wing            = _ekf.control_status_flags().fixed_wing;
		status_flags.cs_mag_fault             = _ekf.control_status_flags().mag_fault;
		status_flags.cs_fuse_aspd             = _ekf.control_status_flags().fuse_aspd;
		status_flags.cs_gnd_effect            = _ekf.control_status_flags().gnd_effect;
		status_flags.cs_rng_stuck             = _ekf.control_status_flags().rng_stuck;
		status_flags.cs_gnss_yaw               = _ekf.control_status_flags().gnss_yaw;
		status_flags.cs_mag_aligned_in_flight = _ekf.control_status_flags().mag_aligned_in_flight;
		status_flags.cs_ev_vel                = _ekf.control_status_flags().ev_vel;
		status_flags.cs_synthetic_mag_z       = _ekf.control_status_flags().synthetic_mag_z;
		status_flags.cs_vehicle_at_rest       = _ekf.control_status_flags().vehicle_at_rest;
		status_flags.cs_gnss_yaw_fault         = _ekf.control_status_flags().gnss_yaw_fault;
		status_flags.cs_rng_fault             = _ekf.control_status_flags().rng_fault;
		status_flags.cs_inertial_dead_reckoning = _ekf.control_status_flags().inertial_dead_reckoning;
		status_flags.cs_wind_dead_reckoning     = _ekf.control_status_flags().wind_dead_reckoning;
		status_flags.cs_rng_kin_consistent      = _ekf.control_status_flags().rng_kin_consistent;
		status_flags.cs_fake_pos                = _ekf.control_status_flags().fake_pos;
		status_flags.cs_fake_hgt                = _ekf.control_status_flags().fake_hgt;
		status_flags.cs_gravity_vector          = _ekf.control_status_flags().gravity_vector;
		status_flags.cs_mag                     = _ekf.control_status_flags().mag;
		status_flags.cs_ev_yaw_fault            = _ekf.control_status_flags().ev_yaw_fault;
		status_flags.cs_mag_heading_consistent  = _ekf.control_status_flags().mag_heading_consistent;
		status_flags.cs_aux_gpos                = _ekf.control_status_flags().aux_gpos;
		status_flags.cs_rng_terrain    = _ekf.control_status_flags().rng_terrain;
		status_flags.cs_opt_flow_terrain    = _ekf.control_status_flags().opt_flow_terrain;
		status_flags.cs_valid_fake_pos      = _ekf.control_status_flags().valid_fake_pos;
		status_flags.cs_constant_pos        = _ekf.control_status_flags().constant_pos;
		status_flags.cs_baro_fault	    = _ekf.control_status_flags().baro_fault;
		status_flags.cs_gnss_vel            = _ekf.control_status_flags().gnss_vel;

		status_flags.fault_status_changes     = _filter_fault_status_changes;
		status_flags.fs_bad_mag_x             = _ekf.fault_status_flags().bad_mag_x;
		status_flags.fs_bad_mag_y             = _ekf.fault_status_flags().bad_mag_y;
		status_flags.fs_bad_mag_z             = _ekf.fault_status_flags().bad_mag_z;
		status_flags.fs_bad_hdg               = _ekf.fault_status_flags().bad_hdg;
		status_flags.fs_bad_mag_decl          = _ekf.fault_status_flags().bad_mag_decl;
		status_flags.fs_bad_airspeed          = _ekf.fault_status_flags().bad_airspeed;
		status_flags.fs_bad_sideslip          = _ekf.fault_status_flags().bad_sideslip;
		status_flags.fs_bad_optflow_x         = _ekf.fault_status_flags().bad_optflow_X;
		status_flags.fs_bad_optflow_y         = _ekf.fault_status_flags().bad_optflow_Y;
		status_flags.fs_bad_acc_vertical      = _ekf.fault_status_flags().bad_acc_vertical;
		status_flags.fs_bad_acc_clipping      = _ekf.fault_status_flags().bad_acc_clipping;

		status_flags.innovation_fault_status_changes = _innov_check_fail_status_changes;
		status_flags.reject_hor_vel                  = _ekf.innov_check_fail_status_flags().reject_hor_vel;
		status_flags.reject_ver_vel                  = _ekf.innov_check_fail_status_flags().reject_ver_vel;
		status_flags.reject_hor_pos                  = _ekf.innov_check_fail_status_flags().reject_hor_pos;
		status_flags.reject_ver_pos                  = _ekf.innov_check_fail_status_flags().reject_ver_pos;
		status_flags.reject_yaw                      = _ekf.innov_check_fail_status_flags().reject_yaw;
		status_flags.reject_airspeed                 = _ekf.innov_check_fail_status_flags().reject_airspeed;
		status_flags.reject_sideslip                 = _ekf.innov_check_fail_status_flags().reject_sideslip;
		status_flags.reject_hagl                     = _ekf.innov_check_fail_status_flags().reject_hagl;
		status_flags.reject_optflow_x                = _ekf.innov_check_fail_status_flags().reject_optflow_X;
		status_flags.reject_optflow_y                = _ekf.innov_check_fail_status_flags().reject_optflow_Y;

		status_flags.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_estimator_status_flags_pub.publish(status_flags);

		_last_status_flags_publish = status_flags.timestamp;
	}
}

#if defined(CONFIG_EKF2_GNSS)
void EKF2::PublishYawEstimatorStatus(const hrt_abstime &timestamp)
{
	static_assert(sizeof(yaw_estimator_status_s::yaw) / sizeof(float) == N_MODELS_EKFGSF,
		      "yaw_estimator_status_s::yaw wrong size");

	yaw_estimator_status_s yaw_est_test_data;

	if (_ekf.getDataEKFGSF(&yaw_est_test_data.yaw_composite, &yaw_est_test_data.yaw_variance,
			       yaw_est_test_data.yaw,
			       yaw_est_test_data.innov_vn, yaw_est_test_data.innov_ve,
			       yaw_est_test_data.weight)) {

		yaw_est_test_data.yaw_composite_valid = _ekf.isYawEmergencyEstimateAvailable();
		yaw_est_test_data.timestamp_sample = _ekf.time_delayed_us();
		yaw_est_test_data.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

		_yaw_est_pub.publish(yaw_est_test_data);
	}
}
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_WIND)
void EKF2::PublishWindEstimate(const hrt_abstime &timestamp)
{
	if (_ekf.get_wind_status()) {
		// Publish wind estimate only if ekf declares them valid
		wind_s wind{};
		wind.timestamp_sample = _ekf.time_delayed_us();

		const Vector2f wind_vel = _ekf.getWindVelocity();
		const Vector2f wind_vel_var = _ekf.getWindVelocityVariance();

#if defined(CONFIG_EKF2_AIRSPEED)
		wind.tas_innov = _ekf.aid_src_airspeed().innovation;
		wind.tas_innov_var = _ekf.aid_src_airspeed().innovation_variance;
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_SIDESLIP)
		wind.beta_innov = _ekf.aid_src_sideslip().innovation;
		wind.beta_innov = _ekf.aid_src_sideslip().innovation_variance;
#endif // CONFIG_EKF2_SIDESLIP

		wind.windspeed_north = wind_vel(0);
		wind.windspeed_east = wind_vel(1);
		wind.variance_north = wind_vel_var(0);
		wind.variance_east = wind_vel_var(1);
		wind.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

		_wind_pub.publish(wind);
	}
}
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
void EKF2::PublishOpticalFlowVel(const hrt_abstime &timestamp)
{
	const hrt_abstime timestamp_sample = _ekf.aid_src_optical_flow().timestamp_sample;

	if ((timestamp_sample != 0) && (timestamp_sample > _optical_flow_vel_pub_last)) {

		vehicle_optical_flow_vel_s flow_vel{};
		flow_vel.timestamp_sample = _ekf.aid_src_optical_flow().timestamp_sample;

		_ekf.getFlowVelBody().copyTo(flow_vel.vel_body);
		_ekf.getFlowVelNE().copyTo(flow_vel.vel_ne);

		_ekf.getFilteredFlowVelBody().copyTo(flow_vel.vel_body_filtered);
		_ekf.getFilteredFlowVelNE().copyTo(flow_vel.vel_ne_filtered);

		_ekf.getFlowUncompensated().copyTo(flow_vel.flow_rate_uncompensated);
		_ekf.getFlowCompensated().copyTo(flow_vel.flow_rate_compensated);

		_ekf.getFlowGyro().copyTo(flow_vel.gyro_rate);

		_ekf.getFlowGyroBias().copyTo(flow_vel.gyro_bias);
		_ekf.getFlowRefBodyRate().copyTo(flow_vel.ref_gyro);

		flow_vel.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

		_estimator_optical_flow_vel_pub.publish(flow_vel);

		_optical_flow_vel_pub_last = timestamp_sample;
	}
}
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_AIRSPEED)
void EKF2::UpdateAirspeedSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF airspeed sample
	// prefer ORB_ID(airspeed_validated) if available, otherwise fallback to raw airspeed ORB_ID(airspeed)
	if (_airspeed_validated_sub.updated()) {
		airspeed_validated_s airspeed_validated;

		if (_airspeed_validated_sub.update(&airspeed_validated)) {

			if (PX4_ISFINITE(airspeed_validated.true_airspeed_m_s)
			    && (airspeed_validated.selected_airspeed_index > 0)
			   ) {
				float cas2tas = 1.f;

				if (PX4_ISFINITE(airspeed_validated.calibrated_airspeed_m_s)
				    && (airspeed_validated.calibrated_airspeed_m_s > FLT_EPSILON)) {
					cas2tas = airspeed_validated.true_airspeed_m_s / airspeed_validated.calibrated_airspeed_m_s;
				}

				airspeedSample airspeed_sample {
					.time_us = airspeed_validated.timestamp,
					.true_airspeed = airspeed_validated.true_airspeed_m_s,
					.eas2tas = cas2tas,
				};
				_ekf.setAirspeedData(airspeed_sample);
			}

			_airspeed_validated_timestamp_last = airspeed_validated.timestamp;

			ekf2_timestamps.airspeed_validated_timestamp_rel = (int16_t)((int64_t)airspeed_validated.timestamp / 100 -
					(int64_t)ekf2_timestamps.timestamp / 100);
		}

	} else if (((ekf2_timestamps.timestamp - _airspeed_validated_timestamp_last) > 3_s) && _airspeed_sub.updated()) {
		// use ORB_ID(airspeed) if ORB_ID(airspeed_validated) is unavailable
		airspeed_s airspeed;

		if (_airspeed_sub.update(&airspeed)) {
			// The airspeed measurement received via ORB_ID(airspeed) topic has not been corrected
			// for scale factor errors and requires the ASPD_SCALE correction to be applied.
			const float true_airspeed_m_s = airspeed.true_airspeed_m_s * _airspeed_scale_factor;

			if (PX4_ISFINITE(airspeed.true_airspeed_m_s)
			    && PX4_ISFINITE(airspeed.indicated_airspeed_m_s)
			    && (airspeed.indicated_airspeed_m_s > 0.f)
			   ) {
				airspeedSample airspeed_sample {
					.time_us = airspeed.timestamp_sample,
					.true_airspeed = true_airspeed_m_s,
					.eas2tas = airspeed.true_airspeed_m_s / airspeed.indicated_airspeed_m_s,
				};
				_ekf.setAirspeedData(airspeed_sample);
			}

			ekf2_timestamps.airspeed_timestamp_rel = (int16_t)((int64_t)airspeed.timestamp / 100 -
					(int64_t)ekf2_timestamps.timestamp / 100);
		}
	}
}
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_AUXVEL)
void EKF2::UpdateAuxVelSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF auxiliary velocity sample
	//  - use the landing target pose estimate as another source of velocity data
	landing_target_pose_s landing_target_pose;

	if (_landing_target_pose_sub.update(&landing_target_pose)) {
		// we can only use the landing target if it has a fixed position and  a valid velocity estimate
		if (landing_target_pose.is_static && landing_target_pose.rel_vel_valid) {
			// velocity of vehicle relative to target has opposite sign to target relative to vehicle
			auxVelSample auxvel_sample{
				.time_us = landing_target_pose.timestamp,
				.vel = Vector2f{-landing_target_pose.vx_rel, -landing_target_pose.vy_rel},
				.velVar = Vector2f{landing_target_pose.cov_vx_rel, landing_target_pose.cov_vy_rel},
			};
			_ekf.setAuxVelData(auxvel_sample);
		}
	}
}
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_BAROMETER)
void EKF2::UpdateBaroSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF baro sample
	vehicle_air_data_s airdata;

	if (_airdata_sub.update(&airdata)) {

		bool reset = false;

		// check if barometer has changed
		if (airdata.baro_device_id != _device_id_baro) {
			if (_device_id_baro != 0) {
				PX4_DEBUG("%d - baro sensor ID changed %" PRIu32 " -> %" PRIu32, _instance, _device_id_baro, airdata.baro_device_id);
			}

			reset = true;

		} else if (airdata.calibration_count != _baro_calibration_count) {
			// existing calibration has changed, reset saved baro bias
			PX4_DEBUG("%d - baro %" PRIu32 " calibration updated, resetting bias", _instance, _device_id_baro);
			reset = true;
		}

		if (reset) {
			_device_id_baro = airdata.baro_device_id;
			_baro_calibration_count = airdata.calibration_count;
		}

		_ekf.set_air_density(airdata.rho);

		_ekf.setBaroData(baroSample{airdata.timestamp_sample, airdata.baro_alt_meter, reset});

		ekf2_timestamps.vehicle_air_data_timestamp_rel = (int16_t)((int64_t)airdata.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}
}
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
bool EKF2::UpdateExtVisionSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF external vision sample
	bool new_ev_odom = false;

	vehicle_odometry_s ev_odom;

	if (_ev_odom_sub.update(&ev_odom)) {

		extVisionSample ev_data{};
		ev_data.pos.setNaN();
		ev_data.vel.setNaN();
		ev_data.quat.setNaN();

		// check for valid velocity data
		const Vector3f ev_odom_vel(ev_odom.velocity);
		const Vector3f ev_odom_vel_var(ev_odom.velocity_variance);

		bool velocity_frame_valid = false;

		switch (ev_odom.velocity_frame) {
		case vehicle_odometry_s::VELOCITY_FRAME_NED:
			ev_data.vel_frame = VelocityFrame::LOCAL_FRAME_NED;
			velocity_frame_valid = true;
			break;

		case vehicle_odometry_s::VELOCITY_FRAME_FRD:
			ev_data.vel_frame = VelocityFrame::LOCAL_FRAME_FRD;
			velocity_frame_valid = true;
			break;

		case vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD:
			ev_data.vel_frame = VelocityFrame::BODY_FRAME_FRD;
			velocity_frame_valid = true;
			break;
		}

		if (ev_odom_vel.isAllFinite()) {
			if (velocity_frame_valid) {
				ev_data.vel = ev_odom_vel;

				const float evv_noise_var = sq(_param_ekf2_evv_noise.get());

				// velocity measurement error from ev_data or parameters
				if ((_param_ekf2_ev_noise_md.get() == 0) && ev_odom_vel_var.isAllFinite()) {

					ev_data.velocity_var(0) = fmaxf(evv_noise_var, ev_odom_vel_var(0));
					ev_data.velocity_var(1) = fmaxf(evv_noise_var, ev_odom_vel_var(1));
					ev_data.velocity_var(2) = fmaxf(evv_noise_var, ev_odom_vel_var(2));

				} else {
					ev_data.velocity_var.setAll(evv_noise_var);
				}

				new_ev_odom = true;
			}
		}

		// check for valid position data
		const Vector3f ev_odom_pos(ev_odom.position);
		const Vector3f ev_odom_pos_var(ev_odom.position_variance);

		bool position_frame_valid = false;

		switch (ev_odom.pose_frame) {
		case vehicle_odometry_s::POSE_FRAME_NED:
			ev_data.pos_frame = PositionFrame::LOCAL_FRAME_NED;
			position_frame_valid = true;
			break;

		case vehicle_odometry_s::POSE_FRAME_FRD:
			ev_data.pos_frame = PositionFrame::LOCAL_FRAME_FRD;
			position_frame_valid = true;
			break;
		}

		if (ev_odom_pos.isAllFinite()) {
			if (position_frame_valid) {
				ev_data.pos = ev_odom_pos;

				const float evp_noise_var = sq(_param_ekf2_evp_noise.get());

				// position measurement error from ev_data or parameters
				if ((_param_ekf2_ev_noise_md.get() == 0) && ev_odom_pos_var.isAllFinite()) {

					ev_data.position_var(0) = fmaxf(evp_noise_var, ev_odom_pos_var(0));
					ev_data.position_var(1) = fmaxf(evp_noise_var, ev_odom_pos_var(1));
					ev_data.position_var(2) = fmaxf(evp_noise_var, ev_odom_pos_var(2));

				} else {
					ev_data.position_var.setAll(evp_noise_var);
				}

				new_ev_odom = true;
			}
		}

		// check for valid orientation data
		const Quatf ev_odom_q(ev_odom.q);
		const Vector3f ev_odom_q_var(ev_odom.orientation_variance);
		const bool non_zero = (fabsf(ev_odom_q(0)) > 0.f) || (fabsf(ev_odom_q(1)) > 0.f)
				      || (fabsf(ev_odom_q(2)) > 0.f) || (fabsf(ev_odom_q(3)) > 0.f);
		const float eps = 1e-5f;
		const bool no_element_larger_than_one = (fabsf(ev_odom_q(0)) <= 1.f + eps)
							&& (fabsf(ev_odom_q(1)) <= 1.f + eps)
							&& (fabsf(ev_odom_q(2)) <= 1.f + eps)
							&& (fabsf(ev_odom_q(3)) <= 1.f + eps);
		const bool norm_in_tolerance = fabsf(1.f - ev_odom_q.norm()) <= eps;

		const bool orientation_valid = ev_odom_q.isAllFinite() && non_zero && no_element_larger_than_one && norm_in_tolerance;

		if (orientation_valid) {
			ev_data.quat = ev_odom_q;
			ev_data.quat.normalize();

			// orientation measurement error from ev_data or parameters
			const float eva_noise_var = sq(_param_ekf2_eva_noise.get());

			if ((_param_ekf2_ev_noise_md.get() == 0) && ev_odom_q_var.isAllFinite()) {

				ev_data.orientation_var(0) = fmaxf(eva_noise_var, ev_odom_q_var(0));
				ev_data.orientation_var(1) = fmaxf(eva_noise_var, ev_odom_q_var(1));
				ev_data.orientation_var(2) = fmaxf(eva_noise_var, ev_odom_q_var(2));

			} else {
				ev_data.orientation_var.setAll(eva_noise_var);
			}

			new_ev_odom = true;
		}

		// use timestamp from external computer, clocks are synchronized when using MAVROS
		ev_data.time_us = ev_odom.timestamp_sample;
		ev_data.reset_counter = ev_odom.reset_counter;
		ev_data.quality = ev_odom.quality;

		if (new_ev_odom)  {
			_ekf.setExtVisionData(ev_data);
		}

		ekf2_timestamps.visual_odometry_timestamp_rel = (int16_t)((int64_t)ev_odom.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}

	return new_ev_odom;
}
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
bool EKF2::UpdateFlowSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF flow sample
	bool new_optical_flow = false;
	vehicle_optical_flow_s optical_flow;

	if (_vehicle_optical_flow_sub.update(&optical_flow)) {

		const float dt = 1e-6f * (float)optical_flow.integration_timespan_us;
		Vector2f flow_rate;
		Vector3f gyro_rate;

		if (dt > FLT_EPSILON) {
			// NOTE: the EKF uses the reverse sign convention to the flow sensor. EKF assumes positive LOS rate
			// is produced by a RH rotation of the image about the sensor axis.
			flow_rate = Vector2f(-optical_flow.pixel_flow[0], -optical_flow.pixel_flow[1]) / dt;
			gyro_rate = Vector3f(-optical_flow.delta_angle[0], -optical_flow.delta_angle[1], -optical_flow.delta_angle[2]) / dt;

		} else if (optical_flow.quality == 0) {
			// handle special case of SITL and PX4Flow where dt is forced to zero when the quaity is 0
			flow_rate.zero();
			gyro_rate.zero();
		}

		flowSample flow {
			.time_us = optical_flow.timestamp_sample - optical_flow.integration_timespan_us / 2, // correct timestamp to midpoint of integration interval as the data is converted to rates
			.flow_rate = flow_rate,
			.gyro_rate = gyro_rate,
			.quality = optical_flow.quality
		};

		if (Vector2f(optical_flow.pixel_flow).isAllFinite() && optical_flow.integration_timespan_us < 1e6) {

			// Save sensor limits reported by the optical flow sensor
			_ekf.set_optical_flow_limits(optical_flow.max_flow_rate, optical_flow.min_ground_distance,
						     optical_flow.max_ground_distance);

			_ekf.setOpticalFlowData(flow);

			new_optical_flow = true;
		}

#if defined(CONFIG_EKF2_RANGE_FINDER)

		// use optical_flow distance as range sample if distance_sensor unavailable
		if (PX4_ISFINITE(optical_flow.distance_m) && (ekf2_timestamps.timestamp > _last_range_sensor_update + 1_s)) {

			int8_t quality = static_cast<float>(optical_flow.quality) / static_cast<float>(UINT8_MAX) * 100.f;

			estimator::sensor::rangeSample range_sample {
				.time_us = optical_flow.timestamp_sample,
				.rng = optical_flow.distance_m,
				.quality = quality,
			};
			_ekf.setRangeData(range_sample);

			// set sensor limits
			_ekf.set_rangefinder_limits(optical_flow.min_ground_distance, optical_flow.max_ground_distance);
		}

#endif // CONFIG_EKF2_RANGE_FINDER

		ekf2_timestamps.optical_flow_timestamp_rel = (int16_t)((int64_t)optical_flow.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}

	return new_optical_flow;
}
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_GNSS)
void EKF2::UpdateGpsSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF GPS message
	sensor_gps_s vehicle_gps_position;

	if (_vehicle_gps_position_sub.update(&vehicle_gps_position)) {

		Vector3f vel_ned;

		if (vehicle_gps_position.vel_ned_valid) {
			vel_ned = Vector3f(vehicle_gps_position.vel_n_m_s,
					   vehicle_gps_position.vel_e_m_s,
					   vehicle_gps_position.vel_d_m_s);

		} else {
			return; //TODO: change and set to NAN
		}

		if (fabsf(_param_ekf2_gps_yaw_off.get()) > 0.f) {
			if (!PX4_ISFINITE(vehicle_gps_position.heading_offset) && PX4_ISFINITE(vehicle_gps_position.heading)) {
				// Apply offset
				float yaw_offset = matrix::wrap_pi(math::radians(_param_ekf2_gps_yaw_off.get()));
				vehicle_gps_position.heading_offset = yaw_offset;
				vehicle_gps_position.heading = matrix::wrap_pi(vehicle_gps_position.heading - yaw_offset);
			}
		}

		const float altitude_amsl = static_cast<float>(vehicle_gps_position.altitude_msl_m);
		const float altitude_ellipsoid = static_cast<float>(vehicle_gps_position.altitude_ellipsoid_m);

		gnssSample gnss_sample{
			.time_us = vehicle_gps_position.timestamp,
			.lat = vehicle_gps_position.latitude_deg,
			.lon = vehicle_gps_position.longitude_deg,
			.alt = altitude_amsl,
			.vel = vel_ned,
			.hacc = vehicle_gps_position.eph,
			.vacc = vehicle_gps_position.epv,
			.sacc = vehicle_gps_position.s_variance_m_s,
			.fix_type = vehicle_gps_position.fix_type,
			.nsats = vehicle_gps_position.satellites_used,
			.pdop = sqrtf(vehicle_gps_position.hdop *vehicle_gps_position.hdop
				      + vehicle_gps_position.vdop * vehicle_gps_position.vdop),
			.yaw = vehicle_gps_position.heading, //TODO: move to different message
			.yaw_acc = vehicle_gps_position.heading_accuracy,
			.yaw_offset = vehicle_gps_position.heading_offset,
			.spoofed = vehicle_gps_position.spoofing_state == sensor_gps_s::SPOOFING_STATE_MULTIPLE,
		};

		_ekf.setGpsData(gnss_sample);

		const float geoid_height = altitude_ellipsoid - altitude_amsl;

		if (_last_geoid_height_update_us == 0) {
			_geoid_height_lpf.reset(geoid_height);
			_last_geoid_height_update_us = gnss_sample.time_us;

		} else if (gnss_sample.time_us > _last_geoid_height_update_us) {
			const float dt = 1e-6f * (gnss_sample.time_us - _last_geoid_height_update_us);
			_geoid_height_lpf.setParameters(dt, kGeoidHeightLpfTimeConstant);
			_geoid_height_lpf.update(geoid_height);
			_last_geoid_height_update_us = gnss_sample.time_us;
		}

	}
}

float EKF2::altEllipsoidToAmsl(float ellipsoid_alt) const
{
	return ellipsoid_alt - _geoid_height_lpf.getState();
}

float EKF2::altAmslToEllipsoid(float amsl_alt) const
{
	return amsl_alt + _geoid_height_lpf.getState();
}
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_MAGNETOMETER)
void EKF2::UpdateMagSample(ekf2_timestamps_s &ekf2_timestamps)
{
	vehicle_magnetometer_s magnetometer;

	if (_magnetometer_sub.update(&magnetometer)) {

		bool reset = false;

		// check if magnetometer has changed
		if (magnetometer.device_id != _device_id_mag) {
			if (_device_id_mag != 0) {
				PX4_DEBUG("%d - mag sensor ID changed %" PRIu32 " -> %" PRIu32, _instance, _device_id_mag, magnetometer.device_id);
			}

			reset = true;

		} else if (magnetometer.calibration_count != _mag_calibration_count) {
			// existing calibration has changed, reset saved mag bias
			PX4_DEBUG("%d - mag %" PRIu32 " calibration updated, resetting bias", _instance, _device_id_mag);
			reset = true;
		}

		if (reset) {
			_device_id_mag = magnetometer.device_id;
			_mag_calibration_count = magnetometer.calibration_count;

			// reset magnetometer bias learning
			_mag_cal = {};
		}

		_ekf.setMagData(magSample{magnetometer.timestamp_sample, Vector3f{magnetometer.magnetometer_ga}, reset});

		ekf2_timestamps.vehicle_magnetometer_timestamp_rel = (int16_t)((int64_t)magnetometer.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}
}
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_RANGE_FINDER)
void EKF2::UpdateRangeSample(ekf2_timestamps_s &ekf2_timestamps)
{
	distance_sensor_s distance_sensor;

	if (_distance_sensor_selected < 0) {

		// only consider distance sensors that have updated within the last 0.1s
		const hrt_abstime timestamp_stale = math::max(ekf2_timestamps.timestamp, 100_ms) - 100_ms;

		if (_distance_sensor_subs.advertised()) {
			for (unsigned i = 0; i < _distance_sensor_subs.size(); i++) {

				if (_distance_sensor_subs[i].update(&distance_sensor)) {
					// only use the first instace which has the correct orientation
					if ((distance_sensor.timestamp != 0) && (distance_sensor.timestamp > timestamp_stale)
					    && (distance_sensor.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING)) {

						int ndist = orb_group_count(ORB_ID(distance_sensor));

						if (ndist > 1) {
							PX4_INFO("%d - selected distance_sensor:%d (%d advertised)", _instance, i, ndist);
						}

						_distance_sensor_selected = i;
						_last_range_sensor_update = distance_sensor.timestamp;
						break;
					}
				}
			}
		}
	}

	if (_distance_sensor_selected >= 0 && _distance_sensor_subs[_distance_sensor_selected].update(&distance_sensor)) {
		// EKF range sample
		if (distance_sensor.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING) {
			estimator::sensor::rangeSample range_sample {
				.time_us = distance_sensor.timestamp,
				.rng = distance_sensor.current_distance,
				.quality = distance_sensor.signal_quality,
			};
			_ekf.setRangeData(range_sample);

			// Save sensor limits reported by the rangefinder
			_ekf.set_rangefinder_limits(distance_sensor.min_distance, distance_sensor.max_distance);

			_last_range_sensor_update = ekf2_timestamps.timestamp;
		}

		ekf2_timestamps.distance_sensor_timestamp_rel = (int16_t)((int64_t)distance_sensor.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}

	if (_last_range_sensor_update < ekf2_timestamps.timestamp - 1_s) {
		// force reselection after timeout
		_distance_sensor_selected = -1;
	}
}
#endif // CONFIG_EKF2_RANGE_FINDER

void EKF2::UpdateSystemFlagsSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF system flags
	if (_status_sub.updated() || _vehicle_land_detected_sub.updated()) {

		systemFlagUpdate flags{};
		flags.time_us = ekf2_timestamps.timestamp;

		// vehicle_status
		vehicle_status_s vehicle_status;

		if (_status_sub.copy(&vehicle_status)
		    && (ekf2_timestamps.timestamp < vehicle_status.timestamp + 3_s)) {

			// initially set in_air from arming_state (will be overridden if land detector is available)
			flags.in_air = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

			// let the EKF know if the vehicle motion is that of a fixed wing (forward flight only relative to wind)
			flags.is_fixed_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);

#if defined(CONFIG_EKF2_SIDESLIP)

			if (vehicle_status.is_vtol_tailsitter && _params->beta_fusion_enabled) {
				PX4_WARN("Disable EKF beta fusion as unsupported for tailsitter");
				_param_ekf2_fuse_beta.set(0);
				_param_ekf2_fuse_beta.commit_no_notification();
			}

#endif // CONFIG_EKF2_SIDESLIP
		}

		// vehicle_land_detected
		vehicle_land_detected_s vehicle_land_detected;

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)
		    && (ekf2_timestamps.timestamp < vehicle_land_detected.timestamp + 3_s)) {

			flags.at_rest = vehicle_land_detected.at_rest;
			flags.in_air = !vehicle_land_detected.landed;
			flags.gnd_effect = vehicle_land_detected.in_ground_effect;
		}

		launch_detection_status_s launch_detection_status;

		if (_launch_detection_status_sub.copy(&launch_detection_status)
		    && (ekf2_timestamps.timestamp < launch_detection_status.timestamp + 3_s)) {

			flags.constant_pos = (launch_detection_status.launch_detection_state ==
					      launch_detection_status_s::STATE_WAITING_FOR_LAUNCH);
		}

		_ekf.setSystemFlagData(flags);
	}
}

void EKF2::UpdateCalibration(const hrt_abstime &timestamp, InFlightCalibration &cal, const matrix::Vector3f &bias,
			     const matrix::Vector3f &bias_variance, float bias_limit, bool bias_valid, bool learning_valid)
{
	// reset existing cal on takeoff
	if (!_ekf.control_status_prev_flags().in_air && _ekf.control_status_flags().in_air) {
		cal = {};
	}

	// Check if conditions are OK for learning of accelerometer bias values
	// the EKF is operating in the correct mode and there are no filter faults
	static constexpr float max_var_allowed = 1e-3f;
	static constexpr float max_var_ratio = 1e2f;

	const bool valid = bias_valid
			   && (bias_variance.max() < max_var_allowed)
			   && (bias_variance.max() < max_var_ratio * bias_variance.min());

	if (valid && learning_valid) {
		// consider bias estimates stable when all checks pass consistently and bias hasn't changed more than 10% of the limit
		const float bias_change_limit = 0.1f * bias_limit;

		if (!(cal.bias - bias).longerThan(bias_change_limit)) {
			if (cal.last_us != 0) {
				cal.total_time_us += timestamp - cal.last_us;
			}

			if (cal.total_time_us > 10_s) {
				cal.cal_available = true;
			}

		} else {
			cal.total_time_us = 0;
			cal.bias = bias;
			cal.cal_available = false;
		}

		cal.last_us = timestamp;

	} else {
		// conditions are NOT OK for learning bias, reset timestamp
		// but keep the accumulated calibration time
		cal.last_us = 0;

		if (!valid && (cal.total_time_us != 0)) {
			// if a filter fault has occurred, assume previous learning was invalid and do not
			// count it towards total learning time.
			cal = {};
		}
	}
}

void EKF2::UpdateAccelCalibration(const hrt_abstime &timestamp)
{
	// the EKF is operating in the correct mode and there are no filter faults
	const bool bias_valid = (_param_ekf2_imu_ctrl.get() & static_cast<int32_t>(ImuCtrl::AccelBias))
				&& _ekf.control_status_flags().tilt_align
				&& (_ekf.fault_status().value == 0)
				&& !_ekf.fault_status_flags().bad_acc_clipping
				&& !_ekf.fault_status_flags().bad_acc_vertical;

	const bool learning_valid = bias_valid && !_ekf.accel_bias_inhibited();

	UpdateCalibration(timestamp, _accel_cal, _ekf.getAccelBias(), _ekf.getAccelBiasVariance(), _ekf.getAccelBiasLimit(),
			  bias_valid, learning_valid);
}

void EKF2::UpdateGyroCalibration(const hrt_abstime &timestamp)
{
	// the EKF is operating in the correct mode and there are no filter faults
	const bool bias_valid = (_param_ekf2_imu_ctrl.get() & static_cast<int32_t>(ImuCtrl::GyroBias))
				&& _ekf.control_status_flags().tilt_align
				&& (_ekf.fault_status().value == 0);

	const bool learning_valid = bias_valid && !_ekf.gyro_bias_inhibited();

	UpdateCalibration(timestamp, _gyro_cal, _ekf.getGyroBias(), _ekf.getGyroBiasVariance(), _ekf.getGyroBiasLimit(),
			  bias_valid, learning_valid);
}

#if defined(CONFIG_EKF2_MAGNETOMETER)
void EKF2::UpdateMagCalibration(const hrt_abstime &timestamp)
{
	const Vector3f mag_bias = _ekf.getMagBias();
	const Vector3f mag_bias_var = _ekf.getMagBiasVariance();

	const bool bias_valid = (_ekf.fault_status().value == 0)
				&& _ekf.control_status_flags().yaw_align
				&& mag_bias_var.longerThan(0.f) && !mag_bias_var.longerThan(0.02f);

	const bool learning_valid = bias_valid && _ekf.control_status_flags().mag;

	UpdateCalibration(timestamp, _mag_cal, mag_bias, mag_bias_var, _ekf.getMagBiasLimit(), bias_valid, learning_valid);

	// update stored declination value
	if (!_mag_decl_saved) {
		float declination_deg;

		if (_ekf.get_mag_decl_deg(declination_deg)) {
			_param_ekf2_mag_decl.update();

			if (PX4_ISFINITE(declination_deg) && (fabsf(declination_deg - _param_ekf2_mag_decl.get()) > 0.1f)) {
				_param_ekf2_mag_decl.set(declination_deg);
				_param_ekf2_mag_decl.commit_no_notification();
			}

			_mag_decl_saved = true;
		}
	}
}
#endif // CONFIG_EKF2_MAGNETOMETER

int EKF2::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int EKF2::task_spawn(int argc, char *argv[])
{
	bool success = false;
	bool replay_mode = false;

	if (argc > 1 && !strcmp(argv[1], "-r")) {
		PX4_INFO("replay mode enabled");
		replay_mode = true;
	}

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
	bool multi_mode = false;
	int32_t imu_instances = 0;
	int32_t mag_instances = 0;

	int32_t sens_imu_mode = 1;
	param_get(param_find("SENS_IMU_MODE"), &sens_imu_mode);

	if (sens_imu_mode == 0) {
		// ekf selector requires SENS_IMU_MODE = 0
		multi_mode = true;

		// IMUs (1 - MAX_NUM_IMUS supported)
		param_get(param_find("EKF2_MULTI_IMU"), &imu_instances);

		if (imu_instances < 1 || imu_instances > MAX_NUM_IMUS) {
			const int32_t imu_instances_limited = math::constrain(imu_instances, static_cast<int32_t>(1),
							      static_cast<int32_t>(MAX_NUM_IMUS));
			PX4_WARN("EKF2_MULTI_IMU limited %" PRId32 " -> %" PRId32, imu_instances, imu_instances_limited);
			param_set_no_notification(param_find("EKF2_MULTI_IMU"), &imu_instances_limited);
			imu_instances = imu_instances_limited;
		}

#if defined(CONFIG_EKF2_MAGNETOMETER)
		int32_t sens_mag_mode = 1;
		const param_t param_sens_mag_mode = param_find("SENS_MAG_MODE");
		param_get(param_sens_mag_mode, &sens_mag_mode);

		if (sens_mag_mode == 0) {
			const param_t param_ekf2_mult_mag = param_find("EKF2_MULTI_MAG");
			param_get(param_ekf2_mult_mag, &mag_instances);

			// Mags (1 - MAX_NUM_MAGS supported)
			if (mag_instances > MAX_NUM_MAGS) {
				const int32_t mag_instances_limited = math::constrain(mag_instances, static_cast<int32_t>(1),
								      static_cast<int32_t>(MAX_NUM_MAGS));
				PX4_WARN("EKF2_MULTI_MAG limited %" PRId32 " -> %" PRId32, mag_instances, mag_instances_limited);
				param_set_no_notification(param_ekf2_mult_mag, &mag_instances_limited);
				mag_instances = mag_instances_limited;

			} else if (mag_instances <= 1) {
				// properly disable multi-magnetometer at sensors hub level
				PX4_WARN("EKF2_MULTI_MAG disabled, resetting SENS_MAG_MODE");

				// re-enable at sensors level
				sens_mag_mode = 1;
				param_set(param_sens_mag_mode, &sens_mag_mode);

				mag_instances = 1;
			}

		} else {
			mag_instances = 1;
		}

#endif // CONFIG_EKF2_MAGNETOMETER
	}

	if (multi_mode && !replay_mode) {
		// Start EKF2Selector if it's not already running
		if (_ekf2_selector.load() == nullptr) {
			EKF2Selector *inst = new EKF2Selector();

			if (inst) {
				_ekf2_selector.store(inst);

			} else {
				PX4_ERR("Failed to create EKF2 selector");
				return PX4_ERROR;
			}
		}

		const hrt_abstime time_started = hrt_absolute_time();
		const int multi_instances = math::min(imu_instances * mag_instances, static_cast<int32_t>(EKF2_MAX_INSTANCES));
		int multi_instances_allocated = 0;

		// allocate EKF2 instances until all found or arming
		uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};

		bool ekf2_instance_created[MAX_NUM_IMUS][MAX_NUM_MAGS] {}; // IMUs * mags

		while ((multi_instances_allocated < multi_instances)
		       && (vehicle_status_sub.get().arming_state != vehicle_status_s::ARMING_STATE_ARMED)
		       && ((hrt_elapsed_time(&time_started) < 30_s)
			   || (vehicle_status_sub.get().hil_state == vehicle_status_s::HIL_STATE_ON))) {

			vehicle_status_sub.update();

			for (uint8_t mag = 0; mag < mag_instances; mag++) {
				uORB::SubscriptionData<vehicle_magnetometer_s> vehicle_mag_sub{ORB_ID(vehicle_magnetometer), mag};

				for (uint8_t imu = 0; imu < imu_instances; imu++) {

					uORB::SubscriptionData<vehicle_imu_s> vehicle_imu_sub{ORB_ID(vehicle_imu), imu};
					vehicle_mag_sub.update();

					// Mag & IMU data must be valid, first mag can be ignored initially
					if ((vehicle_mag_sub.advertised() || mag == 0) && (vehicle_imu_sub.advertised())) {

						if (!ekf2_instance_created[imu][mag]) {
							EKF2 *ekf2_inst = new EKF2(true, px4::ins_instance_to_wq(imu), false);

							if (ekf2_inst && ekf2_inst->multi_init(imu, mag)) {
								int actual_instance = ekf2_inst->instance(); // match uORB instance numbering

								if ((actual_instance >= 0) && (_objects[actual_instance].load() == nullptr)) {
									_objects[actual_instance].store(ekf2_inst);
									success = true;
									multi_instances_allocated++;
									ekf2_instance_created[imu][mag] = true;

									PX4_DEBUG("starting instance %d, IMU:%" PRIu8 " (%" PRIu32 "), MAG:%" PRIu8 " (%" PRIu32 ")", actual_instance,
										  imu, vehicle_imu_sub.get().accel_device_id,
										  mag, vehicle_mag_sub.get().device_id);

									_ekf2_selector.load()->ScheduleNow();

								} else {
									PX4_ERR("instance numbering problem instance: %d", actual_instance);
									delete ekf2_inst;
									break;
								}

							} else {
								PX4_ERR("alloc and init failed imu: %" PRIu8 " mag:%" PRIu8, imu, mag);
								px4_usleep(100000);
								break;
							}
						}

					} else {
						px4_usleep(1000); // give the sensors extra time to start
						break;
					}
				}
			}

			if (multi_instances_allocated < multi_instances) {
				px4_usleep(10000);
			}
		}

	} else

#endif // CONFIG_EKF2_MULTI_INSTANCE

	{
		// otherwise launch regular
		EKF2 *ekf2_inst = new EKF2(false, px4::wq_configurations::INS0, replay_mode);

		if (ekf2_inst) {
			_objects[0].store(ekf2_inst);
			ekf2_inst->ScheduleNow();
			success = true;
		}
	}

	return success ? PX4_OK : PX4_ERROR;
}

int EKF2::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Attitude and position estimator using an Extended Kalman Filter. It is used for Multirotors and Fixed-Wing.

The documentation can be found on the [ECL/EKF Overview & Tuning](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html) page.

ekf2 can be started in replay mode (`-r`): in this mode, it does not access the system time, but only uses the
timestamps from the sensor topics.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ekf2", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('r', "Enable replay mode", true);
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "print status info");
#if defined(CONFIG_EKF2_VERBOSE_STATUS)
	PRINT_MODULE_USAGE_ARG("-v", "verbose (print all states and full covariance matrix)", true);
#endif // CONFIG_EKF2_VERBOSE_STATUS
#if defined(CONFIG_EKF2_MULTI_INSTANCE)
	PRINT_MODULE_USAGE_COMMAND_DESCR("select_instance", "Request switch to new estimator instance");
	PRINT_MODULE_USAGE_ARG("<instance>", "Specify desired estimator instance", false);
#endif // CONFIG_EKF2_MULTI_INSTANCE
	return 0;
}

extern "C" __EXPORT int ekf2_main(int argc, char *argv[])
{
	if (argc <= 1 || strcmp(argv[1], "-h") == 0) {
		return EKF2::print_usage();
	}

	if (strcmp(argv[1], "start") == 0) {
		int ret = 0;
		EKF2::lock_module();

		ret = EKF2::task_spawn(argc - 1, argv + 1);

		if (ret < 0) {
			PX4_ERR("start failed (%i)", ret);
		}

		EKF2::unlock_module();
		return ret;

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
	} else if (strcmp(argv[1], "select_instance") == 0) {

		if (EKF2::trylock_module()) {
			if (_ekf2_selector.load()) {
				if (argc > 2) {
					int instance = atoi(argv[2]);
					_ekf2_selector.load()->RequestInstance(instance);
				} else {
					EKF2::unlock_module();
					return EKF2::print_usage("instance required");
				}

			} else {
				PX4_ERR("multi-EKF not active, unable to select instance");
			}

			EKF2::unlock_module();

		} else {
			PX4_WARN("module locked, try again later");
		}

		return 0;
#endif // CONFIG_EKF2_MULTI_INSTANCE
	} else if (strcmp(argv[1], "status") == 0) {
		if (EKF2::trylock_module()) {
#if defined(CONFIG_EKF2_MULTI_INSTANCE)
			if (_ekf2_selector.load()) {
				_ekf2_selector.load()->PrintStatus();
			}
#endif // CONFIG_EKF2_MULTI_INSTANCE

			bool verbose_status = false;

#if defined(CONFIG_EKF2_VERBOSE_STATUS)
			if (argc > 2 && (strcmp(argv[2], "-v") == 0)) {
				verbose_status = true;
			}
#endif // CONFIG_EKF2_VERBOSE_STATUS

			for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
				if (_objects[i].load()) {
					PX4_INFO_RAW("\n");
					_objects[i].load()->print_status(verbose_status);
				}
			}

			EKF2::unlock_module();

		} else {
			PX4_WARN("module locked, try again later");
		}

		return 0;

	} else if (strcmp(argv[1], "stop") == 0) {
		EKF2::lock_module();

		if (argc > 2) {
			int instance = atoi(argv[2]);

			if (instance >= 0 && instance < EKF2_MAX_INSTANCES) {
				PX4_INFO("stopping instance %d", instance);
				EKF2 *inst = _objects[instance].load();

				if (inst) {
					inst->request_stop();
					px4_usleep(20000); // 20 ms
					delete inst;
					_objects[instance].store(nullptr);
				}
			} else {
				PX4_ERR("invalid instance %d", instance);
			}

		} else {
			// otherwise stop everything
			bool was_running = false;

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
			if (_ekf2_selector.load()) {
				PX4_INFO("stopping ekf2 selector");
				_ekf2_selector.load()->Stop();
				delete _ekf2_selector.load();
				_ekf2_selector.store(nullptr);
				was_running = true;
			}
#endif // CONFIG_EKF2_MULTI_INSTANCE

			for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
				EKF2 *inst = _objects[i].load();

				if (inst) {
					PX4_INFO("stopping ekf2 instance %d", i);
					was_running = true;
					inst->request_stop();
					px4_usleep(20000); // 20 ms
					delete inst;
					_objects[i].store(nullptr);
				}
			}

			if (!was_running) {
				PX4_WARN("not running");
			}
		}

		EKF2::unlock_module();
		return PX4_OK;
	}

	EKF2::lock_module(); // Lock here, as the method could access _object.
	int ret = EKF2::custom_command(argc - 1, argv + 1);
	EKF2::unlock_module();

	return ret;
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file EKF2.hpp
 * Implementation of the attitude and position estimator.
 *
 * @author Roman Bapst
 */

#ifndef EKF2_HPP
#define EKF2_HPP

#include "EKF/ekf.h"

#include "EKF2Selector.hpp"
#include "mathlib/math/filter/AlphaFilter.hpp"

#include <float.h>

#include <containers/LockGuard.hpp>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/topics/estimator_bias.h>
#include <uORB/topics/estimator_bias3d.h>
#include <uORB/topics/estimator_event_flags.h>
#include <uORB/topics/estimator_innovations.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/estimator_states.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/launch_detection_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/yaw_estimator_status.h>

#if defined(CONFIG_EKF2_AIRSPEED)
# include <uORB/topics/airspeed.h>
# include <uORB/topics/airspeed_validated.h>
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_AUXVEL)
# include <uORB/topics/landing_target_pose.h>
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_BAROMETER)
# include <uORB/topics/vehicle_air_data.h>
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_GNSS)
# include <uORB/topics/estimator_gps_status.h>
# include <uORB/topics/sensor_gps.h>
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_MAGNETOMETER)
# include <uORB/topics/vehicle_magnetometer.h>
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
# include <uORB/topics/vehicle_optical_flow.h>
# include <uORB/topics/vehicle_optical_flow_vel.h>
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_RANGE_FINDER)
# include <uORB/topics/distance_sensor.h>
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_WIND)
# include <uORB/topics/wind.h>
#endif // CONFIG_EKF2_WIND

extern pthread_mutex_t ekf2_module_mutex;

class EKF2 final : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	EKF2() = delete;
	EKF2(bool multi_mode, const px4::wq_config_t &config, bool replay_mode);
	~EKF2() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int print_status(bool verbose = false);

	bool should_exit() const { return _task_should_exit.load(); }

	void request_stop() { _task_should_exit.store(true); }

	static void lock_module() { pthread_mutex_lock(&ekf2_module_mutex); }
	static bool trylock_module() { return (pthread_mutex_trylock(&ekf2_module_mutex) == 0); }
	static void unlock_module() { pthread_mutex_unlock(&ekf2_module_mutex); }

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
	bool multi_init(int imu, int mag);
#endif // CONFIG_EKF2_MULTI_INSTANCE

	int instance() const { return _instance; }

private:

	static constexpr uint8_t MAX_NUM_IMUS = 4;
	static constexpr uint8_t MAX_NUM_MAGS = 4;

	void Run() override;

	void AdvertiseTopics();
	void VerifyParams();

	void PublishAidSourceStatus(const hrt_abstime &timestamp);
	void PublishAttitude(const hrt_abstime &timestamp);

#if defined(CONFIG_EKF2_BAROMETER)
	void PublishBaroBias(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_RANGE_FINDER)
	void PublishRngHgtBias(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	void PublishEvPosBias(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_EXTERNAL_VISION
	estimator_bias_s fillEstimatorBiasMsg(const BiasEstimator::status &status, uint64_t timestamp_sample_us,
					      uint64_t timestamp, uint32_t device_id = 0);
	void PublishEventFlags(const hrt_abstime &timestamp);
	void PublishGlobalPosition(const hrt_abstime &timestamp);
	void PublishInnovations(const hrt_abstime &timestamp);
	void PublishInnovationTestRatios(const hrt_abstime &timestamp);
	void PublishInnovationVariances(const hrt_abstime &timestamp);
	void PublishLocalPosition(const hrt_abstime &timestamp);
	void PublishOdometry(const hrt_abstime &timestamp, const imuSample &imu_sample);
	void PublishSensorBias(const hrt_abstime &timestamp);
	void PublishStates(const hrt_abstime &timestamp);
	void PublishStatus(const hrt_abstime &timestamp);
	void PublishStatusFlags(const hrt_abstime &timestamp);
#if defined(CONFIG_EKF2_WIND)
	void PublishWindEstimate(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_AIRSPEED)
	void UpdateAirspeedSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_AUXVEL)
	void UpdateAuxVelSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_AUXVEL
#if defined(CONFIG_EKF2_BAROMETER)
	void UpdateBaroSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_BAROMETER
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	bool UpdateExtVisionSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_GNSS)
	float altEllipsoidToAmsl(float ellipsoid_alt) const;
	float altAmslToEllipsoid(float amsl_alt) const;

	void PublishGpsStatus(const hrt_abstime &timestamp);
	void PublishGnssHgtBias(const hrt_abstime &timestamp);
	void PublishYawEstimatorStatus(const hrt_abstime &timestamp);
	void UpdateGpsSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_GNSS
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	bool UpdateFlowSample(ekf2_timestamps_s &ekf2_timestamps);
	void PublishOpticalFlowVel(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_OPTICAL_FLOW
#if defined(CONFIG_EKF2_MAGNETOMETER)
	void UpdateMagSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_MAGNETOMETER
#if defined(CONFIG_EKF2_RANGE_FINDER)
	void UpdateRangeSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_RANGE_FINDER

	void UpdateSystemFlagsSample(ekf2_timestamps_s &ekf2_timestamps);

	// Used to check, save and use learned accel/gyro/mag biases
	struct InFlightCalibration {
		hrt_abstime last_us{0};         ///< last time the EKF was operating a mode that estimates accelerometer biases (uSec)
		hrt_abstime total_time_us{0};   ///< accumulated calibration time since the last save
		matrix::Vector3f bias{};
		bool cal_available{false};      ///< true when an unsaved valid calibration for the XYZ accelerometer bias is available
	};

	void UpdateCalibration(const hrt_abstime &timestamp, InFlightCalibration &cal, const matrix::Vector3f &bias,
			       const matrix::Vector3f &bias_variance, float bias_limit, bool bias_valid, bool learning_valid);
	void UpdateAccelCalibration(const hrt_abstime &timestamp);
	void UpdateGyroCalibration(const hrt_abstime &timestamp);
#if defined(CONFIG_EKF2_MAGNETOMETER)
	void UpdateMagCalibration(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_MAGNETOMETER

	// publish helper for estimator_aid_source topics
	template <typename T>
	void PublishAidSourceStatus(const T &status, hrt_abstime &status_publish_last, uORB::PublicationMulti<T> &pub)
	{
		if (status.timestamp_sample > status_publish_last) {
			// publish if updated
			T status_out{status};
			status_out.estimator_instance = _instance;
			status_out.timestamp = hrt_absolute_time();
			pub.publish(status_out);

			// record timestamp sample
			status_publish_last = status.timestamp_sample;
		}
	}

	static constexpr float sq(float x) { return x * x; };

	const bool _replay_mode{false};			///< true when we use replay data from a log
	const bool _multi_mode;
	int _instance{0};

	px4::atomic_bool _task_should_exit{false};

	// time slip monitoring
	uint64_t _integrated_time_us = 0;	///< integral of gyro delta time from start (uSec)
	uint64_t _start_time_us = 0;		///< system time at EKF start (uSec)
	int64_t _last_time_slip_us = 0;		///< Last time slip (uSec)

	perf_counter_t _ekf_update_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": EKF update")};
	perf_counter_t _msg_missed_imu_perf{perf_alloc(PC_COUNT, MODULE_NAME": IMU message missed")};

	InFlightCalibration _accel_cal{};
	InFlightCalibration _gyro_cal{};

	uint8_t _accel_calibration_count{0};
	uint8_t _gyro_calibration_count{0};

	uint32_t _device_id_accel{0};
	uint32_t _device_id_gyro{0};

	Vector3f _last_accel_bias_published{};
	Vector3f _last_gyro_bias_published{};

	hrt_abstime _last_sensor_bias_published{0};

	hrt_abstime _status_fake_hgt_pub_last{0};
	hrt_abstime _status_fake_pos_pub_last{0};

#if defined(CONFIG_EKF2_MAGNETOMETER)
	uint32_t _device_id_mag {0};

	// Used to control saving of mag declination to be used on next startup
	bool _mag_decl_saved = false;	///< true when the magnetic declination has been saved

	InFlightCalibration _mag_cal{};
	uint8_t _mag_calibration_count{0};
	Vector3f _last_mag_bias_published{};

	hrt_abstime _status_mag_pub_last{0};

	uORB::Subscription _magnetometer_sub{ORB_ID(vehicle_magnetometer)};

	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_mag_pub{ORB_ID(estimator_aid_src_mag)};
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_ev_hgt_pub {ORB_ID(estimator_aid_src_ev_hgt)};
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_ev_pos_pub{ORB_ID(estimator_aid_src_ev_pos)};
	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_ev_vel_pub{ORB_ID(estimator_aid_src_ev_vel)};
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_ev_yaw_pub{ORB_ID(estimator_aid_src_ev_yaw)};
	hrt_abstime _status_ev_hgt_pub_last{0};
	hrt_abstime _status_ev_pos_pub_last{0};
	hrt_abstime _status_ev_vel_pub_last{0};
	hrt_abstime _status_ev_yaw_pub_last{0};

	matrix::Vector3f _last_ev_bias_published{};

	uORB::Subscription _ev_odom_sub{ORB_ID(vehicle_visual_odometry)};

	uORB::PublicationMulti<estimator_bias3d_s> _estimator_ev_pos_bias_pub{ORB_ID(estimator_ev_pos_bias)};
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AUXVEL)
	uORB::Subscription _landing_target_pose_sub {ORB_ID(landing_target_pose)};

	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_aux_vel_pub{ORB_ID(estimator_aid_src_aux_vel)};
	hrt_abstime _status_aux_vel_pub_last{0};
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	uORB::Subscription _vehicle_optical_flow_sub {ORB_ID(vehicle_optical_flow)};
	uORB::PublicationMulti<vehicle_optical_flow_vel_s> _estimator_optical_flow_vel_pub{ORB_ID(estimator_optical_flow_vel)};

	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_optical_flow_pub{ORB_ID(estimator_aid_src_optical_flow)};
	hrt_abstime _status_optical_flow_pub_last{0};
	hrt_abstime _optical_flow_vel_pub_last{0};
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_BAROMETER)
	uint8_t _baro_calibration_count {0};
	uint32_t _device_id_baro{0};
	hrt_abstime _status_baro_hgt_pub_last{0};

	float _last_baro_bias_published{};

	uORB::Subscription _airdata_sub{ORB_ID(vehicle_air_data)};

	uORB::PublicationMulti<estimator_bias_s> _estimator_baro_bias_pub{ORB_ID(estimator_baro_bias)};
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_baro_hgt_pub {ORB_ID(estimator_aid_src_baro_hgt)};
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_DRAG_FUSION)
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_drag_pub {ORB_ID(estimator_aid_src_drag)};
	hrt_abstime _status_drag_pub_last{0};
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_AIRSPEED)
	uORB::Subscription _airspeed_sub {ORB_ID(airspeed)};
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};

	float _airspeed_scale_factor{1.0f}; ///< scale factor correction applied to airspeed measurements
	hrt_abstime _airspeed_validated_timestamp_last{0};

	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_airspeed_pub {ORB_ID(estimator_aid_src_airspeed)};
	hrt_abstime _status_airspeed_pub_last{0};
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_sideslip_pub {ORB_ID(estimator_aid_src_sideslip)};
	hrt_abstime _status_sideslip_pub_last {0};
#endif // CONFIG_EKF2_SIDESLIP

	orb_advert_t _mavlink_log_pub{nullptr};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _launch_detection_status_sub{ORB_ID(launch_detection_status)};

	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};

	uORB::SubscriptionCallbackWorkItem _sensor_combined_sub{this, ORB_ID(sensor_combined)};
	uORB::SubscriptionCallbackWorkItem _vehicle_imu_sub{this, ORB_ID(vehicle_imu)};

#if defined(CONFIG_EKF2_RANGE_FINDER)
	hrt_abstime _status_rng_hgt_pub_last {0};

	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_rng_hgt_pub{ORB_ID(estimator_aid_src_rng_hgt)};

	uORB::SubscriptionMultiArray<distance_sensor_s> _distance_sensor_subs{ORB_ID::distance_sensor};
	hrt_abstime _last_range_sensor_update{0};
	int _distance_sensor_selected{-1}; // because we can have several distance sensor instances with different orientations
#endif // CONFIG_EKF2_RANGE_FINDER

	bool _callback_registered{false};

	hrt_abstime _last_event_flags_publish{0};
	hrt_abstime _last_status_flags_publish{0};

	uint64_t _filter_control_status{0};
	uint32_t _filter_fault_status{0};
	uint32_t _innov_check_fail_status{0};

	uint32_t _filter_control_status_changes{0};
	uint32_t _filter_fault_status_changes{0};
	uint32_t _innov_check_fail_status_changes{0};
	uint32_t _filter_information_event_changes{0};

	uORB::PublicationMulti<ekf2_timestamps_s>            _ekf2_timestamps_pub{ORB_ID(ekf2_timestamps)};
	uORB::PublicationMultiData<estimator_event_flags_s>  _estimator_event_flags_pub{ORB_ID(estimator_event_flags)};
	uORB::PublicationMulti<estimator_innovations_s>      _estimator_innovation_test_ratios_pub{ORB_ID(estimator_innovation_test_ratios)};
	uORB::PublicationMulti<estimator_innovations_s>      _estimator_innovation_variances_pub{ORB_ID(estimator_innovation_variances)};
	uORB::PublicationMulti<estimator_innovations_s>      _estimator_innovations_pub{ORB_ID(estimator_innovations)};
	uORB::PublicationMulti<estimator_sensor_bias_s>      _estimator_sensor_bias_pub{ORB_ID(estimator_sensor_bias)};
	uORB::PublicationMulti<estimator_states_s>           _estimator_states_pub{ORB_ID(estimator_states)};
	uORB::PublicationMulti<estimator_status_flags_s>     _estimator_status_flags_pub{ORB_ID(estimator_status_flags)};
	uORB::PublicationMulti<estimator_status_s>           _estimator_status_pub{ORB_ID(estimator_status)};

	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_fake_hgt_pub{ORB_ID(estimator_aid_src_fake_hgt)};
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_fake_pos_pub{ORB_ID(estimator_aid_src_fake_pos)};

	// publications with topic dependent on multi-mode
	uORB::PublicationMulti<vehicle_attitude_s>           _attitude_pub;
	uORB::PublicationMulti<vehicle_local_position_s>     _local_position_pub;
	uORB::PublicationMulti<vehicle_global_position_s>    _global_position_pub;
	uORB::PublicationMulti<vehicle_odometry_s>           _odometry_pub;

#if defined(CONFIG_EKF2_WIND)
	uORB::PublicationMulti<wind_s>              _wind_pub;
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_GNSS)

	uint64_t _last_geoid_height_update_us{0};
	static constexpr float kGeoidHeightLpfTimeConstant = 10.f;
	AlphaFilter<float> _geoid_height_lpf;  ///< height offset between AMSL and ellipsoid

	hrt_abstime _last_gps_status_published{0};

	hrt_abstime _status_gnss_hgt_pub_last{0};
	hrt_abstime _status_gnss_pos_pub_last{0};
	hrt_abstime _status_gnss_vel_pub_last{0};

	float _last_gnss_hgt_bias_published{};

	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};

	uORB::PublicationMulti<estimator_bias_s> _estimator_gnss_hgt_bias_pub{ORB_ID(estimator_gnss_hgt_bias)};
	uORB::PublicationMulti<estimator_gps_status_s> _estimator_gps_status_pub{ORB_ID(estimator_gps_status)};
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_gnss_hgt_pub{ORB_ID(estimator_aid_src_gnss_hgt)};
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_gnss_pos_pub{ORB_ID(estimator_aid_src_gnss_pos)};
	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_gnss_vel_pub{ORB_ID(estimator_aid_src_gnss_vel)};

	uORB::PublicationMulti<yaw_estimator_status_s> _yaw_est_pub{ORB_ID(yaw_estimator_status)};

# if defined(CONFIG_EKF2_GNSS_YAW)
	hrt_abstime _status_gnss_yaw_pub_last {0};
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_gnss_yaw_pub {ORB_ID(estimator_aid_src_gnss_yaw)};
# endif // CONFIG_EKF2_GNSS_YAW
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	hrt_abstime _status_gravity_pub_last {0};
	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_gravity_pub{ORB_ID(estimator_aid_src_gravity)};
#endif // CONFIG_EKF2_GRAVITY_FUSION

	Ekf _ekf;

	parameters *_params;	///< pointer to ekf parameter struct (located in _ekf class instance)

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::EKF2_LOG_VERBOSE>) _param_ekf2_log_verbose,
		(ParamExtInt<px4::params::EKF2_PREDICT_US>) _param_ekf2_predict_us,
		(ParamExtFloat<px4::params::EKF2_DELAY_MAX>) _param_ekf2_delay_max,
		(ParamExtInt<px4::params::EKF2_IMU_CTRL>) _param_ekf2_imu_ctrl,
		(ParamExtFloat<px4::params::EKF2_VEL_LIM>) _param_ekf2_vel_lim,

#if defined(CONFIG_EKF2_AUXVEL)
		(ParamExtFloat<px4::params::EKF2_AVEL_DELAY>)
		_param_ekf2_avel_delay,	///< auxiliary velocity measurement delay relative to the IMU (mSec)
#endif // CONFIG_EKF2_AUXVEL

		(ParamExtFloat<px4::params::EKF2_GYR_NOISE>)
		_param_ekf2_gyr_noise,	///< IMU angular rate noise used for covariance prediction (rad/sec)
		(ParamExtFloat<px4::params::EKF2_ACC_NOISE>)
		_param_ekf2_acc_noise,	///< IMU acceleration noise use for covariance prediction (m/sec**2)

		// process noise
		(ParamExtFloat<px4::params::EKF2_GYR_B_NOISE>)
		_param_ekf2_gyr_b_noise,	///< process noise for IMU rate gyro bias prediction (rad/sec**2)
		(ParamExtFloat<px4::params::EKF2_ACC_B_NOISE>)
		_param_ekf2_acc_b_noise,///< process noise for IMU accelerometer bias prediction (m/sec**3)

#if defined(CONFIG_EKF2_WIND)
		(ParamExtFloat<px4::params::EKF2_WIND_NSD>) _param_ekf2_wind_nsd,
#endif // CONFIG_EKF2_WIND

		(ParamExtFloat<px4::params::EKF2_NOAID_NOISE>) _param_ekf2_noaid_noise,

#if defined(CONFIG_EKF2_GNSS)
		(ParamExtInt<px4::params::EKF2_GPS_CTRL>) _param_ekf2_gps_ctrl,
		(ParamExtFloat<px4::params::EKF2_GPS_DELAY>) _param_ekf2_gps_delay,

		(ParamExtFloat<px4::params::EKF2_GPS_POS_X>) _param_ekf2_gps_pos_x,
		(ParamExtFloat<px4::params::EKF2_GPS_POS_Y>) _param_ekf2_gps_pos_y,
		(ParamExtFloat<px4::params::EKF2_GPS_POS_Z>) _param_ekf2_gps_pos_z,

		(ParamExtFloat<px4::params::EKF2_GPS_V_NOISE>) _param_ekf2_gps_v_noise,
		(ParamExtFloat<px4::params::EKF2_GPS_P_NOISE>) _param_ekf2_gps_p_noise,

		(ParamExtFloat<px4::params::EKF2_GPS_P_GATE>) _param_ekf2_gps_p_gate,
		(ParamExtFloat<px4::params::EKF2_GPS_V_GATE>) _param_ekf2_gps_v_gate,

		(ParamExtInt<px4::params::EKF2_GPS_CHECK>) _param_ekf2_gps_check,
		(ParamExtFloat<px4::params::EKF2_REQ_EPH>)    _param_ekf2_req_eph,
		(ParamExtFloat<px4::params::EKF2_REQ_EPV>)    _param_ekf2_req_epv,
		(ParamExtFloat<px4::params::EKF2_REQ_SACC>)   _param_ekf2_req_sacc,
		(ParamExtInt<px4::params::EKF2_REQ_NSATS>)    _param_ekf2_req_nsats,
		(ParamExtFloat<px4::params::EKF2_REQ_PDOP>)   _param_ekf2_req_pdop,
		(ParamExtFloat<px4::params::EKF2_REQ_HDRIFT>) _param_ekf2_req_hdrift,
		(ParamExtFloat<px4::params::EKF2_REQ_VDRIFT>) _param_ekf2_req_vdrift,
		(ParamFloat<px4::params::EKF2_REQ_GPS_H>)     _param_ekf2_req_gps_h,

		// Used by EKF-GSF experimental yaw estimator
		(ParamExtFloat<px4::params::EKF2_GSF_TAS>) _param_ekf2_gsf_tas_default,
		(ParamFloat<px4::params::EKF2_GPS_YAW_OFF>) _param_ekf2_gps_yaw_off,
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_BAROMETER)
		(ParamExtInt<px4::params::EKF2_BARO_CTRL>) _param_ekf2_baro_ctrl,///< barometer control selection
		(ParamExtFloat<px4::params::EKF2_BARO_DELAY>) _param_ekf2_baro_delay,
		(ParamExtFloat<px4::params::EKF2_BARO_NOISE>) _param_ekf2_baro_noise,
		(ParamExtFloat<px4::params::EKF2_BARO_GATE>) _param_ekf2_baro_gate,
		(ParamExtFloat<px4::params::EKF2_GND_EFF_DZ>) _param_ekf2_gnd_eff_dz,
		(ParamExtFloat<px4::params::EKF2_GND_MAX_HGT>) _param_ekf2_gnd_max_hgt,

# if defined(CONFIG_EKF2_BARO_COMPENSATION)
		// Corrections for static pressure position error where Ps_error = Ps_meas - Ps_truth
		(ParamExtFloat<px4::params::EKF2_ASPD_MAX>) _param_ekf2_aspd_max,
		(ParamExtFloat<px4::params::EKF2_PCOEF_XP>) _param_ekf2_pcoef_xp,
		(ParamExtFloat<px4::params::EKF2_PCOEF_XN>) _param_ekf2_pcoef_xn,
		(ParamExtFloat<px4::params::EKF2_PCOEF_YP>) _param_ekf2_pcoef_yp,
		(ParamExtFloat<px4::params::EKF2_PCOEF_YN>) _param_ekf2_pcoef_yn,
		(ParamExtFloat<px4::params::EKF2_PCOEF_Z>) _param_ekf2_pcoef_z,
# endif // CONFIG_EKF2_BARO_COMPENSATION
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_AIRSPEED)
		(ParamExtFloat<px4::params::EKF2_ASP_DELAY>)
		_param_ekf2_asp_delay, ///< airspeed measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_TAS_GATE>)
		_param_ekf2_tas_gate, ///< True Airspeed innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_EAS_NOISE>)
		_param_ekf2_eas_noise, ///< measurement noise used for airspeed fusion (m/sec)

		// control of airspeed fusion
		(ParamExtFloat<px4::params::EKF2_ARSP_THR>)
		_param_ekf2_arsp_thr, ///< A value of zero will disabled airspeed fusion. Any positive value sets the minimum airspeed which will be used (m/sec)
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
		(ParamExtFloat<px4::params::EKF2_BETA_GATE>) _param_ekf2_beta_gate,
		(ParamExtFloat<px4::params::EKF2_BETA_NOISE>) _param_ekf2_beta_noise,
		(ParamExtInt<px4::params::EKF2_FUSE_BETA>) _param_ekf2_fuse_beta,
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_MAGNETOMETER)
		(ParamExtFloat<px4::params::EKF2_MAG_DELAY>) _param_ekf2_mag_delay,
		(ParamExtFloat<px4::params::EKF2_MAG_E_NOISE>) _param_ekf2_mag_e_noise,
		(ParamExtFloat<px4::params::EKF2_MAG_B_NOISE>) _param_ekf2_mag_b_noise,
		(ParamExtFloat<px4::params::EKF2_HEAD_NOISE>) _param_ekf2_head_noise,
		(ParamExtFloat<px4::params::EKF2_MAG_NOISE>) _param_ekf2_mag_noise,
		(ParamExtFloat<px4::params::EKF2_MAG_DECL>) _param_ekf2_mag_decl,
		(ParamExtFloat<px4::params::EKF2_HDG_GATE>) _param_ekf2_hdg_gate,
		(ParamExtFloat<px4::params::EKF2_MAG_GATE>) _param_ekf2_mag_gate,
		(ParamExtInt<px4::params::EKF2_DECL_TYPE>) _param_ekf2_decl_type,
		(ParamExtInt<px4::params::EKF2_MAG_TYPE>) _param_ekf2_mag_type,
		(ParamExtFloat<px4::params::EKF2_MAG_ACCLIM>) _param_ekf2_mag_acclim,
		(ParamExtInt<px4::params::EKF2_MAG_CHECK>) _param_ekf2_mag_check,
		(ParamExtFloat<px4::params::EKF2_MAG_CHK_STR>) _param_ekf2_mag_chk_str,
		(ParamExtFloat<px4::params::EKF2_MAG_CHK_INC>) _param_ekf2_mag_chk_inc,
		(ParamExtInt<px4::params::EKF2_SYNT_MAG_Z>) _param_ekf2_synthetic_mag_z,
#endif // CONFIG_EKF2_MAGNETOMETER

		(ParamExtInt<px4::params::EKF2_HGT_REF>) _param_ekf2_hgt_ref,    ///< selects the primary source for height data

		(ParamExtInt<px4::params::EKF2_NOAID_TOUT>)
		_param_ekf2_noaid_tout,	///< maximum lapsed time from last fusion of measurements that constrain drift before the EKF will report the horizontal nav solution invalid (uSec)

#if defined(CONFIG_EKF2_TERRAIN) || defined(CONFIG_EKF2_OPTICAL_FLOW) || defined(CONFIG_EKF2_RANGE_FINDER)
		(ParamExtFloat<px4::params::EKF2_MIN_RNG>) _param_ekf2_min_rng,
#endif // CONFIG_EKF2_TERRAIN || CONFIG_EKF2_OPTICAL_FLOW || CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_TERRAIN)
		(ParamExtFloat<px4::params::EKF2_TERR_NOISE>) _param_ekf2_terr_noise,
		(ParamExtFloat<px4::params::EKF2_TERR_GRAD>) _param_ekf2_terr_grad,
#endif // CONFIG_EKF2_TERRAIN
#if defined(CONFIG_EKF2_RANGE_FINDER)
		// range finder fusion
		(ParamExtInt<px4::params::EKF2_RNG_CTRL>) _param_ekf2_rng_ctrl,
		(ParamExtFloat<px4::params::EKF2_RNG_DELAY>) _param_ekf2_rng_delay,
		(ParamExtFloat<px4::params::EKF2_RNG_NOISE>) _param_ekf2_rng_noise,
		(ParamExtFloat<px4::params::EKF2_RNG_SFE>) _param_ekf2_rng_sfe,
		(ParamExtFloat<px4::params::EKF2_RNG_GATE>) _param_ekf2_rng_gate,
		(ParamExtFloat<px4::params::EKF2_RNG_PITCH>) _param_ekf2_rng_pitch,
		(ParamExtFloat<px4::params::EKF2_RNG_A_VMAX>) _param_ekf2_rng_a_vmax,
		(ParamExtFloat<px4::params::EKF2_RNG_A_HMAX>) _param_ekf2_rng_a_hmax,
		(ParamExtFloat<px4::params::EKF2_RNG_A_IGATE>) _param_ekf2_rng_a_igate,
		(ParamExtFloat<px4::params::EKF2_RNG_QLTY_T>) _param_ekf2_rng_qlty_t,
		(ParamExtFloat<px4::params::EKF2_RNG_K_GATE>) _param_ekf2_rng_k_gate,
		(ParamExtFloat<px4::params::EKF2_RNG_FOG>) _param_ekf2_rng_fog,
		(ParamExtFloat<px4::params::EKF2_RNG_POS_X>) _param_ekf2_rng_pos_x,
		(ParamExtFloat<px4::params::EKF2_RNG_POS_Y>) _param_ekf2_rng_pos_y,
		(ParamExtFloat<px4::params::EKF2_RNG_POS_Z>) _param_ekf2_rng_pos_z,
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		// vision estimate fusion
		(ParamExtFloat<px4::params::EKF2_EV_DELAY>)
		_param_ekf2_ev_delay, ///< off-board vision measurement delay relative to the IMU (mSec)

		(ParamExtInt<px4::params::EKF2_EV_CTRL>) _param_ekf2_ev_ctrl,	 ///< external vision (EV) control selection
		(ParamInt<px4::params::EKF2_EV_NOISE_MD>) _param_ekf2_ev_noise_md, ///< determine source of vision observation noise
		(ParamExtInt<px4::params::EKF2_EV_QMIN>) _param_ekf2_ev_qmin,
		(ParamExtFloat<px4::params::EKF2_EVP_NOISE>)
		_param_ekf2_evp_noise, ///< default position observation noise for exernal vision measurements (m)
		(ParamExtFloat<px4::params::EKF2_EVV_NOISE>)
		_param_ekf2_evv_noise, ///< default velocity observation noise for exernal vision measurements (m/s)
		(ParamExtFloat<px4::params::EKF2_EVA_NOISE>)
		_param_ekf2_eva_noise, ///< default angular observation noise for exernal vision measurements (rad)
		(ParamExtFloat<px4::params::EKF2_EVV_GATE>)
		_param_ekf2_evv_gate, ///< external vision velocity innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_EVP_GATE>)
		_param_ekf2_evp_gate, ///< external vision position innovation consistency gate size (STD)

		(ParamExtFloat<px4::params::EKF2_EV_POS_X>)
		_param_ekf2_ev_pos_x, ///< X position of VI sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_EV_POS_Y>)
		_param_ekf2_ev_pos_y, ///< Y position of VI sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_EV_POS_Z>)
		_param_ekf2_ev_pos_z, ///< Z position of VI sensor focal point in body frame (m)
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
		// optical flow fusion
		(ParamExtInt<px4::params::EKF2_OF_CTRL>)
		_param_ekf2_of_ctrl, ///< optical flow fusion selection
		(ParamExtInt<px4::params::EKF2_OF_GYR_SRC>)
		_param_ekf2_of_gyr_src,
		(ParamExtFloat<px4::params::EKF2_OF_DELAY>)
		_param_ekf2_of_delay, ///< optical flow measurement delay relative to the IMU (mSec) - this is to the middle of the optical flow integration interval
		(ParamExtFloat<px4::params::EKF2_OF_N_MIN>)
		_param_ekf2_of_n_min, ///< best quality observation noise for optical flow LOS rate measurements (rad/sec)
		(ParamExtFloat<px4::params::EKF2_OF_N_MAX>)
		_param_ekf2_of_n_max, ///< worst quality observation noise for optical flow LOS rate measurements (rad/sec)
		(ParamExtInt<px4::params::EKF2_OF_QMIN>)
		_param_ekf2_of_qmin, ///< minimum acceptable quality integer from  the flow sensor when in air
		(ParamExtInt<px4::params::EKF2_OF_QMIN_GND>)
		_param_ekf2_of_qmin_gnd, ///< minimum acceptable quality integer from  the flow sensor when on ground
		(ParamExtFloat<px4::params::EKF2_OF_GATE>)
		_param_ekf2_of_gate, ///< optical flow fusion innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_OF_POS_X>)
		_param_ekf2_of_pos_x, ///< X position of optical flow sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_OF_POS_Y>)
		_param_ekf2_of_pos_y, ///< Y position of optical flow sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_OF_POS_Z>)
		_param_ekf2_of_pos_z, ///< Z position of optical flow sensor focal point in body frame (m)
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_DRAG_FUSION)
		(ParamExtInt<px4::params::EKF2_DRAG_CTRL>) _param_ekf2_drag_ctrl,		///< drag fusion selection
		// Multi-rotor drag specific force fusion
		(ParamExtFloat<px4::params::EKF2_DRAG_NOISE>)
		_param_ekf2_drag_noise,	///< observation noise variance for drag specific force measurements (m/sec**2)**2
		(ParamExtFloat<px4::params::EKF2_BCOEF_X>) _param_ekf2_bcoef_x,		///< ballistic coefficient along the X-axis (kg/m**2)
		(ParamExtFloat<px4::params::EKF2_BCOEF_Y>) _param_ekf2_bcoef_y,		///< ballistic coefficient along the Y-axis (kg/m**2)
		(ParamExtFloat<px4::params::EKF2_MCOEF>) _param_ekf2_mcoef,		///< propeller momentum drag coefficient (1/s)
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
		(ParamExtFloat<px4::params::EKF2_GRAV_NOISE>) _param_ekf2_grav_noise,
#endif // CONFIG_EKF2_GRAVITY_FUSION

		// sensor positions in body frame
		(ParamExtFloat<px4::params::EKF2_IMU_POS_X>) _param_ekf2_imu_pos_x,		///< X position of IMU in body frame (m)
		(ParamExtFloat<px4::params::EKF2_IMU_POS_Y>) _param_ekf2_imu_pos_y,		///< Y position of IMU in body frame (m)
		(ParamExtFloat<px4::params::EKF2_IMU_POS_Z>) _param_ekf2_imu_pos_z,		///< Z position of IMU in body frame (m)

		// IMU switch on bias parameters
		(ParamExtFloat<px4::params::EKF2_GBIAS_INIT>)
		_param_ekf2_gbias_init,	///< 1-sigma gyro bias uncertainty at switch on (rad/sec)
		(ParamExtFloat<px4::params::EKF2_ABIAS_INIT>)
		_param_ekf2_abias_init,	///< 1-sigma accelerometer bias uncertainty at switch on (m/sec**2)
		(ParamExtFloat<px4::params::EKF2_ANGERR_INIT>)
		_param_ekf2_angerr_init,	///< 1-sigma tilt error after initial alignment using gravity vector (rad)

		// EKF accel bias learning control
		(ParamExtFloat<px4::params::EKF2_ABL_LIM>) _param_ekf2_abl_lim,	///< Accelerometer bias learning limit (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_ACCLIM>)
		_param_ekf2_abl_acclim,	///< Maximum IMU accel magnitude that allows IMU bias learning (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_GYRLIM>)
		_param_ekf2_abl_gyrlim,	///< Maximum IMU gyro angular rate magnitude that allows IMU bias learning (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_TAU>)
		_param_ekf2_abl_tau,	///< Time constant used to inhibit IMU delta velocity bias learning (sec)

		(ParamExtFloat<px4::params::EKF2_GYR_B_LIM>) _param_ekf2_gyr_b_lim,	///< Gyro bias learning limit (rad/s)

		// output predictor filter time constants
		(ParamFloat<px4::params::EKF2_TAU_VEL>) _param_ekf2_tau_vel,
		(ParamFloat<px4::params::EKF2_TAU_POS>) _param_ekf2_tau_pos
	)
};
#endif // !EKF2_HPP
/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "EKF2Selector.hpp"

using namespace time_literals;
using matrix::Quatf;
using matrix::Vector2f;
using math::constrain;
using math::radians;

EKF2Selector::EKF2Selector() :
	ModuleParams(nullptr),
	ScheduledWorkItem("ekf2_selector", px4::wq_configurations::nav_and_controllers)
{
	_estimator_selector_status_pub.advertise();
	_sensor_selection_pub.advertise();
	_vehicle_attitude_pub.advertise();
	_vehicle_global_position_pub.advertise();
	_vehicle_local_position_pub.advertise();
	_vehicle_odometry_pub.advertise();
	_wind_pub.advertise();
}

EKF2Selector::~EKF2Selector()
{
	Stop();
}

void EKF2Selector::Stop()
{
	for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
		_instance[i].estimator_attitude_sub.unregisterCallback();
		_instance[i].estimator_status_sub.unregisterCallback();
	}

	ScheduleClear();
}

void EKF2Selector::PrintInstanceChange(const uint8_t old_instance, uint8_t new_instance)
{
	const char *old_reason = nullptr;

	if (_instance[old_instance].filter_fault) {
		old_reason = " (filter fault)";

	} else if (_instance[old_instance].timeout) {
		old_reason = " (timeout)";

	} else if (_gyro_fault_detected) {
		old_reason = " (gyro fault)";

	} else if (_accel_fault_detected) {
		old_reason = " (accel fault)";

	} else if (!_instance[_selected_instance].healthy.get_state() && (_instance[_selected_instance].healthy_count > 0)) {
		// skipped if previous instance was never healthy in the first place (eg initialization)
		old_reason = " (unhealthy)";
	}

	const char *new_reason = nullptr;

	if (_request_instance.load() == new_instance) {
		new_reason = " (user selected)";
	}

	if (old_reason || new_reason) {
		if (old_reason == nullptr) {
			old_reason = "";
		}

		if (new_reason == nullptr) {
			new_reason = "";
		}

		PX4_WARN("primary EKF changed %" PRIu8 "%s -> %" PRIu8 "%s", old_instance, old_reason, new_instance, new_reason);
	}
}

bool EKF2Selector::SelectInstance(uint8_t ekf_instance)
{
	if ((ekf_instance != _selected_instance) && (ekf_instance < _available_instances)) {
		// update sensor_selection immediately
		sensor_selection_s sensor_selection{};
		sensor_selection.accel_device_id = _instance[ekf_instance].accel_device_id;
		sensor_selection.gyro_device_id = _instance[ekf_instance].gyro_device_id;
		sensor_selection.timestamp = hrt_absolute_time();
		_sensor_selection_pub.publish(sensor_selection);

		if (_selected_instance != INVALID_INSTANCE) {
			// switch callback registration
			_instance[_selected_instance].estimator_attitude_sub.unregisterCallback();
			_instance[_selected_instance].estimator_status_sub.unregisterCallback();

			PrintInstanceChange(_selected_instance, ekf_instance);
		}

		_instance[ekf_instance].estimator_attitude_sub.registerCallback();
		_instance[ekf_instance].estimator_status_sub.registerCallback();

		_selected_instance = ekf_instance;
		_instance_changed_count++;
		_last_instance_change = sensor_selection.timestamp;
		_instance[ekf_instance].time_last_selected = _last_instance_change;

		// reset all relative test ratios
		for (uint8_t i = 0; i < _available_instances; i++) {
			_instance[i].relative_test_ratio = 0;
		}

		return true;
	}

	return false;
}

bool EKF2Selector::UpdateErrorScores()
{
	// first check imu inconsistencies
	_gyro_fault_detected = false;
	uint32_t faulty_gyro_id = 0;
	_accel_fault_detected = false;
	uint32_t faulty_accel_id = 0;

	if (_sensors_status_imu.updated()) {
		sensors_status_imu_s sensors_status_imu;

		if (_sensors_status_imu.copy(&sensors_status_imu)) {

			const float time_step_s = constrain((sensors_status_imu.timestamp - _last_update_us) * 1e-6f, 0.f, 0.02f);
			_last_update_us = sensors_status_imu.timestamp;

			{
				const float angle_rate_threshold = radians(_param_ekf2_sel_imu_angle_rate.get());
				const float angle_threshold = radians(_param_ekf2_sel_imu_angle.get());
				uint8_t n_gyros = 0;
				uint8_t n_gyro_exceedances = 0;
				float largest_accumulated_gyro_error = 0.0f;
				uint8_t largest_gyro_error_index = 0;

				for (unsigned i = 0; i < IMU_STATUS_SIZE; i++) {
					// check for gyros with excessive difference to mean using accumulated error
					if (sensors_status_imu.gyro_device_ids[i] != 0) {
						n_gyros++;
						_accumulated_gyro_error[i] += (sensors_status_imu.gyro_inconsistency_rad_s[i] - angle_rate_threshold) * time_step_s;
						_accumulated_gyro_error[i] = fmaxf(_accumulated_gyro_error[i], 0.f);

						if (_accumulated_gyro_error[i] > angle_threshold) {
							n_gyro_exceedances++;
						}

						if (_accumulated_gyro_error[i] > largest_accumulated_gyro_error) {
							largest_accumulated_gyro_error = _accumulated_gyro_error[i];
							largest_gyro_error_index = i;
						}

					} else {
						// no sensor
						_accumulated_gyro_error[i] = NAN;
					}
				}

				if (n_gyro_exceedances > 0) {
					if (n_gyros >= 3) {
						// If there are 3 or more sensors, the one with the largest accumulated error is faulty
						_gyro_fault_detected = true;
						faulty_gyro_id = sensors_status_imu.gyro_device_ids[largest_gyro_error_index];

					} else if (n_gyros == 2) {
						// A fault is present, but the faulty sensor identity cannot be determined
						_gyro_fault_detected = true;
					}
				}
			}

			{
				const float accel_threshold = _param_ekf2_sel_imu_accel.get();
				const float velocity_threshold = _param_ekf2_sel_imu_velocity.get();
				uint8_t n_accels = 0;
				uint8_t n_accel_exceedances = 0;
				float largest_accumulated_accel_error = 0.0f;
				uint8_t largest_accel_error_index = 0;

				for (unsigned i = 0; i < IMU_STATUS_SIZE; i++) {
					// check for accelerometers with excessive difference to mean using accumulated error
					if (sensors_status_imu.accel_device_ids[i] != 0) {
						n_accels++;
						_accumulated_accel_error[i] += (sensors_status_imu.accel_inconsistency_m_s_s[i] - accel_threshold) * time_step_s;
						_accumulated_accel_error[i] = fmaxf(_accumulated_accel_error[i], 0.f);

						if (_accumulated_accel_error[i] > velocity_threshold) {
							n_accel_exceedances++;
						}

						if (_accumulated_accel_error[i] > largest_accumulated_accel_error) {
							largest_accumulated_accel_error = _accumulated_accel_error[i];
							largest_accel_error_index = i;
						}

					} else {
						// no sensor
						_accumulated_accel_error[i] = NAN;
					}
				}

				if (n_accel_exceedances > 0) {
					if (n_accels >= 3) {
						// If there are 3 or more sensors, the one with the largest accumulated error is faulty
						_accel_fault_detected = true;
						faulty_accel_id = sensors_status_imu.accel_device_ids[largest_accel_error_index];

					} else if (n_accels == 2) {
						// A fault is present, but the faulty sensor identity cannot be determined
						_accel_fault_detected = true;
					}
				}
			}
		}
	}

	bool updated = false;
	bool primary_updated = false;

	// default estimator timeout
	const hrt_abstime status_timeout = 50_ms;

	// calculate individual error scores
	for (uint8_t i = 0; i < EKF2_MAX_INSTANCES; i++) {
		const bool prev_healthy = _instance[i].healthy.get_state();

		estimator_status_s status;

		if (_instance[i].estimator_status_sub.update(&status)) {

			_instance[i].timestamp_last = status.timestamp;

			_instance[i].accel_device_id = status.accel_device_id;
			_instance[i].gyro_device_id = status.gyro_device_id;
			_instance[i].baro_device_id = status.baro_device_id;
			_instance[i].mag_device_id = status.mag_device_id;

			if ((i + 1) > _available_instances) {
				_available_instances = i + 1;
				updated = true;
			}

			if (i == _selected_instance) {
				primary_updated = true;
			}

			// test ratios are invalid when 0, >= 1 is a failure
			if (!PX4_ISFINITE(status.vel_test_ratio) || (status.vel_test_ratio <= 0.f)) {
				status.vel_test_ratio = 1.f;
			}

			if (!PX4_ISFINITE(status.pos_test_ratio) || (status.pos_test_ratio <= 0.f)) {
				status.pos_test_ratio = 1.f;
			}

			if (!PX4_ISFINITE(status.hgt_test_ratio) || (status.hgt_test_ratio <= 0.f)) {
				status.hgt_test_ratio = 1.f;
			}

			float combined_test_ratio = fmaxf(0.5f * (status.vel_test_ratio + status.pos_test_ratio), status.hgt_test_ratio);

			_instance[i].combined_test_ratio = combined_test_ratio;

			const bool healthy = (status.filter_fault_flags == 0) && (combined_test_ratio > 0.f);
			_instance[i].healthy.set_state_and_update(healthy, status.timestamp);

			_instance[i].warning = (combined_test_ratio >= 1.f);
			_instance[i].filter_fault = (status.filter_fault_flags != 0);
			_instance[i].timeout = false;

			if (!_instance[i].warning) {
				_instance[i].time_last_no_warning = status.timestamp;
			}

			if (!PX4_ISFINITE(_instance[i].relative_test_ratio)) {
				_instance[i].relative_test_ratio = 0;
			}

		} else if (!_instance[i].timeout && (hrt_elapsed_time(&_instance[i].timestamp_last) > status_timeout)) {
			_instance[i].healthy.set_state_and_update(false, hrt_absolute_time());
			_instance[i].timeout = true;
		}

		// if the gyro used by the EKF is faulty, declare the EKF unhealthy without delay
		if (_gyro_fault_detected && (faulty_gyro_id != 0) && (_instance[i].gyro_device_id == faulty_gyro_id)) {
			_instance[i].healthy.set_state_and_update(false, hrt_absolute_time());
		}

		// if the accelerometer used by the EKF is faulty, declare the EKF unhealthy without delay
		if (_accel_fault_detected && (faulty_accel_id != 0) && (_instance[i].accel_device_id == faulty_accel_id)) {
			_instance[i].healthy.set_state_and_update(false, hrt_absolute_time());
		}

		if (prev_healthy != _instance[i].healthy.get_state()) {
			updated = true;
			_selector_status_publish = true;

			if (!prev_healthy) {
				_instance[i].healthy_count++;
			}
		}
	}

	// update relative test ratios if primary has updated
	if (primary_updated) {
		for (uint8_t i = 0; i < _available_instances; i++) {
			if (i != _selected_instance) {

				const float error_delta = _instance[i].combined_test_ratio - _instance[_selected_instance].combined_test_ratio;

				// reduce error only if its better than the primary instance by at least EKF2_SEL_ERR_RED to prevent unnecessary selection changes
				const float threshold = _gyro_fault_detected ? 0.0f : fmaxf(_param_ekf2_sel_err_red.get(), 0.05f);

				if (error_delta > 0 || error_delta < -threshold) {
					_instance[i].relative_test_ratio += error_delta;
					_instance[i].relative_test_ratio = constrain(_instance[i].relative_test_ratio, -_rel_err_score_lim, _rel_err_score_lim);

					if ((error_delta < -threshold) && (_instance[i].relative_test_ratio < 1.f)) {
						// increase status publication rate if there's movement towards a potential instance change
						_selector_status_publish = true;
					}
				}
			}
		}
	}

	return (primary_updated || updated);
}

void EKF2Selector::PublishVehicleAttitude()
{
	// selected estimator_attitude -> vehicle_attitude
	vehicle_attitude_s attitude;

	if (_instance[_selected_instance].estimator_attitude_sub.update(&attitude)) {
		bool instance_change = false;

		if (_instance[_selected_instance].estimator_attitude_sub.get_instance() != _attitude_instance_prev) {
			_attitude_instance_prev = _instance[_selected_instance].estimator_attitude_sub.get_instance();
			instance_change = true;
		}

		if (_attitude_last.timestamp != 0) {
			if (!instance_change && (attitude.quat_reset_counter == _attitude_last.quat_reset_counter + 1)) {
				// propogate deltas from estimator data while maintaining the overall reset counts
				++_quat_reset_counter;
				_delta_q_reset = Quatf{attitude.delta_q_reset};

			} else if (instance_change || (attitude.quat_reset_counter != _attitude_last.quat_reset_counter)) {
				// on reset compute deltas from last published data
				++_quat_reset_counter;
				_delta_q_reset = (Quatf(attitude.q) * Quatf(_attitude_last.q).inversed()).normalized();
			}

		} else {
			_quat_reset_counter = attitude.quat_reset_counter;
			_delta_q_reset = Quatf{attitude.delta_q_reset};
		}

		bool publish = true;

		// ensure monotonically increasing timestamp_sample through reset, don't publish
		//  estimator's attitude for system (vehicle_attitude) if it's stale
		if ((attitude.timestamp_sample <= _attitude_last.timestamp_sample)
		    || (hrt_elapsed_time(&attitude.timestamp) > 10_ms)) {

			publish = false;
		}

		// save last primary estimator_attitude as published with original resets
		_attitude_last = attitude;

		if (publish) {
			// republish with total reset count and current timestamp
			attitude.quat_reset_counter = _quat_reset_counter;
			_delta_q_reset.copyTo(attitude.delta_q_reset);

			attitude.timestamp = hrt_absolute_time();
			_vehicle_attitude_pub.publish(attitude);
		}
	}
}

void EKF2Selector::PublishVehicleLocalPosition()
{
	// selected estimator_local_position -> vehicle_local_position
	vehicle_local_position_s local_position;

	if (_instance[_selected_instance].estimator_local_position_sub.update(&local_position)) {
		bool instance_change = false;

		if (_instance[_selected_instance].estimator_local_position_sub.get_instance() != _local_position_instance_prev) {
			_local_position_instance_prev = _instance[_selected_instance].estimator_local_position_sub.get_instance();
			instance_change = true;
		}

		if (_local_position_last.timestamp != 0) {
			// XY reset
			if (!instance_change && (local_position.xy_reset_counter == _local_position_last.xy_reset_counter + 1)) {
				++_xy_reset_counter;
				_delta_xy_reset = Vector2f{local_position.delta_xy};

			} else if (instance_change || (local_position.xy_reset_counter != _local_position_last.xy_reset_counter)) {
				++_xy_reset_counter;
				_delta_xy_reset = Vector2f{local_position.x, local_position.y} - Vector2f{_local_position_last.x, _local_position_last.y};
			}

			// Z reset
			if (!instance_change && (local_position.z_reset_counter == _local_position_last.z_reset_counter + 1)) {
				++_z_reset_counter;
				_delta_z_reset = local_position.delta_z;

			} else if (instance_change || (local_position.z_reset_counter != _local_position_last.z_reset_counter)) {
				++_z_reset_counter;
				_delta_z_reset = local_position.z - _local_position_last.z;
			}

			// VXY reset
			if (!instance_change && (local_position.vxy_reset_counter == _local_position_last.vxy_reset_counter + 1)) {
				++_vxy_reset_counter;
				_delta_vxy_reset = Vector2f{local_position.delta_vxy};

			} else if (instance_change || (local_position.vxy_reset_counter != _local_position_last.vxy_reset_counter)) {
				++_vxy_reset_counter;
				_delta_vxy_reset = Vector2f{local_position.vx, local_position.vy} - Vector2f{_local_position_last.vx, _local_position_last.vy};
			}

			// VZ reset
			if (!instance_change && (local_position.vz_reset_counter == _local_position_last.vz_reset_counter + 1)) {
				++_vz_reset_counter;
				_delta_vz_reset = local_position.delta_vz;

			} else if (instance_change || (local_position.vz_reset_counter != _local_position_last.vz_reset_counter)) {
				++_vz_reset_counter;
				_delta_vz_reset = local_position.vz - _local_position_last.vz;
			}

			// heading reset
			if (!instance_change && (local_position.heading_reset_counter == _local_position_last.heading_reset_counter + 1)) {
				++_heading_reset_counter;
				_delta_heading_reset = local_position.delta_heading;

			} else if (instance_change || (local_position.heading_reset_counter != _local_position_last.heading_reset_counter)) {
				++_heading_reset_counter;
				_delta_heading_reset = matrix::wrap_pi(local_position.heading - _local_position_last.heading);
			}

			// HAGL (dist_bottom) reset
			if (!instance_change
			    && (local_position.dist_bottom_reset_counter == _local_position_last.dist_bottom_reset_counter + 1)) {
				++_hagl_reset_counter;
				_delta_hagl_reset = local_position.delta_dist_bottom;

			} else if (instance_change
				   || (local_position.dist_bottom_reset_counter != _local_position_last.dist_bottom_reset_counter)) {
				++_hagl_reset_counter;
				_delta_hagl_reset = local_position.dist_bottom - _local_position_last.dist_bottom;
			}

		} else {
			_xy_reset_counter = local_position.xy_reset_counter;
			_z_reset_counter = local_position.z_reset_counter;
			_vxy_reset_counter = local_position.vxy_reset_counter;
			_vz_reset_counter = local_position.vz_reset_counter;
			_heading_reset_counter = local_position.heading_reset_counter;
			_hagl_reset_counter = local_position.dist_bottom_reset_counter;

			_delta_xy_reset = Vector2f{local_position.delta_xy};
			_delta_z_reset = local_position.delta_z;
			_delta_vxy_reset = Vector2f{local_position.delta_vxy};
			_delta_vz_reset = local_position.delta_vz;
			_delta_heading_reset = local_position.delta_heading;
			_delta_hagl_reset = local_position.delta_dist_bottom;
		}

		bool publish = true;

		// ensure monotonically increasing timestamp_sample through reset, don't publish
		//  estimator's local position for system (vehicle_local_position) if it's stale
		if ((local_position.timestamp_sample <= _local_position_last.timestamp_sample)
		    || (hrt_elapsed_time(&local_position.timestamp) > 20_ms)) {

			publish = false;
		}

		// save last primary estimator_local_position as published with original resets
		_local_position_last = local_position;

		if (publish) {
			// republish with total reset count and current timestamp
			local_position.xy_reset_counter = _xy_reset_counter;
			local_position.z_reset_counter = _z_reset_counter;
			local_position.vxy_reset_counter = _vxy_reset_counter;
			local_position.vz_reset_counter = _vz_reset_counter;
			local_position.heading_reset_counter = _heading_reset_counter;
			local_position.dist_bottom_reset_counter = _hagl_reset_counter;

			_delta_xy_reset.copyTo(local_position.delta_xy);
			local_position.delta_z = _delta_z_reset;
			_delta_vxy_reset.copyTo(local_position.delta_vxy);
			local_position.delta_vz = _delta_vz_reset;
			local_position.delta_heading = _delta_heading_reset;

			local_position.timestamp = hrt_absolute_time();
			_vehicle_local_position_pub.publish(local_position);
		}
	}
}

void EKF2Selector::PublishVehicleOdometry()
{
	// selected estimator_odometry -> vehicle_odometry
	vehicle_odometry_s odometry;

	if (_instance[_selected_instance].estimator_odometry_sub.update(&odometry)) {

		bool instance_change = false;

		if (_instance[_selected_instance].estimator_odometry_sub.get_instance() != _odometry_instance_prev) {
			_odometry_instance_prev = _instance[_selected_instance].estimator_odometry_sub.get_instance();
			instance_change = true;
		}

		if (_odometry_last.timestamp != 0) {
			// reset
			if (instance_change || (odometry.reset_counter != _odometry_last.reset_counter)) {
				++_odometry_reset_counter;
			}

		} else {
			_odometry_reset_counter = odometry.reset_counter;
		}

		bool publish = true;

		// ensure monotonically increasing timestamp_sample through reset, don't publish
		//  estimator's odometry for system (vehicle_odometry) if it's stale
		if ((odometry.timestamp_sample <= _odometry_last.timestamp_sample)
		    || (hrt_elapsed_time(&odometry.timestamp) > 20_ms)) {

			publish = false;
		}

		// save last primary estimator_odometry as published with original resets
		_odometry_last = odometry;

		if (publish) {
			// republish with total reset count and current timestamp
			odometry.reset_counter = _odometry_reset_counter;

			odometry.timestamp = hrt_absolute_time();
			_vehicle_odometry_pub.publish(odometry);
		}
	}
}

void EKF2Selector::PublishVehicleGlobalPosition()
{
	// selected estimator_global_position -> vehicle_global_position
	vehicle_global_position_s global_position;

	if (_instance[_selected_instance].estimator_global_position_sub.update(&global_position)) {
		bool instance_change = false;

		if (_instance[_selected_instance].estimator_global_position_sub.get_instance() != _global_position_instance_prev) {
			_global_position_instance_prev = _instance[_selected_instance].estimator_global_position_sub.get_instance();
			instance_change = true;
		}

		if (_global_position_last.timestamp != 0) {
			// lat/lon reset
			if (!instance_change && (global_position.lat_lon_reset_counter == _global_position_last.lat_lon_reset_counter + 1)) {
				++_lat_lon_reset_counter;

				// TODO: delta latitude/longitude
				_delta_lat_reset = global_position.lat - _global_position_last.lat;
				_delta_lon_reset = global_position.lon - _global_position_last.lon;

			} else if (instance_change || (global_position.lat_lon_reset_counter != _global_position_last.lat_lon_reset_counter)) {
				++_lat_lon_reset_counter;

				_delta_lat_reset = global_position.lat - _global_position_last.lat;
				_delta_lon_reset = global_position.lon - _global_position_last.lon;
			}

			// alt reset
			if (!instance_change && (global_position.alt_reset_counter == _global_position_last.alt_reset_counter + 1)) {
				++_alt_reset_counter;
				_delta_alt_reset = global_position.delta_alt;

			} else if (instance_change || (global_position.alt_reset_counter != _global_position_last.alt_reset_counter)) {
				++_alt_reset_counter;
				_delta_alt_reset = global_position.delta_alt - _global_position_last.delta_alt;
			}

			// terrain reset
			if (!instance_change && (global_position.terrain_reset_counter == _global_position_last.terrain_reset_counter + 1)) {
				++_terrain_reset_counter;
				_delta_terrain_reset = global_position.delta_terrain;

			} else if (instance_change || (global_position.terrain_reset_counter != _global_position_last.terrain_reset_counter)) {
				++_terrain_reset_counter;
				_delta_terrain_reset = global_position.delta_terrain - _global_position_last.delta_terrain;
			}

		} else {
			_lat_lon_reset_counter = global_position.lat_lon_reset_counter;
			_alt_reset_counter = global_position.alt_reset_counter;

			_delta_alt_reset = global_position.delta_alt;
		}

		bool publish = true;

		// ensure monotonically increasing timestamp_sample through reset, don't publish
		//  estimator's global position for system (vehicle_global_position) if it's stale
		if ((global_position.timestamp_sample <= _global_position_last.timestamp_sample)
		    || (hrt_elapsed_time(&global_position.timestamp) > 20_ms)) {

			publish = false;
		}

		// save last primary estimator_global_position as published with original resets
		_global_position_last = global_position;

		if (publish) {
			// republish with total reset count and current timestamp
			global_position.lat_lon_reset_counter = _lat_lon_reset_counter;
			global_position.alt_reset_counter = _alt_reset_counter;
			global_position.delta_alt = _delta_alt_reset;

			global_position.timestamp = hrt_absolute_time();
			_vehicle_global_position_pub.publish(global_position);
		}
	}
}

void EKF2Selector::PublishWindEstimate()
{
	// selected estimator_wind -> wind
	wind_s wind;

	if (_instance[_selected_instance].estimator_wind_sub.update(&wind)) {
		bool publish = true;

		// ensure monotonically increasing timestamp_sample through reset, don't publish
		//  estimator's wind for system (wind) if it's stale
		if ((wind.timestamp_sample <= _wind_last.timestamp_sample)
		    || (hrt_elapsed_time(&wind.timestamp) > 100_ms)) {

			publish = false;
		}

		// save last primary wind
		_wind_last = wind;

		// publish estimator's wind for system unless it's stale
		if (publish) {
			// republish with current timestamp
			wind.timestamp = hrt_absolute_time();
			_wind_pub.publish(wind);
		}
	}
}

void EKF2Selector::Run()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	// update combined test ratio for all estimators
	const bool updated = UpdateErrorScores();

	// if no valid instance then force select first instance with valid IMU
	if (_selected_instance == INVALID_INSTANCE) {
		for (uint8_t i = 0; i < EKF2_MAX_INSTANCES; i++) {
			if ((_instance[i].accel_device_id != 0)
			    && (_instance[i].gyro_device_id != 0)) {

				if (SelectInstance(i)) {
					break;
				}
			}
		}

		// if still invalid return early and check again on next scheduled run
		if (_selected_instance == INVALID_INSTANCE) {
			ScheduleDelayed(100_ms);
			return;
		}
	}

	if (updated) {
		const uint8_t available_instances_prev = _available_instances;
		const uint8_t selected_instance_prev = _selected_instance;
		const uint32_t instance_changed_count_prev = _instance_changed_count;
		const hrt_abstime last_instance_change_prev = _last_instance_change;

		bool lower_error_available = false;
		float alternative_error = 0.f; // looking for instances that have error lower than the current primary
		float best_test_ratio = FLT_MAX;

		uint8_t best_ekf = _selected_instance;
		uint8_t best_ekf_alternate = INVALID_INSTANCE;
		uint8_t best_ekf_different_imu = INVALID_INSTANCE;

		// loop through all available instances to find if an alternative is available
		for (int i = 0; i < _available_instances; i++) {
			// Use an alternative instance if  -
			// (healthy and has updated recently)
			// AND
			// (has relative error less than selected instance and has not been the selected instance for at least 10 seconds
			// OR
			// selected instance has stopped updating
			if (_instance[i].healthy.get_state() && (i != _selected_instance)) {
				const float test_ratio = _instance[i].combined_test_ratio;
				const float relative_error = _instance[i].relative_test_ratio;

				if (relative_error < alternative_error) {
					best_ekf_alternate = i;
					alternative_error = relative_error;

					// relative error less than selected instance and has not been the selected instance for at least 10 seconds
					if ((relative_error <= -_rel_err_thresh) && hrt_elapsed_time(&_instance[i].time_last_selected) > 10_s) {
						lower_error_available = true;
					}
				}

				if ((test_ratio > 0) && (test_ratio < best_test_ratio)) {
					best_ekf = i;
					best_test_ratio = test_ratio;

					// also check next best available ekf using a different IMU
					if (_instance[i].accel_device_id != _instance[_selected_instance].accel_device_id) {
						best_ekf_different_imu = i;
					}
				}
			}
		}

		if (!_instance[_selected_instance].healthy.get_state()) {
			// prefer the best healthy instance using a different IMU
			if (!SelectInstance(best_ekf_different_imu)) {
				// otherwise switch to the healthy instance with best overall test ratio
				SelectInstance(best_ekf);
			}

		} else if (lower_error_available
			   && ((hrt_elapsed_time(&_last_instance_change) > 10_s)
			       || (_instance[_selected_instance].warning
				   && (hrt_elapsed_time(&_instance[_selected_instance].time_last_no_warning) > 1_s)))) {

			// if this instance has a significantly lower relative error to the active primary, we consider it as a
			// better instance and would like to switch to it even if the current primary is healthy
			SelectInstance(best_ekf_alternate);

		} else if (_request_instance.load() != INVALID_INSTANCE) {

			const uint8_t new_instance = _request_instance.load();

			// attempt to switch to user manually selected instance
			if (!SelectInstance(new_instance)) {
				PX4_ERR("unable to switch to user selected instance %d", new_instance);
			}

			// reset
			_request_instance.store(INVALID_INSTANCE);
		}

		// publish selector status at ~1 Hz or immediately on any change
		if (_selector_status_publish || (hrt_elapsed_time(&_last_status_publish) > 1_s)
		    || (available_instances_prev != _available_instances)
		    || (selected_instance_prev != _selected_instance)
		    || (instance_changed_count_prev != _instance_changed_count)
		    || (last_instance_change_prev != _last_instance_change)
		    || _accel_fault_detected || _gyro_fault_detected) {

			PublishEstimatorSelectorStatus();
			_selector_status_publish = false;
		}
	}

	// republish selected estimator data for system
	PublishVehicleAttitude();
	PublishVehicleLocalPosition();
	PublishVehicleGlobalPosition();
	PublishVehicleOdometry();
	PublishWindEstimate();

	// re-schedule as backup timeout
	ScheduleDelayed(FILTER_UPDATE_PERIOD);
}

void EKF2Selector::PublishEstimatorSelectorStatus()
{
	estimator_selector_status_s selector_status{};
	selector_status.primary_instance = _selected_instance;
	selector_status.instances_available = _available_instances;
	selector_status.instance_changed_count = _instance_changed_count;
	selector_status.last_instance_change = _last_instance_change;
	selector_status.accel_device_id = _instance[_selected_instance].accel_device_id;
	selector_status.baro_device_id = _instance[_selected_instance].baro_device_id;
	selector_status.gyro_device_id = _instance[_selected_instance].gyro_device_id;
	selector_status.mag_device_id = _instance[_selected_instance].mag_device_id;
	selector_status.gyro_fault_detected = _gyro_fault_detected;
	selector_status.accel_fault_detected = _accel_fault_detected;

	for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
		selector_status.combined_test_ratio[i] = _instance[i].combined_test_ratio;
		selector_status.relative_test_ratio[i] = _instance[i].relative_test_ratio;
		selector_status.healthy[i] = _instance[i].healthy.get_state();
	}

	for (int i = 0; i < IMU_STATUS_SIZE; i++) {
		selector_status.accumulated_gyro_error[i] = _accumulated_gyro_error[i];
		selector_status.accumulated_accel_error[i] = _accumulated_accel_error[i];
	}

	selector_status.timestamp = hrt_absolute_time();
	_estimator_selector_status_pub.publish(selector_status);
	_last_status_publish = selector_status.timestamp;
}

void EKF2Selector::PrintStatus()
{
	PX4_INFO("available instances: %" PRIu8, _available_instances);

	if (_selected_instance == INVALID_INSTANCE) {
		PX4_WARN("selected instance: None");
	}

	for (int i = 0; i < _available_instances; i++) {
		const EstimatorInstance &inst = _instance[i];

		PX4_INFO("%" PRIu8 ": ACC: %" PRIu32 ", GYRO: %" PRIu32 ", MAG: %" PRIu32 ", %s, test ratio: %.7f (%.5f) %s",
			 inst.instance, inst.accel_device_id, inst.gyro_device_id, inst.mag_device_id,
			 inst.healthy.get_state() ? "healthy" : "unhealthy",
			 (double)inst.combined_test_ratio, (double)inst.relative_test_ratio,
			 (_selected_instance == i) ? "*" : "");
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef EKF2SELECTOR_HPP
#define EKF2SELECTOR_HPP

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/time.h>
#include <lib/hysteresis/hysteresis.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/sensors_status_imu.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/wind.h>

#if CONSTRAINED_MEMORY
# define EKF2_MAX_INSTANCES 2
#else
# define EKF2_MAX_INSTANCES 9
#endif

using namespace time_literals;

class EKF2Selector : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	EKF2Selector();
	~EKF2Selector() override;

	void Stop();

	void PrintStatus();

	void RequestInstance(uint8_t instance) { _request_instance.store(instance); }

private:
	static constexpr uint8_t INVALID_INSTANCE{UINT8_MAX};
	static constexpr uint64_t FILTER_UPDATE_PERIOD{10_ms};

	void Run() override;

	void PrintInstanceChange(const uint8_t old_instance, uint8_t new_instance);

	void PublishEstimatorSelectorStatus();
	void PublishVehicleAttitude();
	void PublishVehicleLocalPosition();
	void PublishVehicleGlobalPosition();
	void PublishVehicleOdometry();
	void PublishWindEstimate();

	bool SelectInstance(uint8_t instance);

	// Update the error scores for all available instances
	bool UpdateErrorScores();

	// Subscriptions (per estimator instance)
	struct EstimatorInstance {

		EstimatorInstance(EKF2Selector *selector, uint8_t i) :
			estimator_attitude_sub{selector, ORB_ID(estimator_attitude), i},
			estimator_status_sub{selector, ORB_ID(estimator_status), i},
			estimator_local_position_sub{ORB_ID(estimator_local_position), i},
			estimator_global_position_sub{ORB_ID(estimator_global_position), i},
			estimator_odometry_sub{ORB_ID(estimator_odometry), i},
			estimator_wind_sub{ORB_ID(estimator_wind), i},
			instance(i)
		{
			healthy.set_hysteresis_time_from(false, 1_s);
		}

		uORB::SubscriptionCallbackWorkItem estimator_attitude_sub;
		uORB::SubscriptionCallbackWorkItem estimator_status_sub;

		uORB::Subscription estimator_local_position_sub;
		uORB::Subscription estimator_global_position_sub;
		uORB::Subscription estimator_odometry_sub;
		uORB::Subscription estimator_wind_sub;

		uint64_t timestamp_last{0};

		uint32_t accel_device_id{0};
		uint32_t gyro_device_id{0};
		uint32_t baro_device_id{0};
		uint32_t mag_device_id{0};

		hrt_abstime time_last_selected{0};
		hrt_abstime time_last_no_warning{0};

		float combined_test_ratio{NAN};
		float relative_test_ratio{NAN};

		systemlib::Hysteresis healthy{false};

		bool warning{false};
		bool filter_fault{false};
		bool timeout{false};

		uint8_t healthy_count{0};

		const uint8_t instance;
	};

	static constexpr float _rel_err_score_lim{1.0f}; // +- limit applied to the relative error score
	static constexpr float _rel_err_thresh{0.5f};    // the relative score difference needs to be greater than this to switch from an otherwise healthy instance

	EstimatorInstance _instance[EKF2_MAX_INSTANCES] {
		{this, 0},
		{this, 1},
#if EKF2_MAX_INSTANCES > 2
		{this, 2},
		{this, 3},
#if EKF2_MAX_INSTANCES > 4
		{this, 4},
		{this, 5},
		{this, 6},
		{this, 7},
		{this, 8},
#endif
#endif
	};

	static constexpr uint8_t IMU_STATUS_SIZE = (sizeof(sensors_status_imu_s::gyro_inconsistency_rad_s) / sizeof(
				sensors_status_imu_s::gyro_inconsistency_rad_s[0]));
	static_assert(IMU_STATUS_SIZE == sizeof(estimator_selector_status_s::accumulated_gyro_error) / sizeof(
			      estimator_selector_status_s::accumulated_gyro_error[0]),
		      "increase estimator_selector_status_s::accumulated_gyro_error size");
	static_assert(IMU_STATUS_SIZE == sizeof(estimator_selector_status_s::accumulated_accel_error) / sizeof(
			      estimator_selector_status_s::accumulated_accel_error[0]),
		      "increase estimator_selector_status_s::accumulated_accel_error size");
	static_assert(EKF2_MAX_INSTANCES <= sizeof(estimator_selector_status_s::combined_test_ratio) / sizeof(
			      estimator_selector_status_s::combined_test_ratio[0]),
		      "increase estimator_selector_status_s::combined_test_ratio size");

	float _accumulated_gyro_error[IMU_STATUS_SIZE] {};
	float _accumulated_accel_error[IMU_STATUS_SIZE] {};
	hrt_abstime _last_update_us{0};
	bool _gyro_fault_detected{false};
	bool _accel_fault_detected{false};

	uint8_t _available_instances{0};
	uint8_t _selected_instance{INVALID_INSTANCE};
	px4::atomic<uint8_t> _request_instance{INVALID_INSTANCE};

	uint32_t _instance_changed_count{0};
	hrt_abstime _last_instance_change{0};

	hrt_abstime _last_status_publish{0};
	bool _selector_status_publish{false};

	// vehicle_attitude: reset counters
	vehicle_attitude_s _attitude_last{};
	matrix::Quatf _delta_q_reset{};
	uint8_t _quat_reset_counter{0};

	// vehicle_local_position: reset counters
	vehicle_local_position_s _local_position_last{};
	matrix::Vector2f _delta_xy_reset{};
	float _delta_z_reset{0.f};
	matrix::Vector2f _delta_vxy_reset{};
	float _delta_vz_reset{0.f};
	float _delta_heading_reset{0};
	float _delta_hagl_reset{0.f};

	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _vxy_reset_counter{0};
	uint8_t _vz_reset_counter{0};
	uint8_t _heading_reset_counter{0};
	uint8_t _hagl_reset_counter{0};

	// vehicle_odometry
	vehicle_odometry_s _odometry_last{};
	uint8_t _odometry_reset_counter{0};

	// vehicle_global_position: reset counters
	vehicle_global_position_s _global_position_last{};
	double _delta_lat_reset{0};
	double _delta_lon_reset{0};
	float _delta_alt_reset{0.f};
	float _delta_terrain_reset{0.f};

	uint8_t _lat_lon_reset_counter{0};
	uint8_t _alt_reset_counter{0};
	uint8_t _terrain_reset_counter{0};

	// wind estimate
	wind_s _wind_last{};

	uint8_t _attitude_instance_prev{INVALID_INSTANCE};
	uint8_t _local_position_instance_prev{INVALID_INSTANCE};
	uint8_t _global_position_instance_prev{INVALID_INSTANCE};
	uint8_t _odometry_instance_prev{INVALID_INSTANCE};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _sensors_status_imu{ORB_ID(sensors_status_imu)};

	// Publications
	uORB::Publication<estimator_selector_status_s> _estimator_selector_status_pub{ORB_ID(estimator_selector_status)};
	uORB::Publication<sensor_selection_s>          _sensor_selection_pub{ORB_ID(sensor_selection)};
	uORB::Publication<vehicle_attitude_s>          _vehicle_attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_global_position_s>   _vehicle_global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_local_position_s>    _vehicle_local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_odometry_s>          _vehicle_odometry_pub{ORB_ID(vehicle_odometry)};
	uORB::Publication<wind_s>             _wind_pub{ORB_ID(wind)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::EKF2_SEL_ERR_RED>) _param_ekf2_sel_err_red,
		(ParamFloat<px4::params::EKF2_SEL_IMU_RAT>) _param_ekf2_sel_imu_angle_rate,
		(ParamFloat<px4::params::EKF2_SEL_IMU_ANG>) _param_ekf2_sel_imu_angle,
		(ParamFloat<px4::params::EKF2_SEL_IMU_ACC>) _param_ekf2_sel_imu_accel,
		(ParamFloat<px4::params::EKF2_SEL_IMU_VEL>) _param_ekf2_sel_imu_velocity
	)
};
#endif // !EKF2SELECTOR_HPP
// --------------------------------------------------
// This file was autogenerated, do NOT modify by hand
// --------------------------------------------------

#ifndef EKF_STATE_H
#define EKF_STATE_H

#include <matrix/math.hpp>

namespace estimator
{
struct StateSample {
	matrix::Quaternion<float> quat_nominal{};
	matrix::Vector3<float> vel{};
	matrix::Vector3<float> pos{};
	matrix::Vector3<float> gyro_bias{};
	matrix::Vector3<float> accel_bias{};
	matrix::Vector3<float> mag_I{};
	matrix::Vector3<float> mag_B{};
	matrix::Vector2<float> wind_vel{};
	float terrain{};

	matrix::Vector<float, 25> Data() const {
		matrix::Vector<float, 25> state;
		state.slice<4, 1>(0, 0) = quat_nominal;
		state.slice<3, 1>(4, 0) = vel;
		state.slice<3, 1>(7, 0) = pos;
		state.slice<3, 1>(10, 0) = gyro_bias;
		state.slice<3, 1>(13, 0) = accel_bias;
		state.slice<3, 1>(16, 0) = mag_I;
		state.slice<3, 1>(19, 0) = mag_B;
		state.slice<2, 1>(22, 0) = wind_vel;
		state.slice<1, 1>(24, 0) = terrain;
		return state;
	};

	const matrix::Vector<float, 25>& vector() const {
		return *reinterpret_cast<matrix::Vector<float, 25>*>(const_cast<float*>(reinterpret_cast<const float*>(&quat_nominal)));
	};

};
static_assert(sizeof(matrix::Vector<float, 25>) == sizeof(StateSample), "state vector doesn't match StateSample size");

struct IdxDof { unsigned idx; unsigned dof; };
namespace State {
	static constexpr IdxDof quat_nominal{0, 3};
	static constexpr IdxDof vel{3, 3};
	static constexpr IdxDof pos{6, 3};
	static constexpr IdxDof gyro_bias{9, 3};
	static constexpr IdxDof accel_bias{12, 3};
	static constexpr IdxDof mag_I{15, 3};
	static constexpr IdxDof mag_B{18, 3};
	static constexpr IdxDof wind_vel{21, 2};
	static constexpr IdxDof terrain{23, 1};
	static constexpr uint8_t size{24};
};
}
#endif // !EKF_STATE_H
