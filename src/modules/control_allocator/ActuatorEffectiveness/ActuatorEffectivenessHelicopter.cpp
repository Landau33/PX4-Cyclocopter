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

#include "ActuatorEffectivenessHelicopter.hpp"
#include <lib/mathlib/mathlib.h>

using namespace matrix;
using namespace time_literals;

ActuatorEffectivenessHelicopter::ActuatorEffectivenessHelicopter(ModuleParams *parent, ActuatorType tail_actuator_type)
	: ModuleParams(parent), _tail_actuator_type(tail_actuator_type)
{
	for (int i = 0; i < NUM_SWASH_PLATE_SERVOS_MAX; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SP0_ANG%u", i);
		_param_handles.swash_plate_servos[i].angle = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SP0_ARM_L%u", i);
		_param_handles.swash_plate_servos[i].arm_length = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TRIM", i);
		_param_handles.swash_plate_servos[i].trim = param_find(buffer);
	}

	_param_handles.num_swash_plate_servos = param_find("CA_SP0_COUNT");

	for (int i = 0; i < NUM_CURVE_POINTS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_HELI_THR_C%u", i);
		_param_handles.throttle_curve[i] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_HELI_PITCH_C%u", i);
		_param_handles.pitch_curve[i] = param_find(buffer);
	}

	_param_handles.yaw_collective_pitch_scale = param_find("CA_HELI_YAW_CP_S");
	_param_handles.yaw_collective_pitch_offset = param_find("CA_HELI_YAW_CP_O");
	_param_handles.yaw_throttle_scale = param_find("CA_HELI_YAW_TH_S");
	_param_handles.yaw_ccw = param_find("CA_HELI_YAW_CCW");
	_param_handles.spoolup_time = param_find("COM_SPOOLUP_TIME");

	updateParams();
}

void ActuatorEffectivenessHelicopter::updateParams()
{
	ModuleParams::updateParams();

	int32_t count = 0;

	if (param_get(_param_handles.num_swash_plate_servos, &count) != 0) {
		PX4_ERR("param_get failed");
		return;
	}

	_geometry.num_swash_plate_servos = math::constrain((int)count, 3, NUM_SWASH_PLATE_SERVOS_MAX);

	for (int i = 0; i < _geometry.num_swash_plate_servos; ++i) {
		float angle_deg{};
		param_get(_param_handles.swash_plate_servos[i].angle, &angle_deg);
		_geometry.swash_plate_servos[i].angle = math::radians(angle_deg);
		param_get(_param_handles.swash_plate_servos[i].arm_length, &_geometry.swash_plate_servos[i].arm_length);
		param_get(_param_handles.swash_plate_servos[i].trim, &_geometry.swash_plate_servos[i].trim);
	}

	for (int i = 0; i < NUM_CURVE_POINTS; ++i) {
		param_get(_param_handles.throttle_curve[i], &_geometry.throttle_curve[i]);
		param_get(_param_handles.pitch_curve[i], &_geometry.pitch_curve[i]);
	}

	param_get(_param_handles.yaw_collective_pitch_scale, &_geometry.yaw_collective_pitch_scale);
	param_get(_param_handles.yaw_collective_pitch_offset, &_geometry.yaw_collective_pitch_offset);
	param_get(_param_handles.yaw_throttle_scale, &_geometry.yaw_throttle_scale);
	param_get(_param_handles.spoolup_time, &_geometry.spoolup_time);
	int32_t yaw_ccw = 0;
	param_get(_param_handles.yaw_ccw, &yaw_ccw);
	_geometry.yaw_sign = (yaw_ccw == 1) ? -1.f : 1.f;
}

bool ActuatorEffectivenessHelicopter::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// As the allocation is non-linear, we use updateSetpoint() instead of the matrix
	configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{});

	// Tail (yaw) (either ESC or Servo)
	configuration.addActuator(_tail_actuator_type, Vector3f{}, Vector3f{});

	// N swash plate servos
	_first_swash_plate_servo_index = configuration.num_actuators_matrix[0];

	for (int i = 0; i < _geometry.num_swash_plate_servos; ++i) {
		configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
		configuration.trim[configuration.selected_matrix](i) = _geometry.swash_plate_servos[i].trim;
	}

	return true;
}

// 更新直升机执行器效应的有效性
// 该函数根据控制设定点更新执行器设定点，并进行饱和检查
// 参数:
// - control_sp: 控制设定点向量，包含各个控制轴的设定值
// - matrix_index: 执行器矩阵索引，用于选择特定的执行器配置
// - actuator_sp: 执行器设定点向量，将被更新以反映新的控制设定点
// - actuator_min: 执行器最小限制向量，用于饱和检查
// - actuator_max: 执行器最大限制向量，用于饱和检查
void ActuatorEffectivenessHelicopter::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	// 初始化饱和标志
	_saturation_flags = {};

	// throttle/collective pitch curve
	// 计算油门和总距
	const float throttle = math::interpolateN(-control_sp(ControlAxis::THRUST_Z),
			       _geometry.throttle_curve) * throttleSpoolupProgress();
	const float collective_pitch = math::interpolateN(-control_sp(ControlAxis::THRUST_Z), _geometry.pitch_curve);

	// actuator mapping
	// 根据主电机是否启动来设置油门执行器设定点
	actuator_sp(0) = mainMotorEnaged() ? throttle : NAN;

	// 计算航向执行器设定点，并进行饱和检查
	actuator_sp(1) = control_sp(ControlAxis::YAW) * _geometry.yaw_sign
			 + fabsf(collective_pitch - _geometry.yaw_collective_pitch_offset) * _geometry.yaw_collective_pitch_scale
			 + throttle * _geometry.yaw_throttle_scale;

	// Saturation check for yaw
	if (actuator_sp(1) < actuator_min(1)) {
		setSaturationFlag(_geometry.yaw_sign, _saturation_flags.yaw_neg, _saturation_flags.yaw_pos);

	} else if (actuator_sp(1) > actuator_max(1)) {
		setSaturationFlag(_geometry.yaw_sign, _saturation_flags.yaw_pos, _saturation_flags.yaw_neg);
	}

	// 计算变距伺服执行器设定点，并进行饱和检查
	for (int i = 0; i < _geometry.num_swash_plate_servos; i++) {
		float roll_coeff = sinf(_geometry.swash_plate_servos[i].angle) * _geometry.swash_plate_servos[i].arm_length;
		float pitch_coeff = cosf(_geometry.swash_plate_servos[i].angle) * _geometry.swash_plate_servos[i].arm_length;
		actuator_sp(_first_swash_plate_servo_index + i) = collective_pitch
				+ control_sp(ControlAxis::PITCH) * pitch_coeff
				- control_sp(ControlAxis::ROLL) * roll_coeff
				+ _geometry.swash_plate_servos[i].trim;

		// Saturation check for roll & pitch饱和
		if (actuator_sp(_first_swash_plate_servo_index + i) < actuator_min(_first_swash_plate_servo_index + i)) {
			setSaturationFlag(roll_coeff, _saturation_flags.roll_pos, _saturation_flags.roll_neg);
			setSaturationFlag(pitch_coeff, _saturation_flags.pitch_neg, _saturation_flags.pitch_pos);

		} else if (actuator_sp(_first_swash_plate_servo_index + i) > actuator_max(_first_swash_plate_servo_index + i)) {
			setSaturationFlag(roll_coeff, _saturation_flags.roll_neg, _saturation_flags.roll_pos);
			setSaturationFlag(pitch_coeff, _saturation_flags.pitch_pos, _saturation_flags.pitch_neg);
		}
	}
}

bool ActuatorEffectivenessHelicopter::mainMotorEnaged()
{
	manual_control_switches_s manual_control_switches;

	if (_manual_control_switches_sub.update(&manual_control_switches)) {
		_main_motor_engaged = manual_control_switches.engage_main_motor_switch == manual_control_switches_s::SWITCH_POS_NONE
				      || manual_control_switches.engage_main_motor_switch == manual_control_switches_s::SWITCH_POS_ON;
	}

	return _main_motor_engaged;
}

/**
 * @brief 计算直升机油门杆上升进度
 *
 * 该函数根据直升机的武装状态和自上次武装以来的时间，计算油门杆上升的进度。
 * 它用于确定直升机在起飞过程中发动机的加速进度。
 *
 * @return float 表示油门杆上升进度的浮点数，范围从0到1，1表示完全加速。
 */
float ActuatorEffectivenessHelicopter::throttleSpoolupProgress()
{
	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
		_armed_time = vehicle_status.armed_time;
	}

	const float time_since_arming = (hrt_absolute_time() - _armed_time) / 1e6f;
	const float spoolup_progress = time_since_arming / _geometry.spoolup_time;

	if (_armed && spoolup_progress < 1.f) {
		return spoolup_progress;
	}

	return 1.f;
}

void ActuatorEffectivenessHelicopter::setSaturationFlag(float coeff, bool &positive_flag, bool &negative_flag)
{
	if (coeff > 0.f) {
		// A positive change in given axis will increase saturation
		positive_flag = true;

	} else if (coeff < 0.f) {
		// A negative change in given axis will increase saturation
		negative_flag = true;
	}
}

/**
 * @brief 获取未分配的控制力矩和推力
 *
 * 根据饱和标志设置未分配的控制力矩和推力。该函数旨在向速率控制器提供反馈，
 * 指示哪些控制轴处于饱和状态。饱和状态可以是正饱和（表示过量的正控制输出），
 * 负饱和（表示过量的负控制输出）或无饱和。
 *
 * @param matrix_index 矩阵索引，用于选择不同的控制分配矩阵（未使用）
 * @param status 控制分配状态引用，用于存储未分配的控制力矩和推力
 */
void ActuatorEffectivenessHelicopter::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	// Note: the values '-1', '1' and '0' are just to indicate a negative,
	// positive or no saturation to the rate controller. The actual magnitude is not used.
	if (_saturation_flags.roll_pos) {
		status.unallocated_torque[0] = 1.f;

	} else if (_saturation_flags.roll_neg) {
		status.unallocated_torque[0] = -1.f;

	} else {
		status.unallocated_torque[0] = 0.f;
	}

	if (_saturation_flags.pitch_pos) {
		status.unallocated_torque[1] = 1.f;

	} else if (_saturation_flags.pitch_neg) {
		status.unallocated_torque[1] = -1.f;

	} else {
		status.unallocated_torque[1] = 0.f;
	}

	if (_saturation_flags.yaw_pos) {
		status.unallocated_torque[2] = 1.f;

	} else if (_saturation_flags.yaw_neg) {
		status.unallocated_torque[2] = -1.f;

	} else {
		status.unallocated_torque[2] = 0.f;
	}

	if (_saturation_flags.thrust_pos) {
		status.unallocated_thrust[2] = 1.f;

	} else if (_saturation_flags.thrust_neg) {
		status.unallocated_thrust[2] = -1.f;

	} else {
		status.unallocated_thrust[2] = 0.f;
	}
}
