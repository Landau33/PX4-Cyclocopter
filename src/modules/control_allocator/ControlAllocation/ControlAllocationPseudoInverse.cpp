#include "ControlAllocationPseudoInverse.hpp"

void
ControlAllocationPseudoInverse::setEffectivenessMatrix(
	const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> &effectiveness,
	const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
	bool update_normalization_scale)
{
	// 设置效果矩阵、执行器零位、线性化点和执行器数量
	ControlAllocation::setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, num_actuators,
			update_normalization_scale);
	// 标记需要更新伪逆矩阵
	_mix_update_needed = true;
	// 根据参数设置是否需要更新归一化矩阵
	_normalization_needs_update = update_normalization_scale;
}

void
ControlAllocationPseudoInverse::updatePseudoInverse()
{
	// 如果需要更新伪逆矩阵
	if (_mix_update_needed) {
		// 计算效果矩阵的伪逆矩阵
		matrix::geninv(_effectiveness, _mix);

		// 如果需要更新归一化矩阵且没有执行器故障
		if (_normalization_needs_update && !_had_actuator_failure) {
			// 更新控制分配矩阵的缩放因子
			updateControlAllocationMatrixScale();
			// 标记已经更新归一化矩阵
			_normalization_needs_update = false;
		}

		// 归一化控制分配矩阵
		normalizeControlAllocationMatrix();
		// 标记已经更新伪逆矩阵
		_mix_update_needed = false;
	}
}

void
ControlAllocationPseudoInverse::updateControlAllocationMatrixScale()
{
	// 滚转和俯仰缩放相同比例
	if (_normalize_rpy) {

		int num_non_zero_roll_torque = 0;
		int num_non_zero_pitch_torque = 0;

		// 统计对滚转和俯仰有贡献的执行器数量
		for (int i = 0; i < _num_actuators; i++) {

			if (fabsf(_mix(i, 0)) > 1e-3f) {
				++num_non_zero_roll_torque;
			}

			if (fabsf(_mix(i, 1)) > 1e-3f) {
				++num_non_zero_pitch_torque;
			}
		}

		// 计算滚转和俯仰的缩放
		float roll_norm_scale = 1.f;
		if (num_non_zero_roll_torque > 0) {
			roll_norm_scale = sqrtf(_mix.col(0).norm_squared() / (num_non_zero_roll_torque / 2.f));
		}

		float pitch_norm_scale = 1.f;
		if (num_non_zero_pitch_torque > 0) {
			pitch_norm_scale = sqrtf(_mix.col(1).norm_squared() / (num_non_zero_pitch_torque / 2.f));
		}

		// 设置滚转和俯仰的缩放为两者中的较大值
		_control_allocation_scale(0) = fmaxf(roll_norm_scale, pitch_norm_scale);
		_control_allocation_scale(1) = _control_allocation_scale(0);

		// 单独计算偏航的缩放
		_control_allocation_scale(2) = _mix.col(2).max();

	} else {
		_control_allocation_scale(0) = 1.f;
		_control_allocation_scale(1) = 1.f;
		_control_allocation_scale(2) = 1.f;
	}

	// 按单个推力轴的总和缩放推力，如果没有执行器，则使用Z轴的缩放
	_control_allocation_scale(THRUST_Z) = 1.f;

	for (int axis_idx = 2; axis_idx >= 0; --axis_idx) {
		int num_non_zero_thrust = 0;
		float norm_sum = 0.f;

		// 统计对推力有贡献的执行器数量并计算其范数之和
		for (int i = 0; i < _num_actuators; i++) {
			float norm = fabsf(_mix(i, 3 + axis_idx));
			norm_sum += norm;

			if (norm > FLT_EPSILON) {
				++num_non_zero_thrust;
			}
		}
		// 计算推力的缩放因子
		if (num_non_zero_thrust > 0) {
			_control_allocation_scale(3 + axis_idx) = norm_sum / num_non_zero_thrust;

		} else {
			_control_allocation_scale(3 + axis_idx) = _control_allocation_scale(THRUST_Z);
		}
	}
}

void
ControlAllocationPseudoInverse::normalizeControlAllocationMatrix()
{
	// 根据缩放因子归一化控制分配矩阵
	if (_control_allocation_scale(0) > FLT_EPSILON) {
		_mix.col(0) /= _control_allocation_scale(0);
		_mix.col(1) /= _control_allocation_scale(1);
	}

	if (_control_allocation_scale(2) > FLT_EPSILON) {
		_mix.col(2) /= _control_allocation_scale(2);
	}

	if (_control_allocation_scale(3) > FLT_EPSILON) {
		_mix.col(3) /= _control_allocation_scale(3);
		_mix.col(4) /= _control_allocation_scale(4);
		_mix.col(5) /= _control_allocation_scale(5);
	}

	// 将控制分配矩阵中小于阈值的元素设为0
	for (int i = 0; i < _num_actuators; i++) {
		for (int j = 0; j < NUM_AXES; j++) {
			if (fabsf(_mix(i, j)) < 1e-3f) {
				_mix(i, j) = 0.f;
			}
		}
	}
}

void
ControlAllocationPseudoInverse::allocate()
{
	//Compute new gains if needed
	updatePseudoInverse();

	_prev_actuator_sp = _actuator_sp;

	// Allocate
	// _actuator_trim：执行器的零位点，即执行器在没有控制输入时的默认值。
	// _control_sp：控制目标，即期望的控制量（例如期望的姿态或力矩）。
	// _control_trim：控制零位点，即控制量在没有控制输入时的默认值。
	// _mix：控制分配矩阵，用于将控制目标转换为执行器命令。
	// _actuator_sp：计算得到的新执行器设置点。
	_actuator_sp = _actuator_trim + _mix * (_control_sp - _control_trim);
}
