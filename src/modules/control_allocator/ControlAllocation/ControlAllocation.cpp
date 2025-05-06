#include "ControlAllocation.hpp"

ControlAllocation::ControlAllocation()
{
	// 初始化控制分配缩放因子为1
	_control_allocation_scale.setAll(1.f);
	// 初始化执行器最小值为0
	_actuator_min.setAll(0.f);
	// 初始化执行器最大值为1
	_actuator_max.setAll(1.f);
}

void
ControlAllocation::setEffectivenessMatrix(
	const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> &effectiveness,
	const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
	bool update_normalization_scale)
{
	// 设置效果矩阵
	_effectiveness = effectiveness;
	// 复制线性化点并进行裁剪
	ActuatorVector linearization_point_clipped = linearization_point;
	clipActuatorSetpoint(linearization_point_clipped);
	// 设置执行器零位点
	_actuator_trim = actuator_trim + linearization_point_clipped;
	clipActuatorSetpoint(_actuator_trim);
	// 设置执行器数量
	_num_actuators = num_actuators;
	// 计算控制零位点
	_control_trim = _effectiveness * linearization_point_clipped;
}

void
ControlAllocation::setActuatorSetpoint(
	const matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> &actuator_sp)
{
	// 设置执行器设置点
	_actuator_sp = actuator_sp;

	// Clip
	clipActuatorSetpoint(_actuator_sp);
}

void
ControlAllocation::clipActuatorSetpoint(matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> &actuator) const
{
	// 对每个执行器设置点进行裁剪，确保其在最小值和最大值之间
	for (int i = 0; i < _num_actuators; i++) {
		if (_actuator_max(i) < _actuator_min(i)) {
			// 如果最大值小于最小值，设置为执行器零位点
			actuator(i) = _actuator_trim(i);

		} else if (actuator(i) < _actuator_min(i)) {
			// 如果设置点小于最小值，设置为最小值
			actuator(i) = _actuator_min(i);

		} else if (actuator(i) > _actuator_max(i)) {
			// 如果设置点大于最大值，设置为最大值
			actuator(i) = _actuator_max(i);
		}
	}
}

matrix::Vector<float, ControlAllocation::NUM_ACTUATORS>
ControlAllocation::normalizeActuatorSetpoint(const matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> &actuator)
const
{
	// 归一化执行器设置点，使其在0到1之间
	matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> actuator_normalized;

	for (int i = 0; i < _num_actuators; i++) {
		// 线性归一化
		if (_actuator_min(i) < _actuator_max(i)) {
			actuator_normalized(i) = (actuator(i) - _actuator_min(i)) / (_actuator_max(i) - _actuator_min(i));

		} else {
			actuator_normalized(i) = (_actuator_trim(i) - _actuator_min(i)) / (_actuator_max(i) - _actuator_min(i));
		}
	}

	return actuator_normalized;
}

void ControlAllocation::applySlewRateLimit(float dt)
{
	// 应用执行器转速限制
	for (int i = 0; i < _num_actuators; i++) {
		if (_actuator_slew_rate_limit(i) > FLT_EPSILON) {
			// 计算最大允许的变化量
			float delta_sp_max = dt * (_actuator_max(i) - _actuator_min(i)) / _actuator_slew_rate_limit(i);
			// 计算当前设置点与前一个设置点的变化量
			float delta_sp = _actuator_sp(i) - _prev_actuator_sp(i);

			if (delta_sp > delta_sp_max) {
				// 如果变化量超过最大允许值，限制为最大允许值
				_actuator_sp(i) = _prev_actuator_sp(i) + delta_sp_max;

			} else if (delta_sp < -delta_sp_max) {
				// 如果变化量小于最小允许值，限制为最小允许值
				_actuator_sp(i) = _prev_actuator_sp(i) - delta_sp_max;
			}
		}
	}
}
