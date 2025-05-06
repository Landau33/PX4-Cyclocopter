#include "ActuatorEffectivenessCyclocopter.hpp"

using namespace matrix;

ActuatorEffectivenessCyclocopter::ActuatorEffectivenessCyclocopter(ModuleParams *parent)
	: ModuleParams(parent),
	  _cc_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards, true),
	  _cc_tilts(this)
{
	ActuatorEffectivenessRotors::isCyclocopter = true;
}

bool
ActuatorEffectivenessCyclocopter::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// 多旋翼电机
	_cc_rotors.enableYawByDifferentialThrust(!_cc_tilts.hasYawControl()); // 如果倾斜机构没有偏航控制，则启用差分推力偏航
	const bool rotors_added_successfully = _cc_rotors.addActuators(configuration); // 添加电机效应器

	// 倾斜机构
	_first_tilt_idx = configuration.num_actuators_matrix[0]; // 获取第一个倾斜机构的索引
	_cc_tilts.updateTorqueSign(_cc_rotors.geometry()); // 更新倾斜机构的扭矩符号
	const bool tilts_added_successfully = _cc_tilts.addActuators(configuration); // 添加倾斜机构效应器

	// 设置偏移量，使控制输入为0时倾斜机构指向上方（如果min_angle == -max_angle，则trim为0）。
	// 注意：我们不在此处设置configuration.trim，因为在trim == +-1的情况下，偏航总是饱和并被顺序去饱和方法降低到0。因此我们在之后添加它。
	_tilt_offsets.setZero(); // 初始化倾斜偏移量为零

	for (int i = 0; i < _cc_tilts.count(); ++i) { // 遍历所有倾斜机构
		float delta_angle = _cc_tilts.config(i).max_angle - _cc_tilts.config(i).min_angle; // 计算角度范围

		if (delta_angle > FLT_EPSILON) { // 如果角度范围有效
			float trim = -1.f - 2.f * _cc_tilts.config(i).min_angle / delta_angle; // 计算trim值
			_tilt_offsets(_first_tilt_idx + i) = trim; // 设置对应的倾斜偏移量
		}
	}

	return (rotors_added_successfully && tilts_added_successfully); // 返回是否成功添加电机和倾斜机构
}

void ActuatorEffectivenessCyclocopter::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	actuator_sp += _tilt_offsets; // 将倾斜偏移量添加到效应器设定点中
	// TODO: 动态矩阵更新

	bool yaw_saturated_positive = true; // 标记偏航正向是否饱和
	bool yaw_saturated_negative = true; // 标记偏航负向是否饱和

	for (int i = 0; i < _cc_tilts.count(); ++i) { // 遍历所有倾斜机构
		// 自定义偏航饱和逻辑：只有当所有倾斜机构都达到负向或正向偏航极限时，才声明偏航饱和

		if (_cc_tilts.getYawTorqueOfTilt(i) > FLT_EPSILON) { // 如果当前倾斜机构产生正向偏航扭矩
			if (yaw_saturated_positive && actuator_sp(i + _first_tilt_idx) < actuator_max(i + _first_tilt_idx) - FLT_EPSILON) {
				yaw_saturated_positive = false; // 如果未达到最大值，则标记正向未饱和
			}

			if (yaw_saturated_negative && actuator_sp(i + _first_tilt_idx) > actuator_min(i + _first_tilt_idx) + FLT_EPSILON) {
				yaw_saturated_negative = false; // 如果超过最小值，则标记负向未饱和
			}

		} else if (_cc_tilts.getYawTorqueOfTilt(i) < -FLT_EPSILON) { // 如果当前倾斜机构产生负向偏航扭矩
			if (yaw_saturated_negative && actuator_sp(i + _first_tilt_idx) < actuator_max(i + _first_tilt_idx) - FLT_EPSILON) {
				yaw_saturated_negative = false; // 如果未达到最大值，则标记负向未饱和
			}

			if (yaw_saturated_positive && actuator_sp(i + _first_tilt_idx) > actuator_min(i + _first_tilt_idx) + FLT_EPSILON) {
				yaw_saturated_positive = false; // 如果超过最小值，则标记正向未饱和
			}
		}
	}

	_yaw_tilt_saturation_flags.tilt_yaw_neg = yaw_saturated_negative; // 设置负向偏航饱和标志
	_yaw_tilt_saturation_flags.tilt_yaw_pos = yaw_saturated_positive; // 设置正向偏航饱和标志
}

void ActuatorEffectivenessCyclocopter::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	// 注意：值'-1'、'1'和'0'仅用于向速率控制器指示负向、正向或无饱和。实际幅度未使用。
	if (_yaw_tilt_saturation_flags.tilt_yaw_pos) {
		status.unallocated_torque[2] = 1.f; // 正向偏航饱和

	} else if (_yaw_tilt_saturation_flags.tilt_yaw_neg) {
		status.unallocated_torque[2] = -1.f; // 负向偏航饱和

	} else {
		status.unallocated_torque[2] = 0.f; // 无偏航饱和
	}
}
