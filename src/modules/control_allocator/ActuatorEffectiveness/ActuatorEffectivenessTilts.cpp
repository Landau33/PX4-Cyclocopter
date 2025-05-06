#include "ActuatorEffectivenessTilts.hpp"

#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>

using namespace matrix;

// 构造函数，初始化倾斜机构的有效性矩阵。
// 初始化过程中，通过参数查找动态获取每个倾斜机构的控制、最小角度、最大角度和倾斜方向等参数。
ActuatorEffectivenessTilts::ActuatorEffectivenessTilts(ModuleParams *parent)
	: ModuleParams(parent)
{
	for (int i = 0; i < MAX_COUNT; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_CT", i);
		_param_handles[i].control = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_MINA", i);
		_param_handles[i].min_angle = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_MAXA", i);
		_param_handles[i].max_angle = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_TD", i);
		_param_handles[i].tilt_direction = param_find(buffer);
	}

	_count_handle = param_find("CA_SV_TL_COUNT");
	updateParams();
}

// 更新倾斜机构的参数。
// 从参数中读取倾斜机构的数量，并为每个倾斜机构读取其控制类型、倾斜方向、最小和最大角度。
// 将角度从度数转换为弧度，并初始化扭矩向量。
void ActuatorEffectivenessTilts::updateParams()
{
	ModuleParams::updateParams();

	int32_t count = 0;

	if (param_get(_count_handle, &count) != 0) {
		PX4_ERR("param_get failed");
		return;
	}

	_count = count;

	for (int i = 0; i < count; i++) {
		param_get(_param_handles[i].control, (int32_t *)&_params[i].control);
		param_get(_param_handles[i].tilt_direction, (int32_t *)&_params[i].tilt_direction);
		param_get(_param_handles[i].min_angle, &_params[i].min_angle);
		param_get(_param_handles[i].max_angle, &_params[i].max_angle);

		// 角度转换为弧度
		_params[i].min_angle = math::radians(_params[i].min_angle);
		_params[i].max_angle = math::radians(_params[i].max_angle);
		// 初始化扭矩向量
		_torque[i].setZero();
	}
}

/**
 * 将倾斜机构添加到配置中。
 *
 * 对于每个倾斜机构，调用 addActuator 方法将其添加到配置中。
 *
 * @param configuration 配置对象，用于存储执行器信息。
 * @return 返回 true 表示成功添加所有执行器。
 */
bool ActuatorEffectivenessTilts::addActuators(Configuration &configuration)
{
	for (int i = 0; i < _count; i++) {
		configuration.addActuator(ActuatorType::SERVOS, _torque[i], Vector3f{});
	}

	return true;
}

/**
 * 更新倾斜机构的扭矩符号。
 *
 * 根据旋翼的位置和倾斜方向计算偏航扭矩符号，并根据倾斜方向更新俯仰扭矩符号。
 *
 * @param geometry 旋翼几何信息。
 * @param disable_pitch 是否禁用俯仰控制。
 */
void ActuatorEffectivenessTilts::updateTorqueSign(const ActuatorEffectivenessRotors::Geometry &geometry,
		bool disable_pitch)
{
	// 遍历所有电机
	for (int i = 0; i < geometry.num_rotors; ++i) {
		int tilt_index = geometry.rotors[i].tilt_index;

		// 检查舵机索引是否有效；如果无效，跳过当前旋翼。
		if (tilt_index == -1 || tilt_index >= _count) {
			continue;
		}


		if (_params[tilt_index].control == Control::Yaw || _params[tilt_index].control == Control::YawAndPitch) {

			// 通过检查电机位置和倾斜方向来确定偏航扭矩符号。
            		// 将位置绕 z 轴旋转 -tilt_direction，然后检查 y 坐标的符号。
			// Find the yaw torque sign by checking the motor position and tilt direction.
			// Rotate position by -tilt_direction around z, then check the sign of y pos
			float tilt_direction = math::radians((float)_params[tilt_index].tilt_direction);
			Vector3f rotated_pos = Dcmf{Eulerf{0.f, 0.f, -tilt_direction}} * geometry.rotors[i].position;
			// 根据旋转后的位置设置偏航扭矩符号。
			if (rotated_pos(1) < -0.01f) { // 添加最小边界
				_torque[tilt_index](2) = 1.f;

			} else if (rotated_pos(1) > 0.01f) {
				_torque[tilt_index](2) = -1.f;
			}
		}

		// 如果未禁用俯仰控制，确定参与俯仰控制的旋翼的俯仰扭矩符号。
		if (!disable_pitch && (_params[tilt_index].control == Control::Pitch
				       || _params[tilt_index].control == Control::YawAndPitch)) {
			// 根据倾斜方向判断旋翼是否向前倾斜。
			bool tilting_forwards = (int)_params[tilt_index].tilt_direction < 90 || (int)_params[tilt_index].tilt_direction > 270;
			// 根据倾斜方向设置俯仰扭矩符号。
			_torque[tilt_index](1) = tilting_forwards ? -1.f : 1.f;
		}

	}
}

bool ActuatorEffectivenessTilts::hasYawControl() const
{
	for (int i = 0; i < _count; i++) {
		if (_params[i].control == Control::Yaw || _params[i].control == Control::YawAndPitch) {
			return true;
		}
	}

	return false;
}
