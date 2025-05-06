#include "ActuatorEffectivenessRotors.hpp"

#include "ActuatorEffectivenessTilts.hpp"


using namespace matrix;

// 构造函数初始化了旋翼的有效性矩阵，包括位置、轴向、推力系数和力矩比等参数。
// 参数通过 param_find 动态查找，并存储在 _param_handles 中。
// 如果支持倾斜机构，则额外初始化倾斜索引。
ActuatorEffectivenessRotors::ActuatorEffectivenessRotors(ModuleParams *parent, AxisConfiguration axis_config,
		bool tilt_support)
	: ModuleParams(parent), _axis_config(axis_config), _tilt_support(tilt_support)

{
	for (int i = 0; i < NUM_ROTORS_MAX; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PX", i);
		_param_handles[i].position_x = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PY", i);
		_param_handles[i].position_y = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PZ", i);
		_param_handles[i].position_z = param_find(buffer);

		if (_axis_config == AxisConfiguration::Configurable) {
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AX", i);
			_param_handles[i].axis_x = param_find(buffer);
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AY", i);
			_param_handles[i].axis_y = param_find(buffer);
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AZ", i);
			_param_handles[i].axis_z = param_find(buffer);
		}

		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_CT", i);
		_param_handles[i].thrust_coef = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_KM", i);
		_param_handles[i].moment_ratio = param_find(buffer);

		if (_tilt_support) {
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_TILT", i);
			_param_handles[i].tilt_index = param_find(buffer);
		}
	}

	updateParams();
}

// 更新旋翼几何参数，包括位置、轴向、推力系数和力矩比。
// 根据 _axis_config 配置不同的轴向（可配置、固定向前或固定向上）。
// 如果支持倾斜机构，则更新倾斜索引。
void ActuatorEffectivenessRotors::updateParams()
{
	ModuleParams::updateParams();

	_geometry.num_rotors = math::min(NUM_ROTORS_MAX, static_cast<int>(_param_ca_rotor_count.get()));

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		Vector3f &position = _geometry.rotors[i].position;
		param_get(_param_handles[i].position_x, &position(0));
		param_get(_param_handles[i].position_y, &position(1));
		param_get(_param_handles[i].position_z, &position(2));

		Vector3f &axis = _geometry.rotors[i].axis;

		switch (_axis_config) {
		case AxisConfiguration::Configurable:
			param_get(_param_handles[i].axis_x, &axis(0));
			param_get(_param_handles[i].axis_y, &axis(1));
			param_get(_param_handles[i].axis_z, &axis(2));
			break;

		case AxisConfiguration::FixedForward:
			axis = Vector3f(1.f, 0.f, 0.f);
			break;

		case AxisConfiguration::FixedUpwards:
			axis = Vector3f(0.f, 0.f, -1.f);
			break;
		}

		param_get(_param_handles[i].thrust_coef, &_geometry.rotors[i].thrust_coef);
		param_get(_param_handles[i].moment_ratio, &_geometry.rotors[i].moment_ratio);

		if (_tilt_support) {
			int32_t tilt_param{0};
			param_get(_param_handles[i].tilt_index, &tilt_param);
			_geometry.rotors[i].tilt_index = tilt_param - 1;

		} else {
			_geometry.rotors[i].tilt_index = -1;
		}
	}
}

// 将旋翼添加到配置中。
// 调用 computeEffectivenessMatrix 计算有效性矩阵。
// 如果伺服执行器数量大于零，则返回错误，因为伺服执行器需要排在电机之后。
bool
ActuatorEffectivenessRotors::addActuators(Configuration &configuration)
{
	if (configuration.num_actuators[(int)ActuatorType::SERVOS] > 0) {
		PX4_ERR("Wrong actuator ordering: servos need to be after motors");
		return false;
	}

	int num_actuators = computeEffectivenessMatrix(_geometry,
			    configuration.effectiveness_matrices[configuration.selected_matrix],
			    configuration.num_actuators_matrix[configuration.selected_matrix]);
	configuration.actuatorsAdded(ActuatorType::MOTORS, num_actuators);
	return true;
}

/**
 * 计算旋翼的有效性矩阵。
 *
 * 该函数根据给定的几何信息，计算每个旋翼对整体推力和力矩的贡献，并填充到有效性矩阵中。
 * 它考虑了多种条件，例如是否禁用螺旋桨扭矩、是否通过差分推力控制偏航、以及是否禁用三维推力等。
 *
 * @param geometry 包含旋翼几何信息的对象，包括位置、轴向、推力系数和力矩比等参数。
 * @param effectiveness 要填充的有效性矩阵，表示每个执行器的效果。
 * @param actuator_start_index 执行器在有效性矩阵中的起始索引。
 * @return 返回处理的执行器数量。
 */
int
ActuatorEffectivenessRotors::computeEffectivenessMatrix(const Geometry &geometry,
		EffectivenessMatrix &effectiveness, int actuator_start_index)
{
	int num_actuators = 0;

	// 遍历所有旋翼
	for (int i = 0; i < geometry.num_rotors; i++) {

		// 检查当前索引是否超出总执行器数量
		if (i + actuator_start_index >= NUM_ACTUATORS) {
			break;
		}

		// 增加执行器计数
		++num_actuators;

		// 获取旋翼轴向
		Vector3f axis = geometry.rotors[i].axis;
		// std::printf("Axis %d: (%.2f, %.2f, %.2f)\n", i, static_cast<double>(axis(0)), static_cast<double>(axis(1)), static_cast<double>(axis(2)));

		// 归一化轴向
		float axis_norm = axis.norm();

		if (axis_norm > FLT_EPSILON) {
			axis /= axis_norm;

		} else {
			// 如果轴向定义无效，则忽略该旋翼
			continue;
		}

		// 获取旋翼位置
		const Vector3f &position = geometry.rotors[i].position;

		// 获取系数
		float ct = geometry.rotors[i].thrust_coef;
		float km = geometry.rotors[i].moment_ratio;

		// 如果禁用螺旋桨扭矩，则将力矩比设置为零
		if (geometry.propeller_torque_disabled) {
			km = 0.f;
		}

		// 如果仅禁用非向上方向的螺旋桨扭矩，则检查轴向是否向上
		if (geometry.propeller_torque_disabled_non_upwards) {
			bool upwards = fabsf(axis(0)) < 0.1f && fabsf(axis(1)) < 0.1f && axis(2) < -0.5f;

			if (!upwards) {
				km = 0.f;
			}
		}

		// 如果推力系数接近零，则跳过该旋翼
		if (fabsf(ct) < FLT_EPSILON) {
			continue;
		}

		// 计算该旋翼产生的推力
		matrix::Vector3f thrust = ct * axis;
		// std::printf("拉力方向 %d: (%.2f, %.2f, %.2f)\n", i, static_cast<double>(axis(0)), static_cast<double>(axis(1)), static_cast<double>(axis(2)));
		// std::printf("推力 %d: (%.2f, %.2f, %.2f)\n", i, static_cast<double>(thrust(0)), static_cast<double>(thrust(1)), static_cast<double>(thrust(2)));
		matrix::Vector3f moment = ct * position.cross(axis) - ct * km * axis;

		// 计算该旋翼产生的力矩
		if (isCyclocopter) {
			matrix::Vector3f cc_axis(0, 1, 0);
			if (i == 1 or i == 2){
				cc_axis = matrix::Vector3f(0, -1, 0);
			}
			// position.cross(axis)：位置向量与推力方向的叉乘，得到力臂力矩方向
			// matrix::Vector3f moment1 = ct * position.cross(axis);
			// matrix::Vector3f moment2 = - ct * km * cc_axis;
			moment = ct * position.cross(axis) - ct * km * cc_axis;
			// std::printf("推力力矩 %d: (%.2f, %.2f, %.2f)\n", i, static_cast<double>(moment1(0)), static_cast<double>(moment1(1)), static_cast<double>(moment1(2)));
			// std::printf("电机反扭矩 %d: (%.2f, %.2f, %.2f)\n", i, static_cast<double>(moment2(0)), static_cast<double>(moment2(1)), static_cast<double>(moment2(2)));
		}

		// 填充有效性矩阵中的对应项
		for (size_t j = 0; j < 3; j++) {
			effectiveness(j, i + actuator_start_index) = moment(j);
			effectiveness(j + 3, i + actuator_start_index) = thrust(j);
		}

		// 如果禁用通过差分推力控制偏航，则将偏航力矩设置为零
		if (geometry.yaw_by_differential_thrust_disabled) {
			effectiveness(2, i + actuator_start_index) = 0.f;
		}

		// 如果禁用三维推力，则仅保留 z 方向的推力
		if (geometry.three_dimensional_thrust_disabled) {
			effectiveness(0 + 3, i + actuator_start_index) = 0.f;
			effectiveness(1 + 3, i + actuator_start_index) = 0.f;
			effectiveness(2 + 3, i + actuator_start_index) = -ct;
		}
	}

	return num_actuators;
}

// 根据倾斜机构更新旋翼轴向。
// 如果旋翼没有关联的倾斜机构，则标记为非倾斜旋翼。
uint32_t ActuatorEffectivenessRotors::updateAxisFromTilts(const ActuatorEffectivenessTilts &tilts,
		float collective_tilt_control)
{
	if (!PX4_ISFINITE(collective_tilt_control)) {
		collective_tilt_control = -1.f;
	}

	uint32_t nontilted_motors = 0;

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		int tilt_index = _geometry.rotors[i].tilt_index;

		if (tilt_index == -1 || tilt_index >= tilts.count()) {
			nontilted_motors |= 1u << i;
			continue;
		}

		const ActuatorEffectivenessTilts::Params &tilt = tilts.config(tilt_index);
		const float tilt_angle = math::lerp(tilt.min_angle, tilt.max_angle, (collective_tilt_control + 1.f) / 2.f);
		const float tilt_direction = math::radians((float)tilt.tilt_direction);
		_geometry.rotors[i].axis = tiltedAxis(tilt_angle, tilt_direction);
	}

	return nontilted_motors;
}

Vector3f ActuatorEffectivenessRotors::tiltedAxis(float tilt_angle, float tilt_direction)
{
	Vector3f axis{0.f, 0.f, -1.f};
	return Dcmf{Eulerf{0.f, -tilt_angle, tilt_direction}} * axis;
}

uint32_t ActuatorEffectivenessRotors::getMotors() const
{
	uint32_t motors = 0;

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		motors |= 1u << i;
	}

	return motors;
}

uint32_t ActuatorEffectivenessRotors::getUpwardsMotors() const
{
	uint32_t upwards_motors = 0;

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		const Vector3f &axis = _geometry.rotors[i].axis;

		if (fabsf(axis(0)) < 0.1f && fabsf(axis(1)) < 0.1f && axis(2) < -0.5f) {
			upwards_motors |= 1u << i;
		}
	}

	return upwards_motors;
}

uint32_t ActuatorEffectivenessRotors::getForwardsMotors() const
{
	uint32_t forward_motors = 0;

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		const Vector3f &axis = _geometry.rotors[i].axis;

		if (axis(0) > 0.5f && fabsf(axis(1)) < 0.1f && fabsf(axis(2)) < 0.1f) {
			forward_motors |= 1u << i;
		}
	}

	return forward_motors;
}

bool
ActuatorEffectivenessRotors::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	return addActuators(configuration);
}

bool ActuatorEffectivenessRotors::isCyclocopter = false;
