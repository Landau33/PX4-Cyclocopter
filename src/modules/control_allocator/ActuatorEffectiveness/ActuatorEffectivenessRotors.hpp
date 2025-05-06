#pragma once

#include "ActuatorEffectiveness.hpp"

#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>

class ActuatorEffectivenessTilts;

using namespace time_literals;

class ActuatorEffectivenessRotors : public ModuleParams, public ActuatorEffectiveness
{
public:
	enum class AxisConfiguration {
		Configurable, ///< 轴可以配置
		FixedForward, ///< 轴固定，指向前方（正X方向）
		FixedUpwards, ///< 轴固定，指向上方（负Z方向）
	};

	static constexpr int NUM_ROTORS_MAX = 12; // 最大旋翼数量

	struct RotorGeometry { // 旋翼几何结构
		matrix::Vector3f position; // 旋翼位置
		matrix::Vector3f axis; // 旋翼轴向
		float thrust_coef; // 推力系数
		float moment_ratio; // 力矩比
		int tilt_index; // 倾斜索引
	};

	struct Geometry { // 几何结构
		RotorGeometry rotors[NUM_ROTORS_MAX]; // 所有旋翼的几何信息
		int num_rotors{0}; // 旋翼数量
		bool propeller_torque_disabled{false}; // 是否禁用螺旋桨扭矩
		bool yaw_by_differential_thrust_disabled{false}; // 是否禁用差分推力偏航
		bool propeller_torque_disabled_non_upwards{false}; ///< 对于向上指向的电机，保持螺旋桨扭矩启用
		bool three_dimensional_thrust_disabled{false}; ///< 用于处理倾转旋翼 VTOL，因为它们传递一维推力和集体倾斜
	};

	// 构造函数
	ActuatorEffectivenessRotors(ModuleParams *parent, AxisConfiguration axis_config = AxisConfiguration::Configurable,
				    bool tilt_support = false);
	virtual ~ActuatorEffectivenessRotors() = default;

	// 获取效应矩阵
	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	// 获取期望的分配方法
	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
	}

	// 获取是否归一化横滚、俯仰和偏航
	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;
	}

	// 计算效应矩阵
	static int computeEffectivenessMatrix(const Geometry &geometry,
					      EffectivenessMatrix &effectiveness, int actuator_start_index = 0);

	// 添加执行器
	bool addActuators(Configuration &configuration);
	static bool isCyclocopter;

	// 获取名称
	const char *name() const override { return "Rotors"; }

	/**
	 * 根据倾斜配置和当前倾斜控制设置电机轴。
	 * @param tilts 配置的倾斜伺服
	 * @param tilt_control 当前倾斜控制值（范围为 [-1, 1]，可以是 NAN）
	 * @return 返回不可倾斜电机的位集
	 */
	uint32_t updateAxisFromTilts(const ActuatorEffectivenessTilts &tilts, float tilt_control);

	// 获取几何结构
	const Geometry &geometry() const { return _geometry; }

	/**
	 * 获取倾斜后的轴 {0, 0, -1}，首先绕 Y 轴旋转 -tilt_angle，然后绕 Z 轴旋转 tilt_direction。
	 */
	static matrix::Vector3f tiltedAxis(float tilt_angle, float tilt_direction);

	// 启用螺旋桨扭矩
	void enablePropellerTorque(bool enable) { _geometry.propeller_torque_disabled = !enable; }

	// 启用差分推力偏航
	void enableYawByDifferentialThrust(bool enable) { _geometry.yaw_by_differential_thrust_disabled = !enable; }

	// 启用非向上电机的螺旋桨扭矩
	void enablePropellerTorqueNonUpwards(bool enable) { _geometry.propeller_torque_disabled_non_upwards = !enable; }

	// 启用三维推力
	void enableThreeDimensionalThrust(bool enable) { _geometry.three_dimensional_thrust_disabled = !enable; }

	// 获取所有电机的位掩码
	uint32_t getMotors() const;

	// 获取向上电机的位掩码
	uint32_t getUpwardsMotors() const;

	// 获取向前电机的位掩码
	uint32_t getForwardsMotors() const;

private:
	// 更新参数
	void updateParams() override;

	const AxisConfiguration _axis_config; // 轴配置
	const bool _tilt_support; ///< 如果为 true，则加载倾斜伺服分配参数

	struct ParamHandles { // 参数句柄
		param_t position_x;
		param_t position_y;
		param_t position_z;
		param_t axis_x;
		param_t axis_y;
		param_t axis_z;
		param_t thrust_coef;
		param_t moment_ratio;
		param_t tilt_index;
	};
	ParamHandles _param_handles[NUM_ROTORS_MAX]; // 每个旋翼的参数句柄

	Geometry _geometry{}; // 几何结构

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CA_ROTOR_COUNT>) _param_ca_rotor_count // 旋翼数量参数
	)
};
