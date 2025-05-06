#pragma once

#include "ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"

#include <px4_platform_common/module_params.h>

class ActuatorEffectivenessTilts : public ModuleParams, public ActuatorEffectiveness
{
public:

	static constexpr int MAX_COUNT = 4; // 最大数量为4

	enum class Control : int32_t {
		// 此枚举值与参数匹配
		None = 0,       // 无控制
		Yaw = 1,        // 偏航控制
		Pitch = 2,      // 俯仰控制
		YawAndPitch = 3 // 偏航和俯仰控制
	};

	enum class TiltDirection : int32_t {
		// 此枚举值与参数匹配
		TowardsFront = 0, // 倾斜方向：向前
		TowardsRight = 90 // 倾斜方向：向右
	};

	struct Params {
		Control control;         // 控制类型
		float min_angle;         // 最小角度
		float max_angle;         // 最大角度
		TiltDirection tilt_direction; // 倾斜方向
	};

	ActuatorEffectivenessTilts(ModuleParams *parent); // 构造函数
	virtual ~ActuatorEffectivenessTilts() = default;  // 默认析构函数

	bool addActuators(Configuration &configuration); // 添加执行器配置

	const char *name() const override { return "Tilts"; } // 返回名称"Tilts"

	int count() const { return _count; } // 返回倾斜执行器的数量

	const Params &config(int idx) const { return _params[idx]; } // 获取指定索引的配置参数

	void updateTorqueSign(const ActuatorEffectivenessRotors::Geometry &geometry, bool disable_pitch = false);
	// 更新扭矩符号，可选择禁用俯仰控制

	bool hasYawControl() const; // 判断是否有偏航控制

	float getYawTorqueOfTilt(int tilt_index) const { return _torque[tilt_index](2); }
	// 获取指定倾斜执行器的偏航扭矩（Z轴分量）

private:
	void updateParams() override; // 更新参数

	struct ParamHandles {
		param_t control;         // 控制参数句柄
		param_t min_angle;       // 最小角度参数句柄
		param_t max_angle;       // 最大角度参数句柄
		param_t tilt_direction;  // 倾斜方向参数句柄
	};

	ParamHandles _param_handles[MAX_COUNT]; // 参数句柄数组
	param_t _count_handle;                  // 数量参数句柄

	Params _params[MAX_COUNT] {}; // 配置参数数组
	int _count{0};                // 当前倾斜执行器数量

	matrix::Vector3f _torque[MAX_COUNT] {}; // 每个倾斜执行器的扭矩向量
};
