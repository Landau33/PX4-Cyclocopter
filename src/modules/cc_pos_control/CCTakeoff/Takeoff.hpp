#pragma once

#include <lib/hysteresis/hysteresis.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/takeoff_status.h>

using namespace time_literals;

enum class TakeoffState {
	disarmed = takeoff_status_s::TAKEOFF_STATE_DISARMED, // 未武装状态
	spoolup = takeoff_status_s::TAKEOFF_STATE_SPOOLUP, // 预热状态
	ready_for_takeoff = takeoff_status_s::TAKEOFF_STATE_READY_FOR_TAKEOFF, // 准备起飞状态
	rampup = takeoff_status_s::TAKEOFF_STATE_RAMPUP, // 爬升斜坡状态
	flight = takeoff_status_s::TAKEOFF_STATE_FLIGHT // 飞行状态
};

class TakeoffHandling
{
public:
	TakeoffHandling() = default; // 默认构造函数
	~TakeoffHandling() = default; // 默认析构函数

	// 初始化参数
	void setSpoolupTime(const float seconds) { _spoolup_time_hysteresis.set_hysteresis_time_from(false, seconds * 1_s); } ///< 设置预热时间
	void setTakeoffRampTime(const float seconds) { _takeoff_ramp_time = seconds; } ///< 设置起飞斜坡时间

	/**
	 * 计算一个垂直速度以初始化起飞斜坡，使得传递给速度控制器时产生零油门设定点。
	 * @param velocity_p_gain 速度控制器的比例增益，用于计算推力
	 */
	void generateInitialRampValue(const float velocity_p_gain);

	/**
	 * 更新起飞状态。
	 * 即使在不进行高度控制飞行时也必须调用此函数，以跳过起飞并在模式切换时不在空中执行起飞。
	 */
	void updateTakeoffState(const bool armed, const bool landed, const bool want_takeoff,
				const float takeoff_desired_vz, const bool skip_takeoff, const hrt_abstime &now_us);

	/**
	 * 更新并返回起飞期间的速度约束斜坡值。
	 * 通过在起飞过程中逐渐增加 `_takeoff_ramp_vz` 并使用它来限制最大爬升率，可以实现平滑的起飞行为。
	 * 在地面上返回零，在飞行中返回 `takeoff_desired_vz`。
	 * @param dt 自上次调用/循环迭代以来的时间（秒）
	 * @param takeoff_desired_vz 速度斜坡的最终值
	 * @return 返回更新后的上升速度限制
	 */
	float updateRamp(const float dt, const float takeoff_desired_vz);

	TakeoffState getTakeoffState() { return _takeoff_state; } ///< 获取当前起飞状态

private:
	TakeoffState _takeoff_state = TakeoffState::disarmed; ///< 当前起飞状态，默认为未武装状态

	systemlib::Hysteresis _spoolup_time_hysteresis{false}; ///< 在飞行器武装后 COM_SPOOLUP_TIME 秒后变为 true

	float _takeoff_ramp_time{0.f}; ///< 起飞斜坡时间
	float _takeoff_ramp_vz_init{0.f}; ///< 导致零推力的垂直速度
	float _takeoff_ramp_progress{0.f}; ///< 从 0 到 1 递增的爬升进度
};
