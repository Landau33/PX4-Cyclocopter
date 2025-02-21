#pragma once

#include <lib/rate_control/rate_control.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

class CyclocopterRateControl : public ModuleBase<CyclocopterRateControl>, public ModuleParams, public px4::WorkItem
{
public:
	CyclocopterRateControl();
	~CyclocopterRateControl() override

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;
	void parameters_updated();

	/**
	 * 更新执行器控制状态
	 * @param vehicle_torque_setpoint 扭矩设定点
	 * @param dt 时间间隔
	 */
	void updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint, float dt);

	RateControl _rate_control;

	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)}; // 电池状态订阅
	uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)}; // 控制分配器状态订阅
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)}; // 手动控制设定点订阅
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)}; // 控制模式订阅
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)}; // 着陆检测订阅
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)}; // 速率设定点订阅
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)}; // 状态订阅

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // 参数更新订阅

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)}; // 角速度订阅回调

	uORB::Publication<actuator_controls_status_s>	_actuator_controls_status_pub{ORB_ID(actuator_controls_status_0)}; // 执行器控制状态发布
	uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status)}; // 速率控制器状态发布
	uORB::Publication<vehicle_rates_setpoint_s>	_vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)}; // 速率设定点发布
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub; // 力矩设定点发布
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub; // 推力设定点发布

	vehicle_control_mode_s 	_vehicle_control_mode{};
	vehicle_status_s 	_vehicle_status{};

	bool _landed{true}; ///< 是否着陆
	bool _maybe_landed{true}; ///< 可能着陆

	hrt_abstime _last_run{0}; ///< 上次运行的时间戳

	perf_counter_t	_loop_perf; ///< 循环持续时间性能计数器

	matrix::Vector3f _rates_setpoint{};	///< 速率设定点
	matrix::Vector3f _thrust_setpoint{}; ///< 推力设定点

	float _battery_status_scale{0.0f}; ///< 电池状态缩放因子
	float _energy_integration_time{0.0f}; ///< 能量积分时间
	float _control_energy[4] {}; ///< 控制能量

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CC_ROLL_RATE_P>) _param_cc_roll_rate_p, ///< 滚转速率 P 增益
		(ParamFloat<px4::params::CC_ROLL_RATE_I>) _param_cc_roll_rate_i, ///< 滚转速率 I 增益
		(ParamFloat<px4::params::CC_RR_INT_LIM>) _param_cc_rr_int_lim, ///< 滚转速率积分器限制
		(ParamFloat<px4::params::CC_ROLL_RATE_D>) _param_cc_roll_rate_d, ///< 滚转速率 D 增益
		(ParamFloat<px4::params::CC_ROLL_RATE_FF>) _param_cc_roll_rate_ff, ///< 滚转速率前馈
		(ParamFloat<px4::params::CC_ROLL_RATE_K>) _param_cc_roll_rate_k, ///< 滚转速率全局增益

		(ParamFloat<px4::params::CC_PITCH_RATE_P>) _param_cc_pitch_rate_p, ///< 俯仰速率 P 增益
		(ParamFloat<px4::params::CC_PITCH_RATE_I>) _param_cc_pitch_rate_i, ///< 俯仰速率 I 增益
		(ParamFloat<px4::params::CC_PR_INT_LIM>) _param_cc_pr_int_lim, ///< 俯仰速率积分器限制
		(ParamFloat<px4::params::CC_PITCH_RATE_D>) _param_cc_pitch_rate_d, ///< 俯仰速率 D 增益
		(ParamFloat<px4::params::CC_PITCH_RATE_FF>) _param_cc_pitch_rate_ff, ///< 俯仰速率前馈
		(ParamFloat<px4::params::CC_PITCH_RATE_K>) _param_cc_pitch_rate_k, ///< 俯仰速率全局增益

		(ParamFloat<px4::params::CC_YAW_RATE_P>) _param_cc_yaw_rate_p, ///< 偏航速率 P 增益
		(ParamFloat<px4::params::CC_YAW_RATE_I>) _param_cc_yaw_rate_i, ///< 偏航速率 I 增益
		(ParamFloat<px4::params::CC_YR_INT_LIM>) _param_cc_yr_int_lim, ///< 偏航速率积分器限制
		(ParamFloat<px4::params::CC_YAW_RATE_D>) _param_cc_yaw_rate_d, ///< 偏航速率 D 增益
		(ParamFloat<px4::params::CC_YAW_RATE_FF>) _param_cc_yaw_rate_ff, ///< 偏航速率前馈
		(ParamFloat<px4::params::CC_YAW_RATE_K>) _param_cc_yaw_rate_k, ///< 偏航速率全局增益

		(ParamBool<px4::params::CC_BAT_SCALE_EN>) _param_cc_bat_scale_en ///< 电池功率水平缩放启用标志
	)

};
