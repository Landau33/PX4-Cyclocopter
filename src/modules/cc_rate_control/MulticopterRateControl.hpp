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

class MulticopterRateControl : public ModuleBase<MulticopterRateControl>, public ModuleParams, public px4::WorkItem
{
public:
	MulticopterRateControl(bool vtol = false); // 构造函数，可选参数 vtol 表示是否为 VTOL 模式
	~MulticopterRateControl() override; // 析构函数

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]); // 启动任务

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]); // 自定义命令处理

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr); // 打印使用说明

	bool init(); // 初始化函数

private:
	void Run() override; // 主运行函数

	/**
	 * 从参数初始化一些向量/矩阵
	 */
	void parameters_updated();

	/**
	 * 更新执行器控制状态
	 * @param vehicle_torque_setpoint 车辆扭矩设定点
	 * @param dt 时间间隔
	 */
	void updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint, float dt);

	RateControl _rate_control; ///< 速率控制计算类

	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)}; // 电池状态订阅
	uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)}; // 控制分配器状态订阅
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)}; // 手动控制设定点订阅
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)}; // 车辆控制模式订阅
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)}; // 车辆着陆检测订阅
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)}; // 车辆速率设定点订阅
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)}; // 车辆状态订阅

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // 参数更新订阅

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)}; // 车辆角速度订阅回调

	uORB::Publication<actuator_controls_status_s>	_actuator_controls_status_pub{ORB_ID(actuator_controls_status_0)}; // 执行器控制状态发布
	uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status)}; // 速率控制器状态发布
	uORB::Publication<vehicle_rates_setpoint_s>	_vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)}; // 车辆速率设定点发布
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub; // 车辆扭矩设定点发布
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub; // 车辆推力设定点发布

	vehicle_control_mode_s	_vehicle_control_mode{}; ///< 车辆控制模式
	vehicle_status_s	_vehicle_status{}; ///< 车辆状态

	bool _landed{true}; ///< 是否着陆
	bool _maybe_landed{true}; ///< 可能着陆

	hrt_abstime _last_run{0}; ///< 上次运行的时间戳

	perf_counter_t	_loop_perf;			///< 循环持续时间性能计数器

	// 在更新之间保持设定值
	matrix::Vector3f _acro_rate_max;		///< Acro 模式下的最大姿态速率
	matrix::Vector3f _rates_setpoint{};	///< 速率设定点

	float _battery_status_scale{0.0f}; ///< 电池状态缩放因子
	matrix::Vector3f _thrust_setpoint{}; ///< 推力设定点

	float _energy_integration_time{0.0f}; ///< 能量积分时间
	float _control_energy[4] {}; ///< 控制能量

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ROLLRATE_P>) _param_mc_rollrate_p, ///< 滚转速率 P 增益
		(ParamFloat<px4::params::MC_ROLLRATE_I>) _param_mc_rollrate_i, ///< 滚转速率 I 增益
		(ParamFloat<px4::params::MC_RR_INT_LIM>) _param_mc_rr_int_lim, ///< 滚转速率积分器限制
		(ParamFloat<px4::params::MC_ROLLRATE_D>) _param_mc_rollrate_d, ///< 滚转速率 D 增益
		(ParamFloat<px4::params::MC_ROLLRATE_FF>) _param_mc_rollrate_ff, ///< 滚转速率前馈
		(ParamFloat<px4::params::MC_ROLLRATE_K>) _param_mc_rollrate_k, ///< 滚转速率全局增益

		(ParamFloat<px4::params::MC_PITCHRATE_P>) _param_mc_pitchrate_p, ///< 俯仰速率 P 增益
		(ParamFloat<px4::params::MC_PITCHRATE_I>) _param_mc_pitchrate_i, ///< 俯仰速率 I 增益
		(ParamFloat<px4::params::MC_PR_INT_LIM>) _param_mc_pr_int_lim, ///< 俯仰速率积分器限制
		(ParamFloat<px4::params::MC_PITCHRATE_D>) _param_mc_pitchrate_d, ///< 俯仰速率 D 增益
		(ParamFloat<px4::params::MC_PITCHRATE_FF>) _param_mc_pitchrate_ff, ///< 俯仰速率前馈
		(ParamFloat<px4::params::MC_PITCHRATE_K>) _param_mc_pitchrate_k, ///< 俯仰速率全局增益

		(ParamFloat<px4::params::MC_YAWRATE_P>) _param_mc_yawrate_p, ///< 偏航速率 P 增益
		(ParamFloat<px4::params::MC_YAWRATE_I>) _param_mc_yawrate_i, ///< 偏航速率 I 增益
		(ParamFloat<px4::params::MC_YR_INT_LIM>) _param_mc_yr_int_lim, ///< 偏航速率积分器限制
		(ParamFloat<px4::params::MC_YAWRATE_D>) _param_mc_yawrate_d, ///< 偏航速率 D 增益
		(ParamFloat<px4::params::MC_YAWRATE_FF>) _param_mc_yawrate_ff, ///< 偏航速率前馈
		(ParamFloat<px4::params::MC_YAWRATE_K>) _param_mc_yawrate_k, ///< 偏航速率全局增益

		(ParamFloat<px4::params::MC_ACRO_R_MAX>) _param_mc_acro_r_max, ///< Acro 模式最大滚转速率
		(ParamFloat<px4::params::MC_ACRO_P_MAX>) _param_mc_acro_p_max, ///< Acro 模式最大俯仰速率
		(ParamFloat<px4::params::MC_ACRO_Y_MAX>) _param_mc_acro_y_max, ///< Acro 模式最大偏航速率
		(ParamFloat<px4::params::MC_ACRO_EXPO>) _param_mc_acro_expo, ///< Acro 模式滚转和俯仰指数因子（操纵杆曲线形状）
		(ParamFloat<px4::params::MC_ACRO_EXPO_Y>) _param_mc_acro_expo_y, ///< Acro 模式偏航指数因子（操纵杆曲线形状）
		(ParamFloat<px4::params::MC_ACRO_SUPEXPO>) _param_mc_acro_supexpo, ///< Acro 模式滚转和俯仰超级指数因子（操纵杆曲线形状）
		(ParamFloat<px4::params::MC_ACRO_SUPEXPOY>) _param_mc_acro_supexpoy, ///< Acro 模式偏航超级指数因子（操纵杆曲线形状）

		(ParamBool<px4::params::MC_BAT_SCALE_EN>) _param_mc_bat_scale_en ///< 电池功率水平缩放启用标志
	)
};
