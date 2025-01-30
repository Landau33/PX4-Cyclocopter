#pragma once

#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/autotune_attitude_control_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/slew_rate/SlewRate.hpp>

#include <AttitudeControl.hpp>

using namespace time_literals;

class CyclocopterAttitudeControl : public ModuleBase<CyclocopterAttitudeControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	CyclocopterAttitudeControl(bool vtol = false); // 构造函数，可选参数 vtol 表示是否为 VTOL 模式
	~CyclocopterAttitudeControl() override; // 析构函数

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
	 * 计算油门曲线
	 * @param throttle_stick_input 输入的油门杆位置
	 * @return 计算后的油门值
	 */
	float throttle_curve(float throttle_stick_input);

	/**
	 * 根据操纵杆输入生成并发布姿态设定点
	 * @param q 当前姿态四元数
	 * @param dt 时间间隔
	 * @param reset_yaw_sp 是否重置偏航设定点
	 */
	void generate_attitude_setpoint(const matrix::Quatf &q, float dt, bool reset_yaw_sp);

	AttitudeControl _attitude_control; /**< 姿态控制计算类 */

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // 参数更新订阅

	uORB::Subscription _autotune_attitude_control_status_sub{ORB_ID(autotune_attitude_control_status)}; // 自动调整姿态控制状态订阅
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)}; // 手动控制设定点订阅
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)}; // 车辆姿态设定点订阅
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)}; // 车辆控制模式订阅
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)}; // 车辆着陆检测订阅
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)}; // 车辆本地位置订阅
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)}; // 车辆状态订阅

	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)}; // 车辆姿态订阅回调

	uORB::Publication<vehicle_rates_setpoint_s>     _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};    /**< 速率设定点发布 */
	uORB::Publication<vehicle_attitude_setpoint_s>  _vehicle_attitude_setpoint_pub; // 姿态设定点发布

	manual_control_setpoint_s       _manual_control_setpoint {};    /**< 手动控制设定点 */
	vehicle_control_mode_s          _vehicle_control_mode {};       /**< 车辆控制模式 */

	perf_counter_t  _loop_perf;             /**< 循环持续时间性能计数器 */

	matrix::Vector3f _thrust_setpoint_body; /**< 机体坐标系下的三维推力向量 */

	float _man_yaw_sp{0.f};                 /**< 手动模式下的当前偏航设定点 */
	float _man_tilt_max;                    /**< 手动飞行时允许的最大倾斜角 [弧度] */

	SlewRate<float> _manual_throttle_minimum{0.f}; ///< 着陆时为0，在空中逐渐增加到MPC_MANTHR_MIN
	SlewRate<float> _manual_throttle_maximum{0.f}; ///< 解除武装时为0，启动后逐渐增加到1
	AlphaFilter<float> _man_roll_input_filter;
	AlphaFilter<float> _man_pitch_input_filter;

	hrt_abstime _last_run{0}; // 上次运行的时间戳
	hrt_abstime _last_attitude_setpoint{0}; // 上次姿态设定点的时间戳

	bool _spooled_up{false}; ///< 用于确保车辆在启动时间内不能起飞
	bool _landed{true}; // 是否着陆
	bool _reset_yaw_sp{true}; // 是否重置偏航设定点
	bool _heading_good_for_control{true}; ///< 初始化为 true，以确保在本地位置未发布时锁定航向
	bool _vehicle_type_rotary_wing{true}; // 是否为旋翼机类型
	bool _vtol{false}; // 是否为 VTOL 模式
	bool _vtol_tailsitter{false}; // 是否为尾座式 VTOL
	bool _vtol_in_transition_mode{false}; // 是否处于 VTOL 过渡模式

	uint8_t _quat_reset_counter{0}; // 四元数重置计数器

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MC_AIRMODE>)         _param_mc_airmode,        /**< 多旋翼空气模式 */
		(ParamFloat<px4::params::MC_MAN_TILT_TAU>)  _param_mc_man_tilt_tau,   /**< 手动倾斜时间常数 */

		(ParamFloat<px4::params::MC_ROLL_P>)        _param_mc_roll_p,         /**< 滚转比例增益 */
		(ParamFloat<px4::params::MC_PITCH_P>)       _param_mc_pitch_p,        /**< 俯仰比例增益 */
		(ParamFloat<px4::params::MC_YAW_P>)         _param_mc_yaw_p,          /**< 偏航比例增益 */
		(ParamFloat<px4::params::MC_YAW_WEIGHT>)    _param_mc_yaw_weight,     /**< 偏航权重 */

		(ParamFloat<px4::params::MC_ROLLRATE_MAX>)  _param_mc_rollrate_max,   /**< 最大滚转角速度 */
		(ParamFloat<px4::params::MC_PITCHRATE_MAX>) _param_mc_pitchrate_max,  /**< 最大俯仰角速度 */
		(ParamFloat<px4::params::MC_YAWRATE_MAX>)   _param_mc_yawrate_max,    /**< 最大偏航角速度 */

		/* 自稳模式参数 */
		(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _param_mpc_man_tilt_max,  /**< 手动飞行时允许的最大倾斜角 */
		(ParamFloat<px4::params::MPC_MAN_Y_MAX>)    _param_mpc_man_y_max,     /**< 从操纵杆到偏航速率的缩放因子 */
		(ParamFloat<px4::params::MPC_MANTHR_MIN>)   _param_mpc_manthr_min,    /**< 稳定模式下的最小油门 */
		(ParamFloat<px4::params::MPC_THR_MAX>)      _param_mpc_thr_max,       /**< 稳定模式下的最大油门 */
		(ParamFloat<px4::params::MPC_THR_HOVER>)    _param_mpc_thr_hover,     /**< 悬停时的油门 */
		(ParamInt<px4::params::MPC_THR_CURVE>)      _param_mpc_thr_curve,     /**< 油门曲线行为 */

		(ParamFloat<px4::params::COM_SPOOLUP_TIME>) _param_com_spoolup_time   /**< 启动时间 */
	)
};
