#pragma once

#include "PositionControl/PositionControl.hpp"
#include "Takeoff/Takeoff.hpp"
#include "GotoControl/GotoControl.hpp"

#include <drivers/drv_hrt.h>
#include <lib/controllib/blocks.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRateYaw.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

using namespace time_literals;

class CyclocopterPositionControl : public ModuleBase<CyclocopterPositionControl>, public control::SuperBlock,
	public ModuleParams, public px4::ScheduledWorkItem
{
public:
	CyclocopterPositionControl();
	~CyclocopterPositionControl() override;

	static int task_spawn(int argc, char *argv[]); ///< 启动任务

	static int custom_command(int argc, char *argv[]); ///< 自定义命令处理

	static int print_usage(const char *reason = nullptr); ///< 打印使用说明

	bool init(); ///< 初始化函数

private:
	void Run() override; ///< 主运行函数

	TakeoffHandling _takeoff; ///< 状态机和斜坡，用于平稳地将飞行器从地面起飞而不会出现突变

	orb_advert_t _mavlink_log_pub{nullptr}; ///< MAVLink 日志发布对象

	uORB::PublicationData<takeoff_status_s>              _takeoff_status_pub{ORB_ID(takeoff_status)}; ///< 起飞状态发布
	uORB::Publication<vehicle_attitude_setpoint_s>	     _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)}; ///< 飞行器姿态设定点发布
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)}; ///< 飞行器本地位置设定点发布

	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)}; ///< 飞行器本地位置订阅回调

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s}; ///< 参数更新订阅

	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)}; ///< 悬停推力估计订阅
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)}; ///< 轨迹设定点订阅
	uORB::Subscription _vehicle_constraints_sub{ORB_ID(vehicle_constraints)}; ///< 飞行器约束订阅
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)}; ///< 飞行器控制模式订阅
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)}; ///< 飞行器着陆检测订阅

	hrt_abstime _time_stamp_last_loop{0}; ///< 上次循环迭代的时间戳
	hrt_abstime _time_position_control_enabled{0}; ///< 位置控制启用时间戳

	trajectory_setpoint_s _setpoint{PositionControl::empty_trajectory_setpoint}; ///< 轨迹设定点
	vehicle_control_mode_s _vehicle_control_mode{}; ///< 飞行器控制模式

	vehicle_constraints_s _vehicle_constraints {
		.timestamp = 0,
		.speed_up = NAN,
		.speed_down = NAN,
		.want_takeoff = false,
	};

	vehicle_land_detected_s _vehicle_land_detected {
		.timestamp = 0,
		.freefall = false,
		.ground_contact = true,
		.maybe_landed = true,
		.landed = true,
	};

	DEFINE_PARAMETERS(
		// 位置控制参数
		(ParamFloat<px4::params::CPC_XY_P>) 		_param_cpc_xy_p,
		(ParamFloat<px4::params::CPC_Z_P>)		_param_cpc_z_p,
		// 速度控制参数
		(ParamFloat<px4::params::CPC_XY_VEL_P>)		_param_cpc_xy_vel_p,
		(ParamFloat<px4::params::CPC_XY_VEL_I>)		_param_cpc_xy_vel_i,
		(ParamFloat<px4::params::CPC_XY_VEL_D>)		_param_cpc_xy_vel_d,
		(ParamFloat<px4::params::CPC_Z_VEL_P>)		_param_cpc_z_vel_p,
		(ParamFloat<px4::params::CPC_Z_VEL_I>)		_param_cpc_z_vel_i,
		(ParamFloat<px4::params::CPC_Z_VEL_D>)		_param_cpc_z_vel_d,

		// 最大速度限制
		(ParamFloat<px4::params::CPC_XY_VEL_MAX>)	_param_cpc_xy_vel_max,
		(ParamFloat<px4::params::CPC_Z_VEL_MAX_UP>)	_param_cpc_z_vel_max_up,
		(ParamFloat<px4::params::CPC_Z_VEL_MAX_DOWN>)	_param_cpc_z_vel_max_down,
		// 最大加速度限制
		(ParamFloat<px4::params::CPC_XY_ACC_MAX>)	_param_cpc_xy_acc_max,
		(ParamFloat<px4::params::CPC_Z_ACC_MAX_UP>)	_param_cpc_z_acc_max_up,
		(ParamFloat<px4::params::CPC_Z_ACC_MAX_DOWN>)	_param_cpc_z_acc_max_down,
		// 最大倾角限制
		(ParamFloat<px4::params::CPC_TILT_MAX>)		_param_cpc_tilt_max,
		(ParamFloat<px4::params::CPC_TILT_MAX_LAND>)	_param_cpc_tilt_max_land,
		// 总推力限制
		(ParamFloat<px4::params::CPC_THR_MAX>)      	_param_cpc_thr_max,
		(ParamFloat<px4::params::CPC_THR_MIN>)      	_param_cpc_thr_min,

		// 水平推力裕度
		(ParamFloat<px4::params::CPC_XY_THR_MARG>)	_param_cpc_xy_thr_marg,
		// 水平加速度
		(ParamFloat<px4::params::CPC_XY_ACC>)		_param_cpc_xy_acc,
		// 悬停推力
		(ParamFloat<px4::params::CPC_THR_HOVER>)	_param_cpc_thr_hover,
		// 悬停推力估计
		(ParamBool<px4::params::CPC_USE_HTE>)		_param_cpc_use_hte,
		// 加速度与倾斜解耦
		(ParamBool<px4::params::CPC_ACC_DECOUPLE>)	_param_cpc_acc_decouple,

		// 解锁后电机怠速时间
		(ParamFloat<px4::params::COM_SPOOLUP_TIME>)	_param_com_spoolup_time,
		// 平滑起飞斜坡时间常数
		(ParamFloat<px4::params::CPC_TKOF_RAMP_T>)	_param_cpc_tkof_ramp_t,
		// 起降速度
		(ParamFloat<px4::params::CPC_TKOF_SPEED>)	_param_cpc_tkof_speed,
		(ParamFloat<px4::params::CPC_LAND_SPEED>)	_param_cpc_land_speed,

		// 位置参考模式
		(ParamInt<px4::params::CPC_POS_MODE>)		_param_cpc_pos_mode,
		// 高度参考模式
		(ParamInt<px4::params::CPC_ALT_MODE>)		_param_cpc_alt_mode,

		// 整体响应性
		(ParamFloat<px4::params::SYS_VEHICLE_RESP>) 	_param_sys_vehicle_resp,
		// 整体抖动限制
		(ParamFloat<px4::params::CPC_JERK_MAX>)     	_param_cpc_jerk_max,
		// 整体速度限制
		(ParamFloat<px4::params::CPC_XY_VEL_ALL>)    	_param_cpc_xy_vel_all,
		(ParamFloat<px4::params::CPC_Z_VEL_ALL>)    	_param_cpc_z_vel_all,
		// 轨迹生成器允许的最大水平误差
		(ParamFloat<px4::params::CPC_XY_ERR_MAX>)   	_param_cpc_xy_err_max,
	);

	// 速度导数
	control::BlockDerivative _vel_x_deriv;
	control::BlockDerivative _vel_y_deriv;
	control::BlockDerivative _vel_z_deriv;

	// 处理平滑前往位置设定点的类
	GotoControl _goto_control;
	// 核心 PID 位置控制类
	PositionControl _position_control;

	// 上次警告消息发送的时间戳
	hrt_abstime _last_warn{0};

	// 悬停推力初始化标志
	bool _hover_thrust_initialized{false};

	// 轨迹数据被认为无效的超时时间
	static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500_ms;
	// 在平滑起飞过程中，低于此高度时禁用偏航控制并限制倾斜
	static constexpr float ALTITUDE_THRESHOLD = 0.3f;
	// 超过89deg的数据由于tan函数而出错
	static constexpr float MAX_SAFE_TILT_DEG = 89.f;

	SlewRate<float> _tilt_limit_slew_rate; ///< 倾斜限制的斜率变化率

	uint8_t _vxy_reset_counter{0}; ///< 水平速度重置计数器
	uint8_t _vz_reset_counter{0};  ///< 垂直速度重置计数器
	uint8_t _xy_reset_counter{0};  ///< 水平位置重置计数器
	uint8_t _z_reset_counter{0};   ///< 垂直位置重置计数器
	uint8_t _heading_reset_counter{0}; ///< 航向重置计数器
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")}; ///< 循环性能计数器

	/**
	 * 更新本地参数缓存。
	 * 如果参数为 true，则强制更新参数。
	 * @param force 强制更新参数
	 */
	void parameters_update(bool force);

	// 检查位置/速度状态的有效性。
	PositionControlStates set_vehicle_states(const vehicle_local_position_s &local_pos);

	/**
	 * 生成一个备用设定点以应对没有可用的可执行设定点的情况。
	 * 用于处理尚未生成适当设定点或接收到无效设定点的转换阶段。
	 * 这种情况应该只在转换期间短暂发生，而在模式操作期间或设计上不应发生。
	 * @param now 当前时间戳
	 * @param states 位置控制状态
	 * @param warn 是否发出警告
	 */
	trajectory_setpoint_s generateFailsafeSetpoint(const hrt_abstime &now, const PositionControlStates &states, bool warn);

	/**
	 * 使用 EKF 重置增量调整现有（或旧的）设定点，并更新本地计数器。
	 * @param vehicle_local_position 包含 EKF 重置增量和计数器的结构体
	 * @param setpoint 要调整的轨迹设定点结构体
	 */
	void adjustSetpointForEKFResets(const vehicle_local_position_s &vehicle_local_position,
					trajectory_setpoint_s &setpoint);
};
