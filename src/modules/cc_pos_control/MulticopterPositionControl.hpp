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

class MulticopterPositionControl : public ModuleBase<MulticopterPositionControl>, public control::SuperBlock,
	public ModuleParams, public px4::ScheduledWorkItem
{
public:
	MulticopterPositionControl(bool vtol = false); // 构造函数，可选参数 vtol 表示是否为 VTOL 模式
	~MulticopterPositionControl() override; // 析构函数

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]); // 启动任务

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]); // 自定义命令处理

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr); // 打印使用说明

	bool init(); // 初始化函数

private:
	void Run() override; // 主运行函数

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
		(ParamFloat<px4::params::MPC_XY_P>)         _param_mpc_xy_p,          ///< 水平位置误差的比例增益
		(ParamFloat<px4::params::MPC_Z_P>)          _param_mpc_z_p,           ///< 垂直位置误差的比例增益
		(ParamFloat<px4::params::MPC_XY_VEL_P_ACC>) _param_mpc_xy_vel_p_acc,  ///< 水平速度误差的比例增益
		(ParamFloat<px4::params::MPC_XY_VEL_I_ACC>) _param_mpc_xy_vel_i_acc,  ///< 水平速度误差的积分增益
		(ParamFloat<px4::params::MPC_XY_VEL_D_ACC>) _param_mpc_xy_vel_d_acc,  ///< 水平速度误差的微分增益
		(ParamFloat<px4::params::MPC_Z_VEL_P_ACC>)  _param_mpc_z_vel_p_acc,   ///< 垂直速度误差的比例增益
		(ParamFloat<px4::params::MPC_Z_VEL_I_ACC>)  _param_mpc_z_vel_i_acc,   ///< 垂直速度误差的积分增益
		(ParamFloat<px4::params::MPC_Z_VEL_D_ACC>)  _param_mpc_z_vel_d_acc,   ///< 垂直速度误差的微分增益
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>)   _param_mpc_xy_vel_max,    ///< 最大水平速度
		(ParamFloat<px4::params::MPC_Z_V_AUTO_UP>)  _param_mpc_z_v_auto_up,   ///< 自主导航模式下的上升速度
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,  ///< 最大上升速度
		(ParamFloat<px4::params::MPC_Z_V_AUTO_DN>)  _param_mpc_z_v_auto_dn,   ///< 自主导航模式下的下降速度
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,  ///< 最大下降速度
		(ParamFloat<px4::params::MPC_TILTMAX_AIR>)  _param_mpc_tiltmax_air,   ///< 空中最大倾斜角
		(ParamFloat<px4::params::MPC_THR_HOVER>)    _param_mpc_thr_hover,     ///< 悬停所需的垂直推力
		(ParamBool<px4::params::MPC_USE_HTE>)       _param_mpc_use_hte,       ///< 是否使用悬停推力估计器
		(ParamBool<px4::params::MPC_ACC_DECOUPLE>)  _param_mpc_acc_decouple,  ///< 加速度与倾斜解耦

		// 起飞 / 着陆参数
		(ParamFloat<px4::params::COM_SPOOLUP_TIME>) _param_com_spoolup_time,  ///< 武装后电机预热时间
		(ParamBool<px4::params::COM_THROW_EN>)      _param_com_throw_en,      ///< 投掷启动使能
		(ParamFloat<px4::params::MPC_TKO_RAMP_T>)   _param_mpc_tko_ramp_t,   ///< 平滑起飞斜坡时间常数
		(ParamFloat<px4::params::MPC_TKO_SPEED>)    _param_mpc_tko_speed,     ///< 起飞爬升速度
		(ParamFloat<px4::params::MPC_LAND_SPEED>)   _param_mpc_land_speed,    ///< 着陆下降速度

		(ParamFloat<px4::params::MPC_VEL_MANUAL>)   _param_mpc_vel_manual,    ///< 位置模式下的最大水平速度设定点
		(ParamFloat<px4::params::MPC_VEL_MAN_BACK>) _param_mpc_vel_man_back,  ///< 位置模式下的最大后向速度
		(ParamFloat<px4::params::MPC_VEL_MAN_SIDE>) _param_mpc_vel_man_side,  ///< 位置模式下的最大侧向速度
		(ParamFloat<px4::params::MPC_XY_CRUISE>)    _param_mpc_xy_cruise,     ///< 自主导航模式下的默认水平速度
		(ParamFloat<px4::params::MPC_LAND_ALT2>)    _param_mpc_land_alt2,     ///< 慢速降落第二阶段的高度
		(ParamInt<px4::params::MPC_POS_MODE>)       _param_mpc_pos_mode,      ///< 位置模式变体
		(ParamInt<px4::params::MPC_ALT_MODE>)       _param_mpc_alt_mode,      ///< 高度参考模式
		(ParamFloat<px4::params::MPC_TILTMAX_LND>)  _param_mpc_tiltmax_lnd,  ///< 着陆和起飞时的最大倾斜角
		(ParamFloat<px4::params::MPC_THR_MIN>)      _param_mpc_thr_min,       ///< 最小总推力
		(ParamFloat<px4::params::MPC_THR_MAX>)      _param_mpc_thr_max,       ///< 最大总推力
		(ParamFloat<px4::params::MPC_THR_XY_MARG>)  _param_mpc_thr_xy_marg,  ///< 水平推力裕度

		(ParamFloat<px4::params::SYS_VEHICLE_RESP>) _param_sys_vehicle_resp,  ///< 整体响应性
		(ParamFloat<px4::params::MPC_ACC_HOR>)      _param_mpc_acc_hor,       ///< 水平加速度
		(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _param_mpc_acc_down_max,  ///< 最大向下加速度
		(ParamFloat<px4::params::MPC_ACC_UP_MAX>)   _param_mpc_acc_up_max,    ///< 最大向上加速度
		(ParamFloat<px4::params::MPC_ACC_HOR_MAX>)  _param_mpc_acc_hor_max,   ///< 最大水平加速度
		(ParamFloat<px4::params::MPC_JERK_AUTO>)    _param_mpc_jerk_auto,     ///< 自主导航模式下的抖动限制
		(ParamFloat<px4::params::MPC_JERK_MAX>)     _param_mpc_jerk_max,      ///< 整体水平和垂直抖动限制
		(ParamFloat<px4::params::MPC_MAN_Y_MAX>)    _param_mpc_man_y_max,     ///< 手动偏航速率最大值
		(ParamFloat<px4::params::MPC_MAN_Y_TAU>)    _param_mpc_man_y_tau,     ///< 手动偏航速率输入滤波器时间常数

		(ParamFloat<px4::params::MPC_XY_VEL_ALL>)   _param_mpc_xy_vel_all,    ///< 整体水平速度限制
		(ParamFloat<px4::params::MPC_Z_VEL_ALL>)    _param_mpc_z_vel_all,     ///< 整体垂直速度限制

		(ParamFloat<px4::params::MPC_XY_ERR_MAX>)   _param_mpc_xy_err_max,    ///< 轨迹生成器允许的最大水平误差
		(ParamFloat<px4::params::MPC_YAWRAUTO_MAX>) _param_mpc_yawrauto_max,  ///< 自主导航模式下的最大偏航速率
		(ParamFloat<px4::params::MPC_YAWRAUTO_ACC>) _param_mpc_yawrauto_acc   ///< 自主导航模式下的最大偏航加速度
	);

	control::BlockDerivative _vel_x_deriv; ///< x 方向的速度导数
	control::BlockDerivative _vel_y_deriv; ///< y 方向的速度导数
	control::BlockDerivative _vel_z_deriv; ///< z 方向的速度导数

	GotoControl _goto_control; ///< 处理平滑前往位置设定点的类
	PositionControl _control; ///< 核心 PID 位置控制类

	hrt_abstime _last_warn{0}; ///< 上次警告消息发送的时间戳

	bool _hover_thrust_initialized{false}; ///< 悬停推力初始化标志

	/** 轨迹数据被认为无效的超时时间（微秒） */
	static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500_ms;

	/** 在平滑起飞过程中，低于此高度时禁用偏航控制并限制倾斜 */
	static constexpr float ALTITUDE_THRESHOLD = 0.3f;

	static constexpr float MAX_SAFE_TILT_DEG = 89.f; // 数值问题出现在这个值以上，因为 tanf 函数的原因

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

	/**
	 * 检查位置/速度状态的有效性。
	 */
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
