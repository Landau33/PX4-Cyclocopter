/**
 * 用于平面飞行器平滑位置和航向参考的类。
 *
 * 确保在第一次调用 update() 方法之前使用 setGotoConstraints() 设置约束条件
 */

#pragma once

#include <lib/motion_planning/HeadingSmoothing.hpp>
#include <lib/motion_planning/PositionSmoothing.hpp>
#include <mathlib/math/Limits.hpp>
#include <matrix/matrix/math.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/goto_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>

class GotoControl
{
public:
	GotoControl() = default;
	~GotoControl() = default;

	/**
	 * 检查是否需要处理目标设定点。
	 *
	 * @param now 当前时间戳（hrt_abstime）
	 * @param enabled 是否启用目标控制
	 * @return 如果需要运行则返回 true，否则返回 false
	 */
	bool checkForSetpoint(const hrt_abstime &now, const bool enabled);

	/**
	 * @brief 在当前位置以零速度和加速度重置位置平滑器。
	 *
	 * @param position [m] (NED) 当前本地车辆位置
	 */
	void resetPositionSmoother(const matrix::Vector3f &position);

	/**
	 * @brief 在当前航向以零航向速率和加速度重置航向平滑器。
	 *
	 * @param heading [rad] (从北开始) 当前车辆航向
	 */
	void resetHeadingSmoother(const float heading);

	/**
	 * @brief 使用当前设定点更新平滑器，并输出轨迹设定点供下层控制环跟踪。
	 *
	 * @param[in] dt [s] 自上次控制更新以来的时间（秒）
 	 * @param[in] position [m] (NED) 当前本地车辆位置（米）
 	 * @param[in] heading [rad] (从北开始) 当前车辆航向（弧度）
 	 * @param[in] goto_setpoint 包含当前目标设定点的结构体
 	 * @param[out] trajectory_setpoint 包含轨迹（跟踪）设定点的结构体
	 */
	void update(const float dt, const matrix::Vector3f &position, const float heading);

	// 从外部设置参数可以节省 300 字节的闪存空间
	void setParamCpcXyAcc(const float param_cpc_xy_acc) { _param_cpc_xy_acc = param_cpc_xy_acc; }
	void setParamCpcZAccMaxUP(const float param_cpc_z_acc_max_up) { _param_cpc_z_acc_max_up = param_cpc_z_acc_max_up; }
	void setParamCpcZAccMaxDown(const float param_cpc_z_acc_max_down) { _param_cpc_z_acc_max_down = param_cpc_z_acc_max_down; }
	void setParamCpcXyErrMax(const float param_cpc_xy_err_max) { _position_smoothing.setMaxAllowedHorizontalError(param_cpc_xy_err_max); }
	void setParamCpcXyVelMax(const float param_cpc_xy_vel_max) { _position_smoothing.setMaxVelocityXY(param_cpc_xy_vel_max); }

private:
	/**
	 * @brief 可选地设置动态平移速度限制及其对应的加速度缩放。
	 *
	 * @param goto_setpoint 包含当前目标设定点的结构体
	 */
	void setPositionSmootherLimits(const goto_setpoint_s &goto_setpoint);

	/**
	 * @brief 可选地设置动态航向速率限制及其对应的航向加速度缩放。
	 *
	 * @param goto_setpoint 包含当前目标设定点的结构体
	 */
	void setHeadingSmootherLimits(const goto_setpoint_s &goto_setpoint);

	uORB::SubscriptionData<goto_setpoint_s> _goto_setpoint_sub{ORB_ID(goto_setpoint)}; // 目标设定点订阅
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)}; // 轨迹设定点发布
	uORB::Publication<vehicle_constraints_s> _vehicle_constraints_pub{ORB_ID(vehicle_constraints)}; // 车辆约束发布

	PositionSmoothing _position_smoothing; // 位置平滑器
	HeadingSmoothing _heading_smoothing; // 航向平滑器

	bool _is_initialized{false}; ///< 如果平滑器已重置为当前状态，则为 true

	// 标记下一次 update() 需要有效的当前车辆位置来重置平滑器
	bool _need_smoother_reset{true};

	// 标记上一次 update() 是否控制了航向
	bool _controlling_heading{false};

	float _param_cpc_xy_acc{0.f}; // 水平加速度最大值
	float _param_cpc_z_acc_max_up{0.f}; // 最大垂直上升加速度
	float _param_cpc_z_acc_max_down{0.f}; // 最大垂直下降加速度
	float _param_cpc_yaw_rate_max{0.f} // 最大偏航速率
	float _param_cpc_yaw_acc_max{0.f}; // 最大偏航加速度
};
