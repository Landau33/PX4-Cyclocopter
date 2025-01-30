#include "GotoControl.hpp"

#include <drivers/drv_hrt.h>
#include <float.h>
#include <lib/mathlib/mathlib.h>

using namespace time_literals;

bool GotoControl::checkForSetpoint(const hrt_abstime &now, const bool enabled)
{
	_goto_setpoint_sub.update(); // 更新目标设定点订阅
	const bool timestamp_initialized = _goto_setpoint_sub.get().timestamp != 0; // 检查时间戳是否已初始化
	const bool no_timeout = now < (_goto_setpoint_sub.get().timestamp + 500_ms); // 检查是否未超时（500毫秒）
	const bool need_to_run = timestamp_initialized && no_timeout && enabled; // 确定是否需要运行

	if (!need_to_run) {
		_is_initialized = false; // 如果不需要运行，则重置初始化标志
	}

	return need_to_run;
}

void GotoControl::update(const float dt, const matrix::Vector3f &position, const float heading)
{
	if (!_is_initialized) {
		resetPositionSmoother(position); // 重置位置平滑器
		resetHeadingSmoother(heading); // 重置航向平滑器
		_is_initialized = true; // 设置初始化标志
	}

	const goto_setpoint_s &goto_setpoint = _goto_setpoint_sub.get(); // 获取目标设定点

	const Vector3f position_setpoint(_goto_setpoint_sub.get().position); // 获取位置设定点

	if (!position_setpoint.isAllFinite()) {
		// TODO: 错误消息
		_need_smoother_reset = true; // 需要重置平滑器
		return;
	}

	if (!position.isAllFinite()) {
		// TODO: 错误消息
		_need_smoother_reset = true; // 需要重置平滑器
		return;
	}

	if (_need_smoother_reset) {
		resetPositionSmoother(position); // 重置位置平滑器
	}

	setPositionSmootherLimits(goto_setpoint); // 设置位置平滑器限制

	const Vector3f feedforward_velocity{}; // 前馈速度为零
	const bool force_zero_velocity_setpoint = false; // 不强制速度设定点为零
	PositionSmoothing::PositionSmoothingSetpoints out_setpoints; // 输出设定点结构体
	_position_smoothing.generateSetpoints(position, position_setpoint, feedforward_velocity, dt,
					      force_zero_velocity_setpoint, out_setpoints); // 生成位置设定点

	trajectory_setpoint_s trajectory_setpoint{}; // 初始化轨迹设定点结构体
	out_setpoints.position.copyTo(trajectory_setpoint.position); // 设置位置设定点
	out_setpoints.velocity.copyTo(trajectory_setpoint.velocity); // 设置速度设定点
	out_setpoints.acceleration.copyTo(trajectory_setpoint.acceleration); // 设置加速度设定点
	out_setpoints.jerk.copyTo(trajectory_setpoint.jerk); // 设置加加速度设定点

	if (goto_setpoint.flag_control_heading && PX4_ISFINITE(goto_setpoint.heading) && PX4_ISFINITE(heading)) {
		if (!_controlling_heading || _need_smoother_reset) {
			resetHeadingSmoother(heading); // 重置航向平滑器
		}

		setHeadingSmootherLimits(goto_setpoint); // 设置航向平滑器限制
		_heading_smoothing.update(goto_setpoint.heading, dt); // 更新航向平滑器

		trajectory_setpoint.yaw = _heading_smoothing.getSmoothedHeading(); // 设置偏航角设定点
		trajectory_setpoint.yawspeed = _heading_smoothing.getSmoothedHeadingRate(); // 设置偏航速率设定点

		_controlling_heading = true; // 设置控制航向标志

	} else {
		trajectory_setpoint.yaw = NAN; // 设置偏航角设定点为 NaN
		trajectory_setpoint.yawspeed = NAN; // 设置偏航速率设定点为 NaN

		_controlling_heading = false; // 清除控制航向标志
	}

	_need_smoother_reset = false; // 清除需要重置平滑器标志

	trajectory_setpoint.timestamp = goto_setpoint.timestamp; // 设置时间戳
	_trajectory_setpoint_pub.publish(trajectory_setpoint); // 发布轨迹设定点

	vehicle_constraints_s vehicle_constraints{ // 初始化车辆约束结构体
		.timestamp = goto_setpoint.timestamp, // 设置时间戳
		.speed_up = NAN, // 上升速度设为 NaN
		.speed_down = NAN, // 下降速度设为 NaN
		.want_takeoff = false // 不请求起飞
	};
	_vehicle_constraints_pub.publish(vehicle_constraints); // 发布车辆约束
}

void GotoControl::resetPositionSmoother(const matrix::Vector3f &position)
{
	if (!position.isAllFinite()) {
		// TODO: 错误消息
		_need_smoother_reset = true; // 需要重置平滑器
		return;
	}

	const Vector3f initial_acceleration{}; // 初始加速度为零
	const Vector3f initial_velocity{}; // 初始速度为零
	_position_smoothing.reset(initial_acceleration, initial_velocity, position); // 重置位置平滑器

	_need_smoother_reset = false; // 清除需要重置平滑器标志
}

void GotoControl::resetHeadingSmoother(const float heading)
{
	if (!PX4_ISFINITE(heading)) {
		// TODO: 错误消息
		_controlling_heading = false; // 清除控制航向标志
		return;
	}

	const float initial_heading_rate{0.f}; // 初始航向速率为零
	_heading_smoothing.reset(heading, initial_heading_rate); // 重置航向平滑器
}

void GotoControl::setPositionSmootherLimits(const goto_setpoint_s &goto_setpoint)
{
	// 水平约束
	float max_horizontal_speed = _param_mpc_xy_cruise; // 最大水平速度
	float max_horizontal_accel = _param_mpc_acc_hor; // 最大水平加速度

	if (goto_setpoint.flag_set_max_horizontal_speed
	    && PX4_ISFINITE(goto_setpoint.max_horizontal_speed)) {
		max_horizontal_speed = math::constrain(goto_setpoint.max_horizontal_speed, 0.f,
						       _param_mpc_xy_cruise); // 限制最大水平速度

		// 根据水平速度限制线性缩放水平加速度限制以保持平滑动态
		if (!_position_smoothing.getCurrentVelocityXY().longerThan(max_horizontal_speed)) {
			const float speed_scale = max_horizontal_speed / _param_mpc_xy_cruise;
			max_horizontal_accel = math::constrain(_param_mpc_acc_hor * speed_scale, 0.f, _param_mpc_acc_hor);
		}
	}

	_position_smoothing.setCruiseSpeed(max_horizontal_speed); // 设置巡航速度
	_position_smoothing.setMaxAccelerationXY(max_horizontal_accel); // 设置最大水平加速度

	// 垂直约束
	float vehicle_max_vertical_speed = _param_mpc_z_v_auto_dn; // 最大垂直下降速度
	float vehicle_max_vertical_accel = _param_mpc_acc_down_max; // 最大垂直下降加速度

	if (goto_setpoint.position[2] < _position_smoothing.getCurrentPositionZ()) { // 目标高度更高 -> 更负的 Z 轴值
		vehicle_max_vertical_speed = _param_mpc_z_v_auto_up; // 最大垂直上升速度
		vehicle_max_vertical_accel = _param_mpc_acc_up_max; // 最大垂直上升加速度
	}

	float max_vertical_speed = vehicle_max_vertical_speed; // 最大垂直速度
	float max_vertical_accel = vehicle_max_vertical_accel; // 最大垂直加速度

	if (goto_setpoint.flag_set_max_vertical_speed && PX4_ISFINITE(goto_setpoint.max_vertical_speed)) {
		max_vertical_speed = math::constrain(goto_setpoint.max_vertical_speed, 0.f, vehicle_max_vertical_speed); // 限制最大垂直速度

		// 根据垂直速度限制线性缩放垂直加速度限制以保持平滑动态
		if (fabsf(_position_smoothing.getCurrentVelocityZ()) <= max_vertical_speed) {
			const float speed_scale = max_vertical_speed / vehicle_max_vertical_speed;
			max_vertical_accel = math::constrain(vehicle_max_vertical_accel * speed_scale, 0.f, vehicle_max_vertical_accel);
		}
	}

	_position_smoothing.setMaxVelocityZ(max_vertical_speed); // 设置最大垂直速度
	_position_smoothing.setMaxAccelerationZ(max_vertical_accel); // 设置最大垂直加速度
}

void GotoControl::setHeadingSmootherLimits(const goto_setpoint_s &goto_setpoint)
{
	float max_heading_rate = _param_mpc_yawrauto_max; // 最大航向速率
	float max_heading_accel = _param_mpc_yawrauto_acc; // 最大航向加速度

	if (goto_setpoint.flag_set_max_heading_rate && PX4_ISFINITE(goto_setpoint.max_heading_rate)) {
		max_heading_rate = math::constrain(goto_setpoint.max_heading_rate, 0.f, _param_mpc_yawrauto_max); // 限制最大航向速率

		// 根据航向速率限制线性缩放航向加速度限制以保持平滑动态
		if (fabsf(_heading_smoothing.getSmoothedHeadingRate()) <= max_heading_rate) {
			const float rate_scale = max_heading_rate / _param_mpc_yawrauto_max;
			max_heading_accel = math::constrain(_param_mpc_yawrauto_acc * rate_scale, 0.f, _param_mpc_yawrauto_acc);
		}
	}

	_heading_smoothing.setMaxHeadingRate(max_heading_rate); // 设置最大航向速率
	_heading_smoothing.setMaxHeadingAccel(max_heading_accel); // 设置最大航向加速度
}
