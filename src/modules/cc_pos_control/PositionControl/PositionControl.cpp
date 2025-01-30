#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>

using namespace matrix;

const trajectory_setpoint_s PositionControl::empty_trajectory_setpoint = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P; // 设置速度控制器的比例增益
	_gain_vel_i = I; // 设置速度控制器的积分增益
	_gain_vel_d = D; // 设置速度控制器的微分增益
}

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal; // 设置水平速度限制
	_lim_vel_up = vel_up; // 设置上升速度限制
	_lim_vel_down = vel_down; // 设置下降速度限制
}

void PositionControl::setThrustLimits(const float min, const float max)
{
	// 确保推力向量长度足够大以推断姿态
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin; // 设置水平推力裕度
}

void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// 计算新的悬停推力对加速度积分器的影响，以保持相同的推力
	const float previous_hover_thrust = _hover_thrust;
	setHoverThrust(hover_thrust_new);

	_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
		       + CONSTANTS_ONE_G - _acc_sp(2);
}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position; // 设置当前位置
	_vel = states.velocity; // 设置当前速度
	_yaw = states.yaw; // 设置当前偏航角
	_vel_dot = states.acceleration; // 设置当前加速度
}

void PositionControl::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.position); // 设置位置设定点
	_vel_sp = Vector3f(setpoint.velocity); // 设置速度设定点
	_acc_sp = Vector3f(setpoint.acceleration); // 设置加速度设定点
	_yaw_sp = setpoint.yaw; // 设置偏航角设定点
	_yawspeed_sp = setpoint.yawspeed; // 设置偏航速率设定点
}

bool PositionControl::update(const float dt)
{
	bool valid = _inputValid(); // 检查输入是否有效

	if (valid) {
		_positionControl(); // 执行位置控制逻辑
		_velocityControl(dt); // 执行速度控制逻辑

		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f; // 处理无效的偏航速率设定点
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // 处理无效的偏航角设定点
	}

	// 确保输出的加速度和推力设定点是有效的
	return valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}

void PositionControl::_positionControl()
{
	// P 位置控制器
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p); // 计算基于位置误差的速度设定点
	// 如果位置或前馈速度设定点为 NaN，则不对其产生影响
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// 确保后续约束时没有 NaN 元素
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// 优先考虑位置设定点方向上的速度分量，限制水平速度
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// 限制垂直方向的速度
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

void PositionControl::_velocityControl(const float dt)
{
	// 限制垂直方向的速度积分
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

	// PID 速度控制器
	Vector3f vel_error = _vel_sp - _vel; // 计算速度误差
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d); // 计算期望加速度

	// 如果设定点或对应状态为 NaN，则不产生控制输入
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	_accelerationControl(); // 执行加速度控制逻辑

	// 垂直方向的积分抗饱和
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {
		vel_error(2) = 0.f;
	}

	// 优先处理垂直控制并保持水平裕度
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	// 确定在保持水平裕度的情况下可用的垂直推力
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// 饱和最大垂直推力
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// 确定在优先处理垂直控制后剩余的水平推力
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0.f;

	if (thrust_max_xy_squared > 0.f) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// 饱和水平方向的推力
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// 使用跟踪抗饱和算法处理水平方向：在饱和期间，使用积分器解除饱和
	const Vector2f acc_sp_xy_produced = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);
	const float arw_gain = 2.f / _gain_vel_p(0);

	// 产生的加速度可能大于或小于期望的加速度，由于饱和和实际垂直推力（独立计算）。
	// 只有在信号饱和时才需要运行 ARW 循环。
	const Vector2f acc_sp_xy = _acc_sp.xy();
	const Vector2f acc_limited_xy = (acc_sp_xy.norm_squared() > acc_sp_xy_produced.norm_squared())
					? acc_sp_xy_produced
					: acc_sp_xy;
	vel_error.xy() = Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_limited_xy);

	// 确保积分器中没有 NaN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// 更新速度控制器的积分部分
	_vel_int += vel_error.emult(_gain_vel_i) * dt;
}

void PositionControl::_accelerationControl()
{
	// 假设垂直方向的标准重力加速度用于姿态生成
	float z_specific_force = -CONSTANTS_ONE_G;

	if (!_decouple_horizontal_and_vertical_acceleration) {
		// 包括垂直加速度设定点以更好地跟踪水平加速度
		z_specific_force += _acc_sp(2);
	}

	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized(); // 计算机体 Z 轴向量
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt); // 限制倾斜角度
	// 假设悬停推力产生标准重力加速度
	const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// 将推力投影到计划的机体姿态上
	const float cos_ned_body = (Vector3f(0, 0, 1).dot(body_z));
	const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust; // 设置推力设定点
}

bool PositionControl::_inputValid()
{
	bool valid = true;

	// 每个轴 x, y, z 必须有一些设定点
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// x 和 y 输入设定点必须成对出现
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// 对于每个受控状态，估计值必须有效
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0); // 设置本地位置设定点的 X 分量
	local_position_setpoint.y = _pos_sp(1); // 设置本地位置设定点的 Y 分量
	local_position_setpoint.z = _pos_sp(2); // 设置本地位置设定点的 Z 分量
	local_position_setpoint.yaw = _yaw_sp; // 设置偏航角设定点
	local_position_setpoint.yawspeed = _yawspeed_sp; // 设置偏航速率设定点
	local_position_setpoint.vx = _vel_sp(0); // 设置速度设定点的 X 分量
	local_position_setpoint.vy = _vel_sp(1); // 设置速度设定点的 Y 分量
	local_position_setpoint.vz = _vel_sp(2); // 设置速度设定点的 Z 分量
	_acc_sp.copyTo(local_position_setpoint.acceleration); // 设置加速度设定点
	_thr_sp.copyTo(local_position_setpoint.thrust); // 设置推力设定点
}

void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint); // 将推力向量和偏航角转换为姿态设定点
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp; // 设置偏航速率设定点
}
