#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

struct PositionControlStates {
	matrix::Vector3f position; // 位置
	matrix::Vector3f velocity; // 速度
	matrix::Vector3f acceleration; // 加速度
	float yaw; // 偏航角
};

/**
 * 多旋翼的核心位置控制器。
 * 此类包含用于位置的比例控制器和用于速度的PID控制器。
 * 输入：
 *   - 飞行器的位置、速度和偏航角
 *   - 期望的位置、速度、推力、偏航角和偏航速率设定点
 *   - 比全局限制更严格的约束
 * 输出：
 *   - 推力向量和偏航角设定点
 *
 * 如果存在位置和速度设定点，则使用速度设定点作为前馈。如果前馈有效，
 * 则速度比例控制器输出的速度分量优先于前馈分量。
 *
 * 设定点为 NaN 表示未设置。
 * 如果同时存在位置/速度设定点和推力设定点，则忽略推力设定点并从位置-速度 PID 环重新计算。
 */
class PositionControl
{
public:

	PositionControl() = default;
	~PositionControl() = default;

	/**
	 * 设置位置控制增益
	 * @param P x,y,z 轴的比例增益三维向量
	 */
	void setPositionGains(const matrix::Vector3f &P) { _gain_pos_p = P; }

	/**
	 * 设置速度控制增益
	 * @param P x,y,z 轴的比例增益三维向量
	 * @param I 积分增益三维向量
	 * @param D 微分增益三维向量
	 */
	void setVelocityGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	/**
	 * 设置执行前馈和位置控制时的最大速度
	 * @param vel_horizontal 水平速度限制
	 * @param vel_up 上升速度限制
	 * @param vel_down 下降速度限制
	 */
	void setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down);

	/**
	 * 设置控制器可以输出的最小和最大归一化总推力 [0,1]
	 * @param min 最小推力，例如 0.1 或 0
	 * @param max 最大推力，例如 0.9 或 1
	 */
	void setThrustLimits(const float min, const float max);

	/**
	 * 设置在优先处理垂直推力时为水平控制保留的裕度
	 * @param margin 为水平控制保留的归一化推力裕度，例如 0.3
	 */
	void setHorizontalThrustMargin(const float margin);

	/**
	 * 设置输出姿态允许的最大倾斜角度（弧度）
	 * @param tilt 与水平方向的最大倾斜角度（弧度）
	 */
	void setTiltLimit(const float tilt) { _lim_tilt = tilt; }

	/**
	 * 设置归一化的悬停推力
	 * @param hover_thrust [HOVER_THRUST_MIN, HOVER_THRUST_MAX]，飞行器在水平姿态下不加速上升或下降的悬停推力
	 */
	void setHoverThrust(const float hover_thrust) { _hover_thrust = math::constrain(hover_thrust, HOVER_THRUST_MIN, HOVER_THRUST_MAX); }

	/**
	 * 更新悬停推力而不立即影响输出，通过调整积分器实现。
	 * 这样可以防止悬停推力信号的动态直接传播到控制器的输出。
	 */
	void updateHoverThrust(const float hover_thrust_new);

	/**
	 * 将当前飞行器状态传递给控制器
	 * @param states 包含当前位置、速度等的状态结构体
	 */
	void setState(const PositionControlStates &states);

	/**
	 * 传递期望的设定点
	 * 注意：NaN 值表示无前馈/若没有更高阶的设定点则不控制该状态。
	 * @param setpoint 包括前馈在内的设定点，将在 update() 中执行
	 */
	void setInputSetpoint(const trajectory_setpoint_s &setpoint);

	/**
	 * 应用 P 位置控制器和 PID 速度控制器，更新成员变量推力、偏航角和偏航速率设定点。
	 * @see _thr_sp
	 * @see _yaw_sp
	 * @see _yawspeed_sp
	 * @param dt 自上次迭代以来的时间（秒）
	 * @return 如果更新成功且输出设定点可执行则返回 true，否则返回 false
	 */
	bool update(const float dt);

	/**
	 * 将积分项 xy 设为 0。
	 * @see _vel_int
	 */
	void resetIntegral() { _vel_int.setZero(); }

	/**
	 * 如果设置，则假设没有垂直加速度来计算倾斜设定点
	 */
	void decoupleHorizontalAndVecticalAcceleration(bool val) { _decouple_horizontal_and_vertical_acceleration = val; }

	/**
	 * 获取控制器输出的本地位置设定点
	 * 这些设定点包括 PID 输出和前馈，是实际执行的设定点。
	 * 可以使用加速度或推力设定点进行姿态控制。
	 * @param local_position_setpoint 要填充的结构体引用
	 */
	void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const;

	/**
	 * 获取控制器输出的姿态设定点
	 * 该姿态设定点是从位置和速度控制后生成的加速度设定点中产生的。
	 * 需要由姿态控制器执行此设定点以实现速度和位置跟踪。
	 * @param attitude_setpoint 要填充的结构体引用
	 */
	void getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const;

	/**
	 * 所有设定点均设为 NaN（未控制），时间戳为零。
	 */
	static const trajectory_setpoint_s empty_trajectory_setpoint;

private:
	// 悬停推力配置/估计的范围限制
	static constexpr float HOVER_THRUST_MIN = 0.05f;
	static constexpr float HOVER_THRUST_MAX = 0.9f;

	bool _inputValid();

	void _positionControl(); ///< 位置比例控制
	void _velocityControl(const float dt); ///< 速度 PID 控制
	void _accelerationControl(); ///< 加速度设定点处理

	// 增益
	matrix::Vector3f _gain_pos_p; ///< 位置控制比例增益
	matrix::Vector3f _gain_vel_p; ///< 速度控制比例增益
	matrix::Vector3f _gain_vel_i; ///< 速度控制积分增益
	matrix::Vector3f _gain_vel_d; ///< 速度控制微分增益

	// 限制
	float _lim_vel_horizontal{}; ///< 使用前馈和位置控制时的水平速度限制
	float _lim_vel_up{}; ///< 使用前馈和位置控制时的上升速度限制
	float _lim_vel_down{}; ///< 使用前馈和位置控制时的下降速度限制
	float _lim_thr_min{}; ///< 允许输出的最小总推力 [-1,0]，例如 -0.9
	float _lim_thr_max{}; ///< 允许输出的最大总推力 [-1,0]，例如 -0.1
	float _lim_thr_xy_margin{}; ///< 在饱和优先垂直推力时为水平控制保留的裕度
	float _lim_tilt{}; ///< 输出姿态允许的最大倾斜角度（弧度）

	float _hover_thrust{}; ///< 飞行器在水平姿态下不加速上升或下降的悬停推力 [HOVER_THRUST_MIN, HOVER_THRUST_MAX]
	bool _decouple_horizontal_and_vertical_acceleration{true}; ///< 忽略垂直加速度设定点以消除其对倾斜设定点的影响

	// 状态
	matrix::Vector3f _pos; /**< 当前位置 */
	matrix::Vector3f _vel; /**< 当前速度 */
	matrix::Vector3f _vel_dot; /**< 速度导数（替代加速度估计） */
	matrix::Vector3f _vel_int; /**< 速度控制器的积分项 */
	float _yaw{}; /**< 当前航向 */

	// 设定点
	matrix::Vector3f _pos_sp; /**< 期望的位置 */
	matrix::Vector3f _vel_sp; /**< 期望的速度 */
	matrix::Vector3f _acc_sp; /**< 期望的加速度 */
	matrix::Vector3f _thr_sp; /**< 期望的推力 */
	float _yaw_sp{}; /**< 期望的航向 */
	float _yawspeed_sp{}; /** 期望的偏航速率 */
};
