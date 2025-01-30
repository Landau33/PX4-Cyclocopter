#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>

class AttitudeControl
{
public:
	AttitudeControl() = default;
	~AttitudeControl() = default;

	/**
	 * 设置比例姿态控制增益
	 * @param proportional_gain 包含滚转、俯仰和偏航增益的3D向量
	 * @param yaw_weight 一个介于[0,1]之间的分数，用于降低偏航相对于滚转和俯仰的优先级
	 */
	void setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight);

	/**
	 * 设置输出速率设定值的硬限制
	 * @param rate_limit [弧度/秒] 包含滚转、俯仰和偏航限制的3D向量
	 */
	void setRateLimit(const matrix::Vector3f &rate_limit) { _rate_limit = rate_limit; }

	/**
	 * 设置一个新的姿态设定点，替换之前跟踪的姿态设定点
	 * @param qd 期望的车辆姿态设定点
	 * @param yawspeed_setpoint [弧度/秒] 世界坐标系中的偏航前馈角速度
	 */
	void setAttitudeSetpoint(const matrix::Quatf &qd, const float yawspeed_setpoint)
	{
		_attitude_setpoint_q = qd;
		_attitude_setpoint_q.normalize();
		_yawspeed_setpoint = yawspeed_setpoint;
	}

	/**
	 * 通过一个增量旋转调整最后已知的姿态设定点
	 * 可选使用，以避免在姿态估计参考（例如航向）变化时出现突变。
	 * @param q_delta 要应用的增量旋转
	 */
	void adaptAttitudeSetpoint(const matrix::Quatf &q_delta)
	{
		_attitude_setpoint_q = q_delta * _attitude_setpoint_q;
		_attitude_setpoint_q.normalize();
	}

	/**
	 * 运行一次控制循环计算
	 * @param q 当前车辆姿态单位四元数的估计值
	 * @return [弧度/秒] 由速率控制器执行的机体坐标系中3D角速度设定值向量
	 */
	matrix::Vector3f update(const matrix::Quatf &q) const;

private:
	matrix::Vector3f _proportional_gain; ///< 比例增益
	matrix::Vector3f _rate_limit; ///< 速率限制
	float _yaw_w{0.f}; ///< 偏航权重 [0,1]，用于降低相对于滚转和俯仰的优先级

	matrix::Quatf _attitude_setpoint_q; ///< 最后已知的姿态设定点，例如来自位置控制器
	float _yawspeed_setpoint{0.f}; ///< 最后已知的偏航前馈设定点
};
