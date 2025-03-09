#include <AttitudeControl.hpp>

#include <mathlib/math/Functions.hpp>

using namespace matrix;

// 设置比例增益
void AttitudeControl::setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight)
{
	_proportional_gain = proportional_gain;
	_yaw_w = math::constrain(yaw_weight, 0.f, 1.f);

	// 补偿由于偏航权重重新缩放输出的影响
	if (_yaw_w > 1e-4f) {
		_proportional_gain(2) /= _yaw_w;
	}
}

matrix::Vector3f AttitudeControl::update(const Quatf &q) const
{
	Quatf qd = _attitude_setpoint_q;

	// 计算忽略车辆偏航的简化期望姿态，以优先考虑滚转和俯仰
	const Vector3f e_z = q.dcm_z();
	const Vector3f e_z_d = qd.dcm_z();
	Quatf qd_red(e_z, e_z_d);

	if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
		// 在极小的情况下，当车辆和推力方向完全相反时，
		// 完整的姿态控制无论如何都不会生成偏航输入，并直接结合滚转和俯仰以获得正确的期望偏航。
		// 忽略这种情况仍然完全是安全和稳定的。
		qd_red = qd;

	} else {
		// 将从当前到期望推力向量的旋转转换为世界坐标系中的简化期望姿态
		qd_red *= q;
	}

	// 混合完整和简化的期望姿态
	Quatf q_mix = qd_red.inversed() * qd;
	q_mix.canonicalize();
	// 捕捉 acosf 和 asinf 的数值问题
	q_mix(0) = math::constrain(q_mix(0), -1.f, 1.f);
	q_mix(3) = math::constrain(q_mix(3), -1.f, 1.f);
	qd = qd_red * Quatf(cosf(_yaw_w * acosf(q_mix(0))), 0, 0, sinf(_yaw_w * asinf(q_mix(3))));

	// 四元数姿态控制律，qe 是从 q 到 qd 的旋转
	const Quatf qe = q.inversed() * qd;

	// 使用 sin(alpha/2) 缩放的旋转轴作为姿态误差（参见四元数的轴角定义）
	// 同时处理反四元数的二义性
	const Vector3f eq = 2.f * qe.canonical().imag();

	// 计算角速度设定值
	Vector3f rate_setpoint = eq.emult(_proportional_gain);

	// 前馈偏航设定速率。
	// yawspeed_setpoint 是绕世界 z 轴的命令旋转（在世界坐标系中表示），
	// 但我们需要将其应用到机体坐标系中（因为 _rates_sp 是在机体坐标系中表示的）。
	// 因此我们通过取 R.transposed（== q.inversed）的最后一列来推断世界 z 轴（在机体坐标系中表示）
	// 并乘以偏航设定速率（yawspeed_setpoint）。
	// 这样可以得到一个向量，表示绕世界 z 轴的命令旋转，在机体坐标系中表示，
	// 以便可以添加到速率设定值中。
	if (std::isfinite(_yawspeed_setpoint)) {
		rate_setpoint += q.inversed().dcm_z() * _yawspeed_setpoint;
	}

	// 限制速率
	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	return rate_setpoint;
}
