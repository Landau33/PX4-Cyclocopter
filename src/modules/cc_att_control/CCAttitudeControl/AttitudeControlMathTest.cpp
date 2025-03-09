#include <gtest/gtest.h>

#include "AttitudeControlMath.hpp"

using namespace matrix;
using namespace AttitudeControlMath;

static const Vector3f z_unit(0.f, 0.f, 1.f);

TEST(AttitudeControlMath, tiltCorrectionNoError)
{
	// 给定：一个期望的（不包含偏航旋转）倾斜设定值
	Quatf q_tilt_sp_ne(z_unit, Vector3f(-0.3, 0.1, 0.7));

	// 以及：一个期望的偏航设定值
	const Quatf q_sp_yaw = AxisAnglef(z_unit, -1.23f);

	// 当：当前的偏航误差为零（无论倾斜角度如何）
	const Quatf q = q_sp_yaw * Quatf(z_unit, Vector3f(0.1f, -0.2f, 1.f));
	const Quatf q_tilt_sp_ne_before = q_tilt_sp_ne;
	correctTiltSetpointForYawError(q_tilt_sp_ne, q, q_sp_yaw);

	// 那么：倾斜设定值保持不变
	EXPECT_TRUE(isEqual(q_tilt_sp_ne_before, q_tilt_sp_ne));
}

TEST(AttitudeControlMath, tiltCorrectionYaw180)
{
	// 给定：一个期望的（不包含偏航旋转）倾斜设定值和一个期望的偏航设定值
	Quatf q_tilt_sp_ne(z_unit, Vector3f(-0.3, 0.1, 0.7));
	const Quatf q_sp_yaw = AxisAnglef(z_unit, -M_PI_F / 2.f);

	// 当：存在180度的偏航误差
	const Quatf q_yaw = Quatf(AxisAnglef(z_unit, M_PI_F / 2.f));
	const Quatf q = q_yaw * Quatf(z_unit, Vector3f(0.1f, -0.2f, 1.f));
	const Quatf q_tilt_sp_ne_before = q_tilt_sp_ne;
	correctTiltSetpointForYawError(q_tilt_sp_ne, q, q_sp_yaw);

	// 那么：倾斜方向反转（修正后的倾斜角度相同，但旋转轴相反）
	EXPECT_FLOAT_EQ(AxisAnglef(q_tilt_sp_ne_before).angle(), AxisAnglef(q_tilt_sp_ne).angle());
	EXPECT_TRUE(isEqual(AxisAnglef(q_tilt_sp_ne_before).axis(), -AxisAnglef(q_tilt_sp_ne).axis()));
}

TEST(AttitudeControlMath, tiltCorrection)
{
	// 给定：一个期望的（不包含偏航旋转）倾斜设定值和一个期望的偏航设定值
	Quatf q_tilt_sp_ne(z_unit, Vector3f(0.5, -0.1, 0.7));
	const Quatf q_sp_yaw = AxisAnglef(z_unit, -1.23f);

	// 当：存在一定的偏航误差
	const Quatf q_yaw = Quatf(AxisAnglef(z_unit, 3.1f));
	const Quatf q = q_yaw * Quatf(z_unit, Vector3f(0.1f, -0.2f, 1.f));
	const Quatf q_tilt_sp_ne_before = q_tilt_sp_ne;
	correctTiltSetpointForYawError(q_tilt_sp_ne, q, q_sp_yaw);

	// 那么：通过偏航设定值旋转修正后的倾斜所得到的倾斜向量与
	// 通过当前车辆偏航角度旋转初始倾斜所得到的倾斜向量相同
	EXPECT_TRUE(isEqual((q_sp_yaw * q_tilt_sp_ne).dcm_z(), (q_yaw * q_tilt_sp_ne_before).dcm_z()));
}
