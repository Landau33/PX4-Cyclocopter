#include <cmath>
#include <gtest/gtest.h>
#include <ControlMath.hpp>
#include <px4_platform_common/defines.h>

using namespace matrix;
using namespace ControlMath;

TEST(ControlMathTest, LimitTiltUnchanged)
{
	Vector3f body = Vector3f(0.f, 0.f, 1.f).normalized();
	Vector3f body_before = body;
	limitTilt(body, Vector3f(0.f, 0.f, 1.f), M_DEG_TO_RAD_F * 45.f);
	EXPECT_EQ(body, body_before);

	body = Vector3f(0.f, .1f, 1.f).normalized();
	body_before = body;
	limitTilt(body, Vector3f(0.f, 0.f, 1.f), M_DEG_TO_RAD_F * 45.f);
	EXPECT_EQ(body, body_before);
}

TEST(ControlMathTest, LimitTiltOpposite)
{
	Vector3f body = Vector3f(0.f, 0.f, -1.f).normalized();
	limitTilt(body, Vector3f(0.f, 0.f, 1.f), M_DEG_TO_RAD_F * 45.f);
	float angle = acosf(body.dot(Vector3f(0.f, 0.f, 1.f)));
	EXPECT_NEAR(angle * M_RAD_TO_DEG_F, 45.f, 1e-4f);
	EXPECT_FLOAT_EQ(body.length(), 1.f);
}

TEST(ControlMathTest, LimitTiltAlmostOpposite)
{
	// 此情况不会触发特殊情况处理，但非常接近这种情况
	Vector3f body = Vector3f(0.001f, 0.f, -1.f).normalized();
	limitTilt(body, Vector3f(0.f, 0.f, 1.f), M_DEG_TO_RAD_F * 45.f);
	float angle = acosf(body.dot(Vector3f(0.f, 0.f, 1.f)));
	EXPECT_NEAR(angle * M_RAD_TO_DEG_F, 45.f, 1e-4f);
	EXPECT_FLOAT_EQ(body.length(), 1.f);
}

TEST(ControlMathTest, LimitTilt45degree)
{
	Vector3f body = Vector3f(1.f, 0.f, 0.f);
	limitTilt(body, Vector3f(0.f, 0.f, 1.f), M_DEG_TO_RAD_F * 45.f);
	EXPECT_EQ(body, Vector3f(M_SQRT1_2_F, 0, M_SQRT1_2_F));

	body = Vector3f(0.f, 1.f, 0.f);
	limitTilt(body, Vector3f(0.f, 0.f, 1.f), M_DEG_TO_RAD_F * 45.f);
	EXPECT_EQ(body, Vector3f(0.f, M_SQRT1_2_F, M_SQRT1_2_F));
}

TEST(ControlMathTest, LimitTilt10degree)
{
	Vector3f body = Vector3f(1.f, 1.f, .1f).normalized();
	limitTilt(body, Vector3f(0.f, 0.f, 1.f), M_DEG_TO_RAD_F * 10.f);
	float angle = acosf(body.dot(Vector3f(0.f, 0.f, 1.f)));
	EXPECT_NEAR(angle * M_RAD_TO_DEG_F, 10.f, 1e-4f);
	EXPECT_FLOAT_EQ(body.length(), 1.f);
	EXPECT_FLOAT_EQ(body(0), body(1));

	body = Vector3f(1, 2, .2f);
	limitTilt(body, Vector3f(0.f, 0.f, 1.f), M_DEG_TO_RAD_F * 10.f);
	angle = acosf(body.dot(Vector3f(0.f, 0.f, 1.f)));
	EXPECT_NEAR(angle * M_RAD_TO_DEG_F, 10.f, 1e-4f);
	EXPECT_FLOAT_EQ(body.length(), 1.f);
	EXPECT_FLOAT_EQ(2.f * body(0), body(1));
}

TEST(ControlMathTest, ThrottleAttitudeMapping)
{
	/* 预期：滚转为零，俯仰为零，偏航为零，推力大小为最大
	 * 原因：推力完全向上 */
	Vector3f thr{0.f, 0.f, -1.f};
	float yaw = 0.f;
	vehicle_attitude_setpoint_s att{};
	thrustToAttitude(thr, yaw, att);
	EXPECT_FLOAT_EQ(att.roll_body, 0.f);
	EXPECT_FLOAT_EQ(att.pitch_body, 0.f);
	EXPECT_FLOAT_EQ(att.yaw_body, 0.f);
	EXPECT_FLOAT_EQ(att.thrust_body[2], -1.f);

	/* 预期：与之前相同但偏航为90度
	 * 原因：只有偏航发生了变化 */
	yaw = M_PI_2_F;
	thrustToAttitude(thr, yaw, att);
	EXPECT_FLOAT_EQ(att.roll_body, 0.f);
	EXPECT_FLOAT_EQ(att.pitch_body, 0.f);
	EXPECT_FLOAT_EQ(att.yaw_body, M_PI_2_F);
	EXPECT_FLOAT_EQ(att.thrust_body[2], -1.f);

	/* 预期：与之前相同但滚转为180度
	 * 原因：推力指向正下方，欧拉角顺序是：1. 滚转，2. 俯仰，3. 偏航 */
	thr = Vector3f(0.f, 0.f, 1.f);
	thrustToAttitude(thr, yaw, att);
	EXPECT_FLOAT_EQ(att.roll_body, -M_PI_F);
	EXPECT_FLOAT_EQ(att.pitch_body, 0.f);
	EXPECT_FLOAT_EQ(att.yaw_body, M_PI_2_F);
	EXPECT_FLOAT_EQ(att.thrust_body[2], -1.f);
}

TEST(ControlMathTest, ConstrainXYPriorities)
{
	const float max = 5.f;
	// v0 已经达到最大值
	Vector2f v0(max, 0.f);
	Vector2f v1(v0(1), -v0(0));

	Vector2f v_r = constrainXY(v0, v1, max);
	EXPECT_FLOAT_EQ(v_r(0), max);
	EXPECT_FLOAT_EQ(v_r(1), 0.f);

	// v1 的模长超过最大值但 v0 是零
	v0.zero();
	v_r = constrainXY(v0, v1, max);
	EXPECT_FLOAT_EQ(v_r(1), -max);
	EXPECT_FLOAT_EQ(v_r(0), 0.f);

	v0 = Vector2f(.5f, .5f);
	v1 = Vector2f(.5f, -.5f);
	v_r = constrainXY(v0, v1, max);
	const float diff = Vector2f(v_r - (v0 + v1)).length();
	EXPECT_FLOAT_EQ(diff, 0.f);

	// v0 和 v1 超过最大值且垂直
	v0 = Vector2f(4.f, 0.f);
	v1 = Vector2f(0.f, -4.f);
	v_r = constrainXY(v0, v1, max);
	EXPECT_FLOAT_EQ(v_r(0), v0(0));
	EXPECT_GT(v_r(0), 0.f);
	const float remaining = sqrtf(max * max - (v0(0) * v0(0)));
	EXPECT_FLOAT_EQ(v_r(1), -remaining);
}

TEST(ControlMathTest, CrossSphereLine)
{
	/* 测试围绕航路点（o）的9个位置 (+)：
	 *
	 * 远处             +              +              +
	 *
	 * 附近            +              +              +
	 * 在轨迹上 --+----o---------+---------o----+--
	 *                    上一个                当前
	 *
	 * 预期目标 (1, 2, 3):
	 * 远处             +              +              +
	 *
	 *
	 * 在轨迹上 -------1---------2---------3-------
	 *
	*
	* 附近            +              +              +
	* 在轨迹上 -------o---1---------2-----3-------
	*
	*
	* 在轨迹上 --+----o----1----+--------2/3---+-- */
	const Vector3f prev = Vector3f(0.f, 0.f, 0.f);
	const Vector3f curr = Vector3f(0.f, 0.f, 2.f);
	Vector3f res;
	bool retval = false;

	// 在线上，靠近，位于前一个航路点之前
	retval = cross_sphere_line(Vector3f(0.f, 0.f, -.5f), 1.f, prev, curr, res);
	EXPECT_TRUE(retval);
	EXPECT_EQ(res, Vector3f(0.f, 0.f, 0.5f));

	// 在线上，靠近，位于当前航路点之前
	retval = cross_sphere_line(Vector3f(0.f, 0.f, 1.f), 1.f, prev, curr, res);
	EXPECT_TRUE(retval);
	EXPECT_EQ(res, Vector3f(0.f, 0.f, 2.f));

	// 在线上，靠近，位于当前航路点之后
	retval = cross_sphere_line(Vector3f(0.f, 0.f, 2.5f), 1.f, prev, curr, res);
	EXPECT_TRUE(retval);
	EXPECT_EQ(res, Vector3f(0.f, 0.f, 2.f));

	// 附近，位于前一个航路点之前
	retval = cross_sphere_line(Vector3f(0.f, .5f, -.5f), 1.f, prev, curr, res);
	EXPECT_TRUE(retval);
	EXPECT_EQ(res, Vector3f(0.f, 0.f, .366025388f));

	// 附近，位于当前航路点之前
	retval = cross_sphere_line(Vector3f(0.f, .5f, 1.f), 1.f, prev, curr, res);
	EXPECT_TRUE(retval);
	EXPECT_EQ(res, Vector3f(0.f, 0.f, 1.866025448f));

	// 附近，位于当前航路点之后
	retval = ControlMath::cross_sphere_line(matrix::Vector3f(0.f, .5f, 2.5f), 1.f, prev, curr, res);
	EXPECT_TRUE(retval);
	EXPECT_EQ(res, Vector3f(0.f, 0.f, 2.f));

	// 远处，位于前一个航路点之前
	retval = ControlMath::cross_sphere_line(matrix::Vector3f(0.f, 2.f, -.5f), 1.f, prev, curr, res);
	EXPECT_FALSE(retval);
	EXPECT_EQ(res, Vector3f());

	// 远处，位于当前航路点之前
	retval = ControlMath::cross_sphere_line(matrix::Vector3f(0.f, 2.f, 1.f), 1.f, prev, curr, res);
	EXPECT_FALSE(retval);
	EXPECT_EQ(res, Vector3f(0.f, 0.f, 1.f));

	// 远处，位于当前航路点之后
	retval = ControlMath::cross_sphere_line(matrix::Vector3f(0.f, 2.f, 2.5f), 1.f, prev, curr, res);
	EXPECT_FALSE(retval);
	EXPECT_EQ(res, Vector3f(0.f, 0.f, 2.f));
}

TEST(ControlMathTest, addIfNotNan)
{
	float v = 1.f;
	// 正常相加
	ControlMath::addIfNotNan(v, 2.f);
	EXPECT_EQ(v, 3.f);
	// 添加 NaN 不影响结果
	ControlMath::addIfNotNan(v, NAN);
	EXPECT_EQ(v, 3.f);
	v = NAN;
	// 两个操作数都是 NaN
	ControlMath::addIfNotNan(v, NAN);
	EXPECT_TRUE(std::isnan(v));
	// 正常值覆盖 NaN
	ControlMath::addIfNotNan(v, 3.f);
	EXPECT_EQ(v, 3.f);
}
