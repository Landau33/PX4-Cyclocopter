#include <gtest/gtest.h>
#include <AttitudeControl.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

TEST(AttitudeControlTest, AllZeroCase)
{
	// 测试：所有输入为零的情况
	AttitudeControl attitude_control;
	Vector3f rate_setpoint = attitude_control.update(Quatf());
	EXPECT_EQ(rate_setpoint, Vector3f()); // 期望输出也为零向量
}

class AttitudeControlConvergenceTest : public ::testing::Test
{
public:
	AttitudeControlConvergenceTest()
	{
		// 初始化姿态控制器的比例增益和速率限制
		_attitude_control.setProportionalGain(Vector3f(.5f, .6f, .3f), .4f);
		_attitude_control.setRateLimit(Vector3f(100.f, 100.f, 100.f));
	}

	void checkConvergence()
	{
		int i; // 需要在函数范围内检查迭代次数
		Vector3f rate_setpoint(1000.f, 1000.f, 1000.f);

		// 设置姿态设定值
		_attitude_control.setAttitudeSetpoint(_quat_goal, 0.f);

		for (i = 100; i > 0; i--) {
			// 运行姿态控制器以获取速率设定值
			const Vector3f rate_setpoint_new = _attitude_control.update(_quat_state);
			// 根据速率设定值旋转模拟状态四元数
			_quat_state = _quat_state * Quatf(AxisAnglef(rate_setpoint_new));
			_quat_state = -_quat_state; // 生成间歇性的反向四元数状态，以测试解绕问题

			// 期望每次迭代后误差减小，即输出也减小
			if (rate_setpoint_new.norm() >= rate_setpoint.norm()) {
				break;
			}

			rate_setpoint = rate_setpoint_new;
		}

		// 期望最终状态四元数与目标四元数一致（标准化后）
		EXPECT_EQ(_quat_state.canonical(), _quat_goal.canonical());
		// 期望收敛过程不超过最大迭代次数
		EXPECT_GT(i, 0);
	}

	AttitudeControl _attitude_control;
	Quatf _quat_state;
	Quatf _quat_goal;
};

TEST_F(AttitudeControlConvergenceTest, AttitudeControlConvergence)
{
	const int inputs = 8;

	const Quatf QArray[inputs] = {
		Quatf(),
		Quatf(0, 1, 0, 0),
		Quatf(0, 0, 1, 0),
		Quatf(0, 0, 0, 1),
		Quatf(0.698f, 0.024f, -0.681f, -0.220f),
		Quatf(-0.820f, -0.313f, 0.225f, -0.423f),
		Quatf(0.599f, -0.172f, 0.755f, -0.204f),
		Quatf(0.216f, -0.662f, 0.290f, -0.656f)
	};

	for (int i = 0; i < inputs; i++) {
		for (int j = 0; j < inputs; j++) {
			printf("--- Input combination: %d %d\n", i, j);
			_quat_state = QArray[i];
			_quat_goal = QArray[j];
			_quat_state.normalize();
			_quat_goal.normalize();
			checkConvergence();
		}
	}
}

TEST(AttitudeControlTest, YawWeightScaling)
{
	// 给定：默认调校参数和纯偏航转向命令
	AttitudeControl attitude_control;
	const float yaw_gain = 2.8f;
	const float yaw_sp = .1f;
	Quatf pure_yaw_attitude(cosf(yaw_sp / 2.f), 0, 0, sinf(yaw_sp / 2.f));
	attitude_control.setProportionalGain(Vector3f(6.5f, 6.5f, yaw_gain), .4f);
	attitude_control.setRateLimit(Vector3f(1000.f, 1000.f, 1000.f));
	attitude_control.setAttitudeSetpoint(pure_yaw_attitude, 0.f);

	// 当：运行一次控制器迭代
	Vector3f rate_setpoint = attitude_control.update(Quatf());

	// 那么：滚转和俯仰方向没有动作
	EXPECT_EQ(Vector2f(rate_setpoint), Vector2f());
	// 那么：偏航方向的动作等于误差乘以增益
	EXPECT_NEAR(rate_setpoint(2), yaw_sp * yaw_gain, 1e-4f);

	// 给定：额外的特殊情况，偏航权重为零
	attitude_control.setProportionalGain(Vector3f(6.5f, 6.5f, yaw_gain), 0.f);
	// 当：运行一次控制器迭代
	rate_setpoint = attitude_control.update(Quatf());
	// 那么：没有任何动作（也不应出现NaN）
	EXPECT_EQ(rate_setpoint, Vector3f());
}
