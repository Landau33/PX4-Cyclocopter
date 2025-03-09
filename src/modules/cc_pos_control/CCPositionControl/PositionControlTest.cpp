#include <gtest/gtest.h>
#include <PositionControl.hpp>
#include <px4_defines.h>

using namespace matrix;

TEST(PositionControlTest, EmptySetpoint)
{
	PositionControl position_control;

	vehicle_local_position_setpoint_s output_setpoint{};
	position_control.getLocalPositionSetpoint(output_setpoint);
	EXPECT_FLOAT_EQ(output_setpoint.x, 0.f);
	EXPECT_FLOAT_EQ(output_setpoint.y, 0.f);
	EXPECT_FLOAT_EQ(output_setpoint.z, 0.f);
	EXPECT_FLOAT_EQ(output_setpoint.yaw, 0.f);
	EXPECT_FLOAT_EQ(output_setpoint.yawspeed, 0.f);
	EXPECT_FLOAT_EQ(output_setpoint.vx, 0.f);
	EXPECT_FLOAT_EQ(output_setpoint.vy, 0.f);
	EXPECT_FLOAT_EQ(output_setpoint.vz, 0.f);
	EXPECT_EQ(Vector3f(output_setpoint.acceleration), Vector3f(0.f, 0.f, 0.f));
	EXPECT_EQ(Vector3f(output_setpoint.thrust), Vector3f(0, 0, 0));

	vehicle_attitude_setpoint_s attitude{};
	position_control.getAttitudeSetpoint(attitude);
	EXPECT_FLOAT_EQ(attitude.roll_body, 0.f);
	EXPECT_FLOAT_EQ(attitude.pitch_body, 0.f);
	EXPECT_FLOAT_EQ(attitude.yaw_body, 0.f);
	EXPECT_FLOAT_EQ(attitude.yaw_sp_move_rate, 0.f);
	EXPECT_EQ(Quatf(attitude.q_d), Quatf(1.f, 0.f, 0.f, 0.f));
	EXPECT_EQ(Vector3f(attitude.thrust_body), Vector3f(0.f, 0.f, 0.f));
	EXPECT_EQ(attitude.reset_integral, false);
	EXPECT_EQ(attitude.fw_control_yaw_wheel, false);
}

class PositionControlBasicTest : public ::testing::Test
{
public:
	PositionControlBasicTest()
	{
		_position_control.setPositionGains(Vector3f(1.f, 1.f, 1.f));
		_position_control.setVelocityGains(Vector3f(20.f, 20.f, 20.f), Vector3f(20.f, 20.f, 20.f), Vector3f(20.f, 20.f, 20.f));
		_position_control.setVelocityLimits(1.f, 1.f, 1.f);
		_position_control.setThrustLimits(0.1f, MAXIMUM_THRUST);
		_position_control.setHorizontalThrustMargin(HORIZONTAL_THRUST_MARGIN);
		_position_control.setTiltLimit(1.f);
		_position_control.setHoverThrust(.5f);
	}

	bool runController()
	{
		_position_control.setInputSetpoint(_input_setpoint);
		const bool ret = _position_control.update(.1f);
		_position_control.getLocalPositionSetpoint(_output_setpoint);
		_position_control.getAttitudeSetpoint(_attitude);
		return ret;
	}

	PositionControl _position_control;
	trajectory_setpoint_s _input_setpoint{PositionControl::empty_trajectory_setpoint};
	vehicle_local_position_setpoint_s _output_setpoint{};
	vehicle_attitude_setpoint_s _attitude{};

	static constexpr float MAXIMUM_THRUST = 0.9f;
	static constexpr float HORIZONTAL_THRUST_MARGIN = 0.3f;
};

class PositionControlBasicDirectionTest : public PositionControlBasicTest
{
public:
	void checkDirection()
	{
		Vector3f thrust(_output_setpoint.thrust);
		EXPECT_GT(thrust(0), 0.f);
		EXPECT_GT(thrust(1), 0.f);
		EXPECT_LT(thrust(2), 0.f);

		Vector3f body_z = Quatf(_attitude.q_d).dcm_z();
		EXPECT_LT(body_z(0), 0.f);
		EXPECT_LT(body_z(1), 0.f);
		EXPECT_GT(body_z(2), 0.f);
	}
};

TEST_F(PositionControlBasicDirectionTest, PositionDirection)
{
	Vector3f(.1f, .1f, -.1f).copyTo(_input_setpoint.position);
	EXPECT_TRUE(runController());
	checkDirection();
}

TEST_F(PositionControlBasicDirectionTest, VelocityDirection)
{
	Vector3f(.1f, .1f, -.1f).copyTo(_input_setpoint.velocity);
	EXPECT_TRUE(runController());
	checkDirection();
}

TEST_F(PositionControlBasicTest, TiltLimit)
{
	Vector3f(10.f, 10.f, 0.f).copyTo(_input_setpoint.position);

	EXPECT_TRUE(runController());
	Vector3f body_z = Quatf(_attitude.q_d).dcm_z();
	float angle = acosf(body_z.dot(Vector3f(0.f, 0.f, 1.f)));
	EXPECT_GT(angle, 0.f);
	EXPECT_LE(angle, 1.f);

	_position_control.setTiltLimit(0.5f);
	EXPECT_TRUE(runController());
	body_z = Quatf(_attitude.q_d).dcm_z();
	angle = acosf(body_z.dot(Vector3f(0.f, 0.f, 1.f)));
	EXPECT_GT(angle, 0.f);
	EXPECT_LE(angle, .50001f);

	_position_control.setTiltLimit(1.f);  // 恢复原始值
}

TEST_F(PositionControlBasicTest, VelocityLimit)
{
	Vector3f(10.f, 10.f, -10.f).copyTo(_input_setpoint.position);

	EXPECT_TRUE(runController());
	Vector2f velocity_xy(_output_setpoint.vx, _output_setpoint.vy);
	EXPECT_LE(velocity_xy.norm(), 1.f);
	EXPECT_LE(abs(_output_setpoint.vz), 1.f);
}

TEST_F(PositionControlBasicTest, PositionControlMaxThrustLimit)
{
	// 给定一个将控制器驱动到垂直和水平饱和的设定点
	Vector3f(10.f, 10.f, -10.f).copyTo(_input_setpoint.position);

	// 运行一次迭代
	runController();
	Vector3f thrust(_output_setpoint.thrust);

	// 推力向量长度由最大值限制
	EXPECT_FLOAT_EQ(thrust.norm(), MAXIMUM_THRUST);

	// 水平推力由其裕度限制
	EXPECT_FLOAT_EQ(thrust(0), HORIZONTAL_THRUST_MARGIN / sqrt(2.f));
	EXPECT_FLOAT_EQ(thrust(1), HORIZONTAL_THRUST_MARGIN / sqrt(2.f));
	EXPECT_FLOAT_EQ(thrust(2),
			-sqrt(MAXIMUM_THRUST * MAXIMUM_THRUST - HORIZONTAL_THRUST_MARGIN * HORIZONTAL_THRUST_MARGIN));
	thrust.print();

	// 总推力由最大值限制
	EXPECT_EQ(_attitude.thrust_body[0], 0.f);
	EXPECT_EQ(_attitude.thrust_body[1], 0.f);
	EXPECT_FLOAT_EQ(_attitude.thrust_body[2], -MAXIMUM_THRUST);

	// 水平裕度导致倾斜角度为：水平裕度 / 最大推力
	EXPECT_FLOAT_EQ(_attitude.roll_body, asin((HORIZONTAL_THRUST_MARGIN / sqrt(2.f)) / MAXIMUM_THRUST));
	// TODO: 在姿态设定点生成策略不再总是将机体偏航角与航向对齐时，添加此行
	// EXPECT_FLOAT_EQ(_attitude.pitch_body, -asin((HORIZONTAL_THRUST_MARGIN / sqrt(2.f)) / MAXIMUM_THRUST));
}

TEST_F(PositionControlBasicTest, PositionControlMinThrustLimit)
{
	Vector3f(10.f, 0.f, 10.f).copyTo(_input_setpoint.position);

	runController();
	Vector3f thrust(_output_setpoint.thrust);
	EXPECT_FLOAT_EQ(thrust.length(), 0.1f);

	EXPECT_FLOAT_EQ(_attitude.thrust_body[2], -0.1f);

	EXPECT_FLOAT_EQ(_attitude.roll_body, 0.f);
	EXPECT_FLOAT_EQ(_attitude.pitch_body, -1.f);
}

TEST_F(PositionControlBasicTest, FailsafeInput)
{
	_input_setpoint.acceleration[0] = _input_setpoint.acceleration[1] = 0.f;
	_input_setpoint.velocity[2] = .1f;

	EXPECT_TRUE(runController());
	EXPECT_FLOAT_EQ(_attitude.thrust_body[0], 0.f);
	EXPECT_FLOAT_EQ(_attitude.thrust_body[1], 0.f);
	EXPECT_LT(_output_setpoint.thrust[2], -.1f);
	EXPECT_GT(_output_setpoint.thrust[2], -.5f);
	EXPECT_GT(_attitude.thrust_body[2], -.5f);
	EXPECT_LE(_attitude.thrust_body[2], -.1f);
}

TEST_F(PositionControlBasicTest, IdleThrustInput)
{
	// 高向下的加速度以确保没有推力
	Vector3f(0.f, 0.f, 100.f).copyTo(_input_setpoint.acceleration);

	EXPECT_TRUE(runController());
	EXPECT_FLOAT_EQ(_output_setpoint.thrust[0], 0.f);
	EXPECT_FLOAT_EQ(_output_setpoint.thrust[1], 0.f);
	EXPECT_FLOAT_EQ(_output_setpoint.thrust[2], -.1f); // 最小推力
}

TEST_F(PositionControlBasicTest, InputCombinationsPosition)
{
	Vector3f(.1f, .2f, .3f).copyTo(_input_setpoint.position);

	EXPECT_TRUE(runController());
	EXPECT_FLOAT_EQ(_output_setpoint.x, .1f);
	EXPECT_FLOAT_EQ(_output_setpoint.y, .2f);
	EXPECT_FLOAT_EQ(_output_setpoint.z, .3f);
	EXPECT_FALSE(isnan(_output_setpoint.vx));
	EXPECT_FALSE(isnan(_output_setpoint.vy));
	EXPECT_FALSE(isnan(_output_setpoint.vz));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[0]));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[1]));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[2]));
}

TEST_F(PositionControlBasicTest, InputCombinationsPositionVelocity)
{
	_input_setpoint.velocity[0] = .1f;
	_input_setpoint.velocity[1] = .2f;
	_input_setpoint.position[2] = .3f; // 高度

	EXPECT_TRUE(runController());
	EXPECT_TRUE(isnan(_output_setpoint.x));
	EXPECT_TRUE(isnan(_output_setpoint.y));
	EXPECT_FLOAT_EQ(_output_setpoint.z, .3f);
	EXPECT_FLOAT_EQ(_output_setpoint.vx, .1f);
	EXPECT_FLOAT_EQ(_output_setpoint.vy, .2f);
	EXPECT_FALSE(isnan(_output_setpoint.vz));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[0]));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[1]));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[2]));
}

TEST_F(PositionControlBasicTest, SetpointValiditySimple)
{
	EXPECT_FALSE(runController());
	_input_setpoint.position[0] = .1f;
	EXPECT_FALSE(runController());
	_input_setpoint.position[1] = .2f;
	EXPECT_FALSE(runController());
	_input_setpoint.acceleration[2] = .3f;
	EXPECT_TRUE(runController());
}

TEST_F(PositionControlBasicTest, SetpointValidityAllCombinations)
{
	// 该测试运行所有组合的设定点和未设定（NAN）设定点，并检查它们是否被正确接受或拒绝
	float *const setpoint_loop_access_map[] = {&_input_setpoint.position[0], &_input_setpoint.velocity[0], &_input_setpoint.acceleration[0],
						   &_input_setpoint.position[1], &_input_setpoint.velocity[1], &_input_setpoint.acceleration[1],
						   &_input_setpoint.position[2], &_input_setpoint.velocity[2], &_input_setpoint.acceleration[2]
						  };

	for (int combination = 0; combination < 512; combination++) {
		_input_setpoint = PositionControl::empty_trajectory_setpoint;

		for (int j = 0; j < 9; j++) {
			if (combination & (1 << j)) {
				// 设置任意有限值，一些值明显达到极限以检查这些边界情况组合
				*(setpoint_loop_access_map[j]) = static_cast<float>(combination) / static_cast<float>(j + 1);
			}
		}

		// 期望每轴至少有一个设定点
		const bool has_x_setpoint = ((combination & 7) != 0);
		const bool has_y_setpoint = (((combination >> 3) & 7) != 0);
		const bool has_z_setpoint = (((combination >> 6) & 7) != 0);
		// 期望 xy 设定点成对出现
		const bool has_xy_pairs = (combination & 7) == ((combination >> 3) & 7);
		const bool expected_result = has_x_setpoint && has_y_setpoint && has_z_setpoint && has_xy_pairs;

		EXPECT_EQ(runController(), expected_result) << "组合 " << combination << std::endl
				<< "输入" << std::endl
				<< "位置     " << _input_setpoint.position[0] << ", "
				<< _input_setpoint.position[1] << ", " << _input_setpoint.position[2] << std::endl
				<< "速度     " << _input_setpoint.velocity[0] << ", "
				<< _input_setpoint.velocity[1] << ", " << _input_setpoint.velocity[2] << std::endl
				<< "加速度   " << _input_setpoint.acceleration[0] << ", "
				<< _input_setpoint.acceleration[1] << ", " << _input_setpoint.acceleration[2] << std::endl
				<< "输出" << std::endl
				<< "位置     " << _output_setpoint.x << ", " << _output_setpoint.y << ", " << _output_setpoint.z << std::endl
				<< "速度     " << _output_setpoint.vx << ", " << _output_setpoint.vy << ", " << _output_setpoint.vz << std::endl
				<< "加速度   " << _output_setpoint.acceleration[0] << ", "
				<< _output_setpoint.acceleration[1] << ", " << _output_setpoint.acceleration[2] << std::endl;
	}
}

TEST_F(PositionControlBasicTest, InvalidState)
{
	Vector3f(.1f, .2f, .3f).copyTo(_input_setpoint.position);

	PositionControlStates states{};
	states.position(0) = NAN;
	_position_control.setState(states);
	EXPECT_FALSE(runController());

	states.velocity(0) = NAN;
	_position_control.setState(states);
	EXPECT_FALSE(runController());

	states.position(0) = 0.f;
	_position_control.setState(states);
	EXPECT_FALSE(runController());

	states.velocity(0) = 0.f;
	states.acceleration(1) = NAN;
	_position_control.setState(states);
	EXPECT_FALSE(runController());
}

TEST_F(PositionControlBasicTest, UpdateHoverThrust)
{
	// 给定：一些悬停推力和 0 速度变化
	const float hover_thrust = 0.6f;
	_position_control.setHoverThrust(hover_thrust);

	Vector3f(0.f, 0.f, 0.f).copyTo(_input_setpoint.velocity);

	// 当我们运行控制器时
	EXPECT_TRUE(runController());

	// 输出推力等于悬停推力
	EXPECT_EQ(_output_setpoint.thrust[2], -hover_thrust);

	// 但是当我们通过 update 函数设置新的悬停推力时
	const float hover_thrust_new = 0.7f;
	_position_control.updateHoverThrust(hover_thrust_new);
	EXPECT_TRUE(runController());

	// 积分器更新以避免不连续性
	// 输出仍然相同
	EXPECT_EQ(_output_setpoint.thrust[2], -hover_thrust);
}

TEST_F(PositionControlBasicTest, IntegratorWindupWithInvalidSetpoint)
{
	// 给定：控制器运行了一个包含一些有效值的无效设定点
	_input_setpoint.position[0] = .1f;
	_input_setpoint.position[1] = .2f;
	// 所有 z 轴设定点保持为 NAN
	EXPECT_FALSE(runController());

	// 当我们使用有效的设定点运行控制器时
	_input_setpoint = PositionControl::empty_trajectory_setpoint;
	Vector3f(0.f, 0.f, 0.f).copyTo(_input_setpoint.velocity);
	EXPECT_TRUE(runController());

	// 积分器没有产生意外偏差
	EXPECT_FLOAT_EQ(_attitude.roll_body, 0.f);
	EXPECT_FLOAT_EQ(_attitude.pitch_body, 0.f);
}
