#include <gtest/gtest.h>
#include <Takeoff.hpp>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>

TEST(TakeoffTest, Initialization)
{
	TakeoffHandling takeoff;
	// 检查初始化状态是否为未武装
	EXPECT_EQ(takeoff.getTakeoffState(), TakeoffState::disarmed);
}

TEST(TakeoffTest, RegularTakeoffRamp)
{
	TakeoffHandling takeoff;
	takeoff.setSpoolupTime(1.f); // 设置预热时间为 1 秒
	takeoff.setTakeoffRampTime(2.f); // 设置起飞斜坡时间为 2 秒
	takeoff.generateInitialRampValue(CONSTANTS_ONE_G / 0.5f); // 生成初始起飞斜坡值

	// 状态：未武装，着陆，不需要起飞
	takeoff.updateTakeoffState(false, true, false, 1.f, false, 0);
	EXPECT_EQ(takeoff.getTakeoffState(), TakeoffState::disarmed); // 预期状态仍为未武装

	// 状态：武装，不再着陆，但仍然不需要起飞
	takeoff.updateTakeoffState(true, false, false, 1.f, false, 500_ms);
	EXPECT_EQ(takeoff.getTakeoffState(), TakeoffState::spoolup); // 预期状态为预热

	// 状态：武装，不着陆，不需要起飞，预热时间已过
	takeoff.updateTakeoffState(true, false, false, 1.f, false, 2_s);
	EXPECT_EQ(takeoff.getTakeoffState(), TakeoffState::ready_for_takeoff); // 预期状态为准备起飞

	// 状态：武装，不着陆，需要起飞
	takeoff.updateTakeoffState(true, false, true, 1.f, false, 3_s);
	EXPECT_EQ(takeoff.getTakeoffState(), TakeoffState::rampup); // 预期状态为爬升斜坡

	// 状态：武装，不着陆，需要起飞，正在爬升斜坡
	takeoff.updateTakeoffState(true, false, true, 1.f, false, 4_s);
	EXPECT_FLOAT_EQ(takeoff.updateRamp(.5f, 1.5f), 0.f); // 初始上升速度限制为 0
	EXPECT_FLOAT_EQ(takeoff.updateRamp(.5f, 1.5f), .5f); // 第一次更新后上升速度限制为 0.5 m/s
	EXPECT_FLOAT_EQ(takeoff.updateRamp(.5f, 1.5f), 1.f); // 第二次更新后上升速度限制为 1 m/s
	EXPECT_FLOAT_EQ(takeoff.updateRamp(.5f, 1.5f), 1.5f); // 第三次更新后上升速度限制为 1.5 m/s
	EXPECT_FLOAT_EQ(takeoff.updateRamp(.5f, 1.5f), 1.5f); // 达到最终上升速度限制 1.5 m/s

	// 状态：武装，不着陆，需要起飞，爬升斜坡时间已过
	takeoff.updateTakeoffState(true, false, true, 1.f, false, 6500_ms);
	EXPECT_EQ(takeoff.getTakeoffState(), TakeoffState::flight); // 预期状态为飞行
}
