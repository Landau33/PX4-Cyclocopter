#include "Takeoff.hpp"
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>

void TakeoffHandling::generateInitialRampValue(float velocity_p_gain)
{
	velocity_p_gain = math::max(velocity_p_gain, 0.01f); // 确保速度比例增益不小于最小值
	_takeoff_ramp_vz_init = -CONSTANTS_ONE_G / velocity_p_gain; // 计算初始起飞斜坡的垂直速度
}

void TakeoffHandling::updateTakeoffState(const bool armed, const bool landed, const bool want_takeoff,
		const float takeoff_desired_vz, const bool skip_takeoff, const hrt_abstime &now_us)
{
	_spoolup_time_hysteresis.set_state_and_update(armed, now_us); // 更新预热时间滞后状态

	switch (_takeoff_state) {
	case TakeoffState::disarmed: // 未武装状态
		if (armed) { // 如果武装了，则进入预热状态
			_takeoff_state = TakeoffState::spoolup;
		} else {
			break;
		}

	// 直接跳转到下一个状态，无需 break
	case TakeoffState::spoolup: // 预热状态
		if (_spoolup_time_hysteresis.get_state()) { // 如果预热时间满足条件，则准备起飞
			_takeoff_state = TakeoffState::ready_for_takeoff;
		} else {
			break;
		}

	// 直接跳转到下一个状态，无需 break
	case TakeoffState::ready_for_takeoff: // 准备起飞状态
		if (want_takeoff) { // 如果需要起飞，则进入爬升斜坡状态
			_takeoff_state = TakeoffState::rampup;
			_takeoff_ramp_progress = 0.f; // 初始化爬升进度
		} else {
			break;
		}

	// 直接跳转到下一个状态，无需 break
	case TakeoffState::rampup: // 爬升斜坡状态
		if (_takeoff_ramp_progress >= 1.f) { // 如果爬升进度完成，则进入飞行状态
			_takeoff_state = TakeoffState::flight;
		} else {
			break;
		}

	// 直接跳转到下一个状态，无需 break
	case TakeoffState::flight: // 飞行状态
		if (landed) { // 如果着陆，则回到准备起飞状态
			_takeoff_state = TakeoffState::ready_for_takeoff;
		}
		break;

	default:
		break;
	}

	if (armed && skip_takeoff) { // 如果武装且跳过起飞，则直接进入飞行状态
		_takeoff_state = TakeoffState::flight;
	}

	// TODO: 需要考虑自由落体情况
	if (!armed) { // 如果未武装，则回到未武装状态
		_takeoff_state = TakeoffState::disarmed;
	}
}

float TakeoffHandling::updateRamp(const float dt, const float takeoff_desired_vz)
{
	float upwards_velocity_limit = takeoff_desired_vz; // 默认上升速度限制为期望的起飞垂直速度

	if (_takeoff_state < TakeoffState::rampup) { // 如果还未进入爬升斜坡状态
		upwards_velocity_limit = _takeoff_ramp_vz_init; // 使用初始起飞斜坡的垂直速度
	}

	if (_takeoff_state == TakeoffState::rampup) { // 如果处于爬升斜坡状态
		if (_takeoff_ramp_time > dt) { // 如果剩余爬升时间大于当前时间步长
			_takeoff_ramp_progress += dt / _takeoff_ramp_time; // 更新爬升进度

		} else {
			_takeoff_ramp_progress = 1.f; // 爬升进度已完成
		}

		if (_takeoff_ramp_progress < 1.f) { // 如果爬升进度未完成
			upwards_velocity_limit = _takeoff_ramp_vz_init + _takeoff_ramp_progress * (takeoff_desired_vz - _takeoff_ramp_vz_init);
			// 根据爬升进度线性插值计算上升速度限制
		}
	}

	return upwards_velocity_limit; // 返回最终的上升速度限制
}
