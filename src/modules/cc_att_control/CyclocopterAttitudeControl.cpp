#include "CyclocopterAttitudeControl.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

#include "AttitudeControl/AttitudeControlMath.hpp"

using namespace matrix;

CyclocopterAttitudeControl::CyclocopterAttitudeControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
{
	parameters_updated();
	// 5%每秒的变化率 -> 1.6秒内线性变化到默认的8% CPC_THR_MIN
	_manual_throttle_minimum.setSlewRate(0.05f);
	// 50%每秒的变化率 -> 2秒内线性变化到100%
	_manual_throttle_maximum.setSlewRate(0.5f);
}

CyclocopterAttitudeControl::~CyclocopterAttitudeControl()
{
	perf_free(_loop_perf);
}

bool CyclocopterAttitudeControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void CyclocopterAttitudeControl::parameters_updated()
{

}
