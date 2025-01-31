#include "CyclocopterPositionControl.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/events.h>
#include "PositionControl/ControlMath.hpp"


using namespace matrix;

CyclocopterPositionControl::CyclocopterPositionControl() :
	SuperBlock(nullptr, "CPC"),
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(ORB_ID(vehicle_attitude_setpoint)),
	_vel_x_deriv(this, "VELD"),
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD")
{
	parameters_update(true);
	_tilt_limit_slew_rate.setSlewRate(.2f);
	_takeoff_status_pub.advertise();
}

CyclocopterPositionControl::~CyclocopterPositionControl()
{
	perf_free(_cycle_perf);
}

bool CyclocopterPositionControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	return true;
}

void CyclocopterPositionControl::parameters_update(bool force)
{

}

