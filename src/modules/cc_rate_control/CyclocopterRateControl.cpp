#include "CyclocopterRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

CyclocopterRateControl::CyclocopterRateControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_vehicle_torque_setpoint_pub(ORB_ID(vehicle_torque_setpoint)),
	_vehicle_thrust_setpoint_pub(ORB_ID(vehicle_thrust_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	parameters_updated();
	_controller_status_pub.advertise();
}

CyclocopterRateControl::~CyclocopterRateControl()
{
	perf_free(_loop_perf);
}

bool CyclocopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed!");
		return false;
	}
	return true;
}

void CyclocopterRateControl::parameters_updated()
{
	const Vector3f rate_k = Vector3f(_param_cc_roll_rate_k.get(), _param_cc_pitch_rate_k.get(), _param_cc_yaw_rate_k.get());

	// 配置速率控制参数
	_rate_control.setPidGains(
		// a.emult(b)=Vector3 [a.xb.x, a.yb.y, a.z*b.z]
		rate_k.emult(Vector3f(_param_cc_roll_rate_p.get(), _param_cc_pitch_rate_p.get(), _param_cc_yaw_rate_p.get())),
		rate_k.emult(Vector3f(_param_cc_roll_rate_i.get(), _param_cc_pitch_rate_i.get(), _param_cc_yaw_rate_i.get())),
		rate_k.emult(Vector3f(_param_cc_roll_rate_d.get(), _param_cc_pitch_rate_d.get(), _param_cc_yaw_rate_d.get())));
	_rate_control.setIntegratorLimit(Vector3f(_param_cc_rr_int_lim.get(), _param_cc_pr_int_lim.get(), _param_cc_yr_int_lim.get()));
	_rate_control.setFeedForwardGain(Vector3f(_param_cc_roll_rate_ff.get(), _param_cc_pitch_rate_ff.get(), _param_cc_yaw_rate_ff.get()));
}

void CyclocopterRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// 检测参数更新
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
		parameters_updated();
	}

	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {
		const hrt_abstime now = angular_velocity.timestamp_sample;
		// 限制dt范围
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;
		// 定义角速度和角加速度
		const Vector3f rates{angular_velocity.xyz};
		const Vector3f angular_accel{angular_velocity.xyz_derivative};
		// 检测控制模式更新
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);
		// 检测落地
		if (_vehicle_land_detected_sub.update()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}
		// 检测状态更新
		_vehicle_status_sub.update(&_vehicle_status);

		vehicle_rates_setpoint_s vehicle_rates_setpoint{};
		// 检测速率设定点更新
		if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
			if (_vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint)) {
				_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
				_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
				_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
				_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
			}
		}
		// 运行速率控制器
		if (_vehicle_control_mode.flag_control_rates_enabled) {
			// 如果未解锁则重置积分器
			if (!_vehicle_control_mode.flag_armed) {
				_rate_control.resetIntegral();
			}

			control_allocator_status_s control_allocator_status;

			if (_control_allocator_status_sub.update(&control_allocator_status)) {
				Vector<bool, 3> saturation_positive;
				Vector<bool, 3> saturation_negative;
				// 检测力矩分配是否饱和
				if (!control_allocator_status.torque_setpoint_achieved) {
					for (size_t i = 0; i < 3; i++) {
						if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
							saturation_positive(i) = true;

						} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
							saturation_negative(i) = true;
						}
					}
				}
				_rate_control.setSaturationStatus(saturation_positive, saturation_negative);
			}

			// 运行速率控制器
			const Vector3f att_control = _rate_control.update(rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed);
			// 发布速率控制器状态
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			// 推力和力矩设定点
			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			_thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);
			vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.f;
			vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.f;
			vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.f;

			// 如果启用了电池缩放，则根据电池状态缩放推力和力矩设定点
			if (_param_cc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.f) {
					for (int i = 0; i < 3; i++) {
						vehicle_thrust_setpoint.xyz[i] = math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
						vehicle_torque_setpoint.xyz[i] = math::constrain(vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
					}
				}
			}

			// 发布推力和力矩设定点
			vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

			vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);

			updateActuatorControlsStatus(vehicle_torque_setpoint, dt);
		}
	}
	perf_end(_loop_perf);
}

void CyclocopterRateControl::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint, float dt)
{
	// 累积三轴扭矩的能量积分（扭矩平方的时间积分）
	for (int i = 0; i < 3; i++) {
		_control_energy[i] += vehicle_torque_setpoint.xyz[i] * vehicle_torque_setpoint.xyz[i] * dt;
	}

	_energy_integration_time += dt;

	// 当累计时间超过500ms时计算并发布控制功率
	if (_energy_integration_time > 500e-3f) {
		actuator_controls_status_s status;
		status.timestamp = vehicle_torque_setpoint.timestamp;

		// 计算平均功率并重置能量累计值
		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		// 发布控制状态并重置计时器
		_actuator_controls_status_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int CyclocopterRateControl::task_spawn(int argc, char *argv[])
{
	CyclocopterRateControl *instance = new CyclocopterRateControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int CyclocopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CyclocopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
		### Description
		The controller has a PID loop for angular rate error.

		It takes rate setpoints as inputs and outputs actuator control messages.

		)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("cc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int cc_rate_control_main(int argc, char *argv[])
{
	return CyclocopterRateControl::main(argc, argv);
}
