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
	// 设定姿态控制参数
	_attitude_control.setProportionalGain(Vector3f(_param_cc_roll_p.get(), _param_cc_pitch_p.get(), _param_cc_yaw_p.get()),
					      _param_cc_yaw_weight.get());
	// 角速度限制
	using math::radians;
	_attitude_control.setRateLimit(Vector3f(radians(_param_cc_roll_rate_max.get()), radians(_param_cc_pitch_rate_max.get()),
						radians(_param_cc_yaw_rate_max.get())));

	_man_tilt_max = math::radians(_param_cpc_man_tilt_max.get());
}

float CyclocopterAttitudeControl::throttle_curve(float throttle_stick_input)
{
	float thrust = 0.f;
	switch (_param_cpc_thr_curve.get()) {
	case 1: // 线性插值计算油门 float interpolate(float input, float in_min, float in_max, float out_min, float out_max)
		thrust = math::interpolate(throttle_stick_input, -1.f, 1.f,
					   _manual_throttle_minimum.getState(), _param_cpc_thr_max.get());
		break;
	default: // 多点插值计算油门 根据输入范围在min~hover或hover~max间插值计算 使得油门归中时悬停
		thrust = math::interpolateNXY(throttle_stick_input, {-1.f, 0.f, 1.f},
		{_manual_throttle_minimum.getState(), _param_cpc_thr_hover.get(), _param_cpc_thr_max.get()});
		break;
	}
	return math::min(thrust, _manual_throttle_maximum.getState());
}

void CyclocopterAttitudeControl::generate_attitude_setpoint(const Quatf &q, float dt, bool reset_yaw_sp)
{
	vehicle_attitude_setpoint_s attitude_setpoint{};
	const float yaw = Eulerf(q).psi();

	attitude_setpoint.yaw_sp_move_rate = _manual_control_setpoint.yaw * math::radians(_param_cpc_yaw_rate_max.get());

	// 重置解锁时的摇杆输出导致的yaw变化
	if ((_manual_control_setpoint.throttle < -0.9f) && (_param_mc_airmode.get() != 2)) {
		reset_yaw_sp = true;
	}

	if (reset_yaw_sp) {
		_man_yaw_sp = yaw;

	} else {
		// 根据角速度计算当前yaw角度
		_man_yaw_sp = wrap_pi(_man_yaw_sp + attitude_setpoint.yaw_sp_move_rate * dt);
	}

	_man_roll_input_filter.setParameters(dt, _param_cc_man_tilt_tau.get());
	_man_pitch_input_filter.setParameters(dt, _param_cc_man_tilt_tau.get());
	// 运动方向为(roll, pitch),俯仰角输入与运动方向相反
	Vector2f v = Vector2f(_man_roll_input_filter.update(_manual_control_setpoint.roll * _man_tilt_max),
			     -_man_pitch_input_filter.update(_manual_control_setpoint.pitch * _man_tilt_max));
	// 向量模长为倾斜角
	float v_norm = v.norm();

	if (v_norm > _man_tilt_max) {
		v *= _man_tilt_max / v_norm;
	}

	// 轴角也可以改变偏航（在较大倾斜角时明显）。
	// 偏航变化的公式如下：
	// 设 a := 倾斜角, b := atan(y/x) (最大倾斜方向)
	// yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2)))
	// 合成roll和pitch轴角四元数
	Quatf q_sp_rp = AxisAnglef(v(0), v(1), 0.f);
	// 定义yaw轴角四元数
	const Quatf q_sp_yaw(cosf(_man_yaw_sp / 2.f), 0.f, 0.f, sinf(_man_yaw_sp / 2.f));
	// 合成最终姿态四元数
	Quatf q_sp = q_sp_yaw * q_sp_rp;
	q_sp.copyTo(attitude_setpoint.q_d);

	// 仅用于日志记录的欧拉角转换
	const Eulerf euler_sp(q_sp);
	attitude_setpoint.roll_body = euler_sp(0);
	attitude_setpoint.pitch_body = euler_sp(1);
	attitude_setpoint.yaw_body = euler_sp(2);
	// 计算z轴推力 此处向上为正
	attitude_setpoint.thrust_body[2] = -throttle_curve(_manual_control_setpoint.throttle);

	attitude_setpoint.timestamp = hrt_absolute_time();
	_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
}

void CyclocopterAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}
	perf_begin(_loop_perf);

	// 检查参数更新状态
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&pupdate);
		parameters_updated();
	}

	vehicle_attitude_s v_att;

	if (_vehicle_attitude_sub.update(&v_att)) {
		const float dt = math::constrain(((v_att.timestamp - _last_run) * 1e-6f), 0.0002f, 0.02f);
		_last_run = v_att.timestamp_sample;

		const Quatf q{v_att.q};

		_manual_control_setpoint_sub.update(&_manual_control_setpoint);
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);
		// 检测怠速状态
		if (_vehicle_status_sub.updated()) {
			vehicle_status_s vehicle_status;

			if (_vehicle_status_sub.copy(&vehicle_status)) {
				const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
				_spooled_up = armed && hrt_elapsed_time(&vehicle_status.armed_time) > _param_com_spoolup_time.get() * 1_s;
			}
		}
		// 检测着陆状态
		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
			}
		}
		// 检测位置数据
		if (_vehicle_local_position_sub.updated()) {
			vehicle_local_position_s vehicle_local_position;
			if (_vehicle_local_position_sub.copy(&vehicle_local_position)) {
				// 标识当前航向角数据是否可靠
				_heading_good_for_control = vehicle_local_position.heading_good_for_control;
			}
		}

		bool attitude_setpoint_generated = false;
		const bool run_att_ctrl = _vehicle_control_mode.flag_control_attitude_enabled;

		if (run_att_ctrl) {
			// 手动模式和自稳模式下根据摇杆输入生成姿态设定点
			if (_vehicle_control_mode.flag_control_manual_enabled &&
			   !_vehicle_control_mode.flag_control_altitude_enabled &&
			   !_vehicle_control_mode.flag_control_velocity_enabled &&
			   !_vehicle_control_mode.flag_control_position_enabled) {
				generate_attitude_setpoint(q, dt, _reset_yaw_sp);
				attitude_setpoint_generated = true;
			} else {
				_man_roll_input_filter.reset(0.f);
				_man_pitch_input_filter.reset(0.f);
			}
			// 更新姿态设定点
			if (_vehicle_attitude_setpoint_sub.updated()) {
				vehicle_attitude_setpoint_s attitude_setpoint;

				if (_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint) &&
				    (vehicle_attitude_setpoint.timestamp > _last_attitude_setpoint)) {
					_attitude_control.setAttitudeSetpoint(Quatd(vehicle_attitude_setpoint.q_d),
									vehicle_attitude_setpoint.yaw_sp_move_rate);
					_thrust_setpoint_body = Vector3f(vehicle_attitude_setpoint.thrust_body);
					_last_attitude_setpoint = vehicle_attitude_setpoint.timestamp;
				}
			}
			// 检测航向重置
			if (_quat_reset_counter != v_att.quat_reset_counter) {
				const Quatf delta_q_reset(v_att.delta_q_reset);
				// 用四元数中航向变化量更新偏航设定点
				_man_yaw_sp = wrap_pi(_man_yaw_sp + Eulerf(delta_q_reset).psi());

				if (v_att.timestamp > _last_attitude_setpoint) {
					_attitude_control.adaptAttitudeSetpoint(delta_q_reset);
				}

				_quat_reset_counter = v_att.quat_reset_counter;
			}

			Vector3f rates_sp = _attitude_control.update(q);
			const hrt_abstime now = hrt_absolute_time();
			autotune_attitude_control_status_s pid_autotune;

			// 根据autotune状态更新速率设定点
			if (_autotune_attitude_control_status_sub.copy(&pid_autotune)) {
				if ((pid_autotune.state == autotune_attitude_control_status_s::STATE_ROLL
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_PITCH
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_YAW
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_TEST)
				    && ((now - pid_autotune.timestamp) < 1_s)) {
					rates_sp += Vector3f(pid_autotune.rate_sp);
				}
			}

			// 发布速率设定点
			vehicle_rates_setpoint_s rates_setpoint{};
			rates_setpoint.roll = rates_sp(0);
			rates_setpoint.pitch = rates_sp(1);
			rates_setpoint.yaw = rates_sp(2);
			_thrust_setpoint_body.copyTo(rates_setpoint.thrust_body);
			rates_setpoint.timestamp = hrt_absolute_time();
			_vehicle_rates_setpoint_pub.publish(rates_setpoint);
		}

		if (_landed) {
			_manual_throttle_minimum.update(0.f, dt);
		} else {
			_manual_throttle_minimum.update(_param_cpc_thr_min.get(), dt);
		}

		if (_spooled_up) {
			_manual_throttle_maximum.update(1.f, dt);
		} else {
			_manual_throttle_maximum.setForcedValue(0.f);
		}

		_reset_yaw_sp = !attitude_setpoint_generated || !_heading_good_for_control;
	}
	perf_end(_loop_perf);
}

int CyclocopterAttitudeControl::task_spawn(int argc, char *argv[])
{
	// 创建姿态控制任务实例
	CyclocopterAttitudeControl *instance = new CyclocopterAttitudeControl();

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

int CyclocopterAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CyclocopterAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
		### Description
		The controller has a P loop for angular error.
		It takes attitude setpoints (`vehicle_attitude_setpoint`) as inputs and outputs a rate setpoint.
		)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("cc_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

// 姿态控制应用程序启动/停止处理函数
extern "C" __EXPORT int cc_att_control_main(int argc, char *argv[])
{
	return CyclocopterAttitudeControl::main(argc, argv);
}
