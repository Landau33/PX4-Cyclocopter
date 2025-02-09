#include "CyclocopterAttitudeControl.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

#include "AttitudeControl/AttitudeControlMath.hpp"

using namespace matrix;

CyclocopterAttitudeControl::CyclocopterAttitudeControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_vtol(vtol)
{
	parameters_updated();
	// 5%每秒的变化率 -> 1.6秒内线性变化到默认的8% MPC_MANTHR_MIN
	_manual_throttle_minimum.setSlewRate(0.05f);
	// 50%每秒的变化率 -> 2秒内线性变化到100%
	_manual_throttle_maximum.setSlewRate(0.5f);
}

CyclocopterAttitudeControl::~CyclocopterAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterAttitudeControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
MulticopterAttitudeControl::parameters_updated()
{
	// 存储一些参数以更方便的方式并预先计算常用的值
	_attitude_control.setProportionalGain(Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(), _param_mc_yaw_p.get()),
					      _param_mc_yaw_weight.get());

	// 角速度限制
	using math::radians;
	_attitude_control.setRateLimit(Vector3f(radians(_param_mc_rollrate_max.get()), radians(_param_mc_pitchrate_max.get()),
						radians(_param_mc_yawrate_max.get())));

	_man_tilt_max = math::radians(_param_mpc_man_tilt_max.get());
}

float
MulticopterAttitudeControl::throttle_curve(float throttle_stick_input)
{
	float thrust = 0.f;

	switch (_param_mpc_thr_curve.get()) {
	case 1: // 不重新缩放到悬停油门
		thrust = math::interpolate(throttle_stick_input, -1.f, 1.f,
					   _manual_throttle_minimum.getState(), _param_mpc_thr_max.get());
		break;

	default: // 0 或其他：重新缩放，使居中的油门杆对应悬停油门
		thrust = math::interpolateNXY(throttle_stick_input, {-1.f, 0.f, 1.f},
		{_manual_throttle_minimum.getState(), _param_mpc_thr_hover.get(), _param_mpc_thr_max.get()});
		break;
	}

	return math::min(thrust, _manual_throttle_maximum.getState());
}

void
MulticopterAttitudeControl::generate_attitude_setpoint(const Quatf &q, float dt, bool reset_yaw_sp)
{
	vehicle_attitude_setpoint_s attitude_setpoint{};
	const float yaw = Eulerf(q).psi();

	attitude_setpoint.yaw_sp_move_rate = _manual_control_setpoint.yaw * math::radians(_param_mpc_man_y_max.get());

	// 避免在武装手柄手势时累积绝对偏航误差（如果 heading_good_for_control 保持为真）
	if ((_manual_control_setpoint.throttle < -.9f) && (_param_mc_airmode.get() != 2)) {
		reset_yaw_sp = true;
	}

	// 确保不累积绝对航向误差
	if (reset_yaw_sp) {
		_man_yaw_sp = yaw;

	} else {
		_man_yaw_sp = wrap_pi(_man_yaw_sp + attitude_setpoint.yaw_sp_move_rate * dt);
	}

	/*
	 * 滚转和俯仰设定点的输入映射
	 * ----------------------------------------
	 * 我们控制以下两个角度：
	 * - 倾斜角，由 sqrt(roll*roll + pitch*pitch) 给出
	 * - XY 平面上的最大倾斜方向，也定义了运动方向
	 *
	 * 这样可以简单地限制倾斜角，飞机飞向操纵杆指向的方向，并且操纵杆输入的变化是线性的。
	 */
	_man_roll_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());
	_man_pitch_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());

	// 我们希望朝 (roll, pitch) 的方向飞行
	Vector2f v = Vector2f(_man_roll_input_filter.update(_manual_control_setpoint.roll * _man_tilt_max),
			      -_man_pitch_input_filter.update(_manual_control_setpoint.pitch * _man_tilt_max));
	float v_norm = v.norm(); // v 的范数定义了倾斜角

	if (v_norm > _man_tilt_max) { // 限制到配置的最大倾斜角
		v *= _man_tilt_max / v_norm;
	}

	Quatf q_sp_rp = AxisAnglef(v(0), v(1), 0.f);
	// 轴角也可以改变偏航（在较大倾斜角时明显）。
	// 偏航变化的公式如下：
	//   设 a := 倾斜角, b := atan(y/x) (最大倾斜方向)
	//   yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2))).
	const Quatf q_sp_yaw(cosf(_man_yaw_sp / 2.f), 0.f, 0.f, sinf(_man_yaw_sp / 2.f));

	if (_vtol) {
		// 修改滚转和俯仰设定点，使其反映用户意图，即使存在较大的偏航误差(yaw_sp - yaw)。
		// 在存在偏航误差的情况下，根据偏航设定点构造姿态设定点会导致用户意想不到的姿态行为，
		// 因为倾斜将不会与车辆的航向对齐。

		AttitudeControlMath::correctTiltSetpointForYawError(q_sp_rp, q, q_sp_yaw);
	}

	// 将期望的倾斜与偏航设定点对齐
	Quatf q_sp = q_sp_yaw * q_sp_rp;

	q_sp.copyTo(attitude_setpoint.q_d);

	// 仅用于日志记录的欧拉角转换
	const Eulerf euler_sp(q_sp);
	attitude_setpoint.roll_body = euler_sp(0);
	attitude_setpoint.pitch_body = euler_sp(1);
	attitude_setpoint.yaw_body = euler_sp(2);

	attitude_setpoint.thrust_body[2] = -throttle_curve(_manual_control_setpoint.throttle);

	attitude_setpoint.timestamp = hrt_absolute_time();
	_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
}

void
MulticopterAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// 检查参数是否已更改
	if (_parameter_update_sub.updated()) {
		// 清除更新
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	// 在姿态更新时运行控制器
	vehicle_attitude_s v_att;

	if (_vehicle_attitude_sub.update(&v_att)) {

		// 保护过小 (< 0.2ms) 和过大 (> 20ms) 的 dt。
		const float dt = math::constrain(((v_att.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
		_last_run = v_att.timestamp_sample;

		const Quatf q{v_att.q};

		/* 检查其他主题的更新 */
		_manual_control_setpoint_sub.update(&_manual_control_setpoint);
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if (_vehicle_status_sub.updated()) {
			vehicle_status_s vehicle_status;

			if (_vehicle_status_sub.copy(&vehicle_status)) {
				_vehicle_type_rotary_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
				_vtol = vehicle_status.is_vtol;
				_vtol_in_transition_mode = vehicle_status.in_transition_mode;
				_vtol_tailsitter = vehicle_status.is_vtol_tailsitter;

				const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
				_spooled_up = armed && hrt_elapsed_time(&vehicle_status.armed_time) > _param_com_spoolup_time.get() * 1_s;
			}
		}

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
			}
		}

		if (_vehicle_local_position_sub.updated()) {
			vehicle_local_position_s vehicle_local_position;

			if (_vehicle_local_position_sub.copy(&vehicle_local_position)) {
				_heading_good_for_control = vehicle_local_position.heading_good_for_control;
			}
		}

		bool attitude_setpoint_generated = false;

		const bool is_hovering = (_vehicle_type_rotary_wing && !_vtol_in_transition_mode);

		// 尾座式 VTOL 处于过渡模式
		const bool is_tailsitter_transition = (_vtol_tailsitter && _vtol_in_transition_mode);

		const bool run_att_ctrl = _vehicle_control_mode.flag_control_attitude_enabled && (is_hovering
					  || is_tailsitter_transition);

		if (run_att_ctrl) {

			// 如果处于手动/稳定模式，则根据操纵杆输入生成姿态设定点
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

			// 检查新的姿态设定点
			if (_vehicle_attitude_setpoint_sub.updated()) {
				vehicle_attitude_setpoint_s vehicle_attitude_setpoint;

				if (_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint)
				    && (vehicle_attitude_setpoint.timestamp > _last_attitude_setpoint)) {

					_attitude_control.setAttitudeSetpoint(Quatf(vehicle_attitude_setpoint.q_d), vehicle_attitude_setpoint.yaw_sp_move_rate);
					_thrust_setpoint_body = Vector3f(vehicle_attitude_setpoint.thrust_body);
					_last_attitude_setpoint = vehicle_attitude_setpoint.timestamp;
				}
			}

			// 检查是否有航向重置
			if (_quat_reset_counter != v_att.quat_reset_counter) {
				const Quatf delta_q_reset(v_att.delta_q_reset);

				// 对于稳定的姿态生成，仅从重置四元数中提取航向变化
				_man_yaw_sp = wrap_pi(_man_yaw_sp + Eulerf(delta_q_reset).psi());

				if (v_att.timestamp > _last_attitude_setpoint) {
					// 除非当前姿态估计后生成了姿态设定点，否则调整现有的姿态设定点
					_attitude_control.adaptAttitudeSetpoint(delta_q_reset);
				}

				_quat_reset_counter = v_att.quat_reset_counter;
			}

			Vector3f rates_sp = _attitude_control.update(q);

			const hrt_abstime now = hrt_absolute_time();
			autotune_attitude_control_status_s pid_autotune;

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
			_manual_throttle_minimum.update(_param_mpc_manthr_min.get(), dt);
		}

		if (_spooled_up) {
			_manual_throttle_maximum.update(1.f, dt);

		} else {
			_manual_throttle_maximum.setForcedValue(0.f);
		}

		// 在过渡期间重置偏航设定点，尾座式 VTOL 在过渡期间生成姿态设定点
		_reset_yaw_sp = !attitude_setpoint_generated || !_heading_good_for_control || (_vtol && _vtol_in_transition_mode);
	}

	perf_end(_loop_perf);
}

int MulticopterAttitudeControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterAttitudeControl *instance = new MulticopterAttitudeControl(vtol);

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

int MulticopterAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### 描述
该模块实现了多旋翼姿态控制器。它接收姿态设定点 (`vehicle_attitude_setpoint`) 作为输入，并输出速率设定点。

控制器具有一个 P 环来控制角误差。

参考文献：
非线性四旋翼姿态控制 (2013)
作者：Dario Brescianini, Markus Hehn 和 Raffaello D'Andrea
苏黎世联邦理工学院动态系统与控制研究所 (IDSC)

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


/**
 * 多旋翼姿态控制应用程序启动/停止处理函数
 */
extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[])
{
	return MulticopterAttitudeControl::main(argc, argv);
}
