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
	_tilt_limit_slew_rate.setSlewRate(0.2f);
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
	// 检查参数更新
	if (_parameter_update_sub.updated() || force) {
		// 清除更新
		parameter_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		// 更新参数
		ModuleParams::updateParams();
		SuperBlock::updateParams();

		int num_changed = 0;

		if (_param_sys_vehicle_resp.get() >= 0.f) {
			// 在lower end降低灵敏度
			float responsiveness = _param_sys_vehicle_resp.get() * _param_sys_vehicle_resp.get();
			// 参数线性插值更新
			num_changed += _param_cpc_xy_acc.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_cpc_xy_acc_max.commit_no_notification(math::lerp(2.f, 15.f, responsiveness));
			num_changed += _param_cpc_z_acc_max_up.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_cpc_z_acc_max_down.commit_no_notification(math::lerp(0.8f, 15.f, responsiveness));
			num_changed += _param_cpc_jerk_max.commit_no_notification(math::lerp(2.f, 50.f, responsiveness));
			if (responsiveness < 0.5f) {
				num_changed += _param_cpc_tilt_max.commit_no_notification(45.f);
			} else {
				num_changed += _param_cpc_tilt_max.commit_no_notification(math::min
				(MAX_SAFE_TILT_DEG, math::lerp(45.f, 70.f, (responsiveness - 0.5f) * 2.f)));
			}
		}

		if (_param_cpc_xy_vel_all.get() >= 0.f) {
			float xy_vel = _param_cpc_xy_vel_all.get();
			num_changed += _param_cpc_xy_vel_max.commit_no_notification(xy_vel);
		}

		if (_param_cpc_z_vel_all.get() >= 0.f) {
			float z_vel = _param_cpc_z_vel_all.get();
			num_changed += _param_cpc_z_vel_max_up.commit_no_notification(z_vel);
			num_changed += _param_cpc_z_vel_max_down.commit_no_notification(z_vel * 0.75f);
			num_changed += _param_cpc_tkof_speed.commit_no_notification(z_vel * 0.6f);
			num_changed += _param_cpc_land_speed.commit_no_notification(z_vel * 0.5f);
		}

		if (num_changed > 0) {
			param_notify_changes();
		}

		if (_param_cpc_tilt_max.get() > MAX_SAFE_TILT_DEG) {
			// 防止倾角过大
			_param_cpc_tilt_max.set(MAX_SAFE_TILT_DEG);
			_param_cpc_tilt_max.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Tilt constrained to safe value\t");
			events::send<float>(events::ID("cc_pos_ctrl_tilt_set"), events::Log::Warning,
                   			    "Maximum tilt limit has been constrained to a safe value", MAX_SAFE_TILT_DEG);
		}

		if (_param_cpc_tilt_max_land.get() > _param_cpc_tilt_max.get()) {
			// 防止降落时倾角过大
			_param_cpc_tilt_max_land.set(_param_cpc_tilt_max.get());
			_param_cpc_tilt_max_land.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Land tilt has been constrained by max tilt\t");
			events::send<float>(events::ID("cc_pos_ctrl_land_tilt_set"), events::Log::Warning,
					    "Land tilt limit has been constrained by maximum tilt", _param_cpc_tilt_max.get());
		}

		if (_param_cpc_thr_hover.get() > _param_cpc_thr_max.get() ||
		    _param_cpc_thr_hover.get() < _param_cpc_thr_min.get()) {
			// 防止推力过大
			_param_cpc_thr_hover.set(math::constrain(_param_cpc_thr_hover.get(), _param_cpc_thr_min.get(),
						 _param_cpc_thr_max.get()));
			_param_cpc_thr_hover.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Hover thrust has been constrained by min/max\t");
			events::send<float>(events::ID("cc_pos_ctrl_hover_thrust_set"), events::Log::Warning,
					    "Hover thrust has been constrained by min/max thrust", _param_cpc_thr_hover.get());
		}

		if (!_param_cpc_use_hte.get() || !_hover_thrust_initialized) {
			// 初始化悬停推力
			_control.setHoverThrust(_param_cpc_thr_hover.get());
			_hover_thrust_initialized = true;
		}

		// 防止起降速度过大
		_param_cpc_tkof_speed.set(math::min(_param_cpc_tkof_speed.get(), _param_cpc_z_vel_max_up.get()));
		_param_cpc_land_speed.set(math::min(_param_cpc_land_speed.get(), _param_cpc_z_vel_max_down.get()));

		// 位置环 P 控制
		_control.setPositionGains(Vector3f(_param_cpc_xy_p.get(), _param_cpc_xy_p.get(), _param_cpc_z_p.get()));
		// 速度环 PID 控制
		_control.setVelocityGains(
			Vector3f(_param_cpc_xy_vel_p.get(), _param_cpc_xy_vel_p.get(), _param_cpc_z_vel_p.get())
			Vector3f(_param_cpc_xy_vel_i.get(), _param_cpc_xy_vel_i.get(), _param_cpc_z_vel_i.get())
			Vector3f(_param_cpc_xy_vel_d.get(), _param_cpc_xy_vel_d.get(), _param_cpc_z_vel_d.get()));
		_control.setHorizontalThrustMargin(_param_cpc_xy_thr_marg.get());
		_control.decoupleHorizontalAndVecticalAcceleration(_param_cpc_acc_decouple.get());
		// 配置定点参数
		_goto_control.setParamCpcXyAcc(_param_cpc_xy_acc.get());
		_goto_control.setParamCpcZAccMaxUP(_param_cpc_z_acc_max_up.get());
		_goto_control.setParamCpcZAccMaxDown(_param_cpc_z_acc_max_down.get());
		_goto_control.setParamMpcXyErrMax(_param_cpc_xy_err_max.get());
		_goto_control.setParamMpcXyVelMax(_param_cpc_xy_vel_max.get());
		// 配置起飞参数
		_takeoff.setSpoolupTime(_param_com_spoolup_time.get());
		_takeoff.setTakeoffRampTime(_param_cpc_tkof_ramp_t.get());
		_takeoff.generateInitialRampValue(_param_cpc_z_vel_p.get());
	}
}

PositionControlStates CyclocopterPositionControl::set_vehicle_states(const vehicle_local_position_s &vehicle_local_position)
{
	PositionControlStates states;

	const Vector2f position_xy(vehicle_local_position.x, vehicle_local_position.y);

	// 设置位置状态
	if (vehicle_local_position.xy_valid && position_xy.isAllFinite()) {
		states.position.xy() = position_xy;
	} else {
		states.position(0) = states.position(1) = NAN;
	}

	if (PX4_ISFINITE(vehicle_local_position.z) && vehicle_local_position.z_valid) {
		states.position(2) = vehicle_local_position.z;
	} else {
		states.position(2) = NAN;
	}

	const Vector2f velocity_xy(vehicle_local_position.vx, vehicle_local_position.vy);

	// 设置速度和加速度状态
	if (vehicle_local_position.v_xy_valid && velocity_xy.isAllFinite()) {
		states.velocity.xy() = velocity_xy;
		states.acceleration(0) = _vel_x_deriv.update(velocity_xy(0));
		states.acceleration(1) = _vel_y_deriv.update(velocity_xy(1));
	} else {
		states.velocity(0) = states.velocity(1) = NAN;
		states.acceleration(0) = states.acceleration(1) = NAN;
		// 重置导数以防止恢复速度时出现加速度峰值
		_vel_x_deriv.reset();
		_vel_y_deriv.reset();
	}

	if (PX4_ISFINITE(vehicle_local_position.vz) && vehicle_local_position.v_z_valid) {
		states.velocity(2) = vehicle_local_position.vz;
		states.acceleration(2) = _vel_z_deriv.update(states.velocity(2));
	} else {
		states.velocity(2) = NAN;
		states.acceleration(2) = NAN;
		// 重置导数以防止恢复速度时出现加速度峰值
		_vel_z_deriv.reset();
	}

	states.yaw = vehicle_local_position.heading;

	return states;
}

void CyclocopterPositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}
	// 延迟调度
	ScheduleDelayed(100_ms);
	// 根据需要更新参数
	parameters_update(false);
	// 开启性能监控
	perf_begin(_cycle_perf);

	vehicle_local_position_s vehicle_local_position;

	if (_local_pos_sub.update(&vehicle_local_position)) {
		// 计算自上次循环以来的时间差dt，并限制其范围为2ms到40ms
		const float dt = math::constrain(((vehicle_local_position.timestamp_sample - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = vehicle_local_position.timestamp_sample;
		setDt(dt);
		// 检查飞行模式
		if (_vehicle_control_mode_sub.updated()) {
			const bool previous_position_control_enabled = _vehicle_control_mode.flag_multicopter_position_control_enabled;
			if (_vehicle_control_mode_sub.update(&_vehicle_control_mode)) {
				if (!previous_position_control_enabled && _vehicle_control_mode.flag_multicopter_position_control_enabled) {
					// 记录开启位置控制的时间
					_time_position_control_enabled = _vehicle_control_mode.timestamp;
				} else if (previous_position_control_enabled && !_vehicle_control_mode.flag_multicopter_position_control_enabled) {
					// 位置控制关闭时清除设定点
					_setpoint = PositionControl::empty_trajectory_setpoint;
				}
			}
		}
		// 更新着陆检测状态
		_vehicle_land_detected_sub.update(&_vehicle_land_detected);
		// 更新悬停推力
		if (_param_mpc_use_hte.get()) {
			hover_thrust_estimate_s hte;
			if (_hover_thrust_estimate_sub.update(&hte)) {
				if (hte.valid) {
					_control.updateHoverThrust(hte.hover_thrust);
				}
			}
		}

		// 设置当前车辆状态，包括位置、速度和航向等
		PositionControlStates states{set_vehicle_states(vehicle_local_position)};
		// 如果有可用的目标设定点，则发布轨迹设定点以前往该目标
		if (_goto_control.checkForSetpoint(vehicle_local_position.timestamp_sample,
						   _vehicle_control_mode.flag_multicopter_position_control_enabled)) {
			_goto_control.update(dt, states.position, states.yaw);
		}
		// 更新轨迹设定点
		_trajectory_setpoint_sub.update(&_setpoint);
		// 调整设定点以适应 EKF 重置
		adjustSetpointForEKFResets(vehicle_local_position, _setpoint);
		// 如果位置控制启用且自位置控制启动以来没有新的轨迹设定点，则设置安全设定点
		if (_vehicle_control_mode.flag_multicopter_position_control_enabled) {
			if ((_setpoint.timestamp < _time_position_control_enabled)
			    && (vehicle_local_position.timestamp_sample > _time_position_control_enabled)) {
				_setpoint = generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, false);
			}
		}

		// 位置控制启用且设定点时间戳有效
		if (_vehicle_control_mode.flag_multicopter_position_control_enabled
		    && (_setpoint.timestamp >= _time_position_control_enabled)) {
			// 更新车辆约束
			_vehicle_constraints_sub.update(&_vehicle_constraints);
			// 修复起飞斜坡过高的问题或因 NaN 卡住的问题
			if (!PX4_ISFINITE(_vehicle_constraints.speed_up) || (_vehicle_constraints.speed_up > _param_mpc_z_vel_max_up.get())) {
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
			}
			// 处理平滑起飞
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed,
						    _vehicle_constraints.want_takeoff,
						    _vehicle_constraints.speed_up, skip_takeoff, vehicle_local_position.timestamp_sample);

			// 飞行状态变量
			const bool not_taken_off             = (_takeoff.getTakeoffState() < TakeoffState::rampup);
			const bool flying                    = (_takeoff.getTakeoffState() >= TakeoffState::flight);
			const bool flying_but_ground_contact = (flying && _vehicle_land_detected.ground_contact);
			// 如果尚未起飞，则设置悬停推力
			if (!flying) {
				_control.setHoverThrust(_param_cpc_thr_hover.get());
			}
			// 确保起飞斜坡不受加速度前馈的影响
			if (_takeoff.getTakeoffState() == TakeoffState::rampup && PX4_ISFINITE(_setpoint.velocity[2])) {
				_setpoint.acceleration[2] = NAN;
			}
			// 如果尚未起飞或正在飞行但接触地面，则避免任何修正
			if (not_taken_off || flying_but_ground_contact) {
				_setpoint = PositionControl::empty_trajectory_setpoint;
				_setpoint.timestamp = vehicle_local_position.timestamp_sample;
				 // 向下高加速度以确保无推力
				Vector3f(0.f, 0.f, 100.f).copyTo(_setpoint.acceleration);
				// 防止积分器饱和
				_control.resetIntegral();
			}

			// 在起飞斜坡期间限制倾斜角度
			const float tilt_limit_deg = (_takeoff.getTakeoffState() < TakeoffState::flight)
						     ? _param_cpc_tilt_max_land.get() : _param_cpc_tilt_max.get();
			_control.setTiltLimit(_tilt_limit_slew_rate.update(math::radians(tilt_limit_deg), dt));
			// 更新起飞斜坡的速度限制
			const float speed_up = _takeoff.updateRamp(dt, PX4_ISFINITE(_vehicle_constraints.speed_up) ?
							_vehicle_constraints.speed_up : _param_cpc_z_vel_max_up.get());
			const float speed_down = PX4_ISFINITE(_vehicle_constraints.speed_down) ? _vehicle_constraints.speed_down :
						 _param_cpc_z_vel_max_down.get();

			// 设置推力限制
			const float minimum_thrust = flying ? _param_cpc_thr_min.get() : 0.f;
			_control.setThrustLimits(minimum_thrust, _param_cpc_thr_max.get());
			// 设置速度限制
			float max_speed_xy = _param_cpc_xy_vel_max.get();

			if (PX4_ISFINITE(vehicle_local_position.vxy_max)) {
				max_speed_xy = math::min(max_speed_xy, vehicle_local_position.vxy_max);
			}

			_control.setVelocityLimits(
				max_speed_xy,
				math::min(speed_up, _param_cpc_z_vel_max_up.get()), // 起飞斜坡以负速度限制开始
				math::max(speed_down, 0.f));

			// 设置输入设定点
			_control.setInputSetpoint(_setpoint);

			// Z轴位置设定点无效，Z轴速度设定点有效，Z轴位置导数有效，Z轴位置有效
			if (!PX4_ISFINITE(_setpoint.position[2]) && PX4_ISFINITE(_setpoint.velocity[2]) && (fabsf(_setpoint.velocity[2]) > FLT_EPSILON)
			    && PX4_ISFINITE(vehicle_local_position.z_deriv) && vehicle_local_position.z_valid && vehicle_local_position.v_z_valid) {
				float weighting = fminf(fabsf(_setpoint.velocity[2]) / _param_cpc_land_speed.get(), 1.f);
				// 位置导数和速度加权更新当前速度
				states.velocity(2) = vehicle_local_position.z_deriv * weighting + vehicle_local_position.vz * (1.f - weighting);
			}

			_control.setState(states);
			// 更新位置控制
			if (!_control.update(dt)) {
				// 如果位置控制失败，则使用安全设定点
				_vehicle_constraints = {0, NAN, NAN, false, {}};
				_control.setInputSetpoint(generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, true));
				_control.setVelocityLimits(_param_cpc_xy_vel_max.get(), _param_cpc_z_vel_max_up.get(), _param_cpc_z_vel_max_down.get());
				_control.update(dt);
			}

			// 发布内部位置控制设定点，这些设定点包含 PID 校正
			// 此消息用于其他模块（如 Landdetector）以确定车辆意图
			vehicle_local_position_setpoint_s local_pos_sp{};
			_control.getLocalPositionSetpoint(local_pos_sp);
			local_pos_sp.timestamp = hrt_absolute_time();
			_local_pos_sp_pub.publish(local_pos_sp);

			// 发布姿态设定点输出
			vehicle_attitude_setpoint_s attitude_setpoint{};
			_control.getAttitudeSetpoint(attitude_setpoint);
			attitude_setpoint.timestamp = hrt_absolute_time();
			_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
		} else {
			// 在非高度控制模式下必须更新起飞状态，否则起飞状态不会被跳过
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed,
							false, 10.f, true, vehicle_local_position.timestamp_sample);
		}

		// 起飞状态或倾斜限制发生变化时更新状态
		const uint8_t takeoff_state = static_cast<uint8_t>(_takeoff.getTakeoffState());
		if (takeoff_state != _takeoff_status_pub.get().takeoff_state
		    || !isEqualF(_tilt_limit_slew_rate.getState(), _takeoff_status_pub.get().tilt_limit)) {
			_takeoff_status_pub.get().takeoff_state = takeoff_state;
			_takeoff_status_pub.get().tilt_limit = _tilt_limit_slew_rate.getState();
			_takeoff_status_pub.get().timestamp = hrt_absolute_time();
			_takeoff_status_pub.update();
		}
	}
	// 结束性能计数器测量循环时间
	perf_end(_cycle_perf);
}

trajectory_setpoint_s CyclocopterPositionControl::generateFailsafeSetpoint(const hrt_abstime &now,
									   const PositionControlStates &states, bool warn)
{
    // 限制警告频率，每2秒最多一次警告
    warn = warn && (now - _last_warn) > 2_s;

    if (warn) {
        PX4_WARN("invalid setpoints");
        _last_warn = now;
    }

    // 生成故障保护模式的设定点
    trajectory_setpoint_s failsafe_setpoint = PositionControl::empty_trajectory_setpoint;
    failsafe_setpoint.timestamp = now;

    if (Vector2f(states.velocity).isAllFinite()) {
        // 水平速度置0
        failsafe_setpoint.velocity[0] = failsafe_setpoint.velocity[1] = 0.f;

        if (warn) {
            PX4_WARN("Failsafe: stop and wait");
        }
    } else {
        // 如果无法停止，则以降落速度下降
        failsafe_setpoint.acceleration[0] = failsafe_setpoint.acceleration[1] = 0.f;
        failsafe_setpoint.velocity[2] = _param_cpc_land_speed.get();

        if (warn) {
            PX4_WARN("Failsafe: blind land");
        }
    }

    if (PX4_ISFINITE(states.velocity(2))) {
        // 如果水平速度已经置0，则不在z轴上移动
        if (!PX4_ISFINITE(failsafe_setpoint.velocity[2])) {
            failsafe_setpoint.velocity[2] = 0.f;
        }
    } else {
        // 紧急下降，推力略低于悬停推力
        failsafe_setpoint.velocity[2] = NAN;
        failsafe_setpoint.acceleration[2] = .3f;

        if (warn) {
            PX4_WARN("Failsafe: blind descent");
        }
    }
    return failsafe_setpoint;
}

void CyclocopterPositionControl::adjustSetpointForEKFResets(const vehicle_local_position_s &vehicle_local_position,
								  trajectory_setpoint_s &setpoint)
{
	if ((setpoint.timestamp != 0) && (setpoint.timestamp < vehicle_local_position.timestamp)) {
        // 检查水平速度重置
		if (vehicle_local_position.vxy_reset_counter != _vxy_reset_counter) {
			setpoint.velocity[0] += vehicle_local_position.delta_vxy[0];
			setpoint.velocity[1] += vehicle_local_position.delta_vxy[1];
		}
        // 检查垂直速度重置
		if (vehicle_local_position.vz_reset_counter != _vz_reset_counter) {
			setpoint.velocity[2] += vehicle_local_position.delta_vz;
		}
        // 检查水平位置重置
		if (vehicle_local_position.xy_reset_counter != _xy_reset_counter) {
			setpoint.position[0] += vehicle_local_position.delta_xy[0];
			setpoint.position[1] += vehicle_local_position.delta_xy[1];
		}
        // 检查垂直位置重置
		if (vehicle_local_position.z_reset_counter != _z_reset_counter) {
			setpoint.position[2] += vehicle_local_position.delta_z;
		}
        // 检查航向重置
		if (vehicle_local_position.heading_reset_counter != _heading_reset_counter) {
			setpoint.yaw = wrap_pi(setpoint.yaw + vehicle_local_position.delta_heading);
		}
	}
    	// 如果水平速度重置计数器发生变化，重置水平速度导数
	if (vehicle_local_position.vxy_reset_counter != _vxy_reset_counter) {
		_vel_x_deriv.reset();
		_vel_y_deriv.reset();
	}
    	// 如果垂直速度重置计数器发生变化，重置垂直速度导数
	if (vehicle_local_position.vz_reset_counter != _vz_reset_counter) {
		_vel_z_deriv.reset();
	}
    	// 保存最新的重置计数器
	_vxy_reset_counter = vehicle_local_position.vxy_reset_counter;
	_vz_reset_counter = vehicle_local_position.vz_reset_counter;
	_xy_reset_counter = vehicle_local_position.xy_reset_counter;
	_z_reset_counter = vehicle_local_position.z_reset_counter;
	_heading_reset_counter = vehicle_local_position.heading_reset_counter;
}

int CyclocopterPositionControl::task_spawn(int argc, char *argv[])
{
	CyclocopterPositionControl *instance = new CyclocopterPositionControl();

	// 成功创建实例后，将其存储到_object中，并设置任务ID
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

int CyclocopterPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CyclocopterPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_USAGE_NAME("cc_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int cc_pos_control_main(int argc, char *argv[])
{
	return CyclocopterPositionControl::main(argc, argv);
}

