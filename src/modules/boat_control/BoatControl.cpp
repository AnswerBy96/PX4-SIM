#include "BoatControl.hpp"

BoatControl::BoatControl() : ModuleParams(nullptr),
					   ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	updateParams();
	pid_init(&_head_pid, PID_MODE_DERIVATIV_CALC, 0.01f);

}

bool BoatControl::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void BoatControl::updateParams()
{
	ModuleParams::updateParams();
	_heading_pid.setParameters(_param_ob_heading_kp.get(),
								     _param_ob_heading_ki.get(),
								     _param_ob_heading_kd.get());
	_speed_pid.setParameters(_param_ob_speed_kp.get(),
								_param_ob_speed_ki.get(),
								_param_ob_speed_kd.get());

	pid_set_parameters(&_head_pid,
					   _param_ob_heading_kp.get(), // P
					   _param_ob_heading_ki.get(), // I
					   _param_ob_heading_kd.get(), // D
					   1.0f, // Integral limit
					   1.0f); // Output limit)

	_max_speed = _param_ob_max_speed.get();
	_stop_min_dist = _param_stop_min_distance.get();
}

void BoatControl::updateSubscriptions()
{

	if (_parameter_update_sub.updated())
	{
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
	}

	if (_vehicle_status_sub.updated())
	{
		_vehicle_status_sub.copy(&_vehicle_status);
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
	}

	if (_vehicle_angular_velocity_sub.updated()) {
		vehicle_angular_velocity_s vehicle_angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity);
		_vehicle_yaw_rate = vehicle_angular_velocity.xyz[2];
	}

	if (_vehicle_acceleration_sub.updated()) {
		vehicle_acceleration_s vehicle_acceleration{};
		_vehicle_acceleration_sub.copy(&vehicle_acceleration);
		_actual_accel = vehicle_acceleration.xyz[0];
	}

	if (_vehicle_global_position_sub.updated()) {
		vehicle_global_position_s vehicle_global_position{};
		_vehicle_global_position_sub.copy(&vehicle_global_position);
		_curr_pos = matrix::Vector2d(vehicle_global_position.lat, vehicle_global_position.lon);
	}

	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		_vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(_vehicle_attitude_quaternion).psi();
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);
		Vector3f velocity_in_local_frame(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
		Vector3f velocity_in_body_frame = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_in_local_frame);
		_vehicle_forward_speed = velocity_in_body_frame(0);

		if (!_global_ned_proj_ref.isInitialized() || (_global_ned_proj_ref.getProjectionReferenceTimestamp() != vehicle_local_position.ref_timestamp)) {
			_global_ned_proj_ref.initReference(vehicle_local_position.ref_lat, vehicle_local_position.ref_lon, vehicle_local_position.ref_timestamp);
		}

		_curr_pos_ned = matrix::Vector2f(vehicle_local_position.x, vehicle_local_position.y);

		//更新顶流目标位置
		if(_vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_POSCTL)
			_destination_ned = _curr_pos_ned;

	}

	if (_home_position_sub.updated()) {
		home_position_s home_position{};
		_home_position_sub.copy(&home_position);
		_home_position = Vector2d(home_position.lat, home_position.lon);
		_home_position_ned = _global_ned_proj_ref.project(home_position.lat, home_position.lon);
	}

	if (_position_setpoint_triplet_sub.updated()) {
		updateWaypoints();
	}
}

void BoatControl::updateWaypoints()
{
	position_setpoint_triplet_s position_setpoint_triplet{};
	_position_setpoint_triplet_sub.copy(&position_setpoint_triplet);

	// Global waypoint coordinates
	if (position_setpoint_triplet.current.valid && PX4_ISFINITE(position_setpoint_triplet.current.lat)
	    && PX4_ISFINITE(position_setpoint_triplet.current.lon)) {
		_curr_wp = matrix::Vector2d(position_setpoint_triplet.current.lat, position_setpoint_triplet.current.lon);

	} else {
		_curr_wp = matrix::Vector2d(0, 0);
	}

	if (position_setpoint_triplet.previous.valid && PX4_ISFINITE(position_setpoint_triplet.previous.lat)
	    && PX4_ISFINITE(position_setpoint_triplet.previous.lon)) {
		_prev_wp = matrix::Vector2d(position_setpoint_triplet.previous.lat, position_setpoint_triplet.previous.lon);

	} else {
		_prev_wp = _curr_pos;
	}

	if (position_setpoint_triplet.next.valid && PX4_ISFINITE(position_setpoint_triplet.next.lat)
	    && PX4_ISFINITE(position_setpoint_triplet.next.lon)) {
		_next_wp = matrix::Vector2d(position_setpoint_triplet.next.lat, position_setpoint_triplet.next.lon);

	} else {
		_next_wp = _home_position;
	}
	// NED waypoint coordinates
	_curr_wp_ned = _global_ned_proj_ref.project(_curr_wp(0), _curr_wp(1));
	_prev_wp_ned = _global_ned_proj_ref.project(_prev_wp(0), _prev_wp(1));
	_next_wp_ned = _global_ned_proj_ref.project(_next_wp(0), _next_wp(1));

}

float BoatControl::speed_control(float target_speed){
	float delta_u = 0.0;
	// float needAccel = 0.0;
	// if(_vehicle_forward_speed > target_speed){
	// 	needAccel = (target_speed - _vehicle_forward_speed) / ACCEL_NEG_TIME;
	// }else{
	// 	needAccel = (target_speed - _vehicle_forward_speed) / ACCEL_POS_TIME;
	// }
	// delta_u = _speed_pid.calculate(needAccel,_actual_accel);
	//PX4_INFO("_vehicle_forward_speed: %f",_vehicle_forward_speed);
	delta_u = _speed_pid.calculate(target_speed,_vehicle_forward_speed);
	return delta_u;
}

float BoatControl::heading_control(float target_heading){
	float delta_u = 0.0;
	delta_u = _heading_pid.calculate(target_heading,_vehicle_yaw);
	return delta_u;
}

void BoatControl::manual_control()
{
	if(_custom_commander.drive_mode == custom_commander_s::DRIVE_MODE_MANUAL){
		float desireSpeed = 0.0f;
		switch (_custom_commander.current_gear)
		{
			case chassis_data_s::GEAR_P:
				// Set heading from the manual roll input channel
				_torque_setpoint_z = 0.0f;
				// Set throttle from the manual throttle channel
				_thrust_setpoint_x = 0.0f;
				_thrust_setpoint_y = 0.0f;
				break;
			case chassis_data_s::GEAR_D:
				// Set heading from the manual roll input channel
				_torque_setpoint_z = _custom_commander.target_steeringwheel;
				// Set throttle from the manual throttle channel
				desireSpeed = _custom_commander.target_throttle * _param_ob_max_speed.get();
				_thrust_setpoint_x += speed_control(desireSpeed);//速度控制
				_thrust_setpoint_x = (_thrust_setpoint_x < -1.0f) ? -1.0f : ((_thrust_setpoint_x > 1.0f) ? 1.0f : _thrust_setpoint_x);
				break;
			case chassis_data_s::GEAR_R:
				/* code */
				break;
			default:
				break;
		}
	}
}

void BoatControl::remote_control()
{
	//if( _custom_commander.drive_mode == custom_commander_s::DRIVE_MODE_REMOTE){
		if (_control_mode.flag_control_manual_enabled) {
			manual_control_setpoint_s manual_control_setpoint_;
			if (_manual_control_setpoint_sub.copy(&manual_control_setpoint_)) {
				// Set heading from the manual roll input channel
				// _torque_setpoint_z = manual_control_setpoint_.y; // Nominally yaw: _manual_control_setpoint.r;
				// // Set throttle from the manual throttle channel
				// _thrust_setpoint_x = manual_control_setpoint_.z;
				// _thrust_setpoint_y = 0.0;

				float speed_delta_u = speed_control(_max_speed * manual_control_setpoint_.z);

				_thrust_setpoint_x += speed_delta_u;

				_thrust_setpoint_x = (_thrust_setpoint_x < -1.0f) ? -1.0f : ((_thrust_setpoint_x > 1.0f) ? 1.0f : _thrust_setpoint_x);
				_thrust_setpoint_y = 0.0f;
				_torque_setpoint_z = manual_control_setpoint_.y; // Nominally yaw: _manual_control_setpoint.r;

			}
		}
	//}
}

void BoatControl::position_control()
{
	float desired_speed_gain = _param_ob_speed_gain.get();
	float acceptable_radius_m = _param_ob_accept_radius.get();
	const float distance_to_destination = get_distance_to_next_waypoint(_curr_pos(0), _curr_pos(1), _destination_ned(0), _destination_ned(1));
	float desired_speed = math::min((distance_to_destination - acceptable_radius_m) * desired_speed_gain , _max_speed);
	if(distance_to_destination > acceptable_radius_m){
		hrt_abstime now = hrt_absolute_time();
		const float dt = math::min((now - _time_stamp_last), 5000_ms) / 1e3f;
		_time_stamp_last = now;

		float dx_currpos_to_destination = _destination_ned(0) - _curr_pos_ned(0);
		float dy_currpos_to_destination = _destination_ned(1) - _curr_pos_ned(1);

		float desired_heading = atan2f(dy_currpos_to_destination , dx_currpos_to_destination);

		float heading_error = desired_heading - _vehicle_yaw;
		//如果目标点在船后面，则使用反推，屁股对着目标点
		if(fabs(heading_error) > (M_PI/2.0f)){
			desired_speed = -desired_speed;
			desired_heading += M_PI;
			heading_error = desired_heading - _vehicle_yaw;
		}
		if(heading_error < -M_PI){
			heading_error = heading_error + 2 * M_PI;
		}
		if(heading_error > M_PI){
			heading_error = heading_error - 2 * M_PI;
		}

		float speed_delta_u = speed_control(desired_speed);
		_thrust_setpoint_x += speed_delta_u;
		_thrust_setpoint_x = (_thrust_setpoint_x < -1.0f) ? -1.0f : ((_thrust_setpoint_x > 1.0f) ? 1.0f : _thrust_setpoint_x);

		_torque_setpoint_z = pid_calculate(&_head_pid, heading_error , 0.0 , 0.0 , dt);
		_torque_setpoint_z = (_torque_setpoint_z < -1.0f) ? -1.0f : ((_torque_setpoint_z > 1.0f) ? 1.0f : _torque_setpoint_z);
		_thrust_setpoint_y = 0.0f;
	}
	else{
		_heading_pid.reset();
		_speed_pid.reset();
		pid_reset_integral(&_head_pid);
		_thrust_setpoint_x = 0.0f;
		_thrust_setpoint_y = 0.0f;
		_torque_setpoint_z = 0.0f;
	}
}

void  BoatControl::mission_control()
{
	mission_result_s mission_result{};
	if (_mission_result_sub.updated()) {
		_mission_result_sub.copy(&mission_result);
	}

	float dist_to_curr_wp = distance_point_to_point(_curr_pos_ned , _curr_wp_ned);
	if(dist_to_curr_wp < _stop_min_dist || mission_result.finished == true){
		_guid_state = STOPPING;
		_is_arrive_goal = true;
	}else{
		if(_guid_state == STOPPING){
			float dist_between_waypoints = get_distance_to_next_waypoint((double)_prev_wp(0), (double)_prev_wp(1),
							(double)_curr_wp(0), (double)_curr_wp(1));
			if(dist_between_waypoints > 0){
				_guid_state = GOTO_WAYPOINT;
				_is_arrive_goal = false;
			}
		}
	}

	// if(!mission_result.finished || _guid_state == GOTO_WAYPOINT){
	if( _guid_state == GOTO_WAYPOINT){
		//PX4_INFO("Mission running");
		hrt_abstime now = hrt_absolute_time();
		const float dt = math::min((now - _time_stamp_last), 5000_ms) / 1e3f;
		_time_stamp_last = now;
		const float desired_heading = _pure_pursuit.calcDesiredHeading(_curr_wp_ned, _prev_wp_ned, _curr_pos_ned,
					math::max(_vehicle_forward_speed, 0.f));
		//PX4_INFO("target heading : %f",desired_heading);
		float heading_error = desired_heading - _vehicle_yaw;
		if(heading_error < -M_PI){
			heading_error = heading_error + 2 * M_PI;
		}
		if(heading_error > M_PI){
			heading_error = heading_error - 2 * M_PI;
		}
		// 转向时减速
		float k_speed = 1 / (1 + fabsf(heading_error) * _param_ob_speed_gain.get());

		float desired_speed = k_speed * _max_speed;

		float speed_delta_u = speed_control(desired_speed);

		_thrust_setpoint_x += speed_delta_u;
		_thrust_setpoint_x = (_thrust_setpoint_x < -1.0f) ? -1.0f : ((_thrust_setpoint_x > 1.0f) ? 1.0f : _thrust_setpoint_x);

		// float heading_delta_u = heading_control(desired_heading);
		// _torque_setpoint_z += heading_delta_u;
		// _torque_setpoint_z = (_torque_setpoint_z < -1.0f) ? -1.0f : ((_torque_setpoint_z > 1.0f) ? 1.0f : _torque_setpoint_z);

		_torque_setpoint_z = pid_calculate(&_head_pid, heading_error , 0.0 , 0.0 , dt);
		_torque_setpoint_z = (_torque_setpoint_z < -1.0f) ? -1.0f : ((_torque_setpoint_z > 1.0f) ? 1.0f : _torque_setpoint_z);

		_thrust_setpoint_y = 0.0f;
		//PX4_INFO("_thrust_setpoint_x : %f",_thrust_setpoint_x);
		//PX4_INFO("_torque_setpoint_z : %f",_torque_setpoint_z);

		_boat_guidance_status.timestamp = now;
		_boat_guidance_status.desired_heading = desired_heading;
		_boat_guidance_status.heading_error = heading_error;
		_boat_guidance_status.actual_heading = _vehicle_yaw;
		_boat_guidance_status.desired_speed = desired_speed;
		_boat_guidance_status.actual_speed = _vehicle_forward_speed;
		_boat_guidance_status.speed_error = desired_speed - _vehicle_forward_speed;
	}
	else{
		_heading_pid.reset();
		_speed_pid.reset();
		pid_reset_integral(&_head_pid);
		_thrust_setpoint_x = 0.0f;
		_thrust_setpoint_y = 0.0f;
		_torque_setpoint_z = 0.0f;
	}

}

void BoatControl::auto_return()
{
	const float distance_to_home = get_distance_to_next_waypoint((double)_curr_pos(0), (double)_curr_pos(1),
														(double) _home_position(0),(double) _home_position(1));
	if(distance_to_home <= _stop_min_dist){
		_is_arrive_goal = true;
	}else{
		_is_arrive_goal = false;
	}
	if(!_is_arrive_goal){
		hrt_abstime now = hrt_absolute_time();
		const float dt = math::min((now - _time_stamp_last), 5000_ms) / 1e3f;
		_time_stamp_last = now;

		float dx_currpos_to_home = _home_position_ned(0) - _curr_pos_ned(0);
		float dy_currpos_to_home = _home_position_ned(1) - _curr_pos_ned(1);

		const float desired_heading = atan2f(dy_currpos_to_home , dx_currpos_to_home);

		float heading_error = desired_heading - _vehicle_yaw;
		if(heading_error < -M_PI){
			heading_error = heading_error + 2 * M_PI;
		}
		if(heading_error > M_PI){
			heading_error = heading_error - 2 * M_PI;
		}

		// float heading_delta_u = heading_control(desired_heading);

		// _torque_setpoint_z += heading_delta_u;

		_torque_setpoint_z = pid_calculate(&_head_pid, desired_heading , _vehicle_yaw , 0.0 , dt);
		_torque_setpoint_z = (_torque_setpoint_z < -1.0f) ? -1.0f : ((_torque_setpoint_z > 1.0f) ? 1.0f : _torque_setpoint_z);

		float k_speed = 1 / (1 + fabsf(heading_error) * _param_ob_speed_gain.get());

		float desired_speed = k_speed * _max_speed;

		float speed_delta_u = speed_control(desired_speed);

		_thrust_setpoint_x += speed_delta_u;
		_thrust_setpoint_x = (_thrust_setpoint_x < -1.0f) ? -1.0f : ((_thrust_setpoint_x > 1.0f) ? 1.0f : _thrust_setpoint_x);
		_thrust_setpoint_y = 0.0f;

		//返航时先完成航向调整
		// if(heading_error < (10.f / 180.0f * M_PI)){
		// 	float speed_delta_u = speed_control(return_speed);

		// 	_thrust_setpoint_x += speed_delta_u;
		// 	_thrust_setpoint_x = (_thrust_setpoint_x < -1.0f) ? -1.0f : ((_thrust_setpoint_x > 1.0f) ? 1.0f : _thrust_setpoint_x);
		// 	_thrust_setpoint_y = 0.0f;
		// }
		// else{
		// 	_thrust_setpoint_x = 0.0f;
		// 	_thrust_setpoint_y = 0.0f;
		// }

		_boat_guidance_status.timestamp = now;
		_boat_guidance_status.desired_heading = desired_heading;
		_boat_guidance_status.heading_error = heading_error;
		_boat_guidance_status.actual_heading = _vehicle_yaw;
		_boat_guidance_status.desired_speed = desired_speed;
		_boat_guidance_status.actual_speed = _vehicle_forward_speed;
		_boat_guidance_status.speed_error = desired_speed - _vehicle_forward_speed;

	}
	else{
		_heading_pid.reset();
		_speed_pid.reset();
		pid_reset_integral(&_head_pid);
		_torque_setpoint_z = 0.0f;
		_thrust_setpoint_x = 0.0f;
		_thrust_setpoint_y = 0.0f;
	}

}

void BoatControl::publish_control_setpoint()
{
	vehicle_thrust_setpoint_s v_thrust_sp{};
	v_thrust_sp.timestamp = hrt_absolute_time();
	v_thrust_sp.xyz[0] = _thrust_setpoint_x * _param_ob_thrust_scaling.get();
	v_thrust_sp.xyz[1] = 0.0f;
	v_thrust_sp.xyz[2] = 0.0f;
	_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);

	vehicle_torque_setpoint_s v_torque_sp{};
	v_torque_sp.timestamp = hrt_absolute_time();
	v_torque_sp.xyz[0] = 0.0;
	v_torque_sp.xyz[1] = 0.0;
	v_torque_sp.xyz[2] = _torque_setpoint_z * _param_ob_torque_scaling.get();
	_vehicle_torque_setpoint_pub.publish(v_torque_sp);

	if(_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL ||
		_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ||
		_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL){

		_boat_guidance_status_pub.publish(_boat_guidance_status);
	}

}

void BoatControl::Run()
{
	if (should_exit())
	{
		ScheduleClear();
		exit_and_cleanup();
	}

	updateParams();

	updateSubscriptions();
	if(_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED){

		//hrt_abstime now = hrt_absolute_time();
		switch (_vehicle_status.nav_state){
			case vehicle_status_s::NAVIGATION_STATE_MANUAL:
				// Manual mode
				// directly produce setpoints from the manual control setpoint (joystick)
				//manual_control();
				remote_control();
				break;
			case vehicle_status_s::NAVIGATION_STATE_POSCTL:
				position_control();
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
				auto_return();
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
				mission_control();
				break;
			default:
				_thrust_setpoint_x = 0.0f;
				_thrust_setpoint_y = 0.0f;
				_torque_setpoint_z = 0.0f;
				break;
		}

		publish_control_setpoint();
	}

}

int BoatControl::task_spawn(int argc, char *argv[])
{
	BoatControl *instance = new BoatControl();

	if (instance)
	{
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init())
		{
			return PX4_OK;
		}
	}
	else
	{
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int BoatControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int BoatControl::print_usage(const char *reason)
{
	if (reason)
	{
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Boat Drive controller.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("boat_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int boat_control_main(int argc, char *argv[])
{
	return BoatControl::main(argc, argv);
}
