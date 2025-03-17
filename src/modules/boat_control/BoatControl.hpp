#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>


#include <uORB/topics/mission_result.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/boat_guidance_status.h>
#include <uORB/topics/chassis_data.h>
#include <uORB/topics/custom_commander.h>
#include <uORB/topics/ui_to_px4_cruiseparam.h>
#include <uORB/topics/ui_to_px4_missionstate.h>

#include <lib/pid/pid.h>
#include <matrix/matrix/math.hpp>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <lib/pure_pursuit/PurePursuit.hpp>
#include <math.h>

#include "pid.hpp"

#define ACCEL_NEG_TIME 2.0
#define ACCEL_POS_TIME 4.0
using namespace time_literals;

class BoatControl : public ModuleBase<BoatControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	BoatControl();
	~BoatControl() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

protected:
	void updateParams() override;

private:
	void Run() override;

	void updateWaypoints();

	void updateSubscriptions();

	void manual_control();

	void remote_control();

	void position_control();

	void mission_control();

	void auto_return();

	void cruise_control(float target_speed , float target_heading);

	float heading_control(float target_heading);

	float speed_control(float target_speed);

	void publish_control_setpoint();

	float distance_point_to_point(matrix::Vector2f point1, matrix::Vector2f point2){
		return sqrt(pow(point1(0)-point2(0),2)+pow(point1(1) - point2(1),2));
	}

	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_local_pos_setpoint_sub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Subscription _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _mission_result_sub{ORB_ID(mission_result)};
	uORB::Subscription _home_position_sub{ORB_ID(home_position)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _custom_commander_sub{ORB_ID(custom_commander)};
	uORB::Subscription _chassis_data_sub{ORB_ID(chassis_data)};
	uORB::Subscription _ui_to_px4_cruiseparam_sub{ORB_ID(ui_to_px4_cruiseparam)};
	uORB::Subscription _ui_to_px4_missionstate_sub{ORB_ID(ui_to_px4_missionstate)};

	// Add Publications for control allocator
	uORB::Publication<actuator_controls_status_s> _actuator_controls_status_pub{ORB_ID(actuator_controls_status_0)};
	uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)}; /**< vehicle torque setpoint publication */
	uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)}; /**< vehicle thrust setpoint publication */
	uORB::Publication<boat_guidance_status_s> _boat_guidance_status_pub{ORB_ID(boat_guidance_status)};

	enum GuidState{
		STOPPING = 0,
		GOTO_WAYPOINT = 1,
		PAUSED
	}_guid_state{STOPPING};

	enum class MissionRequest{
		MISSION_PAUSE  = 0,
		MISSION_RESUME = 1
	};


	matrix::Quatf _vehicle_attitude_quaternion{};
	float _vehicle_yaw_rate{0.f};
	float _vehicle_forward_speed{0.f};
	float _vehicle_yaw{0.f};
	float _max_yaw_rate{0.f};
	float _thrust_setpoint_x{0.0f};
	float _thrust_setpoint_y{0.0f};
	float _torque_setpoint_z{0.0f};
	float _actual_accel{0.0};

	custom_commander_s			_custom_commander{};
	chassis_data_s						_chassis_data{};
	vehicle_local_position_setpoint_s _vehicle_local_pos_setpoint{};
	vehicle_local_position_s _vehicle_local_pos{};
	vehicle_status_s _vehicle_status{};
	boat_guidance_status_s _boat_guidance_status;
	ui_to_px4_cruiseparam_s _ui_to_px4_cruiseparam;
	ui_to_px4_missionstate_s _ui_to_px4_missionstate;


	hrt_abstime _time_stamp_last{0}; /**< time stamp when task was last updated */

	// PID heading controller
	PID_t _head_pid;
	IncrementalPID _heading_pid{};
	// PID speed controller
	IncrementalPID _speed_pid{};


	float _max_speed{0.1f};
	float _max_angular_velocity{0.1f};

	float _x_pos_sp{0.0};
	float _y_pos_sp{0.0};

	float _stop_min_dist{5.0};
	bool _is_arrive_goal{false};

	MapProjection _global_ned_proj_ref{}; // Transform global to ned coordinates.
	PurePursuit _pure_pursuit{this}; // Pure pursuit library

	// Waypoints
	matrix::Vector2d _curr_pos{};
	matrix::Vector2f _curr_pos_ned{};
	matrix::Vector2d _prev_wp{};
	matrix::Vector2f _prev_wp_ned{};
	matrix::Vector2d _curr_wp{};
	matrix::Vector2f _curr_wp_ned{};
	matrix::Vector2d _next_wp{};
	matrix::Vector2f _next_wp_ned{};
	matrix::Vector2d _home_position{};
	matrix::Vector2f _home_position_ned{};
	// hold position
	matrix::Vector2f _destination_ned{};
	bool _is_update_destination{true};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::OB_HEADING_KP>) _param_ob_heading_kp,
		(ParamFloat<px4::params::OB_HEADING_KI>) _param_ob_heading_ki,
		(ParamFloat<px4::params::OB_HEADING_KD>) _param_ob_heading_kd,
		(ParamFloat<px4::params::OB_SPEED_KP>) _param_ob_speed_kp,
		(ParamFloat<px4::params::OB_SPEED_KI>) _param_ob_speed_ki,
		(ParamFloat<px4::params::OB_SPEED_KD>) _param_ob_speed_kd,
		(ParamFloat<px4::params::OB_HEADING_SP>) _param_ob_heading_sp,
		(ParamFloat<px4::params::OB_X_POS_SP>) _param_ob_x_pos_sp,
		(ParamFloat<px4::params::OB_Y_POS_SP>) _param_ob_y_pos_sp,
		(ParamFloat<px4::params::OB_THRUST_SCAL>) _param_ob_thrust_scaling,
		(ParamFloat<px4::params::OB_TORQUE_SCAL>) _param_ob_torque_scaling,
		(ParamFloat<px4::params::OB_MAX_SPEED>) _param_ob_max_speed,
		(ParamFloat<px4::params::OB_ACCEPT_RADIUS>) _param_ob_accept_radius,
		(ParamFloat<px4::params::OB_SPEED_GAIN>) _param_ob_speed_gain,
		(ParamFloat<px4::params::OB_STOP_MIN_DIST>) _param_stop_min_distance

	);
};
