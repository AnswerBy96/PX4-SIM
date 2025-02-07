/*
 * @Descripttion:
 * @version:
 * @Author: chenjw
 * @Date: 2023-11-01 02:06:28
 * @LastEditors: rsj
 * @LastEditTime: 2025-02-07 01:47:11
 */
#include "eboat_mavlink.hpp"

eboat_mavlink::eboat_mavlink() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
        px4_to_ui_.eboat_speed = 0.0f;
        px4_to_ui_.eboat_heading = 0.0f;
}

eboat_mavlink::~eboat_mavlink()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool eboat_mavlink::init()
{
	// alternatively, Run on fixed interval
	ScheduleOnInterval(20000_us); // 2000 us interval, 200 Hz rate

	return true;
}

void eboat_mavlink::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}


    //eboat_speed & eboat_direction
    if (vehicle_local_position_sub.update(&vehicle_local_position_))
    {
        matrix::Vector3f ground_speed(vehicle_local_position_.vx, vehicle_local_position_.vy,  vehicle_local_position_.vz);
        vehicle_attitude_sub.copy(&vehicle_attitude_);
        const Dcmf R_to_body(matrix::Quatf(vehicle_attitude_.q).inversed());
        const  matrix::Vector3f vel = R_to_body * matrix::Vector3f(ground_speed(0), ground_speed(1), ground_speed(2));
        const float x_vel = vel(0);
	// m/s -> mph
	float x_mph = x_vel * 2.23694;

        px4_to_ui_.eboat_speed = x_mph;
        px4_to_ui_.eboat_heading = (vehicle_local_position_.heading >= 0) ? (vehicle_local_position_.heading / M_PI * 180.0f) : (360.0f + vehicle_local_position_.heading / M_PI * 180.0f);

    }

    //gear
    if(custom_commander_sub.update(&custom_commander_))
    {
        px4_to_ui_.gear = custom_commander_.current_gear;
    }

    //motor speed and direction
//     if(motor_state_sub.update(&motor_state_))
//     {
// 	px4_to_ui_.motor1_speed = motor_state_.motor1_speed;
//        	px4_to_ui_.motor2_speed = motor_state_.motor2_speed;

// 	px4_to_ui_.motor1_direction = motor_state_.motor1_direction;
// 	px4_to_ui_.motor2_direction = motor_state_.motor2_direction;
//     }

// Test
//     px4_to_ui_.motor1_speed = 10;
//     px4_to_ui_.motor2_speed = 10;
//     px4_to_ui_.motor1_direction = 1;
//     px4_to_ui_.motor2_direction = 0;
//     px4_to_ui_.eboat_speed = 50;
//     px4_to_ui_.eboat_heading = 50;
//     px4_to_ui_.gear = 1;
//     px4_to_ui_.timestamp = hrt_absolute_time();

    //PX4_INFO("px4_to_ui_.eboat_speed : %f",px4_to_ui_.eboat_speed);
    eboat_mavlink_pub.publish(px4_to_ui_);


}

int eboat_mavlink::task_spawn(int argc, char *argv[])
{
	eboat_mavlink *instance = new eboat_mavlink();

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

int eboat_mavlink::print_status()
{
	printf("running...\n");
	return 0;
}

int eboat_mavlink::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int eboat_mavlink::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("eboat_mavlink", "dev");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int eboat_mavlink_main(int argc, char *argv[])
{
	return eboat_mavlink::main(argc, argv);
}
