/*
 * @Descripttion:
 * @version:
 * @Author: chenjw
 * @Date: 2023-11-01 02:06:28
 * @LastEditors: rsj
 * @LastEditTime: 2023-11-21 18:48:46
 */
#include "eboat_mavlink.hpp"

eboat_mavlink::eboat_mavlink() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

eboat_mavlink::~eboat_mavlink()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool eboat_mavlink::init()
{
	// alternatively, Run on fixed interval
	ScheduleOnInterval(100000_us); // 2000 us interval, 200 Hz rate

	return true;
}

void eboat_mavlink::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

    px4_to_ui_.timestamp = (int)time((time_t*) NULL);

    //eboat_speed & eboat_direction
    if (vehicle_local_position_sub.update(&vehicle_local_position_))
    {
        matrix::Vector3f ground_speed(vehicle_local_position_.vx, vehicle_local_position_.vy,  vehicle_local_position_.vz);
        vehicle_attitude_sub.copy(&vehicle_attitude_);
        const Dcmf R_to_body(matrix::Quatf(vehicle_attitude_.q).inversed());
        const  matrix::Vector3f vel = R_to_body * matrix::Vector3f(ground_speed(0), ground_speed(1), ground_speed(2));
        const float x_vel = vel(0);

        px4_to_ui_.eboat_speed = x_vel;
        px4_to_ui_.eboat_heading = vehicle_local_position_.heading;
    }

    //gear
    if(custom_commander_sub.update(&custom_commander_))
    {
        px4_to_ui_.gear = custom_commander_.current_gear;
    }

    //motor speed and direction
       px4_to_ui_.motor1_speed = 10;
       px4_to_ui_.motor2_speed = 10;
       px4_to_ui_.motor1_direction = 1;
       px4_to_ui_.motor2_direction = 0;

// Test
//     px4_to_ui_.motor1_speed = 10;
//     px4_to_ui_.motor2_speed = 10;
//     px4_to_ui_.motor1_direction = 1;
//     px4_to_ui_.motor2_direction = 0;
//     px4_to_ui_.eboat_speed = 5;
//     px4_to_ui_.eboat_heading = 100;
//     px4_to_ui_.gear = 1;

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

	PRINT_MODULE_USAGE_NAME("custom_commander", "dev");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int eboat_mavlink_main(int argc, char *argv[])
{
	return eboat_mavlink::main(argc, argv);
}
