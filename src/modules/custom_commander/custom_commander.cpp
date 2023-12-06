#include "custom_commander.hpp"

CustomCommander::CustomCommander() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_custom_commander.current_gear = GEAR_P;
	_custom_commander.target_steeringwheel = 0.0f;
	_custom_commander.target_throttle = 0.0f;
	_custom_commander.drive_mode = MANUAL;				//默认手动驾驶模式
}

CustomCommander::~CustomCommander()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

void CustomCommander::auto_commander()
{

}

void CustomCommander::gear_commander()
{
	if( _chassis_data.gear_left != _chassis_data.gear_right)
	{
		_custom_commander.current_gear = GEAR_P;
		_custom_commander.system_error = _custom_commander.system_error | 0x01;	//两个档位不匹配
		_custom_commander.target_steeringwheel = 0.0f;
		_custom_commander.target_throttle = 0.0f;
		return;
	}
	else if(_chassis_data.gear_left == GEAR_P || _chassis_data.gear_right == GEAR_P)
	{
		_custom_commander.current_gear = GEAR_P;
		_custom_commander.system_error = _custom_commander.system_error & 0xFE;
		_custom_commander.target_steeringwheel = 0.0f;
		_custom_commander.target_throttle = 0.0f;
		return;
	}
	else if(_chassis_data.gear_left == GEAR_D && _chassis_data.gear_right == GEAR_D)
	{
		_custom_commander.current_gear = GEAR_D;
		_custom_commander.system_error = _custom_commander.system_error & 0xFE;
		return;
	}
	else if(_chassis_data.gear_left == GEAR_R && _chassis_data.gear_right == GEAR_R)
	{
		_custom_commander.current_gear = GEAR_R;
		_custom_commander.system_error = _custom_commander.system_error & 0xFE;
		return;
	}

}

//获取当前油门、方向盘数据
void CustomCommander::cal_throttle_sheeringwheel()
{

	if(abs(_chassis_data.throttle_left - _chassis_data.throttle_right)<=10)
	{
		averageThrottle = (_chassis_data.throttle_left + _chassis_data.throttle_right) / 2 ;

		if(isUpdateThrottle)
		{
			_custom_commander.target_throttle += (float)(averageThrottle - last_averageThrottle)/100/100.0f;
			if(fabs(_custom_commander.target_throttle - (averageThrottle/100.0f)) < 0.01f)
			{
				isUpdateThrottle = false;
			}
		}
		else
		{
			_custom_commander.target_throttle = averageThrottle / 100.0f;
			last_averageThrottle = averageThrottle;
		}
	}

	if(_chassis_data.have_steeringwheel)	//如果有方向盘则转向信号使用方向盘，否则使用两个推杆的差
		_custom_commander.target_steeringwheel = _chassis_data.steeringwheel;
	else
	{
		if(abs(_chassis_data.throttle_left - _chassis_data.throttle_right)>=15)
		{
			_custom_commander.target_steeringwheel = (_chassis_data.throttle_left - _chassis_data.throttle_right) / 100.0f;
			isUpdateThrottle = true;

		}
		else
			_custom_commander.target_steeringwheel = 0.0f;
	}

}

void CustomCommander::publish_offboard_control_mode(bool position,bool velocity,bool acceleration,bool attitude,bool body_rate,bool actuator)
{
	offboard_control_mode_s msg{};
	msg.position = position;
	msg.velocity = velocity;
	msg.acceleration = acceleration;
	msg.attitude = attitude;
	msg.body_rate = body_rate;
	msg.actuator = actuator;
	msg.timestamp = hrt_absolute_time();
	_offboard_control_mode_pub.publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 * @param param3    Command parameter 3
 */
void CustomCommander::publish_vehicle_command(uint16_t command, float param1, float param2, float param3)
{
	vehicle_command_s msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.command = command;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	msg.source_system = vehicle_status_sub.get().system_id;
	msg.target_system = vehicle_status_sub.get().system_id;
	msg.source_component = vehicle_status_sub.get().component_id;
	msg.target_component = vehicle_status_sub.get().component_id;

	msg.from_external = true;
	msg.timestamp = hrt_absolute_time();
	_vehicle_command_pub.publish(msg);
}

//进入offboard模式
void CustomCommander::into_offboard_mode()
{
	// offboard mode
	uint8_t offboard_count = 0;
	if(_custom_commander.system_start && _custom_commander.drive_mode == MANUAL)
	{
		while (!should_exit())
		{
			/* code */

			_vehicle_status_sub.update(&_status);
			if((_status.nav_state==vehicle_status_s::NAVIGATION_STATE_OFFBOARD)&&(_status.arming_state==vehicle_status_s::ARMING_STATE_ARMED))
			{
				break;
			}
			if(offboard_count == 10)
			{
				publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_OFFBOARD);		//切换为OFFBOARD模式
				publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, ARM);	//解锁

			}
			publish_offboard_control_mode();	//发布offboard模式，actuator control
			usleep(100000);
			if(offboard_count<11)	offboard_count++;
		}
		publish_offboard_control_mode(false,false,false,false,false,true);	//发布offboard模式，actuator control

	}
}

//安全状态检查
bool CustomCommander::safety_check()
{
	bool isSafety = true;
	if(_custom_commander.target_throttle != 0.0f || _custom_commander.target_steeringwheel != 0.0f)
	{
		isSafety = false;
		events::send(events::ID("safe_check"),
		{events::Log::Info, events::LogInternal::Info},
		"current drive_mode : manual");
	}
	return isSafety;
}

bool CustomCommander::init()
{
	// alternatively, Run on fixed interval
	ScheduleOnInterval(10000_us); // 10000 us interval, 100 Hz rate

	return true;
}

void CustomCommander::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}
	PX4_INFO("CustomCommander");
	if(_ui2px4_ignition_sub.update(&_ui2px4_ignition))
	{
		_custom_commander.system_start = _ui2px4_ignition.control_start_stop;
	}

	if(_custom_commander.system_start == true)	//总开关打开后才执行以下动作
	{
		if(_chassis_data_sub.update(&_chassis_data))	//获取档位、油门、方向盘等传感器数据
		{
			gear_commander();		//判断当前档位
			if(_custom_commander.current_gear == GEAR_D || _custom_commander.current_gear == GEAR_R)
			{
				cal_throttle_sheeringwheel();		//计算油门和转向
			}
		}
		if(_ui2px4_mode_sub.update(&_ui2px4_mode))
		{
			_vehicle_status_sub.update(&_status);
			switch (_ui2px4_mode.mode)
			{
				case MANUAL:
					events::send(events::ID("drive_mode_manual"),
					{events::Log::Info, events::LogInternal::Info},
					"current drive_mode : manual");
					_custom_commander.drive_mode = MANUAL;
					break;
				case REMOTE:
					publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_MANUAL);		//切换为MANUAL模式
					publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, ARM);	//解锁
					if((_status.nav_state==vehicle_status_s::NAVIGATION_STATE_MANUAL)&&(_status.arming_state==vehicle_status_s::ARMING_STATE_ARMED))
					{
						_custom_commander.drive_mode = REMOTE;
						events::send(events::ID("drive_mode_remote"),
						{events::Log::Info, events::LogInternal::Info},
						"current drive_mode : remote");
						break;
					}
					break;
				case AUTO:
					_custom_commander.drive_mode = AUTO;
					events::send(events::ID("drive_mode_auto"),
					{events::Log::Info, events::LogInternal::Info},
					"current drive_mode : auto");
					break;
				default:
					break;
			}
		}
		into_offboard_mode();	//MANUAL模式时进入offboard模式
	}
	else{
		_custom_commander.current_gear = GEAR_P;
		_custom_commander.target_steeringwheel = 0.0f;
		_custom_commander.target_throttle = 0.0f;
		_custom_commander.drive_mode = MANUAL;				//默认手动驾驶模式
		publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, DISARM , 21196.f);	//加锁,param2需要为21196才会跳过预检查
	}
	_custom_commander.timestamp = hrt_absolute_time();
	_custom_commander_pub.publish(_custom_commander);

}

int CustomCommander::task_spawn(int argc, char *argv[])
{
	CustomCommander *instance = new CustomCommander();

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

int CustomCommander::print_status()
{
	printf("running...\n");
	return 0;
}

int CustomCommander::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CustomCommander::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Capture steeringwheel and throttle data.Capture ui command.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("custom_commander", "dev");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int custom_commander_main(int argc, char *argv[])
{
	return CustomCommander::main(argc, argv);
}
