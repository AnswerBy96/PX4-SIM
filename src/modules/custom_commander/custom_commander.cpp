#include "custom_commander.hpp"

CustomCommander::CustomCommander() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_custom_commander.current_gear = GEAR_P;
	_custom_commander.target_steeringwheel = 0.0f;
	_custom_commander.target_throttle = 0.0f;
	_custom_commander.drive_mode = 0;				//默认手动驾驶模式
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
		return;
	}
	else if(_chassis_data.gear_left == GEAR_P || _chassis_data.gear_right == GEAR_P)
	{
		_custom_commander.current_gear = GEAR_P;
		_custom_commander.system_error = _custom_commander.system_error & 0xFE;
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

void CustomCommander::cal_throttle_sheeringwheel()
{
	static int8_t averageThrottle = 0;
	if(_chassis_data.have_steeringwheel)	//如果有方向盘则转向信号使用方向盘，否则使用两个推杆的差
		_custom_commander.target_steeringwheel = _chassis_data.steeringwheel;
	else
	{
		if(abs(_chassis_data.throttle_left - _chassis_data.throttle_right)>=15)
			_custom_commander.target_steeringwheel = (_chassis_data.throttle_left - _chassis_data.throttle_right) / 100.0f;
		else
			_custom_commander.target_steeringwheel = 0.0f;
	}

	if(abs(_chassis_data.throttle_left - _chassis_data.throttle_right)<=10)
	{
		averageThrottle = (_chassis_data.throttle_left + _chassis_data.throttle_right) / 2 ;

		_custom_commander.target_throttle = averageThrottle / 100.0f;

	}
}

bool CustomCommander::init()
{
	// alternatively, Run on fixed interval
	ScheduleOnInterval(10000_us); // 2000 us interval, 200 Hz rate

	return true;
}

void CustomCommander::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if(_ui2px4_sub.copy(&_ui2px4))
	{
		_custom_commander.system_start = _ui2px4.control_start_stop;
	}
	_ui2px4.mode = MANUAL;
	switch (_ui2px4.mode)
	{
		case MANUAL:
			_custom_commander.drive_mode = MANUAL;
			if(_chassis_data_sub.copy(&_chassis_data))
			{
				gear_commander();
				if(_custom_commander.current_gear == GEAR_D || _custom_commander.current_gear == GEAR_R)
				{
					_chassis_data.have_steeringwheel = false;	//测试用，实际使用删除
					cal_throttle_sheeringwheel();		//计算油门和转向
				}
			}
			break;
		case AUTO:
			_custom_commander.drive_mode = AUTO;
			break;
		default:
			break;
	}
	_custom_commander.timestamp = (int)time((time_t*) NULL);
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
Example of a simple module running out of a work queue.

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
