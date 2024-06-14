/*
 * @Descripttion:
 * @version:
 * @Author: chenjw
 * @Date: 2023-11-01 02:06:28
 * @LastEditors: rsj
 * @LastEditTime: 2024-06-13 02:50:16
 */
#include "MotorDriver.hpp"

MotorDriver::MotorDriver() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::plc_px4),
	udp_(nullptr)
{
	last_power_request_ = 0;
}

MotorDriver::~MotorDriver()
{
	isInit = false;
	udp_ = nullptr;
	delete udp_;
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool MotorDriver::init()
{
	// alternatively, Run on fixed interval
	ScheduleOnInterval(20000_us);

	return true;
}

void MotorDriver::GetMotorSpeed(uint32_t &motorspeed,unsigned char * data)
{
	if(data[0] == 0x40)
	{
		if(data[1] == 0x0A && data[2] == 0x21)
		{
			std::memcpy(&motorspeed,data + 4,sizeof(uint32_t));
		}
	}
	else
	{
		//读取失败
		return;
	}
}

void MotorDriver::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if(!isInit)
	{
		isInit = true;
		udp_ = new UdpSocket(udpServer_port);
	}

	custom_commander_sub.update(&custom_commander_);

	if(((custom_commander_.drive_mode == DriveMode::MANUAL&&
	(custom_commander_.current_gear == Gear::GEAR_D )||
	(custom_commander_.current_gear == Gear::GEAR_R )) ||
	(custom_commander_.drive_mode == DriveMode::REMOTE) ||
	(custom_commander_.drive_mode == DriveMode::AUTO)) && (actuator_motors_sub.update(&actuator_motors_)))
	{
		//(-1,1) -> (-1000,1000)
		motor1_pwm_ = actuator_motors_.control[0] * 1000;
		motor2_pwm_ = actuator_motors_.control[1] * 1000;

		//motor_pwm根据标定结果转换为功率
		if(motor1_pwm_ < 50)
		{
			motor1_pwm_ = 0;
			motor1_power_ = 0;
		}
		else
		{
			motor1_power_ = GET_MOTOR_POWER(motor1_pwm_);
		}

		if(motor2_pwm_ < 50)
		{
			motor2_pwm_ = 0;
			motor2_power_ = 0;
		}
		else
		{
			motor2_power_ = GET_MOTOR_POWER(motor2_pwm_);
		}


		px4_to_plc_.power_request = motor1_power_ + motor2_power_;

		if(last_power_request_ != px4_to_plc_.power_request)
		{
			last_power_request_ = px4_to_plc_.power_request;
			px4_to_plc_.timestamp = hrt_absolute_time();
			printf("power_request : %f\n",px4_to_plc_.power_request);
			px4_to_plc_pub.publish(px4_to_plc_);
		}

		//收到PLC降额指令
		if(plc_to_px4_sub.update(&plc_to_px4_))
		{
			motor1_pwm_ = motor1_pwm_ * plc_to_px4_.derating_ratio;
			canframe_ = canopen_motor_.PackageSdo(motor1_nodeid,INDEX_ADDR_Motor_Control,SUB_INDEX_ADDR_Motor_Control,(int16_t)motor1_pwm_);
			can_eth_.CanToEth(canframe_.data,canframe_.can_id,canframe_.can_dlc);
			udp_->send(can_eth_.getSendData(),eth_length,udpTarget_ip,udpTarget_port);

			motor2_pwm_ = motor2_pwm_ * plc_to_px4_.derating_ratio;
			canframe_ = canopen_motor_.PackageSdo(motor2_nodeid,INDEX_ADDR_Motor_Control,SUB_INDEX_ADDR_Motor_Control,(int16_t)motor2_pwm_);
			can_eth_.CanToEth(canframe_.data,canframe_.can_id,canframe_.can_dlc);
			udp_->send(can_eth_.getSendData(),eth_length,udpTarget_ip,udpTarget_port);
		}

		//获取电机方向
		if(actuator_motors_.control[0] < 0)
		{
			motor_state_.motor1_direction = 1;
		}
		else
		{
			motor_state_.motor1_direction = 0;
		}

		if(actuator_motors_.control[1] < 0)
		{
			motor_state_.motor2_direction = 1;
		}
		else
		{
			motor_state_.motor2_direction = 0;
		}

		//获取电机转速
		// canframe_ = canopen_motor_.PackageSdo(motor1_nodeid,INDEX_ADDR_Motor_Speed,SUB_INDEX_ADDR_Motor_Speed,0);
		// can_eth_.CanToEth(canframe_.data,canframe_.can_id,canframe_.can_dlc);
		// udp_->send(can_eth_.getSendData(),eth_length,udpTarget_ip,udpTarget_port);

		// canframe_ = canopen_motor_.PackageSdo(motor2_nodeid,INDEX_ADDR_Motor_Speed,SUB_INDEX_ADDR_Motor_Speed,0);
		// can_eth_.CanToEth(canframe_.data,canframe_.can_id,canframe_.can_dlc);
		// udp_->send(can_eth_.getSendData(),eth_length,udpTarget_ip,udpTarget_port);

		// if(udp_->receive(recvCANbuffer,sizeof(recvCANbuffer)))
		// {
		// 	can_eth_.EthToCan(recvCANbuffer);

		// 	switch (can_eth_.getRecvID())
		// 	{
		// 		case(identifier_recvsdo + motor1_nodeid):
		// 			GetMotorSpeed(motor_state_.motor1_speed,can_eth_.getRecvData());
		// 			break;

		// 		case(identifier_recvsdo + motor2_nodeid):
		// 			GetMotorSpeed(motor_state_.motor2_speed,can_eth_.getRecvData());
		// 			break;

		// 		default:
		// 			break;
		// 	}
		// }

		motor_state_pub.publish(motor_state_);
	}
	udp_->receive(recvCANbuffer,sizeof(recvCANbuffer));
}

int MotorDriver::task_spawn(int argc, char *argv[])
{
	MotorDriver *instance = new MotorDriver();

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

int MotorDriver::print_status()
{
	printf("running...\n");
	return 0;
}

int MotorDriver::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MotorDriver::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("MotorDriver", "dev");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int motor_driver_main(int argc, char *argv[])
{
	return MotorDriver::main(argc, argv);
}
