/*
 * @Descripttion:
 * @version:
 * @Author: chenjw
 * @Date: 2023-11-01 02:06:28
 * @LastEditors: rsj
 * @LastEditTime: 2024-05-23 18:39:59
 */
#include "MotorDriver.hpp"

MotorDriver::MotorDriver() :
	ModuleParams(nullptr),
	udp(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{

}

MotorDriver::~MotorDriver()
{
	isInit = false;
	udp = nullptr;
	delete udp;
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool MotorDriver::init()
{
	// alternatively, Run on fixed interval
	ScheduleOnInterval(20000_us); // 2000 us interval, 200 Hz rate

	return true;
}

void MotorDriver::SendSDOMsg(unsigned char nodeid,unsigned short objIndex,
                             unsigned char subIndex,int16_t value)
{
	unsigned char msg[8];
        msg[0] = 0x2B;
        msg[1] = objIndex & 0xff;
        msg[2] = (objIndex >> 8) & 0xff;
        msg[3] = subIndex;
        msg[4] = value & 0xff;
        msg[5] = (value >> 8) & 0xff;
        msg[6] = (value >> 16) & 0xff;
        msg[7] = (value >> 24) & 0xff;

	uint32_t can_id = identifier_sendsdo + nodeid;
	can_eth.CanToEth(msg,can_id,can_length);
	udp->send(can_eth.getSendData(),eth_length,udpTarget_ip,udpTarget_port);
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
		udp = new UdpSocket(udpServer_port);
	}

	if(actuator_motors_sub.update(&actuator_motors_))
	{
		//(-1,1) -> (-1000,1000)
		motor_pwm_ = actuator_motors_.control[0] * 1000;

		//motor_pwm根据标定结果转换为功率
		px4_to_plc_.power_request = 100;
		px4_to_plc_.timestamp = hrt_absolute_time();

		px4_to_plc_pub.publish(px4_to_plc_);

		udp->receive(recvCANbuffer,sizeof(recvCANbuffer));


		if(plc_to_px4_sub.update(&plc_to_px4_))
		{
			motor_pwm_ = motor_pwm_ * plc_to_px4_.derating_ratio;
			SendSDOMsg(leftmotor_nodeid,INDEX_ADDR_Motor_Control,SUB_INDEX_ADDR_Motor_Control,(int16_t)motor_pwm_);
		}
	}

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
