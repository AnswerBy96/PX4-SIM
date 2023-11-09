/*获取底盘数据，如方向盘、油门等，原始数据经过解析后，通过uORB发布*/


#include "ChassisData.hpp"

ChassisData::ChassisData() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

ChassisData::~ChassisData()
{
	isInit = false;
	delete udp;
	delete uart;
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

void ChassisData::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// // clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		switch (_param_uart_portname.get())
		{
			case 0:
				uartPortName = (char *)"/dev/ttyS0";
				break;
			case 1:
				uartPortName = (char *)"/dev/ttyS1";
				break;
			case 2:
				uartPortName = (char *)"/dev/ttyS3";
				break;
			case 3:
				uartPortName = (char *)"/dev/ttyS4";
				break;
			case 4:
				uartPortName = (char *)"/dev/ttyS6";
				break;
			case 5:
				uartPortName = (char *)"/dev/ttyS7";
				break;
			default:
				break;
		}

		uartBaudRate = _param_uart_baudrate.get();

	}
}

bool ChassisData::init()
{
	// parameters_update(true);
	ScheduleOnInterval(20000_us);//单位是微秒
	return true;
}

void ChassisData::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	parameters_update(true);

	if(isInit==false)
	{
		isInit = true;
		udp = new UdpSocket(udpServer_port);
		uart = new Uart(uartPortName,uartBaudRate);
	}

	// uart->send(sendData,sizeof(sendData));
	// if(uart->receive(uartBuffer,sizeof(uartBuffer)))
	// {
	// 	switch (uartBuffer[0])	//检查地址码
	// 	{
	// 		case STEERINGWHEEL:
	// 			for(int i = 0;i<4;i++)
	// 			{
	// 				pendProcessData[i] = uartBuffer[i+2];
	// 			}
	// 			chassisData.timestamp = (int)time((time_t*) NULL);
	// 			chassisData.steeringwheel = decodeSteerWheel(pendProcessData);
	// 			chassisData.throttle = 0.2f;
	// 			_orb_chassisData_pub.publish(chassisData);
	// 			break;
	// 		case THROTTLE:
	// 			for(int i = 0;i<4;i++)
	// 			{
	// 				pendProcessData[i] = uartBuffer[i+2];
	// 			}
	// 			chassisData.timestamp = (int)time((time_t*) NULL);
	// 			//chassisData.throttle = decodeThrottle(pendProcessData);
	// 			_orb_chassisData_pub.publish(chassisData);
	// 			break;
	// 		default:
	// 			break;
	// 	}
	// }

	if(udp->receive(recvCANbuffer,sizeof(recvCANbuffer)))
	{
		eth_can.CanToEth(recvCANbuffer);
		CanID = eth_can.getRecvID();
		switch (CanID)
		{
			case SMC180_ID:
				decodeThrottle_SMC180(eth_can.getRecvData());
				break;

			default:
				break;
		}
		// eth_can.EthToCan(eth_can.getRecvData(),eth_can.getRecvID(),8);
		// udp->send(eth_can.getSendData(),13,"192.168.0.101",4501);
	}

	// perf_end(_loop_perf);
}

int ChassisData::task_spawn(int argc, char *argv[])
{
	ChassisData *instance = new ChassisData();

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

int ChassisData::print_status()
{
	PX4_INFO("running... \n");

	return 0;
}

int ChassisData::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ChassisData::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
	recive steering wheel and throttle raw data,then publish them with uORB;

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("chassis_data", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


extern "C" __EXPORT int chassis_data_main(int argc, char *argv[])
{
	return ChassisData::main(argc, argv);
}
