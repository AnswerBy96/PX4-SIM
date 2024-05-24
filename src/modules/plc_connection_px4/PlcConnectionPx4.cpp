#include "PlcConnectionPx4.hpp"

PlcConnectionPx4::PlcConnectionPx4() :
	ModuleParams(nullptr),
	udp(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

PlcConnectionPx4::~PlcConnectionPx4()
{
	isInit = false;
	delete udp;
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

void PlcConnectionPx4::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// // clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

	}
}

void PlcConnectionPx4::DecodePlcConnectionPx4(unsigned char* buf)
{
	// 将 buf 数组中的数据转换为 float 类型变量
	float result;
	std::memcpy(&result, buf, sizeof(float));

	//将结果转换为降额的比例
	plc_to_px4_.derating_ratio = 0;
	plc_to_px4_.derating_ratio = result * 0.01;

	plc_to_px4_.timestamp = hrt_absolute_time();
	plc_to_px4_pub.publish(plc_to_px4_);
}

bool PlcConnectionPx4::init()
{
	ScheduleOnInterval(20000_us);//单位是微秒
	return true;
}

void PlcConnectionPx4::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	parameters_update(true);

	if(!isInit && udp == nullptr)
	{
		isInit = true;
		udp = new UdpSocket(udpServer_port);
	}

	if(px4_to_plc_sub.update(&px4_to_plc_))
	{
		static long long tick_clock = 1;
		unsigned char buf[8] = {0};
		std::memcpy(buf,&px4_to_plc_.power_request,sizeof(px4_to_plc_.power_request));
		eth_can.CanToEth(buf,PX4_CANID,CanFrame_Len);
		if(tick_clock % 15 == 0)
		udp->send(eth_can.getSendData(),EthFrame_Len,CANNET_IP,CANNET_Port);

		tick_clock++ ;

	}

	if(udp->receive(recvCANbuffer,sizeof(recvCANbuffer)))
	{
		eth_can.EthToCan(recvCANbuffer);
		CanID = eth_can.getRecvID();

		switch (CanID)
		{
			case PLC_CANID:
				DecodePlcConnectionPx4(eth_can.getRecvData());
				break;

			default:
				break;
		}
	}

	// perf_end(_loop_perf);
}

int PlcConnectionPx4::task_spawn(int argc, char *argv[])
{
	PlcConnectionPx4 *instance = new PlcConnectionPx4();

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

int PlcConnectionPx4::print_status()
{
	PX4_INFO("running... \n");

	return 0;
}

int PlcConnectionPx4::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PlcConnectionPx4::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
	recive steering wheel and throttle raw data,then publish them with uORB;

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("PlcConnectionPx4", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


extern "C" __EXPORT int plc_connection_px4_main(int argc, char *argv[])
{
	return PlcConnectionPx4::main(argc, argv);
}
