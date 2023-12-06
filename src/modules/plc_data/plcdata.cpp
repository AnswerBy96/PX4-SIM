#include "plcdata.hpp"

PlcData::PlcData() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	plc_data.power_request = 0;
}

PlcData::~PlcData()
{
	isInit = false;
	delete udp;
	delete uart;
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

void PlcData::parameters_update(bool force)
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

void PlcData::PackgeCanFrame(unsigned char* src,unsigned char* buf)
{
	unsigned char msg[CanFrame_Len] = {0};
	msg[0] = 0x88; //扩展帧
	msg[1] = PX4_CANID;
	msg[5] = buf[0];
	std::memcpy(src,msg,sizeof(msg));
}

void PlcData::DecodePlcData(unsigned char* buf)
{

}

bool PlcData::init()
{
	// parameters_update(true);
	ScheduleOnInterval(20000_us);//单位是微秒
	return true;
}

void PlcData::Run()
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
	}

	if(_orb_plcdata_sub.update(&plc_data))
	{
		unsigned char* buf = reinterpret_cast<unsigned char*>(&plc_data.power_request);
		PackgeCanFrame(Send_Can_Msg,buf);
		udp->send(Send_Can_Msg,CanFrame_Len,CANNET_IP,CANNET_Port);
	}

	if(udp->receive(recvCANbuffer,sizeof(recvCANbuffer)))
	{
		eth_can.CanToEth(recvCANbuffer);
		CanID = eth_can.getRecvID();
		switch (CanID)
		{
			case PLC_CANID:
				DecodePlcData(eth_can.getRecvData());
				break;

			default:
				break;
		}
	}

	// perf_end(_loop_perf);
}

int PlcData::task_spawn(int argc, char *argv[])
{
	PlcData *instance = new PlcData();

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

int PlcData::print_status()
{
	PX4_INFO("running... \n");

	return 0;
}

int PlcData::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PlcData::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
	recive steering wheel and throttle raw data,then publish them with uORB;

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("PlcData", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


extern "C" __EXPORT int plc_data_main(int argc, char *argv[])
{
	return PlcData::main(argc, argv);
}
