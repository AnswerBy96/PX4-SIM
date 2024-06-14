#include "PlcConnectionPx4.hpp"

PlcConnectionPx4::PlcConnectionPx4() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::plc_px4),
	udp_(nullptr)
{
	// tcp_ = new TcpSocket();
}

PlcConnectionPx4::~PlcConnectionPx4()
{
	isInit = false;
	// delete tcp_;
	delete udp_;
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

void PlcConnectionPx4::DecodePlcConnectionPx4(unsigned char* buf)
{
	// 将 buf 数组中的数据转换为 float 类型变量
	float result;
	std::memcpy(&result, buf + 1, sizeof(float));

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

	// if(!isInit && tcp_ == nullptr)
	// {
	// 	isInit = true;
	// 	tcp_->InitSocket(CANNET_Port);
	// }

	if(!isInit && udp_ == nullptr)
	{
		isInit = true;
		udp_ = new UdpSocket(Px4_Port);
	}

	// unsigned char buf[13] = {"hello world"};
	// int ret = tcp_->sendmsg(buf,EthFrame_Len);
	// printf("ret: %d\n",ret);

	// if(px4_to_plc_sub.update(&px4_to_plc_))
	// {
	// 	unsigned char buf[8] = {0};
	// 	std::memcpy(buf,&px4_to_plc_.power_request,sizeof(px4_to_plc_.power_request));
	// 	printf("before send plc power_request : %f\n",px4_to_plc_.power_request);
	// 	eth_can.CanToEth(buf,PX4_CANID,CanFrame_Len);
	// 	tcp_->sendmsg(eth_can.getSendData(),EthFrame_Len);
	// 	tcp_->sendmsg(eth_can.getSendData(),EthFrame_Len);
	// 	tcp_->sendmsg(eth_can.getSendData(),EthFrame_Len);
	// }

	// if(tcp_->receive(recvCANbuffer,sizeof(recvCANbuffer)))
	// {
	// 	eth_can.EthToCan(recvCANbuffer);
	// 	CanID = eth_can.getRecvID();

	// 	switch (CanID)
	// 	{
	// 		case PLC_CANID:
	// 			DecodePlcConnectionPx4(eth_can.getRecvData());
	// 			break;

	// 		default:
	// 			break;
	// 	}
	// }

	if(px4_to_plc_sub.update(&px4_to_plc_))
	{
		unsigned char buf[UdpFrame_Len] = {0};
		buf[0] = PX4_ID;
		std::memcpy(buf + 1,&px4_to_plc_.power_request,sizeof(px4_to_plc_.power_request));
		printf("before send plc power_request : %f\n",px4_to_plc_.power_request);
		int ret = udp_->send(buf,UdpFrame_Len,PLC_IP,PLC_Port);
		printf("send1 : %d\n",ret);
	}

	if(udp_->receive(recvUdpbuffer,sizeof(recvUdpbuffer)))
	{
		// eth_can.EthToCan(recvCANbuffer);
		// CanID = eth_can.getRecvID();

		// switch (CanID)
		// {
		// 	case PLC_CANID:
		// 		DecodePlcConnectionPx4(eth_can.getRecvData());
		// 		break;

		// 	default:
		// 		break;
		// }
		if(recvUdpbuffer[0] == PLC_ID)
		{
			DecodePlcConnectionPx4(recvUdpbuffer);
		}

		//清除ID缓存 避免重复解析
		recvUdpbuffer[0] = 0;
	}
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
