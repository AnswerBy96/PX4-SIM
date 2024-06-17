/*
 * @Descripttion:
 * @version:
 * @Author: chenjw
 * @Date: 2023-12-04 03:57:49
 * @LastEditors: rsj
 * @LastEditTime: 2024-06-14 02:58:30
 */
#pragma once

#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/udp/udp.h>
#include <lib/tcp/tcp.h>
#include <lib/uart/uart.h>
#include <lib/can_eth/can_eth.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <termios.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/plc_to_px4.h>
#include <uORB/topics/px4_to_plc.h>

#define Px4_Port 4700		//本地端口

#define PLC_IP "192.168.0.2"
#define PLC_Port 4701
#define UdpFrame_Len 8
#define CanFrame_Len 8
#define EthFrame_Len  13
#define PLC_ID 0xC9
#define PX4_ID 0x65

using namespace time_literals;

class PlcConnectionPx4 : public ModuleBase<PlcConnectionPx4>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	PlcConnectionPx4();
	~PlcConnectionPx4() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;


	void DecodePlcConnectionPx4(unsigned char* buf);


private:
	void Run() override;

	char* uartPortName=(char *)"/dev/ttyS4";
	unsigned char recvCANbuffer[EthFrame_Len];
	unsigned char recvUdpbuffer[8] = {0};

	bool isInit{false};
	uint32_t CanID;

	px4_to_plc_s px4_to_plc_;
	plc_to_px4_s plc_to_px4_;


	//TcpSocket* tcp_;
	UdpSocket* udp_;
	CanEth eth_can;

	unsigned char Send_Can_Msg[CanFrame_Len];


	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::UART_PORTNAME>) _param_uart_portname,
		(ParamInt<px4::params::UART_BAUDRATE>) _param_uart_baudrate,
		(ParamInt<px4::params::MIN_ANGLE>) _param_min_angle,
		(ParamInt<px4::params::MAX_ANGLE>) _param_max_angle,
		(ParamInt<px4::params::MAX_THROTTLE>) _param_max_throttle
	)

	//Publications
	uORB::Publication<plc_to_px4_s> plc_to_px4_pub{ORB_ID(plc_to_px4)};

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription         px4_to_plc_sub{ORB_ID(px4_to_plc)};
};
