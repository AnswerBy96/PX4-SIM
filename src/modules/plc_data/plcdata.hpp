/*
 * @Descripttion:
 * @version:
 * @Author: chenjw
 * @Date: 2023-12-04 03:57:49
 * @LastEditors: rsj
 * @LastEditTime: 2023-12-05 01:23:23
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
#include <uORB/topics/plc_data.h>

#define udpServer_port 4500		//本地端口

#define CANNET_IP "192.168.0.101"
#define CANNET_Port 4501
#define CanFrame_Len 13
#define PLC_CANID 0xC9
#define PX4_CANID 0x65

using namespace time_literals;

class PlcData : public ModuleBase<PlcData>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	PlcData();
	~PlcData() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

	void parameters_update(bool force);


	uint16_t crcCheck(unsigned char* pendBuffer);

	unsigned char* PackgeCanFrame(unsigned char* buf);
	void DecodePlcData(unsigned char* buf);


private:
	void Run() override;

	char* uartPortName=(char *)"/dev/ttyS4";
	unsigned char recvCANbuffer[CanFrame_Len];

	bool isInit{false};
	uint32_t CanID;

	plc_data_s plc_data;

	UdpSocket* udp;
	Uart* uart;
	CanEth eth_can;

	unsigned char Send_Can_Msg[CanFrame_Len];

	// Publications
	uORB::Publication<plc_data_s> _orb_plcdata_pub{ORB_ID(plc_data)};

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

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription         _orb_plcdata_sub{ORB_ID(plc_data)};
};
