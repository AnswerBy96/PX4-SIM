/*
 * @Descripttion:
 * @version:
 * @Author: rsj
 * @Date: 2024-05-09 01:24:06
 * @LastEditors: rsj
 * @LastEditTime: 2024-05-23 01:19:42
 */
#pragma once

#include <lib/mathlib/mathlib.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/plc_to_px4.h>
#include <uORB/topics/px4_to_plc.h>
#include <lib/can_eth/can_eth.h>
#include <lib/udp/udp.h>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

using namespace time_literals;

#define udpServer_port 5100		//本地端口
#define udpTarget_port 5101
#define udpTarget_ip   "192.168.0.102"

#define can_length 8
#define eth_length 13

#define leftmotor_nodeid 1
#define identifier_sendsdo 0x600
#define identifier_recvsdo 0x580
#define INDEX_ADDR_Motor_Control 0x2001
#define SUB_INDEX_ADDR_Motor_Control 0x00

class MotorDriver : public ModuleBase<MotorDriver>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	MotorDriver();
	~MotorDriver() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;
	void SendSDOMsg(unsigned char nodeid ,unsigned short objIndex,
                                unsigned char subIndex,int16_t value);

	bool isInit{false};
	unsigned char recvCANbuffer[13];
	unsigned char CanData[8];


	UdpSocket* udp;
	CanEth can_eth;

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	actuator_motors_s actuator_motors_;
	px4_to_plc_s      px4_to_plc_;
	plc_to_px4_s      plc_to_px4_;
	int16_t motor_pwm_;

    // Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
    	uORB::Subscription actuator_motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription plc_to_px4_sub{ORB_ID(plc_to_px4)};

//     // Publications
	uORB::Publication<px4_to_plc_s>  px4_to_plc_pub{ORB_ID(px4_to_plc)};
};
