/*
 * @Descripttion:
 * @version:
 * @Author: rsj
 * @Date: 2024-05-09 01:24:06
 * @LastEditors: rsj
 * @LastEditTime: 2025-01-20 00:06:16
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
#include <uORB/topics/motor_state.h>
#include <uORB/topics/custom_commander.h>
#include <lib/can_eth/can_eth.h>
#include <lib/udp/udp.h>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/can_communication/can_communication.h>
#include <modules/custom_commander/custom_commander.hpp>

using namespace time_literals;

#define GET_MOTOR_POWER(x) 0.0000000713*x*x*x - 0.0001346823*x*x + 0.1609654855*x - 6.8407223933

#define udpServer_port 5100		//本地端口
#define udpTarget_port 5101
#define udpTarget_ip   "192.168.0.102"

#define eth_length 13

#define motor1_nodeid 1
#define motor2_nodeid 2
#define INDEX_ADDR_Motor_Control 0x2001
#define SUB_INDEX_ADDR_Motor_Control 0x00

#define INDEX_ADDR_Motor_Speed	 0x210A
#define SUB_INDEX_ADDR_Motor_Speed 0x00

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
	void GetMotorSpeed(uint32_t &motorspeed,unsigned char * data);
	bool isInit{false};
	unsigned char recvCANbuffer[13];


	UdpSocket* udp_;
	CanEth can_eth_;
	can_frame canframe_;
	can_communication canopen_motor_;

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	actuator_motors_s actuator_motors_;
	px4_to_plc_s      px4_to_plc_;
	plc_to_px4_s      plc_to_px4_;
	motor_state_s     motor_state_;
	custom_commander_s custom_commander_;

	int16_t motor1_pwm_;
	int16_t motor2_pwm_;

	float motor1_power_;
	float motor2_power_;

	float   last_power_request_;

    // Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
    	uORB::Subscription actuator_motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription plc_to_px4_sub{ORB_ID(plc_to_px4)};
	uORB::Subscription custom_commander_sub{ORB_ID(custom_commander)};

//     // Publications
	uORB::Publication<px4_to_plc_s>  px4_to_plc_pub{ORB_ID(px4_to_plc)};
	uORB::Publication<motor_state_s>  motor_state_pub{ORB_ID(motor_state)};
};
