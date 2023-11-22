/*
 * @Descripttion:
 * @version:
 * @Author: chenjw
 * @Date: 2023-11-09 18:56:46
 * @LastEditors: rsj
 * @LastEditTime: 2023-11-19 18:19:49
 */
#pragma once

#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <time.h>
#include <termios.h>
#include <math.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/chassis_data.h>
#include <uORB/topics/custom_commander.h>
#include <uORB/topics/ui_to_px4_ignition.h>
#include <uORB/topics/ui_to_px4_mode.h>

using namespace time_literals;


class CustomCommander : public ModuleBase<CustomCommander>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	CustomCommander();
	~CustomCommander() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

	// void parameters_update(bool force);

	enum Gear {
		GEAR_P = 0,
		GEAR_N = 1,
		GEAR_R = 2,
		GEAR_D = 3
	};

	enum DriveMode {
		MANUAL = 0,
		AUTO,
		REMOTE
	};

	struct BoatStatus
	{
		bool isStart;			  // 0-> stop , 1 -> start
		uint8_t throttleGear;	//0 -> P , 1 -> N , 2 -> R , 3 -> D	 		 当前油门传感器档位
		uint8_t commanderGear;	//0 -> P , 1 -> N , 2 -> R , 3 -> D	        状态机输出的目标档位
		uint8_t driveMode;	// 0 -> auto , 1 -> maunal
	}boat_status;

private:
	void Run() override;

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	chassis_data_s _chassis_data{};
	custom_commander_s _custom_commander{};
	ui_to_px4_ignition_s _ui2px4_ignition{};
	ui_to_px4_mode_s _ui2px4_mode{};



	void auto_commander();
	void gear_commander();

	void cal_throttle_sheeringwheel();

	// Subscriptions
	uORB::Subscription _chassis_data_sub{ORB_ID(chassis_data)};
	uORB::Subscription _ui2px4_ignition_sub{ORB_ID(ui_to_px4_ignition)};
	uORB::Subscription _ui2px4_mode_sub{ORB_ID(ui_to_px4_mode)};
	// uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	//Publication
	uORB::Publication<custom_commander_s> _custom_commander_pub{ORB_ID(custom_commander)};
};
