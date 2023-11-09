#pragma once

#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/chassis_data.h>
#include <uORB/topics/custom_command.h>

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
		AUTO = 1
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
	custom_command_s _custom_command{};

	void auto_commander();
	void gear_commander();

	// Subscriptions
	uORB::Subscription _chassis_data_sub{ORB_ID(chassis_data)};
	// uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	//Publication
	uORB::Publication<custom_command_s> _custom_command_pub{ORB_ID(custom_command)};
};
