/*
 * @Descripttion:
 * @version:
 * @Author: chenjw
 * @Date: 2023-11-01 02:06:36
 * @LastEditors: rsj
 * @LastEditTime: 2024-05-10 23:43:25
 */
#pragma once

#include <lib/mathlib/mathlib.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/px4_to_ui.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/custom_commander.h>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
using matrix::Dcmf;

using namespace time_literals;


class eboat_mavlink : public ModuleBase<eboat_mavlink>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	eboat_mavlink();
	~eboat_mavlink() override;

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

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

    px4_to_ui_s px4_to_ui_;
    vehicle_local_position_s vehicle_local_position_;
    vehicle_attitude_s vehicle_attitude_;
    custom_commander_s custom_commander_;

    // Subscriptions
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
    uORB::Subscription vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription custom_commander_sub{ORB_ID(custom_commander)};
    uORB::Publication<px4_to_ui_s>	eboat_mavlink_pub{ORB_ID(px4_to_ui)};
};
