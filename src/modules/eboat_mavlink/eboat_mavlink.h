/*
 * @Descripttion:
 * @version:
 * @Author: chenjw
 * @Date: 2023-11-01 02:06:36
 * @LastEditors: rsj
 * @LastEditTime: 2023-11-02 00:55:20
 */
#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/px4_to_ui.h>
#include <uORB/topics/ui_to_px4.h>
using namespace time_literals;

extern "C" __EXPORT int eboat_mavlink_main(int argc, char *argv[]);


class eboat_mavlink : public ModuleBase<eboat_mavlink>, public ModuleParams
{
public:
    eboat_mavlink(int example_param, bool example_flag);

    virtual ~eboat_mavlink() = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static eboat_mavlink *instantiate(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase::run() */
    void run() override;

    /** @see ModuleBase::print_status() */
    int print_status() override;

private:

    /**
     * Check for parameter changes and update them if needed.
     * @param parameter_update_sub uorb subscription to parameter_update
     * @param force for a parameter update
     */
    void parameters_update(bool force = false);


    DEFINE_PARAMETERS(
        (ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
        (ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
    )
    ui_to_px4_s ui_to_px4;
    px4_to_ui_s px4_to_ui;
    // Subscriptions
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
    uORB::Subscription eboat_mavlink_sub{ORB_ID(ui_to_px4)};
    uORB::Publication<px4_to_ui_s>	eboat_mavlink_pub{ORB_ID(px4_to_ui)};
};
