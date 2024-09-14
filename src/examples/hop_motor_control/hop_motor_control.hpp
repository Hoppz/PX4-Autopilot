#pragma once


#include "commander/px4_custom_mode.h"
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionInterval.hpp>
// uORB topics
#include <uORB/topics/actuator_test.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>

using namespace time_literals;

extern "C" __EXPORT int hop_motor_control_main(int argc, char *argv[]);

class HopMotorControl : public ModuleBase<HopMotorControl>, public ModuleParams
{
public:
	HopMotorControl();
	virtual ~HopMotorControl() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static HopMotorControl *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;
private:
	void updateParameters();


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)

	hrt_abstime time_tick_ms;
		// 	将无人机的各项信息 加载为内部变量
	// 位置信息
	vehicle_local_position_s _vehicle_local_position;
	//
	vehicle_status_s			_vehicle_status;
	vehicle_command_s			_vehicle_command = {};
	offboard_control_mode_s			_offboard_control_mode{};
	position_setpoint_triplet_s 		_position_setpoint_triplet{};
	vehicle_command_ack_s 			_vehicle_command_ack{};
	vehicle_local_position_setpoint_s 	_vehicle_local_position_setpoint{};
	vehicle_attitude_setpoint_s 		_vehicle_attitude_setpoint{};
	vehicle_land_detected_s			_vehicle_land_detected{};		// 触地检测

	// Subscriptions
	uORB::SubscriptionInterval 		_parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription 			_vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription 			_vehicle_status_sub{ORB_ID(vehicle_status)};

	// Publications
	uORB::Publication<offboard_control_mode_s>		_offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<vehicle_command_s>			_vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<position_setpoint_triplet_s>		_position_setpoint_triplet_pub{ORB_ID(position_setpoint_triplet)};
	uORB::Publication<vehicle_local_position_setpoint_s>	_trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};

};
