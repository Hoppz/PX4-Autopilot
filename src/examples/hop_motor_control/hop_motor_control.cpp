#include "hop_motor_control.hpp"
#include <px4_platform_common/posix.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>



HopMotorControl::HopMotorControl() :
	ModuleParams(nullptr)
{
	_vehicle_land_detected.landed = true;

	_vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	_vehicle_status.system_id = 1;
	_vehicle_status.component_id = 1;
	_vehicle_status.system_type = 0;
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_UNKNOWN;
	// _vehicle_status.nav_state = _user_mode_intention.get();
	// _vehicle_status.nav_state_user_intention = _user_mode_intention.get();
	_vehicle_status.nav_state_timestamp = hrt_absolute_time();
	_vehicle_status.gcs_connection_lost = true;
	_vehicle_status.power_input_valid = true;

	updateParameters();
}

int HopMotorControl::task_spawn(int argc, char *argv[])
{
	//px4_task_spawn_cmd 是一个用于在 PX4 飞行控制固件中启动任务的函数。
	// 它允许你为新任务指定名称、调度器、优先级、堆栈大小、入口函数。
	_task_id = px4_task_spawn_cmd("HopMotorControl",
			SCHED_DEFAULT,
			SCHED_PRIORITY_DEFAULT,
			1024,
			(px4_main_t)&run_trampoline,
			(char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	// wait until task is up & running
	if (wait_until_running() < 0) {
		_task_id = -1;
		return -1;
	}

	return 0;
}

HopMotorControl *HopMotorControl::instantiate(int argc, char *argv[])
{
	HopMotorControl *instance = new HopMotorControl();
	if( instance == nullptr ){
		PX4_ERR("alloc failed");
	}
	return instance;
}

bool hop_check_motor(float value)
{
	if (value < -1.f || value > 1.f) {
		HopMotorControl::print_usage("value invalid");
		return false;
	}
	return true;
}

void hop_actuator_test(int function, float value, int timeout_ms, bool release_control)
{
	actuator_test_s actuator_test{};
	actuator_test.timestamp = hrt_absolute_time();
	actuator_test.function = function;
	actuator_test.value = value;
	actuator_test.action = release_control ? actuator_test_s::ACTION_RELEASE_CONTROL : actuator_test_s::ACTION_DO_CONTROL;
	actuator_test.timeout_ms = timeout_ms;

	uORB::Publication<actuator_test_s> actuator_test_pub{ORB_ID(actuator_test)};
	actuator_test_pub.publish(actuator_test);
}

//切换飞行模式
static bool send_vehicle_command(const uint32_t cmd, const float param1 = NAN, const float param2 = NAN,
				 const float param3 = NAN,  const float param4 = NAN, const double param5 = static_cast<double>(NAN),
				 const double param6 = static_cast<double>(NAN), const float param7 = NAN)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;
	vcmd.param4 = param4;
	vcmd.param5 = param5;
	vcmd.param6 = param6;
	vcmd.param7 = param7;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = vehicle_status_sub.get().system_id;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = vehicle_status_sub.get().component_id;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	return vcmd_pub.publish(vcmd);
}

int HopMotorControl::custom_command(int argc, char *argv[])
{

	if( is_running() ){
		PX4_INFO(" hop_motor_control already running! ");
		return 0;
	}


	int function = 0;
	float motor_in[5] = {0.0f,0.0f,0.0f,0.0f,0.0f};
	int ch;
	uint64_t timeout_ms = 0;

	int myoptind = 1;
	const char* myoptarg = nullptr;

	while( ( ch = px4_getopt(argc,argv, "a:b:c:d:t:", &myoptind, &myoptarg) ) != EOF ){

		// PX4_INFO("",myoptarg);

		switch (ch){


		case 'a':
			motor_in[0] = strtof(myoptarg,nullptr);
			function |= 1;
			if( !hop_check_motor( motor_in[0] ) ){ return 1; }
			break;

		case 'b':
			function |= 2;
			motor_in[1] = strtof(myoptarg,nullptr);
			if( !hop_check_motor( motor_in[1] ) ){ return 1; }
			break;

		case 'c':
			function |= 4;
			motor_in[2] = strtof(myoptarg,nullptr);
			if( !hop_check_motor( motor_in[2] ) ){ return 1; }
			break;

		case 'd':
			function |= 8;
			motor_in[3] = strtof(myoptarg,nullptr);
			if( !hop_check_motor( motor_in[3] ) ){ return 1; }
			break;

		case 't':
			timeout_ms = strtof(myoptarg, nullptr) * 1000.f;
			break;
		default:
			print_usage(nullptr);
			return 1;
		}
	}


	PX4_INFO("function: %d",function);
	PX4_INFO("m1: %.2f, m2: %.2f, m3: %.2f, m4: %.2f",(double)motor_in[0],(double)motor_in[1],(double)motor_in[2],(double)motor_in[3]);

	PX4_INFO("argv[0]: %s  %d  %s",argv[0],myoptind, argv[myoptind]);

	// if( myoptind >=0 && myoptind < argc ){
		if( strcmp( "set", argv[0] ) == 0 ){
			// 只适用于 4 轴


			if( timeout_ms != 0 ){	// 有时间的控制

				for(int i = 0; i < 4 ; i++){
					if( (function>>i) & 1  ){
						if( motor_in[i] > 9.f ){
							print_usage("Missing argument: value");
							return 3;
						}
// 使用  v6x 的时候必须要 lld 而使用 sitl 的时候要用 ld
#ifdef __NUTTX_CONFIG_PX4_FMU_V6X_INCLUDE_BOARD_H
						PX4_INFO("motor : %d,  function : %d, value: %.2f, time : %lld",i, function, (double)motor_in[i], timeout_ms);
#else
						PX4_INFO("motor : %d,  function : %d, value: %.2f, time : %ld",i, function, (double)motor_in[i], timeout_ms);
#endif
						int inner_function = actuator_test_s::FUNCTION_MOTOR1 + i;

						if( inner_function == 0 ){
							print_usage("Missing argument: function");
							return 4;
						}

						hop_actuator_test(inner_function, motor_in[i], timeout_ms, false);
					}
				}

				// 记录时间，转换模式
				uint64_t time_trans_hold = hrt_absolute_time();
				// PX4_INFO("------mode change wait begin %ld------",hrt_absolute_time());

				while( ( hrt_absolute_time() - time_trans_hold ) < timeout_ms ){}
				// PX4_INFO("------mode change wait end %ld------",hrt_absolute_time());

				bool mode_change_flag = send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_POSCTL,
						     PX4_CUSTOM_SUB_MODE_POSCTL_SLOW);

				if( mode_change_flag ) PX4_INFO("------mode change succese------");
				else PX4_INFO("------mode change fail------");

			} else {		// 无时间的控制
				for(int i = 0; i < 4 ; i++){

					if( (function>>i) & 1  ){
						if( motor_in[i] > 9.f ){
							print_usage("Missing argument: value");
							return 3;
						}
#ifdef __NUTTX_CONFIG_PX4_FMU_V6X_INCLUDE_BOARD_H
						PX4_INFO("motor : %d,  function : %d, value: %.2f, time : %lld",i, function, (double)motor_in[i], timeout_ms);
#else
						PX4_INFO("motor : %d,  function : %d, value: %.2f, time : %ld",i, function, (double)motor_in[i], timeout_ms);
#endif
						int inner_function = actuator_test_s::FUNCTION_MOTOR1 + i;

						if( inner_function == 0 ){
							print_usage("Missing argument: function");
							return 4;
						}


						hop_actuator_test(inner_function, motor_in[i], 0, false);
					}
				}

				/* stop on any user request */
				PX4_INFO("Press Enter to stop");
				char c;
				ssize_t ret = read(0, &c, 1);

				if (ret < 0) {
					PX4_ERR("read failed: %i", errno);
				}

				// for(int i = 0; i < 4;  i++){
				// 	if( (function >> i) & 1 ){
				// 		int inner_function = actuator_test_s::FUNCTION_MOTOR1 + i;
				// 		hop_actuator_test(inner_function, NAN, 0, true);

				// 	}
				// }
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_POSCTL,
						     PX4_CUSTOM_SUB_MODE_POSCTL_SLOW);


			}
			PX4_INFO("------test end, exit hop motor------");
			return 0;
		}
	// }

	print_usage(nullptr);

	return 0;
}

int HopMotorControl::print_status()
{
	PX4_INFO("HopMotorControl Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}


void HopMotorControl::run()
{
	PX4_INFO("INFO_run");

	// 切换到 offboard 模式
	// should_exit -> #include <px4_platform_common/posix.h>
	while (!should_exit()) {

		_vehicle_status_sub.copy(&_vehicle_status);
		if ((_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD)
			&& (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)) {
			PX4_INFO("in offboard and armed");
			break;
		}

		_offboard_control_mode.timestamp = hrt_absolute_time();

		_vehicle_local_position_sub.copy(&_vehicle_local_position);
		_vehicle_local_position_setpoint.x = _vehicle_local_position.x;
		_vehicle_local_position_setpoint.y = _vehicle_local_position.y;
		_vehicle_local_position_setpoint.z = _vehicle_local_position.z;
		_vehicle_local_position_setpoint.timestamp = hrt_absolute_time();
		_trajectory_setpoint_pub.publish(_vehicle_local_position_setpoint);
		_offboard_control_mode.position = true;
		_offboard_control_mode.timestamp = hrt_absolute_time();
		_offboard_control_mode_pub.publish(_offboard_control_mode);

		// 更新模式为 OFFBOARD
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_OFFBOARD);

		// 暂停线程 10ms
		usleep(10000);
		// 解锁飞行器
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,
			static_cast<float>(vehicle_command_s::ARMING_ACTION_ARM),
			0.0f);
	}
}


int HopMotorControl::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s\n", reason);
	}
	// 字符串前面 + R 会自动换行，不用 \n
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Utility to test actuators.

WARNING: remove all props before using this command.
)DESCR_STR");

	/**
	 * 	actuator_test set -m 1 -v 0.5 -t 2
	 * 	|              |   |
	 *     name      command  param
	 */

	PRINT_MODULE_USAGE_NAME("hop_motor_control", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set", "Set an actuator to a specific output value");
	PRINT_MODULE_USAGE_PARAM_COMMENT("\t");
	PRINT_MODULE_USAGE_PARAM_FLOAT('a', 0, -1, 1, "control motor 1 value (-1...1)", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('b', 0, -1, 1, "control motor 2 value (-1...1)", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('c', 0, -1, 1, "control motor 3 value (-1...1)", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('d', 0, -1, 1, "control motor 4 value (-1...1)", true);
	PRINT_MODULE_USAGE_PARAM_INT('t', 0, 0, 100, "Timeout in seconds (run interactive if not set)", true);

	return 0;
}




int hop_motor_control_main(int argc, char *argv[])
{

	return HopMotorControl::main(argc,argv);
}


void HopMotorControl::updateParameters()
{
	// check for parameter updates
	updateParams();
}



