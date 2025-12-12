#include "offboard_ctrl.hpp"
#include <math.h>
#include <px4_platform_common/defines.h>

OffboardCtrl::OffboardCtrl() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{

}

OffboardCtrl::~OffboardCtrl()
{

}

bool OffboardCtrl::init()
{
	// execute Run() on every sensor_accel publication
	if (!_vehicle_status_sub.registerCallback())
	{
		PX4_ERR("callback registration failed");
		return false;
	}

	// alternatively, Run on fixed interval
	///ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate
	//ScheduleOnInterval(20000);
	return true;
}

void OffboardCtrl::PublishOffboardMode()
{
	offboard_control_mode_s offboard_ctrl_mode;
	memset(&offboard_ctrl_mode, 0, sizeof(offboard_control_mode_s));

	offboard_ctrl_mode.timestamp = hrt_absolute_time();
	offboard_ctrl_mode.position = false;
	offboard_ctrl_mode.velocity = false;
	offboard_ctrl_mode.acceleration = true;
	_offboard_ctrl_pub.publish(offboard_ctrl_mode);
}

// bool OffboardCtrl::IsOffboardMode()
// {
// 	if (_vehicle_status_sub.updated())
// 	{
// 		_vehicle_status_sub.copy(&_vehicle_status);
// 	}

// 	return (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);
// }
void OffboardCtrl::SwitchMode()
{

	action_request_s action_request;
	action_request.action = action_request_s::ACTION_SWITCH_MODE;
	action_request.source = action_request_s::SOURCE_RC_SWITCH;
	action_request.timestamp = hrt_absolute_time();


	// if(_flag && _input_rc.values[5] < 1600 )
	// {
	// 	action_request.mode = vehicle_status_s::NAVIGATION_STATE_POSCTL;
	// 	_action_request_pub.publish(action_request);
	// 	_flag = 0;
	// }
	// if(!_flag && _input_rc.values[5] > 1600)
	// {
	// 	action_request.mode = vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
	// 	PublishOffboardMode();
	// 	_action_request_pub.publish(action_request);
	// 	_switched_pos = _vehicle_local_position;
	// 	_switched_time = hrt_absolute_time();
	// 	_flag = 1;
	// }

	action_request.mode = vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
	PublishOffboardMode();
	_action_request_pub.publish(action_request);
	if(_count == 1)
	{
		_count ++;
		_switched_pos = _vehicle_local_position;
		_switched_time = hrt_absolute_time();

	}


}



void OffboardCtrl::PublishTrajectorySetpoint()
{
    if(!_flag)
    {
        trajectory_setpoint_s trajectory_setpoint;
        memset(&trajectory_setpoint, 0, sizeof(trajectory_setpoint_s));
        trajectory_setpoint.timestamp = hrt_absolute_time();

        _current_time = hrt_absolute_time();
        const float dt = (_current_time - _switched_time) * 1e-6;


        trajectory_setpoint.position[0] =  _switched_pos.x + 1.0f * sinf(dt*0.5f);
        trajectory_setpoint.position[1] =  _switched_pos.y + 1.0f - 1.0f * cosf(dt*0.5f);
        trajectory_setpoint.position[2] =  _switched_pos.z;

        _trajectory_setpoint_pub.publish(trajectory_setpoint);
	PublishOffboardMode();
    }
}


void OffboardCtrl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if(_input_rc_sub.updated())
	{
		_input_rc_sub.copy(&_input_rc);
	}

	if(_vehicle_local_position_sub.updated())
	{
		_vehicle_local_position_sub.copy(&_vehicle_local_position);
	}

	if (_vehicle_status_sub.updated())
	{
		_vehicle_status_sub.copy(&_vehicle_status);
	}

	SwitchMode();
	PublishTrajectorySetpoint();
}



int OffboardCtrl::task_spawn(int argc, char *argv[])
{
	OffboardCtrl *instance = new OffboardCtrl();

	if (instance)
	{
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init())
		{
			return PX4_OK;
		}

	}
	else
	{
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int OffboardCtrl::print_status()
{
	return 0;

}

int OffboardCtrl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int OffboardCtrl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int offboard_ctrl_main(int argc, char *argv[])
{
	return OffboardCtrl::main(argc, argv);
}
