#include <math.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/action_request.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/input_rc.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <px4_platform_common/defines.h>


class RemoteControlSim: public px4::ScheduledWorkItem  {
private:
	int _count{};
	int _count2{};
	float radius = 1.0f;   // 圆的半径（米）
	float center_y_offset = 0.5f; // 圆心在y方向的偏移
	uint64_t _switched_time{};

	uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub {this, ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};

	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<offboard_control_mode_s> _offboard_ctrl_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<action_request_s> _action_request_pub{ORB_ID(action_request)};

	trajectory_setpoint_s trajectory_setpoint{};
	vehicle_local_position_s _vehicle_local_position{};
	offboard_control_mode_s offboard_ctrl_mode{};
	action_request_s action_request{};
	vehicle_status_s _vehicle_status{};
	input_rc_s _input_rc{};

	void Run() override {

		if(_vehicle_local_position_sub.updated())
		{
			if(_count2 == 0)
			{
				_vehicle_local_position_sub.copy(&_vehicle_local_position);
				_count2++;
			}

			if(_count++ >= 10)
			{
				_count = 0;
				switch_2_offboard_sim();

			}
			pub_traj();
		}
	}

public:
	RemoteControlSim(): px4::ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {}
	void Init() {
		_vehicle_local_position_sub.registerCallback();
		_switched_time = hrt_absolute_time();
	}

	bool IsOffboardMode()
	{
		if (_vehicle_status_sub.updated())
		{
			_vehicle_status_sub.copy(&_vehicle_status);
		}

		return (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);
	}
	void switch_2_offboard_sim(){
		offboard_ctrl_mode.timestamp = hrt_absolute_time();
		offboard_ctrl_mode.position = true;
		offboard_ctrl_mode.velocity = true;
		offboard_ctrl_mode.acceleration = false;
		_offboard_ctrl_pub.publish(offboard_ctrl_mode);

		action_request.timestamp = hrt_absolute_time();
		action_request.action = action_request_s::ACTION_SWITCH_MODE;
		action_request.source = action_request_s::SOURCE_RC_SWITCH;
		action_request.mode = vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
		_action_request_pub.publish(action_request);
	}
	void pub_traj()
	{
		if(IsOffboardMode())
		{
			trajectory_setpoint.timestamp = hrt_absolute_time();

			const float dt = (hrt_absolute_time() - _switched_time) * 1e-6f;

			trajectory_setpoint.position[0] = _vehicle_local_position.x + radius * sinf(dt);
			trajectory_setpoint.position[1] = _vehicle_local_position.y + center_y_offset - radius * cosf(dt);
			trajectory_setpoint.position[2] = _vehicle_local_position.z;

			_trajectory_setpoint_pub.publish(trajectory_setpoint);
		}


	}


};
