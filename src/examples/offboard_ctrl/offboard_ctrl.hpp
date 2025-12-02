#pragma once
#include <px4_platform_common/log.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/action_request.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/SubscriptionCallback.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

class OffboardCtrl : public ModuleBase<OffboardCtrl>,  public px4::ScheduledWorkItem
{
public:
	OffboardCtrl();
	~OffboardCtrl() override;

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
	uORB::Publication<offboard_control_mode_s> _offboard_ctrl_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<action_request_s> _action_request_pub{ORB_ID(action_request)};
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};

	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this,ORB_ID(vehicle_local_position)};

	input_rc_s _input_rc;
	vehicle_status_s _vehicle_status;
	vehicle_local_position_s _vehicle_local_position;
	vehicle_local_position_s _switched_pos;

    	uint64_t _switched_time{};
	uint64_t _current_time{};
	// int _count{1};
	int _flag{0};

	void PublishOffboardMode();
	//bool IsOffboardMode();
	void SwitchMode();
	void PublishTrajectorySetpoint();
};
