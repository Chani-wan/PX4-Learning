#pragma once
#include <px4_platform_common/log.h>
#include <uORB/uORB.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/person_example.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>

class PersonReceiver : public px4::ScheduledWorkItem
{
public:
	PersonReceiver():px4::ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {}
	~PersonReceiver() {}

	void Init()
	{
		_person_sub.registerCallback();
	}

private:
	uORB::SubscriptionCallbackWorkItem _person_sub{this,ORB_ID(person_example)};
	void Run() override{
		person_example_s msg;
		if(_person_sub.update(&msg)){
			PX4_INFO("Message received!");
			orb_print_message_internal(ORB_ID(person_example), &msg, true);
		}
	}

};
