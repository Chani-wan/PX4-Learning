#pragma once
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>

class SchedWorkItemTest : public px4::ScheduledWorkItem
{
public:
	explicit SchedWorkItemTest();
	virtual ~SchedWorkItemTest() {}

private:
	void Run() override;

};
