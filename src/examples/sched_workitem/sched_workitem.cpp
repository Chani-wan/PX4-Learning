#include "sched_workitem.hpp"
static px4::atomic<SchedWorkItemTest *> instance;

SchedWorkItemTest::SchedWorkItemTest():
ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {}

void SchedWorkItemTest::Run() {
	printf("The test work item run at: %lld\n", hrt_absolute_time());
      }

extern "C"  __EXPORT int sched_workitem_main(int argc, char* argv[])
{
	instance.store(new SchedWorkItemTest);
	//instance.load()->ScheduleOnInterval(10 * 1000ULL); // 每隔10毫秒启动一次
	instance.load()->ScheduleDelayed(1 * 100 * 1000ULL); // 0.5秒后启动一次
	printf("Process exiting...\n");

	return PX4_OK;
}
