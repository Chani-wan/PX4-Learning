<h3 align="center"> PX4二次开发 </h3>

<div align="center">
    <h4 style="background-color: #FFFF00; display: inline-block; padding: 5px;">PX4任务调度机制</h4>
</div>
#### 1、ScheduleWorkItem

​	在`px4`中，`ScheduledWorkItem`是实现异步任务调度的常用工具，它表示一个**工作任务**，其接口定义如下:

```c++
#pragma once
#include "WorkItem.hpp"
#include <drivers/drv_hrt.h>

namespace px4
{
class ScheduledWorkItem : public WorkItem
{
public:
	bool Scheduled() { return !hrt_called(&_call); }
	/**
	 * Schedule next run with a delay in microseconds.
	 *
	 * @param delay_us		The delay in microseconds.
	 */
	void ScheduleDelayed(uint32_t delay_us);
	/**
	 * Schedule repeating run with optional delay.
	 *
	 * @param interval_us		The interval in microseconds.
	 * @param delay_us		The delay (optional) in microseconds.
	 */
	void ScheduleOnInterval(uint32_t interval_us, uint32_t delay_us = 0);
	/**
	 * Schedule next run at a specific time.
	 *
	 * @param time_us		The time in microseconds.
	 */
	void ScheduleAt(hrt_abstime time_us);
	/**
	 * Clear any scheduled work.
	 */
	void ScheduleClear();

protected:

	ScheduledWorkItem(const char *name, const wq_config_t &config) : WorkItem(name, config) {}
	virtual ~ScheduledWorkItem() override;
	virtual void print_run_status() override;

private:

	virtual void Run() override = 0;
	static void	schedule_trampoline(void *arg);
	hrt_call	_call{};
};
} // namespace px4

```

​	我们来一一解释上面的这些接口的主要功能。

- `ScheduledWorkItem(const char* name, const wq_config_t &config)`:构造函数，它接收两个参数，意义分别如下: `name`：是工作任务的名称，`config`：px4的每一个工作任务托管在一个工作队列(`WorkQueue`)里进行统一管理，这个`config`参数指示了该工作任务将在哪一个工作队列中被调度。在文件`platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp`中定义了很多配置，在实际书写过程中可进行选择。

- `Schedule`: 继承自`WorkItem`，将当前工作任务添加到工作队列等待执行。
- `ScheduleDelayed`延迟一定时间(微秒为单位)之后将此工作任务添加到工作队列等待执行。
- `ScheduleOnInterval`每隔一定时间将此任务添加到工作队列等待执行，实现周期性调用。
- `ScheduleAt`在某个特定的时间将此任务添加到工作队列等待执行。
- `Run`此任务添加到工作队列中之后，会在工作队列中等待执行，在执行过程中会调用此接口，也是继承此基类的子类必须要实现的接口。

#### 2、**PX4任务调度实验**

​	创建`src/examples/sched_workitem`文件并在其中添加`sched_workitem.hpp`文件，然后写入下面的内容:

```c++
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
```

​	然后在`sched_workitem.cpp`文件中实现之:

```c++
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
```

​	与之前的CMakeLists不同，本例程需要依赖`px4_work_queue`，因此cmake的书写也会略有区别:

```c++
px4_add_module(
  MODULE examples__sched_workitem
  MAIN sched_workitem
  STACK_MAIN 2000
  SRCS
    sched_workitem.cpp
  DEPENDS
    px4_work_queue
)
```

​	最后编译运行，在`px4`终端键入`sched_wotkitem`后会打印相应内容，可以看到，我们定义的工作任务正在以大约10ms一次的频率被调用。另外几个调度接口的使用方式与此接口类似，我们这里不再展示。（在px4代码中看到任何数据结构只要继承了ScheduledWorkItem，就要关注其Run函数的实现，并且在代码中查找Schedule*函数分析其调用时机。）
