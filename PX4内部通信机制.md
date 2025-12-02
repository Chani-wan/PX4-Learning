<h3 align="center"> PX4二次开发-通信机制 </h3>

<div align="center">
    <h4 style="background-color: #FFFF00; display: inline-block; padding: 5px;">PX4内部通信机制-uORB</h4>
</div>



#### **一、消息定义与发布**

​	Msg: 定义消息（这一步和ROS）很像

​	CMake：将自定义消息加入编译列表

​	make：编译生成消息代码

​	C++：书写发布器发布消息

​	make：编译

​	QGC：固件烧录，运行

**（1）定义消息**

​	在源码`msg`路径下，创建一个名为`PersonExample.msg`的文件，然后我们可以像在ROS里一样，进行消息格式的定义，我们将消息格式定义如下：

``````c++
uint64 timestamp

char[128] name
char[11] id
uint8 gender
uint32 age
float32 height

uint8 MALE=0
uint8 FEMALE=1
``````

​	在这里，我们定义了一个`PersonExample`消息格式，每一个Person定义了姓名、ID、性别、身高、年龄等主要信息，接下来我们需要书写编译规则，并对消息代码进行编译，这一步像ROS里面的自定义消息一样，将生成消息相关的C++代码。

**（2）编译消息**

​	首先打开`msg/CMakeLists.txt`，然后将我们自定义的消息类型添加进来，然后在终端键入下面的命令进行固件编译：`make hkust_nxt-dual`，编译完成之后，我们可以在`build/hkust_nxt-dual_default/uORB/topics`目录下生成一个名为`person_example.h`的头文件，这是通过工具自动生成的头文件，也是我们后续代码书写过程中会使用的代码。

**（3）消息的订阅与发布**

​	经过上面的流程，我们已经自定义了一个消息，接下来为了使用这个自定义消息，书写一个APP来进行消息发布。在`src/examples`文件夹下创建新的APP，并在C++文件中键入下面的代码进行消息发布：

```c++
#include <stdlib.h>
#include <string.h>

#include <px4_platform_common/log.h>
#include <uORB/topics/person_example.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

extern "C" __EXPORT int uorb_person_main(int argc, char* argv[]) {
    uORB::Publication<person_example_s> person_pub{ORB_ID(person_example)};

    uORB::Subscription person_sub{ORB_ID(person_example)};

    person_pub.advertise();

    person_example_s person; memset(&person, 0, sizeof(person));

    strcpy(&person.id[0], "1234567890");
    strcpy(&person.name[0], "Bob");
    person.gender = person_example_s::FEMALE;
    person.age = 17;
    person.height = 1.78;

    person_pub.publish(person);

    person_example_s person_recv;

    if (person_sub.updated()) {
        person_sub.copy(&person_recv);

        printf("A person message received. Name: %s, Age: %ld, Height: %f\n", person_recv.name, person_recv.age, (double)person_recv.height);
    }

    return 0;
}

```

​	这里解释一下上面的几行代码。首先`#include <uORB/topics/person_example.h>`包含了我们刚才通过编译自动生成的消息定义代码。下面的两句分别包含了`uORB/Publication.hpp`和`uORB/Subscription.hpp`，这两个头文件分别声明了用于消息发布的`Publication`和用于消息订阅的`Subscription`。接下来看源码部分，首先第一句：

```cpp
uORB::Publication<person_example_s> person_pub{ORB_ID(person_example)};
```

声明了一个话题发布器，模板参数声明了话题发布器即将发布的消息类型是`person_example_s`，这个类型在我们生成的消息文件`person_example.h`中声明定义。接下来，`Publication`对象构造的参数是话题名称，在这里是一个**固定写法**`ORB_ID(person_example)`，下面的这一句：

```
uORB::Subscription person_sub{ORB_ID(person_example)};
```

用同样的方法定义了一个话题订阅器，这个对象主要用于接收对应话题下的消息。接下来的代码直到第25行，我们构造了一个`person_example_s`类型的消息，并通过`Publication`发布出去。

​	从29行开始，我们通过订阅器查询话题中的数据并打印,这里面涉及到`Subscription`的几个重要接口，`updated`接口是一个主动查询接口，它的主要作用是检查对应话题中的消息是否有新的消息进入。如果有新的消息进入，接下来的另外一个重要接口是`copy`，它的作用是将最新的消息内容拷贝到指定的内存中，接下来的一个打印语句检查消息内容的一致性。我们将固件烧录到飞控板上，并查看运行结果。



#### **二、ROS与uORB通信机制对比**

​	下面的代码是ROS中消息发布和订阅机制的一般写法：

```c++
ros::Publication odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 20);

...

odom_pub.publish(odom);

...

void OdomCallback(nav_msgs::OdometryConstPtr odom) {
  ...
}

...
  
ros::Subscription odom_sub = nh.subscribe("/odom", 20, OdomCallback);
```

​	在ROS的消息通信机制中，发布者通过调用`NodeHandle`的`advertise`接口来注册一个话题，`advertise`是一个模板函数，模板参数是待发布的消息类型，该接口有两个参数，其中第一个参数是一个用`string`类型表示的**话题**。而订阅者通过调用`subscribe`接口来订阅话题中的消息，该接口有三个参数，其中第一个是一个用`string`类型表示的**话题**，而第三个是一个回调函数，每一次对应的话题中有消息到来的时候，都会调用这个`OdomCallback`函数来进行数据处理。

**==异同==：**

（1）在接收端，uORB通过订阅者**主动轮询**的方式来主动查询是否有新消息产生，而ROS通过注册回调的方式被动调用。这个区别使得PX4中充斥着类似于下面这样的代码：

```c++
person_sub.update(&received_data)
```

（2）另外一个关键的区别在于话题的灵活性。在ROS中，即使是同一个类型的消息，我们也可以自由地通过不同的话题名称来进行发布和订阅。

​	而在uORB通信中，似乎并没有显式区分类型和话题，从上面的代码中我们可以看到，在uORB中，无论是发布者还是订阅者，其所谓的话题参数都是不能像在ROS里一样可以随便给的，而只能是一个约定俗成的值：**用下划线分割的消息类型名称**。`PersonExample-->person_example`。



#### **三、uORB回调形式的消息通信机制**

​	从上面的例子我们可以看到，与ROS的msg通信不同，uORB通信机制采用主动轮询的方式来不断读取消息，而不是被动回调。而px4的确提供了类似于回调的使用方式，这是通过`uORB::SubscriptionCallbackWorkItem`类来实现的，例程如下:

==hpp:==

```c++
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
		_person_sub.registerCallback(); //告诉uORB系统，当有新消息时，触发回调，自动将对应的工作项加入到工作队列中。
	}

private:
	uORB::SubscriptionCallbackWorkItem _person_sub{this,ORB_ID(person_example)};
    //这里将订阅与当前对象关联，当消息到达时会调用Run()方法。
	void Run() override{
		person_example_s msg;
		if(_person_sub.update(&msg)){
			PX4_INFO("Message received!");
			orb_print_message_internal(ORB_ID(person_example), &msg, true);
		}
	}

};
```

==cpp:==	

```c++
#include <stdlib.h>
#include <string.h>
#include <px4_platform_common/log.h>
#include <uORB/topics/person_example.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include "uorb_person.hpp"

static px4::atomic<PersonReceiver*> person_sub;
extern "C" __EXPORT int uorb_person_main(int argc, char *argv[])
{
	PersonReceiver* instance = new PersonReceiver;
	instance->Init();
	person_sub.store(instance);

	uORB::PublicationMulti<person_example_s> person_pub(ORB_ID(person_example));
	for (int n = 0; n < 10; ++n) {
		person_example_s msg;
		const char *id = "12345678";
		const char *name = "Tommy";

		memset(&msg, 0, sizeof(msg));
		memcpy(msg.name, name, strlen(name));
		memcpy(msg.id, id, strlen(id));
		msg.age = n + 10;
		msg.height = 1.6 + n * 0.2;

		person_pub.publish(msg);
		px4_usleep(1000ULL);
	      }
	      return PX4_OK;
}
```

​	这里解释下相应逻辑：当发布者调用 `person_pub.publish(msg)` 时，uORB系统检测到新消息，工作队列调度器将 `PersonReceiver::Run()` 加入执行队列，在工作队列线程中异步执行 `Run()` 方法，在 `Run()` 方法中的 `person_sub.update(&msg)` 调用，这个调用会检查是否有新消息，如果有则复制到局部变量`msg`中。

​	编译运行后，我们看到，程序中定义的订阅者已经成功接收到了来自`person_pub`发布的消息并执行了期望的回调函数。



==譬如：==

在PX4的电池状态读取`battery_status.cpp` 里面，也是通过这样的回调形式获取消息。如下代码：

```c++
private:
	void Run() override;

	uORB::SubscriptionInterval	_parameter_update_sub{ORB_ID(parameter_update), 1_s};				
	uORB::SubscriptionCallbackWorkItem _adc_report_sub{this, ORB_ID(adc_report)};
```

同样使用 `SubscriptionCallbackWorkItem` 类，当新数据到达时会自动触发回调。

```c++
bool
BatteryStatus::init()
{
	return _adc_report_sub.registerCallback();
}

void
BatteryStatus::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* check parameters for updates */
	parameter_update_poll();

	/* check battery voltage */
	adc_poll();

	perf_end(_loop_perf);
}
```

接着我们查看 `Run()` 方法，进入 `adc_poll()` 里：

```c++
if (_adc_report_sub.update(&adc_report)) {

		/* Read add channels we got */
		for (unsigned i = 0; i < PX4_MAX_ADC_CHANNELS; ++i) {
			for (int b = 0; b < BOARD_NUMBER_BRICKS; b++) {

				/* Once we have subscriptions, Do this once for the lowest (highest priority
				 * supply on power controller) that is valid.
				 */
				if (selected_source < 0 && _analogBatteries[b]->is_valid()) {
					/* Indicate the lowest brick (highest priority supply on power controller)
					 * that is valid as the one that is the selected source for the
					 * VDD_5V_IN
					 */
					selected_source = b;
				}

				/* look for specific channels and process the raw voltage to measurement data */

				if (adc_report.channel_id[i] >= 0) {
					if (adc_report.channel_id[i] == _analogBatteries[b]->get_voltage_channel()) {
						/* Voltage in volts */
						bat_voltage_adc_readings[b] = adc_report.raw_data[i] *
									      adc_report.v_ref /
									      adc_report.resolution;
						has_bat_voltage_adc_channel[b] = true;

					} else if (adc_report.channel_id[i] == _analogBatteries[b]->get_current_channel()) {
						bat_current_adc_readings[b] = adc_report.raw_data[i] *
									      adc_report.v_ref /
									      adc_report.resolution;
					}
				}

			}
		}

```

​	首先这段代码检查是否有新的ADC数据，如果有，便从各ADC通道读取新的电压电流数据，最后进行电池的状态计算。



#### 四、多发布实例和独立订阅	

​	在`Subscription`的构造函数，还有一个`instance`参数，这个instance参数是发布实例对应的ID，在PX4中，每一个发布实例都有一个ID，我们在订阅的时候可以指定一个ID来接收特定发布实例发布出来的话题。我们再写一个`uorb_person_multi`的APP，并在APP源码中键入下面的代码：

```c++
#include <uORB/topics/person_example.h>

#include <px4_platform_common/log.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/PublicationMulti.hpp>

extern "C" __EXPORT int uorb_person_multi_main(int argc, char* argv[]) {
    uORB::PublicationMulti<person_example_s> person_pub0{ORB_ID(person_example)};

    uORB::PublicationMulti<person_example_s> person_pub1{ORB_ID(person_example)};

    uORB::Subscription person_sub0{ORB_ID(person_example), (uint8_t) person_pub0.get_instance()};

    uORB::Subscription person_sub1{ORB_ID(person_example), (uint8_t) person_pub1.get_instance()};

    person_example_s person0, person1;
    memset(&person0, 0, sizeof(person0));
    memset(&person1, 0, sizeof(person1));

    strcpy(person0.id, "1234567890");
    strcpy(person1.id, "9876543210");
    strcpy(person0.name, "Bob");
    strcpy(person1.name, "Amy");
    person0.gender = person_example_s::FEMALE;
    person1.gender = person_example_s::MALE;
    person0.age = 18;
    person1.age = 16;
    person0.height = 1.77;
    person1.height = 1.64;

    person_pub0.publish(person0);
    person_pub1.publish(person1);


    if (person_sub0.updated()) {
	person_example_s person_recv0;
	person_sub0.copy(&person_recv0);

	printf("Instance 0 message received. Name: %s, Age: %ld, Height: %lf.\n", person_recv0.name, person_recv0.age, (double) person_recv0.height);
    }

    if (person_sub1.updated()) {
	person_example_s person_recv1;
	person_sub1.copy(&person_recv1);

	printf("Instance 1 message received. Name: %s, Age: %ld, Height: %lf.\n", person_recv1.name, person_recv1.age, (double) person_recv1.height);
    }

    return 0;
}

```

​	编译烧录并执行后，可以看到终端打印的消息。在PX4的源码中，多实例EKF2就是通过这个多实例发布订阅实现的。



==其他多发布订阅实例：==

在`ManualControl.hpp` 和`ManualControl.cpp` 中，定义了多个`manual_control_input` 的实例订阅对象：

```c++
uORB::SubscriptionCallbackWorkItem _manual_control_input_subs[MAX_MANUAL_INPUT_COUNT] {
		{this, ORB_ID(manual_control_input), 0},
		{this, ORB_ID(manual_control_input), 1},
		{this, ORB_ID(manual_control_input), 2},
```

```c++
for (int i = 0; i < MAX_MANUAL_INPUT_COUNT; i++) {
		manual_control_setpoint_s manual_control_input;

		if (_manual_control_input_subs[i].update(&manual_control_input)) {
			_selector.updateWithNewInputSample(now, manual_control_input, i);
		}
	}
```

在`ManualControlSelector.cpp`中，我们可以看到它定义了三种输入源：1、遥控器输入   2、mavlink输入  3、其他输入源

```c++
bool ManualControlSelector::isInputValid(const manual_control_setpoint_s &input, uint64_t now) const
{
	// Check for timeout
	const bool sample_from_the_past = now >= input.timestamp_sample;
	const bool sample_newer_than_timeout = now - input.timestamp_sample < _timeout;

	// Check if source matches the configuration
	const bool source_rc_matched = (_rc_in_mode == 0) && (input.data_source == manual_control_setpoint_s::SOURCE_RC);
	const bool source_mavlink_matched = (_rc_in_mode == 1) &&
					    (input.data_source == manual_control_setpoint_s::SOURCE_MAVLINK_0
					     || input.data_source == manual_control_setpoint_s::SOURCE_MAVLINK_1
					     || input.data_source == manual_control_setpoint_s::SOURCE_MAVLINK_2
					     || input.data_source == manual_control_setpoint_s::SOURCE_MAVLINK_3
					     || input.data_source == manual_control_setpoint_s::SOURCE_MAVLINK_4
					     || input.data_source == manual_control_setpoint_s::SOURCE_MAVLINK_5);
	const bool source_any_matched = (_rc_in_mode == 2);
	const bool source_first_matched = (_rc_in_mode == 3) &&
					  (input.data_source == _first_valid_source
					   || _first_valid_source == manual_control_setpoint_s::SOURCE_UNKNOWN);

	return sample_from_the_past && sample_newer_than_timeout && input.valid
	       && (source_rc_matched || source_mavlink_matched || source_any_matched || source_first_matched);
}
```



#### 五、同类型多话题发布

​	在uORB中，同样支持同类型多话题发布机制，不过论灵活性远不比ROS中那样可以随心所欲，这首先需要修改`.msg`文件，在最后一行用一个`# TOPICS`开头的行来指定此类型可以使用的话题名称：

```c++
# TOPICS person_example person_example_debug
```

​	然后在代码中就可以用这个话题来进行发布和订阅，我们修改上面的多实例发布示例：

```c++
#include <px4_platform_common/log.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/person_example.h>

extern "C" __EXPORT int uorb_person_multi_main(int argc, char *argv[])
{
	 uORB::PublicationMulti<person_example_s> person_pub0(ORB_ID(person_example));
	 uORB::PublicationMulti<person_example_s> person_pub1(ORB_ID(person_example));
	 uORB::PublicationMulti<person_example_s> person_pub2(ORB_ID(person_example_debug));

	 uORB::Subscription person_sub0{ORB_ID(person_example),(uint8_t) person_pub0.get_instance()};
	 uORB::Subscription person_sub1{ORB_ID(person_example),(uint8_t) person_pub1.get_instance()};
	 uORB::Subscription person_sub2{ORB_ID(person_example_debug)};

	 person_example_s person0, person1,person2;
	 memset(&person0, 0, sizeof(person0));
	 memset(&person1, 0, sizeof(person1));
	 memset(&person2, 0, sizeof(person2));

	 strcpy(person0.id, "1234567890");
	 strcpy(person1.id, "9876543210");
	 strcpy(person2.id, "1122334455");
	 strcpy(person0.name, "Bob");
	 strcpy(person1.name, "Amy");
	 strcpy(person2.name, "DebugPerson");
	 person0.gender = person_example_s::FEMALE;
	 person1.gender = person_example_s::MALE;
	 person2.gender = person_example_s::FEMALE;
	 person0.age = 18;
	 person1.age = 16;
	 person2.age = 30;
	 person0.height = 1.77;
	 person1.height = 1.64;
	 person2.height = 1.70;

	 person_pub0.publish(person0);
	 person_pub1.publish(person1);
	 person_pub2.publish(person2);

	 if (person_sub0.updated()) {
		person_example_s person_recv0;
		person_sub0.copy(&person_recv0);
		printf("Instance 0 message received. Name: %s, Age: %ld, Height: %lf.\n", person_recv0.name, person_recv0.age, (double) person_recv0.height);
	    }

	    if (person_sub1.updated()) {
		person_example_s person_recv1;
		person_sub1.copy(&person_recv1);
		printf("Instance 1 message received. Name: %s, Age: %ld, Height: %lf.\n", person_recv1.name, person_recv1.age, (double) person_recv1.height);
	    }

	    if(person_sub2.updated()){
		person_example_s person_recv2;
		person_sub2.copy(&person_recv2);
		printf("Debug message received. Name: %s, Age: %ld, Height: %lf.\n", person_recv2.name, person_recv2.age, (double) person_recv2.height);
	    }
	    return 0;
}

```

​	在PX4中，对遥控器输入的处理就利用了这种同类型多话题发布的机制。



其他同类型多话题发布：

在GPS传感器中也对同类型的话题进行了多话题发布

```c++
uint8 satellites_used		# Number of satellites used

float32 heading			
float32 heading_offset		
float32 heading_accuracy	# heading accuracy (rad, [0, 2PI])
float32 rtcm_injection_rate	# RTCM message injection rate Hz
uint8 selected_rtcm_instance	
bool rtcm_crc_ailed		# RTCM message CRC failure detected

uint8 RTCM_MSG_USED_UNKNOWN = 0
uint8 RTCM_MSG_USED_NOT_USED = 1
uint8 RTCM_MSG_USED_USED = 2
uint8 rtcm_msg_used		
    
# TOPICS sensor_gps vehicle_gps_position
```

