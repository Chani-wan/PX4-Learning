#include <stdlib.h>
#include <string.h>
#include <px4_platform_common/log.h>
#include <uORB/topics/person_example.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include "uorb_person.hpp"


// extern "C" __EXPORT int uorb_person_main(int argc, char *argv[])
// {
// 	uORB::Publication<person_example_s> person_pub{ORB_ID(person_example)};
// 	uORB::Subscription person_sub{ORB_ID(person_example)};

// 	person_pub.advertise();
// 	person_example_s person_data{};
// 	strcpy(person_data.name, "Peter");
// 	strcpy(person_data.id, "123");
// 	person_data.age = 20;
// 	person_data.height = 180.5f;
// 	person_data.gender = person_example_s::MALE;

// 	person_pub.publish(person_data);

// 	person_example_s received_data{};
// 	if(person_sub.update(&received_data)){
// 		person_sub.copy(&received_data);
// 		printf("Received Person Data: name: %s, id: %s, age: %ld, height: %f, gender: %d\n",received_data.name,
// 			received_data.id,received_data.age,(double)received_data.height,received_data.gender);
// 	}
// 	return 0;

// }
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
