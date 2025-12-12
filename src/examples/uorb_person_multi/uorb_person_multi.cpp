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
