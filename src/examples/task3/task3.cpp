#include "remote_control.hpp"

static px4::atomic<RemoteControl*> rc;
extern "C" __EXPORT int task3_main(int argc, char *argv[])
{
	RemoteControl* instance = new RemoteControl;
	rc.store(instance);
	instance->Init();
	return 0;

}
