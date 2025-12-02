#include "remote_control_sim.hpp"

static px4::atomic<RemoteControlSim*> rc;
extern "C" __EXPORT int task3_sim_main(int argc, char *argv[])
{
	RemoteControlSim* instance = new RemoteControlSim;
	rc.store(instance);
	instance->Init();
	return 0;
}
