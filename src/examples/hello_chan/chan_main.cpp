#include <stdio.h>
#include <px4_platform_common/log.h>

extern "C" __EXPORT int chan_main(
	int argc, char* argv[]
) {
    printf("Hello PX4!\n");

    return 0;
}
