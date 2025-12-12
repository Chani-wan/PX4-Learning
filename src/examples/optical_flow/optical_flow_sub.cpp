#include <stdlib.h>
#include <string.h>
#include <px4_platform_common/log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_optical_flow.h>
#include <stdio.h>

extern "C" __EXPORT int optical_flow_sub_main(int argc, char *argv[])
{
    uORB::Subscription optical_flow_sub{ORB_ID(vehicle_optical_flow)};

    uint64_t last_print_time = 0;

    while (true) {
        if (optical_flow_sub.updated()) {
            vehicle_optical_flow_s msg;

            if (optical_flow_sub.copy(&msg)) {
                uint64_t now = hrt_absolute_time();
                if (now - last_print_time > 500000) { // 500ms

			PX4_INFO("=== 光流传感器数据 ===");
			PX4_INFO("时间戳: %" PRIu64, msg.timestamp);
			PX4_INFO("采样时间: %" PRIu64, msg.timestamp_sample);
			PX4_INFO("设备ID: %" PRIu32, msg.device_id);

			PX4_INFO("像素流 [X, Y]: [%.3f, %.3f] pixels",
				 (double)msg.pixel_flow[0],
				 (double)msg.pixel_flow[1]);

			PX4_INFO("角度增量 [X, Y, Z]: [%.3f, %.3f, %.3f] rad",
				 (double)msg.delta_angle[0],
				 (double)msg.delta_angle[1],
				 (double)msg.delta_angle[2]);

			PX4_INFO("测量距离: %.3f m", (double)msg.distance_m);

			PX4_INFO("最大流率: %.3f pixels/s", (double)msg.max_flow_rate);
			PX4_INFO("地面距离范围: [%.3f, %.3f] m",
				 (double)msg.min_ground_distance,
				 (double)msg.max_ground_distance);

			PX4_INFO("数据质量: %u/255", msg.quality);

			PX4_INFO("========================\n");

                    last_print_time = now;
                }
            }
        }

        usleep(50000);
    }

    return 0;
}
