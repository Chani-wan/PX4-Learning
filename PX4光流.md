<h3 align="center"> PX4二次开发 </h3>

<div align="center">
    <h4 style="background-color: #FFFF00; display: inline-block; padding: 5px;">PX4光流传感器实现定点飞行</h4>
</div>

#### 1、QGC参数设置

​	可参考微空科技官网：https://micoair.cn/docs/MTF01-guang-liu-ce-ju-yi-ti-chuan-gan-qi-yong-hu-shou-ce

#### 2、通信链路

​	接下来阐述下光流传感器数据从传感器侧到QGC的完整通信链路：

​	在`MavlinkReceiver.cpp` 中，定义了函数 `handle_message_optical_flow_rad` ,首先他会接收来自外部光流传感器的MAVLink的消息，并解码存储到`sensor_optical_flow_s` 这样一个消息类型的话题`sensor_optical_flow` 中，最后发布出去。

```c++
void
MavlinkReceiver::handle_message_optical_flow_rad(mavlink_message_t *msg)
{
	mavlink_optical_flow_rad_t flow;
	mavlink_msg_optical_flow_rad_decode(msg, &flow);

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_MAVLINK;
	device_id.devid_s.bus = _mavlink.get_instance_id();
	device_id.devid_s.address = msg->sysid;
	device_id.devid_s.devtype = DRV_FLOW_DEVTYPE_MAVLINK;

	sensor_optical_flow_s sensor_optical_flow{};

	sensor_optical_flow.timestamp_sample = hrt_absolute_time();
	sensor_optical_flow.device_id = device_id.devid;

	sensor_optical_flow.pixel_flow[0] = flow.integrated_x;
	sensor_optical_flow.pixel_flow[1] = flow.integrated_y;

	sensor_optical_flow.integration_timespan_us = flow.integration_time_us;
	sensor_optical_flow.quality = flow.quality;

	const matrix::Vector3f integrated_gyro(flow.integrated_xgyro, flow.integrated_ygyro, flow.integrated_zgyro);

	if (integrated_gyro.isAllFinite()) {
		integrated_gyro.copyTo(sensor_optical_flow.delta_angle);
		sensor_optical_flow.delta_angle_available = true;
	}

	sensor_optical_flow.max_flow_rate       = NAN;
	sensor_optical_flow.min_ground_distance = NAN;
	sensor_optical_flow.max_ground_distance = NAN;

	// Use distance value for distance sensor topic
	if (PX4_ISFINITE(flow.distance) && (flow.distance >= 0.f)) {
		// Positive value (including zero): distance known. Negative value: Unknown distance.
		sensor_optical_flow.distance_m = flow.distance;
		sensor_optical_flow.distance_available = true;
	}

	sensor_optical_flow.timestamp = hrt_absolute_time();

	_sensor_optical_flow_pub.publish(sensor_optical_flow);
}
```

​	那么接下去便是通过 MavlinkStream 将uROB的消息格式打包成 Mavlink 的格式转发出去，具体代码我们可以在 `mavlink/streams/OPTICAL_FLOW_RAD.hpp` 中看到：

```c++
bool send() override
	{
		vehicle_optical_flow_s flow;

		if (_vehicle_optical_flow_sub.update(&flow)) {
			mavlink_optical_flow_rad_t msg{};

			msg.time_usec = flow.timestamp_sample;
			msg.sensor_id = _vehicle_optical_flow_sub.get_instance();
			msg.integration_time_us = flow.integration_timespan_us;
			msg.integrated_x = flow.pixel_flow[0];
			msg.integrated_y = flow.pixel_flow[1];
			msg.integrated_xgyro = flow.delta_angle[0];
			msg.integrated_ygyro = flow.delta_angle[1];
			msg.integrated_zgyro = flow.delta_angle[2];
			// msg.temperature = 0;
			msg.quality = flow.quality;
			// msg.time_delta_distance_us = 0;

			if (PX4_ISFINITE(flow.distance_m)) {
				msg.distance = flow.distance_m;

			} else {
				msg.distance = -1;
			}

			mavlink_msg_optical_flow_rad_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
```

​	那么此时我们会发现拿到的数据和我们转发的数据及类型并不一样，因为在PX4里`VehicleOpticalFlow`模块会对原始的光流数据进行处理并融合陀螺仪的数据，最后将其发布出去，具体的代码在`modules/sensors/vehicle_optical_flow`下：

```c++
if (publish) {
			vehicle_optical_flow_s vehicle_optical_flow{};

			vehicle_optical_flow.timestamp_sample = sensor_optical_flow.timestamp_sample;
			vehicle_optical_flow.device_id = sensor_optical_flow.device_id;

			_flow_integral *= _param_sens_flow_scale.get();
			_flow_integral.copyTo(vehicle_optical_flow.pixel_flow);
			_delta_angle.copyTo(vehicle_optical_flow.delta_angle);

			vehicle_optical_flow.integration_timespan_us = _integration_timespan_us;

			vehicle_optical_flow.quality = _quality_sum / _accumulated_count;

			if (_distance_sum_count > 0 && PX4_ISFINITE(_distance_sum)) {
				vehicle_optical_flow.distance_m = _distance_sum / _distance_sum_count;

			} else {
				vehicle_optical_flow.distance_m = NAN;
			}
```

我们可以看到话题`vehicle_optical_flow`订阅了`sensor_optical_flow` 里的消息，并做了相关融合处理，最后发布出去，也就是我们在QGC上看到的`OPTICAL_FLOW_RAD` 。

 