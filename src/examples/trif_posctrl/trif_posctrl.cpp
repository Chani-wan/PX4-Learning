#include "trif_posctrl.hpp"
#include <math.h>
#include <px4_platform_common/defines.h>

TrifPosCtrl::TrifPosCtrl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{

}

TrifPosCtrl::~TrifPosCtrl()
{

}

bool TrifPosCtrl::init()
{
	// execute Run() on every sensor_accel publication
	// if (!_trif_trajectory_sub.registerCallback())
	// {
	// 	PX4_ERR("callback registration failed");
	// 	return false;
	// }

	// alternatively, Run on fixed interval
	///ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate
	ScheduleOnInterval(10000);
	return true;
}

void TrifPosCtrl::getParameters()
{
	_pos_kp(0) = _pos_kp(1) = trif_pos_kp_xy.get();
	_pos_kp(2) = trif_pos_kp_z.get();

	_vel_kp(0) = _vel_kp(1) = trif_vel_kp_xy.get();
	_vel_kp(2) = trif_vel_kp_z.get();

	_vel_ki(0) = _vel_ki(1) = trif_vel_ki_xy.get();
	_vel_ki(2) = trif_vel_ki_z.get();

	printf("PosCtrl parameter updated:\n"
		"\tPOS_KP: [%f, %f, %f]\n"
		"\tVEL_KP: [%f, %f, %f]\n"
		"\tVEL_KI: [%f, %f, %f]\n" ,
		(double) _pos_kp(0), (double) _pos_kp(1), (double) _pos_kp(2),
		(double) _vel_kp(0), (double) _vel_kp(1), (double) _vel_kp(2),
		(double) _vel_ki(0), (double) _vel_ki(1), (double) _vel_ki(2)
	    );

}

void TrifPosCtrl::setZeroIfNanVec3(Vector3f &vec)
{
	for (int i = 0; i < 3; i++)
	{
		if (!PX4_ISFINITE(vec(i)))
		{
			vec(i) = 0.0f;
		}
	}
}


void TrifPosCtrl::ComputeAccCommand()
{
	uint64_t now = hrt_absolute_time();
	float deltat = (_last_time > 0) ? ((now - _last_time) * 1e-6f) : 0.02f;
	_last_time = now;
    	if (deltat > 0.1f) deltat = 0.02f;

	_des_pos = Vector3f(_trif_trajectory.position);
	_des_vel = Vector3f(_trif_trajectory.velocity);
	_des_acc = Vector3f(_trif_trajectory.acceleration);
	_pos_error = _des_pos - Vector3f(_vehicle_local_position.x, _vehicle_local_position.y, _vehicle_local_position.z);
	_vel_error = _des_vel - Vector3f(_vehicle_local_position.vx, _vehicle_local_position.vy, _vehicle_local_position.vz);

	setZeroIfNanVec3(_pos_error);
	setZeroIfNanVec3(_vel_error);
	setZeroIfNanVec3(_des_acc);

	_acc_command = _des_acc + _pos_error.emult(_pos_kp) + _vel_error.emult(_vel_kp) + _vel_err_int.emult(_vel_ki);

	setZeroIfNanVec3(_acc_command);
	_vel_err_int = _vel_err_int + _vel_error * deltat ;
	setZeroIfNanVec3(_vel_err_int);

	_trif_status.timestamp = hrt_absolute_time();
	_pos_error.copyTo(_trif_status.pos_err);
	_vel_error.copyTo(_trif_status.vel_err);
	_vel_err_int.copyTo(_trif_status.vel_err_int);
	_trif_status.dt = deltat;
	_trif_status_pub.publish(_trif_status);
}

void TrifPosCtrl::ExecuteAccCommand()
{
	trajectory_setpoint_s trajectory;
	trajectory.timestamp = hrt_absolute_time();

	matrix::Vector3f nan_vec(NAN, NAN, NAN);
	nan_vec.copyTo(trajectory.position);
	nan_vec.copyTo(trajectory.velocity);
	nan_vec.copyTo(trajectory.jerk);
	trajectory.yaw = 0;
	trajectory.yawspeed = 0;

	_acc_command.copyTo(trajectory.acceleration);
	_trajectory_setpoint_pub.publish(trajectory);
}

void TrifPosCtrl::HoverThrottleEstimate()
{
	float thrust_z = _vehicle_thrust_setpoint.xyz[2];
    	float y = -thrust_z;
	if (y < 0.1f) return;
	float g = 9.81f;
    	//float a_z = _sensor_accel.z;
	float a_z = _vehicle_local_position.az;
    	float x = (g - a_z) / g;

	float gamma_sq = _gamma * _gamma;
	float P_x = _P * x;
	float K = P_x / (x * P_x + gamma_sq);  //K计算
	_P = (1.0f - K * x) * _P / gamma_sq;   //P更新

	float y_e = y - _hover_thr * x;
    	_hover_thr = _hover_thr + K * y_e;  //悬停油门更新

	if (_hover_thr > 0.9f) _hover_thr = 0.9f;
	else if (_hover_thr < 0.1f) _hover_thr = 0.1f;

	trif_thrust_estimate_s trif_thrust_estimate;
	trif_thrust_estimate.timestamp = hrt_absolute_time();
	trif_thrust_estimate.hover_thrust = _hover_thr;
	_trif_thrust_estimate_pub.publish(trif_thrust_estimate);

}

void TrifPosCtrl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if(_vehicle_local_position_sub.updated())
	{
		_vehicle_local_position_sub.copy(&_vehicle_local_position);
	}

	// if(_sensor_accel_sub.updated())
	// {
	// 	_sensor_accel_sub.copy(&_sensor_accel);
	// }

	if (_vehicle_thrust_setpoint_sub.updated())
	{
		_vehicle_thrust_setpoint_sub.copy(&_vehicle_thrust_setpoint);
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
		getParameters();
	}

	if(_trif_trajectory_sub.updated())
	{
		_trif_trajectory_sub.copy(&_trif_trajectory);
		ComputeAccCommand();
		ExecuteAccCommand();
	}
	HoverThrottleEstimate();
}



int TrifPosCtrl::task_spawn(int argc, char *argv[])
{
	TrifPosCtrl *instance = new TrifPosCtrl();

	if (instance)
	{
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init())
		{
			return PX4_OK;
		}

	}
	else
	{
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int TrifPosCtrl::print_status()
{
	return 0;

}

int TrifPosCtrl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int TrifPosCtrl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int trif_posctrl_main(int argc, char *argv[])
{
	return TrifPosCtrl::main(argc, argv);
}
