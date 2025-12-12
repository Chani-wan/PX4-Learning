#pragma once
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trif_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/trif_thrust_estimate.h>
//#include <uORB/topics/sensor_accel.h>
#include <uORB/SubscriptionCallback.hpp>
#include <matrix/math.hpp>

class TrifPosCtrl : public ModuleBase<TrifPosCtrl>, public ModuleParams , public px4::ScheduledWorkItem
{
public:
	TrifPosCtrl();
	~TrifPosCtrl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:

	void Run() override;

	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<trif_status_s> _trif_status_pub{ORB_ID(trif_status)};
	uORB::Publication<trif_thrust_estimate_s> _trif_thrust_estimate_pub{ORB_ID(trif_thrust_estimate)};

	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::SubscriptionCallbackWorkItem _trif_trajectory_sub{this,ORB_ID(trif_trajectory_setpoint)};
	uORB::Subscription _vehicle_thrust_setpoint_sub{ORB_ID(vehicle_thrust_setpoint)};
	//uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)};


	vehicle_local_position_s _vehicle_local_position;
	vehicle_local_position_s _switched_pos;
	trajectory_setpoint_s _trif_trajectory;
	trif_status_s _trif_status;
	vehicle_thrust_setpoint_s _vehicle_thrust_setpoint{};
	//sensor_accel_s _sensor_accel;

	uint64_t _last_time{0};

	using Vector3f = matrix::Vector3f;
	Vector3f _pos_kp{};
	Vector3f _vel_kp{};
	Vector3f _vel_ki{};
	Vector3f _des_pos{};
	Vector3f _des_vel{};
	Vector3f _des_acc{};
	Vector3f _pos_error{};
	Vector3f _vel_error{};
	Vector3f _vel_err_int{};
	Vector3f _acc_command{};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::TRIF_POS_KP_XY>) trif_pos_kp_xy,
		(ParamFloat<px4::params::TRIF_POS_KP_Z>) trif_pos_kp_z,
		(ParamFloat<px4::params::TRIF_VEL_KP_XY>) trif_vel_kp_xy,
		(ParamFloat<px4::params::TRIF_VEL_KP_Z>) trif_vel_kp_z,
		(ParamFloat<px4::params::TRIF_VEL_KI_XY>) trif_vel_ki_xy,
		(ParamFloat<px4::params::TRIF_VEL_KI_Z>) trif_vel_ki_z
	)

	float _hover_thr{0.2f};     // omega: 估计出的悬停油门
    	float _P{100.0f};       // P: 协方差 (初始不确定性)
    	float _gamma{0.998f};    // gamma: 遗忘因子

	void getParameters();
	void setZeroIfNanVec3(Vector3f &vec);
	void ComputeAccCommand();
	void ExecuteAccCommand();
	void HoverThrottleEstimate();
};
