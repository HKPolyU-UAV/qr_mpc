#ifndef Airo_PX4_FSM_H
#define Airo_PX4_FSM_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/SetMode.h>
#include <qr_mpc/TakeoffLandTrigger.h>
#include <qr_mpc/AiroPx4.h>

#include "rc_input.h"
#include "quadrotor_mpc.h"


class AIRO_PX4_FSM{
    private:

    enum STATE_FSM{
		RC_MANUAL,
		AUTO_HOVER,
		AUTO_TAKEOFF,
		AUTO_LAND,
		POS_COMMAND
	};

	// Parameters
	const double MESSAGE_TIMEOUT = 0.5;
	const double MOTOR_SPEEDUP_TIME = 5.0;
	const double HOVER_THRUST = 0.56;
	const double TAKEOFF_HEIGHT = 1.0;
	const double TAKEOFF_LAND_SPEED = 0.3;
	const double HOVER_MAX_VELOCITY = 0.5;
	const double SAFETY_VOLUMN[6] = {-2.0, 2.0, -2.0, 2.0, -100, 4.0}; // min_x max_x min_y max_y min_z max_z

	// Variables
	STATE_FSM state_fsm;
	RC_INPUT rc_input;
	bool solve_controller;
	bool is_landed;
	bool is_armed;

	// Times
	ros::Time current_time;
	ros::Time takeoff_land_time;
	ros::Time last_hover_time;
	
	// ROS Sub & Pub
	ros::Subscriber pose_sub;
	ros::Subscriber twist_sub;
	ros::Subscriber state_sub;
	ros::Subscriber extended_state_sub;
	ros::Subscriber rc_input_sub;
	ros::Subscriber command_sub;
	ros::Subscriber takeoff_land_sub;
	ros::Publisher setpoint_pub;
	ros::Publisher airo_px4_pub;

	// ROS Services
	ros::ServiceClient setmode_srv;
	ros::ServiceClient arm_srv;
	ros::ServiceClient reboot_srv;

	// Messages
	qr_mpc::TakeoffLandTrigger takeoff_land_trigger; // 1 for takeoff 0 for landing
	qr_mpc::AiroPx4 airo_px4_state;
	geometry_msgs::PoseStamped local_pose;
	geometry_msgs::PoseStamped takeoff_land_pose;
	geometry_msgs::PoseStamped command_pose;
	geometry_msgs::TwistStamped local_twist;
	mavros_msgs::AttitudeTarget attitude_target;
	mavros_msgs::State current_state;
	mavros_msgs::State previous_state;
	mavros_msgs::ExtendedState current_extended_state;

	//Controller
	QUADROTOR_MPC controller;

	//Ref
	Eigen::VectorXd ref;

	public:

	AIRO_PX4_FSM(ros::NodeHandle& nh);
	void process();
	void fsm();
	void publish_control_commands(mavros_msgs::AttitudeTarget,ros::Time);
	bool toggle_offboard(bool);
	bool toggle_arm(bool);
	void get_motor_speedup();
	void get_takeoff_land_ref(const double);
	void set_ref(double, double, double);
	void set_ref_with_rc();
	void set_ref_with_command();
	void land_detector();
	void motor_idle_and_disarm();
	void takeoff_land_init();
	void auto_hover_init();
	Eigen::Vector3d check_safety_volumn(const Eigen::Vector3d);
	void pose_cb(const geometry_msgs::PoseStamped::ConstPtr&);
	void twist_cb(const geometry_msgs::TwistStamped::ConstPtr&);
	void state_cb(const mavros_msgs::State::ConstPtr&);
	void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr&);
	void rc_input_cb(const mavros_msgs::RCIn::ConstPtr&);
	void command_cb(const geometry_msgs::PoseStamped::ConstPtr&);
	void takeoff_land_cb(const qr_mpc::TakeoffLandTrigger::ConstPtr&);
	bool rc_received(const ros::Time&);
	bool odom_received(const ros::Time&);
	bool command_received(const ros::Time&);
	bool takeoff_land_received(const ros::Time&);
	bool takeoff_trigered(const ros::Time& time);
	bool land_trigered(const ros::Time& time);
	double twist_norm(const geometry_msgs::TwistStamped);
	void reboot();
};

#endif