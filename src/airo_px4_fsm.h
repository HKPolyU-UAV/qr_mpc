#ifndef Airo_PX4_FSM_H
#define Airo_PX4_FSM_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/SetMode.h>

// #include "qr_mpc/PositionCommand.h"

#include "rc_input.h"
#include "quadrotor_mpc.h"


class AIRO_PX4_FSM{
    private:

    enum State_FSM{
		RC_CONTROL,
		AUTO_HOVER,
		AUTO_TAKEOFF,
		AUTO_LAND,
		POS_CONTROL
	};

	// Parameters
	static constexpr double MESSAGE_TIMEOUT = 0.5;
	static constexpr double MOTOR_SPEEDUP_TIME = 5.0;
	static constexpr double HOVER_THRUST = 0.56;

	//
	State_FSM state_fsm;
	RC_INPUT rc_input;

	// Times
	ros::Time current_time;
	ros::Time takeoff_time;
	
	// ROS Sub & Pub
	ros::Subscriber pose_sub;
	ros::Subscriber twist_sub;
	ros::Subscriber state_sub;
	ros::Subscriber rc_input_sub;
	ros::Publisher setpoint_pub;

	// ROS Services
	ros::ServiceClient setmode_srv;
	ros::ServiceClient arm_srv;

	// Messages
	geometry_msgs::PoseStamped local_pose;
	geometry_msgs::PoseStamped takeoff_pose;
	geometry_msgs::TwistStamped local_twist;
	mavros_msgs::AttitudeTarget attitude_target;
	mavros_msgs::State current_mode;
	mavros_msgs::State previous_mode;

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
	void pose_cb(const geometry_msgs::PoseStamped::ConstPtr&);
	void twist_cb(const geometry_msgs::TwistStamped::ConstPtr&);
	void state_cb(const mavros_msgs::State::ConstPtr&);
	void rc_input_cb(const mavros_msgs::RCIn::ConstPtr&);
	bool rc_received(const ros::Time&);
	bool odom_received(const ros::Time&);
	double twist_norm(const geometry_msgs::TwistStamped);
};

#endif