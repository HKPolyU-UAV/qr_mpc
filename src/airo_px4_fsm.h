#ifndef Airo_PX4_FSM_H
#define Airo_PX4_FSM_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "quadrotor_mpc.h"


class AIRO_PX4_FSM{
    private:

    enum State_FSM
	{
		MANUAL_CTRL = 1, // px4ctrl is deactived. FCU is controled by the remote controller only
		AUTO_HOVER, // px4ctrl is actived, it will keep the drone hover from odom measurments while waiting for commands from PositionCommand topic.
		CMD_CTRL,	// px4ctrl is actived, and controling the drone.
		AUTO_TAKEOFF,
		AUTO_LAND
	};
    State_FSM state;
	mavros_msgs::AttitudeTarget attitude_target;
	ros::Time current_time;

	//ROS Sub & Pub
	ros::Subscriber pose_sub;
	ros::Subscriber twist_sub;
	ros::Publisher setpoint_pub;

	//States
	geometry_msgs::PoseStamped local_pose;
	geometry_msgs::TwistStamped local_twist;

	//Controller
	QUADROTOR_MPC controller;

	//Ref
	Eigen::VectorXd ref;

	public:

	AIRO_PX4_FSM(ros::NodeHandle& nh);
	void process();
	void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void publish_control_commands(mavros_msgs::AttitudeTarget,ros::Time);
};

#endif