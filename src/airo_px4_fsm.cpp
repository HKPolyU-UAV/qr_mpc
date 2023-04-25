#include "airo_px4_fsm.h"

AIRO_PX4_FSM::AIRO_PX4_FSM(ros::NodeHandle& nh){
    // ROS Sub & Pub
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",100,&AIRO_PX4_FSM::pose_cb,this);
    twist_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local",100,&AIRO_PX4_FSM::twist_cb,this);
    setpoint_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",20); 

    ref.resize(11);

    // Ref
    ref << 1,1,1,0,0,0,0,0,0.56,0,0;
}

void AIRO_PX4_FSM::process(){
    current_time = ros::Time::now();
    attitude_target = controller.run(local_pose,local_twist,ref);
    publish_control_commands(attitude_target,current_time);
}

void AIRO_PX4_FSM::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.pose = msg->pose;
}

void AIRO_PX4_FSM::twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    local_twist.twist.linear.x = msg->twist.linear.x;
    local_twist.twist.linear.y = msg->twist.linear.y;
    local_twist.twist.linear.z = msg->twist.linear.z;
    local_twist.twist.angular.x = msg->twist.angular.x;
    local_twist.twist.angular.y = msg->twist.angular.y;
    local_twist.twist.angular.z = msg->twist.angular.z;
}

void AIRO_PX4_FSM::publish_control_commands(mavros_msgs::AttitudeTarget target,ros::Time time){
    attitude_target.header.stamp = time;
    attitude_target.header.frame_id = std::string("FCU");
    attitude_target.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

    setpoint_pub.publish(attitude_target);
}