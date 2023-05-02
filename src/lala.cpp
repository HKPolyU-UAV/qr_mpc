#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <qr_mpc/AiroPx4.h>
#include <qr_mpc/TakeoffLandTrigger.h>

geometry_msgs::PoseStamped local_pose;
geometry_msgs::PoseStamped target_pose;
qr_mpc::AiroPx4 airo_px4;
qr_mpc::TakeoffLandTrigger takeoff_land_trigger;

enum State{
    TAKEOFF,
    COMMAND,
    LAND
};

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.header = msg->header;
    local_pose.pose = msg->pose;
}

void airo_px4_cb(const qr_mpc::AiroPx4::ConstPtr& msg){
    airo_px4.header = msg->header;
    airo_px4.is_landed = msg->is_landed;
    airo_px4.is_waiting_for_command = msg->is_waiting_for_command;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);
    State state = TAKEOFF;

    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",100,pose_cb);
    ros::Subscriber airo_px4_sub = nh.subscribe<qr_mpc::AiroPx4>("/airo_px4/state",10,airo_px4_cb);
    ros::Publisher command_pub = nh.advertise<geometry_msgs::PoseStamped>("/airo_px4/position_setpoint",10);
    ros::Publisher takeoff_land_pub = nh.advertise<qr_mpc::TakeoffLandTrigger>("/airo_px4/takeoff_land_trigger",10);

    target_pose.pose.position.x = 1.5;
    target_pose.pose.position.y = 1.5;
    target_pose.pose.position.z = 1.5;

    while(ros::ok()){
        switch(state){
            case TAKEOFF:{
                    std::cout<<"takeoff"<<std::endl;
                    state = COMMAND;
                break;
            }

            case COMMAND:{
                if(1){
                    std::cout<<"command"<<std::endl;
                    break;
                }
                else{
                    state = LAND;
                }
            }

            case LAND:{
                std::cout<<"land"<<std::endl;
            }
        }

        ros::spinOnce();
        ros::Duration(rate).sleep();
    }

    return 0;
}