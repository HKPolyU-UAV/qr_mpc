#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <qr_mpc/AiroPx4.h>
#include <qr_mpc/TakeoffLandTrigger.h>

geometry_msgs::PoseStamped local_pose;
geometry_msgs::PoseStamped target_pose_1;
geometry_msgs::PoseStamped target_pose_2;
qr_mpc::AiroPx4 airo_px4;
qr_mpc::TakeoffLandTrigger takeoff_land_trigger;
bool target_1_reached = false;

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

    target_pose_1.pose.position.x = 1.5;
    target_pose_1.pose.position.y = 1.5;
    target_pose_1.pose.position.z = 1.5;

    target_pose_2.pose.position.x = -1.5;
    target_pose_2.pose.position.y = -1.5;
    target_pose_2.pose.position.z = 2.5;

    while(ros::ok()){
        switch(state){
            case TAKEOFF:{
                if(airo_px4.is_landed == true){
                    while(ros::ok()){
                        takeoff_land_trigger.takeoff_land_trigger = true; // Takeoff
                        takeoff_land_trigger.header.stamp = ros::Time::now();
                        takeoff_land_pub.publish(takeoff_land_trigger);
                        ros::spinOnce();
                        ros::Duration(0.5).sleep();
                        if(airo_px4.is_waiting_for_command){
                            state = COMMAND;
                            break;
                        }
                    }
                }
                break;
            }

            case COMMAND:{
                if(airo_px4.is_waiting_for_command){
                    if(!target_1_reached){
                        target_pose_1.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_1);
                        if(abs(local_pose.pose.position.x - target_pose_1.pose.position.x)
                         + abs(local_pose.pose.position.y - target_pose_1.pose.position.y)
                         + abs(local_pose.pose.position.z - target_pose_1.pose.position.z) < 0.5){
                            target_1_reached = true;
                        }
                    }
                    else{
                        target_pose_2.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_2);
                        if(abs(local_pose.pose.position.x - target_pose_2.pose.position.x)
                         + abs(local_pose.pose.position.y - target_pose_2.pose.position.y)
                         + abs(local_pose.pose.position.z - target_pose_2.pose.position.z) < 0.5){
                            state = LAND;
                        }
                    }
                }
                break;
            }

            case LAND:{
                if(airo_px4.is_waiting_for_command){
                    takeoff_land_trigger.takeoff_land_trigger = false; // Land
                    takeoff_land_trigger.header.stamp = ros::Time::now();
                    takeoff_land_pub.publish(takeoff_land_trigger);
                }
                break;
            }
        }

        ros::spinOnce();
        ros::Duration(rate).sleep();
    }

    return 0;
}