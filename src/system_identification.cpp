#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "system_identification_node");
    ros::NodeHandle nh;

    ros::Rate loop_rate(40);

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
