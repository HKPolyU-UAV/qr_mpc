#include <ros/ros.h>

#include "quadrotor_mpc.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_control_node");
    ros::NodeHandle nh;
    std::string ref_traj;
    nh.getParam("/px4_control_node/ref_traj", ref_traj);
    QUADROTOR_MPC mpc(nh, ref_traj);
    ros::Rate loop_rate(40);

    while(ros::ok()){
        mpc.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
