#ifndef QUADROTOR_MPC_H
#define QUADROTOR_MPC_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <iostream>
#include <fstream>

#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#include "quadrotor_model/quadrotor_model.h"
#include "acados_solver_quadrotor.h"

class QUADROTOR_MPC{

    private:

    enum SystemStates{
        x = 0,
        y = 1,
        z = 2,
        u = 3,
        v = 4,
        w = 5,
        phi = 6,
        theta = 7,
        psi = 8,
    };

    enum ControlInputs{
        thrust = 0,
        phi_cmd = 1,
        theta_cmd = 2,
        psi_cmd = 3,
    };

    struct SolverInput{
        double x0[QUADROTOR_NX];
        double yref[QUADROTOR_N+1][QUADROTOR_NY];
    };

    struct SolverOutput{
        double u0[QUADROTOR_NU];
        double x1[QUADROTOR_NX];
        double status, kkt_res, cpu_time;
    };

    struct Euler{
        double phi;
        double theta;
        double psi;
    };

    // ROS subscriber and publisher
    ros::Subscriber local_pose_sub;
    ros::Subscriber local_twist_sub;
    ros::Publisher setpoint_pub;

    // ROS message variables
    geometry_msgs::PoseStamped local_pose;
    geometry_msgs::TwistStamped local_twist;
    Euler local_euler;
    Euler target_euler;
    mavros_msgs::AttitudeTarget attitude_target;

    // Acados variables
    SolverInput acados_in;
    SolverOutput acados_out;
    int acados_status;   
    quadrotor_solver_capsule * mpc_capsule = quadrotor_acados_create_capsule();
    
    // Trajectory variables
    std::vector<std::vector<double>> trajectory;
    int line_number = 0;
    int number_of_steps = 0;

    // Other variables
    tf::Quaternion tf_quaternion;
    int cout_counter = 0;
    double logger_time;

    public:

    QUADROTOR_MPC(ros::NodeHandle& nh, const std::string& ref_traj);
    void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose);
    void local_twist_cb(const geometry_msgs::TwistStamped::ConstPtr& twist);
    int readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data);
    void ref_cb(int line_to_read);
    void run();

};

#endif