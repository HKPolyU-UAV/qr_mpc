#include "quadrotor_mpc.h"

QUADROTOR_MPC::QUADROTOR_MPC(ros::NodeHandle& nh, const std::string& ref_traj)
{

    // Pre-load the trajectory
    const char * c = ref_traj.c_str();
    number_of_steps = readDataFromFile(c, trajectory);
    if (number_of_steps == 0){
        ROS_WARN("Cannot load CasADi optimal trajectory!");
    }
    else{
        ROS_INFO_STREAM("Number of steps of selected trajectory: " << number_of_steps << std::endl);
    }

    // Initialize MPC
    int create_status = 1;
    create_status = quadrotor_acados_create(mpc_capsule);
    if (create_status != 0){
        ROS_INFO_STREAM("acados_create() returned status " << create_status << ". Exiting." << std::endl);
        exit(1);
    }

    // ROS Subscriber & Publisher
    local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, &QUADROTOR_MPC::local_pose_cb, this);
    local_twist_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 100, &QUADROTOR_MPC::local_twist_cb, this);
    setpoint_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",100); 

    // Initialize
    for(unsigned int i=0; i < QUADROTOR_NU; i++) acados_out.u0[i] = 0.0;
    for(unsigned int i=0; i < QUADROTOR_NX; i++) acados_in.x0[i] = 0.0;

}

void QUADROTOR_MPC::local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    local_pose.pose.position.x = pose->pose.position.x;
    local_pose.pose.position.y = pose->pose.position.y;
    local_pose.pose.position.z = pose->pose.position.z;

    tf::quaternionMsgToTF(pose->pose.orientation,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(local_euler.phi, local_euler.theta, local_euler.psi);
}

void QUADROTOR_MPC::local_twist_cb(const geometry_msgs::TwistStamped::ConstPtr& twist)
{
    local_twist.twist.linear.x = twist->twist.linear.x;
    local_twist.twist.linear.y = twist->twist.linear.y;
    local_twist.twist.linear.z = twist->twist.linear.z;
}

int QUADROTOR_MPC::readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data)
{
    std::ifstream file(fileName);
    std::string line;
    int number_of_lines = 0;

    if (file.is_open())
    {
        while(getline(file, line)){
            number_of_lines++;
            std::istringstream linestream( line );
            std::vector<double> linedata;
            double number;

            while( linestream >> number ){
                linedata.push_back( number );
            }
            data.push_back( linedata );
        }

        file.close();
    }
    else
    {
        return 0;
    }

    return number_of_lines;
}

void QUADROTOR_MPC::ref_cb(int line_to_read)
{
    if (QUADROTOR_N+line_to_read+1 <= number_of_steps)  // All ref points within the file
    {
        for (unsigned int i = 0; i <= QUADROTOR_N; i++)  // Fill all horizon with file data
        {
            for (unsigned int j = 0; j <= QUADROTOR_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[i+line_to_read][j];
            }
        }
    }
    else if(line_to_read < number_of_steps)    // Part of ref points within the file
    {
        for (unsigned int i = 0; i < number_of_steps-line_to_read; i++)    // Fill part of horizon with file data
        {
            for (unsigned int j = 0; j <= QUADROTOR_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[i+line_to_read][j];
            }
        }

        for (unsigned int i = number_of_steps-line_to_read; i <= QUADROTOR_N; i++)  // Fill the rest horizon with the last point
        {
            for (unsigned int j = 0; j <= QUADROTOR_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[number_of_steps-1][j];
            }
        }
    }
    else    // none of ref points within the file
    {
        for (unsigned int i = 0; i <= QUADROTOR_N; i++)  // Fill all horizon with the last point
        {
            for (unsigned int j = 0; j <= QUADROTOR_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[number_of_steps-1][j];
            }
        }
    }
}

void QUADROTOR_MPC::run()
{
    acados_in.x0[x] = local_pose.pose.position.x;
    acados_in.x0[y] = local_pose.pose.position.y;
    acados_in.x0[z] = local_pose.pose.position.z;
    acados_in.x0[u] = local_twist.twist.linear.x;
    acados_in.x0[v] = local_twist.twist.linear.y;
    acados_in.x0[w] = local_twist.twist.linear.z;
    acados_in.x0[phi] = local_euler.phi;
    acados_in.x0[theta] = local_euler.theta;
    acados_in.x0[psi] = local_euler.psi;

    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "lbx", acados_in.x0);
    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "ubx", acados_in.x0);

    ref_cb(line_number);
    line_number++;

    for (unsigned int i = 0; i <= QUADROTOR_N; i++)
        {
        ocp_nlp_cost_model_set(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_in, i, "yref", acados_in.yref[i]);
        }

    acados_status = quadrotor_acados_solve(mpc_capsule);

    if (acados_status != 0){
        ROS_INFO_STREAM("acados returned status " << acados_status << std::endl);
    }

    acados_out.status = acados_status;
    acados_out.kkt_res = (double)mpc_capsule->nlp_out->inf_norm_res;

    ocp_nlp_get(mpc_capsule->nlp_config, mpc_capsule->nlp_solver, "time_tot", &acados_out.cpu_time);

    ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_out, 0, "u", (void *)acados_out.u0);

    attitude_target.thrust = acados_out.u0[0];  
    target_euler.phi = acados_out.u0[1];
    target_euler.theta = acados_out.u0[2];
    target_euler.psi = acados_out.u0[3];

    geometry_msgs::Quaternion target_quaternion = tf::createQuaternionMsgFromRollPitchYaw(target_euler.phi, target_euler.theta, target_euler.psi);

    attitude_target.orientation.w = target_quaternion.w;
    attitude_target.orientation.x = target_quaternion.x;
    attitude_target.orientation.y = target_quaternion.y;
    attitude_target.orientation.z = target_quaternion.z;

    setpoint_pub.publish(attitude_target);


    /*Mission information cout**********************************************/        
    if(cout_counter > 2){ //reduce cout rate
        std::cout << "------------------------------------------------------------------------------" << std::endl;
        std::cout << "x_ref:      " << acados_in.yref[0][0] << "\ty_ref:      " << acados_in.yref[0][1] << "\tz_ref:         " << acados_in.yref[0][2] << std::endl;
        std::cout << "x_gt:       " << acados_in.x0[x] << "\ty_gt:       " << acados_in.x0[y] << "\tz_gt:          " << acados_in.x0[z] << std::endl;
        std::cout << "theta_cmd:  " << target_euler.theta << "\tphi_cmd:    " << target_euler.phi <<  "\tpsi_cmd:       " << target_euler.psi << std::endl;
        std::cout << "theta_gt:   " << local_euler.theta << "\tphi_gt:     " << local_euler.phi <<  "\tpsi_gt:        " << local_euler.psi << std::endl;
        std::cout << "thrust_cmd: " << attitude_target.thrust << "\tsolve_time: "<< acados_out.cpu_time  << "\tacados_status: " << acados_out.status << std::endl;
        std::cout << "ros_time:   " << std::fixed << ros::Time::now().toSec() << std::endl;
        std::cout << "------------------------------------------------------------------------------" << std::endl;
        cout_counter = 0;
    }
    else{
        cout_counter++;
    }
}