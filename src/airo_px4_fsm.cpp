#include "airo_px4_fsm.h"

AIRO_PX4_FSM::AIRO_PX4_FSM(ros::NodeHandle& nh){
    // Initialize
    state_fsm = RC_CONTROL;

    // ROS Sub & Pub
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",100,&AIRO_PX4_FSM::pose_cb,this);
    twist_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local",100,&AIRO_PX4_FSM::twist_cb,this);
    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state",10,&AIRO_PX4_FSM::state_cb,this);
    rc_input_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",10,&AIRO_PX4_FSM::rc_input_cb,this);
    setpoint_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",20);

    // ROS Services
    setmode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arm_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    // Ref to controller
    ref.resize(11);
    ref<<0,0,0,0,0,0,0,0,0,0,0;
}

void AIRO_PX4_FSM::process(){
    current_time = ros::Time::now();
    fsm();
    attitude_target = controller.run(local_pose,local_twist,ref);
    publish_control_commands(attitude_target,current_time);
}

void AIRO_PX4_FSM::fsm(){
    switch (state_fsm){
        case RC_CONTROL:{
            // To AUTO_HOVER
            if (0){

            }

            // To AUTO_TAKEOFF
            else if (rc_input.enter_offboard){
                if (!odom_received(current_time)){
                    ROS_ERROR("[AIRo PX4] Reject AUTO_TAKEOFF. No odom!");
                    break;
                }
                if (twist_norm(local_twist) > 0.1){
                    ROS_ERROR("[AIRo PX4] Reject AUTO_TAKEOFF. Norm Twist=%fm/s, dynamic takeoff is not allowed!", twist_norm(local_twist));
                    break;
                }
                
			    if (toggle_offboard(true)){
                    for (int i = 0; i < 10 && ros::ok(); ++i) // wait for 0.1 seconds to allow FMU mode change
                    {
                        ros::Duration(0.01).sleep();
                        ros::spinOnce();
                    }

                    if(toggle_arm(true)){
                        takeoff_pose = local_pose;
                        takeoff_time = current_time;
                        ROS_INFO("\033[32m[AIRo PX4] RC_CONTROL ==> AUTO_TAKEOFF\033[32m");
                        state_fsm = AUTO_TAKEOFF;
                    }
                }
            }

            // Try to reboot


            break;
        }

        case AUTO_HOVER:{

        }

        case AUTO_TAKEOFF:{
            if ((current_time - takeoff_time).toSec() < MOTOR_SPEEDUP_TIME) // Wait for several seconds to warn prople
            {
                get_motor_speedup();
            }
            // else if (odom_data.p(2) >= (takeoff_land.start_pose(2) + param.takeoff_land.height)) // reach the desired height
            // {
            //     state = AUTO_HOVER;
            //     set_hov_with_odom();
            //     ROS_INFO("\033[32m[px4ctrl] AUTO_TAKEOFF --> AUTO_HOVER(L2)\033[32m");

            //     takeoff_land.delay_trigger.first = true;
            //     takeoff_land.delay_trigger.second = now_time + ros::Duration(AutoTakeoffLand_t::DELAY_TRIGGER_TIME);
            // }
            // else
            // {
            //     des = get_takeoff_land_des(param.takeoff_land.speed);
            // }
            // break;
        }

        case AUTO_LAND:{

        }

        case POS_CONTROL:{

        }
    }
}

void AIRO_PX4_FSM::publish_control_commands(mavros_msgs::AttitudeTarget target,ros::Time time){
    attitude_target.header.stamp = time;
    attitude_target.header.frame_id = std::string("FCU");
    attitude_target.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

    setpoint_pub.publish(attitude_target);
}

bool AIRO_PX4_FSM::toggle_offboard(bool flag)
{
	mavros_msgs::SetMode offboard_setmode;
    ROS_INFO("Trying set offboard mode");
	if (flag)
	{
		previous_mode = current_mode;
		if (previous_mode.mode == "OFFBOARD"){
            previous_mode.mode = "MANUAL"; // Not allowed
        }
		offboard_setmode.request.custom_mode = "OFFBOARD";
		if (!(setmode_srv.call(offboard_setmode) && offboard_setmode.response.mode_sent))
		{
			ROS_ERROR("Enter OFFBOARD rejected by PX4!");
			return false;
		}
	}
	else
	{
		offboard_setmode.request.custom_mode = previous_mode.mode;
		if (!(setmode_srv.call(offboard_setmode) && offboard_setmode.response.mode_sent))
		{
			ROS_ERROR("Exit OFFBOARD rejected by PX4!");
			return false;
		}
	}

	return true;

	// if (param.print_dbg)
	// 	printf("offb_set_mode mode_sent=%d(uint8_t)\n", offb_set_mode.response.mode_sent);
}

bool AIRO_PX4_FSM::toggle_arm(bool flag){
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = flag;

	if (!(arm_srv.call(arm_cmd) && arm_cmd.response.success))
	{
		if (flag)
			ROS_ERROR("ARM rejected by PX4!");
		else
			ROS_ERROR("DISARM rejected by PX4!");

		return false;
	}

    if (flag)
        ROS_WARN("Vehicle arming!");
    else
        ROS_WARN("Vehicle disarmed!");
    
	return true; 
}

void AIRO_PX4_FSM::get_motor_speedup(){
	double delta_t = (current_time - takeoff_time).toSec();
	double ref_thrust = (delta_t/MOTOR_SPEEDUP_TIME)*HOVER_THRUST*0.8 + 0.005;
	ref<<takeoff_pose.pose.position.x,takeoff_pose.pose.position.y,takeoff_pose.pose.position.z,0.0,0.0,0.0,0.0,0.0,ref_thrust,0.0,0.0;
}

void AIRO_PX4_FSM::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.header.stamp = msg->header.stamp;
    local_pose.pose = msg->pose;
}

void AIRO_PX4_FSM::twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    local_twist.header.stamp = msg->header.stamp;
    local_twist.twist = msg->twist;
}

void AIRO_PX4_FSM::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_mode = *msg;
}

void AIRO_PX4_FSM::rc_input_cb(const mavros_msgs::RCIn::ConstPtr& msg){
    rc_input.process(msg);
}

bool AIRO_PX4_FSM::rc_received(const ros::Time& time){
    return (time - rc_input.stamp).toSec() < MESSAGE_TIMEOUT;
}

bool AIRO_PX4_FSM::odom_received(const ros::Time& time){
    return (time - local_pose.header.stamp).toSec() < MESSAGE_TIMEOUT && (time - local_twist.header.stamp).toSec() < MESSAGE_TIMEOUT;
}

double AIRO_PX4_FSM::twist_norm(const geometry_msgs::TwistStamped twist){
    return sqrt(twist.twist.linear.x*twist.twist.linear.x + twist.twist.linear.y*twist.twist.linear.y + twist.twist.linear.z*twist.twist.linear.z);
}