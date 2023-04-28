#include "airo_px4_fsm.h"

AIRO_PX4_FSM::AIRO_PX4_FSM(ros::NodeHandle& nh){
    // Initialize
    state_fsm = RC_MANUAL;

    // ROS Sub & Pub
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",100,&AIRO_PX4_FSM::pose_cb,this);
    twist_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local",100,&AIRO_PX4_FSM::twist_cb,this);
    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state",10,&AIRO_PX4_FSM::state_cb,this);
    extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state",10,&AIRO_PX4_FSM::extended_state_cb,this);
    rc_input_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",10,&AIRO_PX4_FSM::rc_input_cb,this);
    setpoint_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",20);

    // ROS Services
    setmode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arm_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    // Ref to controller
    ref.resize(11);
    ref<<0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
}

void AIRO_PX4_FSM::process(){
    // Step 1: Update varialbes
    current_time = ros::Time::now();
    solve_controller = false;

    // Step 2: State machine
    fsm();

    // Step 3: Solve position controller if needed
    if(solve_controller){
        attitude_target = controller.run(local_pose,local_twist,ref);
    }

    // Step 4: Publish control commands
    publish_control_commands(attitude_target,current_time);

    // Step 5: Detect if landed
    land_detector();
    
    // Step 6: Reset all triggers
	rc_input.enter_offboard = false;
	rc_input.enter_command = false;
	rc_input.enter_reboot = false;
}

void AIRO_PX4_FSM::fsm(){
    switch (state_fsm){
        case RC_MANUAL:{
            // To AUTO_HOVER
            if (0){

            }

            // To AUTO_TAKEOFF
            else if (rc_input.enter_offboard){
                if (!odom_received(current_time)){
                    ROS_ERROR("[AIRo PX4] Reject AUTO_TAKEOFF. No odom!");
                    break;
                }
                if (twist_norm(local_twist) > 0.15){
                    ROS_ERROR("[AIRo PX4] Reject AUTO_TAKEOFF. Norm Twist=%fm/s, dynamic takeoff is not allowed!", twist_norm(local_twist));
                    break;
                }
                if (!rc_input.is_offboard || !rc_input.check_centered()){
                    ROS_ERROR("[AIRo PX4] Reject AUTO_TAKEOFF. Center the joysticks and enable offboard switch!");
                    while (ros::ok()){
                        ros::Duration(0.1).sleep();
                        ros::spinOnce();
                        if (rc_input.is_offboard && rc_input.check_centered()){
                            current_time = ros::Time::now();
                            break;
                        }
                    }
                }
			    if (toggle_offboard(true)){
                    for (int i = 0; i < 10 && ros::ok(); ++i){ // Wait for 0.1 seconds to allow FMU mode change
                        ros::Duration(0.01).sleep();
                        ros::spinOnce();
                    }

                    if(toggle_arm(true)){
                        takeoff_land_init();
                        state_fsm = AUTO_TAKEOFF;
                        ROS_INFO("\033[32m[AIRo PX4] RC_CONTROL ==>> AUTO_TAKEOFF\033[32m");
                        break;
                    }
                }
            }

            // Try to reboot


            break;
        }

        case AUTO_HOVER:{
            if (!rc_input.is_offboard || !odom_received(current_time)){
                state_fsm = RC_MANUAL;
                toggle_offboard(false);
                if(!rc_input.is_offboard){
                ROS_INFO("\033[32m[AIRo PX4] AUTO_HOVER ==>> RC_CONTROL\033[32m");
                }
                else{
                ROS_ERROR("[AIRo PX4] No odom! Switching to RC_CONTROL mode.");
                }
            }
            else if (0){
                takeoff_land_init();
                state_fsm = AUTO_LAND;
                ROS_INFO("\033[32m[AIRo PX4] AUTO_HOVER ==>> AUTO_LAND\033[32m");
            }
            else if (is_landed){
                motor_idle_and_disarm();
                ROS_INFO("\033[32m[AIRo PX4] AUTO_HOVER ==>> RC_MANUAL\033[32m");
            }
            else{
                set_ref_with_rc();
            }

            break;
        }

        case AUTO_TAKEOFF:{
            get_motor_speedup(); // Wait for several seconds to warn prople

            if (!rc_input.is_offboard || !odom_received(current_time)){
                state_fsm = RC_MANUAL;
                toggle_offboard(false);
                if(!rc_input.is_offboard){
                ROS_INFO("\033[32m[AIRo PX4] AUTO_TAKEOFF ==>> RC_CONTROL\033[32m");
                }
                else{
                ROS_ERROR("[AIRo PX4] No odom! Switching to RC_CONTROL mode.");
                }
            }
            else{
                if (local_pose.pose.position.z > (takeoff_land_pose.pose.position.z + TAKEOFF_HEIGHT)){ // Reach the desired height
                    auto_hover_init();
                    state_fsm = AUTO_HOVER;
                    ROS_INFO("\033[32m[AIRo PX4] AUTO_TAKEOFF ==>> AUTO_HOVER\033[32m");
                }
                else{
                    get_takeoff_land_ref(TAKEOFF_LAND_SPEED);
                }
            }

            break;
        }

        case AUTO_LAND:{
            if (!rc_input.is_offboard || !odom_received(current_time)){
                state_fsm = RC_MANUAL;
                toggle_offboard(false);
                if(!rc_input.is_offboard){
                ROS_INFO("\033[32m[AIRo PX4] AUTO_LAND ==>> RC_CONTROL\033[32m");
                }
                else{
                ROS_ERROR("[AIRo PX4] No odom! Switching to RC_CONTROL mode.");
                }
            }
            else if (!rc_input.is_command){
                auto_hover_init();
                state_fsm = AUTO_HOVER;
                ROS_INFO("\033[32m[AIRo PX4] AUTO_LAND ==>> AUTO_HOVER\033[32m");
            }
            else if (!is_landed){
                get_takeoff_land_ref(-TAKEOFF_LAND_SPEED);
            }
            else{
                motor_idle_and_disarm();
                ROS_INFO("\033[32m[AIRo PX4] AUTO_LAND ==>> RC_MANUAL\033[32m");
            }

            break;            
        }

        case POS_COMMAND:{
            

            break;
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
    ROS_WARN("[AIRo PX4] Setting offboard mode");
	if (flag){
		previous_state = current_state;
		if (previous_state.mode == "OFFBOARD"){
            previous_state.mode = "MANUAL"; // Not allowed
        }
		offboard_setmode.request.custom_mode = "OFFBOARD";
		if (!(setmode_srv.call(offboard_setmode) && offboard_setmode.response.mode_sent)){
			ROS_ERROR("[AIRo PX4] Enter OFFBOARD rejected by PX4!");
			return false;
		}
	}
	else{
		offboard_setmode.request.custom_mode = previous_state.mode;
		if (!(setmode_srv.call(offboard_setmode) && offboard_setmode.response.mode_sent)){
			ROS_ERROR("[AIRo PX4] Exit OFFBOARD rejected by PX4!");
			return false;
		}
	}
	return true;
}

bool AIRO_PX4_FSM::toggle_arm(bool flag){
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = flag;

	if (!(arm_srv.call(arm_cmd) && arm_cmd.response.success)){
		if (flag)
			ROS_ERROR("[AIRo PX4] ARM rejected by PX4!");
		else
			ROS_ERROR("[AIRo PX4] DISARM rejected by PX4!");

		return false;
	}

    if (flag){
        ROS_WARN("[AIRo PX4] Vehicle arming!");
    }
    else{
        ROS_WARN("[AIRo PX4] Vehicle disarmed!");
    }
    
	return true; 
}

void AIRO_PX4_FSM::get_motor_speedup(){
    while(ros::ok() && (current_time - takeoff_land_time).toSec() < MOTOR_SPEEDUP_TIME){
        double delta_t = (current_time - takeoff_land_time).toSec();
	    double ref_thrust = (delta_t/MOTOR_SPEEDUP_TIME)*HOVER_THRUST*0.8 + 0.005;

        attitude_target.thrust = ref_thrust;
        attitude_target.orientation.w = takeoff_land_pose.pose.orientation.w;
        attitude_target.orientation.x = takeoff_land_pose.pose.orientation.x;
        attitude_target.orientation.y = takeoff_land_pose.pose.orientation.y;
        attitude_target.orientation.z = takeoff_land_pose.pose.orientation.z;

        publish_control_commands(attitude_target,current_time);
        ros::Duration(0.05).sleep();
        ros::spinOnce();
        current_time = ros::Time::now();
    }
}

void AIRO_PX4_FSM::get_takeoff_land_ref(const double speed){
    current_time = ros::Time::now();
    double delta_t = (current_time - takeoff_land_time).toSec() - (speed > 0 ? MOTOR_SPEEDUP_TIME : 0); // speed > 0 means takeoff
    Eigen::Vector4d takeoff_land_ref;
    takeoff_land_ref(0) = takeoff_land_pose.pose.position.x;
    takeoff_land_ref(1) = takeoff_land_pose.pose.position.y;
    takeoff_land_ref(2) = takeoff_land_pose.pose.position.z + speed * delta_t;
    takeoff_land_ref = check_safety_volumn(takeoff_land_ref);

	set_ref(takeoff_land_ref(0),takeoff_land_ref(1),takeoff_land_ref(2));
}

void AIRO_PX4_FSM::set_ref(double x, double y, double z){
    ref<<x,y,z,0.0,0.0,0.0,0.0,0.0,HOVER_THRUST,0.0,0.0;
    solve_controller = true;
}

void AIRO_PX4_FSM::set_ref_with_rc(){
    double delta_t = (current_time - last_hover_time).toSec();
    Eigen::Vector4d ref_rc(4);
    ref_rc(0) = ref(0) + rc_input.channel[0]*HOVER_MAX_VELOCITY*delta_t;
    ref_rc(1) = ref(1) - rc_input.channel[1]*HOVER_MAX_VELOCITY*delta_t;
    ref_rc(2) = ref(2) + rc_input.channel[3]*HOVER_MAX_VELOCITY*delta_t;

    ref_rc = check_safety_volumn(ref_rc);
    set_ref(ref_rc[0],ref_rc[1],ref_rc[2]);
    
    last_hover_time = current_time;
}

void AIRO_PX4_FSM::land_detector(){
	static STATE_FSM last_state_fsm = STATE_FSM::RC_MANUAL;
	if (state_fsm == STATE_FSM::AUTO_HOVER || state_fsm == STATE_FSM::AUTO_TAKEOFF)
	{
		is_landed = false; // Always holds
	}
	last_state_fsm = state_fsm;

	if (state_fsm == STATE_FSM::RC_MANUAL && !current_state.armed)
	{
		is_landed = true;
		return; // No need of other decisions
	}

	// Land_detector parameters
	constexpr double POSITION_DEVIATION = -0.5; // Constraint 1: target position below real position for POSITION_DEVIATION meters.
	constexpr double VELOCITY_THRESHOLD = 0.1; // Constraint 2: velocity below VELOCITY_THRESHOLD m/s.
	constexpr double TIME_KEEP = 2.0; // Constraint 3: Constraint 1&2 satisfied for TIME_KEEP seconds.

	static ros::Time time_C12_reached;
	static bool is_last_C12_satisfy;
	if (is_landed)
	{
		time_C12_reached = ros::Time::now();
		is_last_C12_satisfy = false;
	}
	else
	{
		bool C12_satisfy = (ref(2) - local_pose.pose.position.z) < POSITION_DEVIATION && twist_norm(local_twist) < VELOCITY_THRESHOLD;
		if (C12_satisfy && !is_last_C12_satisfy)
		{
			time_C12_reached = ros::Time::now();
		}
		else if (C12_satisfy && is_last_C12_satisfy)
		{
			if ((ros::Time::now() - time_C12_reached).toSec() > TIME_KEEP) //Constraint 3 reached
			{
				is_landed = true;
			}
		}

		is_last_C12_satisfy = C12_satisfy;
	}
}

void AIRO_PX4_FSM::motor_idle_and_disarm(){
    double last_disarm_time = 0;
    while(ros::ok()){
        if (current_extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) // PX4 allows disarm after this
        {
            if (current_time.toSec() - last_disarm_time > 1.0)
            {
                if (toggle_arm(false)){ // Successful disarm
                    state_fsm = RC_MANUAL;
                    toggle_offboard(false); // Toggle off offboard after disarm
                    break;
                }
                last_disarm_time = current_time.toSec();
            }
        }
        else{            
            attitude_target.thrust = 0.03;
            attitude_target.orientation.w = local_pose.pose.orientation.w;
            attitude_target.orientation.x = local_pose.pose.orientation.x;
            attitude_target.orientation.y = local_pose.pose.orientation.y;
            attitude_target.orientation.z = local_pose.pose.orientation.z;
            publish_control_commands(attitude_target,current_time);
        }
        ros::Duration(0.05).sleep();
        ros::spinOnce();
        current_time = ros::Time::now();
    }
}

void AIRO_PX4_FSM::takeoff_land_init(){
    takeoff_land_pose = local_pose;
    takeoff_land_time = current_time;
}

void AIRO_PX4_FSM::auto_hover_init(){
    last_hover_time = current_time;
    set_ref(local_pose.pose.position.x,local_pose.pose.position.y,local_pose.pose.position.z);
}

Eigen::Vector4d AIRO_PX4_FSM::check_safety_volumn(const Eigen::Vector4d ref_rc){
    Eigen::Vector4d ref_safe(4);

    if(ref_rc(0) < SAFETY_VOLUMN[0]){ // x_ref < x_min
        ref_safe(0) = SAFETY_VOLUMN[0];
        ROS_WARN_STREAM_THROTTLE(1.0,"[AIRo PX4] X command too large!");
    }
    else if (ref_rc(0) > SAFETY_VOLUMN[1]){ // x_ref > x_max
        ref_safe(0) = SAFETY_VOLUMN[1];
        ROS_WARN_STREAM_THROTTLE(1.0,"[AIRo PX4] X command too small!");
    }
    else ref_safe(0) = ref_rc(0); // x_min < x_ref < x_max

    if(ref_rc(1) < SAFETY_VOLUMN[2]){ // y_ref < y_min
        ref_safe(1) = SAFETY_VOLUMN[2];
        ROS_WARN_STREAM_THROTTLE(1.0,"[AIRo PX4] Y command too large!");
    }
    else if (ref_rc(1) > SAFETY_VOLUMN[3]){ // y_ref > y_max
        ref_safe(1) = SAFETY_VOLUMN[3];
        ROS_WARN_STREAM_THROTTLE(1.0,"[AIRo PX4] Y command too small!");
    }
    else ref_safe(1) = ref_rc(1); // y_min < y_ref < y_max

    if (ref_rc(2) > SAFETY_VOLUMN[5]){ // z_ref > z_max
        ref_safe(2) = SAFETY_VOLUMN[5];
        ROS_WARN_STREAM_THROTTLE(1.0,"[AIRo PX4] Z command too large!");
    }
    else ref_safe(2) = ref_rc(2); // z_ref < z_max

    ref_safe(3) = ref_rc(3);
    return ref_safe;
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
    current_state = *msg;
}

void AIRO_PX4_FSM::extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg){
    current_extended_state = *msg;
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