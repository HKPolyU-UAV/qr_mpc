#ifndef PX4_CONTROL_FSM_H
#define PX4_CONTROL_FSM_H

#include <ros/ros.h>

class PX4_CONTROL_FSM{
    
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

};

#endif