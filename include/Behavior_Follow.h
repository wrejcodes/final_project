#ifndef BEHAVIOR_FOLLOW_H
#define BEHAVIOR_FOLLOW_H

#include "ros/ros.h"
#include "final_project/behavior.h"
#include "sensor_msgs/LaserScan.h"
#include "final_project/trigger.h"

#define BH_FOLLOW_RATE 70.00f

class Behavior_Follow{
public:
	
	//static constexpr double DRIVE_TURN = .15f;
	//static constexpr double DELTA_THRESHOLD = .15f;
	
	//Follow Math Constants
	static constexpr double DESIRED_FOLLOW_DISTANCE = 1.50f;
	static constexpr double FWD_VEL = 0.25f;		
	static constexpr double TRN_VEL = 0.25f;
	static constexpr double TRN_THRESHOLD = 0.15f;

	//The boundaries for a front pointing ray.
	static constexpr int FOLLOW_RANGE_SIZE = 15; 

	static constexpr double gain_switch = 0.02;

	//PID Constants
	static constexpr double F_P_GAIN = gain_switch;
	static constexpr double F_I_GAIN = gain_switch;
	static constexpr double F_D_GAIN = gain_switch;
	static constexpr double TR_P_GAIN = gain_switch;
	static constexpr double TR_I_GAIN = gain_switch;
	static constexpr double TR_D_GAIN = gain_switch;
	static constexpr double TL_P_GAIN = gain_switch;
	static constexpr double TL_I_GAIN = gain_switch;
	static constexpr double TL_D_GAIN = gain_switch;

	//Don't start detecting until
	static constexpr double TRIGGER_FOLLOW_DISTANCE = 3.00f;

	Behavior_Follow();

	//Functions to make it go
	void hokuyo_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void process_behavior();
	
private:
	//Handle, Publisher, Subscriber
	ros::NodeHandle nh;
	ros::Publisher pub_arbiter;
	ros::Publisher pub_peek;
	ros::Subscriber hokuyo_laser;
	
	//Average distances
	double hokuyo_right_avg;
	double hokuyo_center_avg;
	double hokuyo_left_avg;
	
	//FORWARD PID CONTROL VARIABLES
	double f_pid;
	double f_sum;
	double f_last_error;
	double f_error;
	
	//TURN RIGHT PID CONTROL VARIABLES
	double tr_pid;
	double tr_sum;
	double tr_last_error;
	double tr_error;
	
	//TURN LEFT PID CONTROL VARIABLES
	double tl_pid;
	double tl_sum;
	double tl_last_error;
	double tl_error;

};

#endif