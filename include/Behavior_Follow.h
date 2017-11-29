#ifndef BEHAVIOR_FOLLOW_H
#define BEHAVIOR_FOLLOW_H

#include "ros/ros.h"
#include "final_project/behavior.h"
#include "sensor_msgs/LaserScan.h"
#include "final_project/trigger.h"

#define BH_FOLLOW_RATE 10.00f

class Behavior_Follow{
public:
	
	//static constexpr double DRIVE_TURN = .15f;
	//static constexpr double DELTA_THRESHOLD = .15f;
	
	//Follow Math Constants
	static constexpr double DESIRED_FOLLOW_DISTANCE = 1.0f;
	static constexpr double FWD_VEL = 0.27f;		
	static constexpr double TRN_VEL = 0.05f;
	static constexpr double TRN_THRESHOLD = 0.15f;

	//The boundaries for a front pointing ray.
	static constexpr int FOLLOW_RANGE_SIZE = 12; 

	static constexpr double F_GAIN_SWITCH = 0.05;
	static constexpr double T_GAIN_SWITCH = 0.03;


	//PID Constants
	static constexpr double F_P_GAIN = F_GAIN_SWITCH;
	static constexpr double F_I_GAIN = F_GAIN_SWITCH;
	static constexpr double F_D_GAIN = F_GAIN_SWITCH;
	static constexpr double T_P_GAIN = T_GAIN_SWITCH;
	static constexpr double T_I_GAIN = T_GAIN_SWITCH;
	static constexpr double T_D_GAIN = T_GAIN_SWITCH;
	
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
	double t_pid;
	double t_sum;
	double t_last_error;
	double t_error;
	

};

#endif