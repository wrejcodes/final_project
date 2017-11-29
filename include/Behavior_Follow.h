#ifndef BEHAVIOR_FOLLOW_H
#define BEHAVIOR_FOLLOW_H

#include "ros/ros.h"
#include "final_project/behavior.h"
#include "sensor_msgs/LaserScan.h"

#define BH_FOLLOW_RATE 70

class Behavior_Follow{
public:
	
	//static constexpr double DRIVE_TURN = .15f;
	//static constexpr double DELTA_THRESHOLD = .15f;
	
	//Follow Math Constants
	static constexpr double DESIRED_FOLLOW_DISTANCE = 1.00f;
	static constexpr double FWD_VEL = 0.25f;		
	static constexpr double TRN_VEL = 0.25f;
	static constexpr double TRN_THRESHOLD = 0.15f;

	//The boundaries for a front pointing ray.
	static constexpr int FOLLOW_RANGE_SIZE = 15; 

	//PID Constants
	double F_P_GAIN = 1;
	double F_I_GAIN = 1;
	double F_D_GAIN = 1;
	double T_P_GAIN = 1;
	double T_I_GAIN = 1;
	double T_D_GAIN = 1;

	//Don't start detecting until
	static constexpr double TRIGGER_FOLLOW_DISTANCE = 4.00f;

	Behavior_Follow();

	void hokuyo_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void process_behavior();
	
private:
	//Handle, Publisher, Subscriber
	ros::NodeHandle nh;
	ros::Publisher pub_arbiter;
	ros::Subscriber hokuyo_scan;
	
	//Average distances
	double hokuyo_right_avg;
	double hokuyo_center_avg;
	double hokuyo_left_avg;
	
	//FORWARD PID CONTROL VARIABLES
	double f_pid;
	double f_sum;
	double f_last_error;
	double f_error;
	
	//TURN PID CONTROL VARIABLES
	double t_pid;
	double t_sum;
	double t_last_error;
	double t_error;
	
};

#endif