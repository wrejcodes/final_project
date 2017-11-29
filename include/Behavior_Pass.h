#ifndef BEHAVIOR_PASS_H
#define BEHAVIOR_PASS_H

#include "ros/ros.h"
#include "final_project/behavior.h"
#include "final_project/trigger.h"
#include "sensor_msgs/LaserScan.h"

#define BH_PASS_RATE 10
#define COUNT_MAX 90

class Behavior_Pass{
public:
	
	static constexpr double VEL_TURN = .45f;
	static constexpr double VEL_FW = .25f;
	static constexpr double VEL_PASS = 1.25f;
	static constexpr double THRESHOLD = .7f;

	// conditions for states
	static constexpr double TURN_LEFT = 90;
	static constexpr double STABILIZE_LEFT = 70;
	static constexpr double PASS = 50;
	static constexpr double TURN_RIGHT = 40;
	static constexpr double STABILIZE_RIGHT = 20;

	Behavior_Pass();

	void right_side_laser_cb(const sensor_msgs::LaserScan::ConstPtr& msg);
	void trigger_cb(const final_project::trigger::ConstPtr& msg);
	void process_behavior();
	
private:

	ros::NodeHandle nh;

	ros::Publisher pub_arbiter;

	ros::Subscriber peeker;
	ros::Subscriber right_side_scan;
 	
 	double right_front;
 	double right_mid;
 	double right_back;

 	int count;
};	

#endif