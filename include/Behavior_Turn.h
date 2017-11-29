#ifndef Behavior_Turn_H
#define Behavior_Turn_H

#include "ros/ros.h"
#include "final_project/behavior.h"
#include "sensor_msgs/LaserScan.h"

#define BH_TURN_RATE 10


class Behavior_Turn{
public:
	
	static constexpr double VEL_TURN = .5f;
	static constexpr double VEL_FW = .25f;
	static constexpr double THRESHOLD = .7f;

	Behavior_Turn();

	void right_front_laser_cb(const sensor_msgs::LaserScan::ConstPtr& msg);
	
	void process_behavior();
	
private:

	ros::NodeHandle nh;

	ros::Publisher pub_arbiter;

	ros::Subscriber right_front_scan;
 	
 	double avg;
};	

#endif
