#ifndef BEHAVIOR_FOLLOW_H
#define BEHAVIOR_FOLLOW_H

#include "ros/ros.h"
#include "control/behavior.h"
#include "sensor_msgs/LaserScan.h"

#define BH_DRIVE_RATE 70


class Behavior_Follow{
public:
	static constexpr double DRIVE_SPEED = .25f;
	static constexpr double DRIVE_TURN = .15f;
	static constexpr double MID_LASER_THRESHOLD = .9f;
	static constexpr double DELTA_THRESHOLD = .15f;

	Behavior_Follow();

	void center_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void process_behavior();
	
private:
	ros::NodeHandle nh;

	ros::Publisher pub_arbiter;

	ros::Subscriber left_side_laser;
	ros::Subscriber right_side_laser;

	double center_min;
	double center_max;
};

#endif