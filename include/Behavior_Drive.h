#ifndef BEHAVIOR_DRIVE_H
#define BEHAVIOR_DRIVE_H

#include "ros/ros.h"
#include "final_project/behavior.h"
#include "sensor_msgs/LaserScan.h"

#define BH_DRIVE_RATE 70


class Behavior_Drive{
public:
	static constexpr double DRIVE_SPEED = .25f;
	static constexpr double DRIVE_TURN = .25f;
	static constexpr double MID_LASER_THRESHOLD = 1.0f;
	static constexpr double DELTA_THRESHOLD = .3f;

	Behavior_Drive();

	void left_side_laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void right_side_laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void process_behavior();
	
private:
	ros::NodeHandle nh;

	ros::Publisher pub_arbiter;

	ros::Subscriber left_side_laser;
	ros::Subscriber right_side_laser;

	double left_front;
	double left_mid;
	double left_back; 
	double right_front;
	double right_mid;
	double right_back; 
};

#endif