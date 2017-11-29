#ifndef BEHAVIOR_DRIVE_FASTER_H
#define BEHAVIOR_DRIVE_FASTER_H

#include "ros/ros.h"
#include "final_project/behavior.h"
#include "sensor_msgs/LaserScan.h"

#define BH_DRIVE_FASTER_RATE 70


class Behavior_Drive_Faster{
public:
	static constexpr double DRIVE_SPEED = .5f;
	static constexpr double DRIVE_TURN2 = .25f;
	static constexpr double DRIVE_TURN = .15f;
	static constexpr double MID_LASER_THRESHOLD = 0.30f;
	static constexpr double DELTA_THRESHOLD = .05f;


	Behavior_Drive_Faster();

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
