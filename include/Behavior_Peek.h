#ifndef BEHAVIOR_PEEK_H
#define BEHAVIOR_PEEK_H

#include "ros/ros.h"
#include "final_project/behavior.h"
#include "final_project/trigger.h"
#include "sensor_msgs/LaserScan.h"

#define BH_PEEK_RATE 70


class Behavior_Peek{
public:
	

	Behavior_Peek();

	void cb_follow_trigger(const final_project::trigger::ConstPtr& msg);
	void process_behavior();
	
private:
	ros::NodeHandle nh;

	ros::Publisher pub_arbiter;

	ros::Publisher pub_passer;

	ros::Subscriber follower;

	int count;
	bool peeking;	
 
};

#endif