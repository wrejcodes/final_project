#ifndef BEHAVIOR_PEEK_H
#define BEHAVIOR_PEEK_H

#include "ros/ros.h"
#include "final_project/behavior.h"
#include "final_project/trigger.h"
#include "sensor_msgs/LaserScan.h"

#define BH_PEEK_RATE 10
#define COUNT_MAX 100


class Behavior_Peek{
public:
	
	int count;
	static constexpr double VEL_MERGE = .43f;
	static constexpr double VEL_FW = .25f;
	static constexpr double MERGE_LEFT = COUNT_MAX;
	static constexpr double STABILIZE_MERGE_LEFT = MERGE_LEFT - 20;
	static constexpr double DRIVE = STABILIZE_MERGE_LEFT - 20;
	static constexpr double MERGE_RIGHT = DRIVE -20;
	static constexpr double STABILIZE_MERGE_RIGHT = MERGE_RIGHT - 20;

	static constexpr double PEEK_THRESHOLD = 5.0f;

	Behavior_Peek();

	void cb_follow_trigger(const final_project::trigger::ConstPtr& msg);
	void lidar_cb(const sensor_msgs::LaserScan::ConstPtr &msg);
	void process_behavior();
	
private:
	ros::NodeHandle nh;

	ros::Publisher pub_arbiter;

	ros::Publisher pub_passer;

	ros::Subscriber follower;

	ros::Subscriber lidar;

	double avg;
	
 
};

#endif