#include "Behavior_Drive.h"

Behavior_Drive::Behavior_Drive(){
	// set up initial stuff
	left_front = 2.0f;
	left_mid = 2.0f;
	left_back = 2.0f;
	right_front = 2.0f;
	right_mid = 2.0f;
	right_back = 2.0f;
	// publisher
	pub_arbiter = nh.advertise<control::behavior>("behavior/drive", 1);
	// subscribe
	left_side_laser = nh.subscribe("left_side_scan", 1, &Behavior_Drive::left_side_laser_callback, this);
	right_side_laser = nh.subscribe("right_side_scan", 1, &Behavior_Drive::right_side_laser_callback, this);

}

void Behavior_Drive::left_side_laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
	left_back = msg->ranges[0];
	left_mid = msg->ranges[1];
	left_front = msg->ranges[2];
}

void Behavior_Drive::right_side_laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
	right_back = msg->ranges[2];
	right_mid = msg->ranges[1];
	right_front = msg->ranges[0];
}
void Behavior_Drive::process_behavior(){
	control::behavior msg_move;
	msg_move.vel_fw = 0;
	msg_move.vel_turn = 0;
	msg_move.active = false;
	
	if(right_mid <= MID_LASER_THRESHOLD){
		// follow right
		if(right_front > right_back + DELTA_THRESHOLD){
			msg_move.vel_turn = -DRIVE_TURN;
		} else if(right_back > right_front + DELTA_THRESHOLD){
			msg_move.vel_turn = DRIVE_TURN;
		}
		msg_move.vel_fw = DRIVE_SPEED;
		msg_move.active = true;
	} else if(left_mid <= MID_LASER_THRESHOLD){
		// follow left
		if(left_front > left_back + DELTA_THRESHOLD){
			msg_move.vel_turn = DRIVE_TURN;
		} else if(left_back > left_front + DELTA_THRESHOLD){
			msg_move.vel_turn = -DRIVE_TURN;
		}
		msg_move.vel_fw = DRIVE_SPEED;
		msg_move.active = true;
	}
	
	pub_arbiter.publish(msg_move);

}

int main(int argc, char **argv){
	ros::init(argc, argv, "fp_bh_drive");

	ROS_INFO("Drive behavior node started...");

	Behavior_Drive driver;
	ros::Rate loop_rate(BH_DRIVE_RATE);

	while(ros::ok()){
		driver.process_behavior();
		ros::spinOnce();
		if(BH_DRIVE_RATE != 0) {
			loop_rate.sleep();
		}
	}

	return 0;
}