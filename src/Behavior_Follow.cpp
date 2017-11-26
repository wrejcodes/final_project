#include "Behavior_Follow.h"

Behavior_Follow::Behavior_Follow(){
	// set up initial stuff
	center_min = 1.5f;
	center_max = 3.0f
	// publisher
	pub_arbiter = nh.advertise<final_project::behavior>("behavior/follow", 1);
	// subscribe
	center_hokuyo_laser = nh.subscribe("left_side_scan", 1, &Behavior_Drive::center_scan_callback, this);
	
}

void Behavior_Drive::center_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
	left_back = msg->ranges[0];
	left_mid = msg->ranges[1];
	left_front = msg->ranges[2];
}

void Behavior_Drive::process_behavior(){
	final_project::behavior msg_move;
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