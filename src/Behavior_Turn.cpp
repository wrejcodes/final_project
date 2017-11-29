#include "Behavior_Turn.h"

Behavior_Turn::Behavior_Turn(){
	// set up initial stuff

	// publisher
	pub_arbiter = nh.advertise<final_project::behavior>("behavior/turn", 1);
	// subscribe
	right_front_scan = nh.subscribe<sensor_msgs::LaserScan>("right_front_scan", 1, &Behavior_Turn::right_front_laser_cb, this);
	
}

void Behavior_Turn::right_front_laser_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
	double sum = 0;
	for(int i = 0; i < msg->ranges.size(); i++){
		sum += msg->ranges[i];
	}
	avg = sum / msg->ranges.size();
}



void Behavior_Turn::process_behavior(){
	final_project::behavior msg_move;
	msg_move.vel_fw = 0;
	msg_move.vel_turn = 0;
	msg_move.active = false;

	
	if(avg <= THRESHOLD){

		msg_move.active = true;
		msg_move.vel_turn = VEL_TURN;
		msg_move.vel_fw = VEL_FW;
	}
	
	
	pub_arbiter.publish(msg_move);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "fp_bh_turn");

	ROS_INFO("Turn node started...");


	Behavior_Turn tina_turner;
	ros::Rate loop_rate(BH_TURN_RATE);

	while(ros::ok()){
		tina_turner.process_behavior();
		ros::spinOnce();
		if(BH_TURN_RATE != 0) {
			loop_rate.sleep();
		}
	}

	return 0;
}