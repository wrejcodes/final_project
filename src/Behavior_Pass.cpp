#include "Behavior_Pass.h"

Behavior_Pass::Behavior_Pass(){
	// set up initial stuff

	// publisher
	pub_arbiter = nh.advertise<final_project::behavior>("behavior/pass", 1);
	
	// subscribe
	//peeker = nh.subscribe<final_project::trigger>("trigger/peek",1, &Behavior_Pass::trigger_cb, this);
	//right_side_scan = nh.subscribe<sensor_msgs::LaserScan>("right_side_scan", 1, &Behavior_Pass::right_side_laser_cb, this);
	
}

void Behavior_Pass::right_side_laser_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
	right_back = msg->ranges[2];
	right_mid = msg->ranges[1];
	right_front = msg->ranges[0];
}

void Behavior_Pass::trigger_cb(const final_project::trigger::ConstPtr &msg){
	
	if(msg->active && count == 0){
		ROS_INFO("We're here");
		count = COUNT_MAX;
	}

	if(!msg->active){
		count = 0;
	}
}



void Behavior_Pass::process_behavior(){
	final_project::behavior msg_move;
	msg_move.vel_fw = 0;
	msg_move.vel_turn = 0;
	msg_move.active = false;

	ROS_INFO("i'm here");	

	if(count > 0){
		msg_move.active = true;
		if( count <= TURN_LEFT && count > STABILIZE_LEFT){
			msg_move.vel_turn = VEL_TURN;
			msg_move.vel_fw = VEL_FW;
		} else if(count <= STABILIZE_LEFT && count > PASS){
			msg_move.vel_fw = VEL_FW;
			msg_move.vel_turn = -VEL_TURN;
		} else if(count <= PASS && count > TURN_RIGHT){
			if(right_back > THRESHOLD){
				count = TURN_RIGHT;
				msg_move.vel_fw = VEL_FW;
				msg_move.vel_turn = -VEL_TURN;
			}else{
				msg_move.vel_fw = VEL_PASS;
			}
		} else if (count <= TURN_RIGHT && count > STABILIZE_RIGHT){
			msg_move.vel_fw = VEL_FW;
			msg_move.vel_turn = -VEL_TURN;
		} else if(count <= STABILIZE_RIGHT && count > 0){
			msg_move.vel_fw = VEL_FW;
			msg_move.vel_turn = VEL_TURN;
		}
		count--;
	}
	pub_arbiter.publish(msg_move);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "fp_bh_pass");

	ROS_INFO("Pass node started...");


	Behavior_Pass passer;
	ros::Rate loop_rate(BH_PASS_RATE);

	while(ros::ok()){
		passer.process_behavior();
		ros::spinOnce();
		if(BH_PASS_RATE != 0) {
			loop_rate.sleep();
		}
	}

	return 0;
}