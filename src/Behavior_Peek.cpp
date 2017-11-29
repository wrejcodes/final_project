#include "Behavior_Peek.h"

Behavior_Peek::Behavior_Peek(){
	// set up initial stuff
	count = 0;

	// publisher
	//pub_passer = nh.advertise<final_project::trigger>("trigger/peek", 1);
	pub_arbiter = nh.advertise<final_project::behavior>("behavior/peek", 1);
	// subscribe

	follower = nh.subscribe<final_project::trigger>("trigger/follow", 1, &Behavior_Peek::cb_follow_trigger, this);
}

void Behavior_Peek::cb_follow_trigger(const final_project::trigger::ConstPtr& msg){

	if(msg->active && count == 0){
		count = 50;
	}
	
}

void Behavior_Peek::process_behavior(){
	final_project::behavior msg_move;
	msg_move.vel_fw = 0;
	msg_move.vel_turn = 0;
	msg_move.active = false;
	

	
	
	
	pub_arbiter.publish(msg_move);

}

int main(int argc, char **argv){
	ros::init(argc, argv, "fp_bh_peek");

	ROS_INFO("Peek node started...");

	Behavior_Peek peeker;
	ros::Rate loop_rate(BH_PEEK_RATE);

	while(ros::ok()){
		peeker.process_behavior();
		ros::spinOnce();
		if(BH_PEEK_RATE != 0) {
			loop_rate.sleep();
		}
	}

	return 0;
}