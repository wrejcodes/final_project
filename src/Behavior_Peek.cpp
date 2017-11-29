#include "Behavior_Peek.h"

Behavior_Peek::Behavior_Peek(){
	// set up initial stuff
	count = 0;

	// publisher
	//pub_passer = nh.advertise<final_project::trigger>("trigger/peek", 1);
	pub_arbiter = nh.advertise<final_project::behavior>("behavior/peek", 1);
	// subscribe

	//follower = nh.subscribe<final_project::trigger>("trigger/follow", 1, &Behavior_Peek::cb_follow_trigger, this);
	ROS_INFO("MERGE_LEFT %lf STABILIZE_MERGE_LEFT %lf DRIVE %lf MERGE_RIGHT %lf STABILIZE_MERGE_RIGHT %lf",MERGE_LEFT, STABILIZE_MERGE_LEFT,DRIVE,MERGE_RIGHT,STABILIZE_MERGE_RIGHT);
}

void Behavior_Peek::cb_follow_trigger(const final_project::trigger::ConstPtr& msg){

	if(msg->active && count == 0){
		count = COUNT_MAX;
	}

	// if(!msg->active){
	// 	count = 0;
	// }

	if(peek){
		// check for the on coming car;
	}
	
}

void Behavior_Peek::process_behavior(){
	final_project::behavior msg_move;
	msg_move.vel_fw = 0;
	msg_move.vel_turn = 0;
	msg_move.active = false;
	
	if(count > 0){
		ROS_INFO("count: %d",count);
		msg_move.active = true;
		msg_move.vel_fw = VEL_FW;
		if( count <= MERGE_LEFT && count > STABILIZE_MERGE_LEFT){
			// merge into the left lane

			msg_move.vel_turn = VEL_MERGE;
			pub_arbiter.publish(msg_move);
		} else if( count <= STABILIZE_MERGE_LEFT && count > DRIVE){
			
			// stablize into the left lane
			msg_move.vel_turn = -VEL_MERGE;
			pub_arbiter.publish(msg_move);
		} else if(count <= DRIVE && count > MERGE_RIGHT){ 
			// here is where we peek
			peek = true;
			msg_move.vel_turn = 0;
			pub_arbiter.publish(msg_move);
		} else if( count <= MERGE_RIGHT && count > STABILIZE_MERGE_RIGHT){
			// merge into the right lane
			msg_move.vel_turn = -VEL_MERGE;
			pub_arbiter.publish(msg_move);
		} else if(count <= STABILIZE_MERGE_RIGHT && count > 0){
			// stabilize the right merge
			msg_move.vel_turn = VEL_MERGE;
			pub_arbiter.publish(msg_move);
		}
		--count;
	}
	
	

}

int main(int argc, char **argv){
	ros::init(argc, argv, "fp_bh_peek");

	ROS_INFO("Peek node started...");


	Behavior_Peek peeker;
	ros::Rate loop_rate(BH_PEEK_RATE);

	scanf("%d", &(peeker.count));
	while(ros::ok()){
		peeker.process_behavior();
		ros::spinOnce();
		if(BH_PEEK_RATE != 0) {
			loop_rate.sleep();
		}
	}

	return 0;
}