#include "Behavior_Peek.h"

Behavior_Peek::Behavior_Peek(){
	// set up initial stuff
	count = COUNT_MAX + 30;

	// publisher
	pub_passer = nh.advertise<final_project::trigger>("trigger/peek", 1);
	pub_arbiter = nh.advertise<final_project::behavior>("behavior/peek", 1);
	// subscribe
	lidar = nh.subscribe<sensor_msgs::LaserScan>("/iRobot/hokuyo_scan", 1, &Behavior_Peek::lidar_cb, this);
	//follower = nh.subscribe<final_project::trigger>("trigger/follow", 1, &Behavior_Peek::cb_follow_trigger, this);
	ROS_INFO("MERGE_LEFT %lf STABILIZE_MERGE_LEFT %lf DRIVE %lf MERGE_RIGHT %lf STABILIZE_MERGE_RIGHT %lf",MERGE_LEFT, STABILIZE_MERGE_LEFT,DRIVE,MERGE_RIGHT,STABILIZE_MERGE_RIGHT);
}

void Behavior_Peek::cb_follow_trigger(const final_project::trigger::ConstPtr& msg){

	if(true && count == 0){
		count = COUNT_MAX + 20;
	}

	// if(!msg->active){
	// 	count = 0;
	// }
}

void Behavior_Peek::lidar_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
	double sum = 0;
	for(int i = 180; i < 187; i++){
		sum += msg->ranges[i];
	}
	avg = sum / 8;
}

void Behavior_Peek::process_behavior(){
	final_project::behavior msg_move;
	msg_move.vel_fw = 0;
	msg_move.vel_turn = 0;
	msg_move.active = false;
	
	if(count > 0){
		//ROS_INFO("count: %d",count);
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
			//ROS_INFO("avg: %.3lf", avg);
			if( avg >= PEEK_THRESHOLD){
				final_project::trigger trigger_msg;
				trigger_msg.active = true;
				pub_passer.publish(trigger_msg);
			}
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

		final_project::trigger trigger_msg;
				trigger_msg.active = true;
				pub_passer.publish(trigger_msg);
		--count;
	}
	
	

}

int main(int argc, char **argv){
	ros::init(argc, argv, "fp_bh_peek");

	ROS_INFO("Peek node started...");


	Behavior_Peek peeker;
	ros::Rate loop_rate(BH_PEEK_RATE);

	//scanf("%d", &(peeker.count));
	while(ros::ok()){
		peeker.process_behavior();
		ros::spinOnce();
		if(BH_PEEK_RATE != 0) {
			loop_rate.sleep();
		}
	}

	return 0;
}