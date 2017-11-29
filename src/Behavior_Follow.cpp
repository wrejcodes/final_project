#include "Behavior_Follow.h"

Behavior_Follow::Behavior_Follow(){
	//Set up initial stuff
	//Average distances
	hokuyo_right_avg = 0;
	hokuyo_center_avg = 0;
	hokuyo_left_avg = 0;
	
	//FORWARD PID CONTROL VARIABLES
	f_pid = 0;
	f_sum = 0;
	f_last_error = 0;
	f_error = 0;
	
	//TURN PID CONTROL VARIABLES
	t_pid = 0;
	t_sum = 0;
	t_last_error = 0;
	t_error = 0;

	// publisher
	pub_arbiter = nh.advertise<final_project::behavior>("behavior/follow", 1);
	
	// subscribe
	hokuyo_laser = nh.subscribe("hokuyo_scan", 1, &Behavior_Follow::hokuyo_center_callback, this);
}

//hokuyo_callback
void Behavior_Drive::hokuyo_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
	int start = 180 + FOLLOW_RANGE_SIZE/2;
	int end = 180 - FOLLOW_RANGE_SIZE/2;
	int offset = FOLLOW_RANGE_SIZE/3;

	double hokuyo_left_sum = 0;
	double hokuyo_center_sum = 0;
	double hokuyo_right_sum = 0;
		
	for(int i = start; i < end; ++i){
		//right side
		if(i >= start && i < start + offset){
			hokuyo_right_sum += msg->ranges[i];
		}
		//center side
		else if(i >= start + offset && i < end - offset){
			hokuyo_center_sum += msg->ranges[i];
		}
		//left side
		else{
			hokuyo_left_sum += msg->ranges[i];
		}
	}
	
	//averages
	hokuyo_right_avg = hokuyo_right_sum / FOLLOW_RANGE_SIZE;
	hokuyo_center_avg = hokuyo_center_sum / FOLLOW_RANGE_SIZE;
	hokuyo_left_avg = hokuyo_left_sum / FOLLOW_RANGE_SIZE;

	//forward pid
	f_error = DESIRED_FOLLOW_DISTANCE - hokuyo_center_avg;
	f_sum += f_error; 
	f_pid = (F_P_GAIN * f_error) + (F_I_GAIN * ((1/loop_rate)*f_sum)) + (F_D_GAIN * ((f_error - f_last_error)/(1/loop_rate)))
	f_last_error = f_error;

	//turn pid	


	if(hokuyo_center_avg > DESIRED_FOLLOW_DISTANCE){
		f_error = 0;
		f_last_error = 0;
		f_sum = 0;
		
		t_error = 0;
		t_last_error = 0;
		t_sum = 0;
	}
}

void Behavior_Follow::process_behavior(){
	final_project::behavior msg_move;
	msg_move.vel_fw = 0;
	msg_move.vel_turn = 0;
	msg_move.active = false;
	
	//Trigger to begin follow forward
	if(TRIGGER_FOLLOW_DISTANCE > hokuyo_center_avg_range){
		msg_move.vel_fw = f_pid;
		msg_move.vel_turn = 0;
		msg_move.active = true;

		//Trigger to turn left
		if(hokuyo_right_avg > hokuyo_left_avg){
			msg_move.vel_fw = f_pid;
			msg_move.vel_turn = t_pid;
			msg_move.active = true;
		}

		//Trigger to turn right
		if(hokuyo_left_avg > hokuyo_right_avg){
			msg_move_vel_fw = f_pid;
			msg_move.vel_turn = -t_pid;
			msg_move.active = true;
		}
	}	

	pub_arbiter.publish(msg_move);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "fp_bh_follow");

	ROS_INFO("Follow behavior node started...");

	Behavior_Follow following;
	ros::Rate loop_rate(BH_FOLLOW_RATE);

	while(ros::ok()){
		following.process_behavior();
		ros::spinOnce();
		if(BH_FOLLOW_RATE != 0) {
			loop_rate.sleep();
		}
	}

	return 0;
}