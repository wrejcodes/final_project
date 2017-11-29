#include "Behavior_Follow.h"

Behavior_Follow::Behavior_Follow(){
	//Set up initial stuff
	//Average distances
	hokuyo_right_avg = 0;
	hokuyo_center_avg = 0;
	hokuyo_left_avg = 0;
	
	//FORWARD PID CONTROL VARIABLES
	f_pid = 0.00f;
	f_sum = 0.00f;
	f_last_error = 0.00f;
	f_error = 0.00f;
	
	//TURN PID CONTROL VARIABLES
	t_pid = 0.00f;
	t_sum = 0.00f;
	t_last_error = 0.00f;
	t_error = 0.00f;

	// publisher
	pub_arbiter = nh.advertise<final_project::behavior>("behavior/follow", 1);
	pub_peek = nh.advertise<final_project::trigger>("trigger/follow", 1);
	
	// subscribe
	hokuyo_laser = nh.subscribe("/iRobot/hokuyo_scan", 1, &Behavior_Follow::hokuyo_callback, this);
}

//hokuyo_callback
void Behavior_Follow::hokuyo_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
	int start = 180 - FOLLOW_RANGE_SIZE/2;
	int end = 180 + FOLLOW_RANGE_SIZE/2;
	int offset = FOLLOW_RANGE_SIZE/3;

	double hokuyo_left_sum = 0;
	double hokuyo_center_sum = 0;
	double hokuyo_right_sum = 0;
	//ROS_INFO("start: %d, end: %d, offset %d", start, end, offset);
	for(int i = start; i < end; ++i){
		//right side
		if(i >= start && i < start + offset){
			//ROS_INFO("I'm here this time");
			hokuyo_right_sum += msg->ranges[i];
		}
		//center side
		else if(i >= start + offset && i < end - offset){
			//ROS_INFO("and now I'm doing center stuff yayyy");
		
			hokuyo_center_sum += msg->ranges[i];
		}
		//left side
		else{
			//ROS_INFO("this is super cool");
			hokuyo_left_sum += msg->ranges[i];
		}
	}
	
	//averages
	hokuyo_right_avg = hokuyo_right_sum / offset;
	hokuyo_center_avg = hokuyo_center_sum / offset;
	hokuyo_left_avg = hokuyo_left_sum / offset;
	//ROS_INFO("left avg: %.3f left sum: %.3f", hokuyo_left_avg, hokuyo_left_sum);
	//ROS_INFO("right avg: %.3f right sum: %.3f", hokuyo_right_avg, hokuyo_right_sum);


	//forward pid
	f_error = DESIRED_FOLLOW_DISTANCE - hokuyo_center_avg;
	f_sum += f_error; 
	f_pid = (F_P_GAIN * f_error) + (F_I_GAIN * ((1/BH_FOLLOW_RATE)*f_sum)) + (F_D_GAIN * ((f_error - f_last_error)/(1/BH_FOLLOW_RATE)));
	f_last_error = f_error;
	ROS_INFO("f_error: %.3f, f_pid: %.3f", f_error, f_pid);

	//turn pid
	t_error = hokuyo_right_avg - hokuyo_left_avg;
	t_sum += t_error; 
	t_pid = (T_P_GAIN * t_error) + (T_I_GAIN * ((1/BH_FOLLOW_RATE)*t_sum)) + (T_D_GAIN * ((t_error - t_last_error)/(1/BH_FOLLOW_RATE)));
	t_last_error = t_error;
	//ROS_INFO("t_error: %.3f, t_pid: %.3f", t_error, t_pid);

	

	//if no longer following reset all values.
	if(hokuyo_center_avg < DESIRED_FOLLOW_DISTANCE){
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
	
	//Trigger to begin behavior
	if(TRIGGER_FOLLOW_DISTANCE > hokuyo_center_avg){
		//Go Forward
		msg_move.vel_fw = f_pid < 0 ? FWD_VEL : FWD_VEL * f_pid;
		msg_move.vel_turn = 0;
		msg_move.active = true;
	}

	// TODO: when this is working use this message to tell the robot to peek
	//=====================================//
		// final_project::trigger trigger;
		// trigger.active = true;
	//====================================//
	
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