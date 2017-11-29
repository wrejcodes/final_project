#include "Arbiter.h"

Arbiter::Arbiter() {
    //Set subscriptions & advertisement
    this->sub_bh_drive = this->nh.subscribe("behavior/drive", 3, &Arbiter::cb_bh_drive, this);
    this->sub_bh_follow = this->nh.subscribe("behavior/follow", 3, &Arbiter::cb_bh_follow, this);

    this->pub_vel = this->nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

void Arbiter::process_behaviors() {
    if (!this->behavior_queue.empty()) {
        final_project::behavior priority_msg = this->behavior_queue.top().second;
        move_robot(priority_msg);
        while(this->behavior_queue.size() > 0) {
            //C++ Priority Queue has no clear function,
            //so we loop to pop off the old messages
            this->behavior_queue.pop();
        }
    }
    else {   
        stop_robot();
    }
}


void Arbiter::cb_bh_drive(const final_project::behavior::ConstPtr &msg) {
    if (msg->active) {
        this->behavior_queue.push(std::pair<int, final_project::behavior>(PRIORITY_DRIVE, *msg));
    }
    ROS_DEBUG("Arbiter: Drive(%s) Fw: %.1f Turn: %.1f", msg->active ? "on" : "off", msg->vel_fw, msg->vel_turn);
}

void Arbiter::cb_bh_follow(const final_project::behavior::ConstPtr &msg) {
    if (msg->active) {
        this->behavior_queue.push(std::pair<int, final_project::behavior>(PRIORITY_FOLLOW, *msg));
    }
    ROS_DEBUG("Arbiter: Follow(%s) Fw: %.1f Turn: %.1f", msg->active ? "on" : "off", msg->vel_fw, msg->vel_turn);
}


void Arbiter::move_robot(final_project::behavior& msg) {
    geometry_msgs::Twist msg_move;
    msg_move.linear.x = msg.vel_fw;
    msg_move.angular.z = msg.vel_turn;
    this->pub_vel.publish(msg_move);
}

void Arbiter::stop_robot() {
    geometry_msgs::Twist msg_move;
    msg_move.linear.x = 0;
    msg_move.angular.z = 0;
    this->pub_vel.publish(msg_move);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fp_bh_arbiter");
    ros::NodeHandle nh;
    ROS_INFO("Arbiter node started...");
    ros::Rate loop_rate(BH_ARBITER_RATE);

    //Create the arbiter
    Arbiter arbiter;

    while (ros::ok()) {
        arbiter.process_behaviors();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
