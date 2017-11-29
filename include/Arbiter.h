#ifndef FINAL_PROJ_ARBITER_H
#define FINAL_PROJ_ARBITER_H

#define BH_ARBITER_RATE 10

#include <vector>
#include <queue>
#include "ros/ros.h"
#include "final_project/behavior.h"
#include "geometry_msgs/Twist.h"

#define PRIORITY_DRIVE 0
#define PRIORITY_DRIVE_FASTER 0
#define PRIORITY_TURN 1
#define PRIORITY_FOLLOW 2
#define PRIORITY_PEEK 3
#define PRIORITY_PASS 4


//comparison function for priority queue
//function is outside of class because it was simpler
//in order to be inside of the class, there had to be workarounds
//that made it not worth the trouble for this simple use
bool compare_priorities(std::pair<int, final_project::behavior> a,
             std::pair<int, final_project::behavior> b) {
    return a.first < b.first;
}

class Arbiter {
  public:
    
    Arbiter();
    void process_behaviors();

  private:
    ros::NodeHandle nh;

    //Priority queue for hold behavior messages
    std::priority_queue<std::pair<int, final_project::behavior>,
            std::vector<std::pair<int, final_project::behavior>>,
            decltype(&compare_priorities)> behavior_queue{compare_priorities};

    //Subscribers to behaviors, one callback for every behavior that is added
    ros::Subscriber sub_bh_drive;
    ros::Subscriber sub_bh_drive_faster;
    ros::Subscriber sub_bh_follow;
    ros::Subscriber sub_bh_peek;
    ros::Subscriber sub_bh_turn;
    ros::Subscriber sub_bh_pass;

    //Publisher to cmd_vel to move the robot
    ros::Publisher pub_vel;

    //Behavior Callbacks
    void cb_bh_drive(const final_project::behavior::ConstPtr& msg);
    void cb_bh_drive_faster(const final_project::behavior::ConstPtr& msg);
    void cb_bh_follow(const final_project::behavior::ConstPtr& msg);
    void cb_bh_peek(const final_project::behavior::ConstPtr& msg);
    void cb_bh_turn(const final_project::behavior::ConstPtr& msg);
    void cb_bh_pass(const final_project::behavior::ConstPtr& msg);

    //Robot movement
    void move_robot(final_project::behavior& msg);
    void stop_robot();
};

#endif
