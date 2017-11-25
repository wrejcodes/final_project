#ifndef FINAL_PROJ_ARBITER_H
#define FINAL_PROJ_ARBITER_H

#define BH_ARBITER_RATE 10

#include <vector>
#include <queue>
#include "ros/ros.h"
#include "control/behavior.h"
#include "geometry_msgs/Twist.h"

#define PRIORITY_DRIVE 0


//comparison function for priority queue
//function is outside of class because it was simpler
//in order to be inside of the class, there had to be workarounds
//that made it not worth the trouble for this simple use
bool compare_priorities(std::pair<int, control::behavior> a,
             std::pair<int, control::behavior> b) {
    return a.first < b.first;
}

class Arbiter {
  public:
    
    Arbiter();
    void process_behaviors();

  private:
    ros::NodeHandle nh;

    //Priority queue for hold behavior messages
    std::priority_queue<std::pair<int, control::behavior>,
            std::vector<std::pair<int, control::behavior>>,
            decltype(&compare_priorities)> behavior_queue{compare_priorities};

    //Subscribers to behaviors, one callback for every behavior that is added
    ros::Subscriber sub_bh_drive;
    
    //Publisher to cmd_vel to move the robot
    ros::Publisher pub_vel;

    //Behavior Callbacks
    void cb_bh_drive(const control::behavior::ConstPtr& msg);
   
    //Robot movement
    void move_robot(control::behavior& msg);
    void stop_robot();
};

#endif