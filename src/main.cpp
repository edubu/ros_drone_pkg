#include <string>
#include <ros/ros.h>
#include "RosDrone.h"

using namespace std;

int main(int argc, char ** argv){

    ROS_INFO("Creating drone...");

    ros::init(argc, argv, "drone");

    ros::NodeHandle nh;

    RosDrone drone(&nh);
    
    return 0;
}

