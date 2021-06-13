#include "RosDrone.h"

RosDrone::RosDrone(ros::NodeHandle *nodeHandle){
    ROS_INFO("[INFO] Starting RosDrone class...");
    initializeSubscribers();
    initializePublishers();

    
}