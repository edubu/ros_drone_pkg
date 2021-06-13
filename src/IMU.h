#ifndef IMU_H_
#define IMU_H_

#include <rc/mpu.h>
#include <sstream>
#include <std_msgs/String.h>
#include <string>

class IMU {
    public:
        IMU(string topic_name, ros::NodeHandle *nodeHandle);

    private:

};

#endif