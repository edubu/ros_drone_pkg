#ifndef ROS_DRONE_H_
#define ROS_DRONE_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rc/mpu.h>
#include <sstream>
#include <math.h>
#include <string>
#include <vector>
#include <stdlib.h>
#include "BB_MPU.h"

#define I2C_BUS 2

struct mpuData {
  double ACCEL_X;
  double ACCEL_Y;
  double ACCEL_Z;

  double GYRO_X;
  double GYRO_Y;
  double GYRO_Z;

  double MAG_X;
  double MAG_Y;
  double MAG_Z;
};

class RosDrone {

    public:
        // Constructor handles initialization of publishers and subscribers
        RosDrone(ros::NodeHandle *nodehandle);

    private:
        // Helper classes
        BB_MPU mpu;

        // Ros subscribers and publisher definitions
        ros::NodeHandle nh;
        ros::Subscriber mpu_sub;
        int cycle_rate = 10;
        int cycles;

        // Publishers
        std::string mpuDataTopic;

        // Helper structs
        mpuData mpuD;

        // Continuously runs while ros is stable
        void run();

        void initializeMPU(std::string pub_name, ros::NodeHandle *nodehandle);  

        //PRINTS
        void printMPU();

        // Parses and prints out data received by imu_publisher
        void mpu_subscriber_callback(const std_msgs::String::ConstPtr& msg);

};


#endif