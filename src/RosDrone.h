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

#define I2C_BUS 2

// IMU Variables
typedef enum g_mode_t{
        G_MODE_RAD,
        G_MODE_DEG,
        G_MODE_RAW
} g_mode_t;

typedef enum a_mode_t{
        A_MODE_MS2,
        A_MODE_G,
        A_MODE_RAW
} a_mode_t;

class RosDrone {

    public:
        // Constructor handles initialization of publishers and subscribers
        RosDrone(ros::NodeHandle* nodehandle);


    private:
        // Ros subscribers and publisher definitions
        ros::NodeHandle nh;
        
        ros::Subscriber minimal_subscriber;
        ros::Publisher minimal_publisher;

        void initializeSubscribers();
        void initializePublishers();    

        // Parses and prints out data received by imu_publisher
        void imu_subscriber_callback(const std_msgs::String::ConstPtr& msg);

};


#endif