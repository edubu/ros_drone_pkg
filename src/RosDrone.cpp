#include "RosDrone.h"

RosDrone::RosDrone(ros::NodeHandle *nodehandle){
    ROS_INFO("Starting RosDrone class...");

    nh = *nodehandle;

    initializeMPU("mpupub", &nh);

    run();
}

void RosDrone::run(){
    ros::Rate loop_rate(cycle_rate);
    cycles = 0;

    // Start Spinning while ros is stable
    while(ros::ok()){

        mpu.sample();

        loop_rate.sleep();
        ros::spinOnce();
    }

    ROS_INFO("Cleaning up...");
    mpu.stop();
}

void RosDrone::initializeMPU(std::string pub_name, ros::NodeHandle *nodehandle){
    ROS_INFO("Deploying publishers and subscribers");
    // Create and deploy publisher
    mpu.init(pub_name, &nh);
    mpuDataTopic = pub_name;

    // Create and deploy subscriber
    mpu_sub = nh.subscribe(mpuDataTopic, 1, &RosDrone::mpu_subscriber_callback, this);
}

void RosDrone::mpu_subscriber_callback(const std_msgs::String::ConstPtr &msg){
    //ROS_INFO("[INFO] Heard from mpu publisher: [%s]", msg->data.c_str());
    std::stringstream ss(msg->data);

    std::string entry;
    int temp;
    ss >> entry;
    temp = stoi(entry);
    mpuD.ACCEL_X = temp;
    ss >> entry;
    temp = stoi(entry);
    mpuD.ACCEL_Y = temp;
    ss >> entry;
    temp = stoi(entry);
    mpuD.ACCEL_Z = temp;

    ss >> entry;
    temp = stoi(entry);
    mpuD.GYRO_X = temp;
    ss >> entry;
    temp = stoi(entry);
    mpuD.GYRO_Y = temp;
    ss >> entry;
    temp = stoi(entry);
    mpuD.GYRO_Z = temp;

    ss >> entry;
    temp = stoi(entry);
    mpuD.MAG_X = temp;
    ss >> entry;
    temp = stoi(entry);
    mpuD.MAG_Y = temp;
    ss >> entry;
    temp = stoi(entry);
    mpuD.MAG_Z = temp;

    printMPU();
}

void RosDrone::printMPU(){
    ROS_INFO("ACCEL_X: %6.2f; ACCEL_Y: %6.2f; ACCEL_Z: %6.2f; GYRO_X: %6.1f; GYRO_Y: %6.1f; GYRO_Z: %6.1f; MAG_X: %6.1f; MAG_Y: %6.1f; MAG_Z: %6.1f;",
            mpuD.ACCEL_X, mpuD.ACCEL_Y, mpuD.ACCEL_Z, mpuD.GYRO_X, mpuD.GYRO_Y, mpuD.GYRO_Z, mpuD.MAG_X, mpuD.MAG_Y, mpuD.MAG_Z);
}