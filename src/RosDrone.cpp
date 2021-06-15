#include "RosDrone.h"

RosDrone::RosDrone(ros::NodeHandle *nodehandle){
    ROS_INFO("[INFO] Starting RosDrone class...");

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

    // Clean up
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
    ROS_INFO("[INFO] ACCEL_X: %d; ACCEL_Y: %d; ACCEL_Z: %d; GYRO_X: %d; GYRO_Y: %d; GYRO_Z: %d; MAG_X: %d; MAG_Y: %d; MAG_Z: %d;",
            mpuD.ACCEL_X, mpuD.ACCEL_Y, mpuD.ACCEL_Z, mpuD.GYRO_X, mpuD.GYRO_Y, mpuD.GYRO_Z, mpuD.MAG_X, mpuD.MAG_Y, mpuD.MAG_Z);

}