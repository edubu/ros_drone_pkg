#include <ros/ros.h>
#include <rc/mpu.h>
#include <rc/time.h>
#include <getopt.h>
#include <stdio.h>
#include <iostream>

/*
    Node that will calibrate specific components on the BBBl MPU

    Usages:

    roslaunch <package_name> <launch_file> params:="-arg1 -arg2"

    roslaunch ros_drone_pkg calibrate.launch params:="-m -g -a"

    available args:
        -m : recalibrates magnetometer
        -a : recalibrates accelerometer
        -g : recalibrates gyroscope
*/

#define I2C_BUS 2

int main(int argc, char **argv){

    // Initialize node
    ros::init(argc, argv, "calibrate_mpu");

    int options_index = 0, option = 0;

    // Input variables
    bool calGyro = false;
    bool calMag = false;
    bool calAccel = false;

    // Set variable formats
    struct option longOpts[] = {{"gyro", no_argument, nullptr, 'g'},
                                {"mag", no_argument, nullptr, 'm'},
                                {"accel", no_argument, nullptr, 'a'},
                                {nullptr, 0, nullptr, '\0'}};
    
    // Gather input parameters
    while((option = getopt_long(argc, argv, "gma", longOpts, &options_index)) != -1){
        switch(option){
            case 'g':
                calGyro = true;
                break;

            case 'm':
                calMag = true;
                break;

            case 'a':
                calAccel = true;
                break;
        }
    }

    // Check for no arguments
    if(!calGyro && !calMag && !calAccel){
        ROS_INFO("[ERROR] No sensors targeted for calibration\nAvailable args are: -m -g -a");
        return 0;
    }

    // Set to default configs
    rc_mpu_config_t config = rc_mpu_default_config();
    config.i2c_bus = I2C_BUS;

    // Calibrate set sensors
    if(calAccel){
        ROS_INFO("[INFO] Calibrating accelerometer...");
        if(rc_mpu_calibrate_accel_routine(config)<0){
                ROS_INFO("[ERROR] Failed to complete accelerometer calibration\n");
                return -1;
        }

        ROS_INFO("[INFO] Accelerometer successfully calibrated.");
    }

    if(calMag){
        ROS_INFO("[INFO] Calibrating magnetometer...");
        ROS_INFO("This will sample the magnetometer for the next 15 seconds\nRotate the board around in the air through as many orientations\nas possible to collect sufficient data for calibration\n");
        ROS_INFO("Press any key to start...\n");

        getchar();

        ROS_INFO("Start spinning...");
        rc_usleep(2000000);

        if(rc_mpu_calibrate_mag_routine(config)<0){
                ROS_INFO("[ERROR] Failed to complete magnetometer calibration\n");
                return -1;
        }

        ROS_INFO("[INFO] Magnetometer successfully calibrated.");
    }

    if(calGyro){
        ROS_INFO("[INFO] Starting gyroscope calibration, keep board very still...");

        if(rc_mpu_calibrate_gyro_routine(config)<0){
                ROS_INFO("[ERROR] Failed to complete gyroscope calibration\n");
                return -1;
        }

        ROS_INFO("[INFO] Gyroscope successfully calibrated.");
    }

    return 0;
}