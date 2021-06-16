#ifndef BB_MPU_H_
#define BB_MPU_H_

#include <rc/mpu.h>
#include <rc/time.h>
#include <sstream>
#include <std_msgs/String.h>
#include <string>

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

class BB_MPU {
    public:

        // Publishes IMU data to <topic_name>
        BB_MPU() {
            g_mode = G_MODE_DEG; // gyro default to degree mode.
            a_mode = A_MODE_MS2; // accel default to m/s^2

            // set signal handler so the loop can exit cleanly
            running = 1;

            // use defaults for now, except also enable magnetometer.
            conf = rc_mpu_default_config();
            conf.i2c_bus = I2C_BUS;
            conf.enable_magnetometer = enable_magnetometer;
            conf.show_warnings = enable_warnings;
        }

        void init(std::string topic_name, ros::NodeHandle *nodehandle){
            ROS_INFO("Initializing mpu publisher...");

            if(rc_mpu_initialize(&data, conf)){
                ROS_INFO("rc_mpu_initialize_failed\n");
                fprintf(stderr,"rc_mpu_initialize_failed\n");
                return;
            }

            this->topic_name = topic_name;
            nh = *nodehandle;
            mpu_pub = nh.advertise<std_msgs::String>(topic_name, 1);

        }

        // Called every loop on main -- publishes to /topic_name
        void sample(){
            if(running){
                // read sensor data
                if(rc_mpu_read_accel(&data)<0){
                        printf("read accel data failed\n");
                }
                if(rc_mpu_read_gyro(&data)<0){
                        printf("read gyro data failed\n");
                }
                if(enable_magnetometer && rc_mpu_read_mag(&data)){
                        printf("read mag data failed\n");
                }
                if(enable_thermometer && rc_mpu_read_temp(&data)){
                        printf("read imu thermometer failed\n");
                }
            }
        
            // Clear current message
            ss.str(std::string());

            ss.precision(2);
            ss << data.accel[0]; // Accel_X
            ss << " " << data.accel[1]; // Accel_Y
            ss << " " << data.accel[2]; // Accel_Z
            
            ss << " " << data.gyro[0]; // X
            ss << " " << data.gyro[1]; // Y
            ss << " " << data.gyro[2]; // Z
            
            ss << " " << data.mag[0]; // X
            ss << " " << data.mag[1]; // Y
            ss << " " << data.mag[2]; // Z
            
            ss << " " << data.temp; 
            
            msg.data = ss.str();

            mpu_pub.publish(msg);

        }

        void stop(){
            running = 0;
        }

    private:
        // enables
        int enable_magnetometer = 1;
        int enable_thermometer = 1;
        int enable_warnings = 1;
        int running = 0;

        // Information
        std::string topic_name;
        std_msgs::String msg;
        std::stringstream ss;

        // rc/mpu.h variables
        rc_mpu_data_t data; //struct to hold new data
        rc_mpu_config_t conf;
        int c;
        g_mode_t g_mode;
        a_mode_t a_mode;

        // ROS specific variables
        ros::Publisher mpu_pub;
        ros::NodeHandle nh;
};

#endif