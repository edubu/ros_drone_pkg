#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include "rc/mpu.h" 

#define I2C_BUS 2
/* include the library that will allow imu reading */


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

int rc_mpu_initialize( rc_mpu_data* data, rc_mpu_config conf) 
/* set up the imu in normal one shot sampling mode, this will allow you to read sensor data at any time  */

int rc_mpu_read_accel(rc_mpu_data* data);
int rc_mpu_read_gyro(rc_mpu_data* data);
int rc_mpu_read_temp(rc_mpu_data* data);
int rc_mpu_read_mag(rc_mpu_data* data);
/*read the imu data and store it in the data struct, NEED TO CREATE A DATA STRUCT */

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

static int enable_magnetometer = 0;
static int enable_thermometer = 0;
static int enable_warnings = 0;
static int running = 0;


// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
        running=0;
        return;
}

int main(int argc, char **argv)
{

  rc_mpu_data_t data; //struct to hold new data
  int c;
  g_mode_t g_mode = G_MODE_DEG; // gyro default to degree mode.
  a_mode_t a_mode = A_MODE_MS2; // accel default to m/s^2

  g_mode = G_MODE_DEG;
  a_mode = A_MODE_MS2;

  // set signal handler so the loop can exit cleanly
  signal(SIGINT, __signal_handler);
  running = 1;
  // use defaults for now, except also enable magnetometer.
  rc_mpu_config_t conf = rc_mpu_default_config();
  conf.i2c_bus = I2C_BUS;
  conf.enable_magnetometer = enable_magnetometer;
  conf.show_warnings = enable_warnings;

  if(rc_mpu_initialize(&data, conf)){
    fprintf(stderr,"rc_mpu_initialize_failed\n");
    return -1;
  }


   
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  

  
  int count = 0;
  while (ros::ok())
  {
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
   
    
    std_msgs::String msg;

    std::stringstream ss;
    /*ss << "hello world " << count;*/ 
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

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  

  return 0;
}

int rc_mpu_power_off(void)
  /*rc_mpu_power_off() powers off the imu at the end of the reading, need to include this if you use initialize*/ 

