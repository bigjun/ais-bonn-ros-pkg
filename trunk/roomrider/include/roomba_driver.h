/*
 * roomba_driver.h
 *
 * modified by Andreas Hochrath & Torsten Fiolka in August 2010
 *
 * at           University Bonn,
 *              Institute for Computer Science VI,
 *              Autonomous Intelligent Systems
 *
 * contact: fiolka@cs.uni-bonn.de, hochrath@cs.uni-bonn.de
 *
 * based on driver.cpp from the  irobot_create_rustic package
 *
 * original header follows:
 */

/*
 *  driver.cpp
 *
 *
 *  Created by Gheric Speiginer and Keenan Black on 6/15/09.
 *
 *  Modified by Michael Otte University of Colorado at Boulder 2010 for irobot_create_rustic
 *
 */


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include "roomba_comms.h"
#include <roomrider/Power.h>
#include <roomrider/Lightsensor.h>
#include <roomrider/Bumper.h>
#include <roomrider/Wheeldrop.h>
#include <roomrider/Infrared.h>
#include <roomrider/Led.h>

class IRobotRoombaDriver
{
public:

  IRobotRoombaDriver();
  void mainDeviceLoop();
  void shutdown();
  int wrapAround16(unsigned short current, unsigned short last);
  void cmdVelocityCallback(const geometry_msgs::TwistConstPtr& cmd_msg);
  void ledCallback(const roomrider::LedConstPtr& led_msg);
  void handleLeds();
  void publishOdometry();
  void publishBumper();
  void publishWheeldrop();
  void publishPower();
  void publishLightsensor();
  void publishInfrared();

protected:
  roomba500_comm_t* roomba_dev_; // the underlying roomba object
  std::string serial_port_; // serial port where the roomba is connected
  bool safe_; // full control or not
  bool debug_mode_; // in debug mode all sensor values are printed out

  // last read wheel encoder counts for comparison
  unsigned short encoder_counts_left_last_, encoder_counts_right_last_;
  // last calculated x- and y-position for comparison
  double pos_X_old_, pos_Y_old_;
  // two timestamps for calculating
  ros::Time current_time_, last_time_;

  // publisher for the sensors od the roomba
  ros::Publisher odom_pub_;
  ros::Publisher bumper_pub_;
  ros::Publisher power_pub_;
  ros::Publisher lightsensor_pub_;
  ros::Publisher wheeldrop_pub_;
  ros::Publisher infrared_pub_;

  // subscriber for movement and leds
  ros::Subscriber cmd_sub_;
  ros::Subscriber led_sub_;

  // transformation between odometry and roomba
  tf::TransformBroadcaster odom_broadcaster_;

  ros::NodeHandle node_handle_;

  // stores status and commands for the leds
  roomrider::Led led_;
};
