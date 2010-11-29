/*
 * roomrider_driver.h
 *
 *
 * Copyright (C) 2010
 * Autonomous Intelligent Systems Group
 * University of Bonn, Germany
 *
 *
 * Authors: Andreas Hochrath, Torsten Fiolka
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *
 * Origin:
 *
 *  driver.cpp
 *
 *
 *  Created by Gheric Speiginer and Keenan Black on 6/15/09.
 *
 *  Modified by Michael Otte University of Colorado at Boulder 2010 for irobot_create_rustic
 *
 *
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include "roomba_comms.h"
#include <roomrider_driver/Power.h>
#include <roomrider_driver/Lightsensor.h>
#include <roomrider_driver/Bumper.h>
#include <roomrider_driver/Wheeldrop.h>
#include <roomrider_driver/Infrared.h>
#include <roomrider_driver/Led.h>

/**
 * \class RoomriderDriver
 * @brief Controls a iRobot Roomba using roombaComms, accepts commands and publishes
 * data to a ROS system
 */
class RoomriderDriver
{
public:

  RoomriderDriver();
  void mainDeviceLoop();
  void shutdown();

  int wrapAround16(unsigned short current, unsigned short last);
  void cmdVelocityCallback(const geometry_msgs::TwistConstPtr& cmd_msg);
  void ledCallback(const roomrider_driver::LedConstPtr& led_msg);

  void handleLeds();
  void publishOdometry();
  void publishBumper();
  void publishWheeldrop();
  void publishPower();
  void publishLightsensor();
  void publishInfrared();

protected:

  //! the underlying roomba object
  roomba500_comm_t* roomba_dev_;
  std::string serial_port_;

  //! full control or not
  bool safe_;

  //! in verbose mode all sensor values are printed out
  bool verbose_mode_;

  unsigned short encoder_counts_left_last_,     //!< last read wheel encoder counts for comparison
                  encoder_counts_right_last_;   //!< last read wheel encoder counts for comparison

  double pos_X_old_,    //!< last calculated x- and y-position for comparison
          pos_Y_old_;     //!< last calculated x- and y-position for comparison

  ros::Time current_time_,      //!< timestamp for calculating
            last_time_;         //!< timestamp for calculating

  /**
   * @name Subscriber & Publisher
   * @{
   */
  ros::Publisher odom_pub_;
  ros::Publisher bumper_pub_;
  ros::Publisher power_pub_;
  ros::Publisher lightsensor_pub_;
  ros::Publisher wheeldrop_pub_;
  ros::Publisher infrared_pub_;

  ros::Subscriber cmd_sub_;
  ros::Subscriber led_sub_;
  // @}

  //! transformation broadcaster between odometry and roomba
  tf::TransformBroadcaster odom_broadcaster_;

  ros::NodeHandle node_handle_;

  //! stores status and commands for the leds
  roomrider_driver::Led led_;
};
