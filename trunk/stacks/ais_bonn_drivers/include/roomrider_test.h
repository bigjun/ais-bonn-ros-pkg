/*
 * roomrider_test.h
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
 */

#include <ros/ros.h>
#include <joy/Joy.h>
#include <geometry_msgs/Twist.h>
#include <roomrider_driver/Power.h>
#include <roomrider_driver/Lightsensor.h>
#include <roomrider_driver/Bumper.h>
#include <roomrider_driver/Infrared.h>
#include <roomrider_driver/Led.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

/**
 * \class RoomriderTest
 * @brief The class accepts joystick commands to control a roomba and its LEDs and writes
 * some sensor data (including laser, if available) to the screen
 */
class RoomriderTest
{

public:

  RoomriderTest();

  // some callback methods
  void joyCallback(const joy::JoyConstPtr& joy_msg);
  void powerCallback(const roomrider_driver::PowerConstPtr& power_msg);
  void lightsensorCallback(const roomrider_driver::LightsensorConstPtr& light_msg);
  void bumperCallback(const roomrider_driver::BumperConstPtr& bumper_msg);
  void wheeldropCallback(const roomrider_driver::BumperConstPtr& wheeldrop_msg);
  void infraredCallback(const roomrider_driver::InfraredConstPtr& infrared_msg);
  void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);

  //! normalizes joystick_ values
  void normalizeJoystickValues(float &value, float min, float max, float maximum_robot_speed);

  //! sets the leds according to joystick_-buttons
  void setLeds();

  //! calculates speed according to joystick_-values
  void calculateSpeed();

  //! prints some values to screen
  void outputRoombaData();

  void mainLoop();

protected:

  static const double MAX_TRANSLATIONAL_SPEED = 0.5;    //!< static maximum translational speed for the robot
  static const double MAX_ANGULAR_SPEED = 1.0;          //!< static maximum rotational speed for the robot

  double MAX_JOYSTICK_VALUE;    //!< incoming joystick_ values will be normalized to the following ones
  double MIN_JOYSTICK_VALUE;    //!< incoming joystick_ values will be normalized to the following ones

  ros::NodeHandle n_;

  /**
   * @name Subscriber & Publisher
   * @{
   */
  ros::Subscriber joy_sub_;
  ros::Subscriber power_sub_;
  ros::Subscriber light_sub_;
  ros::Subscriber bumper_sub_;
  ros::Subscriber wheeldrop_sub_;
  ros::Subscriber infrared_sub_;
  ros::Publisher command_pub_;
  ros::Publisher led_pub_;
  ros::Subscriber laser_sub_;
  // @}

  /**
   * @name Messages
   * @brief These member variables store data from messages received/sent
   * @{
   */
  joy::Joy joystick_;
  geometry_msgs::Twist roomba_cmd_;
  sensor_msgs::LaserScan laserscan_;

  roomrider_driver::Power power_;
  roomrider_driver::Lightsensor lightsensor_;
  roomrider_driver::Bumper bumper_;
  roomrider_driver::Bumper wheeldrop_;
  roomrider_driver::Infrared infrared_;
  roomrider_driver::Led led_;
  // @}

  unsigned int x_axis_,         //!< Stores the joystick-axis for translational movement
               y_axis_;         //!< Stores the joystick-axis for rotational movement
  double left_laser_,           //!< Range of the outermost left laserbeam
          middle_laser_,        //!< Range of the middle laserbeam
          right_laser_;         //!< Range of the outermost right laserbeam
};
