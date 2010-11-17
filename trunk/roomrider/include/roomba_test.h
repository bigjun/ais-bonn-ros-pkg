/*
 * roomba_test.h
 *
 * by Andreas Hochrath & Torsten Fiolka in August 2010
 *
 * at	        University Bonn,
 * 		Institute for Computer Science VI,
 * 		Autonomous Intelligent Systems
 *
 * contact: fiolka@cs.uni-bonn.de, hochrath@cs.uni-bonn.de
 *
 */

/*
 * Copyright (C) 2010 Autonomous Intelligent Systems, University Bonn, Germany
 *
 * This program is free software; you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation;
 * either version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this
 * program; if not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <ros/ros.h>
#include <joy/Joy.h>
#include <geometry_msgs/Twist.h>
#include <roomrider/Power.h>
#include <roomrider/Lightsensor.h>
#include <roomrider/Bumper.h>
#include <roomrider/Infrared.h>
#include <roomrider/Led.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

class RoombaTest
{

public:

  // static maximum speeds for the robot
  static const double MAX_TRANSLATIONAL_SPEED = 0.5;
  static const double MAX_ANGULAR_SPEED = 1.0;
  // incoming joystick_ values will be normalized to the following ones
  // these will be initialized in the constructor and after that only read
  double MAX_JOYSTICK_VALUE;
  double MIN_JOYSTICK_VALUE;

  RoombaTest();

  // some callback methods
  void joyCallback(const joy::JoyConstPtr& joy_msg);
  void powerCallback(const roomrider::PowerConstPtr& power_msg);
  void lightsensorCallback(const roomrider::LightsensorConstPtr& light_msg);
  void bumperCallback(const roomrider::BumperConstPtr& bumper_msg);
  void wheeldropCallback(const roomrider::BumperConstPtr& wheeldrop_msg);
  void infraredCallback(const roomrider::InfraredConstPtr& infrared_msg);
  void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);

  // normalizes joystick_ values
  void normalizeJoystickValues(float &value, float min, float max, float maximum_robot_speed);

  // sets the leds according to joystick_-buttons
  void setLeds();

  // calculates speed according to joystick_-values
  void calculateSpeed();

  // prints some values to screen
  void outputRoombaData();

  void mainLoop();

protected:

  ros::NodeHandle n_;
  ros::Subscriber joy_sub_;
  ros::Subscriber power_sub_;
  ros::Subscriber light_sub_;
  ros::Subscriber bumper_sub_;
  ros::Subscriber wheeldrop_sub_;
  ros::Subscriber infrared_sub_;
  ros::Publisher command_pub_;
  ros::Publisher led_pub_;
  ros::Subscriber laser_sub_;

  joy::Joy joystick_;
  geometry_msgs::Twist roomba_cmd_;
  sensor_msgs::LaserScan laserscan_;

  roomrider::Power power_;
  roomrider::Lightsensor lightsensor_;
  roomrider::Bumper bumper_;
  roomrider::Bumper wheeldrop_;
  roomrider::Infrared infrared_;
  roomrider::Led led_;

  unsigned int x_axis_, y_axis_;
  double left_laser_, middle_laser_, right_laser_;

};
