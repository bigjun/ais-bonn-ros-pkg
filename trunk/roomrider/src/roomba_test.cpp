/*
 * roomba_test.cpp
 *
 * by Andreas Hochrath & Torsten Fiolka in August 2010
 *
 * at           University Bonn,
 *              Institute for Computer Science VI,
 *              Autonomous Intelligent Systems
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


#include "roomba_test.h"

// Initializing RoombaTest
RoombaTest::RoombaTest()
{

  // Reading parameters
  int param = 0;

  ros::NodeHandle n_("/");
  ros::NodeHandle paramNode("~");

  paramNode.param(std::string("x_axis"), param, 1);
  if (param >= 0)
  {
    x_axis_ = (uint)param;
  }
  else
  {
    x_axis_ = 1;
  }

  paramNode.param(std::string("y_axis"), param, 0);
  if (param >= 0)
  {
    y_axis_ = (uint)param;
  }
  else
  {
    y_axis_ = 1;
  }

  paramNode.param(std::string("min_joystick_value"), MIN_JOYSTICK_VALUE, 0.0);
  paramNode.param(std::string("max_joystick_value"), MAX_JOYSTICK_VALUE, 1.0);

  ROS_INFO(" X-Axis: %d   Y-Axis: %d", x_axis_, y_axis_);
  ROS_INFO(" Minimum joystick value: %f", MIN_JOYSTICK_VALUE);
  ROS_INFO(" Maximum joystick value: %f", MAX_JOYSTICK_VALUE);

  // Initializing ROS communication
  joy_sub_ = n_.subscribe<joy::Joy> ("joy", 20, &RoombaTest::joyCallback, this);
  power_sub_ = n_.subscribe<roomrider::Power> ("power", 20, &RoombaTest::powerCallback, this);
  light_sub_ = n_.subscribe<roomrider::Lightsensor> ("lightsensor", 20, &RoombaTest::lightsensorCallback, this);
  bumper_sub_ = n_.subscribe<roomrider::Bumper> ("bumper", 20, &RoombaTest::bumperCallback, this);
  wheeldrop_sub_ = n_.subscribe<roomrider::Bumper> ("wheeldrop", 20, &RoombaTest::wheeldropCallback, this);
  infrared_sub_ = n_.subscribe<roomrider::Infrared> ("infrared", 20, &RoombaTest::infraredCallback, this);
  command_pub_ = n_.advertise<geometry_msgs::Twist> ("cmd_vel", 20);
  led_pub_ = n_.advertise<roomrider::Led> ("led", 20);
  laser_sub_ = n_.subscribe<sensor_msgs::LaserScan> ("laserscan", 20, &RoombaTest::laserCallback, this);
}

// normalizes joystick_ values if the incoming values have other limits than 0.0 and 1.0
// new values are read at initialization
// min is the new minimum value, up to which the speed will be set to 0.0 (default: 0.0)
// max is the new maximum value, at which the speed will be set to maximumAllowedValue (default: 1.0)
// maximumAllowedValue specifies the maximum robot speed (translational: 0.5, angular: 1.0)
void RoombaTest::normalizeJoystickValues(float &value, float min, float max, float maximumAllowedValue)
{
  if ((fabs(value) < min) || (min == max))
  {
    value = 0.0;
  }
  else if (value > max)
  {
    value = maximumAllowedValue;
  }
  else if (value < -max)
  {
    value = -maximumAllowedValue;
  }
  else
  {
    value = (value / max) * maximumAllowedValue;
  }
}

// reads the axes five and six to set color and intensity of the main led_
// the buttons one to four are used to switch the other leds
void RoombaTest::setLeds()
{
  if (joystick_.get_axes_size() >= 6)
  {
    led_.main_color += (int)(floor(joystick_.axes[4] + 0.5));
    led_.main_intensity += (int)(floor(joystick_.axes[5] + 0.5));
  }
  if (joystick_.get_buttons_size() >= 4)
  {
    if (joystick_.buttons[0])
    {
      led_.status = !led_.status;
    }
    if (joystick_.buttons[1])
    {
      led_.spot = !led_.spot;
    }
    if (joystick_.buttons[2])
    {
      led_.dock = !led_.dock;
    }
    if (joystick_.buttons[3])
    {
      led_.dirt = !led_.dirt;
    }
  }
}

// reads the joystick_ values and normalizes them
void RoombaTest::joyCallback(const joy::JoyConstPtr& joy_msg)
{
  joystick_.set_axes_size(joy_msg->get_axes_size());
  joystick_.set_buttons_size(joy_msg->get_buttons_size());
  uint i = 0;
  for (i = 0; i < joy_msg->get_axes_size(); i++)
  {
    joystick_.axes[i] = joy_msg->axes[i];
  }
  for (i = 0; i < joy_msg->get_buttons_size(); i++)
  {
    joystick_.buttons[i] = joy_msg->buttons[i];
  }
  if (x_axis_ < joystick_.get_axes_size())
  {
    normalizeJoystickValues(joystick_.axes[x_axis_], MIN_JOYSTICK_VALUE, MAX_JOYSTICK_VALUE, MAX_TRANSLATIONAL_SPEED);
  }
  if (y_axis_ < joystick_.get_axes_size())
  {
    normalizeJoystickValues(joystick_.axes[y_axis_], MIN_JOYSTICK_VALUE, MAX_JOYSTICK_VALUE, MAX_ANGULAR_SPEED);
  }
}

// callback for the power_-messages
void RoombaTest::powerCallback(const roomrider::PowerConstPtr& power_msg)
{
  power_.capacity = power_msg->capacity;
  power_.charging = power_msg->charging;
  power_.percent = power_msg->percent;
  power_.temperature = power_msg->temperature;
  power_.volts = power_msg->volts;
  power_.ampere = power_msg->ampere;
  power_.charge = power_msg->charge;
}

// callback for the lightsensors
void RoombaTest::lightsensorCallback(const roomrider::LightsensorConstPtr& light_msg)
{
  lightsensor_.cliff_frontleft = light_msg->cliff_frontleft;
  lightsensor_.cliff_frontleft_data = light_msg->cliff_frontleft_data;
  lightsensor_.cliff_frontright = light_msg->cliff_frontright;
  lightsensor_.cliff_frontright_data = light_msg->cliff_frontright_data;
  lightsensor_.cliff_left = light_msg->cliff_left;
  lightsensor_.cliff_left_data = light_msg->cliff_left_data;
  lightsensor_.cliff_right = light_msg->cliff_right;
  lightsensor_.cliff_right_data = light_msg->cliff_right_data;
  lightsensor_.wall_sensor = light_msg->wall_sensor;
  lightsensor_.wall_sensor_data = light_msg->wall_sensor_data;
  lightsensor_.light_bump_centerleft = light_msg->light_bump_centerleft;
  lightsensor_.light_bump_centerleft_data = light_msg->light_bump_centerleft_data;
  lightsensor_.light_bump_centerright = light_msg->light_bump_centerright;
  lightsensor_.light_bump_centerright_data = light_msg->light_bump_centerright_data;
  lightsensor_.light_bump_frontleft = light_msg->light_bump_frontleft;
  lightsensor_.light_bump_frontleft_data = light_msg->light_bump_frontleft_data;
  lightsensor_.light_bump_frontright = light_msg->light_bump_frontright;
  lightsensor_.light_bump_frontright_data = light_msg->light_bump_frontright_data;
  lightsensor_.light_bump_left = light_msg->light_bump_left;
  lightsensor_.light_bump_left_data = light_msg->light_bump_left_data;
  lightsensor_.light_bump_right = light_msg->light_bump_right;
  lightsensor_.light_bump_right_data = light_msg->light_bump_right_data;
}

// callback for the bumper_
void RoombaTest::bumperCallback(const roomrider::BumperConstPtr& bumper_msg)
{
  bumper_.left = bumper_msg->left;
  bumper_.right = bumper_msg->right;
}

// callback for the wheeldrop_
void RoombaTest::wheeldropCallback(const roomrider::BumperConstPtr& wheeldrop_msg)
{
  wheeldrop_.left = wheeldrop_msg->left;
  wheeldrop_.right = wheeldrop_msg->right;
}

// callback for the infrared_
void RoombaTest::infraredCallback(const roomrider::InfraredConstPtr& infrared_msg)
{
  infrared_.opcode_left = infrared_msg->opcode_left;
  infrared_.opcode_right = infrared_msg->opcode_right;
  infrared_.virtualwall = infrared_msg->virtualwall;
}

// callback for the laser
void RoombaTest::laserCallback(const sensor_msgs::LaserScanConstPtr& m_scanData)
{
  if (m_scanData->get_ranges_size() > 0)
  {
    left_laser_ = m_scanData->ranges[m_scanData->get_ranges_size() - 1];
    middle_laser_ = m_scanData->ranges[m_scanData->get_ranges_size() / 2];
    right_laser_ = m_scanData->ranges[0];
  }
}

// sets the robots speed to the normalized values of the corresponding axes
void RoombaTest::calculateSpeed()
{
  if (x_axis_ < joystick_.get_axes_size())
  {
    roomba_cmd_.linear.x = joystick_.axes[x_axis_];
  }
  if (x_axis_ < joystick_.get_axes_size())
  {
    roomba_cmd_.angular.z = joystick_.axes[y_axis_];
  }
}

// outputs some data
void RoombaTest::outputRoombaData()
{
  if (system("clear") != 0)
  {
    std::cout << "Having Problems clearing the screen, sorry. " << std::endl;
  }
  std::cout << std::endl;
  std::cout << "Bumper: ";
  if (bumper_.left)
  {
    std::cout << "XXXXXXXXXX\t\t\t";
  }
  else
  {
    std::cout << "----------\t\t\t";
  }
  if (bumper_.right)
  {
    std::cout << "XXXXXXXXXX\t\t\t";
  }
  else
  {
    std::cout << "----------\t\t\t";
  }
  std::cout << std::endl << std::endl;
  std::cout << "Lightsensors: left: " << lightsensor_.light_bump_left_data << " frontleft: "
      << lightsensor_.light_bump_frontleft_data << " centerleft:" << lightsensor_.light_bump_centerleft_data
      << " centerright: " << lightsensor_.light_bump_centerright_data << " frontright: "
      << lightsensor_.light_bump_frontright_data << " right:" << lightsensor_.light_bump_right_data << " wall_sensor: "
      << lightsensor_.wall_sensor_data << std::endl << std::endl;
  std::cout << "Battery status: " << power_.percent << "% - " << power_.volts << "V - " << power_.ampere
      << "A - charge " << power_.charge << " capacity: " << power_.capacity << " temperature: " << power_.temperature
      << " charging status: " << (float)power_.charging << std::endl << std::endl;
  std::cout << "Infrared: left: " << infrared_.opcode_left << " right: " << infrared_.opcode_right << " virtualwall: "
      << infrared_.virtualwall << std::endl << std::endl;
  std::cout << "Laser: left: " << left_laser_ << " middle: " << middle_laser_ << " right: " << right_laser_
      << std::endl << std::endl;
  std::cout << "Linear Speed: " << roomba_cmd_.linear.x << "\t Angular Speed: " << roomba_cmd_.angular.z << std::endl
      << std::endl;

}

// mainloop at 20 Hz
void RoombaTest::mainLoop()
{
  ros::Rate loop_rate(20);

  while (n_.ok())
  {
    setLeds();
    calculateSpeed();

    command_pub_.publish(roomba_cmd_);
    led_pub_.publish(led_);

    outputRoombaData();

    ros::spinOnce();
    loop_rate.sleep();
  }

}

int main(int argc, char** argv)
{

  // initialize ROS
  ros::init(argc, argv, "roomba_test");

  std::cout << "test beginnt" << std::endl;

  RoombaTest::RoombaTest test;

  ROS_INFO("Starting roomrider test program");
  test.mainLoop();
  ROS_INFO("Stopping roomrider test program");

  return (0);
}
