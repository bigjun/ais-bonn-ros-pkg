/*
 * roomba_driver.cpp
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

#include "roomba_driver.h"

IRobotRoombaDriver::IRobotRoombaDriver()
{
  // NOTE: ROS now gets mad at using '~', I've modified the next 30 or so lines of code to fix this (M.O.)

  // local variable for parameter reading
  int param = 0;

  ros::NodeHandle param_node("~");
  ros::NodeHandle node_handle_("/");

  // setting defaults
  safe_ = true;
  debug_mode_ = false;

  // reading parameter for safe mode
  param_node.param("safe_mode", param, 1);
  if (param != 1)
  {
    safe_ = false;
    std::cout << "WARNING: Setting roomba in full mode!" << std::endl;
    ROS_WARN("Setting roomba in full mode!");
  }
  ROS_DEBUG("Safe mode: %s",(safe_)?"true":"false");

  // reading parameter for debug_mode_
  param_node.param("debug_mode", param, 0);
  if (param != 0)
  {
    debug_mode_ = true;
  }
  ROS_DEBUG("Debug mode: %s",(debug_mode_)?"true":"false");

  // Retrieve internal serial port parameter from parameter server if it exists
  param_node.param("serial_port", serial_port_, std::string("/dev/roomba"));
  ROS_DEBUG_STREAM("Serial Port: " << serial_port_);

  // --- Parameter reading finished ---

  // Attempt to creating the datastructure for the roomba
  roomba_dev_ = roomba500_create(serial_port_.c_str());

  // some variables for calculating the pose
  encoder_counts_left_last_ = roomba_dev_->encoder_counts_left_offset;
  encoder_counts_right_last_ = roomba_dev_->encoder_counts_right_offset;
  pos_X_old_ = 0.0;
  pos_Y_old_ = 0.0;

  if (roomba500_open(roomba_dev_, !safe_) < 0)
  {
    roomba500_destroy(roomba_dev_);
    roomba_dev_ = NULL;
    ROS_FATAL("Failed to connect to roomba (on port %s)",serial_port_.c_str());
    ROS_BREAK();
  }
else    ROS_INFO("connect to: %s successfull", serial_port_.c_str());

    power_pub_ = node_handle_.advertise<roomrider::Power>("power", 20);
    lightsensor_pub_ = node_handle_.advertise<roomrider::Lightsensor>("lightsensor", 20);
    bumper_pub_ = node_handle_.advertise<roomrider::Bumper>("bumper", 20);
    wheeldrop_pub_ = node_handle_.advertise<roomrider::Wheeldrop>("wheeldrop", 20);
    infrared_pub_ = node_handle_.advertise<roomrider::Infrared>("infrared", 20);
    odom_pub_ = node_handle_.advertise<nav_msgs::Odometry>("odom", 20);
    cmd_sub_ = node_handle_.subscribe<geometry_msgs::Twist>("cmd_vel", 20, &IRobotRoombaDriver::cmdVelocityCallback, this);
    led_sub_ = node_handle_.subscribe<roomrider::Led>("led", 20, &IRobotRoombaDriver::ledCallback, this);
  }

void IRobotRoombaDriver::mainDeviceLoop()
{
  ros::Rate loop_rate(20);

  current_time_ = ros::Time::now();
  last_time_ = ros::Time::now();

  while (node_handle_.ok())
  {
    if (roomba500_get_sensors(roomba_dev_, -1) < 0)
    {
      ROS_ERROR("Failed to get sensor data from roomba");
      roomba500_close(roomba_dev_);
      return;
    }
    //roomba500_print(roomba_dev_);
    publishOdometry();
    publishPower();
    publishBumper();
    publishLightsensor();
    publishWheeldrop();
    publishInfrared();
    //			handleLeds();

    if (debug_mode_)
    {
      roomba500_print(roomba_dev_);
    }

    ros::spinOnce();
    loop_rate.sleep();

  }
}

void IRobotRoombaDriver::shutdown()
{
  if (roomba500_set_leds(roomba_dev_, 0, 0, 0, 0, 0, 0) < 0)
  {
    ROS_ERROR("Failed to turn off the leds of the roomba");
  }
  if (roomba500_close(roomba_dev_))
  {
    ROS_ERROR("Failed to close roomba connection");
  }
  roomba500_destroy(roomba_dev_);
  roomba_dev_ = NULL;
}

// Utility function to calculate difference of two
// wrap-around values
int IRobotRoombaDriver::wrapAround16(unsigned short current, unsigned short last)
{
  int straight_diff = ((int)current) - ((int)last);
  int negative_wrap = ((int)current) + 65535 - ((int)last);
  int positive_wrap = ((int)current) - 65535 - ((int)last);

  // Return smallest distance
  if (abs(straight_diff) < abs(negative_wrap))
  {
    if (abs(straight_diff) < abs(positive_wrap))
      return straight_diff;
    else
      return positive_wrap;
  }
  else // negative_wrap <= abs(straight_diff)
  {
    if (abs(negative_wrap) < abs(positive_wrap))
      return negative_wrap;
    else
      return positive_wrap;
  }
}

// Callback method for motion commands to thr roomba
void IRobotRoombaDriver::cmdVelocityCallback(const geometry_msgs::TwistConstPtr& cmd_msg)
{

  double forward_speed = cmd_msg->linear.x;
  double turn_speed = cmd_msg->angular.z;

  ROS_DEBUG("Sending motor commands forward: %f, rotate: %f", forward_speed, turn_speed);
  if (roomba500_set_speeds(roomba_dev_, forward_speed, turn_speed) < 0)
  {
    ROS_ERROR("Failed to set speeds to roomba");
  }

}

// Callback method for the led_ messages
void IRobotRoombaDriver::ledCallback(const roomrider::LedConstPtr& led_msg)
{
  ROS_DEBUG("Receiving LED Commands");
  led_.dirt = led_msg->dirt;
  led_.dock = led_msg->dock;
  led_.spot = led_msg->spot;
  led_.status = led_msg->status;
  led_.main_color = led_msg->main_color;
  led_.main_intensity = led_msg->main_intensity;

  ROS_DEBUG("Dirt Detector Led: %d", led_.dirt);
  ROS_DEBUG("Docking Led: %d", led_.dock);
  ROS_DEBUG("Spot Led: %d", led_.spot);
  ROS_DEBUG("Status Led: %d", led_.status);

  ROS_DEBUG("Main Led Color: %d", led_.main_color);
  ROS_DEBUG("Main Led Intensity: %d", led_.main_intensity);
  handleLeds();
}

// method for sending led_-commands to the roomba
void IRobotRoombaDriver::handleLeds()
{
  if (roomba500_set_leds(roomba_dev_, led_.dirt, led_.dock, led_.main_color, led_.main_intensity, led_.spot,
                         led_.status) < 0)
  {
    ROS_ERROR("Failed to set the leds of the roomba");
  }
  else
  {
    ROS_DEBUG("Setting the leds successful");
  }
}

// publishes the odometry
void IRobotRoombaDriver::publishOdometry()
{

  // difference between measured encoder counts
  int delta_left, delta_right, delta_encode;
  // velocities of wheels, x-y-direction, the average distance and difference of the two wheels
  double left_vel, right_vel, vel_w, x_vel, y_vel, distance, delta_a;
  // roombas pose and angular velocity
  double pos_deltaA, posA, posX, posY;

  ROS_DEBUG("left_enc:[%d] right_enc:[%d]", roomba_dev_->encoder_counts_left, roomba_dev_->encoder_counts_right);

  // Calculating differences between new encoder counts and the last ones in order to calculate the speed
  delta_left = wrapAround16(roomba_dev_->encoder_counts_left, encoder_counts_left_last_);
  delta_right = wrapAround16(roomba_dev_->encoder_counts_right, encoder_counts_right_last_);

  ROS_DEBUG( "delta_left:[%d] delta_right:[%d]", delta_left, delta_right );

  delta_encode = delta_right - delta_left;
  // differnce of the two wheel distances
  delta_a = (((double)(delta_encode)) / TICKS_PER_MM) / 1000;

  // angular rotation
  roomba_dev_->pose_deltaA = delta_a / ROOMBA_WHEEL_DISTANCE;
  pos_deltaA = roomba_dev_->pose_deltaA;
  // angle of last pose
  posA = roomba_dev_->poseA;
  // new angle of pose
  roomba_dev_->poseA = atan2(sin(posA + pos_deltaA), cos(posA + pos_deltaA));
  posA = roomba_dev_->poseA;

  // average distance
  distance = (((delta_left + delta_right) / 2.0) / TICKS_PER_MM) / 1000;

  roomba_dev_->distance += distance;

  // calculate cartesian coord
  roomba_dev_->poseX += distance * cos(posA);
  posX = roomba_dev_->poseX;
  ROS_DEBUG("x:[%f]", posX);
  roomba_dev_->poseY += distance * sin(posA);
  posY = roomba_dev_->poseY;
  ROS_DEBUG("y:[%f]", posY);

  current_time_ = ros::Time::now();

  //compute odometry in a typical way given the velocities of the robot
  double dt = (current_time_ - last_time_).toSec();

  left_vel = (delta_left / TICKS_PER_MM) / 1000 / dt;
  right_vel = (delta_right / TICKS_PER_MM) / 1000 / dt;

  roomba_dev_->poseVX = (posX - pos_X_old_) / dt;
  x_vel = roomba_dev_->poseVX;
  ROS_DEBUG("xVel:[%f]", x_vel);
  roomba_dev_->poseVY = (posY - pos_Y_old_) / dt;
  y_vel = roomba_dev_->poseVY;
  ROS_DEBUG("yVel:[%f]", y_vel);

  roomba_dev_->pose_deltaA = (right_vel - left_vel) / ROOMBA_WHEEL_DISTANCE;
  vel_w = roomba_dev_->pose_deltaA;
  ROS_DEBUG("wVel:[%f]", vel_w);

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(posA);

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time_;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = posX;
  odom_trans.transform.translation.y = posY;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster_.sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time_;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = posX;
  odom.pose.pose.position.y = posY;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = x_vel;
  //	odom.twist.twist.linear.y = y_vel;
  odom.twist.twist.angular.z = vel_w;

  //publish the message
  odom_pub_.publish(odom);

  pos_X_old_ = posX;
  pos_Y_old_ = posY;
  encoder_counts_left_last_ = roomba_dev_->encoder_counts_left;
  encoder_counts_right_last_ = roomba_dev_->encoder_counts_right;
  last_time_ = current_time_;
}

// publisher for the bumper_ values
void IRobotRoombaDriver::publishBumper()
{
  roomrider::Bumper bumper;
  bumper.left = roomba_dev_->bumper_left;
  bumper.right = roomba_dev_->bumper_right;
  bumper_pub_.publish(bumper);
}

// publisher for the wheeldrop_ values
void IRobotRoombaDriver::publishWheeldrop()
{
  roomrider::Wheeldrop wheeldrop;
  wheeldrop.left = roomba_dev_->wheeldrop_left;
  wheeldrop.right = roomba_dev_->wheeldrop_right;
  wheeldrop_pub_.publish(wheeldrop);
}

// publisher for the battery status
void IRobotRoombaDriver::publishPower()
{
  roomrider::Power power;
  power.charging = roomba_dev_->charging_state;
  power.temperature = roomba_dev_->temperature;
  power.charge = roomba_dev_->charge;
  power.volts = roomba_dev_->voltage;

  power.ampere = roomba_dev_->current;
  power.capacity = roomba_dev_->capacity;

  if (roomba_dev_->capacity != 0.0)
  {
    power.percent = 100.0 * (roomba_dev_->charge / roomba_dev_->capacity);
    if ((power.percent > 110.0) || (power.percent < 0.0))
    {
      power.percent = -1.0;
    }
  }
  else
  {
    power.percent = -1.0;
  }
  power_pub_.publish(power);
}

// publisher for the light- and cliffsensors
void IRobotRoombaDriver::publishLightsensor()
{
  roomrider::Lightsensor sensor_msg;
  sensor_msg.cliff_left = sensor_msg.sensors[0] = roomba_dev_->cliff_left;
  sensor_msg.cliff_left_data = sensor_msg.data[0] = roomba_dev_->cliff_left_data;
  sensor_msg.cliff_frontleft = sensor_msg.sensors[1] = roomba_dev_->cliff_frontleft;
  sensor_msg.cliff_frontleft_data = sensor_msg.data[1] = roomba_dev_->cliff_frontleft_data;
  sensor_msg.cliff_frontright = sensor_msg.sensors[2] = roomba_dev_->cliff_frontright;
  sensor_msg.cliff_frontright_data = sensor_msg.data[2] = roomba_dev_->cliff_frontright_data;
  sensor_msg.cliff_right = sensor_msg.sensors[3] = roomba_dev_->cliff_right;
  sensor_msg.cliff_right_data = sensor_msg.data[3] = roomba_dev_->cliff_right_data;

  sensor_msg.light_bump_left = sensor_msg.sensors[4] = roomba_dev_->light_bump_left;
  sensor_msg.light_bump_left_data = sensor_msg.data[4] = roomba_dev_->light_bump_left_data;
  sensor_msg.light_bump_frontleft = sensor_msg.sensors[5] = roomba_dev_->light_bump_frontleft;
  sensor_msg.light_bump_frontleft_data = sensor_msg.data[5] = roomba_dev_->light_bump_frontleft_data;
  sensor_msg.light_bump_centerleft = sensor_msg.sensors[6] = roomba_dev_->light_bump_centerleft;
  sensor_msg.light_bump_centerleft_data = sensor_msg.data[6] = roomba_dev_->light_bump_centerleft_data;
  sensor_msg.light_bump_centerright = sensor_msg.sensors[7] = roomba_dev_->light_bump_centerright;
  sensor_msg.light_bump_centerright_data = sensor_msg.data[7] = roomba_dev_->light_bump_centerright_data;
  sensor_msg.light_bump_frontright = sensor_msg.sensors[8] = roomba_dev_->light_bump_frontright;
  sensor_msg.light_bump_frontright_data = sensor_msg.data[8] = roomba_dev_->light_bump_frontright_data;
  sensor_msg.light_bump_right = sensor_msg.sensors[9] = roomba_dev_->light_bump_right;
  sensor_msg.light_bump_right_data = sensor_msg.data[9] = roomba_dev_->light_bump_right_data;

  sensor_msg.wall_sensor = sensor_msg.sensors[10] = roomba_dev_->wall;
  sensor_msg.wall_sensor_data = sensor_msg.data[10] = roomba_dev_->wall_data;

  lightsensor_pub_.publish(sensor_msg);
}

void IRobotRoombaDriver::publishInfrared()
{
  roomrider::Infrared infrared;
  infrared.virtualwall = roomba_dev_->virtual_wall;
  infrared.opcode_left = roomba_dev_->ir_opcode_left;
  infrared.opcode_right = roomba_dev_->ir_opcode_right;
  infrared_pub_.publish(infrared);
}

int main(int argc, char** argv)
{

  // Initialize
  ros::init(argc, argv, "roomba");

  // Create a new instance of IRobot_Create
  ROS_INFO("Initializing roomba driver...");

  IRobotRoombaDriver driver;

  ROS_INFO("Connection to roomba established");

  // main loop
  driver.mainDeviceLoop();

  //destroy
  ROS_INFO("Shutting down roomba...");
  driver.shutdown();
  ROS_INFO("roomba shut down.");

  return 0;
}
