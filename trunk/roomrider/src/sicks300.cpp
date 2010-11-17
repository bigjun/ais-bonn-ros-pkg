/*
 * sicks300.cpp
 *
 * written by: ?
 * modified by Andreas Hochrath & Torsten Fiolka in August 2010
 *
 * at   University Bonn,
 *              Institute for Computer Science VI,
 *              Autonomous Intelligent Systems
 *
 * contact: fiolka@cs.uni-bonn.de, hochrath@cs.uni-bonn.de
 *
 * We have no idea who wrote this driver originally, since no header was provided.
 * If you know who wrote this driver, please let us know so we can complete the header.
 *
 */


#include "sicks300.h"

#include "termios.h"

SickS300::SickS300()
{

  ros::NodeHandle param_node("~");
  ros::NodeHandle node_handle_("/");

  int param = 0;
  reduced_fov = false;

  // reading transformation parameters from parameter server
  param_node.param(std::string("frame"), m_scanData.header.frame_id, std::string("base_laser_link"));
  param_node.param(std::string("tf_x"), x, 0.115);
  param_node.param(std::string("tf_y"), y, 0.0);
  param_node.param(std::string("tf_z"), z, 0.21);

  // Setting full field of view (270 degree) or reduced (180 degree)
  param_node.param(std::string("reduced_fov"), param, 0);
  if (param != 0)
  {
    reduced_fov = true;
    ROS_INFO("INFO: Starting Sick300-Laser with reduced field ov view of 180 degree");
  }

  if (!reduced_fov)
  {
    m_scanData.angle_min = -135.f / 180.f * M_PI;
    m_scanData.angle_max = 135.f / 180.f * M_PI;
    m_scanData.angle_increment = 0.5f / 180.f * M_PI;
    m_scanData.time_increment = 0;
    m_scanData.scan_time = 0.08;
    m_scanData.range_min = 0.1;
    m_scanData.range_max = 29.0;
    m_scanData.ranges.resize(541);
    m_scanData.intensities.resize(541);
  }
  else
  {
    m_scanData.angle_min = -90.f / 180.f * M_PI;
    m_scanData.angle_max = 90.f / 180.f * M_PI;
    m_scanData.angle_increment = 0.5f / 180.f * M_PI;
    m_scanData.time_increment = 0;
    m_scanData.scan_time = 0.08;
    m_scanData.range_min = 0.1;
    m_scanData.range_max = 29.0;
    m_scanData.ranges.resize(361);
    m_scanData.intensities.resize(361);
  }

  // Reading device parameters
  param_node.param(std::string("devicename"), m_deviceName, std::string("/dev/sick300"));
  param_node.param(std::string("baudrate"), m_baudRate, 500000);

  m_connected = m_serialComm.connect(m_deviceName, m_baudRate);

  m_scanDataPublisher = node_handle_.advertise<sensor_msgs::LaserScan> ("laserscan", 10);

}

SickS300::~SickS300()
{
}

void SickS300::update()
{

  if (m_connected != 0)
  {
    m_connected = m_serialComm.connect(m_deviceName, m_baudRate);
  }

  if (m_connected == 0 && m_serialComm.readData() == 0)
  {

    float* ranges = m_serialComm.getRanges();
    unsigned int numRanges = m_serialComm.getNumRanges();
    if (!reduced_fov)
    {
      m_scanData.ranges.resize(numRanges);
      for (unsigned int i = 0; i < numRanges; i++)
        m_scanData.ranges[i] = ranges[i];
    }
    else
    {
      for (unsigned int i = 0; i < 361; i++)
        m_scanData.ranges[i] = ranges[i + 89];
    }
    m_scanData.header.stamp = ros::Time::now();

    m_scanDataPublisher.publish(m_scanData);

  }

}

void SickS300::broadcast_transform()
{
  tf_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(x, y, z)),
                                                    ros::Time::now(), "base_link", "base_laser_link"));

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "sicks300");
  ros::Rate loop_rate(20);

  ROS_INFO("Opening connection to Sick300-Laser...");

  SickS300 sickS300;

  ROS_INFO("Sick300 connected.");

  while (sickS300.node_handle_.ok())
  {

    sickS300.update();
    sickS300.broadcast_transform();

    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("Laser shut down.");

  return 0;
}

