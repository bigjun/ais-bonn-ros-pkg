/*
 * sick300.h
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


#ifndef __SICKS300_H__
#define __SICKS300_H__

#include "serialcomm_s300.h"

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>



class SickS300 {
public:

	SickS300();
	~SickS300();

	void update();
	void broadcast_transform();

	ros::NodeHandle node_handle_;

protected:

	SerialCommS300 m_serialComm;

	sensor_msgs::LaserScan m_scanData;
	ros::Publisher m_scanDataPublisher;

	tf::TransformBroadcaster tf_broadcaster;
	double x, y, z;
	bool reduced_fov;

	std::string m_deviceName;
	int m_baudRate;
	int m_connected;
};



#endif // __SICKS300_H__


