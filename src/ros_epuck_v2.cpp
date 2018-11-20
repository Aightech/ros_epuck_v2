#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "ros_epuck_v2/ros_epuck_v2.hpp"
#include "ros_epuck_v2/epuck_bluetooth.hpp"

Epuck::Epuck(ros::NodeHandle &n, const char * path)
{
	node = n;
	epuck_fd = init_connection(path);
	if(epuck_fd>-1)
		std::cout << "Connected ..." << std::endl;
}


void Epuck::init_IR_sensors()
{
	for(int i=0; i<8; i++)
	{
		std::stringstream ss;
		ss.str("");
		ss << "IR_sensor" << i;
		IR_sensors_pub[i] = node.advertise<sensor_msgs::Range>(ss.str(), 10);
		IR_sensors_msg[i].radiation_type = sensor_msgs::Range::INFRARED;
		
		ss.str("");
		ss << "epuckv2/base_IR_sensor" << i;
		IR_sensors_msg[i].header.frame_id =  ss.str();
		
		IR_sensors_msg[i].field_of_view = 0.26;    // About 15 degrees...to be checked!
		IR_sensors_msg[i].min_range = 0.005;       // 0.5 cm.
		IR_sensors_msg[i].max_range = 0.05;        // 5 cm.                    
	}
	//laser_pub = node.advertise<sensor_msgs::LaserScan>("scan", 10);
}

void Epuck::update_IR_sensors(int *IR_values)
{
	for(int i=0; i<8; i++)
	{
		if(IR_values[i] > 0) 
			IR_sensors_msg[i].range = 0.5/sqrt(IR_values[i]);  // Transform the analog value to a distance value in meters (given from field tests).
		else
			IR_sensors_msg[i].range = IR_sensors_msg[i].max_range;

		if(IR_sensors_msg[i].range > IR_sensors_msg[i].max_range)
			IR_sensors_msg[i].range = IR_sensors_msg[i].max_range;
		
		if(IR_sensors_msg[i].range < IR_sensors_msg[i].min_range)
			IR_sensors_msg[i].range = IR_sensors_msg[i].min_range;

		IR_sensors_msg[i].header.stamp = ros::Time::now();
		IR_sensors_pub[i].publish(IR_sensors_msg[i]);
	}
}

//void Epuck::update_laserScan(int *IR_values)
//{
//	
//}

