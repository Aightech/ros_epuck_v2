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
	m_node = n;
//	m_epuck_fd = init_connection(path);
//	if(m_epuck_fd>-1)
//		std::cout << "Connected ..." << std::endl;
//	
	init_IR_sensors();
	init_laserScan();
}

void Epuck::update()
{
//	char buff[20];
//	int len = cmd_get_spd(buff);//get the code to get speed 
//	len += cmd_set_spd(buff+len,m_speedLeft,m_speedRight);//get the code to set speed 
//	len += cmd_get_sIR(buff+len);//get the code to get IR sensors states
//	send_cmd(m_epuck_fd,buff,len+1);//send cmd

//	int values[10];
//	int n = recv_rep(m_epuck_fd,buff);
//	if(n>-1)
//	{
//		printSHORT(buff,n);
//		get_values(buff,n-1,values);
//		update_IR_sensors(values+2);
//		update_laserScan(values+2);
//	}
	int values[8]={0,0,0,0,0,0,0,0};
	update_laserScan(values);
}



void Epuck::init_IR_sensors()
{
	for(int i=0; i<8; i++)
	{
		std::stringstream ss;
		ss.str("");
		ss << "IR_sensor" << i;
		m_IR_sensors_pub[i] = m_node.advertise<sensor_msgs::Range>(ss.str(), 10);
		m_IR_sensors_msg[i].radiation_type = sensor_msgs::Range::INFRARED;
		
		ss.str("");
		ss << "epuckv2/base_IR_sensor" << i;
		m_IR_sensors_msg[i].header.frame_id =  ss.str();
		
		m_IR_sensors_msg[i].field_of_view = 0.26;    // About 15 degrees...to be checked!
		m_IR_sensors_msg[i].min_range = 0.005;       // 0.5 cm.
		m_IR_sensors_msg[i].max_range = 0.05;        // 5 cm.                    
	}
}

void Epuck::update_IR_sensors(int *IR_values)
{
	for(int i=0; i<8; i++)
	{
		if(IR_values[i] > 0) 
			m_IR_sensors_msg[i].range = 0.5/sqrt(IR_values[i]);  // Transform the analog value to a distance value in meters (given from field tests).
		else
			m_IR_sensors_msg[i].range = m_IR_sensors_msg[i].max_range;

		if(m_IR_sensors_msg[i].range > m_IR_sensors_msg[i].max_range)
			m_IR_sensors_msg[i].range = m_IR_sensors_msg[i].max_range;
		
		if(m_IR_sensors_msg[i].range < m_IR_sensors_msg[i].min_range)
			m_IR_sensors_msg[i].range = m_IR_sensors_msg[i].min_range;

		m_IR_sensors_msg[i].header.stamp = ros::Time::now();
		m_IR_sensors_pub[i].publish(m_IR_sensors_msg[i]);
	}
}

void Epuck::init_laserScan()
{
	std::stringstream ss;
	ss.str("");
	ss << "epuckv2/base_laser";
	m_laser_msg.header.frame_id = ss.str();
	m_laser_msg.angle_min = -M_PI/2.0;
	m_laser_msg.angle_max = M_PI/2.0;
	m_laser_msg.angle_increment = M_PI/18.0; // 10 degrees.
	m_laser_msg.range_min = 0.005+0.035; // 0.5 cm + ROBOT_RADIUS.
	m_laser_msg.range_max = 0.05+0.035; // 5 cm + ROBOT_RADIUS. 
	m_laser_msg.ranges.resize(19);
	m_laser_msg.intensities.resize(19);
	m_laser_pub = m_node.advertise<sensor_msgs::LaserScan>("scan", 10);
}

void Epuck::update_laserScan(int *IR_values)
{
	//Using the 6 IR sensors on the front to simulate 19 laser scan points.

	// -90|-80|-70|-60|-50|-40|-30|-20|-10|  0| 10| 20| 30| 40| 50| 60| 70| 80|-90|
	//mask of interpolation
	float mask[19][8] = {	{0	,0	,1	,0	,0	,0	,0	,0	},
				{0	,1./5	,4./5	,0	,0	,0	,0	,0	},
				{0	,2./5	,3./5	,0	,0	,0	,0	,0	},
				{0	,3./5	,2./5	,0	,0	,0	,0	,0	},
				{0	,4./5	,1./5	,0	,0	,0	,0	,0	},
				{0	,1	,0	,0	,0	,0	,0	,0	},
				{1./3	,2./3	,0	,0	,0	,0	,0	,0	},
				{2./3	,1/3	,0	,0	,0	,0	,0	,0	},
				{1	,0	,0	,0	,0	,0	,0	,0	},
				{1./2	,0	,0	,0	,0	,0	,0	,1./2	},
				{0	,0	,0	,0	,0	,0	,0	,1	},
				{0	,0	,0	,0	,0	,0	,1./3	,2./3	},
				{0	,0	,0	,0	,0	,0	,2./3	,1./3	},
				{0	,0	,0	,0	,0	,0	,1	,0	},
				{0	,0	,0	,0	,0	,1./5	,4./5	,0	},
				{0	,0	,0	,0	,0	,2./5	,3./5	,0	},
				{0	,0	,0	,0	,0	,3./5	,2./5	,0	},
				{0	,0	,0	,0	,0	,4./5	,1./5	,0	},
				{0	,0	,0	,0	,0	,1	,0	,0	}
				};

	for(int i=0;i<19;i++)
	{
		float tempProx = 0;
		for(int j=0;j<8;j++) tempProx += mask[i][j]*IR_values[j];
		//std::cout << tempProx << " | ";
		if(tempProx > 0)
		{
			m_laser_msg.ranges[i] = (0.5/sqrt(tempProx))+0.035; //analog to meters.
			m_laser_msg.intensities[i] = tempProx; 
		}
		else
		{
			m_laser_msg.ranges[i] = m_laser_msg.range_max;
			m_laser_msg.intensities[i] = 0;
		}
	}
	//std::cout << std::endl;
	m_laser_pub.publish(m_laser_msg);
}

