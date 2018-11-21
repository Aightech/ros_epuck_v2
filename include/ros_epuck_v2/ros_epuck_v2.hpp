#ifndef ROS_EPUCK_V2_HPP
#define ROS_EPUCK_V2_HPP
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"

class Epuck
{
public:
	Epuck(ros::NodeHandle &n,const char * path);
	~Epuck(){};
	void init_IR_sensors();
	void update_IR_sensors(int *IR_values);
	void init_laserScan();
	void update_laserScan(int *IR_values);
	void set_spd(int speedLeft, int speedRight)
	{m_speedLeft=speedLeft;	m_speedRight=speedRight;};
	
	void update();
private:
	int m_epuck_fd;
	int m_speedLeft;
	int m_speedRight;
	ros::Publisher m_IR_sensors_pub[8];
	sensor_msgs::Range m_IR_sensors_msg[8];
	ros::Publisher m_laser_pub;
	sensor_msgs::LaserScan m_laser_msg;
	ros::NodeHandle m_node;
};

#endif
