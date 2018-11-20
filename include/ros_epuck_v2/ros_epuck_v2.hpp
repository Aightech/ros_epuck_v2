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
private:
	int epuck_fd;
	int speedLeft;
	int speedRight;
	ros::Publisher IR_sensors_pub[8];
	sensor_msgs::Range IR_sensors_msg[8];
	ros::Publisher laser_pub;
	sensor_msgs::LaserScan laser_msg;
	ros::NodeHandle node;
};

#endif
