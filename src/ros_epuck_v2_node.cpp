#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "ros_epuck_v2/ros_epuck_v2.hpp"
#include "ros_epuck_v2/epuck_bluetooth.hpp"

#define MOTOR1 0
#define MOTOR2 1
#define IR_SENSOR 2
#define NB_IR_SENSORS 8

int rate = 10; //10 Hz
float speed_left = 0.0;
float speed_right = 0.0;




int main(int argc, char* argv[])
{

	ros::init(argc, argv, "epuckv2");
	ros::NodeHandle n;
        
	ros::Rate loop_rate(rate);
	
	Epuck epuck(n,"/dev/rfcomm0");

	while(ros::ok())
	{
		ros::spinOnce();
		epuck.update();
	}
	return 0;
}
