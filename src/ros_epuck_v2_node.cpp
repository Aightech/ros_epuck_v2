#include "ros/ros.h"

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "ros_epuck_v2/ros_epuck_v2.hpp"
#include "ros_epuck_v2/epuck_bluetooth.hpp"


int rate = 10; //10 Hz 


int main(int argc, char* argv[])
{

	ros::init(argc, argv, "epuckv2");
	ros::NodeHandle n;
        
	ros::Rate loop_rate(rate);

	//create a epuck object that try to connect to the epuck link to the path given
	Epuck epuck(n,"/dev/rfcomm0");

	//update the state of the epuck indefinately
	while(ros::ok())
	{
		ros::spinOnce();
		epuck.update();
	}
	return 0;
}
