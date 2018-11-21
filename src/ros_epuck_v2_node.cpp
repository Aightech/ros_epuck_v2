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
float linear_speed = 0.0;
float angular_speed = 0.0;



void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	linear_speed = msg->axes[1];
	angular_speed = msg->axes[0];
}



int main(int argc, char* argv[])
{

	ros::init(argc, argv, "epuckv2");
	ros::NodeHandle n;

	ros::Subscriber subJoy = n.subscribe("/joy", 10, joyCallback);
	ros::Publisher pubSens = n.advertise<geometry_msgs::Twist>("epuckv2_sensors", 10);
	ros::Rate loop_rate(rate);
	
	Epuck epuck(n,"/dev/rfcomm0");

//	int epuck = init_connection("/dev/rfcomm0");
//	if(epuck>-1)
//		std::cout << "Connected ..." << std::endl;
	float r=1;
	float D=2;
	float coef=745;


	while(ros::ok())
	{
		ros::spinOnce();
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = linear_speed;
		cmd_vel.angular.z = angular_speed;
		epuck.set_spd(-coef*(-4.*linear_speed+D*angular_speed)/(4.*r), coef*(4.*linear_speed+D*angular_speed)/(4.*r));
		
		epuck.update();
//		int speedLeft = -coef*(-4.*linear_speed+D*angular_speed)/(4.*r);
//		int speedRight = coef*(4.*linear_speed+D*angular_speed)/(4.*r);
//		
//		char buff[10];
//		int len = cmd_get_spd(buff);
//		len += cmd_set_spd(buff+len,speedLeft,speedRight);
//		len += cmd_get_sIR(buff+len);
//		//printBIN(buff,len+1);
//		send_cmd(epuck,buff,len+1);
//		int n = recv_rep(epuck,buff);
//		
//		if(n>-1)
//			printSHORT(buff,n-1);
		
		//pubSens.publish(cmd_vel);
	}
//	close(epuck);
	return 0;
}
