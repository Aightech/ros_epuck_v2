#ifndef ROS_EPUCK_V2_HPP
#define ROS_EPUCK_V2_HPP
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

class Epuck
{
public:
	Epuck(ros::NodeHandle &n,const char * path);
	~Epuck(){};
	
	//	void init_motors();
	//	void update_motors(int *IR_values);
	//	
	//	void init_temp_sensors();
	//	void update_temp_sensors(int *IR_values);
	//	
	//	void init_odometry();
	//	void update_odometry(int *IR_values);
	//	
	//	void init_microphone();
	//	void update_microphone(int *IR_values);
	//	
	//	void init_floor_sensors();
	//	void update_floor_sensors(int *IR_values);
	
	void init_IR_sensors();
	void update_IR_sensors(int *IR_values);
	
	void init_laserScan();
	void update_laserScan(int *IR_values);

	void init_lasers();
	void update_lasers(int *IR_values);
	
	void init_cmdSpeedLeft();

	void init_cmdSpeedRight();

	void speed_leftCallback(const std_msgs::Float32::ConstPtr& msg);
	void speed_rightCallback(const std_msgs::Float32::ConstPtr& msg);

	void set_spd(int speedLeft, int speedRight);
  
	void update();
private:
	int m_time=0;
	bool m_isConnected=false;
	int m_epuck_fd;
	int m_speedLeft=0;
	int m_speedRight=0;
	ros::NodeHandle m_node;
	
	ros::Publisher m_IR_sensors_pub[8];
	sensor_msgs::Range m_IR_sensors_msg[8];
	
	ros::Publisher m_laser_pub;
	sensor_msgs::LaserScan m_laser_msg;

	ros::Publisher m_lasers_pub;

	ros::Subscriber subCmdSpdLeft;
	ros::Subscriber subCmdSpdRight;

  

};

#endif
