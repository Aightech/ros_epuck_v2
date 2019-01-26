#ifndef ROS_EPUCK_V2_HPP
#define ROS_EPUCK_V2_HPP
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

// #include <sensor_msgs/Range.h>
// #include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
// #include <geometry_msgs/Twist.h>
// #include <std_msgs/UInt8MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>
// #include <opencv/cv.h>
// #include <sensor_msgs/LaserScan.h>

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

	void init_cmdVel();
	void init_position();
	void update_pos(int *pos_values);

	void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg);

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
	std::string m_name;

	double m_wheel_radius = 0.02;
	double m_diameter = 0.035;

	double m_stp_rot[2];//motors steps
	double m_pos[3];//x,y,theta
	ros::Time m_currentTime, m_lastTime;
	
	ros::Publisher m_IR_sensors_pub[8];
	sensor_msgs::Range m_IR_sensors_msg[8];
	
	ros::Publisher m_laser_pub;
	sensor_msgs::LaserScan m_laser_msg;

	ros::Publisher m_lasers_pub;

	geometry_msgs::Twist cmd_vel;
	ros::Subscriber m_cmdVel_sub;
	
	ros::Subscriber m_cmdSpdLeft_sub;
	ros::Subscriber m_cmdSpdRight_sub;

	
	ros::Publisher m_odom_pub;
	nav_msgs::Odometry m_odomMsg;
	geometry_msgs::TransformStamped m_odomTrans;
	tf::TransformBroadcaster m_tf_transform;
	
	ros::Publisher imagePublisher;
	ros::Publisher accelPublisher;

	
	sensor_msgs::Imu accelMsg;
	ros::Publisher motorSpeedPublisher;
	visualization_msgs::Marker motorSpeedMsg;
	ros::Publisher microphonePublisher;
	visualization_msgs::Marker microphoneMsg;
	ros::Publisher floorPublisher;
	visualization_msgs::Marker floorMsg;

  

};

#endif
