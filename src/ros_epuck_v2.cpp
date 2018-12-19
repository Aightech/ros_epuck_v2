#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "ros_epuck_v2/ros_epuck_v2.hpp"
#include "ros_epuck_v2/epuck_bluetooth.hpp"

#define EPUCK_RAD 0.035

#define IR_MIN_RANGE 0.005
#define IR_MAX_RANGE 0.05
#define IR_FIELD_VIEW 0.26
#define IR_MODEL(X)  0.5/sqrt(X)

/**
 * Constructor
 * needs the a ros node and a path to the epuck bluetooth devices : ex "/dev/refcomm0"
 **/
Epuck::Epuck(ros::NodeHandle &n, const char * path)
{
	m_node = n;
	//try to connect to the epuck
	m_epuck_fd = init_connection(path);
	if(m_epuck_fd>-1)
		{
			std::cout << "[INFO EPUCK] Connected ..." << std::endl;

			//init all the publishers
			init_IR_sensors();
			init_laserScan();
			init_lasers();
	
			init_cmdSpeedLeft();
			init_cmdSpeedRight();

			//set the epuck as connected
			m_isConnected = true;

		}
	else
		std::cout << "[INFO EPUCK] Connection failed ..." << std::endl;

}

void Epuck::update()
{
	char buff[20];
	int len = 0;

	//
	//Fill the local buffer with the differents comands
	//
	
	//if(1)	len += cmd_get_acc(buff+len);                         	//[-'a'] : get the code to get Accel
	//if(1)	len += cmd_get_bat(buff+len);                         	//[-'b'] : get battery
	if(1)	len += cmd_set_spd(buff+len,m_speedLeft,m_speedRight);	//[-'D'][...] : get the code to set speed 
	if(1)	len += cmd_get_spd(buff+len);                         	//[-'E'] : get the code to get speed 
	//if(1)	len += cmd_get_gyr(buff+len);                         	//[-'g'] : get the code to get gyr
	//if(1)	len += cmd_get_cam(buff+len);                         	//[-'I'] : get the code to get cam

	//for(int i=0;i<NB_LED;i++)
	//if(1)	len += cmd_set_led(buff+len,i,led_state[i]);          	//[-'L'][n][state] : get the code to set LEDs
	
	//if(1)	len += cmd_get_flr(buff+len);                         	//[-'M'] : get the code to get floor sensor
	if(1)	len += cmd_get_sIR(buff+len);                         	//[-'N'] : get the code to get IR sensors states
	//if(1)	len += cmd_get_lgt(buff+len);                         	//[-'O'] : get the code to get light ambient state
	//if(1)	len += cmd_set_mot(buff+len);                         	//[-'P'] : get the code to set motor step
	//if(1)	len += cmd_get_mot(buff+len);                         	//[-'Q'] : get the code to get motor step
	//if(1)	len += cmd_get_tmp(buff+len);                         	//[-'t'] : get the code to get temperature
	//if(1)	len += cmd_get_mcA(buff+len);                         	//[-'u'] : get the code to get microphone amplitude
	//if(1)	len += cmd_get_mcB(buff+len);                         	//[-'U'] : get the code to get microphone buffer

	send_cmd(m_epuck_fd,buff,len+1);//send cmd to the epuck

	int values[10];
	int n = recv_rep(m_epuck_fd,buff);
	if(n>-1)
	{
		//printSHORT(buff,n);
		
		char str[] = "o        ";
		for(int i=0;i<10;i++) str[i]= (i==abs((m_time%10 - (m_time/10)%2*9)%10))?'o':' ';
		std::cout << "[INFO EPUCK] running ... [" << str << "]"  <<"\xd"<<std::flush;

		//send requests of the differents actuator and get the vvalues of the sensors
		get_values(buff,n-1,values);

		//update the different topic
		update_IR_sensors(values+2);
		update_laserScan(values+2);
		update_lasers(values+2);
	}
	m_time++;
//	int values[8]={100,0,0,0,0,0,0,0};
//	update_laserScan(values);
}


/**
 * initialize IR sensors:
 **/
void Epuck::init_IR_sensors()
{
	for(int i=0; i<8; i++)//for each ir sensors
	{
		//init a topic
		std::stringstream ss;
		ss.str("");
		ss << "IR_sensor" << i;
		m_IR_sensors_pub[i] = m_node.advertise<sensor_msgs::Range>(ss.str(), 10);
		m_IR_sensors_msg[i].radiation_type = sensor_msgs::Range::INFRARED;

		//set frame
		ss.str("");
		ss << "epuckv2/base_IR_sensor" << i;
		m_IR_sensors_msg[i].header.frame_id =  ss.str();
		
		m_IR_sensors_msg[i].field_of_view = IR_FIELD_VIEW;    // About 15 degrees...to be checked!
		m_IR_sensors_msg[i].min_range = IR_MIN_RANGE;       // 0.5 cm.
		m_IR_sensors_msg[i].max_range = IR_MAX_RANGE;        // 5 cm.                    
	}
}

/**
 *  update IR sensors:
 * use a model to update the value of the rostopic value from the values sent by the epuck
 **/
void Epuck::update_IR_sensors(int *IR_values)
{
	for(int i=0; i<8; i++)//for each sensors
	{
		if(IR_values[i] > 0)
			m_IR_sensors_msg[i].range = IR_MODEL(IR_values[i]);  // use the IR model to get meter from analog value
		else//if too far we got nothing
			m_IR_sensors_msg[i].range = m_IR_sensors_msg[i].max_range;

		//clip the values
		if(m_IR_sensors_msg[i].range > m_IR_sensors_msg[i].max_range)
			m_IR_sensors_msg[i].range = m_IR_sensors_msg[i].max_range;
		
		if(m_IR_sensors_msg[i].range < m_IR_sensors_msg[i].min_range)
			m_IR_sensors_msg[i].range = m_IR_sensors_msg[i].min_range;

		//publish the values
		m_IR_sensors_msg[i].header.stamp = ros::Time::now();
		m_IR_sensors_pub[i].publish(m_IR_sensors_msg[i]);
	}
}

/**
 * initialize laser topic:
 **/
void Epuck::init_lasers()
{
	m_lasers_pub = m_node.advertise<std_msgs::Float32MultiArray>("lasers", 10);
}

/**
 * update the value of the lasers topics :
 **/
void Epuck::update_lasers(int *IR_values)
{
    std_msgs::Float32MultiArray laser_msg;
    float val;
    for (size_t i = 5; i < 5+6; ++i)//clip the value beetween 0 and 100 
	    laser_msg.data.push_back( ( (val=IR_MODEL(IR_values[i%8]))>0.1)?-1:(val<0.01)?0:val*1000 );
    m_lasers_pub.publish(laser_msg);
}

/**
 * initialize laser scan topic:
 **/
void Epuck::init_laserScan()
{
	std::stringstream ss;
	ss.str("");
	ss << "epuckv2/base_laser";
	m_laser_msg.header.frame_id = ss.str();
	m_laser_msg.angle_min = -M_PI/2.0;
	m_laser_msg.angle_max = M_PI/2.0;
	m_laser_msg.angle_increment = M_PI/18.0; // 10 degrees.
	m_laser_msg.range_min = IR_MIN_RANGE+EPUCK_RAD; // 0.5 cm + radius.
	m_laser_msg.range_max = IR_MAX_RANGE+EPUCK_RAD; // 5 cm + radis. 
	m_laser_msg.ranges.resize(19);
	m_laser_msg.intensities.resize(19);
	m_laser_pub = m_node.advertise<sensor_msgs::LaserScan>("scan", 10);
}

/**
 * Update laser scan by interpolling 19 point from the value of the IR sensors:
 **/
void Epuck::update_laserScan(int *IR_values)
{
	//Using the 6 IR sensors on the front to simulate 19 laser scan points.

	// -90|-80|-70|-60|-50|-40|-30|-20|-10|  0| 10| 20| 30| 40| 50| 60| 70| 80|-90|
	//mask of interpolation
	float mask[19][8] = {	{0	,0	,1	,0	,0	,0	,0	,0	},
				{0	,1./5.	,4./5.	,0	,0	,0	,0	,0	},
				{0	,2./5.	,3./5.	,0	,0	,0	,0	,0	},
				{0	,3./5.	,2./5.	,0	,0	,0	,0	,0	},
				{0	,4./5.	,1./5.	,0	,0	,0	,0	,0	},
				{0	,1	,0	,0	,0	,0	,0	,0	},
				{1./3.	,2./3.	,0	,0	,0	,0	,0	,0	},
				{2./3.	,1/3.	,0	,0	,0	,0	,0	,0	},
				{1	,0	,0	,0	,0	,0	,0	,0	},
				{1./2.	,0	,0	,0	,0	,0	,0	,1./2.	},
				{0	,0	,0	,0	,0	,0	,0	,1	},
				{0	,0	,0	,0	,0	,0	,1./3.	,2./3.	},
				{0	,0	,0	,0	,0	,0	,2./3.	,1./3.	},
				{0	,0	,0	,0	,0	,0	,1	,0	},
				{0	,0	,0	,0	,0	,1./5.	,4./5.	,0	},
				{0	,0	,0	,0	,0	,2./5.	,3./5.	,0	},
				{0	,0	,0	,0	,0	,3./5.	,2./5.	,0	},
				{0	,0	,0	,0	,0	,4./5.	,1./5.	,0	},
				{0	,0	,0	,0	,0	,1	,0	,0	}
				};
	// for(int j=0;j<8;j++) std::cout << IR_values[j] << " | ";
	// std::cout << std::endl << std::endl;
	for(int i=0;i<19;i++)
	{
		float tempProx = 0;
		for(int j=0;j<8;j++) tempProx += mask[i][j]*IR_values[j];
		if(tempProx > 0)
		{
			m_laser_msg.ranges[i] = IR_MODEL(tempProx)+EPUCK_RAD; //analog to meters.
			m_laser_msg.intensities[i] = tempProx; 
		}
		else
		{
			m_laser_msg.ranges[i] = m_laser_msg.range_max;
			m_laser_msg.intensities[i] = 0;
		}
		//std::cout << m_laser_msg.ranges[i] << " | ";
					
	}
	// td::cout << std::endl;
	m_laser_pub.publish(m_laser_msg);
}

/**
 * initialize left wheel speed subscriber
 **/
void Epuck::init_cmdSpeedLeft()
{
	subCmdSpdLeft = m_node.subscribe("/simu_fastsim/speed_left", 10, &Epuck::speed_leftCallback,this);
}

/**
 * initialize right wheel speed subscriber:
 **/
void Epuck::init_cmdSpeedRight()
{
	
	subCmdSpdRight = m_node.subscribe("/simu_fastsim/speed_right", 10,&Epuck::speed_rightCallback,this);
}

/**
 * callback function use when the left wheel tipic is updated:
 **/
void Epuck::speed_leftCallback(const std_msgs::Float32::ConstPtr& msg)
{
	float conv = msg->data*1000/0.8;
	m_speedLeft = conv;
}

/**
 * callback function use when the left wheel tipic is updated:
 **/
void Epuck::speed_rightCallback(const std_msgs::Float32::ConstPtr& msg)
{
	float conv = msg->data*1000/0.8;
	m_speedRight = conv;
}
