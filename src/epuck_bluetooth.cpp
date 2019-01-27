#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "ros_epuck_v2/epuck_bluetooth.hpp"


void printBIN(char *str,int n)
{

  printf("(BIN)\t |");
	int i=0;
	while(i<n)
		printf("%d|",(int)str[i++]);
	printf("\n");
}

void printSHORT(char *str,int n)
{
	printf("(SHORT)\t |");
	int i=0;
	char msb, lsb;
	int combined;
	while(i<n)
	{
		lsb=str[i];
		msb=str[i+1];
		combined = (msb << 8 ) | (lsb & 0xff);
		printf("%d|",combined);
		i+=2;
	}
	printf("\n");
}


int combine(char lsb, char msb)
{
	return (msb << 8 ) | (lsb & 0xff);
}

int init_connection(const char *path)
{
	
	int i,fd = open(path, O_RDWR | O_NOCTTY);
	//	std::cout << fd << std::endl;
	if(fd < 0)
		for(i=0;i<3;i++,sleep(1))
			if((fd = open(path, O_RDWR | O_NOCTTY))>-1)
				return fd;
	return fd;
}


int send_cmd(int fd,char *cmd, int len)
{
	int n = write(fd,cmd,len);
	if(n<0)
		return -1;
	return n;
}
int recv_rep(int fd, char *rep)
{
	int n = read(fd, rep, 1000);
	if(n<0)
		return -1;
	return n;
}




int get_values(char *str, int len, int *val)
{
	int i=0;
	char msb, lsb;
	while(i<len)
	{
		lsb=str[i];
		msb=str[i+1];
		val[i/2] = (msb << 8 ) | (lsb & 0xff);
		i+=2;
	}
}

int cmd_get_acc(char *cmd)//[-'a'] : get the code to get Accel
{
	cmd[0] = -'a';
	cmd[1] = 0;
	return 1;
}
int cmd_get_bat(char *cmd)//[-'b'] : get battery
{
	cmd[0] = -'b';
	cmd[1] = 0;
	return 1;
}

int cmd_set_spd(char *cmd, int speedLeft, int speedRight)//[-'D'][...]: get the code to set speed ( speed €[-1000,1000])
{
	speedLeft = (speedLeft>1000)?1000:(speedLeft<-1000)?-1000:speedLeft;
	speedRight = (speedRight>1000)?1000:(speedRight<-1000)?-1000:speedRight;
	cmd[0] = -'D';
	cmd[1] = speedLeft&0xFF;
	cmd[2] = (speedLeft>>8)&0xFF;
	cmd[3] = speedRight&0xFF;
	cmd[4] = (speedRight>>8)&0xFF;
	cmd[5] = 0;
	return 5;
}

int cmd_get_spd(char *cmd)//[-'E'] : get the code to get speed 
{
	cmd[0] = -'E';
	cmd[1] = 0;
	return 1;
}

int cmd_get_gyr(char *cmd)//[-'g'] : get the code to get gyr
{
	cmd[0] = -'g';
	cmd[1] = 0;
	return 1;
}
int cmd_get_cam(char *cmd)//[-'I'] : get the code to get cam
{
	cmd[0] = -'I';
	cmd[1] = 0;
	return 1;
}
int cmd_set_led(char *cmd)//[-'L'][n][state] : get the code to set LEDs
{
	cmd[0] = -'L';
	cmd[1] = 0;
	return 1;
}
int cmd_get_flr(char *cmd)//[-'M'] : get the code to get floor sensor
{
	cmd[0] = -'M';
	cmd[1] = 0;
	return 1;
}
int  cmd_get_sIR(char *cmd)//[-'N'] : get the code to get IR sensors states
{
	cmd[0] = -'N';
	cmd[1] = 0;
	return 1;
}
int cmd_get_lgt(char *cmd)//[-'O'] : get the code to get light ambient state
{
	cmd[0] = -'O';
	cmd[1] = 0;
	return 1;
}
int cmd_set_mot(char *cmd, int left_stp, int right_stp)//[-'P'] : get the code to set motor step
{
	cmd[0] = -'P';
	cmd[1] = left_stp&0xFF;
	cmd[2] = (left_stp>>8)&0xFF;
	cmd[3] = right_stp&0xFF;
	cmd[4] = (right_stp>>8)&0xFF;
	cmd[5] = 0;
	return 5;
}

int cmd_get_mot(char *cmd)//[-'Q'] : get the code to get motor step
{
	cmd[0] = -'Q';
	cmd[1] = 0;
	return 1;
}
int cmd_get_tmp(char *cmd)//[-'t'] : get the code to get temperature
{
	cmd[0] = -'t';
	cmd[1] = 0;
	return 1;
}
int cmd_get_mcA(char *cmd)//[-'u'] : get the code to get microphone amplitude
{
	cmd[0] = -'u';
	cmd[1] = 0;
	return 1;
}
int cmd_get_mcB(char *cmd)//[-'U'] : get the code to get microphone buffer
{
	cmd[0] = -'U';
	cmd[1] = 0;
	return 1;
}
