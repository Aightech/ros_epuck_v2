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

int cmd_set_spd(char *cmd, int speedLeft, int speedRight)
{
	cmd[0] = -'D';
	cmd[1] = speedLeft&0xFF;
	cmd[2] = (speedLeft>>8)&0xFF;
	cmd[3] = speedRight&0xFF;
	cmd[4] = (speedRight>>8)&0xFF;
	cmd[5] = 0;
	return 5;
}

int cmd_get_spd(char *cmd)
{
	cmd[0] = -'E';
	cmd[1] = 0;
	return 1;
}

int  cmd_get_sIR(char *cmd)
{
	cmd[0] = -'N';
	cmd[1] = 0;
	return 1;
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
