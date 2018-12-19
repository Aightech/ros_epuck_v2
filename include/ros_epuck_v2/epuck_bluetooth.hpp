#ifndef EPUCK_BLUETOOTH_HPP
#define EPUCK_BLUETOOTH_HPP

void printBIN(char *str,int n);

void printSHORT(char *str,int n);

int combine(char lsb, char msb);


int init_connection(const char *path);

int send_cmd(int fd,char *cmd, int len);

int recv_rep(int fd, char *rep);


int get_values(char *str, int len, int *val);

int cmd_get_acc(char *str);//[-'a'] : get the code to get Accel
int cmd_get_bat(char *str);//[-'b'] : get battery
int cmd_get_spd(char *cmd);
int cmd_set_spd(char *cmd, int speedLeft, int speedRight);//[-'D'][...] : get the code to set speed 
int cmd_get_gyr(char *cmd);//[-'g'] : get the code to get gyr
int cmd_get_cam(char *cmd);//[-'I'] : get the code to get cam
int cmd_set_led(char *cmd, int n, int s);//[-'L'][n][state] : get the code to set LEDs
int cmd_get_flr(char *cmd);                         	//[-'M'] : get the code to get floor sensor
int cmd_get_sIR(char *cmd);//[-'N'] : get the code to get IR sensors states
int cmd_get_lgt(char *cmd);//[-'O'] : get the code to get light ambient state
int cmd_set_mot(char *cmd);//[-'P'] : get the code to set motor step
int cmd_get_mot(char *cmd);//[-'Q'] : get the code to get motor step
int cmd_get_tmp(char *cmd);//[-'t'] : get the code to get temperature
int cmd_get_mcA(char *cmd);//[-'u'] : get the code to get microphone amplitude
int cmd_get_mcB(char *cmd);//[-'U'] : get the code to get microphone buffer

#endif
