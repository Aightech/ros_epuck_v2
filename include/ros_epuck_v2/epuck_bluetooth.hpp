#ifndef EPUCK_BLUETOOTH_HPP
#define EPUCK_BLUETOOTH_HPP

void printBIN(char *str,int n);

void printSHORT(char *str,int n);

int combine(char lsb, char msb);


int init_connection(const char *path);

int send_cmd(int fd,char *cmd, int len);

int recv_rep(int fd, char *rep);


int cmd_set_spd(char *cmd, int speedLeft, int speedRight);

int cmd_get_spd(char *cmd);

int  cmd_get_sIR(char *cmd);


int get_values(char *str, int len, int *val);

#endif
