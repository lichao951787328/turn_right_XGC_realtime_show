#ifndef _TCPIP_PORT_H_
#define _TCPIP_PORT_H_
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>  
#include <unistd.h> 
// #define REALTIME
// 需要的数据，检测触发，通过机器人左右脚推断质心相对机器人脚的位置，状态估计值（主要是角度）
// 1个char flag位
// 左右脚的位置 6个double
// 质心的位姿  6个double
struct need_data
{
    char flag_detection;
    char walk_case;
    uint32_t secs; //减去今日0时0分0秒后的时间
    uint32_t nsecs;
    double pose_roll;
    double pose_pitch;
    double pose_yaw;
    double base_x;
    double base_y;
    double base_height;
#ifdef REALTIME
    double base_vx;
    double base_vy;
    double base_vz;
    double left_foot_x;
    double left_foot_y;
    double left_foot_z;
    double right_foot_x;
    double right_foot_y;
    double right_foot_z;
    double next_foot_x;
    double next_foot_y;
    double next_foot_z;
#endif
};

class tcpip_port
{
private:
    // int sock_fd,client_fd;
    int sin_size;
    struct sockaddr_in my_addr;
    struct sockaddr_in remote_addr;
    char recevBuf[128];
    char recvFlag = '\0';
public:
    need_data recevData;
    tcpip_port(/* args */);
    ~tcpip_port();
    int initial();
    
    int accept_client();
    int recvChar();
    int recvData();
    void analysisBuf();
    int sendSteps(char* msg, size_t num);
    void close_client();
    void closetcpip();
    char getRecvFlag();
    void resetRecvFlag();
};

#endif