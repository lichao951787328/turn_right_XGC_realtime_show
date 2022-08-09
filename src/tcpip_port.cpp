#include"turn_right/tcpip_port.h"
#include <iostream>
#include <mutex>
#define SERVPORT 3333
#define BACKLOG 10
using namespace std;
extern int sock_fd,client_fd;
extern std::mutex m_tcpip;
tcpip_port::tcpip_port(/* args */)
{
}

tcpip_port::~tcpip_port()
{
    closetcpip();
}

int tcpip_port::initial()
{
    {
        unique_lock<mutex> g(m_tcpip, std::defer_lock);
        g.lock();
        if((sock_fd = socket(AF_INET, SOCK_STREAM/* |SOCK_NONBLOCK */, 0)) == -1) {
                perror("socket construct error");
                exit(1);
                return -1;
        }
        g.unlock();
    }
    
    my_addr.sin_family=AF_INET;
    my_addr.sin_port=htons(SERVPORT);
    my_addr.sin_addr.s_addr = INADDR_ANY;
    bzero(&(my_addr.sin_zero),8);
    if(bind(sock_fd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) == -1) {
        perror("bind error");
        exit(1);
        return -1;
    }
    if(listen(sock_fd, BACKLOG) == -1) {
        perror("listen error");
        exit(1);
        return -1;
    }
    // int timeout = 50;
    
    // setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));
    sin_size = sizeof(struct sockaddr_in);
    printf("initial port finish...\n");
    return 0;
}

int tcpip_port::accept_client()
{
    unique_lock<mutex> g(m_tcpip, std::defer_lock);
    g.lock();
    if((client_fd = accept(sock_fd, (struct sockaddr *)&remote_addr, (socklen_t *)&sin_size)) == -1) {
        perror("accept error");
        exit(1);
        return -1;
    }
    g.unlock();
    printf("received a connection from %s\n", inet_ntoa(remote_addr.sin_addr));
    return 0;
}

int tcpip_port::recvChar()
{
    return recv(client_fd, &recvFlag, sizeof(recvFlag), 0);
}

int tcpip_port::recvData()
{
    // struct data{char a[2];double b[15];};
    // for (size_t i = 0; i < 2; i++)
    // {
    //     std::cout<<i+1<<": "<<*(recevBuf + i)<<std::endl;
    //     *(recevBuf + i) = '\0';
    // }
    // cout<<"recev"<<endl;
    cout<<"enter recve data "<<endl;
    int tmp = recv(client_fd, &recevBuf, sizeof(recevBuf), 0);
    // cout<<"recev size "<<tmp<<endl;
    // for (size_t i = 0; i < 2; i++)
    // {
    //     std::cout<<i+1<<": "<<*((char*)(recevBuf + i))<<std::endl;
    // }

    // cout<<*(recevBuf)<<endl;
    // cout<<recevBuf[1]<<endl;
    // cout<<", "<<(int)recevBuf[1]<<endl;

    // for (int i = 0; i < 15; i++)
    // {
    //     std::cout<<i+1<<": "<<*((double*)(recevBuf + i*8 + 2))<<std::endl;
    // }
    // // cout<<sizeof(double)<<endl;
    // data cc = *((data*)recevBuf);
    // std::cout<<cc.a[0]<<", "<<cc.a[1]<<std::endl;
    // for(int i=0;i<15;i++)
    // {
    //     std::cout<<cc.b[i]<<std::endl;
    // }
    
    return tmp; 
}

void tcpip_port::analysisBuf()
{
#ifdef REALTIME
    struct data{char a[2]; uint32_t time[2]; double b[18];};
#else
    // struct data{char a[2]; uint32_t time[2]; double b[6];};
    struct data{char a[2];  double b[6];};
#endif
    data cc = *((data*)recevBuf);
    recevData.flag_detection = cc.a[0];
    recevData.walk_case      = cc.a[1];
    // recevData.secs           = cc.time[0];
    // recevData.nsecs          = cc.time[1];
    recevData.pose_roll      = cc.b[0];
    recevData.pose_pitch     = cc.b[1];
    recevData.pose_yaw       = cc.b[2];
    recevData.base_x         = cc.b[3];
    recevData.base_y         = cc.b[4];
    recevData.base_height    = cc.b[5];
#ifdef REALTIME
    recevData.base_vx        = cc.b[6];
    recevData.base_vy        = cc.b[7];
    recevData.base_vz        = cc.b[8];
    recevData.left_foot_x    = cc.b[9];
    recevData.left_foot_y    = cc.b[10];
    recevData.left_foot_z    = cc.b[11];
    recevData.right_foot_x   = cc.b[12];
    recevData.right_foot_y   = cc.b[13];
    recevData.right_foot_z   = cc.b[14];
    recevData.next_foot_x    = cc.b[15];
    recevData.next_foot_y    = cc.b[16];
    recevData.next_foot_z    = cc.b[17];
#endif
    cout<<"receive data :"<<endl;
    cout<<"recevData.flag_detection"<<recevData.flag_detection<<endl;
    cout<<"recevData.flag_foot     "<<recevData.walk_case     <<endl;
    cout<<"recevData.pose_roll     "<<recevData.pose_roll     <<endl;
    cout<<"recevData.pose_pitch    "<<recevData.pose_pitch    <<endl;
    cout<<"recevData.pose_yaw      "<<recevData.pose_yaw      <<endl;
    cout<<"recevData.base_x        "<<recevData.base_x        <<endl;
    cout<<"recevData.base_y        "<<recevData.base_y        <<endl;
    cout<<"recevData.base_height   "<<recevData.base_height   <<endl;
}

int tcpip_port::sendSteps(char* msg, size_t num)
{
    // cout<<"send data to win"<<endl;
    return send(client_fd, (char*)msg, num, 0);
}

void tcpip_port::close_client()
{
    cout<<"close client"<<endl;
    close(client_fd);

}



void tcpip_port::closetcpip()
{
    cout<<"CLOSE TCPIP PORT..."<<endl;
    close(sock_fd);
}

char tcpip_port::getRecvFlag()
{
    // return recvFlag;
    return recevData.flag_detection;
}

void tcpip_port::resetRecvFlag()
{
    recvFlag = '\0';
}
