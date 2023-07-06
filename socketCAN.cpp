#include <linux/can.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <unistd.h>
// #include <string>
// #include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include "socketCAN.h"

using namespace std;
// socketCAN::socketCAN(int com = 24){
//     CanPort = com;
// }

void* socketCAN::can0_get_status(void){
    pthread_detach(pthread_self());
    int err;
    struct can_frame frame;
    while(true){
        
        if(CanReadDataNum[0]>=CanReadDataNumMax)
            CanReadDataNum[0] = 0;
        // cout<<num<<endl;
        err = read(Can_fd[0], &frame, sizeof(frame));
        CanReadData[0][CanReadDataNum[0]].can_id = frame.can_id;
        CanReadData[0][CanReadDataNum[0]].can_dlc = frame.can_dlc;
        for(int i=0; i<8; i++)
            CanReadData[0][CanReadDataNum[0]].data[i] = frame.data[i];
        CanReadDataNum[0] ++;
    }
}

void* socketCAN::staticCan0_get_status(void *param){
    socketCAN* pThis = (socketCAN*)param;
    pThis->can0_get_status();
}

void* socketCAN::can1_get_status(void){
    pthread_detach(pthread_self());
    int err;
    struct can_frame frame;
    while(true){
        
        if(CanReadDataNum[1]>=CanReadDataNumMax)
            CanReadDataNum[1] = 0;
        // cout<<num<<endl;
        // err = read(Can_fd[1], &CanReadData[1][CanReadDataNum[1]], sizeof(CanReadData[1][CanReadDataNum[1]]));
        err = read(Can_fd[1], &frame, sizeof(frame));
        CanReadData[1][CanReadDataNum[1]].can_id = frame.can_id;
        CanReadData[1][CanReadDataNum[1]].can_dlc = frame.can_dlc;
        for(int i=0; i<8; i++)
            CanReadData[1][CanReadDataNum[1]].data[i] = frame.data[i];
        // cout<<"read "<<CanReadDataNum[1]<<" ID "<<CanReadData[1][CanReadDataNum[1]].can_id<<endl;
        CanReadDataNum[1] ++;
        }
}

void* socketCAN::staticCan1_get_status(void *param){
    socketCAN* pThis = (socketCAN*)param;
    pThis->can1_get_status();
}

void socketCAN::canOpen(void){
    Can_fd[0] = socket(AF_CAN, SOCK_RAW, CAN_RAW);
    Can_fd[1] = socket(AF_CAN, SOCK_RAW, CAN_RAW);
    if(Can_fd < 0)
    {
        perror("socket can creat error!\n");
        exit(0);
    }
    struct ifreq ifr;  // if.h
    strcpy(ifr.ifr_name, "can0");
    ioctl(Can_fd[0], SIOCGIFINDEX, &ifr); // 指定编号为 can0 的设备，获取设备索引
    

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;  // 指定协议族
    addr.can_ifindex = ifr.ifr_ifindex;  // 设备索引
    // 将套接字与 can0 绑定
    int bind_res = bind(Can_fd[0], (struct sockaddr *)&addr, sizeof(addr));
    if(bind_res < 0)
    {
        perror("bind error!");
        exit(0);
    }

    strcpy(ifr.ifr_name, "can1");
    ioctl(Can_fd[1], SIOCGIFINDEX, &ifr); // 指定编号为 can0 的设备，获取设备索引
    addr.can_ifindex = ifr.ifr_ifindex;  // 设备索引

    // 将套接字与 can0 绑定
    bind_res = bind(Can_fd[1], (struct sockaddr *)&addr, sizeof(addr));
    if(bind_res < 0)
    {
        perror("bind error!");
        exit(0);
    }

    int loopback = 0; //0 表示关闭，1 表示开启(默认) 
    setsockopt(Can_fd[0], SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
    setsockopt(Can_fd[1], SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
}

void socketCAN::canStartPthread(int Can){
    pthread_t threads;
    int rc;
    long fd;
    if(Can == Can0 || Can == CanAll){
        fd = Can_fd[0];
        rc = pthread_create(&threads, NULL, staticCan0_get_status, (void *)this);
    }
    if(Can == Can1 || Can == CanAll){
        fd = Can_fd[1];
        rc = pthread_create(&threads, NULL, staticCan0_get_status, (void *)this);
    }
}

void socketCAN::canRead(int com, can_frame* frame){
    read(Can_fd[com], frame, sizeof(*frame));
}

void socketCAN::canClean(int com){
    CanReadDataNum[com] = 0;
}

void socketCAN::canSetSendSleep(int sleep){
    CanSendSleep = sleep;
}

void socketCAN::canSend(int com, int ID, int Len, long data, unsigned long pass){
    struct can_frame frame;

    long pass_new = 0;
    for(int i = 0;i<4;i++)
    {
        pass_new *= 256;
        pass_new += pass % 256;
        pass /= 256;
    }

    for(int i = Len - 1; i >= 0; i--)
    {
        // canData[canDataNum].Data[i] = data % 256;
        frame.data[i] = data % 256 + pass_new % 256; 
        // cout<<i<<" "<<(int)canData[canDataNum].Data[i]<<" "<<data % 256<<endl;
        data = data / 256;
        pass_new = pass_new / 256;
        // data > 2;
    }

    /************ 写数据 ************/
    frame.can_dlc = Len;  // 设置数据长度（CAN协议规定一帧最多有八个字节的有效数据）
    frame.can_id = ID;    // 设置 ID 号，假设这里 ID 号为1，实际的 ID 号要根据是标准帧（11位）还是拓展帧（29）位来设置
    write(Can_fd[com], &frame, sizeof(frame));  // 写数据
    usleep(CanSendSleep);
}