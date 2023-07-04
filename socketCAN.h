#pragma once
#include <linux/can.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <Eigen/Dense>
#include <iostream>

using namespace std;
enum
{
    Can0,
    Can1,
    CanAll
};

class socketCAN
{
public:
    int CanPort = 24;
    int Can_fd[2];
    struct can_frame CanReadData[2][1000];
    int CanReadDataNumMax = 1000;
    int CanReadDataNum[2] = {0, 0};
    int CanSendSleep = 2000;//ms

    // socketCAN();
    // socketCAN(int com);

    void *can0_get_status(void);

    static void* staticCan0_get_status(void *param);

    void *can1_get_status(void);

    static void* staticCan1_get_status(void *param);

    void canOpen(void);

    void canStartPthread(int Can = CanAll);
    
    void canRead(int com, can_frame* frame);

    void canClean(int com);

    void canSetSendSleep(int sleep);

    void canSend(int com, int ID, int Len, long data, unsigned long pass = 0);
};
