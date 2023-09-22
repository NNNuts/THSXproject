#pragma once

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
// #include <pthread.h>

#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include <HubMotor_pkg/controlcan.h>
using namespace Eigen;
using namespace std;

class CAN2USB
{
private:
    // int isOpen = false;
    // int isInit     = false;
    // int isStart  = false;
public:
    // VCI_CAN_OBJ canData[100];
    // int canDataNum = 0;
    int CanSendSleep = 50000;//50ms


    void canOpen(void);

    void canSend(int com, int ID, int Len, long data, unsigned long pass = 0);

    void canSetSendSleep(int us);

    int canRead(VCI_CAN_OBJ* frame);
};