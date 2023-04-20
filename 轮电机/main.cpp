#include <iostream>
#include <thread>
#include <signal.h>
#include <string.h>
#include <chrono>
#include <mutex>
#include<fstream>
#include <Eigen/Dense>
#include<ctime>
#include "HubMotor.hpp"
#define msleep(ms)  usleep((ms)*1000)

using namespace std; 
using namespace std::chrono;
using namespace Eigen;

HubMotor rob;

int main(int argc, char* argv[])
{

    //exit(0);
    cout << "系统启动" << endl;

    rob.canOpen();
    // rob.canInit();
    rob.canStart();

    rob.motorInit(1,rob.SpeedMod);
    rob.motorInit(2,rob.SpeedMod);

    rob.motorSetSpeed(1,200);
    rob.motorSetSpeed(2,300);
    usleep(2000000);

    rob.motorSetSpeed(1,0);
    rob.motorSetSpeed(2,0);
    usleep(2000000);

    rob.motorDisEnable(1);
    rob.motorDisEnable(2);

    return 0;
} 
