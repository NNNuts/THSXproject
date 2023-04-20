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
    rob.canInit();
    rob.canStart();

    // rob.setCommond(0x600 + 1, 8, 0x2B4060000F000000);
    // rob.sendCommond();
    // exit(1);

    // //设置速度控制模式
    // rob.setCommond(0x600 + 1, 8, 0x2F60600003000000);
    // rob.sendCommond();
    // rob.motorClearWarning(1);
    // rob.motorInit(1,rob.PositionMod);
    // rob.motorInit(2,rob.PositionMod);

    // rob.motorSetPosition(1,10);
    // usleep(3000000);
    // rob.motorSetPosition(1,000);
    // usleep(3000000);
    // rob.motorDisEnable(1);

    // exit(1);

    //speed
    rob.motorInit(1,rob.SpeedMod);
    rob.motorInit(2,rob.SpeedMod);

    rob.motorSetSpeed(1,200000);
    rob.motorSetSpeed(2,200000);
    usleep(2000000);

    rob.motorSetSpeed(1,0);
    rob.motorSetSpeed(2,0);
    usleep(2000000);

    rob.motorDisEnable(1);
    rob.motorDisEnable(2);

    return 0;
} 
