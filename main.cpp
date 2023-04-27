#include <iostream>
#include <thread>
#include <signal.h>
#include <string.h>
#include <chrono>
#include <mutex>
#include <fstream>
#include <Eigen/Dense>
#include <ctime>
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

    

    // //设置速度控制模式
    // rob.motorInit(3,rob.PositionMod);

    // rob.motorSetPosition(3,10);
    // usleep(3000000);
    // rob.motorSetPosition(3,000);
    // usleep(3000000);
    // rob.motorDisEnable(3);

    // exit(1);

    //speed
    rob.motorInit(1,rob.SpeedMod);
    rob.motorInit(2,rob.SpeedMod);
    rob.motorInit(3,rob.SpeedMod);
    rob.motorInit(4,rob.SpeedMod);

    rob.motorSetSpeed(1,200000);
    rob.motorSetSpeed(2,200000);
    rob.motorSetSpeed(3,600000);
    rob.motorSetSpeed(4,600000);
    usleep(2000000);

    rob.motorSetSpeed(1,0);
    rob.motorSetSpeed(2,0);
    rob.motorSetSpeed(3,0);
    rob.motorSetSpeed(4,0);
    usleep(2000000);

    // rob.motorDisEnable(1);
    // rob.motorDisEnable(2);
    // rob.motorDisEnable(3);
    // rob.motorDisEnable(4);

    return 0;
} 
