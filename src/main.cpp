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

    rob.motorClearWarning(1);
    rob.motorClearWarning(2);
    rob.motorClearWarning(3);
    rob.motorClearWarning(4);
    rob.motorDisEnable(1);
    rob.motorDisEnable(2);
    rob.motorDisEnable(3);
    rob.motorDisEnable(4);

    //设置位置控制模式
    // rob.motorInit(3,rob.PositionMod);

    // rob.motorChangeTrapezoidalVelocityInPosition(3,50);
    // rob.motorSetPosition(3,10);
    // usleep(3000000);
    // rob.motorSetPosition(3,00);
    // usleep(3000000);
    // rob.motorDisEnable(3);
    // usleep(3000000);

    // exit(1);

    //speed
    rob.motorInit(1,rob.SpeedMod);
    rob.motorInit(2,rob.SpeedMod);
    rob.motorInit(3,rob.SpeedMod);
    cout<<"速度模式"<<endl;
    rob.motorInit(4,rob.SpeedMod);

    // rob.motorChangeUpAccelerationInSpeed(3,20);
    // rob.motorChangeUpAccelerationInSpeed(4,20);
    // rob.motorChangeDownAccelerationInSpeed(3,20);
    // rob.motorChangeDownAccelerationInSpeed(4,20);
    // rob.motorChangeUpAccelerationInSpeed(1,20);
    // rob.motorChangeUpAccelerationInSpeed(2,20);
    // rob.motorChangeDownAccelerationInSpeed(1,20);
    // rob.motorChangeDownAccelerationInSpeed(2,20);
    

    rob.motorSetSpeed(1,1);
    rob.motorSetSpeed(2,1);
    rob.motorSetSpeed(3,1);
    rob.motorSetSpeed(4,1);
    usleep(5000000);

    rob.motorSetSpeed(1,0);
    rob.motorSetSpeed(2,0);
    rob.motorSetSpeed(3,0);
    rob.motorSetSpeed(4,0);
    // usleep(2000000);

    // rob.motorDisEnable(1);
    // rob.motorDisEnable(2);
    // rob.motorDisEnable(3);
    // rob.motorDisEnable(4);

    return 0;
} 
       