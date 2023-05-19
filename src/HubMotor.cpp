// #include <iostream>
// #include <thread>
// #include <signal.h>
// #include <string.h>
// #include <chrono>
// #include <mutex>
// #include <fstream>
// #include <Eigen/Dense>
// #include <ctime>
#include "HubMotor.hpp"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#define msleep(ms)  usleep((ms)*1000)

// using namespace std; 
// using namespace std::chrono;
// using namespace Eigen;

HubMotor rob;

void HubMotorCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    rob.motorSetSpeed(1,msg->data.at(0));
    // rob.motorSetSpeed(2,msg->data.at(1));
    // rob.motorSetSpeed(3,msg->data.at(2));
    // rob.motorSetSpeed(4,msg->data.at(3));
    ROS_INFO("HubMotor:speed = [%f],[%f],[%f],[%f]", msg->data.at(0),msg->data.at(1),msg->data.at(2),msg->data.at(3));
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "HubMotor");  //解析参数，命名结点
    ros::NodeHandle nh;  //创建句柄，实例化node

    //exit(0);
    cout << "系统启动" << endl;

    // rob.canOpen();
    // rob.canInit();
    // rob.canStart();

    //设置速度控制模式
    // rob.motorInit(1,rob.SpeedMod);
    // rob.motorInit(2,rob.SpeedMod);
    // rob.motorInit(3,rob.SpeedMod);
    // rob.motorInit(4,rob.SpeedMod);

    ros::Subscriber sub = nh.subscribe("HubControl", 1000, HubMotorCallback);
    ros::spin();
    return 0;

    // rob.motorChangeTrapezoidalVelocityInPosition(3,50);
    // rob.motorSetPosition(3,10);
    // usleep(3000000);
    // rob.motorSetPosition(3,00);
    // usleep(3000000);
    // rob.motorDisEnable(3);

    // exit(1);

    //speed
    // rob.motorInit(1,rob.SpeedMod);
    // rob.motorInit(2,rob.SpeedMod);
    // rob.motorInit(3,rob.SpeedMod);
    // rob.motorInit(4,rob.SpeedMod);

    // rob.motorChangeUpAccelerationInSpeed(1,20);

    // rob.motorSetSpeed(1,10);
    // rob.motorSetSpeed(2,10);
    // rob.motorSetSpeed(3,10);
    // rob.motorSetSpeed(4,10);
    // usleep(2000000);

    // rob.motorSetSpeed(1,0);
    // rob.motorSetSpeed(2,0);
    // rob.motorSetSpeed(3,0);
    // rob.motorSetSpeed(4,0);
    // usleep(2000000);

    // rob.motorDisEnable(1);
    // rob.motorDisEnable(2);
    // rob.motorDisEnable(3);
    // rob.motorDisEnable(4);

    // return 0;
} 
       